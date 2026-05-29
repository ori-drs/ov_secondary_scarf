#!/usr/bin/env python3

import argparse
import copy
import csv
from pathlib import Path

import numpy as np
from evo.core import geometry, sync
from evo.core import lie_algebra as lie
from evo.core.trajectory import PoseTrajectory3D
import rosbag2_py
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.serialization import deserialize_message


CSV_HEADER = "# counter,sec,nsec,x,y,z,qx,qy,qz,qw\n"


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Extract ov_secondary trajectory snapshots and odometry poses from a "
            "rosbag2 bag into CSV files."
        )
    )
    parser.add_argument(
        "bag",
        help="Path to the rosbag2 directory, e.g. /path/to/output/ov_slam/ov_slam_bag",
    )
    parser.add_argument(
        "output_folder",
        help="Folder where trajectory CSVs and odometry.csv will be written.",
    )
    parser.add_argument(
        "--skip-sec",
        type=float,
        default=0.0,
        help=(
            "Minimum header timestamp difference in seconds between saved "
            "trajectory snapshots. Defaults to 0, saving every trajectory message."
        ),
    )
    parser.add_argument(
        "--trajectory-topic",
        default="/ov_slam/trajectory",
        help="Trajectory topic containing nav_msgs/msg/Path snapshots.",
    )
    parser.add_argument(
        "--trajectory-final-topic",
        default="/ov_slam/trajectory_final",
        help="Final trajectory topic containing one nav_msgs/msg/Path snapshot.",
    )
    parser.add_argument(
        "--odometry-topic",
        default="/ov_slam/odometry",
        help="Odometry topic containing nav_msgs/msg/Odometry poses.",
    )
    parser.add_argument(
        "--final-align-max-diff",
        type=float,
        default=0.01,
        help=(
            "Maximum timestamp difference in seconds when associating the final "
            "trajectory to the last trajectory snapshot for SE(3) alignment. "
            "Default: 0.01."
        ),
    )
    parser.add_argument(
        "--no-align-final-trajectory",
        action="store_true",
        help="Disable SE(3) alignment before writing the final trajectory CSV.",
    )
    parser.add_argument(
        "--storage-id",
        default="mcap",
        help="rosbag2 storage id. Defaults to mcap.",
    )
    return parser.parse_args()


def open_reader(bag_path, storage_id):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)
    return reader


def topic_type_map(reader):
    return {
        topic_metadata.name: topic_metadata.type
        for topic_metadata in reader.get_all_topics_and_types()
    }


def validate_topic(topic_types, topic, expected_type):
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types.keys()))
        raise RuntimeError(f"Topic '{topic}' not found in bag. Available topics: {available}")

    actual_type = topic_types[topic]
    if actual_type != expected_type:
        raise RuntimeError(
            f"Topic '{topic}' has type '{actual_type}', expected '{expected_type}'"
        )


def validate_optional_topic(topic_types, topic, expected_type):
    if topic not in topic_types:
        return False

    validate_topic(topic_types, topic, expected_type)
    return True


def set_topic_filter(reader, topics):
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))


def stamp_to_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def stamp_filename(stamp):
    return f"{stamp.sec:010d}_{stamp.nanosec:09d}.csv"


def should_save_trajectory(stamp, last_saved_stamp_ns, skip_sec):
    if last_saved_stamp_ns is None or skip_sec <= 0.0:
        return True
    return (stamp_to_nanoseconds(stamp) - last_saved_stamp_ns) >= int(skip_sec * 1e9)


def pose_row(counter, stamp, pose):
    position = pose.position
    orientation = pose.orientation
    return [
        counter,
        stamp.sec,
        stamp.nanosec,
        f"{position.x:.12f}",
        f"{position.y:.12f}",
        f"{position.z:.12f}",
        f"{orientation.x:.12f}",
        f"{orientation.y:.12f}",
        f"{orientation.z:.12f}",
        f"{orientation.w:.12f}",
    ]


def write_trajectory_csv(trajectory, output_path):
    with output_path.open("w", newline="") as csv_file:
        csv_file.write(CSV_HEADER)
        writer = csv.writer(csv_file)
        for counter, pose_stamped in enumerate(trajectory.poses):
            writer.writerow(
                pose_row(counter, pose_stamped.header.stamp, pose_stamped.pose)
            )


def write_odometry_row(writer, counter, odometry):
    writer.writerow(pose_row(counter, odometry.header.stamp, odometry.pose.pose))


def path_msg_to_evo_trajectory(trajectory):
    timestamps = []
    positions = []
    quaternions_wxyz = []

    for pose_stamped in trajectory.poses:
        stamp = pose_stamped.header.stamp
        pose = pose_stamped.pose
        timestamps.append(stamp.sec + stamp.nanosec * 1e-9)
        positions.append([pose.position.x, pose.position.y, pose.position.z])
        quaternions_wxyz.append(
            [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ]
        )

    if not timestamps:
        raise RuntimeError("Cannot align an empty trajectory")

    return PoseTrajectory3D(
        positions_xyz=np.asarray(positions, dtype=float),
        orientations_quat_wxyz=np.asarray(quaternions_wxyz, dtype=float),
        timestamps=np.asarray(timestamps, dtype=float),
    )


def evo_trajectory_to_path_msg(evo_trajectory, template_trajectory):
    trajectory = copy.deepcopy(template_trajectory)
    if evo_trajectory.num_poses != len(trajectory.poses):
        raise RuntimeError(
            "Aligned evo trajectory pose count does not match source Path message"
        )

    for pose_stamped, position, quat_wxyz in zip(
        trajectory.poses,
        evo_trajectory.positions_xyz,
        evo_trajectory.orientations_quat_wxyz,
    ):
        pose_stamped.pose.position.x = float(position[0])
        pose_stamped.pose.position.y = float(position[1])
        pose_stamped.pose.position.z = float(position[2])
        pose_stamped.pose.orientation.w = float(quat_wxyz[0])
        pose_stamped.pose.orientation.x = float(quat_wxyz[1])
        pose_stamped.pose.orientation.y = float(quat_wxyz[2])
        pose_stamped.pose.orientation.z = float(quat_wxyz[3])

    return trajectory


def align_final_trajectory_to_last(final_trajectory, last_trajectory, max_diff):
    reference = path_msg_to_evo_trajectory(last_trajectory)
    final = path_msg_to_evo_trajectory(final_trajectory)
    reference_assoc, final_assoc = sync.associate_trajectories(
        reference, final, max_diff=max_diff
    )

    if reference_assoc.num_poses < 3:
        raise RuntimeError(
            "Need at least 3 timestamp-associated poses to align final trajectory; "
            f"got {reference_assoc.num_poses}"
        )

    rotation, translation, _ = geometry.umeyama_alignment(
        final_assoc.positions_xyz.T,
        reference_assoc.positions_xyz.T,
        with_scale=False,
    )

    aligned_final = copy.deepcopy(final)
    aligned_final.transform(lie.se3(rotation, translation))
    return evo_trajectory_to_path_msg(aligned_final, final_trajectory), reference_assoc.num_poses


def extract_bag_data(reader, args, output_folder):
    topic_types = topic_type_map(reader)
    validate_topic(topic_types, args.trajectory_topic, "nav_msgs/msg/Path")
    has_final_trajectory_topic = validate_optional_topic(
        topic_types, args.trajectory_final_topic, "nav_msgs/msg/Path"
    )
    validate_topic(topic_types, args.odometry_topic, "nav_msgs/msg/Odometry")
    filtered_topics = [args.trajectory_topic, args.odometry_topic]
    if has_final_trajectory_topic:
        filtered_topics.append(args.trajectory_final_topic)
    set_topic_filter(reader, filtered_topics)

    last_saved_trajectory_stamp_ns = None
    trajectory_count = 0
    final_trajectory_count = 0
    skipped_trajectory_count = 0
    saved_trajectory_stamps = set()
    odometry_count = 0
    last_trajectory = None
    final_trajectories = []

    with (output_folder / "odometry.csv").open("w", newline="") as odometry_file:
        odometry_file.write(CSV_HEADER)
        odometry_writer = csv.writer(odometry_file)

        while reader.has_next():
            topic, serialized_msg, _ = reader.read_next()

            if topic == args.trajectory_topic:
                trajectory = deserialize_message(serialized_msg, PathMsg)
                last_trajectory = trajectory
                header_stamp = trajectory.header.stamp
                if should_save_trajectory(
                    header_stamp, last_saved_trajectory_stamp_ns, args.skip_sec
                ):
                    output_path = output_folder / stamp_filename(header_stamp)
                    write_trajectory_csv(trajectory, output_path)
                    last_saved_trajectory_stamp_ns = stamp_to_nanoseconds(header_stamp)
                    saved_trajectory_stamps.add(stamp_to_nanoseconds(header_stamp))
                    trajectory_count += 1
                else:
                    skipped_trajectory_count += 1

            elif topic == args.trajectory_final_topic:
                final_trajectory = deserialize_message(serialized_msg, PathMsg)
                final_trajectories.append(final_trajectory)

            elif topic == args.odometry_topic:
                odometry = deserialize_message(serialized_msg, Odometry)
                write_odometry_row(odometry_writer, odometry_count, odometry)
                odometry_count += 1

    if final_trajectories and last_trajectory is None:
        raise RuntimeError(
            "Cannot align final trajectory because no "
            f"'{args.trajectory_topic}' message was read"
        )

    for final_trajectory in final_trajectories:
        if not args.no_align_final_trajectory:
            final_trajectory, associated_count = align_final_trajectory_to_last(
                final_trajectory,
                last_trajectory,
                args.final_align_max_diff,
            )
            print(
                "Aligned final trajectory to last trajectory snapshot with "
                f"{associated_count} associated poses"
            )
        header_stamp = final_trajectory.header.stamp
        header_stamp_ns = stamp_to_nanoseconds(header_stamp)
        output_path = output_folder / stamp_filename(header_stamp)
        if header_stamp_ns in saved_trajectory_stamps:
            print(
                "Overwriting trajectory CSV with final trajectory for "
                f"timestamp {header_stamp.sec}.{header_stamp.nanosec:09d}: {output_path}"
            )
        write_trajectory_csv(final_trajectory, output_path)
        saved_trajectory_stamps.add(header_stamp_ns)
        final_trajectory_count += 1

    if not has_final_trajectory_topic:
        print(f"Final trajectory topic '{args.trajectory_final_topic}' not found in bag")

    return (
        trajectory_count,
        final_trajectory_count,
        skipped_trajectory_count,
        odometry_count,
    )


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser()
    output_folder = Path(args.output_folder).expanduser()

    if args.skip_sec < 0.0:
        raise RuntimeError("--skip-sec must be non-negative")
    if args.final_align_max_diff < 0.0:
        raise RuntimeError("--final-align-max-diff must be non-negative")
    if not bag_path.exists():
        raise RuntimeError(f"Bag path does not exist: {bag_path}")

    output_folder.mkdir(parents=True, exist_ok=True)
    reader = open_reader(bag_path, args.storage_id)
    (
        trajectory_count,
        final_trajectory_count,
        skipped_trajectory_count,
        odometry_count,
    ) = extract_bag_data(reader, args, output_folder)

    print(f"Wrote {trajectory_count} trajectory CSV files to {output_folder}")
    print(f"Wrote {final_trajectory_count} final trajectory CSV files to {output_folder}")
    print(f"Skipped {skipped_trajectory_count} trajectory messages")
    print(f"Wrote {odometry_count} odometry poses to {output_folder / 'odometry.csv'}")


if __name__ == "__main__":
    main()
