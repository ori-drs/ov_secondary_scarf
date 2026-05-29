#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import rosbag2_py
from nav_msgs.msg import Path as PathMsg
from rclpy.serialization import deserialize_message


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export the final saved ov_secondary trajectory from a rosbag2 bag to CSV."
    )
    parser.add_argument(
        "bag",
        help="Path to the rosbag2 directory, e.g. /path/to/output/ov_slam/ov_slam_bag",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="all_ov_poses.csv",
        help="Output CSV path. Defaults to all_ov_poses.csv in the current directory.",
    )
    parser.add_argument(
        "-t",
        "--topic",
        default="/ov_slam/trajectory",
        help="Trajectory topic containing nav_msgs/msg/Path snapshots.",
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


def read_final_trajectory(reader, topic):
    topic_types = {
        topic_metadata.name: topic_metadata.type
        for topic_metadata in reader.get_all_topics_and_types()
    }

    if topic not in topic_types:
        available = ", ".join(sorted(topic_types.keys()))
        raise RuntimeError(f"Topic '{topic}' not found in bag. Available topics: {available}")

    if topic_types[topic] != "nav_msgs/msg/Path":
        raise RuntimeError(
            f"Topic '{topic}' has type '{topic_types[topic]}', expected 'nav_msgs/msg/Path'"
        )

    final_trajectory = None
    while reader.has_next():
        msg_topic, serialized_msg, _ = reader.read_next()
        if msg_topic == topic:
            final_trajectory = deserialize_message(serialized_msg, PathMsg)

    if final_trajectory is None:
        raise RuntimeError(f"No messages found on topic '{topic}'")

    return final_trajectory


def write_csv(trajectory, output_path):
    with output_path.open("w", newline="") as csv_file:
        csv_file.write("# counter,sec,nsec,x,y,z,qx,qy,qz,qw\n")
        writer = csv.writer(csv_file)

        for counter, pose_stamped in enumerate(trajectory.poses):
            stamp = pose_stamped.header.stamp
            position = pose_stamped.pose.position
            orientation = pose_stamped.pose.orientation
            writer.writerow(
                [
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
            )


def main():
    args = parse_args()
    bag_path = Path(args.bag).expanduser()
    output_path = Path(args.output).expanduser()

    if not bag_path.exists():
        raise RuntimeError(f"Bag path does not exist: {bag_path}")

    reader = open_reader(bag_path, args.storage_id)
    trajectory = read_final_trajectory(reader, args.topic)
    write_csv(trajectory, output_path)
    print(f"Wrote {len(trajectory.poses)} poses to {output_path}")


if __name__ == "__main__":
    main()
