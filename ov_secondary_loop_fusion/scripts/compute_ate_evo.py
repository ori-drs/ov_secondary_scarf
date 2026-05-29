#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import numpy as np
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compute ATE between two CSV trajectories using evo."
    )
    parser.add_argument(
        "--gt",
        required=True,
        help="Ground-truth CSV path: counter,sec,nsec,x,y,z,qx,qy,qz,qw",
    )
    parser.add_argument(
        "--traj",
        required=True,
        help="Estimated trajectory CSV path: counter,sec,nsec,x,y,z,qx,qy,qz,qw",
    )
    parser.add_argument(
        "--max-diff",
        type=float,
        default=0.01,
        help="Maximum timestamp association difference in seconds. Default: 0.01.",
    )
    parser.add_argument(
        "--align",
        action="store_true",
        help="Align estimated trajectory to ground truth with SE(3) before ATE.",
    )
    parser.add_argument(
        "--correct-scale",
        action="store_true",
        help="Estimate scale during alignment. Usually false for VIO.",
    )
    parser.add_argument(
        "--align-origin",
        action="store_true",
        help="Only align the trajectory origins instead of full SE(3) alignment.",
    )
    return parser.parse_args()


def read_csv_trajectory(csv_path):
    timestamps = []
    positions = []
    quaternions_wxyz = []

    with csv_path.open("r", newline="") as csv_file:
        reader = csv.reader(row for row in csv_file if not row.lstrip().startswith("#"))
        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue
            if len(row) != 10:
                raise RuntimeError(
                    f"{csv_path}:{line_number} expected 10 columns, got {len(row)}"
                )

            sec = int(row[1])
            nsec = int(row[2])
            timestamps.append(sec + nsec * 1e-9)
            positions.append([float(row[3]), float(row[4]), float(row[5])])
            # evo expects quaternion order w, x, y, z.
            quaternions_wxyz.append(
                [float(row[9]), float(row[6]), float(row[7]), float(row[8])]
            )

    if not timestamps:
        raise RuntimeError(f"No poses found in {csv_path}")

    return PoseTrajectory3D(
        positions_xyz=np.asarray(positions, dtype=float),
        orientations_quat_wxyz=np.asarray(quaternions_wxyz, dtype=float),
        timestamps=np.asarray(timestamps, dtype=float),
    )


def main():
    args = parse_args()
    gt_path = Path(args.gt).expanduser()
    traj_path = Path(args.traj).expanduser()

    gt = read_csv_trajectory(gt_path)
    traj = read_csv_trajectory(traj_path)

    gt_assoc, traj_assoc = sync.associate_trajectories(gt, traj, max_diff=args.max_diff)

    if args.align_origin:
        traj_assoc.align_origin(gt_assoc)
    elif args.align or args.correct_scale:
        traj_assoc.align(gt_assoc, correct_scale=args.correct_scale)

    ape = metrics.APE(metrics.PoseRelation.translation_part)
    ape.process_data((gt_assoc, traj_assoc))
    stats = ape.get_all_statistics()

    print(f"associated_pairs {gt_assoc.num_poses}")
    print(f"rmse {stats['rmse']:.12f}")
    print(f"mean {stats['mean']:.12f}")
    print(f"median {stats['median']:.12f}")
    print(f"std {stats['std']:.12f}")
    print(f"min {stats['min']:.12f}")
    print(f"max {stats['max']:.12f}")
    print(f"sse {stats['sse']:.12f}")


if __name__ == "__main__":
    main()
