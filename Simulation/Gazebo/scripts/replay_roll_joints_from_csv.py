#!/usr/bin/env python3

import argparse
import math
import subprocess
import time
from pathlib import Path

import pandas as pd


ROLL_JOINT_TOPICS = {
    "RL1_wrap": "/step/right_hip_roll_joint/cmd_pos",
    "RL5_wrap": "/step/right_ankle_roll_joint/cmd_pos",
    "LL1_wrap": "/step/left_hip_roll_joint/cmd_pos",
    "LL5_wrap": "/step/left_ankle_roll_joint/cmd_pos",
}


def publish_double(topic: str, value: float) -> None:
    subprocess.run(
        [
            "gz",
            "topic",
            "-t",
            topic,
            "-m",
            "gz.msgs.Double",
            "-p",
            f"data: {value}",
        ],
        check=False,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Replay STEP roll joint commands from Dynamics CSV into Gazebo topics."
    )
    parser.add_argument("csv_path", help="Path to Dynamics CSV")
    parser.add_argument("--start-frame", type=int, default=398)
    parser.add_argument("--end-frame", type=int, default=405)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument(
        "--hold-start",
        type=float,
        default=0.0,
        help="Hold the start-frame command before replay (seconds)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print commands without publishing to Gazebo",
    )
    args = parser.parse_args()

    if args.dt <= 0.0:
        raise ValueError("--dt must be greater than 0")
    if args.hold_start < 0.0:
        raise ValueError("--hold-start must be greater than or equal to 0")

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    df = pd.read_csv(csv_path)

    required_cols = ["frame"] + list(ROLL_JOINT_TOPICS.keys())
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in CSV: {missing}")

    selected = df[(df["frame"] >= args.start_frame) & (df["frame"] <= args.end_frame)]

    if selected.empty:
        raise ValueError(f"No frames found from {args.start_frame} to {args.end_frame}")

    print("[Gazebo Roll Joint CSV Replay]")
    print(f"csv: {csv_path}")
    print(f"frames: {args.start_frame}~{args.end_frame}")
    print(f"dt: {args.dt}")
    print(f"hold_start: {args.hold_start}")
    print(f"dry_run: {args.dry_run}")
    print()

    if args.hold_start > 0.0:
        start_row = selected.iloc[0]
        hold_frame = int(start_row["frame"])
        publish_count = math.ceil(args.hold_start / args.dt)

        print("[Hold Start Frame]")
        print(f"hold start frame: {hold_frame}")
        print(f"hold duration: {args.hold_start} s")
        print(f"publish count: {publish_count}")

        for publish_index in range(publish_count):
            print(f"hold publish {publish_index + 1}/{publish_count}")

            for col, topic in ROLL_JOINT_TOPICS.items():
                value = float(start_row[col])
                print(f"  {col:8s} -> {topic:40s} value={value:+.9f}")

                if not args.dry_run:
                    publish_double(topic, value)

            time.sleep(args.dt)

        print()

    for _, row in selected.iterrows():
        frame = int(row["frame"])
        print(f"frame {frame}")

        for col, topic in ROLL_JOINT_TOPICS.items():
            value = float(row[col])
            print(f"  {col:8s} -> {topic:40s} value={value:+.9f}")

            if not args.dry_run:
                publish_double(topic, value)

        time.sleep(args.dt)

    print()
    print("[DONE] Replay finished.")


if __name__ == "__main__":
    main()
