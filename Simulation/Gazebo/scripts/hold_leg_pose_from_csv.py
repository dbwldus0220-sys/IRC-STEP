#!/usr/bin/env python3
"""Repeatedly publish one CSV leg pose for free-base Gazebo checks."""

import argparse
import csv
import math
import time
from pathlib import Path

from replay_roll_joints_from_csv import (
    LEG_JOINT_TOPICS,
    GazeboDoublePublishers,
    command_from_csv,
)


DEFAULT_DT = 0.02
DEFAULT_HOLD_SECONDS = 10.0


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Hold one STEP leg pose from a replay CSV to check whether the "
            "free-base Gazebo model can maintain its initial posture."
        )
    )
    parser.add_argument("csv_path", type=Path, help="Path to Gazebo leg replay CSV")
    parser.add_argument(
        "--frame",
        type=int,
        default=None,
        help="Select this frame value instead of the first CSV data row",
    )
    parser.add_argument(
        "--use-gazebo-offsets",
        action="store_true",
        help="Apply the same Gazebo joint offsets as the replay script",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_DT,
        help=f"Publish period in seconds (default: {DEFAULT_DT})",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=DEFAULT_HOLD_SECONDS,
        help=f"Pose hold duration in seconds (default: {DEFAULT_HOLD_SECONDS})",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print commands without publishing to Gazebo",
    )
    args = parser.parse_args()

    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and greater than 0")
    if not math.isfinite(args.hold_seconds) or args.hold_seconds <= 0.0:
        parser.error("--hold-seconds must be finite and greater than 0")
    return args


def load_pose_row(csv_path: Path, requested_frame):
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    with csv_path.open(newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row")

        required_columns = list(LEG_JOINT_TOPICS)
        if requested_frame is not None:
            required_columns.append("frame")
        missing = [
            column for column in required_columns if column not in reader.fieldnames
        ]
        if missing:
            raise ValueError(
                "CSV is missing required column(s): " + ", ".join(missing)
            )

        first_row = None
        for row in reader:
            if first_row is None:
                first_row = row
            if requested_frame is None:
                return row
            try:
                row_frame = int(float(row["frame"]))
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"Invalid frame value in CSV: {row.get('frame')!r}"
                ) from error
            if row_frame == requested_frame:
                return row

    if first_row is None:
        raise ValueError("CSV contains no data rows")
    raise ValueError(f"Frame {requested_frame} was not found in {csv_path}")


def prepare_commands(row, use_gazebo_offsets: bool):
    commands = []
    for column, topic in LEG_JOINT_TOPICS.items():
        try:
            csv_value = float(row[column])
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Invalid numeric value for {column}: {row.get(column)!r}"
            ) from error
        if not math.isfinite(csv_value):
            raise ValueError(f"Non-finite value for {column}: {csv_value}")

        command = command_from_csv(
            column=column,
            csv_value=csv_value,
            base_values=None,
            use_gazebo_offsets=use_gazebo_offsets,
            mirror_left_pitch_chain=False,
        )
        commands.append((column, topic, csv_value, command))
    return commands


def print_commands(row, commands, args) -> None:
    frame_text = row.get("frame", "first data row")
    print("[HOLD LEG POSE]")
    print(f"CSV: {args.csv_path}")
    print(f"frame: {frame_text}")
    print(f"use_gazebo_offsets: {args.use_gazebo_offsets}")
    print(f"dt: {args.dt:.6f} s")
    print(f"hold_seconds: {args.hold_seconds:.6f} s")
    for column, topic, csv_value, command in commands:
        print(
            f"  {column:8s} -> {topic:40s} "
            f"csv={csv_value:+.9f} command={command:+.9f}"
        )


def main() -> int:
    args = parse_args()
    try:
        row = load_pose_row(args.csv_path, args.frame)
        commands = prepare_commands(row, args.use_gazebo_offsets)
    except (FileNotFoundError, ValueError) as error:
        print(f"[ERROR] {error}")
        return 1

    print_commands(row, commands, args)
    publish_count = max(1, math.ceil(args.hold_seconds / args.dt))
    print(f"publish_count: {publish_count}")

    if args.dry_run:
        print("[DRY RUN] Commands were not published.")
        return 0

    try:
        publishers = GazeboDoublePublishers(
            topics=LEG_JOINT_TOPICS.values(),
            dry_run=False,
        )
        start_time = time.monotonic()
        for publish_index in range(publish_count):
            publishers.publish(commands)
            next_publish_time = start_time + (publish_index + 1) * args.dt
            remaining = next_publish_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted; no further commands will be published.")
        return 130
    except RuntimeError as error:
        print(f"[ERROR] {error}")
        return 1

    elapsed = time.monotonic() - start_time
    print(
        f"[DONE] Held frame {row.get('frame', 'first data row')} with "
        f"{len(commands)} joints for {elapsed:.3f} s "
        f"({publish_count} publishes)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
