#!/usr/bin/env python3
"""Read and display STEP leg actual positions from Gazebo joint states."""

import argparse
import csv
import math
import time
from pathlib import Path

from publish_leg_constant_pose import CONSTANT_POSE
from replay_roll_joints_from_csv import (
    LEG_JOINT_TOPICS,
    JointTrackingSubscriber,
    joint_state_message_type,
)


DEFAULT_TOPIC = "/step/leg_joint_states"
DEFAULT_DURATION = 10.0
DEFAULT_DT = 0.5

DOCUMENTED_LEG_JOINT_TARGETS = {
    topic.removeprefix("/step/").removesuffix("/cmd_pos"): CONSTANT_POSE[column]
    for column, topic in LEG_JOINT_TOPICS.items()
}
LEG_JOINT_NAMES = tuple(DOCUMENTED_LEG_JOINT_TARGETS)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Log the 12 STEP leg actual positions from the read-only "
            "/step/leg_joint_states Gazebo topic."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION,
        help=f"Logging duration in seconds (default: {DEFAULT_DURATION})",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_DT,
        help=f"Console and CSV sample period in seconds (default: {DEFAULT_DT})",
    )
    parser.add_argument(
        "--csv-output",
        type=Path,
        default=None,
        metavar="PATH",
        help="Optionally save samples to this CSV file",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"Joint-state topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--no-target",
        action="store_true",
        help="Hide the constant-pose target and error columns on the console",
    )
    parser.add_argument(
        "--target-mode",
        choices=("default", "zero"),
        default="default",
        help=(
            "Target used for display and error calculation: documented "
            "default pose or all-zero pose (default: default)"
        ),
    )
    args = parser.parse_args()

    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and greater than 0")
    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and greater than 0")
    if not args.topic:
        parser.error("--topic must not be empty")
    return args


def format_value(value: float) -> str:
    return "nan" if math.isnan(value) else f"{value:+.9f}"


def print_sample(
    sample_index: int,
    elapsed: float,
    positions,
    targets,
    show_target: bool,
):
    print(f"\n[SAMPLE {sample_index}] time={elapsed:.3f} s")
    if show_target:
        print("joint_name, actual_position_rad, target_position_rad, error_rad")
    else:
        print("joint_name, actual_position_rad")

    rows = []
    for joint_name in LEG_JOINT_NAMES:
        actual = positions.get(joint_name, math.nan)
        target = targets[joint_name]
        error = target - actual if math.isfinite(actual) else math.nan
        if show_target:
            print(
                f"{joint_name}, {format_value(actual)}, "
                f"{target:+.9f}, {format_value(error)}"
            )
        else:
            print(f"{joint_name}, {format_value(actual)}")
        rows.append((joint_name, actual, target, error))
    return rows


def main() -> int:
    args = parse_args()
    targets = (
        {joint_name: 0.0 for joint_name in LEG_JOINT_NAMES}
        if args.target_mode == "zero"
        else DOCUMENTED_LEG_JOINT_TARGETS
    )
    message_type = joint_state_message_type(args.topic)
    print("[GAZEBO LEG ACTUAL POSITION LOGGER]")
    print(f"topic: {args.topic}")
    print(f"message_type: {message_type or 'unknown (expected gz.msgs.Model)'}")
    print(f"duration: {args.duration:.6f} s")
    print(f"dt: {args.dt:.6f} s")
    print(f"target_mode: {args.target_mode}")
    if message_type not in (None, "gz.msgs.Model"):
        print(
            f"[WARNING] Expected gz.msgs.Model but {args.topic} reports "
            f"{message_type}."
        )

    try:
        subscriber = JointTrackingSubscriber(args.topic, LEG_JOINT_NAMES)
    except (ImportError, RuntimeError) as error:
        print(f"[ERROR] Could not subscribe to {args.topic}: {error}")
        return 1

    csv_file = None
    csv_writer = None
    if args.csv_output is not None:
        try:
            csv_file = args.csv_output.open("w", newline="", encoding="utf-8")
        except OSError as error:
            print(f"[ERROR] Could not open CSV output {args.csv_output}: {error}")
            return 1
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            (
                "sample",
                "time",
                "joint_name",
                "actual_position_rad",
                "target_position_rad",
                "error_rad",
            )
        )
        print(f"csv_output: {args.csv_output}")

    start_time = time.monotonic()
    sample_count = max(1, math.ceil(args.duration / args.dt))
    received_any_message = False
    try:
        for sample_index in range(sample_count):
            sample_time = start_time + (sample_index + 1) * args.dt
            remaining = sample_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

            positions, received_message = subscriber.snapshot()
            received_any_message = received_any_message or received_message
            elapsed = time.monotonic() - start_time
            rows = print_sample(
                sample_index + 1,
                elapsed,
                positions,
                targets,
                show_target=not args.no_target,
            )
            if csv_writer is not None:
                for joint_name, actual, target, error in rows:
                    csv_writer.writerow(
                        (
                            sample_index + 1,
                            f"{elapsed:.9f}",
                            joint_name,
                            actual,
                            target,
                            error,
                        )
                    )
                csv_file.flush()
    except KeyboardInterrupt:
        print("\n[STOP] Logging interrupted.")
        return_code = 130
    else:
        return_code = 0
    finally:
        if csv_file is not None:
            csv_file.close()

    if not received_any_message:
        print(f"[ERROR] No gz.msgs.Model messages were received from {args.topic}.")
        return 1

    print(f"\n[DONE] Logged {sample_count} samples for {len(LEG_JOINT_NAMES)} joints.")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
