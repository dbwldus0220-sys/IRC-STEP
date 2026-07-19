#!/usr/bin/env python3
"""Publish and track one STEP leg joint position command in Gazebo."""

import argparse
import csv
import math
import time
from pathlib import Path

from replay_roll_joints_from_csv import (
    LEG_JOINT_TOPICS,
    GazeboDoublePublishers,
    JointTrackingSubscriber,
)


JOINT_TOPICS = {
    topic.removeprefix("/step/").removesuffix("/cmd_pos"): topic
    for topic in LEG_JOINT_TOPICS.values()
}
JOINT_STATE_TOPIC = "/step/leg_joint_states"
DEFAULT_RAMP_DURATION = 2.0


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Continuously publish one STEP leg joint position command and "
            "track its actual Gazebo position."
        )
    )
    parser.add_argument(
        "--joint-name",
        required=True,
        choices=tuple(JOINT_TOPICS),
        help="Leg joint to test",
    )
    parser.add_argument(
        "--target",
        required=True,
        type=float,
        help="Constant target position in radians",
    )
    parser.add_argument(
        "--duration",
        required=True,
        type=float,
        help="Test duration in seconds",
    )
    parser.add_argument(
        "--dt",
        required=True,
        type=float,
        help="Command publish and logging period in seconds",
    )
    parser.add_argument(
        "--ramp-duration",
        type=float,
        default=DEFAULT_RAMP_DURATION,
        help=(
            "Smoothstep ramp time from 0.0 rad to --target in seconds "
            f"(default: {DEFAULT_RAMP_DURATION})"
        ),
    )
    parser.add_argument(
        "--csv-output",
        type=Path,
        default=None,
        metavar="PATH",
        help="Optionally save target, actual, and error samples to CSV",
    )
    args = parser.parse_args()

    if not math.isfinite(args.target):
        parser.error("--target must be finite")
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and greater than 0")
    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and greater than 0")
    if not math.isfinite(args.ramp_duration) or args.ramp_duration < 0.0:
        parser.error("--ramp-duration must be finite and greater than or equal to 0")
    return args


def format_position(value: float) -> str:
    return "nan" if math.isnan(value) else f"{value:+.9f}"


def ramp_target(final_target: float, elapsed: float, ramp_duration: float) -> float:
    if ramp_duration == 0.0 or elapsed >= ramp_duration:
        return final_target
    progress = max(0.0, elapsed / ramp_duration)
    smoothstep = progress * progress * (3.0 - 2.0 * progress)
    return final_target * smoothstep


def main() -> int:
    args = parse_args()
    command_topic = JOINT_TOPICS[args.joint_name]

    print("[GAZEBO SINGLE JOINT CMD_POS TEST]")
    print(f"joint_name: {args.joint_name}")
    print(f"command_topic: {command_topic}")
    print(f"joint_state_topic: {JOINT_STATE_TOPIC}")
    print(f"final_target_rad: {args.target:+.9f}")
    print(f"duration: {args.duration:.6f} s")
    print(f"dt: {args.dt:.6f} s")
    print(f"ramp_duration: {args.ramp_duration:.6f} s (smoothstep)")
    if args.duration < args.ramp_duration:
        print(
            "[WARNING] --duration is shorter than --ramp-duration; the "
            "published command will not reach the final target."
        )

    try:
        subscriber = JointTrackingSubscriber(
            JOINT_STATE_TOPIC,
            (args.joint_name,),
        )
        publishers = GazeboDoublePublishers(
            topics=(command_topic,),
            dry_run=False,
        )
    except (ImportError, RuntimeError) as error:
        print(f"[ERROR] Gazebo transport setup failed: {error}")
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
                "target_position_rad",
                "actual_position_rad",
                "error_rad",
            )
        )
        print(f"csv_output: {args.csv_output}")

    sample_count = max(1, math.ceil(args.duration / args.dt))
    received_any_message = False
    start_time = time.monotonic()
    print("sample, time, joint_name, target_rad, actual_rad, error_rad")

    try:
        for sample_index in range(sample_count):
            publish_elapsed = time.monotonic() - start_time
            current_target = (
                0.0
                if sample_index == 0 and args.ramp_duration > 0.0
                else ramp_target(
                    args.target,
                    publish_elapsed,
                    args.ramp_duration,
                )
            )
            command = (
                args.joint_name,
                command_topic,
                current_target,
                current_target,
            )
            publishers.publish((command,))

            completed = sample_index + 1
            sample_time = start_time + completed * args.dt
            remaining = sample_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

            positions, received_message = subscriber.snapshot()
            received_any_message = received_any_message or received_message
            actual = positions.get(args.joint_name, math.nan)
            error = current_target - actual if math.isfinite(actual) else math.nan
            elapsed = time.monotonic() - start_time

            print(
                f"{completed}, {elapsed:.6f}, {args.joint_name}, "
                f"{current_target:+.9f}, {format_position(actual)}, "
                f"{format_position(error)}"
            )
            if csv_writer is not None:
                csv_writer.writerow(
                    (
                        completed,
                            f"{elapsed:.9f}",
                            args.joint_name,
                            current_target,
                            actual,
                        error,
                    )
                )
                csv_file.flush()
    except KeyboardInterrupt:
        print("\n[STOP] Test interrupted; no further commands will be published.")
        return_code = 130
    except RuntimeError as error:
        print(f"[ERROR] Gazebo publish failed: {error}")
        return_code = 1
    else:
        return_code = 0
    finally:
        if csv_file is not None:
            csv_file.close()

    if not received_any_message:
        print(f"[ERROR] No joint states were received from {JOINT_STATE_TOPIC}.")
        return 1

    print(f"[DONE] Published and logged {sample_count} samples.")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
