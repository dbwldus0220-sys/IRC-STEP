#!/usr/bin/env python3
"""Publish a constant 12-joint STEP leg pose for Gazebo diagnostics."""

import argparse
import math
import time

from replay_roll_joints_from_csv import (
    LEG_JOINT_TOPICS,
    GazeboDoublePublishers,
)


DEFAULT_DURATION = 10.0
DEFAULT_DT = 0.05

CONSTANT_POSE = {
    "RL0_wrap": 0.0,
    "RL1_wrap": 0.10,
    "RL2_wrap": -0.63,
    "RL3_wrap": 0.15,
    "RL4_wrap": -0.91,
    "RL5_wrap": -0.10,
    "LL0_wrap": 0.0,
    "LL1_wrap": -0.10,
    "LL2_wrap": 0.63,
    "LL3_wrap": -0.15,
    "LL4_wrap": 0.91,
    "LL5_wrap": 0.10,
}


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Repeatedly publish the fixed STEP leg pose to Gazebo without "
            "reading a command CSV."
        )
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION,
        help=f"Publish duration in seconds (default: {DEFAULT_DURATION})",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_DT,
        help=f"Publish period in seconds (default: {DEFAULT_DT})",
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=20,
        metavar="N",
        help="Print progress every N publishes (default: 20)",
    )
    parser.add_argument(
        "--zero-pose",
        action="store_true",
        help="Publish 0.0 rad to all 12 leg joints instead of the documented pose",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print and validate commands without publishing to Gazebo",
    )
    args = parser.parse_args()

    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and greater than 0")
    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and greater than 0")
    if args.progress_every <= 0:
        parser.error("--progress-every must be greater than 0")
    return args


def build_commands(zero_pose: bool):
    if set(CONSTANT_POSE) != set(LEG_JOINT_TOPICS):
        missing = sorted(set(LEG_JOINT_TOPICS) - set(CONSTANT_POSE))
        extra = sorted(set(CONSTANT_POSE) - set(LEG_JOINT_TOPICS))
        raise RuntimeError(
            "Constant-pose mapping does not match replay joint mapping: "
            f"missing={missing}, extra={extra}"
        )

    commands = []
    for column, topic in LEG_JOINT_TOPICS.items():
        command = 0.0 if zero_pose else CONSTANT_POSE[column]
        commands.append((column, topic, command, command))
    return commands


def print_configuration(args, commands, publish_count: int) -> None:
    print("[GAZEBO CONSTANT LEG POSE]")
    print(f"pose_mode: {'zero' if args.zero_pose else 'documented default'}")
    print(f"duration: {args.duration:.6f} s")
    print(f"dt: {args.dt:.6f} s")
    print(f"publish_count: {publish_count}")
    for column, topic, _, command in commands:
        joint_name = topic.removeprefix("/step/").removesuffix("/cmd_pos")
        print(
            f"  {column:8s} {joint_name:31s} "
            f"command={command:+.9f} topic={topic}"
        )


def main() -> int:
    args = parse_args()
    try:
        commands = build_commands(args.zero_pose)
    except RuntimeError as error:
        print(f"[ERROR] {error}")
        return 1

    publish_count = max(1, math.ceil(args.duration / args.dt))
    print_configuration(args, commands, publish_count)

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
            completed = publish_index + 1
            if (
                completed == 1
                or completed % args.progress_every == 0
                or completed == publish_count
            ):
                elapsed = time.monotonic() - start_time
                print(
                    f"[PROGRESS] {completed}/{publish_count} publishes, "
                    f"elapsed={elapsed:.3f} s"
                )

            next_publish_time = start_time + completed * args.dt
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
        f"[DONE] Published {len(commands)} constant joint commands "
        f"{publish_count} times in {elapsed:.3f} s."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
