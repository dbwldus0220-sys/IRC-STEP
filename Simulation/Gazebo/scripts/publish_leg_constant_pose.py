#!/usr/bin/env python3
"""Publish a constant 12-joint STEP leg pose for Gazebo diagnostics."""

import argparse
import math
import time

from gazebo_leg_transport import (
    LEG_JOINT_TOPICS,
    GazeboDoublePublishers,
)


DEFAULT_DURATION = 10.0
DEFAULT_DT = 0.05
DEFAULT_RAMP_DURATION = 2.0

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

# Minimum-L2 candidate with at least 20 mm static margin for the free-base test.
STANDING_CANDIDATE_POSE = {
    **CONSTANT_POSE,
    "RL2_wrap": -0.878,
    "RL3_wrap": 0.130,
    "RL4_wrap": -0.650,
    "LL2_wrap": 0.878,
    "LL3_wrap": -0.130,
    "LL4_wrap": 0.650,
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
        "--ramp-duration",
        type=float,
        default=DEFAULT_RAMP_DURATION,
        help=(
            "Time in seconds to ramp from zero to the documented pose "
            f"(default: {DEFAULT_RAMP_DURATION})"
        ),
    )
    parser.add_argument(
        "--progress-every",
        type=int,
        default=20,
        metavar="N",
        help="Print progress every N publishes (default: 20)",
    )
    pose_group = parser.add_mutually_exclusive_group()
    pose_group.add_argument(
        "--zero-pose",
        action="store_true",
        help="Publish 0.0 rad to all 12 leg joints instead of the documented pose",
    )
    pose_group.add_argument(
        "--standing-candidate",
        action="store_true",
        help=(
            "Use the static minimum-L2 standing candidate selected for the "
            "gravity free-base initialization test"
        ),
    )
    parser.add_argument(
        "--hold-initial-pose",
        action="store_true",
        help=(
            "Publish the selected target immediately without a ramp; use with "
            "a model already initialized to the same joint pose"
        ),
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
    if not math.isfinite(args.ramp_duration) or args.ramp_duration < 0.0:
        parser.error("--ramp-duration must be finite and non-negative")
    if args.progress_every <= 0:
        parser.error("--progress-every must be greater than 0")
    return args


def build_commands(zero_pose: bool, standing_candidate: bool = False):
    if set(CONSTANT_POSE) != set(LEG_JOINT_TOPICS):
        missing = sorted(set(LEG_JOINT_TOPICS) - set(CONSTANT_POSE))
        extra = sorted(set(CONSTANT_POSE) - set(LEG_JOINT_TOPICS))
        raise RuntimeError(
            "Constant-pose mapping does not match replay joint mapping: "
            f"missing={missing}, extra={extra}"
        )

    selected_pose = (
        STANDING_CANDIDATE_POSE if standing_candidate else CONSTANT_POSE
    )
    commands = []
    for column, topic in LEG_JOINT_TOPICS.items():
        command = 0.0 if zero_pose else selected_pose[column]
        commands.append((column, topic, command, command))
    return commands


def smoothstep_interpolation(elapsed: float, ramp_duration: float) -> float:
    """Return a zero-to-one smoothstep ramp factor."""
    if ramp_duration == 0.0:
        return 1.0
    linear_factor = min(1.0, max(0.0, elapsed / ramp_duration))
    return linear_factor * linear_factor * (3.0 - 2.0 * linear_factor)


def interpolate_commands(commands, interpolation_factor: float):
    return [
        (column, topic, source, target * interpolation_factor)
        for column, topic, source, target in commands
    ]


def print_configuration(args, commands, publish_count: int) -> None:
    print("[GAZEBO CONSTANT LEG POSE]")
    pose_mode = "zero"
    if args.standing_candidate:
        pose_mode = "standing candidate"
    elif not args.zero_pose:
        pose_mode = "documented default"
    print(f"pose_mode: {pose_mode}")
    print(f"duration: {args.duration:.6f} s")
    print(f"dt: {args.dt:.6f} s")
    ramp_mode = (
        "disabled (initial-pose hold)"
        if args.hold_initial_pose
        else "smoothstep"
    )
    print(f"ramp_duration: {args.ramp_duration:.6f} s ({ramp_mode})")
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
        commands = build_commands(args.zero_pose, args.standing_candidate)
    except RuntimeError as error:
        print(f"[ERROR] {error}")
        return 1

    publish_count = max(1, math.ceil(args.duration / args.dt))
    print_configuration(args, commands, publish_count)

    if args.dry_run:
        initial_factor = (
            1.0
            if args.hold_initial_pose
            else smoothstep_interpolation(0.0, args.ramp_duration)
        )
        final_factor = smoothstep_interpolation(
            args.ramp_duration,
            args.ramp_duration,
        )
        initial_phase = "hold" if initial_factor >= 1.0 else "ramp"
        print(
            "[DRY RUN] ramp preview: "
            f"phase={initial_phase}, interpolation={initial_factor:.6f} -> "
            f"phase=hold, interpolation={final_factor:.6f}"
        )
        print("[DRY RUN] Commands were not published.")
        return 0

    try:
        publishers = GazeboDoublePublishers(
            topics=LEG_JOINT_TOPICS.values(),
            dry_run=False,
        )
        start_time = time.monotonic()
        for publish_index in range(publish_count):
            elapsed_before_publish = time.monotonic() - start_time
            if publish_index == 0:
                elapsed_before_publish = 0.0
            interpolation_factor = (
                1.0
                if args.hold_initial_pose
                else smoothstep_interpolation(
                    elapsed_before_publish,
                    args.ramp_duration,
                )
            )
            current_commands = interpolate_commands(
                commands,
                interpolation_factor,
            )
            publishers.publish(current_commands)
            completed = publish_index + 1
            if (
                completed == 1
                or completed % args.progress_every == 0
                or completed == publish_count
            ):
                elapsed = time.monotonic() - start_time
                ramp_phase = (
                    "hold" if interpolation_factor >= 1.0 else "ramp"
                )
                print(
                    f"[PROGRESS] {completed}/{publish_count} publishes, "
                    f"elapsed={elapsed:.3f} s, phase={ramp_phase}, "
                    f"interpolation={interpolation_factor:.6f}"
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
        f"[DONE] Published {len(commands)} joint commands "
        f"{publish_count} times in {elapsed:.3f} s."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
