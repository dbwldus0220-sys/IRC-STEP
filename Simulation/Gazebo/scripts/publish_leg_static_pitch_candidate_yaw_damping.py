#!/usr/bin/env python3
"""Publish Candidate B with native-odometry base yaw-rate damping."""

import argparse
import csv
import math
import threading
import time
from pathlib import Path

from google.protobuf import symbol_database
from gz.msgs10 import (
    header_pb2,
    odometry_pb2,
    pose_pb2,
    quaternion_pb2,
    time_pb2,
    twist_pb2,
    vector3d_pb2,
)
from gz.transport13 import Node

from gazebo_leg_transport import LEG_JOINT_TOPICS, GazeboDoublePublishers
from publish_leg_static_pitch_candidate import build_targets


DEFAULT_HIP_PITCH_MAGNITUDE = 0.816
DEFAULT_KNEE_PITCH_MAGNITUDE = 0.138
DEFAULT_ANKLE_PITCH_MAGNITUDE = 0.714
DEFAULT_YAW_DAMPING_GAIN = 0.003
DEFAULT_YAW_RATE_DEADBAND = 0.1
DEFAULT_HIP_YAW_CORRECTION_LIMIT = 0.005
DEFAULT_ODOM_TOPIC = "/step/base_odometry"
DEFAULT_DURATION = 10.0
DEFAULT_DT = 0.05

RIGHT_HIP_YAW_KEY = "RL0_wrap"
LEFT_HIP_YAW_KEY = "LL0_wrap"

_SYMBOLS = symbol_database.Default()
for _message_class in (
    time_pb2.Time,
    header_pb2.Header,
    header_pb2.Header.Map,
    vector3d_pb2.Vector3d,
    quaternion_pb2.Quaternion,
    pose_pb2.Pose,
    twist_pb2.Twist,
    odometry_pb2.Odometry,
):
    _SYMBOLS.RegisterMessage(_message_class)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hip-pitch-magnitude", type=float,
                        default=DEFAULT_HIP_PITCH_MAGNITUDE, metavar="H")
    parser.add_argument("--knee-pitch-magnitude", type=float,
                        default=DEFAULT_KNEE_PITCH_MAGNITUDE, metavar="K")
    parser.add_argument("--ankle-pitch-magnitude", type=float,
                        default=DEFAULT_ANKLE_PITCH_MAGNITUDE, metavar="A")
    parser.add_argument("--yaw-damping-gain", type=float,
                        default=DEFAULT_YAW_DAMPING_GAIN)
    parser.add_argument("--yaw-rate-deadband", type=float,
                        default=DEFAULT_YAW_RATE_DEADBAND)
    parser.add_argument("--hip-yaw-correction-limit", type=float,
                        default=DEFAULT_HIP_YAW_CORRECTION_LIMIT)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--command-log", type=Path, default=None, metavar="CSV")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="validate fixed targets and logging without Gazebo transport",
    )
    args = parser.parse_args()
    numeric = (
        args.hip_pitch_magnitude, args.knee_pitch_magnitude,
        args.ankle_pitch_magnitude, args.yaw_damping_gain,
        args.yaw_rate_deadband, args.hip_yaw_correction_limit,
        args.duration, args.dt,
    )
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if not 0.60 <= args.hip_pitch_magnitude <= 0.90:
        parser.error("--hip-pitch-magnitude must be in [0.60, 0.90]")
    if not 0.10 <= args.knee_pitch_magnitude <= 0.30:
        parser.error("--knee-pitch-magnitude must be in [0.10, 0.30]")
    if not 0.60 <= args.ankle_pitch_magnitude <= 0.95:
        parser.error("--ankle-pitch-magnitude must be in [0.60, 0.95]")
    if args.yaw_damping_gain < 0.0:
        parser.error("--yaw-damping-gain must be non-negative")
    if args.yaw_rate_deadband < 0.0:
        parser.error("--yaw-rate-deadband must be non-negative")
    if args.hip_yaw_correction_limit <= 0.0:
        parser.error("--hip-yaw-correction-limit must be positive")
    if args.duration <= 0.0 or args.dt <= 0.0:
        parser.error("--duration and --dt must be positive")
    return args


def yaw_damping_correction(wz, gain, deadband, limit):
    """Return deadbanded rate and saturated common hip-yaw correction."""
    if abs(wz) <= deadband:
        omega_effective = 0.0
    else:
        omega_effective = math.copysign(abs(wz) - deadband, wz)
    correction = max(-limit, min(limit, -gain * omega_effective))
    return omega_effective, correction


class OdometryState:
    def __init__(self, topic, enabled=True):
        self.lock = threading.Lock()
        self.received = False
        self.simulation_time = math.nan
        self.base_wz = math.nan
        self.node = None
        self.callback = self.on_odometry
        self.subscribe_return = None
        if enabled:
            self.node = Node()
            self.subscribe_return = self.node.subscribe(
                odometry_pb2.Odometry, topic, self.callback
            )
            if self.subscribe_return is False:
                raise RuntimeError(f"failed to subscribe to {topic}")

    def on_odometry(self, message):
        stamp = message.header.stamp
        simulation_time = stamp.sec + stamp.nsec * 1e-9
        with self.lock:
            self.received = True
            self.simulation_time = simulation_time
            self.base_wz = float(message.twist.angular.z)

    def snapshot(self):
        with self.lock:
            return self.received, self.simulation_time, self.base_wz


def command_tuples(targets):
    return [
        (key, topic, targets[key], targets[key])
        for key, topic in LEG_JOINT_TOPICS.items()
    ]


def log_columns():
    return (
        "publish_index", "scheduled_publish_time", "wall_elapsed_time",
        "odometry_received", "odometry_simulation_time", "base_wz",
        "omega_eff", "hip_yaw_correction",
        "right_hip_yaw_target", "left_hip_yaw_target",
        "hip_pitch_magnitude", "knee_pitch_magnitude",
        "ankle_pitch_magnitude",
        *(f"{key}_command" for key in LEG_JOINT_TOPICS),
    )


def main():
    args = parse_args()
    fixed_targets = build_targets(
        args.hip_pitch_magnitude,
        args.knee_pitch_magnitude,
        args.ankle_pitch_magnitude,
    )
    odometry = OdometryState(args.odom_topic, enabled=not args.dry_run)
    publishers = GazeboDoublePublishers(
        list(LEG_JOINT_TOPICS.values()), args.dry_run
    )
    publish_count = max(1, math.ceil(args.duration / args.dt))

    print("[CANDIDATE B NATIVE-WZ YAW DAMPING]")
    print(f"odom_topic={args.odom_topic}")
    print(f"subscribe_return={odometry.subscribe_return}")
    print(f"H/K/A={args.hip_pitch_magnitude:.9f}/"
          f"{args.knee_pitch_magnitude:.9f}/"
          f"{args.ankle_pitch_magnitude:.9f}")
    print(f"gain={args.yaw_damping_gain:.9f} "
          f"deadband={args.yaw_rate_deadband:.9f} "
          f"limit={args.hip_yaw_correction_limit:.9f}")
    print("contact_gating=disabled filter=disabled yaw_angle_kp=disabled "
          "slew_limit=disabled")

    log_stream = None
    writer = None
    if args.command_log is not None:
        args.command_log.parent.mkdir(parents=True, exist_ok=True)
        log_stream = args.command_log.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(log_stream, fieldnames=log_columns())
        writer.writeheader()

    start = time.monotonic()
    try:
        for index in range(publish_count):
            scheduled = index * args.dt
            if not args.dry_run:
                delay = scheduled - (time.monotonic() - start)
                if delay > 0.0:
                    time.sleep(delay)
            wall_elapsed = scheduled if args.dry_run else time.monotonic() - start
            received, odom_time, base_wz = odometry.snapshot()
            if received:
                omega_eff, correction = yaw_damping_correction(
                    base_wz, args.yaw_damping_gain,
                    args.yaw_rate_deadband, args.hip_yaw_correction_limit,
                )
            else:
                omega_eff, correction = 0.0, 0.0

            targets = dict(fixed_targets)
            targets[RIGHT_HIP_YAW_KEY] = correction
            targets[LEFT_HIP_YAW_KEY] = correction
            if targets[RIGHT_HIP_YAW_KEY] != targets[LEFT_HIP_YAW_KEY]:
                raise RuntimeError("bilateral hip-yaw commands are not common-sign")
            if abs(correction) > args.hip_yaw_correction_limit + 1e-15:
                raise RuntimeError("hip-yaw correction exceeded its limit")
            publishers.publish(command_tuples(targets))

            if writer is not None:
                writer.writerow({
                    "publish_index": index,
                    "scheduled_publish_time": f"{scheduled:.9f}",
                    "wall_elapsed_time": f"{wall_elapsed:.9f}",
                    "odometry_received": int(received),
                    "odometry_simulation_time": (
                        f"{odom_time:.9f}" if received else "nan"
                    ),
                    "base_wz": f"{base_wz:+.12f}" if received else "nan",
                    "omega_eff": f"{omega_eff:+.12f}",
                    "hip_yaw_correction": f"{correction:+.12f}",
                    "right_hip_yaw_target": f"{correction:+.12f}",
                    "left_hip_yaw_target": f"{correction:+.12f}",
                    "hip_pitch_magnitude": f"{args.hip_pitch_magnitude:.9f}",
                    "knee_pitch_magnitude": f"{args.knee_pitch_magnitude:.9f}",
                    "ankle_pitch_magnitude": f"{args.ankle_pitch_magnitude:.9f}",
                    **{
                        f"{key}_command": f"{targets[key]:+.12f}"
                        for key in LEG_JOINT_TOPICS
                    },
                })
                log_stream.flush()
    except (OSError, RuntimeError) as error:
        print(f"[ERROR] {error}")
        return 1
    finally:
        if log_stream is not None:
            log_stream.close()

    if args.command_log is not None:
        print(f"Wrote command log: {args.command_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
