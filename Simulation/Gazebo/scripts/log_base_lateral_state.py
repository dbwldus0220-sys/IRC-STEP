#!/usr/bin/env python3

import argparse
import csv
import math
import signal
import threading
import time
from pathlib import Path

from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


DEFAULT_JOINT_TOPIC = "/world/step_test/model/step_humanoid/joint_state"
DEFAULT_POSE_TOPIC = "/world/step_test/dynamic_pose/info"
DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "logs"
    / "base_lateral_scale000_pose_log.csv"
)
JOINT_NAME = "base_lateral_to_world"
MODEL_NAME = "step_humanoid"
LINK_NAME = "base_link"
CSV_COLUMNS = (
    "time",
    "base_lateral_position",
    "base_lateral_velocity",
    "model_x",
    "model_y",
    "model_z",
    "model_qx",
    "model_qy",
    "model_qz",
    "model_qw",
)


def message_time_seconds(message):
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nsec) * 1e-9
    return value if value > 0.0 else None


def is_named(name: str, target: str) -> bool:
    return name == target or name.endswith(f"::{target}")


def pose_values(pose):
    return (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


class BaseStateLogger:
    def __init__(self, joint_topic: str, pose_topic: str):
        self.lock = threading.Lock()
        self.start_monotonic = time.monotonic()
        self.joint_time = None
        self.joint_position = None
        self.joint_velocity = None
        self.pose = None
        self.pose_source = None

        self.node = Node()
        self.node.subscribe(Model, joint_topic, self.on_joint_state)
        self.node.subscribe(Pose_V, pose_topic, self.on_pose)

    def on_joint_state(self, message):
        for joint in message.joint:
            if is_named(joint.name, JOINT_NAME):
                with self.lock:
                    self.joint_time = message_time_seconds(message)
                    self.joint_position = joint.axis1.position
                    self.joint_velocity = joint.axis1.velocity
                return

    def on_pose(self, message):
        model_pose = None
        for pose in message.pose:
            if is_named(pose.name, LINK_NAME):
                with self.lock:
                    self.pose = pose_values(pose)
                    self.pose_source = pose.name
                return
            if is_named(pose.name, MODEL_NAME):
                model_pose = pose

        if model_pose is not None:
            with self.lock:
                self.pose = pose_values(model_pose)
                self.pose_source = model_pose.name

    def snapshot(self):
        with self.lock:
            elapsed = time.monotonic() - self.start_monotonic
            log_time = self.joint_time if self.joint_time is not None else elapsed
            position = self.joint_position
            velocity = self.joint_velocity
            pose = self.pose
            pose_source = self.pose_source

        nan = math.nan
        return (
            log_time,
            position if position is not None else nan,
            velocity if velocity is not None else nan,
            *(pose if pose is not None else (nan,) * 7),
        ), position is not None, pose_source


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Log the STEP lateral base joint state and base_link/model pose "
            "from Gazebo Transport topics."
        )
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--joint-topic", default=DEFAULT_JOINT_TOPIC)
    parser.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    parser.add_argument("--rate", type=float, default=100.0, help="CSV sampling rate in Hz")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Stop after this many seconds; Ctrl-C stops logging when omitted",
    )
    args = parser.parse_args()

    if args.rate <= 0.0:
        parser.error("--rate must be greater than 0")
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be greater than 0")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    logger = BaseStateLogger(args.joint_topic, args.pose_topic)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop.set())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: stop.set())

    print(f"[Joint topic] {args.joint_topic}")
    print(f"[Pose topic]  {args.pose_topic}")
    print(f"[Output]      {args.output}")
    print("Waiting for base_lateral_to_world joint state (Ctrl-C to stop)...")

    period = 1.0 / args.rate
    start = time.monotonic()
    next_sample = start
    received_joint = False
    reported_pose_source = None

    with args.output.open("w", newline="") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(CSV_COLUMNS)

        while not stop.is_set():
            now = time.monotonic()
            if args.duration is not None and now - start >= args.duration:
                break

            row, has_joint, pose_source = logger.snapshot()
            if has_joint:
                received_joint = True
                writer.writerow(row)

            if pose_source is not None and pose_source != reported_pose_source:
                print(f"[Pose source] {pose_source}")
                reported_pose_source = pose_source

            next_sample += period
            wait_time = next_sample - time.monotonic()
            if wait_time > 0.0:
                stop.wait(wait_time)
            else:
                next_sample = time.monotonic()

    if not received_joint:
        raise RuntimeError(
            f"No {JOINT_NAME} state was received from {args.joint_topic}. "
            "Check that Gazebo is running with the lateral-slide SDF."
        )

    if reported_pose_source is None:
        print("[WARNING] No base_link or step_humanoid pose was received; pose columns are NaN.")
    print(f"Saved log: {args.output}")


if __name__ == "__main__":
    main()
