#!/usr/bin/env python3

import argparse
import csv
import math
import re
import subprocess
import threading
import time
from pathlib import Path

from replay_roll_joints_from_csv import (
    load_pose_protobuf_types,
    quaternion_to_rpy,
)


JOINT_CONFIGS = {
    "right_ankle_roll_joint": (
        "/step/right_ankle_roll_joint/cmd_pos",
        "right_foot_link",
    ),
    "left_ankle_roll_joint": (
        "/step/left_ankle_roll_joint/cmd_pos",
        "left_foot_link",
    ),
}
DEFAULT_COMMAND_VALUES = (0.10, -0.10, 0.20, -0.20)
DEFAULT_JOINT_STATE_TOPIC = "/step/leg_joint_states"
DEFAULT_POSE_TOPIC = "/world/step_test/pose/info"
DEFAULT_OUTPUT = Path("gazebo_ankle_roll_sign_orientation_log.csv")


class AnkleStateSubscriber:
    def __init__(self, topic: str):
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node

        self.lock = threading.Lock()
        self.positions = {}
        self.node = Node()
        subscribed = self.node.subscribe(Model, topic, self.on_joint_state)
        if subscribed is False:
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def on_joint_state(self, message):
        positions = {}
        for joint in message.joint:
            name = joint.name.replace("/", "::").rsplit("::", 1)[-1]
            if name in JOINT_CONFIGS:
                positions[name] = float(joint.axis1.position)
        with self.lock:
            self.positions.update(positions)

    def snapshot(self, joint_name: str):
        with self.lock:
            return self.positions.get(joint_name)


class FootPoseSubscriber:
    def __init__(
        self,
        topic: str,
        print_pose_names: bool = False,
        debug_pose: bool = False,
    ):
        from gz.transport13 import Node

        pose_module, pose_v_module, pose_v_type = load_pose_protobuf_types()
        self.pose_message_module = pose_module
        self.pose_v_message_module = pose_v_module
        self.message_type = pose_v_type
        self.lock = threading.Lock()
        self.orientations = {}
        self.received_names = []
        self.callback_count = 0
        self.last_pose_count = 0
        self.pose_access_succeeded = False
        self.pose_access_failed = False
        self.fallback_in_use = False
        self.fallback_process = None
        self.fallback_thread = None
        self.topic = topic
        self.print_pose_names = print_pose_names
        self.debug_pose = debug_pose
        self.node = Node()
        subscribed = self.node.subscribe(
            self.message_type, topic, self.on_pose
        )
        if subscribed is False:
            raise RuntimeError(f"Failed to subscribe Pose_V to {topic}")

    def on_pose(self, message):
        orientations = {}
        names = []
        pose_count = 0

        try:
            for pose in message.pose:
                pose_count += 1
                name = str(pose.name)
                names.append(name)
                if self.print_pose_names:
                    print(f"[POSE NAME][Pose_V] {name}")

                for _, target in JOINT_CONFIGS.values():
                    if self.name_matches(name, target):
                        quaternion = pose.orientation
                        orientations[target] = quaternion_to_rpy(
                            float(quaternion.x),
                            float(quaternion.y),
                            float(quaternion.z),
                            float(quaternion.w),
                        )
            pose_access_succeeded = True
        except TypeError as error:
            pose_access_succeeded = False
            print(f"[WARNING] Pose_V.pose access failed: {error}")
            self.start_text_fallback()

        with self.lock:
            self.callback_count += 1
            self.last_pose_count = pose_count
            self.received_names = names
            self.orientations.update(orientations)
            if pose_access_succeeded:
                self.pose_access_succeeded = True
            else:
                self.pose_access_failed = True
            callback_count = self.callback_count
            matched_links = tuple(sorted(self.orientations))
            fallback_in_use = self.fallback_in_use

        if self.debug_pose:
            print(
                f"[POSE DEBUG] callbacks={callback_count}, "
                f"last_pose_count={pose_count}, "
                f"pose_access={'success' if pose_access_succeeded else 'failed'}, "
                f"fallback={'on' if fallback_in_use else 'off'}, "
                "matched_links="
                + (", ".join(matched_links) if matched_links else "<none>")
            )

    @staticmethod
    def name_matches(name: str, target: str) -> bool:
        return (
            name == target
            or target in name
            or name.endswith(target)
            or name.split("::")[-1] == target
        )

    def start_text_fallback(self):
        with self.lock:
            if self.fallback_in_use:
                return
            self.fallback_in_use = True

        try:
            process = subprocess.Popen(
                ["gz", "topic", "-e", "-t", self.topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                bufsize=1,
            )
        except OSError as error:
            with self.lock:
                self.fallback_in_use = False
            print(f"[WARNING] Could not start pose text fallback: {error}")
            return

        self.fallback_process = process
        self.fallback_thread = threading.Thread(
            target=self.read_text_fallback,
            args=(process,),
            daemon=True,
        )
        self.fallback_thread.start()
        print(f"[POSE FALLBACK] Reading text poses from {self.topic}")

    def read_text_fallback(self, process):
        block_lines = []
        block_depth = 0
        for raw_line in process.stdout:
            stripped = raw_line.strip()
            if not block_lines:
                if stripped != "pose {":
                    continue
                block_lines = [raw_line]
                block_depth = 1
                continue

            block_lines.append(raw_line)
            block_depth += raw_line.count("{") - raw_line.count("}")
            if block_depth > 0:
                continue

            self.handle_text_pose_block("".join(block_lines))
            block_lines = []
            block_depth = 0

    def handle_text_pose_block(self, block: str):
        name_match = re.search(r'^\s*name:\s*"([^"]+)"', block, re.MULTILINE)
        orientation_match = re.search(
            r"orientation\s*\{([^}]*)\}", block, re.DOTALL
        )
        if name_match is None or orientation_match is None:
            return

        name = name_match.group(1)
        if self.print_pose_names:
            print(f"[POSE NAME][text fallback] {name}")
        orientation_text = orientation_match.group(1)

        def component_value(component: str, default: float) -> float:
            match = re.search(
                rf"^\s*{component}:\s*([-+0-9.eE]+)",
                orientation_text,
                re.MULTILINE,
            )
            return float(match.group(1)) if match else default

        quaternion = (
            component_value("x", 0.0),
            component_value("y", 0.0),
            component_value("z", 0.0),
            component_value("w", 1.0),
        )
        matched = {}
        for _, target in JOINT_CONFIGS.values():
            if self.name_matches(name, target):
                matched[target] = quaternion_to_rpy(*quaternion)

        with self.lock:
            self.received_names.append(name)
            self.orientations.update(matched)

    def snapshot(self, foot_link: str):
        with self.lock:
            return self.orientations.get(foot_link)

    def diagnostic_snapshot(self):
        with self.lock:
            return (
                self.callback_count,
                self.last_pose_count,
                list(self.received_names),
                set(self.orientations),
                self.pose_access_succeeded,
                self.pose_access_failed,
                self.fallback_in_use,
            )

    def close(self):
        process = self.fallback_process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)


class PositionPublishers:
    def __init__(self):
        from gz.msgs10.double_pb2 import Double
        from gz.transport13 import Node

        self.message_type = Double
        self.node = Node()
        self.publishers = {}
        for joint_name, (topic, _) in JOINT_CONFIGS.items():
            publisher = self.node.advertise(topic, Double)
            if not publisher.valid():
                raise RuntimeError(f"Failed to advertise {topic}")
            self.publishers[joint_name] = publisher

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if all(
                publisher.has_connections()
                for publisher in self.publishers.values()
            ):
                return
            time.sleep(0.01)

        missing = [
            joint_name
            for joint_name, publisher in self.publishers.items()
            if not publisher.has_connections()
        ]
        raise RuntimeError(
            "No Gazebo controller subscription for: " + ", ".join(missing)
        )

    def publish(self, joint_name: str, position: float):
        published = self.publishers[joint_name].publish(
            self.message_type(data=position)
        )
        if published is False:
            raise RuntimeError(f"Publish failed for {joint_name}")


def publish_neutral_for_duration(
    publishers,
    duration: float,
    dt: float,
):
    publish_count = math.ceil(duration / dt)
    for _ in range(publish_count):
        for joint_name in JOINT_CONFIGS:
            publishers.publish(joint_name, 0.0)
        time.sleep(dt)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Apply isolated ankle-roll step commands and log the matching "
            "foot world orientation and ankle actual position."
        )
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        choices=tuple(JOINT_CONFIGS),
        default=list(JOINT_CONFIGS),
        help="Joints to test (default: both ankle-roll joints)",
    )
    parser.add_argument(
        "--command-values",
        nargs="+",
        type=float,
        default=list(DEFAULT_COMMAND_VALUES),
        metavar="RAD",
        help="Step commands in radians (default: +0.10 -0.10 +0.20 -0.20)",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--reset-duration", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--joint-state-topic", default=DEFAULT_JOINT_STATE_TOPIC)
    parser.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    parser.add_argument(
        "--print-pose-names",
        action="store_true",
        help=(
            "Print every pose.name directly from each received Pose_V "
            "callback"
        ),
    )
    parser.add_argument(
        "--debug-pose",
        action="store_true",
        help=(
            "Print callback count, Pose_V.pose access status, fallback "
            "status, last pose count, and matched foot links"
        ),
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be greater than 0")
    if args.reset_duration < 0.0:
        parser.error("--reset-duration must be greater than or equal to 0")
    if args.dt <= 0.0:
        parser.error("--dt must be greater than 0")
    if not all(math.isfinite(value) for value in args.command_values):
        parser.error("--command-values must be finite")

    state_subscriber = AnkleStateSubscriber(args.joint_state_topic)
    pose_subscriber = FootPoseSubscriber(
        args.pose_topic,
        print_pose_names=args.print_pose_names,
        debug_pose=args.debug_pose,
    )
    publishers = PositionPublishers()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f"[ANKLE SIGN TEST] output={args.output}")
    print(f"[ANKLE SIGN TEST] pose_topic={args.pose_topic}")
    print("[ANKLE SIGN TEST] pose_message_type=gz.msgs.Pose_V")
    print(f"[ANKLE SIGN TEST] joint_state_topic={args.joint_state_topic}")

    with args.output.open("w", newline="") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(
            (
                "joint_name",
                "command_value",
                "frame",
                "time",
                "foot_link",
                "foot_roll",
                "foot_pitch",
                "foot_yaw",
                "ankle_command",
                "ankle_actual",
            )
        )

        for joint_name in args.joints:
            _, foot_link = JOINT_CONFIGS[joint_name]
            for command_value in args.command_values:
                print(
                    f"[ANKLE SIGN TEST] reset {joint_name} to 0 rad "
                    f"for {args.reset_duration:g} s"
                )
                publish_neutral_for_duration(
                    publishers,
                    args.reset_duration,
                    args.dt,
                )

                initial_orientation = pose_subscriber.snapshot(foot_link)
                initial_actual = state_subscriber.snapshot(joint_name)
                initial_roll, initial_pitch, initial_yaw = (
                    initial_orientation
                    if initial_orientation is not None
                    else (math.nan, math.nan, math.nan)
                )
                writer.writerow(
                    (
                        joint_name,
                        command_value,
                        0,
                        0.0,
                        foot_link,
                        initial_roll,
                        initial_pitch,
                        initial_yaw,
                        0.0,
                        initial_actual
                        if initial_actual is not None
                        else math.nan,
                    )
                )

                print(
                    f"[ANKLE SIGN TEST] {joint_name} "
                    f"command={command_value:+.3f} rad"
                )
                sample_count = math.ceil(args.duration / args.dt)
                test_start = time.monotonic()
                for sample_index in range(sample_count):
                    for published_joint_name in JOINT_CONFIGS:
                        command = (
                            command_value
                            if published_joint_name == joint_name
                            else 0.0
                        )
                        publishers.publish(published_joint_name, command)
                    time.sleep(args.dt)

                    orientation = pose_subscriber.snapshot(foot_link)
                    ankle_actual = state_subscriber.snapshot(joint_name)
                    roll, pitch, yaw = (
                        orientation
                        if orientation is not None
                        else (math.nan, math.nan, math.nan)
                    )
                    writer.writerow(
                        (
                            joint_name,
                            command_value,
                            sample_index + 1,
                            time.monotonic() - test_start,
                            foot_link,
                            roll,
                            pitch,
                            yaw,
                            command_value,
                            ankle_actual
                            if ankle_actual is not None
                            else math.nan,
                        )
                    )

        for joint_name in args.joints:
            publishers.publish(joint_name, 0.0)

    (
        pose_callback_count,
        last_pose_count,
        received_pose_names,
        found_foot_links,
        pose_access_succeeded,
        pose_access_failed,
        fallback_in_use,
    ) = pose_subscriber.diagnostic_snapshot()
    if args.debug_pose:
        print(
            f"[POSE DEBUG FINAL] callbacks={pose_callback_count}, "
            f"last_pose_count={last_pose_count}, "
            f"pose_access_success={pose_access_succeeded}, "
            f"pose_access_failed={pose_access_failed}, "
            f"fallback={'on' if fallback_in_use else 'off'}, matched_links="
            + (
                ", ".join(sorted(found_foot_links))
                if found_foot_links
                else "<none>"
            )
        )
    candidate_names = [
        name
        for name in received_pose_names
        if any(
            keyword in name.lower()
            for keyword in ("foot", "ankle", "leg")
        )
    ]
    missing_foot_links = sorted(
        set(foot_link for _, foot_link in JOINT_CONFIGS.values())
        - found_foot_links
    )
    if missing_foot_links:
        print(
            "[WARNING] Foot pose matching failed for: "
            + ", ".join(missing_foot_links)
        )
        names_to_report = candidate_names or received_pose_names
        if names_to_report:
            print("[WARNING] Received pose name candidates:")
            for name in names_to_report:
                print(f"  - {name}")
        else:
            print(
                f"[WARNING] No pose entity names were received from "
                f"{args.pose_topic}."
            )
    pose_subscriber.close()


if __name__ == "__main__":
    main()
