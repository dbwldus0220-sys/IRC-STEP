#!/usr/bin/env python3

import argparse
import csv
import math
import threading
import time
from pathlib import Path


JOINT_CONFIGS = {
    "right": (
        "right_ankle_roll_joint",
        "/step/right_ankle_roll_joint/cmd_pos",
    ),
    "left": (
        "left_ankle_roll_joint",
        "/step/left_ankle_roll_joint/cmd_pos",
    ),
}
DEFAULT_JOINT_STATE_TOPIC = "/step/leg_joint_states"
DEFAULT_OUTPUT = Path("gazebo_single_joint_tracking_log.csv")


class JointStateSubscriber:
    def __init__(self, topic: str, joint_name: str):
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node

        self.joint_name = joint_name
        self.lock = threading.Lock()
        self.position = None
        self.received_names = None
        self.node = Node()
        subscribed = self.node.subscribe(Model, topic, self.on_joint_state)
        if subscribed is False:
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def on_joint_state(self, message):
        names = []
        selected_position = None
        for joint in message.joint:
            names.append(joint.name)
            normalized_name = joint.name.replace("/", "::").rsplit("::", 1)[-1]
            if normalized_name == self.joint_name:
                selected_position = float(joint.axis1.position)

        with self.lock:
            if self.received_names is None:
                self.received_names = names
            if selected_position is not None:
                self.position = selected_position

    def snapshot(self):
        with self.lock:
            names = (
                list(self.received_names)
                if self.received_names is not None
                else None
            )
            return self.position, names


class PositionPublisher:
    def __init__(self, topic: str):
        from gz.msgs10.double_pb2 import Double
        from gz.transport13 import Node

        self.message_type = Double
        self.node = Node()
        self.publisher = self.node.advertise(topic, Double)
        if not self.publisher.valid():
            raise RuntimeError(f"Failed to advertise {topic}")

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if self.publisher.has_connections():
                break
            time.sleep(0.01)
        if not self.publisher.has_connections():
            raise RuntimeError(f"No Gazebo controller subscribes to {topic}")

    def publish(self, position: float):
        published = self.publisher.publish(
            self.message_type(data=position)
        )
        if published is False:
            raise RuntimeError("Gazebo position command publish failed")


def command_at_time(
    waveform: str,
    amplitude: float,
    frequency: float,
    elapsed: float,
) -> float:
    if waveform == "step":
        return amplitude
    return amplitude * math.sin(2.0 * math.pi * frequency * elapsed)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Test one STEP ankle-roll Gazebo controller and log command "
            "versus actual joint position."
        )
    )
    parser.add_argument(
        "--joint",
        choices=tuple(JOINT_CONFIGS),
        default="right",
        help="Ankle-roll joint to test (default: right)",
    )
    parser.add_argument(
        "--joint-name",
        default=None,
        help=(
            "Explicit Gazebo joint name; publishes to "
            "/step/<joint_name>/cmd_pos and overrides --joint"
        ),
    )
    parser.add_argument(
        "--waveform",
        choices=("sine", "step"),
        default="sine",
        help="Command waveform (default: sine)",
    )
    parser.add_argument("--amplitude", type=float, default=0.20)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.5,
        help="Sine frequency in Hz (default: 0.5)",
    )
    parser.add_argument(
        "--joint-state-topic",
        default=DEFAULT_JOINT_STATE_TOPIC,
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    if args.amplitude < 0.0:
        parser.error("--amplitude must be greater than or equal to 0")
    if args.duration <= 0.0:
        parser.error("--duration must be greater than 0")
    if args.dt <= 0.0:
        parser.error("--dt must be greater than 0")
    if args.frequency <= 0.0:
        parser.error("--frequency must be greater than 0")

    if args.joint_name is not None:
        joint_name = args.joint_name.strip()
        if not joint_name:
            parser.error("--joint-name must not be empty")
        if "/" in joint_name:
            parser.error("--joint-name must be a joint name, not a topic")
        command_topic = f"/step/{joint_name}/cmd_pos"
    else:
        joint_name, command_topic = JOINT_CONFIGS[args.joint]
    subscriber = JointStateSubscriber(args.joint_state_topic, joint_name)
    publisher = PositionPublisher(command_topic)

    print(f"[SINGLE JOINT TEST] joint={joint_name}")
    print(f"[SINGLE JOINT TEST] command_topic={command_topic}")
    print(f"[SINGLE JOINT TEST] state_topic={args.joint_state_topic}")
    print(f"[SINGLE JOINT TEST] waveform={args.waveform}")
    print(f"[SINGLE JOINT TEST] output={args.output}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    sample_count = math.ceil(args.duration / args.dt)
    start_time = time.monotonic()
    reported_names = False

    try:
        with args.output.open("w", newline="") as output_file:
            writer = csv.writer(output_file)
            writer.writerow(
                (
                    "frame",
                    "time",
                    "joint_name",
                    "command_position",
                    "actual_position",
                    "error",
                )
            )

            for frame in range(sample_count):
                target_time = frame * args.dt
                command_position = command_at_time(
                    args.waveform,
                    args.amplitude,
                    args.frequency,
                    target_time,
                )
                publisher.publish(command_position)

                next_time = start_time + (frame + 1) * args.dt
                sleep_time = next_time - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

                actual_position, received_names = subscriber.snapshot()
                if received_names is not None and not reported_names:
                    print(
                        "[SINGLE JOINT TEST] first joint names: "
                        + ", ".join(received_names)
                    )
                    if actual_position is None:
                        print(
                            f"[WARNING] {joint_name} was not found in "
                            f"{args.joint_state_topic}"
                        )
                    reported_names = True

                actual_value = (
                    actual_position
                    if actual_position is not None
                    else math.nan
                )
                error = (
                    command_position - actual_value
                    if math.isfinite(actual_value)
                    else math.nan
                )
                writer.writerow(
                    (
                        frame,
                        target_time,
                        joint_name,
                        command_position,
                        actual_value,
                        error,
                    )
                )
    finally:
        # Return the isolated joint command to neutral even after interruption.
        publisher.publish(0.0)

    final_position, received_names = subscriber.snapshot()
    if received_names is None:
        print(
            f"[WARNING] No gz.msgs.Model message was received from "
            f"{args.joint_state_topic}"
        )
    elif final_position is None:
        print(
            f"[WARNING] No actual position was received for {joint_name}"
        )

    print(f"[SINGLE JOINT TEST] complete: samples={sample_count}")


if __name__ == "__main__":
    main()
