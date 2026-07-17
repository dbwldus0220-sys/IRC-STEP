#!/usr/bin/env python3

import argparse
import csv
import math
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd


LEG_JOINT_TOPICS = {
    "RL0_wrap": "/step/right_hip_yaw_joint/cmd_pos",
    "RL1_wrap": "/step/right_hip_roll_joint/cmd_pos",
    "RL2_wrap": "/step/right_hip_pitch_joint/cmd_pos",
    "RL3_wrap": "/step/right_knee_pitch_joint/cmd_pos",
    "RL4_wrap": "/step/right_ankle_pitch_joint/cmd_pos",
    "RL5_wrap": "/step/right_ankle_roll_joint/cmd_pos",
    "LL0_wrap": "/step/left_hip_yaw_joint/cmd_pos",
    "LL1_wrap": "/step/left_hip_roll_joint/cmd_pos",
    "LL2_wrap": "/step/left_hip_pitch_joint/cmd_pos",
    "LL3_wrap": "/step/left_knee_pitch_joint/cmd_pos",
    "LL4_wrap": "/step/left_ankle_pitch_joint/cmd_pos",
    "LL5_wrap": "/step/left_ankle_roll_joint/cmd_pos",
}

JOINT_SIGNS = {column: 1.0 for column in LEG_JOINT_TOPICS}

GAZEBO_OFFSETS = {
    "RL0_wrap": 0.0,
    "RL1_wrap": 0.0,
    "RL2_wrap": -0.61,
    "RL3_wrap": 1.22,
    "RL4_wrap": -0.61,
    "RL5_wrap": 0.0,
    "LL0_wrap": 0.0,
    "LL1_wrap": 0.0,
    "LL2_wrap": -0.61,
    "LL3_wrap": 1.22,
    "LL4_wrap": -0.61,
    "LL5_wrap": 0.0,
}

ROLL_JOINT_COLUMNS = ("RL1_wrap", "RL5_wrap", "LL1_wrap", "LL5_wrap")
ROLL_JOINT_TOPICS = {
    column: LEG_JOINT_TOPICS[column] for column in ROLL_JOINT_COLUMNS
}
LEFT_PITCH_JOINT_COLUMNS = frozenset(
    ("LL2_wrap", "LL3_wrap", "LL4_wrap")
)
BASE_LATERAL_TOPIC = "/step/base_lateral_joint/cmd_pos"
BASE_LATERAL_JOINT_NAME = "base_lateral_to_world"
BASE_LATERAL_CSV_COLUMNS = ("COM_y", "Ycom")
DEFAULT_TRACKING_OUTPUT = Path("gazebo_joint_tracking_log.csv")
DEFAULT_STEP_JOINT_STATE_TOPIC = (
    "/world/step_test/model/step_humanoid/joint_state"
)
LEG_JOINT_STATE_TOPIC = "/step/leg_joint_states"
TRACKED_LEG_JOINTS = tuple(
    topic.removeprefix("/step/").removesuffix("/cmd_pos")
    for topic in LEG_JOINT_TOPICS.values()
)

CONTROLLER_SDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "step_urdf"
    / "step_fixed_base_ctrl.sdf"
)
LATERAL_CONTROLLER_SDF_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "step_urdf"
    / "step_lateral_slide_ctrl.sdf"
)
POSITION_CONTROLLER_PLUGIN = "gz-sim-joint-position-controller-system"


def angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def command_from_csv(
    column: str,
    csv_value: float,
    base_values,
    use_gazebo_offsets: bool,
    mirror_left_pitch_chain: bool,
) -> float:
    value = csv_value
    if base_values is not None:
        value = angle_wrap(csv_value - base_values[column])

    command = JOINT_SIGNS[column] * value
    if use_gazebo_offsets:
        command += GAZEBO_OFFSETS[column]

    if mirror_left_pitch_chain and column in LEFT_PITCH_JOINT_COLUMNS:
        # Left pitch joints have mirrored spatial axes in the Gazebo model, so
        # the full logical command including offset must be negated, not only
        # the delta.
        command = -command
    return command


def frame_commands(
    row,
    joint_topics,
    base_values,
    use_gazebo_offsets: bool,
    mirror_left_pitch_chain: bool,
    base_lateral_config=None,
):
    commands = []
    for column, topic in joint_topics.items():
        csv_value = float(row[column])
        command = command_from_csv(
            column,
            csv_value,
            base_values,
            use_gazebo_offsets,
            mirror_left_pitch_chain,
        )
        commands.append((column, topic, csv_value, command))

    if base_lateral_config is not None:
        column = base_lateral_config["column"]
        csv_value = float(row[column])
        value_m = csv_value * base_lateral_config["unit_scale"]
        command = (
            base_lateral_config["scale"]
            * base_lateral_config["axis_sign"]
            * (value_m - base_lateral_config["base_value_m"])
        )
        commands.append((column, BASE_LATERAL_TOPIC, csv_value, command))
    return commands


def unwrap_joint_command_frames(prepared_frames, joint_topics):
    """Unwrap transformed joint commands without changing CSV source values."""
    joint_topic_set = set(joint_topics.values())
    command_indices = [
        index
        for index, (_, topic, _, _) in enumerate(prepared_frames[0][1])
        if topic in joint_topic_set
    ]

    for command_index in command_indices:
        command_sequence = np.asarray(
            [commands[command_index][3] for _, commands in prepared_frames],
            dtype=float,
        )
        unwrapped_sequence = np.unwrap(command_sequence)
        for frame_index, unwrapped_command in enumerate(unwrapped_sequence):
            column, topic, csv_value, _ = prepared_frames[frame_index][1][
                command_index
            ]
            prepared_frames[frame_index][1][command_index] = (
                column,
                topic,
                csv_value,
                float(unwrapped_command),
            )


def find_base_lateral_column(columns):
    for column in BASE_LATERAL_CSV_COLUMNS:
        if column in columns:
            return column
    return None


def detect_com_y_unit_scale(values) -> tuple[float, str]:
    if not values.map(math.isfinite).all():
        raise ValueError("COM_y/Ycom contains non-finite values")

    max_abs_value = float(values.abs().max())
    if max_abs_value > 1.0:
        return 0.001, "mm"
    return 1.0, "m"


def load_position_controller_mappings(sdf_path: Path):
    try:
        sdf_root = ET.parse(sdf_path).getroot()
    except (OSError, ET.ParseError) as error:
        return None, str(error)

    controller_mappings = {}
    for plugin in sdf_root.iter("plugin"):
        if plugin.get("filename") != POSITION_CONTROLLER_PLUGIN:
            continue

        joint_name = plugin.findtext("joint_name")
        topic = plugin.findtext("topic")
        if joint_name and topic:
            controller_mappings[topic.strip()] = joint_name.strip()

    return controller_mappings, None


def find_joint_state_topic(configured_topic=None):
    if configured_topic:
        return configured_topic

    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as error:
        print(f"[WARNING] Could not run 'gz topic -l': {error}")
        return None

    topics = [line.strip() for line in result.stdout.splitlines()]
    if LEG_JOINT_STATE_TOPIC in topics:
        return LEG_JOINT_STATE_TOPIC

    world_model_pattern = re.compile(
        r"^/world/[^/]+/model/[^/]+/joint_state$"
    )
    candidates = [
        topic for topic in topics if world_model_pattern.fullmatch(topic)
    ]
    if not candidates:
        candidates = [
            topic for topic in topics if topic.endswith("/joint_state")
        ]
    if DEFAULT_STEP_JOINT_STATE_TOPIC in candidates:
        candidates.remove(DEFAULT_STEP_JOINT_STATE_TOPIC)
        candidates.insert(0, DEFAULT_STEP_JOINT_STATE_TOPIC)
    if not candidates:
        return None
    if len(candidates) > 1:
        print(
            "[WARNING] Multiple Gazebo joint-state topics found; using "
            f"{candidates[0]}. Set --joint-state-topic to override."
        )
    return candidates[0]


def joint_state_message_type(topic):
    if topic is None:
        return None
    try:
        result = subprocess.run(
            ["gz", "topic", "-i", "-t", topic],
            check=True,
            capture_output=True,
            text=True,
            timeout=5.0,
        )
    except (OSError, subprocess.SubprocessError) as error:
        print(
            f"[WARNING] Could not inspect message type for {topic}: {error}"
        )
        return None

    match = re.search(
        r"Message Type:\s*([^\s]+)", result.stdout + result.stderr
    )
    return match.group(1) if match else None


def joint_name_from_command_topic(topic: str):
    prefix = "/step/"
    suffix = "/cmd_pos"
    if topic.startswith(prefix) and topic.endswith(suffix):
        return topic[len(prefix):-len(suffix)]
    return None


class JointTrackingSubscriber:
    def __init__(self, topic, expected_joint_names):
        from gz.msgs10.model_pb2 import Model
        from gz.transport13 import Node

        self.lock = threading.Lock()
        self.positions = {}
        self.received_message = False
        self.expected_joint_names = tuple(expected_joint_names)
        self.reported_first_message = False
        self.node = Node()
        subscribed = self.node.subscribe(Model, topic, self.on_joint_state)
        if subscribed is False:
            raise RuntimeError(f"Failed to subscribe to {topic}")

    def on_joint_state(self, message):
        positions = {}
        received_names = []
        for joint in message.joint:
            received_names.append(joint.name)
            name = joint.name.replace("/", "::").rsplit("::", 1)[-1]
            positions[name] = float(joint.axis1.position)

        if not self.reported_first_message:
            print(
                "[JOINT TRACKING] first joint names: "
                + (", ".join(received_names) if received_names else "<none>")
            )
            missing_names = [
                name
                for name in self.expected_joint_names
                if name not in positions
            ]
            if missing_names:
                print(
                    "[WARNING] First joint-state message did not contain: "
                    + ", ".join(missing_names)
                )
            self.reported_first_message = True
        with self.lock:
            self.positions.update(positions)
            self.received_message = True

    def snapshot(self):
        with self.lock:
            return dict(self.positions), self.received_message


class GazeboDoublePublishers:
    def __init__(self, topics, dry_run: bool, optional_topics=None):
        self.dry_run = dry_run
        self.node = None
        self.publishers = {}
        self.double_message_type = None
        self.optional_topics = set(optional_topics or ())
        self.warned_publish_topics = set()

        if dry_run:
            return

        try:
            from gz.msgs10.double_pb2 import Double
            from gz.transport13 import Node
        except ImportError as error:
            raise RuntimeError(
                "Gazebo Python transport bindings are required for replay: "
                "gz.transport13 and gz.msgs10"
            ) from error

        self.double_message_type = Double
        self.node = Node()
        for topic in topics:
            try:
                publisher = self.node.advertise(topic, Double)
            except Exception as error:  # Gazebo bindings expose runtime errors.
                if topic in self.optional_topics:
                    print(
                        f"[WARNING] Could not advertise optional Gazebo "
                        f"topic {topic}: {error}"
                    )
                    continue
                raise
            if not publisher.valid():
                if topic in self.optional_topics:
                    print(
                        f"[WARNING] Failed to advertise optional Gazebo "
                        f"topic: {topic}"
                    )
                    continue
                raise RuntimeError(f"Failed to advertise Gazebo topic: {topic}")
            self.publishers[topic] = publisher

        connection_deadline = time.monotonic() + 2.0
        while time.monotonic() < connection_deadline:
            if all(
                publisher.has_connections()
                for publisher in self.publishers.values()
            ):
                break
            time.sleep(0.01)

        disconnected_topics = [
            topic
            for topic, publisher in self.publishers.items()
            if not publisher.has_connections()
        ]
        disconnected_required_topics = [
            topic
            for topic in disconnected_topics
            if topic not in self.optional_topics
        ]
        for topic in disconnected_topics:
            if topic in self.optional_topics:
                print(
                    f"[WARNING] No Gazebo controller subscription found for "
                    f"optional topic: {topic}"
                )
        if disconnected_required_topics:
            missing_topics = ", ".join(disconnected_required_topics)
            raise RuntimeError(
                "No Gazebo controller subscription found for topic(s): "
                f"{missing_topics}"
            )

    def publish(self, commands) -> None:
        if self.dry_run:
            return

        for _, topic, _, command in commands:
            if topic not in self.publishers:
                continue
            message = self.double_message_type(data=command)
            try:
                published = self.publishers[topic].publish(message)
            except Exception as error:  # Keep an optional diagnostic lock safe.
                if topic in self.optional_topics:
                    if topic not in self.warned_publish_topics:
                        print(
                            f"[WARNING] Failed to publish optional Gazebo "
                            f"topic {topic}: {error}"
                        )
                        self.warned_publish_topics.add(topic)
                    continue
                raise
            if published is False:
                if topic in self.optional_topics:
                    if topic not in self.warned_publish_topics:
                        print(
                            f"[WARNING] Failed to publish optional Gazebo "
                            f"topic: {topic}"
                        )
                        self.warned_publish_topics.add(topic)
                    continue
                raise RuntimeError(f"Failed to publish Gazebo topic: {topic}")


def print_command_details(label: str, frame: int, commands) -> None:
    print(f"{label} frame {frame}, topics={len(commands)}")
    for column, topic, csv_value, command in commands:
        print(
            f"  {column:8s} -> {topic:40s} "
            f"csv={csv_value:+.9f} command={command:+.9f}"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Replay STEP leg joint commands from Dynamics CSV into Gazebo topics."
    )
    parser.add_argument("csv_path", help="Path to Dynamics CSV")
    parser.add_argument(
        "--mode",
        choices=("roll", "legs"),
        default="roll",
        help="Publish four roll joints (default) or all 12 leg joints",
    )
    parser.add_argument("--start-frame", type=int, default=398)
    parser.add_argument("--end-frame", type=int, default=405)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument(
        "--progress-every",
        type=int,
        default=50,
        metavar="N",
        help="Print replay progress every N published frames (default: 50)",
    )
    parser.add_argument(
        "--relative-to-frame",
        type=int,
        default=None,
        metavar="N",
        help="Replay wrapped joint deltas relative to CSV frame N",
    )
    parser.add_argument(
        "--use-gazebo-offsets",
        action="store_true",
        help="Add the configured Gazebo initial-pose offset to each command",
    )
    parser.add_argument(
        "--mirror-left-pitch-chain",
        action="store_true",
        help=(
            "Negate the full LL2/LL3/LL4 logical command to account for "
            "mirrored Gazebo pitch axes"
        ),
    )
    parser.add_argument(
        "--unwrap-joint-commands",
        action="store_true",
        help=(
            "Apply frame-to-frame angle unwrapping to transformed joint "
            "commands immediately before replay"
        ),
    )
    parser.add_argument(
        "--replay-base-lateral-from-com-y",
        action="store_true",
        help=(
            "Replay COM_y/Ycom relative motion through the diagnostic "
            "base lateral prismatic joint"
        ),
    )
    parser.add_argument(
        "--lock-base-lateral",
        action="store_true",
        help=(
            "Publish 0.0 to /step/base_lateral_joint/cmd_pos on every "
            "replay frame"
        ),
    )
    parser.add_argument(
        "--log-joint-tracking",
        action="store_true",
        help=(
            "Log commanded and actual Gazebo leg-joint positions to "
            "gazebo_joint_tracking_log.csv"
        ),
    )
    parser.add_argument(
        "--joint-state-topic",
        default=None,
        metavar="TOPIC",
        help=(
            "Gazebo Model joint-state topic; when omitted, inspect "
            "'gz topic -l' automatically"
        ),
    )
    parser.add_argument(
        "--base-lateral-axis-sign",
        type=int,
        choices=(-1, 1),
        default=1,
        help="Gazebo lateral-axis sign applied to COM_y/Ycom (default: 1)",
    )
    parser.add_argument(
        "--base-lateral-scale",
        type=float,
        default=1.0,
        help="Scale applied to relative COM_y/Ycom motion (default: 1.0)",
    )
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
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print every joint command for every replay frame",
    )
    parser.add_argument(
        "--check-publish-frame",
        type=int,
        default=None,
        metavar="FRAME",
        help="Print all topics and commands immediately before this frame is published",
    )
    args = parser.parse_args()

    if args.dt <= 0.0:
        raise ValueError("--dt must be greater than 0")
    if args.hold_start < 0.0:
        raise ValueError("--hold-start must be greater than or equal to 0")
    if args.progress_every <= 0:
        raise ValueError("--progress-every must be greater than 0")
    if args.base_lateral_scale < 0.0:
        raise ValueError("--base-lateral-scale must be greater than or equal to 0")
    if args.lock_base_lateral and args.replay_base_lateral_from_com_y:
        parser.error(
            "--lock-base-lateral cannot be combined with "
            "--replay-base-lateral-from-com-y"
        )

    csv_path = Path(args.csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(csv_path)

    df = pd.read_csv(csv_path)

    joint_topics = (
        ROLL_JOINT_TOPICS if args.mode == "roll" else LEG_JOINT_TOPICS
    )
    required_cols = ["frame"] + list(joint_topics.keys())
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        missing_columns = ", ".join(missing)
        parser.error(
            f"CSV is missing required column(s) for --mode {args.mode}: "
            f"{missing_columns}"
        )

    relative_base_row = None
    base_values = None
    if args.relative_to_frame is not None:
        base_rows = df[df["frame"] == args.relative_to_frame]
        if base_rows.empty:
            parser.error(
                f"Relative base frame {args.relative_to_frame} was not found "
                "in the CSV"
            )
        relative_base_row = base_rows.iloc[0]
        base_values = {
            column: float(relative_base_row[column]) for column in joint_topics
        }

    selected = df[(df["frame"] >= args.start_frame) & (df["frame"] <= args.end_frame)]

    if selected.empty:
        raise ValueError(f"No frames found from {args.start_frame} to {args.end_frame}")

    base_lateral_config = None
    if args.replay_base_lateral_from_com_y:
        base_lateral_column = find_base_lateral_column(df.columns)
        if base_lateral_column is None:
            parser.error(
                "--replay-base-lateral-from-com-y requires a COM_y or Ycom "
                "column in the CSV"
            )

        unit_scale, detected_unit = detect_com_y_unit_scale(
            df[base_lateral_column]
        )
        lateral_base_row = (
            relative_base_row
            if relative_base_row is not None
            else selected.iloc[0]
        )
        base_lateral_config = {
            "column": base_lateral_column,
            "unit_scale": unit_scale,
            "detected_unit": detected_unit,
            "base_frame": int(lateral_base_row["frame"]),
            "base_value_m": (
                float(lateral_base_row[base_lateral_column]) * unit_scale
            ),
            "axis_sign": args.base_lateral_axis_sign,
            "scale": args.base_lateral_scale,
        }

        print(
            f"[BASE LATERAL] column={base_lateral_column}, "
            f"detected_unit={detected_unit}, "
            f"base_frame={base_lateral_config['base_frame']}, "
            f"base_value_m={base_lateral_config['base_value_m']:+.9f}, "
            f"axis_sign={args.base_lateral_axis_sign:+d}, "
            f"scale={args.base_lateral_scale:g}"
        )

    if args.check_publish_frame is not None:
        check_rows = selected[selected["frame"] == args.check_publish_frame]
        if check_rows.empty:
            parser.error(
                f"Check-publish frame {args.check_publish_frame} is not in "
                "the selected replay range"
            )

    if args.verbose:
        print("[Gazebo Leg Joint CSV Replay]")
        print(f"csv: {csv_path}")
        print(f"mode: {args.mode}")
        print(f"frames: {args.start_frame}~{args.end_frame}")
        print(f"dt: {args.dt}")
        print(f"progress_every: {args.progress_every}")
        print(f"relative_to_frame: {args.relative_to_frame}")
        print(f"use_gazebo_offsets: {args.use_gazebo_offsets}")
        print(f"mirror_left_pitch_chain: {args.mirror_left_pitch_chain}")
        print(f"unwrap_joint_commands: {args.unwrap_joint_commands}")
        print(
            "replay_base_lateral_from_com_y: "
            f"{args.replay_base_lateral_from_com_y}"
        )
        print(f"lock_base_lateral: {args.lock_base_lateral}")
        print(f"log_joint_tracking: {args.log_joint_tracking}")
        print(f"base_lateral_axis_sign: {args.base_lateral_axis_sign:+d}")
        print(f"base_lateral_scale: {args.base_lateral_scale:g}")
        print(f"hold_start: {args.hold_start}")
        print(f"dry_run: {args.dry_run}")
        print()
        print("[Publish Targets]")
        for column, topic in joint_topics.items():
            print(f"  {column:8s} -> {topic}")
        if base_lateral_config is not None:
            print(
                f"  {base_lateral_config['column']:8s} -> "
                f"{BASE_LATERAL_TOPIC}"
            )
        elif args.lock_base_lateral:
            print(f"  {'base_lock':8s} -> {BASE_LATERAL_TOPIC}")
        print()
        print("[Joint Transform]")
        for column in joint_topics:
            base_text = (
                f"{base_values[column]:+.9f}"
                if base_values is not None
                else "not used"
            )
            offset_status = (
                "applied" if args.use_gazebo_offsets else "not applied"
            )
            print(
                f"  {column:8s} base={base_text:>12s} "
                f"sign={JOINT_SIGNS[column]:+g} "
                f"offset={GAZEBO_OFFSETS[column]:+.9f} ({offset_status})"
            )
    if args.mode == "legs":
        controller_mappings, controller_error = load_position_controller_mappings(
            CONTROLLER_SDF_PATH
        )
        if controller_error is not None:
            print(
                f"[WARNING] Could not inspect controllers in "
                f"{CONTROLLER_SDF_PATH}: {controller_error}"
            )
        else:
            missing_controller_topics = []
            mismatched_controllers = []
            for topic in joint_topics.values():
                expected_joint_name = topic.removeprefix("/step/").removesuffix(
                    "/cmd_pos"
                )
                configured_joint_name = controller_mappings.get(topic)
                if configured_joint_name is None:
                    missing_controller_topics.append(topic)
                elif configured_joint_name != expected_joint_name:
                    mismatched_controllers.append(
                        (topic, expected_joint_name, configured_joint_name)
                    )

            if missing_controller_topics or mismatched_controllers:
                print("[WARNING] Gazebo leg controller mapping is incomplete.")
                for topic in missing_controller_topics:
                    print(f"          missing topic: {topic}")
                for topic, expected, configured in mismatched_controllers:
                    print(
                        f"          joint mismatch: {topic} expects {expected}, "
                        f"configured as {configured}"
                    )
            else:
                if args.verbose:
                    print(
                        f"[Controller Check] Found all {len(joint_topics)} leg "
                        f"controllers in {CONTROLLER_SDF_PATH}."
                    )

    if base_lateral_config is not None:
        lateral_mappings, lateral_error = load_position_controller_mappings(
            LATERAL_CONTROLLER_SDF_PATH
        )
        if lateral_error is not None:
            print(
                f"[WARNING] Could not inspect the base lateral controller in "
                f"{LATERAL_CONTROLLER_SDF_PATH}: {lateral_error}"
            )
        elif lateral_mappings.get(BASE_LATERAL_TOPIC) != BASE_LATERAL_JOINT_NAME:
            parser.error(
                "The lateral-slide SDF does not map "
                f"{BASE_LATERAL_TOPIC} to {BASE_LATERAL_JOINT_NAME}"
            )

    publish_topics = list(joint_topics.values())
    optional_topics = []
    if base_lateral_config is not None:
        publish_topics.append(BASE_LATERAL_TOPIC)
    elif args.lock_base_lateral:
        publish_topics.append(BASE_LATERAL_TOPIC)
        optional_topics.append(BASE_LATERAL_TOPIC)

    gazebo_publishers = GazeboDoublePublishers(
        publish_topics,
        args.dry_run,
        optional_topics,
    )

    tracking_subscriber = None
    tracking_output_file = None
    tracking_writer = None
    tracking_topic = None
    if args.log_joint_tracking:
        tracking_topic = find_joint_state_topic(args.joint_state_topic)
        tracking_message_type = joint_state_message_type(tracking_topic)
        print(
            "[JOINT TRACKING] selected joint state topic: "
            f"{tracking_topic if tracking_topic is not None else '<not found>'}"
        )
        print(
            "[JOINT TRACKING] joint state message type: "
            f"{tracking_message_type or 'gz.msgs.Model (expected)'}"
        )
        if tracking_topic is None:
            print(
                "[WARNING] No Gazebo joint-state topic was found. Check "
                "'gz topic -l' for a topic ending in /joint_state, or pass "
                "--joint-state-topic TOPIC. Actual positions will be NaN."
            )
        elif args.dry_run:
            print(
                f"[WARNING] Dry-run does not subscribe to {tracking_topic}; "
                "actual positions will be NaN."
            )
        else:
            try:
                if tracking_message_type not in (None, "gz.msgs.Model"):
                    print(
                        f"[WARNING] {tracking_topic} publishes "
                        f"{tracking_message_type}; this logger expects "
                        "gz.msgs.Model."
                    )
                tracking_subscriber = JointTrackingSubscriber(
                    tracking_topic, TRACKED_LEG_JOINTS
                )
            except (ImportError, RuntimeError) as error:
                print(
                    f"[WARNING] Could not subscribe to joint-state topic "
                    f"{tracking_topic}: {error}. Actual positions will be NaN."
                )

        tracking_output_file = DEFAULT_TRACKING_OUTPUT.open("w", newline="")
        tracking_writer = csv.writer(tracking_output_file)
        tracking_writer.writerow(
            (
                "frame",
                "time",
                "joint_name",
                "command_position",
                "actual_position",
                "error",
            )
        )
        print(f"[JOINT TRACKING] output={DEFAULT_TRACKING_OUTPUT}")

    prepared_frames = []
    for _, row in selected.iterrows():
        commands = frame_commands(
            row,
            joint_topics,
            base_values,
            args.use_gazebo_offsets,
            args.mirror_left_pitch_chain,
            base_lateral_config,
        )
        if args.lock_base_lateral:
            commands.append(
                ("base_lock", BASE_LATERAL_TOPIC, 0.0, 0.0)
            )
        prepared_frames.append(
            (
                int(row["frame"]),
                commands,
            )
        )
    if args.unwrap_joint_commands:
        unwrap_joint_command_frames(prepared_frames, joint_topics)
    print(f"[UNWRAP] enabled={str(args.unwrap_joint_commands).lower()}")

    if args.hold_start > 0.0:
        hold_frame, hold_commands = prepared_frames[0]
        publish_count = math.ceil(args.hold_start / args.dt)

        if args.verbose:
            print(
                f"[HOLD] frame={hold_frame}, duration={args.hold_start} s, "
                f"publish_count={publish_count}"
            )

        for publish_index in range(publish_count):
            if args.verbose:
                print_command_details(
                    f"[HOLD {publish_index + 1}/{publish_count}]",
                    hold_frame,
                    hold_commands,
                )
            gazebo_publishers.publish(hold_commands)

            time.sleep(args.dt)

    replay_start_time = time.monotonic()
    last_replay_index = len(prepared_frames) - 1
    last_replay_frame = prepared_frames[-1][0]
    published_frames = 0

    for replay_index, (frame, commands) in enumerate(prepared_frames):

        if args.dry_run:
            print_command_details("[DRY RUN]", frame, commands)
        elif args.verbose:
            print_command_details("[VERBOSE]", frame, commands)
        if frame == args.check_publish_frame:
            check_label = (
                "[CHECK PUBLISH][DRY RUN]"
                if args.dry_run
                else "[CHECK PUBLISH]"
            )
            print_command_details(check_label, frame, commands)

        gazebo_publishers.publish(commands)
        published_frames += 1

        if (
            replay_index % args.progress_every == 0
            or replay_index == last_replay_index
        ):
            elapsed_time = time.monotonic() - replay_start_time
            print(
                f"[REPLAY] frame {frame} / {last_replay_frame} "
                f"(elapsed={elapsed_time:.3f} s)"
            )

        time.sleep(args.dt)

        if tracking_writer is not None:
            actual_positions = {}
            received_state = False
            if tracking_subscriber is not None:
                actual_positions, received_state = tracking_subscriber.snapshot()
            command_positions = {
                joint_name_from_command_topic(topic): command
                for _, topic, _, command in commands
                if joint_name_from_command_topic(topic) is not None
            }
            elapsed_time = time.monotonic() - replay_start_time
            for joint_name in TRACKED_LEG_JOINTS:
                command_position = command_positions.get(joint_name, math.nan)
                actual_position = actual_positions.get(joint_name, math.nan)
                error = (
                    command_position - actual_position
                    if math.isfinite(command_position)
                    and math.isfinite(actual_position)
                    else math.nan
                )
                tracking_writer.writerow(
                    (
                        frame,
                        elapsed_time,
                        joint_name,
                        command_position,
                        actual_position,
                        error,
                    )
                )

            if not received_state and replay_index == last_replay_index:
                print(
                    "[WARNING] No joint-state message was received during "
                    "replay. Check 'gz topic -l' and verify that the model "
                    "loads gz-sim-joint-state-publisher-system for the leg "
                    "joints."
                )

    print(
        f"[REPLAY] done. published_frames={published_frames}, "
        f"topics_per_frame={len(publish_topics)}"
    )
    if tracking_subscriber is not None:
        final_positions, _ = tracking_subscriber.snapshot()
        missing_tracking_joints = [
            joint_name
            for joint_name in TRACKED_LEG_JOINTS
            if joint_name not in final_positions
        ]
        if missing_tracking_joints:
            print(
                f"[WARNING] Joint-state topic {tracking_topic} did not "
                "provide the requested leg joints: "
                f"{', '.join(missing_tracking_joints)}. Verify the "
                "gz-sim-joint-state-publisher-system <joint_name> settings."
            )
    if tracking_output_file is not None:
        tracking_output_file.close()


if __name__ == "__main__":
    main()
