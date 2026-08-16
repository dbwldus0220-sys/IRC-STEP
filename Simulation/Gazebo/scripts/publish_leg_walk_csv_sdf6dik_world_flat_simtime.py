#!/usr/bin/env python3
"""Replay SDF-6D-IK walking with opt-in right-sole world-flat feedback."""

import argparse
import csv
import math
import threading
from pathlib import Path

import numpy as np
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
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node
from scipy.spatial.transform import Rotation

from gazebo_leg_transport import LEG_JOINT_TOPICS, GazeboDoublePublishers
from generate_walk_csv_sdf_6d_ik import (
    CANDIDATE_B,
    DEFAULT_SDF,
    load_leg_chains,
    pose_errors,
    solve_leg_frame,
)


DEFAULT_DT = 0.01
DEFAULT_HOLD_BEFORE = 1.0
DEFAULT_REPLAY_END = 1.44
DEFAULT_HOLD_AFTER = 1.0
DEFAULT_ODOM_TOPIC = "/step/base_odometry"
DEFAULT_RIGHT_CONTACT_TOPIC = "/step/right_sole_contacts"
DEFAULT_LEG_JOINT_STATE_TOPIC = "/step/leg_joint_states"
RIGHT_SOLE_COLLISION_NAME = "right_sole_box_collision"
VELOCITY_WARNING_RAD_S = 4.71238898
ORIENTATION_CACHE_EPS_RAD = 1e-6

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_CSV = (
    REPO_ROOT / "Dynamics" / "walk_forward_gazebo_sdf6dik_candidateB_ref.csv"
)
DEFAULT_PLANNER_CSV = REPO_ROOT / "Dynamics" / (
    "walk_forward_debug_slow_y_scale002_continuation5_baseline_jumpguard_"
    "th005_lpf_a050_pitchlpf_a080_long.csv"
)
DEFAULT_COMMAND_LOG = (
    SCRIPT_DIR.parent / "logs" / "walk_sdf6dik_world_flat_commands.csv"
)

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
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--hold-before", type=float, default=DEFAULT_HOLD_BEFORE)
    parser.add_argument(
        "--replay-end", type=float, default=DEFAULT_REPLAY_END,
        help="trajectory time to replay through, in simulation seconds",
    )
    parser.add_argument("--hold-after", type=float, default=DEFAULT_HOLD_AFTER)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--command-log", type=Path, default=DEFAULT_COMMAND_LOG)
    parser.add_argument(
        "--world-flat-leg", choices=("right", "left", "both"), required=True,
        help="leg or legs receiving world-flat feedback",
    )
    parser.add_argument(
        "--world-flat-start", type=float, required=True,
        help="inclusive trajectory-time start of world-flat feedback",
    )
    parser.add_argument(
        "--world-flat-end", type=float, required=True,
        help="inclusive trajectory-time end of world-flat feedback",
    )
    parser.add_argument("--world-flat-ramp-in", type=float, required=True)
    parser.add_argument("--world-flat-ramp-out", type=float, required=True)
    parser.add_argument(
        "--swing-world-z-leg", choices=("right",),
        help="swing leg receiving world-relative Z compensation",
    )
    parser.add_argument("--swing-world-z-start", type=float)
    parser.add_argument("--swing-world-z-end", type=float)
    parser.add_argument("--swing-world-z-ramp-in", type=float)
    parser.add_argument("--swing-world-z-ramp-out", type=float)
    parser.add_argument(
        "--max-feedback-joint-speed-deg-s", type=float,
        help="optional per-leg feedback command rate limit",
    )
    parser.add_argument("--lateral-balance-planner-csv", type=Path)
    parser.add_argument("--lateral-balance-start", type=float)
    parser.add_argument("--lateral-balance-end", type=float)
    parser.add_argument("--lateral-balance-kp", type=float)
    parser.add_argument("--lateral-balance-kd", type=float)
    parser.add_argument("--lateral-balance-max-offset-m", type=float)
    parser.add_argument("--lateral-balance-ramp-in", type=float)
    parser.add_argument("--lateral-balance-ramp-out", type=float)
    parser.add_argument("--pitch-balance-start", type=float)
    parser.add_argument("--pitch-balance-end", type=float)
    parser.add_argument("--pitch-balance-kp", type=float)
    parser.add_argument("--pitch-balance-kd", type=float)
    parser.add_argument("--pitch-balance-max-angle-deg", type=float)
    parser.add_argument("--pitch-balance-ramp-in", type=float)
    parser.add_argument("--pitch-balance-ramp-out", type=float)
    parser.add_argument(
        "--touchdown-support-hold", action="store_true",
        help="enable contact-latched right sole world-pose hold",
    )
    parser.add_argument(
        "--touchdown-right-contact-topic",
        default=DEFAULT_RIGHT_CONTACT_TOPIC,
    )
    parser.add_argument(
        "--touchdown-leg-joint-state-topic",
        default=DEFAULT_LEG_JOINT_STATE_TOPIC,
    )
    parser.add_argument(
        "--touchdown-right-fz-threshold", type=float, default=20.0,
    )
    parser.add_argument(
        "--touchdown-confirm-s", type=float, default=0.02,
    )
    parser.add_argument(
        "--touchdown-left-feedback-ramp-out", type=float, default=0.05,
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    swing_values = (
        args.swing_world_z_leg,
        args.swing_world_z_start,
        args.swing_world_z_end,
        args.swing_world_z_ramp_in,
        args.swing_world_z_ramp_out,
    )
    if any(value is not None for value in swing_values) and not all(
        value is not None for value in swing_values
    ):
        parser.error(
            "--swing-world-z-leg, --swing-world-z-start, and "
            "--swing-world-z-end must be specified together"
        )
    numeric = [
        args.dt, args.hold_before, args.replay_end, args.hold_after,
        args.world_flat_start, args.world_flat_end,
        args.world_flat_ramp_in, args.world_flat_ramp_out,
    ]
    if args.swing_world_z_start is not None:
        numeric.extend((
            args.swing_world_z_start, args.swing_world_z_end,
            args.swing_world_z_ramp_in, args.swing_world_z_ramp_out,
        ))
    if args.max_feedback_joint_speed_deg_s is not None:
        numeric.append(args.max_feedback_joint_speed_deg_s)
    lateral_values = (
        args.lateral_balance_planner_csv,
        args.lateral_balance_start, args.lateral_balance_end,
        args.lateral_balance_kp, args.lateral_balance_kd,
        args.lateral_balance_max_offset_m,
        args.lateral_balance_ramp_in, args.lateral_balance_ramp_out,
    )
    if any(value is not None for value in lateral_values) and not all(
        value is not None for value in lateral_values
    ):
        parser.error("all --lateral-balance-* options must be specified together")
    if args.lateral_balance_start is not None:
        numeric.extend(lateral_values[1:])
    pitch_values = (
        args.pitch_balance_start, args.pitch_balance_end,
        args.pitch_balance_kp, args.pitch_balance_kd,
        args.pitch_balance_max_angle_deg,
        args.pitch_balance_ramp_in, args.pitch_balance_ramp_out,
    )
    if any(value is not None for value in pitch_values) and not all(
        value is not None for value in pitch_values
    ):
        parser.error("all --pitch-balance-* options must be specified together")
    if args.pitch_balance_start is not None:
        numeric.extend(pitch_values)
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if args.dt <= 0.0:
        parser.error("--dt must be positive")
    if min(args.hold_before, args.replay_end, args.hold_after,
           args.world_flat_start, args.world_flat_end) < 0.0:
        parser.error("time arguments must be non-negative")
    if args.world_flat_end < args.world_flat_start:
        parser.error("--world-flat-end must be >= --world-flat-start")
    if min(args.world_flat_ramp_in, args.world_flat_ramp_out) < 0.0:
        parser.error("world-flat ramp durations must be non-negative")
    if args.world_flat_start + args.world_flat_ramp_in > args.world_flat_end:
        parser.error("world-flat ramp-in must finish no later than its end")
    if args.swing_world_z_start is not None:
        if min(args.swing_world_z_start, args.swing_world_z_end) < 0.0:
            parser.error("swing world-Z times must be non-negative")
        if args.swing_world_z_end < args.swing_world_z_start:
            parser.error("--swing-world-z-end must be >= --swing-world-z-start")
        if min(args.swing_world_z_ramp_in, args.swing_world_z_ramp_out) < 0.0:
            parser.error("swing world-Z ramp durations must be non-negative")
        if args.swing_world_z_start + args.swing_world_z_ramp_in > args.swing_world_z_end:
            parser.error("swing world-Z ramp-in must finish no later than its end")
    if (
        args.max_feedback_joint_speed_deg_s is not None
        and args.max_feedback_joint_speed_deg_s <= 0.0
    ):
        parser.error("--max-feedback-joint-speed-deg-s must be positive")
    if args.lateral_balance_start is not None:
        if min(
            args.lateral_balance_start, args.lateral_balance_end,
            args.lateral_balance_kp, args.lateral_balance_kd,
            args.lateral_balance_ramp_in, args.lateral_balance_ramp_out,
        ) < 0.0:
            parser.error("lateral balance times and gains must be non-negative")
        if args.lateral_balance_max_offset_m <= 0.0:
            parser.error("--lateral-balance-max-offset-m must be positive")
        if args.lateral_balance_end < args.lateral_balance_start:
            parser.error("lateral balance end must be >= start")
        if (
            args.lateral_balance_start + args.lateral_balance_ramp_in
            > args.lateral_balance_end
        ):
            parser.error("lateral balance ramp-in must finish by its end")
    if args.pitch_balance_start is not None:
        if min(
            args.pitch_balance_start, args.pitch_balance_end,
            args.pitch_balance_kp, args.pitch_balance_kd,
            args.pitch_balance_ramp_in, args.pitch_balance_ramp_out,
        ) < 0.0:
            parser.error("pitch balance times and gains must be non-negative")
        if args.pitch_balance_max_angle_deg <= 0.0:
            parser.error("--pitch-balance-max-angle-deg must be positive")
        if args.pitch_balance_end < args.pitch_balance_start:
            parser.error("pitch balance end must be >= start")
        if (
            args.pitch_balance_start + args.pitch_balance_ramp_in
            > args.pitch_balance_end
        ):
            parser.error("pitch balance ramp-in must finish by its end")
        if args.world_flat_leg in ("left", "both"):
            parser.error(
                "pitch balance cannot be combined with left world-flat; "
                "use --world-flat-leg right"
            )
    touchdown_numeric = (
        args.touchdown_right_fz_threshold,
        args.touchdown_confirm_s,
        args.touchdown_left_feedback_ramp_out,
    )
    if not all(math.isfinite(value) for value in touchdown_numeric):
        parser.error("touchdown numeric arguments must be finite")
    if args.touchdown_right_fz_threshold <= 0.0:
        parser.error("--touchdown-right-fz-threshold must be positive")
    if args.touchdown_confirm_s < 0.0:
        parser.error("--touchdown-confirm-s must be non-negative")
    if args.touchdown_left_feedback_ramp_out < 0.0:
        parser.error("--touchdown-left-feedback-ramp-out must be non-negative")
    if args.touchdown_support_hold and args.swing_world_z_start is None:
        parser.error(
            "--touchdown-support-hold requires the right swing-world-Z options "
            "so contact detection can be armed after swing starts"
        )
    return args


def quaternion_rotation(quaternion):
    values = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm <= 0.0:
        return None
    return Rotation.from_quat(values / norm).as_matrix()


class SimulationClock:
    def __init__(self, topic):
        self.condition = threading.Condition()
        self.simulation_time = None
        self.base_rotation = None
        self.base_position = np.full(3, math.nan)
        self.base_x = math.nan
        self.base_vx = math.nan
        self.base_pitch_rate = math.nan
        self.callback = self.on_odometry
        self.node = Node()
        if self.node.subscribe(odometry_pb2.Odometry, topic, self.callback) is False:
            raise RuntimeError(f"failed to subscribe to {topic}")

    def on_odometry(self, message):
        stamp = message.header.stamp
        orientation = message.pose.orientation
        simulation_time = float(stamp.sec) + float(stamp.nsec) * 1e-9
        base_rotation = quaternion_rotation((
            orientation.x, orientation.y, orientation.z, orientation.w,
        ))
        base_x = float(message.pose.position.x)
        base_position = np.array((
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ), dtype=float)
        base_vx = float(message.twist.linear.x)
        base_pitch_rate = float(message.twist.angular.y)
        with self.condition:
            self.simulation_time = simulation_time
            self.base_rotation = base_rotation
            self.base_position = base_position
            self.base_x = base_x
            self.base_vx = base_vx
            self.base_pitch_rate = base_pitch_rate
            self.condition.notify_all()

    def wait_for_time_after(self, previous_time, timeout=None):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: not (
                    self.simulation_time is None
                    or (previous_time is not None and self.simulation_time <= previous_time)
                ),
                timeout=timeout,
            )
            rotation = (
                None if self.base_rotation is None else self.base_rotation.copy()
            )
            return (
                self.simulation_time, rotation, self.base_position.copy(),
                self.base_x, self.base_vx, self.base_pitch_rate, ready,
            )


def joint_name_from_command_topic(topic):
    return topic.removeprefix("/step/").removesuffix("/cmd_pos")


RIGHT_JOINT_NAMES = tuple(
    joint_name_from_command_topic(LEG_JOINT_TOPICS[f"RL{index}_wrap"])
    for index in range(6)
)


class TouchdownMeasurements:
    """Read right contact force and measured leg joints for touchdown latch."""

    def __init__(self, contact_topic, joint_state_topic):
        self.lock = threading.Lock()
        self.right_fz = math.nan
        self.contact_sequence = 0
        self.joint_positions = {}
        self.node = Node()
        subscriptions = (
            self.node.subscribe(Contacts, contact_topic, self.on_contacts),
            self.node.subscribe(Model, joint_state_topic, self.on_joint_state),
        )
        if not all(subscriptions):
            raise RuntimeError("failed to subscribe to touchdown measurements")

    def on_contacts(self, message):
        total_fz = 0.0
        found = False
        for contact in message.contact:
            collision1_is_sole = (
                RIGHT_SOLE_COLLISION_NAME in contact.collision1.name
            )
            collision2_is_sole = (
                RIGHT_SOLE_COLLISION_NAME in contact.collision2.name
            )
            if not collision1_is_sole and not collision2_is_sole:
                continue
            found = True
            for wrench in contact.wrench:
                force = (
                    wrench.body_1_wrench.force
                    if collision1_is_sole else wrench.body_2_wrench.force
                )
                total_fz += float(force.z)
        with self.lock:
            self.right_fz = total_fz if found else 0.0
            self.contact_sequence += 1

    def on_joint_state(self, message):
        positions = {}
        for joint in message.joint:
            name = joint.name.replace("/", "::").rsplit("::", 1)[-1]
            positions[name] = float(joint.axis1.position)
        with self.lock:
            self.joint_positions.update(positions)

    def snapshot(self):
        with self.lock:
            right_q = np.array([
                self.joint_positions.get(name, math.nan)
                for name in RIGHT_JOINT_NAMES
            ])
            return self.right_fz, self.contact_sequence, right_q


def load_targets(path):
    required = [f"RL{i}" for i in range(6)] + [f"LL{i}" for i in range(6)]
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"CSV is empty: {path}")
    missing = [name for name in required if name not in rows[0]]
    if missing:
        raise RuntimeError(f"missing CSV columns: {', '.join(missing)}")
    targets = []
    for index, row in enumerate(rows):
        frame = {}
        for side in ("RL", "LL"):
            for joint in range(6):
                key = f"{side}{joint}"
                value = float(row[key])
                if not math.isfinite(value):
                    raise RuntimeError(f"non-finite target at row {index}, {key}")
                frame[f"{key}_wrap"] = value
        targets.append(frame)
    return targets


def load_planner_lateral_reference(path, dt):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or "COM_y" not in rows[0]:
        raise RuntimeError(f"planner CSV missing COM_y: {path}")
    com_y = np.array([float(row["COM_y"]) for row in rows])
    if not np.all(np.isfinite(com_y)):
        raise RuntimeError(f"non-finite planner COM_y: {path}")
    displacement = com_y - com_y[0]
    velocity = np.empty_like(com_y)
    if len(com_y) == 1:
        velocity[0] = 0.0
    else:
        velocity[0] = (com_y[1] - com_y[0]) / dt
        velocity[-1] = (com_y[-1] - com_y[-2]) / dt
        if len(com_y) > 2:
            velocity[1:-1] = (com_y[2:] - com_y[:-2]) / (2.0 * dt)
    return displacement, velocity


def leg_vector(targets, prefix):
    return np.array([targets[f"{prefix}{joint}_wrap"] for joint in range(6)])


def set_leg_vector(targets, prefix, values):
    result = dict(targets)
    for joint, value in enumerate(values):
        result[f"{prefix}{joint}_wrap"] = float(value)
    return result


def limit_feedback_joint_rate(requested_q, previous_published_q,
                              max_speed_deg_s, dt):
    requested_delta = requested_q - previous_published_q
    requested_speed = np.max(np.abs(requested_delta)) / dt
    if max_speed_deg_s is None:
        return requested_q.copy(), 0, requested_speed, requested_speed
    max_delta = math.radians(max_speed_deg_s) * dt
    limited_delta = np.clip(requested_delta, -max_delta, max_delta)
    published_q = previous_published_q + limited_delta
    limited_count = int(np.count_nonzero(
        np.abs(requested_delta) > max_delta + 1e-15
    ))
    published_speed = np.max(np.abs(limited_delta)) / dt
    return published_q, limited_count, requested_speed, published_speed


def rotation_error_angle(target, actual):
    return np.linalg.norm(
        Rotation.from_matrix(target.T @ actual).as_rotvec()
    )


def smoothstep(unit_value):
    value = max(0.0, min(1.0, unit_value))
    return 3.0 * value * value - 2.0 * value * value * value


def feedback_beta(phase, trajectory_time, start, end, ramp_in, ramp_out):
    if phase != "REPLAY" or start is None:
        return 0.0
    if trajectory_time < start or trajectory_time > end + ramp_out:
        return 0.0
    if ramp_in > 0.0 and trajectory_time < start + ramp_in:
        return smoothstep((trajectory_time - start) / ramp_in)
    if trajectory_time <= end:
        return 1.0
    if ramp_out <= 0.0:
        return 0.0
    return 1.0 - smoothstep((trajectory_time - end) / ramp_out)


def world_flat_orientation(world_reference, nominal_world_rotation):
    """Keep the flat reference tilt while following nominal world yaw."""
    reference_heading = world_reference[:2, 0]
    nominal_heading = nominal_world_rotation[:2, 0]
    reference_norm = np.linalg.norm(reference_heading)
    nominal_norm = np.linalg.norm(nominal_heading)
    if reference_norm <= 1e-12 or nominal_norm <= 1e-12:
        return world_reference.copy()
    reference_heading = reference_heading / reference_norm
    nominal_heading = nominal_heading / nominal_norm
    yaw_delta = math.atan2(
        reference_heading[0] * nominal_heading[1]
        - reference_heading[1] * nominal_heading[0],
        np.dot(reference_heading, nominal_heading),
    )
    return Rotation.from_euler("z", yaw_delta).as_matrix() @ world_reference


class WorldFlatRightSolver:
    def __init__(self, chain, world_reference, dt, initial_base_rotation,
                 initial_right_position, initial_left_position,
                 leg_name="right"):
        self.chain = chain
        self.leg_name = leg_name
        self.world_reference = world_reference
        self.dt = dt
        self.initial_right_position = initial_right_position.copy()
        self.initial_left_position = initial_left_position.copy()
        self.initial_world_relative_z = (
            initial_base_rotation
            @ (initial_right_position - initial_left_position)
        )[2]
        self.previous_solution = None
        self.cached_frame = None
        self.cached_base_rotation = None
        self.cached_sim_time = None
        self.cached_result = None

    def _can_reuse(self, frame, base_rotation, simulation_time):
        if self.cached_result is None or frame != self.cached_frame:
            return False
        base_change = rotation_error_angle(
            self.cached_base_rotation, base_rotation,
        )
        if base_change <= ORIENTATION_CACHE_EPS_RAD:
            return True
        return simulation_time - self.cached_sim_time < self.dt - 1e-12

    def solve(self, frame, simulation_time, base_rotation, nominal_q,
              nominal_left_q, world_flat_beta, swing_world_z_beta,
              left_chain, support_x_offset=0.0,
              support_pitch_correction=0.0):
        if self._can_reuse(frame, base_rotation, simulation_time):
            return self.cached_result

        nominal_pose = self.chain.forward(nominal_q)
        nominal_left_pose = left_chain.forward(nominal_left_q)
        target = nominal_pose.copy()
        target[0, 3] += support_x_offset
        # Pitch balance is defined about the LEFT base-frame +Y axis.  A
        # negative correction therefore pre-multiplies the nominal sole
        # rotation; post-multiplication would rotate about the sole-local Y
        # axis and would couple the correction through nominal roll/yaw.
        if abs(support_pitch_correction) > 0.0:
            target[:3, :3] = (
                Rotation.from_rotvec(
                    [0.0, support_pitch_correction, 0.0]
                ).as_matrix()
                @ nominal_pose[:3, :3]
            )
        nominal_world_rotation = base_rotation @ nominal_pose[:3, :3]
        world_target_rotation = base_rotation @ target[:3, :3]
        if world_flat_beta > 0.0:
            flat_world_rotation = world_flat_orientation(
                self.world_reference, nominal_world_rotation,
            )
            flat_base_rotation = base_rotation.T @ flat_world_rotation
            relative_rotation = Rotation.from_matrix(
                nominal_pose[:3, :3].T @ flat_base_rotation
            ).as_rotvec()
            target[:3, :3] = (
                nominal_pose[:3, :3]
                @ Rotation.from_rotvec(
                    world_flat_beta * relative_rotation
                ).as_matrix()
            )
            world_target_rotation = base_rotation @ target[:3, :3]
        before_error = rotation_error_angle(
            world_target_rotation, nominal_world_rotation,
        )

        right_position = nominal_pose[:3, 3]
        left_position = nominal_left_pose[:3, 3]
        relative = right_position - left_position
        delta_z_motion = (
            right_position[2] - self.initial_right_position[2]
            - (left_position[2] - self.initial_left_position[2])
        )
        desired_relative_world_z = self.initial_world_relative_z + delta_z_motion
        predicted_before = (base_rotation @ relative)[2]
        tilt_r20_dx = base_rotation[2, 0] * relative[0]
        tilt_r21_dy = base_rotation[2, 1] * relative[1]
        z_valid = True
        if swing_world_z_beta > 0.0:
            if abs(base_rotation[2, 2]) <= 1e-6:
                z_valid = False
                print(
                    f"[WARNING] swing world-Z R22 too small at frame {frame}; "
                    "using nominal Z"
                )
            else:
                compensated_z = left_position[2] + (
                    desired_relative_world_z - tilt_r20_dx - tilt_r21_dy
                ) / base_rotation[2, 2]
                target[2, 3] = right_position[2] + swing_world_z_beta * (
                    compensated_z - right_position[2]
                )
        target_relative = target[:3, 3] - left_position
        predicted_after_target = (base_rotation @ target_relative)[2]

        if swing_world_z_beta > 0.0 and not z_valid and world_flat_beta <= 0.0:
            result = self._result(
                nominal_q, False, nominal_pose, nominal_left_pose, target,
                base_rotation, world_target_rotation, before_error,
                desired_relative_world_z, predicted_before,
                predicted_before, tilt_r20_dx, tilt_r21_dy,
            )
            self._cache(frame, simulation_time, base_rotation, result)
            return result
        seed = (
            nominal_q if self.previous_solution is None
            else self.previous_solution
        )
        try:
            solve_result = solve_leg_frame(self.chain, target, seed)
            candidate = solve_result.x
            valid = (
                solve_result.success
                and np.all(np.isfinite(candidate))
                and np.all(candidate >= self.chain.lower - 1e-10)
                and np.all(candidate <= self.chain.upper + 1e-10)
            )
        except Exception as error:  # Keep nominal commands on solver failure.
            print(
                f"[WARNING] {self.leg_name} 6D IK exception at frame "
                f"{frame}: {error}"
            )
            solve_result = None
            candidate = nominal_q
            valid = False

        solved_q = candidate if valid else nominal_q
        if not valid:
            print(
                f"[WARNING] {self.leg_name} 6D IK failed at frame {frame}; "
                "using nominal CSV"
            )
        else:
            self.previous_solution = candidate.copy()

        predicted_after = (
            predicted_after_target if valid else predicted_before
        )
        result = self._result(
            solved_q, valid, nominal_pose, nominal_left_pose, target,
            base_rotation, world_target_rotation, before_error,
            desired_relative_world_z, predicted_before, predicted_after,
            tilt_r20_dx, tilt_r21_dy,
        )
        result["world_flat_success"] = bool(valid and world_flat_beta > 0.0)
        result["swing_world_z_success"] = bool(
            valid and z_valid and swing_world_z_beta > 0.0
        )
        result["lateral_balance_success"] = bool(
            valid and abs(support_x_offset) > 0.0
        )
        result["pitch_balance_success"] = bool(
            valid and abs(support_pitch_correction) > 0.0
        )
        result["pitch_target_delta"] = support_pitch_correction
        result["nominal_base_pitch"] = Rotation.from_matrix(
            nominal_pose[:3, :3]
        ).as_euler("xyz")[1]
        result["target_base_pitch"] = Rotation.from_matrix(
            target[:3, :3]
        ).as_euler("xyz")[1]
        result["world_flat_beta"] = world_flat_beta
        result["swing_world_z_beta"] = swing_world_z_beta
        self._cache(frame, simulation_time, base_rotation, result)
        return result

    def _result(self, solved_q, valid, nominal_pose, nominal_left_pose, target,
                base_rotation, world_target_rotation, before_error,
                desired_relative_world_z, predicted_before, predicted_after,
                tilt_r20_dx, tilt_r21_dy):
        solved_pose = self.chain.forward(solved_q)
        position_error, orientation_error = pose_errors(solved_pose, target)
        after_error = rotation_error_angle(
            world_target_rotation, base_rotation @ solved_pose[:3, :3],
        )
        desired_rpy = Rotation.from_matrix(target[:3, :3]).as_euler("xyz")
        return {
            "q": solved_q.copy(),
            "success": bool(valid),
            "world_flat_success": False,
            "swing_world_z_success": False,
            "nominal_pos_err_mm": 1000.0 * np.linalg.norm(
                nominal_pose[:3, 3] - target[:3, 3]
            ),
            "world_ori_before_deg": math.degrees(before_error),
            "world_ori_after_deg": math.degrees(after_error),
            "ik_pos_err_mm": 1000.0 * np.linalg.norm(position_error),
            "ik_ori_err_deg": math.degrees(np.linalg.norm(orientation_error)),
            "desired_rpy": desired_rpy,
            "world_target_rpy": Rotation.from_matrix(
                world_target_rotation
            ).as_euler("xyz"),
            "nominal_relative_world_z": (
                nominal_pose[2, 3] - nominal_left_pose[2, 3]
            ),
            "desired_relative_world_z": desired_relative_world_z,
            "predicted_relative_world_z_before": predicted_before,
            "predicted_relative_world_z_after": predicted_after,
            "world_z_compensation": target[2, 3] - nominal_pose[2, 3],
            "target_base_z": target[2, 3],
            "tilt_r20_dx": tilt_r20_dx,
            "tilt_r21_dy": tilt_r21_dy,
            "lateral_target_x": target[0, 3],
        }

    def _cache(self, frame, simulation_time, base_rotation, result):
        self.cached_frame = frame
        self.cached_base_rotation = base_rotation.copy()
        self.cached_sim_time = simulation_time
        self.cached_result = result


class RightSupportHold:
    """Latch measured touchdown pose and keep it fixed in the world frame."""

    def __init__(self, chain, fz_threshold, confirm_duration, arm_trajectory_time,
                 left_ramp_out_duration):
        self.chain = chain
        self.fz_threshold = fz_threshold
        self.confirm_duration = confirm_duration
        self.arm_trajectory_time = arm_trajectory_time
        self.left_ramp_out_duration = left_ramp_out_duration
        self.state = "SWING"
        self.above_threshold_since = None
        self.contact_released_after_swing_start = False
        self.last_contact_sequence = None
        self.touchdown_sim_time = math.nan
        self.touchdown_trajectory_time = math.nan
        self.touchdown_right_fz = math.nan
        self.world_pose = None
        self.left_offset_at_touchdown = 0.0
        self.previous_successful_q = None
        self.warned_missing_joints = False

    @property
    def active(self):
        return self.state != "SWING"

    def update(self, sim_time, trajectory_time, right_fz, contact_sequence,
               measured_q,
               base_rotation, base_position, left_lateral_offset):
        if self.active:
            return False
        # Only new contact messages advance the simulation-time debounce. This
        # prevents one retained high-force sample from confirming touchdown.
        if contact_sequence == self.last_contact_sequence:
            return False
        self.last_contact_sequence = contact_sequence
        if trajectory_time + 1e-12 < self.arm_trajectory_time:
            self.above_threshold_since = None
            return False
        if not self.contact_released_after_swing_start:
            if math.isfinite(right_fz) and right_fz < self.fz_threshold:
                self.contact_released_after_swing_start = True
            return False
        if not math.isfinite(right_fz) or right_fz < self.fz_threshold:
            self.above_threshold_since = None
            return False
        if self.above_threshold_since is None:
            self.above_threshold_since = sim_time
        if sim_time - self.above_threshold_since + 1e-12 < self.confirm_duration:
            return False
        if not np.all(np.isfinite(measured_q)):
            if not self.warned_missing_joints:
                print(
                    "[WARNING] touchdown force confirmed but measured right "
                    "joint positions are incomplete; remaining in SWING"
                )
                self.warned_missing_joints = True
            return False

        base_pose = self.chain.forward(measured_q)
        self.world_pose = np.eye(4)
        self.world_pose[:3, :3] = base_rotation @ base_pose[:3, :3]
        self.world_pose[:3, 3] = (
            base_position + base_rotation @ base_pose[:3, 3]
        )
        self.touchdown_sim_time = sim_time
        self.touchdown_trajectory_time = trajectory_time
        self.touchdown_right_fz = right_fz
        self.left_offset_at_touchdown = left_lateral_offset
        self.previous_successful_q = measured_q.copy()
        self.state = "TOUCHDOWN_CONFIRMED"
        return True

    def left_handoff(self, sim_time):
        if not self.active:
            return 1.0, math.nan, math.nan
        if self.left_ramp_out_duration <= 0.0:
            beta = 0.0
        else:
            elapsed = max(0.0, sim_time - self.touchdown_sim_time)
            beta = 1.0 - smoothstep(elapsed / self.left_ramp_out_duration)
        return (
            beta,
            self.left_offset_at_touchdown,
            beta * self.left_offset_at_touchdown,
        )

    def solve(self, base_rotation, base_position, measured_q, fallback_q):
        target = np.eye(4)
        target[:3, :3] = base_rotation.T @ self.world_pose[:3, :3]
        target[:3, 3] = base_rotation.T @ (
            self.world_pose[:3, 3] - base_position
        )
        seed = (
            self.previous_successful_q
            if self.previous_successful_q is not None else fallback_q
        )
        try:
            solve_result = solve_leg_frame(self.chain, target, seed)
            candidate = solve_result.x
            valid = (
                solve_result.success
                and np.all(np.isfinite(candidate))
                and np.all(candidate >= self.chain.lower - 1e-10)
                and np.all(candidate <= self.chain.upper + 1e-10)
            )
        except Exception as error:
            print(f"[WARNING] right support-hold IK exception: {error}")
            candidate = fallback_q
            valid = False
        if valid:
            solved_q = candidate
            self.previous_successful_q = candidate.copy()
        else:
            # Preserve the last valid support solution; before one exists,
            # retain the last safely published right-leg command.
            solved_q = (
                self.previous_successful_q.copy()
                if self.previous_successful_q is not None else fallback_q.copy()
            )
            print(
                "[WARNING] right support-hold IK failed; retaining previous "
                "successful support command"
            )

        solved_pose = self.chain.forward(solved_q)
        solved_world_position = (
            base_position + base_rotation @ solved_pose[:3, 3]
        )
        solved_world_rotation = base_rotation @ solved_pose[:3, :3]
        actual_world_position = np.full(3, math.nan)
        actual_world_rotation = None
        if np.all(np.isfinite(measured_q)):
            actual_pose = self.chain.forward(measured_q)
            actual_world_position = (
                base_position + base_rotation @ actual_pose[:3, 3]
            )
            actual_world_rotation = base_rotation @ actual_pose[:3, :3]
        diagnostic_position = (
            actual_world_position
            if np.all(np.isfinite(actual_world_position))
            else solved_world_position
        )
        diagnostic_rotation = (
            actual_world_rotation
            if actual_world_rotation is not None else solved_world_rotation
        )
        return {
            "q": solved_q.copy(),
            "success": bool(valid),
            "target_base_pose": target,
            "target_world_position": self.world_pose[:3, 3].copy(),
            "actual_or_fk_world_position": diagnostic_position,
            "world_pos_error_mm": 1000.0 * np.linalg.norm(
                diagnostic_position - self.world_pose[:3, 3]
            ),
            "world_ori_error_deg": math.degrees(rotation_error_angle(
                self.world_pose[:3, :3], diagnostic_rotation,
            )),
        }

    def finish_confirmation_frame(self):
        if self.state == "TOUCHDOWN_CONFIRMED":
            self.state = "RIGHT_SUPPORT_HOLD"


def command_tuples(targets):
    return [
        (key, topic, targets[key], targets[key])
        for key, topic in LEG_JOINT_TOPICS.items()
    ]


def publish_targets(publishers, targets):
    publishers.publish(command_tuples(targets))


def log_columns():
    return (
        ["publish_index", "simulation_time", "trajectory_time", "source_frame", "phase"]
        + [f"RL{i}" for i in range(6)]
        + [f"LL{i}" for i in range(6)]
        + [
            "base_roll", "base_pitch", "base_yaw",
            "world_flat_active", "world_flat_solver_success",
            "swing_world_z_active", "swing_world_z_solver_success",
            "right_nominal_pos_err_mm",
            "right_world_ori_err_before_deg", "right_world_ori_err_after_deg",
            "right_ik_pos_err_mm", "right_ik_ori_err_deg",
            "desired_base_relative_roll", "desired_base_relative_pitch",
            "desired_base_relative_yaw", "max_joint_delta_deg",
            "max_joint_velocity_deg_s",
            "right_nominal_relative_world_z_mm",
            "right_desired_relative_world_z_mm",
            "right_predicted_relative_world_z_before_mm",
            "right_predicted_relative_world_z_after_mm",
            "right_world_z_compensation_mm", "right_target_base_z_mm",
            "tilt_term_r20_dx_mm", "tilt_term_r21_dy_mm",
            "feedback_rate_limit_active",
            "feedback_rate_limited_joint_count",
            "feedback_max_requested_speed_deg_s",
            "feedback_max_published_speed_deg_s",
            "left_world_ori_err_before_deg", "left_world_ori_err_after_deg",
            "left_world_flat_solver_success",
            "right_world_flat_solver_success",
            "left_feedback_rate_limited_joint_count",
            "right_feedback_rate_limited_joint_count",
            "left_feedback_max_requested_speed_deg_s",
            "right_feedback_max_requested_speed_deg_s",
            "left_feedback_max_published_speed_deg_s",
            "right_feedback_max_published_speed_deg_s",
            "lateral_balance_active", "planner_lateral_dx_m",
            "planner_lateral_vx_mps", "base_dx_actual_m",
            "base_vx_actual_mps", "lateral_position_error_m",
            "lateral_velocity_error_mps",
            "lateral_support_x_raw_offset_m",
            "lateral_support_x_applied_offset_m", "lateral_balance_beta",
            "left_lateral_solver_success", "left_lateral_target_x_m",
            "left_lateral_ik_pos_err_mm",
            "touchdown_state", "touchdown_detected", "touchdown_right_fz",
            "touchdown_sim_time", "touchdown_trajectory_time",
            "right_support_hold_active",
            "right_support_target_world_x", "right_support_target_world_y",
            "right_support_target_world_z",
            "right_support_actual_or_fk_world_x",
            "right_support_actual_or_fk_world_y",
            "right_support_actual_or_fk_world_z",
            "right_support_world_pos_error_mm",
            "right_support_world_ori_error_deg",
            "left_touchdown_handoff_beta",
            "left_lateral_offset_before_handoff_m",
            "left_lateral_offset_after_handoff_m",
            "right_support_solver_success",
            "pitch_balance_active", "base_pitch_reference_rad",
            "base_pitch_actual_rad", "base_pitch_error_rad",
            "base_pitch_rate_actual_rad_s", "base_pitch_rate_error_rad_s",
            "pitch_balance_raw_correction_rad",
            "pitch_balance_applied_correction_rad", "pitch_balance_beta",
            "left_pitch_balance_solver_success",
            "left_nominal_base_pitch_deg", "left_target_base_pitch_deg",
            "left_pitch_target_delta_deg",
        ]
    )


def finite_text(value):
    return f"{value:+.12f}" if math.isfinite(value) else "nan"


def write_log(writer, publish_index, simulation_time, trajectory_time,
              source_frame, phase, targets, base_rpy, world_flat_active,
              swing_world_z_active, solve_result,
              max_delta_deg, max_velocity_deg_s, rate_limit_diagnostics=None,
              left_solve_result=None, left_rate_diagnostics=None,
              lateral_diagnostics=None, touchdown_diagnostics=None,
              pitch_diagnostics=None):
    desired_rpy = (
        np.full(3, math.nan) if solve_result is None
        else solve_result["desired_rpy"]
    )
    row = {
        "publish_index": publish_index,
        "simulation_time": finite_text(simulation_time),
        "trajectory_time": f"{trajectory_time:.9f}",
        "source_frame": source_frame,
        "phase": phase,
        "base_roll": finite_text(base_rpy[0]),
        "base_pitch": finite_text(base_rpy[1]),
        "base_yaw": finite_text(base_rpy[2]),
        "world_flat_active": int(world_flat_active),
        "world_flat_solver_success": int(
            solve_result is not None and solve_result["world_flat_success"]
        ),
        "swing_world_z_active": int(swing_world_z_active),
        "swing_world_z_solver_success": int(
            solve_result is not None and solve_result["swing_world_z_success"]
        ),
        "right_nominal_pos_err_mm": finite_text(
            math.nan if solve_result is None
            else solve_result["nominal_pos_err_mm"]
        ),
        "right_world_ori_err_before_deg": finite_text(
            math.nan if solve_result is None
            else solve_result["world_ori_before_deg"]
        ),
        "right_world_ori_err_after_deg": finite_text(
            math.nan if solve_result is None
            else solve_result["world_ori_after_deg"]
        ),
        "right_ik_pos_err_mm": finite_text(
            math.nan if solve_result is None else solve_result["ik_pos_err_mm"]
        ),
        "right_ik_ori_err_deg": finite_text(
            math.nan if solve_result is None else solve_result["ik_ori_err_deg"]
        ),
        "desired_base_relative_roll": finite_text(desired_rpy[0]),
        "desired_base_relative_pitch": finite_text(desired_rpy[1]),
        "desired_base_relative_yaw": finite_text(desired_rpy[2]),
        "max_joint_delta_deg": finite_text(max_delta_deg),
        "max_joint_velocity_deg_s": finite_text(max_velocity_deg_s),
    }
    rate_limit_diagnostics = rate_limit_diagnostics or {
        "active": False,
        "limited_joint_count": 0,
        "requested_speed_deg_s": 0.0,
        "published_speed_deg_s": 0.0,
    }
    row.update({
        "feedback_rate_limit_active": int(rate_limit_diagnostics["active"]),
        "feedback_rate_limited_joint_count": (
            rate_limit_diagnostics["limited_joint_count"]
        ),
        "feedback_max_requested_speed_deg_s": finite_text(
            rate_limit_diagnostics["requested_speed_deg_s"]
        ),
        "feedback_max_published_speed_deg_s": finite_text(
            rate_limit_diagnostics["published_speed_deg_s"]
        ),
    })
    lateral_diagnostics = lateral_diagnostics or {
        "active": False,
        "planner_dx": math.nan, "planner_vx": math.nan,
        "base_dx": math.nan, "base_vx": math.nan,
        "position_error": math.nan, "velocity_error": math.nan,
        "raw_offset": math.nan, "applied_offset": math.nan,
        "beta": 0.0,
    }
    row.update({
        "lateral_balance_active": int(lateral_diagnostics["active"]),
        "planner_lateral_dx_m": finite_text(lateral_diagnostics["planner_dx"]),
        "planner_lateral_vx_mps": finite_text(lateral_diagnostics["planner_vx"]),
        "base_dx_actual_m": finite_text(lateral_diagnostics["base_dx"]),
        "base_vx_actual_mps": finite_text(lateral_diagnostics["base_vx"]),
        "lateral_position_error_m": finite_text(
            lateral_diagnostics["position_error"]
        ),
        "lateral_velocity_error_mps": finite_text(
            lateral_diagnostics["velocity_error"]
        ),
        "lateral_support_x_raw_offset_m": finite_text(
            lateral_diagnostics["raw_offset"]
        ),
        "lateral_support_x_applied_offset_m": finite_text(
            lateral_diagnostics["applied_offset"]
        ),
        "lateral_balance_beta": finite_text(lateral_diagnostics["beta"]),
        "left_lateral_solver_success": int(
            left_solve_result is not None
            and left_solve_result.get("lateral_balance_success", False)
        ),
        "left_lateral_target_x_m": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["lateral_target_x"]
        ),
        "left_lateral_ik_pos_err_mm": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["ik_pos_err_mm"]
        ),
    })
    touchdown_diagnostics = touchdown_diagnostics or {
        "state": "DISABLED", "detected": False, "right_fz": math.nan,
        "sim_time": math.nan, "trajectory_time": math.nan,
        "support_active": False, "support_result": None,
        "left_handoff_beta": 1.0,
        "left_offset_before": math.nan, "left_offset_after": math.nan,
    }
    support_result = touchdown_diagnostics["support_result"]
    target_world = (
        np.full(3, math.nan) if support_result is None
        else support_result["target_world_position"]
    )
    actual_world = (
        np.full(3, math.nan) if support_result is None
        else support_result["actual_or_fk_world_position"]
    )
    row.update({
        "touchdown_state": touchdown_diagnostics["state"],
        "touchdown_detected": int(touchdown_diagnostics["detected"]),
        "touchdown_right_fz": finite_text(touchdown_diagnostics["right_fz"]),
        "touchdown_sim_time": finite_text(touchdown_diagnostics["sim_time"]),
        "touchdown_trajectory_time": finite_text(
            touchdown_diagnostics["trajectory_time"]
        ),
        "right_support_hold_active": int(
            touchdown_diagnostics["support_active"]
        ),
        "right_support_target_world_x": finite_text(target_world[0]),
        "right_support_target_world_y": finite_text(target_world[1]),
        "right_support_target_world_z": finite_text(target_world[2]),
        "right_support_actual_or_fk_world_x": finite_text(actual_world[0]),
        "right_support_actual_or_fk_world_y": finite_text(actual_world[1]),
        "right_support_actual_or_fk_world_z": finite_text(actual_world[2]),
        "right_support_world_pos_error_mm": finite_text(
            math.nan if support_result is None
            else support_result["world_pos_error_mm"]
        ),
        "right_support_world_ori_error_deg": finite_text(
            math.nan if support_result is None
            else support_result["world_ori_error_deg"]
        ),
        "left_touchdown_handoff_beta": finite_text(
            touchdown_diagnostics["left_handoff_beta"]
        ),
        "left_lateral_offset_before_handoff_m": finite_text(
            touchdown_diagnostics["left_offset_before"]
        ),
        "left_lateral_offset_after_handoff_m": finite_text(
            touchdown_diagnostics["left_offset_after"]
        ),
        "right_support_solver_success": int(
            support_result is not None and support_result["success"]
        ),
    })
    pitch_diagnostics = pitch_diagnostics or {
        "active": False, "reference": math.nan, "actual": math.nan,
        "error": math.nan, "rate": math.nan, "rate_error": math.nan,
        "raw_correction": math.nan, "applied_correction": math.nan,
        "beta": 0.0,
    }
    row.update({
        "pitch_balance_active": int(pitch_diagnostics["active"]),
        "base_pitch_reference_rad": finite_text(pitch_diagnostics["reference"]),
        "base_pitch_actual_rad": finite_text(pitch_diagnostics["actual"]),
        "base_pitch_error_rad": finite_text(pitch_diagnostics["error"]),
        "base_pitch_rate_actual_rad_s": finite_text(pitch_diagnostics["rate"]),
        "base_pitch_rate_error_rad_s": finite_text(
            pitch_diagnostics["rate_error"]
        ),
        "pitch_balance_raw_correction_rad": finite_text(
            pitch_diagnostics["raw_correction"]
        ),
        "pitch_balance_applied_correction_rad": finite_text(
            pitch_diagnostics["applied_correction"]
        ),
        "pitch_balance_beta": finite_text(pitch_diagnostics["beta"]),
        "left_pitch_balance_solver_success": int(
            left_solve_result is not None
            and left_solve_result.get("pitch_balance_success", False)
        ),
        "left_nominal_base_pitch_deg": finite_text(
            math.nan if left_solve_result is None
            else math.degrees(left_solve_result.get("nominal_base_pitch", math.nan))
        ),
        "left_target_base_pitch_deg": finite_text(
            math.nan if left_solve_result is None
            else math.degrees(left_solve_result.get("target_base_pitch", math.nan))
        ),
        "left_pitch_target_delta_deg": finite_text(
            math.nan if left_solve_result is None
            else math.degrees(left_solve_result.get("pitch_target_delta", math.nan))
        ),
    })
    left_rate_diagnostics = left_rate_diagnostics or {
        "limited_joint_count": 0,
        "requested_speed_deg_s": 0.0,
        "published_speed_deg_s": 0.0,
    }
    row.update({
        "left_world_ori_err_before_deg": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["world_ori_before_deg"]
        ),
        "left_world_ori_err_after_deg": finite_text(
            math.nan if left_solve_result is None
            else left_solve_result["world_ori_after_deg"]
        ),
        "left_world_flat_solver_success": int(
            left_solve_result is not None
            and left_solve_result["world_flat_success"]
        ),
        "right_world_flat_solver_success": int(
            solve_result is not None and solve_result["world_flat_success"]
        ),
        "left_feedback_rate_limited_joint_count": (
            left_rate_diagnostics["limited_joint_count"]
        ),
        "right_feedback_rate_limited_joint_count": (
            rate_limit_diagnostics["limited_joint_count"]
        ),
        "left_feedback_max_requested_speed_deg_s": finite_text(
            left_rate_diagnostics["requested_speed_deg_s"]
        ),
        "right_feedback_max_requested_speed_deg_s": finite_text(
            rate_limit_diagnostics["requested_speed_deg_s"]
        ),
        "left_feedback_max_published_speed_deg_s": finite_text(
            left_rate_diagnostics["published_speed_deg_s"]
        ),
        "right_feedback_max_published_speed_deg_s": finite_text(
            rate_limit_diagnostics["published_speed_deg_s"]
        ),
    })
    diagnostic_keys = {
        "right_nominal_relative_world_z_mm": "nominal_relative_world_z",
        "right_desired_relative_world_z_mm": "desired_relative_world_z",
        "right_predicted_relative_world_z_before_mm": (
            "predicted_relative_world_z_before"
        ),
        "right_predicted_relative_world_z_after_mm": (
            "predicted_relative_world_z_after"
        ),
        "right_world_z_compensation_mm": "world_z_compensation",
        "right_target_base_z_mm": "target_base_z",
        "tilt_term_r20_dx_mm": "tilt_r20_dx",
        "tilt_term_r21_dy_mm": "tilt_r21_dy",
    }
    for column, key in diagnostic_keys.items():
        row[column] = finite_text(
            math.nan if solve_result is None else 1000.0 * solve_result[key]
        )
    for side in ("RL", "LL"):
        for joint in range(6):
            row[f"{side}{joint}"] = finite_text(targets[f"{side}{joint}_wrap"])
    writer.writerow(row)


class CommandSafetyMonitor:
    def __init__(self):
        self.previous = None
        self.previous_change_time = None

    def check(self, targets, simulation_time):
        current = np.array([targets[key] for key in LEG_JOINT_TOPICS])
        if self.previous is None:
            self.previous = current
            self.previous_change_time = simulation_time
            return 0.0, 0.0
        delta = np.max(np.abs(current - self.previous))
        max_velocity = 0.0
        if delta > 1e-14:
            elapsed = simulation_time - self.previous_change_time
            if elapsed > 0.0:
                max_velocity = delta / elapsed
            self.previous = current
            self.previous_change_time = simulation_time
            if max_velocity > VELOCITY_WARNING_RAD_S:
                print(
                    "[WARNING] command velocity exceeds 270 deg/s: "
                    f"{math.degrees(max_velocity):.3f} deg/s"
                )
        return math.degrees(delta), math.degrees(max_velocity)


def nominal_validation(targets, chains):
    expected = np.concatenate((CANDIDATE_B["right"], CANDIDATE_B["left"]))
    actual = np.concatenate((leg_vector(targets[0], "RL"), leg_vector(targets[0], "LL")))
    frame_zero_exact = np.array_equal(actual, expected)
    poses_finite = True
    for frame in targets:
        poses_finite = poses_finite and np.all(np.isfinite(
            chains["right"].forward(leg_vector(frame, "RL"))
        ))
        poses_finite = poses_finite and np.all(np.isfinite(
            chains["left"].forward(leg_vector(frame, "LL"))
        ))
    return frame_zero_exact, poses_finite


def main():
    args = parse_args()
    targets = load_targets(args.csv)
    planner_dx = None
    planner_vx = None
    if args.lateral_balance_planner_csv is not None:
        planner_dx, planner_vx = load_planner_lateral_reference(
            args.lateral_balance_planner_csv, args.dt,
        )
        if len(planner_dx) < len(targets):
            raise RuntimeError("planner CSV is shorter than nominal command CSV")
    chains = load_leg_chains(args.sdf)
    frame_zero_exact, nominal_poses_finite = nominal_validation(targets, chains)
    replay_last_frame = min(
        int(math.floor(args.replay_end / args.dt + 1e-12)), len(targets) - 1,
    )
    print("[STEP SDF6D WORLD-FLAT REPLAY]")
    print(f"CSV={args.csv}")
    print(f"source rows={len(targets)}")
    print(f"SDF={args.sdf}")
    print(f"hold-before={args.hold_before:.3f} sim-s")
    print(f"replay=0.000~{replay_last_frame * args.dt:.3f} sim-s")
    print(f"hold-after={args.hold_after:.3f} sim-s")
    print(
        f"world-flat leg={args.world_flat_leg} "
        f"window={args.world_flat_start:.3f}~{args.world_flat_end:.3f} sim-s "
        f"ramp-in/out={args.world_flat_ramp_in:.3f}/"
        f"{args.world_flat_ramp_out:.3f} sim-s"
    )
    if planner_dx is None:
        print("lateral-balance=disabled")
    else:
        print(
            f"lateral-balance planner={args.lateral_balance_planner_csv} "
            f"window={args.lateral_balance_start:.3f}~"
            f"{args.lateral_balance_end:.3f} sim-s "
            f"Kp/Kd={args.lateral_balance_kp:.6f}/"
            f"{args.lateral_balance_kd:.6f} "
            f"max-offset={args.lateral_balance_max_offset_m:.6f} m"
        )
    if args.pitch_balance_start is None:
        print("pitch-balance=disabled")
    else:
        print(
            f"pitch-balance window={args.pitch_balance_start:.3f}~"
            f"{args.pitch_balance_end:.3f} sim-s "
            f"Kp/Kd={args.pitch_balance_kp:.6f}/"
            f"{args.pitch_balance_kd:.6f} s "
            f"max-angle={args.pitch_balance_max_angle_deg:.3f} deg "
            f"ramp-in/out={args.pitch_balance_ramp_in:.3f}/"
            f"{args.pitch_balance_ramp_out:.3f} sim-s"
        )
    if args.swing_world_z_leg is None:
        print("swing-world-Z=disabled")
    else:
        print(
            f"swing-world-Z leg={args.swing_world_z_leg} "
            f"window={args.swing_world_z_start:.3f}~"
            f"{args.swing_world_z_end:.3f} sim-s "
            f"ramp-in/out={args.swing_world_z_ramp_in:.3f}/"
            f"{args.swing_world_z_ramp_out:.3f} sim-s"
        )
    print(
        "max-feedback-joint-speed="
        + (
            "disabled"
            if args.max_feedback_joint_speed_deg_s is None
            else f"{args.max_feedback_joint_speed_deg_s:.3f} deg/s"
        )
    )
    if args.touchdown_support_hold:
        print(
            "touchdown-support-hold=enabled "
            f"contact={args.touchdown_right_contact_topic} "
            f"joint-state={args.touchdown_leg_joint_state_topic} "
            f"Fz>={args.touchdown_right_fz_threshold:.3f} N for "
            f"{args.touchdown_confirm_s:.3f} sim-s "
            f"left-ramp-out={args.touchdown_left_feedback_ramp_out:.3f} sim-s"
        )
    else:
        print("touchdown-support-hold=disabled")
    print(f"frame0 Candidate B exact={'YES' if frame_zero_exact else 'NO'}")
    print(f"nominal SDF FK finite={'YES' if nominal_poses_finite else 'NO'}")
    if not frame_zero_exact or not nominal_poses_finite:
        raise RuntimeError("nominal CSV/SDF static validation failed")

    if args.dry_run:
        print("world-flat runtime solve requires live base odometry")
        print("[DRY RUN COMPLETE]")
        return

    args.command_log.parent.mkdir(parents=True, exist_ok=True)
    publishers = GazeboDoublePublishers(list(LEG_JOINT_TOPICS.values()), False)
    clock = SimulationClock(args.odom_topic)
    touchdown_measurements = (
        TouchdownMeasurements(
            args.touchdown_right_contact_topic,
            args.touchdown_leg_joint_state_topic,
        )
        if args.touchdown_support_hold else None
    )

    with args.command_log.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=log_columns())
        writer.writeheader()
        publish_targets(publishers, targets[0])
        write_log(
            writer, 0, math.nan, 0.0, 0, "INITIAL_PRELOAD", targets[0],
            np.full(3, math.nan), False, False, None, 0.0, 0.0,
        )
        stream.flush()

        previous_sim_time = None
        while True:
            (
                sim_time, base_rotation, base_position, base_x, base_vx,
                odom_pitch_rate, ready,
            ) = clock.wait_for_time_after(
                previous_sim_time, timeout=5.0,
            )
            if ready and sim_time is not None and base_rotation is not None:
                start_sim_time = sim_time
                break
            print("[WAIT] no valid simulation-time/base-orientation update yet")

        initial_right_pose = chains["right"].forward(CANDIDATE_B["right"])
        initial_left_pose = chains["left"].forward(CANDIDATE_B["left"])
        right_solver = WorldFlatRightSolver(
            chains["right"], base_rotation @ initial_right_pose[:3, :3],
            args.dt, base_rotation,
            initial_right_pose[:3, 3], initial_left_pose[:3, 3],
            leg_name="right",
        )
        left_solver = WorldFlatRightSolver(
            chains["left"], base_rotation @ initial_left_pose[:3, :3],
            args.dt, base_rotation,
            initial_left_pose[:3, 3], initial_right_pose[:3, 3],
            leg_name="left",
        )
        touchdown_hold = (
            RightSupportHold(
                chains["right"], args.touchdown_right_fz_threshold,
                args.touchdown_confirm_s, args.swing_world_z_start,
                args.touchdown_left_feedback_ramp_out,
            )
            if args.touchdown_support_hold else None
        )
        safety = CommandSafetyMonitor()
        safety.check(targets[0], start_sim_time)
        last_published_targets = dict(targets[0])
        print(f"[START] sim_time={start_sim_time:.9f}")

        publish_index = 1
        next_frame = 0
        replay_finished_at = None
        feedback_release_pending = {"RL": False, "LL": False}
        replay_base_x = None
        replay_base_pitch = None
        derivative_previous_pitch = None
        derivative_previous_time = None
        while True:
            (
                sim_time, base_rotation, base_position, base_x, base_vx,
                odom_pitch_rate, ready,
            ) = clock.wait_for_time_after(
                previous_sim_time, timeout=5.0,
            )
            if not ready:
                print("[WAIT] simulation time not advancing")
                continue
            previous_sim_time = sim_time
            if base_rotation is None:
                print("[WARNING] invalid base quaternion; retaining prior command")
                continue
            base_rpy = Rotation.from_matrix(base_rotation).as_euler("xyz")
            if math.isfinite(odom_pitch_rate):
                base_pitch_rate = odom_pitch_rate
            elif (
                derivative_previous_pitch is not None
                and sim_time > derivative_previous_time
            ):
                base_pitch_rate = (
                    base_rpy[1] - derivative_previous_pitch
                ) / (sim_time - derivative_previous_time)
            else:
                base_pitch_rate = 0.0
            derivative_previous_pitch = base_rpy[1]
            derivative_previous_time = sim_time
            elapsed = sim_time - start_sim_time
            if elapsed < args.hold_before:
                continue
            trajectory_elapsed = elapsed - args.hold_before

            while (
                next_frame <= replay_last_frame
                and trajectory_elapsed + 1e-9 >= next_frame * args.dt
            ):
                trajectory_time = next_frame * args.dt
                nominal = targets[next_frame]
                if replay_base_x is None:
                    replay_base_x = base_x
                if replay_base_pitch is None:
                    replay_base_pitch = base_rpy[1]
                world_flat_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.world_flat_start, args.world_flat_end,
                    args.world_flat_ramp_in, args.world_flat_ramp_out,
                )
                swing_world_z_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.swing_world_z_start, args.swing_world_z_end,
                    args.swing_world_z_ramp_in, args.swing_world_z_ramp_out,
                )
                world_flat_active = world_flat_beta > 0.0
                swing_world_z_active = swing_world_z_beta > 0.0
                right_world_beta = (
                    world_flat_beta
                    if args.world_flat_leg in ("right", "both") else 0.0
                )
                left_world_beta = (
                    world_flat_beta
                    if args.world_flat_leg in ("left", "both") else 0.0
                )
                lateral_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.lateral_balance_start, args.lateral_balance_end,
                    args.lateral_balance_ramp_in, args.lateral_balance_ramp_out,
                )
                lateral_active = lateral_beta > 0.0
                lateral_diagnostics = None
                lateral_offset = 0.0
                if planner_dx is not None:
                    base_dx = base_x - replay_base_x
                    position_error = base_dx - planner_dx[next_frame]
                    velocity_error = base_vx - planner_vx[next_frame]
                    raw_offset = (
                        args.lateral_balance_kp * position_error
                        + args.lateral_balance_kd * velocity_error
                    )
                    clamped_offset = max(
                        -args.lateral_balance_max_offset_m,
                        min(args.lateral_balance_max_offset_m, raw_offset),
                    )
                    lateral_offset = lateral_beta * clamped_offset
                    lateral_diagnostics = {
                        "active": lateral_active,
                        "planner_dx": planner_dx[next_frame],
                        "planner_vx": planner_vx[next_frame],
                        "base_dx": base_dx, "base_vx": base_vx,
                        "position_error": position_error,
                        "velocity_error": velocity_error,
                        "raw_offset": raw_offset,
                        "applied_offset": lateral_offset,
                        "beta": lateral_beta,
                    }
                pitch_beta = feedback_beta(
                    "REPLAY", trajectory_time,
                    args.pitch_balance_start, args.pitch_balance_end,
                    args.pitch_balance_ramp_in, args.pitch_balance_ramp_out,
                )
                pitch_active = pitch_beta > 0.0
                pitch_correction = 0.0
                pitch_diagnostics = None
                if args.pitch_balance_start is not None:
                    pitch_error = base_rpy[1] - replay_base_pitch
                    pitch_rate_error = base_pitch_rate
                    pitch_raw = (
                        args.pitch_balance_kp * pitch_error
                        + args.pitch_balance_kd * pitch_rate_error
                    )
                    max_pitch = math.radians(
                        args.pitch_balance_max_angle_deg
                    )
                    pitch_clamped = max(
                        -max_pitch, min(max_pitch, pitch_raw)
                    )
                    pitch_correction = pitch_beta * pitch_clamped
                    pitch_diagnostics = {
                        "active": pitch_active,
                        "reference": replay_base_pitch,
                        "actual": base_rpy[1], "error": pitch_error,
                        "rate": base_pitch_rate,
                        "rate_error": pitch_rate_error,
                        "raw_correction": pitch_raw,
                        "applied_correction": pitch_correction,
                        "beta": pitch_beta,
                    }
                right_fz = math.nan
                measured_right_q = np.full(6, math.nan)
                touchdown_detected_now = False
                if touchdown_measurements is not None:
                    (
                        right_fz, contact_sequence, measured_right_q,
                    ) = touchdown_measurements.snapshot()
                    touchdown_detected_now = touchdown_hold.update(
                        sim_time, trajectory_time, right_fz, contact_sequence,
                        measured_right_q,
                        base_rotation, base_position, lateral_offset,
                    )
                left_handoff_beta = 1.0
                left_offset_before_handoff = math.nan
                left_offset_after_handoff = math.nan
                if touchdown_hold is not None and touchdown_hold.active:
                    (
                        left_handoff_beta, left_offset_before_handoff,
                        left_offset_after_handoff,
                    ) = touchdown_hold.left_handoff(sim_time)
                    lateral_offset = left_offset_after_handoff
                    lateral_active = abs(lateral_offset) > 1e-12
                    if lateral_diagnostics is not None:
                        lateral_diagnostics["active"] = lateral_active
                        lateral_diagnostics["applied_offset"] = lateral_offset
                        lateral_diagnostics["beta"] = left_handoff_beta
                solve_result = None
                support_result = None
                left_solve_result = None
                final = nominal
                rate_diagnostics = None
                left_rate_diagnostics = None
                if touchdown_hold is not None and touchdown_hold.active:
                    support_result = touchdown_hold.solve(
                        base_rotation, base_position, measured_right_q,
                        leg_vector(last_published_targets, "RL"),
                    )
                    requested_q = support_result["q"]
                    (
                        published_q, limited_count,
                        requested_speed, published_speed,
                    ) = limit_feedback_joint_rate(
                        requested_q,
                        leg_vector(last_published_targets, "RL"),
                        args.max_feedback_joint_speed_deg_s,
                        args.dt,
                    )
                    rate_diagnostics = {
                        "active": args.max_feedback_joint_speed_deg_s is not None,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                    final = set_leg_vector(nominal, "RL", published_q)
                    feedback_release_pending["RL"] = True
                    world_flat_active = left_world_beta > 0.0
                    swing_world_z_active = False
                elif right_world_beta > 0.0 or swing_world_z_active:
                    solve_result = right_solver.solve(
                        next_frame, sim_time, base_rotation,
                        leg_vector(nominal, "RL"),
                        leg_vector(nominal, "LL"),
                        right_world_beta, swing_world_z_beta,
                        chains["left"],
                    )
                    requested_q = solve_result["q"]
                    published_q = requested_q
                    limited_count = 0
                    requested_speed = 0.0
                    published_speed = 0.0
                    limiter_active = (
                        solve_result["success"]
                        and args.max_feedback_joint_speed_deg_s is not None
                    )
                    if solve_result["success"]:
                        (
                            published_q, limited_count,
                            requested_speed, published_speed,
                        ) = limit_feedback_joint_rate(
                            requested_q,
                            leg_vector(last_published_targets, "RL"),
                            args.max_feedback_joint_speed_deg_s,
                            args.dt,
                        )
                    rate_diagnostics = {
                        "active": limiter_active,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                    final = set_leg_vector(nominal, "RL", published_q)
                    feedback_release_pending["RL"] = (
                        np.max(np.abs(published_q - leg_vector(nominal, "RL")))
                        > 1e-12
                    )
                elif (
                    feedback_release_pending["RL"]
                    and args.max_feedback_joint_speed_deg_s is not None
                ):
                    requested_q = leg_vector(nominal, "RL")
                    (
                        published_q, limited_count,
                        requested_speed, published_speed,
                    ) = limit_feedback_joint_rate(
                        requested_q,
                        leg_vector(last_published_targets, "RL"),
                        args.max_feedback_joint_speed_deg_s,
                        args.dt,
                    )
                    final = set_leg_vector(nominal, "RL", published_q)
                    feedback_release_pending["RL"] = limited_count > 0
                    rate_diagnostics = {
                        "active": True,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }

                if left_world_beta > 0.0 or lateral_active or pitch_active:
                    left_solve_result = left_solver.solve(
                        next_frame, sim_time, base_rotation,
                        leg_vector(nominal, "LL"),
                        leg_vector(nominal, "RL"),
                        left_world_beta, 0.0,
                        chains["right"],
                        support_x_offset=lateral_offset,
                        support_pitch_correction=pitch_correction,
                    )
                    requested_q = left_solve_result["q"]
                    published_q = requested_q
                    limited_count = 0
                    requested_speed = 0.0
                    published_speed = 0.0
                    limiter_active = (
                        left_solve_result["success"]
                        and args.max_feedback_joint_speed_deg_s is not None
                    )
                    if left_solve_result["success"]:
                        (
                            published_q, limited_count,
                            requested_speed, published_speed,
                        ) = limit_feedback_joint_rate(
                            requested_q,
                            leg_vector(last_published_targets, "LL"),
                            args.max_feedback_joint_speed_deg_s,
                            args.dt,
                        )
                    left_rate_diagnostics = {
                        "active": limiter_active,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                    final = set_leg_vector(final, "LL", published_q)
                    feedback_release_pending["LL"] = (
                        np.max(np.abs(published_q - leg_vector(nominal, "LL")))
                        > 1e-12
                    )
                elif (
                    feedback_release_pending["LL"]
                    and args.max_feedback_joint_speed_deg_s is not None
                ):
                    requested_q = leg_vector(nominal, "LL")
                    (
                        published_q, limited_count,
                        requested_speed, published_speed,
                    ) = limit_feedback_joint_rate(
                        requested_q,
                        leg_vector(last_published_targets, "LL"),
                        args.max_feedback_joint_speed_deg_s,
                        args.dt,
                    )
                    final = set_leg_vector(final, "LL", published_q)
                    feedback_release_pending["LL"] = limited_count > 0
                    left_rate_diagnostics = {
                        "active": True,
                        "limited_joint_count": limited_count,
                        "requested_speed_deg_s": math.degrees(requested_speed),
                        "published_speed_deg_s": math.degrees(published_speed),
                    }
                if not all(math.isfinite(value) for value in final.values()):
                    raise RuntimeError("refusing to publish non-finite joint target")
                max_delta, max_velocity = safety.check(final, sim_time)
                publish_targets(publishers, final)
                last_published_targets = dict(final)
                write_log(
                    writer, publish_index, sim_time, trajectory_time,
                    next_frame, "REPLAY", final, base_rpy,
                    world_flat_active, swing_world_z_active,
                    solve_result, max_delta, max_velocity, rate_diagnostics,
                    left_solve_result, left_rate_diagnostics,
                    lateral_diagnostics,
                    {
                        "state": (
                            "DISABLED" if touchdown_hold is None
                            else touchdown_hold.state
                        ),
                        "detected": touchdown_detected_now,
                        "right_fz": right_fz,
                        "sim_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_sim_time
                        ),
                        "trajectory_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_trajectory_time
                        ),
                        "support_active": (
                            touchdown_hold is not None and touchdown_hold.active
                        ),
                        "support_result": support_result,
                        "left_handoff_beta": left_handoff_beta,
                        "left_offset_before": left_offset_before_handoff,
                        "left_offset_after": left_offset_after_handoff,
                    },
                    pitch_diagnostics,
                )
                stream.flush()
                if touchdown_detected_now:
                    print(
                        f"[TOUCHDOWN] sim={sim_time:.3f}s "
                        f"traj={trajectory_time:.3f}s Fz={right_fz:.3f} N"
                    )
                if touchdown_hold is not None:
                    touchdown_hold.finish_confirmation_frame()
                if next_frame % 10 == 0 or next_frame == replay_last_frame:
                    print(
                        f"[REPLAY] frame={next_frame:4d} traj={trajectory_time:.2f}s "
                        f"sim={sim_time:.3f}s "
                        f"world_flat={int(world_flat_active)} "
                        f"swing_world_z={int(swing_world_z_active)}"
                    )
                publish_index += 1
                next_frame += 1

            if next_frame > replay_last_frame and replay_finished_at is None:
                replay_finished_at = sim_time
                print(f"[REPLAY COMPLETE] holding frame {replay_last_frame}")
            if replay_finished_at is not None:
                if sim_time - replay_finished_at >= args.hold_after:
                    break
                trajectory_time = replay_last_frame * args.dt
                world_flat_beta = feedback_beta(
                    "HOLD_AFTER", trajectory_time,
                    args.world_flat_start, args.world_flat_end,
                    args.world_flat_ramp_in, args.world_flat_ramp_out,
                )
                swing_world_z_beta = feedback_beta(
                    "HOLD_AFTER", trajectory_time,
                    args.swing_world_z_start, args.swing_world_z_end,
                    args.swing_world_z_ramp_in, args.swing_world_z_ramp_out,
                )
                world_flat_active = world_flat_beta > 0.0
                swing_world_z_active = swing_world_z_beta > 0.0
                solve_result = None
                final = dict(last_published_targets)
                if not all(math.isfinite(value) for value in final.values()):
                    raise RuntimeError("refusing to publish non-finite joint target")
                max_delta, max_velocity = safety.check(final, sim_time)
                publish_targets(publishers, final)
                write_log(
                    writer, publish_index, sim_time, trajectory_time,
                    replay_last_frame, "HOLD_AFTER", final, base_rpy,
                    world_flat_active, swing_world_z_active,
                    solve_result, max_delta, max_velocity,
                    touchdown_diagnostics={
                        "state": (
                            "DISABLED" if touchdown_hold is None
                            else touchdown_hold.state
                        ),
                        "detected": False,
                        "right_fz": math.nan,
                        "sim_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_sim_time
                        ),
                        "trajectory_time": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.touchdown_trajectory_time
                        ),
                        "support_active": False,
                        "support_result": None,
                        "left_handoff_beta": 0.0,
                        "left_offset_before": (
                            math.nan if touchdown_hold is None
                            else touchdown_hold.left_offset_at_touchdown
                        ),
                        "left_offset_after": 0.0,
                    },
                )
                stream.flush()
                publish_index += 1

    print("[DONE]")
    print(f"command log={args.command_log}")


if __name__ == "__main__":
    main()
