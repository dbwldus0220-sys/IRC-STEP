#!/usr/bin/env python3
"""Replay STEP leg CSV targets with minimal left-support roll feedback."""

import argparse
import csv
import math
import threading
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


DEFAULT_DT = 0.01
DEFAULT_HOLD_BEFORE = 1.0
DEFAULT_REPLAY_END = 1.30
DEFAULT_HOLD_AFTER = 1.0
DEFAULT_ODOM_TOPIC = "/step/base_odometry"
DEFAULT_SUPPORT_ROLL_START = 0.75
DEFAULT_SUPPORT_ROLL_KP = 0.30
DEFAULT_SUPPORT_ROLL_KD = 0.0
DEFAULT_SUPPORT_ROLL_RATE_ALPHA = 0.20
DEFAULT_SUPPORT_ROLL_MAX_CORRECTION = 0.03

LEFT_HIP_ROLL = "LL1_wrap"
LEFT_ANKLE_ROLL = "LL5_wrap"

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_CSV = (
    REPO_ROOT
    / "Dynamics"
    / "walk_forward_gazebo_candidateB_relative_rollfix_direction_scale200_swingroll100.csv"
)
DEFAULT_COMMAND_LOG = (
    SCRIPT_DIR.parent / "logs" / "walk_support_roll_feedback_commands.csv"
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
        "--support-roll-start", type=float, default=DEFAULT_SUPPORT_ROLL_START,
        help="trajectory time at which roll_ref is captured, in seconds",
    )
    parser.add_argument(
        "--support-roll-kp", type=float, default=DEFAULT_SUPPORT_ROLL_KP,
    )
    parser.add_argument(
        "--support-roll-kd", type=float, default=DEFAULT_SUPPORT_ROLL_KD,
        help="base-roll derivative gain, in seconds",
    )
    parser.add_argument(
        "--support-roll-rate-alpha", type=float,
        default=DEFAULT_SUPPORT_ROLL_RATE_ALPHA,
        help="roll-rate low-pass coefficient in (0, 1]",
    )
    parser.add_argument(
        "--support-roll-max-correction", type=float,
        default=DEFAULT_SUPPORT_ROLL_MAX_CORRECTION, metavar="RAD",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    numeric = (
        args.dt, args.hold_before, args.replay_end, args.hold_after,
        args.support_roll_start, args.support_roll_kp, args.support_roll_kd,
        args.support_roll_rate_alpha,
        args.support_roll_max_correction,
    )
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all numeric arguments must be finite")
    if args.dt <= 0.0:
        parser.error("--dt must be positive")
    if args.hold_before < 0.0:
        parser.error("--hold-before must be non-negative")
    if args.replay_end < 0.0:
        parser.error("--replay-end must be non-negative")
    if args.hold_after < 0.0:
        parser.error("--hold-after must be non-negative")
    if args.support_roll_start < 0.0:
        parser.error("--support-roll-start must be non-negative")
    if args.support_roll_kp < 0.0:
        parser.error("--support-roll-kp must be non-negative")
    if args.support_roll_kd < 0.0:
        parser.error("--support-roll-kd must be non-negative")
    if not 0.0 < args.support_roll_rate_alpha <= 1.0:
        parser.error("--support-roll-rate-alpha must be in (0, 1]")
    if args.support_roll_max_correction <= 0.0:
        parser.error("--support-roll-max-correction must be positive")
    return args


def quaternion_roll(quaternion):
    """Return roll from an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return math.nan
    x, y, z, w = (value / norm for value in quaternion)
    return math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )


class SimulationClock:
    def __init__(self, topic, roll_rate_alpha):
        self.condition = threading.Condition()
        self.simulation_time = None
        self.base_roll = math.nan
        self.roll_rate_alpha = roll_rate_alpha
        self.previous_rate_time = None
        self.previous_rate_roll = math.nan
        self.base_roll_rate_raw = math.nan
        self.base_roll_rate_filtered = 0.0
        self.rate_filter_initialized = False
        self.callback = self.on_odometry
        self.node = Node()
        if self.node.subscribe(odometry_pb2.Odometry, topic, self.callback) is False:
            raise RuntimeError(f"failed to subscribe to {topic}")

    def on_odometry(self, message):
        stamp = message.header.stamp
        orientation = message.pose.orientation
        simulation_time = float(stamp.sec) + float(stamp.nsec) * 1e-9
        base_roll = quaternion_roll((
            orientation.x, orientation.y, orientation.z, orientation.w,
        ))
        with self.condition:
            if (
                self.previous_rate_time is not None
                and math.isfinite(self.previous_rate_roll)
                and math.isfinite(base_roll)
            ):
                dt = simulation_time - self.previous_rate_time
                if dt > 0.0:
                    delta_roll = math.atan2(
                        math.sin(base_roll - self.previous_rate_roll),
                        math.cos(base_roll - self.previous_rate_roll),
                    )
                    self.base_roll_rate_raw = delta_roll / dt
                    if self.rate_filter_initialized:
                        alpha = self.roll_rate_alpha
                        self.base_roll_rate_filtered = (
                            alpha * self.base_roll_rate_raw
                            + (1.0 - alpha) * self.base_roll_rate_filtered
                        )
                    else:
                        self.base_roll_rate_filtered = self.base_roll_rate_raw
                        self.rate_filter_initialized = True
            if (
                self.previous_rate_time is None
                or simulation_time > self.previous_rate_time
            ):
                self.previous_rate_time = simulation_time
                self.previous_rate_roll = base_roll
            self.simulation_time = simulation_time
            self.base_roll = base_roll
            self.condition.notify_all()

    def reset_roll_rate(self):
        """Start derivative history at the latest feedback-start sample."""
        with self.condition:
            self.previous_rate_time = self.simulation_time
            self.previous_rate_roll = self.base_roll
            self.base_roll_rate_raw = 0.0
            self.base_roll_rate_filtered = 0.0
            self.rate_filter_initialized = False

    def wait_for_time_after(self, previous_time, timeout=None):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: not (
                    self.simulation_time is None
                    or (previous_time is not None and self.simulation_time <= previous_time)
                ),
                timeout=timeout,
            )
            return (
                self.simulation_time,
                self.base_roll,
                self.base_roll_rate_raw,
                self.base_roll_rate_filtered,
                ready,
            )


def load_targets(path):
    required = [f"RL{i}" for i in range(6)] + [f"LL{i}" for i in range(6)]
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"CSV is empty: {path}")
    for name in required:
        if name not in rows[0]:
            raise RuntimeError(f"missing CSV column: {name}")

    result = []
    for index, row in enumerate(rows):
        targets = {}
        for side in ("RL", "LL"):
            for joint in range(6):
                csv_key = f"{side}{joint}"
                value = float(row[csv_key])
                if not math.isfinite(value):
                    raise RuntimeError(f"non-finite target at row {index}, {csv_key}")
                targets[f"{csv_key}_wrap"] = value
        result.append(targets)
    return result


def support_roll_correction(roll_ref, roll_actual, roll_rate, kp, kd, limit):
    roll_error = roll_ref - roll_actual
    # Example: ref=-4.5 deg and actual=-8.0 deg gives error=+3.5 deg.
    # The minus sign makes correction negative, so LL1 decreases and LL5 increases.
    p_term = -kp * roll_error
    # Negative roll rate must also decrease LL1 and increase LL5.
    d_term = kd * roll_rate
    correction = max(-limit, min(limit, p_term + d_term))
    return roll_error, p_term, d_term, correction


def apply_support_roll_feedback(csv_targets, correction):
    targets = dict(csv_targets)
    # If actual roll is more negative than ref, correction is negative:
    # LL1_final < LL1_csv and LL5_final > LL5_csv.
    targets[LEFT_HIP_ROLL] += correction
    targets[LEFT_ANKLE_ROLL] -= correction
    return targets


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
            "base_roll", "roll_ref", "roll_error", "support_roll_correction",
            "base_roll_rate_raw", "base_roll_rate_filtered",
            "support_roll_p_term", "support_roll_d_term",
            "LL1_csv_target", "LL1_final_target", "LL5_csv_target",
            "LL5_final_target", "feedback_active",
        ]
    )


def format_float(value):
    return f"{value:+.12f}" if math.isfinite(value) else "nan"


def write_log(writer, publish_index, simulation_time, trajectory_time,
              source_frame, phase, csv_targets, final_targets, base_roll,
              roll_ref, roll_error, correction, feedback_active,
              roll_rate_raw, roll_rate_filtered, p_term, d_term):
    row = {
        "publish_index": publish_index,
        "simulation_time": (
            f"{simulation_time:.9f}" if math.isfinite(simulation_time) else "nan"
        ),
        "trajectory_time": f"{trajectory_time:.9f}",
        "source_frame": source_frame,
        "phase": phase,
        "base_roll": format_float(base_roll),
        "roll_ref": format_float(roll_ref),
        "roll_error": format_float(roll_error),
        "support_roll_correction": format_float(correction),
        "base_roll_rate_raw": format_float(roll_rate_raw),
        "base_roll_rate_filtered": format_float(roll_rate_filtered),
        "support_roll_p_term": format_float(p_term),
        "support_roll_d_term": format_float(d_term),
        "LL1_csv_target": format_float(csv_targets[LEFT_HIP_ROLL]),
        "LL1_final_target": format_float(final_targets[LEFT_HIP_ROLL]),
        "LL5_csv_target": format_float(csv_targets[LEFT_ANKLE_ROLL]),
        "LL5_final_target": format_float(final_targets[LEFT_ANKLE_ROLL]),
        "feedback_active": int(feedback_active),
    }
    for side in ("RL", "LL"):
        for joint in range(6):
            row[f"{side}{joint}"] = format_float(final_targets[f"{side}{joint}_wrap"])
    writer.writerow(row)


def feedback_targets(csv_targets, trajectory_time, base_roll, roll_rate,
                     roll_ref, args):
    active = trajectory_time >= args.support_roll_start and math.isfinite(base_roll)
    if active and roll_ref is None:
        roll_ref = base_roll
    if active:
        error, p_term, d_term, correction = support_roll_correction(
            roll_ref, base_roll, roll_rate, args.support_roll_kp,
            args.support_roll_kd,
            args.support_roll_max_correction,
        )
        final_targets = apply_support_roll_feedback(csv_targets, correction)
    else:
        error, p_term, d_term, correction = math.nan, 0.0, 0.0, 0.0
        final_targets = dict(csv_targets)
    return final_targets, roll_ref, error, p_term, d_term, correction, active


def main():
    args = parse_args()
    targets = load_targets(args.csv)
    replay_last_frame = min(
        int(math.floor(args.replay_end / args.dt + 1e-12)), len(targets) - 1,
    )

    print("[STEP WALK CSV SUPPORT-ROLL FEEDBACK]")
    print(f"CSV={args.csv}")
    print(f"source rows={len(targets)}")
    print(f"hold-before={args.hold_before:.3f} sim-s")
    print(f"replay=0.000~{replay_last_frame * args.dt:.3f} sim-s")
    print(f"replay frames=0~{replay_last_frame}")
    print(f"hold-after={args.hold_after:.3f} sim-s")
    print(f"support-roll-start={args.support_roll_start:.3f} sim-s")
    print(f"support-roll-kp={args.support_roll_kp:.6f}")
    print(f"support-roll-kd={args.support_roll_kd:.6f} s")
    print(f"support-roll-rate-alpha={args.support_roll_rate_alpha:.6f}")
    print(
        "support-roll-max-correction="
        f"{args.support_roll_max_correction:.6f} rad"
    )
    if args.dry_run:
        print("support-roll feedback requires live odometry")
        args.command_log.parent.mkdir(parents=True, exist_ok=True)
        publishers = GazeboDoublePublishers(
            list(LEG_JOINT_TOPICS.values()), True,
        )
        with args.command_log.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=log_columns())
            writer.writeheader()
            publish_targets(publishers, targets[0])
            write_log(
                writer, 0, math.nan, 0.0, 0, "INITIAL_PRELOAD",
                targets[0], targets[0], math.nan, math.nan, math.nan,
                0.0, False, math.nan, math.nan, 0.0, 0.0,
            )
            for frame in range(replay_last_frame + 1):
                trajectory_time = frame * args.dt
                simulation_time = args.hold_before + trajectory_time
                publish_targets(publishers, targets[frame])
                write_log(
                    writer, frame + 1, simulation_time, trajectory_time,
                    frame, "REPLAY", targets[frame], targets[frame],
                    math.nan, math.nan, math.nan, 0.0, False,
                    math.nan, math.nan, 0.0, 0.0,
                )
        print("[DRY RUN COMPLETE]")
        print(f"last source frame={replay_last_frame}")
        return

    args.command_log.parent.mkdir(parents=True, exist_ok=True)
    publishers = GazeboDoublePublishers(list(LEG_JOINT_TOPICS.values()), False)
    clock = SimulationClock(args.odom_topic, args.support_roll_rate_alpha)

    with args.command_log.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=log_columns())
        writer.writeheader()

        # Preload frame 0 while Gazebo may still be paused; feedback starts in replay.
        publish_targets(publishers, targets[0])
        write_log(
            writer, 0, math.nan, 0.0, 0, "INITIAL_PRELOAD", targets[0],
            targets[0], math.nan, math.nan, math.nan, 0.0, False,
            math.nan, math.nan, 0.0, 0.0,
        )
        stream.flush()

        previous_sim_time = None
        while True:
            sim_time, base_roll, roll_rate_raw, roll_rate_filtered, ready = (
                clock.wait_for_time_after(
                previous_sim_time, timeout=5.0,
                )
            )
            if ready and sim_time is not None:
                start_sim_time = sim_time
                break
            print("[WAIT] no simulation-time update yet")

        print(f"[START] sim_time={start_sim_time:.9f}")
        publish_index = 1
        next_frame = 0
        replay_finished_at = None
        roll_ref = None

        while True:
            sim_time, base_roll, roll_rate_raw, roll_rate_filtered, ready = (
                clock.wait_for_time_after(
                previous_sim_time, timeout=5.0,
                )
            )
            if not ready:
                print("[WAIT] simulation time not advancing")
                continue
            previous_sim_time = sim_time
            elapsed = sim_time - start_sim_time
            if elapsed < args.hold_before:
                continue
            trajectory_elapsed = elapsed - args.hold_before

            while (
                next_frame <= replay_last_frame
                and trajectory_elapsed + 1e-9 >= next_frame * args.dt
            ):
                trajectory_time = next_frame * args.dt
                if (
                    roll_ref is None
                    and trajectory_time >= args.support_roll_start
                    and math.isfinite(base_roll)
                ):
                    # Capture ref and discard pre-feedback derivative history.
                    clock.reset_roll_rate()
                    roll_rate_raw = 0.0
                    roll_rate_filtered = 0.0
                (
                    final, roll_ref, error, p_term, d_term, correction, active,
                ) = feedback_targets(
                    targets[next_frame], trajectory_time, base_roll,
                    roll_rate_filtered, roll_ref, args,
                )
                publish_targets(publishers, final)
                write_log(
                    writer, publish_index, sim_time, trajectory_time, next_frame,
                    "REPLAY", targets[next_frame], final, base_roll,
                    roll_ref if roll_ref is not None else math.nan,
                    error, correction, active, roll_rate_raw,
                    roll_rate_filtered, p_term, d_term,
                )
                stream.flush()
                if next_frame % 10 == 0 or next_frame == replay_last_frame:
                    print(
                        f"[REPLAY] frame={next_frame:4d} traj={trajectory_time:.2f}s "
                        f"sim={sim_time:.3f}s"
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
                (
                    final, roll_ref, error, p_term, d_term, correction, active,
                ) = feedback_targets(
                    targets[replay_last_frame], trajectory_time, base_roll,
                    roll_rate_filtered, roll_ref, args,
                )
                publish_targets(publishers, final)
                write_log(
                    writer, publish_index, sim_time, trajectory_time,
                    replay_last_frame, "HOLD_AFTER", targets[replay_last_frame],
                    final, base_roll, roll_ref if roll_ref is not None else math.nan,
                    error, correction, active, roll_rate_raw,
                    roll_rate_filtered, p_term, d_term,
                )
                stream.flush()
                publish_index += 1

    print("[DONE]")
    print(f"command log={args.command_log}")


if __name__ == "__main__":
    main()
