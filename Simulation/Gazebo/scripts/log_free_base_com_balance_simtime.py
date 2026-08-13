#!/usr/bin/env python3
"""Log STEP COM and support state on a Gazebo simulation-time schedule."""

import argparse
import csv
import math
import signal
import threading
import time
from pathlib import Path

from google.protobuf import symbol_database
from gz.msgs10 import (
    contact_pb2,
    contacts_pb2,
    entity_pb2,
    header_pb2,
    joint_wrench_pb2,
    odometry_pb2,
    pose_pb2,
    pose_v_pb2,
    quaternion_pb2,
    time_pb2,
    twist_pb2,
    vector3d_pb2,
    wrench_pb2,
)
from gz.transport13 import Node

from log_free_base_com_balance import (
    CSV_COLUMNS as LEGACY_CSV_COLUMNS,
    DEFAULT_LEFT_CONTACT_TOPIC,
    DEFAULT_MODEL_NAME,
    DEFAULT_MODEL_SDF,
    DEFAULT_POSE_TOPIC,
    DEFAULT_RIGHT_CONTACT_TOPIC,
    BalanceStateSubscriber,
    DebugMarkerPublisher,
    build_geometry_diagnostics,
    calculate_row,
    complete_row_with_runtime_fields,
    load_link_inertials,
    load_sole_boxes,
    print_initial_geometry,
    print_initial_summary,
)


GAZEBO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = GAZEBO_DIR / "logs/free_base_com_balance_simtime.csv"
DEFAULT_DURATION_SIM = 5.0
DEFAULT_SAMPLE_DT_SIM = 0.01
DEFAULT_ODOM_TOPIC = "/step/base_odometry"
CSV_COLUMNS = (
    "simulation_time",
    "simulation_elapsed",
    *LEGACY_CSV_COLUMNS,
)

_SYMBOLS = symbol_database.Default()
for _message_class in (
    time_pb2.Time,
    header_pb2.Header,
    header_pb2.Header.Map,
    vector3d_pb2.Vector3d,
    quaternion_pb2.Quaternion,
    pose_pb2.Pose,
    pose_v_pb2.Pose_V,
    entity_pb2.Entity,
    wrench_pb2.Wrench,
    joint_wrench_pb2.JointWrench,
    contact_pb2.Contact,
    contacts_pb2.Contacts,
    twist_pb2.Twist,
    odometry_pb2.Odometry,
):
    _SYMBOLS.RegisterMessage(_message_class)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sim", type=float, default=DEFAULT_DURATION_SIM)
    parser.add_argument("--sample-dt-sim", type=float, default=DEFAULT_SAMPLE_DT_SIM)
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--model-sdf", type=Path, default=DEFAULT_MODEL_SDF)
    parser.add_argument("--model-name", default=DEFAULT_MODEL_NAME)
    parser.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    parser.add_argument("--left-contact-topic", default=DEFAULT_LEFT_CONTACT_TOPIC)
    parser.add_argument("--right-contact-topic", default=DEFAULT_RIGHT_CONTACT_TOPIC)
    parser.add_argument("--contact-timeout", type=float, default=0.1)
    parser.add_argument("--angle-change-threshold", type=float, default=0.01)
    parser.add_argument("--publish-markers", action="store_true")
    args = parser.parse_args()
    positive_values = (
        ("--duration-sim", args.duration_sim),
        ("--sample-dt-sim", args.sample_dt_sim),
        ("--contact-timeout", args.contact_timeout),
        ("--angle-change-threshold", args.angle_change_threshold),
    )
    for option, value in positive_values:
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"{option} must be finite and greater than zero")
    return args


class OdometrySimulationClock:
    """Expose new /step/base_odometry header timestamps to the logger loop."""

    def __init__(self, topic):
        self.condition = threading.Condition()
        self.sequence = 0
        self.simulation_time = None
        self.callback = self.on_odometry
        self.node = Node()
        subscribed = self.node.subscribe(
            odometry_pb2.Odometry, topic, self.callback
        )
        if subscribed is False:
            raise RuntimeError(f"failed to subscribe to {topic}")

    def on_odometry(self, message):
        stamp = message.header.stamp
        simulation_time = float(stamp.sec) + float(stamp.nsec) * 1e-9
        with self.condition:
            self.sequence += 1
            self.simulation_time = simulation_time
            self.condition.notify_all()

    def wait_for_update(self, previous_sequence, stop, timeout=0.1):
        """Wait in wall time, but return only when a new sim stamp arrives."""
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence != previous_sequence or stop.is_set(),
                timeout=timeout,
            )
            return self.sequence, self.simulation_time


def main():
    args = parse_args()
    try:
        inertials = load_link_inertials(args.model_sdf)
        sole_boxes = load_sole_boxes(args.model_sdf)
        state = BalanceStateSubscriber(args)
        clock = OdometrySimulationClock(args.odom_topic)
        marker_publisher = (
            DebugMarkerPublisher() if args.publish_markers else None
        )
    except (ImportError, RuntimeError, ValueError) as error:
        print(f"[ERROR] {error}")
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop.set())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: stop.set())

    total_mass = sum(mass for mass, _ in inertials.values())
    print("[GAZEBO FREE-BASE COM / BALANCE LOGGER — SIM TIME]")
    print(f"model_sdf: {args.model_sdf}")
    print(f"model_name: {args.model_name}")
    print(f"links: {len(inertials)}, total_mass: {total_mass:.6f} kg")
    print("support_polygon: 8 sole-box corners -> world XY convex hull")
    print(f"pose_topic: {args.pose_topic}")
    print(f"odom_clock_topic: {args.odom_topic}")
    print(f"duration_sim: {args.duration_sim:.6f} s")
    print(f"sample_dt_sim: {args.sample_dt_sim:.6f} s")
    print(f"output: {args.output}")
    print("Waiting for the first odometry simulation timestamp.")

    rows_written = 0
    initial_summary_rows = []
    printed_initial_geometry = False
    next_progress_time = 0.5
    wall_start = time.monotonic()

    try:
        with args.output.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.writer(output_file)
            writer.writerow(CSV_COLUMNS)

            odom_sequence = 0
            sim_start = None
            pose_start_time = None
            next_sample_elapsed = 0.0
            previous_pose_sequence = -1

            while not stop.is_set():
                odom_sequence, simulation_time = clock.wait_for_update(
                    odom_sequence, stop
                )
                if simulation_time is None:
                    continue
                if sim_start is None:
                    sim_start = simulation_time
                    print(f"sim_start: {sim_start:.9f} s")

                simulation_elapsed = max(0.0, simulation_time - sim_start)
                if simulation_elapsed + 1e-12 < next_sample_elapsed:
                    continue

                pose_sequence, pose_time, link_poses, contacts = state.snapshot()
                if pose_time is None or not link_poses:
                    continue
                if pose_start_time is None:
                    pose_start_time = pose_time

                pose_updated = int(pose_sequence != previous_pose_sequence)
                previous_pose_sequence = pose_sequence
                try:
                    row = calculate_row(
                        simulation_elapsed,
                        pose_time,
                        link_poses,
                        contacts,
                        inertials,
                        sole_boxes,
                        args.model_name,
                        args.contact_timeout,
                    )
                except (RuntimeError, ValueError) as error:
                    print(f"[ERROR] {error}")
                    return 1

                source_sim_time = pose_time - pose_start_time
                legacy_complete_row = complete_row_with_runtime_fields(
                    row, source_sim_time, pose_updated
                )
                complete_row = (
                    simulation_time,
                    simulation_elapsed,
                    *legacy_complete_row,
                )
                writer.writerow(complete_row)
                output_file.flush()
                rows_written += 1

                legacy_dict = dict(zip(LEGACY_CSV_COLUMNS, legacy_complete_row))
                if simulation_elapsed <= 0.25 + 1e-9:
                    initial_summary_rows.append(legacy_dict)

                diagnostics = build_geometry_diagnostics(
                    link_poses,
                    inertials,
                    sole_boxes,
                    args.model_name,
                    (row[1], row[2]),
                )
                if not printed_initial_geometry:
                    print_initial_geometry(
                        diagnostics, sole_boxes, (row[1], row[2])
                    )
                    printed_initial_geometry = True
                if marker_publisher is not None:
                    marker_publisher.publish(diagnostics, (row[1], row[2]))

                while (
                    simulation_elapsed + 1e-12 >= next_progress_time
                    and next_progress_time <= args.duration_sim + 1e-12
                ):
                    print(
                        f"[SIM PROGRESS] t={next_progress_time:.2f} / "
                        f"{args.duration_sim:.2f} s"
                    )
                    next_progress_time += 0.5

                if simulation_elapsed + 1e-12 >= args.duration_sim:
                    break

                missed_intervals = math.floor(
                    (simulation_elapsed - next_sample_elapsed)
                    / args.sample_dt_sim
                )
                next_sample_elapsed += (
                    max(0, missed_intervals) + 1
                ) * args.sample_dt_sim
    except OSError as error:
        print(f"[ERROR] Could not write {args.output}: {error}")
        return 1
    except KeyboardInterrupt:
        stop.set()

    wall_elapsed = time.monotonic() - wall_start
    final_sim_elapsed = (
        0.0 if sim_start is None or clock.simulation_time is None
        else max(0.0, clock.simulation_time - sim_start)
    )
    print_initial_summary(initial_summary_rows, args.angle_change_threshold)
    print("[DONE]")
    print(f"simulation elapsed = {final_sim_elapsed:.9f} s")
    print(f"wall elapsed = {wall_elapsed:.9f} s")
    print(f"samples = {rows_written}")
    print(f"output = {args.output}")
    return 0 if rows_written else 1


if __name__ == "__main__":
    raise SystemExit(main())
