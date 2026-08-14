#!/usr/bin/env python3
"""Replay STEP leg joint targets from CSV using Gazebo simulation time."""

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

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]

DEFAULT_CSV = (
    REPO_ROOT
    / "Dynamics"
    / "walk_forward_gazebo_candidateB_relative.csv"
)

DEFAULT_COMMAND_LOG = (
    SCRIPT_DIR.parent
    / "logs"
    / "walk_candidateB_first_step_commands.csv"
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

    parser.add_argument(
        "--csv",
        type=Path,
        default=DEFAULT_CSV,
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_DT,
    )
    parser.add_argument(
        "--hold-before",
        type=float,
        default=DEFAULT_HOLD_BEFORE,
    )
    parser.add_argument(
        "--replay-end",
        type=float,
        default=DEFAULT_REPLAY_END,
        help="trajectory time to replay through, in simulation seconds",
    )
    parser.add_argument(
        "--hold-after",
        type=float,
        default=DEFAULT_HOLD_AFTER,
    )
    parser.add_argument(
        "--odom-topic",
        default=DEFAULT_ODOM_TOPIC,
    )
    parser.add_argument(
        "--command-log",
        type=Path,
        default=DEFAULT_COMMAND_LOG,
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
    )

    args = parser.parse_args()

    if args.dt <= 0.0:
        parser.error("--dt must be positive")

    if args.hold_before < 0.0:
        parser.error("--hold-before must be non-negative")

    if args.replay_end < 0.0:
        parser.error("--replay-end must be non-negative")

    if args.hold_after < 0.0:
        parser.error("--hold-after must be non-negative")

    return args


class SimulationClock:
    def __init__(self, topic):
        self.condition = threading.Condition()
        self.simulation_time = None
        self.callback = self.on_odometry
        self.node = Node()

        subscribed = self.node.subscribe(
            odometry_pb2.Odometry,
            topic,
            self.callback,
        )

        if subscribed is False:
            raise RuntimeError(
                f"failed to subscribe to {topic}"
            )

    def on_odometry(self, message):
        stamp = message.header.stamp

        value = (
            float(stamp.sec)
            + float(stamp.nsec) * 1e-9
        )

        with self.condition:
            self.simulation_time = value
            self.condition.notify_all()

    def wait_for_time_after(
        self,
        previous_time,
        timeout=None,
    ):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: not (
                    self.simulation_time is None
                    or (
                        previous_time is not None
                        and self.simulation_time
                        <= previous_time
                    )
                ),
                timeout=timeout,
            )

            return self.simulation_time, ready


def load_targets(path):
    required = (
        [f"RL{i}" for i in range(6)]
        + [f"LL{i}" for i in range(6)]
    )

    with path.open(
        newline="",
        encoding="utf-8",
    ) as stream:
        rows = list(csv.DictReader(stream))

    if not rows:
        raise RuntimeError(
            f"CSV is empty: {path}"
        )

    for name in required:
        if name not in rows[0]:
            raise RuntimeError(
                f"missing CSV column: {name}"
            )

    result = []

    for index, row in enumerate(rows):
        targets = {}

        for side in ("RL", "LL"):
            for joint in range(6):
                csv_key = f"{side}{joint}"
                gazebo_key = f"{side}{joint}_wrap"

                value = float(row[csv_key])

                if not math.isfinite(value):
                    raise RuntimeError(
                        f"non-finite target at "
                        f"row {index}, {csv_key}"
                    )

                targets[gazebo_key] = value

        result.append(targets)

    return result


def command_tuples(targets):
    return [
        (
            key,
            topic,
            targets[key],
            targets[key],
        )
        for key, topic
        in LEG_JOINT_TOPICS.items()
    ]


def publish_targets(
    publishers,
    targets,
):
    publishers.publish(
        command_tuples(targets)
    )


def log_columns():
    return (
        ["publish_index",
         "simulation_time",
         "trajectory_time",
         "source_frame",
         "phase"]
        + [f"RL{i}" for i in range(6)]
        + [f"LL{i}" for i in range(6)]
    )


def write_log(
    writer,
    publish_index,
    simulation_time,
    trajectory_time,
    source_frame,
    phase,
    targets,
):
    row = {
        "publish_index": publish_index,
        "simulation_time": (
            f"{simulation_time:.9f}"
            if math.isfinite(simulation_time)
            else "nan"
        ),
        "trajectory_time": (
            f"{trajectory_time:.9f}"
        ),
        "source_frame": source_frame,
        "phase": phase,
    }

    for side in ("RL", "LL"):
        for joint in range(6):
            row[f"{side}{joint}"] = (
                f"{targets[f'{side}{joint}_wrap']:+.12f}"
            )

    writer.writerow(row)


def main():
    args = parse_args()

    targets = load_targets(args.csv)

    replay_last_frame = min(
        int(
            math.floor(
                args.replay_end / args.dt
                + 1e-12
            )
        ),
        len(targets) - 1,
    )

    args.command_log.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    publishers = GazeboDoublePublishers(
        list(LEG_JOINT_TOPICS.values()),
        args.dry_run,
    )

    clock = (
        None
        if args.dry_run
        else SimulationClock(args.odom_topic)
    )

    print(
        "[STEP WALK CSV REPLAY]"
    )
    print(
        f"CSV={args.csv}"
    )
    print(
        f"source rows={len(targets)}"
    )
    print(
        f"hold-before={args.hold_before:.3f} sim-s"
    )
    print(
        f"replay=0.000~"
        f"{replay_last_frame * args.dt:.3f} sim-s"
    )
    print(
        f"replay frames=0~{replay_last_frame}"
    )
    print(
        f"hold-after={args.hold_after:.3f} sim-s"
    )

    with args.command_log.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as stream:

        writer = csv.DictWriter(
            stream,
            fieldnames=log_columns(),
        )

        writer.writeheader()

        # Candidate B preload:
        # transformed CSV frame 0 is exactly Candidate B.
        publish_targets(
            publishers,
            targets[0],
        )

        write_log(
            writer,
            0,
            math.nan,
            0.0,
            0,
            "INITIAL_PRELOAD",
            targets[0],
        )

        stream.flush()

        if args.dry_run:
            publish_index = 1

            for frame in range(
                replay_last_frame + 1
            ):
                sim_t = (
                    args.hold_before
                    + frame * args.dt
                )

                publish_targets(
                    publishers,
                    targets[frame],
                )

                write_log(
                    writer,
                    publish_index,
                    sim_t,
                    frame * args.dt,
                    frame,
                    "REPLAY",
                    targets[frame],
                )

                publish_index += 1

            print(
                "[DRY RUN COMPLETE]"
            )
            print(
                f"last source frame="
                f"{replay_last_frame}"
            )

            return

        previous_sim_time = None

        # Wait until Gazebo simulation time exists.
        while True:
            sim_time, ready = (
                clock.wait_for_time_after(
                    previous_sim_time,
                    timeout=5.0,
                )
            )

            if ready and sim_time is not None:
                start_sim_time = sim_time
                break

            print(
                "[WAIT] no simulation-time update yet"
            )

        print(
            f"[START] sim_time="
            f"{start_sim_time:.9f}"
        )

        publish_index = 1
        next_frame = 0
        replay_started = False
        replay_finished_at = None

        while True:
            sim_time, ready = (
                clock.wait_for_time_after(
                    previous_sim_time,
                    timeout=5.0,
                )
            )

            if not ready:
                print(
                    "[WAIT] simulation time "
                    "not advancing"
                )
                continue

            previous_sim_time = sim_time

            elapsed = (
                sim_time - start_sim_time
            )

            if elapsed < args.hold_before:
                continue

            trajectory_elapsed = (
                elapsed - args.hold_before
            )

            while (
                next_frame
                <= replay_last_frame
                and trajectory_elapsed + 1e-9
                >= next_frame * args.dt
            ):
                publish_targets(
                    publishers,
                    targets[next_frame],
                )

                write_log(
                    writer,
                    publish_index,
                    sim_time,
                    next_frame * args.dt,
                    next_frame,
                    "REPLAY",
                    targets[next_frame],
                )

                stream.flush()

                if (
                    next_frame % 10 == 0
                    or next_frame
                    == replay_last_frame
                ):
                    print(
                        f"[REPLAY] "
                        f"frame={next_frame:4d} "
                        f"traj="
                        f"{next_frame * args.dt:.2f}s "
                        f"sim={sim_time:.3f}s"
                    )

                publish_index += 1
                next_frame += 1

            if (
                next_frame > replay_last_frame
                and replay_finished_at is None
            ):
                replay_finished_at = sim_time

                print(
                    "[REPLAY COMPLETE] "
                    f"holding frame "
                    f"{replay_last_frame}"
                )

            if (
                replay_finished_at is not None
                and sim_time - replay_finished_at
                >= args.hold_after
            ):
                break

    print(
        "[DONE]"
    )
    print(
        f"command log="
        f"{args.command_log}"
    )


if __name__ == "__main__":
    main()
