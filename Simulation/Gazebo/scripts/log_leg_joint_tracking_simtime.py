#!/usr/bin/env python3

import argparse
import csv
import math
import threading
from pathlib import Path

from gz.msgs10 import model_pb2, odometry_pb2
from gz.transport13 import Node


JOINTS = [
    ("RHY", "right_hip_yaw_joint",      0.000),
    ("RHR", "right_hip_roll_joint",    +0.100),
    ("RHP", "right_hip_pitch_joint",   -0.816),
    ("RKN", "right_knee_pitch_joint",  +0.138),
    ("RAP", "right_ankle_pitch_joint", -0.714),
    ("RAR", "right_ankle_roll_joint",  -0.100),

    ("LHY", "left_hip_yaw_joint",       0.000),
    ("LHR", "left_hip_roll_joint",     -0.100),
    ("LHP", "left_hip_pitch_joint",    +0.816),
    ("LKN", "left_knee_pitch_joint",   -0.138),
    ("LAP", "left_ankle_pitch_joint",  +0.714),
    ("LAR", "left_ankle_roll_joint",   +0.100),
]

NAME_TO_INFO = {
    name: (short, target)
    for short, name, target in JOINTS
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--duration-sim", type=float, default=2.5)
    p.add_argument("--sample-dt-sim", type=float, default=0.01)
    p.add_argument("--joint-topic", default="/step/leg_joint_states")
    p.add_argument("--odom-topic", default="/step/base_odometry")
    p.add_argument("--output", type=Path, required=True)
    return p.parse_args()


class State:
    def __init__(self):
        self.condition = threading.Condition()

        self.odom_seq = 0
        self.sim_time = None

        self.joint_seq = 0
        self.joints = {}

    def on_odom(self, msg):
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nsec) * 1e-9

        with self.condition:
            self.sim_time = t
            self.odom_seq += 1
            self.condition.notify_all()

    def on_joints(self, msg):
        data = {}

        for j in msg.joint:
            if j.name not in NAME_TO_INFO:
                continue

            data[j.name] = (
                float(j.axis1.position),
                float(j.axis1.velocity),
            )

        with self.condition:
            if data:
                self.joints = data
                self.joint_seq += 1
                self.condition.notify_all()


def main():
    args = parse_args()

    state = State()
    node = Node()

    ok1 = node.subscribe(
        odometry_pb2.Odometry,
        args.odom_topic,
        state.on_odom,
    )

    ok2 = node.subscribe(
        model_pb2.Model,
        args.joint_topic,
        state.on_joints,
    )

    if ok1 is False:
        raise RuntimeError(
            f"failed to subscribe: {args.odom_topic}"
        )

    if ok2 is False:
        raise RuntimeError(
            f"failed to subscribe: {args.joint_topic}"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)

    columns = [
        "simulation_time",
        "simulation_elapsed",
    ]

    for short, _, _ in JOINTS:
        columns += [
            f"{short}_position",
            f"{short}_velocity",
            f"{short}_error",
        ]

    print("[LEG JOINT TRACKING LOGGER]")
    print("joint_topic :", args.joint_topic)
    print("odom_topic  :", args.odom_topic)
    print("duration    :", args.duration_sim)
    print("sample_dt   :", args.sample_dt_sim)
    print("output      :", args.output)
    print("Waiting for simulation...")

    with args.output.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        previous_odom_seq = 0
        sim_start = None
        next_sample = 0.0
        rows_written = 0

        while True:
            with state.condition:
                state.condition.wait_for(
                    lambda: state.odom_seq != previous_odom_seq,
                    timeout=0.5,
                )

                if state.odom_seq == previous_odom_seq:
                    continue

                previous_odom_seq = state.odom_seq
                sim_time = state.sim_time
                joints = dict(state.joints)

            if sim_time is None:
                continue

            if sim_start is None:
                sim_start = sim_time
                print(f"sim_start = {sim_start:.9f}")

            elapsed = sim_time - sim_start

            if elapsed > args.duration_sim + 1e-9:
                break

            if elapsed + 1e-12 < next_sample:
                continue

            if len(joints) < len(JOINTS):
                continue

            row = [
                f"{sim_time:.9f}",
                f"{elapsed:.9f}",
            ]

            valid = True

            for short, name, target in JOINTS:
                if name not in joints:
                    valid = False
                    break

                pos, vel = joints[name]
                err = target - pos

                row += [
                    f"{pos:.12f}",
                    f"{vel:.12f}",
                    f"{err:.12f}",
                ]

            if not valid:
                continue

            writer.writerow(row)
            rows_written += 1

            while next_sample <= elapsed + 1e-12:
                next_sample += args.sample_dt_sim

    print(f"done: {rows_written} rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
