#!/usr/bin/env python3
"""Generate Gazebo leg commands by solving 6D IK on the SDF kinematics."""

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_SOURCE = REPO_ROOT / "Dynamics" / (
    "walk_forward_debug_slow_y_scale002_continuation5_baseline_jumpguard_"
    "th005_lpf_a050_pitchlpf_a080_long.csv"
)
DEFAULT_SDF = SCRIPT_DIR.parent / "models" / "step_urdf" / (
    "step_free_base_constant_pose_candidate_B_initial_sole_aligned_box_only_"
    "D0_diag_test.sdf"
)
DEFAULT_DT = 0.01
VELOCITY_WARNING_RAD_S = 4.71238898

JOINT_NAMES = {
    "right": [
        "right_hip_yaw_joint", "right_hip_roll_joint",
        "right_hip_pitch_joint", "right_knee_pitch_joint",
        "right_ankle_pitch_joint", "right_ankle_roll_joint",
    ],
    "left": [
        "left_hip_yaw_joint", "left_hip_roll_joint",
        "left_hip_pitch_joint", "left_knee_pitch_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint",
    ],
}
COLLISION_NAMES = {
    "right": "right_sole_box_collision",
    "left": "left_sole_box_collision",
}
CANDIDATE_B = {
    "right": np.array([0.0, 0.100, -0.816, 0.138, -0.714, -0.100]),
    "left": np.array([0.0, -0.100, 0.816, -0.138, 0.714, 0.100]),
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    parser.add_argument(
        "--target-source", choices=("ref", "ik-target"), required=True,
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--diagnostics-output", type=Path)
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    args = parser.parse_args()
    if args.max_frames is not None and args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and positive")
    suffix = "ref" if args.target_source == "ref" else "iktarget"
    if args.output is None:
        args.output = REPO_ROOT / "Dynamics" / (
            f"walk_forward_gazebo_sdf6dik_candidateB_{suffix}.csv"
        )
    if args.diagnostics_output is None:
        args.diagnostics_output = REPO_ROOT / "Dynamics" / (
            f"walk_forward_gazebo_sdf6dik_candidateB_{suffix}_diagnostics.csv"
        )
    for path in (args.output, args.diagnostics_output):
        if path.exists():
            parser.error(f"refusing to overwrite existing file: {path}")
    if args.output.resolve() == args.diagnostics_output.resolve():
        parser.error("--output and --diagnostics-output must differ")
    return args


def pose_transform(text):
    values = np.fromstring(text.strip() if text else "0 0 0 0 0 0", sep=" ")
    if values.size != 6 or not np.all(np.isfinite(values)):
        raise ValueError(f"invalid SDF pose: {text!r}")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", values[3:]).as_matrix()
    transform[:3, 3] = values[:3]
    return transform


def axis_rotation(axis, angle):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
    return transform


@dataclass(frozen=True)
class JointKinematics:
    name: str
    parent: str
    child: str
    parent_to_joint: np.ndarray
    joint_to_child: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float


@dataclass(frozen=True)
class LegChain:
    side: str
    joints: tuple
    child_to_sole: np.ndarray

    @property
    def lower(self):
        return np.array([joint.lower for joint in self.joints])

    @property
    def upper(self):
        return np.array([joint.upper for joint in self.joints])

    def forward(self, q):
        transform = np.eye(4)
        for joint, angle in zip(self.joints, q):
            transform = (
                transform
                @ joint.parent_to_joint
                @ axis_rotation(joint.axis, angle)
                @ joint.joint_to_child
            )
        return transform @ self.child_to_sole


class SdfFrames:
    def __init__(self, model):
        self.model = model
        self.elements = {}
        self.parents = {}
        self.local = {}
        self.resolved = {"__model__": np.eye(4)}

        for link in model.findall("link"):
            name = link.get("name")
            self._add(name, link.find("pose"), "__model__")
        for joint in model.findall("joint"):
            name = joint.get("name")
            pose = joint.find("pose")
            if pose is None or not pose.get("relative_to"):
                raise ValueError(f"joint {name} must have an explicit pose relative_to")
            self._add(name, pose, None)

    def _add(self, name, pose, default_parent):
        if not name or name in self.elements:
            raise ValueError(f"missing or duplicate SDF frame name: {name!r}")
        relative_to = pose.get("relative_to") if pose is not None else None
        self.elements[name] = pose
        self.parents[name] = relative_to or default_parent
        self.local[name] = pose_transform(pose.text if pose is not None else None)

    def resolve(self, name, active=None):
        if name in self.resolved:
            return self.resolved[name]
        if name not in self.elements:
            raise ValueError(f"unknown SDF relative_to frame: {name}")
        active = set() if active is None else active
        if name in active:
            raise ValueError(f"cycle in SDF relative_to graph at {name}")
        active.add(name)
        parent = self.parents[name]
        if parent is None:
            raise ValueError(f"frame {name} has no resolvable relative_to parent")
        result = self.resolve(parent, active) @ self.local[name]
        active.remove(name)
        self.resolved[name] = result
        return result


def find_collision(model, collision_name):
    matches = []
    for link in model.findall("link"):
        for collision in link.findall("collision"):
            if collision.get("name") == collision_name:
                matches.append((link, collision))
    if len(matches) != 1:
        raise ValueError(
            f"expected one collision named {collision_name}, found {len(matches)}"
        )
    return matches[0]


def load_leg_chains(sdf_path):
    root = ET.parse(sdf_path).getroot()
    model = root.find("model") or root.find(".//model")
    if model is None:
        raise ValueError(f"no model in SDF: {sdf_path}")
    frames = SdfFrames(model)
    joints_by_name = {joint.get("name"): joint for joint in model.findall("joint")}
    chains = {}

    for side in ("right", "left"):
        parsed = []
        expected_parent = "base_link"
        for name in JOINT_NAMES[side]:
            element = joints_by_name.get(name)
            if element is None:
                raise ValueError(f"missing SDF joint: {name}")
            parent = (element.findtext("parent") or "").strip()
            child = (element.findtext("child") or "").strip()
            if parent != expected_parent:
                raise ValueError(
                    f"{name} parent is {parent}, expected chain parent {expected_parent}"
                )
            axis_element = element.find("axis")
            if axis_element is None:
                raise ValueError(f"missing axis for {name}")
            xyz_element = axis_element.find("xyz")
            if xyz_element is None:
                raise ValueError(f"missing axis/xyz for {name}")
            if xyz_element.get("expressed_in"):
                raise ValueError(f"unsupported axis expressed_in on {name}")
            axis = np.fromstring(xyz_element.text, sep=" ")
            norm = np.linalg.norm(axis)
            if axis.size != 3 or not np.isfinite(norm) or norm <= 0.0:
                raise ValueError(f"invalid axis for {name}")
            axis = axis / norm
            limit = axis_element.find("limit")
            if limit is None:
                raise ValueError(f"missing axis limit for {name}")
            lower = float(limit.findtext("lower"))
            upper = float(limit.findtext("upper"))
            velocity = float(limit.findtext("velocity"))
            if not all(math.isfinite(v) for v in (lower, upper, velocity)):
                raise ValueError(f"non-finite axis limit for {name}")

            world_parent = frames.resolve(parent)
            world_joint = frames.resolve(name)
            world_child = frames.resolve(child)
            parsed.append(JointKinematics(
                name=name,
                parent=parent,
                child=child,
                parent_to_joint=np.linalg.inv(world_parent) @ world_joint,
                joint_to_child=np.linalg.inv(world_joint) @ world_child,
                axis=axis,
                lower=lower,
                upper=upper,
                velocity=velocity,
            ))
            expected_parent = child

        sole_link, collision = find_collision(model, COLLISION_NAMES[side])
        if sole_link.get("name") != expected_parent:
            raise ValueError(
                f"{COLLISION_NAMES[side]} is on {sole_link.get('name')}, "
                f"expected {expected_parent}"
            )
        collision_pose = collision.find("pose")
        relative_to = (
            collision_pose.get("relative_to") if collision_pose is not None else None
        )
        if relative_to and relative_to != expected_parent:
            world_collision = frames.resolve(relative_to) @ pose_transform(
                collision_pose.text
            )
            child_to_sole = np.linalg.inv(frames.resolve(expected_parent)) @ world_collision
        else:
            child_to_sole = pose_transform(
                collision_pose.text if collision_pose is not None else None
            )
        chains[side] = LegChain(side, tuple(parsed), child_to_sole)
    return chains


def load_source(path, target_source, max_frames):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if max_frames is not None:
        rows = rows[:max_frames]
    if not rows:
        raise ValueError(f"source CSV has no selected rows: {path}")
    prefix = "Ref_" if target_source == "ref" else ""
    suffix = "" if target_source == "ref" else "_target"
    unit_scale = 1.0 if target_source == "ref" else 1e-3
    positions = {}
    for side, code in (("right", "RL"), ("left", "LL")):
        if target_source == "ref":
            columns = [f"Ref_{code}_{axis}" for axis in "xyz"]
        else:
            columns = [f"{code}_target_{axis}" for axis in "xyz"]
        missing = [column for column in columns if column not in rows[0]]
        if missing:
            raise ValueError(f"source CSV missing columns: {', '.join(missing)}")
        values = np.array([
            [float(row[column]) * unit_scale for column in columns]
            for row in rows
        ])
        if not np.all(np.isfinite(values)):
            raise ValueError(f"non-finite {target_source} positions for {side} leg")
        positions[side] = values
    return rows, positions


def mapped_delta(source_positions):
    delta = source_positions - source_positions[0]
    # STEP lateral is Gazebo +X; STEP forward is Gazebo -Y.
    return np.column_stack((delta[:, 1], -delta[:, 0], delta[:, 2]))


def pose_errors(actual, target):
    position_error = actual[:3, 3] - target[:3, 3]
    orientation_error = Rotation.from_matrix(
        target[:3, :3].T @ actual[:3, :3]
    ).as_rotvec()
    return position_error, orientation_error


def solve_leg_frame(chain, target, previous_q):
    def residual(q):
        actual = chain.forward(q)
        position_error, orientation_error = pose_errors(actual, target)
        return np.concatenate((
            1000.0 * position_error,
            orientation_error,
            1e-7 * (q - previous_q),
        ))

    return least_squares(
        residual,
        previous_q,
        bounds=(chain.lower, chain.upper),
        xtol=1e-13,
        ftol=1e-13,
        gtol=1e-13,
        max_nfev=300,
    )


def solve_trajectory(chain, position_delta, dt):
    seed = CANDIDATE_B[chain.side].copy()
    if np.any(seed < chain.lower) or np.any(seed > chain.upper):
        raise ValueError(f"Candidate B violates {chain.side} SDF joint limits")
    initial_pose = chain.forward(seed)
    solutions = []
    diagnostics = []
    previous = seed.copy()

    for frame, delta in enumerate(position_delta):
        target = initial_pose.copy()
        target[:3, 3] = initial_pose[:3, 3] + delta
        if frame == 0:
            solution = seed.copy()
            success = True
            status = 0
            nfev = 0
        else:
            result = solve_leg_frame(chain, target, previous)
            solution = result.x
            success = bool(result.success)
            status = int(result.status)
            nfev = int(result.nfev)
        actual = chain.forward(solution)
        position_error, orientation_error = pose_errors(actual, target)
        frame_delta = solution - previous if frame > 0 else np.zeros(6)
        diagnostics.append({
            "success": success,
            "status": status,
            "nfev": nfev,
            "pos_err_mm": 1000.0 * np.linalg.norm(position_error),
            "ori_err_deg": math.degrees(np.linalg.norm(orientation_error)),
            "max_delta_deg": math.degrees(np.max(np.abs(solution - seed))),
            "max_frame_delta_deg": math.degrees(np.max(np.abs(frame_delta))),
            "max_velocity_deg_s": math.degrees(np.max(np.abs(frame_delta)) / dt),
            "target_delta": delta.copy(),
            "solved_delta": actual[:3, 3] - initial_pose[:3, 3],
            "limit_violation": bool(
                np.any(solution < chain.lower - 1e-10)
                or np.any(solution > chain.upper + 1e-10)
            ),
        })
        solutions.append(solution)
        previous = solution
    return np.array(solutions), diagnostics, initial_pose


def write_outputs(args, source_rows, solutions, diagnostics):
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.diagnostics_output.parent.mkdir(parents=True, exist_ok=True)
    command_fields = ["frame", "time"] + [
        f"{side}{joint}" for side in ("RL", "LL") for joint in range(6)
    ]
    with args.output.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=command_fields)
        writer.writeheader()
        for index in range(len(source_rows)):
            row = {
                "frame": source_rows[index].get("frame", index),
                "time": f"{index * args.dt:.9f}",
            }
            for prefix, side in (("RL", "right"), ("LL", "left")):
                for joint, value in enumerate(solutions[side][index]):
                    row[f"{prefix}{joint}"] = f"{value:+.12f}"
            writer.writerow(row)

    diagnostic_fields = [
        "frame",
        "RL_pos_err_mm", "RL_ori_err_deg", "LL_pos_err_mm", "LL_ori_err_deg",
        "RL_max_delta_deg", "LL_max_delta_deg",
        "RL_max_frame_delta_deg", "LL_max_frame_delta_deg",
        "RL_max_velocity_deg_s", "LL_max_velocity_deg_s",
        "RL_solver_success", "LL_solver_success",
        "RL_solver_status", "LL_solver_status", "RL_nfev", "LL_nfev",
        "RL_limit_violation", "LL_limit_violation",
    ]
    with args.diagnostics_output.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=diagnostic_fields)
        writer.writeheader()
        for index in range(len(source_rows)):
            right = diagnostics["right"][index]
            left = diagnostics["left"][index]
            writer.writerow({
                "frame": source_rows[index].get("frame", index),
                "RL_pos_err_mm": f"{right['pos_err_mm']:.12g}",
                "RL_ori_err_deg": f"{right['ori_err_deg']:.12g}",
                "LL_pos_err_mm": f"{left['pos_err_mm']:.12g}",
                "LL_ori_err_deg": f"{left['ori_err_deg']:.12g}",
                "RL_max_delta_deg": f"{right['max_delta_deg']:.12g}",
                "LL_max_delta_deg": f"{left['max_delta_deg']:.12g}",
                "RL_max_frame_delta_deg": f"{right['max_frame_delta_deg']:.12g}",
                "LL_max_frame_delta_deg": f"{left['max_frame_delta_deg']:.12g}",
                "RL_max_velocity_deg_s": f"{right['max_velocity_deg_s']:.12g}",
                "LL_max_velocity_deg_s": f"{left['max_velocity_deg_s']:.12g}",
                "RL_solver_success": int(right["success"]),
                "LL_solver_success": int(left["success"]),
                "RL_solver_status": right["status"],
                "LL_solver_status": left["status"],
                "RL_nfev": right["nfev"],
                "LL_nfev": left["nfev"],
                "RL_limit_violation": int(right["limit_violation"]),
                "LL_limit_violation": int(left["limit_violation"]),
            })


def vector_mm(vector):
    return "[" + ", ".join(f"{1000.0 * value:+.6f}" for value in vector) + "] mm"


def print_summary(args, chains, solutions, diagnostics):
    frame_count = len(solutions["right"])
    print(f"[SDF 6D IK] target-source={args.target_source} frames={frame_count}")
    for side, label in (("right", "right"), ("left", "left")):
        data = diagnostics[side]
        pos = np.array([row["pos_err_mm"] for row in data])
        ori = np.array([row["ori_err_deg"] for row in data])
        step = np.array([row["max_frame_delta_deg"] for row in data])
        velocity = np.array([row["max_velocity_deg_s"] for row in data])
        successes = sum(row["success"] for row in data)
        print(f"{label} solver success={successes}/{frame_count}")
        print(
            f"{label} position RMS/MAX={math.sqrt(np.mean(pos ** 2)):.9f}/"
            f"{np.max(pos):.9f} mm (max frame {int(np.argmax(pos))})"
        )
        print(
            f"{label} orientation RMS/MAX={math.sqrt(np.mean(ori ** 2)):.9f}/"
            f"{np.max(ori):.9f} deg (max frame {int(np.argmax(ori))})"
        )
        print(
            f"{label} max delta={np.max(step):.6f} deg/frame, "
            f"{np.max(velocity):.6f} deg/s (frame {int(np.argmax(velocity))})"
        )
        failed = [index for index, row in enumerate(data) if not row["success"]]
        violations = [index for index, row in enumerate(data) if row["limit_violation"]]
        over_velocity = [
            index for index, row in enumerate(data)
            if math.radians(row["max_velocity_deg_s"]) > VELOCITY_WARNING_RAD_S
        ]
        print(f"{label} solver failed frames={failed or 'none'}")
        print(f"{label} joint-limit violations={violations or 'none'}")
        if over_velocity:
            print(f"[WARNING] {label} >270 deg/s frames={over_velocity}")

    exact = all(np.array_equal(solutions[side][0], CANDIDATE_B[side])
                for side in ("right", "left"))
    print(f"frame 0 Candidate B exact={'YES' if exact else 'NO'}")
    report_frame = min(90, frame_count - 1)
    right = diagnostics["right"][report_frame]
    print(
        f"frame {report_frame} right target dXYZ={vector_mm(right['target_delta'])} "
        f"solved dXYZ={vector_mm(right['solved_delta'])}"
    )
    left_index = int(np.argmax([
        np.linalg.norm(row["target_delta"]) for row in diagnostics["left"]
    ]))
    left = diagnostics["left"][left_index]
    print(
        f"left representative frame {left_index} "
        f"target dXYZ={vector_mm(left['target_delta'])} "
        f"solved dXYZ={vector_mm(left['solved_delta'])}"
    )
    print(f"output={args.output}")
    print(f"diagnostics={args.diagnostics_output}")


def main():
    args = parse_args()
    chains = load_leg_chains(args.sdf)
    source_rows, source_positions = load_source(
        args.source, args.target_source, args.max_frames,
    )
    solutions = {}
    diagnostics = {}
    for side in ("right", "left"):
        solutions[side], diagnostics[side], _ = solve_trajectory(
            chains[side], mapped_delta(source_positions[side]), args.dt,
        )
    for side in ("right", "left"):
        if not np.all(np.isfinite(solutions[side])):
            raise RuntimeError(f"non-finite {side} joint solution")
    write_outputs(args, source_rows, solutions, diagnostics)
    print_summary(args, chains, solutions, diagnostics)


if __name__ == "__main__":
    main()
