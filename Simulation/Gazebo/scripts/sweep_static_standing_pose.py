#!/usr/bin/env python3
"""Sweep symmetric STEP leg pitch poses using static SDF kinematics only."""

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from log_free_base_com_balance import (
    DEFAULT_MODEL_SDF,
    compose_poses,
    convex_hull,
    load_link_inertials,
    load_sole_boxes,
    parse_pose_text,
    quaternion_to_rpy,
    rotate_vector,
    rpy_to_quaternion,
    signed_distance_to_polygon,
    sole_world_geometry,
)
from publish_leg_constant_pose import CONSTANT_POSE


DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "logs"
    / "static_standing_pose_sweep.csv"
)
DEFAULT_MODEL_POSE = "0 0 0.413989 0 0 0"

JOINT_COMMAND_KEYS = {
    "right_hip_yaw_joint": "RL0_wrap",
    "right_hip_roll_joint": "RL1_wrap",
    "right_hip_pitch_joint": "RL2_wrap",
    "right_knee_pitch_joint": "RL3_wrap",
    "right_ankle_pitch_joint": "RL4_wrap",
    "right_ankle_roll_joint": "RL5_wrap",
    "left_hip_yaw_joint": "LL0_wrap",
    "left_hip_roll_joint": "LL1_wrap",
    "left_hip_pitch_joint": "LL2_wrap",
    "left_knee_pitch_joint": "LL3_wrap",
    "left_ankle_pitch_joint": "LL4_wrap",
    "left_ankle_roll_joint": "LL5_wrap",
}

CSV_COLUMNS = (
    "hip_pitch_delta",
    "knee_pitch_delta",
    "ankle_pitch_delta",
    "joint_delta_l2",
    "right_hip_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
    "left_hip_pitch",
    "left_knee_pitch",
    "left_ankle_pitch",
    "com_x",
    "com_y",
    "com_z",
    "support_x_min",
    "support_x_max",
    "support_y_min",
    "support_y_max",
    "support_polygon_xy",
    "com_signed_distance",
    "com_inside_support",
    "left_sole_roll",
    "left_sole_pitch",
    "right_sole_roll",
    "right_sole_pitch",
    "sole_tilt_max",
    "candidate_valid",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Find symmetric hip/knee/ankle pitch candidates from static SDF "
            "geometry. Gazebo physics and controllers are not started."
        )
    )
    parser.add_argument("--model-sdf", type=Path, default=DEFAULT_MODEL_SDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--model-pose", default=DEFAULT_MODEL_POSE)
    parser.add_argument("--delta-min", type=float, default=-0.30)
    parser.add_argument("--delta-max", type=float, default=0.30)
    parser.add_argument("--step", type=float, default=0.02)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument(
        "--max-sole-tilt",
        type=float,
        default=0.01,
        help=(
            "Maximum absolute sole roll/pitch for a valid standing candidate "
            "(default: 0.01 rad)"
        ),
    )
    parser.add_argument(
        "--no-refine",
        action="store_true",
        help=(
            "Do not refine around minimum-change, margin-threshold, and "
            "maximum-margin coarse candidates"
        ),
    )
    args = parser.parse_args()
    numeric = (args.delta_min, args.delta_max, args.step, args.max_sole_tilt)
    if not all(math.isfinite(value) for value in numeric):
        parser.error("Sweep bounds and step must be finite")
    if args.delta_min > args.delta_max:
        parser.error("--delta-min must not exceed --delta-max")
    if args.step <= 0.0:
        parser.error("--step must be greater than zero")
    if args.max_sole_tilt < 0.0:
        parser.error("--max-sole-tilt must be non-negative")
    if args.top <= 0:
        parser.error("--top must be greater than zero")
    try:
        parse_pose_text(args.model_pose)
    except ValueError as error:
        parser.error(f"--model-pose: {error}")
    return args


def pose_from_element(element):
    values = parse_pose_text(element.text if element is not None else None)
    x, y, z, roll, pitch, yaw = values
    return ((x, y, z), rpy_to_quaternion(roll, pitch, yaw))


def load_kinematic_tree(model_sdf):
    try:
        model = ET.parse(model_sdf).getroot().find("model")
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"Could not read model SDF {model_sdf}: {error}") from error
    if model is None:
        raise RuntimeError(f"No <model> found in {model_sdf}")

    link_poses = {}
    for link in model.findall("link"):
        name = link.get("name")
        if not name:
            continue
        pose_element = link.find("pose")
        link_poses[name] = (
            pose_from_element(pose_element),
            pose_element.get("relative_to") if pose_element is not None else None,
        )

    joints = {}
    child_links = set()
    for joint in model.findall("joint"):
        name = joint.get("name")
        parent = joint.findtext("parent")
        child = joint.findtext("child")
        pose_element = joint.find("pose")
        if not name or not parent or not child:
            raise RuntimeError("Every joint must have name, parent, and child")
        relative_to = pose_element.get("relative_to") if pose_element is not None else None
        if relative_to not in (None, parent):
            raise RuntimeError(
                f"Joint {name} pose relative_to={relative_to!r}, expected {parent!r}"
            )
        child_pose, child_relative_to = link_poses[child]
        if child_relative_to not in (None, name):
            raise RuntimeError(
                f"Link {child} pose relative_to={child_relative_to!r}, expected {name!r}"
            )
        joints[child] = {
            "name": name,
            "type": joint.get("type"),
            "parent": parent,
            "joint_pose": pose_from_element(pose_element),
            "child_pose": child_pose,
        }
        child_links.add(child)

    roots = set(link_poses) - child_links
    if len(roots) != 1:
        raise RuntimeError(f"Expected one root link, found {sorted(roots)}")
    return link_poses, joints, roots.pop()


def forward_kinematics(link_poses, joints, root_link, model_pose, commands):
    root_local, root_relative_to = link_poses[root_link]
    if root_relative_to:
        raise RuntimeError(
            f"Root link {root_link} has unsupported relative_to={root_relative_to!r}"
        )
    world_poses = {root_link: compose_poses(model_pose, root_local)}
    unresolved = dict(joints)
    while unresolved:
        progressed = False
        for child, joint in list(unresolved.items()):
            parent_pose = world_poses.get(joint["parent"])
            if parent_pose is None:
                continue
            joint_world = compose_poses(parent_pose, joint["joint_pose"])
            angle = commands.get(joint["name"], 0.0)
            if joint["type"] == "fixed" and abs(angle) > 1e-12:
                raise RuntimeError(f"Fixed joint {joint['name']} received a command")
            rotation = ((0.0, 0.0, 0.0), rpy_to_quaternion(0.0, 0.0, angle))
            child_world = compose_poses(
                compose_poses(joint_world, rotation), joint["child_pose"]
            )
            world_poses[child] = child_world
            del unresolved[child]
            progressed = True
        if not progressed:
            names = ", ".join(sorted(unresolved))
            raise RuntimeError(f"Could not resolve kinematic links: {names}")
    return world_poses


def whole_body_com(world_poses, inertials):
    weighted = [0.0, 0.0, 0.0]
    total_mass = 0.0
    for link_name, (mass, local_com) in inertials.items():
        if link_name not in world_poses:
            raise RuntimeError(f"Missing world pose for inertial link {link_name}")
        position, orientation = world_poses[link_name]
        offset = rotate_vector(local_com, orientation)
        for axis in range(3):
            weighted[axis] += mass * (position[axis] + offset[axis])
        total_mass += mass
    return tuple(value / total_mass for value in weighted)


def symmetric_commands(hip_delta, knee_delta, ankle_delta):
    commands = {
        joint_name: CONSTANT_POSE[key]
        for joint_name, key in JOINT_COMMAND_KEYS.items()
    }
    for joint, delta in (
        ("right_hip_pitch_joint", hip_delta),
        ("right_knee_pitch_joint", knee_delta),
        ("right_ankle_pitch_joint", ankle_delta),
    ):
        commands[joint] += delta
    for joint, delta in (
        ("left_hip_pitch_joint", hip_delta),
        ("left_knee_pitch_joint", knee_delta),
        ("left_ankle_pitch_joint", ankle_delta),
    ):
        commands[joint] -= delta
    return commands


def evaluate_candidate(
    hip_delta,
    knee_delta,
    ankle_delta,
    link_poses,
    joints,
    root_link,
    model_pose,
    inertials,
    sole_boxes,
):
    commands = symmetric_commands(hip_delta, knee_delta, ankle_delta)
    world_poses = forward_kinematics(
        link_poses, joints, root_link, model_pose, commands
    )
    com = whole_body_com(world_poses, inertials)
    sole_geometry = sole_world_geometry(world_poses, sole_boxes)
    polygon = convex_hull(
        [
            (corner[0], corner[1])
            for side in ("left", "right")
            for corner in sole_geometry[side]["corners"]
        ]
    )
    signed_distance = signed_distance_to_polygon((com[0], com[1]), polygon)
    sole_angles = {
        side: quaternion_to_rpy(sole_geometry[side]["pose"][1])
        for side in ("left", "right")
    }
    xs = [point[0] for point in polygon]
    ys = [point[1] for point in polygon]
    polygon_text = ";".join(f"{x:+.6f},{y:+.6f}" for x, y in polygon)
    sole_tilt_max = max(
        abs(sole_angles[side][axis])
        for side in ("left", "right")
        for axis in (0, 1)
    )
    return {
        "hip_pitch_delta": hip_delta,
        "knee_pitch_delta": knee_delta,
        "ankle_pitch_delta": ankle_delta,
        "joint_delta_l2": math.sqrt(
            2.0 * (hip_delta**2 + knee_delta**2 + ankle_delta**2)
        ),
        "right_hip_pitch": commands["right_hip_pitch_joint"],
        "right_knee_pitch": commands["right_knee_pitch_joint"],
        "right_ankle_pitch": commands["right_ankle_pitch_joint"],
        "left_hip_pitch": commands["left_hip_pitch_joint"],
        "left_knee_pitch": commands["left_knee_pitch_joint"],
        "left_ankle_pitch": commands["left_ankle_pitch_joint"],
        "com_x": com[0],
        "com_y": com[1],
        "com_z": com[2],
        "support_x_min": min(xs),
        "support_x_max": max(xs),
        "support_y_min": min(ys),
        "support_y_max": max(ys),
        "support_polygon_xy": polygon_text,
        "com_signed_distance": signed_distance,
        "com_inside_support": int(signed_distance >= 0.0),
        "left_sole_roll": sole_angles["left"][0],
        "left_sole_pitch": sole_angles["left"][1],
        "right_sole_roll": sole_angles["right"][0],
        "right_sole_pitch": sole_angles["right"][1],
        "sole_tilt_max": sole_tilt_max,
        "candidate_valid": 0,
    }


def inclusive_range(start, stop, step):
    count = int(math.floor((stop - start) / step + 1e-9))
    values = [start + index * step for index in range(count + 1)]
    if not values or values[-1] < stop - 1e-9:
        values.append(stop)
    if start <= 0.0 <= stop and all(abs(value) > 1e-12 for value in values):
        values.append(0.0)
    return sorted(set(round(value, 12) for value in values))


def sweep(values, evaluator, existing=None):
    results = [] if existing is None else existing
    seen = {
        (
            round(row["hip_pitch_delta"], 10),
            round(row["knee_pitch_delta"], 10),
            round(row["ankle_pitch_delta"], 10),
        )
        for row in results
    }
    for hip_delta in values[0]:
        for knee_delta in values[1]:
            for ankle_delta in values[2]:
                key = tuple(round(value, 10) for value in (
                    hip_delta, knee_delta, ankle_delta
                ))
                if key not in seen:
                    results.append(evaluator(*key))
                    seen.add(key)
    return results


def candidate_sort_key(row):
    if row["candidate_valid"]:
        return (0, row["joint_delta_l2"], -row["com_signed_distance"])
    if row["com_inside_support"]:
        return (1, row["sole_tilt_max"], row["joint_delta_l2"])
    return (2, -row["com_signed_distance"], row["joint_delta_l2"])


def print_table(rows, top, title="TOP STATIC CANDIDATES"):
    print(f"\n[{title}]")
    print(
        " rk V dHip   dKnee  dAnkl  |dQ|   signed   COM(x,y,z)"
        "                    support x / y                 sole L(r,p) / R(r,p)"
    )
    for rank, row in enumerate(rows[:top], 1):
        print(
            f"{rank:3d} {row['candidate_valid']:1d} "
            f"{row['hip_pitch_delta']:+.3f} "
            f"{row['knee_pitch_delta']:+.3f} "
            f"{row['ankle_pitch_delta']:+.3f} "
            f"{row['joint_delta_l2']:.3f} "
            f"{row['com_signed_distance']:+.5f}  "
            f"({row['com_x']:+.4f},{row['com_y']:+.4f},{row['com_z']:+.4f})  "
            f"[{row['support_x_min']:+.3f},{row['support_x_max']:+.3f}] / "
            f"[{row['support_y_min']:+.3f},{row['support_y_max']:+.3f}]  "
            f"({row['left_sole_roll']:+.3f},{row['left_sole_pitch']:+.3f}) / "
            f"({row['right_sole_roll']:+.3f},{row['right_sole_pitch']:+.3f})"
        )


def minimum_l2_with_margin(valid_rows, minimum_margin):
    eligible = [
        row
        for row in valid_rows
        if row["com_signed_distance"] >= minimum_margin
    ]
    if not eligible:
        return None
    return min(
        eligible,
        key=lambda row: (
            row["joint_delta_l2"],
            -row["com_signed_distance"],
        ),
    )


def refinement_centers(valid_rows):
    if not valid_rows:
        return []
    centers = [minimum_l2_with_margin(valid_rows, 0.0)]
    for margin in (0.005, 0.010, 0.020):
        candidate = minimum_l2_with_margin(valid_rows, margin)
        if candidate is not None:
            centers.append(candidate)
    centers.append(
        max(
            valid_rows,
            key=lambda row: (
                row["com_signed_distance"],
                -row["joint_delta_l2"],
            ),
        )
    )
    unique = {}
    for row in centers:
        key = tuple(
            round(row[name], 10)
            for name in (
                "hip_pitch_delta",
                "knee_pitch_delta",
                "ankle_pitch_delta",
            )
        )
        unique[key] = row
    return list(unique.values())


def refinement_axes(center, radius, step, lower_bound, upper_bound):
    return tuple(
        inclusive_range(
            max(lower_bound, center[key] - radius),
            min(upper_bound, center[key] + radius),
            step,
        )
        for key in (
            "hip_pitch_delta",
            "knee_pitch_delta",
            "ankle_pitch_delta",
        )
    )


def print_compromise_candidates(valid_rows):
    print("\n[STATIC MARGIN / JOINT CHANGE COMPROMISE]")
    thresholds = (
        ("MINIMUM-L2 POSITIVE", 0.0),
        ("MINIMUM-L2 WITH MARGIN >= 5 mm", 0.005),
        ("MINIMUM-L2 WITH MARGIN >= 10 mm", 0.010),
        ("MINIMUM-L2 WITH MARGIN >= 20 mm", 0.020),
    )
    for label, threshold in thresholds:
        candidate = minimum_l2_with_margin(valid_rows, threshold)
        if candidate is None:
            print(f"\n[{label}] no candidate in the configured sweep range")
        else:
            print_candidate_details(label, candidate)

    if valid_rows:
        maximum_margin = max(
            valid_rows,
            key=lambda row: (
                row["com_signed_distance"],
                -row["joint_delta_l2"],
            ),
        )
        print_candidate_details("MAXIMUM-MARGIN CANDIDATE", maximum_margin)


def print_candidate_details(label, row):
    print(f"\n[{label}]")
    print(
        "pitch commands: "
        f"right=({row['right_hip_pitch']:+.6f}, "
        f"{row['right_knee_pitch']:+.6f}, "
        f"{row['right_ankle_pitch']:+.6f}), "
        f"left=({row['left_hip_pitch']:+.6f}, "
        f"{row['left_knee_pitch']:+.6f}, "
        f"{row['left_ankle_pitch']:+.6f})"
    )
    print(
        f"COM: ({row['com_x']:+.6f}, {row['com_y']:+.6f}, "
        f"{row['com_z']:+.6f}) m"
    )
    print(f"support polygon: {row['support_polygon_xy']}")
    print(
        f"signed distance: {row['com_signed_distance']:+.6f} m, "
        f"inside={row['com_inside_support']}, valid={row['candidate_valid']}"
    )
    print(
        "sole roll/pitch: "
        f"left=({row['left_sole_roll']:+.6f}, {row['left_sole_pitch']:+.6f}), "
        f"right=({row['right_sole_roll']:+.6f}, "
        f"{row['right_sole_pitch']:+.6f}) rad"
    )


def main():
    args = parse_args()
    try:
        link_poses, joints, root_link = load_kinematic_tree(args.model_sdf)
        inertials = load_link_inertials(args.model_sdf)
        sole_boxes = load_sole_boxes(args.model_sdf)
        model_pose_values = parse_pose_text(args.model_pose)
        model_pose = (
            model_pose_values[:3],
            rpy_to_quaternion(*model_pose_values[3:]),
        )

        def evaluator(hip_delta, knee_delta, ankle_delta):
            row = evaluate_candidate(
                hip_delta,
                knee_delta,
                ankle_delta,
                link_poses,
                joints,
                root_link,
                model_pose,
                inertials,
                sole_boxes,
            )
            row["candidate_valid"] = int(
                row["com_inside_support"]
                and row["sole_tilt_max"] <= args.max_sole_tilt
            )
            return row

        coarse_values = inclusive_range(args.delta_min, args.delta_max, args.step)
        total = len(coarse_values) ** 3
        print("[STATIC STANDING POSE SWEEP]")
        print(f"model: {args.model_sdf}")
        print("physics/controller: not started")
        print(
            f"symmetric pitch deltas: [{args.delta_min:+.3f}, "
            f"{args.delta_max:+.3f}] rad, step={args.step:.3f}, "
            f"coarse candidates={total}"
        )
        print(f"valid sole tilt limit: {args.max_sole_tilt:.6f} rad")
        results = sweep((coarse_values,) * 3, evaluator)
        coarse_valid = [row for row in results if row["candidate_valid"]]
        if coarse_valid and not args.no_refine:
            fine_step = args.step / 5.0
            for center in refinement_centers(coarse_valid):
                results = sweep(
                    refinement_axes(
                        center,
                        args.step,
                        fine_step,
                        args.delta_min,
                        args.delta_max,
                    ),
                    evaluator,
                    results,
                )

        ranked = sorted(results, key=candidate_sort_key)
        valid = [row for row in results if row["candidate_valid"]]
        margin_ranked = sorted(
            valid,
            key=lambda row: (
                -row["com_signed_distance"],
                row["joint_delta_l2"],
            ),
        )
        baseline = min(
            results,
            key=lambda row: abs(row["hip_pitch_delta"])
            + abs(row["knee_pitch_delta"])
            + abs(row["ankle_pitch_delta"]),
        )
        positive_count = sum(row["com_inside_support"] for row in results)
        valid_count = sum(row["candidate_valid"] for row in results)
        margin_10mm_count = sum(
            row["candidate_valid"] and row["com_signed_distance"] > 0.010
            for row in results
        )
        margin_20mm_count = sum(
            row["candidate_valid"] and row["com_signed_distance"] > 0.020
            for row in results
        )
        print(
            f"evaluated: {len(results)}, positive signed distance: "
            f"{positive_count}, valid positive + level sole: {valid_count}"
        )
        print(
            f"valid margin > 10 mm: {margin_10mm_count}, "
            f"> 20 mm: {margin_20mm_count}"
        )
        print_candidate_details("BASELINE CONSTANT_POSE", baseline)
        print_table(ranked, args.top, "MINIMUM-L2 VALID CANDIDATES")
        if valid_count:
            print_candidate_details("MINIMUM-L2 VALID CANDIDATE", ranked[0])
            print_table(
                margin_ranked,
                args.top,
                "MAXIMUM STATIC STABILITY MARGIN CANDIDATES",
            )
            maximum_margin = margin_ranked[0]
            boundary_tolerance = args.step / 5.0 + 1e-12
            if any(
                abs(maximum_margin[key] - boundary) <= boundary_tolerance
                for key in (
                    "hip_pitch_delta",
                    "knee_pitch_delta",
                    "ankle_pitch_delta",
                )
                for boundary in (args.delta_min, args.delta_max)
            ):
                print(
                    "[WARNING] Maximum-margin candidate touches a sweep "
                    "boundary; it is only the maximum within the configured "
                    "range. Expand --delta-min/--delta-max if joint limits "
                    "permit."
                )
            print_compromise_candidates(valid)
        else:
            print_candidate_details("CLOSEST CANDIDATE (NO POSITIVE RESULT)", ranked[0])

        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(ranked)
        print(f"\nCSV: {args.output}")
        return 0
    except (RuntimeError, ValueError) as error:
        print(f"[ERROR] {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
