#!/usr/bin/env python3
"""Diagnose static left/right leg asymmetry in the free-base test model."""

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from log_free_base_com_balance import (
    compose_poses,
    parse_pose_text,
    rotate_vector,
    rpy_to_quaternion,
)
from publish_leg_constant_pose import STANDING_CANDIDATE_POSE
from sweep_static_standing_pose import (
    JOINT_COMMAND_KEYS,
    forward_kinematics,
    load_kinematic_tree,
)


GAZEBO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_SDF = (
    GAZEBO_DIR
    / "models"
    / "step_urdf"
    / "step_free_base_constant_pose_initial.sdf"
)
DEFAULT_MODEL_POSE = "0 0 0.410461 0 0 0"

LINK_PAIRS = (
    ("hip_yaw", "left_hip_yaw_link", "right_hip_yaw_link"),
    ("hip_roll", "left_hip_roll_link", "right_hip_roll_link"),
    ("thigh", "left_thigh_link", "right_thigh_link"),
    ("calf", "left_calf_link", "right_calf_link"),
    ("ankle", "left_ankle_link", "right_ankle_link"),
    ("foot", "left_foot_link", "right_foot_link"),
)
JOINT_PAIRS = (
    ("hip_yaw", "left_hip_yaw_joint", "right_hip_yaw_joint"),
    ("hip_roll", "left_hip_roll_joint", "right_hip_roll_joint"),
    ("hip_pitch", "left_hip_pitch_joint", "right_hip_pitch_joint"),
    ("knee_pitch", "left_knee_pitch_joint", "right_knee_pitch_joint"),
    ("ankle_pitch", "left_ankle_pitch_joint", "right_ankle_pitch_joint"),
    ("ankle_roll", "left_ankle_roll_joint", "right_ankle_roll_joint"),
)
MIRROR_X = ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Compare left/right leg inertial, joint, and controller symmetry "
            "without running Gazebo physics."
        )
    )
    parser.add_argument("--model-sdf", type=Path, default=DEFAULT_MODEL_SDF)
    parser.add_argument("--model-pose", default=DEFAULT_MODEL_POSE)
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-6,
        help="Threshold for the non-zero asymmetry summary (default: 1e-6)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.tolerance) or args.tolerance < 0.0:
        parser.error("--tolerance must be finite and non-negative")
    try:
        parse_pose_text(args.model_pose)
    except ValueError as error:
        parser.error(f"--model-pose: {error}")
    return args


def vector_norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def vector_sub(left, right):
    return tuple(a - b for a, b in zip(left, right))


def mirror_position(position):
    return (-position[0], position[1], position[2])


def matrix_multiply(left, right):
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def matrix_transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def matrix_sub(left, right):
    return tuple(
        tuple(left[row][column] - right[row][column] for column in range(3))
        for row in range(3)
    )


def matrix_frobenius(matrix):
    return math.sqrt(sum(value * value for row in matrix for value in row))


def quaternion_matrix(quaternion):
    x, y, z, w = quaternion
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def transform_inertia(local_inertia, orientation):
    rotation = quaternion_matrix(orientation)
    return matrix_multiply(
        matrix_multiply(rotation, local_inertia), matrix_transpose(rotation)
    )


def mirror_tensor(tensor):
    return matrix_multiply(matrix_multiply(MIRROR_X, tensor), MIRROR_X)


def format_vector(vector):
    return "(" + ", ".join(f"{value:+.9g}" for value in vector) + ")"


def format_matrix(matrix):
    return "[" + "; ".join(
        ", ".join(f"{value:+.9g}" for value in row) for row in matrix
    ) + "]"


def pose_from_element(element):
    values = parse_pose_text(element.text if element is not None else None)
    return values, (values[:3], rpy_to_quaternion(*values[3:]))


def load_model_details(model_sdf):
    try:
        model = ET.parse(model_sdf).getroot().find("model")
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"Could not read {model_sdf}: {error}") from error
    if model is None:
        raise RuntimeError(f"No <model> found in {model_sdf}")

    links = {}
    for link in model.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if not name or inertial is None:
            continue
        mass = float(inertial.findtext("mass"))
        pose_values, inertial_pose = pose_from_element(inertial.find("pose"))
        tensor = inertial.find("inertia")
        ixx = float(tensor.findtext("ixx"))
        ixy = float(tensor.findtext("ixy"))
        ixz = float(tensor.findtext("ixz"))
        iyy = float(tensor.findtext("iyy"))
        iyz = float(tensor.findtext("iyz"))
        izz = float(tensor.findtext("izz"))
        links[name] = {
            "mass": mass,
            "inertial_pose_values": pose_values,
            "inertial_pose": inertial_pose,
            "inertia": ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz)),
        }

    joints = {}
    for joint in model.findall("joint"):
        name = joint.get("name")
        if not name:
            continue
        pose_values, pose = pose_from_element(joint.find("pose"))
        axis_element = joint.find("axis")
        axis = (0.0, 0.0, 0.0)
        effort = velocity = math.nan
        if axis_element is not None:
            axis_values = [float(value) for value in axis_element.findtext("xyz").split()]
            axis = tuple(axis_values)
            effort = float(axis_element.findtext("limit/effort", "nan"))
            velocity = float(axis_element.findtext("limit/velocity", "nan"))
        joints[name] = {
            "parent": joint.findtext("parent"),
            "child": joint.findtext("child"),
            "pose_values": pose_values,
            "pose": pose,
            "axis": axis,
            "effort": effort,
            "velocity": velocity,
        }

    controllers = {}
    for plugin in model.findall("plugin"):
        if plugin.get("name") != "gz::sim::systems::JointPositionController":
            continue
        name = plugin.findtext("joint_name")
        controllers[name] = {
            "p": float(plugin.findtext("p_gain")),
            "i": float(plugin.findtext("i_gain")),
            "d": float(plugin.findtext("d_gain")),
            "target": float(plugin.findtext("initial_position")),
        }
    return links, joints, controllers


def build_world_geometry(model_sdf, model_pose):
    link_poses, tree_joints, root_link = load_kinematic_tree(model_sdf)
    commands = {
        joint_name: STANDING_CANDIDATE_POSE[column]
        for joint_name, column in JOINT_COMMAND_KEYS.items()
    }
    world_links = forward_kinematics(
        link_poses, tree_joints, root_link, model_pose, commands
    )
    joint_world = {}
    for joint in tree_joints.values():
        joint_world[joint["name"]] = compose_poses(
            world_links[joint["parent"]], joint["joint_pose"]
        )
    return world_links, joint_world


def link_world_com(world_link_pose, inertial_pose):
    return compose_poses(world_link_pose, inertial_pose)[0]


def joint_expected_axis(left_axis, right_axis, left_target, right_target):
    mirrored = mirror_position(left_axis)
    if abs(left_target) > 1e-12 and abs(right_target) > 1e-12:
        command_ratio = left_target / right_target
        return tuple(-command_ratio * value for value in mirrored)
    direct_error = vector_norm(vector_sub(right_axis, mirrored))
    axial_mirror = tuple(-value for value in mirrored)
    axial_error = vector_norm(vector_sub(right_axis, axial_mirror))
    return mirrored if direct_error <= axial_error else axial_mirror


def main():
    args = parse_args()
    try:
        model_pose_values = parse_pose_text(args.model_pose)
        model_pose = (
            model_pose_values[:3], rpy_to_quaternion(*model_pose_values[3:])
        )
        links, joints, controllers = load_model_details(args.model_sdf)
        world_links, world_joints = build_world_geometry(args.model_sdf, model_pose)
    except (RuntimeError, ValueError, KeyError) as error:
        print(f"[ERROR] {error}")
        return 1

    asymmetries = []
    left_mass = right_mass = 0.0
    left_weighted = [0.0, 0.0, 0.0]
    right_weighted = [0.0, 0.0, 0.0]

    print("[FREE-BASE LEG LINK SYMMETRY]")
    print(f"model: {args.model_sdf}")
    print("mirror plane: world x=0")
    for label, left_name, right_name in LINK_PAIRS:
        left = links[left_name]
        right = links[right_name]
        left_com = link_world_com(world_links[left_name], left["inertial_pose"])
        right_com = link_world_com(world_links[right_name], right["inertial_pose"])
        left_world_inertia = transform_inertia(left["inertia"], world_links[left_name][1])
        right_world_inertia = transform_inertia(right["inertia"], world_links[right_name][1])
        mass_diff = right["mass"] - left["mass"]
        com_error = vector_sub(right_com, mirror_position(left_com))
        inertia_error = matrix_sub(right_world_inertia, mirror_tensor(left_world_inertia))
        inertia_error_norm = matrix_frobenius(inertia_error)
        left_mass += left["mass"]
        right_mass += right["mass"]
        for axis in range(3):
            left_weighted[axis] += left["mass"] * left_com[axis]
            right_weighted[axis] += right["mass"] * right_com[axis]

        print(f"\n[LINK PAIR: {label}]")
        for side, name, data, com, world_inertia in (
            ("L", left_name, left, left_com, left_world_inertia),
            ("R", right_name, right, right_com, right_world_inertia),
        ):
            print(
                f"{side} {name}: mass={data['mass']:.12g}, "
                f"inertial_pose={format_vector(data['inertial_pose_values'])}"
            )
            print(f"  inertia_link={format_matrix(data['inertia'])}")
            print(f"  world_com={format_vector(com)}")
            print(f"  inertia_world={format_matrix(world_inertia)}")
        print(
            f"DIFF: mass(R-L)={mass_diff:+.9g}, "
            f"mirrored_world_com_error={format_vector(com_error)}, "
            f"norm={vector_norm(com_error):.9g}, "
            f"mirrored_world_inertia_error_norm={inertia_error_norm:.9g}"
        )
        if abs(mass_diff) > args.tolerance:
            asymmetries.append(("mass", label, abs(mass_diff)))
        if vector_norm(com_error) > args.tolerance:
            asymmetries.append(("link COM mirror", label, vector_norm(com_error)))
        if inertia_error_norm > args.tolerance:
            asymmetries.append(("world inertia mirror", label, inertia_error_norm))

    print("\n[FREE-BASE LEG JOINT SYMMETRY]")
    for label, left_name, right_name in JOINT_PAIRS:
        left = joints[left_name]
        right = joints[right_name]
        left_controller = controllers[left_name]
        right_controller = controllers[right_name]
        left_joint_world = world_joints[left_name]
        right_joint_world = world_joints[right_name]
        left_axis_world = rotate_vector(left["axis"], left_joint_world[1])
        right_axis_world = rotate_vector(right["axis"], right_joint_world[1])
        expected_axis = joint_expected_axis(
            left_axis_world,
            right_axis_world,
            left_controller["target"],
            right_controller["target"],
        )
        axis_error = vector_norm(vector_sub(right_axis_world, expected_axis))
        position_error = vector_sub(
            right_joint_world[0], mirror_position(left_joint_world[0])
        )
        left_rotation = quaternion_matrix(left_joint_world[1])
        right_rotation = quaternion_matrix(right_joint_world[1])
        expected_right_rotation = matrix_multiply(
            matrix_multiply(MIRROR_X, left_rotation), MIRROR_X
        )
        orientation_error = matrix_frobenius(
            matrix_sub(right_rotation, expected_right_rotation)
        )
        target_sign_error = right_controller["target"] + left_controller["target"]
        gain_error = (
            right_controller["p"] - left_controller["p"],
            right_controller["i"] - left_controller["i"],
            right_controller["d"] - left_controller["d"],
        )
        limit_error = (
            right["effort"] - left["effort"],
            right["velocity"] - left["velocity"],
        )
        print(f"\n[JOINT PAIR: {label}]")
        for side, name, data, controller, pose, axis in (
            ("L", left_name, left, left_controller, left_joint_world, left_axis_world),
            ("R", right_name, right, right_controller, right_joint_world, right_axis_world),
        ):
            print(f"{side} {name}: parent={data['parent']}, child={data['child']}")
            print(
                f"  joint_pose={format_vector(data['pose_values'])}, "
                f"world_position={format_vector(pose[0])}, "
                f"axis_world={format_vector(axis)}"
            )
            print(
                f"  effort={data['effort']:.9g}, velocity={data['velocity']:.9g}, "
                f"PID=({controller['p']:.9g},{controller['i']:.9g},"
                f"{controller['d']:.9g}), target={controller['target']:+.9g}"
            )
        print(
            f"DIFF: mirrored_joint_position_error={format_vector(position_error)}, "
            f"norm={vector_norm(position_error):.9g}, "
            f"joint_frame_mirror_error={orientation_error:.9g}, "
            f"axis_mirror_error={axis_error:.9g}, "
            f"limits(R-L)={format_vector(limit_error)}, "
            f"PID(R-L)={format_vector(gain_error)}, "
            f"target(L+R)={target_sign_error:+.9g}"
        )
        if vector_norm(position_error) > args.tolerance:
            asymmetries.append(("joint position mirror", label, vector_norm(position_error)))
        if axis_error > args.tolerance:
            asymmetries.append(("joint axis mirror", label, axis_error))
        if orientation_error > args.tolerance:
            asymmetries.append(("joint frame mirror", label, orientation_error))
        if vector_norm(limit_error) > args.tolerance:
            asymmetries.append(("joint limits", label, vector_norm(limit_error)))
        if vector_norm(gain_error) > args.tolerance:
            asymmetries.append(("controller gains", label, vector_norm(gain_error)))
        if abs(target_sign_error) > args.tolerance:
            asymmetries.append(("controller target sign", label, abs(target_sign_error)))

    left_leg_com = tuple(value / left_mass for value in left_weighted)
    right_leg_com = tuple(value / right_mass for value in right_weighted)
    leg_com_error = vector_sub(right_leg_com, mirror_position(left_leg_com))
    whole_mass = 0.0
    whole_weighted = [0.0, 0.0, 0.0]
    for name, data in links.items():
        com = link_world_com(world_links[name], data["inertial_pose"])
        whole_mass += data["mass"]
        for axis in range(3):
            whole_weighted[axis] += data["mass"] * com[axis]
    whole_com = tuple(value / whole_mass for value in whole_weighted)

    print("\n[CONCISE ASYMMETRY SUMMARY]")
    print(f"left leg total mass:  {left_mass:.12g} kg")
    print(f"right leg total mass: {right_mass:.12g} kg")
    print(f"mass difference R-L:  {right_mass-left_mass:+.12g} kg")
    print(f"left leg world COM:   {format_vector(left_leg_com)} m")
    print(f"right leg world COM:  {format_vector(right_leg_com)} m")
    print(f"mirrored leg COM error: {format_vector(leg_com_error)} m")
    print(f"whole-body world COM: {format_vector(whole_com)} m")
    print(f"whole-body lateral offset x: {whole_com[0]:+.9g} m")
    if asymmetries:
        print(f"non-zero items above tolerance {args.tolerance:g}:")
        for kind, pair, magnitude in sorted(asymmetries, key=lambda item: -item[2]):
            print(f"  {kind:24s} {pair:12s} magnitude={magnitude:.9g}")
    else:
        print(f"no pair asymmetry exceeds tolerance {args.tolerance:g}")
    print(
        "Yaw-torque relevance: mass/COM, world-inertia, joint-position, "
        "joint-axis, limit, or PID differences above tolerance are candidates; "
        "small CAD rounding differences require comparison against the observed "
        "contact-force impulse before assigning causality."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
