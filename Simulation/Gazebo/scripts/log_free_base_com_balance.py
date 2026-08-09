#!/usr/bin/env python3
"""Log STEP whole-body COM, base pose, and sole support bounds from Gazebo."""

import argparse
import csv
import math
import signal
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


GAZEBO_DIR = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_SDF = (
    GAZEBO_DIR
    / "models"
    / "step_urdf"
    / "step_free_base_constant_pose_initial.sdf"
)
DEFAULT_OUTPUT = GAZEBO_DIR / "logs" / "free_base_com_balance.csv"
DEFAULT_POSE_TOPIC = (
    "/world/step_constant_pose_initial_test/dynamic_pose/info"
)
DEFAULT_LEFT_CONTACT_TOPIC = "/step/left_sole_contacts"
DEFAULT_RIGHT_CONTACT_TOPIC = "/step/right_sole_contacts"
DEFAULT_MODEL_NAME = "step_humanoid_constant_pose_initial"
DEFAULT_DURATION = 2.0
DEFAULT_DT = 0.01

CSV_COLUMNS = (
    "time",
    "com_x",
    "com_y",
    "com_z",
    "base_x",
    "base_y",
    "base_z",
    "base_roll",
    "base_pitch",
    "base_yaw",
    "support_x_min",
    "support_x_max",
    "support_y_min",
    "support_y_max",
    "com_inside_support",
    "left_contact",
    "right_contact",
    "support_point_count",
    "left_fx",
    "left_fy",
    "left_fz",
    "right_fx",
    "right_fy",
    "right_fz",
    "left_contact_x_mean",
    "left_contact_y_mean",
    "right_contact_x_mean",
    "right_contact_y_mean",
    "right_minus_left_fz",
    "left_force_age",
    "right_force_age",
    "left_force_stale",
    "right_force_stale",
    "com_support_signed_distance",
    "source_sim_time",
    "pose_updated",
)

SOLE_COLLISION_NAMES = {
    "left": "left_sole_box_collision",
    "right": "right_sole_box_collision",
}


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Log whole-body COM and base orientation against the current "
            "two-sole contact support bounds."
        )
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--model-sdf", type=Path, default=DEFAULT_MODEL_SDF)
    parser.add_argument("--model-name", default=DEFAULT_MODEL_NAME)
    parser.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC)
    parser.add_argument(
        "--left-contact-topic",
        default=DEFAULT_LEFT_CONTACT_TOPIC,
    )
    parser.add_argument(
        "--right-contact-topic",
        default=DEFAULT_RIGHT_CONTACT_TOPIC,
    )
    parser.add_argument(
        "--contact-timeout",
        type=float,
        default=0.1,
        help="Force message age above which a sample is marked stale",
    )
    parser.add_argument(
        "--angle-change-threshold",
        type=float,
        default=0.01,
        help="Angle change used by the initial summary (default: 0.01 rad)",
    )
    parser.add_argument(
        "--publish-markers",
        action="store_true",
        help="Publish sole, support-hull, and COM debug markers for the GUI",
    )
    args = parser.parse_args()

    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and greater than 0")
    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and greater than 0")
    if not math.isfinite(args.contact_timeout) or args.contact_timeout <= 0.0:
        parser.error("--contact-timeout must be finite and greater than 0")
    if (
        not math.isfinite(args.angle_change_threshold)
        or args.angle_change_threshold <= 0.0
    ):
        parser.error("--angle-change-threshold must be finite and greater than 0")
    return args


def parse_pose_text(text: str | None):
    values = [float(value) for value in (text or "0 0 0 0 0 0").split()]
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Expected a finite six-value pose, got: {text!r}")
    return tuple(values)


def load_link_inertials(model_sdf: Path):
    try:
        model = ET.parse(model_sdf).getroot().find("model")
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"Could not read model SDF {model_sdf}: {error}") from error
    if model is None:
        raise RuntimeError(f"No <model> found in {model_sdf}")

    inertials = {}
    for link in model.findall("link"):
        link_name = link.get("name")
        inertial = link.find("inertial")
        if not link_name or inertial is None:
            continue
        mass_element = inertial.find("mass")
        if mass_element is None or mass_element.text is None:
            raise RuntimeError(f"Link {link_name} has no inertial mass")
        mass = float(mass_element.text)
        if not math.isfinite(mass) or mass <= 0.0:
            raise RuntimeError(f"Link {link_name} has invalid mass {mass}")

        pose_element = inertial.find("pose")
        if pose_element is not None and pose_element.get("relative_to"):
            raise RuntimeError(
                f"Link {link_name} inertial pose uses unsupported relative_to="
                f"{pose_element.get('relative_to')!r}"
            )
        x, y, z, _, _, _ = parse_pose_text(
            pose_element.text if pose_element is not None else None
        )
        inertials[link_name] = (mass, (x, y, z))

    if not inertials:
        raise RuntimeError(f"No link inertials found in {model_sdf}")
    return inertials


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def load_sole_boxes(model_sdf: Path):
    try:
        model = ET.parse(model_sdf).getroot().find("model")
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"Could not read model SDF {model_sdf}: {error}") from error
    if model is None:
        raise RuntimeError(f"No <model> found in {model_sdf}")

    sole_boxes = {}
    for side, collision_name in SOLE_COLLISION_NAMES.items():
        collision = model.find(f".//collision[@name='{collision_name}']")
        if collision is None:
            raise RuntimeError(f"Could not find collision {collision_name}")
        link = next(
            (
                candidate
                for candidate in model.findall("link")
                if collision in candidate.findall("collision")
            ),
            None,
        )
        if link is None or not link.get("name"):
            raise RuntimeError(f"Could not find parent link for {collision_name}")
        pose_element = collision.find("pose")
        if pose_element is not None and pose_element.get("relative_to"):
            raise RuntimeError(
                f"Collision {collision_name} uses unsupported relative_to="
                f"{pose_element.get('relative_to')!r}"
            )
        x, y, z, roll, pitch, yaw = parse_pose_text(
            pose_element.text if pose_element is not None else None
        )
        size_element = collision.find("geometry/box/size")
        if size_element is None or size_element.text is None:
            raise RuntimeError(f"Collision {collision_name} is not a box")
        size = tuple(float(value) for value in size_element.text.split())
        if len(size) != 3 or not all(value > 0.0 for value in size):
            raise RuntimeError(f"Collision {collision_name} has invalid size")
        sole_boxes[side] = {
            "link_name": link.get("name"),
            "pose": ((x, y, z), rpy_to_quaternion(roll, pitch, yaw)),
            "size": size,
        }
    return sole_boxes


def message_time_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nsec) * 1e-9


def unscoped_name(name: str) -> str:
    return name.split("::")[-1]


def pose_values(pose):
    return (
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def rotate_vector(vector, quaternion):
    vx, vy, vz = vector
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("Pose contains a zero-length quaternion")
    qx, qy, qz, qw = (value / norm for value in quaternion)

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def multiply_quaternions(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def compose_poses(parent_pose, child_pose):
    parent_position, parent_quaternion = parent_pose
    child_position, child_quaternion = child_pose
    rotated_position = rotate_vector(child_position, parent_quaternion)
    return (
        tuple(
            parent_position[index] + rotated_position[index]
            for index in range(3)
        ),
        multiply_quaternions(parent_quaternion, child_quaternion),
    )


def sole_world_geometry(world_link_poses, sole_boxes):
    geometry = {}
    for side, sole_box in sole_boxes.items():
        sole_world_pose = compose_poses(
            world_link_poses[sole_box["link_name"]],
            sole_box["pose"],
        )
        half_x = sole_box["size"][0] * 0.5
        half_y = sole_box["size"][1] * 0.5
        corners = []
        for local_x, local_y in (
            (-half_x, -half_y),
            (half_x, -half_y),
            (half_x, half_y),
            (-half_x, half_y),
        ):
            rotated = rotate_vector(
                (local_x, local_y, 0.0),
                sole_world_pose[1],
            )
            corners.append(
                (
                    sole_world_pose[0][0] + rotated[0],
                    sole_world_pose[0][1] + rotated[1],
                    sole_world_pose[0][2] + rotated[2],
                )
            )
        geometry[side] = {"pose": sole_world_pose, "corners": tuple(corners)}
    return geometry


def convex_hull(points):
    unique_points = sorted(set(points))
    if len(unique_points) <= 1:
        return unique_points

    def cross(origin, first, second):
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower = []
    for point in unique_points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)

    upper = []
    for point in reversed(unique_points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def point_in_convex_polygon(point, polygon, tolerance=1e-9):
    if len(polygon) < 3:
        return False
    sign = 0
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        cross = (
            (second[0] - first[0]) * (point[1] - first[1])
            - (second[1] - first[1]) * (point[0] - first[0])
        )
        if abs(cross) <= tolerance:
            continue
        current_sign = 1 if cross > 0.0 else -1
        if sign and current_sign != sign:
            return False
        sign = current_sign
    return True


def point_to_segment_distance(point, first, second):
    edge_x = second[0] - first[0]
    edge_y = second[1] - first[1]
    length_squared = edge_x * edge_x + edge_y * edge_y
    if length_squared <= 0.0:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    projection = (
        (point[0] - first[0]) * edge_x
        + (point[1] - first[1]) * edge_y
    ) / length_squared
    projection = min(1.0, max(0.0, projection))
    closest_x = first[0] + projection * edge_x
    closest_y = first[1] + projection * edge_y
    return math.hypot(point[0] - closest_x, point[1] - closest_y)


def signed_distance_to_polygon(point, polygon):
    if len(polygon) < 3:
        return math.nan
    distance = min(
        point_to_segment_distance(
            point,
            polygon[index],
            polygon[(index + 1) % len(polygon)],
        )
        for index in range(len(polygon))
    )
    return distance if point_in_convex_polygon(point, polygon) else -distance


def quaternion_to_rpy(quaternion):
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("Pose contains a zero-length quaternion")
    qx, qy, qz, qw = (value / norm for value in quaternion)

    sin_roll = 2.0 * (qw * qx + qy * qz)
    cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def empty_contact_state():
    return {
        "points": (),
        "force": (math.nan, math.nan, math.nan),
        "timestamp": None,
    }


def extract_contact_state(message, sole_collision_name: str):
    points = []
    total_force = [0.0, 0.0, 0.0]
    for contact in message.contact:
        collision1_is_sole = sole_collision_name in contact.collision1.name
        collision2_is_sole = sole_collision_name in contact.collision2.name
        if not collision1_is_sole and not collision2_is_sole:
            continue

        points.extend(
            (position.x, position.y, position.z)
            for position in contact.position
        )
        for wrench in contact.wrench:
            force = (
                wrench.body_1_wrench.force
                if collision1_is_sole
                else wrench.body_2_wrench.force
            )
            total_force[0] += force.x
            total_force[1] += force.y
            total_force[2] += force.z
    return {
        "points": tuple(points),
        "force": tuple(total_force),
        "timestamp": message_time_seconds(message),
    }


def resolve_world_link_poses(link_poses, inertials, model_name):
    if model_name not in link_poses:
        raise RuntimeError(f"Pose topic is missing model pose {model_name}")
    missing_links = sorted(set(inertials) - set(link_poses))
    if missing_links:
        raise RuntimeError(
            "Pose topic is missing inertial link(s): " + ", ".join(missing_links)
        )
    model_world_pose = link_poses[model_name]
    return {
        link_name: compose_poses(model_world_pose, link_poses[link_name])
        for link_name in inertials
    }


class BalanceStateSubscriber:
    def __init__(self, args):
        self.lock = threading.Lock()
        self.pose_sequence = 0
        self.pose_time = None
        self.link_poses = {}
        self.contact_states = {
            "left": empty_contact_state(),
            "right": empty_contact_state(),
        }

        self.node = Node()
        subscriptions = (
            self.node.subscribe(Pose_V, args.pose_topic, self.on_pose),
            self.node.subscribe(
                Contacts,
                args.left_contact_topic,
                lambda message: self.on_contacts("left", message),
            ),
            self.node.subscribe(
                Contacts,
                args.right_contact_topic,
                lambda message: self.on_contacts("right", message),
            ),
        )
        if not all(subscriptions):
            raise RuntimeError("One or more Gazebo topic subscriptions failed")

    def on_pose(self, message):
        link_poses = {
            unscoped_name(pose.name): pose_values(pose)
            for pose in message.pose
        }
        with self.lock:
            self.pose_sequence += 1
            self.pose_time = message_time_seconds(message)
            self.link_poses = link_poses

    def on_contacts(self, side: str, message):
        state = extract_contact_state(
            message,
            SOLE_COLLISION_NAMES[side],
        )
        with self.lock:
            self.contact_states[side] = state

    def snapshot(self):
        with self.lock:
            contacts = {
                side: {
                    "points": tuple(state["points"]),
                    "force": tuple(state["force"]),
                    "timestamp": state["timestamp"],
                }
                for side, state in self.contact_states.items()
            }
            return (
                self.pose_sequence,
                self.pose_time,
                dict(self.link_poses),
                contacts,
            )


def calculate_row(
    log_time,
    pose_time,
    link_poses,
    contacts,
    inertials,
    sole_boxes,
    model_name,
    force_stale_threshold,
):
    world_link_poses = resolve_world_link_poses(
        link_poses,
        inertials,
        model_name,
    )

    total_mass = 0.0
    weighted_com = [0.0, 0.0, 0.0]
    for link_name, (mass, local_com) in inertials.items():
        link_position, link_quaternion = world_link_poses[link_name]
        rotated_com = rotate_vector(local_com, link_quaternion)
        world_com = tuple(
            link_position[index] + rotated_com[index] for index in range(3)
        )
        total_mass += mass
        for index in range(3):
            weighted_com[index] += mass * world_com[index]
    com = tuple(value / total_mass for value in weighted_com)

    base_position, base_quaternion = world_link_poses["base_link"]
    base_rpy = quaternion_to_rpy(base_quaternion)

    sole_geometry = sole_world_geometry(world_link_poses, sole_boxes)
    support_polygon = convex_hull(
        [
            (corner[0], corner[1])
            for geometry in sole_geometry.values()
            for corner in geometry["corners"]
        ]
    )
    support_x_min = min(point[0] for point in support_polygon)
    support_x_max = max(point[0] for point in support_polygon)
    support_y_min = min(point[1] for point in support_polygon)
    support_y_max = max(point[1] for point in support_polygon)
    inside = int(point_in_convex_polygon((com[0], com[1]), support_polygon))
    support_signed_distance = signed_distance_to_polygon(
        (com[0], com[1]),
        support_polygon,
    )

    left_points = contacts["left"]["points"]
    right_points = contacts["right"]["points"]
    all_points = left_points + right_points
    left_force = contacts["left"]["force"]
    right_force = contacts["right"]["force"]

    def force_age(side):
        timestamp = contacts[side]["timestamp"]
        return max(0.0, pose_time - timestamp) if timestamp is not None else math.nan

    left_force_age = force_age("left")
    right_force_age = force_age("right")

    def mean_coordinate(points, index):
        return (
            sum(point[index] for point in points) / len(points)
            if points
            else math.nan
        )

    return (
        log_time,
        *com,
        *base_position,
        *base_rpy,
        support_x_min,
        support_x_max,
        support_y_min,
        support_y_max,
        inside,
        int(bool(left_points)),
        int(bool(right_points)),
        len(all_points),
        *left_force,
        *right_force,
        mean_coordinate(left_points, 0),
        mean_coordinate(left_points, 1),
        mean_coordinate(right_points, 0),
        mean_coordinate(right_points, 1),
        right_force[2] - left_force[2],
        left_force_age,
        right_force_age,
        int(not math.isfinite(left_force_age) or left_force_age > force_stale_threshold),
        int(not math.isfinite(right_force_age) or right_force_age > force_stale_threshold),
        support_signed_distance,
    )


def build_geometry_diagnostics(link_poses, inertials, sole_boxes, model_name, com_xy):
    world_link_poses = resolve_world_link_poses(link_poses, inertials, model_name)
    sole_geometry = sole_world_geometry(world_link_poses, sole_boxes)
    support_polygon = convex_hull(
        [
            (corner[0], corner[1])
            for geometry in sole_geometry.values()
            for corner in geometry["corners"]
        ]
    )
    return {
        "world_link_poses": world_link_poses,
        "sole_geometry": sole_geometry,
        "support_polygon": support_polygon,
        "signed_distance": signed_distance_to_polygon(com_xy, support_polygon),
    }


def print_initial_geometry(diagnostics, sole_boxes, com_xy):
    print("\n[INITIAL SUPPORT GEOMETRY]")
    print(f"COM projection: x={com_xy[0]:+.9f}, y={com_xy[1]:+.9f}")
    for side in ("left", "right"):
        link_name = sole_boxes[side]["link_name"]
        link_position, link_quaternion = diagnostics["world_link_poses"][link_name]
        link_rpy = quaternion_to_rpy(link_quaternion)
        sole_position, sole_quaternion = diagnostics["sole_geometry"][side]["pose"]
        sole_rpy = quaternion_to_rpy(sole_quaternion)
        print(
            f"{side} foot link pose: "
            f"xyz=({link_position[0]:+.9f}, {link_position[1]:+.9f}, "
            f"{link_position[2]:+.9f}), "
            f"rpy=({link_rpy[0]:+.9f}, {link_rpy[1]:+.9f}, "
            f"{link_rpy[2]:+.9f})"
        )
        print(
            f"{side} sole box pose: "
            f"xyz=({sole_position[0]:+.9f}, {sole_position[1]:+.9f}, "
            f"{sole_position[2]:+.9f}), "
            f"rpy=({sole_rpy[0]:+.9f}, {sole_rpy[1]:+.9f}, "
            f"{sole_rpy[2]:+.9f})"
        )
        for index, corner in enumerate(
            diagnostics["sole_geometry"][side]["corners"]
        ):
            print(
                f"  {side} corner[{index}]: "
                f"x={corner[0]:+.9f}, y={corner[1]:+.9f}, "
                f"z={corner[2]:+.9f}"
            )
    print("support polygon vertices (counter-clockwise):")
    for index, vertex in enumerate(diagnostics["support_polygon"]):
        print(f"  hull[{index}]: x={vertex[0]:+.9f}, y={vertex[1]:+.9f}")
    print(
        "COM signed distance to support boundary: "
        f"{diagnostics['signed_distance']:+.9f} m "
        "(positive=inside, negative=outside)"
    )


class DebugMarkerPublisher:
    def __init__(self):
        self.node = Node()
        self.publisher = self.node.advertise("/marker", Marker)
        if not self.publisher.valid():
            raise RuntimeError("Could not advertise /marker")

    @staticmethod
    def line_marker(marker_id, points, color):
        marker = Marker()
        marker.action = Marker.ADD_MODIFY
        marker.ns = "step_com_balance"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.visibility = Marker.GUI
        marker.scale.x = 0.004
        marker.material.diffuse.r = color[0]
        marker.material.diffuse.g = color[1]
        marker.material.diffuse.b = color[2]
        marker.material.diffuse.a = 1.0
        for x, y, z in (*points, points[0]):
            point = marker.point.add()
            point.x = x
            point.y = y
            point.z = z
        return marker

    def publish(self, diagnostics, com_xy):
        geometry = diagnostics["sole_geometry"]
        markers = (
            self.line_marker(1, geometry["left"]["corners"], (0.1, 0.4, 1.0)),
            self.line_marker(2, geometry["right"]["corners"], (1.0, 0.4, 0.1)),
        )
        hull_z = max(
            corner[2]
            for side_geometry in geometry.values()
            for corner in side_geometry["corners"]
        ) + 0.01
        hull_points = tuple(
            (vertex[0], vertex[1], hull_z)
            for vertex in diagnostics["support_polygon"]
        )
        for marker in (*markers, self.line_marker(3, hull_points, (0.1, 1.0, 0.2))):
            self.publisher.publish(marker)

        com_marker = Marker()
        com_marker.action = Marker.ADD_MODIFY
        com_marker.ns = "step_com_balance"
        com_marker.id = 4
        com_marker.type = Marker.SPHERE
        com_marker.visibility = Marker.GUI
        com_marker.pose.position.x = com_xy[0]
        com_marker.pose.position.y = com_xy[1]
        com_marker.pose.position.z = hull_z + 0.01
        com_marker.pose.orientation.w = 1.0
        com_marker.scale.x = 0.025
        com_marker.scale.y = 0.025
        com_marker.scale.z = 0.025
        com_marker.material.diffuse.r = 1.0
        com_marker.material.diffuse.g = 0.1
        com_marker.material.diffuse.b = 0.8
        com_marker.material.diffuse.a = 1.0
        self.publisher.publish(com_marker)

def print_initial_summary(rows, angle_change_threshold):
    if not rows:
        print("[0-0.25 s SUMMARY] No samples were recorded.")
        return

    def values(column):
        return [
            float(row[column])
            for row in rows
            if row[column] != "" and math.isfinite(float(row[column]))
        ]

    def mean(column):
        samples = values(column)
        return sum(samples) / len(samples) if samples else math.nan

    first = rows[0]
    last = rows[-1]
    force_difference = values("right_minus_left_fz")
    known_inside = [row["com_inside_support"] for row in rows if row["com_inside_support"] != ""]
    inside_ratio = (
        sum(int(value) for value in known_inside) / len(known_inside)
        if known_inside
        else math.nan
    )

    print("\n[0-0.25 s SUMMARY]")
    print(f"samples: {len(rows)}, end_time: {float(last['time']):.3f} s")
    print(
        "COM start -> end [m]: "
        f"({float(first['com_x']):+.5f}, {float(first['com_y']):+.5f}, "
        f"{float(first['com_z']):+.5f}) -> "
        f"({float(last['com_x']):+.5f}, {float(last['com_y']):+.5f}, "
        f"{float(last['com_z']):+.5f})"
    )
    for axis in ("roll", "pitch", "yaw"):
        column = f"base_{axis}"
        samples = values(column)
        delta = float(last[column]) - float(first[column])
        maximum = max((abs(value) for value in samples), default=math.nan)
        initial_value = float(first[column])
        change_start = next(
            (
                float(row["time"])
                for row in rows
                if abs(float(row[column]) - initial_value)
                >= angle_change_threshold
            ),
            math.nan,
        )
        print(
            f"base_{axis}: start={float(first[column]):+.6f}, "
            f"end={float(last[column]):+.6f}, delta={delta:+.6f}, "
            f"max_abs={maximum:.6f} rad, change_start={change_start:.3f} s"
        )
    print(
        "mean vertical force [N]: "
        f"left={mean('left_fz'):+.4f}, right={mean('right_fz'):+.4f}"
    )
    print(
        "right_fz-left_fz [N]: "
        f"mean={mean('right_minus_left_fz'):+.4f}, "
        f"min={min(force_difference, default=math.nan):+.4f}, "
        f"max={max(force_difference, default=math.nan):+.4f}"
    )
    print(
        "mean contact XY [m]: "
        f"left=({mean('left_contact_x_mean'):+.5f}, "
        f"{mean('left_contact_y_mean'):+.5f}), "
        f"right=({mean('right_contact_x_mean'):+.5f}, "
        f"{mean('right_contact_y_mean'):+.5f})"
    )
    print(
        "force message age [s]: "
        f"left mean/max={mean('left_force_age'):.4f}/"
        f"{max(values('left_force_age'), default=math.nan):.4f}, "
        f"right mean/max={mean('right_force_age'):.4f}/"
        f"{max(values('right_force_age'), default=math.nan):.4f}"
    )
    print(
        "force stale samples: "
        f"left={sum(int(row['left_force_stale']) for row in rows)}/"
        f"{len(rows)}, right={sum(int(row['right_force_stale']) for row in rows)}/"
        f"{len(rows)}"
    )
    print(
        f"COM inside support: start={first['com_inside_support']}, "
        f"end={last['com_inside_support']}, ratio={inside_ratio:.3f}"
    )


def main() -> int:
    args = parse_args()
    try:
        inertials = load_link_inertials(args.model_sdf)
        sole_boxes = load_sole_boxes(args.model_sdf)
        subscriber = BalanceStateSubscriber(args)
        marker_publisher = DebugMarkerPublisher() if args.publish_markers else None
    except (ImportError, RuntimeError, ValueError) as error:
        print(f"[ERROR] {error}")
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda _signum, _frame: stop.set())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: stop.set())

    total_mass = sum(mass for mass, _ in inertials.values())
    print("[GAZEBO FREE-BASE COM / BALANCE LOGGER]")
    print(f"model_sdf: {args.model_sdf}")
    print(f"model_name: {args.model_name}")
    print(f"links: {len(inertials)}, total_mass: {total_mass:.6f} kg")
    print("support_polygon: 8 sole-box corners -> world XY convex hull")
    print(f"pose_topic: {args.pose_topic}")
    print(f"left_contact_topic: {args.left_contact_topic}")
    print(f"right_contact_topic: {args.right_contact_topic}")
    print(f"debug_markers: {'enabled on /marker' if marker_publisher else 'disabled'}")
    print(f"duration: {args.duration:.6f} s, dt: {args.dt:.6f} s")
    print(f"output: {args.output}")
    print("Waiting for simulation pose updates; start Gazebo physics when ready.")

    rows_written = 0
    last_progress_bucket = -1
    initial_summary_rows = []
    printed_initial_geometry = False

    try:
        with args.output.open("w", newline="", encoding="utf-8") as output_file:
            writer = csv.writer(output_file)
            writer.writerow(CSV_COLUMNS)

            initial_sequence = -1
            initial_sim_time = None
            while not stop.is_set():
                pose_sequence, sim_time, _, _ = subscriber.snapshot()
                if sim_time is None:
                    time.sleep(0.001)
                    continue
                if initial_sim_time is None:
                    initial_sequence = pose_sequence
                    initial_sim_time = sim_time
                elif (
                    pose_sequence != initial_sequence
                    and sim_time > initial_sim_time + 1e-9
                ):
                    break
                time.sleep(0.001)

            if stop.is_set():
                print("[STOP] Interrupted before physics started.")
                return 130

            logging_start_wall = time.monotonic()
            logging_start_sim_time = sim_time
            previous_pose_sequence = -1
            sample_count = max(1, math.ceil(args.duration / args.dt))

            for sample_index in range(sample_count):
                if stop.is_set():
                    break
                sample_time = logging_start_wall + sample_index * args.dt
                remaining = sample_time - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)

                pose_sequence, sim_time, link_poses, contacts = subscriber.snapshot()
                elapsed = sample_index * args.dt
                pose_updated = int(pose_sequence != previous_pose_sequence)
                previous_pose_sequence = pose_sequence

                try:
                    row = calculate_row(
                        elapsed,
                        sim_time,
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
                source_sim_time = sim_time - logging_start_sim_time
                complete_row = (*row, source_sim_time, pose_updated)
                writer.writerow(complete_row)
                output_file.flush()
                rows_written += 1
                if elapsed <= 0.25 + 1e-9:
                    initial_summary_rows.append(
                        dict(zip(CSV_COLUMNS, complete_row))
                    )

                diagnostics = build_geometry_diagnostics(
                    link_poses,
                    inertials,
                    sole_boxes,
                    args.model_name,
                    (row[1], row[2]),
                )
                if not printed_initial_geometry:
                    print_initial_geometry(
                        diagnostics,
                        sole_boxes,
                        (row[1], row[2]),
                    )
                    printed_initial_geometry = True
                if marker_publisher is not None:
                    marker_publisher.publish(diagnostics, (row[1], row[2]))

                progress_bucket = int(elapsed / 0.25)
                if progress_bucket != last_progress_bucket:
                    inside_label = (
                        str(row[14]) if row[14] != "" else "unknown"
                    )
                    print(
                        f"[PROGRESS] t={elapsed:.3f} s, "
                        f"COM=({row[1]:+.4f}, {row[2]:+.4f}, {row[3]:+.4f}), "
                        f"inside_support={inside_label}, "
                        f"right-left Fz={row[28]:+.3f} N"
                    )
                    last_progress_bucket = progress_bucket
    except OSError as error:
        print(f"[ERROR] Could not write {args.output}: {error}")
        return 1
    except KeyboardInterrupt:
        stop.set()

    print_initial_summary(initial_summary_rows, args.angle_change_threshold)
    print(f"[DONE] Wrote {rows_written} samples to {args.output}")
    return 0 if rows_written else 1


if __name__ == "__main__":
    raise SystemExit(main())
