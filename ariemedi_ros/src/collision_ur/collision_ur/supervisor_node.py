#!/usr/bin/env python3

from __future__ import annotations

import copy
import json
import threading
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float64, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from collision_ur.configuration import (
    load_config,
    load_right_to_left_transform,
)
from collision_ur.geometry import (
    clamp_position,
    closest_points_on_segments,
    interpolate_transform,
    matrix_quaternion,
    pose_matrix,
    transform_segment,
)


def _pose_from_matrix(transform: np.ndarray) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = transform[:3, 3]
    quaternion = matrix_quaternion(transform[:3, :3])
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion
    return pose


def _cylinder_marker(
    marker_id: int,
    namespace: str,
    frame_id: str,
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    color,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    direction = end - start
    length = float(np.linalg.norm(direction))
    center = 0.5 * (start + end)
    transform = np.eye(4)
    transform[:3, 3] = center
    if length > 1.0e-9:
        z_axis = direction / length
        reference = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(reference, z_axis))) > 0.9:
            reference = np.array([0.0, 1.0, 0.0])
        y_axis = np.cross(z_axis, reference)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        transform[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
    marker.pose = _pose_from_matrix(transform)
    marker.scale.x = 2.0 * radius
    marker.scale.y = 2.0 * radius
    marker.scale.z = max(length, 1.0e-4)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


class CollisionSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("collision_supervisor")
        self.declare_parameter(
            "config_file",
            "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros/"
            "src/collision_ur/config/collision_regions.yaml",
        )
        self.declare_parameter(
            "base_transform_file",
            "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/dual_arm_cali/"
            "dual_base_left_to_right_v2.json",
        )
        self.declare_parameter("left_actual_topic", "/left_arm/eef_pose")
        self.declare_parameter("right_actual_topic", "/right_arm/eef_pose")
        self.declare_parameter("left_target_topic", "/left_arm/target_pose")
        self.declare_parameter("right_target_topic", "/right_arm/target_pose")
        self.declare_parameter("left_safe_topic", "/left_arm/safe_target_pose")
        self.declare_parameter("right_safe_topic", "/right_arm/safe_target_pose")
        self.declare_parameter("marker_rate_hz", 20.0)
        self.declare_parameter("path_sample_resolution_m", 0.01)
        self.declare_parameter("minimum_path_samples", 10)
        config_file = str(Path(self.get_parameter("config_file").value).expanduser())
        self.config = load_config(config_file)
        self.config_lock = threading.RLock()
        transform_path = self.get_parameter("base_transform_file").value
        try:
            self.right_to_global = load_right_to_left_transform(transform_path)
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f"Failed to load dual-base transform: {exc}") from exc

        self.actual: Dict[str, Optional[PoseStamped]] = {"left": None, "right": None}
        self.last_safe: Dict[str, Optional[PoseStamped]] = {"left": None, "right": None}
        self.latest_target: Dict[str, Optional[PoseStamped]] = {"left": None, "right": None}
        self.last_distance = float("inf")
        self.last_threshold = 0.0
        self.last_closest = None
        self.last_safe_state = True
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        left_actual_topic = self.get_parameter("left_actual_topic").value
        right_actual_topic = self.get_parameter("right_actual_topic").value
        left_target_topic = self.get_parameter("left_target_topic").value
        right_target_topic = self.get_parameter("right_target_topic").value
        left_safe_topic = self.get_parameter("left_safe_topic").value
        right_safe_topic = self.get_parameter("right_safe_topic").value

        self.create_subscription(
            PoseStamped, left_actual_topic, lambda msg: self._actual_cb("left", msg), 20
        )
        self.create_subscription(
            PoseStamped, right_actual_topic, lambda msg: self._actual_cb("right", msg), 20
        )
        self.create_subscription(
            PoseStamped, left_target_topic, lambda msg: self._target_cb("left", msg), 20
        )
        self.create_subscription(
            PoseStamped, right_target_topic, lambda msg: self._target_cb("right", msg), 20
        )

        self.safe_publishers = {
            "left": self.create_publisher(PoseStamped, left_safe_topic, 20),
            "right": self.create_publisher(PoseStamped, right_safe_topic, 20),
        }
        self.safe_state_pub = self.create_publisher(Bool, "/collision_ur/is_safe", 20)
        self.distance_pub = self.create_publisher(
            Float64, "/collision_ur/centerline_distance", 20
        )
        self.threshold_pub = self.create_publisher(
            Float64, "/collision_ur/required_distance", 20
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/collision_ur/markers", QoSProfile(depth=1)
        )
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/collision_ur/config_update", self._config_cb, config_qos
        )
        rate = max(1.0, float(self.get_parameter("marker_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_markers)
        self.get_logger().info(
            "Collision supervisor ready: wrist_2 + wrist_3 + tool envelopes; "
            "tool is TCP +Z from 0.00 m to 0.20 m"
        )

    def _config_cb(self, message: String) -> None:
        try:
            update = json.loads(message.data)
            with self.config_lock:
                self.config = update
            self.get_logger().info("Applied collision-region update from RViz editor")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Rejected invalid configuration update: {exc}")

    def _actual_cb(self, arm: str, message: PoseStamped) -> None:
        self.actual[arm] = copy.deepcopy(message)
        if self.last_safe[arm] is None:
            self.last_safe[arm] = copy.deepcopy(message)

    def _arm_transform(self, arm: str) -> np.ndarray:
        return np.eye(4) if arm == "left" else self.right_to_global

    def _pose_in_global(self, arm: str, pose: PoseStamped) -> np.ndarray:
        return self._arm_transform(arm) @ pose_matrix(
            pose.pose.position, pose.pose.orientation
        )

    def _clamp_to_workspace(self, arm: str, message: PoseStamped) -> PoseStamped:
        result = copy.deepcopy(message)
        with self.config_lock:
            minimum = np.asarray(self.config[arm]["workspace_min"], dtype=np.float64)
            maximum = np.asarray(self.config[arm]["workspace_max"], dtype=np.float64)
        position = np.array(
            [
                result.pose.position.x,
                result.pose.position.y,
                result.pose.position.z,
            ],
            dtype=np.float64,
        )
        clamped = clamp_position(position, minimum, maximum)
        result.pose.position.x, result.pose.position.y, result.pose.position.z = clamped
        return result

    def _actual_bodies(self, arm: str, pose: PoseStamped):
        tcp_global = self._pose_in_global(arm, pose)
        with self.config_lock:
            body_configs = copy.deepcopy(self.config[arm]["bodies"])
            global_frame = self.config["global_frame"]
        bodies = []
        for name, body in body_configs.items():
            start = np.asarray(body["segment_start"], dtype=np.float64)
            end = np.asarray(body["segment_end"], dtype=np.float64)
            if body["frame"] == "tcp":
                transform = tcp_global
            else:
                try:
                    stamped = self.tf_buffer.lookup_transform(
                        global_frame, body["frame"], Time()
                    )
                except TransformException as exc:
                    self.get_logger().warn(
                        f"Waiting for {global_frame} -> {body['frame']} TF: {exc}",
                        throttle_duration_sec=2.0,
                    )
                    return None
                transform = pose_matrix(
                    stamped.transform.translation, stamped.transform.rotation
                )
            segment_start, segment_end = transform_segment(transform, start, end)
            bodies.append(
                {
                    "name": name,
                    "radius": float(body["radius"]),
                    "start": segment_start,
                    "end": segment_end,
                }
            )
        return bodies

    def _body_templates_tcp(self, arm: str, pose: PoseStamped):
        actual_bodies = self._actual_bodies(arm, pose)
        if actual_bodies is None:
            return None
        inverse_tcp = np.linalg.inv(self._pose_in_global(arm, pose))
        templates = []
        for body in actual_bodies:
            start, end = transform_segment(
                inverse_tcp, body["start"], body["end"]
            )
            templates.append(
                {
                    "name": body["name"],
                    "radius": body["radius"],
                    "start": start,
                    "end": end,
                }
            )
        return templates

    @staticmethod
    def _bodies_from_tcp(tcp_global: np.ndarray, templates):
        bodies = []
        for template in templates:
            start, end = transform_segment(
                tcp_global, template["start"], template["end"]
            )
            bodies.append(
                {
                    "name": template["name"],
                    "radius": template["radius"],
                    "start": start,
                    "end": end,
                }
            )
        return bodies

    def _closest_body_pair(self, left_bodies, right_bodies):
        with self.config_lock:
            clearance = float(self.config["clearance"])
        closest_pair = None
        for left_body in left_bodies:
            for right_body in right_bodies:
                result = closest_points_on_segments(
                    left_body["start"],
                    left_body["end"],
                    right_body["start"],
                    right_body["end"],
                )
                threshold = (
                    left_body["radius"] + right_body["radius"] + clearance
                )
                signed_clearance = result.distance - threshold
                if (
                    closest_pair is None
                    or signed_clearance < closest_pair["signed_clearance"]
                ):
                    closest_pair = {
                        "result": result,
                        "threshold": threshold,
                        "signed_clearance": signed_clearance,
                        "left_name": left_body["name"],
                        "right_name": right_body["name"],
                    }
        return closest_pair

    def _check_candidate_path(
        self, arm: str, candidate: PoseStamped, other_pose: PoseStamped
    ):
        start_pose = self.latest_target[arm] or self.actual[arm]
        if start_pose is None:
            return None, False
        moving_start = self._pose_in_global(arm, start_pose)
        moving_end = self._pose_in_global(arm, candidate)
        translation = float(np.linalg.norm(moving_end[:3, 3] - moving_start[:3, 3]))
        resolution = max(
            0.001, float(self.get_parameter("path_sample_resolution_m").value)
        )
        sample_count = max(
            int(self.get_parameter("minimum_path_samples").value),
            int(np.ceil(translation / resolution)),
        )
        other_transform = self._pose_in_global(
            "right" if arm == "left" else "left", other_pose
        )
        other_arm = "right" if arm == "left" else "left"
        moving_templates = self._body_templates_tcp(arm, self.actual[arm])
        other_templates = self._body_templates_tcp(other_arm, self.actual[other_arm])
        if moving_templates is None or other_templates is None:
            return None, False
        other_bodies = self._bodies_from_tcp(other_transform, other_templates)
        minimum_pair = None
        for fraction in np.linspace(0.0, 1.0, sample_count + 1):
            moving_transform = interpolate_transform(
                moving_start, moving_end, float(fraction)
            )
            moving_bodies = self._bodies_from_tcp(
                moving_transform, moving_templates
            )
            if arm == "left":
                pair = self._closest_body_pair(moving_bodies, other_bodies)
            else:
                pair = self._closest_body_pair(other_bodies, moving_bodies)
            if (
                minimum_pair is None
                or pair["signed_clearance"] < minimum_pair["signed_clearance"]
            ):
                minimum_pair = pair
            if pair["signed_clearance"] < 0.0:
                return pair, False
        return minimum_pair, True

    def _target_cb(self, arm: str, message: PoseStamped) -> None:
        expected_frame = f"{arm}_base"
        if message.header.frame_id and message.header.frame_id != expected_frame:
            self.get_logger().error(
                f"Rejected {arm} target in frame '{message.header.frame_id}'; "
                f"expected '{expected_frame}'"
            )
            return

        candidate = self._clamp_to_workspace(arm, message)
        other = "right" if arm == "left" else "left"
        other_pose = self.latest_target[other] or self.actual[other]
        if other_pose is None:
            self.get_logger().warn(
                f"Cannot validate {arm} target before receiving {other} pose"
            )
            return

        closest_pair, path_is_safe = self._check_candidate_path(
            arm, candidate, other_pose
        )
        if closest_pair is None:
            return
        closest = closest_pair["result"]
        threshold = closest_pair["threshold"]
        is_safe = path_is_safe and closest_pair["signed_clearance"] >= 0.0
        self.last_distance = closest.distance
        self.last_threshold = threshold
        self.last_closest = closest
        self.last_safe_state = is_safe
        self._publish_status()

        if is_safe:
            self.latest_target[arm] = copy.deepcopy(candidate)
            self.last_safe[arm] = copy.deepcopy(candidate)
            output = candidate
        else:
            output = self.last_safe[arm]
            self.get_logger().warn(
                f"Blocked {arm} target at "
                f"{closest_pair['left_name']} <-> "
                f"{closest_pair['right_name']}: centerline distance "
                f"{closest.distance:.3f} m < required {threshold:.3f} m"
            )
        if output is not None:
            output = copy.deepcopy(output)
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = expected_frame
            self.safe_publishers[arm].publish(output)

    def _publish_status(self) -> None:
        safe_message = Bool()
        safe_message.data = self.last_safe_state
        self.safe_state_pub.publish(safe_message)
        distance_message = Float64()
        distance_message.data = self.last_distance
        self.distance_pub.publish(distance_message)
        threshold_message = Float64()
        threshold_message.data = self.last_threshold
        self.threshold_pub.publish(threshold_message)

    def _workspace_marker(self, arm: str, marker_id: int) -> Marker:
        with self.config_lock:
            minimum = np.asarray(self.config[arm]["workspace_min"], dtype=np.float64)
            maximum = np.asarray(self.config[arm]["workspace_max"], dtype=np.float64)
        center = 0.5 * (minimum + maximum)
        size = maximum - minimum
        transform = self._arm_transform(arm).copy()
        transform[:3, 3] = transform[:3, :3] @ center + transform[:3, 3]
        marker = Marker()
        marker.header.frame_id = self.config["global_frame"]
        marker.ns = "workspaces"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = _pose_from_matrix(transform)
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r = 0.15 if arm == "left" else 0.95
        marker.color.g = 0.75
        marker.color.b = 0.95 if arm == "left" else 0.15
        marker.color.a = 0.10
        return marker

    def _publish_markers(self) -> None:
        if self.actual["left"] is None or self.actual["right"] is None:
            return
        left_bodies = self._actual_bodies("left", self.actual["left"])
        right_bodies = self._actual_bodies("right", self.actual["right"])
        if left_bodies is None or right_bodies is None:
            return
        closest_pair = self._closest_body_pair(left_bodies, right_bodies)
        closest = closest_pair["result"]
        threshold = closest_pair["threshold"]
        safe = closest_pair["signed_clearance"] >= 0.0
        with self.config_lock:
            global_frame = self.config["global_frame"]
        markers = [
            self._workspace_marker("left", 0),
            self._workspace_marker("right", 1),
        ]
        marker_id = 0
        for arm, bodies in (("left", left_bodies), ("right", right_bodies)):
            for body in bodies:
                is_closest_body = (
                    body["name"]
                    == closest_pair[f"{arm}_name"]
                )
                if not safe and is_closest_body:
                    color = (1.0, 0.05, 0.05, 0.82)
                elif arm == "left":
                    color = (0.1, 0.85, 0.2, 0.55)
                else:
                    color = (0.95, 0.75, 0.05, 0.55)
                markers.append(
                    _cylinder_marker(
                        marker_id,
                        f"{arm}_distal_envelopes",
                        global_frame,
                        body["start"],
                        body["end"],
                        body["radius"],
                        color,
                    )
                )
                marker_id += 1
        line = Marker()
        line.header.frame_id = global_frame
        line.ns = "minimum_distance"
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = 0.008
        line.color.r = 0.1 if safe else 1.0
        line.color.g = 0.9 if safe else 0.05
        line.color.b = 0.9 if safe else 0.05
        line.color.a = 1.0
        for point_np in (closest.point_a, closest.point_b):
            point = Point()
            point.x, point.y, point.z = point_np
            line.points.append(point)
        markers.append(line)

        text = Marker()
        text.header.frame_id = global_frame
        text.ns = "status"
        text.id = 0
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        midpoint = 0.5 * (closest.point_a + closest.point_b)
        text.pose.position.x, text.pose.position.y, text.pose.position.z = midpoint
        text.pose.position.z += 0.08
        text.pose.orientation.w = 1.0
        text.scale.z = 0.045
        text.color.r = 0.1 if safe else 1.0
        text.color.g = 1.0 if safe else 0.1
        text.color.b = 0.1
        text.color.a = 1.0
        text.text = (
            f"{closest_pair['left_name']} <-> "
            f"{closest_pair['right_name']}: "
            f"d={closest.distance:.3f} m, limit={threshold:.3f} m "
            f"({'SAFE' if safe else 'BLOCKED'})"
        )
        markers.append(text)
        now = self.get_clock().now().to_msg()
        for marker in markers:
            marker.header.stamp = now
        self.marker_pub.publish(MarkerArray(markers=markers))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
