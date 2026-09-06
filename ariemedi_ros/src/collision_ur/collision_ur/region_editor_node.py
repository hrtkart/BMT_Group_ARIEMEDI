#!/usr/bin/env python3

from __future__ import annotations

import json
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

from collision_ur.configuration import (
    load_config,
    load_right_to_left_transform,
    save_config,
)
from collision_ur.geometry import matrix_quaternion, pose_matrix


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


def _make_axis_control(name: str, mode: int, axis: str) -> InteractiveMarkerControl:
    control = InteractiveMarkerControl()
    control.name = f"{name}_{axis}"
    control.interaction_mode = mode
    control.orientation_mode = InteractiveMarkerControl.FIXED
    if axis == "x":
        control.orientation.x = 1.0
        control.orientation.w = 1.0
    elif axis == "y":
        control.orientation.y = 1.0
        control.orientation.w = 1.0
    else:
        control.orientation.z = 1.0
        control.orientation.w = 1.0
    return control


class RegionEditor(Node):
    def __init__(self) -> None:
        super().__init__("collision_region_editor")
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
        self.config_file = self.get_parameter("config_file").value
        self.config = load_config(self.config_file)
        self.right_to_global = load_right_to_left_transform(
            self.get_parameter("base_transform_file").value
        )
        self.lock = threading.RLock()
        self.actual = {"left": None, "right": None}
        self.server = InteractiveMarkerServer(self, "collision_region_editor")
        self.tool_menus = {}
        self.workspace_menus = {}
        self.clearance_menu = MenuHandler()
        self.markers_created = False

        self.create_subscription(
            PoseStamped,
            self.get_parameter("left_actual_topic").value,
            lambda msg: self._actual_cb("left", msg),
            20,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("right_actual_topic").value,
            lambda msg: self._actual_cb("right", msg),
            20,
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.config_pub = self.create_publisher(
            String, "/collision_ur/config_update", qos
        )
        self.create_service(Trigger, "/collision_ur/save_config", self._save_cb)
        self.create_timer(0.1, self._refresh)
        self._publish_config()
        self.get_logger().info(
            "RViz editor ready. Add an InteractiveMarkers display and select "
            "'/collision_region_editor/update'."
        )

    def _actual_cb(self, arm: str, message: PoseStamped) -> None:
        self.actual[arm] = message

    def _arm_transform(self, arm: str) -> np.ndarray:
        return np.eye(4) if arm == "left" else self.right_to_global

    def _tcp_global(self, arm: str) -> np.ndarray:
        message = self.actual[arm]
        return self._arm_transform(arm) @ pose_matrix(
            message.pose.position, message.pose.orientation
        )

    def _tool_global(self, arm: str) -> np.ndarray:
        with self.lock:
            tool = self.config[arm]["bodies"]["tool"]
            start = np.asarray(tool["segment_start"], dtype=np.float64)
            end = np.asarray(tool["segment_end"], dtype=np.float64)
        direction = end - start
        length = float(np.linalg.norm(direction))
        transform_tcp = np.eye(4)
        transform_tcp[:3, 3] = 0.5 * (start + end)
        if length > 1.0e-9:
            z_axis = direction / length
            reference = np.array([1.0, 0.0, 0.0])
            if abs(float(np.dot(reference, z_axis))) > 0.9:
                reference = np.array([0.0, 1.0, 0.0])
            y_axis = np.cross(z_axis, reference)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)
            transform_tcp[:3, :3] = np.column_stack((x_axis, y_axis, z_axis))
        return self._tcp_global(arm) @ transform_tcp

    def _workspace_global(self, arm: str):
        with self.lock:
            minimum = np.asarray(self.config[arm]["workspace_min"], dtype=np.float64)
            maximum = np.asarray(self.config[arm]["workspace_max"], dtype=np.float64)
        center = 0.5 * (minimum + maximum)
        size = maximum - minimum
        transform = self._arm_transform(arm).copy()
        transform[:3, 3] = transform[:3, :3] @ center + transform[:3, 3]
        return transform, size

    def _tool_marker(self, arm: str) -> InteractiveMarker:
        with self.lock:
            tool = self.config[arm]["bodies"]["tool"]
            start = np.asarray(tool["segment_start"], dtype=np.float64)
            end = np.asarray(tool["segment_end"], dtype=np.float64)
            radius = float(tool["radius"])
            frame_id = self.config["global_frame"]
        marker = InteractiveMarker()
        marker.header.frame_id = frame_id
        marker.name = f"{arm}_tool_cylinder"
        marker.description = f"{arm} tool envelope"
        marker.pose = _pose_from_matrix(self._tool_global(arm))
        marker.scale = max(0.25, float(np.linalg.norm(end - start)) + 0.12)

        visual = Marker()
        visual.type = Marker.CYLINDER
        visual.scale.x = 2.0 * radius
        visual.scale.y = 2.0 * radius
        visual.scale.z = max(float(np.linalg.norm(end - start)), 1.0e-4)
        visual.color.r = 0.1 if arm == "left" else 1.0
        visual.color.g = 0.85
        visual.color.b = 0.2
        visual.color.a = 0.65
        visible = InteractiveMarkerControl()
        visible.always_visible = True
        visible.markers.append(visual)
        marker.controls.append(visible)
        for axis in ("x", "y", "z"):
            marker.controls.append(
                _make_axis_control("move", InteractiveMarkerControl.MOVE_AXIS, axis)
            )
            marker.controls.append(
                _make_axis_control("rotate", InteractiveMarkerControl.ROTATE_AXIS, axis)
            )
        return marker

    def _workspace_marker(self, arm: str) -> InteractiveMarker:
        transform, size = self._workspace_global(arm)
        marker = InteractiveMarker()
        marker.header.frame_id = self.config["global_frame"]
        marker.name = f"{arm}_workspace"
        marker.description = f"{arm} Cartesian workspace"
        marker.pose = _pose_from_matrix(transform)
        marker.scale = max(0.3, float(np.max(size)) * 0.35)
        visual = Marker()
        visual.type = Marker.CUBE
        visual.scale.x, visual.scale.y, visual.scale.z = size
        visual.color.r = 0.15 if arm == "left" else 0.95
        visual.color.g = 0.7
        visual.color.b = 0.95 if arm == "left" else 0.15
        visual.color.a = 0.18
        visible = InteractiveMarkerControl()
        visible.always_visible = True
        visible.markers.append(visual)
        marker.controls.append(visible)
        for axis in ("x", "y", "z"):
            marker.controls.append(
                _make_axis_control("move", InteractiveMarkerControl.MOVE_AXIS, axis)
            )
        return marker

    def _clearance_marker(self) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header.frame_id = self.config["global_frame"]
        marker.name = "safety_clearance"
        marker.description = "Right-click to adjust clearance"
        marker.scale = 0.25
        marker.pose.position.x = 0.15
        marker.pose.position.y = -0.25
        marker.pose.position.z = 0.85
        marker.pose.orientation.w = 1.0
        text = Marker()
        text.type = Marker.TEXT_VIEW_FACING
        text.scale.z = 0.05
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"clearance = {float(self.config['clearance']):.3f} m"
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.markers.append(text)
        marker.controls.append(control)
        return marker

    def _create_markers(self) -> None:
        for arm in ("left", "right"):
            self.server.insert(
                self._tool_marker(arm),
                feedback_callback=lambda feedback, arm=arm: self._tool_feedback(
                    arm, feedback
                ),
            )
            tool_menu = MenuHandler()
            tool_menu.insert(
                "Reset to TCP +Z, 20 cm",
                callback=lambda feedback, arm=arm: self._reset_tool(arm),
            )
            tool_menu.apply(self.server, f"{arm}_tool_cylinder")
            self.tool_menus[arm] = tool_menu

            self.server.insert(
                self._workspace_marker(arm),
                feedback_callback=lambda feedback, arm=arm: self._workspace_feedback(
                    arm, feedback
                ),
            )
            menu = MenuHandler()
            for axis_index, axis_name in enumerate(("X", "Y", "Z")):
                menu.insert(
                    f"Expand {axis_name} by 2 cm",
                    callback=lambda feedback, arm=arm, axis=axis_index: (
                        self._resize_workspace(arm, axis, 0.02)
                    ),
                )
                menu.insert(
                    f"Shrink {axis_name} by 2 cm",
                    callback=lambda feedback, arm=arm, axis=axis_index: (
                        self._resize_workspace(arm, axis, -0.02)
                    ),
                )
            menu.apply(self.server, f"{arm}_workspace")
            self.workspace_menus[arm] = menu

        self.server.insert(self._clearance_marker())
        for title, delta in (
            ("Increase by 1 cm", 0.01),
            ("Decrease by 1 cm", -0.01),
            ("Increase by 5 mm", 0.005),
            ("Decrease by 5 mm", -0.005),
        ):
            self.clearance_menu.insert(
                title,
                callback=lambda feedback, delta=delta: self._adjust_clearance(delta),
            )
        self.clearance_menu.apply(self.server, "safety_clearance")
        self.server.applyChanges()
        self.markers_created = True

    def _tool_feedback(self, arm: str, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        cylinder_global = pose_matrix(feedback.pose.position, feedback.pose.orientation)
        tcp_inverse = np.linalg.inv(self._tcp_global(arm))
        cylinder_tcp = tcp_inverse @ cylinder_global
        with self.lock:
            tool = self.config[arm]["bodies"]["tool"]
            start_old = np.asarray(tool["segment_start"], dtype=np.float64)
            end_old = np.asarray(tool["segment_end"], dtype=np.float64)
            half_length = 0.5 * float(np.linalg.norm(end_old - start_old))
            center = cylinder_tcp[:3, 3]
            z_axis = cylinder_tcp[:3, 2]
            tool["segment_start"] = (center - half_length * z_axis).tolist()
            tool["segment_end"] = (center + half_length * z_axis).tolist()
        self._configuration_changed()

    def _workspace_feedback(
        self, arm: str, feedback: InteractiveMarkerFeedback
    ) -> None:
        if feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            return
        center_global = np.array(
            [
                feedback.pose.position.x,
                feedback.pose.position.y,
                feedback.pose.position.z,
                1.0,
            ]
        )
        center_local = np.linalg.inv(self._arm_transform(arm)) @ center_global
        with self.lock:
            minimum = np.asarray(self.config[arm]["workspace_min"], dtype=np.float64)
            maximum = np.asarray(self.config[arm]["workspace_max"], dtype=np.float64)
            half_size = 0.5 * (maximum - minimum)
            self.config[arm]["workspace_min"] = (
                center_local[:3] - half_size
            ).tolist()
            self.config[arm]["workspace_max"] = (
                center_local[:3] + half_size
            ).tolist()
        self._configuration_changed()

    def _reset_tool(self, arm: str) -> None:
        with self.lock:
            tool = self.config[arm]["bodies"]["tool"]
            tool["segment_start"] = [0.0, 0.0, 0.0]
            tool["segment_end"] = [0.0, 0.0, 0.20]
            tool["radius"] = 0.05
        self._configuration_changed()

    def _resize_workspace(self, arm: str, axis: int, delta: float) -> None:
        with self.lock:
            minimum = np.asarray(self.config[arm]["workspace_min"], dtype=np.float64)
            maximum = np.asarray(self.config[arm]["workspace_max"], dtype=np.float64)
            current_size = maximum[axis] - minimum[axis]
            if current_size + 2.0 * delta < 0.10:
                return
            minimum[axis] -= delta
            maximum[axis] += delta
            self.config[arm]["workspace_min"] = minimum.tolist()
            self.config[arm]["workspace_max"] = maximum.tolist()
        self._configuration_changed(rebuild=True)

    def _adjust_clearance(self, delta: float) -> None:
        with self.lock:
            self.config["clearance"] = max(
                0.0, min(0.30, float(self.config["clearance"]) + delta)
            )
        self._configuration_changed(rebuild=True)

    def _configuration_changed(self, rebuild: bool = False) -> None:
        self._publish_config()
        if rebuild:
            self.server.clear()
            self.markers_created = False
            self.server.applyChanges()

    def _publish_config(self) -> None:
        with self.lock:
            payload = json.dumps(self.config)
        message = String()
        message.data = payload
        self.config_pub.publish(message)

    def _save_cb(self, request, response):
        del request
        try:
            with self.lock:
                save_config(self.config_file, self.config)
            response.success = True
            response.message = f"Saved collision regions to {self.config_file}"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _refresh(self) -> None:
        if self.actual["left"] is None or self.actual["right"] is None:
            return
        if not self.markers_created:
            self._create_markers()
            return
        for arm in ("left", "right"):
            self.server.setPose(f"{arm}_tool_cylinder", _pose_from_matrix(self._tool_global(arm)))
        self.server.applyChanges()

    def destroy_node(self):
        self.server.shutdown()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RegionEditor()
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
