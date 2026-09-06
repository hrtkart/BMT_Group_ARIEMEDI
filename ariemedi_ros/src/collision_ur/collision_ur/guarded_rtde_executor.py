#!/usr/bin/env python3

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from collision_ur.geometry import quaternion_matrix

try:
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
except ImportError:
    RTDEControlInterface = None
    RTDEReceiveInterface = None


def _matrix_to_rotvec(rotation: np.ndarray):
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(math.acos(cosine))
    if angle < 1.0e-9:
        return [0.0, 0.0, 0.0]
    if abs(math.pi - angle) < 1.0e-6:
        diagonal = np.maximum((np.diag(rotation) + 1.0) * 0.5, 0.0)
        axis = np.sqrt(diagonal)
        axis[0] = math.copysign(axis[0], rotation[2, 1] - rotation[1, 2])
        axis[1] = math.copysign(axis[1], rotation[0, 2] - rotation[2, 0])
        axis[2] = math.copysign(axis[2], rotation[1, 0] - rotation[0, 1])
        norm = float(np.linalg.norm(axis))
        axis = axis / norm if norm > 1.0e-9 else np.array([1.0, 0.0, 0.0])
    else:
        axis = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ]
        ) / (2.0 * math.sin(angle))
    return (axis * angle).tolist()


@dataclass
class Arm:
    name: str
    receive: object
    control: object
    publisher: object
    joint_names: list
    target: Optional[list]
    condition: threading.Condition


class GuardedRtdeExecutor(Node):
    """The only hardware command sink; it accepts supervisor-approved poses."""

    def __init__(self) -> None:
        super().__init__("guarded_rtde_executor")
        if RTDEControlInterface is None or RTDEReceiveInterface is None:
            raise RuntimeError("ur-rtde Python bindings are not installed")
        self.declare_parameter("left_robot_ip", "192.168.5.202")
        self.declare_parameter("right_robot_ip", "192.168.5.101")
        self.declare_parameter("linear_speed_mps", 0.02)
        self.declare_parameter("linear_acc_mps2", 0.10)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.speed = float(self.get_parameter("linear_speed_mps").value)
        self.acceleration = float(self.get_parameter("linear_acc_mps2").value)
        self.stopping = threading.Event()
        self.arms = {}
        self.workers = []
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", 20
        )

        self._create_arm(
            "left",
            self.get_parameter("left_robot_ip").value,
            "/left_arm/eef_pose",
            "/left_arm/safe_target_pose",
            [
                "L_Base",
                "L_Shoulder",
                "L_Elbow",
                "L_Wrist1",
                "L_Wrist2",
                "L_Wrist3",
            ],
        )
        self._create_arm(
            "right",
            self.get_parameter("right_robot_ip").value,
            "/right_arm/eef_pose",
            "/right_arm/safe_target_pose",
            [
                "R_Base",
                "R_Shoulder",
                "R_Elbow",
                "R_Wrist1",
                "R_Wrist2",
                "R_Wrist3",
            ],
        )
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish_actual)
        self.get_logger().warn(
            "RTDE executor armed. Only /left_arm/safe_target_pose and "
            "/right_arm/safe_target_pose are accepted."
        )

    def _create_arm(
        self,
        name: str,
        ip_address: str,
        actual_topic: str,
        safe_topic: str,
        joint_names: list,
    ) -> None:
        self.get_logger().info(f"[{name}] Connecting to UR5e at {ip_address}")
        receive = RTDEReceiveInterface(ip_address)
        control = RTDEControlInterface(ip_address)
        publisher = self.create_publisher(PoseStamped, actual_topic, 20)
        condition = threading.Condition()
        arm = Arm(name, receive, control, publisher, joint_names, None, condition)
        self.arms[name] = arm
        self.create_subscription(
            PoseStamped,
            safe_topic,
            lambda message, arm=arm: self._safe_target_cb(arm, message),
            20,
        )
        worker = threading.Thread(target=self._worker, args=(arm,), daemon=True)
        worker.start()
        self.workers.append(worker)

    def _safe_target_cb(self, arm: Arm, message: PoseStamped) -> None:
        expected_frame = f"{arm.name}_base"
        if message.header.frame_id != expected_frame:
            self.get_logger().error(
                f"[{arm.name}] Ignored safe target in '{message.header.frame_id}', "
                f"expected '{expected_frame}'"
            )
            return
        rotation = quaternion_matrix(
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        rotvec = _matrix_to_rotvec(rotation)
        target = [
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            *rotvec,
        ]
        with arm.condition:
            arm.target = target
            arm.condition.notify()

    def _worker(self, arm: Arm) -> None:
        while not self.stopping.is_set():
            with arm.condition:
                arm.condition.wait_for(lambda: arm.target is not None or self.stopping.is_set())
                if self.stopping.is_set():
                    return
                target = arm.target
                arm.target = None
            try:
                success = arm.control.moveL(target, self.speed, self.acceleration)
                if not success:
                    self.get_logger().error(f"[{arm.name}] RTDE moveL failed")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[{arm.name}] RTDE command failed: {exc}")
                time.sleep(0.1)

    def _publish_actual(self) -> None:
        stamp = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        for arm in self.arms.values():
            try:
                tcp = arm.receive.getActualTCPPose()
                joints = arm.receive.getActualQ()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[{arm.name}] RTDE read failed: {exc}")
                continue
            angle = float(np.linalg.norm(tcp[3:6]))
            if angle > 1.0e-12:
                axis = np.asarray(tcp[3:6], dtype=np.float64) / angle
                half = 0.5 * angle
                quaternion = np.r_[axis * math.sin(half), math.cos(half)]
            else:
                quaternion = np.array([0.0, 0.0, 0.0, 1.0])
            message = PoseStamped()
            message.header.stamp = stamp
            message.header.frame_id = f"{arm.name}_base"
            message.pose.position.x = tcp[0]
            message.pose.position.y = tcp[1]
            message.pose.position.z = tcp[2]
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ) = quaternion
            arm.publisher.publish(message)
            joint_state.name.extend(arm.joint_names)
            joint_state.position.extend(joints)
        if joint_state.name:
            self.joint_state_publisher.publish(joint_state)

    def destroy_node(self):
        self.stopping.set()
        for arm in self.arms.values():
            with arm.condition:
                arm.condition.notify_all()
            try:
                arm.control.stopL(2.0)
            except Exception:  # noqa: BLE001
                pass
        for worker in self.workers:
            worker.join(timeout=2.0)
        for arm in self.arms.values():
            for interface in (arm.control, arm.receive):
                try:
                    interface.disconnect()
                except Exception:  # noqa: BLE001
                    pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuardedRtdeExecutor()
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
