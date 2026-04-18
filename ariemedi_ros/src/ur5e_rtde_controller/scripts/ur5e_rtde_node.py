#!/usr/bin/python3

import math
import threading
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


def rotvec_to_quaternion(rx: float, ry: float, rz: float) -> Tuple[float, float, float, float]:
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return 0.0, 0.0, 0.0, 1.0

    ax = rx / theta
    ay = ry / theta
    az = rz / theta

    half = 0.5 * theta
    s = math.sin(half)
    return ax * s, ay * s, az * s, math.cos(half)


def rotvec_to_matrix(rx: float, ry: float, rz: float) -> List[List[float]]:
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    kx = rx / theta
    ky = ry / theta
    kz = rz / theta

    c = math.cos(theta)
    s = math.sin(theta)
    v = 1.0 - c

    return [
        [kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
        [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
        [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
    ]


class UR5eRtdeNode(Node):
    def __init__(self) -> None:
        super().__init__("ur5e_rtde_node")

        self.declare_parameter("robot_ip", "192.168.56.5")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("travel_distance_m", 0.20)
        self.declare_parameter("linear_speed_mps", 0.03)
        self.declare_parameter("linear_acc_mps2", 0.20)
        self.declare_parameter("pause_at_switch_sec", 2.0)

        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.travel_distance_m = self.get_parameter("travel_distance_m").get_parameter_value().double_value
        self.linear_speed_mps = self.get_parameter("linear_speed_mps").get_parameter_value().double_value
        self.linear_acc_mps2 = self.get_parameter("linear_acc_mps2").get_parameter_value().double_value
        self.pause_at_switch_sec = self.get_parameter("pause_at_switch_sec").get_parameter_value().double_value

        self.pose_pub = self.create_publisher(PoseStamped, "ur5e/eef_pose", 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_pose)

        self._stop_event = threading.Event()

        self.get_logger().info(f"Connecting to UR5e via RTDE at {self.robot_ip}")
        self.rtde_r = RTDEReceiveInterface(self.robot_ip)
        self.rtde_c = RTDEControlInterface(self.robot_ip)

        self.start_pose = self.rtde_r.getActualTCPPose()
        self.home_pose = list(self.start_pose)
        self.minus_y_pose = self._build_minus_y_target(self.start_pose, self.travel_distance_m)

        self.get_logger().info(
            f"Motion profile: start -> -y {self.travel_distance_m:.3f} m -> start, "
            f"pause {self.pause_at_switch_sec:.1f}s at each direction switch"
        )
        self.get_logger().info(
            f"Home pose: [{self.home_pose[0]:.4f}, {self.home_pose[1]:.4f}, {self.home_pose[2]:.4f}]"
        )
        self.get_logger().info(
            f"Minus-Y target: [{self.minus_y_pose[0]:.4f}, {self.minus_y_pose[1]:.4f}, {self.minus_y_pose[2]:.4f}]"
        )

        self.motion_thread = threading.Thread(target=self.motion_loop, daemon=True)
        self.motion_thread.start()

    def _build_minus_y_target(self, pose: List[float], distance: float) -> List[float]:
        target = list(pose)
        _, _, _, rx, ry, rz = pose

        rot = rotvec_to_matrix(rx, ry, rz)

        local_minus_y = [0.0, -distance, 0.0]
        dx = rot[0][0] * local_minus_y[0] + rot[0][1] * local_minus_y[1] + rot[0][2] * local_minus_y[2]
        dy = rot[1][0] * local_minus_y[0] + rot[1][1] * local_minus_y[1] + rot[1][2] * local_minus_y[2]
        dz = rot[2][0] * local_minus_y[0] + rot[2][1] * local_minus_y[1] + rot[2][2] * local_minus_y[2]

        target[0] = pose[0] + dx
        target[1] = pose[1] + dy
        target[2] = pose[2] + dz
        return target

    def publish_pose(self) -> None:
        try:
            tcp = self.rtde_r.getActualTCPPose()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to read EEF pose: {exc}")
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"

        msg.pose.position.x = tcp[0]
        msg.pose.position.y = tcp[1]
        msg.pose.position.z = tcp[2]

        qx, qy, qz, qw = rotvec_to_quaternion(tcp[3], tcp[4], tcp[5])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)

    def move_to(self, target_pose: List[float], label: str) -> bool:
        if self._stop_event.is_set():
            return False

        self.get_logger().info(
            f"Moving to {label} target with speed={self.linear_speed_mps:.3f} m/s, acc={self.linear_acc_mps2:.3f} m/s^2"
        )
        ok = self.rtde_c.moveL(target_pose, self.linear_speed_mps, self.linear_acc_mps2)
        if not ok:
            self.get_logger().error(f"moveL failed when moving to {label} target")
            return False
        return True

    def motion_loop(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                if not self.move_to(self.minus_y_pose, "-Y"):
                    break
                if self._wait_with_abort(self.pause_at_switch_sec):
                    break

                if not self.move_to(self.home_pose, "home"):
                    break
                if self._wait_with_abort(self.pause_at_switch_sec):
                    break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Motion loop exception: {exc}")
                break

        self.get_logger().warn("Motion loop stopped")

    def _wait_with_abort(self, sec: float) -> bool:
        deadline = time.time() + sec
        while time.time() < deadline:
            if self._stop_event.is_set():
                return True
            time.sleep(0.05)
        return False

    def destroy_node(self) -> bool:
        self._stop_event.set()
        try:
            self.rtde_c.stopL(2.0)
        except Exception:  # noqa: BLE001
            pass

        try:
            self.rtde_c.disconnect()
        except Exception:  # noqa: BLE001
            pass

        try:
            self.rtde_r.disconnect()
        except Exception:  # noqa: BLE001
            pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UR5eRtdeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
