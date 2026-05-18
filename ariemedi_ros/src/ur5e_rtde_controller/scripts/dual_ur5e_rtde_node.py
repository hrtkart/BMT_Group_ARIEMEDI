#!/usr/bin/python3

import math
import random
import threading
import time
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Bool
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


def matrix_to_rotvec(rot: List[List[float]]) -> Tuple[float, float, float]:
    trace = rot[0][0] + rot[1][1] + rot[2][2]
    cos_theta = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return 0.0, 0.0, 0.0

    denom = 2.0 * math.sin(theta)
    rx = (rot[2][1] - rot[1][2]) / denom
    ry = (rot[0][2] - rot[2][0]) / denom
    rz = (rot[1][0] - rot[0][1]) / denom
    return rx * theta, ry * theta, rz * theta


def euler_xyz_to_matrix(rx: float, ry: float, rz: float) -> List[List[float]]:
    cx = math.cos(rx)
    sx = math.sin(rx)
    cy = math.cos(ry)
    sy = math.sin(ry)
    cz = math.cos(rz)
    sz = math.sin(rz)

    return [
        [cy * cz, -cy * sz, sy],
        [sx * sy * cz + cx * sz, -sx * sy * sz + cx * cz, -sx * cy],
        [-cx * sy * cz + sx * sz, cx * sy * sz + sx * cz, cx * cy],
    ]


def mat3_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ],
        [
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ],
        [
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ],
    ]


@dataclass
class ArmRuntime:
    name: str
    ip: str
    eef_topic: str
    rtde_r: RTDEReceiveInterface
    rtde_c: RTDEControlInterface
    home_pose: List[float]
    home_rot: List[List[float]]
    pub: any
    time_ref_pub: any
    clock_offset_sec: float = 0.0
    clock_offset_initialized: bool = False


class DualUr5eRtdeNode(Node):
    def __init__(self) -> None:
        super().__init__("dual_ur5e_rtde_node")

        self.declare_parameter("left_robot_ip", "192.168.5.202")
        self.declare_parameter("right_robot_ip", "192.168.5.101")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("sphere_radius_m", 0.10)
        self.declare_parameter("linear_speed_mps", 0.01)
        self.declare_parameter("linear_acc_mps2", 0.05)
        self.declare_parameter("min_target_distance_m", 0.02)
        self.declare_parameter("angular_range_deg", 20.0)
        self.declare_parameter("stop_delay_sec", 30.0)

        self.left_robot_ip = self.get_parameter("left_robot_ip").get_parameter_value().string_value
        self.right_robot_ip = self.get_parameter("right_robot_ip").get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.sphere_radius_m = self.get_parameter("sphere_radius_m").get_parameter_value().double_value
        self.linear_speed_mps = self.get_parameter("linear_speed_mps").get_parameter_value().double_value
        self.linear_acc_mps2 = self.get_parameter("linear_acc_mps2").get_parameter_value().double_value
        self.min_target_distance_m = (
            self.get_parameter("min_target_distance_m").get_parameter_value().double_value
        )
        self.angular_range_deg = self.get_parameter("angular_range_deg").get_parameter_value().double_value
        self.stop_delay_sec = self.get_parameter("stop_delay_sec").get_parameter_value().double_value

        self._stop_event = threading.Event()
        self.stop_after_next_home_ = False
        self.stop_request_time_monotonic_ = None
        self.stop_delay_armed_logged_ = False
        self._arm_done_lock = threading.Lock()
        self._arm_done = set()
        self.rng = random.Random()
        self.arms: List[ArmRuntime] = []
        self.motion_threads: List[threading.Thread] = []
        self.stop_request_sub = self.create_subscription(
            Bool,
            "/dual_system/stop_after_next_home",
            self.stop_request_cb,
            10,
        )

        self._init_arm("left", self.left_robot_ip, "/left_arm/eef_pose")
        self._init_arm("right", self.right_robot_ip, "/right_arm/eef_pose")

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_pose_all)

        for arm in self.arms:
            thread = threading.Thread(target=self.motion_loop, args=(arm,), daemon=True)
            thread.start()
            self.motion_threads.append(thread)

    def stop_request_cb(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.stop_request_time_monotonic_ is None:
            self.stop_request_time_monotonic_ = time.monotonic()
            self.get_logger().info(
                f"Received stop request: stop will arm after {self.stop_delay_sec:.1f}s, "
                "then exit on next home pose for each arm."
            )

    def should_stop_after_home(self) -> bool:
        if self.stop_request_time_monotonic_ is None:
            return False
        elapsed = time.monotonic() - self.stop_request_time_monotonic_
        if elapsed < self.stop_delay_sec:
            return False
        if not self.stop_after_next_home_:
            self.stop_after_next_home_ = True
        if not self.stop_delay_armed_logged_:
            self.stop_delay_armed_logged_ = True
            self.get_logger().info("Stop delay elapsed: controller will now stop at next home pose.")
        return True

    def _init_arm(self, arm_name: str, arm_ip: str, eef_topic: str) -> None:
        self.get_logger().info(f"[{arm_name}] Connecting via RTDE at {arm_ip}")
        rtde_r = RTDEReceiveInterface(arm_ip)
        rtde_c = RTDEControlInterface(arm_ip)

        start_pose = rtde_r.getActualTCPPose()
        home_pose = list(start_pose)
        home_rot = rotvec_to_matrix(start_pose[3], start_pose[4], start_pose[5])

        pub = self.create_publisher(PoseStamped, eef_topic, 10)
        time_ref_pub = self.create_publisher(TimeReference, f"/{arm_name}_arm/eef_time_reference", 20)
        runtime = ArmRuntime(
            name=arm_name,
            ip=arm_ip,
            eef_topic=eef_topic,
            rtde_r=rtde_r,
            rtde_c=rtde_c,
            home_pose=home_pose,
            home_rot=home_rot,
            pub=pub,
            time_ref_pub=time_ref_pub,
        )
        self.arms.append(runtime)

        self.get_logger().info(
            f"[{arm_name}] Random motion: radius={self.sphere_radius_m:.3f} m, "
            f"speed={self.linear_speed_mps:.3f} m/s, ang_range=+/-{self.angular_range_deg:.1f} deg"
        )
        self.get_logger().info(
            f"[{arm_name}] Home xyz: [{home_pose[0]:.4f}, {home_pose[1]:.4f}, {home_pose[2]:.4f}]"
        )

    def _sample_random_target(self, arm: ArmRuntime) -> List[float]:
        radius = max(0.0, self.sphere_radius_m)
        min_dist = max(0.0, min(self.min_target_distance_m, radius))
        for _ in range(50):
            z = self.rng.uniform(-1.0, 1.0)
            t = self.rng.uniform(0.0, 2.0 * math.pi)
            r = radius * (self.rng.random() ** (1.0 / 3.0))
            xy = math.sqrt(max(0.0, 1.0 - z * z))
            dx = r * xy * math.cos(t)
            dy = r * xy * math.sin(t)
            dz = r * z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist >= min_dist:
                break
        else:
            dx = dy = dz = 0.0

        ang = math.radians(self.angular_range_deg)
        rx = self.rng.uniform(-ang, ang)
        ry = self.rng.uniform(-ang, ang)
        rz = self.rng.uniform(-ang, ang)
        rot_offset = euler_xyz_to_matrix(rx, ry, rz)
        rot_target = mat3_mul(arm.home_rot, rot_offset)
        rvec = matrix_to_rotvec(rot_target)

        target = list(arm.home_pose)
        target[0] = arm.home_pose[0] + dx
        target[1] = arm.home_pose[1] + dy
        target[2] = arm.home_pose[2] + dz
        target[3] = rvec[0]
        target[4] = rvec[1]
        target[5] = rvec[2]
        return target

    def publish_pose_all(self) -> None:
        for arm in self.arms:
            try:
                tcp = arm.rtde_r.getActualTCPPose()
                robot_clock_sec = arm.rtde_r.getTimestamp()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[{arm.name}] Failed to read EEF pose: {exc}")
                continue

            sys_now_sec = float(self.get_clock().now().nanoseconds) * 1e-9
            if not arm.clock_offset_initialized:
                arm.clock_offset_sec = sys_now_sec - robot_clock_sec
                arm.clock_offset_initialized = True
            else:
                measured_offset = sys_now_sec - robot_clock_sec
                arm.clock_offset_sec = 0.995 * arm.clock_offset_sec + 0.005 * measured_offset
            mapped_sec = robot_clock_sec + arm.clock_offset_sec
            mapped_ns = int(mapped_sec * 1e9)

            msg = PoseStamped()
            msg.header.stamp.sec = mapped_ns // 1_000_000_000
            msg.header.stamp.nanosec = mapped_ns % 1_000_000_000
            msg.header.frame_id = f"{arm.name}_base"
            msg.pose.position.x = tcp[0]
            msg.pose.position.y = tcp[1]
            msg.pose.position.z = tcp[2]

            qx, qy, qz, qw = rotvec_to_quaternion(tcp[3], tcp[4], tcp[5])
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            arm.pub.publish(msg)

            raw_time_ns = int(robot_clock_sec * 1e9)
            time_ref = TimeReference()
            time_ref.header = msg.header
            time_ref.time_ref.sec = raw_time_ns // 1_000_000_000
            time_ref.time_ref.nanosec = raw_time_ns % 1_000_000_000
            time_ref.source = f"{arm.name}_rtde_clock_sec"
            arm.time_ref_pub.publish(time_ref)

    def move_to(self, arm: ArmRuntime, target_pose: List[float], label: str) -> bool:
        if self._stop_event.is_set():
            return False

        self.get_logger().info(
            f"[{arm.name}] Moving to {label} target with speed={self.linear_speed_mps:.3f} m/s, "
            f"acc={self.linear_acc_mps2:.3f} m/s^2"
        )
        ok = arm.rtde_c.moveL(target_pose, self.linear_speed_mps, self.linear_acc_mps2)
        if not ok:
            self.get_logger().error(f"[{arm.name}] moveL failed when moving to {label}")
            return False
        return True

    def motion_loop(self, arm: ArmRuntime) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                if self.should_stop_after_home():
                    if not self.move_to(arm, arm.home_pose, "home"):
                        break
                    self.get_logger().info(f"[{arm.name}] Reached home pose after stop request.")
                    break

                target_pose = self._sample_random_target(arm)
                if not self.move_to(arm, target_pose, "random"):
                    break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"[{arm.name}] Motion loop exception: {exc}")
                break

        self.get_logger().warn(f"[{arm.name}] Motion loop stopped")
        self.mark_arm_done(arm.name)

    def mark_arm_done(self, arm_name: str) -> None:
        with self._arm_done_lock:
            self._arm_done.add(arm_name)
            should_shutdown = self.stop_after_next_home_ and len(self._arm_done) == len(self.arms)
        if should_shutdown:
            self.get_logger().info("All arms reached home. Shutting down dual-arm controller.")
            self._stop_event.set()
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        for arm in self.arms:
            try:
                arm.rtde_c.stopL(2.0)
            except Exception:  # noqa: BLE001
                pass
            try:
                arm.rtde_c.disconnect()
            except Exception:  # noqa: BLE001
                pass
            try:
                arm.rtde_r.disconnect()
            except Exception:  # noqa: BLE001
                pass

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualUr5eRtdeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
