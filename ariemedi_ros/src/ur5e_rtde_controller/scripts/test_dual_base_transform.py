#!/usr/bin/env python3

import argparse
import json
import math
import sys
import time
from typing import Tuple

import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from ariemedi_tracker.msg import ToolTrackingData
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


def rotvec_to_matrix(rx: float, ry: float, rz: float) -> np.ndarray:
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta < 1e-12:
        return np.eye(3, dtype=np.float64)
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    c = math.cos(theta)
    s = math.sin(theta)
    v = 1.0 - c
    return np.array(
        [
            [kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s],
            [ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s],
            [kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c],
        ],
        dtype=np.float64,
    )


def matrix_to_rotvec(r: np.ndarray) -> Tuple[float, float, float]:
    trace = float(np.trace(r))
    cos_theta = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return 0.0, 0.0, 0.0
    denom = 2.0 * math.sin(theta)
    rx = (r[2, 1] - r[1, 2]) / denom
    ry = (r[0, 2] - r[2, 0]) / denom
    rz = (r[1, 0] - r[0, 1]) / denom
    return rx * theta, ry * theta, rz * theta


def make_transform(pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = rot
    t[:3, 3] = pos
    return t


def inv_transform(t: np.ndarray) -> np.ndarray:
    r = t[:3, :3]
    p = t[:3, 3]
    t_inv = np.eye(4, dtype=np.float64)
    t_inv[:3, :3] = r.T
    t_inv[:3, 3] = -r.T @ p
    return t_inv


def quat_to_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def wait_for_markers(
    topic: str,
    left_name: str,
    right_name: str,
    timeout_sec: float,
) -> Tuple[ToolTrackingData, ToolTrackingData]:
    left_key = left_name.lower()
    right_key = right_name.lower()
    cache = {}

    rclpy.init()
    node = rclpy.create_node("dual_marker_check")

    def cb(msg: ToolTrackingData) -> None:
        key = msg.name.lower()
        if key in (left_key, right_key):
            cache[key] = msg

    node.create_subscription(ToolTrackingData, topic, cb, 50)
    start = time.time()
    while rclpy.ok() and time.time() - start < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.1)
        if left_key in cache and right_key in cache:
            break

    node.destroy_node()
    rclpy.shutdown()

    if left_key not in cache:
        raise RuntimeError(f"Missing tracker data for {left_name}")
    if right_key not in cache:
        raise RuntimeError(f"Missing tracker data for {right_name}")
    return cache[left_key], cache[right_key]


def main() -> int:
    parser = argparse.ArgumentParser(description="Test dual-base transform by moving left arm")
    parser.add_argument(
        "--json",
        default="/home/us-mrc/Documents/BMT_Group_ARIEMEDI/dual_arm_cali/dual_base_left_to_right.json",
        help="Path to dual base transform json",
    )
    parser.add_argument("--left_robot_ip", default="192.168.5.202")
    parser.add_argument("--right_robot_ip", default="192.168.5.101")
    parser.add_argument("--speed", type=float, default=0.01, help="Linear speed (m/s)")
    parser.add_argument("--acc", type=float, default=0.05, help="Linear acceleration (m/s^2)")
    parser.add_argument("--marker_topic", default="/ARMDpos", help="Tracker marker topic")
    parser.add_argument("--left_marker_name", default="caliorange")
    parser.add_argument("--right_marker_name", default="caliblack2")
    parser.add_argument(
        "--pause_sec",
        type=float,
        default=2.0,
        help="Pause seconds between steps",
    )
    parser.add_argument(
        "--tracker_timeout_sec",
        type=float,
        default=5.0,
        help="Timeout waiting for tracker data",
    )
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    try:
        with open(args.json, "r", encoding="utf-8") as f:
            payload = json.load(f)
    except Exception as exc:
        print(f"Failed to load json: {exc}")
        return 1

    if "left_base_T_right_base" not in payload:
        print("Missing left_base_T_right_base in json")
        return 1

    left_base_T_right_base = np.asarray(payload["left_base_T_right_base"], dtype=np.float64)
    if left_base_T_right_base.shape != (4, 4):
        print("left_base_T_right_base must be 4x4")
        return 1

    right_rtde_r = RTDEReceiveInterface(args.right_robot_ip)
    left_rtde_c = RTDEControlInterface(args.left_robot_ip)

    try:
        right_pose = right_rtde_r.getActualTCPPose()
        right_pos = np.array(right_pose[:3], dtype=np.float64)
        right_rot = rotvec_to_matrix(right_pose[3], right_pose[4], right_pose[5])
        t_right_base_right_eef = make_transform(right_pos, right_rot)
        time.sleep(args.pause_sec)

        t_eef_tip = np.eye(4, dtype=np.float64)
        t_eef_tip[:3, 3] = np.array([0.0, 0.0, 0.130], dtype=np.float64)

        t_right_base_right_tip = t_right_base_right_eef @ t_eef_tip

        r_right_left = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=np.float64,
        )
        t_right_tip_left_tip = make_transform(np.zeros(3, dtype=np.float64), r_right_left)

        t_right_base_left_tip = t_right_base_right_tip @ t_right_tip_left_tip
        t_left_base_left_tip = left_base_T_right_base @ t_right_base_left_tip
        t_left_base_left_eef = t_left_base_left_tip @ inv_transform(t_eef_tip)

        target_pos = t_left_base_left_eef[:3, 3]
        target_rot = t_left_base_left_eef[:3, :3]
        rx, ry, rz = matrix_to_rotvec(target_rot)
        target_pose = [
            float(target_pos[0]),
            float(target_pos[1]),
            float(target_pos[2]),
            float(rx),
            float(ry),
            float(rz),
        ]

        print("Target left TCP pose (base frame):", target_pose)
        ok = left_rtde_c.moveL(target_pose, args.speed, args.acc)
        if not ok:
            print("moveL failed")
            return 2
        time.sleep(args.pause_sec)

        try:
            left_msg, right_msg = wait_for_markers(
                args.marker_topic,
                args.left_marker_name,
                args.right_marker_name,
                args.tracker_timeout_sec,
            )
        except RuntimeError as exc:
            print(str(exc))
            return 3
        time.sleep(args.pause_sec)

        t_tracker_left_marker = make_transform(
            np.array(
                [
                    left_msg.transform.tx / 1000.0,
                    left_msg.transform.ty / 1000.0,
                    left_msg.transform.tz / 1000.0,
                ],
                dtype=np.float64,
            ),
            quat_to_matrix(
                left_msg.transform.qw,
                left_msg.transform.qx,
                left_msg.transform.qy,
                left_msg.transform.qz,
            ),
        )
        t_tracker_right_marker = make_transform(
            np.array(
                [
                    right_msg.transform.tx / 1000.0,
                    right_msg.transform.ty / 1000.0,
                    right_msg.transform.tz / 1000.0,
                ],
                dtype=np.float64,
            ),
            quat_to_matrix(
                right_msg.transform.qw,
                right_msg.transform.qx,
                right_msg.transform.qy,
                right_msg.transform.qz,
            ),
        )

        t_eef_marker = np.eye(4, dtype=np.float64)
        t_eef_marker[:3, 3] = np.array([0.019, 0.0, 0.055], dtype=np.float64)
        t_eef_marker[:3, :3] = np.array(
            [
                [0.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            dtype=np.float64,
        )
        t_marker_eef = inv_transform(t_eef_marker)

        t_eef_tip = np.eye(4, dtype=np.float64)
        t_eef_tip[:3, 3] = np.array([0.0, 0.0, 0.130], dtype=np.float64)

        t_tracker_left_tip = t_tracker_left_marker @ t_marker_eef @ t_eef_tip
        t_tracker_right_tip = t_tracker_right_marker @ t_marker_eef @ t_eef_tip

        tip_distance_m = float(
            np.linalg.norm(t_tracker_left_tip[:3, 3] - t_tracker_right_tip[:3, 3])
        )

        r_rel = t_tracker_left_tip[:3, :3] @ t_tracker_right_tip[:3, :3].T
        cos_theta = max(-1.0, min(1.0, 0.5 * (np.trace(r_rel) - 1.0)))
        angle_deg = math.degrees(math.acos(cos_theta))

        print(f"Tip distance: {tip_distance_m * 1000.0:.2f} mm")
        print(f"Tip orientation difference: {angle_deg:.2f} deg")
    except Exception as exc:
        print(f"Failed to compute or move: {exc}")
        return 1
    finally:
        try:
            left_rtde_c.disconnect()
        except Exception:
            pass
        try:
            right_rtde_r.disconnect()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
