#!/usr/bin/python3

import os
import threading
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List

import numpy as np
import rclpy
from ariemedi_tracker.msg import ToolTrackingData
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Bool


@dataclass
class PoseSeries:
    stamp_sec: List[float] = field(default_factory=list)
    device_clock_sec: List[float] = field(default_factory=list)
    pos: List[List[float]] = field(default_factory=list)
    quat_xyzw: List[List[float]] = field(default_factory=list)
    first_stamp_sec: float = 0.0
    has_first: bool = False


@dataclass
class ArmSeries:
    arm_name: str
    marker_name: str
    eef_topic: str
    eef: PoseSeries = field(default_factory=PoseSeries)
    marker: PoseSeries = field(default_factory=PoseSeries)
    pending_eef_device_clock_by_ros_ns: Dict[int, float] = field(default_factory=dict)


class DualEefMarkerDataRecorder(Node):
    def __init__(self) -> None:
        super().__init__("dual_eef_marker_data_recorder")

        self.declare_parameter("duration_sec", 60.0)
        self.declare_parameter("output_data", "/home/us-mrc/Documents/dual_arm_eef_marker_raw.npy")
        self.declare_parameter("marker_topic", "/ARMDpos")
        self.declare_parameter("left_eef_topic", "/left_arm/eef_pose")
        self.declare_parameter("right_eef_topic", "/right_arm/eef_pose")
        self.declare_parameter("left_eef_time_ref_topic", "/left_arm/eef_time_reference")
        self.declare_parameter("right_eef_time_ref_topic", "/right_arm/eef_time_reference")
        self.declare_parameter("left_marker_name", "caliorange")
        self.declare_parameter("right_marker_name", "caliblack")

        self.duration_sec = self.get_parameter("duration_sec").get_parameter_value().double_value
        self.output_data = self.get_parameter("output_data").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value

        left = ArmSeries(
            arm_name="left",
            marker_name=self.get_parameter("left_marker_name").get_parameter_value().string_value,
            eef_topic=self.get_parameter("left_eef_topic").get_parameter_value().string_value,
        )
        right = ArmSeries(
            arm_name="right",
            marker_name=self.get_parameter("right_marker_name").get_parameter_value().string_value,
            eef_topic=self.get_parameter("right_eef_topic").get_parameter_value().string_value,
        )
        self.arms: Dict[str, ArmSeries] = {"left": left, "right": right}
        self.marker_name_to_arm = {
            left.marker_name.lower(): "left",
            right.marker_name.lower(): "right",
        }

        self.start_stamp_sec = None
        self.latest_observed_stamp_sec = None
        self.finished = False

        self.create_subscription(PoseStamped, left.eef_topic, self.left_eef_cb, 50)
        self.create_subscription(PoseStamped, right.eef_topic, self.right_eef_cb, 50)
        self.create_subscription(
            TimeReference,
            self.get_parameter("left_eef_time_ref_topic").get_parameter_value().string_value,
            self.left_eef_time_ref_cb,
            50,
        )
        self.create_subscription(
            TimeReference,
            self.get_parameter("right_eef_time_ref_topic").get_parameter_value().string_value,
            self.right_eef_time_ref_cb,
            50,
        )
        self.create_subscription(ToolTrackingData, self.marker_topic, self.marker_cb, 100)
        self.stop_req_pub = self.create_publisher(Bool, "/dual_system/stop_after_next_home", 10)
        self.timer = self.create_timer(0.1, self.check_finish)

        self.get_logger().info(
            "Recorder waiting first frames from both EEF topics and both markers: "
            f"[left={left.marker_name}, right={right.marker_name}]"
        )

    def stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def stamp_to_ns_key(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def parse_tracker_timespec(self, timespec: str) -> float:
        # SDK format: YYYY-MM-DD HH:MM:SS:ms
        dt = datetime.strptime(timespec, "%Y-%m-%d %H:%M:%S:%f")
        return dt.timestamp()

    def update_latest_stamp(self, stamp_sec: float) -> None:
        if self.latest_observed_stamp_sec is None or stamp_sec > self.latest_observed_stamp_sec:
            self.latest_observed_stamp_sec = stamp_sec

    def all_streams_ready(self) -> bool:
        for arm in self.arms.values():
            if not arm.eef.has_first or not arm.marker.has_first:
                return False
        return True

    def maybe_start(self) -> None:
        if self.start_stamp_sec is not None:
            return
        if not self.all_streams_ready():
            return

        all_first = []
        for arm in self.arms.values():
            all_first.append(arm.eef.first_stamp_sec)
            all_first.append(arm.marker.first_stamp_sec)
        self.start_stamp_sec = max(all_first)
        self.get_logger().info(f"Started raw recording in ROS-time for {self.duration_sec:.1f}s")

    def _ingest_first(self, series: PoseSeries, stamp_sec: float) -> None:
        if not series.has_first:
            series.first_stamp_sec = stamp_sec
            series.has_first = True

    def _within_window(self, stamp_sec: float) -> bool:
        if self.start_stamp_sec is None:
            return False
        dt = stamp_sec - self.start_stamp_sec
        return 0.0 <= dt <= self.duration_sec

    def _record(
        self,
        series: PoseSeries,
        stamp_sec: float,
        device_clock_sec: float,
        pos: List[float],
        quat_xyzw: List[float],
    ) -> None:
        series.stamp_sec.append(stamp_sec)
        series.device_clock_sec.append(device_clock_sec)
        series.pos.append(pos)
        series.quat_xyzw.append(quat_xyzw)

    def left_eef_cb(self, msg: PoseStamped) -> None:
        self.eef_cb("left", msg)

    def right_eef_cb(self, msg: PoseStamped) -> None:
        self.eef_cb("right", msg)

    def left_eef_time_ref_cb(self, msg: TimeReference) -> None:
        self.eef_time_ref_cb("left", msg)

    def right_eef_time_ref_cb(self, msg: TimeReference) -> None:
        self.eef_time_ref_cb("right", msg)

    def eef_time_ref_cb(self, arm_key: str, msg: TimeReference) -> None:
        arm = self.arms[arm_key]
        ros_ns_key = self.stamp_to_ns_key(msg.header.stamp)
        device_sec = self.stamp_to_sec(msg.time_ref)
        arm.pending_eef_device_clock_by_ros_ns[ros_ns_key] = device_sec

    def eef_cb(self, arm_key: str, msg: PoseStamped) -> None:
        if self.finished:
            return

        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        ros_ns_key = self.stamp_to_ns_key(msg.header.stamp)
        self.update_latest_stamp(stamp_sec)
        arm = self.arms[arm_key]
        self._ingest_first(arm.eef, stamp_sec)
        self.maybe_start()
        if not self._within_window(stamp_sec):
            return

        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        device_clock_sec = arm.pending_eef_device_clock_by_ros_ns.pop(ros_ns_key, stamp_sec)
        if not np.isfinite(device_clock_sec):
            device_clock_sec = stamp_sec
        self._record(arm.eef, stamp_sec, device_clock_sec, pos, quat)

    def marker_cb(self, msg: ToolTrackingData) -> None:
        if self.finished:
            return

        arm_key = self.marker_name_to_arm.get(msg.name.lower())
        if arm_key is None:
            return

        stamp_sec = self.stamp_to_sec(msg.header.stamp)
        self.update_latest_stamp(stamp_sec)
        arm = self.arms[arm_key]
        self._ingest_first(arm.marker, stamp_sec)
        self.maybe_start()
        if not self._within_window(stamp_sec):
            return

        pos = [
            msg.transform.tx / 1000.0,
            msg.transform.ty / 1000.0,
            msg.transform.tz / 1000.0,
        ]
        quat = [
            msg.transform.qx,
            msg.transform.qy,
            msg.transform.qz,
            msg.transform.qw,
        ]
        try:
            marker_device_clock_sec = self.parse_tracker_timespec(msg.timespec)
        except Exception:
            marker_device_clock_sec = stamp_sec
        self._record(arm.marker, stamp_sec, marker_device_clock_sec, pos, quat)

    def elapsed_now(self) -> float:
        if self.start_stamp_sec is None or self.latest_observed_stamp_sec is None:
            return 0.0
        return self.latest_observed_stamp_sec - self.start_stamp_sec

    def check_finish(self) -> None:
        if self.finished or self.start_stamp_sec is None:
            return
        if self.elapsed_now() >= self.duration_sec:
            self.finished = True
            self.get_logger().info("Recording complete, saving .npz ...")
            threading.Thread(target=self.save_and_shutdown, daemon=True).start()

    def save_and_shutdown(self) -> None:
        out_path = self.output_data
        save_as_npy = out_path.lower().endswith(".npy")
        if not (save_as_npy or out_path.lower().endswith(".npz")):
            out_path = f"{out_path}.npy"
            save_as_npy = True
        out_dir = os.path.dirname(out_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        payload = {
            "start_stamp_sec": np.array([self.start_stamp_sec], dtype=np.float64),
            "duration_sec": np.array([self.duration_sec], dtype=np.float64),
            "left_marker_name": np.array([self.arms["left"].marker_name], dtype=object),
            "right_marker_name": np.array([self.arms["right"].marker_name], dtype=object),
        }

        for arm_key in ["left", "right"]:
            arm = self.arms[arm_key]
            payload[f"{arm_key}_eef_stamp_sec"] = np.array(arm.eef.stamp_sec, dtype=np.float64)
            payload[f"{arm_key}_eef_device_clock_sec"] = np.array(arm.eef.device_clock_sec, dtype=np.float64)
            payload[f"{arm_key}_eef_pos"] = np.array(arm.eef.pos, dtype=np.float64)
            payload[f"{arm_key}_eef_quat_xyzw"] = np.array(arm.eef.quat_xyzw, dtype=np.float64)
            payload[f"{arm_key}_marker_stamp_sec"] = np.array(arm.marker.stamp_sec, dtype=np.float64)
            payload[f"{arm_key}_marker_device_clock_sec"] = np.array(arm.marker.device_clock_sec, dtype=np.float64)
            payload[f"{arm_key}_marker_pos"] = np.array(arm.marker.pos, dtype=np.float64)
            payload[f"{arm_key}_marker_quat_xyzw"] = np.array(arm.marker.quat_xyzw, dtype=np.float64)

        if save_as_npy:
            np.save(out_path, payload, allow_pickle=True)
        else:
            np.savez(out_path, **payload)
        self.get_logger().info(f"Raw EEF/marker data saved to {out_path}")
        self.stop_req_pub.publish(Bool(data=True))
        self.get_logger().info("Stop request sent. Waiting dual_ur5e_rtde_node to reach next home and exit.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualEefMarkerDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
