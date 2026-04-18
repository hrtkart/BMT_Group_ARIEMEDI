#!/usr/bin/python3

import math
import threading
import time
from typing import List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from ariemedi_tracker.msg import ToolTrackingData
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def quat_to_rotmat(qw: float, qx: float, qy: float, qz: float) -> List[List[float]]:
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm

    return [
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ]


def rotate_with_transpose(r: List[List[float]], v: List[float]) -> List[float]:
    return [
        r[0][0] * v[0] + r[1][0] * v[1] + r[2][0] * v[2],
        r[0][1] * v[0] + r[1][1] * v[1] + r[2][1] * v[2],
        r[0][2] * v[0] + r[1][2] * v[1] + r[2][2] * v[2],
    ]


class RelativeRecorder(Node):
    def __init__(self) -> None:
        super().__init__("eef_marker_relative_plotter")

        self.declare_parameter("duration_sec", 300.0)
        self.declare_parameter("output_plot", "/tmp/eef_marker_relative_xyz.png")

        self.duration_sec = self.get_parameter("duration_sec").get_parameter_value().double_value
        self.output_plot = self.get_parameter("output_plot").get_parameter_value().string_value

        self.eef_sub = self.create_subscription(PoseStamped, "/ur5e/eef_pose", self.eef_cb, 50)
        self.marker_sub = self.create_subscription(ToolTrackingData, "/ARMDpos", self.marker_cb, 50)

        self.timer = self.create_timer(0.1, self.check_finish)

        self.start_time = None
        self.finished = False

        self.eef_ref_pos = None
        self.eef_ref_rot = None

        self.marker_ref_pos = None
        self.marker_ref_rot = None

        self.eef_t = []
        self.eef_x = []
        self.eef_y = []
        self.eef_z = []

        self.marker_t = []
        self.marker_x = []
        self.marker_y = []
        self.marker_z = []

        self.get_logger().info("Waiting first frames from /ur5e/eef_pose and /ARMDpos ...")

    def maybe_start(self) -> None:
        if self.start_time is not None:
            return
        if self.eef_ref_pos is not None and self.marker_ref_pos is not None:
            self.start_time = time.time()
            self.get_logger().info(f"Started synchronized recording for {self.duration_sec:.1f}s")

    def elapsed(self) -> float:
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time

    def eef_cb(self, msg: PoseStamped) -> None:
        if self.finished:
            return

        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        qw = msg.pose.orientation.w
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z

        if self.eef_ref_pos is None:
            self.eef_ref_pos = [px, py, pz]
            self.eef_ref_rot = quat_to_rotmat(qw, qx, qy, qz)
            self.get_logger().info("Captured UR5e EEF first-frame reference")
            self.maybe_start()
            return

        if self.start_time is None:
            self.maybe_start()
            return

        dt = self.elapsed()
        if dt > self.duration_sec:
            return

        delta_world = [
            px - self.eef_ref_pos[0],
            py - self.eef_ref_pos[1],
            pz - self.eef_ref_pos[2],
        ]
        delta_local = rotate_with_transpose(self.eef_ref_rot, delta_world)

        self.eef_t.append(dt)
        self.eef_x.append(delta_local[0])
        self.eef_y.append(delta_local[1])
        self.eef_z.append(delta_local[2])

    def marker_cb(self, msg: ToolTrackingData) -> None:
        if self.finished:
            return

        px = msg.transform.tx / 1000.0
        py = msg.transform.ty / 1000.0
        pz = msg.transform.tz / 1000.0

        qw = msg.transform.qw
        qx = msg.transform.qx
        qy = msg.transform.qy
        qz = msg.transform.qz

        if self.marker_ref_pos is None:
            self.marker_ref_pos = [px, py, pz]
            self.marker_ref_rot = quat_to_rotmat(qw, qx, qy, qz)
            self.get_logger().info("Captured marker first-frame reference")
            self.maybe_start()
            return

        if self.start_time is None:
            self.maybe_start()
            return

        dt = self.elapsed()
        if dt > self.duration_sec:
            return

        delta_world = [
            px - self.marker_ref_pos[0],
            py - self.marker_ref_pos[1],
            pz - self.marker_ref_pos[2],
        ]
        delta_local = rotate_with_transpose(self.marker_ref_rot, delta_world)

        self.marker_t.append(dt)
        self.marker_x.append(delta_local[0])
        self.marker_y.append(delta_local[1])
        self.marker_z.append(delta_local[2])

    def check_finish(self) -> None:
        if self.finished:
            return
        if self.start_time is None:
            return

        if self.elapsed() >= self.duration_sec:
            self.finished = True
            self.get_logger().info("Recording complete, generating plots ...")
            threading.Thread(target=self.plot_and_shutdown, daemon=True).start()

    def plot_and_shutdown(self) -> None:
        if len(self.eef_t) == 0:
            self.get_logger().warn("No UR5e EEF data recorded")
        if len(self.marker_t) == 0:
            self.get_logger().warn("No marker data recorded")

        out_path = self.output_plot
        if not out_path.lower().endswith((".png", ".jpg", ".jpeg", ".pdf", ".svg")):
            out_path = f"{out_path}.png"

        # Axis mapping requested by user, all compared in marker coordinate convention:
        # marker X <-> eef Y, marker Z <-> eef X, remaining axis paired as marker Y <-> eef Z.
        pairs = [
            ("X", self.marker_t, self.marker_x, self.eef_t, self.eef_y),
            ("Y", self.marker_t, self.marker_y, self.eef_t, self.eef_z),
            ("Z", self.marker_t, self.marker_z, self.eef_t, self.eef_x),
        ]

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

        for idx, (axis_name, t_m, y_m, t_e, y_e) in enumerate(pairs):
            ax = axes[idx]
            if t_m and y_m:
                ax.plot(t_m, y_m, color="tab:blue", linewidth=1.8, label=f"Marker {axis_name}")
            if t_e and y_e:
                ax.plot(t_e, y_e, color="tab:orange", linewidth=1.6, linestyle="--", label=f"EEF mapped to {axis_name}")

            ax.set_ylabel(f"{axis_name} position (m)")
            ax.grid(True, linestyle=":", alpha=0.6)
            ax.legend(loc="best")
            # Force showing numeric scale on every subplot
            ax.tick_params(axis="both", which="major", labelsize=9)

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("Marker-frame Relative Position: Marker vs EEF (mapped axes)")
        fig.tight_layout()
        fig.savefig(out_path, dpi=160)
        plt.close(fig)

        self.get_logger().info(f"Plot saved to {out_path}")
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelativeRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
