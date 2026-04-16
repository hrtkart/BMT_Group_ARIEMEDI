#!/usr/bin/env python3.10
import math
from time import time

import rclpy
from rclpy.node import Node
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from ariemedi_tracker.msg import ToolTrackingData


class MarkerRelativeRotationLogger(Node):
    def __init__(self):
        super().__init__('marker_relative_rotation_logger')

        self.declare_parameter('topic', '/ARMDpos')
        self.declare_parameter('duration_sec', 60.0)
        self.declare_parameter('output_png', '/tmp/marker_relative_rotation_plot.png')

        topic = self.get_parameter('topic').value
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.output_png = str(self.get_parameter('output_png').value)

        self.subscription = self.create_subscription(
            ToolTrackingData,
            topic,
            self.listener_callback,
            10,
        )

        self.start_time = None
        self.frame_count = 0
        self.first_quat = None
        self.times = []
        self.rx_deg = []
        self.ry_deg = []
        self.rz_deg = []
        self.timespecs = []

        self.get_logger().info(
            f'Started relative rotation recording. topic={topic}, duration={self.duration_sec}s, output={self.output_png}'
        )

    @staticmethod
    def normalize_quaternion(q):
        w, x, y, z = q
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm <= 1e-12:
            return (1.0, 0.0, 0.0, 0.0)
        inv = 1.0 / norm
        return (w * inv, x * inv, y * inv, z * inv)

    @staticmethod
    def quat_conjugate(q):
        w, x, y, z = q
        return (w, -x, -y, -z)

    @staticmethod
    def quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    @staticmethod
    def quat_to_euler_xyz_deg(q):
        w, x, y, z = q

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch_y = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch_y = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(siny_cosp, cosy_cosp)

        return (
            math.degrees(roll_x),
            math.degrees(pitch_y),
            math.degrees(yaw_z),
        )

    def listener_callback(self, msg):
        current_time = time()
        if self.start_time is None:
            self.start_time = current_time

        elapsed = current_time - self.start_time
        if elapsed > self.duration_sec:
            self.flush_and_shutdown()
            return

        q_cur = self.normalize_quaternion(
            (
                float(msg.transform.qw),
                float(msg.transform.qx),
                float(msg.transform.qy),
                float(msg.transform.qz),
            )
        )

        if self.first_quat is None:
            self.first_quat = q_cur
            self.get_logger().info('First frame orientation captured as reference frame.')

        # Rotation from first frame to current frame: q_rel = inv(q_first) * q_current
        q_rel = self.quat_multiply(self.quat_conjugate(self.first_quat), q_cur)
        q_rel = self.normalize_quaternion(q_rel)

        rx_deg, ry_deg, rz_deg = self.quat_to_euler_xyz_deg(q_rel)

        self.frame_count += 1
        self.times.append(elapsed)
        self.rx_deg.append(rx_deg)
        self.ry_deg.append(ry_deg)
        self.rz_deg.append(rz_deg)
        self.timespecs.append(msg.timespec)

        self.get_logger().info(
            f'Frame {self.frame_count} | dRx={rx_deg:.5f} deg dRy={ry_deg:.5f} deg dRz={rz_deg:.5f} deg | TS={msg.timespec}'
        )

    def flush_and_shutdown(self):
        if len(self.times) == 0:
            self.get_logger().warn('No data received, skip plotting.')
            rclpy.shutdown()
            return

        plt.figure('Marker Relative Rotation (First Frame Reference)', figsize=(10, 8))

        plt.subplot(3, 1, 1)
        plt.plot(self.times, self.rx_deg, 'r-', linewidth=1.2, label='dRx')
        plt.ylabel('dRx (deg)')
        plt.grid(True)
        plt.legend(loc='upper right')

        plt.subplot(3, 1, 2)
        plt.plot(self.times, self.ry_deg, 'g-', linewidth=1.2, label='dRy')
        plt.ylabel('dRy (deg)')
        plt.grid(True)
        plt.legend(loc='upper right')

        plt.subplot(3, 1, 3)
        plt.plot(self.times, self.rz_deg, 'b-', linewidth=1.2, label='dRz')
        plt.xlabel('Time (s)')
        plt.ylabel('dRz (deg)')
        plt.grid(True)
        plt.legend(loc='upper right')

        plt.suptitle('Relative Rotation Per Frame (Current vs First Frame)')
        plt.tight_layout()
        plt.savefig(self.output_png, dpi=150, bbox_inches='tight')
        plt.close()

        self.get_logger().info(
            f'Recorded {len(self.times)} frames. Rotation plot written to {self.output_png}'
        )
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerRelativeRotationLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
