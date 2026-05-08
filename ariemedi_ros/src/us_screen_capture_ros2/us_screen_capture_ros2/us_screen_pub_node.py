#!/usr/bin/env python3

import cv2
import threading
import time
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class UltrasoundScreenPublisher(Node):
    """Capture frames from a fixed USB camera, crop ROI, and publish grayscale images."""

    VIDEO_DEVICE = "/dev/video2"
    TOPIC_NAME = "frame_grabber/us_img"

    # Cropping region in source image coordinates: x in [X0, X1), y in [Y0, Y1)
    # Recomputed from frame_000037_0_000000000.png (1920x1080),
    # tightly covering the central ultrasound image area.
    # Approximate corners: LT(401,91), RT(1516,91), RB(1516,986), LB(401,986)
    # Python slicing is end-exclusive, so X1/Y1 are right/bottom + 1.
    X0 = 401
    X1 = 1517
    Y0 = 91
    Y1 = 987

    TIMER_PERIOD_SEC = 0.005
    RECONNECT_INTERVAL_SEC = 1.0
    STATS_INTERVAL_SEC = 1.0
    CAPTURE_IDLE_SLEEP_SEC = 0.002

    def __init__(self) -> None:
        super().__init__("us_screen_pub_node")
        self._publisher = self.create_publisher(Image, self.TOPIC_NAME, 1)
        self.declare_parameter("resize_scale", 1)
        self._resize_scale = float(self.get_parameter("resize_scale").get_parameter_value().double_value)

        self._bridge = CvBridge() if CvBridge is not None else None
        if self._bridge is None:
            self.get_logger().warning("cv_bridge not available, using slower raw Image conversion path")

        self._cap = None
        self._last_reconnect_time = 0.0
        self._roi_checked = False
        self._running = True
        self._data_lock = threading.Lock()
        self._latest_gray = None
        self._latest_seq = 0
        self._last_published_seq = 0

        self._loop_count = 0
        self._read_ok_count = 0
        self._read_fail_count = 0
        self._publish_count = 0
        self._stat_window_start_sec = self.get_clock().now().nanoseconds / 1e9

        self._open_capture(force_log=True)
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self._timer = self.create_timer(self.TIMER_PERIOD_SEC, self._on_timer)

    def _open_capture(self, force_log: bool = False) -> bool:
        if self._cap is not None and self._cap.isOpened():
            return True

        cap = cv2.VideoCapture(self.VIDEO_DEVICE, cv2.CAP_V4L2)
        # Keep native YUYV and avoid full-frame BGR conversion to reduce CPU load.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 60)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            cap.release()
            if force_log:
                self.get_logger().warning(
                    f"Unable to open video device: {self.VIDEO_DEVICE}. Retrying..."
                )
            return False

        self._cap = cap
        self._roi_checked = False
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"Connected to video device: {self.VIDEO_DEVICE}, mode={actual_w}x{actual_h}@{actual_fps:.1f}"
        )
        return True

    def _close_capture(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def _validated_roi(self, frame_shape):
        height, width = frame_shape[:2]

        x0 = max(0, min(self.X0, width))
        x1 = max(0, min(self.X1, width))
        y0 = max(0, min(self.Y0, height))
        y1 = max(0, min(self.Y1, height))

        if x1 <= x0 or y1 <= y0:
            return None
        return x0, x1, y0, y1

    def _on_timer(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._loop_count += 1

        gray_to_publish = None
        with self._data_lock:
            if self._latest_seq > self._last_published_seq and self._latest_gray is not None:
                gray_to_publish = self._latest_gray
                self._last_published_seq = self._latest_seq

        if gray_to_publish is not None:
            msg = self._gray_to_image_msg(gray_to_publish)
            self._publisher.publish(msg)
            self._publish_count += 1

        self._maybe_log_stats(now_sec)

    def _capture_loop(self) -> None:
        while self._running:
            if self._cap is None or not self._cap.isOpened():
                now_sec = self.get_clock().now().nanoseconds / 1e9
                if now_sec - self._last_reconnect_time >= self.RECONNECT_INTERVAL_SEC:
                    self._last_reconnect_time = now_sec
                    self._open_capture(force_log=True)
                time.sleep(self.CAPTURE_IDLE_SLEEP_SEC)
                continue

            ret, frame = self._cap.read()
            if not ret or frame is None:
                self._read_fail_count += 1
                self.get_logger().warning("Invalid frame read, reconnecting camera...")
                self._close_capture()
                time.sleep(self.CAPTURE_IDLE_SLEEP_SEC)
                continue

            self._read_ok_count += 1
            roi = self._validated_roi(frame.shape)
            if roi is None:
                if not self._roi_checked:
                    self.get_logger().error(
                        "Configured ROI is invalid for current frame size. "
                        f"Frame size: {frame.shape[1]}x{frame.shape[0]}, "
                        f"ROI: x[{self.X0},{self.X1}), y[{self.Y0},{self.Y1})"
                    )
                    self._roi_checked = True
                continue

            x0, x1, y0, y1 = roi
            if len(frame.shape) == 3 and frame.shape[2] >= 2:
                gray = frame[y0:y1, x0:x1, 0].copy()
            else:
                cropped = frame[y0:y1, x0:x1]
                gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY) if len(cropped.shape) == 3 else cropped

            if self._resize_scale > 0.0 and abs(self._resize_scale - 1.0) > 1e-6:
                gray = cv2.resize(gray, None, fx=self._resize_scale, fy=self._resize_scale, interpolation=cv2.INTER_AREA)

            with self._data_lock:
                self._latest_gray = gray
                self._latest_seq += 1

    def _maybe_log_stats(self, now_sec: float) -> None:
        elapsed = now_sec - self._stat_window_start_sec
        if elapsed < self.STATS_INTERVAL_SEC:
            return

        read_hz = self._read_ok_count / elapsed if elapsed > 0.0 else 0.0
        publish_hz = self._publish_count / elapsed if elapsed > 0.0 else 0.0
        fail_hz = self._read_fail_count / elapsed if elapsed > 0.0 else 0.0

        if self._read_ok_count == 0:
            diagnosis = "camera_bottleneck(no_valid_frames)"
        elif self._publish_count < self._read_ok_count * 0.95:
            diagnosis = "publish_path_bottleneck"
        else:
            diagnosis = "camera_limited_or_balanced"

        self.get_logger().info(
            "stats(1s): "
            f"loops={self._loop_count}, "
            f"read_ok={self._read_ok_count} ({read_hz:.1f}Hz), "
            f"read_fail={self._read_fail_count} ({fail_hz:.1f}Hz), "
            f"publish={self._publish_count} ({publish_hz:.1f}Hz), "
            f"diagnosis={diagnosis}"
        )

        self._loop_count = 0
        self._read_ok_count = 0
        self._read_fail_count = 0
        self._publish_count = 0
        self._stat_window_start_sec = now_sec

    def _gray_to_image_msg(self, gray):
        if self._bridge is not None:
            return self._bridge.cv2_to_imgmsg(gray, encoding="mono8")

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = gray.shape[0]
        msg.width = gray.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = gray.shape[1]
        msg.data = gray.tobytes()
        return msg

    def destroy_node(self):
        self._running = False
        if hasattr(self, "_capture_thread") and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        self._close_capture()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasoundScreenPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
