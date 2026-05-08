#!/usr/bin/python3

import argparse
from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self, topic: str, duration_sec: float, output_dir: Path):
        super().__init__("us_img_saver_10s")
        self.topic = topic
        self.duration_sec = duration_sec
        self.output_dir = output_dir

        self.start_time = time.time()
        self.saved_count = 0
        self.unsupported_encoding_warned = False

        self.create_subscription(Image, self.topic, self._callback, 10)
        self.timer = self.create_timer(0.05, self._check_timeout)

        self.get_logger().info(
            f"Saving images from {self.topic} for {self.duration_sec:.1f}s to {self.output_dir}"
        )

    def _check_timeout(self):
        if time.time() - self.start_time >= self.duration_sec:
            self.get_logger().info(
                f"Finished capture window. Saved {self.saved_count} images to {self.output_dir}"
            )
            rclpy.shutdown()

    def _callback(self, msg: Image):
        if time.time() - self.start_time >= self.duration_sec:
            return

        image = self._to_cv_image(msg)
        if image is None:
            return

        stamp = msg.header.stamp
        filename = (
            f"frame_{self.saved_count:06d}_"
            f"{stamp.sec}_{stamp.nanosec:09d}.png"
        )
        file_path = self.output_dir / filename

        ok = cv2.imwrite(str(file_path), image)
        if ok:
            self.saved_count += 1

    def _to_cv_image(self, msg: Image):
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc == "mono8":
            row = data.reshape((msg.height, msg.step))
            return row[:, : msg.width]

        if enc in ("bgr8", "rgb8"):
            row = data.reshape((msg.height, msg.step))
            pixels = row[:, : msg.width * 3]
            img = pixels.reshape((msg.height, msg.width, 3))
            if enc == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img

        if not self.unsupported_encoding_warned:
            self.unsupported_encoding_warned = True
            self.get_logger().error(
                f"Unsupported image encoding: {msg.encoding}. Supported: mono8, bgr8, rgb8"
            )
        return None


def main():
    parser = argparse.ArgumentParser(description="Save /frame_grabber/us_img frames for 10 seconds")
    parser.add_argument("--topic", default="/frame_grabber/us_img", help="Image topic")
    parser.add_argument("--duration", type=float, default=10.0, help="Capture duration in seconds")
    parser.add_argument(
        "--output-dir",
        default="",
        help="Output directory. Default: ./us_img_capture_<timestamp>",
    )
    args = parser.parse_args()

    if args.output_dir:
        output_dir = Path(args.output_dir).expanduser().resolve()
    else:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = (Path.cwd() / f"us_img_capture_{stamp}").resolve()

    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ImageSaver(args.topic, args.duration, output_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
