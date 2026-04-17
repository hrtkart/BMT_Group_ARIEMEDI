#!/usr/bin/python3

from __future__ import annotations

from typing import Union

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import glob
import os

# Default crop region (pixel coordinates).
# Used only when enable_crop:=true.
DEFAULT_CROP = {
    "y0": 70,
    "y1": 930,
    "x0": 521,
    "x1": 1401,
}

def get_device_name(device_path):
    """获取video设备的名称"""
    try:
        # 从 /dev/video0 提取 video0
        device_name = os.path.basename(device_path)
        # 读取设备名称文件
        name_file = f'/sys/class/video4linux/{device_name}/name'
        if os.path.exists(name_file):
            with open(name_file, 'r') as f:
                return f.read().strip()
    except:
        pass
    return None

def find_device_by_name(target_name):
    """根据设备名称查找video设备路径
    Args:
        target_name: 要搜索的设备名称（支持部分匹配，不区分大小写）
    Returns:
        匹配的设备路径，如果未找到则返回None
    """
    devices = glob.glob('/dev/video*')
    
    for device in sorted(devices):
        device_name = get_device_name(device)
        if device_name and target_name.lower() in device_name.lower():
            return device, device_name
    
    return None, None

class USScreenPub(Node):
    def __init__(self):
        super().__init__('us_screen_pub_node')
        self.bridge = CvBridge()
        self.pub_us_img = self.create_publisher(Image, "/us_img", 0)
        self.frame_fail_count = 0
        self.last_error_log_time = 0

        # Parameters (can be overridden by ros2 run ... --ros-args -p key:=value)
        self.declare_parameter("video_device", "/dev/video0")  # can be "/dev/video2" or "2"
        self.declare_parameter("frame_width", 1920)
        self.declare_parameter("frame_height", 1080)
        self.declare_parameter("publish_hz", 60.0)
        # Switch: crop (True) vs publish full frame (False)
        self.declare_parameter("enable_crop", True)
        # Resize multiplier (1.0 = no resize, 0.5 = half, 2.0 = double)
        self.declare_parameter("resize_scale", 0.5)

        # Cropping parameters (defined in-code)
        self.x0 = int(DEFAULT_CROP["x0"])
        self.x1 = int(DEFAULT_CROP["x1"])
        self.y0 = int(DEFAULT_CROP["y0"])
        self.y1 = int(DEFAULT_CROP["y1"])

        # Capture - 自动查找HDMI Capture设备
        device_param = self.get_parameter("video_device").get_parameter_value().string_value
        
        # 尝试查找名为 "HDMI Capture" 的设备
        self.get_logger().info("Searching for 'HDMI Capture' device...")
        hdmi_device, hdmi_name = find_device_by_name("HDMI Capture")
        
        if hdmi_device:
            device = hdmi_device
            self.get_logger().info(f"Found HDMI Capture device: {device} ({hdmi_name})")
        else:
            device = device_param
            self.get_logger().warn(f"HDMI Capture device not found, using parameter: {device}")
            # 显示当前设备的名称（如果有）
            current_name = get_device_name(device)
            if current_name:
                self.get_logger().info(f"Current device name: {current_name}")
        
        w = int(self.get_parameter("frame_width").get_parameter_value().integer_value)
        h = int(self.get_parameter("frame_height").get_parameter_value().integer_value)

        self.screen_cap = self._open_capture(device=device)
        if self.screen_cap is not None:
            self.screen_cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.screen_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.get_logger().info(f"VideoCapture opened: device={device}, size={(self.y1 - self.y0)}x{self.x1 - self.x0}")
        else:
            self.get_logger().error(
                "Failed to open video device. Common fixes: check `/dev/video*`, permissions, device busy, or change "
                "parameter `video_device` (e.g. /dev/video0)."
            )

        hz = float(self.get_parameter("publish_hz").get_parameter_value().double_value)
        hz = hz if hz > 0 else 30.0
        self.timer = self.create_timer(1.0 / hz, self.timer_callback)

    def _open_capture(self, device: str):
        # Accept "/dev/video2" or "2"
        src: Union[str, int] = int(device) if device.isdigit() else device

        # Always try V4L2 first, then fall back to OpenCV's auto backend.
        apis = [("v4l2", cv2.CAP_V4L2), ("auto", None)]

        for name, api in apis:
            try:
                cap = cv2.VideoCapture(src) if api is None else cv2.VideoCapture(src, api)
                if cap.isOpened():
                    return cap
                cap.release()
            except Exception as e:
                self.get_logger().warn(f"VideoCapture open failed via {name}: {e}")
        return None

    def timer_callback(self):
        # For testing, publish a dummy image if no camera
        if self.screen_cap is None or (hasattr(self.screen_cap, "isOpened") and not self.screen_cap.isOpened()):
            # Create an error message image
            dummy_img = np.zeros((480, 640), dtype=np.uint8)
            cv2.putText(dummy_img, "HDMI Capture Device", (100, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(dummy_img, "Not Found", (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(dummy_img, "Check connection", (150, 300), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, 200, 2)
            msg_img = self.bridge.cv2_to_imgmsg(dummy_img)
            self.pub_us_img.publish(msg_img)
            return

        ret, frame = self.screen_cap.read()
        if ret:
            # Reset fail counter on success
            self.frame_fail_count = 0
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            enable_crop = bool(self.get_parameter("enable_crop").get_parameter_value().bool_value)
            if enable_crop:
                # Clamp crop bounds to the frame
                h, w = gray.shape[:2]
                x0 = max(0, min(self.x0, w))
                x1 = max(0, min(self.x1, w))
                y0 = max(0, min(self.y0, h))
                y1 = max(0, min(self.y1, h))
                gray = gray[y0:y1, x0:x1]

            scale = float(self.get_parameter("resize_scale").get_parameter_value().double_value)
            if scale > 0.0 and abs(scale - 1.0) > 1e-6:
                gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

            msg_img = self.bridge.cv2_to_imgmsg(gray, encoding="mono8")
            self.pub_us_img.publish(msg_img)

        else:
            # Frame read failed - publish error image
            self.frame_fail_count += 1
            
            # Create error message image
            error_img = np.zeros((480, 640), dtype=np.uint8)
            cv2.putText(error_img, "Camera Read Error", (120, 180), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(error_img, f"Failed: {self.frame_fail_count} times", (130, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, 200, 2)
            cv2.putText(error_img, "Check HDMI source", (150, 300), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, 180, 2)
            
            msg_img = self.bridge.cv2_to_imgmsg(error_img, encoding="mono8")
            self.pub_us_img.publish(msg_img)
            
            # Log warning occasionally (every 5 seconds)
            import time
            current_time = time.time()
            if current_time - self.last_error_log_time > 5.0:
                self.get_logger().warn(f"VideoCapture read() failed ({self.frame_fail_count} times)")
                self.last_error_log_time = current_time
    

if __name__ == '__main__':
    rclpy.init()
    node = USScreenPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "screen_cap", None) is not None:
            node.screen_cap.release()
        try:
            rclpy.shutdown()
        except:
            pass
