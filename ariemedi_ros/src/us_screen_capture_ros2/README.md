# us_screen_capture_ros2

ROS 2 package that crops a fixed ROI from a USB camera and publishes grayscale ultrasound frames.

## Behavior

- Captures from `/dev/video2` (V4L2).
- Crops a fixed ROI: `x[401,1517)`, `y[91,987)`.
- Converts to grayscale (`mono8`).
- Publishes on topic `frame_grabber/us_img`.
- Runs a capture thread with automatic reconnect and periodic stats logging.

## Parameters

- `resize_scale` (double, default: `1.0`)

## Build

```bash
cd /home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
# Source your ROS2 underlay first (example: /opt/ros/<distro>/setup.bash)
source /path/to/ros2/setup.bash
colcon build --packages-select us_screen_capture_ros2
source install/setup.bash
```

## Run

```bash
ros2 run us_screen_capture_ros2 us_screen_pub_node
```

Example (half resolution):

```bash
ros2 run us_screen_capture_ros2 us_screen_pub_node --ros-args -p resize_scale:=0.5
```

## Save Frames for a Short Window

```bash
python3 /home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros/src/us_screen_capture_ros2/scripts/save_us_img_10s.py \
  --topic /frame_grabber/us_img \
  --duration 10.0
```
