# us_capture

ROS 2 package that publishes grayscale ultrasound frames from an HDMI capture device.

## Behavior

- Searches for a video device named "HDMI Capture"; falls back to `video_device` parameter.
- Publishes `sensor_msgs/Image` to `/us_img`.
- Optional cropping and resize.
- If the camera fails, publishes a diagnostic dummy frame.

## Parameters

- `video_device` (string, default: `/dev/video0`)
- `frame_width` (int, default: `1920`)
- `frame_height` (int, default: `1080`)
- `publish_hz` (double, default: `60.0`)
- `enable_crop` (bool, default: `true`)
- `resize_scale` (double, default: `0.5`)

## Build

```bash
cd /home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
# Source your ROS2 underlay first (example: /opt/ros/<distro>/setup.bash)
source /path/to/ros2/setup.bash
colcon build --packages-select us_capture
source install/setup.bash
```

## Run

```bash
ros2 run us_capture us_pub
```

Example (no crop, custom device):

```bash
ros2 run us_capture us_pub --ros-args \
  -p video_device:=/dev/video2 \
  -p enable_crop:=false \
  -p resize_scale:=1.0
```
