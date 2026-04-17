# us_screen_capture_ros2

ROS 2 package for publishing cropped ultrasound screen images from a fixed USB camera.

## Behavior

- Captures from a fixed video device: `/dev/video2`
- Crops a fixed ROI: `x[626,1294), y[199,855)`
- Converts to grayscale (`mono8`)
- Publishes to topic: `frame_grabber/us_img`
- Publishes only when frame content changes
- Reconnects automatically if the camera stream drops

## Build

```bash
cd /home/kart_laptop/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
colcon build --packages-select us_screen_capture_ros2
```

## Run

```bash
source /home/kart_laptop/Documents/BMT_Group_ARIEMEDI/ariemedi_ros/install/setup.bash
ros2 run us_screen_capture_ros2 us_screen_pub_node
```
