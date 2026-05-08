# ur5e_rtde_controller

ROS 2 package for UR5e RTDE control and real-time EEF pose publishing.

## Features
- Connects to UR5e via RTDE (default IP: `192.168.56.5`).
- Publishes `base -> eef` pose (`geometry_msgs/PoseStamped`) on topic `ur5e/eef_pose`.
- Executes reciprocal motion in EEF local Y-axis:
  - start from current EEF pose
  - move 10 cm along local `-Y`
  - pause 2 seconds
  - move back to start
  - pause 2 seconds
  - repeat

## Dependency
This node uses Python bindings from **ur-rtde**:

```bash
/usr/bin/python3 -m pip install ur-rtde
```

## Build
```bash
cd /home/kart_laptop/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
# Source your ROS2 underlay first (example: /opt/ros/<distro>/setup.bash)
source /path/to/ros2/setup.bash
colcon build --packages-select ur5e_rtde_controller
source install/setup.bash
```

## Run
```bash
ros2 run ur5e_rtde_controller ur5e_rtde_node
```

## Run UR5e + ARMD Together
```bash
ros2 launch ur5e_rtde_controller ur5e_armd.launch.py robot_ip:=192.168.56.5
```

## Record Relative Trajectories and Plot (60s)
This script records for 60 seconds and plots:
- UR5e EEF position relative to first-frame EEF coordinate frame
- Marker position relative to first-frame marker coordinate frame

```bash
ros2 run ur5e_rtde_controller eef_marker_relative_plot
```

Optional:

```bash
ros2 run ur5e_rtde_controller eef_marker_relative_plot --ros-args -p duration_sec:=60.0 -p output_plot:=/tmp/eef_marker_relative_xyz.png
```

## Optional Parameters
- `robot_ip` (string, default: `192.168.56.5`)
- `publish_rate_hz` (double, default: `50.0`)
- `travel_distance_m` (double, default: `0.10`)
- `linear_speed_mps` (double, default: `0.03`)
- `linear_acc_mps2` (double, default: `0.20`)
- `pause_at_switch_sec` (double, default: `2.0`)

Example:

```bash
ros2 run ur5e_rtde_controller ur5e_rtde_node --ros-args -p robot_ip:=192.168.56.5 -p travel_distance_m:=0.10 -p pause_at_switch_sec:=2.0
```
