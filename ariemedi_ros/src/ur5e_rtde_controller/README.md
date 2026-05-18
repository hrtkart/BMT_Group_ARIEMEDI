# ur5e_rtde_controller

ROS 2 package for UR5e RTDE control, EEF pose publishing, and dual-arm timing analysis.

## Nodes and Scripts

- `ur5e_rtde_node`
  - Connects to a single UR5e via RTDE.
  - Publishes `geometry_msgs/PoseStamped` on `/ur5e/eef_pose`.
  - Executes a repeatable reciprocating motion along local -Y of the EEF.

- `dual_ur5e_rtde_node`
  - Connects to left/right UR5e arms and publishes `/left_arm/eef_pose` and `/right_arm/eef_pose`.
  - Generates randomized motion around each arm home pose.
  - Publishes `/left_arm/eef_time_reference` and `/right_arm/eef_time_reference` (device clock).

- `eef_marker_relative_plot`
  - Records EEF and tracker data and plots local frame deltas for a single arm.

- `dual_eef_marker_data_recorder`
  - Records dual-arm EEF + tracker data into `.npz` (ROS time + device time).

- `dual_eef_marker_relative_plot`
  - Offline plot from `.npz` with raw timestamps (no time alignment).

- `dual_eef_marker_time_aligned_plot_from_numpy`
  - Offline plot from `.npz` with time alignment (drift compensation).

- `test_dual_base_transform`
  - CLI tool to validate the dual-base transform by moving the left arm and comparing tracker data.

## Dependencies

This package uses Python bindings from `ur-rtde`:

```bash
/usr/bin/python3 -m pip install ur-rtde
```

## Build

```bash
cd /home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
# Source your ROS2 underlay first (example: /opt/ros/<distro>/setup.bash)
source /path/to/ros2/setup.bash
colcon build --packages-select ur5e_rtde_controller
source install/setup.bash
```

## Run: Single UR5e Motion + Pose Publish

```bash
ros2 run ur5e_rtde_controller ur5e_rtde_node
```

Optional parameters:
- `robot_ip` (string, default: `192.168.56.5`)
- `publish_rate_hz` (double, default: `50.0`)
- `travel_distance_m` (double, default: `0.20`)
- `linear_speed_mps` (double, default: `0.03`)
- `linear_acc_mps2` (double, default: `0.20`)
- `pause_at_switch_sec` (double, default: `2.0`)

Example:

```bash
ros2 run ur5e_rtde_controller ur5e_rtde_node --ros-args \
  -p robot_ip:=192.168.56.5 \
  -p travel_distance_m:=0.10 \
  -p pause_at_switch_sec:=2.0
```

## Run: UR5e + Tracker Combined (Launch)

```bash
ros2 launch ur5e_rtde_controller ur5e_armd.launch.py robot_ip:=192.168.56.5
```

## Record and Plot: Single Arm

```bash
ros2 run ur5e_rtde_controller eef_marker_relative_plot
```

Optional:

```bash
ros2 run ur5e_rtde_controller eef_marker_relative_plot --ros-args \
  -p duration_sec:=60.0 \
  -p output_plot:=/tmp/eef_marker_relative_xyz.png
```

## Dual-arm Workflow: Record Then Plot

1) Start dual-arm motion + tracker recording:

```bash
ros2 launch ur5e_rtde_controller dual_ur5e_tracker_compare.launch.py \
  duration_sec:=60.0 \
  output_data:=/home/us-mrc/Documents/dual_arm_eef_marker_raw.npz
```

2) Plot offline (raw timestamps):

```bash
ros2 run ur5e_rtde_controller dual_eef_marker_relative_plot -- \
  --input-data /home/us-mrc/Documents/dual_arm_eef_marker_raw.npz \
  --output-plot /home/us-mrc/Documents/dual_arm_eef_marker_relative_xyz.png
```

3) Plot offline (time aligned):

```bash
ros2 run ur5e_rtde_controller dual_eef_marker_time_aligned_plot_from_numpy -- \
  --input-data /home/us-mrc/Documents/dual_arm_eef_marker_raw.npz \
  --output-plot /home/us-mrc/Documents/dual_arm_eef_marker_time_aligned_xyz.png
```

## Dual-base Transform Test

```bash
ros2 run ur5e_rtde_controller test_dual_base_transform -- \
  --marker_topic /ARMDpos \
  --pause_sec 2.0 \
  --json /home/us-mrc/Documents/BMT_Group_ARIEMEDI/dual_arm_cali/dual_base_left_to_right.json
```
