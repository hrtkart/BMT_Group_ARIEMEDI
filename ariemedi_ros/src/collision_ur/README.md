# collision_ur

ROS 2 Jazzy implementation of the lightweight dual-arm kinematic collision
avoidance method described in DAISS (arXiv:2603.07663v1). The package is
integrated with the ROS 2 version of the Popeye dual-UR5e model in
`popeye_description`.

The complete RViz view contains:

- Popeye torso and both UR5e visual meshes.
- Live or demonstration joint positions for all 12 joints.
- Three green left and three yellow right distal protection cylinders.
- A line connecting the nearest points on the most critical cylinder pair.
- Transparent Cartesian workspace boxes.
- Interactive controls for editing cylinders, workspaces, and clearance.

## Model And Collision Geometry

The ROS 2 model is based on:

```text
/home/us-mrc/Documents/BMT_UR5/src/ur5e_robot/urdf/popeye.urdf
/home/us-mrc/Documents/BMT_UR5/src/ur5e_robot/urdf/popeye.rviz
```

The migrated description is installed by `popeye_description`. Meshes are
copied into that package, so the ROS 1 workspace is not required at runtime.

The right mounting transform was updated to match:

```text
/home/us-mrc/Documents/BMT_Group_ARIEMEDI/dual_arm_cali/
dual_base_left_to_right_v2.json
```

Therefore the RobotModel and collision geometry use the same measured
`left_base -> right_base` transform.

Following the paper's distal-segment abstraction, each arm uses three finite
cylinders:

```text
wrist_2: follows L/R_wrist_2_link TF, radius 0.06 m
wrist_3: follows L/R_wrist_3_link TF, radius 0.06 m
tool: wrist-3-centered TCP local +Z, from 0.00 m to 0.20 m, radius 0.05 m
default clearance: 0.03 m
```

The wrist centerlines and radii are editable in
`config/collision_regions.yaml`. Their defaults are based on the URDF
collision-mesh extents. The tool centerline is explicitly measured from the
TCP origin toward TCP `+Z`. The obsolete URDF `tool0` links and their 0.21 m
offsets have been removed. In demonstration mode, `L_wrist_3_link` and
`R_wrist_3_link` are used as the untooled TCP frames, avoiding the flange
frame's fixed axis rotation. Each tool cylinder therefore starts exactly where
the corresponding `wrist_3` cylinder ends and continues along the same Z axis.

Every left cylinder is checked against every right cylinder. A pose or sampled
`moveL` path is safe only when all nine pairs satisfy:

```text
d_pair >= left_pair_radius + right_pair_radius + clearance
```

An unsafe command is replaced by the arm's last safe target.

## Build

From the ROS 2 workspace:

```bash
cd /home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
source /opt/ros/jazzy/setup.bash
colcon build --packages-select popeye_description collision_ur --symlink-install
source install/setup.bash
```

Validate the URDF separately:

```bash
check_urdf src/popeye_description/urdf/popeye.urdf
```

## 1. Robot Model Only

Use this mode to inspect the migrated ROS 2 model without collision nodes or
robot hardware:

```bash
ros2 launch popeye_description display.launch.py
```

This starts:

- `robot_state_publisher`
- A demonstration `/joint_states` publisher
- RViz with the Popeye RobotModel

The default demonstration pose is:

```text
[base, shoulder, elbow, wrist1, wrist2, wrist3]
[0, -1.5708, 1.5708, -1.5708, -1.5708, 0]
```

The RViz fixed frame is `torso_link`, which is the fixed root of the physical
dual-arm system.

## 2. Robot Model And Collision Volumes

This is the recommended visualization command:

```bash
ros2 launch collision_ur collision_ur.launch.py
```

The default is a hardware-safe demonstration mode:

- It does not connect to either UR5e.
- It publishes a demonstration 12-joint pose.
- The URDF TF tree generates both tool poses.
- Collision cylinders and workspace boxes follow the model.

RViz should display:

1. The torso and both UR5e arms.
2. Three green cylinders covering the left final two links and tool.
3. Three yellow cylinders covering the right final two links and tool.
4. Two transparent Cartesian workspace boxes.
5. A nearest-distance line and safety text.

Only the most critical pair turns red when the required separation is
violated. The status text names that pair, for example
`wrist_3 <-> tool`.

## 3. Editing In RViz

Select the **Interact** tool in the RViz toolbar before manipulating regions.

### Tool cylinders

- Drag an arrow to translate a cylinder relative to its TCP.
- Drag a rotation ring to correct the cylinder direction.
- Right-click the cylinder and choose `Reset to TCP +Z, 20 cm` to restore
  the configured 5 cm radius and 20 cm length.

The interactive markers modify only the two tool cylinders. The two wrist
cylinders on each arm continuously follow `wrist_2_link` and `wrist_3_link`
TF. Their `segment_start`, `segment_end`, and `radius` values can be corrected
directly in the YAML while viewing the result in RViz.

### Cartesian workspaces

- Drag a workspace box to change its center.
- Right-click a box to expand or shrink X, Y, or Z by 2 cm.
- Commands outside the corresponding box are clamped to its boundary.

Workspace limits are expressed in that arm's controller base frame.

### Safety clearance

Right-click the `clearance = ... m` label and choose:

- Increase/decrease by 1 cm.
- Increase/decrease by 5 mm.

Each pair's collision threshold is its two cylinder radii plus this clearance.

### Save edits

RViz edits are applied immediately but are persisted only after:

```bash
ros2 service call /collision_ur/save_config std_srvs/srv/Trigger {}
```

The default file is inside this ROS 2 workspace:

```text
/home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros/
src/collision_ur/config/collision_regions.yaml
```

The service writes RViz changes directly back to that source YAML. Use another
file with:

```bash
ros2 launch collision_ur collision_ur.launch.py \
  config_file:=/absolute/path/to/regions.yaml
```

## 4. Real Dual-UR5e Mode

Before enabling hardware:

1. Verify both robots are reachable.
2. Verify the dual-base calibration file.
3. Set low speed and acceleration.
4. Keep an operator at both emergency stops.
5. Test the same targets in demonstration mode first.

Start RTDE mode:

```bash
ros2 launch collision_ur collision_ur.launch.py \
  use_rtde_executor:=true \
  left_robot_ip:=192.168.5.202 \
  right_robot_ip:=192.168.5.101
```

In this mode `guarded_rtde_executor` publishes:

- `/joint_states` from both robots' `getActualQ()`.
- `/left_arm/eef_pose` and `/right_arm/eef_pose`.

It accepts only supervisor-approved targets:

```text
/left_arm/safe_target_pose
/right_arm/safe_target_pose
```

Candidate commands must be sent to:

```text
/left_arm/target_pose
/right_arm/target_pose
```

The left message must use `header.frame_id: left_base`; the right message must
use `header.frame_id: right_base`.

Example left target:

```bash
ros2 topic pub --once /left_arm/target_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: left_base},
    pose: {position: {x: -0.45, y: 0.0, z: 0.40},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 5. Useful RViz Displays

The supplied RViz configuration already contains:

- `RobotModel`: `/robot_description`
- `MarkerArray`: `/collision_ur/markers`
- `InteractiveMarkers`: `/collision_region_editor/update`
- `TF`: disabled by default, useful for debugging

If a display is missing, add it manually:

1. Click **Add** in the Displays panel.
2. Select **RobotModel**, **MarkerArray**, **InteractiveMarkers**, or **TF**.
3. Set the topics listed above.
4. Set **Global Options -> Fixed Frame** to `torso_link`.

To inspect URDF collision meshes rather than visual meshes, enable
`Collision Enabled` and disable `Visual Enabled` in the RobotModel display.
These URDF collision meshes are separate from the paper-inspired distal
cylinders, although the default wrist cylinder dimensions were chosen from
their mesh extents.

## Topics

| Topic | Type | Purpose |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | Twelve UR5e joint positions |
| `/robot_description` | `std_msgs/String` | Popeye URDF for RViz |
| `/left_arm/eef_pose` | `geometry_msgs/PoseStamped` | Left TCP in `left_base` |
| `/right_arm/eef_pose` | `geometry_msgs/PoseStamped` | Right TCP in `right_base` |
| `/left_arm/target_pose` | `geometry_msgs/PoseStamped` | Candidate left target |
| `/right_arm/target_pose` | `geometry_msgs/PoseStamped` | Candidate right target |
| `/left_arm/safe_target_pose` | `geometry_msgs/PoseStamped` | Filtered left target |
| `/right_arm/safe_target_pose` | `geometry_msgs/PoseStamped` | Filtered right target |
| `/collision_ur/markers` | `visualization_msgs/MarkerArray` | Cylinders, boxes and distance |
| `/collision_ur/is_safe` | `std_msgs/Bool` | Latest target safety result |
| `/collision_ur/centerline_distance` | `std_msgs/Float64` | Current minimum distance |
| `/collision_ur/required_distance` | `std_msgs/Float64` | Current safety threshold |

## TF And Joint Checks

Confirm all 12 joints:

```bash
ros2 topic echo --once /joint_states
```

Confirm the measured dual-base transform:

```bash
ros2 run tf2_ros tf2_echo left_base right_base
```

Confirm the wrist-3-centered model TCP frames:

```bash
ros2 run tf2_ros tf2_echo left_base L_wrist_3_link
ros2 run tf2_ros tf2_echo right_base R_wrist_3_link
```

Confirm collision markers:

```bash
ros2 topic echo --once /collision_ur/markers
```

## Launch Options

```text
start_rviz:=true|false
use_rtde_executor:=true|false
left_robot_ip:=192.168.5.202
right_robot_ip:=192.168.5.101
config_file:=/home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros/src/collision_ur/config/collision_regions.yaml
base_transform_file:=.../dual_base_left_to_right_v2.json
rviz_use_mesa:=true|false
```

NVIDIA is the default RViz rendering path. Use the AMD/Mesa fallback only if
NVIDIA GLX is temporarily unavailable:

```bash
ros2 launch collision_ur collision_ur.launch.py rviz_use_mesa:=true
```

## Troubleshooting

### RobotModel is absent

Check:

```bash
ros2 topic info /robot_description
ros2 topic echo --once /joint_states
```

Then verify the RobotModel description topic is `/robot_description`.

### Cylinders are absent

Both EEF topics and the final-link TF frames must be active:

```bash
ros2 topic info /left_arm/eef_pose
ros2 topic info /right_arm/eef_pose
ros2 run tf2_ros tf2_echo left_base L_wrist_2_link
ros2 run tf2_ros tf2_echo left_base L_wrist_3_link
```

Also verify the MarkerArray topic is `/collision_ur/markers`.

### Model and right cylinder do not align

Verify that the launch and model use the same calibration file:

```bash
ros2 run tf2_ros tf2_echo left_base right_base
```

Its matrix should match `left_base_T_right_base` in the selected JSON.

### Interactive handles are not visible

- Select the RViz **Interact** tool.
- Confirm the InteractiveMarkers update topic is
  `/collision_region_editor/update`.
- Wait until both EEF poses have been received.

### RViz OpenGL failure

First try the normal NVIDIA path. If it fails:

```bash
ros2 launch collision_ur collision_ur.launch.py rviz_use_mesa:=true
```

Do not enable the Mesa fallback when the AMD context repeatedly reports
`context is lost`; use the repaired NVIDIA path instead.
