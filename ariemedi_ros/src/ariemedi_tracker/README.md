# ariemedi_tracker

ROS 2 package for Ariemedi/ARMD optical tracking, tool pose publishing, and TF broadcasting.

## Nodes

- `tracker_node`
  - Connects to one ARMD tracker, loads a single tool, and publishes `/ARMDpos`.
  - Broadcasts TF from `tracker_base` to the tool frame.

- `multi_tracker_node`
  - Loads multiple tool `.arom` files and publishes each tool on the same output topic.

- `marker_pair_broadcaster_node`
  - Subscribes to `/ARMDpos` and publishes TF frames for selected marker names.

## Topics

- `/ARMDpos` (`ariemedi_tracker/msg/ToolTrackingData`)
- TF: `tracker_base -> <tool_name>` (default tool frame `toolcali0` if name is empty)

## Parameters

`tracker_node`:
- `hostname` (string, default: empty, auto-scan device)
- `enable_imaging` (bool, default: false)
- `base_frame_id` (string, default: `tracker_base`)
- `default_tool_frame_id` (string, default: `toolcali0`)

`multi_tracker_node`:
- `hostname` (string, default: empty, auto-scan device)
- `enable_imaging` (bool, default: false)
- `output_topic` (string, default: `/ARMDpos`)
- `tool_paths` (string array, default: example tool files in `tools_marker/`)

`marker_pair_broadcaster_node`:
- `input_topic` (string, default: `/ARMDpos`)
- `tracker_frame` (string, default: `tracker_base`)
- `use_msg_frame_id` (bool, default: true)
- `tracked_marker_names` (string array, default: `["caliorange", "caliblack2"]`)
- `child_frame_prefix` (string, default: `marker_`)

## Build

```bash
cd /home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros
# Source your ROS2 underlay first (example: /opt/ros/<distro>/setup.bash)
source /path/to/ros2/setup.bash
colcon build --packages-select ariemedi_tracker
source install/setup.bash
```

## Run

Single-tool tracking:

```bash
ros2 run ariemedi_tracker tracker_node
```

Single-tool with device hostname and imaging enabled:

```bash
ros2 run ariemedi_tracker tracker_node --ros-args \
  -p hostname:=192.168.5.10 \
  -p enable_imaging:=true
```

Multi-tool tracking:

```bash
ros2 run ariemedi_tracker multi_tracker_node --ros-args \
  -p tool_paths:="['/home/us-mrc/Documents/BMT_Group_ARIEMEDI/tools_marker/caliblack2.arom', '/home/us-mrc/Documents/BMT_Group_ARIEMEDI/tools_marker/caliorange.arom']" \
  -p output_topic:=/ARMDpos
```

Marker TF broadcaster:

```bash
ros2 run ariemedi_tracker marker_pair_broadcaster_node --ros-args \
  -p tracked_marker_names:="['caliorange', 'caliblack2']" \
  -p child_frame_prefix:=marker_
```

## Analysis Scripts

- Relative rotation log + plot (first frame as reference):

```bash
ros2 run ariemedi_tracker marker_relative_rotation_logger --ros-args \
  -p duration_sec:=60.0 \
  -p output_png:=/tmp/marker_relative_rotation_plot.png
```

- Tool pose + marker relative plot (60s, saves to `/tmp`):

```bash
ros2 run ariemedi_tracker marker_test_plot
```
