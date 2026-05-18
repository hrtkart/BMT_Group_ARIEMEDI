from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    left_robot_ip_arg = DeclareLaunchArgument(
        "left_robot_ip",
        default_value="192.168.5.202",
        description="Left UR5e IP (caliorange arm)",
    )
    right_robot_ip_arg = DeclareLaunchArgument(
        "right_robot_ip",
        default_value="192.168.5.101",
        description="Right UR5e IP (caliblack arm)",
    )
    duration_sec_arg = DeclareLaunchArgument(
        "duration_sec",
        default_value="300.0",
        description="Record duration for trajectory comparison",
    )
    output_data_arg = DeclareLaunchArgument(
        "output_data",
        default_value="/home/us-mrc/Documents/dual_arm_eef_marker_raw.npz",
        description="Output numpy data path (.npz)",
    )
    marker_topic_arg = DeclareLaunchArgument(
        "marker_topic",
        default_value="/ARMDpos",
        description="Tracker output topic from multi_tracker_node",
    )
    tracker_hostname_arg = DeclareLaunchArgument(
        "tracker_hostname",
        default_value="",
        description="Tracker hostname/IP (empty means auto scan)",
    )

    dual_rtde_node = Node(
        package="ur5e_rtde_controller",
        executable="dual_ur5e_rtde_node",
        name="dual_ur5e_rtde_node",
        output="screen",
        parameters=[
            {
                "left_robot_ip": LaunchConfiguration("left_robot_ip"),
                "right_robot_ip": LaunchConfiguration("right_robot_ip"),
                "travel_distance_m": 0.10,
                "linear_speed_mps": 0.02,
                "pause_at_switch_sec": 2.0,
                "stop_delay_sec": 30.0,
            }
        ],
    )

    tracker_node = Node(
        package="ariemedi_tracker",
        executable="multi_tracker_node",
        name="multi_tracker_node",
        output="screen",
        parameters=[
            {
                "output_topic": LaunchConfiguration("marker_topic"),
                "hostname": LaunchConfiguration("tracker_hostname"),
                "tool_paths": [
                    "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/tools_marker/caliblack2.arom",
                    "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/tools_marker/caliorange.arom",
                ],
            }
        ],
    )

    recorder_node = Node(
        package="ur5e_rtde_controller",
        executable="dual_eef_marker_data_recorder",
        name="dual_eef_marker_data_recorder",
        output="screen",
        parameters=[
            {
                "duration_sec": LaunchConfiguration("duration_sec"),
                "output_data": LaunchConfiguration("output_data"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "left_eef_topic": "/left_arm/eef_pose",
                "right_eef_topic": "/right_arm/eef_pose",
                "left_marker_name": "caliorange",
                "right_marker_name": "caliblack2",
            }
        ],
    )
    shutdown_on_rtde_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=dual_rtde_node,
            on_exit=[EmitEvent(event=Shutdown(reason="dual_ur5e_rtde_node exited"))],
        )
    )

    return LaunchDescription(
        [
            left_robot_ip_arg,
            right_robot_ip_arg,
            duration_sec_arg,
            output_data_arg,
            marker_topic_arg,
            tracker_hostname_arg,
            dual_rtde_node,
            tracker_node,
            recorder_node,
            shutdown_on_rtde_exit,
        ]
    )
