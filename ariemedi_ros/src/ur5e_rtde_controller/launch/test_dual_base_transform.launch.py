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
    marker_topic_arg = DeclareLaunchArgument(
        "marker_topic",
        default_value="/ARMDpos",
        description="Tracker output topic",
    )
    tracker_hostname_arg = DeclareLaunchArgument(
        "tracker_hostname",
        default_value="",
        description="Tracker hostname/IP (empty means auto scan)",
    )
    pause_sec_arg = DeclareLaunchArgument(
        "pause_sec",
        default_value="2.0",
        description="Pause seconds between steps",
    )
    json_path_arg = DeclareLaunchArgument(
        "json_path",
        default_value="/home/us-mrc/Documents/BMT_Group_ARIEMEDI/dual_arm_cali/dual_base_left_to_right.json",
        description="Path to dual base transform json",
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

    run_test = Node(
        package="ur5e_rtde_controller",
        executable="test_dual_base_transform",
        name="test_dual_base_transform",
        output="screen",
        arguments=[
            "--left_robot_ip",
            LaunchConfiguration("left_robot_ip"),
            "--right_robot_ip",
            LaunchConfiguration("right_robot_ip"),
            "--marker_topic",
            LaunchConfiguration("marker_topic"),
            "--pause_sec",
            LaunchConfiguration("pause_sec"),
            "--json",
            LaunchConfiguration("json_path"),
        ],
    )

    shutdown_on_test_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=run_test,
            on_exit=[EmitEvent(event=Shutdown(reason="test_dual_base_transform finished"))],
        )
    )

    return LaunchDescription(
        [
            left_robot_ip_arg,
            right_robot_ip_arg,
            marker_topic_arg,
            tracker_hostname_arg,
            pause_sec_arg,
            json_path_arg,
            tracker_node,
            run_test,
            shutdown_on_test_exit,
        ]
    )
