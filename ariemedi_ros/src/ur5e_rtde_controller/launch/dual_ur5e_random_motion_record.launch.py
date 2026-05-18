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
        description="Record duration for random motion",
    )
    output_data_arg = DeclareLaunchArgument(
        "output_data",
        default_value="/home/us-mrc/Documents/dual_arm_eef_marker_raw.npy",
        description="Output numpy data path (.npy or .npz)",
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
    sphere_radius_arg = DeclareLaunchArgument(
        "sphere_radius_m",
        default_value="0.10",
        description="Random position radius around home in meters",
    )
    speed_arg = DeclareLaunchArgument(
        "linear_speed_mps",
        default_value="0.01",
        description="Linear speed in m/s",
    )
    acc_arg = DeclareLaunchArgument(
        "linear_acc_mps2",
        default_value="0.05",
        description="Linear acceleration in m/s^2",
    )
    min_dist_arg = DeclareLaunchArgument(
        "min_target_distance_m",
        default_value="0.02",
        description="Minimum distance from home in meters",
    )
    ang_range_arg = DeclareLaunchArgument(
        "angular_range_deg",
        default_value="20.0",
        description="Random orientation range around home in degrees",
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
                "sphere_radius_m": LaunchConfiguration("sphere_radius_m"),
                "linear_speed_mps": LaunchConfiguration("linear_speed_mps"),
                "linear_acc_mps2": LaunchConfiguration("linear_acc_mps2"),
                "min_target_distance_m": LaunchConfiguration("min_target_distance_m"),
                "angular_range_deg": LaunchConfiguration("angular_range_deg"),
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
            sphere_radius_arg,
            speed_arg,
            acc_arg,
            min_dist_arg,
            ang_range_arg,
            dual_rtde_node,
            tracker_node,
            recorder_node,
            shutdown_on_rtde_exit,
        ]
    )
