from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.5",
        description="UR5e robot IP address",
    )

    ur5e_node = Node(
        package="ur5e_rtde_controller",
        executable="ur5e_rtde_node",
        name="ur5e_rtde_node",
        output="screen",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
            }
        ],
    )

    armd_node = Node(
        package="ariemedi_tracker",
        executable="tracker_node",
        name="tracker_node",
        output="screen",
    )

    return LaunchDescription([
        robot_ip_arg,
        ur5e_node,
        armd_node,
    ])
