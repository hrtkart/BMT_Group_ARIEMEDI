from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("popeye_description"))
    robot_description = (share / "urdf" / "popeye.urdf").read_text()

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="popeye_description",
                executable="demo_joint_state_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "rviz" / "popeye.rviz")],
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                output="screen",
            ),
        ]
    )
