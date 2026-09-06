from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    collision_share = Path(get_package_share_directory("collision_ur"))
    description_share = Path(get_package_share_directory("popeye_description"))
    robot_description = (
        description_share / "urdf" / "popeye.urdf"
    ).read_text()
    config_file = LaunchConfiguration("config_file")
    transform_file = LaunchConfiguration("base_transform_file")
    left_ip = LaunchConfiguration("left_robot_ip")
    right_ip = LaunchConfiguration("right_robot_ip")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=(
                    "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/ariemedi_ros/"
                    "src/collision_ur/config/collision_regions.yaml"
                ),
                description="Editable collision-region YAML in the ROS workspace",
            ),
            DeclareLaunchArgument(
                "base_transform_file",
                default_value=(
                    "/home/us-mrc/Documents/BMT_Group_ARIEMEDI/dual_arm_cali/"
                    "dual_base_left_to_right_v2.json"
                ),
            ),
            DeclareLaunchArgument("left_robot_ip", default_value="192.168.5.202"),
            DeclareLaunchArgument("right_robot_ip", default_value="192.168.5.101"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(collision_share / "rviz" / "collision_ur.rviz"),
            ),
            DeclareLaunchArgument(
                "use_rtde_executor",
                default_value="false",
                description="Connect to and command the real UR5e arms",
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_use_mesa",
                default_value="false",
                description=(
                    "Run RViz through the AMD/Mesa GLX vendor as a fallback "
                    "instead of the default NVIDIA display path."
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="popeye_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="popeye_description",
                executable="demo_joint_state_publisher",
                name="popeye_demo_joint_states",
                condition=UnlessCondition(
                    LaunchConfiguration("use_rtde_executor")
                ),
                output="screen",
            ),
            Node(
                package="collision_ur",
                executable="model_eef_pose_publisher",
                condition=UnlessCondition(
                    LaunchConfiguration("use_rtde_executor")
                ),
                output="screen",
            ),
            Node(
                package="collision_ur",
                executable="collision_supervisor",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "base_transform_file": transform_file,
                    }
                ],
            ),
            Node(
                package="collision_ur",
                executable="collision_region_editor",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "base_transform_file": transform_file,
                    }
                ],
            ),
            Node(
                package="collision_ur",
                executable="guarded_rtde_executor",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rtde_executor")),
                parameters=[
                    {
                        "left_robot_ip": left_ip,
                        "right_robot_ip": right_ip,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="collision_ur_rviz_mesa",
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config"),
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("start_rviz"),
                            "' == 'true' and '",
                            LaunchConfiguration("rviz_use_mesa"),
                            "' == 'true'",
                        ]
                    )
                ),
                additional_env={
                    "__GLX_VENDOR_LIBRARY_NAME": "mesa",
                    "DRI_PRIME": "1",
                    "QT_X11_NO_MITSHM": "1",
                },
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="collision_ur_rviz",
                arguments=[
                    "-d",
                    LaunchConfiguration("rviz_config"),
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("start_rviz"),
                            "' == 'true' and '",
                            LaunchConfiguration("rviz_use_mesa"),
                            "' != 'true'",
                        ]
                    )
                ),
                output="screen",
            ),
        ]
    )
