from glob import glob
from setuptools import find_packages, setup


package_name = "collision_ur"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ariemedi",
    maintainer_email="ariemedi@todo.todo",
    description="Kinematic collision avoidance for a dual UR5e ROS 2 system",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "collision_supervisor = collision_ur.supervisor_node:main",
            "collision_region_editor = collision_ur.region_editor_node:main",
            "guarded_rtde_executor = collision_ur.guarded_rtde_executor:main",
            "model_eef_pose_publisher = "
            "collision_ur.model_eef_pose_publisher:main",
        ],
    },
)
