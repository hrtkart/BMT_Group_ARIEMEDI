from pathlib import Path

from setuptools import find_packages, setup


package_name = "popeye_description"
root = Path(__file__).parent


def package_files(directory):
    return [
        str(path.relative_to(root))
        for path in (root / directory).rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and path.suffix != ".pyc"
    ]


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", package_files("urdf")),
        ("share/" + package_name + "/launch", package_files("launch")),
        ("share/" + package_name + "/rviz", package_files("rviz")),
        (
            "share/" + package_name + "/meshes/visual",
            package_files("meshes/visual"),
        ),
        (
            "share/" + package_name + "/meshes/collision",
            package_files("meshes/collision"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ariemedi",
    maintainer_email="ariemedi@todo.todo",
    description="ROS 2 URDF and meshes for the Popeye dual-UR5e robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_joint_state_publisher = "
            "popeye_description.demo_joint_state_publisher:main",
        ],
    },
)
