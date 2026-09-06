from pathlib import Path
import xml.etree.ElementTree as ET


URDF_PATH = Path(__file__).parents[1] / "urdf" / "popeye.urdf"


def test_popeye_urdf_has_expected_dual_arm_structure():
    root = ET.parse(URDF_PATH).getroot()
    links = {element.attrib["name"] for element in root.findall("link")}
    joints = {element.attrib["name"] for element in root.findall("joint")}

    assert root.attrib["name"] == "popeye"
    assert {"world_link", "torso_link", "left_base", "right_base"} <= links
    assert "world_to_torso" in joints
    assert {"L_flange", "R_flange"} <= links
    assert "L_tool0" not in links
    assert "R_tool0" not in links
    assert {
        "L_Base",
        "L_Shoulder",
        "L_Elbow",
        "L_Wrist1",
        "L_Wrist2",
        "L_Wrist3",
        "R_Base",
        "R_Shoulder",
        "R_Elbow",
        "R_Wrist1",
        "R_Wrist2",
        "R_Wrist3",
    } <= joints


def test_mesh_resources_use_ros2_package_name():
    urdf_text = URDF_PATH.read_text()
    assert "package://popeye_description/" in urdf_text
    assert "package://ur5e_robot/" not in urdf_text
