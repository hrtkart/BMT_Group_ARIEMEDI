from collision_ur.configuration import load_config, save_config


def test_default_tool_geometry(tmp_path):
    config = load_config(str(tmp_path / "missing.yaml"))
    assert set(config["left"]["bodies"]) == {"wrist_2", "wrist_3", "tool"}
    assert config["left"]["bodies"]["tool"]["radius"] == 0.05
    assert config["left"]["bodies"]["tool"]["segment_start"] == [0.0, 0.0, 0.0]
    assert config["left"]["bodies"]["tool"]["segment_end"] == [0.0, 0.0, 0.20]


def test_configuration_round_trip(tmp_path):
    path = tmp_path / "regions.yaml"
    config = load_config(str(path))
    config["clearance"] = 0.047
    save_config(str(path), config)
    assert load_config(str(path))["clearance"] == 0.047
