from __future__ import annotations

import json
import copy
from pathlib import Path
from typing import Any, Dict

import numpy as np
import yaml


DEFAULT_CONFIG: Dict[str, Any] = {
    "global_frame": "left_base",
    "clearance": 0.03,
    "left": {
        "bodies": {
            "wrist_2": {
                "frame": "L_wrist_2_link",
                "radius": 0.06,
                "segment_start": [0.0, -0.055, 0.0],
                "segment_end": [0.0, 0.100, 0.0],
            },
            "wrist_3": {
                "frame": "L_wrist_3_link",
                "radius": 0.06,
                "segment_start": [0.0, 0.0, -0.050],
                "segment_end": [0.0, 0.0, 0.0],
            },
            "tool": {
                "frame": "tcp",
                "radius": 0.05,
                "segment_start": [0.0, 0.0, 0.0],
                "segment_end": [0.0, 0.0, 0.20],
            },
        },
        "workspace_min": [-0.85, -0.85, -0.20],
        "workspace_max": [0.85, 0.85, 1.10],
    },
    "right": {
        "bodies": {
            "wrist_2": {
                "frame": "R_wrist_2_link",
                "radius": 0.06,
                "segment_start": [0.0, -0.055, 0.0],
                "segment_end": [0.0, 0.100, 0.0],
            },
            "wrist_3": {
                "frame": "R_wrist_3_link",
                "radius": 0.06,
                "segment_start": [0.0, 0.0, -0.050],
                "segment_end": [0.0, 0.0, 0.0],
            },
            "tool": {
                "frame": "tcp",
                "radius": 0.05,
                "segment_start": [0.0, 0.0, 0.0],
                "segment_end": [0.0, 0.0, 0.20],
            },
        },
        "workspace_min": [-0.85, -0.85, -0.20],
        "workspace_max": [0.85, 0.85, 1.10],
    },
}


def _deep_merge(base: Dict[str, Any], update: Dict[str, Any]) -> Dict[str, Any]:
    result = {}
    for key, value in base.items():
        if isinstance(value, dict):
            result[key] = _deep_merge(value, update.get(key, {}))
        else:
            result[key] = update.get(key, value)
    for key, value in update.items():
        if key not in result:
            result[key] = value
    return result


def load_config(path: str) -> Dict[str, Any]:
    config = copy.deepcopy(DEFAULT_CONFIG)
    config_path = Path(path).expanduser()
    if config_path.is_file():
        with config_path.open("r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream) or {}
        config = _deep_merge(DEFAULT_CONFIG, loaded)
    return config


def save_config(path: str, config: Dict[str, Any]) -> None:
    config_path = Path(path).expanduser()
    config_path.parent.mkdir(parents=True, exist_ok=True)
    with config_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)


def load_right_to_left_transform(path: str) -> np.ndarray:
    """Load left_base_T_right_base from the existing calibration JSON."""
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        payload = json.load(stream)
    transform = np.asarray(payload["left_base_T_right_base"], dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("left_base_T_right_base must be a 4x4 matrix")
    return transform
