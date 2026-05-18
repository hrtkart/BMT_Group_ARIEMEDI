#!/usr/bin/env python3

import argparse
import json
import math
import sys
from typing import Dict, List, Tuple

import numpy as np


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def mat_to_quat(r: np.ndarray) -> np.ndarray:
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    else:
        if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm > 0.0:
        q /= norm
    return q


def make_transform(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[:3, :3] = quat_to_mat(quat)
    t[:3, 3] = pos
    return t


def inv_transform(t: np.ndarray) -> np.ndarray:
    r = t[:3, :3]
    p = t[:3, 3]
    t_inv = np.eye(4, dtype=np.float64)
    t_inv[:3, :3] = r.T
    t_inv[:3, 3] = -r.T @ p
    return t_inv


def check_series(name: str, stamps: np.ndarray) -> Tuple[bool, str, float, float]:
    if stamps.size == 0:
        return False, f"{name} has no samples", 0.0, 0.0
    if not np.all(np.isfinite(stamps)):
        return False, f"{name} has non-finite timestamps", 0.0, 0.0
    if np.any(np.diff(stamps) <= 0.0):
        return False, f"{name} timestamps are not strictly increasing", 0.0, 0.0
    dt = np.diff(stamps)
    if dt.size == 0:
        return False, f"{name} has only one timestamp", 0.0, 0.0
    median_dt = float(np.median(dt))
    if median_dt <= 0.0:
        return False, f"{name} has non-positive median dt", 0.0, 0.0
    max_dt = float(np.max(dt))
    if max_dt > 3.0 * median_dt:
        return False, f"{name} has large gaps: max_dt={max_dt:.4f}s", median_dt, max_dt
    return True, "", median_dt, max_dt


def nearest_indices(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    idx = np.searchsorted(b, a)
    idx = np.clip(idx, 1, len(b) - 1)
    left = idx - 1
    right = idx
    pick_right = (a - b[left]) > (b[right] - a)
    return np.where(pick_right, right, left)


def collect_base_tracker(
    arm: str,
    eef_stamps: np.ndarray,
    eef_pos: np.ndarray,
    eef_quat: np.ndarray,
    marker_stamps: np.ndarray,
    marker_pos: np.ndarray,
    marker_quat: np.ndarray,
    max_sync_dt: float,
    t_eef_marker: np.ndarray,
) -> Tuple[np.ndarray, List[np.ndarray]]:
    idx = nearest_indices(marker_stamps, eef_stamps)
    dt = np.abs(marker_stamps - eef_stamps[idx])
    if np.any(dt > max_sync_dt):
        bad = int(np.argmax(dt))
        raise RuntimeError(
            f"{arm} marker->eef sync gap too large: {dt[bad]:.4f}s at sample {bad}"
        )

    base_tracker = []
    for mi, ei in enumerate(idx):
        t_base_eef = make_transform(eef_pos[ei], eef_quat[ei])
        t_base_marker = t_base_eef @ t_eef_marker
        t_tracker_marker = make_transform(marker_pos[mi], marker_quat[mi])
        t_base_tracker = t_base_marker @ inv_transform(t_tracker_marker)
        base_tracker.append(t_base_tracker)
    return marker_stamps, base_tracker


def average_quaternion(quats: List[np.ndarray]) -> np.ndarray:
    if not quats:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    ref = quats[0]
    acc = np.zeros(4, dtype=np.float64)
    for q in quats:
        if np.dot(ref, q) < 0.0:
            acc -= q
        else:
            acc += q
    norm = np.linalg.norm(acc)
    if norm > 0.0:
        acc /= norm
    return acc


def main() -> int:
    parser = argparse.ArgumentParser(description="Compute left base relative to right base")
    parser.add_argument(
        "--input",
        default="/home/us-mrc/Documents/dual_arm_eef_marker_raw.npy",
        help="Input .npy file",
    )
    parser.add_argument(
        "--output",
        default="/home/us-mrc/Documents/dual_base_left_to_right.json",
        help="Output json path",
    )
    parser.add_argument(
        "--max_sync_dt",
        type=float,
        default=0.02,
        help="Max time delta between marker and eef timestamps (sec)",
    )
    args = parser.parse_args()

    try:
        data = np.load(args.input, allow_pickle=True).item()
    except Exception as exc:
        print(f"Failed to load {args.input}: {exc}")
        return 1
    if not isinstance(data, dict):
        print("Loaded data is not a dict payload")
        return 1

    keys = [
        "left_eef_stamp_sec",
        "left_eef_pos",
        "left_eef_quat_xyzw",
        "left_marker_stamp_sec",
        "left_marker_pos",
        "left_marker_quat_xyzw",
        "right_eef_stamp_sec",
        "right_eef_pos",
        "right_eef_quat_xyzw",
        "right_marker_stamp_sec",
        "right_marker_pos",
        "right_marker_quat_xyzw",
    ]
    for key in keys:
        if key not in data:
            print(f"Missing key: {key}")
            return 1

    left_eef_stamps = np.asarray(data["left_eef_stamp_sec"], dtype=np.float64)
    left_eef_pos = np.asarray(data["left_eef_pos"], dtype=np.float64)
    left_eef_quat = np.asarray(data["left_eef_quat_xyzw"], dtype=np.float64)
    left_marker_stamps = np.asarray(data["left_marker_stamp_sec"], dtype=np.float64)
    left_marker_pos = np.asarray(data["left_marker_pos"], dtype=np.float64)
    left_marker_quat = np.asarray(data["left_marker_quat_xyzw"], dtype=np.float64)

    right_eef_stamps = np.asarray(data["right_eef_stamp_sec"], dtype=np.float64)
    right_eef_pos = np.asarray(data["right_eef_pos"], dtype=np.float64)
    right_eef_quat = np.asarray(data["right_eef_quat_xyzw"], dtype=np.float64)
    right_marker_stamps = np.asarray(data["right_marker_stamp_sec"], dtype=np.float64)
    right_marker_pos = np.asarray(data["right_marker_pos"], dtype=np.float64)
    right_marker_quat = np.asarray(data["right_marker_quat_xyzw"], dtype=np.float64)

    ok, msg, med, _ = check_series("left_eef", left_eef_stamps)
    if not ok:
        print(msg)
        return 1
    ok, msg, _, _ = check_series("right_eef", right_eef_stamps)
    if not ok:
        print(msg)
        return 1
    ok, msg, _, _ = check_series("left_marker", left_marker_stamps)
    if not ok:
        print(msg)
        return 1
    ok, msg, _, _ = check_series("right_marker", right_marker_stamps)
    if not ok:
        print(msg)
        return 1

    if left_marker_stamps[0] > left_eef_stamps[0] + 0.05 or left_marker_stamps[-1] < left_eef_stamps[-1] - 0.05:
        print("left_marker does not cover full left_eef time span")
        return 1
    if right_marker_stamps[0] > right_eef_stamps[0] + 0.05 or right_marker_stamps[-1] < right_eef_stamps[-1] - 0.05:
        print("right_marker does not cover full right_eef time span")
        return 1

    if left_eef_pos.shape[1] != 3 or right_eef_pos.shape[1] != 3:
        print("EEF position array shape invalid")
        return 1
    if left_marker_pos.shape[1] != 3 or right_marker_pos.shape[1] != 3:
        print("Marker position array shape invalid")
        return 1
    if left_eef_quat.shape[1] != 4 or right_eef_quat.shape[1] != 4:
        print("EEF quaternion array shape invalid")
        return 1
    if left_marker_quat.shape[1] != 4 or right_marker_quat.shape[1] != 4:
        print("Marker quaternion array shape invalid")
        return 1

    t_eef_marker = np.eye(4, dtype=np.float64)
    t_eef_marker[:3, 3] = np.array([0.019, 0.0, 0.055], dtype=np.float64)
    t_eef_marker[:3, :3] = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )

    try:
        left_ts, left_base_tracker = collect_base_tracker(
            "left",
            left_eef_stamps,
            left_eef_pos,
            left_eef_quat,
            left_marker_stamps,
            left_marker_pos,
            left_marker_quat,
            args.max_sync_dt,
            t_eef_marker,
        )
        right_ts, right_base_tracker = collect_base_tracker(
            "right",
            right_eef_stamps,
            right_eef_pos,
            right_eef_quat,
            right_marker_stamps,
            right_marker_pos,
            right_marker_quat,
            args.max_sync_dt,
            t_eef_marker,
        )
    except RuntimeError as exc:
        print(str(exc))
        return 1

    idx = nearest_indices(left_ts, right_ts)
    dt = np.abs(left_ts - right_ts[idx])
    if np.any(dt > args.max_sync_dt):
        bad = int(np.argmax(dt))
        print(f"left/right time alignment too large: {dt[bad]:.4f}s at sample {bad}")
        return 1

    if len(left_base_tracker) < 10 or len(right_base_tracker) < 10:
        print("Not enough samples for stable base transform")
        return 1

    transforms = []
    quats = []
    poss = []
    for i, j in enumerate(idx):
        t_left = left_base_tracker[i]
        t_right = right_base_tracker[j]
        t_left_right = t_left @ inv_transform(t_right)
        transforms.append(t_left_right)
        poss.append(t_left_right[:3, 3])
        quats.append(mat_to_quat(t_left_right[:3, :3]))

    mean_pos = np.mean(np.stack(poss, axis=0), axis=0)
    mean_quat = average_quaternion(quats)
    t_mean = make_transform(mean_pos, mean_quat)

    out = {
        "left_base_T_right_base": t_mean.tolist(),
        "num_samples": len(transforms),
        "median_eef_dt_left": float(med),
    }
    try:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)
    except Exception as exc:
        print(f"Failed to write {args.output}: {exc}")
        return 1

    print(f"Saved transform to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
