#!/usr/bin/python3

import argparse
import math
from typing import Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def quat_xyzw_to_rotmat(quat_xyzw: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = quat_xyzw.tolist()
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def local_delta_series(pos: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    if pos.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    ref_pos = pos[0]
    ref_rot = quat_xyzw_to_rotmat(quat_xyzw[0])
    return (pos - ref_pos) @ ref_rot


def median_step(ts: np.ndarray) -> float:
    if ts.shape[0] < 2:
        return 0.0
    dt = np.diff(ts)
    dt = dt[dt > 0.0]
    if dt.size == 0:
        return 0.0
    return float(np.median(dt))


def build_common_grid(ts_a: np.ndarray, ts_b: np.ndarray, min_step: float = 0.02) -> np.ndarray:
    a = np.sort(ts_a[np.isfinite(ts_a)])
    b = np.sort(ts_b[np.isfinite(ts_b)])
    if a.shape[0] < 2 or b.shape[0] < 2:
        return np.zeros((0,), dtype=np.float64)
    t0 = max(float(a[0]), float(b[0]))
    t1 = min(float(a[-1]), float(b[-1]))
    if t1 <= t0:
        return np.zeros((0,), dtype=np.float64)
    step = max(median_step(a), median_step(b), min_step)
    n = int((t1 - t0) / step) + 1
    if n < 2:
        return np.zeros((0,), dtype=np.float64)
    return np.linspace(t0, t0 + step * (n - 1), n, dtype=np.float64)


def safe_interp(ts_src: np.ndarray, y_src: np.ndarray, ts_query: np.ndarray) -> np.ndarray:
    if ts_src.shape[0] < 2 or y_src.shape[0] != ts_src.shape[0] or ts_query.shape[0] == 0:
        return np.zeros((0,), dtype=np.float64)
    mask = np.isfinite(ts_src) & np.isfinite(y_src)
    ts = ts_src[mask]
    ys = y_src[mask]
    if ts.shape[0] < 2:
        return np.zeros((0,), dtype=np.float64)
    order = np.argsort(ts)
    return np.interp(ts_query, ts[order], ys[order])


def compute_nearest_time_gap_series(eef_ts: np.ndarray, marker_ts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    eef = np.sort(eef_ts[np.isfinite(eef_ts)])
    marker = np.sort(marker_ts[np.isfinite(marker_ts)])
    if eef.shape[0] < 2 or marker.shape[0] < 2:
        return np.zeros((0,), dtype=np.float64), np.zeros((0,), dtype=np.float64)
    idx = np.searchsorted(marker, eef)
    idx = np.clip(idx, 0, marker.shape[0] - 1)
    idx_prev = np.clip(idx - 1, 0, marker.shape[0] - 1)
    use_prev = np.abs(marker[idx_prev] - eef) < np.abs(marker[idx] - eef)
    use_idx = np.where(use_prev, idx_prev, idx)
    gap = marker[use_idx] - eef
    return eef - eef[0], gap * 1000.0


def map_axes(eef_delta: np.ndarray, marker_delta: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # marker X/Y/Z <-> eef Y/Z/X
    eef_mapped = np.column_stack((eef_delta[:, 1], eef_delta[:, 2], eef_delta[:, 0]))
    marker_mapped = np.column_stack((marker_delta[:, 0], marker_delta[:, 1], marker_delta[:, 2]))
    return eef_mapped, marker_mapped


def get_arm_arrays(data: np.lib.npyio.NpzFile, arm_key: str):
    eef_ts = np.array(data[f"{arm_key}_eef_stamp_sec"], dtype=np.float64)
    eef_pos = np.array(data[f"{arm_key}_eef_pos"], dtype=np.float64)
    eef_q = np.array(data[f"{arm_key}_eef_quat_xyzw"], dtype=np.float64)
    marker_ts = np.array(data[f"{arm_key}_marker_stamp_sec"], dtype=np.float64)
    marker_pos = np.array(data[f"{arm_key}_marker_pos"], dtype=np.float64)
    marker_q = np.array(data[f"{arm_key}_marker_quat_xyzw"], dtype=np.float64)
    return eef_ts, eef_pos, eef_q, marker_ts, marker_pos, marker_q


def plot_axis(ax, t_rel: np.ndarray, marker_v: np.ndarray, eef_v: np.ndarray, arm_name: str, marker_name: str, axis_name: str):
    ax.plot(t_rel, marker_v, color="tab:blue", linewidth=1.8, label=f"{arm_name} marker({marker_name}) {axis_name}")
    ax.plot(t_rel, eef_v, color="tab:orange", linewidth=1.6, linestyle="--", label=f"{arm_name} eef {axis_name}")
    ax.set_ylabel(f"{arm_name} {axis_name} (m)")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best")
    ax.tick_params(axis="both", which="major", labelsize=9)


def run(args) -> None:
    data = np.load(args.input_data, allow_pickle=True)
    left_marker_name = str(data["left_marker_name"][0])
    right_marker_name = str(data["right_marker_name"][0])

    fig, axes = plt.subplots(8, 1, figsize=(13, 18), sharex=False)
    row = 0
    for arm_key, marker_name in [("left", left_marker_name), ("right", right_marker_name)]:
        eef_ts, eef_pos, eef_q, marker_ts, marker_pos, marker_q = get_arm_arrays(data, arm_key)
        eef_delta = local_delta_series(eef_pos, eef_q)
        marker_delta = local_delta_series(marker_pos, marker_q)
        eef_xyz, marker_xyz = map_axes(eef_delta, marker_delta)

        query_t = build_common_grid(eef_ts, marker_ts, min_step=0.02)
        if query_t.shape[0] < 20:
            continue

        marker_on_query = np.column_stack(
            (
                safe_interp(marker_ts, marker_xyz[:, 0], query_t),
                safe_interp(marker_ts, marker_xyz[:, 1], query_t),
                safe_interp(marker_ts, marker_xyz[:, 2], query_t),
            )
        )
        eef_on_query = np.column_stack(
            (
                safe_interp(eef_ts, eef_xyz[:, 0], query_t),
                safe_interp(eef_ts, eef_xyz[:, 1], query_t),
                safe_interp(eef_ts, eef_xyz[:, 2], query_t),
            )
        )

        t_rel = query_t - query_t[0]
        plot_axis(axes[row], t_rel, marker_on_query[:, 0], eef_on_query[:, 0], arm_key, marker_name, "X")
        plot_axis(axes[row + 1], t_rel, marker_on_query[:, 1], eef_on_query[:, 1], arm_key, marker_name, "Y")
        plot_axis(axes[row + 2], t_rel, marker_on_query[:, 2], eef_on_query[:, 2], arm_key, marker_name, "Z")
        row += 3

        t_gap, gap_ms = compute_nearest_time_gap_series(eef_ts, marker_ts)
        gap_ax = axes[row]
        if t_gap.shape[0] > 0:
            gap_ax.plot(t_gap, gap_ms, color="tab:red", linewidth=1.5, label=f"{arm_key} time gap (nearest marker - eef)")
        gap_ax.set_ylabel(f"{arm_key} gap (ms)")
        gap_ax.grid(True, linestyle=":", alpha=0.6)
        gap_ax.legend(loc="best")
        gap_ax.tick_params(axis="both", which="major", labelsize=9)
        row += 1

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Dual-arm EEF/marker comparison (RAW timestamps, no time-warp)")
    fig.tight_layout()
    fig.savefig(args.output_plot, dpi=160)
    plt.close(fig)
    print(f"Saved raw relative plot: {args.output_plot}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline raw EEF/marker plot from npz (no time alignment).")
    parser.add_argument(
        "--input-data",
        default="/home/us-mrc/Documents/dual_arm_eef_marker_raw.npz",
        help="Input .npz recorded by dual_eef_marker_data_recorder.py",
    )
    parser.add_argument(
        "--output-plot",
        default="/home/us-mrc/Documents/dual_arm_eef_marker_relative_xyz.png",
        help="Output plot path",
    )
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
