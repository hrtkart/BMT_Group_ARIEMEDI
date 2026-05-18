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
    ts_a = ts_a[np.isfinite(ts_a)]
    ts_b = ts_b[np.isfinite(ts_b)]
    if ts_a.shape[0] < 2 or ts_b.shape[0] < 2:
        return np.zeros((0,), dtype=np.float64)
    ts_a = np.sort(ts_a)
    ts_b = np.sort(ts_b)
    t0 = max(float(ts_a[0]), float(ts_b[0]))
    t1 = min(float(ts_a[-1]), float(ts_b[-1]))
    if t1 <= t0:
        return np.zeros((0,), dtype=np.float64)
    step = max(median_step(ts_a), median_step(ts_b), min_step)
    if not np.isfinite(step) or step <= 0.0:
        step = min_step
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
    ts = ts[order]
    ys = ys[order]
    tq = ts_query[np.isfinite(ts_query)]
    if tq.shape[0] != ts_query.shape[0]:
        out = np.zeros_like(ts_query, dtype=np.float64)
        finite_mask = np.isfinite(ts_query)
        out[finite_mask] = np.interp(ts_query[finite_mask], ts, ys)
        return out
    return np.interp(ts_query, ts, ys)


def corr_score(a: np.ndarray, b: np.ndarray) -> float:
    if a.shape[0] < 10 or b.shape[0] < 10:
        return -1.0
    sa = float(np.std(a))
    sb = float(np.std(b))
    if sa < 1e-12 or sb < 1e-12:
        return -1.0
    return float(np.corrcoef(a, b)[0, 1])


def fit_clock_map(raw_clock_sec: np.ndarray, ros_stamp_sec: np.ndarray) -> Tuple[float, float]:
    mask = np.isfinite(raw_clock_sec) & np.isfinite(ros_stamp_sec)
    x = raw_clock_sec[mask]
    y = ros_stamp_sec[mask]
    if x.shape[0] < 2:
        return 1.0, 0.0
    a, b = np.polyfit(x, y, 1)
    return float(a), float(b)


def apply_clock_map(raw_clock_sec: np.ndarray, ros_stamp_sec: np.ndarray) -> np.ndarray:
    a, b = fit_clock_map(raw_clock_sec, ros_stamp_sec)
    mapped = a * raw_clock_sec + b
    fallback_mask = ~np.isfinite(mapped)
    if np.any(fallback_mask):
        mapped = np.array(mapped, copy=True)
        mapped[fallback_mask] = ros_stamp_sec[fallback_mask]
    return mapped


def smooth_series(x: np.ndarray, width: int) -> np.ndarray:
    if x.shape[0] == 0 or width <= 1:
        return x
    if width % 2 == 0:
        width += 1
    kernel = np.ones(width, dtype=np.float64) / float(width)
    return np.convolve(x, kernel, mode="same")


def best_lag_for_window(
    t_win: np.ndarray,
    eef_main_axis: np.ndarray,
    marker_ts: np.ndarray,
    marker_main_axis: np.ndarray,
    max_lag_sec: float,
) -> float:
    if t_win.shape[0] < 10:
        return 0.0
    dt = float(np.median(np.diff(t_win)))
    lag_candidates = np.arange(-max_lag_sec, max_lag_sec + dt * 0.5, dt, dtype=np.float64)

    best_lag = 0.0
    best_score = -1e9
    for lag in lag_candidates:
        shifted_t = t_win - lag
        m = safe_interp(marker_ts, marker_main_axis, shifted_t)
        if m.shape[0] != t_win.shape[0]:
            continue
        score = corr_score(eef_main_axis, m)
        if score > best_score:
            best_score = score
            best_lag = float(lag)
    return best_lag


def estimate_lag_curve(
    eef_ts: np.ndarray,
    eef_main_axis: np.ndarray,
    marker_ts: np.ndarray,
    marker_main_axis: np.ndarray,
    window_sec: float,
    hop_sec: float,
    max_lag_sec: float,
) -> Tuple[np.ndarray, np.ndarray]:
    t_grid = build_common_grid(eef_ts, marker_ts, min_step=0.02)
    if t_grid.shape[0] < 20:
        return np.zeros((0,), dtype=np.float64), np.zeros((0,), dtype=np.float64)

    eef_i = safe_interp(eef_ts, eef_main_axis, t_grid)

    dt = float(np.median(np.diff(t_grid)))
    window_n = max(int(window_sec / dt), 30)
    hop_n = max(int(hop_sec / dt), 10)
    if window_n >= t_grid.shape[0]:
        lag = best_lag_for_window(t_grid, eef_i, marker_ts, marker_main_axis, max_lag_sec)
        return np.array([float(np.mean(t_grid))], dtype=np.float64), np.array([lag], dtype=np.float64)

    centers = []
    lags = []
    for s in range(0, t_grid.shape[0] - window_n + 1, hop_n):
        e = s + window_n
        t_win = t_grid[s:e]
        eef_win = eef_i[s:e]
        lag = best_lag_for_window(t_win, eef_win, marker_ts, marker_main_axis, max_lag_sec)
        centers.append(float(np.mean(t_win)))
        lags.append(lag)
    lag_arr = np.array(lags, dtype=np.float64)
    lag_arr = smooth_series(lag_arr, width=5)
    return np.array(centers, dtype=np.float64), lag_arr


def apply_time_warp(
    query_t: np.ndarray,
    lag_t: np.ndarray,
    lag_v: np.ndarray,
    marker_ts: np.ndarray,
    marker_xyz: np.ndarray,
) -> np.ndarray:
    if query_t.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if lag_t.shape[0] == 0:
        lag_on_query = np.zeros_like(query_t)
    elif lag_t.shape[0] == 1:
        lag_on_query = np.full_like(query_t, lag_v[0])
    else:
        lag_on_query = np.interp(query_t, lag_t, lag_v, left=lag_v[0], right=lag_v[-1])

    sample_t = query_t - lag_on_query
    mx = safe_interp(marker_ts, marker_xyz[:, 0], sample_t)
    my = safe_interp(marker_ts, marker_xyz[:, 1], sample_t)
    mz = safe_interp(marker_ts, marker_xyz[:, 2], sample_t)
    return np.column_stack((mx, my, mz))


def map_axes(eef_delta: np.ndarray, marker_delta: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # marker X/Y/Z <-> eef Y/Z/X
    eef_mapped = np.column_stack((eef_delta[:, 1], eef_delta[:, 2], eef_delta[:, 0]))
    marker_mapped = np.column_stack((marker_delta[:, 0], marker_delta[:, 1], marker_delta[:, 2]))
    return eef_mapped, marker_mapped


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


def get_arm_arrays(data: np.lib.npyio.NpzFile, arm_key: str):
    eef_ts = np.array(data[f"{arm_key}_eef_stamp_sec"], dtype=np.float64)
    if f"{arm_key}_eef_device_clock_sec" in data.files:
        eef_device = np.array(data[f"{arm_key}_eef_device_clock_sec"], dtype=np.float64)
    else:
        eef_device = np.array(eef_ts, dtype=np.float64)
    eef_pos = np.array(data[f"{arm_key}_eef_pos"], dtype=np.float64)
    eef_q = np.array(data[f"{arm_key}_eef_quat_xyzw"], dtype=np.float64)
    marker_ts = np.array(data[f"{arm_key}_marker_stamp_sec"], dtype=np.float64)
    if f"{arm_key}_marker_device_clock_sec" in data.files:
        marker_device = np.array(data[f"{arm_key}_marker_device_clock_sec"], dtype=np.float64)
    else:
        marker_device = np.array(marker_ts, dtype=np.float64)
    marker_pos = np.array(data[f"{arm_key}_marker_pos"], dtype=np.float64)
    marker_q = np.array(data[f"{arm_key}_marker_quat_xyzw"], dtype=np.float64)
    return eef_ts, eef_device, eef_pos, eef_q, marker_ts, marker_device, marker_pos, marker_q


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

    fig, axes = plt.subplots(10, 1, figsize=(13, 22), sharex=False)
    row = 0
    for arm_key, marker_name in [("left", left_marker_name), ("right", right_marker_name)]:
        (
            eef_ros_ts,
            eef_device_clock,
            eef_pos,
            eef_q,
            marker_ros_ts,
            marker_device_clock,
            marker_pos,
            marker_q,
        ) = get_arm_arrays(data, arm_key)

        eef_ts = apply_clock_map(eef_device_clock, eef_ros_ts)
        marker_ts = apply_clock_map(marker_device_clock, marker_ros_ts)
        eef_delta = local_delta_series(eef_pos, eef_q)
        marker_delta = local_delta_series(marker_pos, marker_q)
        eef_xyz, marker_xyz = map_axes(eef_delta, marker_delta)

        query_t = build_common_grid(eef_ts, marker_ts, min_step=0.02)
        if query_t.shape[0] < 20:
            continue

        lag_t, lag_v = estimate_lag_curve(
            eef_ts=eef_ts,
            eef_main_axis=eef_xyz[:, 1],
            marker_ts=marker_ts,
            marker_main_axis=marker_xyz[:, 1],
            window_sec=args.window_sec,
            hop_sec=args.hop_sec,
            max_lag_sec=args.max_lag_sec,
        )
        marker_aligned = apply_time_warp(query_t, lag_t, lag_v, marker_ts, marker_xyz)
        eef_on_query = np.column_stack(
            (
                safe_interp(eef_ts, eef_xyz[:, 0], query_t),
                safe_interp(eef_ts, eef_xyz[:, 1], query_t),
                safe_interp(eef_ts, eef_xyz[:, 2], query_t),
            )
        )

        t_rel = query_t - query_t[0]
        plot_axis(axes[row], t_rel, marker_aligned[:, 0], eef_on_query[:, 0], arm_key, marker_name, "X")
        plot_axis(axes[row + 1], t_rel, marker_aligned[:, 1], eef_on_query[:, 1], arm_key, marker_name, "Y")
        plot_axis(axes[row + 2], t_rel, marker_aligned[:, 2], eef_on_query[:, 2], arm_key, marker_name, "Z")
        row += 3

        lag_ax = axes[row]
        if lag_t.shape[0] > 0:
            lag_ax.plot(lag_t - query_t[0], lag_v * 1000.0, color="tab:green", linewidth=1.8, label=f"{arm_key} lag estimate")
        lag_ax.set_ylabel(f"{arm_key} lag (ms)")
        lag_ax.grid(True, linestyle=":", alpha=0.6)
        lag_ax.legend(loc="best")
        lag_ax.tick_params(axis="both", which="major", labelsize=9)
        row += 1

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
    fig.suptitle(
        "Dual-arm EEF/marker comparison with clock mapping + time-warp alignment\n"
        f"window={args.window_sec:.1f}s, hop={args.hop_sec:.1f}s, max_lag={args.max_lag_sec:.1f}s"
    )
    fig.tight_layout()
    fig.savefig(args.output_plot, dpi=160)
    plt.close(fig)
    print(f"Saved time-aligned plot: {args.output_plot}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline EEF/marker time-drift alignment from npz.")
    parser.add_argument(
        "--input-data",
        default="/home/us-mrc/Documents/dual_arm_eef_marker_raw.npz",
        help="Input .npz recorded by dual_eef_marker_data_recorder.py",
    )
    parser.add_argument(
        "--output-plot",
        default="/home/us-mrc/Documents/dual_arm_eef_marker_time_aligned_xyz.png",
        help="Output plot path",
    )
    parser.add_argument("--window-sec", type=float, default=8.0, help="Window length for lag estimation")
    parser.add_argument("--hop-sec", type=float, default=2.0, help="Hop length for lag estimation")
    parser.add_argument("--max-lag-sec", type=float, default=1.0, help="Maximum lag search range")
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
