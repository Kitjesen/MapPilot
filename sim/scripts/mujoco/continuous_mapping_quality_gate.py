#!/usr/bin/env python3
"""Continuous MuJoCo native DDS mapping quality gate (3-5 minute closure).

The short native DDS motion gate only checks one endpoint displacement ratio
plus a final SLAM status snapshot. That is not sufficient evidence for
"continuous mapping is stable". This gate runs one long MuJoCo -> native DDS ->
Fast-LIO2 mapping session and produces a single verdict from four groups:

1. Continuity: periodic SLAM status samples during the whole run must stay
   TRACKING with zero dropped frames, zero rollbacks, no map-frame jump, no
   scan stalls, sane input/processing rates, and bounded odometry/velocity.
2. Scale convergence: the saved native SLAM trajectory is joined against dense
   MuJoCo ground-truth samples on the shared simulated-hardware clock. Windowed
   path-length ratios and the cumulative path ratio must stay inside bounds,
   so endpoint-displacement luck cannot pass a drifting run.
3. Trajectory consistency: after 2D rigid (rotation+translation, no scale)
   alignment the SLAM trajectory must match simulator truth within an absolute
   trajectory error budget.
4. Map quality: the native save-map artifact must pass the existing saved-map
   quality gate against the known world footprint.

Python remains an orchestrator/sensor adapter only; pose estimation and map
building stay in the native C++ SLAM runtime.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from runtime.msgs.numpy_compat import np  # noqa: E402

SCHEMA_VERSION = "lingtu.mujoco_continuous_mapping_quality_gate.v1"
# CycloneDDS on sunrise rejects high domain ids (multicast port out of range).
# Keep isolated MuJoCo gates on 200-232; production robot uses domain 0.
MAX_CYCLONEDDS_DOMAIN_ID = 232
DEFAULT_SLAM_CONFIG = SRC / "localization" / "fastlio2" / "config" / "mid360_mujoco_native_dds.yaml"
DEFAULT_SLAM_RUNTIME_BIN = ROOT / "build" / "slam_core" / "slamd"
DEFAULT_SLAM_CONTROL_BIN = ROOT / "build" / "slam_core" / "slamctl"
BRIDGE_SCRIPT = ROOT / "sim" / "scripts" / "mujoco" / "native_dds_sensors.py"


def validate_domain_id(domain_id: int) -> int:
    value = int(domain_id)
    if value < 0 or value > MAX_CYCLONEDDS_DOMAIN_ID:
        raise ValueError(
            f"--domain-id must be in [0, {MAX_CYCLONEDDS_DOMAIN_ID}] for CycloneDDS; got {value}"
        )
    return value


# ---------------------------------------------------------------------------
# Trajectory / motion-log parsing
# ---------------------------------------------------------------------------


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(math.atan2(siny_cosp, cosy_cosp))


def parse_trajectory_txt(path: Path | str) -> dict[str, Any]:
    """Parse native save-map trajectory.txt: `stamp x y z qx qy qz qw` rows."""

    t: list[float] = []
    xy: list[list[float]] = []
    z: list[float] = []
    yaw: list[float] = []
    trajectory_path = Path(path)
    if not trajectory_path.is_file():
        return {"available": False, "path": str(trajectory_path), "samples": 0}
    for line in trajectory_path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        try:
            values = [float(v) for v in parts[:8]]
        except ValueError:
            continue
        if not all(math.isfinite(v) for v in values):
            continue
        t.append(values[0])
        xy.append([values[1], values[2]])
        z.append(values[3])
        yaw.append(yaw_from_quat(values[4], values[5], values[6], values[7]))
    return {
        "available": len(t) >= 2,
        "path": str(trajectory_path),
        "samples": len(t),
        "t": np.asarray(t, dtype=np.float64),
        "xy": np.asarray(xy, dtype=np.float64).reshape(-1, 2),
        "z": np.asarray(z, dtype=np.float64),
        "yaw": np.asarray(yaw, dtype=np.float64),
    }


def load_motion_log(path: Path | str) -> dict[str, Any]:
    """Parse the bridge --motion-log JSONL ground-truth samples."""

    t: list[float] = []
    xy: list[list[float]] = []
    z: list[float] = []
    yaw: list[float] = []
    driving: list[bool] = []
    log_path = Path(path)
    if not log_path.is_file():
        return {"available": False, "path": str(log_path), "samples": 0}
    for line in log_path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            sample = json.loads(line)
        except json.JSONDecodeError:
            continue
        try:
            t.append(float(sample["t"]))
            xy.append([float(sample["x"]), float(sample["y"])])
            z.append(float(sample.get("z") or 0.0))
            yaw.append(float(sample.get("yaw") or 0.0))
            driving.append(bool(sample.get("driving")))
        except (KeyError, TypeError, ValueError):
            continue
    return {
        "available": len(t) >= 2,
        "path": str(log_path),
        "samples": len(t),
        "t": np.asarray(t, dtype=np.float64),
        "xy": np.asarray(xy, dtype=np.float64).reshape(-1, 2),
        "z": np.asarray(z, dtype=np.float64),
        "yaw": np.asarray(yaw, dtype=np.float64),
        "driving": np.asarray(driving, dtype=bool),
    }


# ---------------------------------------------------------------------------
# Scale convergence + trajectory consistency analysis
# ---------------------------------------------------------------------------


def join_time_series(
    sim_t: np.ndarray,
    slam_t: np.ndarray,
    *,
    max_dt_s: float = 0.25,
) -> tuple[np.ndarray, np.ndarray]:
    """Match each SLAM stamp to the nearest simulator truth stamp."""

    if sim_t.size == 0 or slam_t.size == 0:
        return np.zeros(0, dtype=np.int64), np.zeros(0, dtype=np.int64)
    order = np.argsort(sim_t)
    sorted_sim_t = sim_t[order]
    insert = np.searchsorted(sorted_sim_t, slam_t)
    sim_idx: list[int] = []
    slam_idx: list[int] = []
    for i, pos in enumerate(insert):
        candidates = []
        if pos > 0:
            candidates.append(pos - 1)
        if pos < sorted_sim_t.size:
            candidates.append(pos)
        best = min(candidates, key=lambda c: abs(sorted_sim_t[c] - slam_t[i]))
        if abs(sorted_sim_t[best] - slam_t[i]) <= max_dt_s:
            sim_idx.append(int(order[best]))
            slam_idx.append(i)
    return np.asarray(sim_idx, dtype=np.int64), np.asarray(slam_idx, dtype=np.int64)


def path_length_xy(xy: np.ndarray) -> float:
    if xy.shape[0] < 2:
        return 0.0
    deltas = np.diff(xy, axis=0)
    return float(np.sum(np.hypot(deltas[:, 0], deltas[:, 1])))


def rigid_align_2d(src_xy: np.ndarray, dst_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares rotation+translation (no scale) mapping src onto dst."""

    src = np.asarray(src_xy, dtype=np.float64).reshape(-1, 2)
    dst = np.asarray(dst_xy, dtype=np.float64).reshape(-1, 2)
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    cov = (dst - mu_dst).T @ (src - mu_src) / max(1, src.shape[0])
    u, _, vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(u @ vt))
    correction = np.diag([1.0, d if d != 0 else 1.0])
    rotation = u @ correction @ vt
    translation = mu_dst - rotation @ mu_src
    return rotation, translation


def ate_metrics_2d(sim_xy: np.ndarray, slam_xy: np.ndarray) -> dict[str, Any]:
    """Absolute trajectory error after 2D rigid alignment of slam onto sim."""

    if sim_xy.shape[0] < 3 or sim_xy.shape != slam_xy.shape:
        return {"available": False, "paired_samples": int(min(sim_xy.shape[0], slam_xy.shape[0]))}
    rotation, translation = rigid_align_2d(slam_xy, sim_xy)
    aligned = slam_xy @ rotation.T + translation
    residual = np.hypot(aligned[:, 0] - sim_xy[:, 0], aligned[:, 1] - sim_xy[:, 1])
    yaw_deg = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
    return {
        "available": True,
        "paired_samples": int(sim_xy.shape[0]),
        "rmse_m": float(np.sqrt(np.mean(residual**2))),
        "max_m": float(np.max(residual)),
        "mean_m": float(np.mean(residual)),
        "p90_m": float(np.percentile(residual, 90)),
        "alignment_yaw_deg": float(yaw_deg),
        "alignment_translation_m": [float(translation[0]), float(translation[1])],
    }


def unwrap_angles(yaw: np.ndarray) -> np.ndarray:
    return np.unwrap(np.asarray(yaw, dtype=np.float64))


def windowed_path_ratios(
    t: np.ndarray,
    sim_xy: np.ndarray,
    slam_xy: np.ndarray,
    sim_yaw: np.ndarray,
    slam_yaw: np.ndarray,
    *,
    window_s: float,
    min_window_sim_path_m: float,
) -> list[dict[str, Any]]:
    """Per-window slam/sim path-length ratios plus yaw-delta error."""

    windows: list[dict[str, Any]] = []
    if t.size < 2:
        return windows
    sim_yaw_u = unwrap_angles(sim_yaw)
    slam_yaw_u = unwrap_angles(slam_yaw)
    t0 = float(t[0])
    t1 = float(t[-1])
    edge = t0
    while edge < t1 - 1e-9:
        window_end = min(edge + window_s, t1)
        mask = (t >= edge - 1e-9) & (t <= window_end + 1e-9)
        idx = np.nonzero(mask)[0]
        if idx.size >= 2:
            sim_len = path_length_xy(sim_xy[idx])
            slam_len = path_length_xy(slam_xy[idx])
            sim_yaw_delta = float(sim_yaw_u[idx[-1]] - sim_yaw_u[idx[0]])
            slam_yaw_delta = float(slam_yaw_u[idx[-1]] - slam_yaw_u[idx[0]])
            windows.append(
                {
                    "t_start": float(edge),
                    "t_end": float(window_end),
                    "samples": int(idx.size),
                    "sim_path_m": sim_len,
                    "slam_path_m": slam_len,
                    "ratio": (slam_len / sim_len) if sim_len >= min_window_sim_path_m else None,
                    "sim_yaw_delta_rad": sim_yaw_delta,
                    "slam_yaw_delta_rad": slam_yaw_delta,
                    "yaw_delta_error_rad": slam_yaw_delta - sim_yaw_delta,
                }
            )
        edge = window_end
    return windows


def analyze_scale_convergence(
    motion_log: dict[str, Any],
    slam_trajectory: dict[str, Any],
    *,
    join_max_dt_s: float = 0.25,
    window_s: float = 30.0,
    min_window_sim_path_m: float = 0.3,
    min_window_ratio: float = 0.5,
    max_window_ratio: float = 1.8,
    min_cumulative_ratio: float = 0.7,
    max_cumulative_ratio: float = 1.4,
    max_window_yaw_delta_error_rad: float = 0.35,
    max_ate_rmse_m: float = 0.6,
    max_ate_max_m: float = 1.2,
    min_joined_samples: int = 50,
) -> dict[str, Any]:
    """Windowed + cumulative scale convergence plus rigid-aligned ATE."""

    report: dict[str, Any] = {
        "available": False,
        "thresholds": {
            "join_max_dt_s": float(join_max_dt_s),
            "window_s": float(window_s),
            "min_window_sim_path_m": float(min_window_sim_path_m),
            "min_window_ratio": float(min_window_ratio),
            "max_window_ratio": float(max_window_ratio),
            "min_cumulative_ratio": float(min_cumulative_ratio),
            "max_cumulative_ratio": float(max_cumulative_ratio),
            "max_window_yaw_delta_error_rad": float(max_window_yaw_delta_error_rad),
            "max_ate_rmse_m": float(max_ate_rmse_m),
            "max_ate_max_m": float(max_ate_max_m),
            "min_joined_samples": int(min_joined_samples),
        },
        "remaining_gaps": [],
    }
    gaps: list[str] = report["remaining_gaps"]
    if not motion_log.get("available"):
        gaps.append("sim_motion_log_missing_or_empty")
        return report
    if not slam_trajectory.get("available"):
        gaps.append("slam_trajectory_missing_or_empty")
        return report

    sim_idx, slam_idx = join_time_series(
        motion_log["t"], slam_trajectory["t"], max_dt_s=join_max_dt_s
    )
    joined_total = int(slam_idx.size)
    report["joined_samples"] = joined_total
    report["slam_trajectory_samples"] = int(slam_trajectory["samples"])
    report["sim_motion_samples"] = int(motion_log["samples"])
    if joined_total < max(2, min_joined_samples):
        gaps.append(
            f"trajectory_time_join_failed:joined={joined_total},"
            f"slam={int(slam_trajectory['samples'])},sim={int(motion_log['samples'])}"
        )
        return report
    join_fraction = joined_total / max(1, int(slam_trajectory["samples"]))
    report["join_fraction_of_slam_samples"] = float(join_fraction)
    if join_fraction < 0.5:
        gaps.append(f"trajectory_clock_mismatch:join_fraction={join_fraction:.3f}")

    driving_mask = motion_log["driving"][sim_idx]
    keep = np.nonzero(driving_mask)[0]
    if keep.size < max(2, min_joined_samples):
        gaps.append(f"driving_phase_samples_insufficient:{int(keep.size)}")
        return report
    sim_sel = sim_idx[keep]
    slam_sel = slam_idx[keep]
    t = slam_trajectory["t"][slam_sel]
    sim_xy = motion_log["xy"][sim_sel]
    slam_xy = slam_trajectory["xy"][slam_sel]
    sim_yaw = motion_log["yaw"][sim_sel]
    slam_yaw = slam_trajectory["yaw"][slam_sel]

    report["available"] = True
    report["driving_samples"] = int(keep.size)
    report["driving_span_s"] = float(t[-1] - t[0])

    sim_total = path_length_xy(sim_xy)
    slam_total = path_length_xy(slam_xy)
    cumulative_ratio = (slam_total / sim_total) if sim_total > 1e-6 else None
    report["sim_path_length_m"] = sim_total
    report["slam_path_length_m"] = slam_total
    report["cumulative_path_ratio"] = cumulative_ratio

    windows = windowed_path_ratios(
        t,
        sim_xy,
        slam_xy,
        sim_yaw,
        slam_yaw,
        window_s=window_s,
        min_window_sim_path_m=min_window_sim_path_m,
    )
    report["windows"] = windows

    if cumulative_ratio is None:
        gaps.append("cumulative_ratio_unavailable:sim_path_zero")
    elif not (min_cumulative_ratio <= cumulative_ratio <= max_cumulative_ratio):
        gaps.append(
            "cumulative_path_ratio_out_of_bounds:"
            f"ratio={cumulative_ratio:.3f},bounds=[{min_cumulative_ratio:.2f},{max_cumulative_ratio:.2f}]"
        )

    rated = [w for w in windows if w["ratio"] is not None]
    report["rated_windows"] = len(rated)
    if not rated:
        gaps.append("no_rated_windows:sim_motion_too_small_per_window")
    for window in rated:
        ratio = float(window["ratio"])
        if not (min_window_ratio <= ratio <= max_window_ratio):
            gaps.append(
                "window_path_ratio_out_of_bounds:"
                f"t=[{window['t_start']:.1f},{window['t_end']:.1f}],ratio={ratio:.3f},"
                f"bounds=[{min_window_ratio:.2f},{max_window_ratio:.2f}]"
            )
    for window in windows:
        yaw_error = float(window["yaw_delta_error_rad"])
        if abs(yaw_error) > max_window_yaw_delta_error_rad:
            gaps.append(
                "window_yaw_delta_error_too_large:"
                f"t=[{window['t_start']:.1f},{window['t_end']:.1f}],error={yaw_error:.3f},"
                f"max={max_window_yaw_delta_error_rad:.2f}"
            )

    ate = ate_metrics_2d(sim_xy, slam_xy)
    report["ate"] = ate
    if not ate.get("available"):
        gaps.append("ate_unavailable")
    else:
        if float(ate["rmse_m"]) > max_ate_rmse_m:
            gaps.append(f"ate_rmse_too_large:{float(ate['rmse_m']):.3f}>max={max_ate_rmse_m:.2f}")
        if float(ate["max_m"]) > max_ate_max_m:
            gaps.append(f"ate_max_too_large:{float(ate['max_m']):.3f}>max={max_ate_max_m:.2f}")
    return report


# ---------------------------------------------------------------------------
# Status continuity analysis
# ---------------------------------------------------------------------------


def _sample_float(sample: dict[str, Any], key: str) -> float | None:
    value = sample.get(key)
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def analyze_status_continuity(
    samples: list[dict[str, Any]],
    *,
    duration_s: float,
    bridge_published: dict[str, Any] | None = None,
    bridge_warmup_s: float = 0.0,
    max_tracking_latency_s: float = 25.0,
    max_dropped_lidar_frames: int = 0,
    max_dropped_imu_frames: int = 0,
    max_rollbacks: int = 0,
    min_lidar_input_hz: float = 7.0,
    min_imu_input_hz: float = 140.0,
    min_processed_scan_hz: float = 6.0,
    max_scan_stall_s: float = 3.0,
    max_odom_abs_m: float = 100.0,
    max_odom_z_abs_m: float = 5.0,
    max_velocity_mps: float = 3.5,
    min_final_localization_quality: float = 0.5,
    min_samples: int = 20,
) -> dict[str, Any]:
    """Verify the SLAM runtime never degraded during the continuous run."""

    report: dict[str, Any] = {
        "available": False,
        "samples": len(samples),
        "thresholds": {
            "max_tracking_latency_s": float(max_tracking_latency_s),
            "max_dropped_lidar_frames": int(max_dropped_lidar_frames),
            "max_dropped_imu_frames": int(max_dropped_imu_frames),
            "max_rollbacks": int(max_rollbacks),
            "min_lidar_input_hz": float(min_lidar_input_hz),
            "min_imu_input_hz": float(min_imu_input_hz),
            "min_processed_scan_hz": float(min_processed_scan_hz),
            "max_scan_stall_s": float(max_scan_stall_s),
            "max_odom_abs_m": float(max_odom_abs_m),
            "max_odom_z_abs_m": float(max_odom_z_abs_m),
            "max_velocity_mps": float(max_velocity_mps),
            "min_final_localization_quality": float(min_final_localization_quality),
            "min_samples": int(min_samples),
        },
        "remaining_gaps": [],
    }
    gaps: list[str] = report["remaining_gaps"]
    if len(samples) < max(2, min_samples):
        gaps.append(f"status_samples_insufficient:{len(samples)}")
        return report
    report["available"] = True

    states = [str(sample.get("state") or "").upper() for sample in samples]
    wall_t = [float(sample.get("_gate_wall_t") or 0.0) for sample in samples]
    first_tracking_idx = next((i for i, s in enumerate(states) if s == "TRACKING"), None)
    if first_tracking_idx is None:
        gaps.append("never_reached_tracking")
        return report
    tracking_latency_s = wall_t[first_tracking_idx] - wall_t[0]
    report["tracking_latency_s"] = float(tracking_latency_s)
    if tracking_latency_s > max_tracking_latency_s:
        gaps.append(f"tracking_latency_too_large:{tracking_latency_s:.1f}s")

    regressions = sorted(
        {states[i] for i in range(first_tracking_idx, len(states)) if states[i] != "TRACKING"}
    )
    report["post_tracking_state_regressions"] = regressions
    if regressions:
        gaps.append("tracking_regression:" + ",".join(regressions))

    counters = {
        "dropped_lidar_frames": max_dropped_lidar_frames,
        "dropped_imu_frames": max_dropped_imu_frames,
        "imu_rollback_count": max_rollbacks,
        "lidar_rollback_count": max_rollbacks,
    }
    report["final_counters"] = {}
    for key, limit in counters.items():
        observed = max(int(sample.get(key) or 0) for sample in samples)
        report["final_counters"][key] = observed
        if observed > int(limit):
            gaps.append(f"{key}_exceeded:{observed}>max={int(limit)}")

    if any(bool(sample.get("map_frame_jump")) for sample in samples):
        gaps.append("map_frame_jump_detected")

    tracking_samples = samples[first_tracking_idx:]
    rate_specs = (
        ("lidar_input_hz", min_lidar_input_hz, "/lidar/raw_frame"),
        ("imu_input_hz", min_imu_input_hz, "/imu/raw"),
        ("processed_scan_hz", min_processed_scan_hz, None),
    )
    report["rates"] = {}
    drive_span_s = max(1e-3, float(duration_s))
    bridge_counts = dict(bridge_published or {})
    bridge_imu_hz = None
    bridge_lidar_hz = None
    if bridge_counts:
        bridge_imu_hz = float(bridge_counts.get("/imu/raw") or 0) / drive_span_s
        bridge_lidar_hz = float(bridge_counts.get("/lidar/raw_frame") or 0) / drive_span_s
        report["bridge_published_hz"] = {
            "drive_span_s": drive_span_s,
            "warmup_s": float(bridge_warmup_s),
            "imu_input_hz": bridge_imu_hz,
            "lidar_input_hz": bridge_lidar_hz,
        }
    for key, minimum, bridge_topic in rate_specs:
        values = [v for v in (_sample_float(s, key) for s in tracking_samples) if v is not None]
        median = float(np.median(values)) if values else 0.0
        effective = median
        source = "status_median"
        if bridge_topic == "/imu/raw" and bridge_imu_hz is not None:
            effective = max(effective, bridge_imu_hz)
            if bridge_imu_hz >= float(minimum):
                source = "bridge_published"
        elif bridge_topic == "/lidar/raw_frame" and bridge_lidar_hz is not None:
            effective = max(effective, bridge_lidar_hz)
            if bridge_lidar_hz >= float(minimum):
                source = "bridge_published"
        report["rates"][key] = {
            "median": median,
            "effective": effective,
            "source": source,
            "samples": len(values),
        }
        if effective < float(minimum):
            gaps.append(f"{key}_effective_too_low:{effective:.2f}<min={float(minimum):.2f}")

    stamps = [v for v in (_sample_float(s, "stamp_s") for s in tracking_samples) if v is not None]
    max_stall = 0.0
    for prev, curr in zip(stamps, stamps[1:]):
        if curr > prev:
            max_stall = max(max_stall, curr - prev)
    # A frozen stamp across gate samples is also a stall.
    frozen_run_s = 0.0
    run_start: float | None = None
    for i in range(1, len(tracking_samples)):
        same = stamps[i] == stamps[i - 1] if i < len(stamps) else False
        if same:
            if run_start is None:
                run_start = wall_t[first_tracking_idx + i - 1]
            frozen_run_s = max(
                frozen_run_s, wall_t[first_tracking_idx + i] - run_start
            )
        else:
            run_start = None
    report["max_scan_stamp_gap_s"] = float(max_stall)
    report["max_scan_stamp_frozen_s"] = float(frozen_run_s)
    if max(max_stall, frozen_run_s) > max_scan_stall_s:
        gaps.append(
            f"scan_processing_stall:{max(max_stall, frozen_run_s):.2f}s>max={max_scan_stall_s:.2f}s"
        )

    worst_abs_xy = 0.0
    worst_abs_z = 0.0
    worst_velocity = 0.0
    for sample in tracking_samples:
        pose = ((sample.get("odometry") or {}).get("pose") or {})
        x = _sample_float(pose, "x")
        y = _sample_float(pose, "y")
        z = _sample_float(pose, "z")
        if x is None or y is None or z is None:
            gaps.append("odometry_not_finite")
            break
        worst_abs_xy = max(worst_abs_xy, abs(x), abs(y))
        worst_abs_z = max(worst_abs_z, abs(z))
        velocity = sample.get("fastlio_velocity") or {}
        vx = _sample_float(velocity, "x") or 0.0
        vy = _sample_float(velocity, "y") or 0.0
        vz = _sample_float(velocity, "z") or 0.0
        worst_velocity = max(worst_velocity, math.sqrt(vx * vx + vy * vy + vz * vz))
    report["worst_abs_odom_xy_m"] = float(worst_abs_xy)
    report["worst_abs_odom_z_m"] = float(worst_abs_z)
    report["worst_velocity_mps"] = float(worst_velocity)
    if worst_abs_xy > max_odom_abs_m:
        gaps.append(f"odometry_diverged_xy:{worst_abs_xy:.2f}>max={max_odom_abs_m:.1f}")
    if worst_abs_z > max_odom_z_abs_m:
        gaps.append(f"odometry_diverged_z:{worst_abs_z:.2f}>max={max_odom_z_abs_m:.1f}")
    if worst_velocity > max_velocity_mps:
        gaps.append(f"velocity_spike:{worst_velocity:.2f}>max={max_velocity_mps:.2f}")

    final_quality = _sample_float(samples[-1], "localization_quality") or 0.0
    report["final_localization_quality"] = float(final_quality)
    if final_quality < min_final_localization_quality:
        gaps.append(f"final_localization_quality_low:{final_quality:.2f}")

    observed_span = wall_t[-1] - wall_t[0]
    report["observed_span_s"] = float(observed_span)
    if observed_span < 0.5 * float(duration_s):
        gaps.append(f"status_sampling_span_short:{observed_span:.1f}s<half_duration")
    return report


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="industrial_park")
    parser.add_argument("--duration", type=float, default=180.0, help="Drive duration seconds (3-5 min gate).")
    parser.add_argument(
        "--domain-id",
        type=int,
        default=231,
        help=f"Isolated CycloneDDS domain in [0, {MAX_CYCLONEDDS_DOMAIN_ID}]. Do not use production domain 0.",
    )
    parser.add_argument("--run-dir", default="", help="Artifact directory; default artifacts/mujoco_continuous_mapping_gate_<ts>.")
    parser.add_argument("--drive-profile", choices=["arc", "box_explore", "box_explore_gentle"], default="box_explore")
    parser.add_argument("--drive-vx", type=float, default=0.10)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.04)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--imu-hz", type=float, default=200.0)
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument("--publisher-bin", default=os.environ.get("LINGTU_MUJOCO_NATIVE_DDS_PUBLISHER_BIN", ""))
    parser.add_argument("--slam-runtime-bin", default=str(DEFAULT_SLAM_RUNTIME_BIN))
    parser.add_argument("--slam-control-bin", default=str(DEFAULT_SLAM_CONTROL_BIN))
    parser.add_argument("--slam-config", default=str(DEFAULT_SLAM_CONFIG))
    parser.add_argument(
        "--attach-status-json",
        default="",
        help="Attach to an externally started SLAM runtime via its status JSON instead of spawning one.",
    )
    parser.add_argument("--status-sample-period-s", type=float, default=0.5)
    parser.add_argument("--save-timeout-s", type=float, default=300.0)
    parser.add_argument("--bridge-min-motion-ratio", type=float, default=0.2)
    parser.add_argument("--bridge-max-motion-ratio", type=float, default=3.0)
    parser.add_argument("--bridge-max-yaw-error-rad", type=float, default=0.30)
    parser.add_argument(
        "--bridge-arg",
        action="append",
        default=[],
        help="Extra argument forwarded verbatim to native_dds_sensors.py (repeatable).",
    )
    # Continuity thresholds.
    parser.add_argument("--max-tracking-latency-s", type=float, default=25.0)
    parser.add_argument("--min-lidar-input-hz", type=float, default=7.0)
    parser.add_argument("--min-imu-input-hz", type=float, default=140.0)
    parser.add_argument("--min-processed-scan-hz", type=float, default=6.0)
    parser.add_argument("--max-scan-stall-s", type=float, default=3.0)
    parser.add_argument("--max-odom-abs-m", type=float, default=100.0)
    parser.add_argument("--max-odom-z-abs-m", type=float, default=5.0)
    parser.add_argument("--max-velocity-mps", type=float, default=3.5)
    # Convergence thresholds.
    parser.add_argument("--window-s", type=float, default=30.0)
    parser.add_argument("--min-window-sim-path-m", type=float, default=0.3)
    parser.add_argument("--min-window-ratio", type=float, default=0.5)
    parser.add_argument("--max-window-ratio", type=float, default=1.8)
    parser.add_argument("--min-cumulative-ratio", type=float, default=0.7)
    parser.add_argument("--max-cumulative-ratio", type=float, default=1.4)
    parser.add_argument("--max-window-yaw-delta-error-rad", type=float, default=0.35)
    parser.add_argument("--max-ate-rmse-m", type=float, default=0.6)
    parser.add_argument("--max-ate-max-m", type=float, default=1.2)
    # Map quality thresholds (forwarded to the saved-map quality gate).
    parser.add_argument("--min-near-ratio", type=float, default=0.80)
    parser.add_argument("--max-far-ratio", type=float, default=0.15)
    parser.add_argument("--json-out", default="")
    return parser


def _read_status_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, UnicodeDecodeError):
        return None


def _terminate(process: subprocess.Popen[Any] | None, *, timeout_s: float = 10.0) -> int | None:
    if process is None:
        return None
    if process.poll() is None:
        try:
            if os.name != "nt":
                process.send_signal(signal.SIGTERM)
            else:
                process.terminate()
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                pass
    return process.poll()


def _start_slam_runtime(
    args: argparse.Namespace,
    run_dir: Path,
    status_path: Path,
    domain_id: int,
) -> subprocess.Popen[Any]:
    runtime_bin = Path(args.slam_runtime_bin)
    if not runtime_bin.exists():
        raise FileNotFoundError(
            f"native SLAM runtime missing: {runtime_bin}. Build with "
            "LINGTU_SLAM_BUILD_DDS_RUNTIME=ON bash scripts/build/build_slam_core.sh"
        )
    log = (run_dir / "slam_runtime.log").open("w", encoding="utf-8")
    command = [
        str(runtime_bin),
        "--backend",
        "fastlio2",
        "--mode",
        "mapping",
        "--config",
        str(args.slam_config),
        "--domain-id",
        str(domain_id),
        "--status-json",
        str(status_path),
        "--status-json-hz",
        "10",
        "--log-status-s",
        "10",
    ]
    return subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)


def _bridge_command(
    args: argparse.Namespace,
    run_dir: Path,
    status_path: Path,
    domain_id: int,
) -> list[str]:
    command = [
        sys.executable,
        str(BRIDGE_SCRIPT),
        "--world",
        str(args.world),
        "--duration",
        str(float(args.duration)),
        "--settle-s",
        "3.0",
        "--warmup-s",
        "2.0",
        "--drive-ramp-s",
        "5.0",
        "--publish-hz",
        str(float(args.publish_hz)),
        "--imu-hz",
        str(float(args.imu_hz)),
        "--drive-mode",
        "policy",
        "--drive-profile",
        str(args.drive_profile),
        "--drive-vx",
        str(float(args.drive_vx)),
        "--drive-vy",
        str(float(args.drive_vy)),
        "--drive-wz",
        str(float(args.drive_wz)),
        "--imu-acc-mode",
        "sensor",
        "--imu-acc-conditioning",
        "realistic",
        "--imu-acc-axis-scale",
        "auto",
        "--scan-time-profile",
        "physical_rolling",
        "--physical-rolling-sample-mode",
        "subscan",
        "--timestamp-clock",
        "sim_hardware",
        "--sim-hardware-realtime-factor",
        str(float(args.realtime_factor)),
        "--domain-id",
        str(domain_id),
        "--slam-status-json",
        str(status_path),
        "--min-slam-motion-ratio",
        str(float(args.bridge_min_motion_ratio)),
        "--max-slam-motion-ratio",
        str(float(args.bridge_max_motion_ratio)),
        "--max-slam-yaw-error-rad",
        str(float(args.bridge_max_yaw_error_rad)),
        "--require-slam-output",
        "--motion-log",
        str(run_dir / "sim_motion.jsonl"),
        "--json-out",
        str(run_dir / "bridge_report.json"),
    ]
    if args.publisher_bin:
        command += ["--publisher-bin", str(args.publisher_bin)]
    for extra in args.bridge_arg:
        command += str(extra).split()
    return command


def _write_plots(
    run_dir: Path,
    convergence: dict[str, Any],
    motion_log: dict[str, Any],
    slam_trajectory: dict[str, Any],
) -> dict[str, str]:
    plots: dict[str, str] = {}
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return plots

    windows = convergence.get("windows") or []
    if windows:
        fig, ax = plt.subplots(figsize=(9, 4.5), dpi=140)
        t_mid = [(w["t_start"] + w["t_end"]) / 2.0 - windows[0]["t_start"] for w in windows]
        ratios = [w["ratio"] for w in windows]
        ax.plot(
            [t for t, r in zip(t_mid, ratios) if r is not None],
            [r for r in ratios if r is not None],
            "o-",
            label="window path ratio",
        )
        cumulative = convergence.get("cumulative_path_ratio")
        if cumulative is not None:
            ax.axhline(cumulative, color="tab:green", linestyle="-", alpha=0.6, label=f"cumulative={cumulative:.3f}")
        thresholds = convergence.get("thresholds") or {}
        for bound_key, style in (("min_window_ratio", ":"), ("max_window_ratio", ":")):
            if bound_key in thresholds:
                ax.axhline(float(thresholds[bound_key]), color="tab:red", linestyle=style, alpha=0.6)
        ax.set_xlabel("drive time s")
        ax.set_ylabel("slam/sim path ratio")
        ax.set_title("Continuous mapping scale convergence")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        out = run_dir / "scale_convergence.png"
        fig.savefig(out)
        plt.close(fig)
        plots["scale_convergence"] = str(out)

    if motion_log.get("available") and slam_trajectory.get("available"):
        sim_idx, slam_idx = join_time_series(motion_log["t"], slam_trajectory["t"], max_dt_s=0.25)
        if slam_idx.size >= 3:
            sim_xy = motion_log["xy"][sim_idx]
            slam_xy = slam_trajectory["xy"][slam_idx]
            rotation, translation = rigid_align_2d(slam_xy, sim_xy)
            aligned = slam_xy @ rotation.T + translation
            fig, ax = plt.subplots(figsize=(7, 7), dpi=140)
            ax.plot(sim_xy[:, 0], sim_xy[:, 1], "-", color="tab:gray", label="MuJoCo truth")
            ax.plot(aligned[:, 0], aligned[:, 1], "-", color="tab:blue", label="SLAM (rigid aligned)")
            ax.plot(sim_xy[0, 0], sim_xy[0, 1], "k^", markersize=8, label="start")
            ax.set_aspect("equal", adjustable="box")
            ax.set_xlabel("x m")
            ax.set_ylabel("y m")
            ax.set_title("Continuous mapping trajectory overlay")
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=8)
            fig.tight_layout()
            out = run_dir / "trajectory_overlay.png"
            fig.savefig(out)
            plt.close(fig)
            plots["trajectory_overlay"] = str(out)
    return plots


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    domain_id = validate_domain_id(int(args.domain_id))
    started_at = time.strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.run_dir) if args.run_dir else ROOT / "artifacts" / f"mujoco_continuous_mapping_gate_{started_at}"
    run_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "ok": False,
        "simulation_only": True,
        "no_python_slam": True,
        "started_at": started_at,
        "run_dir": str(run_dir),
        "world": str(args.world),
        "duration_s": float(args.duration),
        "domain_id": domain_id,
        "drive_profile": str(args.drive_profile),
        "remaining_gaps": [],
    }
    gaps: list[str] = summary["remaining_gaps"]

    attach_mode = bool(str(args.attach_status_json or "").strip())
    status_path = Path(args.attach_status_json) if attach_mode else run_dir / "slam_status.json"
    slam_process: subprocess.Popen[Any] | None = None
    bridge_process: subprocess.Popen[Any] | None = None
    status_samples: list[dict[str, Any]] = []
    samples_path = run_dir / "slam_status_samples.jsonl"

    try:
        if not attach_mode:
            slam_process = _start_slam_runtime(args, run_dir, status_path, domain_id)
            summary["slam_runtime"] = {"started": True, "bin": str(args.slam_runtime_bin), "config": str(args.slam_config)}
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and not status_path.exists():
                if slam_process.poll() is not None:
                    raise RuntimeError(f"slam runtime exited early rc={slam_process.returncode}")
                time.sleep(0.2)
            if not status_path.exists():
                raise RuntimeError("slam runtime did not produce status json within 20s")
        else:
            summary["slam_runtime"] = {"started": False, "attached_status_json": str(status_path)}
            if not status_path.exists():
                raise RuntimeError(f"attach status json missing: {status_path}")

        bridge_command = _bridge_command(args, run_dir, status_path, domain_id)
        summary["bridge_command"] = bridge_command
        bridge_log = (run_dir / "bridge.log").open("w", encoding="utf-8")
        bridge_process = subprocess.Popen(bridge_command, stdout=bridge_log, stderr=subprocess.STDOUT, cwd=str(ROOT))

        sample_period = max(0.2, float(args.status_sample_period_s))
        with samples_path.open("w", encoding="utf-8") as samples_stream:
            while True:
                bridge_rc = bridge_process.poll()
                status = _read_status_json(status_path)
                if status is not None:
                    status["_gate_wall_t"] = time.time()
                    status_samples.append(status)
                    samples_stream.write(json.dumps(status, ensure_ascii=True) + "\n")
                if bridge_rc is not None:
                    break
                if slam_process is not None and slam_process.poll() is not None:
                    _terminate(bridge_process)
                    raise RuntimeError(f"slam runtime died during run rc={slam_process.returncode}")
                time.sleep(sample_period)
        summary["bridge_rc"] = int(bridge_process.returncode or 0)

        bridge_report_path = run_dir / "bridge_report.json"
        bridge_report: dict[str, Any] = {}
        if bridge_report_path.is_file():
            bridge_report = json.loads(bridge_report_path.read_text(encoding="utf-8"))
        summary["bridge_report_ok"] = bool(bridge_report.get("ok"))
        summary["bridge_report_gaps"] = list(bridge_report.get("remaining_gaps") or [])
        summary["bridge_motion"] = bridge_report.get("motion") or {}
        summary["published"] = bridge_report.get("published") or {}
        if not bridge_report:
            gaps.append("bridge_report_missing")
        elif not bridge_report.get("ok"):
            gaps.extend(f"bridge:{gap}" for gap in bridge_report.get("remaining_gaps") or ["bridge_report_not_ok"])

        # Save the cumulative native map while the runtime is still alive.
        save_pcd = run_dir / "saved_map" / "map.pcd"
        control_bin = Path(args.slam_control_bin)
        save_report: dict[str, Any] = {"attempted": False}
        if not control_bin.exists():
            gaps.append(f"slam_control_bin_missing:{control_bin}")
        else:
            save_command = [
                str(control_bin),
                "save-map",
                str(save_pcd),
                "--domain-id",
                str(domain_id),
                "--timeout-s",
                str(float(args.save_timeout_s)),
            ]
            save_result = subprocess.run(
                save_command,
                capture_output=True,
                text=True,
                timeout=float(args.save_timeout_s) + 30.0,
            )
            save_report = {
                "attempted": True,
                "rc": int(save_result.returncode),
                "stdout": save_result.stdout.strip()[-2000:],
                "stderr": save_result.stderr.strip()[-2000:],
                "pcd": str(save_pcd),
            }
            if save_result.returncode != 0 or not save_pcd.is_file():
                gaps.append(f"native_save_map_failed:rc={save_result.returncode}")
        summary["save_map"] = save_report
    finally:
        _terminate(bridge_process)
        if slam_process is not None:
            summary.setdefault("slam_runtime", {})["rc"] = _terminate(slam_process)

    # Continuity over the whole run.
    continuity = analyze_status_continuity(
        status_samples,
        duration_s=float(args.duration),
        bridge_published=bridge_report.get("published") if bridge_report else None,
        bridge_warmup_s=float(bridge_report.get("warmup_s") or 0.0) if bridge_report else 0.0,
        max_tracking_latency_s=float(args.max_tracking_latency_s),
        min_lidar_input_hz=float(args.min_lidar_input_hz),
        min_imu_input_hz=float(args.min_imu_input_hz),
        min_processed_scan_hz=float(args.min_processed_scan_hz),
        max_scan_stall_s=float(args.max_scan_stall_s),
        max_odom_abs_m=float(args.max_odom_abs_m),
        max_odom_z_abs_m=float(args.max_odom_z_abs_m),
        max_velocity_mps=float(args.max_velocity_mps),
    )
    summary["continuity"] = continuity
    gaps.extend(f"continuity:{gap}" for gap in continuity.get("remaining_gaps") or [])
    summary["status_samples_path"] = str(samples_path)

    # Scale convergence + trajectory consistency.
    motion_log = load_motion_log(run_dir / "sim_motion.jsonl")
    slam_trajectory = parse_trajectory_txt(run_dir / "saved_map" / "trajectory.txt")
    convergence = analyze_scale_convergence(
        motion_log,
        slam_trajectory,
        window_s=float(args.window_s),
        min_window_sim_path_m=float(args.min_window_sim_path_m),
        min_window_ratio=float(args.min_window_ratio),
        max_window_ratio=float(args.max_window_ratio),
        min_cumulative_ratio=float(args.min_cumulative_ratio),
        max_cumulative_ratio=float(args.max_cumulative_ratio),
        max_window_yaw_delta_error_rad=float(args.max_window_yaw_delta_error_rad),
        max_ate_rmse_m=float(args.max_ate_rmse_m),
        max_ate_max_m=float(args.max_ate_max_m),
    )
    summary["convergence"] = convergence
    gaps.extend(f"convergence:{gap}" for gap in convergence.get("remaining_gaps") or [])

    # Saved-map quality gate.
    quality_summary: dict[str, Any] = {"attempted": False}
    save_pcd = run_dir / "saved_map" / "map.pcd"
    if save_pcd.is_file():
        from sim.scripts.mujoco.saved_map_quality_gate import (
            _write_overlay_plot,
            evaluate_saved_map_quality,
        )

        quality_report, candidates, expected = evaluate_saved_map_quality(
            pcd_path=save_pcd,
            world_xml=str(args.world),
            min_near_ratio=float(args.min_near_ratio),
            max_far_ratio=float(args.max_far_ratio),
        )
        quality_json = run_dir / "saved_map_quality.json"
        quality_json.write_text(json.dumps(quality_report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        quality_summary = {
            "attempted": True,
            "ok": bool(quality_report.get("ok")),
            "json": str(quality_json),
            "near_ratio": float(
                (quality_report.get("scene_overlay") or {}).get("candidate_cells_within_near_distance_ratio") or 0.0
            ),
            "far_ratio": float(
                (quality_report.get("scene_overlay") or {}).get("candidate_cells_farther_than_far_distance_ratio") or 1.0
            ),
            "map_optimization": quality_report.get("map_optimization") or {},
        }
        try:
            plot_path = run_dir / "saved_map_quality.png"
            _write_overlay_plot(
                out_path=plot_path,
                candidates=candidates,
                expected=expected,
                cell_m=0.2,
                far_distance_m=0.6,
                overlay_metrics=quality_report.get("scene_overlay"),
            )
            quality_summary["plot"] = str(plot_path)
        except Exception as exc:  # pragma: no cover - plotting is best effort
            quality_summary["plot_error"] = f"{type(exc).__name__}: {exc}"
        if not quality_report.get("ok"):
            gaps.extend(f"map_quality:{gap}" for gap in quality_report.get("remaining_gaps") or [])
    else:
        gaps.append("saved_map_pcd_missing")
    summary["map_quality"] = quality_summary

    summary["plots"] = _write_plots(run_dir, convergence, motion_log, slam_trajectory)
    summary["ok"] = not gaps
    return summary


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        validate_domain_id(int(args.domain_id))
        summary = run_gate(args)
    except Exception as exc:
        summary = {
            "schema_version": SCHEMA_VERSION,
            "ok": False,
            "remaining_gaps": [f"gate_failed:{type(exc).__name__}"],
            "error": str(exc),
        }
    text = json.dumps(summary, ensure_ascii=True, indent=2, sort_keys=True)
    json_out = str(getattr(args, "json_out", "") or "")
    if json_out:
        out = Path(json_out)
    else:
        run_dir = summary.get("run_dir")
        out = Path(run_dir) / "summary.json" if run_dir else None
    if out is not None:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0 if summary.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
