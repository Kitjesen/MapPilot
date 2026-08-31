from __future__ import annotations

import inspect
import json
import math
from pathlib import Path

import numpy as np
import pytest

from sim.scripts.mujoco.continuous_mapping_quality_gate import (
    MAX_CYCLONEDDS_DOMAIN_ID,
    analyze_scale_convergence,
    analyze_status_continuity,
    ate_metrics_2d,
    build_parser,
    join_time_series,
    load_motion_log,
    parse_trajectory_txt,
    path_length_xy,
    rigid_align_2d,
    run_gate,
    validate_domain_id,
    windowed_path_ratios,
)


def _arc_trajectory(duration_s: float = 120.0, hz: float = 10.0, speed: float = 0.12):
    t = np.arange(0.0, duration_s, 1.0 / hz)
    yaw = 0.04 * t
    x = np.cumsum(np.cos(yaw) * speed / hz)
    y = np.cumsum(np.sin(yaw) * speed / hz)
    return t, np.column_stack([x, y]), yaw


def _motion_log_dict(t, xy, yaw, driving=None):
    if driving is None:
        driving = np.ones(t.shape[0], dtype=bool)
    return {
        "available": True,
        "samples": int(t.size),
        "t": t,
        "xy": xy,
        "z": np.zeros_like(t),
        "yaw": yaw,
        "driving": driving,
    }


def _trajectory_dict(t, xy, yaw):
    return {
        "available": True,
        "samples": int(t.size),
        "t": t,
        "xy": xy,
        "z": np.zeros_like(t),
        "yaw": yaw,
    }


def _status_sample(
    wall_t: float,
    *,
    state: str = "TRACKING",
    stamp_s: float = 0.0,
    dropped_lidar: int = 0,
    dropped_imu: int = 0,
    rollbacks: int = 0,
    lidar_hz: float = 9.5,
    imu_hz: float = 195.0,
    processed_hz: float = 9.2,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.05,
    velocity: float = 0.15,
    quality: float = 1.0,
):
    return {
        "_gate_wall_t": wall_t,
        "state": state,
        "stamp_s": stamp_s,
        "dropped_lidar_frames": dropped_lidar,
        "dropped_imu_frames": dropped_imu,
        "imu_rollback_count": rollbacks,
        "lidar_rollback_count": rollbacks,
        "map_frame_jump": False,
        "lidar_input_hz": lidar_hz,
        "imu_input_hz": imu_hz,
        "processed_scan_hz": processed_hz,
        "localization_quality": quality,
        "fastlio_velocity": {"x": velocity, "y": 0.0, "z": 0.0},
        "odometry": {"pose": {"x": x, "y": y, "z": z}},
    }


def _healthy_samples(duration_s: float = 180.0, period_s: float = 0.5):
    samples = []
    count = int(duration_s / period_s)
    for i in range(count):
        wall = 1000.0 + i * period_s
        state = "INITIALIZING" if wall - 1000.0 < 4.0 else "TRACKING"
        samples.append(
            _status_sample(
                wall,
                state=state,
                stamp_s=5000.0 + i * period_s,
                x=0.01 * i * period_s,
                y=0.005 * i * period_s,
            )
        )
    return samples


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------


def test_rigid_align_recovers_rotation_translation():
    rng = np.random.default_rng(7)
    src = rng.uniform(-5, 5, size=(200, 2))
    theta = 0.7
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    dst = src @ rot.T + np.array([2.5, -1.0])
    est_rot, est_t = rigid_align_2d(src, dst)
    assert np.allclose(est_rot, rot, atol=1e-9)
    assert np.allclose(est_t, [2.5, -1.0], atol=1e-9)


def test_ate_is_frame_invariant():
    t, xy, yaw = _arc_trajectory()
    theta = -1.2
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    translation = np.array([10.0, -4.0])
    moved = xy @ rot.T + translation
    metrics = ate_metrics_2d(xy, moved)
    assert metrics["available"]
    assert metrics["rmse_m"] < 1e-9
    assert metrics["max_m"] < 1e-9


def test_join_time_series_respects_max_dt():
    sim_t = np.arange(0.0, 10.0, 0.1)
    slam_t = np.array([0.02, 5.03, 9.91, 30.0])
    sim_idx, slam_idx = join_time_series(sim_t, slam_t, max_dt_s=0.25)
    assert slam_idx.tolist() == [0, 1, 2]
    assert np.all(np.abs(sim_t[sim_idx] - slam_t[slam_idx]) <= 0.25)


def test_path_length_xy():
    xy = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 4.0]])
    assert abs(path_length_xy(xy) - 7.0) < 1e-12


# ---------------------------------------------------------------------------
# Window / convergence analysis
# ---------------------------------------------------------------------------


def test_windowed_ratios_near_one_for_identical_paths():
    t, xy, yaw = _arc_trajectory()
    windows = windowed_path_ratios(t, xy, xy, yaw, yaw, window_s=30.0, min_window_sim_path_m=0.3)
    assert len(windows) == 4
    for window in windows:
        assert window["ratio"] is not None
        assert abs(window["ratio"] - 1.0) < 1e-9
        assert abs(window["yaw_delta_error_rad"]) < 1e-9


def test_convergence_passes_for_consistent_run():
    t, xy, yaw = _arc_trajectory()
    theta = 0.9
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    slam_xy = xy @ rot.T + np.array([1.0, 2.0])
    report = analyze_scale_convergence(
        _motion_log_dict(t, xy, yaw),
        _trajectory_dict(t, slam_xy, yaw + theta),
    )
    assert report["available"]
    assert report["remaining_gaps"] == []
    assert abs(report["cumulative_path_ratio"] - 1.0) < 1e-6
    assert report["ate"]["rmse_m"] < 1e-6


def test_convergence_flags_scale_overshoot():
    t, xy, yaw = _arc_trajectory()
    slam_xy = xy * 1.9  # severe translation over-shoot, same shape
    report = analyze_scale_convergence(
        _motion_log_dict(t, xy, yaw),
        _trajectory_dict(t, slam_xy, yaw),
    )
    joined = "\n".join(report["remaining_gaps"])
    assert "cumulative_path_ratio_out_of_bounds" in joined
    assert "window_path_ratio_out_of_bounds" in joined


def test_convergence_flags_late_drift():
    t, xy, yaw = _arc_trajectory()
    slam_xy = xy.copy()
    half = t.size // 2
    # Second half drifts away laterally; rigid alignment cannot absorb this.
    drift = np.linspace(0.0, 3.5, t.size - half)
    slam_xy[half:, 1] += drift
    report = analyze_scale_convergence(
        _motion_log_dict(t, xy, yaw),
        _trajectory_dict(t, slam_xy, yaw),
    )
    joined = "\n".join(report["remaining_gaps"])
    assert "ate_rmse_too_large" in joined or "ate_max_too_large" in joined


def test_convergence_flags_yaw_drift():
    t, xy, yaw = _arc_trajectory()
    slam_yaw = yaw + np.linspace(0.0, 1.6, t.size)  # steady heading drift
    report = analyze_scale_convergence(
        _motion_log_dict(t, xy, yaw),
        _trajectory_dict(t, xy, slam_yaw),
    )
    joined = "\n".join(report["remaining_gaps"])
    assert "window_yaw_delta_error_too_large" in joined


def test_convergence_requires_time_overlap():
    t, xy, yaw = _arc_trajectory()
    report = analyze_scale_convergence(
        _motion_log_dict(t, xy, yaw),
        _trajectory_dict(t + 1000.0, xy, yaw),
    )
    assert not report["available"]
    assert any(gap.startswith("trajectory_time_join_failed") for gap in report["remaining_gaps"])


def test_convergence_ignores_warmup_samples():
    t, xy, yaw = _arc_trajectory()
    driving = np.ones(t.size, dtype=bool)
    driving[: t.size // 4] = False
    # SLAM garbage during (excluded) warmup must not affect the verdict.
    slam_xy = xy.copy()
    slam_xy[: t.size // 4] += 50.0
    report = analyze_scale_convergence(
        _motion_log_dict(t, xy, yaw, driving),
        _trajectory_dict(t, slam_xy, yaw),
    )
    assert report["available"]
    assert report["remaining_gaps"] == []


# ---------------------------------------------------------------------------
# Continuity analysis
# ---------------------------------------------------------------------------


def test_continuity_passes_for_healthy_run():
    report = analyze_status_continuity(_healthy_samples(), duration_s=180.0)
    assert report["available"]
    assert report["remaining_gaps"] == []
    assert report["final_counters"]["dropped_lidar_frames"] == 0


def test_continuity_flags_drops_and_rollbacks():
    samples = _healthy_samples()
    samples[-1]["dropped_lidar_frames"] = 4
    samples[-1]["imu_rollback_count"] = 1
    report = analyze_status_continuity(samples, duration_s=180.0)
    joined = "\n".join(report["remaining_gaps"])
    assert "dropped_lidar_frames_exceeded" in joined
    assert "imu_rollback_count_exceeded" in joined


def test_continuity_flags_tracking_regression():
    samples = _healthy_samples()
    samples[200]["state"] = "LOST"
    report = analyze_status_continuity(samples, duration_s=180.0)
    assert any(gap.startswith("tracking_regression:LOST") for gap in report["remaining_gaps"])


def test_continuity_flags_scan_stall():
    samples = _healthy_samples()
    frozen = samples[100]["stamp_s"]
    for sample in samples[100:120]:  # 10 s of frozen scan stamps
        sample["stamp_s"] = frozen
    report = analyze_status_continuity(samples, duration_s=180.0)
    assert any(gap.startswith("scan_processing_stall") for gap in report["remaining_gaps"])


def test_continuity_flags_divergence_and_velocity():
    samples = _healthy_samples()
    samples[150]["odometry"]["pose"]["z"] = 12.0
    samples[151]["fastlio_velocity"]["x"] = 9.0
    report = analyze_status_continuity(samples, duration_s=180.0)
    joined = "\n".join(report["remaining_gaps"])
    assert "odometry_diverged_z" in joined
    assert "velocity_spike" in joined


def test_continuity_flags_low_rates():
    samples = _healthy_samples()
    for sample in samples:
        sample["processed_scan_hz"] = 3.0
    report = analyze_status_continuity(samples, duration_s=180.0)
    assert any(gap.startswith("processed_scan_hz_effective_too_low") for gap in report["remaining_gaps"])


def test_continuity_uses_bridge_published_hz_when_status_under_reports():
    samples = _healthy_samples()
    for sample in samples:
        sample["lidar_input_hz"] = 6.0
        sample["imu_input_hz"] = 120.0
    report = analyze_status_continuity(
        samples,
        duration_s=180.0,
        bridge_published={"/imu/raw": 36000, "/lidar/raw_frame": 1800},
    )
    assert report["remaining_gaps"] == []
    assert report["rates"]["lidar_input_hz"]["source"] == "bridge_published"
    assert report["rates"]["imu_input_hz"]["source"] == "bridge_published"


# ---------------------------------------------------------------------------
# File parsing round trips
# ---------------------------------------------------------------------------


def test_parse_trajectory_txt_roundtrip(tmp_path: Path):
    trajectory = tmp_path / "trajectory.txt"
    yaw = 0.5
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    trajectory.write_text(
        f"100.0 1.0 2.0 0.1 0 0 {qz} {qw}\n"
        f"100.1 1.1 2.0 0.1 0 0 {qz} {qw}\n"
        "bad line\n",
        encoding="utf-8",
    )
    parsed = parse_trajectory_txt(trajectory)
    assert parsed["available"]
    assert parsed["samples"] == 2
    assert abs(parsed["yaw"][0] - yaw) < 1e-9
    assert np.allclose(parsed["xy"][1], [1.1, 2.0])


def test_load_motion_log_roundtrip(tmp_path: Path):
    log = tmp_path / "sim_motion.jsonl"
    lines = [
        json.dumps({"t": 100.0, "sim_time_s": 5.0, "x": 0.0, "y": 0.0, "z": 0.3, "yaw": 0.0, "driving": False}),
        json.dumps({"t": 100.1, "sim_time_s": 5.1, "x": 0.01, "y": 0.0, "z": 0.3, "yaw": 0.01, "driving": True}),
        "not json",
    ]
    log.write_text("\n".join(lines) + "\n", encoding="utf-8")
    parsed = load_motion_log(log)
    assert parsed["available"]
    assert parsed["samples"] == 2
    assert parsed["driving"].tolist() == [False, True]


def test_missing_files_report_unavailable(tmp_path: Path):
    assert not parse_trajectory_txt(tmp_path / "missing.txt")["available"]
    assert not load_motion_log(tmp_path / "missing.jsonl")["available"]


def test_validate_domain_id_accepts_isolated_range():
    assert validate_domain_id(0) == 0
    assert validate_domain_id(231) == 231
    assert validate_domain_id(MAX_CYCLONEDDS_DOMAIN_ID) == MAX_CYCLONEDDS_DOMAIN_ID


def test_validate_domain_id_rejects_out_of_range():
    with pytest.raises(ValueError, match="CycloneDDS"):
        validate_domain_id(MAX_CYCLONEDDS_DOMAIN_ID + 1)
    with pytest.raises(ValueError, match="CycloneDDS"):
        validate_domain_id(-1)


def test_gate_consumes_external_mapd_artifacts_without_slamctl_save() -> None:
    args = build_parser().parse_args([])
    run_source = inspect.getsource(run_gate)

    assert args.saved_map_dir == ""
    assert not hasattr(args, "slam_control_bin")
    assert not hasattr(args, "save_timeout_s")
    assert "subprocess.run(" not in run_source
    assert '"save-map"' not in run_source
