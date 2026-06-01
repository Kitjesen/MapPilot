import pytest

pytestmark = [pytest.mark.sim]

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

from sim.scripts import moving_obstacle_sweep_gate


def _write_child_report(
    path: Path,
    *,
    speed_mps: float,
    spacing_m: float,
    include_live_nav_chain: bool = True,
    scan_time_profile: str = "physical_rolling",
) -> Path:
    payload = {
        "ok": True,
        "simulation_only": True,
        "scan_time_profile": scan_time_profile,
        "lidar_source": {
            "scan_time_profile": scan_time_profile,
            "scan_time_model_contract": (
                "physical_subscans_with_actual_sim_time_offsets"
                if scan_time_profile == "physical_rolling"
                else "synthetic_snapshot_offsets"
            ),
        },
        "nav_data_source": "fastlio2",
        "true_mapping_input_path": (
            "/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_map "
            "-> /nav/odometry + /nav/map_cloud"
        ),
        "outputs": {
            "nav_odometry": 20,
            "nav_map_cloud": 20,
            "nav_cmd_vel_nonzero": 12,
        },
        "lingtu_inspection": {
            "enabled": True,
            "verified": True,
            "global_planner": "pct",
            "global_path_count": 1,
            "global_path_points_max": 15,
            "local_path_count": 24,
            "local_path_points_max": 101,
            "successful_navigation_goal_count": 3,
            "min_required_checkpoints": 3,
            "replan_on_costmap_update": False,
        },
        "navigation_chain": {
            "planner_fallback_used": False,
            "planner_repair_used": False,
        },
        "moving_obstacles": {
            "enabled": True,
            "mode": "robot_crossing",
            "count": 3,
            "period_s": 6.0,
            "point_spacing_m": spacing_m,
            "speed_bounds": {
                "peak_planar_speed_bound_mps": speed_mps,
            },
            "ok": True,
            "published_update_count": 12,
            "published_point_count_max": 96,
            "trail_clearance": {
                "checked": True,
                "collision": False,
                "min_clearance_minus_robot_radius_m": 0.35,
            },
        },
        "video_path": str(path.with_suffix(".mp4")),
        "video_frame_count": 24,
        "video_sample_count": 24,
    }
    if not include_live_nav_chain:
        payload.pop("nav_data_source")
        payload.pop("true_mapping_input_path")
        payload.pop("outputs")
        payload.pop("lingtu_inspection")
        payload.pop("navigation_chain")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_moving_obstacle_sweep_requires_speed_density_pairs(tmp_path: Path):
    reports = [
        _write_child_report(tmp_path / "slow_sparse.json", speed_mps=0.35, spacing_m=0.16),
        _write_child_report(tmp_path / "slow_dense.json", speed_mps=0.35, spacing_m=0.05),
        _write_child_report(tmp_path / "fast_sparse.json", speed_mps=0.95, spacing_m=0.16),
        _write_child_report(tmp_path / "fast_dense.json", speed_mps=0.95, spacing_m=0.05),
    ]

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        reports,
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
    )

    assert summary["ok"] is True
    assert summary["passed_pair_count"] == 4
    assert summary["required_pairs"] == [
        "slow:sparse",
        "slow:dense",
        "fast:sparse",
        "fast:dense",
    ]
    assert summary["missing_pairs"] == []


def test_moving_obstacle_sweep_rejects_single_speed_density_point(tmp_path: Path):
    report = _write_child_report(tmp_path / "single.json", speed_mps=0.95, spacing_m=0.10)

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
    )

    assert summary["ok"] is False
    assert summary["passed_pair_count"] == 0
    assert summary["covered_pairs"] == ["fast:medium"]
    assert summary["missing_pairs"] == [
        "slow:sparse",
        "slow:dense",
        "fast:sparse",
        "fast:dense",
    ]
    assert "missing required speed bins: slow" in summary["blockers"]
    assert "missing required density bins: sparse,dense" in summary["blockers"]


def test_moving_obstacle_sweep_rejects_child_without_realtime_nav_chain(tmp_path: Path):
    report = _write_child_report(
        tmp_path / "weak.json",
        speed_mps=0.35,
        spacing_m=0.16,
        include_live_nav_chain=False,
    )

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("slow",),
        required_density_bins=("sparse",),
    )

    assert summary["ok"] is False
    assert summary["passed_pair_count"] == 0
    assert summary["cases"][0]["live_nav_chain"]["ok"] is False
    assert "live nav chain: nav_data_source is not fastlio2" in summary["cases"][0]["blockers"]
    assert "live nav chain: lingtu_inspection.verified is not true" in summary["cases"][0]["blockers"]


def test_moving_obstacle_sweep_surfaces_fastlio_consistency_failure(tmp_path: Path):
    report = _write_child_report(
        tmp_path / "fastlio_red.json",
        speed_mps=0.35,
        spacing_m=0.16,
    )
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["ok"] = False
    payload["fastlio2_motion_consistency"] = {
        "checked": True,
        "ok": False,
        "fastlio2_moved_m": 9.37,
        "sim_moved_m": 0.24,
        "moved_error_m": 9.13,
    }
    payload["fastlio2_z_consistency"] = {
        "checked": True,
        "ok": False,
        "z_delta_error_m": 3.77,
        "max_allowed_z_drift_m": 1.0,
    }
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("slow",),
        required_density_bins=("sparse",),
    )

    assert summary["ok"] is False
    case = summary["cases"][0]
    assert case["fastlio2_consistency"]["motion_ok"] is False
    assert case["fastlio2_consistency"]["z_delta_error_m"] == 3.77
    assert "slam_localization" in case["blocking_subsystems"]
    assert summary["minimal_red_defect"]["blocking_subsystem"] == "slam_localization"
    assert summary["minimal_red_defect"]["fastlio2_z_delta_error_m"] == 3.77


def test_moving_obstacle_sweep_prioritizes_fastlio_failure_over_downstream_tracking(
    tmp_path: Path,
):
    report = _write_child_report(
        tmp_path / "fastlio_and_tracking_red.json",
        speed_mps=0.95,
        spacing_m=0.05,
    )
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["ok"] = False
    payload["fastlio2_motion_consistency"] = {
        "checked": True,
        "ok": False,
        "fastlio2_moved_m": 10.7,
        "sim_moved_m": 0.27,
        "moved_error_m": 10.43,
    }
    payload["fastlio2_z_consistency"] = {
        "checked": True,
        "ok": False,
        "z_delta_error_m": 4.43,
        "max_allowed_z_drift_m": 1.0,
    }
    payload["lingtu_inspection"]["verified"] = False
    payload["lingtu_inspection"]["successful_navigation_goal_count"] = 1
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("fast",),
        required_density_bins=("dense",),
    )

    case = summary["cases"][0]
    assert case["blocking_subsystems"][:2] == [
        "slam_localization",
        "planning_tracking",
    ]
    assert summary["minimal_red_defect"]["blocking_subsystem"] == "slam_localization"
    assert summary["minimal_red_defect"]["fastlio2_z_delta_error_m"] == 4.43


def test_moving_obstacle_sweep_rejects_non_physical_rolling_child(tmp_path: Path):
    report = _write_child_report(
        tmp_path / "synthetic.json",
        speed_mps=0.35,
        spacing_m=0.16,
        scan_time_profile="synthetic_rolling",
    )

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("slow",),
        required_density_bins=("sparse",),
        required_scan_time_profile="physical_rolling",
    )

    assert summary["ok"] is False
    assert summary["cases"][0]["scan_time_profile"] == "synthetic_rolling"
    assert (
        "scan_time_profile is not physical_rolling"
        in summary["cases"][0]["blockers"]
    )


def test_moving_obstacle_sweep_can_require_video_file(tmp_path: Path):
    report = _write_child_report(
        tmp_path / "no_video_file.json",
        speed_mps=0.35,
        spacing_m=0.16,
    )

    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("slow",),
        required_density_bins=("sparse",),
        require_video_file=True,
    )

    assert summary["ok"] is False
    assert "video file missing" in summary["cases"][0]["blockers"]


def test_moving_obstacle_sweep_builds_runtime_matrix_parameters(tmp_path: Path):
    cases = moving_obstacle_sweep_gate.sweep_matrix_cases(
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
        moving_obstacle_count=3,
        lateral_amplitude_m=0.85,
        along_amplitude_m=0.25,
    )

    assert [case["case_id"] for case in cases] == [
        "slow-sparse",
        "slow-dense",
        "fast-sparse",
        "fast-dense",
    ]
    assert cases[0]["period_s"] > cases[2]["period_s"]
    assert cases[0]["point_spacing_m"] > cases[1]["point_spacing_m"]
    assert cases[0]["expected_speed_bin"] == "slow"
    assert cases[2]["expected_speed_bin"] == "fast"
    assert cases[0]["expected_density_bin"] == "sparse"
    assert cases[1]["expected_density_bin"] == "dense"

    env = moving_obstacle_sweep_gate.build_case_environment(
        cases[3],
        child_run_root=tmp_path / "children",
        duration_s=123.0,
        inspection_goals="0.5,0.0;1.0,0.0;1.5,0.0",
        inspection_tomogram=Path("artifacts/tomogram.pickle"),
        world="industrial_park",
        ros_domain_id="99",
    )

    assert env["LINGTU_MUJOCO_LIVE_RUN_DIR"] == str(tmp_path / "children" / "fast-dense")
    assert env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_COUNT"] == "3"
    assert env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S"] == "2"
    assert math.isclose(
        float(env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S"]),
        cases[3]["period_s"],
        rel_tol=1e-5,
    )
    assert math.isclose(
        float(env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_POINT_SPACING"]),
        cases[3]["point_spacing_m"],
        rel_tol=1e-5,
    )
    assert env["LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER"] == "pct"
    assert env["LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM"] == "artifacts/tomogram.pickle"
    assert env["LINGTU_MUJOCO_LIVE_WORLD"] == "industrial_park"
    assert env["ROS_DOMAIN_ID"] == "99"
    assert env["LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE"] == "physical_rolling"


def test_moving_obstacle_sweep_preflight_rejects_missing_launch_script(tmp_path: Path):
    preflight = moving_obstacle_sweep_gate.run_matrix_preflight(
        launch_script=tmp_path / "missing_launcher.sh",
        world=None,
        inspection_tomogram=None,
    )

    assert preflight["ok"] is False
    assert preflight["launch_script"]["exists"] is False
    assert any("launch_script missing" in item for item in preflight["environment_blockers"])
    assert preflight["blocking_subsystems"] == ["environment_runtime"]


def test_moving_obstacle_sweep_preflight_rejects_missing_world_file(tmp_path: Path):
    launch = tmp_path / "launch.sh"
    launch.write_text("#!/usr/bin/env bash\n", encoding="utf-8")

    preflight = moving_obstacle_sweep_gate.run_matrix_preflight(
        launch_script=launch,
        world=tmp_path / "missing_world.xml",
        inspection_tomogram=None,
    )

    assert preflight["ok"] is False
    assert preflight["world"]["path_checked"] is True
    assert preflight["world"]["exists"] is False
    assert any("world file missing" in item for item in preflight["artifact_blockers"])
    assert preflight["blocking_subsystems"] == ["artifact_contract"]


def test_moving_obstacle_sweep_preflight_rejects_missing_inspection_tomogram(
    tmp_path: Path,
):
    launch = tmp_path / "launch.sh"
    launch.write_text("#!/usr/bin/env bash\n", encoding="utf-8")

    preflight = moving_obstacle_sweep_gate.run_matrix_preflight(
        launch_script=launch,
        world="industrial_park",
        inspection_tomogram=tmp_path / "missing_tomogram.pickle",
    )

    assert preflight["ok"] is False
    assert preflight["inspection_tomogram"]["exists"] is False
    assert any(
        "inspection_tomogram missing" in item
        for item in preflight["artifact_blockers"]
    )
    assert preflight["blocking_subsystems"] == ["artifact_contract"]


def test_moving_obstacle_sweep_run_matrix_skips_runner_when_preflight_fails(
    tmp_path: Path,
    monkeypatch,
):
    def fail_if_called(*args, **kwargs):
        raise AssertionError("run_matrix_cases should not run after preflight failure")

    output = tmp_path / "moving_obstacle_sweep_report.json"
    monkeypatch.setattr(moving_obstacle_sweep_gate, "run_matrix_cases", fail_if_called)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "moving_obstacle_sweep_gate.py",
            "--run-matrix",
            "--launch-script",
            str(tmp_path / "missing_launcher.sh"),
            "--json-out",
            str(output),
            "--strict",
        ],
    )

    assert moving_obstacle_sweep_gate.main() == 1
    summary = json.loads(output.read_text(encoding="utf-8"))
    assert summary["ok"] is False
    assert summary["case_count"] == 0
    assert summary["run_matrix"]["case_count"] == 0
    assert summary["preflight"]["ok"] is False
    assert any("launch_script missing" in item for item in summary["environment_blockers"])


def test_moving_obstacle_sweep_run_matrix_collects_child_reports(tmp_path: Path):
    def fake_runner(command, *, cwd, env, timeout, check):
        case_root = Path(env["LINGTU_MUJOCO_LIVE_RUN_DIR"])
        run_dir = case_root / "inspection-moving-obstacle-video-fake"
        report_path = run_dir / "report.json"
        period_s = float(env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_PERIOD_S"])
        spacing_m = float(env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_POINT_SPACING"])
        speed_mps = (
            2.0
            * math.pi
            * math.hypot(
                float(env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_LATERAL_AMPLITUDE_M"]),
                float(env["LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_ALONG_AMPLITUDE_M"]),
            )
            / period_s
        )
        _write_child_report(report_path, speed_mps=speed_mps, spacing_m=spacing_m)
        case_root.mkdir(parents=True, exist_ok=True)
        (case_root / "latest.txt").write_text(f"latest_run_dir={run_dir}\n", encoding="utf-8")
        return subprocess.CompletedProcess(command, 0)

    cases = moving_obstacle_sweep_gate.sweep_matrix_cases(
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
    )
    run_results = moving_obstacle_sweep_gate.run_matrix_cases(
        cases,
        child_run_root=tmp_path / "children",
        launch_script=Path("sim/scripts/launch_mujoco_fastlio2_live.sh"),
        case_timeout_s=30.0,
        duration_s=20.0,
        runner=fake_runner,
    )

    reports = [Path(result["report_path"]) for result in run_results if result["report_path"]]
    summary = moving_obstacle_sweep_gate.evaluate_reports(
        reports,
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
    )

    assert all(result["returncode"] == 0 for result in run_results)
    assert len(reports) == 4
    assert summary["ok"] is True
    assert summary["covered_pairs"] == [
        "slow:sparse",
        "slow:dense",
        "fast:sparse",
        "fast:dense",
    ]


def test_moving_obstacle_sweep_run_matrix_isolates_ros_domain_per_case(tmp_path: Path):
    domains: list[str] = []

    def fake_runner(command, *, cwd, env, timeout, check):
        domains.append(env["ROS_DOMAIN_ID"])
        case_root = Path(env["LINGTU_MUJOCO_LIVE_RUN_DIR"])
        run_dir = case_root / "inspection-moving-obstacle-video-fake"
        _write_child_report(run_dir / "report.json", speed_mps=0.35, spacing_m=0.16)
        case_root.mkdir(parents=True, exist_ok=True)
        (case_root / "latest.txt").write_text(f"latest_run_dir={run_dir}\n", encoding="utf-8")
        return subprocess.CompletedProcess(command, 0)

    cases = moving_obstacle_sweep_gate.sweep_matrix_cases(
        required_speed_bins=("slow", "fast"),
        required_density_bins=("sparse", "dense"),
    )

    moving_obstacle_sweep_gate.run_matrix_cases(
        cases,
        child_run_root=tmp_path / "children",
        ros_domain_id=90,
        runner=fake_runner,
    )

    assert domains == ["90", "91", "92", "93"]


def test_moving_obstacle_sweep_accepts_timeout_when_child_report_passes(tmp_path: Path):
    report = _write_child_report(
        tmp_path / "children" / "fast-dense" / "inspection" / "report.json",
        speed_mps=0.95,
        spacing_m=0.05,
    )
    summary = moving_obstacle_sweep_gate.evaluate_reports(
        [report],
        required_speed_bins=("fast",),
        required_density_bins=("dense",),
    )
    run_results = [
        {
            "case_id": "fast-dense",
            "returncode": None,
            "report_path": str(report),
            "error": "timeout after 500s",
        }
    ]

    blockers, warnings = moving_obstacle_sweep_gate.matrix_run_diagnostics(
        run_results,
        summary["cases"],
    )

    assert blockers == []
    assert warnings == [
        "fast-dense: timeout after 500s; accepted because the child report passed"
    ]
