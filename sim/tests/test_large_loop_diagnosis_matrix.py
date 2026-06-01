from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import json
from pathlib import Path

from sim.scripts import large_loop_diagnosis_matrix


def _write_report(
    path: Path,
    *,
    nav_data_source: str = "fastlio2",
    drive_source: str = "nav_cmd_vel",
    z_ok: bool = True,
    z_error: float = 0.02,
    verified: bool = True,
    successful_goals: int = 4,
    nav_fresh: bool = True,
    local_paths: int = 60,
) -> Path:
    payload = {
        "ok": bool(z_ok and verified),
        "nav_data_source": nav_data_source,
        "drive_source": drive_source,
        "sim_path_length_m": 22.0 if verified else 0.9,
        "fastlio2_path_length_m": 21.5 if z_ok else 6.7,
        "runtime_faults": (
            [] if z_ok else ["runtime Fast-LIO Z drift (error=1.807m, allowed=1.000m)"]
        ),
        "fastlio2_z_consistency": {
            "checked": True,
            "ok": z_ok,
            "z_delta_error_m": z_error,
            "max_allowed_z_drift_m": 1.0,
        },
        "lingtu_inspection": {
            "enabled": drive_source == "nav_cmd_vel",
            "verified": verified,
            "patrol_state": "SUCCESS" if verified else "PATROLLING",
            "goal_count": 4,
            "successful_navigation_goal_count": successful_goals,
            "global_planner": "pct",
        },
        "outputs": {
            "nav_cmd_vel_nonzero": 64 if drive_source == "nav_cmd_vel" else 0,
        },
        "navigation_diagnostics": {
            "sample_count": 4,
            "last_sample": {
                "nav_cmd": {"fresh": nav_fresh, "linear_norm": 0.25},
                "navigation": {
                    "state": "PATROLLING" if not verified else "SUCCESS",
                    "primary_planner": "pct",
                    "selected_planner": "pct",
                },
                "paths": {"local_path_count": local_paths},
                "latest_runtime_fault": "" if z_ok else "runtime Fast-LIO Z drift",
            },
        },
    }
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_large_loop_diagnosis_classifies_slam_primary_when_truth_nav_passes(
    tmp_path: Path,
) -> None:
    baseline = _write_report(
        tmp_path / "baseline.json",
        z_ok=False,
        z_error=3.3,
        verified=False,
        successful_goals=0,
    )
    truth_nav = _write_report(
        tmp_path / "truth_nav.json",
        nav_data_source="mujoco_ground_truth",
        z_ok=False,
        z_error=3.1,
        verified=True,
        successful_goals=4,
    )
    fixed = _write_report(
        tmp_path / "fixed.json",
        drive_source="fixed",
        z_ok=False,
        z_error=1.4,
        verified=False,
        successful_goals=0,
    )

    summary = large_loop_diagnosis_matrix.evaluate_matrix(
        baseline_fastlio=baseline,
        truth_nav=truth_nav,
        fixed_fastlio=fixed,
    )

    assert summary["ok"] is False
    assert summary["primary_failure"] == "slam_localization"
    assert summary["evidence"]["truth_nav"]["inspection_verified"] is True
    assert summary["evidence"]["baseline_fastlio"]["nav_cmd_fresh"] is True
    assert summary["evidence"]["baseline_fastlio"]["local_path_count"] == 60
    assert "Fast-LIO Z drift" in summary["recommended_next_step"]


def test_large_loop_diagnosis_includes_fastlio_red_diagnostic_report_in_evidence(
    tmp_path: Path,
) -> None:
    baseline = _write_report(
        tmp_path / "baseline.json",
        z_ok=False,
        z_error=3.3,
        verified=False,
        successful_goals=0,
    )
    payload = json.loads(baseline.read_text(encoding="utf-8"))
    payload["fastlio_large_loop_diagnostic_report"] = {
        "segment_consistency": {
            "sample_count": 2,
            "max_z_delta_error_m": 3.3,
        },
        "imu_statistics": {
            "sample_count": 300,
            "mean_dt_s": 0.02,
        },
        "scan_timing_statistics": {
            "profile": "instantaneous",
            "span_s": 0.0,
        },
        "command_trajectory_summary": {
            "sample_count": 40,
            "source": "nav_cmd_vel",
        },
    }
    baseline.write_text(json.dumps(payload), encoding="utf-8")
    truth_nav = _write_report(
        tmp_path / "truth_nav.json",
        nav_data_source="mujoco_ground_truth",
        z_ok=False,
        z_error=3.1,
        verified=True,
        successful_goals=4,
    )
    fixed = _write_report(
        tmp_path / "fixed.json",
        drive_source="fixed",
        z_ok=False,
        z_error=1.4,
        verified=False,
        successful_goals=0,
    )

    summary = large_loop_diagnosis_matrix.evaluate_matrix(
        baseline_fastlio=baseline,
        truth_nav=truth_nav,
        fixed_fastlio=fixed,
    )

    diag = summary["evidence"]["baseline_fastlio"]["diagnostic_report"]
    assert summary["primary_failure"] == "slam_localization"
    assert diag["segment_consistency"]["max_z_delta_error_m"] == 3.3
    assert diag["imu_statistics"]["mean_dt_s"] == 0.02
    assert diag["scan_timing_statistics"]["profile"] == "instantaneous"
    assert diag["command_trajectory_summary"]["sample_count"] == 40


def test_large_loop_diagnosis_classifies_planning_when_truth_nav_also_fails(
    tmp_path: Path,
) -> None:
    baseline = _write_report(
        tmp_path / "baseline.json",
        z_ok=False,
        z_error=2.0,
        verified=False,
        successful_goals=0,
    )
    truth_nav = _write_report(
        tmp_path / "truth_nav.json",
        nav_data_source="mujoco_ground_truth",
        z_ok=True,
        verified=False,
        successful_goals=0,
        local_paths=0,
    )

    summary = large_loop_diagnosis_matrix.evaluate_matrix(
        baseline_fastlio=baseline,
        truth_nav=truth_nav,
        fixed_fastlio=None,
    )

    assert summary["ok"] is False
    assert summary["primary_failure"] == "planning_tracking_or_frame"
    assert "truth-nav" in summary["recommended_next_step"]


def test_large_loop_diagnosis_reports_all_blocking_failures(tmp_path: Path) -> None:
    baseline = _write_report(
        tmp_path / "baseline.json",
        z_ok=False,
        z_error=3.3,
        verified=False,
        successful_goals=0,
    )
    truth_nav = _write_report(
        tmp_path / "truth_nav.json",
        nav_data_source="mujoco_ground_truth",
        z_ok=True,
        verified=False,
        successful_goals=0,
        nav_fresh=False,
        local_paths=1,
    )
    fixed = _write_report(
        tmp_path / "fixed.json",
        drive_source="fixed",
        z_ok=False,
        z_error=1.2,
        verified=False,
        successful_goals=0,
    )

    summary = large_loop_diagnosis_matrix.evaluate_matrix(
        baseline_fastlio=baseline,
        truth_nav=truth_nav,
        fixed_fastlio=fixed,
    )

    assert summary["ok"] is False
    assert summary["primary_failure"] == "planning_tracking_or_frame"
    assert summary["blocking_failures"] == [
        "planning_tracking_or_frame",
        "slam_localization",
    ]


def test_large_loop_diagnosis_reads_nested_commanded_drive_source(
    tmp_path: Path,
) -> None:
    report = _write_report(
        tmp_path / "fixed.json",
        drive_source="fixed",
        z_ok=False,
        z_error=1.2,
        verified=False,
        successful_goals=0,
    )
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload.pop("drive_source")
    payload["commanded_sim_velocity"] = {"source": "fixed"}
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = large_loop_diagnosis_matrix.evaluate_matrix(
        baseline_fastlio=report,
        truth_nav=None,
        fixed_fastlio=report,
    )

    assert summary["evidence"]["baseline_fastlio"]["drive_source"] == "fixed"


def test_large_loop_diagnosis_does_not_blame_planning_when_truth_nav_is_patrolling(
    tmp_path: Path,
) -> None:
    baseline = _write_report(
        tmp_path / "baseline.json",
        z_ok=False,
        z_error=3.3,
        verified=False,
        successful_goals=0,
    )
    truth_nav = _write_report(
        tmp_path / "truth_nav.json",
        nav_data_source="mujoco_ground_truth",
        z_ok=False,
        z_error=6.7,
        verified=False,
        successful_goals=0,
        nav_fresh=True,
        local_paths=111,
    )
    payload = json.loads(truth_nav.read_text(encoding="utf-8"))
    payload["runtime_faults"] = [
        "runtime Fast-LIO motion divergence (fastlio2=12.157m, sim=0.168m, allowed=3.000m)"
    ]
    payload["lingtu_inspection"]["global_path_count"] = 1
    payload["lingtu_inspection"]["global_path_points_max"] = 15
    payload["outputs"]["nav_cmd_vel_nonzero"] = 39
    payload["navigation_diagnostics"]["last_sample"]["navigation"]["state"] = "PATROLLING"
    payload["navigation_diagnostics"]["last_sample"]["nav_cmd"]["linear_norm"] = 0.25
    payload["navigation_diagnostics"]["last_sample"]["latest_runtime_fault"] = payload[
        "runtime_faults"
    ][0]
    truth_nav.write_text(json.dumps(payload), encoding="utf-8")
    fixed = _write_report(
        tmp_path / "fixed.json",
        drive_source="fixed",
        z_ok=False,
        z_error=1.2,
        verified=False,
        successful_goals=0,
    )

    summary = large_loop_diagnosis_matrix.evaluate_matrix(
        baseline_fastlio=baseline,
        truth_nav=truth_nav,
        fixed_fastlio=fixed,
    )

    assert summary["primary_failure"] == "slam_localization"
    assert summary["blocking_failures"] == ["slam_localization"]
