import pytest

pytestmark = [pytest.mark.sim]

from __future__ import annotations

import json
from pathlib import Path

from sim.scripts import large_loop_closure_gate


def _write_live_loop_report(
    path: Path,
    *,
    loop_error_m: float = 0.32,
    sim_path_length_m: float = 24.0,
    planner: str = "pct",
    world: str | None = None,
    tomogram: str | None = None,
    first_sim_xy: tuple[float, float] = (0.0, 0.0),
    map_frame_origin_world_xy: tuple[float, float] = (0.0, 0.0),
    scan_time_profile: str = "physical_rolling",
) -> Path:
    artifact_dir = path.parent / "large_terrain_odom"
    world = world or str(artifact_dir / "large_terrain_scene.xml")
    tomogram = tomogram or str(artifact_dir / "tomogram.pickle")
    artifact_dir.mkdir(parents=True, exist_ok=True)
    (artifact_dir / "metadata.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "map_frame": (
                    "world"
                    if map_frame_origin_world_xy == (0.0, 0.0)
                    else "start_odom"
                ),
                "map_frame_origin_world_xy": list(map_frame_origin_world_xy),
                "start": [first_sim_xy[0], first_sim_xy[1], 0.0],
            }
        ),
        encoding="utf-8",
    )
    payload = {
        "ok": True,
        "world": world,
        "scan_time_profile": scan_time_profile,
        "lidar_source": {
            "scan_time_profile": scan_time_profile,
            "scan_time_model_contract": (
                "physical_subscans_with_actual_sim_time_offsets"
                if scan_time_profile == "physical_rolling"
                else "synthetic_snapshot_offsets"
            ),
        },
        "start_position": [first_sim_xy[0], first_sim_xy[1], 0.0],
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "live_mujoco_lidar_verified": True,
        "live_mujoco_imu_verified": True,
        "slam_algorithm_output_verified": True,
        "canonical_nav_outputs_verified": True,
        "bridge_verified": True,
        "outputs": {
            "nav_odometry": 120,
            "nav_registered_cloud": 110,
            "nav_map_cloud": 110,
            "nav_cmd_vel_nonzero": 420,
        },
        "first_sim_xyz": [first_sim_xy[0], first_sim_xy[1], 0.0],
        "last_sim_xyz": [first_sim_xy[0] + loop_error_m, first_sim_xy[1], 0.0],
        "sim_path_length_m": sim_path_length_m,
        "first_odom_xyz": [0.0, 0.0, 0.0],
        "last_odom_xyz": [loop_error_m + 0.08, 0.0, 0.0],
        "fastlio2_path_length_m": sim_path_length_m - 0.25,
        "first_sim_yaw_rad": 0.0,
        "last_sim_yaw_rad": 0.18,
        "first_odom_yaw_rad": 0.0,
        "last_odom_yaw_rad": 0.20,
        "fastlio2_motion_consistency": {
            "checked": True,
            "ok": True,
            "fastlio2_path_length_m": sim_path_length_m - 0.25,
            "sim_path_length_m": sim_path_length_m,
        },
        "fastlio2_z_consistency": {"checked": True, "ok": True, "z_delta_error_m": 0.03},
        "fastlio2_yaw_consistency": {"checked": True, "ok": True, "yaw_delta_error_rad": 0.02},
        "lingtu_inspection": {
            "enabled": True,
            "verified": True,
            "started_after_slam_ready": True,
            "goal_count": 4,
            "successful_navigation_goal_count": 4,
            "failed_navigation_goal_count": 0,
            "min_required_checkpoints": 4,
            "global_path_count": 4,
            "global_path_points_max": 40,
            "local_path_count": 80,
            "local_path_points_max": 24,
            "patrol_state": "SUCCESS",
            "global_planner": planner,
            "tomogram": tomogram,
            "replan_on_costmap_update": False,
            "goals": [
                [6.0, 0.0, 0.0],
                [6.0, 6.0, 0.0],
                [0.0, 6.0, 0.0],
                [0.1, 0.0, 0.0],
            ],
        },
        "navigation_chain": {
            "planner_fallback_used": False,
            "planner_repair_used": False,
            "direct_goal_fallback": {"used": False},
        },
        "navigation_diagnostics": {
            "sample_period_s": 2.0,
            "sample_count": 2,
            "samples": [
                {
                    "sim_time_s": 2.0,
                    "navigation": {"state": "RUNNING", "patrol_index": 1},
                    "paths": {"local_path_count": 8},
                    "nav_cmd": {"fresh": True},
                    "fastlio2_z_delta_error_m": 0.02,
                },
                {
                    "sim_time_s": 4.0,
                    "navigation": {"state": "SUCCESS", "patrol_index": 4},
                    "paths": {"local_path_count": 80},
                    "nav_cmd": {"fresh": True},
                    "fastlio2_z_delta_error_m": 0.03,
                },
            ],
        },
        "video_path": str(path.with_suffix(".mp4")),
        "video_frame_count": 120,
        "video_sample_count": 120,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_large_loop_closure_gate_accepts_pct_fastlio_local_loop(tmp_path: Path):
    report = _write_live_loop_report(tmp_path / "loop_runtime.json")

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is True
    assert summary["passed_case_count"] == 1
    case = summary["best_case"]
    assert case["global_planner"] == "pct"
    assert case["sim_loop_closure_error_m"] == 0.32
    assert case["sim_path_length_m"] == 24.0
    assert case["nav_cmd_vel_nonzero"] == 420
    assert case["fastlio2_z_delta_error_m"] == 0.03
    assert case["navigation_diagnostics"]["sample_count"] == 2
    assert case["navigation_diagnostics"]["last_sample"]["navigation"]["patrol_index"] == 4


def test_large_loop_closure_gate_preserves_runtime_samples_tail(tmp_path: Path):
    report = _write_live_loop_report(tmp_path / "loop_runtime_samples_tail.json")
    payload = json.loads(report.read_text(encoding="utf-8"))
    samples = payload["navigation_diagnostics"].pop("samples")
    payload["navigation_diagnostics"]["samples_tail"] = samples
    payload["navigation_diagnostics"]["last_sample"] = samples[-1]
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is True
    diagnostics = summary["best_case"]["navigation_diagnostics"]
    assert diagnostics["sample_count"] == 2
    assert len(diagnostics["samples_tail"]) == 2
    assert diagnostics["last_sample"]["navigation"]["patrol_index"] == 4


def test_large_loop_closure_gate_rejects_short_or_non_pct_loop(tmp_path: Path):
    report = _write_live_loop_report(
        tmp_path / "weak_loop_runtime.json",
        loop_error_m=1.4,
        sim_path_length_m=5.0,
        planner="astar",
    )

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is False
    blockers = "\n".join(summary["cases"][0]["blockers"])
    assert "lingtu_inspection.global_planner is not pct" in blockers
    assert "sim_path_length_m < 20" in blockers
    assert "sim_loop_closure_error_m > 0.75" in blockers


def test_large_loop_closure_gate_rejects_non_physical_rolling_input(tmp_path: Path):
    report = _write_live_loop_report(
        tmp_path / "synthetic_loop_runtime.json",
        scan_time_profile="synthetic_rolling",
    )

    summary = large_loop_closure_gate.evaluate_reports(
        [report],
        required_scan_time_profile="physical_rolling",
    )

    assert summary["ok"] is False
    case = summary["cases"][0]
    assert case["scan_time_profile"] == "synthetic_rolling"
    assert "scan_time_profile is not physical_rolling" in case["blockers"]


def test_large_loop_closure_gate_summarizes_slam_blocker_when_planning_is_alive(
    tmp_path: Path,
):
    report = _write_live_loop_report(tmp_path / "fastlio_z_red_loop.json")
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["ok"] = False
    payload["fastlio2_z_consistency"] = {
        "checked": True,
        "ok": False,
        "z_delta_error_m": 10.3896,
        "max_allowed_z_drift_m": 1.0,
    }
    payload["runtime_fault_events"] = [
        {
            "kind": "z",
            "confirmed": True,
            "time_alignment": {
                "time_aligned": True,
                "first_dt_s": 0.0,
                "last_dt_s": 0.0,
            },
        }
    ]
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is False
    defect = summary["minimal_red_defect"]
    assert defect["blocking_subsystem"] == "slam_localization"
    assert defect["fastlio2_z_delta_error_m"] == 10.3896
    assert defect["global_planner"] == "pct"
    assert defect["local_path_count"] == 80
    assert defect["nav_cmd_vel_nonzero"] == 420
    assert defect["time_alignment"]["time_aligned"] is True


def test_large_loop_closure_gate_preserves_fastlio_red_diagnostic_report(
    tmp_path: Path,
):
    report = _write_live_loop_report(tmp_path / "fastlio_red_diagnostics.json")
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["ok"] = False
    payload["fastlio2_z_consistency"] = {
        "checked": True,
        "ok": False,
        "z_delta_error_m": 10.3896,
        "max_allowed_z_drift_m": 1.0,
    }
    payload["fastlio_large_loop_diagnostic_report"] = {
        "segment_consistency": {
            "sample_count": 2,
            "max_z_delta_error_m": 10.3896,
            "worst_segment": {"segment": "goal_3_to_start"},
        },
        "imu_statistics": {
            "sample_count": 500,
            "mean_dt_s": 0.02,
            "max_acc_norm": 10.4,
        },
        "scan_timing_statistics": {
            "profile": "synthetic_rolling",
            "span_s": 0.075,
        },
        "command_trajectory_summary": {
            "sample_count": 80,
            "source": "nav_cmd_vel",
        },
    }
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = large_loop_closure_gate.evaluate_reports([report])
    defect = summary["minimal_red_defect"]

    assert summary["ok"] is False
    assert defect["blocking_subsystem"] == "slam_localization"
    assert defect["fastlio2_z_delta_error_m"] == 10.3896
    assert defect["diagnostic_report"]["segment_consistency"]["worst_segment"]["segment"] == "goal_3_to_start"
    assert defect["diagnostic_report"]["imu_statistics"]["sample_count"] == 500
    assert defect["diagnostic_report"]["scan_timing_statistics"]["profile"] == "synthetic_rolling"
    assert defect["diagnostic_report"]["command_trajectory_summary"]["source"] == "nav_cmd_vel"
    assert summary["thresholds"]["max_loop_closure_error_m"] == 0.75
    assert summary["thresholds"]["max_fastlio_loop_closure_error_m"] == 1.0
    assert summary["thresholds"]["max_loop_yaw_error_rad"] == 0.5


def test_large_loop_closure_gate_classifies_gate_exception_as_validation_harness(
    tmp_path: Path,
):
    report = tmp_path / "gate_exception_report.json"
    fault = "gate_exception: RCLError: Failed to publish: publisher's context is invalid"
    report.write_text(
        json.dumps(
            {
                "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
                "ok": False,
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "remaining_gaps": [fault],
                "runtime_faults": [
                    "RCLError: Failed to publish: publisher's context is invalid"
                ],
            }
        ),
        encoding="utf-8",
    )

    summary = large_loop_closure_gate.evaluate_reports([report])

    defect = summary["minimal_red_defect"]
    assert summary["ok"] is False
    assert defect["blocking_subsystem"] == "validation_harness"
    assert defect["remaining_gaps"] == [fault]
    assert "publisher's context is invalid" in defect["runtime_faults"][0]


def test_large_loop_closure_gate_classifies_timeout_with_live_progress_as_tracking(
    tmp_path: Path,
):
    report = _write_live_loop_report(tmp_path / "timeout_with_progress.json")
    payload = json.loads(report.read_text(encoding="utf-8"))
    payload["ok"] = False
    payload["sim_path_length_m"] = 6.855
    payload["fastlio2_path_length_m"] = 6.9
    payload["gate_wall_timeout"] = {
        "triggered": True,
        "limit_s": 900.0,
        "elapsed_wall_s": 900.033,
    }
    payload["remaining_gaps"] = [
        "gate wall timeout after 900.0s (limit=900.0s)",
        "inspection successful checkpoint count 1 < required 4",
    ]
    payload["lingtu_inspection"]["verified"] = False
    payload["lingtu_inspection"]["patrol_state"] = "PATROLLING"
    payload["lingtu_inspection"]["successful_navigation_goal_count"] = 1
    payload["lingtu_inspection"]["local_path_count"] = 1131
    payload["outputs"]["nav_cmd_vel_nonzero"] = 422
    report.write_text(json.dumps(payload), encoding="utf-8")

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is False
    assert summary["blocking_subsystems"] == ["planning_tracking"]
    defect = summary["minimal_red_defect"]
    assert defect["blocking_subsystem"] == "planning_tracking"
    assert defect["local_path_count"] == 1131
    assert defect["nav_cmd_vel_nonzero"] == 422
    assert defect["gate_wall_timeout"]["triggered"] is True


def test_large_loop_closure_gate_rejects_mixed_world_and_tomogram_sources(tmp_path: Path):
    report = _write_live_loop_report(
        tmp_path / "mixed_source_loop_runtime.json",
        world=str(tmp_path / "sim" / "worlds" / "industrial_park_scene.xml"),
        tomogram=str(tmp_path / "artifacts" / "server_sim_closure" / "large_terrain_odom" / "tomogram.pickle"),
    )

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is False
    case = summary["cases"][0]
    assert case["same_source_artifacts"] is False
    blockers = "\n".join(case["blockers"])
    assert "same-source world/tomogram mismatch" in blockers


def test_large_loop_closure_gate_uses_map_frame_metadata_for_loop_goal_check(
    tmp_path: Path,
):
    report = _write_live_loop_report(
        tmp_path / "start_odom_loop_runtime.json",
        first_sim_xy=(-9.5, -5.6),
        map_frame_origin_world_xy=(-9.5, -5.6),
    )

    summary = large_loop_closure_gate.evaluate_reports([report])

    assert summary["ok"] is True
    case = summary["best_case"]
    assert case["map_frame"] == "start_odom"
    assert case["map_frame_origin_world_xy"] == [-9.5, -5.6]
    assert case["first_sim_map_xy"] == [0.0, 0.0]
    assert case["final_goal_to_start_m"] == 0.1
