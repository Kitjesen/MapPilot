from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest

from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
from sim.diagnostics.dataflow_report import (
    RUNTIME_DATAFLOW_GATES,
    build_runtime_dataflow_from_summary,
    summarize_runtime_report,
)
from sim.scripts import dimos_gap_report


def _write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _fresh_host_preflight_contract() -> dict:
    generated_at = time.time()
    return {
        "report_contract_checked": True,
        "generated_at": generated_at,
        "report_freshness": {
            "checked": True,
            "source": "test_fixture",
            "generated_at": generated_at,
            "fresh": True,
            "stale": False,
            "blockers": [],
        },
    }


def _same_source_map_artifacts(label: str = "runtime") -> dict:
    map_sha = f"{label}-map-sha256"
    return {
        "ok": True,
        "source_contract": {
            "same_source_pcd": True,
            "same_source_tomogram": True,
        },
        "assets": {
            "map_pcd": {
                "sha256": map_sha,
                "point_count": 128,
            },
            "tomogram": {
                "sha256": f"{label}-tomogram-sha256",
                "source_map_sha256": map_sha,
            },
        },
    }


def _same_source_hash_identity(label: str = "runtime") -> dict:
    map_sha = f"{label}-map-sha256"
    tomogram_sha = f"{label}-tomogram-sha256"
    return {
        "ok": True,
        "checks": {
            "relocalization_metadata_loads": True,
            "source_map_sha256_matches_relocalization": True,
            "source_tomogram_sha256_matches_relocalization": True,
            "source_tomogram_file_sha256_matches_metadata": True,
            "relocalization_map_file_sha256_matches_metadata": True,
            "relocalization_tomogram_file_sha256_matches_metadata": True,
            "relocalization_tomogram_source_map_sha256_matches_map": True,
            "source_tomogram_source_map_sha256_matches_map": True,
        },
        "hashes": {
            "relocalization_map_sha256": map_sha,
            "relocalization_tomogram_sha256": tomogram_sha,
            "source_map_sha256": map_sha,
            "source_tomogram_sha256": tomogram_sha,
            "actual_map_sha256": map_sha,
            "actual_tomogram_sha256": tomogram_sha,
        },
    }


def _native_pct_dataflow_report(
    *,
    with_same_source: bool = True,
) -> dict:
    report = {
        "schema_version": "lingtu.native_pct_mujoco_gate.v1",
        "ok": True,
        "global_planner_source": "source_report/pct_tomogram",
        "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
        "pct_planner_runtime_ok": True,
        "pct_path_count": 8,
        "pct_optimizer_enabled": True,
        "pct_optimizer_attempted": True,
        "pct_optimizer_accepted": False,
        "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
        "pct_optimizer_blocked_sample_count": 3,
        "pct_planner_path_mode": "astar_raw_path",
        "selected_planner": "pct",
        "fallback_used": False,
        "path_count": 2,
        "max_path_poses": 5,
        "cmd_count_nonzero": 2,
        "cmd_samples": [{"linear_x": 0.1}],
        "moved_m": 0.4,
        "reached_goal": True,
    }
    if with_same_source:
        report["deliverable_contract"] = {
            "checks": {"same_source_map_artifact": True},
        }
        report["map_artifacts"] = _same_source_map_artifacts("native_pct")
    return report


def _policy_nav_dataflow_report() -> dict:
    return {
        "schema_version": "lingtu.policy_nav_smoke.v1",
        "passed": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "checks": [
            {
                "mode": "direct_policy",
                "passed": True,
                "policy_loaded": True,
                "policy_backend": "onnx",
                "policy_path": "/tmp/policy.onnx",
                "cmd_vel_sent_to_hardware": False,
            },
            {
                "mode": "full_stack_policy_nav",
                "passed": True,
                "drive_mode": "policy",
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "policy_loaded": True,
                "policy_backend": "onnx",
                "policy_path": "/tmp/policy.onnx",
                "finite": True,
                "global_planner_backend_requested": "octoplanner3d",
                "global_planner_backend_status": {
                    "configured_backend": "octoplanner3d",
                    "backend": "octoplanner3d",
                    "degraded": False,
                },
                "local_planner_backend_requested": "nanobind",
                "local_planner_backend_actual": "nanobind",
                "path_follower_backend_requested": "nav_kernel",
                "path_follower_backend_actual": "nav_kernel",
                "seen": {
                    "costmap": 1,
                    "waypoints": 2,
                    "local_path": 4,
                    "path_follower_cmd": 5,
                    "mux_cmd": 5,
                    "direct_fallback": 0,
                },
                "global_path": {"count": 5, "frame_id": "map"},
                "local_path_nonempty_count": 3,
                "last_nonempty_local_path": {"count": 4, "frame_id": "map"},
                "path_follower_cmd_stats": {"nonzero_count": 5},
                "mux_cmd_stats": {"nonzero_count": 5},
                "moved_m": 0.4,
                "success_seen": True,
                "dist_at_success_m": 0.04,
                "dist_to_goal_m": 0.04,
                "nav_state": "SUCCESS",
                "contacts": {
                    "foot_contact_sample_count": 20,
                    "unique_feet_count": 4,
                    "non_foot_ground_contacts": 0,
                },
                "costmap_readiness": {
                    "planner_has_map": True,
                    "source": "live_stack_costmap",
                },
            },
        ],
    }


def _pct_saved_map_dataflow_report() -> dict:
    return {
        "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
        "ok": True,
        "relocalization": {
            "ok": True,
            "latest_health_state": "ok",
            "saved_map_cloud_points_latest": 128,
        },
        "plan_preview": {
            "ok": True,
            "selected_planner": "pct",
            "fallback_reason": "",
            "path_count": 3,
            "path": [[0, 0, 0], [1, 0, 0], [2, 0, 0]],
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
        },
        "source_report": "artifacts/server_sim_closure/large_terrain/report.json",
        "tomogram": "artifacts/server_sim_closure/same_source_map/tomogram.pickle",
        "map_metadata": "artifacts/server_sim_closure/same_source_map/metadata.json",
        "deliverable_contract": {
            "checks": {"same_source_map_artifact": True},
        },
        "map_artifacts": _same_source_map_artifacts("pct_saved_map"),
        "same_source_hash_identity": _same_source_hash_identity("pct_saved_map"),
        "native_gate": {
            "global_planner_source": "source_report/pct_tomogram",
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            "pct_planner_runtime_ok": True,
            "pct_path_count": 8,
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 3,
            "pct_planner_path_mode": "astar_raw_path",
            "selected_planner": "pct",
            "fallback_used": False,
            "path_count": 2,
            "max_path_poses": 5,
            "cmd_count_nonzero": 2,
            "cmd_samples": [{"linear_x": 0.1}],
            "moved_m": 0.4,
            "reached_goal": True,
        },
    }


def _saved_map_relocalize_dataflow_report() -> dict:
    map_sha = "saved-map-relocalize-map-sha256"
    return {
        "schema_version": "lingtu.saved_map_relocalize_runtime.v1",
        "ok": True,
        "validation_level": "runtime_relocalization",
        "runtime_stage": "saved_map_relocalization",
        "map_dependency": "saved_map_required",
        "requires_saved_map": True,
        "requires_live_slam": True,
        "runtime_relocalization_executed": True,
        "runtime_relocalization_validated": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "map_pcd": "artifacts/server_sim_closure/same_source_map/map.pcd",
        "map_metadata_contract": {
            "ok": True,
            "path": "artifacts/server_sim_closure/same_source_map/metadata.json",
            "schema_version": "lingtu.same_source_map_artifacts.v1",
            "world": "artifacts/server_sim_closure/same_source_map/world.xml",
            "scan_time_profile": "physical_rolling",
            "artifacts": {
                "map_pcd": {
                    "path": "artifacts/server_sim_closure/same_source_map/map.pcd",
                    "sha256": map_sha,
                    "actual_sha256": map_sha,
                    "point_count": 2048,
                }
            },
            "checks": {
                "map_pcd_present": True,
                "metadata_file_exists": True,
                "metadata_json_object": True,
                "schema_version_known": True,
                "world_present": True,
                "map_pcd_path_present": True,
                "map_pcd_path_matches": True,
                "map_pcd_sha256_present": True,
                "map_pcd_sha256_matches_file": True,
                "map_pcd_point_count_positive": True,
                "scan_time_profile_valid_or_absent": True,
            },
            "blockers": [],
        },
        "service": {
            "available": True,
            "success": True,
            "message": "relocalize success",
        },
        "live_feed": {
            "ok": True,
            "outputs": {
                "fastlio2_odometry": 10,
                "fastlio2_cloud_registered": 10,
                "fastlio2_cloud_map": 10,
            },
            "fastlio2_z_consistency": {"ok": True, "z_delta_error_m": 0.02},
        },
        "localizer": {
            "tracking_health_samples": 5,
            "latest_health_state": "LOCKED",
            "saved_map_cloud_samples": 3,
            "saved_map_cloud_points_latest": 2048,
            "map_to_odom_tf_samples": 4,
            "map_to_odom_xy_m": 0.12,
            "map_to_odom_z_abs_m": 0.05,
        },
        "thresholds": {
            "min_saved_map_points": 1000,
            "min_tracking_health_samples": 3,
            "max_map_odom_xy_m": 5.0,
            "max_map_odom_z_abs_m": 2.0,
        },
    }


def _dynamic_obstacle_dataflow_report() -> dict:
    phases = [
        {
            "name": "clear_initial",
            "path_count": 101,
            "path_frame_id": "map",
            "avoidance_side": "straight",
            "min_obstacle_clearance_m": None,
        },
        {
            "name": "obstacle_left",
            "path_count": 101,
            "path_frame_id": "map",
            "avoidance_side": "right",
            "min_obstacle_clearance_m": 0.31,
        },
        {
            "name": "obstacle_right",
            "path_count": 101,
            "path_frame_id": "map",
            "avoidance_side": "left",
            "min_obstacle_clearance_m": 0.33,
        },
        {
            "name": "obstacle_center",
            "path_count": 101,
            "path_frame_id": "map",
            "avoidance_side": "right",
            "min_obstacle_clearance_m": 0.29,
        },
        {
            "name": "clear_recovered",
            "path_count": 101,
            "path_frame_id": "map",
            "avoidance_side": "straight",
            "min_obstacle_clearance_m": None,
        },
    ]
    return {
        "schema_version": "lingtu.dynamic_obstacle_local_planner.v1",
        "ok": True,
        "execution_mode": "runtime_gate",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "backend_requested": "nanobind",
        "backend_actual": "nanobind",
        "native_backend_used": True,
        "algorithm_backends": {
            "local_planner": {
                "requested": "nanobind",
                "configured_backend": "nanobind",
                "backend_actual": "nanobind",
                "degraded": False,
                "degraded_reason": "",
                "native_backend_used": True,
                "exercised_by": "dynamic_obstacle",
            },
            "path_follower": {
                "status": "not_exercised",
                "exercised_by": "not_exercised",
            },
        },
        "dynamic_replan_verified": True,
        "obstacle_response_verified": True,
        "clear_path_recovery_verified": True,
        "min_clearance_m": 0.29,
        "phases": phases,
        "frames": {
            "odometry": "map",
            "waypoint": "map",
            "added_obstacles": "map",
            "local_path": "map",
            "cmd_vel": "not_published",
        },
        "errors": [],
    }


def _gazebo_runtime_dataflow_report() -> dict:
    return {
        "schema_version": "lingtu.gazebo_runtime_smoke.v1",
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "samples": 4,
        "odometry_frame_id": "odom",
        "odometry_child_frame_id": "body",
        "topic_samples": {
            "/slam/map_cloud": 3,
            "/slam/registered_cloud": 3,
        },
        "topic_frames": {
            "/slam/map_cloud": "odom",
            "/slam/registered_cloud": "body",
        },
        "point_counts": {
            "/slam/map_cloud": 1024,
            "/slam/registered_cloud": 1024,
        },
        "nav_loop": {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "goal_published": True,
            "odometry_seen": True,
            "global_path_seen": True,
            "local_path_seen": True,
            "cmd_vel_seen": True,
            "cmd_vel_nonzero": True,
            "odom_delta_m": 0.078,
            "samples": {
                "/nav/global_path": 2,
                "/nav/local_path": 12,
                "/nav/cmd_vel": 18,
                "/slam/odometry": 16,
            },
            "publisher_contract": {
                "ok": True,
                "topics": {
                    "/nav/global_path": {
                        "ok": True,
                        "publishers": ["/lingtu_gazebo_line_global_planner"],
                        "disallowed_node_names": [],
                    },
                    "/nav/local_path": {
                        "ok": True,
                        "publishers": ["/localPlanner"],
                        "disallowed_node_names": [],
                    },
                    "/nav/cmd_vel": {
                        "ok": True,
                        "publishers": ["/pathFollower"],
                        "disallowed_node_names": [],
                    },
                    "/slam/odometry": {
                        "ok": True,
                        "publishers": ["/lingtu_gazebo_runtime_adapter"],
                        "disallowed_node_names": [],
                    },
                },
                "errors": [],
            },
        },
        "frontier_exploration": {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "frontier_started": True,
            "frontier_goal_seen": True,
            "frontier_goal_published": True,
            "odometry_seen": True,
            "map_cloud_seen": True,
            "terrain_map_seen": True,
            "terrain_map_ext_seen": True,
            "cumulative_map_cloud_seen": True,
            "global_path_seen": True,
            "local_path_seen": True,
            "cmd_vel_seen": True,
            "cmd_vel_nonzero": True,
            "odom_delta_m": 0.082,
            "odom_delta_x_m": 0.08,
            "known_cells_delta": 39,
            "explored_area_delta_m2": 0.39,
            "frontier_goal": [1.8, 0.0, 0.0],
            "frontier_count_max": 3,
            "trajectory_quality": {
                "ok": True,
                "room_violation_count": 0,
                "max_out_of_room_m": 0.0,
                "min_obstacle_clearance_m": 0.42,
                "local_path_occupied_overlap_count": 0,
            },
            "topic_sync": {
                "ok": True,
                "max_cloud_odom_skew_ms": 0.0,
            },
            "frontier_no_gain_stall": {
                "checked": True,
                "ok": True,
                "mode": "post_pass_observation",
                "stop_reason": "post_pass_observation_elapsed",
                "required_observation_s": 5.0,
                "observed_s": 5.1,
            },
            "cumulative_map_cloud": {
                "samples": 9,
                "frame_ids": ["odom"],
                "unique_voxels_delta": 550,
                "retention_min": 0.85,
            },
            "registered_cloud": {
                "samples": 9,
                "map_vs_registered_voxel_ratio": 2.8333,
            },
            "static_obstacles": {
                "column": {
                    "samples": 4,
                    "centroid_drift_max_m": 0.0,
                }
            },
            "samples": {
                "/nav/goal_pose": 1,
                "/nav/global_path": 2,
                "/nav/local_path": 12,
                "/nav/cmd_vel": 18,
                "/slam/odometry": 16,
                "/slam/map_cloud": 3,
                "/nav/terrain_map": 3,
                "/nav/terrain_map_ext": 9,
                "/slam/cumulative_map_cloud": 9,
                "/slam/registered_cloud": 9,
            },
        },
        "tare_exploration": {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend": "tare",
            "source_contract_ok": True,
            "runtime_required": False,
            "runtime_available": False,
            "gazebo_runtime_verified": False,
        },
    }


def _large_terrain_dataflow_report(*, with_same_source: bool = True) -> dict:
    report = {
        "schema_version": "lingtu.large_terrain_nav_validation.v1",
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "validation_level": "global_planning_assets",
        "selection_policy": "first_route_ok_after_primary",
        "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
        "pct_planner_runtime_ok": True,
        "cases": [
            {
                "route": "terrain_long",
                "ok": True,
                "path_safety": {"ok": True},
                "planning": [
                    {
                        "planner": "pct",
                        "planner_requested": "pct",
                        "selected_planner": "pct",
                        "fallback_reason": "",
                        "feasible": True,
                        "pct_planner_runtime": {
                            "runtime": "rust_process",
                            "ok": True,
                        },
                        "pct_planner_runtime_ok": True,
                        "route_ok": True,
                        "path_safety": {"ok": True},
                        "gate_crossing": {
                            "checked": True,
                            "passed_gate": True,
                            "min_y_at_wall": -0.1,
                            "max_y_at_wall": 0.4,
                        },
                        "metrics": {
                            "route_distance_m": 6.2,
                            "min_required_route_distance_m": 5.8,
                            "route_distance_tolerance_m": 0.05,
                        },
                        "path": [[0, 0, 0], [1, 0, 0], [2, 1, 0]],
                    }
                ],
            }
        ],
    }
    if with_same_source:
        report["deliverable_contract"] = {
            "checks": {"same_source_map_artifact": True},
        }
        report["map_artifacts"] = _same_source_map_artifacts("large_terrain")
    return report


def _summary(*, failed: list[str]) -> dict:
    gates = {}
    gate_categories = {}
    next_actions = []
    for gate in DIMOS_BENCHMARK_REQUIRED_GATES:
        is_failed = gate in failed
        gates[gate] = {
            "name": gate,
            "ok": not is_failed,
            "status": "failed" if is_failed else "passed",
            "path": f"artifacts/server_sim_closure/{gate}/report.json",
            "blockers": [f"{gate} blocker"] if is_failed else [],
        }
        if is_failed:
            category = (
                "dynamic_obstacle"
                if gate == "moving_obstacle_sweep"
                else "slam_localization"
                if gate == "large_loop_closure"
                else "artifact_contract"
            )
            gate_categories[gate] = [category]
            next_actions.append(
                {
                    "gate": gate,
                    "category": category,
                    "command": f"run {gate}",
                    "expected_report_path": f"artifacts/server_sim_closure/{gate}/report.json",
                    "blockers": [f"{gate} blocker"],
                }
            )

    return {
        "schema_version": "lingtu.server_sim_closure.summary.v1",
        "ok": not failed,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "missing_or_failed": failed,
        "gates": gates,
        "algorithm_validation": {
            "claim_allowed": not failed,
            "flow_ok": not failed,
            "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "gate_categories": gate_categories,
            "next_actions": next_actions,
            "validation_flow": [
                {
                    "id": "local_dynamic_avoidance",
                    "required_gates": [
                        "dynamic_obstacle_local_planner",
                        "moving_obstacle_sweep",
                    ],
                },
                {
                    "id": "long_range_loop_closure",
                    "required_gates": ["large_loop_closure"],
                },
            ],
            "claim_boundary": {
                "simulation_only": True,
                "field_readiness": False,
                "live_costmap_role": "local_planning_and_safety_only",
            },
        },
    }


def test_dimos_gap_report_marks_current_stress_gates_as_p0(tmp_path: Path):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["moving_obstacle_sweep", "large_loop_closure"]),
    )

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    assert report["schema_version"] == "lingtu.dimos_gap_report.v1"
    assert report["lingtu_readiness"]["claim_allowed"] is False
    assert report["gap_counts"]["passed"] == len(DIMOS_BENCHMARK_REQUIRED_GATES) - 2
    assert report["gap_counts"]["p0"] == 2
    rows = {row["gate"]: row for row in report["gap_matrix"]}
    assert rows["moving_obstacle_sweep"]["priority"] == "p0"
    assert rows["moving_obstacle_sweep"]["primary_category"] == "dynamic_obstacle"
    assert rows["moving_obstacle_sweep"]["gate_command"] == "run moving_obstacle_sweep"
    assert rows["large_loop_closure"]["validation_stages"] == ["long_range_loop_closure"]


def test_dimos_gap_report_keeps_green_summary_unclaimable_without_dataflow(
    tmp_path: Path,
):
    summary_path = _write_json(tmp_path / "summary.json", _summary(failed=[]))

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    assert report["lingtu_readiness"]["summary_ok"] is True
    assert report["lingtu_readiness"]["claim_allowed"] is True
    assert report["lingtu_readiness"]["runtime_dataflow_checked"] is False
    assert report["lingtu_readiness"]["ok"] is False
    assert "runtime dataflow is checked" in report["lingtu_readiness"]["stop_condition"]
    assert report["gap_counts"] == {
        "required": len(DIMOS_BENCHMARK_REQUIRED_GATES),
        "passed": len(DIMOS_BENCHMARK_REQUIRED_GATES),
        "failed": 0,
        "p0": 0,
        "p1": 0,
        "p2": 0,
        "p3": 0,
    }
    assert report["next_steps"] == []


def test_dimos_gap_report_rejects_stale_summary_file_even_with_green_dataflow(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(tmp_path / "summary.json", _summary(failed=[]))
    old = time.time() - 120
    os.utime(summary_path, (old, old))

    def fake_runtime_dataflow_from_summary(summary):
        return {
            gate: {
                "checked": True,
                "ok": True,
                "schema_detected": "test",
                "primary_blocker": "",
                "flow": [],
            }
            for gate in RUNTIME_DATAFLOW_GATES
        }

    monkeypatch.setattr(
        dimos_gap_report,
        "_runtime_dataflow_from_summary",
        fake_runtime_dataflow_from_summary,
    )

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        max_report_age_s=1,
        include_dataflow=True,
    )

    assert report["summary_freshness"]["checked"] is True
    assert report["summary_freshness"]["fresh"] is False
    assert report["lingtu_readiness"]["summary_ok"] is True
    assert report["lingtu_readiness"]["runtime_dataflow_checked"] is True
    assert report["lingtu_readiness"]["runtime_dataflow_complete"] is True
    assert report["lingtu_readiness"]["ok"] is False
    assert report["gap_matrix"][0]["status"] == "stale"
    assert "summary is regenerated" in report["lingtu_readiness"]["stop_condition"]


def test_dimos_gap_report_checks_current_summary_payload_freshness(
    monkeypatch,
):
    def fake_summary_from_reports(max_report_age_s):
        return _summary(failed=[]), "current_reports"

    monkeypatch.setattr(
        dimos_gap_report,
        "_summary_from_reports",
        fake_summary_from_reports,
    )

    report = dimos_gap_report.build_gap_report(max_report_age_s=1)

    assert report["summary_freshness"]["checked"] is True
    assert report["summary_freshness"]["fresh"] is False
    assert "generated_at missing" in report["summary_freshness"]["blocker"]
    assert report["lingtu_readiness"]["ok"] is False


def test_dimos_gap_report_rejects_stale_host_preflight_file_even_with_green_dataflow(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(tmp_path / "summary.json", _summary(failed=[]))
    old = time.time() - 120
    preflight_path = _write_json(
        tmp_path / "host_preflight.json",
        {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "generated_at": old,
            "ok": True,
            "current_host": {"platform_system": "Linux", "python_tag": "py310"},
            "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "runnable_gates": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "blocked_gates": [],
            "gates": {
                gate: {"ok": True, "status": "runnable", "checks": {}} for gate in DIMOS_BENCHMARK_REQUIRED_GATES
            },
        },
    )
    os.utime(preflight_path, (old, old))

    def fake_runtime_dataflow_from_summary(summary):
        return {
            gate: {
                "checked": True,
                "ok": True,
                "schema_detected": "test",
                "primary_blocker": "",
                "flow": [],
            }
            for gate in RUNTIME_DATAFLOW_GATES
        }

    monkeypatch.setattr(
        dimos_gap_report,
        "_runtime_dataflow_from_summary",
        fake_runtime_dataflow_from_summary,
    )

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        max_report_age_s=1,
        host_preflight_report=preflight_path,
        include_dataflow=True,
    )

    assert report["summary_freshness"]["fresh"] is True
    assert report["host_preflight"]["checked"] is True
    assert report["host_preflight"]["ok"] is False
    assert report["host_setup_plan"]["ok"] is False
    assert report["host_setup_plan"]["failed_checks"][0]["check"] == ("host_preflight_report_freshness")
    assert report["lingtu_readiness"]["host_preflight_ok"] is False
    assert report["lingtu_readiness"]["ok"] is False
    assert "host preflight passes" in report["lingtu_readiness"]["stop_condition"]


def test_dimos_gap_report_rejects_green_host_preflight_without_contract(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(tmp_path / "summary.json", _summary(failed=[]))

    def fake_host_preflight(*, required):
        return {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": True,
            "generated_at": time.time(),
            "current_host": {"platform_system": "Linux", "python_tag": "py310"},
            "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "runnable_gates": list(DIMOS_BENCHMARK_REQUIRED_GATES),
            "blocked_gates": [],
            "gates": {
                gate: {"ok": True, "status": "runnable", "checks": {}} for gate in DIMOS_BENCHMARK_REQUIRED_GATES
            },
        }

    monkeypatch.setattr(
        dimos_gap_report.server_sim_closure,
        "host_preflight",
        fake_host_preflight,
    )

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        host_preflight=True,
    )

    assert report["host_preflight"]["checked"] is True
    assert report["host_preflight"]["ok"] is False
    failed = report["host_setup_plan"]["failed_checks"][0]
    assert failed["check"] == "host_preflight_report_freshness"
    assert "host preflight report contract was not checked" in failed["blockers"][0]
    assert "host preflight report freshness was not checked" in failed["blockers"][0]
    assert report["lingtu_readiness"]["host_preflight_ok"] is False
    assert report["lingtu_readiness"]["ok"] is False


def test_dimos_gap_report_requires_complete_runtime_dataflow_gate_set(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(tmp_path / "summary.json", _summary(failed=[]))

    def fake_runtime_dataflow_from_summary(summary):
        return {
            "native_pct_mujoco": {
                "checked": True,
                "ok": True,
                "schema_detected": "test",
                "primary_blocker": "",
                "flow": [],
            }
        }

    monkeypatch.setattr(
        dimos_gap_report,
        "_runtime_dataflow_from_summary",
        fake_runtime_dataflow_from_summary,
    )

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    missing = report["runtime_dataflow"]["missing_runtime_dataflow_gates"]
    assert report["lingtu_readiness"]["runtime_dataflow_checked"] is True
    assert report["lingtu_readiness"]["runtime_dataflow_complete"] is False
    assert report["lingtu_readiness"]["ok"] is False
    assert "native_pct_mujoco" not in missing
    assert sorted(missing) == sorted(RUNTIME_DATAFLOW_GATES - {"native_pct_mujoco"})
    assert set(report["runtime_dataflow"]["gate_failures"]) >= set(missing)


def test_dimos_gap_report_surfaces_host_preflight_blockers(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["native_pct_mujoco"]),
    )

    def fake_host_preflight(*, required):
        assert required == set(DIMOS_BENCHMARK_REQUIRED_GATES)
        return {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "current_host": {
                "platform_system": "Windows",
                "machine": "AMD64",
                "python_tag": "py313",
            },
            "runnable_gates": ["gateway_runtime_acceptance"],
            "blocked_gates": ["native_pct_mujoco"],
            "gates": {
                "native_pct_mujoco": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": ["PCT planner runtime unavailable"],
                    "checks": {
                        "pct_planner_runtime": {
                            "ok": False,
                            "blocker": "PCT planner runtime unavailable",
                            "evidence": {"python_tag": "py313"},
                        }
                    },
                    "command": "run native_pct_mujoco",
                    "expected_report_path": ("artifacts/server_sim_closure/native_pct_mujoco/report.json"),
                    "host_requirements": ["selected PCT planner runtime"],
                }
            },
        }

    monkeypatch.setattr(
        dimos_gap_report.server_sim_closure,
        "host_preflight",
        fake_host_preflight,
    )

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        host_preflight=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    native = rows["native_pct_mujoco"]
    assert report["host_preflight"]["checked"] is True
    assert report["host_preflight"]["ok"] is False
    assert native["host_preflight"]["ok"] is False
    assert native["host_preflight"]["failed_checks"] == ["pct_planner_runtime"]
    assert native["primary_category"] == "environment_runtime"
    assert native["pipeline_trace"] == [
        "pct_planner_runtime",
        "legacy_pct_to_local_autonomy",
        "mujoco_motion_executor",
    ]
    assert native["recommended_action"] == "fix host preflight before running this gate"
    assert report["host_setup_plan"]["checked"] is True
    assert report["host_setup_plan"]["ok"] is False
    assert report["host_setup_plan"]["failed_check_count"] == 1
    assert report["host_setup_plan"]["failed_checks"][0]["check"] == "pct_planner_runtime"
    assert report["host_setup_plan"]["failed_checks"][0]["gates"] == ["native_pct_mujoco"]
    diagnostic_commands = report["host_setup_plan"]["failed_checks"][0]["diagnostic_commands"]
    assert any("pct_runtime_preflight.py" in command for command in diagnostic_commands)
    assert "bash sim/scripts/setup_linux_validation_host.sh" not in diagnostic_commands
    pct_setup = report["host_setup_plan"]["failed_checks"][0]
    pct_setup_text = " ".join(
        [
            str(pct_setup["recommended_action"]),
            *[str(command) for command in pct_setup["diagnostic_commands"]],
        ]
    )
    assert "Linux" not in pct_setup_text
    assert "CPython 3.10" not in pct_setup_text
    assert "GTSAM" not in pct_setup_text
    assert report["execution_plan"]["ok_to_run_missing"] is False
    assert [phase["id"] for phase in report["execution_plan"]["phases"]] == [
        "host_preflight",
        "host_setup",
        "linux_sim_closure",
        "blocked_gate_commands",
        "final_summary",
    ]
    linux_phase = report["execution_plan"]["phases"][2]
    assert linux_phase["status"] == "target_host"
    assert linux_phase["order"] == "host_preflight_then_dimos_dependency_order"
    assert linux_phase["gates"] == ["native_pct_mujoco"]
    linux_commands = [item["command"] for item in linux_phase["commands"]]
    assert "bash sim/scripts/run_dimos_linux_closure.sh --dry-run" in linux_commands
    assert 'test "$(uname -s)" = Linux' in linux_commands
    assert "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75}" in linux_commands
    assert any("server_sim_closure.py" in command for command in linux_commands)
    assert any("dimos_gap_report.py" in command for command in linux_commands)
    blocked_phase = report["execution_plan"]["phases"][3]
    assert blocked_phase["gates"] == ["native_pct_mujoco"]
    assert blocked_phase["commands"][0]["command"] == "run native_pct_mujoco"
    assert len(report["next_steps"]) == 1
    next_step = report["next_steps"][0]
    assert next_step["gate"] == "native_pct_mujoco"
    assert next_step["priority"] == "p1"
    assert next_step["category"] == "environment_runtime"
    assert next_step["recommended_action"] == "fix host preflight before running this gate"
    assert next_step["command"] == "run native_pct_mujoco"
    assert next_step["expected_report_path"] == "artifacts/server_sim_closure/native_pct_mujoco/report.json"
    assert next_step["dependency_blockers"] == []
    assert next_step["dependency_blocker_status"] == {}
    assert next_step["host_preflight_ok"] is False
    assert next_step["host_preflight_blockers"] == ["PCT planner runtime unavailable"]
    assert next_step["host_failed_checks"] == ["pct_planner_runtime"]
    assert next_step["runtime_dataflow_blocker"] == ""
    assert next_step["runtime_dataflow_failed_edges"] == []
    assert next_step["evidence_blockers"][0]["source"] == "host_preflight"
    assert next_step["evidence_blockers"][0]["failed_checks"] == ["pct_planner_runtime"]
    assert report["pipeline_trace"]["claim_boundary"].startswith("Code path trace only")
    trace_ids = [step["id"] for step in report["pipeline_trace"]["primary_chain"]]
    assert "pct_planner_runtime" in trace_ids
    assert "mujoco_motion_executor" in trace_ids


def test_dimos_gap_report_reads_run_missing_host_blocked_summary(tmp_path: Path):
    gate_preflight = {
        "ok": False,
        "status": "blocked",
        "blockers": ["PCT planner runtime unavailable"],
        "checks": {
            "pct_planner_runtime": {
                "ok": False,
                "blocker": "PCT planner runtime unavailable",
                "evidence": {"python_tag": "py313"},
                "recommended_action": "PCT planner runtime setup hint",
                "diagnostic_commands": ["PCT planner runtime diagnostic"],
            }
        },
        "command": "run native_pct_mujoco",
        "expected_report_path": ("artifacts/server_sim_closure/native_pct_mujoco/report.json"),
    }
    summary = _summary(failed=["native_pct_mujoco"])
    summary.update(
        {
            "execution_mode": "run_missing",
            "run_missing": True,
            "skip_host_blocked": True,
            "gate_runs": [
                {
                    "name": "native_pct_mujoco",
                    "status": "host_blocked",
                    "returncode": None,
                    "executed_command": None,
                    "shell": None,
                    "error": "PCT planner runtime unavailable",
                    "host_preflight": gate_preflight,
                }
            ],
            "run_missing_host_preflight": {
                "schema_version": "lingtu.server_sim_host_preflight.v1",
                "execution_mode": "host_preflight_only",
                "ok": False,
                "current_host": {
                    "platform_system": "Windows",
                    "machine": "AMD64",
                    "python_tag": "py313",
                },
                "runnable_gates": [],
                "blocked_gates": ["native_pct_mujoco"],
                "gates": {"native_pct_mujoco": gate_preflight},
            },
        }
    )
    summary_path = _write_json(tmp_path / "summary_run_missing.json", summary)

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    native = rows["native_pct_mujoco"]
    assert report["host_preflight"]["checked"] is True
    assert report["host_preflight"]["source"] == "summary_run_missing_host_preflight"
    assert report["host_preflight"]["blocked_gates"] == ["native_pct_mujoco"]
    assert native["status"] == "host_blocked"
    assert native["primary_category"] == "environment_runtime"
    assert native["recommended_action"] == "fix host preflight before running this gate"
    assert native["run_missing_attempt"] == {
        "checked": True,
        "status": "host_blocked",
        "returncode": None,
        "executed_command": None,
        "shell": None,
        "error": "PCT planner runtime unavailable",
    }
    assert native["host_preflight"]["failed_checks"] == ["pct_planner_runtime"]
    assert native["host_preflight"]["blockers"] == ["PCT planner runtime unavailable"]
    assert report["host_setup_plan"]["failed_checks"][0]["recommended_action"] == ("PCT planner runtime setup hint")
    assert report["host_setup_plan"]["failed_checks"][0]["diagnostic_commands"] == ["PCT planner runtime diagnostic"]
    assert report["execution_plan"]["ok_to_run_missing"] is False
    phase_ids = [phase["id"] for phase in report["execution_plan"]["phases"]]
    assert "linux_sim_closure" in phase_ids
    assert "blocked_gate_commands" in phase_ids


def test_dimos_gap_report_reads_host_preflight_report_path(tmp_path: Path):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["native_pct_mujoco"]),
    )
    preflight_path = _write_json(
        tmp_path / "host_preflight.json",
        {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "current_host": {
                "platform_system": "Windows",
                "machine": "AMD64",
                "python_tag": "py313",
            },
            "runnable_gates": [],
            "blocked_gates": ["native_pct_mujoco"],
            "host_setup_plan": {
                "checked": True,
                "source": "stale_top_level_plan",
                "failed_checks": [
                    {
                        "check": "stale_check",
                        "gates": ["wrong_gate"],
                        "diagnostic_commands": ["do not use this stale command"],
                    }
                ],
            },
            "gates": {
                "native_pct_mujoco": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": ["PCT planner runtime unavailable"],
                    "checks": {
                        "pct_planner_runtime": {
                            "ok": False,
                            "blocker": "PCT planner runtime unavailable",
                            "evidence": {"python_tag": "py313"},
                        }
                    },
                }
            },
        },
    )

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        host_preflight_report=preflight_path,
    )

    native = {row["gate"]: row for row in report["gap_matrix"]}["native_pct_mujoco"]
    assert report["host_preflight"]["checked"] is True
    assert report["host_preflight"]["source"] == f"file:{preflight_path}"
    assert report["host_preflight"]["blocked_gates"] == ["native_pct_mujoco"]
    assert native["host_preflight"]["failed_checks"] == ["pct_planner_runtime"]
    assert native["recommended_action"] == "fix host preflight before running this gate"
    assert report["host_setup_plan"]["source"] == "recomputed_from_host_preflight_gates"
    assert report["host_setup_plan"]["failed_checks"][0]["check"] == "pct_planner_runtime"
    assert report["host_setup_plan"]["failed_checks"][0]["gates"] == ["native_pct_mujoco"]
    assert "do not use this stale command" not in report["host_setup_plan"]["failed_checks"][0]["diagnostic_commands"]
    assert report["execution_plan"]["ok_to_run_missing"] is False
    phase_ids = [phase["id"] for phase in report["execution_plan"]["phases"]]
    assert "preflight_required_gate_commands" not in phase_ids
    assert "blocked_gate_commands" in phase_ids


def test_dimos_gap_report_orders_blocked_commands_by_dependency(tmp_path: Path):
    failed = [
        "moving_obstacle_sweep",
        "large_loop_closure",
        "large_terrain",
        "native_pct_mujoco",
    ]
    summary = _summary(failed=failed)
    summary["run_missing_host_preflight"] = {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        "ok": False,
        "current_host": {"platform_system": "Windows", "python_tag": "py313"},
        "runnable_gates": [],
        "blocked_gates": failed,
        "gates": {
            gate: {
                "ok": False,
                "status": "blocked",
                "blockers": ["PCT planner runtime unavailable"],
                "checks": {
                    "pct_planner_runtime": {
                        "ok": False,
                        "blocker": "PCT planner runtime unavailable",
                    }
                },
            }
            for gate in failed
        },
    }
    summary_path = _write_json(tmp_path / "summary_dependency_order.json", summary)

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    assert rows["moving_obstacle_sweep"]["priority"] == "p0"
    assert rows["large_loop_closure"]["priority"] == "p0"
    blocked_phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "blocked_gate_commands"
    )
    assert blocked_phase["order"] == "dimos_dependency_order"
    assert blocked_phase["gates"] == [
        "large_terrain",
        "native_pct_mujoco",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]


def test_dimos_gap_report_orders_runnable_commands_by_dependency(tmp_path: Path):
    failed = [
        "moving_obstacle_sweep",
        "large_loop_closure",
        "large_terrain",
        "native_pct_mujoco",
    ]
    summary = _summary(failed=failed)
    summary["run_missing_host_preflight"] = {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        **_fresh_host_preflight_contract(),
        "ok": True,
        "current_host": {"platform_system": "Linux", "python_tag": "py310"},
        "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
        "runnable_gates": failed,
        "blocked_gates": [],
        "gates": {gate: {"ok": True, "status": "runnable", "checks": {}} for gate in failed},
    }
    summary_path = _write_json(
        tmp_path / "summary_runnable_dependency_order.json",
        summary,
    )

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    assert rows["moving_obstacle_sweep"]["priority"] == "p0"
    assert rows["large_loop_closure"]["priority"] == "p0"
    runnable_phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "run_unblocked_gate_commands"
    )
    assert runnable_phase["order"] == "dimos_dependency_order"
    assert runnable_phase["gates"] == [
        "large_terrain",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]
    assert [command["gate"] for command in runnable_phase["commands"]] == [
        "large_terrain",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]
    dependency_phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "dependency_blocked_gate_commands"
    )
    assert dependency_phase["gates"] == ["native_pct_mujoco"]
    assert dependency_phase["commands"][0]["dependency_blockers"] == ["large_terrain"]
    next_step_order = [step["gate"] for step in report["next_steps"][:4]]
    assert next_step_order == [
        "large_terrain",
        "moving_obstacle_sweep",
        "large_loop_closure",
        "native_pct_mujoco",
    ]


def test_dimos_gap_report_blocks_live_dependents_when_prerequisite_is_host_blocked(
    tmp_path: Path,
):
    failed = [
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]
    summary = _summary(failed=failed)
    summary["run_missing_host_preflight"] = {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        "ok": False,
        "current_host": {"platform_system": "Windows", "python_tag": "py313"},
        "runnable_gates": ["moving_obstacle_sweep", "large_loop_closure"],
        "blocked_gates": ["fastlio2_dynamic_inspection"],
        "gates": {
            "fastlio2_dynamic_inspection": {
                "ok": False,
                "status": "blocked",
                "blockers": ["ROS 2 Humble environment is not sourced"],
                "checks": {
                    "ros2_humble": {
                        "ok": False,
                        "blocker": "ROS 2 Humble environment is not sourced",
                    }
                },
            },
            "moving_obstacle_sweep": {
                "ok": True,
                "status": "runnable",
                "checks": {},
            },
            "large_loop_closure": {
                "ok": True,
                "status": "runnable",
                "checks": {},
            },
        },
    }
    summary_path = _write_json(
        tmp_path / "summary_dependency_blocked_live.json",
        summary,
    )

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    assert report["lingtu_readiness"]["highest_actionable_blocker"] == ("fastlio2_dynamic_inspection")
    assert rows["moving_obstacle_sweep"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert rows["large_loop_closure"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert rows["moving_obstacle_sweep"]["recommended_action"] == (
        "run prerequisite gate reports first: fastlio2_dynamic_inspection"
    )
    blocked_phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "blocked_gate_commands"
    )
    assert blocked_phase["gates"] == ["fastlio2_dynamic_inspection"]
    dependency_phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "dependency_blocked_gate_commands"
    )
    assert dependency_phase["gates"] == [
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]
    assert dependency_phase["commands"][0]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    next_steps = {step["gate"]: step for step in report["next_steps"]}
    assert next_steps["moving_obstacle_sweep"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]


def test_dimos_gap_report_marks_missing_prerequisites_as_dependency_blockers(
    tmp_path: Path,
):
    summary_path = _write_json(
        tmp_path / "summary_missing_prerequisites.json",
        _summary(
            failed=[
                "fastlio2_dynamic_inspection",
                "moving_obstacle_sweep",
                "large_loop_closure",
                "saved_map_relocalize",
                "pct_saved_map_navigation",
            ]
        ),
    )

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    assert rows["moving_obstacle_sweep"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert rows["moving_obstacle_sweep"]["dependency_blocker_status"] == {"fastlio2_dynamic_inspection": "failed"}
    assert rows["large_loop_closure"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert rows["pct_saved_map_navigation"]["dependency_blockers"] == ["saved_map_relocalize"]
    assert rows["pct_saved_map_navigation"]["recommended_action"] == (
        "run prerequisite gate reports first: saved_map_relocalize"
    )

    preflight_phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "preflight_required_gate_commands"
    )
    command_by_gate = {command["gate"]: command for command in preflight_phase["commands"]}
    assert command_by_gate["moving_obstacle_sweep"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert command_by_gate["large_loop_closure"]["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert command_by_gate["pct_saved_map_navigation"]["dependency_blockers"] == ["saved_map_relocalize"]
    next_steps = {step["gate"]: step for step in report["next_steps"]}
    assert next_steps["pct_saved_map_navigation"]["dependency_blockers"] == ["saved_map_relocalize"]


def test_dimos_gap_report_surfaces_gate_environment_evidence(tmp_path: Path):
    summary = _summary(failed=["dynamic_obstacle_local_planner", "large_terrain"])
    gate = summary["gates"]["dynamic_obstacle_local_planner"]
    gate["evidence"] = {
        "execution_mode": "host_guard",
        "environment": {
            "accepted_host": False,
            "blocked_reason": "windows_mingw_numpy_not_accepted",
            "claim_boundary": "environment_blocked_no_algorithm_claim",
        },
    }
    large_gate = summary["gates"]["large_terrain"]
    large_gate["evidence"] = {
        "execution_mode": "host_guard",
        "environment": {
            "accepted_host": False,
            "blocked_reason": "pct_planner_runtime_unavailable",
            "claim_boundary": "environment_blocked_no_algorithm_claim",
        },
    }
    summary_path = _write_json(tmp_path / "summary_environment.json", summary)

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    rows = {item["gate"]: item for item in report["gap_matrix"]}
    row = rows["dynamic_obstacle_local_planner"]
    assert row["execution_mode"] == "host_guard"
    assert row["environment"] == {
        "accepted_host": False,
        "blocked_reason": "windows_mingw_numpy_not_accepted",
        "claim_boundary": "environment_blocked_no_algorithm_claim",
    }
    large_row = rows["large_terrain"]
    assert large_row["execution_mode"] == "host_guard"
    assert large_row["environment"]["blocked_reason"] == "pct_planner_runtime_unavailable"


def test_dimos_gap_report_requires_host_preflight_before_gate_commands(
    tmp_path: Path,
):
    summary_path = _write_json(
        tmp_path / "summary_preflight_required.json",
        _summary(failed=["large_terrain", "native_pct_mujoco"]),
    )

    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    assert report["host_preflight"]["checked"] is False
    assert report["execution_plan"]["ok_to_run_missing"] is False
    phase = next(
        phase for phase in report["execution_plan"]["phases"] if phase["id"] == "preflight_required_gate_commands"
    )
    assert phase["status"] == "blocked"
    assert phase["gates"] == ["large_terrain", "native_pct_mujoco"]
    assert [command["gate"] for command in phase["commands"]] == [
        "large_terrain",
        "native_pct_mujoco",
    ]


def test_dimos_gap_report_cli_writes_markdown(tmp_path: Path):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["moving_obstacle_sweep"]),
    )
    out = tmp_path / "gap.md"

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_gap_report.py",
            "--summary",
            str(summary_path),
            "--format",
            "markdown",
            "--json-out",
            str(out),
        ],
        cwd=Path(__file__).resolve().parents[2],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 1
    text = out.read_text(encoding="utf-8")
    assert "# DimOS Gap Report" in text
    assert "`moving_obstacle_sweep`" in text
    assert "dynamic replanning stress matrix" in text


def test_dimos_gap_report_cli_accepts_host_preflight_report(tmp_path: Path):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["native_pct_mujoco"]),
    )
    preflight_path = _write_json(
        tmp_path / "host_preflight.json",
        {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "current_host": {
                "platform_system": "Windows",
                "machine": "AMD64",
                "python_tag": "py313",
            },
            "runnable_gates": [],
            "blocked_gates": ["native_pct_mujoco"],
            "gates": {
                "native_pct_mujoco": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": ["PCT planner runtime unavailable"],
                    "checks": {
                        "pct_planner_runtime": {
                            "ok": False,
                            "blocker": "PCT planner runtime unavailable",
                            "evidence": {"python_tag": "py313"},
                        }
                    },
                }
            },
        },
    )
    out = tmp_path / "gap.json"

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_gap_report.py",
            "--summary",
            str(summary_path),
            "--host-preflight-report",
            str(preflight_path),
            "--json-out",
            str(out),
        ],
        cwd=Path(__file__).resolve().parents[2],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 1
    report = json.loads(out.read_text(encoding="utf-8"))
    assert report["host_preflight"]["checked"] is True
    assert report["host_preflight"]["source"] == f"file:{preflight_path}"
    assert report["execution_plan"]["ok_to_run_missing"] is False


def test_dimos_gap_report_markdown_includes_host_setup_blockers(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["native_pct_mujoco"]),
    )
    out = tmp_path / "gap.md"

    def fake_host_preflight(*, required):
        return {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "current_host": {"platform_system": "Linux", "python_tag": "py311"},
            "runnable_gates": [],
            "blocked_gates": ["native_pct_mujoco"],
            "gates": {
                "native_pct_mujoco": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": ["PCT planner runtime unavailable"],
                    "checks": {
                        "pct_planner_runtime": {
                            "ok": False,
                            "blocker": "PCT planner runtime unavailable",
                            "evidence": {"python_tag": "py311"},
                        }
                    },
                }
            },
        }

    monkeypatch.setattr(
        dimos_gap_report.server_sim_closure,
        "host_preflight",
        fake_host_preflight,
    )
    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        host_preflight=True,
    )

    dimos_gap_report._write_output(out, dimos_gap_report._markdown_table(report))

    text = out.read_text(encoding="utf-8")
    assert "## Host Setup Blockers" in text
    assert "## Execution Plan" in text
    assert "## Pipeline Trace" in text
    assert "`pct_planner_runtime`" in text
    assert "build or install the selected PCT planner runtime" in text
    assert "`blocked_gate_commands`" in text
    assert "dimos_dependency_order" in text
    assert "`mujoco_motion_executor`" in text


def test_dimos_gap_report_attaches_aggregate_child_dataflow(tmp_path: Path):
    child = _write_json(
        tmp_path / "live" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": False,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 0,
            },
            "lingtu_inspection": {
                "enabled": True,
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 0,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    aggregate = _write_json(
        tmp_path / "moving_obstacle_sweep" / "report.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "minimal_red_defect": {"path": str(child)},
        },
    )
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["primary_blocker"] == "path_follower_to_cmd_vel"
    assert flow["source_gate_report"] == str(aggregate)
    assert flow["source_report"] == str(child)
    assert "path_follower_to_cmd_vel" in flow["failed_edges"]
    assert report["runtime_dataflow"]["failing_primary_blockers"]["moving_obstacle_sweep"] == "path_follower_to_cmd_vel"


def test_dimos_gap_report_rejects_stale_aggregate_child_dataflow(tmp_path: Path):
    child = _write_json(
        tmp_path / "live" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "enabled": True,
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    old = time.time() - 172_800.0
    os.utime(child, (old, old))
    aggregate = _write_json(
        tmp_path / "moving_obstacle_sweep" / "report.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "minimal_red_defect": {"path": str(child)},
        },
    )
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is False
    assert flow["ok"] is False
    assert flow["reason"] == "child_report_stale"
    assert flow["source_gate_report"] == str(aggregate)
    assert flow["source_report"] == str(child)
    assert flow["child_report_freshness"]["fresh"] is False
    assert "child_report_age_vs_parent_s" in flow["child_report_freshness"]["blockers"][0]


def test_dimos_gap_report_rejects_old_child_even_when_parent_is_equally_old(
    tmp_path: Path,
):
    child = _write_json(
        tmp_path / "live" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "enabled": True,
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    aggregate = _write_json(
        tmp_path / "moving_obstacle_sweep" / "report.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "minimal_red_defect": {"path": str(child)},
        },
    )
    old = time.time() - 172_800.0
    os.utime(child, (old, old))
    os.utime(aggregate, (old, old))
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is False
    assert flow["ok"] is False
    assert flow["reason"] == "child_report_stale"
    assert flow["child_report_freshness"]["fresh"] is False
    assert flow["child_report_freshness"]["age_vs_parent_s"] == 0.0
    assert any("child_report_age_s" in blocker for blocker in flow["child_report_freshness"]["blockers"])


def test_dimos_gap_report_cli_json_includes_aggregate_child_dataflow(
    tmp_path: Path,
):
    child = _write_json(
        tmp_path / "live" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": False,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 0,
            },
            "lingtu_inspection": {
                "enabled": True,
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 0,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    aggregate = _write_json(
        tmp_path / "moving_obstacle_sweep" / "report.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "minimal_red_defect": {"path": str(child)},
        },
    )
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)
    out = tmp_path / "dimos_gap.json"

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_gap_report.py",
            "--summary",
            str(summary_path),
            "--include-dataflow",
            "--format",
            "json",
            "--json-out",
            str(out),
        ],
        cwd=Path(__file__).resolve().parents[2],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 1
    payload = json.loads(out.read_text(encoding="utf-8"))
    rows = {row["gate"]: row for row in payload["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert payload["runtime_dataflow"]["schema_version"] == ("lingtu.dimos_runtime_dataflow.v1")
    assert flow["checked"] is True
    assert flow["source_gate_report"] == str(aggregate)
    assert flow["source_report"] == str(child)
    assert flow["primary_blocker"] == "path_follower_to_cmd_vel"
    assert flow["edge_status"]["path_follower_to_cmd_vel"] is False
    assert (
        payload["runtime_dataflow"]["failing_primary_blockers"]["moving_obstacle_sweep"] == "path_follower_to_cmd_vel"
    )


def test_dimos_gap_report_missing_dataflow_uses_expected_report_path(
    tmp_path: Path,
):
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = ""
    expected = tmp_path / "expected" / "moving_obstacle_sweep.json"
    for action in summary["algorithm_validation"]["next_actions"]:
        if action["gate"] == "moving_obstacle_sweep":
            action["expected_report_path"] = str(expected)
            action.pop("report_path", None)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is False
    assert flow["reason"] == "report_missing"
    assert flow["source_gate_report"] == str(expected)
    assert flow["source_report"] == str(expected)


def test_dimos_gap_report_missing_fastlio_dataflow_lists_rejected_candidates(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(dimos_gap_report, "ROOT", tmp_path)
    candidate = _write_json(
        tmp_path / "mujoco_fastlio2_live_case" / "legacy_live" / "run_001" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_cloud_map": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 0,
                "nav_cmd_vel_nonzero": 0,
            },
        },
    )
    expected = tmp_path / "mujoco_fastlio2_live*" / "inspection*" / "*" / "report.json"
    summary = _summary(failed=["fastlio2_dynamic_inspection"])
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = ""
    for action in summary["algorithm_validation"]["next_actions"]:
        if action["gate"] == "fastlio2_dynamic_inspection":
            action["expected_report_path"] = str(expected)
            action.pop("report_path", None)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["fastlio2_dynamic_inspection"]["runtime_dataflow"]
    assert flow["checked"] is False
    assert flow["ok"] is False
    assert flow["reason"] == "report_missing"
    assert flow["source_report"] == str(expected)
    assert flow["candidate_reports"][0]["path"] == str(candidate)
    assert flow["candidate_reports"][0]["ok"] is False
    assert flow["candidate_reports"][0]["primary_blocker"] == ("global_planner_to_local_planner")
    assert "path_follower_to_cmd_vel" in flow["candidate_reports"][0]["failed_edges"]
    assert "does not satisfy fastlio2_dynamic_inspection" in flow["candidate_reports"][0]["rejection_reason"]

    chain = report["runtime_dataflow"]["cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
    assert chain["ok"] is False
    assert chain["fastlio_candidate_rejections"][0]["path"] == str(candidate)
    assert chain["fastlio_candidate_rejections"][0]["primary_blocker"] == ("global_planner_to_local_planner")


def test_dimos_gap_report_resolves_matching_glob_dataflow_report(
    tmp_path: Path,
):
    matched = _write_json(
        tmp_path / "mujoco_fastlio2_live_case" / "inspection_smoke" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_cloud_map": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_planner": "pct",
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "last_plan_report": {"selected_planner": "pct"},
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    expected = tmp_path / "mujoco_fastlio2_live*" / "inspection*" / "report.json"
    summary = _summary(failed=["fastlio2_dynamic_inspection"])
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = ""
    for action in summary["algorithm_validation"]["next_actions"]:
        if action["gate"] == "fastlio2_dynamic_inspection":
            action["expected_report_path"] = str(expected)
            action.pop("report_path", None)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["fastlio2_dynamic_inspection"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["source_report"] == str(matched)
    assert flow["reason"] == ""
    assert flow["candidate_reports"] == []


def test_dimos_gap_report_maps_remote_absolute_artifact_dataflow_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    local_report = _write_json(
        tmp_path
        / "artifacts"
        / "server_sim_closure"
        / "mujoco_fastlio2_live"
        / "inspection-moving-obstacle-video-remote"
        / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": False,
            "outputs": {},
            "remaining_gaps": ["Fast-LIO did not publish both odometry and map cloud"],
        },
    )
    remote_report = (
        "/home/bsrl/hongsenpang/lingtu_dimos_20260608/"
        "artifacts/server_sim_closure/mujoco_fastlio2_live/"
        "inspection-moving-obstacle-video-remote/report.json"
    )
    summary = _summary(failed=["fastlio2_dynamic_inspection"])
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = remote_report
    for action in summary["algorithm_validation"]["next_actions"]:
        if action["gate"] == "fastlio2_dynamic_inspection":
            action["report_path"] = remote_report
            action["expected_report_path"] = remote_report
    summary_path = _write_json(tmp_path / "summary.json", summary)
    monkeypatch.setattr(dimos_gap_report, "ROOT", tmp_path)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["fastlio2_dynamic_inspection"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["source_report"] == str(local_report)
    assert flow["source_gate_report"] == str(local_report)
    assert flow["primary_blocker"] == "raw_lidar_to_fastlio"
    assert "raw_lidar_to_fastlio" in flow["failed_edges"]


def test_dimos_gap_report_summarizes_empty_moving_obstacle_sweep_report(
    tmp_path: Path,
):
    aggregate = _write_json(
        tmp_path / "moving_obstacle_sweep" / "report.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 0,
            "passed_pair_count": 0,
            "required_live_nav_chain": True,
            "required_scan_time_profile": "physical_rolling",
            "require_video_file": True,
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "covered_pairs": [],
            "missing_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "cases": [],
            "blockers": ["no moving obstacle reports provided"],
        },
    )
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["schema_detected"] == "moving_obstacle_sweep"
    assert flow["edge_status"]["speed_density_matrix"] is False
    assert flow["edge_status"]["non_motion_claim_boundary"] is True
    assert flow["primary_blocker"] == "speed_density_matrix"


def test_dimos_gap_report_summarizes_empty_large_loop_closure_report(
    tmp_path: Path,
):
    aggregate = _write_json(
        tmp_path / "large_loop_closure" / "report.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 0,
            "passed_case_count": 0,
            "failed_case_count": 0,
            "best_case": {},
            "minimal_red_defect": {},
            "cases": [],
            "thresholds": {
                "min_path_length_m": 20.0,
                "min_goal_span_m": 4.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
                "require_video_file": True,
            },
            "blockers": ["no large-loop runtime reports provided"],
        },
    )
    summary = _summary(failed=["large_loop_closure"])
    summary["gates"]["large_loop_closure"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["large_loop_closure"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["schema_detected"] == "large_loop_closure"
    assert flow["edge_status"]["large_loop_runtime_matrix"] is False
    assert flow["edge_status"]["non_motion_claim_boundary"] is True
    assert flow["primary_blocker"] == "large_loop_runtime_matrix"


def test_dimos_gap_report_attaches_embedded_case_dataflow(tmp_path: Path):
    aggregate = _write_json(
        tmp_path / "large_loop_closure" / "report.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": False,
            "minimal_red_defect": {
                "path": str(tmp_path / "missing_child.json"),
                "fastlio2_path_length_m": 12.0,
                "global_path_points_max": 8,
                "local_path_count": 1,
                "local_path_points_max": 4,
                "nav_cmd_vel_nonzero": 0,
                "successful_navigation_goal_count": 0,
                "min_required_checkpoints": 2,
            },
        },
    )
    summary = _summary(failed=["large_loop_closure"])
    summary["gates"]["large_loop_closure"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["large_loop_closure"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["schema_detected"] == "aggregate_case"
    assert flow["edge_status"]["fastlio_feedback"] is True
    assert flow["edge_status"]["global_path"] is True
    assert flow["edge_status"]["local_path"] is True
    assert flow["edge_status"]["cmd_vel"] is False
    assert flow["primary_blocker"] == "cmd_vel"


def test_dimos_gap_report_rejects_embedded_case_with_empty_latest_local_path(
    tmp_path: Path,
):
    aggregate = _write_json(
        tmp_path / "large_loop_closure" / "report.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": False,
            "minimal_red_defect": {
                "path": str(tmp_path / "missing_child.json"),
                "fastlio2_path_length_m": 12.0,
                "global_path_points_max": 8,
                "local_path_count": 356,
                "local_path_points_max": 101,
                "nav_cmd_vel_nonzero": 310,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 4,
                "navigation_diagnostics": {
                    "last_sample": {
                        "paths": {
                            "local_path_points_latest": 0,
                        }
                    }
                },
                "autonomy_chain": {
                    "local_planner": {
                        "local_planner": {
                            "last_local_path_points": 0,
                            "last_local_path_span_m": 0.0,
                            "last_control_hint": {
                                "reason": "untrackable_local_path",
                                "safety_stop": True,
                                "path_found": False,
                            },
                        }
                    },
                    "path_follower": {
                        "path_follower": {
                            "has_path": False,
                        }
                    },
                },
            },
        },
    )
    summary = _summary(failed=["large_loop_closure"])
    summary["gates"]["large_loop_closure"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["large_loop_closure"]["runtime_dataflow"]
    latest = flow["edge_evidence"]["local_path"]["latest_local_path"]
    assert flow["checked"] is True
    assert flow["schema_detected"] == "aggregate_case"
    assert flow["edge_status"]["local_path"] is False
    assert flow["primary_blocker"] == "local_path"
    assert latest["latest_local_path_points"] == 0
    assert latest["local_planner_reason"] == "untrackable_local_path"
    assert "local planner requested safety_stop" in latest["blockers"]
    assert "path follower has no active path" in latest["blockers"]


def test_dimos_gap_report_allows_empty_latest_local_path_after_terminal_success(
    tmp_path: Path,
):
    aggregate = _write_json(
        tmp_path / "large_loop_closure" / "report.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": False,
            "minimal_red_defect": {
                "path": str(tmp_path / "missing_child.json"),
                "fastlio2_path_length_m": 12.0,
                "global_path_points_max": 8,
                "local_path_count": 356,
                "local_path_points_max": 101,
                "nav_cmd_vel_nonzero": 310,
                "successful_navigation_goal_count": 3,
                "min_required_checkpoints": 3,
                "navigation_state": "SUCCESS",
                "patrol_index": 3,
                "patrol_total": 3,
                "navigation_diagnostics": {
                    "last_sample": {
                        "paths": {
                            "local_path_points_latest": 0,
                        }
                    }
                },
                "autonomy_chain": {
                    "local_planner": {
                        "local_planner": {
                            "last_local_path_points": 0,
                            "last_local_path_span_m": 0.0,
                            "last_control_hint": {
                                "reason": "mission_complete_stop",
                                "safety_stop": True,
                                "path_found": False,
                            },
                        }
                    },
                    "path_follower": {
                        "path_follower": {
                            "has_path": False,
                        }
                    },
                },
            },
        },
    )
    summary = _summary(failed=["large_loop_closure"])
    summary["gates"]["large_loop_closure"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["large_loop_closure"]["runtime_dataflow"]
    latest = flow["edge_evidence"]["local_path"]["latest_local_path"]
    assert flow["checked"] is True
    assert flow["schema_detected"] == "aggregate_case"
    assert flow["edge_status"]["local_path"] is True
    assert latest["terminal_success"] is True
    assert latest["latest_local_path_points"] == 0
    assert latest["blockers"] == []


def test_dimos_gap_report_keeps_missing_child_without_embedded_evidence(
    tmp_path: Path,
):
    aggregate = _write_json(
        tmp_path / "moving_obstacle_sweep" / "report.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "minimal_red_defect": {"path": str(tmp_path / "missing_child.json")},
        },
    )
    summary = _summary(failed=["moving_obstacle_sweep"])
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(aggregate)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["moving_obstacle_sweep"]["runtime_dataflow"]
    assert flow["checked"] is False
    assert flow["reason"] == "child_report_missing"


def test_runtime_dataflow_summarizes_policy_nav_product_chain(tmp_path: Path):
    policy = _write_json(
        tmp_path / "policy_nav" / "report.json",
        _policy_nav_dataflow_report(),
    )
    summary = {
        "gates": {
            "policy_nav": {
                "ok": True,
                "status": "pass",
                "path": str(policy),
            },
        }
    }

    dataflow = build_runtime_dataflow_from_summary(
        summary,
        root=tmp_path,
        gates={"policy_nav"},
    )

    flow = dataflow["policy_nav"]
    edge_status = {edge["id"]: edge["ok"] for edge in flow["flow"]}
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["schema_detected"] == "policy_nav"
    assert edge_status["octoplanner3d_backend"] is True
    assert edge_status["global_path_to_waypoints"] is True
    assert edge_status["nanobind_local_planner"] is True
    assert edge_status["nav_kernel_path_follower"] is True
    assert edge_status["cmd_vel_mux_to_policy_driver"] is True
    assert flow["claim_boundary"] == "product_octoplanner_inprocess_nav_no_ros2"
    assert flow["planner_backend"]["configured_backend"] == "octoplanner3d"
    assert flow["local_planner_backend"] == "nanobind"
    assert flow["path_follower_backend"] == "nav_kernel"


def test_runtime_dataflow_policy_nav_blocks_legacy_backends(tmp_path: Path):
    payload = _policy_nav_dataflow_report()
    nav = payload["checks"][1]
    nav["passed"] = False
    nav["global_planner_backend_requested"] = "pct"
    nav["global_planner_backend_status"] = {
        "configured_backend": "pct",
        "backend": "pct",
        "degraded": False,
    }
    nav["local_planner_backend_requested"] = "simple"
    nav["local_planner_backend_actual"] = "simple"
    nav["path_follower_backend_requested"] = "pid"
    nav["path_follower_backend_actual"] = "pid"
    policy = _write_json(tmp_path / "policy_nav" / "report.json", payload)
    summary = {"gates": {"policy_nav": {"path": str(policy)}}}

    dataflow = build_runtime_dataflow_from_summary(
        summary,
        root=tmp_path,
        gates={"policy_nav"},
    )

    flow = dataflow["policy_nav"]
    edge_status = {edge["id"]: edge["ok"] for edge in flow["flow"]}
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["primary_blocker"] == "octoplanner3d_backend"
    assert edge_status["octoplanner3d_backend"] is False
    assert edge_status["nanobind_local_planner"] is False
    assert edge_status["nav_kernel_path_follower"] is False


def test_dimos_gap_report_attaches_native_pct_dataflow(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": False,
            "global_planner_source": "source_report/pct_tomogram",
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            "pct_planner_runtime_ok": True,
            "pct_path_count": 8,
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 3,
            "pct_planner_path_mode": "astar_raw_path",
            "selected_planner": "pct",
            "fallback_used": False,
            "path_count": 2,
            "max_path_poses": 5,
            "local_path_samples": [{"points": [[0, 0, 0], [1, 0, 0]]}],
            "cmd_count_nonzero": 0,
            "cmd_samples": [],
            "moved_m": 0.0,
            "reached_goal": False,
        },
    )
    summary = _summary(failed=["native_pct_mujoco"])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["native_pct_mujoco"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["schema_detected"] == "native_pct_mujoco"
    assert flow["edge_status"]["pct_planner_runtime"] is True
    assert flow["edge_status"]["pct_optimizer_mode"] is True
    assert flow["edge_status"]["global_path_to_local_planner"] is True
    assert flow["edge_status"]["path_follower_to_cmd_vel"] is False
    assert flow["primary_blocker"] == "path_follower_to_cmd_vel"


@pytest.mark.parametrize(
    ("contract_case", "failed_edge", "reason"),
    [
        (
            "legacy_runtime_fields",
            "pct_planner_runtime",
            "PCT planner runtime is not selected",
        ),
        (
            "legacy_path_mode",
            "pct_optimizer_mode",
            "unsupported PCT planner path mode",
        ),
        (
            "wrong_planner_source",
            "pct_planner_runtime",
            "global planner source is not source_report/pct_tomogram",
        ),
    ],
)
def test_dimos_gap_report_rejects_legacy_pct_acceptance_contract(
    tmp_path: Path,
    contract_case: str,
    failed_edge: str,
    reason: str,
):
    payload = _native_pct_dataflow_report()
    if contract_case == "legacy_runtime_fields":
        payload.pop("pct_planner_runtime")
        payload.pop("pct_planner_runtime_ok")
        payload["pct_runtime_ok"] = True
    elif contract_case == "legacy_path_mode":
        payload["pct_planner_path_mode"] = "native_astar_raw_path"
    else:
        payload["global_planner_source"] = "native_pct_tomogram"

    native = _write_json(
        tmp_path / contract_case / "report.json",
        payload,
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary_path = _write_json(tmp_path / f"summary_{contract_case}.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    flow = {row["gate"]: row for row in report["gap_matrix"]}["native_pct_mujoco"]["runtime_dataflow"]
    assert flow["ok"] is False
    assert flow["primary_blocker"] == failed_edge
    assert flow["edge_evidence"][failed_edge]["reason"] == reason


def test_runtime_dataflow_legacy_pct_runtime_alias_is_not_a_report_shape():
    flow = summarize_runtime_report({"pct_runtime_ok": True}, report_path=None)

    assert flow["checked"] is False
    assert flow["ok"] is False
    assert flow["reason"] == "unsupported_report_shape"


def test_dimos_gap_report_native_pct_dataflow_reports_ros2_runtime_blocker(
    tmp_path: Path,
):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": False,
            "native_gate_skipped": True,
            "claim_boundary": "ros2_runtime_unavailable",
            "blockers": ["ROS2 runtime unavailable for native local planner gate"],
            "global_planner_source": "source_report/pct_tomogram",
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            "pct_planner_runtime_ok": True,
            "pct_path_count": 8,
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 3,
            "pct_planner_path_mode": "astar_raw_path",
            "selected_planner": "pct",
            "fallback_used": False,
            "environment": {
                "ros2_executable": "",
                "ros_distro": "",
                "diagnostic_commands": ["ros2 pkg executables local_planner"],
            },
        },
    )
    summary = _summary(failed=["native_pct_mujoco"])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["native_pct_mujoco"]["runtime_dataflow"]
    assert flow["schema_detected"] == "native_pct_mujoco"
    assert flow["edge_status"]["pct_planner_runtime"] is True
    assert flow["edge_status"]["pct_optimizer_mode"] is True
    assert flow["edge_status"]["ros2_runtime"] is False
    assert flow["primary_blocker"] == "ros2_runtime"
    assert flow["claim_boundary"] == "ros2_runtime_unavailable"
    assert flow["edge_evidence"]["ros2_runtime"]["diagnostic_commands"] == ["ros2 pkg executables local_planner"]


def test_dimos_gap_report_attaches_pct_saved_map_navigation_dataflow(
    tmp_path: Path,
):
    wrapper = _write_json(
        tmp_path / "pct_saved_map_navigation" / "report.json",
        {
            "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "tomogram": str(tmp_path / "same_source_map" / "tomogram.pickle"),
            "scene_xml": str(tmp_path / "same_source_map" / "world.xml"),
            "map_metadata": str(tmp_path / "same_source_map" / "metadata.json"),
            "relocalize_report": str(tmp_path / "relocalize" / "report.json"),
            "source_report": str(tmp_path / "source.json"),
            "deliverable_contract": {
                "checks": {"same_source_map_artifact": True},
            },
            "map_artifacts": _same_source_map_artifacts("pct_saved_map"),
            "same_source_hash_identity": _same_source_hash_identity("pct_saved_map"),
            "relocalization": {
                "ok": True,
                "latest_health_state": "LOCKED",
                "saved_map_cloud_points_latest": 2500,
            },
            "plan_preview": {
                "ok": True,
                "selected_planner": "pct",
                "fallback_reason": "",
                "path_count": 8,
                "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
                "pct_optimizer_enabled": False,
                "pct_planner_path_mode": "astar_raw_path",
            },
            "native_gate": {
                "schema_version": "lingtu.native_pct_mujoco_gate.v1",
                "ok": True,
                "global_planner_source": "source_report/pct_tomogram",
                "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
                "pct_planner_runtime_ok": True,
                "pct_path_count": 8,
                "selected_planner": "pct",
                "fallback_used": False,
                "path_count": 2,
                "max_path_poses": 5,
                "cmd_count_nonzero": 2,
                "moved_m": 0.4,
                "reached_goal": True,
            },
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["pct_saved_map_navigation"]["path"] = str(wrapper)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["pct_saved_map_navigation"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["schema_detected"] == "pct_saved_map_navigation"
    assert flow["edge_status"]["saved_map_relocalization"] is True
    assert flow["edge_status"]["pct_plan_preview"] is True
    assert flow["edge_evidence"]["pct_plan_preview"]["pct_planner_path_mode"] == "astar_raw_path"
    assert flow["edge_evidence"]["pct_plan_preview"]["pct_optimizer_enabled"] is False
    assert flow["edge_status"]["same_source_saved_map_artifacts"] is True
    assert flow["edge_status"]["pct_planner_runtime"] is True
    assert flow["edge_status"]["native_path_follower_to_cmd_vel"] is True
    assert flow["edge_status"]["native_cmd_vel_to_mujoco_motion"] is True
    assert flow["same_source_provenance"]["ok"] is True
    assert flow["same_source_provenance"]["same_source_hash_identity_ok"] is True
    assert flow["primary_blocker"] == ""
    assert flow["source_gate_report"] == str(wrapper)
    assert flow["source_report"] == str(wrapper)


def test_dimos_gap_report_pct_saved_map_navigation_blocks_on_relocalization(
    tmp_path: Path,
):
    wrapper = _write_json(
        tmp_path / "pct_saved_map_navigation" / "report.json",
        {
            "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
            "ok": False,
            "relocalization": {
                "ok": False,
                "latest_health_state": "LOST",
                "saved_map_cloud_points_latest": 0,
            },
            "plan_preview": {
                "ok": True,
                "selected_planner": "pct",
                "fallback_reason": "",
                "path_count": 8,
                "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            },
            "native_gate": {
                "schema_version": "lingtu.native_pct_mujoco_gate.v1",
                "ok": True,
                "global_planner_source": "source_report/pct_tomogram",
                "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
                "pct_planner_runtime_ok": True,
                "pct_path_count": 8,
                "selected_planner": "pct",
                "fallback_used": False,
                "path_count": 2,
                "max_path_poses": 5,
                "cmd_count_nonzero": 2,
                "moved_m": 0.4,
                "reached_goal": True,
            },
        },
    )
    summary = _summary(failed=["pct_saved_map_navigation"])
    summary["gates"]["pct_saved_map_navigation"]["path"] = str(wrapper)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["pct_saved_map_navigation"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["schema_detected"] == "pct_saved_map_navigation"
    assert flow["edge_status"]["saved_map_relocalization"] is False
    assert flow["edge_status"]["pct_plan_preview"] is True
    assert flow["primary_blocker"] == "saved_map_relocalization"


def test_dimos_gap_report_pct_saved_map_navigation_requires_hash_identity(
    tmp_path: Path,
):
    payload = _pct_saved_map_dataflow_report()
    payload.pop("same_source_hash_identity")
    wrapper = _write_json(
        tmp_path / "pct_saved_map_navigation" / "report.json",
        payload,
    )
    summary = _summary(failed=["pct_saved_map_navigation"])
    summary["gates"]["pct_saved_map_navigation"]["path"] = str(wrapper)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["pct_saved_map_navigation"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["edge_status"]["saved_map_relocalization"] is True
    assert flow["edge_status"]["pct_plan_preview"] is True
    assert flow["edge_status"]["same_source_saved_map_artifacts"] is False
    assert flow["primary_blocker"] == "same_source_saved_map_artifacts"
    assert flow["same_source_provenance"]["ok"] is True
    assert flow["same_source_provenance"]["same_source_hash_identity_ok"] is False


def test_dimos_gap_report_attaches_saved_map_relocalize_dataflow(
    tmp_path: Path,
):
    relocalize = _write_json(
        tmp_path / "saved_map_relocalize" / "report.json",
        _saved_map_relocalize_dataflow_report(),
    )
    summary = _summary(failed=[])
    summary["gates"]["saved_map_relocalize"]["path"] = str(relocalize)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["saved_map_relocalize"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["schema_detected"] == "saved_map_relocalize"
    assert flow["edge_status"]["same_source_map_metadata_contract"] is True
    assert flow["edge_status"]["live_fastlio_feed"] is True
    assert flow["edge_status"]["relocalize_service"] is True
    assert flow["edge_status"]["saved_map_cloud"] is True
    assert flow["edge_status"]["localizer_health_lock"] is True
    assert flow["edge_status"]["map_to_odom_correction"] is True
    assert flow["primary_blocker"] == ""


def test_dimos_gap_report_saved_map_relocalize_blocks_on_metadata_contract(
    tmp_path: Path,
):
    payload = _saved_map_relocalize_dataflow_report()
    payload["map_metadata_contract"]["ok"] = False
    payload["map_metadata_contract"]["checks"]["map_pcd_sha256_matches_file"] = False
    relocalize = _write_json(
        tmp_path / "saved_map_relocalize" / "report.json",
        payload,
    )
    summary = _summary(failed=["saved_map_relocalize"])
    summary["gates"]["saved_map_relocalize"]["path"] = str(relocalize)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["saved_map_relocalize"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["edge_status"]["same_source_map_metadata_contract"] is False
    assert flow["edge_status"]["live_fastlio_feed"] is True
    assert flow["primary_blocker"] == "same_source_map_metadata_contract"


def test_dimos_gap_report_attaches_dynamic_obstacle_dataflow(tmp_path: Path):
    dynamic = _write_json(
        tmp_path / "dynamic_obstacle_local_planner" / "report.json",
        _dynamic_obstacle_dataflow_report(),
    )
    summary = _summary(failed=[])
    summary["gates"]["dynamic_obstacle_local_planner"]["path"] = str(dynamic)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["dynamic_obstacle_local_planner"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["schema_detected"] == "dynamic_obstacle_local_planner"
    assert flow["edge_status"]["local_planner_backend"] is True
    assert flow["edge_status"]["dynamic_obstacle_phase_matrix"] is True
    assert flow["edge_status"]["dynamic_replan_response"] is True
    assert flow["edge_status"]["obstacle_clearance"] is True
    assert flow["edge_status"]["clear_path_recovery"] is True
    assert flow["edge_status"]["non_motion_claim_boundary"] is True
    assert flow["primary_blocker"] == ""


def test_dimos_gap_report_dynamic_obstacle_blocks_on_backend(
    tmp_path: Path,
):
    payload = _dynamic_obstacle_dataflow_report()
    payload["backend_actual"] = "cmu_py"
    payload["native_backend_used"] = False
    payload["algorithm_backends"]["local_planner"]["backend_actual"] = "cmu_py"
    payload["algorithm_backends"]["local_planner"]["degraded"] = True
    payload["algorithm_backends"]["local_planner"]["native_backend_used"] = False
    dynamic = _write_json(
        tmp_path / "dynamic_obstacle_local_planner" / "report.json",
        payload,
    )
    summary = _summary(failed=["dynamic_obstacle_local_planner"])
    summary["gates"]["dynamic_obstacle_local_planner"]["path"] = str(dynamic)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    row = rows["dynamic_obstacle_local_planner"]
    flow = row["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["edge_status"]["local_planner_backend"] is False
    assert flow["edge_status"]["dynamic_obstacle_phase_matrix"] is True
    assert flow["primary_blocker"] == "local_planner_backend"
    assert row["runtime_dataflow_blocker"] == "local_planner_backend"
    assert row["runtime_dataflow_failed_edges"] == ["local_planner_backend"]
    assert row["evidence_blockers"][0]["source"] == "runtime_dataflow"
    assert row["evidence_blockers"][0]["blocker"] == "local_planner_backend"
    next_steps = {step["gate"]: step for step in report["next_steps"]}
    assert next_steps["dynamic_obstacle_local_planner"]["runtime_dataflow_blocker"] == "local_planner_backend"
    assert next_steps["dynamic_obstacle_local_planner"]["runtime_dataflow_failed_edges"] == ["local_planner_backend"]


def test_dimos_gap_report_attaches_gazebo_runtime_dataflow(tmp_path: Path):
    gazebo = _write_json(
        tmp_path / "gazebo_runtime" / "report.json",
        _gazebo_runtime_dataflow_report(),
    )
    summary = _summary(failed=[])
    summary["gates"]["gazebo_runtime"]["path"] = str(gazebo)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["gazebo_runtime"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["schema_detected"] == "gazebo_runtime"
    assert flow["edge_status"]["gazebo_tf_topic_contract"] is True
    assert flow["edge_status"]["nav_loop_closed"] is True
    assert flow["edge_status"]["frontier_exploration_closed"] is True
    assert flow["edge_status"]["frontier_no_gain_stall"] is True
    assert flow["edge_status"]["cumulative_map_retention"] is True
    assert flow["edge_status"]["tare_exploration_contract"] is True
    assert flow["edge_status"]["non_motion_claim_boundary"] is True
    assert flow["primary_blocker"] == ""


def test_dimos_gap_report_gazebo_runtime_blocks_on_fake_publishers(
    tmp_path: Path,
):
    payload = _gazebo_runtime_dataflow_report()
    payload["ok"] = False
    payload["nav_loop"]["publisher_contract"] = {
        "ok": False,
        "topics": {
            "/nav/cmd_vel": {
                "ok": False,
                "publishers": ["/fake_cmd_vel_replayer"],
                "disallowed_node_names": ["fake_cmd_vel_replayer"],
            },
        },
        "errors": ["/nav/cmd_vel has disallowed publishers: fake_cmd_vel_replayer"],
    }
    gazebo = _write_json(
        tmp_path / "gazebo_runtime" / "report.json",
        payload,
    )
    summary = _summary(failed=["gazebo_runtime"])
    summary["gates"]["gazebo_runtime"]["path"] = str(gazebo)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["gazebo_runtime"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["edge_status"]["gazebo_tf_topic_contract"] is True
    assert flow["edge_status"]["nav_loop_closed"] is False
    assert flow["primary_blocker"] == "nav_loop_closed"


def test_dimos_gap_report_gazebo_runtime_blocks_on_frontier_stall_window(
    tmp_path: Path,
):
    payload = _gazebo_runtime_dataflow_report()
    payload["ok"] = False
    payload["frontier_exploration"]["frontier_no_gain_stall"] = {
        "checked": True,
        "ok": False,
        "mode": "post_pass_observation",
        "stop_reason": "pass_condition_met",
        "required_observation_s": 5.0,
        "observed_s": 0.2,
    }
    gazebo = _write_json(
        tmp_path / "gazebo_runtime" / "report.json",
        payload,
    )
    summary = _summary(failed=["gazebo_runtime"])
    summary["gates"]["gazebo_runtime"]["path"] = str(gazebo)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["gazebo_runtime"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["edge_status"]["frontier_exploration_closed"] is True
    assert flow["edge_status"]["frontier_no_gain_stall"] is False
    assert flow["primary_blocker"] == "frontier_no_gain_stall"


def test_dimos_gap_report_attaches_large_terrain_dataflow(tmp_path: Path):
    large = _write_json(
        tmp_path / "large_terrain" / "report.json",
        _large_terrain_dataflow_report(),
    )
    summary = _summary(failed=[])
    summary["gates"]["large_terrain"]["path"] = str(large)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["large_terrain"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is True
    assert flow["schema_detected"] == "large_terrain"
    assert flow["edge_status"]["pct_planner_runtime"] is True
    assert flow["edge_status"]["same_source_large_terrain_assets"] is True
    assert flow["edge_status"]["large_terrain_route_safety"] is True
    assert flow["edge_status"]["terrain_gate_constraints"] is True
    assert flow["edge_status"]["non_motion_claim_boundary"] is True
    assert flow["same_source_provenance"]["ok"] is True
    runtime_evidence = flow["edge_evidence"]["pct_planner_runtime"]
    assert runtime_evidence["pct_planner_runtime_ok"] is True
    assert runtime_evidence["pct_planner_runtime"] == {
        "runtime": "rust_process",
        "ok": True,
    }
    assert "native_runtime" not in runtime_evidence
    assert "native_pct_plan_count" not in runtime_evidence
    assert flow["primary_blocker"] == ""


def test_dimos_gap_report_large_terrain_requires_same_source_assets(
    tmp_path: Path,
):
    large = _write_json(
        tmp_path / "large_terrain" / "report.json",
        _large_terrain_dataflow_report(with_same_source=False),
    )
    summary = _summary(failed=["large_terrain"])
    summary["gates"]["large_terrain"]["path"] = str(large)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    rows = {row["gate"]: row for row in report["gap_matrix"]}
    flow = rows["large_terrain"]["runtime_dataflow"]
    assert flow["checked"] is True
    assert flow["ok"] is False
    assert flow["edge_status"]["pct_planner_runtime"] is True
    assert flow["edge_status"]["same_source_large_terrain_assets"] is False
    assert flow["primary_blocker"] == "same_source_large_terrain_assets"


def test_dimos_gap_report_cross_gate_requires_fastlio_live_dataflow(
    tmp_path: Path,
):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        _native_pct_dataflow_report(),
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    for gate in (
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ):
        summary["gates"][gate]["path"] = str(tmp_path / gate / "missing.json")
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    chain = report["runtime_dataflow"]["cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
    assert chain["checked"] is True
    assert chain["ok"] is False
    assert chain["native_pct_gate"]["ok"] is True
    assert "Fast-LIO live dataflow missing" in chain["blockers"]
    assert (
        report["runtime_dataflow"]["failing_cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
        == "Fast-LIO live dataflow missing"
    )
    assert report["lingtu_readiness"]["ok"] is False
    assert report["lingtu_readiness"]["runtime_dataflow_ok"] is False
    assert report["lingtu_readiness"]["cross_gate_failures"] == ["pct_mujoco_and_fastlio_live"]
    assert report["lingtu_readiness"]["runtime_dataflow_complete"] is False
    assert "runtime dataflow evidence is complete" in report["lingtu_readiness"]["stop_condition"]


def test_dimos_gap_report_cross_gate_accepts_native_and_fastlio_live_dataflow(
    tmp_path: Path,
):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        _native_pct_dataflow_report(),
    )
    fastlio = _write_json(
        tmp_path / "fastlio2_dynamic_inspection" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_planner": "pct",
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "last_plan_report": {"selected_planner": "pct"},
            },
            "deliverable_contract": {"checks": {"same_source_map_artifact": True}},
            "map_artifacts": {
                "ok": True,
                "source_contract": {
                    "same_source_pcd": True,
                    "same_source_tomogram": True,
                },
                "assets": {
                    "map_pcd": {
                        "sha256": "map-sha-123",
                        "point_count": 128,
                    },
                    "tomogram": {
                        "sha256": "tomogram-sha-123",
                        "source_map_sha256": "map-sha-123",
                    },
                },
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    saved_map = _write_json(
        tmp_path / "pct_saved_map_navigation" / "report.json",
        _pct_saved_map_dataflow_report(),
    )
    dynamic = _write_json(
        tmp_path / "dynamic_obstacle_local_planner" / "report.json",
        _dynamic_obstacle_dataflow_report(),
    )
    gazebo = _write_json(
        tmp_path / "gazebo_runtime" / "report.json",
        _gazebo_runtime_dataflow_report(),
    )
    large_terrain = _write_json(
        tmp_path / "large_terrain" / "report.json",
        _large_terrain_dataflow_report(),
    )
    relocalize = _write_json(
        tmp_path / "saved_map_relocalize" / "report.json",
        _saved_map_relocalize_dataflow_report(),
    )
    summary = _summary(failed=[])
    summary["gates"]["large_terrain"]["path"] = str(large_terrain)
    summary["gates"]["gazebo_runtime"]["path"] = str(gazebo)
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary["gates"]["dynamic_obstacle_local_planner"]["path"] = str(dynamic)
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = str(fastlio)
    summary["gates"]["moving_obstacle_sweep"]["path"] = str(fastlio)
    summary["gates"]["large_loop_closure"]["path"] = str(fastlio)
    summary["gates"]["saved_map_relocalize"]["path"] = str(relocalize)
    summary["gates"]["pct_saved_map_navigation"]["path"] = str(saved_map)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    chain = report["runtime_dataflow"]["cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
    assert chain["checked"] is True
    assert chain["ok"] is True
    assert chain["selected_fastlio_gate"] == "fastlio2_dynamic_inspection"
    assert chain["native_pct_gate"]["ok"] is True
    assert chain["fastlio_live_gates"]["fastlio2_dynamic_inspection"]["ok"] is True
    assert chain["fastlio_live_gates"]["fastlio2_dynamic_inspection"]["pct_provenance"]["ok"] is True
    assert chain["fastlio_live_gates"]["fastlio2_dynamic_inspection"]["same_source_provenance"]["ok"] is True
    assert chain["same_run_proven"] is True
    assert chain["claim_boundary"] == "same_run_pct_fastlio_live"
    assert report["lingtu_readiness"]["ok"] is True
    assert report["lingtu_readiness"]["runtime_dataflow_complete"] is True
    assert report["lingtu_readiness"]["runtime_dataflow_ok"] is True
    assert report["lingtu_readiness"]["cross_gate_failures"] == []


def test_dimos_gap_report_cross_gate_rejects_fastlio_without_same_run_pct(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(dimos_gap_report, "ROOT", tmp_path)
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        _native_pct_dataflow_report(with_same_source=True),
    )
    fastlio = _write_json(
        tmp_path / "fastlio2_dynamic_inspection" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = str(fastlio)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    chain = report["runtime_dataflow"]["cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
    assert chain["checked"] is True
    assert chain["ok"] is False
    assert chain["same_run_proven"] is False
    assert chain["fastlio_live_gates"]["fastlio2_dynamic_inspection"]["pct_provenance"]["ok"] is False
    assert "Fast-LIO live dataflow did not prove PCT planner in the same run" in chain["blockers"]
    assert report["lingtu_readiness"]["ok"] is False
    assert report["lingtu_readiness"]["runtime_dataflow_ok"] is False
    assert report["lingtu_readiness"]["cross_gate_failures"] == ["pct_mujoco_and_fastlio_live"]


def test_dimos_gap_report_cross_gate_rejects_fastlio_without_same_source(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(dimos_gap_report, "ROOT", tmp_path)
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        _native_pct_dataflow_report(with_same_source=True),
    )
    fastlio = _write_json(
        tmp_path / "fastlio2_dynamic_inspection" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_planner": "pct",
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "last_plan_report": {"selected_planner": "pct"},
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = str(fastlio)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    chain = report["runtime_dataflow"]["cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
    assert chain["ok"] is False
    assert chain["same_run_proven"] is False
    assert chain["fastlio_live_gates"]["fastlio2_dynamic_inspection"]["same_source_provenance"]["ok"] is False
    assert "Fast-LIO live dataflow did not prove same-source map artifacts" in chain["blockers"]


def test_dimos_gap_report_cross_gate_rejects_same_source_flags_without_artifact_sha(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(dimos_gap_report, "ROOT", tmp_path)
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        _native_pct_dataflow_report(with_same_source=True),
    )
    fastlio = _write_json(
        tmp_path / "fastlio2_dynamic_inspection" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_planner": "pct",
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "last_plan_report": {"selected_planner": "pct"},
            },
            "deliverable_contract": {"checks": {"same_source_map_artifact": True}},
            "map_artifacts": {
                "ok": True,
                "source_contract": {
                    "same_source_pcd": True,
                    "same_source_tomogram": True,
                },
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = str(fastlio)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    chain = report["runtime_dataflow"]["cross_gate_chains"]["pct_mujoco_and_fastlio_live"]
    provenance = chain["fastlio_live_gates"]["fastlio2_dynamic_inspection"]["same_source_provenance"]
    assert chain["ok"] is False
    assert provenance["ok"] is False
    assert provenance["reason"] == "map_pcd.sha256 missing"
    assert "Fast-LIO live dataflow did not prove same-source map artifacts" in chain["blockers"]


def test_dimos_gap_report_shell_plan_exposes_cross_gate_boundary(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": True,
            "global_planner_source": "source_report/pct_tomogram",
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            "pct_planner_runtime_ok": True,
            "pct_path_count": 8,
            "selected_planner": "pct",
            "fallback_used": False,
            "path_count": 2,
            "max_path_poses": 5,
            "cmd_count_nonzero": 2,
            "moved_m": 0.4,
            "reached_goal": True,
        },
    )
    fastlio = _write_json(
        tmp_path / "fastlio2_dynamic_inspection" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = str(fastlio)
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )

    text = dimos_gap_report._shell_plan(report)

    assert "# Readiness ok: false" in text
    assert "# Runtime dataflow checked: true" in text
    assert "# Runtime dataflow ok: false" in text
    assert "# Cross-gate failure: pct_mujoco_and_fastlio_live" in text
    assert "# Chain pct_mujoco_and_fastlio_live: ok=false" in text
    assert "same_run_proven=false" in text
    assert "cross_gate_evidence_not_single_run_fusion" in text


def test_dimos_gap_report_cli_shell_enables_dataflow_by_default(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": True,
            "global_planner_source": "source_report/pct_tomogram",
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            "pct_planner_runtime_ok": True,
            "pct_path_count": 8,
            "selected_planner": "pct",
            "fallback_used": False,
            "path_count": 2,
            "max_path_poses": 5,
            "cmd_count_nonzero": 2,
            "moved_m": 0.4,
            "reached_goal": True,
        },
    )
    fastlio = _write_json(
        tmp_path / "fastlio2_dynamic_inspection" / "report.json",
        {
            "schema_version": "lingtu.mujoco_fastlio2_live_gate.v2",
            "ok": True,
            "outputs": {
                "fastlio2_cloud_registered": 2,
                "fastlio2_odometry": 2,
                "nav_odometry": 2,
                "nav_registered_cloud": 2,
                "nav_cmd_vel": 2,
                "nav_cmd_vel_nonzero": 2,
            },
            "lingtu_inspection": {
                "global_path_count": 1,
                "local_path_count": 1,
                "successful_navigation_goal_count": 1,
                "min_required_checkpoints": 1,
            },
            "fastlio2_motion_consistency": {"ok": True},
            "fastlio2_z_consistency": {"ok": True},
            "fastlio2_yaw_consistency": {"ok": True},
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    summary["gates"]["fastlio2_dynamic_inspection"]["path"] = str(fastlio)
    summary_path = _write_json(tmp_path / "summary.json", summary)
    out = tmp_path / "gap.sh"

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_gap_report.py",
            "--summary",
            str(summary_path),
            "--format",
            "shell",
            "--json-out",
            str(out),
        ],
        cwd=Path(__file__).resolve().parents[2],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 1
    text = out.read_text(encoding="utf-8")
    assert "# Runtime dataflow checked: true" in text
    assert "# Runtime dataflow complete: false" in text
    assert "# Runtime dataflow gate failure:" in text
    assert "# Cross-gate failure: pct_mujoco_and_fastlio_live" in text


def test_dimos_gap_report_markdown_includes_cross_gate_chains(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_pct_mujoco" / "report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": True,
            "global_planner_source": "source_report/pct_tomogram",
            "pct_planner_runtime": {"runtime": "rust_process", "ok": True},
            "pct_planner_runtime_ok": True,
            "pct_path_count": 8,
            "selected_planner": "pct",
            "fallback_used": False,
            "path_count": 2,
            "max_path_poses": 5,
            "cmd_count_nonzero": 2,
            "cmd_samples": [{"linear_x": 0.1}],
            "moved_m": 0.4,
            "reached_goal": True,
        },
    )
    summary = _summary(failed=[])
    summary["gates"]["native_pct_mujoco"]["path"] = str(native)
    for gate in (
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ):
        summary["gates"][gate]["path"] = str(tmp_path / gate / "missing.json")
    summary_path = _write_json(tmp_path / "summary.json", summary)

    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        include_dataflow=True,
    )
    text = dimos_gap_report._markdown_table(report)

    assert "### Cross-Gate Chains" in text
    assert "`pct_mujoco_and_fastlio_live`" in text
    assert "Fast-LIO live dataflow missing" in text
    assert "| `pct_mujoco_and_fastlio_live` | False | `` | False |" in text


def test_dimos_gap_report_shell_plan_comments_blocked_gate_commands(
    tmp_path: Path,
):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["native_pct_mujoco"]),
    )
    out = tmp_path / "gap.sh"

    def fake_host_preflight(*, required):
        return {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "current_host": {"platform_system": "Windows", "python_tag": "py313"},
            "runnable_gates": [],
            "blocked_gates": ["native_pct_mujoco"],
            "gates": {
                "native_pct_mujoco": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": ["PCT planner runtime unavailable"],
                    "checks": {
                        "pct_planner_runtime": {
                            "ok": False,
                            "blocker": "PCT planner runtime unavailable",
                        }
                    },
                }
            },
        }

    preflight_path = _write_json(
        tmp_path / "host_preflight.json",
        fake_host_preflight(required=["native_pct_mujoco"]),
    )

    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/dimos_gap_report.py",
            "--summary",
            str(summary_path),
            "--host-preflight-report",
            str(preflight_path),
            "--format",
            "shell",
            "--json-out",
            str(out),
        ],
        cwd=Path(__file__).resolve().parents[2],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 1
    text = out.read_text(encoding="utf-8")
    assert text.startswith("#!/usr/bin/env bash")
    assert "# Phase: linux_sim_closure [target_host]" in text
    assert "bash sim/scripts/run_dimos_linux_closure.sh --dry-run" in text
    assert 'test "$(uname -s)" = Linux' in text
    assert "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-75}" in text
    assert "dimos_gap_report.py --include-dataflow --host-preflight-report" in text
    assert "# Phase: blocked_gate_commands [blocked]" in text
    assert "# Order: dimos_dependency_order" in text
    assert "# Gate: native_pct_mujoco priority=p1" in text
    assert "# Expected report: artifacts/server_sim_closure/native_pct_mujoco/report.json" in text
    assert "# Host preflight ok: false" in text
    assert "# Host failed checks: pct_planner_runtime" in text
    assert "# Host blocker:" in text
    assert "PCT planner runtime unavailable" in text
    assert "# BLOCKED: run native_pct_mujoco" in text
    assert "\nrun native_pct_mujoco" not in text


def test_dimos_gap_report_shell_plan_emits_host_setup_diagnostics(
    tmp_path: Path,
    monkeypatch,
):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["native_pct_mujoco"]),
    )

    def fake_host_preflight(*, required):
        return {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "current_host": {"platform_system": "Windows", "python_tag": "py313"},
            "runnable_gates": [],
            "blocked_gates": ["native_pct_mujoco"],
            "gates": {
                "native_pct_mujoco": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": ["PCT planner runtime unavailable"],
                    "checks": {
                        "pct_planner_runtime": {
                            "ok": False,
                            "blocker": "PCT planner runtime unavailable",
                        }
                    },
                }
            },
        }

    monkeypatch.setattr(
        dimos_gap_report.server_sim_closure,
        "host_preflight",
        fake_host_preflight,
    )
    report = dimos_gap_report.build_gap_report(
        summary_path=summary_path,
        host_preflight=True,
    )

    text = dimos_gap_report._shell_plan(report)

    assert "# Phase: host_setup [blocked]" in text
    assert "# Host setup check: pct_planner_runtime" in text
    assert "# Host setup gates: native_pct_mujoco" in text
    assert "pct_runtime_preflight.py --json-out" in text
    assert "pct_runtime_preflight.py --strict --json-out" in text
    assert "\nbash sim/scripts/setup_linux_validation_host.sh\n" not in text
    assert "# BLOCKED: bash sim/scripts/setup_linux_validation_host.sh" not in text
    assert "# Phase: linux_sim_closure [target_host]" in text
    assert "# Host setup check: target_linux_sim_host" in text
    assert "server_sim_closure.py --preset dimos_benchmark --required-only --run-missing" in text
    assert "# Phase: blocked_gate_commands [blocked]" in text
    assert "# BLOCKED: run native_pct_mujoco" in text
    assert "\nrun native_pct_mujoco\n" not in text


def test_dimos_gap_report_shell_plan_labels_runnable_gate_commands(tmp_path: Path):
    summary = _summary(failed=["large_terrain"])
    summary["run_missing_host_preflight"] = {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        **_fresh_host_preflight_contract(),
        "ok": True,
        "current_host": {"platform_system": "Linux", "python_tag": "py310"},
        "required_gate_sequence": list(DIMOS_BENCHMARK_REQUIRED_GATES),
        "runnable_gates": ["large_terrain"],
        "blocked_gates": [],
        "gates": {"large_terrain": {"ok": True, "status": "runnable", "checks": {}}},
    }
    summary_path = _write_json(tmp_path / "summary.json", summary)
    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    text = dimos_gap_report._shell_plan(report)

    assert "# Phase: run_unblocked_gate_commands [pending]" in text
    assert "# Order: dimos_dependency_order" in text
    assert "# Gate: large_terrain priority=p2" in text
    assert "# Expected report: artifacts/server_sim_closure/large_terrain/report.json" in text
    assert "# Host preflight ok: none" not in text
    assert "# BLOCKED: run large_terrain" not in text
    assert "\nrun large_terrain\n" in text


def test_dimos_gap_report_shell_plan_comments_dependency_blocked_gate_commands(
    tmp_path: Path,
):
    failed = [
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
    ]
    summary = _summary(failed=failed)
    summary["run_missing_host_preflight"] = {
        "schema_version": "lingtu.server_sim_host_preflight.v1",
        "execution_mode": "host_preflight_only",
        "ok": False,
        "current_host": {"platform_system": "Windows", "python_tag": "py313"},
        "runnable_gates": ["moving_obstacle_sweep"],
        "blocked_gates": ["fastlio2_dynamic_inspection"],
        "gates": {
            "fastlio2_dynamic_inspection": {
                "ok": False,
                "status": "blocked",
                "blockers": ["ROS 2 Humble environment is not sourced"],
                "checks": {
                    "ros2_humble": {
                        "ok": False,
                        "blocker": "ROS 2 Humble environment is not sourced",
                    }
                },
            },
            "moving_obstacle_sweep": {
                "ok": True,
                "status": "runnable",
                "checks": {},
            },
        },
    }
    summary_path = _write_json(tmp_path / "summary.json", summary)
    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    text = dimos_gap_report._shell_plan(report)

    assert "# Phase: dependency_blocked_gate_commands [blocked]" in text
    assert "# Gate: moving_obstacle_sweep priority=p0" in text
    assert "# Dependency blockers: fastlio2_dynamic_inspection" in text
    assert "# Dependency blocker status: fastlio2_dynamic_inspection=host_blocked" in text
    assert "# BLOCKED: run moving_obstacle_sweep" in text
    assert "\nrun moving_obstacle_sweep\n" not in text


def test_dimos_gap_report_shell_plan_comments_preflight_required_commands(
    tmp_path: Path,
):
    summary_path = _write_json(
        tmp_path / "summary.json",
        _summary(failed=["large_terrain"]),
    )
    report = dimos_gap_report.build_gap_report(summary_path=summary_path)

    text = dimos_gap_report._shell_plan(report)

    assert "# ok_to_run_missing: false" in text
    assert "# Phase: preflight_required_gate_commands [blocked]" in text
    assert "# Gate: large_terrain priority=p2" in text
    assert "# BLOCKED: run large_terrain" in text
    assert "\nrun large_terrain\n" not in text


def test_dimos_gap_report_can_summarize_current_reports_from_empty_root(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(dimos_gap_report.server_sim_closure, "ROOT", tmp_path)

    report = dimos_gap_report.build_gap_report(summary_path=None)

    assert report["lingtu_readiness"]["ok"] is False
    assert report["gap_counts"]["failed"] == len(DIMOS_BENCHMARK_REQUIRED_GATES)
    assert report["gap_matrix"][0]["gate"] == DIMOS_BENCHMARK_REQUIRED_GATES[0]
