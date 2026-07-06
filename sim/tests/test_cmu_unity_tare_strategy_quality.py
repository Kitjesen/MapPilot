from __future__ import annotations

import argparse

from sim.scripts import cmu_unity_runtime_gate


def _args(**overrides):
    values = {
        "min_waypoint_samples": 1,
        "min_unique_waypoints": 0,
        "unique_waypoint_topics": [],
        "min_path_samples": 1,
        "min_path_poses": 2,
        "required_path_topics": [],
        "min_cmd_vel": 0.01,
        "min_cmd_vel_samples": 3,
        "min_odom_delta_m": 0.10,
        "late_window_sec": 120.0,
        "min_late_odom_delta_m": 0.0,
        "min_late_cmd_vel_samples": 0,
        "min_late_path_samples": 0,
        "min_late_map_area_delta_m2": 0.0,
        "allow_flat_late_map_after_total_growth": False,
        "min_map_area_delta_m2": 0.5,
        "min_required_map_topic_area_m2": 0.5,
        "required_map_topics": [],
        "min_scan_samples": 1,
        "required_scan_topics": [],
        "voxel_size": 0.25,
        "require_motion_progress": False,
        "required_progress_topics": [],
        "min_motion_progress_m": 0.20,
        "gateway_start_exploration_session": False,
        "require_planner_diagnostics": False,
        "require_no_planner_fallback": False,
        "require_no_primary_replan": False,
        "require_planner_path_safety": False,
        "require_exploration_navigation_success": False,
        "min_exploration_navigation_successes": 1,
        "require_tare_strategy_quality": False,
        "min_tare_waypoints": 1,
        "min_tare_paths": 1,
        "min_tare_strategy_paths": 0,
        "max_tare_suppressed_waypoint_ratio": 0.75,
        "require_runtime_contract": False,
        "require_frontier_no_gain_stall": False,
        "require_same_source_tomogram": False,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def _safe_command_graph():
    return {
        "topics": {"/cmd_vel": ["/vehicle_simulator"]},
        "publishers": {"/cmd_vel": ["/lingtu_cmu_unity_adapter"]},
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
    }


def _complete_runtime_metrics():
    return {
        "duration_sec": 180.0,
        "waypoints": {
            "/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]},
        },
        "paths": {
            "/nav/global_path": {
                "samples": 1,
                "nonempty_samples": 1,
                "max_poses": 4,
                "frames": ["map"],
            },
        },
        "cmd_vel": {
            "samples": 5,
            "nonzero_samples": 3,
            "max_norm": 0.2,
        },
        "odometry": {
            "/slam/odometry": {"samples": 5, "delta_m": 0.12},
        },
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/slam/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"samples": 2, "area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
        "gateway_exploration_status": {
            "available": True,
            "data": {
                "backend": "tare",
                "exploring": True,
                "tare": {
                    "status": {
                        "started": True,
                        "waypoint_count": 2,
                        "path_count": 2,
                        "strategy_path_count": 1,
                        "navigation_success_count": 1,
                        "navigation_failure_count": 0,
                    },
                    "stats": {
                        "waypoint_count": 2,
                        "path_count": 2,
                        "strategy_path_count": 1,
                        "suppressed_waypoint_count": 0,
                        "suppressed_far_waypoint_count": 0,
                        "last_waypoint_reject_reason": "",
                        "last_strategy_path_reject_reason": "",
                        "navigation_terminal_count": 1,
                        "navigation_success_count": 1,
                        "navigation_failure_count": 0,
                    },
                },
            },
        },
    }


def test_evaluate_report_emits_tare_strategy_quality_when_required():
    report = cmu_unity_runtime_gate.evaluate_report(
        _complete_runtime_metrics(),
        _args(require_tare_strategy_quality=True),
        "73",
    )

    assert report["ok"] is True
    assert report["tare_strategy_quality"]["ok"] is True
    assert report["tare_strategy_quality"]["waypoint_count"] == 2
    assert report["tare_strategy_quality"]["path_count"] == 2
    assert report["tare_strategy_quality"]["strategy_path_count"] == 1


def test_evaluate_report_rejects_tare_strategy_quality_without_paths():
    metrics = _complete_runtime_metrics()
    metrics["gateway_exploration_status"]["data"]["tare"]["stats"]["path_count"] = 0
    metrics["gateway_exploration_status"]["data"]["tare"]["status"]["path_count"] = 0
    metrics["gateway_exploration_status"]["data"]["tare"]["stats"][
        "last_strategy_path_reject_reason"
    ] = "path_too_short"

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_tare_strategy_quality=True),
        "73",
    )

    assert report["ok"] is False
    assert report["tare_strategy_quality"]["ok"] is False
    assert "TARE strategy quality: path_count below threshold" in report["blockers"]
    assert "path_too_short" in report["tare_strategy_quality"]["reject_reasons"]
