from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import argparse
import struct
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace
from urllib.error import HTTPError

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
        "gateway_start_wait_sec": 30.0,
        "require_planner_diagnostics": False,
        "require_no_planner_fallback": False,
        "require_no_primary_replan": False,
        "require_planner_path_safety": False,
        "require_exploration_navigation_success": False,
        "min_exploration_navigation_successes": 1,
        "require_runtime_contract": False,
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
        "duration_sec": 300.0,
        "waypoints": {
            "/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]},
            "/exploration/way_point": {
                "samples": 1,
                "frames": ["map"],
                "last": [1.0, 0.0, 0.75],
            },
        },
        "paths": {
            "/nav/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 4, "frames": ["map"]},
            "/nav/local_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 3, "frames": ["map"]},
        },
        "cmd_vel": {
            "samples": 5,
            "nonzero_samples": 3,
            "max_norm": 0.2,
            "nonzero_times_sec": [10.0, 20.0, 30.0],
        },
        "odometry": {
            "/nav/odometry": {
                "samples": 5,
                "delta_m": 0.12,
                "history": [
                    {"t": 0.0, "xy": [0.0, 0.0]},
                    {"t": 60.0, "xy": [0.12, 0.0]},
                ],
            }
        },
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {
                    "samples": 2,
                    "area_delta_m2": 0.625,
                    "history": [
                        {"t": 0.0, "area_m2": 1.0},
                        {"t": 60.0, "area_m2": 1.625},
                    ],
                },
                "/nav/registered_cloud": {
                    "samples": 2,
                    "area_delta_m2": 0.625,
                    "history": [
                        {"t": 0.0, "area_m2": 1.0},
                        {"t": 60.0, "area_m2": 1.625},
                    ],
                },
                "/nav/map_cloud": {
                    "samples": 2,
                    "area_delta_m2": 0.625,
                    "history": [
                        {"t": 0.0, "area_m2": 1.0},
                        {"t": 60.0, "area_m2": 1.625},
                    ],
                },
                "/nav/terrain_map_ext": {
                    "samples": 2,
                    "area_delta_m2": 0.625,
                    "history": [
                        {"t": 0.0, "area_m2": 1.0},
                        {"t": 60.0, "area_m2": 1.625},
                    ],
                },
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }


def test_runtime_gate_uses_sensor_data_qos_for_pointcloud_topics():
    source = Path(cmu_unity_runtime_gate.__file__).read_text(encoding="utf-8")

    assert "qos_profile_sensor_data" in source
    assert "cloud_qos = qos_profile_sensor_data" in source
    assert "PointCloud2, topic, lambda msg, topic=topic: sampler.on_cloud(topic, msg), cloud_qos" in source


def test_runtime_gate_direct_default_keeps_extended_terrain_smoke_contract():
    assert cmu_unity_runtime_gate.DEFAULT_REQUIRED_MAP_TOPICS == ("/nav/terrain_map_ext",)


def test_pointcloud_xy_parser_uses_field_offsets():
    fields = [
        SimpleNamespace(name="intensity", offset=12),
        SimpleNamespace(name="x", offset=0),
        SimpleNamespace(name="y", offset=4),
    ]
    data = struct.pack("<ffff", 1.0, 2.0, 0.0, 9.0)
    msg = SimpleNamespace(fields=fields, point_step=16, data=data, is_bigendian=False)

    assert list(cmu_unity_runtime_gate._iter_xy_points(msg)) == [(1.0, 2.0)]


def test_runtime_sampler_records_cmd_components_and_motion_progress():
    sampler = cmu_unity_runtime_gate.RuntimeSampler(
        node=None,
        min_cmd_vel=0.01,
        voxel_size=0.25,
    )
    waypoint = SimpleNamespace(
        point=SimpleNamespace(x=2.0, y=0.0, z=0.75),
        header=SimpleNamespace(frame_id="map"),
    )
    odom0 = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))
        )
    )
    odom1 = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(x=0.7, y=0.0))
        )
    )
    cmd = SimpleNamespace(
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.2, y=-0.1),
            angular=SimpleNamespace(z=0.3),
        )
    )

    sampler.on_odom("/nav/odometry", odom0)
    sampler.on_waypoint("/nav/way_point", waypoint)
    sampler.on_cmd_vel(cmd)
    sampler.on_odom("/nav/odometry", odom1)
    report = sampler.report()

    assert report["cmd_vel"]["nonzero_samples"] == 1
    assert report["cmd_vel"]["max_abs_linear_x"] == 0.2
    assert report["cmd_vel"]["max_abs_linear_y"] == 0.1
    assert report["cmd_vel"]["max_abs_angular_z"] == 0.3
    assert report["motion_progress"]["/nav/way_point"]["best_delta_toward_m"] == 0.7


def test_runtime_sampler_counts_distinct_waypoints():
    sampler = cmu_unity_runtime_gate.RuntimeSampler(
        node=None,
        min_cmd_vel=0.01,
        voxel_size=0.25,
    )
    first = SimpleNamespace(
        point=SimpleNamespace(x=2.0, y=0.0, z=0.75),
        header=SimpleNamespace(frame_id="map"),
    )
    same = SimpleNamespace(
        point=SimpleNamespace(x=2.02, y=0.01, z=0.75),
        header=SimpleNamespace(frame_id="map"),
    )
    second = SimpleNamespace(
        point=SimpleNamespace(x=2.4, y=0.2, z=0.75),
        header=SimpleNamespace(frame_id="map"),
    )

    sampler.on_waypoint("/nav/way_point", first)
    sampler.on_waypoint("/nav/way_point", same)
    sampler.on_waypoint("/nav/way_point", second)
    report = sampler.report()

    assert report["waypoints"]["/nav/way_point"]["samples"] == 3
    assert report["waypoints"]["/nav/way_point"]["unique_count"] == 2
    assert report["waypoints"]["/nav/way_point"]["unique_points"] == [
        [2.0, 0.0, 0.75],
        [2.4, 0.2, 0.75],
    ]


def test_runtime_sampler_records_nonempty_path_times():
    sampler = cmu_unity_runtime_gate.RuntimeSampler(
        node=None,
        min_cmd_vel=0.01,
        voxel_size=0.25,
    )
    empty = SimpleNamespace(poses=[], header=SimpleNamespace(frame_id="map"))
    nonempty = SimpleNamespace(
        poses=[SimpleNamespace(), SimpleNamespace()],
        header=SimpleNamespace(frame_id="map"),
    )

    sampler.on_path("/nav/local_path", empty)
    sampler.on_path("/nav/local_path", nonempty)
    report = sampler.report()

    path = report["paths"]["/nav/local_path"]
    assert path["samples"] == 2
    assert path["nonempty_samples"] == 1
    assert path["last_count"] == 2
    assert len(path["nonempty_times_sec"]) == 1


def test_gateway_session_start_conflict_is_idempotent_success(monkeypatch):
    def fake_urlopen(_request, timeout):
        raise HTTPError(
            url="http://127.0.0.1:5050/api/v1/session/start",
            code=409,
            msg="Conflict",
            hdrs=None,
            fp=BytesIO(b'{"ok": false, "reason": "session already active"}'),
        )

    monkeypatch.setattr(cmu_unity_runtime_gate, "urlopen", fake_urlopen)

    result = cmu_unity_runtime_gate._post_gateway_json(
        "http://127.0.0.1:5050",
        "/api/v1/session/start",
        {"mode": "exploring", "slam_profile": "none"},
        1.0,
    )

    assert result["ok"] is True
    assert result["status"] == 409
    assert result["already_active"] is True


def test_gateway_session_start_retries_until_gateway_is_ready(monkeypatch):
    attempts = [
        {"ok": False, "reason": "URLError: connection refused"},
        {"ok": True, "status": 200, "data": {"ok": True}},
    ]

    def fake_post(*_args, **_kwargs):
        return attempts.pop(0)

    monkeypatch.setattr(cmu_unity_runtime_gate, "_post_gateway_json", fake_post)
    monkeypatch.setattr(cmu_unity_runtime_gate.time, "sleep", lambda _seconds: None)

    result = cmu_unity_runtime_gate._post_gateway_json_with_retry(
        "http://127.0.0.1:5050",
        "/api/v1/session/start",
        {"mode": "exploring", "slam_profile": "none"},
        timeout_sec=1.0,
        wait_sec=5.0,
    )

    assert result["ok"] is True
    assert result["attempt_count"] == 2


def test_evaluate_report_treats_existing_gateway_session_as_success():
    metrics = {
        "waypoints": {"/nav/way_point": {"samples": 2, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "paths": {},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.7}},
        "motion_progress": {},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
        "gateway_session_start": {
            "ok": False,
            "reason": "HTTPError: HTTP Error 409: Conflict",
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(gateway_start_exploration_session=True),
        "73",
    )

    assert report["ok"] is True
    assert "gateway exploration session start failed" not in report["blockers"]


def test_evaluate_report_can_require_motion_progress_to_nav_waypoint():
    metrics = {
        "waypoints": {"/nav/way_point": {"samples": 2, "frames": ["map"], "last": [2.0, 0.0, 0.75]}},
        "paths": {},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.7}},
        "motion_progress": {
            "/nav/way_point": {"segments": [], "best_delta_toward_m": 0.35}
        },
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_motion_progress=True, min_motion_progress_m=0.2),
        "73",
    )

    assert report["ok"] is True
    assert report["progress_requirements"]["/nav/way_point"]["ok"] is True


def test_evaluate_report_rejects_motion_away_from_nav_waypoint():
    metrics = {
        "waypoints": {"/nav/way_point": {"samples": 2, "frames": ["map"], "last": [2.0, 0.0, 0.75]}},
        "paths": {},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.7}},
        "motion_progress": {
            "/nav/way_point": {"segments": [], "best_delta_toward_m": 0.0}
        },
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_motion_progress=True, min_motion_progress_m=0.2),
        "73",
    )

    assert report["ok"] is False
    assert "/nav/way_point motion progress below threshold" in report["blockers"]


def test_evaluate_report_rejects_static_late_runtime_activity():
    metrics = _complete_runtime_metrics()
    metrics["duration_sec"] = 300.0
    metrics["cmd_vel"]["nonzero_times_sec"] = [5.0, 10.0, 15.0]
    metrics["odometry"]["/nav/odometry"]["history"] = [
        {"t": 0.0, "xy": [0.0, 0.0]},
        {"t": 120.0, "xy": [1.0, 0.0]},
        {"t": 250.0, "xy": [1.0, 0.0]},
        {"t": 299.0, "xy": [1.03, 0.0]},
    ]
    metrics["cloud_coverage"]["topics"]["/nav/terrain_map_ext"]["history"] = [
        {"t": 0.0, "area_m2": 1.0},
        {"t": 120.0, "area_m2": 20.0},
        {"t": 250.0, "area_m2": 20.0},
        {"t": 299.0, "area_m2": 20.1},
    ]

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            late_window_sec=120.0,
            min_late_odom_delta_m=0.5,
            min_late_cmd_vel_samples=1,
            min_late_map_area_delta_m2=1.0,
        ),
        "73",
    )

    assert report["ok"] is False
    assert "late odom delta below threshold" in report["blockers"]
    assert "late /nav/cmd_vel nonzero samples below threshold" in report["blockers"]
    assert "late map/exploration area delta below threshold" in report["blockers"]
    assert report["late_activity"]["odometry"]["observed_best_delta_m"] < 0.5


def test_evaluate_report_accepts_sustained_late_runtime_activity():
    metrics = _complete_runtime_metrics()
    metrics["duration_sec"] = 300.0
    metrics["cmd_vel"]["nonzero_times_sec"] = [210.0, 230.0, 250.0]
    metrics["odometry"]["/nav/odometry"]["delta_m"] = 1.2
    metrics["odometry"]["/nav/odometry"]["history"] = [
        {"t": 0.0, "xy": [0.0, 0.0]},
        {"t": 190.0, "xy": [0.0, 0.0]},
        {"t": 240.0, "xy": [0.4, 0.0]},
        {"t": 299.0, "xy": [0.9, 0.0]},
    ]
    metrics["cloud_coverage"]["topics"]["/nav/terrain_map_ext"]["history"] = [
        {"t": 0.0, "area_m2": 1.0},
        {"t": 190.0, "area_m2": 10.0},
        {"t": 299.0, "area_m2": 12.0},
    ]

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            late_window_sec=120.0,
            min_late_odom_delta_m=0.5,
            min_late_cmd_vel_samples=1,
            min_late_map_area_delta_m2=1.0,
        ),
        "73",
    )

    assert report["ok"] is True
    assert report["late_activity"]["odometry"]["ok"] is True
    assert report["late_activity"]["cmd_vel"]["ok"] is True
    assert report["late_activity"]["map_growth"]["ok"] is True


def test_evaluate_report_warns_on_flat_late_map_when_motion_and_paths_are_live():
    metrics = _complete_runtime_metrics()
    metrics["duration_sec"] = 300.0
    metrics["paths"] = {
        "/nav/global_path": {
            "samples": 4,
            "nonempty_samples": 4,
            "max_poses": 6,
            "nonempty_times_sec": [40.0, 210.0, 240.0, 295.0],
        },
        "/nav/local_path": {
            "samples": 4,
            "nonempty_samples": 4,
            "max_poses": 4,
            "nonempty_times_sec": [40.0, 210.0, 240.0, 295.0],
        },
    }
    metrics["cmd_vel"]["nonzero_times_sec"] = [210.0, 230.0, 250.0]
    metrics["odometry"]["/nav/odometry"]["delta_m"] = 1.2
    metrics["odometry"]["/nav/odometry"]["history"] = [
        {"t": 0.0, "xy": [0.0, 0.0]},
        {"t": 190.0, "xy": [0.0, 0.0]},
        {"t": 240.0, "xy": [0.4, 0.0]},
        {"t": 299.0, "xy": [0.9, 0.0]},
    ]
    metrics["cloud_coverage"]["best_area_delta_m2"] = 2.0
    metrics["cloud_coverage"]["topics"]["/nav/terrain_map_ext"]["area_delta_m2"] = 2.0
    metrics["cloud_coverage"]["topics"]["/nav/terrain_map_ext"]["history"] = [
        {"t": 0.0, "area_m2": 1.0},
        {"t": 100.0, "area_m2": 3.0},
        {"t": 190.0, "area_m2": 3.0},
        {"t": 299.0, "area_m2": 3.0},
    ]

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            required_path_topics=["/nav/global_path", "/nav/local_path"],
            late_window_sec=120.0,
            min_late_odom_delta_m=0.5,
            min_late_cmd_vel_samples=1,
            min_late_path_samples=1,
            min_late_map_area_delta_m2=1.0,
            allow_flat_late_map_after_total_growth=True,
        ),
        "73",
    )

    assert report["ok"] is True
    assert report["late_activity"]["map_growth"]["ok"] is False
    assert report["late_activity"]["map_growth"]["accepted_flat_after_total_growth"] is True
    assert report["late_activity"]["paths"]["ok"] is True
    assert report["warnings"]
    assert "late map/exploration area delta below threshold" not in report["blockers"]


def test_evaluate_report_passes_complete_runtime_metrics():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {
            "available": True,
            "data": {
                "diagnostics": {
                    "plan_safety_policy": "fallback_astar",
                    "last_plan_report": {
                        "primary_planner": "pct",
                        "selected_planner": "astar",
                        "fallback_reason": "pct path_safety failed",
                        "rejected_plans": [{"planner": "pct"}],
                        "reached_goal": False,
                    },
                }
            },
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is True
    assert report["runtime_executed"] is True
    assert report["simulation_only"] is True
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["cmd_vel_exclusive_to_lingtu"] is True
    assert report["scan_requirements"]["/registered_scan"]["ok"] is True
    assert report["scan_requirements"]["/nav/registered_cloud"]["ok"] is True
    assert report["map_requirements"]["/nav/terrain_map_ext"]["ok"] is True
    assert report["planner_diagnostics"]["available"] is True
    assert report["planner_diagnostics"]["fallback_used"] is True
    assert report["planner_diagnostics"]["selected_planner"] == "astar"


def test_evaluate_report_can_require_tare_navigation_success():
    metrics = _complete_runtime_metrics()
    metrics["gateway_exploration_status"] = {
        "available": True,
        "data": {
            "backend": "tare",
            "exploring": True,
            "tare": {
                "status": {
                    "navigation_terminal_count": 1,
                    "navigation_success_count": 1,
                    "navigation_failure_count": 0,
                    "last_navigation_status": {"state": "SUCCESS"},
                },
                "stats": {},
            },
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_exploration_navigation_success=True),
        "73",
    )

    assert report["ok"] is True
    assert report["tare_navigation"]["success_count"] == 1


def test_evaluate_report_rejects_missing_tare_navigation_success_when_required():
    metrics = _complete_runtime_metrics()
    metrics["gateway_exploration_status"] = {
        "available": True,
        "data": {
            "backend": "tare",
            "exploring": True,
            "tare": {
                "status": {
                    "navigation_terminal_count": 0,
                    "navigation_success_count": 0,
                    "navigation_failure_count": 0,
                },
                "stats": {},
            },
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_exploration_navigation_success=True),
        "73",
    )

    assert report["ok"] is False
    assert "TARE navigation success count below threshold" in report["blockers"]


def test_evaluate_report_defaults_to_extended_lingtu_map_topic_growth():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/registered_scan",
            "best_cells_delta": 100,
            "best_area_delta_m2": 6.25,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 6.25},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 6.25},
                "/nav/terrain_map": {"area_delta_m2": 0.0},
                "/nav/terrain_map_ext": {"area_delta_m2": 1.0},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is True
    assert "/nav/terrain_map" not in report["map_requirements"]
    assert report["map_requirements"]["/nav/terrain_map_ext"]["ok"] is True


def test_evaluate_report_can_require_base_lingtu_map_topic_growth():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/registered_scan",
            "best_cells_delta": 100,
            "best_area_delta_m2": 6.25,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 6.25},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 6.25},
                "/nav/terrain_map": {"area_delta_m2": 0.0},
                "/nav/terrain_map_ext": {"area_delta_m2": 1.0},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(required_map_topics=["/nav/terrain_map"]),
        "73",
    )

    assert report["ok"] is False
    assert report["map_requirements"]["/nav/terrain_map"]["ok"] is False
    assert "/nav/terrain_map area delta below threshold" in report["blockers"]


def test_evaluate_report_accepts_lingtu_nav_waypoint_topic():
    metrics = {
        "waypoints": {"/nav/way_point": {"samples": 2, "frames": ["map"], "last": [2.0, 1.0, 0.75]}},
        "paths": {},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is True
    assert report["waypoints"]["/nav/way_point"]["samples"] == 2


def test_evaluate_report_can_require_unique_nav_waypoints():
    metrics = {
        "waypoints": {
            "/nav/way_point": {
                "samples": 4,
                "unique_count": 2,
                "frames": ["map"],
                "last": [2.0, 1.0, 0.75],
            }
        },
        "paths": {},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(min_unique_waypoints=2, unique_waypoint_topics=["/nav/way_point"]),
        "73",
    )

    assert report["ok"] is True
    assert report["waypoint_unique_requirements"]["/nav/way_point"]["ok"] is True


def test_evaluate_report_rejects_static_waypoint_when_unique_waypoints_required():
    metrics = {
        "waypoints": {
            "/nav/way_point": {
                "samples": 4,
                "unique_count": 1,
                "frames": ["map"],
                "last": [2.0, 1.0, 0.75],
            }
        },
        "paths": {},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(min_unique_waypoints=2, unique_waypoint_topics=["/nav/way_point"]),
        "73",
    )

    assert report["ok"] is False
    assert "/nav/way_point unique waypoint count below threshold" in report["blockers"]


def test_evaluate_report_can_require_global_and_local_path_topics():
    metrics = {
        "waypoints": {"/nav/way_point": {"samples": 2, "frames": ["map"], "last": [2.0, 1.0, 0.75]}},
        "paths": {
            "/nav/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 4},
            "/nav/local_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 3},
        },
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(required_path_topics=["/nav/global_path", "/nav/local_path"]),
        "73",
    )

    assert report["ok"] is True
    assert report["path_requirements"]["/nav/global_path"]["ok"] is True
    assert report["path_requirements"]["/nav/local_path"]["ok"] is True


def test_runtime_gate_subscribes_to_tare_strategy_path_topics():
    assert "/exploration/global_path_full" in cmu_unity_runtime_gate.PATH_TOPICS
    assert "/exploration/global_path" in cmu_unity_runtime_gate.PATH_TOPICS
    assert "/exploration/local_path" in cmu_unity_runtime_gate.PATH_TOPICS


def test_runtime_gate_publishes_lingtu_and_cmu_tare_start_topics():
    assert cmu_unity_runtime_gate.START_TOPICS == ("/exploration/start", "/start_exploration")


def test_evaluate_report_can_require_tare_strategy_path_topics():
    metrics = {
        "waypoints": {
            "/exploration/way_point": {
                "samples": 2,
                "unique_count": 2,
                "frames": ["map"],
                "last": [2.0, 1.0, 0.75],
            }
        },
        "paths": {
            "/exploration/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 6},
            "/exploration/local_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 4},
            "/nav/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 4},
            "/nav/local_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 3},
        },
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
        "published_start_topics": ["/exploration/start", "/start_exploration"],
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            min_unique_waypoints=2,
            unique_waypoint_topics=["/exploration/way_point"],
            required_path_topics=[
                "/exploration/global_path",
                "/exploration/local_path",
                "/nav/global_path",
                "/nav/local_path",
            ],
        ),
        "73",
    )

    assert report["ok"] is True
    assert report["path_requirements"]["/exploration/global_path"]["ok"] is True
    assert report["path_requirements"]["/exploration/local_path"]["ok"] is True
    assert report["published_start_topics"] == ["/exploration/start", "/start_exploration"]


def test_evaluate_report_rejects_one_target_tare_run_that_stalls_late():
    metrics = _complete_runtime_metrics()
    metrics["duration_sec"] = 300.0
    metrics["waypoints"] = {
        "/nav/way_point": {
            "samples": 6,
            "unique_count": 1,
            "frames": ["map"],
            "last": [2.0, 1.0, 0.75],
        }
    }
    metrics["paths"] = {
        "/exploration/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 6},
        "/exploration/local_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 4},
        "/nav/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 4},
        "/nav/local_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 3},
    }
    metrics["cmd_vel"]["nonzero_times_sec"] = [10.0, 20.0, 30.0]
    metrics["odometry"]["/nav/odometry"]["delta_m"] = 1.0
    metrics["odometry"]["/nav/odometry"]["history"] = [
        {"t": 0.0, "xy": [0.0, 0.0]},
        {"t": 120.0, "xy": [1.0, 0.0]},
        {"t": 220.0, "xy": [1.0, 0.0]},
        {"t": 300.0, "xy": [1.0, 0.0]},
    ]
    for topic in (metrics["cloud_coverage"].get("topics") or {}).values():
        topic["area_delta_m2"] = 1.0
        topic["history"] = [
            {"t": 0.0, "area_m2": 1.0},
            {"t": 120.0, "area_m2": 2.0},
            {"t": 220.0, "area_m2": 2.0},
            {"t": 300.0, "area_m2": 2.0},
        ]
    metrics["cloud_coverage"]["best_area_delta_m2"] = 1.0

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            min_unique_waypoints=2,
            unique_waypoint_topics=["/nav/way_point"],
            required_path_topics=[
                "/exploration/global_path",
                "/exploration/local_path",
                "/nav/global_path",
                "/nav/local_path",
            ],
            min_late_odom_delta_m=0.2,
            min_late_cmd_vel_samples=1,
            min_late_map_area_delta_m2=0.2,
        ),
        "73",
    )

    assert report["ok"] is False
    assert "/nav/way_point unique waypoint count below threshold" in report["blockers"]
    assert "late odom delta below threshold" in report["blockers"]
    assert "late /nav/cmd_vel nonzero samples below threshold" in report["blockers"]
    assert "late map/exploration area delta below threshold" in report["blockers"]
    assert report["path_requirements"]["/exploration/global_path"]["ok"] is True
    assert report["late_activity"]["odometry"]["observed_best_delta_m"] == 0.0


def test_evaluate_report_rejects_default_domain_and_missing_motion():
    metrics = {
        "waypoints": {"/way_point": {"samples": 0}},
        "cmd_vel": {"samples": 0, "nonzero_samples": 0, "max_norm": 0.0},
        "odometry": {"/nav/odometry": {"samples": 1, "delta_m": 0.0}},
        "cloud_coverage": {"best_area_delta_m2": 0.0, "topics": {}},
        "hardware_safety": {
            "topics": {"/cmd_vel": ["/thunder_driver"]},
            "blocked_hardware_nodes": [{"topic": "/cmd_vel", "node": "/thunder_driver"}],
        },
        "gateway_navigation_status": {"available": False, "reason": "connection_refused"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "0")

    assert report["ok"] is False
    assert report["cmd_vel_sent_to_hardware"] is True
    assert "ROS_DOMAIN_ID must be non-empty and non-zero" in report["blockers"]
    assert "hardware-looking command subscriber present" in report["blockers"]


def test_evaluate_report_can_require_planner_diagnostics():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "connection_refused"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_planner_diagnostics=True),
        "73",
    )

    assert report["ok"] is False
    assert "planner diagnostics unavailable" in report["blockers"]


def test_evaluate_report_emits_cmu_runtime_contract_evidence():
    metrics = _complete_runtime_metrics()
    metrics["gateway_navigation_status"] = {
        "available": True,
        "data": {
            "diagnostics": {
                "last_plan_report": {
                    "primary_planner": "pct",
                    "selected_planner": "pct",
                    "selected_path_safety": {"ok": True, "blocked_sample_count": 0},
                    "reached_goal": True,
                },
            }
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            require_runtime_contract=True,
            require_planner_diagnostics=True,
            required_path_topics=["/nav/global_path", "/nav/local_path"],
        ),
        "73",
    )

    assert report["ok"] is True
    assert report["runtime_contract"]["name"] == "cmu_unity_external"
    assert report["runtime_contract"]["ok"] is True
    assert report["runtime_contract"]["definition"]["provider"] == "cmu_unity"
    assert report["runtime_contract"]["topic_evidence"]["/nav/cmd_vel"]["nonzero_samples"] == 3
    assert report["runtime_contract"]["topic_evidence"]["/nav/map_cloud"]["ok"] is True
    assert report["runtime_contract"]["topic_evidence"]["/nav/global_path"]["ok"] is True
    assert report["runtime_contract"]["publisher_identity"]["publishers"]["/cmd_vel"] == [
        "/lingtu_cmu_unity_adapter"
    ]


def test_evaluate_report_can_enforce_cmu_runtime_contract():
    metrics = _complete_runtime_metrics()
    metrics["waypoints"].pop("/exploration/way_point")

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_runtime_contract=True),
        "73",
    )

    assert report["ok"] is False
    assert report["runtime_contract"]["ok"] is False
    assert "runtime contract: /exploration/way_point did not satisfy runtime topic evidence" in report["blockers"]


def test_strict_runtime_gate_requires_lingtu_planner_diagnostics(monkeypatch):
    captured = {}

    def fake_run_gate(args):
        captured["require_planner_diagnostics"] = args.require_planner_diagnostics
        captured["require_runtime_contract"] = args.require_runtime_contract
        captured["require_exploration_navigation_success"] = (
            args.require_exploration_navigation_success
        )
        return {
            "schema_version": "lingtu.cmu_unity_runtime_gate.v1",
            "ok": True,
            "runtime_executed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "blockers": [],
        }

    monkeypatch.setattr(cmu_unity_runtime_gate, "run_gate", fake_run_gate)
    monkeypatch.setattr(
        "sys.argv",
        ["cmu_unity_runtime_gate.py", "--strict"],
    )

    assert cmu_unity_runtime_gate.main() == 0
    assert captured["require_planner_diagnostics"] is True
    assert captured["require_runtime_contract"] is True
    assert captured["require_exploration_navigation_success"] is True


def test_evaluate_report_can_reject_planner_fallback_when_required():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {
            "available": True,
            "data": {
                "diagnostics": {
                    "last_plan_report": {
                        "primary_planner": "pct",
                        "selected_planner": "astar",
                        "fallback_reason": "pct path_safety failed",
                    },
                }
            },
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_no_planner_fallback=True),
        "73",
    )

    assert report["ok"] is False
    assert "planner fallback was used" in report["blockers"]


def test_evaluate_report_separates_pct_path_safety_failure_from_fallback():
    metrics = _complete_runtime_metrics()
    metrics["gateway_navigation_status"] = {
        "available": True,
        "data": {
            "diagnostics": {
                "last_plan_report": {
                    "primary_planner": "pct",
                    "selected_planner": "pct",
                    "fallback_reason": "pct path_safety failed (9 blocked samples)",
                    "selected_path_safety": {
                        "ok": False,
                        "blocked_sample_count": 9,
                    },
                },
            }
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(
            require_no_planner_fallback=True,
            require_planner_path_safety=True,
        ),
        "73",
    )

    assert report["planner_diagnostics"]["fallback_used"] is False
    assert report["planner_diagnostics"]["unsafe_primary_rejected"] is True
    assert report["planner_diagnostics"]["path_safety_ok"] is False
    assert report["ok"] is False
    assert "planner fallback was used" not in report["blockers"]
    assert "planner selected path safety failed" in report["blockers"]


def test_evaluate_report_can_reject_pct_primary_replan_when_required():
    metrics = _complete_runtime_metrics()
    metrics["gateway_navigation_status"] = {
        "available": True,
        "data": {
            "diagnostics": {
                "last_plan_report": {
                    "primary_planner": "pct",
                    "selected_planner": "pct",
                    "primary_replan": {
                        "used": True,
                        "reason": "initial_primary_path_safety_failed",
                        "original_goal": [7.0, -3.0, 0.75],
                        "repaired_goal": [4.5, -0.8, 0.75],
                    },
                },
            }
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(
        metrics,
        _args(require_no_primary_replan=True),
        "73",
    )

    assert report["ok"] is False
    assert report["planner_diagnostics"]["primary_replan_used"] is True
    assert "planner primary replan was used" in report["blockers"]


def test_evaluate_report_rejects_gateway_navigation_failure():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {
            "available": True,
            "data": {
                "state": "FAILED",
                "reason_codes": ["mission_failed"],
                "failure_reason": "GlobalPlannerService: planner returned empty path",
                "diagnostics": {
                    "last_plan_report": {
                        "primary_planner": "astar",
                        "selected_planner": "astar",
                    }
                },
            },
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is False
    assert report["navigation_failure"]["failed"] is True
    assert "navigation mission failed" in report["blockers"]


def test_evaluate_report_records_direct_goal_fallback_from_gateway():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {
            "available": True,
            "data": {
                "state": "EXECUTING",
                "mission": {
                    "raw": {
                        "direct_goal_fallback": {
                            "used": True,
                            "reason": "GlobalPlannerService: planner returned empty path",
                            "goal": [4.0, 1.0, 0.75],
                        }
                    }
                },
                "diagnostics": {
                    "last_plan_report": {
                        "primary_planner": "astar",
                        "selected_planner": "astar",
                    }
                },
            },
        },
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is True
    assert report["direct_goal_fallback"]["used"] is True
    assert "empty path" in report["direct_goal_fallback"]["reason"]


def test_evaluate_report_rejects_missing_cmu_scan_topics():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": _safe_command_graph(),
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is False
    assert report["scan_requirements"]["/registered_scan"]["ok"] is False
    assert report["scan_requirements"]["/nav/registered_cloud"]["ok"] is False
    assert "/registered_scan samples below threshold" in report["blockers"]
    assert "/nav/registered_cloud samples below threshold" in report["blockers"]


def test_evaluate_report_rejects_foreign_cmd_vel_publisher():
    metrics = {
        "waypoints": {"/way_point": {"samples": 1, "frames": ["map"], "last": [1.0, 0.0, 0.75]}},
        "cmd_vel": {"samples": 5, "nonzero_samples": 3, "max_norm": 0.2},
        "odometry": {"/nav/odometry": {"samples": 5, "delta_m": 0.12}},
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 10,
            "best_area_delta_m2": 0.625,
            "topics": {
                "/registered_scan": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/registered_cloud": {"samples": 2, "area_delta_m2": 0.625},
                "/nav/terrain_map": {"area_delta_m2": 0.625},
                "/nav/terrain_map_ext": {"area_delta_m2": 0.625},
            },
        },
        "hardware_safety": {
            "topics": {"/cmd_vel": ["/vehicleSimulator"]},
            "publishers": {"/cmd_vel": ["/lingtu_cmu_unity_adapter", "/pathFollower"]},
            "blocked_hardware_nodes": [],
            "unexpected_command_publishers": [{"topic": "/cmd_vel", "node": "/pathFollower"}],
        },
        "gateway_navigation_status": {"available": False, "reason": "gateway_url_disabled"},
    }

    report = cmu_unity_runtime_gate.evaluate_report(metrics, _args(), "73")

    assert report["ok"] is False
    assert report["cmd_vel_exclusive_to_lingtu"] is False
    assert "unexpected /cmd_vel publisher present" in report["blockers"]
