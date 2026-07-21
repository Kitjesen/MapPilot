from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import pytest

pytestmark = [pytest.mark.sim]

from sim.scripts import server_sim_closure
from sim.scripts.mujoco.native_pct_gate import (
    PctRoute,
    _build_safe_follow_path,
    _default_local_planner_path_folder,
    _load_pct_route,
    _local_path_obstacle_evidence,
    _moving_obstacle_boxes,
    _native_node_commands,
    _omni_cart_cmd,
    _path_is_robot_frame,
    _path_window_by_distance,
    _ros_path_to_points,
    _sample_lidar_points,
    _select_local_target,
    _source_map_artifacts,
    _summarize_local_path_obstacle_evidence,
    _trajectory_correctness,
    run_gate,
)


def _mid360_lidar_source() -> dict:
    return {
        "kind": "MuJoCo mj_multiRay with official Livox MID-360 scan pattern",
        "forced_pattern": True,
        "pattern_path": str(server_sim_closure.ROOT / server_sim_closure.MID360_PATTERN_REL),
        "pattern_sha256": server_sim_closure.MID360_PATTERN_SHA256,
        "samples_per_frame": 24000,
        "fallback_n_rays": 64,
        "body": "lidar_link",
    }


def _complete_dimos_native_pct_mujoco_report() -> dict:
    map_sha = "native-map-sha"
    return {
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "reached_goal": True,
        "final_distance_m": 0.4,
        "planner": "pct",
        "primary_planner": "pct",
        "selected_planner": "pct",
        "fallback_used": False,
        "global_planner_source": "source_report/native_pct_tomogram",
        "pct_native_runtime_used": True,
        "pct_runtime_ok": True,
        "pct_path_count": 8,
        "pct_optimizer_enabled": True,
        "pct_optimizer_attempted": True,
        "pct_optimizer_accepted": False,
        "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
        "pct_optimizer_blocked_sample_count": 3,
        "pct_planner_path_mode": "native_astar_raw_path",
        "moved_m": 10.0,
        "frames": {"goal": "map", "cmd_vel": "base_link"},
        "planning_chain": {
            "local_planner": "cmu_ros2_native/localPlanner",
            "path_follower": "cmu_ros2_native/pathFollower",
            "fallback_allowed": False,
        },
        "source_planning_contract": {
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_used": False,
            "path_safety_ok": True,
            "native_runtime_used": True,
            "tomogram_exists": True,
            "tomogram_sha256": "abc123",
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 3,
            "pct_planner_path_mode": "native_astar_raw_path",
        },
        "deliverable_contract": {
            "checks": {"same_source_map_artifact": True},
        },
        "map_artifacts": {
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
                    "sha256": "native-tomogram-sha",
                    "source_map_sha256": map_sha,
                },
            },
        },
        "obstacle_aware": {"enabled": True, "metadata_points": 32},
        "waypoint_safety_filter": {
            "enabled": True,
            "goal_preserved": True,
            "source_path_goal_xy": [2.0, 0.0],
            "follow_path_goal_xy": [2.0, 0.0],
            "goal_shift_m": 0.0,
        },
        "requested_goal_xy": [2.0, 0.0],
        "pct_path_goal_xy": [2.0, 0.0],
        "tracking_goal_xy": [2.0, 0.0],
        "obstacle_clearance": {
            "checked": True,
            "collision": False,
            "min_clearance_m": 0.65,
        },
        "local_path_samples": [{"frame_id": "body", "point_count": 8}],
        "trajectory_quality": {
            "ok": True,
            "p95_lateral_error_m": 0.12,
            "final_progress_ratio": 0.99,
        },
        "video": {"lidar_source": _mid360_lidar_source()},
    }


def _write_source_report(
    tmp_path: Path,
    *,
    native_runtime_used: bool = True,
    native_runtime_ok: bool = True,
    pct_optimizer_enabled: bool | None = False,
    pct_optimizer_attempted: bool | None = None,
    pct_optimizer_accepted: bool | None = None,
    pct_optimizer_reject_reason: str = "",
    pct_optimizer_blocked_sample_count: int = 0,
    pct_planner_path_mode: str = "native_astar_raw_path",
) -> Path:
    scene_xml = tmp_path / "scene.xml"
    scene_xml.write_text("<mujoco><worldbody/></mujoco>\n", encoding="utf-8")
    tomogram = tmp_path / "tomogram.pickle"
    tomogram.write_bytes(b"pct test tomogram")
    map_pcd = tmp_path / "map.pcd"
    map_pcd.write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n0 0 0\n1 0 0\n",
        encoding="ascii",
    )
    map_sha = "source-map-sha"
    tomogram_sha = "source-tomogram-sha"
    metadata = {
        "schema_version": "lingtu.saved_map_artifacts.v1",
        "source_profile": "native_pct_test_assets",
        "data_source": "synthetic_test_geometry",
        "mapping_source": "synthetic_test_geometry_to_map_pcd_and_tomogram",
        "frame_id": "map",
        "artifacts": {
            "map_pcd": {
                "path": str(map_pcd),
                "sha256": map_sha,
                "point_count": 2,
            },
            "tomogram": {
                "path": str(tomogram),
                "sha256": tomogram_sha,
                "source_map_sha256": map_sha,
            },
        },
        "obstacles": [
            {
                "name": "pillar_lower",
                "floor_id": 0,
                "position": [0.85, -1.55, 0.35],
                "half_size": [0.18, 0.18, 0.35],
            }
        ]
    }
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    report = {
        "cases": [
            {
                "route": "same_floor",
                "assets": {
                    "scene_xml": str(scene_xml),
                    "tomogram": str(tomogram),
                    "map_pcd": str(map_pcd),
                    "metadata": str(metadata_path),
                    "start": [-0.4, -2.1, 0.0],
                    "goal": [1.9, -1.25, 0.0],
                },
                "planning": [
                    {
                        "planner": "pct",
                        "planner_class": "PCTPlanner",
                        "native_runtime_used": native_runtime_used,
                        "native_runtime": {"ok": native_runtime_ok},
                        "pct_optimizer_enabled": pct_optimizer_enabled,
                        "pct_optimizer_attempted": pct_optimizer_attempted,
                        "pct_optimizer_accepted": pct_optimizer_accepted,
                        "pct_optimizer_reject_reason": pct_optimizer_reject_reason,
                        "pct_optimizer_blocked_sample_count": pct_optimizer_blocked_sample_count,
                        "pct_planner_path_mode": pct_planner_path_mode,
                        "plan_ms": 12.3,
                        "goal": [1.9, -1.25, 0.0],
                        "path": [
                            [-0.2, -2.2, 0.6],
                            [0.4, -2.0, 0.6],
                            [1.9, -1.25, 0.0],
                        ],
                    }
                ],
            }
        ]
    }
    source = tmp_path / "report.json"
    source.write_text(json.dumps(report), encoding="utf-8")
    return source


def test_load_pct_route_requires_native_runtime(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path, native_runtime_used=False)

    with pytest.raises(ValueError, match="native runtime"):
        _load_pct_route(source, route="same_floor")


def test_load_pct_route_requires_healthy_native_runtime(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path, native_runtime_ok=False)

    with pytest.raises(ValueError, match="native runtime"):
        _load_pct_route(source, route="same_floor")


def test_load_pct_route_requires_pct_path_mode_evidence(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path, pct_optimizer_enabled=None, pct_planner_path_mode="")

    with pytest.raises(ValueError, match="optimizer enabled/disabled mode"):
        _load_pct_route(source, route="same_floor")


def test_load_pct_route_accepts_optimized_trajectory_mode(tmp_path: Path) -> None:
    source = _write_source_report(
        tmp_path,
        pct_optimizer_enabled=True,
        pct_planner_path_mode="optimized_trajectory",
    )

    route = _load_pct_route(source, route="same_floor")

    assert route.plan["pct_optimizer_enabled"] is True
    assert route.plan["pct_planner_path_mode"] == "optimized_trajectory"


def test_load_pct_route_rejects_mismatched_optimizer_path_mode(tmp_path: Path) -> None:
    source = _write_source_report(
        tmp_path,
        pct_optimizer_enabled=True,
        pct_planner_path_mode="native_astar_raw_path",
    )

    with pytest.raises(ValueError, match="recorded optimizer rejection"):
        _load_pct_route(source, route="same_floor")


def test_load_pct_route_accepts_raw_path_after_optimizer_rejection(tmp_path: Path) -> None:
    source = _write_source_report(
        tmp_path,
        pct_optimizer_enabled=True,
        pct_optimizer_attempted=True,
        pct_optimizer_accepted=False,
        pct_optimizer_reject_reason="optimized_trajectory_hard_obstacle",
        pct_optimizer_blocked_sample_count=3,
        pct_planner_path_mode="native_astar_raw_path",
    )

    route = _load_pct_route(source, route="same_floor")

    assert route.plan["pct_optimizer_enabled"] is True
    assert route.plan["pct_optimizer_attempted"] is True
    assert route.plan["pct_optimizer_accepted"] is False
    assert route.plan["pct_optimizer_reject_reason"] == "optimized_trajectory_hard_obstacle"
    assert route.plan["pct_planner_path_mode"] == "native_astar_raw_path"


def test_native_pct_mujoco_report_satisfies_dimos_closure_evaluator() -> None:
    report = _complete_dimos_native_pct_mujoco_report()

    ok, blockers, evidence = server_sim_closure._eval_native_pct_mujoco(report)

    assert ok is True
    assert blockers == []
    assert evidence["planner"] == "pct"
    assert evidence["source_planning_contract"]["native_runtime_used"] is True
    assert evidence["source_planning_contract"]["tomogram_sha256"] == "abc123"
    assert evidence["same_source_artifacts"]["ok"] is True
    assert evidence["clearance_checked"] is True
    assert evidence["local_path_sample_count"] == 1


def test_native_pct_mujoco_evaluator_requires_same_source_artifacts() -> None:
    report = _complete_dimos_native_pct_mujoco_report()
    report.pop("map_artifacts")
    report["deliverable_contract"] = {"checks": {"same_source_map_artifact": False}}

    ok, blockers, evidence = server_sim_closure._eval_native_pct_mujoco(report)

    assert ok is False
    assert any(
        "native_pct_mujoco.map_artifacts.ok is not true" in blocker
        for blocker in blockers
    )
    assert evidence["same_source_artifacts"]["ok"] is False


def test_native_pct_mujoco_evaluator_requires_waypoint_goal_preservation() -> None:
    report = _complete_dimos_native_pct_mujoco_report()
    report["waypoint_safety_filter"]["goal_preserved"] = False
    report["waypoint_safety_filter"]["follow_path_goal_xy"] = [1.0, 0.0]
    report["waypoint_safety_filter"]["goal_shift_m"] = 1.0
    report["tracking_goal_xy"] = [1.0, 0.0]

    ok, blockers, evidence = server_sim_closure._eval_native_pct_mujoco(report)

    assert ok is False
    assert "waypoint_safety_filter.goal_preserved is not true" in blockers
    assert evidence["waypoint_safety_filter"]["goal_preserved"] is False
    assert evidence["tracking_goal_xy"] == [1.0, 0.0]


def test_native_pct_mujoco_evaluator_reports_waypoint_skip_claim_boundary() -> None:
    report = _complete_dimos_native_pct_mujoco_report()
    report.update(
        {
            "ok": False,
            "native_gate_skipped": True,
            "claim_boundary": "waypoint_safety_filter_truncated_pct_path",
            "reached_goal": False,
            "final_distance_m": 99.0,
            "local_path_samples": [],
            "trajectory_quality": {},
            "obstacle_clearance": {},
            "video": {},
        }
    )
    report["obstacle_aware"]["metadata_points"] = 0
    report["obstacle_aware"]["metadata_obstacle_count"] = 1
    report["waypoint_safety_filter"].update(
        {
            "goal_preserved": False,
            "source_path_goal_xy": [2.0, 0.0],
            "follow_path_goal_xy": [1.0, 0.0],
            "goal_shift_m": 1.0,
            "original_count": 3,
            "path_count": 2,
            "skipped_unsafe_waypoint_count": 1,
        }
    )
    report["tracking_goal_xy"] = [1.0, 0.0]

    ok, blockers, evidence = server_sim_closure._eval_native_pct_mujoco(report)

    assert ok is False
    assert blockers == ["waypoint_safety_filter.goal_preserved is not true"]
    assert evidence["claim_boundary"] == "waypoint_safety_filter_truncated_pct_path"
    assert evidence["native_gate_skipped"] is True
    assert evidence["waypoint_safety_filter"]["goal_shift_m"] == 1.0
    assert evidence["tracking_goal_xy"] == [1.0, 0.0]


def test_load_pct_route_extracts_showcase_inputs(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path)

    route = _load_pct_route(source, route="same_floor")

    assert route.route == "same_floor"
    assert route.scene_xml.exists()
    assert route.start == [-0.4, -2.1, 0.55]
    assert route.goal == [1.9, -1.25, 0.0]
    assert len(route.path) == 3
    assert route.plan["planner_class"] == "PCTPlanner"


def test_source_map_artifacts_extracts_same_source_metadata(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path)
    route = _load_pct_route(source, route="same_floor")

    artifacts = _source_map_artifacts(route)

    assert artifacts["ok"] is True
    assert artifacts["source_contract"]["same_source_pcd"] is True
    assert artifacts["source_contract"]["same_source_tomogram"] is True
    assert artifacts["assets"]["map_pcd"]["sha256"] == "source-map-sha"
    assert artifacts["assets"]["map_pcd"]["point_count"] == 2
    assert (
        artifacts["assets"]["tomogram"]["source_map_sha256"]
        == artifacts["assets"]["map_pcd"]["sha256"]
    )


def test_native_pct_mujoco_contract_only_validates_source_report(
    tmp_path: Path,
) -> None:
    source = _write_source_report(tmp_path)
    out = tmp_path / "contract_report.json"
    args = argparse.Namespace(
        source_report=source,
        generate_source_report=False,
        force_generate_source_report=False,
        route="same_floor",
        planner="pct",
        contract_only=True,
        json_out=out,
    )

    report = run_gate(args)

    assert report["ok"] is True
    assert report["execution_mode"] == "contract_only"
    assert report["validation_only"] is True
    assert report["claim_boundary"] == "contract_only_no_ros_mujoco_motion"
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["pct_native_runtime_used"] is True
    assert report["pct_runtime_ok"] is True
    assert report["pct_path_count"] == 3
    assert report["pct_optimizer_enabled"] is False
    assert report["pct_planner_path_mode"] == "native_astar_raw_path"
    assert report["contract_checks"] == {
        "source_report_loads": True,
        "planner_no_fallback": True,
        "pct_native_runtime_used": True,
        "pct_native_runtime_ok": True,
        "pct_optimizer_mode_recorded": True,
        "pct_path_mode_supported": True,
        "pct_path_mode_matches_optimizer": True,
        "pct_optimizer_rejection_recorded": False,
        "tomogram_exists": True,
        "path_safety_ok": True,
        "same_source_map_artifact": True,
    }
    command_generation = report["command_generation"]
    assert command_generation["ok"] is True
    assert command_generation["claim_boundary"] == "command_generation_only_no_process_launch"
    assert command_generation["source_report_fingerprint"]["path"] == str(source)
    assert command_generation["source_report_fingerprint"]["sha256"] == hashlib.sha256(
        source.read_bytes()
    ).hexdigest()
    assert command_generation["commands"]["localPlanner"][:4] == [
        "ros2",
        "run",
        "local_planner",
        "localPlanner",
    ]
    assert command_generation["commands"]["pathFollower"][:4] == [
        "ros2",
        "run",
        "local_planner",
        "pathFollower",
    ]
    assert command_generation["safety_boundary"] == {
        "starts_gateway": False,
        "starts_driver": False,
        "starts_systemd": False,
        "requires_isolated_ros_domain_for_launch": True,
    }
    assert report["deliverable_contract"]["checks"]["same_source_map_artifact"] is True
    assert "Does not create a MuJoCo engine" in " ".join(
        report["validation_limitations"]
    )
    assert json.loads(out.read_text(encoding="utf-8"))["ok"] is True


def test_native_pct_mujoco_contract_only_accepts_optimized_pct_source(
    tmp_path: Path,
) -> None:
    source = _write_source_report(
        tmp_path,
        pct_optimizer_enabled=True,
        pct_planner_path_mode="optimized_trajectory",
    )
    args = argparse.Namespace(
        source_report=source,
        generate_source_report=False,
        force_generate_source_report=False,
        route="same_floor",
        planner="pct",
        contract_only=True,
        json_out=tmp_path / "contract_report.json",
    )

    report = run_gate(args)

    assert report["ok"] is True
    assert report["pct_optimizer_enabled"] is True
    assert report["pct_planner_path_mode"] == "optimized_trajectory"
    assert report["contract_checks"]["pct_path_mode_matches_optimizer"] is True
    assert report["contract_checks"]["pct_optimizer_rejection_recorded"] is False

    closure_ok, closure_blockers, _ = server_sim_closure._eval_native_pct_mujoco(
        report
    )
    assert closure_ok is False
    assert "reached_goal is not true" in closure_blockers
    assert "local_path_samples missing" in closure_blockers


def test_native_pct_mujoco_contract_only_accepts_optimizer_rejected_raw_source(
    tmp_path: Path,
) -> None:
    source = _write_source_report(
        tmp_path,
        pct_optimizer_enabled=True,
        pct_optimizer_attempted=True,
        pct_optimizer_accepted=False,
        pct_optimizer_reject_reason="optimized_trajectory_hard_obstacle",
        pct_optimizer_blocked_sample_count=3,
        pct_planner_path_mode="native_astar_raw_path",
    )
    args = argparse.Namespace(
        source_report=source,
        generate_source_report=False,
        force_generate_source_report=False,
        route="same_floor",
        planner="pct",
        contract_only=True,
        json_out=tmp_path / "contract_report.json",
    )

    report = run_gate(args)

    assert report["ok"] is True
    assert report["pct_optimizer_enabled"] is True
    assert report["pct_planner_path_mode"] == "native_astar_raw_path"
    checks = report["contract_checks"]
    assert checks["pct_path_mode_matches_optimizer"] is True
    assert checks["pct_optimizer_rejection_recorded"] is True
    assert report["source_planning_contract"]["pct_optimizer_reject_reason"] == (
        "optimized_trajectory_hard_obstacle"
    )
    assert report["source_planning_contract"]["pct_optimizer_blocked_sample_count"] == 3


def test_native_pct_mujoco_contract_only_reports_source_blocker(
    tmp_path: Path,
) -> None:
    source = _write_source_report(tmp_path, native_runtime_used=False)
    args = argparse.Namespace(
        source_report=source,
        generate_source_report=False,
        force_generate_source_report=False,
        route="same_floor",
        planner="pct",
        contract_only=True,
        json_out=tmp_path / "contract_report.json",
    )

    report = run_gate(args)

    assert report["ok"] is False
    assert report["execution_mode"] == "contract_only"
    assert report["validation_only"] is True
    assert "native runtime" in report["blockers"][0]
    assert report["contract_checks"]["source_report_loads"] is False


def test_native_pct_mujoco_full_run_rejects_missing_source_artifacts_before_ros(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import native_pct_mujoco_gate as mod

    source = _write_source_report(tmp_path)
    data = json.loads(source.read_text(encoding="utf-8"))
    data["cases"][0]["assets"].pop("metadata")
    source.write_text(json.dumps(data), encoding="utf-8")

    def fail_load_ros_modules():
        raise AssertionError("ROS modules must not load when source artifacts are invalid")

    monkeypatch.setattr(mod, "_load_ros_modules", fail_load_ros_modules)
    out = tmp_path / "native_report.json"
    args = mod._build_parser().parse_args(
        [
            "--source-report",
            str(source),
            "--route",
            "same_floor",
            "--json-out",
            str(out),
        ]
    )

    report = mod.run_gate(args)

    assert report["ok"] is False
    assert report["native_gate_skipped"] is True
    assert report["claim_boundary"] == "pre_native_source_artifact_contract_failed"
    assert any(
        "same-source map/tomogram artifact proof" in blocker
        for blocker in report["blockers"]
    )
    assert json.loads(out.read_text(encoding="utf-8"))["native_gate_skipped"] is True


def test_native_pct_mujoco_full_run_rejects_truncated_waypoint_goal_before_ros(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import native_pct_mujoco_gate as mod

    source = _write_source_report(tmp_path)
    data = json.loads(source.read_text(encoding="utf-8"))
    case = data["cases"][0]
    case["assets"]["start"] = [0.0, 0.0, 0.0]
    case["assets"]["goal"] = [2.0, 0.0, 0.0]
    case["planning"][0]["goal"] = [2.0, 0.0, 0.0]
    case["planning"][0]["path"] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ]
    metadata_path = Path(case["assets"]["metadata"])
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    metadata["obstacles"] = [
        {
            "name": "terminal_box",
            "floor_id": 0,
            "position": [2.0, 0.0, 0.35],
            "half_size": [0.20, 0.20, 0.35],
        }
    ]
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
    source.write_text(json.dumps(data), encoding="utf-8")

    def fail_load_ros_modules():
        raise AssertionError("ROS modules must not load when waypoint goal is truncated")

    monkeypatch.setattr(mod, "_load_ros_modules", fail_load_ros_modules)
    out = tmp_path / "native_report.json"
    args = mod._build_parser().parse_args(
        [
            "--source-report",
            str(source),
            "--route",
            "same_floor",
            "--json-out",
            str(out),
        ]
    )

    report = mod.run_gate(args)

    assert report["ok"] is False
    assert report["native_gate_skipped"] is True
    assert report["claim_boundary"] == "waypoint_safety_filter_truncated_pct_path"
    assert report["blockers"] == ["waypoint_safety_filter.goal_preserved is not true"]
    assert report["waypoint_safety_filter"]["goal_preserved"] is False
    assert report["tracking_goal_xy"] == [1.0, 0.0]
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["claim_boundary"] == "waypoint_safety_filter_truncated_pct_path"


def test_native_pct_mujoco_full_run_writes_blocked_report_when_ros2_unavailable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from sim.scripts import native_pct_mujoco_gate as mod

    source = _write_source_report(tmp_path)

    def fail_load_ros_modules():
        raise RuntimeError("ros2 CLI is unavailable")

    monkeypatch.setattr(mod, "_load_ros_modules", fail_load_ros_modules)
    out = tmp_path / "native_report.json"
    args = mod._build_parser().parse_args(
        [
            "--source-report",
            str(source),
            "--route",
            "same_floor",
            "--json-out",
            str(out),
        ]
    )

    report = mod.run_gate(args)

    assert report["ok"] is False
    assert report["native_gate_skipped"] is True
    assert report["claim_boundary"] == "ros2_runtime_unavailable"
    assert "ros2 CLI is unavailable" in report["blockers"][0]
    assert report["environment"]["diagnostic_commands"] == [
        "source /opt/ros/humble/setup.bash",
        "source install/setup.bash",
        "ros2 pkg executables local_planner",
    ]
    saved = json.loads(out.read_text(encoding="utf-8"))
    assert saved["claim_boundary"] == "ros2_runtime_unavailable"


def test_load_pct_route_rejects_known_unsafe_source_path(tmp_path: Path) -> None:
    source = _write_source_report(tmp_path)
    data = json.loads(source.read_text(encoding="utf-8"))
    data["cases"][0]["planning"][0]["path_safety"] = {
        "ok": False,
        "blocked_sample_count": 2,
    }
    source.write_text(json.dumps(data), encoding="utf-8")

    with pytest.raises(ValueError, match="failed path_safety"):
        _load_pct_route(source, route="same_floor")


def test_native_node_commands_launch_only_local_planner_stack(tmp_path: Path) -> None:
    local_planner_cmd, path_follower_cmd = _native_node_commands(
        path_folder=tmp_path / "paths",
        max_speed=0.4,
        autonomy_speed=0.35,
        goal_clear_range=0.45,
        near_field_stop_distance=0.35,
        lookahead=0.55,
        stop_distance=0.30,
        obstacle_aware=True,
        check_rot_obstacle=False,
    )

    joined = " ".join(local_planner_cmd + path_follower_cmd)
    assert local_planner_cmd[:4] == ["ros2", "run", "local_planner", "localPlanner"]
    assert path_follower_cmd[:4] == ["ros2", "run", "local_planner", "pathFollower"]
    assert "autonomyMode:=true" in joined
    assert "checkObstacle:=true" in joined
    assert "checkRotObstacle:=false" in joined
    assert "nearFieldStopDis:=0.35" in joined
    assert "Gateway" not in joined
    assert "driver" not in joined.lower()


def test_default_local_planner_path_folder_supports_merge_install(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    from sim.scripts import native_pct_mujoco_gate as mod

    merge_paths = tmp_path / "install/share/local_planner/paths"
    merge_paths.mkdir(parents=True)
    (merge_paths / "startPaths.ply").write_text("ply\n", encoding="utf-8")
    (merge_paths / "paths.ply").write_text("ply\n", encoding="utf-8")
    source_paths = tmp_path / "src/nav/local/paths"
    source_paths.mkdir(parents=True)
    (source_paths / "startPaths.ply").write_text("ply\n", encoding="utf-8")
    (source_paths / "paths.ply").write_text("ply\n", encoding="utf-8")
    monkeypatch.setattr(mod, "ROOT", tmp_path)

    assert _default_local_planner_path_folder() == merge_paths


def test_ros_path_to_points_extracts_native_local_path() -> None:
    class Position:
        x = 1.25
        y = -2.5
        z = 0.6

    class Pose:
        position = Position()

    class PoseStamped:
        pose = Pose()

    class PathMsg:
        poses = [PoseStamped()]

    assert _ros_path_to_points(PathMsg()) == [[1.25, -2.5, 0.6]]


def test_path_frame_detection_treats_empty_native_path_as_robot_frame() -> None:
    assert _path_is_robot_frame("")
    assert _path_is_robot_frame("vehicle")
    assert not _path_is_robot_frame("map")
    assert not _path_is_robot_frame("/odom")


def test_path_window_by_distance_keeps_long_front_view() -> None:
    path = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [4.5, 0.0, 0.0]]

    window = _path_window_by_distance(path, start_idx=1, lookahead_m=2.2)

    assert window == path[1:]


def test_sample_lidar_points_pads_intensity_and_downsamples() -> None:
    class Engine:
        def get_lidar_points(self):
            return [[float(i), float(i + 1), 0.0] for i in range(10)]

    pts = _sample_lidar_points(Engine(), 4)

    assert pts.shape == (4, 4)
    assert pts[:, 3].tolist() == [1.0, 1.0, 1.0, 1.0]
    assert pts[:, 0].tolist() == [0.0, 3.0, 6.0, 9.0]


def test_moving_obstacle_boxes_support_density_and_phase_offsets() -> None:
    route = PctRoute(
        route="unit",
        source_report=Path("report.json"),
        scene_xml=Path("scene.xml"),
        start=[0.0, 0.0, 0.0],
        goal=[4.0, 0.0, 0.0],
        path=[[float(idx), 0.0, 0.0] for idx in range(5)],
        plan={},
        case={},
    )
    args = argparse.Namespace(
        moving_obstacle_mode="route_crossing",
        moving_obstacle_route_ratio=0.5,
        moving_obstacle_count=3,
        moving_obstacle_route_ratio_step=0.1,
        moving_obstacle_lateral_phase_step_rad=math.pi / 2.0,
        moving_obstacle_start_s=0.0,
        moving_obstacle_duration_s=10.0,
        moving_obstacle_period_s=8.0,
        moving_obstacle_lateral_amplitude_m=1.0,
        moving_obstacle_along_amplitude_m=0.0,
        moving_obstacle_radius_m=0.12,
        moving_obstacle_height_m=0.4,
    )

    boxes = _moving_obstacle_boxes(route, args, elapsed_s=2.0)

    assert len(boxes) == 3
    assert [box["name"] for box in boxes] == [
        "moving_crossing_obstacle_0",
        "moving_crossing_obstacle_1",
        "moving_crossing_obstacle_2",
    ]
    assert [box["half_size"] for box in boxes] == [[0.12, 0.12, 0.2]] * 3
    assert boxes[0]["position"][0] < boxes[1]["position"][0] < boxes[2]["position"][0]
    assert boxes[0]["position"][1] != boxes[1]["position"][1]


def test_omni_cart_tracker_uses_native_local_path_without_saturating_yaw() -> None:
    class State:
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    cmd, debug = _omni_cart_cmd(
        state=State(),
        local_path=[[0.0, 0.0, 0.0], [0.4, 0.05, 0.0], [1.2, 0.1, 0.0]],
        local_path_frame_id="body",
        fallback_target=[3.0, 0.0, 0.0],
        lookahead_m=1.0,
        min_speed=0.12,
        max_speed=0.35,
        max_lateral_speed=0.30,
        yaw_gain=0.45,
        max_yaw_rate=0.22,
        yaw_deadband=0.12,
    )

    assert debug["source"] == "omni_local_path_tracker"
    assert cmd.linear_x > 0.30
    assert abs(cmd.linear_y) < 0.05
    assert abs(cmd.angular_z) <= 0.22


def test_select_local_target_falls_back_to_global_waypoint() -> None:
    class State:
        position = [1.0, 2.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    local, world = _select_local_target(
        state=State(),
        local_path=[],
        local_path_frame_id="",
        fallback_target=[3.0, 2.5, 0.0],
        lookahead_m=1.0,
    )

    assert local.tolist() == [2.0, 0.5]
    assert world.tolist() == [3.0, 2.5]


def test_trajectory_correctness_checks_progress_and_lateral_error() -> None:
    reference = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    trail = [(0.0, 0.05), (0.8, 0.04), (1.6, -0.03), (2.4, 0.02), (3.0, 0.0)]

    report = _trajectory_correctness(
        trail=trail,
        reference_path=reference,
        max_p95_error_m=0.20,
        max_progress_regressions=0,
        min_route_progress_ratio=0.98,
    )

    assert report["ok"] is True
    assert report["final_progress_ratio"] == 1.0
    assert report["p95_lateral_error_m"] < 0.08


def test_trajectory_correctness_rejects_wrong_side_track() -> None:
    reference = [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    trail = [(0.0, 1.5), (1.0, 1.5), (2.0, 1.5), (3.0, 1.5)]

    report = _trajectory_correctness(
        trail=trail,
        reference_path=reference,
        max_p95_error_m=0.50,
        max_progress_regressions=0,
        min_route_progress_ratio=0.90,
    )

    assert report["ok"] is False
    assert report["p95_lateral_error_m"] > 1.0


def test_trajectory_correctness_accepts_low_projected_progress_when_goal_reached() -> None:
    reference = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
    ]
    trail = [(0.0, 0.0), (0.8, 0.0), (1.6, 0.0), (2.0, 0.2)]

    report = _trajectory_correctness(
        trail=trail,
        reference_path=reference,
        max_p95_error_m=0.25,
        max_progress_regressions=0,
        min_route_progress_ratio=0.90,
        reached_goal=True,
    )

    assert report["ok"] is True
    assert report["final_progress_ratio"] < 0.90
    assert report["route_progress_ok"] is True
    assert report["goal_reached_override"] is True


def test_waypoint_safety_filter_records_detour_capability(tmp_path: Path) -> None:
    route = _load_pct_route(_write_source_report(tmp_path), route="same_floor")

    path, report = _build_safe_follow_path(
        route,
        enabled=True,
        clearance=0.40,
        extra_margin=0.18,
        sample_step_m=0.05,
    )

    assert report["enabled"] is True
    assert report["auto_detour_enabled"] is True
    assert report["skipped_unsafe_waypoint_count"] >= 1
    assert report["avoidance_clearance_m"] > report["clearance_m"]
    assert report["inserted_count"] >= 0
    assert len(path) != len(route.path)


def _route_with_center_obstacle(tmp_path: Path) -> PctRoute:
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps(
            {
                "obstacles": [
                    {
                        "name": "center_box",
                        "floor_id": 0,
                        "position": [1.0, 0.0, 0.35],
                        "half_size": [0.20, 0.20, 0.35],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    return PctRoute(
        route="unit",
        source_report=tmp_path / "report.json",
        scene_xml=tmp_path / "scene.xml",
        start=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0, 0.0],
        path=[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        plan={},
        case={"assets": {"metadata": str(metadata_path)}},
    )


def _route_with_terminal_obstacle(tmp_path: Path) -> PctRoute:
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps(
            {
                "obstacles": [
                    {
                        "name": "terminal_box",
                        "floor_id": 0,
                        "position": [2.0, 0.0, 0.35],
                        "half_size": [0.20, 0.20, 0.35],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    return PctRoute(
        route="terminal",
        source_report=tmp_path / "report.json",
        scene_xml=tmp_path / "scene.xml",
        start=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0, 0.0],
        path=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        plan={},
        case={"assets": {"metadata": str(metadata_path)}},
    )


def _route_with_terminal_landmark(tmp_path: Path) -> PctRoute:
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps(
            {
                "obstacles": [
                    {
                        "name": "terminal_localization_panel",
                        "kind": "localization_landmark",
                        "floor_id": 0,
                        "position": [2.0, 0.0, 0.35],
                        "half_size": [0.20, 0.20, 0.35],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    return PctRoute(
        route="landmark",
        source_report=tmp_path / "report.json",
        scene_xml=tmp_path / "scene.xml",
        start=[0.0, 0.0, 0.0],
        goal=[2.0, 0.0, 0.0],
        path=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        plan={},
        case={"assets": {"metadata": str(metadata_path)}},
    )


def _route_with_side_obstacle(tmp_path: Path) -> PctRoute:
    metadata_path = tmp_path / "metadata.json"
    metadata_path.write_text(
        json.dumps(
            {
                "obstacles": [
                    {
                        "name": "side_plinth",
                        "floor_id": 0,
                        "position": [-6.8, -3.95, 0.60],
                        "half_size": [0.30, 0.28, 0.60],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    return PctRoute(
        route="side",
        source_report=tmp_path / "report.json",
        scene_xml=tmp_path / "scene.xml",
        start=[-6.9, -4.9, 0.0],
        goal=[-5.9, -4.3, 0.0],
        path=[[-6.9, -4.9, 0.0], [-5.9, -4.3, 0.0]],
        plan={},
        case={"assets": {"metadata": str(metadata_path)}},
    )


def test_waypoint_safety_filter_inserts_progressive_detour_for_blocked_segment(
    tmp_path: Path,
) -> None:
    route = _route_with_center_obstacle(tmp_path)

    path, report = _build_safe_follow_path(
        route,
        enabled=True,
        clearance=0.30,
        extra_margin=0.18,
        sample_step_m=0.05,
    )

    assert report["auto_detour_enabled"] is True
    assert report["inserted_count"] == 3
    assert report["rejected_detour_count"] == 0
    assert report["insertions"][0]["obstacle"] == "center_box"
    assert len(path) > len(route.path)
    assert any(abs(point[1]) > 0.50 for point in path)


def test_waypoint_safety_filter_reports_when_goal_not_preserved(
    tmp_path: Path,
) -> None:
    route = _route_with_terminal_obstacle(tmp_path)

    path, report = _build_safe_follow_path(
        route,
        enabled=True,
        clearance=0.30,
        extra_margin=0.18,
        sample_step_m=0.05,
    )

    assert path[-1][:2] == [1.0, 0.0]
    assert report["goal_preserved"] is False
    assert report["source_path_goal_xy"] == [2.0, 0.0]
    assert report["follow_path_goal_xy"] == [1.0, 0.0]
    assert report["goal_shift_m"] == 1.0
    assert report["skipped_unsafe_waypoint_count"] == 1


def test_waypoint_safety_filter_does_not_truncate_localization_landmark(
    tmp_path: Path,
) -> None:
    route = _route_with_terminal_landmark(tmp_path)

    path, report = _build_safe_follow_path(
        route,
        enabled=True,
        clearance=0.30,
        extra_margin=0.18,
        sample_step_m=0.05,
    )

    assert path[-1][:2] == [2.0, 0.0]
    assert report["goal_preserved"] is True
    assert report["box_count"] == 0
    assert report["skipped_unsafe_waypoint_count"] == 0


def test_waypoint_safety_filter_inserts_side_detour_when_start_overlaps_box_x(
    tmp_path: Path,
) -> None:
    route = _route_with_side_obstacle(tmp_path)

    path, report = _build_safe_follow_path(
        route,
        enabled=True,
        clearance=0.40,
        extra_margin=0.18,
        sample_step_m=0.05,
    )

    assert report["inserted_count"] == 3
    assert report["rejected_detour_count"] == 0
    assert report["insertions"][0]["obstacle"] == "side_plinth"
    assert len(path) > len(route.path)
    assert min(point[1] for point in path) < -4.8


def test_local_path_obstacle_evidence_flags_path_into_obstacle(tmp_path: Path) -> None:
    class State:
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    report = _local_path_obstacle_evidence(
        local_path=[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        local_path_frame_id="body",
        state=State(),
        route=_route_with_center_obstacle(tmp_path),
        robot_radius=0.30,
        sample_step_m=0.05,
    )

    assert report["ok"] is False
    assert report["collision"] is True
    assert report["points_into_obstacle"] is True
    assert report["nearest_obstacle"] == "center_box"


def test_local_path_obstacle_evidence_accepts_detour(tmp_path: Path) -> None:
    class State:
        position = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0, 1.0]

    report = _local_path_obstacle_evidence(
        local_path=[[0.0, 0.75, 0.0], [1.0, 0.75, 0.0], [2.0, 0.75, 0.0]],
        local_path_frame_id="body",
        state=State(),
        route=_route_with_center_obstacle(tmp_path),
        robot_radius=0.30,
        sample_step_m=0.05,
    )

    assert report["ok"] is True
    assert report["collision"] is False
    assert report["points_into_obstacle"] is False
    assert report["min_clearance_minus_robot_radius_m"] > 0.0


def test_local_path_obstacle_summary_requires_native_path_when_obstacle_aware() -> None:
    summary = _summarize_local_path_obstacle_evidence(
        samples=[],
        path_count=0,
        stop_samples=[],
        slow_down_samples=[],
        obstacle_aware=True,
    )

    assert summary["ok"] is False
    assert summary["reasons"] == ["local_path_missing"]


def test_local_path_obstacle_summary_rejects_near_field_stop() -> None:
    summary = _summarize_local_path_obstacle_evidence(
        samples=[
            {
                "min_clearance_minus_robot_radius_m": 0.5,
                "collision": False,
                "points_into_obstacle": False,
            }
        ],
        path_count=1,
        stop_samples=[1],
        slow_down_samples=[],
        obstacle_aware=True,
    )

    assert summary["ok"] is False
    assert summary["near_field_stop_count"] == 1
    assert "near_field_stop" in summary["reasons"]
