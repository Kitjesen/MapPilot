from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = [pytest.mark.sim]

from runtime.contracts.simulation import simulation_runtime_contract
from runtime.runtime_interface import resolved_runtime_data_flow
from sim.scripts import server_sim_closure


REPO_ROOT = Path(__file__).resolve().parents[2]


def _write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _write_artifact(path: Path, data: bytes = b"artifact\n") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


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
                "path": f"{label}/map.pcd",
                "sha256": map_sha,
                "point_count": 4096,
            },
            "tomogram": {
                "path": f"{label}/tomogram.pickle",
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


def _write_moving_obstacle_child_report(
    path: Path,
    *,
    speed_mps: float,
    spacing_m: float,
) -> Path:
    return _write_json(
        path,
        {
            "ok": True,
            "simulation_only": True,
            "scan_time_profile": "physical_rolling",
            "lidar_source": {
                "scan_time_profile": "physical_rolling",
                "scan_time_model_contract": (
                    "physical_subscans_with_actual_sim_time_offsets"
                ),
            },
            "nav_data_source": "fastlio2",
            "true_mapping_input_path": (
                "/points_raw + /imu_raw -> fastlio2 -> /nav/odometry "
                "+ /nav/map_cloud"
            ),
            "outputs": {
                "fastlio2_cloud_registered": 20,
                "fastlio2_odometry": 20,
                "nav_odometry": 20,
                "nav_registered_cloud": 20,
                "nav_map_cloud": 20,
                "nav_cmd_vel": 12,
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
                "last_plan_report": {"selected_planner": "pct"},
            },
            "deliverable_contract": {
                "checks": {"same_source_map_artifact": True},
            },
            "map_artifacts": _same_source_map_artifacts("moving_sweep_child"),
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
        },
    )


def _write_navigation_topic_jsonl(path: Path, *, encoding: str = "utf-8") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    events = [
        {
            "topic": "/nav/global_path",
            "t": 0.0,
            "msg": {
                "poses": [
                    {"pose": {"position": {"x": 0.0, "y": 0.0}}},
                    {"pose": {"position": {"x": 1.0, "y": 0.2}}},
                    {"pose": {"position": {"x": 2.0, "y": 0.4}}},
                ]
            },
        },
        {
            "topic": "/nav/local_path",
            "t": 0.01,
            "msg": {
                "poses": [
                    {"pose": {"position": {"x": 0.0, "y": 0.0}}},
                    {"pose": {"position": {"x": 0.7, "y": 0.14}}},
                    {"pose": {"position": {"x": 1.4, "y": 0.28}}},
                    {"pose": {"position": {"x": 2.0, "y": 0.4}}},
                ]
            },
        },
        {
            "topic": "/goal_pose",
            "t": 0.02,
            "msg": {"pose": {"position": {"x": 2.0, "y": 0.4}}},
        },
    ]
    for index in range(8):
        progress = index / 7.0
        events.extend(
            [
                {
                    "topic": "/nav/cmd_vel",
                    "t": 0.1 + index * 0.2,
                    "msg": {"linear": {"x": 0.35, "y": 0.0}, "angular": {"z": 0.02}},
                },
                {
                    "topic": "/Odometry",
                    "t": 0.11 + index * 0.2,
                    "msg": {
                        "pose": {
                            "pose": {
                                "position": {
                                    "x": progress * 2.0,
                                    "y": progress * 0.4,
                                }
                            }
                        }
                    },
                },
            ]
        )
    path.write_text("\n".join(json.dumps(event) for event in events), encoding=encoding)
    return path


def _module_import_is_safe(module_name: str) -> bool:
    try:
        probe = subprocess.run(
            [sys.executable, "-c", f"import {module_name}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=10,
        )
    except Exception:
        return False
    return probe.returncode == 0


def _write_test_video(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not (_module_import_is_safe("numpy") and _module_import_is_safe("cv2")):
        path.write_bytes(b"LingTu server-sim placeholder video artifact\n")
        return path

    import cv2  # type: ignore
    import numpy as np

    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 8.0, (64, 48))
    assert writer.isOpened()
    for index in range(3):
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        frame[:, :, 1] = 80 + index * 20
        writer.write(frame)
    writer.release()
    return path


def _has_video_decode_gap(gaps: str, prefix: str) -> bool:
    return any(
        expected in gaps
        for expected in (
            f"{prefix} video decoder unavailable",
            f"{prefix} video first frame is not decodable",
            f"{prefix} video nonblack validation unavailable",
        )
    )


def test_dimos_required_gates_come_from_core_algorithm_gate_constant():
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES

    assert tuple(server_sim_closure.ALGORITHM_PRESETS["dimos_benchmark"]) == tuple(
        DIMOS_BENCHMARK_REQUIRED_GATES
    )
    assert DIMOS_BENCHMARK_REQUIRED_GATES[0] == "gateway_runtime_acceptance"


def _readme_current_full_closure_gates() -> tuple[str, ...]:
    readme = (REPO_ROOT / "sim/README.md").read_text(encoding="utf-8")
    lines = readme.splitlines()
    start = lines.index("Current full closure gates:")
    gates: list[str] = []
    for line in lines[start + 1:]:
        if not line.strip():
            if gates:
                break
            continue
        if line.startswith("| `"):
            gates.append(line.split("`", 2)[1])
    return tuple(gates)


def test_g4_server_full_sim_required_gates_come_from_core_algorithm_gate_constant():
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    assert tuple(server_sim_closure.ALGORITHM_PRESETS["g4_server_full_sim"]) == tuple(
        G4_SERVER_FULL_SIM_REQUIRED_GATES
    )
    assert set(G4_SERVER_FULL_SIM_REQUIRED_GATES) == (
        (
            set(DIMOS_BENCHMARK_REQUIRED_GATES)
            - {"native_pct_mujoco", "pct_saved_map_navigation", "gazebo_runtime"}
        )
        | {
            "multifloor_exploration",
            "policy_nav",
            "mujoco_tare_exploration",
            "gateway_dry_run",
        }
    )
    assert "native_pct_mujoco" not in G4_SERVER_FULL_SIM_REQUIRED_GATES
    assert "pct_saved_map_navigation" not in G4_SERVER_FULL_SIM_REQUIRED_GATES
    assert "gazebo_runtime" not in G4_SERVER_FULL_SIM_REQUIRED_GATES


def test_readme_current_full_closure_gates_match_g4_server_full_sim_preset():
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    assert _readme_current_full_closure_gates() == tuple(
        G4_SERVER_FULL_SIM_REQUIRED_GATES
    )


def test_readme_full_closure_command_uses_g4_server_full_sim_preset():
    readme = (REPO_ROOT / "sim/README.md").read_text(encoding="utf-8")
    command_section = readme.split(
        "PYTHONPATH=src:. python sim/scripts/server_sim_closure.py",
        1,
    )[1].split("```", 1)[0]

    assert "--preset g4_server_full_sim" in command_section
    assert "--required-only" in command_section
    assert "--run-missing" not in command_section


def test_g4_required_gates_have_report_override_options():
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    parser = server_sim_closure._build_parser()
    option_strings = {
        option
        for action in parser._actions
        for option in action.option_strings
    }

    for gate in G4_SERVER_FULL_SIM_REQUIRED_GATES:
        assert f"--{gate.replace('_', '-')}-report" in option_strings


def test_g4_summary_required_gate_sequence_preserves_core_order(tmp_path: Path, monkeypatch):
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={},
        required=set(G4_SERVER_FULL_SIM_REQUIRED_GATES),
        include_optional=False,
    )

    assert tuple(summary["required_gate_sequence"]) == G4_SERVER_FULL_SIM_REQUIRED_GATES


def test_g4_missing_required_gates_preserve_core_order(tmp_path: Path, monkeypatch):
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={},
        required=set(G4_SERVER_FULL_SIM_REQUIRED_GATES),
        include_optional=False,
    )

    assert tuple(summary["missing_or_failed"]) == G4_SERVER_FULL_SIM_REQUIRED_GATES
    assert tuple(
        gap.split(":", 1)[0] for gap in summary["remaining_gaps"]
    ) == G4_SERVER_FULL_SIM_REQUIRED_GATES


def test_g4_required_gates_are_default_freshness_required():
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    assert set(G4_SERVER_FULL_SIM_REQUIRED_GATES) <= set(
        server_sim_closure.DEFAULT_FRESHNESS_REQUIRED_GATES
    )


def test_dimos_summary_required_gate_sequence_preserves_core_order(tmp_path: Path, monkeypatch):
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES

    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={},
        required=set(DIMOS_BENCHMARK_REQUIRED_GATES),
        include_optional=False,
    )

    assert tuple(summary["required_gate_sequence"]) == DIMOS_BENCHMARK_REQUIRED_GATES


def test_dimos_summary_requires_fresh_reports_for_every_required_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES

    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    stale_gateway_report = _write_json(
        tmp_path / "artifacts/server_sim_closure/gateway_runtime_acceptance/report.json",
        _complete_gateway_runtime_acceptance_report(),
    )
    old_mtime = server_sim_closure.time.time() - (
        server_sim_closure.DEFAULT_REQUIRED_MAX_REPORT_AGE_S + 60.0
    )
    os.utime(stale_gateway_report, (old_mtime, old_mtime))

    summary = server_sim_closure.summarize(
        report_overrides={},
        required=set(DIMOS_BENCHMARK_REQUIRED_GATES),
        include_optional=False,
    )

    gateway_gate = summary["gates"]["gateway_runtime_acceptance"]
    assert gateway_gate["ok"] is False
    assert gateway_gate["status"] == "failed"
    assert gateway_gate["is_fresh"] is False
    assert gateway_gate["max_report_age_s"] == (
        server_sim_closure.DEFAULT_REQUIRED_MAX_REPORT_AGE_S
    )
    assert any("report_age_s" in blocker for blocker in gateway_gate["blockers"])


def test_structured_planner_result_marks_unavailable_backend_as_skip(monkeypatch, tmp_path: Path):
    import importlib.util

    module_path = Path("tests/benchmark/benchmark_planner_structured.py")
    spec = importlib.util.spec_from_file_location("benchmark_planner_structured", module_path)
    assert spec is not None and spec.loader is not None
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)

    class UnavailableService:
        map_artifact_gate = {"ok": False}

        def __init__(self, **kwargs):
            pass

        def setup(self):
            raise RuntimeError("pct planner unavailable: native backend missing")

    monkeypatch.setattr(benchmark, "GlobalPlanner", UnavailableService)

    result = benchmark.benchmark_planner(
        "pct",
        tmp_path,
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )

    assert result["status"] == "skip"
    assert result["ok"] is False
    assert result["synthetic"] is True
    assert result["claim_scope"] == "planner_regression_only"


def test_structured_planner_plan_unavailable_is_skip_not_fail(monkeypatch, tmp_path: Path):
    import importlib.util

    module_path = Path("tests/benchmark/benchmark_planner_structured.py")
    spec = importlib.util.spec_from_file_location("benchmark_planner_structured", module_path)
    assert spec is not None and spec.loader is not None
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)

    class Backend:
        pass

    class UnavailableAtPlanService:
        map_artifact_gate = {"ok": True}
        last_plan_report = {}
        _backend = Backend()

        def __init__(self, **kwargs):
            pass

        def setup(self):
            return None

        def plan(self, start, goal, safe_goal_tolerance=0.0):
            raise RuntimeError("GlobalPlanner: pct planner unavailable")

    monkeypatch.setattr(benchmark, "GlobalPlanner", UnavailableAtPlanService)

    result = benchmark.benchmark_planner(
        "pct",
        tmp_path,
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )

    assert result["status"] == "skip"
    assert result["ok"] is False


def test_structured_planner_import_dependency_skip_report(tmp_path: Path):
    import importlib.util

    module_path = Path("tests/benchmark/benchmark_planner_structured.py")
    spec = importlib.util.spec_from_file_location("benchmark_planner_structured", module_path)
    assert spec is not None and spec.loader is not None
    benchmark = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark)

    json_out = tmp_path / "report.json"

    exit_code = benchmark.write_import_skip_report(
        json_out=json_out,
        planners=["astar", "pct"],
        routes=["terrain_short"],
        error="missing dependency: numpy",
    )

    payload = json.loads(json_out.read_text(encoding="utf-8"))
    assert exit_code == 0
    assert payload["status_counts"] == {"pass": 0, "skip": 1, "fail": 0}
    assert payload["results"][0]["status"] == "skip"
    assert payload["results"][0]["ok"] is False
    assert payload["synthetic"] is True


def _complete_gateway_runtime_acceptance_report() -> dict:
    return {
        "schema_version": "lingtu.gateway_runtime_acceptance.v1",
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "mode": "non_motion",
        "target_result": (
            "Gateway-only product runtime acceptance; ROS2 topic inspection is not required."
        ),
        "runtime_contract": "real_s100p",
        "ros2_topic_required": False,
        "blockers": [],
        "advisories": [],
        "checks": {
            "gateway_contract": {
                "ok": True,
                "missing_links": [],
            },
            "module_first_dataflow": {
                "ok": True,
                "runtime_contract": "real_s100p",
                "ros2_topic_required": False,
                "module_port_bus_primary": True,
                "ros2_adapter_primary": False,
                "arbitrary_publish_supported": False,
                "command_interface_count": 6,
                "missing_command_interfaces": [],
                "unexpected_command_interfaces": [],
                "observable_topics": [
                    "/nav/odometry",
                    "/nav/map_cloud",
                    "/nav/localization_health",
                    "/nav/global_path",
                    "/nav/local_path",
                    "/nav/cmd_vel",
                    "/nav/mission_status",
                ],
                "streamable_topics": [
                    "/nav/odometry",
                    "/nav/map_cloud",
                    "/nav/localization_health",
                    "/nav/global_path",
                    "/nav/local_path",
                    "/nav/cmd_vel",
                    "/nav/mission_status",
                ],
                "missing_topics": [],
                "non_observable_topics": [],
                "missing_stream_interfaces": [],
                "missing_live_topics": [],
            },
            "stage_evidence": {
                "ok": True,
                "stage_count": 7,
                "missing_stages": [],
                "not_live_stages": [],
                "missing_tokens": {},
            },
        },
    }


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


def _complete_native_pct_mujoco_report() -> dict:
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
        "pct_native_backend_used": True,
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
            "native_backend_used": True,
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
        "map_artifacts": _same_source_map_artifacts("native_pct"),
        "obstacle_aware": {"enabled": True, "metadata_points": 32},
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


def _data_flow_evidence(data_source: str = "mujoco_fastlio2_live") -> dict:
    return {
        stage.name: {
            "ok": True,
            "inputs": list(stage.inputs),
            "outputs": list(stage.outputs),
            "owner": stage.owner,
            "frame_role": stage.frame_role,
            "map_dependency": stage.map_dependency,
        }
        for stage in resolved_runtime_data_flow(data_source)
    }


def _runtime_frame_evidence() -> dict:
    return {
        "map_to_odom": {
            "ok": True,
            "parent": "map",
            "child": "odom",
            "static": True,
        },
        "odom_to_body": {
            "ok": True,
            "parent": "odom",
            "child": "body",
            "samples": 4,
        },
        "body_to_lidar": {
            "ok": True,
            "parent": "body",
            "child": "lidar_link",
            "static": True,
        },
        "body_to_camera": {
            "ok": True,
            "parent": "body",
            "child": "camera_link",
            "static": True,
        },
    }


def _fastlio2_frame_evidence() -> dict:
    return _runtime_frame_evidence()


def _fastlio2_runtime_contract() -> dict:
    return {
        "name": "mujoco_fastlio2_live",
        "ok": True,
        "data_flow_evidence": _data_flow_evidence(),
        "frame_evidence": _fastlio2_frame_evidence(),
    }


def _sim_command_hardware_safety() -> dict:
    return {
        "topics": {"/nav/cmd_vel": ["/mujoco_velocity_adapter"]},
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
    }


def _complete_fastlio2_tare_report() -> dict:
    return {
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "live_mujoco_lidar_verified": True,
        "live_mujoco_imu_verified": True,
        "slam_algorithm_output_verified": True,
        "canonical_nav_outputs_verified": True,
        "bridge_verified": True,
        "outputs": {
            "fastlio2_odometry": 12,
            "nav_odometry": 12,
            "nav_registered_cloud": 8,
            "nav_map_cloud": 8,
            "nav_cmd_vel_nonzero": 10,
        },
        "states_seen": ["TRACKING"],
        "point_count": {"cloud_map": 100},
        "frames": {"published_lidar": "body", "cmd_vel": "/nav/cmd_vel"},
        "runtime_contract": _fastlio2_runtime_contract(),
        "hardware_safety": _sim_command_hardware_safety(),
        "lidar_source": _mid360_lidar_source(),
        "lingtu_tare": {
            "enabled": True,
            "verified": True,
            "started_after_slam_ready": True,
            "exploration_grid_samples": 4,
            "exploration_grid_first": {"free": 40, "occupied": 8, "unknown": 120},
            "exploration_grid_last": {"free": 70, "occupied": 12, "unknown": 82},
            "exploration_grid_metadata_last": {
                "accumulation": "rolling_local_window",
                "semantic": "frontier_input_grid",
                "source": "raycast_lidar_exploration_grid",
            },
            "goal_count": 2,
            "successful_navigation_goal_count": 2,
            "failed_navigation_goal_count": 0,
            "min_required_goals": 2,
            "global_path_points_max": 8,
            "local_path_points_max": 5,
        },
        "navigation_chain": {
            "planner_fallback_used": False,
            "planner_repair_used": False,
            "direct_goal_fallback": {"used": False},
            "health": {"plan_safety_policy": "reject"},
        },
        "map_growth": {
            "accepted_cumulative_growth_source": "/nav/map_cloud",
            "exploration_area_samples": 4,
            "exploration_grid_accumulation": "rolling_local_window",
            "exploration_grid_growth_is_acceptance_metric": False,
            "min_explored_area_growth_m2": 0.25,
            "min_exploration_coverage_growth_ratio": 0.001,
            "exploration_known_area": {"growth_m2": 3.0},
            "exploration_coverage": {"growth_ratio": 0.02},
            "min_map_area_growth_m2": 1.0,
            "nav_map_cloud_xy_area": {"growth_m2": 2.0},
            "nav_map_cloud_area_samples": 4,
        },
    }


def _complete_fastlio2_inspection_report() -> dict:
    return {
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "live_mujoco_lidar_verified": True,
        "live_mujoco_imu_verified": True,
        "slam_algorithm_output_verified": True,
        "canonical_nav_outputs_verified": True,
        "bridge_verified": True,
        "outputs": {
            "fastlio2_odometry": 12,
            "nav_odometry": 12,
            "nav_registered_cloud": 8,
            "nav_map_cloud": 8,
            "nav_cmd_vel_nonzero": 10,
        },
        "states_seen": ["TRACKING"],
        "point_count": {"cloud_map": 100},
        "frames": {"published_lidar": "body", "cmd_vel": "/nav/cmd_vel"},
        "runtime_contract": _fastlio2_runtime_contract(),
        "hardware_safety": _sim_command_hardware_safety(),
        "lidar_source": _mid360_lidar_source(),
        "lingtu_inspection": {
            "enabled": True,
            "verified": True,
            "started_after_slam_ready": True,
            "goal_count": 3,
            "successful_navigation_goal_count": 3,
            "failed_navigation_goal_count": 0,
            "min_required_checkpoints": 3,
            "global_path_points_max": 8,
            "local_path_points_max": 5,
            "patrol_state": "SUCCESS",
            "patrol_index": 3,
            "patrol_total": 3,
        },
        "navigation_chain": {
            "planner_fallback_used": False,
            "planner_repair_used": False,
            "direct_goal_fallback": {"used": False},
            "health": {"plan_safety_policy": "reject"},
        },
        "map_growth": {
            "accepted_cumulative_growth_source": "/nav/map_cloud",
            "min_map_area_growth_m2": 1.0,
            "nav_map_cloud_xy_area": {"growth_m2": 2.0},
            "nav_map_cloud_area_samples": 4,
        },
    }


def _complete_fastlio2_dynamic_inspection_report(tmp_path: Path) -> dict:
    report = _complete_fastlio2_inspection_report()
    report["lingtu_inspection"].update(
        {
            "global_planner": "pct",
            "replan_on_costmap_update": False,
            "local_path_count": 113,
        }
    )
    report["moving_obstacles"] = {
        "enabled": True,
        "mode": "robot_crossing",
        "count": 3,
        "period_s": 6.0,
        "ok": True,
        "published_update_count": 116,
        "published_point_count_max": 48,
        "speed_bounds": {
            "peak_along_speed_mps": 0.1309,
            "peak_lateral_speed_mps": 0.7854,
            "peak_planar_speed_bound_mps": 0.7962,
        },
        "trail_clearance": {
            "checked": True,
            "collision": False,
            "min_clearance_minus_robot_radius_m": 0.4783,
        },
    }
    report["video_path"] = str(_write_test_video(tmp_path / "inspection_dyn3_fast.mp4"))
    report["video_frame_count"] = 181
    report["video_sample_count"] = 181
    report["true_mapping_input_path"] = (
        "/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_map "
        "-> /nav/odometry + /nav/map_cloud"
    )
    report["fastlio2_z_consistency"] = {
        "checked": True,
        "ok": True,
        "z_delta_error_m": 0.025,
    }
    report["fastlio2_yaw_consistency"] = {
        "checked": True,
        "ok": True,
        "yaw_delta_error_rad": 0.0036,
    }
    report["fastlio2_motion_consistency"] = {
        "checked": True,
        "ok": True,
        "fastlio2_moved_m": 1.105,
        "sim_moved_m": 1.099,
    }
    report["deliverable_contract"] = {
        "checks": {
            "raw_mujoco_lidar": True,
            "raw_mujoco_imu": True,
            "fastlio2_odometry_and_map": True,
            "inspection_patrol": True,
            "moving_obstacle_evidence": True,
            "nav_cmd_vel_nonzero": True,
            "same_source_map_artifact": True,
        }
    }
    report["map_artifacts"] = _same_source_map_artifacts("dynamic_inspection")
    return report


def _complete_saved_map_relocalize_runtime_report() -> dict:
    map_sha = "saved-map-relocalize-map-sha256"
    return {
        "ok": True,
        "schema_version": "lingtu.saved_map_relocalize_runtime.v1",
        "validation_level": "runtime_relocalization",
        "runtime_stage": "saved_map_relocalization",
        "map_dependency": "saved_map_required",
        "requires_saved_map": True,
        "requires_live_slam": True,
        "requires_tomogram": False,
        "runtime_relocalization_executed": True,
        "runtime_relocalization_validated": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "map_pcd": "/tmp/lingtu/same_source_map/map.pcd",
        "map_metadata_contract": {
            "ok": True,
            "path": "/tmp/lingtu/same_source_map/metadata.json",
            "schema_version": "lingtu.same_source_map_artifacts.v1",
            "world": "/tmp/lingtu/world.xml",
            "scan_time_profile": "physical_rolling",
            "artifacts": {
                "map_pcd": {
                    "path": "/tmp/lingtu/same_source_map/map.pcd",
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
            "health_samples": 8,
            "tracking_health_samples": 5,
            "latest_health_state": "LOCKED",
            "latest_health": "LOCKED|fitness=0.004",
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


def _complete_pct_saved_map_navigation_report() -> dict:
    native_gate = {
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
        "pct_native_backend_used": True,
        "pct_runtime_ok": True,
        "pct_path_count": 3,
        "pct_optimizer_enabled": False,
        "pct_optimizer_attempted": False,
        "pct_optimizer_accepted": False,
        "pct_optimizer_reject_reason": "",
        "pct_optimizer_blocked_sample_count": 0,
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
            "native_backend_used": True,
            "tomogram_exists": True,
            "tomogram_sha256": "abc123",
            "pct_optimizer_enabled": False,
            "pct_optimizer_attempted": False,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "",
            "pct_optimizer_blocked_sample_count": 0,
            "pct_planner_path_mode": "native_astar_raw_path",
        },
        "deliverable_contract": {
            "checks": {"same_source_map_artifact": True},
        },
        "map_artifacts": _same_source_map_artifacts("pct_saved_map_native"),
        "obstacle_aware": {"enabled": True, "metadata_points": 32},
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
    return {
        "ok": True,
        "schema_version": "lingtu.pct_saved_map_navigation_gate.v1",
        "validation_level": "saved_map_relocalized_pct_navigation",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "tomogram": "/tmp/lingtu/same_source_map/tomogram.pickle",
        "relocalize_report": "/tmp/lingtu/relocalize/report.json",
        "deliverable_contract": {
            "checks": {"same_source_map_artifact": True},
        },
        "map_artifacts": _same_source_map_artifacts("pct_saved_map"),
        "same_source_hash_identity": _same_source_hash_identity("pct_saved_map"),
        "relocalization": {
            "ok": True,
            "latest_health_state": "LOCKED",
            "saved_map_cloud_points_latest": 2048,
            "map_to_odom_xy_m": 0.12,
        },
        "plan_preview": {
            "ok": True,
            "selected_case": "internal_free_route",
            "selected_planner": "pct",
            "fallback_reason": "",
            "path_count": 12,
        },
        "scene_obstacle_metadata": {
            "path": "/tmp/lingtu/scene_obstacles.json",
            "obstacle_count": 8,
        },
        "native_gate": native_gate,
        "blockers": [],
    }


def _complete_multifloor_report() -> dict:
    localization = {"ok": True}
    command_flow = {"ok": True, "cmd_vel_sent_to_hardware": False}
    tracking_replay = {"ok": True, "cmd_vel_sent_to_hardware": False}
    native_pct_gate = {
        "ok": True,
        "floor_graph_composition_verified": False,
        "native_pct_feasible_segments": 1,
    }
    cases = []
    for route in ("same_floor", "lower_approach", "upper_floor"):
        cases.append(
            {
                "route": route,
                "passed": True,
                "lidar_localization": localization,
                "native_pct_gate": native_pct_gate,
                "command_flow": command_flow,
                "tracking_replay": tracking_replay,
            }
        )
    cases.append(
        {
            "route": "cross_floor",
            "passed": True,
            "lidar_localization": localization,
            "native_pct_gate": {
                "ok": True,
                "floor_graph_composition_verified": True,
                "native_pct_feasible_segments": 2,
            },
            "command_flow": command_flow,
            "tracking_replay": tracking_replay,
        }
    )
    return {
        "passed": True,
        "suite": "multifloor_route_matrix",
        "validation_level": "kinematic_module_ports",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "local_planner_backend_verified": "nanobind",
        "production_local_planner_required": True,
        "production_local_planner_verified": True,
        "frontier_loop_enabled": True,
        "routes": ["same_floor", "lower_approach", "upper_floor", "cross_floor"],
        "native_pct_gate_passed_count": 4,
        "exploration": {
            "ok": True,
            "closed_loop": True,
            "probe_mode": "frontier_navigation_closed_loop",
            "rounds": [
                {
                    "goal": {"frontier": [1.0, 0.0, 0.0]},
                    "command_flow": command_flow,
                    "tracking_replay": tracking_replay,
                }
            ],
        },
        "cases": cases,
    }


def _complete_gazebo_report() -> dict:
    return {
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "samples": 4,
        "odometry_frame_id": "odom",
        "odometry_child_frame_id": "body",
        "topic_samples": {
            "/nav/map_cloud": 3,
            "/nav/registered_cloud": 3,
            "/camera/color/image_raw": 3,
            "/camera/depth/image_raw": 3,
            "/camera/color/camera_info": 3,
        },
        "topic_frames": {
            "/nav/map_cloud": "odom",
            "/nav/registered_cloud": "body",
            "/camera/color/image_raw": "camera_link",
            "/camera/depth/image_raw": "camera_link",
            "/camera/color/camera_info": "camera_link",
        },
        "point_counts": {
            "/nav/map_cloud": 1024,
            "/nav/registered_cloud": 1024,
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
            "odom_start_xy": [0.0, 0.0],
            "odom_last_xy": [0.06, 0.05],
            "odom_delta_m": 0.078,
            "samples": {
                "/nav/global_path": 2,
                "/nav/local_path": 12,
                "/nav/cmd_vel": 18,
                "/nav/odometry": 16,
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
                    "/nav/odometry": {
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
            "odom_start_xy": [0.0, 0.0],
            "odom_last_xy": [0.08, 0.02],
            "odom_delta_m": 0.082,
            "odom_delta_x_m": 0.08,
            "known_cells_initial": 101,
            "known_cells_final": 140,
            "known_cells_delta": 39,
            "explored_area_initial_m2": 1.01,
            "explored_area_final_m2": 1.40,
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
                "frames": {
                    "map_cloud": ["odom"],
                    "registered_cloud": ["body"],
                    "terrain_map": ["odom"],
                    "terrain_map_ext": ["odom"],
                },
            },
            "frontier_no_gain_stall": {
                "checked": True,
                "ok": True,
                "mode": "post_pass_observation",
                "stop_reason": "post_pass_observation_elapsed",
                "required_observation_s": 5.0,
                "observed_s": 5.1,
                "known_cells_at_pass": 132,
                "known_cells_final": 140,
                "known_cells_delta_after_pass": 8,
                "explored_area_at_pass_m2": 1.32,
                "explored_area_final_m2": 1.40,
                "explored_area_delta_after_pass_m2": 0.08,
                "frontier_count_max_at_pass": 3,
                "frontier_count_max_final": 3,
            },
            "cumulative_map_cloud": {
                "samples": 9,
                "frame_ids": ["odom"],
                "point_count_initial": 1800,
                "point_count_final": 4200,
                "point_count_delta": 2400,
                "point_growth_ratio": 2.3333,
                "unique_voxels_initial": 300,
                "unique_voxels_final": 850,
                "unique_voxels_delta": 550,
                "unique_voxel_growth_ratio": 2.8333,
                "growth_step_ratio": 0.75,
                "retention_min": 0.85,
            },
            "registered_cloud": {
                "samples": 9,
                "median_unique_voxels": 300,
                "map_vs_registered_voxel_ratio": 2.8333,
            },
            "static_obstacles": {
                "column": {
                    "samples": 4,
                    "centroid_drift_max_m": 0.0,
                }
            },
            "samples": {
                "frontier_goal": 1,
                "/nav/goal_pose": 1,
                "/nav/global_path": 2,
                "/nav/local_path": 12,
                "/nav/cmd_vel": 18,
                "/nav/odometry": 16,
                "/nav/map_cloud": 3,
                "/nav/terrain_map": 3,
                "/nav/terrain_map_ext": 9,
                "/nav/cumulative_map_cloud": 9,
                "/nav/registered_cloud": 9,
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


def _complete_cmu_unity_report() -> dict:
    return {
        "schema_version": "lingtu.cmu_unity_sim_gate.v1",
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "runtime_executed": False,
        "cmu_workspace": {
            "path": "/tmp/cmu",
            "branch": "humble",
            "head": "abc1234",
            "remote": "https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform.git",
            "topic_contract": {
                "/registered_scan": True,
                "/state_estimation": True,
                "/state_estimation_at_scan": True,
                "/terrain_map": True,
                "/terrain_map_ext": True,
                "/way_point": True,
                "/path": True,
                "/cmd_vel": True,
                "/navigation_boundary": True,
            },
        },
        "lingtu_contract": {
            "remaps": {
                "/registered_scan": "/nav/map_cloud",
                "/state_estimation": "/nav/odometry",
                "/state_estimation_at_scan": "/nav/odometry",
                "/terrain_map": "/nav/terrain_map",
                "/terrain_map_ext": "/nav/terrain_map_ext",
                "/way_point": "/exploration/way_point",
            },
            "adapter_required_relays": {
                "/state_estimation->/nav/odometry": "nav_msgs/msg/Odometry",
                "/state_estimation_at_scan->/nav/state_estimation_at_scan": "nav_msgs/msg/Odometry",
                "/registered_scan->/nav/map_cloud": "sensor_msgs/msg/PointCloud2",
                "/terrain_map->/nav/terrain_map": "sensor_msgs/msg/PointCloud2",
                "/terrain_map_ext->/nav/terrain_map_ext": "sensor_msgs/msg/PointCloud2",
                "/way_point->/exploration/way_point": "geometry_msgs/msg/PointStamped",
                "/exploration/start->/start_exploration": "std_msgs/msg/Bool",
                "/nav/cmd_vel->/cmd_vel": "geometry_msgs/msg/TwistStamped",
            },
        },
        "checks": [
            {"name": "host_ros_humble_setup", "ok": True, "required": True},
            {"name": "host_ros2_cli", "ok": True, "required": True},
            {"name": "host_ros2_cli_functional", "ok": True, "required": True},
            {"name": "host_colcon_cli", "ok": True, "required": True},
            {"name": "host_colcon_cli_functional", "ok": True, "required": True},
            {"name": "cmu_workspace_exists", "ok": True, "required": True},
            {"name": "cmu_git_workspace", "ok": True, "required": True},
            {"name": "cmu_humble_branch", "ok": True, "required": True},
            {"name": "cmu_required_source_paths", "ok": True, "required": True},
            {"name": "cmu_unity_environment_assets", "ok": True, "required": True},
            {"name": "cmu_colcon_build_output", "ok": True, "required": True},
            {"name": "cmu_topic_contract", "ok": True, "required": True},
            {"name": "lingtu_tare_remap_contract", "ok": True, "required": True},
            {"name": "lingtu_tare_explore_profile", "ok": True, "required": True},
            {"name": "lingtu_cmu_tare_profile", "ok": True, "required": True},
            {"name": "lingtu_cmu_adapter_exists", "ok": True, "required": True},
            {"name": "lingtu_cmu_adapter_relay_contract", "ok": True, "required": True},
            {"name": "lingtu_cmu_adapter_safety_contract", "ok": True, "required": True},
        ],
        "blockers": [],
    }


def _complete_cmu_unity_runtime_contract_report(
    *,
    planner_diagnostics_required: bool = False,
    planner_diagnostics_available: bool = False,
    errors: list[str] | None = None,
) -> dict:
    contract_errors = list(errors or [])
    return {
        "name": "cmu_unity_external",
        "ok": not contract_errors,
        "definition": simulation_runtime_contract("cmu_unity_external").as_report(),
        "data_flow_evidence": _data_flow_evidence("cmu_unity_external"),
        "frame_evidence": _runtime_frame_evidence(),
        "topic_evidence": {
            "/nav/odometry": {"samples": 12, "delta_m": 0.121, "ok": True},
            "/nav/state_estimation_at_scan": {
                "samples": 12,
                "delta_m": 0.121,
                "ok": True,
            },
            "/registered_scan": {"samples": 4, "area_delta_m2": 0.75, "ok": True},
            "/nav/registered_cloud": {"samples": 4, "area_delta_m2": 0.75, "ok": True},
            "/nav/map_cloud": {"samples": 4, "area_delta_m2": 0.75, "ok": True},
            "/nav/terrain_map_ext": {"samples": 4, "area_delta_m2": 0.75, "ok": True},
            "/exploration/way_point": {"samples": 1, "frames": ["map"], "ok": True},
            "/nav/global_path": {
                "samples": 1,
                "nonempty_samples": 1,
                "max_poses": 5,
                "frames": ["map"],
                "ok": True,
            },
            "/nav/local_path": {
                "samples": 4,
                "nonempty_samples": 4,
                "max_poses": 101,
                "frames": ["map"],
                "ok": True,
            },
            "/nav/cmd_vel": {
                "samples": 8,
                "nonzero_samples": 4,
                "max_norm": 0.22,
                "ok": True,
            },
        },
        "publisher_identity": {
            "subscribers": {"/cmd_vel": ["/vehicle_simulator"], "/nav/cmd_vel": []},
            "publishers": {
                "/cmd_vel": ["/lingtu_cmu_unity_adapter"],
                "/nav/cmd_vel": ["/lingtu_navigation"],
            },
            "blocked_hardware_nodes": [],
            "unexpected_command_publishers": [],
        },
        "planner_diagnostics_required": planner_diagnostics_required,
        "planner_diagnostics_available": planner_diagnostics_available,
        "errors": contract_errors,
    }


def _complete_cmu_unity_runtime_report() -> dict:
    return {
        "schema_version": "lingtu.cmu_unity_runtime_gate.v1",
        "ok": True,
        "runtime_executed": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "ros_domain_id": "73",
        "thresholds": {
            "min_waypoint_samples": 1,
            "min_cmd_vel": 0.01,
            "min_cmd_vel_samples": 3,
            "min_odom_delta_m": 0.10,
            "min_map_area_delta_m2": 0.5,
            "late_window_sec": 60.0,
            "min_late_odom_delta_m": 0.5,
            "min_late_cmd_vel_samples": 10,
            "min_late_path_samples": 5,
            "min_late_map_area_delta_m2": 1.0,
            "allow_flat_late_map_after_total_growth": True,
            "require_frontier_no_gain_stall": True,
            "require_tare_strategy_quality": True,
            "min_tare_waypoints": 1,
            "min_tare_paths": 1,
            "min_tare_strategy_paths": 1,
            "min_tare_navigation_successes": 1,
            "max_tare_suppressed_waypoint_ratio": 0.75,
        },
        "waypoints": {
            "/way_point": {"samples": 1, "frames": ["map"], "last": [1.2, 0.0, 0.75]},
            "/exploration/way_point": {"samples": 1, "frames": ["map"], "last": [1.2, 0.0, 0.75]},
        },
        "cmd_vel": {"samples": 8, "nonzero_samples": 4, "max_norm": 0.22},
        "odometry": {
            "/nav/odometry": {"samples": 12, "first_xy": [0.0, 0.0], "last_xy": [0.12, 0.02], "delta_m": 0.121},
            "/state_estimation": {"samples": 12, "first_xy": [0.0, 0.0], "last_xy": [0.12, 0.02], "delta_m": 0.121},
        },
        "cloud_coverage": {
            "best_topic": "/nav/terrain_map_ext",
            "best_cells_delta": 12,
            "best_area_delta_m2": 0.75,
            "topics": {
                "/registered_scan": {"samples": 4, "area_delta_m2": 0.75},
                "/nav/registered_cloud": {"samples": 4, "area_delta_m2": 0.75},
                "/nav/map_cloud": {"samples": 4, "area_delta_m2": 0.75},
                "/nav/terrain_map_ext": {"samples": 4, "area_delta_m2": 0.75},
            },
        },
        "paths": {
            "/nav/global_path": {"samples": 1, "nonempty_samples": 1, "max_poses": 5, "frames": ["map"]},
            "/nav/local_path": {"samples": 4, "nonempty_samples": 4, "max_poses": 101, "frames": ["map"]},
        },
        "path_requirements": {
            "/nav/global_path": {
                "observed_nonempty_samples": 1,
                "observed_max_poses": 5,
                "ok": True,
            },
            "/nav/local_path": {
                "observed_nonempty_samples": 4,
                "observed_max_poses": 101,
                "ok": True,
            },
        },
        "scan_requirements": {
            "/registered_scan": {"observed_samples": 4, "ok": True},
            "/nav/registered_cloud": {"observed_samples": 4, "ok": True},
        },
        "map_requirements": {
            "/nav/map_cloud": {"observed_area_delta_m2": 0.75, "ok": True},
            "/nav/terrain_map_ext": {"observed_area_delta_m2": 0.75, "ok": True},
        },
        "hardware_safety": {
            "topics": {"/cmd_vel": ["/vehicle_simulator"], "/nav/cmd_vel": []},
            "blocked_hardware_nodes": [],
        },
        "tare_navigation": {
            "available": True,
            "backend": "tare",
            "started": True,
            "success_count": 1,
            "failure_count": 0,
            "terminal_count": 1,
            "last_navigation_status": {"state": "SUCCESS"},
        },
        "tare_strategy_quality": {
            "checked": True,
            "ok": True,
            "available": True,
            "source": "gateway_exploration_status.tare.stats",
            "backend": "tare",
            "started": True,
            "thresholds": {
                "min_tare_waypoints": 1,
                "min_tare_paths": 1,
                "min_tare_strategy_paths": 1,
                "min_tare_navigation_successes": 1,
                "max_tare_suppressed_waypoint_ratio": 0.75,
            },
            "waypoint_count": 2,
            "path_count": 2,
            "strategy_path_count": 1,
            "suppressed_waypoint_count": 0,
            "suppressed_far_waypoint_count": 0,
            "suppressed_waypoint_ratio": 0.0,
            "navigation_terminal_count": 1,
            "navigation_success_count": 1,
            "navigation_failure_count": 0,
            "reject_reasons": [],
            "blockers": [],
        },
        "late_activity": {
            "odometry": {
                "window_sec": 60.0,
                "required_delta_m": 0.5,
                "observed_best_delta_m": 0.75,
                "ok": True,
            },
            "cmd_vel": {
                "window_sec": 60.0,
                "required_nonzero_samples": 10,
                "observed_nonzero_samples": 12,
                "ok": True,
            },
            "paths": {
                "window_sec": 60.0,
                "topics": {
                    "/nav/local_path": {
                        "required_nonempty_samples": 5,
                        "observed_nonempty_samples": 8,
                        "ok": True,
                    }
                },
                "ok": True,
            },
            "map_growth": {
                "window_sec": 60.0,
                "required_area_delta_m2": 1.0,
                "observed_best_area_delta_m2": 1.25,
                "accepted_flat_after_total_growth": False,
                "ok": True,
            },
        },
        "frontier_no_gain_stall": {
            "checked": True,
            "ok": True,
            "mode": "late_activity_observation",
            "stop_reason": "late_activity_window_verified",
            "required_observation_s": 60.0,
            "observed_s": 60.0,
            "duration_sec": 180.0,
            "late_activity": {
                "odometry_ok": True,
                "cmd_vel_ok": True,
                "paths_ok": True,
                "map_growth_ok": True,
                "map_growth_accepted_flat_after_total_growth": False,
            },
            "tare_navigation": {
                "available": True,
                "success_count": 1,
                "failure_count": 0,
                "terminal_count": 1,
            },
            "blockers": [],
        },
        "runtime_contract": _complete_cmu_unity_runtime_contract_report(),
        "blockers": [],
    }


def _complete_cmu_unity_pct_strict_report() -> dict:
    report = _complete_cmu_unity_runtime_report()
    report["cmd_vel"] = {"samples": 24, "nonzero_samples": 18, "max_norm": 0.52}
    report["odometry"] = {
        "/nav/odometry": {"samples": 32, "first_xy": [0.0, 0.0], "last_xy": [2.6, 0.1], "delta_m": 2.602},
        "/state_estimation": {"samples": 32, "first_xy": [0.0, 0.0], "last_xy": [2.6, 0.1], "delta_m": 2.602},
    }
    report["cloud_coverage"] = {
        "best_topic": "/nav/registered_cloud",
        "best_cells_delta": 2349,
        "best_area_delta_m2": 146.8125,
        "topics": {
            "/nav/registered_cloud": {
                "samples": 4,
                "area_delta_m2": 146.8125,
                "cells_delta": 2349,
                "points_seen": 11200,
                "frames": ["map"],
            },
            "/nav/map_cloud": {
                "samples": 4,
                "area_delta_m2": 146.8125,
                "cells_delta": 2349,
                "points_seen": 11200,
                "frames": ["map"],
            },
            "/nav/terrain_map_ext": {
                "samples": 4,
                "area_delta_m2": 71.6875,
                "cells_delta": 1147,
                "points_seen": 2821,
                "frames": ["map"],
            },
        },
    }
    report["paths"] = {
        "/nav/global_path": {"samples": 2, "nonempty_samples": 1, "max_poses": 5, "frames": ["map"]},
        "/nav/local_path": {"samples": 4, "nonempty_samples": 4, "max_poses": 101, "frames": ["map"]},
    }
    report["path_requirements"] = {
        "/nav/global_path": {
            "required_nonempty_samples": 1,
            "required_min_poses": 2,
            "observed_nonempty_samples": 1,
            "observed_max_poses": 5,
            "ok": True,
        },
        "/nav/local_path": {
            "required_nonempty_samples": 1,
            "required_min_poses": 2,
            "observed_nonempty_samples": 4,
            "observed_max_poses": 101,
            "ok": True,
        },
    }
    report["scan_requirements"] = {
        "/nav/registered_cloud": {"required_samples": 1, "observed_samples": 4, "ok": True},
    }
    report["map_requirements"] = {
        "/nav/map_cloud": {
            "required_area_delta_m2": 2.0,
            "observed_area_delta_m2": 146.8125,
            "ok": True,
        },
        "/nav/terrain_map_ext": {
            "required_area_delta_m2": 2.0,
            "observed_area_delta_m2": 71.6875,
            "ok": True,
        },
    }
    report["hardware_safety"] = {
        "topics": {"/cmd_vel": ["/vehicleSimulator"], "/nav/cmd_vel": []},
        "publishers": {"/cmd_vel": ["/lingtu_cmu_unity_adapter"], "/nav/cmd_vel": ["/lingtu_navigation"]},
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
    }
    report["cmd_vel_exclusive_to_lingtu"] = True
    report["planner_diagnostics"] = {
        "available": True,
        "primary_planner": "pct",
        "selected_planner": "pct",
        "policy": "pct",
        "fallback_used": False,
        "fallback_reason": "",
        "rejected_plan_count": 0,
        "reached_goal": True,
        "last_plan_report": {
            "primary_planner": "pct",
            "selected_planner": "pct",
            "reached_goal": True,
            "rejected_plans": [],
        },
    }
    report["runtime_contract"] = _complete_cmu_unity_runtime_contract_report(
        planner_diagnostics_required=True,
        planner_diagnostics_available=True,
    )
    report["navigation_failure"] = {"failed": False, "state": "REACHED", "reason_codes": [], "failure_reason": ""}
    report["direct_goal_fallback"] = {"used": False, "reason": "", "goal": None, "ts": None}
    return report


def test_gateway_goal_dry_run_gate_publishes_goal_without_cmd_vel():
    pytest.importorskip("fastapi")
    from sim.scripts.gateway_goal_dry_run_gate import run_gate

    report = run_gate(x=1.2, y=-0.4, z=0.0, client_id="pytest")

    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["driver_used"] is False
    assert report["frames"]["published_goal"] == "map"
    assert report["frames"]["cmd_vel"] == "not_published"
    assert report["published"]["goal_pose"] == 1
    assert report["published"]["cmd_vel"] == 0


def test_routecheck_preflight_gate_writes_non_motion_summary(tmp_path: Path):
    pytest.importorskip("fastapi")
    from sim.scripts.routecheck_preflight_gate import run_gate

    summary_path = tmp_path / "routecheck" / "summary.json"
    report = run_gate(
        map_name="pytest_map",
        x=1.2,
        y=-0.4,
        yaw=0.1,
        planner="pct",
        json_out=summary_path,
        client_id="pytest",
    )

    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(report), encoding="utf-8")
    ok, blockers, evidence = server_sim_closure._eval_routecheck_preflight(report)

    assert ok is True
    assert blockers == []
    assert report["mode"] == "routecheck_non_motion"
    assert report["outcome"] == "pass"
    assert report["exit_status"] == 0
    assert report["non_motion"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["published"] == {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0}
    assert report["phases"]["baseline"]["feasible"] is True
    assert report["phases"]["candidate"]["feasible"] is True
    assert evidence["phases"]["baseline"]["selected_planner"] == "pct"
    assert Path(report["artifacts"]["baseline_files"]["plan"]).exists()
    assert Path(report["artifacts"]["candidate_files"]["plan_summary"]).exists()


def test_blocked_route_replan_gate_is_required_local_non_motion():
    spec = next(
        item for item in server_sim_closure.GATES if item.name == "blocked_route_replan_preflight"
    )

    assert "sim/scripts/blocked_route_replan_gate.py" in spec.command
    assert "server_sim_closure/blocked_route_replan/report.json" in spec.command
    assert spec.host_requirements == server_sim_closure.LOCAL_NON_MOTION_HOST_REQUIREMENTS


def test_server_sim_closure_accepts_blocked_route_replan_report(tmp_path: Path):
    from sim.scripts.blocked_route_replan_gate import run_gate

    report = run_gate(
        map_name="pytest_map",
        x=2.0,
        y=0.0,
        planner="pct",
        json_out=tmp_path / "blocked_route" / "report.json",
        client_id="pytest",
    )
    report_path = _write_json(tmp_path / "blocked_route_replan.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"blocked_route_replan_preflight": report_path},
        required={"blocked_route_replan_preflight"},
        include_optional=False,
    )

    gate = summary["gates"]["blocked_route_replan_preflight"]
    assert gate["ok"] is True
    assert summary["verified"]["blocked_route_replan_preflight"] is True
    assert gate["evidence"]["baseline_path_intersects_block"] is True
    assert gate["evidence"]["candidate_path_avoids_block"] is True


def test_server_sim_closure_rejects_weak_blocked_route_replan_report(tmp_path: Path):
    weak = _write_json(
        tmp_path / "blocked_route_weak.json",
        {
            "schema_version": "lingtu.blocked_route_replan_gate.v1",
            "ok": True,
            "mode": "blocked_route_replan_non_motion",
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "gateway_used": True,
            "driver_used": False,
            "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
            "baseline_path_intersects_block": False,
            "candidate_path_avoids_block": False,
            "candidate_route_changed": False,
            "blocked_route_replanned": False,
            "thresholds": {"min_block_clearance_m": 0.25},
            "phases": {
                "baseline": {
                    "non_motion": True,
                    "can_accept_goal": True,
                    "active_cmd_source_before": "none",
                    "feasible": True,
                    "count": 3,
                    "selected_planner": "pct",
                    "intersects_block": False,
                },
                "blocked_candidate": {
                    "non_motion": True,
                    "can_accept_goal": True,
                    "active_cmd_source_before": "none",
                    "feasible": True,
                    "count": 3,
                    "selected_planner": "pct",
                    "intersects_block": True,
                    "min_block_clearance_m": 0.0,
                    "reasons": [],
                },
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"blocked_route_replan_preflight": weak},
        required={"blocked_route_replan_preflight"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "baseline_path_intersects_block is not true" in gaps
    assert "candidate_path_avoids_block is not true" in gaps
    assert "candidate_route_changed is not true" in gaps
    assert "blocked_route_replanned is not true" in gaps


def test_navigation_replay_deviation_gate_is_required_local_non_motion():
    spec = next(
        item for item in server_sim_closure.GATES if item.name == "navigation_replay_deviation"
    )

    assert "sim/scripts/navigation_replay_deviation_gate.py" in spec.command
    assert "--routecheck-report artifacts/server_sim_closure/routecheck/summary.json" in spec.command
    assert "--write-trace artifacts/server_sim_closure/navigation_replay_deviation/trace.json" in spec.command
    assert "navigation_replay_deviation/trace.json" in spec.command
    assert spec.host_requirements == server_sim_closure.LOCAL_NON_MOTION_HOST_REQUIREMENTS


def test_server_sim_closure_accepts_navigation_replay_deviation_report(tmp_path: Path):
    from sim.scripts.navigation_replay_deviation_gate import build_report

    report_path = _write_json(tmp_path / "navigation_replay_deviation.json", build_report(fixture=True))

    summary = server_sim_closure.summarize(
        report_overrides={"navigation_replay_deviation": report_path},
        required={"navigation_replay_deviation"},
        include_optional=False,
    )

    gate = summary["gates"]["navigation_replay_deviation"]
    assert gate["ok"] is True
    assert summary["verified"]["navigation_replay_deviation"] is True
    assert gate["evidence"]["cmd_vel_nonzero"] > 0
    assert gate["evidence"]["final_distance_m"] <= 0.8


def test_server_sim_closure_materializes_navigation_replay_deviation_topic_jsonl(
    tmp_path: Path,
):
    topic_jsonl = _write_navigation_topic_jsonl(tmp_path / "recorded_topics.jsonl")
    report_path = tmp_path / "navigation_replay_deviation" / "report.json"
    trace_path = tmp_path / "navigation_replay_deviation" / "trace.json"

    materialized_report, materialized = (
        server_sim_closure.materialize_navigation_replay_deviation_topic_jsonl(
            topic_jsonl,
            report_path=report_path,
            trace_path=trace_path,
        )
    )

    assert materialized_report == report_path
    assert materialized["ok"] is True
    assert materialized["source"] == "recorded_topic_jsonl"
    assert materialized["trace_kind"] == "recorded_topic_replay"
    assert report_path.exists()
    assert trace_path.exists()

    summary = server_sim_closure.summarize(
        report_overrides={"navigation_replay_deviation": report_path},
        required={"navigation_replay_deviation"},
        include_optional=False,
    )

    gate = summary["gates"]["navigation_replay_deviation"]
    assert summary["verified"]["navigation_replay_deviation"] is True
    assert gate["evidence"]["trace_source"] == str(topic_jsonl)
    assert gate["evidence"]["cmd_vel_nonzero"] > 0


def test_server_sim_closure_cli_accepts_navigation_replay_deviation_topic_jsonl(
    tmp_path: Path,
    monkeypatch,
    capsys,
):
    topic_jsonl = _write_navigation_topic_jsonl(tmp_path / "recorded_topics.jsonl")
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--required",
            "navigation_replay_deviation",
            "--required-only",
            "--navigation-replay-deviation-topic-jsonl",
            str(topic_jsonl),
            "--json-out",
            "-",
            "--strict",
        ],
    )

    assert server_sim_closure.main() == 0
    report = json.loads(capsys.readouterr().out)

    assert report["ok"] is True
    assert report["execution_mode"] == "summary_only"
    assert report["run_missing"] is False
    assert report["verified"]["navigation_replay_deviation"] is True
    assert report["materialized_inputs"][0]["source_path"] == str(topic_jsonl)
    gate = report["gates"]["navigation_replay_deviation"]
    assert gate["evidence"]["trace_kind"] == "recorded_topic_replay"
    assert Path(gate["path"]).exists()


def test_server_sim_closure_rejects_weak_navigation_replay_deviation_report(tmp_path: Path):
    from sim.scripts.navigation_replay_deviation_gate import build_report

    weak_report = build_report(fixture=True)
    weak_report["ok"] = True
    weak_report["tracking_error_p95_m"] = 2.0
    weak_report["cmd_vel_nonzero_ratio"] = 0.0
    report_path = _write_json(tmp_path / "navigation_replay_deviation_weak.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"navigation_replay_deviation": report_path},
        required={"navigation_replay_deviation"},
        include_optional=False,
    )

    gate = summary["gates"]["navigation_replay_deviation"]
    assert gate["ok"] is False
    assert summary["verified"]["navigation_replay_deviation"] is False
    assert any("tracking_error_p95_m above threshold" in gap for gap in summary["remaining_gaps"])
    assert any("cmd_vel_nonzero_ratio below threshold" in gap for gap in summary["remaining_gaps"])


def test_server_sim_closure_finds_default_large_terrain_report_path():
    spec = next(item for item in server_sim_closure.GATES if item.name == "large_terrain")

    assert "artifacts/server_sim_closure/large_terrain/report.json" in spec.default_patterns


def test_server_sim_closure_native_pct_command_loads_ros_python_environment():
    spec = next(item for item in server_sim_closure.GATES if item.name == "native_pct_mujoco")

    assert "source /opt/ros/humble/setup.bash" in spec.command
    assert "/opt/ros/humble/local/lib/python3.10/dist-packages" in spec.command
    assert "sim/scripts/mujoco/native_pct_gate.py" in spec.command
    assert "sim/scripts/mujoco/launch_fastlio2_live.sh" not in spec.command
    assert "--route terrain_short" in spec.command
    assert "--timeout-s 80" in spec.command
    assert "--near-field-stop-distance 0.35" in spec.command
    assert (
        "artifacts/server_sim_closure/native_pct_mujoco/report.*.server.json"
        in spec.default_patterns
    )
    assert (
        "--source-report artifacts/server_sim_closure/large_terrain/report.json"
        in spec.command
    )


def test_server_sim_closure_policy_nav_command_uses_verified_policy_gait_params():
    spec = next(item for item in server_sim_closure.GATES if item.name == "policy_nav")

    assert "--nav-duration 18" in spec.command
    assert "--nav-planner-backend octoplanner3d" in spec.command
    assert "--nav-local-planner-backend nanobind" in spec.command
    assert "--nav-path-follower-backend nav_kernel" in spec.command
    assert "--nav-path-min-speed 0.25" in spec.command
    assert "--nav-path-max-speed 0.6" in spec.command
    assert "--nav-max-angular-z 0.15" in spec.command
    assert spec.host_requirements == server_sim_closure.LOCAL_NUMERIC_SIM_HOST_REQUIREMENTS


def test_server_sim_closure_gazebo_runtime_command_starts_runtime_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "gazebo_runtime")

    assert "sim/scripts/gazebo_runtime_gate.py" in spec.command
    assert "--check-nav-loop" in spec.command
    assert "--check-frontier-exploration" in spec.command
    assert "--check-cumulative-map" in spec.command
    assert "--check-tare-contract" in spec.command
    assert "artifacts/server_sim_closure/gazebo_runtime_explore/report_grid_astar_odomfoot.json" in spec.default_patterns
    assert "--nav-goal-x 3.0" in spec.command
    assert "DOMAIN=${ROS_DOMAIN_ID:-30}" in spec.command
    assert "--gz-partition $PART" in spec.command
    assert "gazebo_runtime_explore/report_grid_astar_odomfoot.json" in spec.command
    assert "--launch-log artifacts/server_sim_closure/gazebo_runtime_explore/launch_grid_astar_odomfoot.log" in spec.command
    assert "tf_contract_smoke.py" not in spec.command
    assert "source /opt/ros/humble/setup.bash" in spec.command
    assert spec.host_requirements == server_sim_closure.ROS2_GAZEBO_NAV_HOST_REQUIREMENTS


def test_gazebo_runtime_gate_uses_existing_sim_script_paths():
    gate_source = (REPO_ROOT / "sim/scripts/gazebo_runtime_gate.py").read_text(
        encoding="utf-8"
    )
    launcher_source = (
        REPO_ROOT / "sim/scripts/launch_lingtu_gazebo_industrial_demo.sh"
    ).read_text(encoding="utf-8")

    expected_paths = (
        "sim/scripts/tf_contract_smoke.py",
        "sim/scripts/gazebo_nav_loop_smoke.py",
        "sim/scripts/gazebo_frontier_exploration_smoke.py",
        "sim/planning/sim_navigation.launch.py",
        "sim/planning/lingtu_industrial_demo.rviz",
    )
    combined_source = f"{gate_source}\n{launcher_source}"
    for path in expected_paths:
        assert (REPO_ROOT / path).exists()
        assert Path(path).name in combined_source

    assert "tests/integration/gazebo_frontier_exploration_smoke.py" not in gate_source
    assert "tests/integration/gazebo_frontier_exploration_smoke.py" not in launcher_source
    assert "tests/planning/sim_navigation.launch.py" not in gate_source
    assert "tests/planning/sim_navigation.launch.py" not in launcher_source


def test_gazebo_frontier_smoke_reports_post_pass_stall_metric():
    from sim.scripts.gazebo_frontier_exploration_smoke import GazeboFrontierExplorationResult

    result = GazeboFrontierExplorationResult(
        frontier_no_gain_stall={
            "checked": True,
            "ok": True,
            "stop_reason": "post_pass_observation_elapsed",
            "required_observation_s": 5.0,
            "observed_s": 5.0,
        }
    )

    payload = result.as_dict()

    assert payload["frontier_no_gain_stall"]["checked"] is True
    assert payload["frontier_no_gain_stall"]["ok"] is True
    assert payload["frontier_no_gain_stall"]["stop_reason"] == "post_pass_observation_elapsed"


def test_server_sim_closure_cmu_unity_sim_command_runs_preflight_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "cmu_unity_sim")

    assert "sim/scripts/cmu_unity_sim_gate.py" in spec.command
    assert "server_sim_closure/cmu_unity_sim/report.json" in spec.command
    assert "--strict" in spec.command
    assert spec.host_requirements == server_sim_closure.LOCAL_NUMERIC_SIM_HOST_REQUIREMENTS


def test_server_sim_closure_cmu_unity_runtime_command_runs_runtime_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "cmu_unity_runtime")

    assert "source /opt/ros/humble/setup.bash" in spec.command
    assert "sim/scripts/cmu_unity_runtime_gate.py" in spec.command
    assert "server_sim_closure/cmu_unity_runtime/report.json" in spec.command
    assert "--strict" in spec.command


def test_server_sim_closure_cmu_unity_pct_strict_command_runs_no_fallback_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "cmu_unity_pct_strict")

    assert "sim/scripts/launch_cmu_unity_lingtu_runtime.sh start --gate --rviz" in spec.command
    assert "LINGTU_CMU_PLANNER=pct" in spec.command
    assert "LINGTU_CMU_START_CMU_TARE=1" in spec.command
    assert "LINGTU_CMU_TARE_SCENARIO=${LINGTU_CMU_TARE_SCENARIO:-indoor_large}" in spec.command
    assert "LINGTU_CMU_TARE_AUTOSTART=0" in spec.command
    assert "LINGTU_CMU_ENABLE_FRONTIER=0" in spec.command
    assert "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}" in spec.command
    assert "LINGTU_CMU_AUTO_SESSION=1" in spec.command
    assert "LINGTU_CMU_EXPLORATION_AUTO_START=0" in spec.command
    assert "LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK=0" in spec.command
    assert "LINGTU_CMU_AUTO_TOMOGRAM=${LINGTU_CMU_AUTO_TOMOGRAM:-1}" in spec.command
    assert "LINGTU_CMU_TOMOGRAM_TOPICS=${LINGTU_CMU_TOMOGRAM_TOPICS:-/nav/map_cloud,/nav/terrain_map_ext}" in spec.command
    assert "LINGTU_CMU_TOMOGRAM_MODE=${LINGTU_CMU_TOMOGRAM_MODE:-official}" in spec.command
    assert "LINGTU_CMU_GATE_TIMEOUT_SEC=${LINGTU_CMU_GATE_TIMEOUT_SEC:-240}" in spec.command
    assert "LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS=${LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS:-3}" in spec.command
    assert "LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS=${LINGTU_CMU_GATE_REQUIRED_MAP_TOPICS:-/nav/map_cloud,/nav/terrain_map_ext}" in spec.command


def test_server_sim_closure_dynamic_obstacle_command_uses_nanobind_module_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "dynamic_obstacle_local_planner")

    assert "sim/scripts/dynamic_obstacle_local_planner_gate.py" in spec.command
    assert "--backend nanobind" in spec.command
    assert "server_sim_closure/dynamic_obstacle_local_planner/report.json" in spec.command
    assert spec.host_requirements == server_sim_closure.LOCAL_NUMERIC_NAV_HOST_REQUIREMENTS


def test_server_sim_closure_mujoco_tare_command_uses_live_tare_mode():
    spec = next(item for item in server_sim_closure.GATES if item.name == "mujoco_tare_exploration")

    assert "source /opt/ros/humble/setup.bash" in spec.command
    assert "mujoco/launch_fastlio2_live.sh tare" in spec.command
    assert "server_sim_closure/mujoco_tare_exploration" in spec.command
    assert "industrial_park" in spec.command
    assert "LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM" in spec.command
    assert any("mujoco_tare_exploration" in pattern for pattern in spec.default_patterns)


def test_server_sim_closure_saved_map_relocalize_command_uses_runtime_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "saved_map_relocalize")

    assert "sim/scripts/saved_map_relocalize_runtime_gate.py" in spec.command
    assert "source /opt/ros/humble/setup.bash" in spec.command
    assert "source install/setup.bash" in spec.command
    assert "export MUJOCO_GL=${MUJOCO_GL:-egl}" in spec.command
    assert "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}" in spec.command
    assert "/opt/ros/humble/local/lib/python3.10/dist-packages" in spec.command
    assert "--map-pcd latest" in spec.command
    assert "--duration 12" in spec.command
    assert "--scan-time-profile map_metadata" in spec.command
    assert "--live-drive-source fixed" in spec.command
    assert "--fastlio-lidar-input timed_pointcloud2" in spec.command
    assert "--fastlio-ieskf-max-iter 10" in spec.command
    assert "--runtime-fault-confirm-samples 6" in spec.command
    assert "--runtime-motion-fault-min-sim-m 1.0" in spec.command
    assert "server_sim_closure/saved_map_relocalize_runtime/report.json" in spec.command
    assert "--strict" in spec.command
    assert any("MuJoCo/Fast-LIO live feed" in item for item in spec.host_requirements)
    assert any("localizer runtime" in item for item in spec.host_requirements)


def test_server_sim_closure_pct_saved_map_navigation_command_uses_composed_gate():
    spec = next(item for item in server_sim_closure.GATES if item.name == "pct_saved_map_navigation")

    assert "sim/scripts/pct_saved_map_navigation_gate.py" in spec.command
    assert "/usr/bin/python3" not in spec.command
    assert "python3 sim/scripts/pct_saved_map_navigation_gate.py" in spec.command
    assert "source /opt/ros/humble/setup.bash" in spec.command
    assert "export MUJOCO_GL=${MUJOCO_GL:-egl}" in spec.command
    assert "export PYOPENGL_PLATFORM=${PYOPENGL_PLATFORM:-egl}" in spec.command
    assert (
        "--relocalize-report "
        "artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json"
    ) in spec.command
    assert "--max-relocalize-report-age-s 86400" in spec.command
    assert "--goal 4.5 3.0" in spec.command
    assert "--timeout-s 80" in spec.command
    assert "--near-field-stop-distance 0.35" in spec.command
    assert "server_sim_closure/pct_saved_map_navigation/report.json" in spec.command
    assert "--strict" in spec.command


def test_server_sim_closure_routecheck_command_is_non_motion_preflight():
    spec = next(item for item in server_sim_closure.GATES if item.name == "routecheck_preflight")

    assert "sim/scripts/routecheck_preflight_gate.py" in spec.command
    assert "scripts/lingtu routecheck" not in spec.command
    assert "--json-out artifacts/server_sim_closure/routecheck/summary.json" in spec.command
    assert "--strict" in spec.command


def test_server_sim_closure_accepts_complete_report_set(tmp_path: Path):
    multifloor = _write_json(tmp_path / "multifloor.json", _complete_multifloor_report())
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [
                        {
                            "planner": "pct",
                            "native_backend_used": True,
                        }
                    ],
                }
            ],
        },
    )
    native = _write_json(
        tmp_path / "native.json",
        {
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
            "pct_native_backend_used": True,
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
                "native_backend_used": True,
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
            "map_artifacts": _same_source_map_artifacts("native_acceptance"),
            "obstacle_aware": {
                "enabled": True,
                "metadata_points": 32,
            },
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
        },
    )
    dynamic_obstacles = _write_json(
        tmp_path / "dynamic_obstacles.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_actual": "nanobind",
            "dynamic_replan_verified": True,
            "obstacle_response_verified": True,
            "clear_path_recovery_verified": True,
            "min_clearance_m": 0.42,
            "frames": {"local_path": "map", "cmd_vel": "not_published"},
            "phases": [
                {"name": "clear_initial", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_left", "path_count": 101, "path_frame_id": "map", "avoidance_side": "right"},
                {"name": "obstacle_right", "path_count": 101, "path_frame_id": "map", "avoidance_side": "left"},
                {"name": "obstacle_center", "path_count": 101, "path_frame_id": "map", "avoidance_side": "right"},
                {"name": "clear_recovered", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
            ],
        },
    )
    fastlio = _write_json(
        tmp_path / "fastlio.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 12,
                "nav_odometry": 12,
                "nav_registered_cloud": 8,
                "nav_map_cloud": 8,
            },
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
        },
    )
    mujoco_tare = _write_json(
        tmp_path / "mujoco_tare.json",
        _complete_fastlio2_tare_report(),
    )
    policy = _write_json(
        tmp_path / "policy.json",
        {
            "passed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checks": [
                {
                    "mode": "direct_policy",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "contacts": {
                        "foot_contact_sample_count": 20,
                        "unique_feet_count": 4,
                        "non_foot_ground_contacts": 0,
                    },
                },
                {
                    "mode": "full_stack_policy_nav",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "global_planner_backend_status": {
                        "configured_backend": "octoplanner3d",
                        "backend": "octoplanner3d",
                        "degraded": False,
                    },
                    "local_planner_backend_actual": "nanobind",
                    "path_follower_backend_actual": "nav_kernel",
                    "contacts": {
                        "foot_contact_sample_count": 20,
                        "unique_feet_count": 4,
                        "non_foot_ground_contacts": 0,
                    },
                    "seen": {
                        "direct_fallback": 0,
                        "local_path": 4,
                        "path_follower_cmd": 4,
                        "mux_cmd": 4,
                        "waypoints": 1,
                    },
                    "frames": {"goal": "map", "cmd_vel": "base_link"},
                },
            ],
        },
    )
    gateway = _write_json(
        tmp_path / "gateway.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "gateway_used": True,
            "driver_used": False,
            "published": {"goal_pose": 1, "cmd_vel": 0, "stop_cmd": 0},
            "frames": {"published_goal": "map", "cmd_vel": "not_published"},
        },
    )
    routecheck_phase = {
        "schema_version": 1,
        "phase": "baseline",
        "non_motion": True,
        "navigation_state": "idle",
        "can_accept_goal": True,
        "active_cmd_source_before": "none",
        "feasible": True,
        "count": 3,
        "planner": "pct",
        "selected_planner": "pct",
        "plan_safety_policy": "fallback_astar",
        "path_safety_ok": True,
        "fallback_reason": "",
        "rejected_plan_count": 0,
        "reasons": [],
    }
    routecheck = _write_json(
        tmp_path / "routecheck_summary.json",
        {
            "schema_version": 1,
            "mode": "routecheck_non_motion",
            "outcome": "pass",
            "exit_status": 0,
            "non_motion": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "gateway_used": True,
            "driver_used": False,
            "map": "demo",
            "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "phases": {
                "baseline": routecheck_phase,
                "candidate": {**routecheck_phase, "phase": "candidate"},
            },
            "published": {"goal_pose": 0, "cmd_vel": 0, "stop_cmd": 0},
            "artifacts": {
                "baseline": "/tmp/route/baseline",
                "candidate": "/tmp/route/candidate",
                "after_rollback": "/tmp/route/after_rollback",
            },
        },
    )
    gazebo = _write_json(tmp_path / "gazebo.json", _complete_gazebo_report())
    cmu_unity = _write_json(tmp_path / "cmu_unity.json", _complete_cmu_unity_report())
    cmu_unity_runtime = _write_json(
        tmp_path / "cmu_unity_runtime.json",
        _complete_cmu_unity_runtime_report(),
    )
    cmu_unity_pct_strict = _write_json(
        tmp_path / "cmu_unity_pct_strict.json",
        _complete_cmu_unity_pct_strict_report(),
    )
    saved_map_report = _complete_saved_map_relocalize_runtime_report()
    saved_map_report["map_pcd"] = str(_write_artifact(tmp_path / "same_source_map" / "map.pcd"))
    saved_map_relocalize = _write_json(
        tmp_path / "saved_map_relocalize.json",
        saved_map_report,
    )
    pct_report = _complete_pct_saved_map_navigation_report()
    pct_report["tomogram"] = str(_write_artifact(tmp_path / "same_source_map" / "tomogram.pickle"))
    pct_report["relocalize_report"] = str(saved_map_relocalize)
    pct_saved_map_navigation = _write_json(
        tmp_path / "pct_saved_map_navigation.json",
        pct_report,
    )

    summary = server_sim_closure.summarize(
        report_overrides={
            "multifloor_exploration": multifloor,
            "large_terrain": large,
            "native_pct_mujoco": native,
            "dynamic_obstacle_local_planner": dynamic_obstacles,
            "fastlio2_live": fastlio,
            "mujoco_tare_exploration": mujoco_tare,
            "policy_nav": policy,
            "gateway_dry_run": gateway,
            "routecheck_preflight": routecheck,
            "gazebo_runtime": gazebo,
            "cmu_unity_sim": cmu_unity,
            "cmu_unity_runtime": cmu_unity_runtime,
            "cmu_unity_pct_strict": cmu_unity_pct_strict,
            "saved_map_relocalize": saved_map_relocalize,
            "pct_saved_map_navigation": pct_saved_map_navigation,
        },
        required={
            "multifloor_exploration",
            "large_terrain",
            "native_pct_mujoco",
            "dynamic_obstacle_local_planner",
            "fastlio2_live",
            "mujoco_tare_exploration",
            "policy_nav",
            "gateway_dry_run",
            "routecheck_preflight",
            "gazebo_runtime",
            "cmu_unity_sim",
            "cmu_unity_runtime",
            "cmu_unity_pct_strict",
            "saved_map_relocalize",
            "pct_saved_map_navigation",
        },
    )

    assert summary["ok"] is True, summary["remaining_gaps"]
    assert summary["simulation_only"] is True
    assert summary["real_robot_motion"] is False
    assert summary["cmd_vel_sent_to_hardware"] is False
    assert summary["missing_or_failed"] == []
    assert all(summary["verified"][name] for name in summary["required"])
    routecheck_gate = summary["gates"]["routecheck_preflight"]
    assert routecheck_gate["path"] == str(routecheck)
    assert routecheck_gate["report_mtime"] >= routecheck.stat().st_mtime - 0.001
    assert routecheck_gate["report_age_s"] >= 0.0


def test_server_sim_closure_rejects_weak_multifloor_exploration_report(tmp_path: Path):
    weak = _complete_multifloor_report()
    weak["frontier_loop_enabled"] = False
    weak["exploration"] = {
        "ok": True,
        "closed_loop": False,
        "probe_mode": "candidate_only",
        "rounds": [],
    }
    weak["cases"][-1]["native_pct_gate"] = {
        "ok": True,
        "floor_graph_composition_verified": False,
        "native_pct_feasible_segments": 1,
    }
    report_path = _write_json(tmp_path / "weak_multifloor.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"multifloor_exploration": report_path},
        required={"multifloor_exploration"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["multifloor_exploration"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frontier_loop_enabled is not true" in gaps
    assert "exploration.closed_loop is not true" in gaps
    assert "exploration probe is not frontier_navigation_closed_loop" in gaps
    assert "cross_floor floor-graph composition not verified" in gaps


def test_server_sim_closure_multifloor_surfaces_pct_runtime_blocker(tmp_path: Path):
    weak = _complete_multifloor_report()
    weak["passed"] = False
    weak["production_local_planner_verified"] = False
    weak["native_pct_gate_passed_count"] = 0
    weak["native_pct_blocked_count"] = 4
    for case in weak["cases"]:
        case["passed"] = False
        case["native_pct_gate"] = {
            "ok": False,
            "status": "blocked",
            "runtime": {
                "ok": False,
                "error": "No runnable PCT native modules for arch=x86_64 python=py313",
            },
        }
    report_path = _write_json(tmp_path / "multifloor_pct_runtime_missing.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"multifloor_exploration": report_path},
        required={"multifloor_exploration"},
    )

    gate = summary["gates"]["multifloor_exploration"]
    assert "PCT native runtime unavailable" in gate["blockers"]
    assert "environment_runtime" in summary["algorithm_validation"]["gate_categories"][
        "multifloor_exploration"
    ]
    action = summary["algorithm_validation"]["next_actions"][0]
    assert action["gate"] == "multifloor_exploration"
    assert action["category"] == "environment_runtime"


def test_server_sim_closure_rejects_weak_gateway_dry_run_report(tmp_path: Path):
    weak = _write_json(
        tmp_path / "gateway_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "gateway_used": False,
            "driver_used": True,
            "published": {"goal_pose": 1, "cmd_vel": 1},
            "frames": {"published_goal": "map", "cmd_vel": "base_link"},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"gateway_dry_run": weak},
        required={"gateway_dry_run"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["gateway_dry_run"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "gateway_used is not true" in gaps
    assert "driver_used is not false" in gaps
    assert "published.cmd_vel is not 0" in gaps
    assert "published.stop_cmd is missing" in gaps


def test_server_sim_closure_reports_remaining_gaps(tmp_path: Path):
    native = _write_json(
        tmp_path / "native.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 1.2,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": True, "metadata_points": 24},
            "obstacle_clearance": {"checked": True, "collision": True},
            "local_path_samples": [{"frame_id": "body", "point_count": 4}],
            "trajectory_quality": {"ok": True},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={
            "native_pct_mujoco": native,
            "fastlio2_live": tmp_path / "missing_fastlio.json",
        },
        required={"native_pct_mujoco", "fastlio2_live"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    assert "native_pct_mujoco" in summary["missing_or_failed"]
    assert "fastlio2_live" in summary["missing_or_failed"]
    assert any("collision is true" in gap for gap in summary["remaining_gaps"])
    assert any("missing_fastlio.json" in gap for gap in summary["remaining_gaps"])
    assert summary["gates"]["fastlio2_live"]["exists"] is False
    assert summary["gates"]["fastlio2_live"]["status"] == "missing"
    assert "report_age_s" not in summary["gates"]["fastlio2_live"]


def test_server_sim_closure_separates_optional_gaps(tmp_path: Path, monkeypatch):
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [
                        {
                            "planner": "pct",
                            "native_backend_used": True,
                        }
                    ],
                }
            ],
        },
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={"large_terrain": large},
        required={"large_terrain"},
    )

    assert summary["ok"] is True
    assert summary["missing_or_failed"] == []
    assert summary["remaining_gaps"] == []
    assert "large_terrain" not in summary["optional_missing_or_failed"]
    assert "routecheck_preflight" in summary["optional_missing_or_failed"]
    assert any("routecheck_preflight" in gap for gap in summary["optional_gaps"])


def test_server_sim_closure_can_summarize_required_only(tmp_path: Path, monkeypatch):
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [{"planner": "pct", "native_backend_used": True}],
                }
            ],
        },
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={"large_terrain": large},
        required={"large_terrain"},
        include_optional=False,
    )

    assert summary["ok"] is True
    assert summary["execution_mode"] == "summary_only"
    assert summary["run_missing"] is False
    assert summary["gate_runs"] == []
    assert summary["initial_missing_or_failed"] == []
    assert summary["include_optional"] is False
    assert set(summary["gates"]) == {"large_terrain"}
    assert summary["optional_missing_or_failed"] == []
    assert summary["optional_gaps"] == []


def test_server_sim_closure_summarizes_algorithm_backends_from_gate_reports(tmp_path: Path):
    local_planner_evidence = {
        "requested": "nanobind",
        "configured_backend": "nanobind",
        "backend_actual": "nanobind",
        "native_backend_used": True,
        "exercised_by": "command_flow",
    }
    path_follower_evidence = {
        "requested": "nav_kernel",
        "configured_backend": "nav_kernel",
        "backend_actual": "nav_kernel",
        "native_backend_used": True,
        "exercised_by": "command_flow",
    }
    multifloor_report = _complete_multifloor_report()
    for case in multifloor_report["cases"]:
        case["algorithm_backends"] = {
            "local_planner": local_planner_evidence,
            "path_follower": path_follower_evidence,
        }
        case["command_flow"] = {
            **case["command_flow"],
            "algorithm_backends": case["algorithm_backends"],
        }
    multifloor = _write_json(tmp_path / "multifloor.json", multifloor_report)
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "algorithm_backends": {
                "local_planner": {
                    "status": "not_exercised",
                    "exercised_by": "large_terrain_global_planning_assets",
                },
                "path_follower": {
                    "status": "not_exercised",
                    "exercised_by": "large_terrain_global_planning_assets",
                },
            },
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [{"planner": "pct", "native_backend_used": True}],
                }
            ],
        },
    )
    dynamic = _write_json(
        tmp_path / "dynamic.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_actual": "nanobind",
            "dynamic_replan_verified": True,
            "obstacle_response_verified": True,
            "clear_path_recovery_verified": True,
            "min_clearance_m": 0.42,
            "algorithm_backends": {
                "local_planner": local_planner_evidence,
                "path_follower": {
                    "status": "not_exercised",
                    "exercised_by": "not_exercised",
                },
            },
            "phases": [
                {"name": "clear_initial", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_left", "path_count": 101, "path_frame_id": "map", "avoidance_side": "right"},
                {"name": "obstacle_right", "path_count": 101, "path_frame_id": "map", "avoidance_side": "left"},
                {"name": "obstacle_center", "path_count": 101, "path_frame_id": "map", "avoidance_side": "right"},
                {"name": "clear_recovered", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={
            "multifloor_exploration": multifloor,
            "large_terrain": large,
            "dynamic_obstacle_local_planner": dynamic,
        },
        required={
            "multifloor_exploration",
            "large_terrain",
            "dynamic_obstacle_local_planner",
        },
        include_optional=False,
    )

    assert summary["ok"] is True
    backends = summary["algorithm_backends"]
    assert backends["multifloor_exploration"]["local_planner"]["backend_actual"] == "nanobind"
    assert backends["multifloor_exploration"]["path_follower"]["backend_actual"] == "nav_kernel"
    assert backends["large_terrain"]["local_planner"]["status"] == "not_exercised"
    assert backends["dynamic_obstacle_local_planner"]["local_planner"]["backend_actual"] == "nanobind"
    assert backends["dynamic_obstacle_local_planner"]["path_follower"]["status"] == "not_exercised"
    assert (
        summary["gates"]["dynamic_obstacle_local_planner"]["evidence"]["algorithm_backends"]
        == backends["dynamic_obstacle_local_planner"]
    )


def test_server_sim_closure_can_require_fresh_reports(tmp_path: Path):
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [{"planner": "pct", "native_backend_used": True}],
                }
            ],
        },
    )
    old_mtime = server_sim_closure.time.time() - 120.0
    os.utime(large, (old_mtime, old_mtime))

    summary = server_sim_closure.summarize(
        report_overrides={"large_terrain": large},
        required={"large_terrain"},
        max_report_age_s=1.0,
    )

    assert summary["ok"] is False
    assert summary["max_report_age_s"] == 1.0
    assert summary["verified"]["large_terrain"] is False
    assert "large_terrain" in summary["missing_or_failed"]
    assert summary["gates"]["large_terrain"]["status"] == "failed"
    gaps = "\n".join(summary["remaining_gaps"])
    assert "report_age_s" in gaps
    assert "max_report_age_s" in gaps


def test_server_sim_closure_summary_lists_missing_required_commands(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"large_terrain"},
        include_optional=False,
    )

    assert summary["ok"] is False
    assert summary["required_gate_sequence"] == ["large_terrain"]
    assert [item["name"] for item in summary["missing_required_commands"]] == [
        "large_terrain"
    ]
    command = summary["missing_required_commands"][0]
    assert "large_terrain_nav_validation.py" in command["command"]
    assert "LINGTU_PCT_OPTIMIZE_TRAJECTORY=1" in command["command"]
    assert command["expected_report_path"] == (
        "artifacts/server_sim_closure/large_terrain/report.json"
    )
    assert "artifacts/large_terrain_nav_validation*/report.json" in command["accepted_patterns"]
    assert any("PCT native extension modules" in item for item in command["host_requirements"])
    assert any("CPython 3.10" in item for item in command["host_requirements"])
    assert summary["gates"]["large_terrain"]["host_requirements"] == command["host_requirements"]
    assert summary["gates"]["large_terrain"]["expected_report_path"] == command["expected_report_path"]
    assert summary["host_requirements"]["large_terrain"] == command["host_requirements"]

    for container in (
        summary["gates"]["large_terrain"],
        command,
        summary["next_actions"][0],
    ):
        assert "expected_report_path" in container
        assert "accepted_patterns" in container
        assert "host_requirements" in container


def test_server_sim_closure_run_missing_executes_missing_required_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    calls: list[tuple[object, dict]] = []

    def fake_runner(command: object, **kwargs: object) -> subprocess.CompletedProcess[object]:
        calls.append((command, kwargs))
        _write_json(
            tmp_path / "artifacts/server_sim_closure/large_terrain/report.json",
            {
                "ok": True,
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "cases": [
                    {
                        "route": "terrain_long",
                        "path_safety": {"ok": True},
                        "planning": [
                            {"planner": "pct", "native_backend_used": True},
                        ],
                    }
                ],
            },
        )
        return subprocess.CompletedProcess(command, 0)

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={"large_terrain"},
        include_optional=False,
        runner=fake_runner,
    )

    assert summary["ok"] is True, summary["remaining_gaps"]
    assert summary["initial_missing_or_failed"] == ["large_terrain"]
    assert summary["execution_mode"] == "run_missing"
    assert summary["run_missing"] is True
    assert [item["name"] for item in summary["gate_runs"]] == ["large_terrain"]
    assert summary["gate_runs"][0]["returncode"] == 0
    assert summary["gate_runs"][0]["status"] == "passed"
    assert len(calls) == 1
    assert calls[0][1]["cwd"] == tmp_path
    if server_sim_closure.platform.system().lower() == "windows":
        assert calls[0][1]["shell"] is False
        assert isinstance(calls[0][0], list)
        assert calls[0][0][0] == sys.executable
        assert "PYTHONPATH" in calls[0][1]["env"]
    else:
        assert calls[0][1]["shell"] is True


def test_server_sim_closure_run_missing_failure_overrides_matching_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    def fake_runner(command: object, **_kwargs: object) -> subprocess.CompletedProcess[object]:
        _write_json(
            tmp_path / "artifacts/server_sim_closure/large_terrain/report.json",
            {
                "ok": True,
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "cases": [
                    {
                        "route": "terrain_long",
                        "path_safety": {"ok": True},
                        "planning": [
                            {"planner": "pct", "native_backend_used": True},
                        ],
                    }
                ],
            },
        )
        return subprocess.CompletedProcess(command, 139)

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={"large_terrain"},
        include_optional=False,
        runner=fake_runner,
    )

    assert summary["ok"] is False
    assert summary["gate_runs"][0]["status"] == "failed"
    assert summary["gate_runs"][0]["returncode"] == 139
    gate = summary["gates"]["large_terrain"]
    assert gate["ok"] is False
    assert gate["status"] == "execution_failed"
    assert "gate command failed with returncode 139" in gate["blockers"]
    assert any("ignoring any matching prior report" in item for item in gate["blockers"])
    assert summary["verified"]["large_terrain"] is False
    assert summary["missing_or_failed"] == ["large_terrain"]
    assert summary["remaining_gaps"][0].startswith("large_terrain:")


def test_server_sim_closure_run_missing_nonzero_report_keeps_report_blockers(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    def fake_runner(command: object, **_kwargs: object) -> subprocess.CompletedProcess[object]:
        _write_json(
            tmp_path / "artifacts/server_sim_closure/large_terrain/report.json",
            {
                "ok": False,
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "cases": [],
            },
        )
        return subprocess.CompletedProcess(command, 1)

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={"large_terrain"},
        include_optional=False,
        runner=fake_runner,
    )

    gate = summary["gates"]["large_terrain"]
    assert summary["ok"] is False
    assert gate["ok"] is False
    assert gate["status"] == "report_failed_after_run"
    assert gate["run_status"] == "failed"
    assert gate["effective_run_status"] == "report_failed"
    assert gate["report_from_this_run"] is True
    assert any("report.ok is not true" in item for item in gate["blockers"])
    assert any("returncode 1" in item for item in gate["blockers"])
    assert "report.ok is not true" in summary["remaining_gaps"][0]


def test_server_sim_closure_run_missing_records_host_blocked_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    preflight_required: list[set[str]] = []
    calls: list[tuple[object, dict]] = []

    def fake_host_preflight(*, required: set[str], **_kwargs: object) -> dict[str, object]:
        preflight_required.append(set(required))
        return {
            "schema_version": "lingtu.server_sim_host_preflight.v1",
            "execution_mode": "host_preflight_only",
            "ok": False,
            "runnable_gates": ["large_terrain"],
            "blocked_gates": ["dynamic_obstacle_local_planner"],
            "gates": {
                "large_terrain": {"ok": True, "status": "runnable", "blockers": []},
                "dynamic_obstacle_local_planner": {
                    "ok": False,
                    "status": "blocked",
                    "blockers": [
                        "Windows/MINGW NumPy local-planner runtime is not accepted"
                    ],
                    "checks": {
                        "local_numeric_nav": {
                            "ok": False,
                            "blocker": (
                                "Windows/MINGW NumPy local-planner runtime is not accepted"
                            ),
                        }
                    },
                },
            },
        }

    def fake_runner(command: object, **kwargs: object) -> subprocess.CompletedProcess[object]:
        calls.append((command, kwargs))
        _write_json(
            tmp_path / "artifacts/server_sim_closure/large_terrain/report.json",
            {
                "ok": True,
                "simulation_only": True,
                "real_robot_motion": False,
                "cmd_vel_sent_to_hardware": False,
                "cases": [
                    {
                        "route": "terrain_long",
                        "path_safety": {"ok": True},
                        "planning": [
                            {"planner": "pct", "native_backend_used": True},
                        ],
                    }
                ],
            },
        )
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(server_sim_closure, "host_preflight", fake_host_preflight)
    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={"large_terrain", "dynamic_obstacle_local_planner"},
        include_optional=False,
        runner=fake_runner,
        skip_host_blocked=True,
    )

    assert summary["ok"] is False
    assert summary["skip_host_blocked"] is True
    assert preflight_required == [{"large_terrain", "dynamic_obstacle_local_planner"}]
    assert len(calls) == 1
    assert [item["name"] for item in summary["gate_runs"]] == [
        "large_terrain",
        "dynamic_obstacle_local_planner",
    ]
    assert summary["gate_runs"][0]["status"] == "passed"
    blocked_run = summary["gate_runs"][1]
    assert blocked_run["status"] == "host_blocked"
    assert blocked_run["returncode"] is None
    assert blocked_run["executed_command"] is None
    assert blocked_run["shell"] is None
    assert "Windows/MINGW NumPy" in blocked_run["error"]
    assert blocked_run["failed_checks"] == ["local_numeric_nav"]
    assert "Linux Python host" in blocked_run["recommended_action"]
    assert any("numpy" in command for command in blocked_run["diagnostic_commands"])
    assert blocked_run["host_preflight"]["status"] == "blocked"
    assert summary["skipped_host_blocked_gates"] == ["dynamic_obstacle_local_planner"]
    assert (
        summary["run_missing_host_preflight"]["blocked_gates"]
        == ["dynamic_obstacle_local_planner"]
    )
    assert summary["missing_or_failed"] == ["dynamic_obstacle_local_planner"]
    assert summary["remaining_gaps"][0].startswith("dynamic_obstacle_local_planner:")


def test_server_sim_closure_run_missing_blocks_pct_saved_map_navigation_when_relocalize_report_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    calls: list[str] = []

    def fake_runner(command: object, **_kwargs: object) -> subprocess.CompletedProcess[object]:
        command_text = " ".join(command) if isinstance(command, list) else str(command)
        calls.append(command_text)
        assert "saved_map_relocalize_runtime_gate.py" in command_text
        return subprocess.CompletedProcess(command, 0)

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={"saved_map_relocalize", "pct_saved_map_navigation"},
        include_optional=False,
        runner=fake_runner,
    )

    assert len(calls) == 1
    assert [item["name"] for item in summary["gate_runs"]] == [
        "saved_map_relocalize",
        "pct_saved_map_navigation",
    ]
    relocalize_run = summary["gate_runs"][0]
    assert relocalize_run["status"] == "report_failed"
    assert relocalize_run["returncode"] == 0
    assert "report missing" in relocalize_run["error"]

    pct_run = summary["gate_runs"][1]
    assert pct_run["status"] == "dependency_blocked"
    assert pct_run["returncode"] is None
    assert pct_run["executed_command"] is None
    assert pct_run["shell"] is None
    assert pct_run["dependency_blockers"] == ["saved_map_relocalize"]
    assert "saved_map_relocalize" in pct_run["error"]
    assert summary["skipped_dependency_blocked_gates"] == ["pct_saved_map_navigation"]
    assert summary["missing_or_failed"] == [
        "saved_map_relocalize",
        "pct_saved_map_navigation",
    ]


def test_server_sim_closure_run_missing_runs_pct_saved_map_navigation_after_relocalize_report_passes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    calls: list[str] = []
    map_dir = tmp_path / "same_source_map"
    map_pcd = _write_artifact(map_dir / "map.pcd")
    tomogram = _write_artifact(map_dir / "tomogram.pickle")
    relocalize_report = (
        tmp_path / "artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json"
    )
    pct_report = tmp_path / "artifacts/server_sim_closure/pct_saved_map_navigation/report.json"

    def fake_runner(command: object, **_kwargs: object) -> subprocess.CompletedProcess[object]:
        command_text = " ".join(command) if isinstance(command, list) else str(command)
        calls.append(command_text)
        if "saved_map_relocalize_runtime_gate.py" in command_text:
            payload = _complete_saved_map_relocalize_runtime_report()
            payload["map_pcd"] = str(map_pcd)
            _write_json(relocalize_report, payload)
        elif "pct_saved_map_navigation_gate.py" in command_text:
            payload = _complete_pct_saved_map_navigation_report()
            payload["tomogram"] = str(tomogram)
            payload["relocalize_report"] = str(relocalize_report)
            _write_json(pct_report, payload)
        else:
            raise AssertionError(f"unexpected command: {command_text}")
        return subprocess.CompletedProcess(command, 0)

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={"saved_map_relocalize", "pct_saved_map_navigation"},
        include_optional=False,
        runner=fake_runner,
    )

    assert len(calls) == 2
    assert [item["name"] for item in summary["gate_runs"]] == [
        "saved_map_relocalize",
        "pct_saved_map_navigation",
    ]
    assert [item["status"] for item in summary["gate_runs"]] == ["passed", "passed"]
    assert summary["ok"] is True, summary["remaining_gaps"]
    assert summary["missing_or_failed"] == []
    assert summary["skipped_dependency_blocked_gates"] == []


def test_server_sim_closure_run_missing_blocks_live_dependents_when_fastlio2_dynamic_inspection_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    _write_json(
        tmp_path / "artifacts/server_sim_closure/large_terrain/report.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [{"planner": "pct", "native_backend_used": True}],
                }
            ],
        },
    )
    calls: list[str] = []

    def fake_runner(command: object, **_kwargs: object) -> subprocess.CompletedProcess[object]:
        command_text = " ".join(command) if isinstance(command, list) else str(command)
        calls.append(command_text)
        assert "inspection-moving-obstacle-video" in command_text
        return subprocess.CompletedProcess(command, 1)

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={},
        required={
            "fastlio2_dynamic_inspection",
            "moving_obstacle_sweep",
            "large_loop_closure",
        },
        include_optional=False,
        runner=fake_runner,
    )

    assert len(calls) == 1
    assert [item["name"] for item in summary["gate_runs"]] == [
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]
    assert summary["gate_runs"][0]["status"] == "failed"
    assert summary["gate_runs"][0]["returncode"] == 1
    for run in summary["gate_runs"][1:]:
        assert run["status"] == "dependency_blocked"
        assert run["returncode"] is None
        assert run["executed_command"] is None
        assert run["dependency_blockers"] == ["fastlio2_dynamic_inspection"]
    assert summary["skipped_dependency_blocked_gates"] == [
        "moving_obstacle_sweep",
        "large_loop_closure",
    ]


def test_server_sim_closure_normalizes_windows_local_python_gate_command():
    command, invocation = server_sim_closure._normalized_gate_run_invocation(
        "PYTHONPATH=src:. python3 sim/scripts/blocked_route_replan_gate.py "
        "--json-out artifacts/server_sim_closure/blocked_route_replan/report.json --strict",
        platform_system="Windows",
    )

    assert invocation["shell"] is False
    assert isinstance(command, list)
    assert command[0] == sys.executable
    assert command[1:] == [
        "sim/scripts/blocked_route_replan_gate.py",
        "--json-out",
        "artifacts/server_sim_closure/blocked_route_replan/report.json",
        "--strict",
    ]
    assert str(server_sim_closure.SRC) in invocation["env"]["PYTHONPATH"]


def test_server_sim_closure_normalizes_windows_local_python_gate_command_with_extra_env():
    command, invocation = server_sim_closure._normalized_gate_run_invocation(
        "PYTHONPATH=src:. MUJOCO_GL=egl python3 sim/scripts/policy_nav_smoke.py "
        "--json-out artifacts/server_sim_closure/policy_nav/report.json",
        platform_system="Windows",
    )

    assert invocation["shell"] is False
    assert isinstance(command, list)
    assert command[0] == sys.executable
    assert command[1:] == [
        "sim/scripts/policy_nav_smoke.py",
        "--json-out",
        "artifacts/server_sim_closure/policy_nav/report.json",
    ]
    assert invocation["env"]["MUJOCO_GL"] == "egl"
    assert str(server_sim_closure.SRC) in invocation["env"]["PYTHONPATH"]
    assert str(server_sim_closure.ROOT) in invocation["env"]["PYTHONPATH"]


def test_server_sim_closure_keeps_bash_gate_command_shell_based_on_windows():
    command, invocation = server_sim_closure._normalized_gate_run_invocation(
        "bash -lc 'source /opt/ros/humble/setup.bash && python3 sim/scripts/gazebo_runtime_gate.py'",
        platform_system="Windows",
    )

    assert isinstance(command, str)
    assert invocation == {"shell": True}


def test_server_sim_closure_keeps_shell_expansion_gate_command_shell_based_on_windows():
    command, invocation = server_sim_closure._normalized_gate_run_invocation(
        "PYTHONPATH=src:.:$PYTHONPATH python3 sim/scripts/cmu_unity_runtime_gate.py",
        platform_system="Windows",
    )

    assert isinstance(command, str)
    assert invocation == {"shell": True}


def test_server_sim_closure_run_missing_skips_verified_required_gate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    large = _write_json(
        tmp_path / "large.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [{"planner": "pct", "native_backend_used": True}],
                }
            ],
        },
    )

    def fail_runner(command: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        raise AssertionError(f"unexpected gate execution: {command}")

    summary = server_sim_closure.run_missing_required_gates(
        report_overrides={"large_terrain": large},
        required={"large_terrain"},
        include_optional=False,
        runner=fail_runner,
    )

    assert summary["ok"] is True
    assert summary["gate_runs"] == []
    assert summary["initial_missing_or_failed"] == []


def test_server_sim_closure_rejects_weak_dynamic_obstacle_report(tmp_path: Path):
    weak = _write_json(
        tmp_path / "dynamic_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_actual": "cmu_py",
            "dynamic_replan_verified": False,
            "obstacle_response_verified": True,
            "clear_path_recovery_verified": False,
            "min_clearance_m": 0.10,
            "phases": [
                {"name": "clear_initial", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_left", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
                {"name": "obstacle_right", "path_count": 101, "path_frame_id": "map", "avoidance_side": "straight"},
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"dynamic_obstacle_local_planner": weak},
        required={"dynamic_obstacle_local_planner"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["dynamic_obstacle_local_planner"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "backend_actual is not nanobind" in gaps
    assert "dynamic_replan_verified is not true" in gaps
    assert "clear_path_recovery_verified is not true" in gaps
    assert "min_clearance_m < 0.25" in gaps
    assert "left obstacle did not produce right detour" in gaps


def test_server_sim_closure_dynamic_obstacle_runtime_error_stops_content_checks(tmp_path: Path):
    red = _write_json(
        tmp_path / "dynamic_runtime_red.json",
        {
            "schema_version": "lingtu.dynamic_obstacle_local_planner.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "backend_requested": "nanobind",
            "backend_actual": "",
            "native_backend_used": False,
            "execution_mode": "host_guard",
            "dynamic_replan_verified": False,
            "obstacle_response_verified": False,
            "clear_path_recovery_verified": False,
            "min_clearance_m": None,
            "phases": [],
            "frames": {"cmd_vel": "not_published"},
            "environment": {
                "platform_system": "Windows",
                "accepted_host": False,
                "blocked_reason": "windows_mingw_numpy_not_accepted",
                "blockers": [
                    "Windows/MINGW NumPy local-planner runtime is not accepted"
                ],
                "claim_boundary": "environment_blocked_no_algorithm_claim",
            },
            "errors": ["Windows/MINGW NumPy local-planner runtime is not accepted"],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"dynamic_obstacle_local_planner": red},
        required={"dynamic_obstacle_local_planner"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "Windows/MINGW NumPy local-planner runtime is not accepted" in gaps
    assert "clear_initial phase missing" not in gaps
    evidence = summary["gates"]["dynamic_obstacle_local_planner"]["evidence"]
    assert evidence["execution_mode"] == "host_guard"
    assert evidence["environment"]["accepted_host"] is False
    assert evidence["environment"]["blocked_reason"] == (
        "windows_mingw_numpy_not_accepted"
    )
    assert evidence["environment"]["claim_boundary"] == (
        "environment_blocked_no_algorithm_claim"
    )
    assert evidence["errors"] == [
        "Windows/MINGW NumPy local-planner runtime is not accepted"
    ]


def test_server_sim_closure_rejects_weak_saved_map_relocalize_report(tmp_path: Path):
    weak = _write_json(
        tmp_path / "saved_map_relocalize_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "validation_level": "runtime_relocalization",
            "runtime_stage": "saved_map_relocalization",
            "map_dependency": "saved_map_required",
            "requires_saved_map": True,
            "requires_live_slam": True,
            "runtime_relocalization_executed": True,
            "runtime_relocalization_validated": True,
            "service": {"available": True, "success": True},
            "live_feed": {
                "ok": False,
                "outputs": {
                    "fastlio2_odometry": 10,
                    "fastlio2_cloud_registered": 10,
                    "fastlio2_cloud_map": 10,
                },
                "fastlio2_z_consistency": {"ok": False},
            },
            "localizer": {
                "health_samples": 4,
                "tracking_health_samples": 3,
                "latest_health_state": "LOCKED",
                "saved_map_cloud_samples": 2,
                "saved_map_cloud_points_latest": 2000,
                "map_to_odom_tf_samples": 2,
                "map_to_odom_xy_m": 0.2,
                "map_to_odom_z_abs_m": 12.0,
            },
            "thresholds": {
                "min_saved_map_points": 1000,
                "min_tracking_health_samples": 3,
                "max_map_odom_xy_m": 5.0,
                "max_map_odom_z_abs_m": 2.0,
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"saved_map_relocalize": weak},
        required={"saved_map_relocalize"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["saved_map_relocalize"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "live_feed.ok is not true" in gaps
    assert "live_feed.fastlio2_z_consistency.ok is not true" in gaps
    assert "map->odom Z correction exceeds threshold" in gaps


def test_server_sim_closure_rejects_saved_map_relocalize_missing_map_artifact(
    tmp_path: Path,
):
    report_payload = _complete_saved_map_relocalize_runtime_report()
    report_payload["map_pcd"] = str(tmp_path / "same_source_map" / "missing_map.pcd")
    report = _write_json(tmp_path / "saved_map_relocalize_missing_map.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"saved_map_relocalize": report},
        required={"saved_map_relocalize"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "saved_map_relocalize map_pcd file missing" in gaps
    evidence = summary["gates"]["saved_map_relocalize"]["evidence"]
    assert evidence["artifacts"]["map_pcd"]["exists"] is False


def test_server_sim_closure_rejects_saved_map_relocalize_missing_metadata_contract(
    tmp_path: Path,
):
    report_payload = _complete_saved_map_relocalize_runtime_report()
    report_payload["map_pcd"] = str(_write_artifact(tmp_path / "same_source_map" / "map.pcd"))
    report_payload.pop("map_metadata_contract")
    report = _write_json(
        tmp_path / "saved_map_relocalize_missing_metadata_contract.json",
        report_payload,
    )

    summary = server_sim_closure.summarize(
        report_overrides={"saved_map_relocalize": report},
        required={"saved_map_relocalize"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "map_metadata_contract.ok is not true" in gaps
    evidence = summary["gates"]["saved_map_relocalize"]["evidence"]
    assert evidence["artifacts"]["map_metadata_contract"] == {}


def test_server_sim_closure_rejects_pct_saved_map_navigation_missing_artifacts(
    tmp_path: Path,
):
    report_payload = _complete_pct_saved_map_navigation_report()
    report_payload["tomogram"] = str(tmp_path / "same_source_map" / "missing_tomogram.pickle")
    report_payload["relocalize_report"] = str(tmp_path / "missing_relocalize_report.json")
    report = _write_json(
        tmp_path / "pct_saved_map_navigation_missing_artifacts.json",
        report_payload,
    )

    summary = server_sim_closure.summarize(
        report_overrides={"pct_saved_map_navigation": report},
        required={"pct_saved_map_navigation"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "pct_saved_map_navigation tomogram file missing" in gaps
    assert "pct_saved_map_navigation relocalize_report file missing" in gaps
    evidence = summary["gates"]["pct_saved_map_navigation"]["evidence"]
    assert evidence["artifacts"]["tomogram"]["exists"] is False
    assert evidence["artifacts"]["relocalize_report"]["exists"] is False


def test_server_sim_closure_pct_saved_map_navigation_stops_on_relocalize_prereq(
    tmp_path: Path,
):
    relocalize_report = _write_json(
        tmp_path / "saved_map_relocalize_failed.json",
        {
            "ok": False,
            "runtime_relocalization_validated": False,
            "localizer": {"latest_health_state": "LOST"},
        },
    )
    report_payload = _complete_pct_saved_map_navigation_report()
    report_payload["ok"] = False
    report_payload["tomogram"] = str(tmp_path / "same_source_map" / "missing_tomogram.pickle")
    report_payload["relocalize_report"] = str(relocalize_report)
    report_payload["relocalization"] = {
        "ok": False,
        "latest_health_state": "LOST",
        "saved_map_cloud_points_latest": 0,
        "map_to_odom_xy_m": None,
    }
    report_payload["plan_preview"] = {
        "ok": False,
        "skipped": True,
        "reason": "saved_map_relocalization_prerequisite_failed",
        "selected_planner": "",
        "fallback_reason": "",
        "path_count": 0,
    }
    report_payload["native_gate"] = {
        "ok": False,
        "skipped": True,
        "reason": "saved_map_relocalization_prerequisite_failed",
    }
    report_payload["blockers"] = [
        "runtime_relocalization_validated is not true",
        "saved-map relocalization prerequisite failed",
    ]
    report = _write_json(
        tmp_path / "pct_saved_map_navigation_relocalize_prereq_failed.json",
        report_payload,
    )

    summary = server_sim_closure.summarize(
        report_overrides={"pct_saved_map_navigation": report},
        required={"pct_saved_map_navigation"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "saved-map relocalization prerequisite failed" in gaps
    assert "pct_saved_map_navigation tomogram file missing" not in gaps
    assert "native_gate.report.ok is not true" not in gaps
    evidence = summary["gates"]["pct_saved_map_navigation"]["evidence"]
    assert evidence["plan_preview"]["skipped"] is True
    assert (
        evidence["plan_preview"]["reason"]
        == "saved_map_relocalization_prerequisite_failed"
    )
    assert evidence["native_gate"]["skipped"] is True
    assert evidence["artifacts"]["relocalize_report"]["exists"] is True
    assert "tomogram" not in evidence["artifacts"]


def test_server_sim_closure_rejects_pct_saved_map_navigation_mismatched_map_source(
    tmp_path: Path,
):
    map_pcd = _write_artifact(tmp_path / "same_source_map_a" / "map.pcd")
    _write_artifact(tmp_path / "same_source_map_a" / "tomogram.pickle")
    mismatched_tomogram = _write_artifact(
        tmp_path / "same_source_map_b" / "tomogram.pickle"
    )

    relocalize_payload = _complete_saved_map_relocalize_runtime_report()
    relocalize_payload["map_pcd"] = str(map_pcd)
    relocalize_report = _write_json(tmp_path / "saved_map_relocalize.json", relocalize_payload)

    report_payload = _complete_pct_saved_map_navigation_report()
    report_payload["tomogram"] = str(mismatched_tomogram)
    report_payload["relocalize_report"] = str(relocalize_report)
    report = _write_json(
        tmp_path / "pct_saved_map_navigation_mismatched_source.json",
        report_payload,
    )

    summary = server_sim_closure.summarize(
        report_overrides={"pct_saved_map_navigation": report},
        required={"pct_saved_map_navigation"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert (
        "pct_saved_map_navigation tomogram is not sibling of "
        "relocalize_report.map_pcd"
    ) in gaps
    evidence = summary["gates"]["pct_saved_map_navigation"]["evidence"]
    assert evidence["artifacts"]["same_source_binding"]["ok"] is False


def test_server_sim_closure_rejects_pct_saved_map_navigation_missing_hash_identity(
    tmp_path: Path,
):
    map_pcd = _write_artifact(tmp_path / "same_source_map" / "map.pcd")
    tomogram = _write_artifact(tmp_path / "same_source_map" / "tomogram.pickle")
    relocalize_payload = _complete_saved_map_relocalize_runtime_report()
    relocalize_payload["map_pcd"] = str(map_pcd)
    relocalize_report = _write_json(tmp_path / "saved_map_relocalize.json", relocalize_payload)

    report_payload = _complete_pct_saved_map_navigation_report()
    report_payload["tomogram"] = str(tomogram)
    report_payload["relocalize_report"] = str(relocalize_report)
    report_payload.pop("same_source_hash_identity")
    report = _write_json(
        tmp_path / "pct_saved_map_navigation_missing_hash_identity.json",
        report_payload,
    )

    summary = server_sim_closure.summarize(
        report_overrides={"pct_saved_map_navigation": report},
        required={"pct_saved_map_navigation"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert (
        "pct_saved_map_navigation.same_source_hash_identity.ok is not true"
        in gaps
    )
    evidence = summary["gates"]["pct_saved_map_navigation"]["evidence"]
    assert evidence["artifacts"]["same_source_hash_identity"]["ok"] is False


def test_server_sim_closure_rejects_weak_routecheck_preflight_report(tmp_path: Path):
    weak = _write_json(
        tmp_path / "routecheck_weak.json",
        {
            "schema_version": 1,
            "mode": "routecheck_non_motion",
            "outcome": "pass",
            "exit_status": 0,
            "non_motion": True,
            "simulation_only": False,
            "real_robot_motion": True,
            "cmd_vel_sent_to_hardware": True,
            "gateway_used": False,
            "driver_used": True,
            "map": "demo",
            "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "phases": {
                "baseline": {
                    "non_motion": True,
                    "can_accept_goal": True,
                    "active_cmd_source_before": "teleop",
                    "feasible": True,
                    "count": 3,
                    "selected_planner": "pct",
                    "path_safety_ok": True,
                },
                "candidate": {
                    "non_motion": True,
                    "can_accept_goal": True,
                    "active_cmd_source_before": "none",
                    "feasible": False,
                    "count": 0,
                    "selected_planner": "",
                    "path_safety_ok": False,
                },
            },
            "published": {"goal_pose": 1, "cmd_vel": 1, "stop_cmd": 1},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"routecheck_preflight": weak},
        required={"routecheck_preflight"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["routecheck_preflight"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "simulation_only is not true" in gaps
    assert "real_robot_motion is not false" in gaps
    assert "cmd_vel_sent_to_hardware is not false" in gaps
    assert "gateway_used is not true" in gaps
    assert "driver_used is not false" in gaps
    assert "published.goal_pose is not 0" in gaps
    assert "published.cmd_vel is not 0" in gaps
    assert "published.stop_cmd is not 0" in gaps
    assert "baseline.active_cmd_source_before is not none" in gaps
    assert "candidate.feasible is not true" in gaps
    assert "candidate.count < 2" in gaps
    assert "candidate.selected_planner is missing" in gaps
    assert "candidate.path_safety_ok is not true" in gaps


def test_server_sim_closure_rejects_routecheck_without_publish_counters(tmp_path: Path):
    phase = {
        "non_motion": True,
        "can_accept_goal": True,
        "active_cmd_source_before": "none",
        "feasible": True,
        "count": 3,
        "selected_planner": "pct",
        "path_safety_ok": True,
    }
    weak = _write_json(
        tmp_path / "routecheck_missing_counters.json",
        {
            "schema_version": 1,
            "mode": "routecheck_non_motion",
            "outcome": "pass",
            "exit_status": 0,
            "non_motion": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "gateway_used": True,
            "driver_used": False,
            "map": "demo",
            "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "phases": {"baseline": phase, "candidate": phase},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"routecheck_preflight": weak},
        required={"routecheck_preflight"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["routecheck_preflight"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "published.goal_pose is missing" in gaps
    assert "published.cmd_vel is missing" in gaps
    assert "published.stop_cmd is missing" in gaps


def test_server_sim_closure_rejects_gazebo_without_navigation_loop(tmp_path: Path):
    weak = _complete_gazebo_report()
    weak["nav_loop"] = {
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "goal_published": True,
        "odometry_seen": True,
        "global_path_seen": False,
        "local_path_seen": False,
        "cmd_vel_seen": True,
        "cmd_vel_nonzero": False,
        "odom_delta_m": 0.01,
        "samples": {
            "/nav/cmd_vel": 4,
            "/nav/odometry": 4,
        },
    }
    report = _write_json(tmp_path / "gazebo_weak.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"gazebo_runtime": report},
        required={"gazebo_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["gazebo_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "nav_loop.global_path_seen is not true" in gaps
    assert "nav_loop.local_path_seen is not true" in gaps
    assert "nav_loop.cmd_vel_nonzero is not true" in gaps
    assert "nav_loop.odom_delta_m < 0.05" in gaps
    assert "nav_loop /nav/global_path samples missing" in gaps
    assert "nav_loop /nav/local_path samples missing" in gaps
    assert "nav_loop.publisher_contract.ok is not true" in gaps


def test_server_sim_closure_rejects_gazebo_with_fake_nav_publishers(tmp_path: Path):
    weak = _complete_gazebo_report()
    weak["nav_loop"]["publisher_contract"] = {
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
    report = _write_json(tmp_path / "gazebo_fake_publishers.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"gazebo_runtime": report},
        required={"gazebo_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["gazebo_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "nav_loop.publisher_contract.ok is not true" in gaps
    assert "fake_cmd_vel_replayer" in gaps


def test_server_sim_closure_rejects_gazebo_frontier_without_post_pass_stall_window(
    tmp_path: Path,
):
    weak = _complete_gazebo_report()
    weak["frontier_exploration"]["frontier_no_gain_stall"] = {
        "checked": True,
        "ok": False,
        "mode": "post_pass_observation",
        "stop_reason": "pass_condition_met",
        "required_observation_s": 5.0,
        "observed_s": 0.2,
    }
    report = _write_json(tmp_path / "gazebo_frontier_no_stall.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"gazebo_runtime": report},
        required={"gazebo_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["gazebo_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frontier_exploration.frontier_no_gain_stall.ok is not true" in gaps
    assert (
        "frontier_exploration.frontier_no_gain_stall.stop_reason is not "
        "post_pass_observation_elapsed"
    ) in gaps
    assert "frontier_exploration.frontier_no_gain_stall.observed_s below required" in gaps


def test_server_sim_closure_rejects_gazebo_without_frontier_exploration(tmp_path: Path):
    weak = _complete_gazebo_report()
    weak["frontier_exploration"] = {
        "ok": True,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "frontier_started": True,
        "frontier_goal_seen": False,
        "frontier_goal_published": False,
        "odometry_seen": True,
        "map_cloud_seen": True,
        "global_path_seen": False,
        "local_path_seen": False,
        "cmd_vel_seen": True,
        "cmd_vel_nonzero": False,
        "odom_delta_m": 0.01,
        "known_cells_delta": 0,
        "explored_area_delta_m2": 0.0,
        "frontier_count_max": 0,
        "samples": {
            "/nav/cmd_vel": 4,
            "/nav/odometry": 4,
        },
    }
    report = _write_json(tmp_path / "gazebo_weak_frontier.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"gazebo_runtime": report},
        required={"gazebo_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["gazebo_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frontier_exploration.frontier_goal_seen is not true" in gaps
    assert "frontier_exploration.global_path_seen is not true" in gaps
    assert "frontier_exploration.cmd_vel_nonzero is not true" in gaps
    assert "frontier_exploration.explored_area_delta_m2 <= 0" in gaps
    assert "frontier_exploration.known_cells_delta <= 0" in gaps
    assert "frontier_exploration.frontier_count_max <= 0" in gaps


def test_server_sim_closure_rejects_gazebo_when_tare_contract_missing(tmp_path: Path):
    weak = _complete_gazebo_report()
    weak["tare_exploration"] = {
        "ok": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "backend": "tare",
        "source_contract_ok": False,
        "runtime_required": True,
        "runtime_available": False,
    }
    report = _write_json(tmp_path / "gazebo_weak_tare.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"gazebo_runtime": report},
        required={"gazebo_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["gazebo_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "tare_exploration.ok is not true" in gaps
    assert "tare_exploration.source_contract_ok is not true" in gaps
    assert "tare_exploration runtime required but unavailable" in gaps


def test_server_sim_closure_rejects_cmu_unity_without_required_assets(tmp_path: Path):
    weak_report = _complete_cmu_unity_report()
    weak_report["ok"] = False
    for check in weak_report["checks"]:
        if check["name"] in {"cmu_unity_environment_assets", "cmu_colcon_build_output"}:
            check["ok"] = False
    weak_report["blockers"] = ["cmu_unity_environment_assets", "cmu_colcon_build_output"]
    weak = _write_json(tmp_path / "cmu_unity_weak.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_sim": weak},
        required={"cmu_unity_sim"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_sim"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "cmu_unity_environment_assets is not true" in gaps
    assert "cmu_colcon_build_output is not true" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_without_motion(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["ok"] = False
    weak_report["cmd_vel"]["nonzero_samples"] = 0
    weak_report["odometry"]["/nav/odometry"]["delta_m"] = 0.0
    weak_report["odometry"]["/state_estimation"]["delta_m"] = 0.0
    weak_report["cloud_coverage"]["best_area_delta_m2"] = 0.0
    weak = _write_json(tmp_path / "cmu_unity_runtime_weak.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "/nav/cmd_vel nonzero_samples below threshold" in gaps
    assert "odom delta below threshold" in gaps
    assert "map/exploration area delta below threshold" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_without_contract(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report.pop("runtime_contract")
    weak = _write_json(tmp_path / "cmu_unity_runtime_missing_contract.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "runtime_contract missing" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_failed_contract(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["runtime_contract"] = _complete_cmu_unity_runtime_contract_report(
        errors=["/nav/global_path path contract not satisfied"],
    )
    weak = _write_json(tmp_path / "cmu_unity_runtime_failed_contract.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "runtime_contract.ok is not true" in gaps
    assert "runtime_contract: /nav/global_path path contract not satisfied" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_tampered_contract_definition(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["runtime_contract"]["definition"]["adapter_script"] = "/tmp/fake_adapter.py"
    weak_report["runtime_contract"]["definition"]["slam_source"] = "fastlio2"
    weak = _write_json(tmp_path / "cmu_unity_runtime_tampered_contract.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "runtime_contract.definition.adapter_script does not match canonical contract" in gaps
    assert "runtime_contract.definition.slam_source does not match canonical contract" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_missing_topic_evidence(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["runtime_contract"]["topic_evidence"].pop("/nav/global_path")
    weak = _write_json(tmp_path / "cmu_unity_runtime_missing_topic_evidence.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "runtime_contract topic evidence missing or failed for /nav/global_path" in gaps


def test_server_sim_closure_cmu_runtime_uses_shared_runtime_evidence(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["hardware_safety"]["unexpected_command_publishers"] = ["/foreign_cmd_source"]
    weak = _write_json(tmp_path / "cmu_unity_runtime_unsafe_cmd.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["cmu_unity_runtime"]["evidence"]["runtime_evidence"]
    assert evidence["checked"] is True
    assert evidence["expected_contract"] == "cmu_unity_external"
    assert evidence["frame_links_required"] is True
    assert evidence["data_flow_required"] is True
    assert "unexpected command publisher present" in evidence["blockers"]
    gaps = "\n".join(summary["remaining_gaps"])
    assert "unexpected command publisher present" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_missing_frame_evidence(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["runtime_contract"].pop("frame_evidence")
    weak = _write_json(tmp_path / "cmu_unity_runtime_missing_frame_evidence.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["cmu_unity_runtime"]["evidence"]["runtime_evidence"]
    assert evidence["frame_links_required"] is True
    assert "frame evidence missing" in evidence["blockers"]
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frame evidence missing" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_missing_data_flow_stage(tmp_path: Path):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["runtime_contract"]["data_flow_evidence"].pop("local_planning_and_following")
    weak = _write_json(tmp_path / "cmu_unity_runtime_missing_data_flow.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["cmu_unity_runtime"]["evidence"]["runtime_evidence"]
    assert evidence["data_flow_required"] is True
    assert (
        "data-flow evidence missing or failed for local_planning_and_following"
        in evidence["blockers"]
    )
    gaps = "\n".join(summary["remaining_gaps"])
    assert "data-flow evidence missing or failed for local_planning_and_following" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_without_frontier_stall_evidence(
    tmp_path: Path,
):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report.pop("frontier_no_gain_stall")
    weak = _write_json(tmp_path / "cmu_unity_runtime_missing_frontier_stall.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frontier_no_gain_stall missing" in gaps
    assert "frontier_no_gain_stall.checked is not true" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_short_frontier_stall_window(
    tmp_path: Path,
):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["frontier_no_gain_stall"]["observed_s"] = 20.0
    weak = _write_json(tmp_path / "cmu_unity_runtime_short_frontier_stall.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frontier_no_gain_stall.observed_s below required" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_without_tare_strategy_quality(
    tmp_path: Path,
):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report.pop("tare_strategy_quality", None)
    weak = _write_json(tmp_path / "cmu_unity_runtime_missing_tare_strategy.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "tare_strategy_quality missing" in gaps


def test_server_sim_closure_rejects_cmu_unity_runtime_weak_tare_strategy_quality(
    tmp_path: Path,
):
    weak_report = _complete_cmu_unity_runtime_report()
    weak_report["tare_strategy_quality"] = {
        "checked": True,
        "ok": False,
        "waypoint_count": 2,
        "path_count": 0,
        "strategy_path_count": 0,
        "navigation_success_count": 1,
        "navigation_failure_count": 0,
        "reject_reasons": ["path_too_short"],
        "blockers": ["path_count below threshold"],
    }
    weak = _write_json(tmp_path / "cmu_unity_runtime_weak_tare_strategy.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_runtime": weak},
        required={"cmu_unity_runtime"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_runtime"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "tare_strategy_quality.ok is not true" in gaps
    assert "tare_strategy_quality: path_count below threshold" in gaps


def test_server_sim_closure_accepts_strict_cmu_pct_report_as_runtime_evidence(tmp_path: Path):
    strict = _write_json(
        tmp_path / "artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json",
        _complete_cmu_unity_pct_strict_report(),
    )

    summary = server_sim_closure.summarize(
        report_overrides={
            "cmu_unity_runtime": strict,
            "cmu_unity_pct_strict": strict,
        },
        required={"cmu_unity_runtime", "cmu_unity_pct_strict"},
    )

    assert summary["ok"] is True
    assert summary["verified"]["cmu_unity_runtime"] is True
    assert summary["verified"]["cmu_unity_pct_strict"] is True
    assert summary["gates"]["cmu_unity_runtime"]["path"] == str(strict)
    assert summary["gates"]["cmu_unity_pct_strict"]["path"] == str(strict)


def test_server_sim_closure_finds_strict_cmu_pct_report_for_runtime_gate(
    tmp_path: Path,
    monkeypatch,
):
    strict = _write_json(
        tmp_path / "artifacts/server_sim_closure/cmu_unity_pct_strict/report_unique_waypoints.json",
        _complete_cmu_unity_pct_strict_report(),
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"cmu_unity_runtime"},
        include_optional=False,
    )

    assert summary["ok"] is True
    assert summary["verified"]["cmu_unity_runtime"] is True
    assert summary["gates"]["cmu_unity_runtime"]["path"] == str(strict)


def test_server_sim_closure_rejects_cmu_unity_pct_strict_with_planner_fallback(tmp_path: Path):
    weak_report = _complete_cmu_unity_pct_strict_report()
    weak_report["planner_diagnostics"]["selected_planner"] = "astar"
    weak_report["planner_diagnostics"]["fallback_used"] = True
    weak_report["planner_diagnostics"]["fallback_reason"] = "pct path_safety failed"
    weak_report["planner_diagnostics"]["rejected_plan_count"] = 1
    weak_report["planner_diagnostics"]["reached_goal"] = False
    weak = _write_json(tmp_path / "cmu_unity_pct_strict_fallback.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_pct_strict": weak},
        required={"cmu_unity_pct_strict"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_pct_strict"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "selected_planner is not pct" in gaps
    assert "planner fallback was used" in gaps
    assert "planner rejected_plan_count is not 0" in gaps
    assert "PCT did not report reached_goal" in gaps


def test_server_sim_closure_rejects_cmu_unity_pct_strict_direct_goal_bypass(tmp_path: Path):
    weak_report = _complete_cmu_unity_pct_strict_report()
    weak_report["direct_goal_fallback"]["used"] = True
    weak = _write_json(tmp_path / "cmu_unity_pct_strict_direct_goal.json", weak_report)

    summary = server_sim_closure.summarize(
        report_overrides={"cmu_unity_pct_strict": weak},
        required={"cmu_unity_pct_strict"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["cmu_unity_pct_strict"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "direct goal fallback was used" in gaps


def test_server_sim_closure_rejects_non_pct_native_motion_report(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_astar.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.4,
            "planner": "astar",
            "pct_native_backend_used": False,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": True, "metadata_points": 24},
            "obstacle_clearance": {"checked": True, "collision": False, "min_clearance_m": 0.7},
            "local_path_samples": [{"frame_id": "body", "point_count": 4}],
            "trajectory_quality": {"ok": True},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    assert any("planner is not pct" in gap for gap in summary["remaining_gaps"])
    assert any("pct_native_runtime_used is not true" in gap for gap in summary["remaining_gaps"])


def test_server_sim_closure_rejects_native_pct_fallback_report(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_fallback.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.4,
            "planner": "pct",
            "primary_planner": "pct",
            "selected_planner": "astar",
            "fallback_used": True,
            "global_planner_source": "source_report/native_pct_tomogram",
            "pct_native_backend_used": True,
            "moved_m": 10.0,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "planning_chain": {
                "local_planner": "cmu_ros2_native/localPlanner",
                "path_follower": "cmu_ros2_native/pathFollower",
                "fallback_allowed": False,
            },
            "source_planning_contract": {
                "primary_planner": "pct",
                "selected_planner": "astar",
                "fallback_used": True,
                "path_safety_ok": True,
                "native_backend_used": True,
                "tomogram_exists": True,
                "tomogram_sha256": "abc123",
            },
            "obstacle_aware": {"enabled": True, "metadata_points": 24},
            "obstacle_clearance": {"checked": True, "collision": False, "min_clearance_m": 0.7},
            "local_path_samples": [{"frame_id": "body", "point_count": 4}],
            "trajectory_quality": {"ok": True},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "selected_planner is not pct" in gaps
    assert "fallback_used is not false" in gaps
    assert "source_planning_contract.selected_planner is not pct" in gaps
    assert "source_planning_contract.fallback_used is not false" in gaps


def test_server_sim_closure_rejects_native_pct_without_obstacle_local_path_evidence(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.4,
            "planner": "pct",
            "pct_native_backend_used": True,
            "moved_m": 10.0,
            "frames": {"goal": "map", "cmd_vel": "base_link"},
            "obstacle_aware": {"enabled": False, "metadata_points": 0},
            "obstacle_clearance": {"collision": False, "min_clearance_m": 0.7},
            "local_path_samples": [],
            "trajectory_quality": {"ok": False},
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "obstacle_aware.enabled is not true" in gaps
    assert "obstacle_aware.metadata_points missing" in gaps
    assert "obstacle clearance was not checked" in gaps
    assert "local_path_samples missing" in gaps
    assert "trajectory_quality.ok is not true" in gaps


def test_server_sim_closure_accepts_native_pct_with_moving_obstacle_evidence(
    tmp_path: Path,
):
    report = _complete_native_pct_mujoco_report()
    report["moving_obstacles"] = {
        "enabled": True,
        "mode": "route_crossing",
        "ok": True,
        "published_update_count": 120,
        "published_point_count_max": 20,
        "trail_clearance": {
            "checked": True,
            "collision": False,
            "min_margin_m": 0.08,
        },
    }
    native = _write_json(tmp_path / "native_moving_obstacle.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["native_pct_mujoco"]["evidence"]
    assert evidence["moving_obstacles"]["mode"] == "route_crossing"
    assert evidence["moving_obstacles"]["trail_collision"] is False


def test_server_sim_closure_requires_declared_native_pct_video_evidence(tmp_path: Path):
    report = _complete_native_pct_mujoco_report()
    video_path = tmp_path / "pct_moving_obstacle.mp4"
    report["video"] = {
        "path": str(video_path),
        "exists": False,
        "frames": 0,
        "layout": "evidence",
        "lidar_source": _mid360_lidar_source(),
    }
    native = _write_json(tmp_path / "native_moving_obstacle_video_missing.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "native_pct video.exists is not true" in gaps
    assert "native_pct video.frames missing" in gaps


def test_server_sim_closure_rejects_undecodable_native_pct_video_file(tmp_path: Path):
    report = _complete_native_pct_mujoco_report()
    fake_video = tmp_path / "not_a_video.mp4"
    fake_video.write_bytes(b"LingTu placeholder video artifact\n")
    report["video"] = {
        "path": str(fake_video),
        "exists": True,
        "frames": 24,
        "layout": "evidence",
        "lidar_source": _mid360_lidar_source(),
    }
    native = _write_json(tmp_path / "native_moving_obstacle_fake_video.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert _has_video_decode_gap(gaps, "native_pct")


def test_server_sim_closure_rejects_native_pct_failed_moving_obstacle_evidence(
    tmp_path: Path,
):
    report = _complete_native_pct_mujoco_report()
    report["moving_obstacles"] = {
        "enabled": True,
        "mode": "route_crossing",
        "ok": False,
        "published_update_count": 0,
        "published_point_count_max": 0,
        "trail_clearance": {
            "checked": True,
            "collision": True,
            "min_margin_m": -0.02,
        },
    }
    native = _write_json(tmp_path / "native_moving_obstacle_failed.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "moving_obstacles.ok is not true" in gaps
    assert "moving_obstacles.published_update_count missing" in gaps
    assert "moving_obstacles.published_point_count_max missing" in gaps
    assert "moving_obstacles.trail_clearance.collision is true" in gaps
    assert "moving_obstacles.trail_clearance margin is negative" in gaps


def test_server_sim_closure_finds_nested_fastlio2_live_report(tmp_path: Path, monkeypatch):
    nested = _write_json(
        tmp_path / "artifacts" / "mujoco_fastlio2_live" / "factory_scene" / "report.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 8,
                "nav_odometry": 8,
                "nav_registered_cloud": 6,
                "nav_map_cloud": 6,
            },
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
        },
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(report_overrides={}, required={"fastlio2_live"})

    assert summary["ok"] is True
    assert summary["gates"]["fastlio2_live"]["path"] == str(nested)


def test_server_sim_closure_rejects_fastlio2_without_slam_bridge_or_mid360(tmp_path: Path):
    weak = _write_json(
        tmp_path / "fastlio2_weak.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": False,
            "bridge_verified": False,
            "outputs": {},
            "states_seen": [],
            "point_count": {},
            "frames": {"published_lidar": "map"},
            "lidar_source": {
                "kind": "synthetic fallback lidar",
                "forced_pattern": False,
                "pattern_path": "/tmp/fallback.npy",
                "pattern_sha256": "bad",
                "samples_per_frame": 0,
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": weak},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["fastlio2_live"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "runtime_contract missing" in gaps
    assert "slam_algorithm_output_verified is not true" in gaps
    assert "canonical_nav_outputs_verified is not true" in gaps
    assert "bridge_verified is not true" in gaps
    assert "outputs.nav_odometry missing" in gaps
    assert "published_lidar frame is not body or lidar_link" in gaps
    assert "lidar_source.forced_pattern is not true" in gaps
    assert "lidar_source.pattern_sha256 is not official MID-360 asset" in gaps
    assert "lidar_source.samples_per_frame missing" in gaps


def test_server_sim_closure_fastlio2_records_runtime_evidence_when_declared(tmp_path: Path):
    report = _complete_fastlio2_tare_report()
    report["runtime_contract"] = {
        "name": "mujoco_fastlio2_live",
        "ok": True,
        "data_flow_evidence": _data_flow_evidence(),
        "frame_evidence": _fastlio2_frame_evidence(),
    }
    report["hardware_safety"] = {
        "topics": {"/nav/cmd_vel": ["/mujoco_velocity_adapter"]},
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
    }
    path = _write_json(tmp_path / "fastlio2_with_contract.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": path},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is True, summary["remaining_gaps"]
    evidence = summary["gates"]["fastlio2_live"]["evidence"]["runtime_evidence"]
    assert evidence["checked"] is True
    assert evidence["expected_contract"] == "mujoco_fastlio2_live"
    assert evidence["required"] is True
    assert evidence["contract_required"] is True
    assert evidence["data_flow_required"] is True
    assert evidence["ok"] is True


def test_server_sim_closure_fastlio2_rejects_bad_runtime_frame_evidence(tmp_path: Path):
    report = _complete_fastlio2_tare_report()
    frame_evidence = _fastlio2_frame_evidence()
    frame_evidence["body_to_lidar"]["child"] = "map"
    report["runtime_contract"] = {
        "name": "mujoco_fastlio2_live",
        "ok": True,
        "data_flow_evidence": _data_flow_evidence(),
        "frame_evidence": frame_evidence,
    }
    report["hardware_safety"] = {
        "topics": {"/nav/cmd_vel": ["/mujoco_velocity_adapter"]},
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
    }
    path = _write_json(tmp_path / "fastlio2_bad_frame_contract.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": path},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["fastlio2_live"]["evidence"]["runtime_evidence"]
    assert "frame evidence child mismatch for body_to_lidar" in evidence["blockers"]
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frame evidence child mismatch for body_to_lidar" in gaps


def test_server_sim_closure_fastlio2_rejects_bad_runtime_data_flow(tmp_path: Path):
    report = _complete_fastlio2_tare_report()
    data_flow = _data_flow_evidence()
    data_flow["command_boundary"]["outputs"] = ["hardware_driver_after_cmd_vel_mux"]
    report["runtime_contract"] = {
        "name": "mujoco_fastlio2_live",
        "ok": True,
        "data_flow_evidence": data_flow,
        "frame_evidence": _fastlio2_frame_evidence(),
    }
    report["hardware_safety"] = {
        "topics": {"/nav/cmd_vel": ["/mujoco_velocity_adapter"]},
        "blocked_hardware_nodes": [],
        "unexpected_command_publishers": [],
    }
    path = _write_json(tmp_path / "fastlio2_bad_data_flow_contract.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": path},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["fastlio2_live"]["evidence"]["runtime_evidence"]
    assert "data-flow evidence outputs mismatch for command_boundary" in evidence["blockers"]
    gaps = "\n".join(summary["remaining_gaps"])
    assert "data-flow evidence outputs mismatch for command_boundary" in gaps


def test_server_sim_closure_fastlio2_exception_report_keeps_sensor_contract(tmp_path: Path):
    from sim.scripts.mujoco.live_gate import _gate_exception_report

    report = _gate_exception_report(
        SimpleNamespace(
            duration_clock="wall",
            fastlio_lidar_input="livox_custom_msg",
            mid360_pattern=str((server_sim_closure.ROOT / server_sim_closure.MID360_PATTERN_REL).resolve()),
            mid360_samples_per_frame=1200,
            n_rays=6400,
            scan_time_profile="physical_rolling",
        ),
        RuntimeError("ROS2 Python modules are unavailable"),
    )
    path = _write_json(tmp_path / "fastlio2_exception_report.json", report)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": path},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["fastlio2_live"]["evidence"]
    assert evidence["lidar_source"]["forced_pattern"] is True
    assert evidence["runtime_evidence"]["frame_links_required"] is True
    assert evidence["runtime_evidence"]["data_flow_required"] is True
    gaps = "\n".join(summary["remaining_gaps"])
    assert "frame evidence missing or failed for map_to_odom" in gaps
    assert "data-flow evidence missing or failed for endpoint_adapter" in gaps
    assert "lidar_source.pattern_sha256 is not official MID-360 asset" not in gaps
    assert "lidar_source.samples_per_frame missing" not in gaps


def test_server_sim_closure_rejects_fastlio2_frontier_without_area_growth(tmp_path: Path):
    report = _write_json(
        tmp_path / "fastlio2_frontier_no_growth.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 8,
                "nav_odometry": 8,
                "nav_registered_cloud": 6,
                "nav_map_cloud": 6,
                "nav_cmd_vel_nonzero": 12,
            },
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
            "lingtu_frontier": {
                "enabled": True,
                "verified": True,
                "goal_count": 3,
            },
            "map_growth": {
                "min_map_area_growth_m2": 1.0,
                "min_explored_area_growth_m2": 1.0,
                "min_exploration_coverage_growth_ratio": 0.01,
                "nav_map_cloud_xy_area": {"growth_m2": 0.0},
                "exploration_known_area": {"growth_m2": 0.0},
                "exploration_coverage": {"growth_ratio": 0.0},
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["fastlio2_live"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "/nav/map_cloud area growth below threshold" in gaps
    assert "exploration known area growth below threshold" in gaps
    assert "exploration coverage growth below threshold" in gaps


def test_server_sim_closure_rejects_fastlio2_frontier_without_coverage_growth(tmp_path: Path):
    report = _write_json(
        tmp_path / "fastlio2_frontier_no_coverage_growth.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 8,
                "nav_odometry": 8,
                "nav_registered_cloud": 6,
                "nav_map_cloud": 6,
                "nav_cmd_vel_nonzero": 12,
            },
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
            "lingtu_frontier": {
                "enabled": True,
                "verified": True,
                "started_after_slam_ready": True,
                "goal_count": 3,
                "successful_navigation_goal_count": 3,
                "failed_navigation_goal_count": 0,
                "min_required_goals": 3,
                "frontier_health": {"blocked_goal_count": 0},
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "direct_goal_fallback": {"used": False},
                "health": {"plan_safety_policy": "reject"},
            },
            "map_growth": {
                "min_map_area_growth_m2": 1.0,
                "min_explored_area_growth_m2": 1.0,
                "min_exploration_coverage_growth_ratio": 0.01,
                "nav_map_cloud_xy_area": {"growth_m2": 2.0},
                "exploration_known_area": {"growth_m2": 2.0},
                "exploration_coverage": {"growth_ratio": 0.0},
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "exploration coverage growth below threshold" in gaps


def test_server_sim_closure_uses_nav_map_growth_for_rolling_frontier_grid(tmp_path: Path):
    report = _write_json(
        tmp_path / "fastlio2_frontier_rolling_grid.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 8,
                "nav_odometry": 8,
                "nav_registered_cloud": 6,
                "nav_map_cloud": 6,
                "nav_cmd_vel_nonzero": 12,
            },
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
            "lingtu_frontier": {
                "enabled": True,
                "verified": True,
                "started_after_slam_ready": True,
                "goal_count": 3,
                "successful_navigation_goal_count": 3,
                "failed_navigation_goal_count": 0,
                "min_required_goals": 3,
                "frontier_health": {"blocked_goal_count": 0},
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "direct_goal_fallback": {"used": False},
                "health": {"plan_safety_policy": "reject"},
            },
            "map_growth": {
                "accepted_cumulative_growth_source": "/nav/map_cloud",
                "exploration_grid_growth_is_acceptance_metric": False,
                "exploration_grid_accumulation": "rolling_local_window",
                "min_map_area_growth_m2": 1.0,
                "min_explored_area_growth_m2": 1.0,
                "min_exploration_coverage_growth_ratio": 0.01,
                "nav_map_cloud_xy_area": {"growth_m2": 2.0},
                "nav_map_cloud_area_samples": 4,
                "exploration_known_area": {"growth_m2": 0.0},
                "exploration_coverage": {"growth_ratio": 0.0},
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is True


def test_server_sim_closure_rejects_fastlio2_frontier_planner_fallback(tmp_path: Path):
    report = _write_json(
        tmp_path / "fastlio2_frontier_planner_fallback.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 8,
                "nav_odometry": 8,
                "nav_registered_cloud": 6,
                "nav_map_cloud": 6,
                "nav_cmd_vel_nonzero": 12,
            },
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
            "lingtu_frontier": {
                "enabled": True,
                "verified": True,
                "started_after_slam_ready": True,
                "goal_count": 3,
            },
            "navigation_chain": {
                "planner_fallback_used": True,
                "planner_repair_used": False,
                "direct_goal_fallback": {"used": False},
                "health": {"plan_safety_policy": "reject"},
            },
            "map_growth": {
                "accepted_cumulative_growth_source": "/nav/map_cloud",
                "min_map_area_growth_m2": 1.0,
                "min_explored_area_growth_m2": 1.0,
                "min_exploration_coverage_growth_ratio": 0.01,
                "nav_map_cloud_xy_area": {"growth_m2": 2.0},
                "nav_map_cloud_area_samples": 4,
                "exploration_known_area": {"growth_m2": 2.0},
                "exploration_coverage": {"growth_ratio": 0.02},
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "navigation_chain.planner_fallback_used is true" in gaps


def test_server_sim_closure_accepts_fastlio2_frontier_with_successful_navigation_goals(tmp_path: Path):
    report = _write_json(
        tmp_path / "fastlio2_frontier_success.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "live_mujoco_lidar_verified": True,
            "live_mujoco_imu_verified": True,
            "slam_algorithm_output_verified": True,
            "canonical_nav_outputs_verified": True,
            "bridge_verified": True,
            "outputs": {
                "fastlio2_odometry": 8,
                "nav_odometry": 8,
                "nav_registered_cloud": 6,
                "nav_map_cloud": 6,
                "nav_cmd_vel_nonzero": 12,
            },
            "states_seen": ["TRACKING"],
            "point_count": {"cloud_map": 100},
            "frames": {"published_lidar": "body"},
            "runtime_contract": _fastlio2_runtime_contract(),
            "hardware_safety": _sim_command_hardware_safety(),
            "lidar_source": _mid360_lidar_source(),
            "lingtu_frontier": {
                "enabled": True,
                "verified": True,
                "started_after_slam_ready": True,
                "goal_count": 3,
                "successful_navigation_goal_count": 3,
                "failed_navigation_goal_count": 0,
                "min_required_goals": 3,
                "frontier_health": {"blocked_goal_count": 0},
            },
            "navigation_chain": {
                "planner_fallback_used": False,
                "planner_repair_used": False,
                "direct_goal_fallback": {"used": False},
                "health": {"plan_safety_policy": "reject"},
            },
            "map_growth": {
                "accepted_cumulative_growth_source": "/nav/map_cloud",
                "min_map_area_growth_m2": 1.0,
                "min_explored_area_growth_m2": 1.0,
                "min_exploration_coverage_growth_ratio": 0.01,
                "nav_map_cloud_xy_area": {"growth_m2": 2.0},
                "nav_map_cloud_area_samples": 4,
                "exploration_known_area": {"growth_m2": 2.0},
                "exploration_coverage": {"growth_ratio": 0.02},
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is True


def test_server_sim_closure_accepts_mujoco_tare_exploration_report(tmp_path: Path):
    report = _write_json(
        tmp_path / "mujoco_tare_success.json",
        _complete_fastlio2_tare_report(),
    )

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["mujoco_tare_exploration"]["evidence"]
    assert evidence["lingtu_tare"]["goal_count"] == 2
    assert evidence["lingtu_tare"]["successful_navigation_goal_count"] == 2


def test_server_sim_closure_accepts_mujoco_tare_with_moving_obstacle_video_evidence(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_tare_report()
    report_payload["moving_obstacles"] = {
        "enabled": True,
        "mode": "robot_crossing",
        "count": 2,
        "period_s": 8.0,
        "ok": True,
        "published_update_count": 12,
        "published_point_count_max": 80,
        "trail_clearance": {
            "checked": True,
            "collision": False,
            "min_clearance_minus_robot_radius_m": 0.42,
        },
    }
    report_payload["video_path"] = str(tmp_path / "tare_moving_obstacle.mp4")
    report_payload["video_frame_count"] = 12
    report_payload["video_sample_count"] = 12
    report = _write_json(tmp_path / "mujoco_tare_moving_obstacle.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["mujoco_tare_exploration"]["evidence"]
    assert evidence["moving_obstacles"]["mode"] == "robot_crossing"
    assert evidence["moving_obstacles"]["trail_collision"] is False
    assert evidence["video"]["frame_count"] == 12
    assert evidence["video"]["sample_count"] == 12


def test_server_sim_closure_accepts_fastlio2_inspection_with_moving_obstacle_video_evidence(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_inspection_report()
    report_payload["lingtu_inspection"]["failed_navigation_goal_count"] = 1
    report_payload["moving_obstacles"] = {
        "enabled": True,
        "mode": "robot_crossing",
        "count": 3,
        "period_s": 6.0,
        "ok": True,
        "published_update_count": 16,
        "published_point_count_max": 96,
        "trail_clearance": {
            "checked": True,
            "collision": False,
            "min_clearance_minus_robot_radius_m": 0.35,
        },
    }
    report_payload["video_path"] = str(tmp_path / "inspection_moving_obstacle.mp4")
    report_payload["video_frame_count"] = 14
    report_payload["video_sample_count"] = 14
    report = _write_json(tmp_path / "mujoco_inspection_moving_obstacle.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["fastlio2_live"]["evidence"]
    assert evidence["lingtu_inspection"]["successful_navigation_goal_count"] == 3
    assert evidence["moving_obstacles"]["mode"] == "robot_crossing"
    assert evidence["video"]["frame_count"] == 14


@pytest.mark.skipif(
    not (_module_import_is_safe("numpy") and _module_import_is_safe("cv2")),
    reason="Needs safe NumPy and OpenCV imports to generate a decodable video",
)
def test_server_sim_closure_accepts_fastlio2_dynamic_inspection_core_gate(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_dynamic_inspection_report(tmp_path)
    report = _write_json(tmp_path / "fastlio2_dynamic_inspection.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_dynamic_inspection": report},
        required={"fastlio2_dynamic_inspection"},
        include_optional=False,
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["fastlio2_dynamic_inspection"]["evidence"]["core_algorithm"]
    assert evidence["inspection"]["global_planner"] == "pct"
    assert evidence["inspection"]["replan_on_costmap_update"] is False
    assert evidence["moving_obstacles"]["count"] == 3
    assert evidence["video"]["frame_count"] == 181


def test_server_sim_closure_rejects_weak_fastlio2_dynamic_inspection_core_gate(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_dynamic_inspection_report(tmp_path)
    report_payload["lingtu_inspection"]["replan_on_costmap_update"] = True
    report_payload["moving_obstacles"]["count"] = 1
    report_payload["moving_obstacles"]["speed_bounds"]["peak_planar_speed_bound_mps"] = 0.5
    report_payload["moving_obstacles"]["trail_clearance"][
        "min_clearance_minus_robot_radius_m"
    ] = 0.1
    report_payload["video_frame_count"] = 0
    report_payload["fastlio2_z_consistency"]["ok"] = False
    report = _write_json(tmp_path / "fastlio2_dynamic_inspection_weak.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_dynamic_inspection": report},
        required={"fastlio2_dynamic_inspection"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "lingtu_inspection.replan_on_costmap_update is not false" in gaps
    assert "moving_obstacles.count < 3" in gaps
    assert "moving_obstacles peak_planar_speed_bound_mps < 0.75" in gaps
    assert "moving_obstacles trail clearance margin < 0.25" in gaps
    assert "fastlio2_dynamic_inspection video_frame_count missing" in gaps
    assert "fastlio2_z_consistency.ok is not true" in gaps


def test_server_sim_closure_rejects_fastlio2_dynamic_inspection_without_map_artifact_sha(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_dynamic_inspection_report(tmp_path)
    del report_payload["map_artifacts"]["assets"]["map_pcd"]["sha256"]
    report = _write_json(
        tmp_path / "fastlio2_dynamic_inspection_missing_map_sha.json",
        report_payload,
    )

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_dynamic_inspection": report},
        required={"fastlio2_dynamic_inspection"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "fastlio2_dynamic_inspection.map_pcd.sha256 missing" in gaps


def test_server_sim_closure_rejects_missing_fastlio2_dynamic_inspection_video_file(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_dynamic_inspection_report(tmp_path)
    missing_video = tmp_path / "missing_dyn3_fast.mp4"
    report_payload["video_path"] = str(missing_video)
    report = _write_json(tmp_path / "fastlio2_dynamic_inspection_missing_video.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_dynamic_inspection": report},
        required={"fastlio2_dynamic_inspection"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "fastlio2_dynamic_inspection video file missing" in gaps


def test_server_sim_closure_rejects_undecodable_fastlio2_dynamic_inspection_video_file(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_dynamic_inspection_report(tmp_path)
    fake_video = tmp_path / "not_a_video.mp4"
    fake_video.write_bytes(b"LingTu placeholder video artifact\n")
    report_payload["video_path"] = str(fake_video)
    report_payload["video_frame_count"] = 181
    report_payload["video_sample_count"] = 181
    report = _write_json(tmp_path / "fastlio2_dynamic_inspection_fake_video.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_dynamic_inspection": report},
        required={"fastlio2_dynamic_inspection"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert any(
        expected in gaps
        for expected in (
            "fastlio2_dynamic_inspection video decoder unavailable",
            "fastlio2_dynamic_inspection video first frame is not decodable",
            "fastlio2_dynamic_inspection video nonblack validation unavailable",
        )
    )


def test_server_sim_closure_core_algorithm_preset_selects_strict_gate_set():
    parser = server_sim_closure._build_parser()
    args = parser.parse_args(["--preset", "core_algorithm"])

    assert server_sim_closure._required_from_args(args) == {
        "large_terrain",
        "dynamic_obstacle_local_planner",
        "fastlio2_dynamic_inspection",
        "moving_obstacle_sweep",
        "large_loop_closure",
    }


def test_server_sim_closure_inspection_mvp_preset_selects_product_gate_set():
    parser = server_sim_closure._build_parser()
    args = parser.parse_args(["--preset", "inspection_mvp"])

    assert server_sim_closure._required_from_args(args) == {
        "gateway_runtime_acceptance",
        "routecheck_preflight",
        "large_terrain",
        "fastlio2_dynamic_inspection",
        "dynamic_obstacle_local_planner",
        "moving_obstacle_sweep",
    }


def test_server_sim_closure_dimos_benchmark_preset_selects_reference_gate_set():
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES

    parser = server_sim_closure._build_parser()
    args = parser.parse_args(["--preset", "dimos_benchmark"])

    assert server_sim_closure._required_from_args(args) == set(DIMOS_BENCHMARK_REQUIRED_GATES)


def test_server_sim_closure_g4_server_full_sim_preset_selects_closure_gate_set():
    from runtime.algorithm_gates import G4_SERVER_FULL_SIM_REQUIRED_GATES

    parser = server_sim_closure._build_parser()
    args = parser.parse_args(["--preset", "g4_server_full_sim"])

    assert server_sim_closure._required_from_args(args) == set(
        G4_SERVER_FULL_SIM_REQUIRED_GATES
    )


def test_server_sim_closure_gateway_runtime_acceptance_gate_is_product_data_plane():
    spec = next(
        gate for gate in server_sim_closure.GATES if gate.name == "gateway_runtime_acceptance"
    )

    assert "gateway-runtime-acceptance" in spec.command
    assert "--acceptance-mode non_motion" in spec.command
    assert "server_sim_closure/gateway_runtime_acceptance/report.json" in spec.command
    assert "ros2 topic" not in spec.description.lower()


def test_server_sim_closure_accepts_gateway_runtime_acceptance_report(tmp_path: Path):
    report = _write_json(
        tmp_path / "gateway_runtime_acceptance.json",
        _complete_gateway_runtime_acceptance_report(),
    )

    summary = server_sim_closure.summarize(
        report_overrides={"gateway_runtime_acceptance": report},
        required={"gateway_runtime_acceptance"},
        include_optional=False,
    )

    assert summary["ok"] is True, summary["remaining_gaps"]
    gate = summary["gates"]["gateway_runtime_acceptance"]
    assert gate["evidence"]["ros2_topic_required"] is False
    assert gate["evidence"]["module_port_bus_primary"] is True
    assert gate["evidence"]["ros2_adapter_primary"] is False
    assert gate["evidence"]["streamable_topic_count"] >= 7


def test_server_sim_closure_accepts_non_motion_gateway_stage_token_advisories(
    tmp_path: Path,
):
    payload = _complete_gateway_runtime_acceptance_report()
    payload["checks"]["stage_evidence"]["missing_tokens"] = {
        "global_planning": {"inputs": ["artifact:tomogram"], "outputs": []}
    }
    report = _write_json(tmp_path / "gateway_runtime_acceptance.json", payload)

    summary = server_sim_closure.summarize(
        report_overrides={"gateway_runtime_acceptance": report},
        required={"gateway_runtime_acceptance"},
        include_optional=False,
    )

    assert summary["ok"] is True, summary["remaining_gaps"]
    assert summary["gates"]["gateway_runtime_acceptance"]["evidence"][
        "missing_stage_tokens"
    ] == {
        "global_planning": {"inputs": ["artifact:tomogram"], "outputs": []}
    }


def test_server_sim_closure_rejects_gateway_runtime_acceptance_ros2_primary(
    tmp_path: Path,
):
    payload = _complete_gateway_runtime_acceptance_report()
    payload["ros2_topic_required"] = True
    payload["checks"]["module_first_dataflow"]["ros2_topic_required"] = True
    payload["checks"]["module_first_dataflow"]["module_port_bus_primary"] = False
    payload["checks"]["module_first_dataflow"]["ros2_adapter_primary"] = True
    payload["checks"]["module_first_dataflow"]["missing_stream_interfaces"] = [
        "/nav/odometry"
    ]
    report = _write_json(tmp_path / "gateway_runtime_acceptance_ros2.json", payload)

    summary = server_sim_closure.summarize(
        report_overrides={"gateway_runtime_acceptance": report},
        required={"gateway_runtime_acceptance"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "ros2_topic_required is not false" in gaps
    assert "module_port_bus_primary is not true" in gaps
    assert "ros2_adapter_primary is true" in gaps
    assert "missing Gateway SSE stream interfaces" in gaps


def test_server_sim_closure_validation_flow_includes_runtime_data_plane():
    stages = {stage["id"]: stage for stage in server_sim_closure.ALGORITHM_VALIDATION_FLOW}

    assert stages["runtime_data_plane"]["gates"] == (
        "gateway_runtime_acceptance",
    )
    assert "Gateway" in stages["runtime_data_plane"]["title"]
    assert "ROS2" not in stages["runtime_data_plane"]["title"]


def test_server_sim_closure_moving_obstacle_sweep_command_aggregates_reports():
    spec = next(gate for gate in server_sim_closure.GATES if gate.name == "moving_obstacle_sweep")

    assert "sim/scripts/moving_obstacle_sweep_gate.py" in spec.command
    assert "--run-matrix" in spec.command
    assert (
        "--world ${LINGTU_MUJOCO_LIVE_WORLD:-"
        "artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml}"
    ) in spec.command
    assert "--child-run-root artifacts/server_sim_closure/moving_obstacle_sweep/children" in spec.command
    assert "--report-glob" not in spec.command
    assert "--required-scan-time-profile physical_rolling" in spec.command
    assert "--require-video-file" in spec.command
    assert "server_sim_closure/moving_obstacle_sweep/report.json" in spec.command


def test_server_sim_closure_materializes_moving_obstacle_sweep_report_only(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    child_dir = tmp_path / "children"
    _write_moving_obstacle_child_report(
        child_dir / "slow_sparse.json",
        speed_mps=0.35,
        spacing_m=0.16,
    )
    _write_moving_obstacle_child_report(
        child_dir / "slow_dense.json",
        speed_mps=0.35,
        spacing_m=0.05,
    )
    _write_moving_obstacle_child_report(
        child_dir / "fast_sparse.json",
        speed_mps=0.95,
        spacing_m=0.16,
    )
    _write_moving_obstacle_child_report(
        child_dir / "fast_dense.json",
        speed_mps=0.95,
        spacing_m=0.05,
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    report_path, materialized = (
        server_sim_closure.materialize_moving_obstacle_sweep_report_only(
            report_globs=[str(child_dir / "*.json")],
            allow_missing_video_file=True,
        )
    )

    assert report_path == (
        tmp_path / "artifacts/server_sim_closure/moving_obstacle_sweep/report.json"
    )
    assert materialized["gate"] == "moving_obstacle_sweep"
    assert materialized["source"] == "existing_child_reports"
    assert materialized["ok"] is True
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["execution_mode"] == "report_only_aggregate"
    assert report["run_matrix"]["enabled"] is False
    assert report["claim_boundary"] == (
        "report_only_aggregate_requires_child_live_fastlio_pct_cmd_vel_evidence"
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report_path},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is True
    assert summary["verified"]["moving_obstacle_sweep"] is True
    evidence = summary["gates"]["moving_obstacle_sweep"]["evidence"]
    assert evidence["passed_pair_count"] == 4
    assert evidence["required_live_nav_chain"] is True


def test_server_sim_closure_rejects_moving_obstacle_materialize_with_host_preflight(
    tmp_path: Path,
):
    result = subprocess.run(
        [
            sys.executable,
            "sim/scripts/server_sim_closure.py",
            "--required",
            "moving_obstacle_sweep",
            "--host-preflight",
            "--moving-obstacle-sweep-report-glob",
            str(tmp_path / "*.json"),
            "--json-out",
            str(tmp_path / "summary.json"),
        ],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode != 0
    assert "cannot be combined with --host-preflight" in result.stderr


def test_server_sim_closure_accepts_moving_obstacle_sweep_report(tmp_path: Path):
    report = _write_json(
        tmp_path / "moving_obstacle_sweep.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 4,
            "passed_case_count": 4,
            "passed_pair_count": 4,
            "required_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "covered_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "missing_pairs": [],
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_scan_time_profile": "physical_rolling",
            "required_live_nav_chain": True,
            "cases": [
                {
                    "pair": pair,
                    "ok": True,
                    "scan_time_profile": "physical_rolling",
                    "live_nav_chain": {
                        "ok": True,
                        "nav_data_source": "fastlio2",
                        "outputs": {
                            "nav_odometry": 20,
                            "nav_map_cloud": 20,
                            "nav_cmd_vel_nonzero": 12,
                        },
                        "lingtu_inspection": {
                            "verified": True,
                            "global_planner": "pct",
                            "global_path_count": 1,
                            "local_path_count": 24,
                        },
                        "map_artifacts": _same_source_map_artifacts(
                            pair.replace(":", "_")
                        ),
                        "blockers": [],
                    },
                }
                for pair in [
                    "slow:sparse",
                    "slow:dense",
                    "fast:sparse",
                    "fast:dense",
                ]
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["moving_obstacle_sweep"]["evidence"]
    assert evidence["passed_case_count"] == 4
    assert evidence["covered_pairs"] == [
        "slow:sparse",
        "slow:dense",
        "fast:sparse",
        "fast:dense",
    ]


def test_server_sim_closure_rejects_moving_obstacle_sweep_without_live_map_sha(
    tmp_path: Path,
):
    pairs = ["slow:sparse", "slow:dense", "fast:sparse", "fast:dense"]
    cases = []
    for pair in pairs:
        map_artifacts = _same_source_map_artifacts(pair.replace(":", "_"))
        if pair == "fast:dense":
            del map_artifacts["assets"]["map_pcd"]["sha256"]
        cases.append(
            {
                "pair": pair,
                "ok": True,
                "scan_time_profile": "physical_rolling",
                "live_nav_chain": {
                    "ok": True,
                    "nav_data_source": "fastlio2",
                    "outputs": {
                        "nav_odometry": 20,
                        "nav_map_cloud": 20,
                        "nav_cmd_vel_nonzero": 12,
                    },
                    "lingtu_inspection": {
                        "verified": True,
                        "global_planner": "pct",
                        "global_path_count": 1,
                        "local_path_count": 24,
                    },
                    "map_artifacts": map_artifacts,
                    "blockers": [],
                },
            }
        )
    report = _write_json(
        tmp_path / "moving_obstacle_sweep_missing_map_sha.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 4,
            "passed_case_count": 4,
            "passed_pair_count": 4,
            "required_pairs": pairs,
            "covered_pairs": pairs,
            "missing_pairs": [],
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_scan_time_profile": "physical_rolling",
            "required_live_nav_chain": True,
            "cases": cases,
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert (
        "moving_obstacle_sweep case fast:dense.live_nav_chain.map_pcd.sha256 missing"
        in gaps
    )


def test_server_sim_closure_rejects_moving_obstacle_sweep_undecodable_required_video_file(
    tmp_path: Path,
):
    pairs = [
        "slow:sparse",
        "slow:dense",
        "fast:sparse",
        "fast:dense",
    ]
    cases = []
    for pair in pairs:
        case_dir = tmp_path / "children" / pair.replace(":", "_")
        fake_video = case_dir / "not_a_video.mp4"
        fake_video.parent.mkdir(parents=True, exist_ok=True)
        fake_video.write_bytes(b"LingTu placeholder video artifact\n")
        cases.append(
            {
                "pair": pair,
                "path": str(case_dir / "report.json"),
                "ok": True,
                "scan_time_profile": "physical_rolling",
                "video": {
                    "path": "not_a_video.mp4",
                    "frame_count": 18,
                    "sample_count": 18,
                },
                "live_nav_chain": {
                    "ok": True,
                    "nav_data_source": "fastlio2",
                    "outputs": {
                        "nav_odometry": 20,
                        "nav_map_cloud": 20,
                        "nav_cmd_vel_nonzero": 12,
                    },
                    "lingtu_inspection": {
                        "verified": True,
                        "global_planner": "pct",
                        "global_path_count": 1,
                        "local_path_count": 24,
                    },
                    "map_artifacts": _same_source_map_artifacts(
                        pair.replace(":", "_")
                    ),
                    "blockers": [],
                },
            }
        )
    report = _write_json(
        tmp_path / "moving_obstacle_sweep_fake_video.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 4,
            "passed_case_count": 4,
            "passed_pair_count": 4,
            "required_pairs": pairs,
            "covered_pairs": pairs,
            "missing_pairs": [],
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_scan_time_profile": "physical_rolling",
            "required_live_nav_chain": True,
            "require_video_file": True,
            "cases": cases,
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert _has_video_decode_gap(gaps, "moving_obstacle_sweep case slow:sparse")
    assert "moving_obstacle_sweep case slow:sparse video file missing" not in gaps


def test_server_sim_closure_finds_physical_rolling_moving_obstacle_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    _write_json(
        tmp_path
        / "artifacts/server_sim_closure/moving_obstacle_sweep/report_physical_rolling_20260521.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 4,
            "passed_case_count": 4,
            "passed_pair_count": 4,
            "required_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "covered_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "missing_pairs": [],
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_scan_time_profile": "physical_rolling",
            "required_live_nav_chain": True,
            "cases": [
                {
                    "pair": pair,
                    "ok": True,
                    "scan_time_profile": "physical_rolling",
                    "live_nav_chain": {
                        "ok": True,
                        "nav_data_source": "fastlio2",
                        "outputs": {
                            "nav_odometry": 20,
                            "nav_map_cloud": 20,
                            "nav_cmd_vel_nonzero": 12,
                        },
                        "lingtu_inspection": {
                            "verified": True,
                            "global_planner": "pct",
                            "global_path_count": 1,
                            "local_path_count": 24,
                        },
                        "map_artifacts": _same_source_map_artifacts(
                            pair.replace(":", "_")
                        ),
                        "blockers": [],
                    },
                }
                for pair in [
                    "slow:sparse",
                    "slow:dense",
                    "fast:sparse",
                    "fast:dense",
                ]
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is True
    assert summary["gates"]["moving_obstacle_sweep"]["path"].endswith(
        "report_physical_rolling_20260521.json"
    )


def test_server_sim_closure_rejects_incomplete_moving_obstacle_sweep_report(tmp_path: Path):
    report = _write_json(
        tmp_path / "moving_obstacle_sweep_incomplete.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 0,
            "required_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "covered_pairs": ["fast:medium"],
            "missing_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "blockers": ["missing required speed bins: slow"],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "moving_obstacle_sweep report.ok is not true" in gaps


def test_server_sim_closure_classifies_moving_obstacle_preflight_blocker_as_environment_or_artifact(
    tmp_path: Path,
):
    report = _write_json(
        tmp_path / "moving_obstacle_sweep_preflight_failed.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 0,
            "passed_case_count": 0,
            "passed_pair_count": 0,
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
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_scan_time_profile": "physical_rolling",
            "required_live_nav_chain": True,
            "environment_blockers": [
                "launch_script missing: sim/scripts/mujoco/launch_fastlio2_live.sh"
            ],
            "artifact_blockers": [
                "inspection_tomogram missing: artifacts/server_sim_closure/large_terrain/tomogram.pickle"
            ],
            "blocking_subsystems": ["environment_runtime", "artifact_contract"],
            "preflight": {"ok": False},
            "blockers": [
                "launch_script missing: sim/scripts/mujoco/launch_fastlio2_live.sh",
                "inspection_tomogram missing: artifacts/server_sim_closure/large_terrain/tomogram.pickle",
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is False
    evidence = summary["gates"]["moving_obstacle_sweep"]["evidence"]
    assert evidence["environment_blockers"] == [
        "launch_script missing: sim/scripts/mujoco/launch_fastlio2_live.sh"
    ]
    assert evidence["artifact_blockers"] == [
        "inspection_tomogram missing: artifacts/server_sim_closure/large_terrain/tomogram.pickle"
    ]
    categories = summary["algorithm_validation"]["gate_categories"][
        "moving_obstacle_sweep"
    ]
    assert "environment_runtime" in categories
    assert "artifact_contract" in categories
    action = summary["algorithm_validation"]["next_actions"][0]
    assert action["category"] == "environment_runtime"
    assert action["action_type"] == "fix_runtime_then_rerun"


def test_server_sim_closure_propagates_moving_sweep_root_cause_categories(
    tmp_path: Path,
):
    report = _write_json(
        tmp_path / "moving_obstacle_sweep_slam_red.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 4,
            "passed_case_count": 0,
            "passed_pair_count": 0,
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
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_scan_time_profile": "physical_rolling",
            "required_live_nav_chain": True,
            "minimal_red_defect": {
                "blocking_subsystem": "slam_localization",
                "pair": "fast:dense",
                "fastlio2_z_delta_error_m": 4.43,
            },
            "blocking_subsystems": ["slam_localization", "planning_tracking"],
            "cases": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    evidence = summary["gates"]["moving_obstacle_sweep"]["evidence"]
    assert evidence["minimal_red_defect"]["blocking_subsystem"] == "slam_localization"
    validation = summary["algorithm_validation"]
    assert validation["gate_categories"]["moving_obstacle_sweep"] == [
        "dynamic_obstacle",
        "planning_tracking",
        "slam_localization",
    ]
    assert validation["blocking_categories"]["slam_localization"] == [
        "moving_obstacle_sweep"
    ]


def test_server_sim_closure_rejects_moving_obstacle_sweep_without_live_nav_chain(
    tmp_path: Path,
):
    report = _write_json(
        tmp_path / "moving_obstacle_sweep_weak_nav.json",
        {
            "schema_version": "lingtu.moving_obstacle_sweep_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 4,
            "passed_case_count": 4,
            "passed_pair_count": 4,
            "required_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "covered_pairs": [
                "slow:sparse",
                "slow:dense",
                "fast:sparse",
                "fast:dense",
            ],
            "missing_pairs": [],
            "required_speed_bins": ["slow", "fast"],
            "required_density_bins": ["sparse", "dense"],
            "required_live_nav_chain": False,
            "cases": [
                {
                    "pair": "slow:sparse",
                    "ok": True,
                    "live_nav_chain": {
                        "ok": False,
                        "blockers": ["nav_data_source is not fastlio2"],
                    },
                }
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"moving_obstacle_sweep": report},
        required={"moving_obstacle_sweep"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "required_live_nav_chain is not true" in gaps
    assert "moving_obstacle_sweep case slow:sparse live nav chain is not ok" in gaps


def test_server_sim_closure_accepts_moving_obstacle_sweep_report_override(tmp_path: Path):
    report = tmp_path / "moving_obstacle_sweep.json"
    parser = server_sim_closure._build_parser()
    args = parser.parse_args([
        "--moving-obstacle-sweep-report",
        str(report),
    ])

    assert args.moving_obstacle_sweep_report == report


def test_server_sim_closure_large_loop_closure_command_aggregates_runtime_report():
    spec = next(gate for gate in server_sim_closure.GATES if gate.name == "large_loop_closure")

    assert "sim/scripts/large_loop_closure_gate.py" in spec.command
    assert "inspection-loop-video" in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_WORLD=${LINGTU_MUJOCO_LIVE_WORLD:-"
        "artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S="
        "${LINGTU_MUJOCO_LIVE_MAX_WALL_TIME_S:-7200}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED="
        "${LINGTU_MUJOCO_LIVE_NAV_MAX_LINEAR_SPEED:-0.45}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT="
        "${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_LIMIT:-0.45}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT="
        "${LINGTU_MUJOCO_LIVE_CMD_VEL_LINEAR_ACCEL_LIMIT:-0.8}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT="
        "${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT:-0.25}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z="
        "${LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z:-0.25}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM="
        "${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-1}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER="
        "${LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER:-10}"
    ) in spec.command
    assert (
        "LINGTU_PCT_OPTIMIZE_TRAJECTORY="
        "${LINGTU_PCT_OPTIMIZE_TRAJECTORY:-1}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES="
        "${LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES:-6}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M="
        "${LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M:-1.0}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START="
        "${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_YAW_RATE_START:-0.0}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE="
        "${LINGTU_MUJOCO_LIVE_NAV_TURN_SPEED_MIN_SCALE:-1.0}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_GOALS="
        f"${{LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-{server_sim_closure.LARGE_LOOP_INSPECTION_GOALS}}}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_DOWNSAMPLE_DIST:-1.0}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_WAYPOINT_THRESHOLD:-0.65}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_FINAL_WAYPOINT_THRESHOLD:-0.65}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_COMPLETE_PATH_ON_GOAL_PROXIMITY:-1}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_GOAL_PROXIMITY_COMPLETION_THRESHOLD:-0.65}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_LOOKAHEAD:-2.0}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_YAW_RATE_GAIN:-2.5}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_STOP_YAW_RATE_GAIN="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_STOP_YAW_RATE_GAIN:-2.5}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_PATH_DIR_DIFF_THRE:-1.8}"
    ) in spec.command
    assert "LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE=${LINGTU_MUJOCO_LIVE_SCAN_TIME_PROFILE:-physical_rolling}" in spec.command
    assert "run_start=$(date +%s)" in spec.command
    assert "mujoco/launch_fastlio2_live.sh inspection-loop-video;" in spec.command
    assert "test -n \"$latest\" && test -f \"$latest/report.json\"" in spec.command
    assert "stat -c %Y \"$latest/report.json\"" in spec.command
    assert "stat -c %Y \"$latest/report.json\")\" -ge \"$run_start\" &&" in spec.command
    assert "--required-scan-time-profile physical_rolling" in spec.command
    assert "--require-video-file" in spec.command
    assert "server_sim_closure/large_loop_closure/report.json" in spec.command


def test_mujoco_live_launcher_large_loop_defaults_close_in_live_planning_frame():
    launcher = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(
        encoding="utf-8"
    )

    assert (
        "inspection_default_goals="
        f'"${{LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-{server_sim_closure.LARGE_LOOP_INSPECTION_GOALS}}}"'
    ) in launcher
    assert '"--inspection-goals=$inspection_default_goals"' in launcher
    assert '"--inspection-goals" "$inspection_default_goals"' not in launcher
    assert "inspection_default_min_checkpoints=\"${LINGTU_MUJOCO_LIVE_INSPECTION_MIN_CHECKPOINTS:-4}\"" in launcher
    assert "inspection_default_planner=\"${LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER:-pct}\"" in launcher
    assert 'inspection_downsample_dist_default="1.0"' in launcher
    assert 'cmd_vel_linear_limit_default="0.45"' in launcher
    assert 'cmd_vel_angular_limit_default="0.25"' in launcher
    assert 'nav_max_linear_speed_default="0.45"' in launcher
    assert 'nav_max_angular_z_default="0.25"' in launcher
    assert 'inspection_waypoint_threshold_default="0.65"' in launcher
    assert 'inspection_final_waypoint_threshold_default="0.65"' in launcher
    assert 'inspection_complete_path_on_goal_proximity_default="0"' in launcher
    assert 'inspection_complete_path_on_goal_proximity_default="1"' in launcher
    assert 'inspection_goal_proximity_completion_threshold_default="0.65"' in launcher
    assert 'inspection_path_lookahead_default="2.0"' in launcher
    assert 'inspection_path_min_speed_default="0.15"' in launcher
    assert 'inspection_path_yaw_rate_gain_default="2.5"' in launcher
    assert 'inspection_path_stop_yaw_rate_gain_default="2.5"' in launcher
    assert 'inspection_path_dir_diff_thre_default="1.8"' in launcher
    assert 'nav_turn_speed_yaw_rate_start_default="0.0"' in launcher
    assert 'nav_turn_speed_min_scale_default="1.0"' in launcher


def test_mujoco_live_launcher_writes_red_report_when_runtime_report_missing():
    launcher = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(
        encoding="utf-8"
    )

    assert 'if [[ ! -f "$run_dir/report.json" ]]; then' in launcher
    assert "runtime_report_missing_after_launcher" in launcher
    assert "mujoco_live_gate exited without writing report.json" in launcher
    assert 'partial_path = run_dir / "report.partial.json"' in launcher
    assert '"partial_report": partial' in launcher
    assert '"outputs": partial.get("outputs", {})' in launcher
    assert '"lingtu_inspection": (' in launcher
    assert '"runtime_fault_events": (' in launcher


def test_server_sim_closure_accepts_large_loop_closure_report(tmp_path: Path):
    report = _write_json(
        tmp_path / "large_loop_closure.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": "runtime_loop.json",
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 24.0,
                "fastlio2_path_length_m": 23.8,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 0.41,
                "sim_loop_yaw_error_rad": 0.1,
                "fastlio2_loop_yaw_error_rad": 0.1,
                "nav_cmd_vel_nonzero": 420,
                "local_path_count": 80,
                "video_frame_count": 120,
                "map_artifacts": _same_source_map_artifacts("large_loop"),
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is True
    evidence = summary["gates"]["large_loop_closure"]["evidence"]
    assert evidence["best_case"]["global_planner"] == "pct"
    assert evidence["best_case"]["sim_loop_closure_error_m"] == 0.32


def test_server_sim_closure_rejects_short_large_loop_even_with_report_ok(tmp_path: Path):
    report = _write_json(
        tmp_path / "large_loop_closure_short_path.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": "runtime_loop.json",
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 9.4334,
                "fastlio2_path_length_m": 7.4618,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 0.41,
                "sim_loop_yaw_error_rad": 0.1,
                "fastlio2_loop_yaw_error_rad": 0.1,
                "nav_cmd_vel_nonzero": 376,
                "local_path_count": 382,
                "video_frame_count": 120,
                "map_artifacts": _same_source_map_artifacts("large_loop_short"),
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "best_case.sim_path_length_m below threshold" in gaps
    assert "best_case.fastlio2_path_length_m below threshold" in gaps


def test_server_sim_closure_rejects_large_loop_with_mismatched_tomogram_source(
    tmp_path: Path,
):
    map_artifacts = _same_source_map_artifacts("large_loop_mismatch")
    map_artifacts["assets"]["tomogram"]["source_map_sha256"] = "different-map-sha"
    report = _write_json(
        tmp_path / "large_loop_closure_mismatched_tomogram.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": "runtime_loop.json",
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 24.0,
                "fastlio2_path_length_m": 23.8,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 0.41,
                "sim_loop_yaw_error_rad": 0.1,
                "fastlio2_loop_yaw_error_rad": 0.1,
                "nav_cmd_vel_nonzero": 420,
                "local_path_count": 80,
                "video_frame_count": 120,
                "map_artifacts": map_artifacts,
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert (
        "large_loop_closure best_case.tomogram.source_map_sha256 does not match "
        "map_pcd.sha256"
    ) in gaps


def test_server_sim_closure_rejects_large_loop_undecodable_required_video_file(
    tmp_path: Path,
):
    child_dir = tmp_path / "children" / "loop"
    fake_video = child_dir / "not_a_video.mp4"
    fake_video.parent.mkdir(parents=True, exist_ok=True)
    fake_video.write_bytes(b"LingTu placeholder video artifact\n")
    report = _write_json(
        tmp_path / "large_loop_closure_fake_video.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": str(child_dir / "runtime_loop.json"),
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 24.0,
                "fastlio2_path_length_m": 23.8,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 0.41,
                "sim_loop_yaw_error_rad": 0.1,
                "fastlio2_loop_yaw_error_rad": 0.1,
                "nav_cmd_vel_nonzero": 420,
                "local_path_count": 80,
                "video_path": "not_a_video.mp4",
                "video_frame_count": 120,
                "video_sample_count": 120,
                "map_artifacts": _same_source_map_artifacts("large_loop"),
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
                "require_video_file": True,
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert _has_video_decode_gap(gaps, "large_loop_closure best_case")
    assert "large_loop_closure best_case video file missing" not in gaps


def test_server_sim_closure_finds_same_source_large_loop_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    _write_json(
        tmp_path
        / "artifacts/server_sim_closure/large_loop_physical_rolling_same_source/large_loop_closure_report.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": "runtime_loop.json",
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 24.0,
                "fastlio2_path_length_m": 23.8,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 0.41,
                "sim_loop_yaw_error_rad": 0.1,
                "fastlio2_loop_yaw_error_rad": 0.1,
                "nav_cmd_vel_nonzero": 420,
                "local_path_count": 80,
                "video_frame_count": 120,
                "map_artifacts": _same_source_map_artifacts("large_loop"),
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is True
    assert summary["gates"]["large_loop_closure"]["path"].endswith(
        "large_loop_closure_report.json"
    )


def test_server_sim_closure_rejects_large_loop_fastlio_loop_error(tmp_path: Path):
    report = _write_json(
        tmp_path / "large_loop_closure_bad_fastlio_loop.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": "runtime_loop.json",
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 24.0,
                "fastlio2_path_length_m": 23.8,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 3.0,
                "sim_loop_yaw_error_rad": 0.1,
                "fastlio2_loop_yaw_error_rad": 0.1,
                "nav_cmd_vel_nonzero": 420,
                "local_path_count": 80,
                "video_frame_count": 120,
                "map_artifacts": _same_source_map_artifacts("large_loop_bad_error"),
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "best_case.fastlio2_loop_closure_error_m above threshold" in gaps


def test_server_sim_closure_rejects_large_loop_yaw_error(tmp_path: Path):
    report = _write_json(
        tmp_path / "large_loop_closure_bad_yaw.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 1,
            "best_case": {
                "path": "runtime_loop.json",
                "global_planner": "pct",
                "scan_time_profile": "physical_rolling",
                "sim_path_length_m": 24.0,
                "fastlio2_path_length_m": 23.8,
                "sim_loop_closure_error_m": 0.32,
                "fastlio2_loop_closure_error_m": 0.41,
                "sim_loop_yaw_error_rad": 0.7,
                "fastlio2_loop_yaw_error_rad": 0.8,
                "nav_cmd_vel_nonzero": 420,
                "local_path_count": 80,
                "video_frame_count": 120,
                "map_artifacts": _same_source_map_artifacts("large_loop_bad_yaw"),
            },
            "thresholds": {
                "min_path_length_m": 20.0,
                "max_loop_closure_error_m": 0.75,
                "max_fastlio_loop_closure_error_m": 1.0,
                "max_loop_yaw_error_rad": 0.5,
                "required_scan_time_profile": "physical_rolling",
            },
            "blockers": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "best_case.sim_loop_yaw_error_rad above threshold" in gaps
    assert "best_case.fastlio2_loop_yaw_error_rad above threshold" in gaps


def test_server_sim_closure_rejects_failed_large_loop_closure_report(tmp_path: Path):
    report = _write_json(
        tmp_path / "large_loop_closure_failed.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 0,
            "best_case": {},
            "minimal_red_defect": {
                "blocking_subsystem": "slam_localization",
                "fastlio2_z_delta_error_m": 10.3896,
                "global_planner": "pct",
                "local_path_count": 80,
                "nav_cmd_vel_nonzero": 420,
            },
            "blocking_subsystems": ["slam_localization"],
            "blockers": ["no passing large-loop runtime report"],
            "cases": [
                {
                    "ok": False,
                    "blockers": [
                        "fastlio2_z_consistency.ok is not true",
                        "sim_path_length_m < 20",
                    ],
                }
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "large_loop_closure report.ok is not true" in gaps
    assert "no passing large-loop runtime report" in gaps
    assert "case[0]: fastlio2_z_consistency.ok is not true" in gaps
    assert "best_case.video_frame_count missing" not in gaps
    validation = summary["algorithm_validation"]
    assert validation["claim_allowed"] is False
    assert validation["claim"] == "simulation_algorithm_health"
    assert validation["blocking_categories"]["slam_localization"] == ["large_loop_closure"]
    assert "large_loop_closure" in validation["stop_condition"]
    evidence = summary["gates"]["large_loop_closure"]["evidence"]
    assert evidence["minimal_red_defect"]["blocking_subsystem"] == "slam_localization"
    assert evidence["minimal_red_defect"]["fastlio2_z_delta_error_m"] == 10.3896


def test_server_sim_closure_algorithm_validation_flow_keeps_global_and_local_roles_clear(
    tmp_path: Path,
) -> None:
    report = _write_json(
        tmp_path / "large_loop_closure_failed.json",
        {
            "schema_version": "lingtu.large_loop_closure_gate.v1",
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "case_count": 1,
            "passed_case_count": 0,
            "best_case": {},
            "minimal_red_defect": {
                "blocking_subsystem": "slam_localization",
                "fastlio2_z_delta_error_m": 2.2,
                "global_planner": "pct",
                "local_path_count": 80,
                "nav_cmd_vel_nonzero": 420,
            },
            "blocking_subsystems": ["slam_localization"],
            "blockers": ["no passing large-loop runtime report"],
            "cases": [
                {
                    "ok": False,
                    "blockers": ["fastlio2_z_consistency.ok is not true"],
                }
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_loop_closure": report},
        required={"large_loop_closure"},
        include_optional=False,
    )

    validation = summary["algorithm_validation"]
    assert validation["claim_allowed"] is False
    assert validation["claim_boundary"]["global_planning_source"] == "static_saved_map_tomogram"
    assert validation["claim_boundary"]["realtime_mapping_source"] == "fastlio2_lidar_imu"
    assert validation["claim_boundary"]["live_costmap_role"] == "local_planning_and_safety_only"
    assert validation["flow_ok"] is False
    stages = {stage["id"]: stage for stage in validation["validation_flow"]}
    assert stages["realtime_mapping_localization"]["status"] == "failed"
    assert stages["long_range_loop_closure"]["status"] == "failed"
    assert stages["static_global_planning"]["status"] == "not_required"
    assert stages["long_range_loop_closure"]["blocking_gates"] == ["large_loop_closure"]


def test_server_sim_closure_accepts_large_loop_closure_report_override(tmp_path: Path):
    report = tmp_path / "large_loop_closure.json"
    parser = server_sim_closure._build_parser()
    args = parser.parse_args([
        "--large-loop-closure-report",
        str(report),
    ])

    assert args.large_loop_closure_report == report


def test_server_sim_closure_prefers_fresh_failed_report_over_stale_pass(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    fresh_failed = _write_json(
        tmp_path / "artifacts/server_sim_closure/large_terrain/report.json",
        {
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [],
        },
    )
    stale_pass = _write_json(
        tmp_path / "artifacts/large_terrain_nav_validation_old/report.json",
        {
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "cases": [
                {
                    "route": "terrain_long",
                    "path_safety": {"ok": True},
                    "planning": [
                        {
                            "planner": "pct",
                            "native_backend_used": True,
                        }
                    ],
                }
            ],
        },
    )
    os.utime(stale_pass, (1, 1))

    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"large_terrain"},
        max_report_age_s=3600,
        include_optional=False,
    )

    assert summary["ok"] is False
    assert Path(summary["gates"]["large_terrain"]["path"]) == fresh_failed
    gaps = "\n".join(summary["remaining_gaps"])
    assert "large_terrain: report.ok is not true" in gaps
    assert "report_age_s" not in gaps


def test_server_sim_closure_prefers_fresh_failed_native_pct_over_stale_pass(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    fresh_failed = _write_json(
        tmp_path / "artifacts/server_sim_closure/native_pct_mujoco/report.json",
        {"ok": False},
    )
    stale_pass = _write_json(
        tmp_path / "artifacts/native_pct_mujoco_gate_old/report.json",
        {
            "ok": True,
            "planner": "pct",
            "pct_native_backend_used": True,
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_used": False,
        },
    )
    os.utime(stale_pass, (1, 1))

    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"native_pct_mujoco"},
        include_optional=False,
    )

    assert summary["ok"] is False
    assert Path(summary["gates"]["native_pct_mujoco"]["path"]) == fresh_failed


def test_server_sim_closure_large_terrain_surfaces_pct_runtime_blocker(tmp_path: Path) -> None:
    report = _write_json(
        tmp_path / "large_terrain_pct_runtime_missing.json",
        {
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "execution_mode": "host_guard",
            "native_runtime": {
                "ok": False,
                "canonical_arch": "x86_64",
                "python_tag": "py313",
                "missing": ["ele_planner.cpython-313-x86_64-linux-gnu.so"],
                "error": "No runnable PCT native modules",
            },
            "environment": {
                "accepted_host": False,
                "blocked_reason": "pct_native_runtime_unavailable",
                "claim_boundary": "environment_blocked_no_algorithm_claim",
            },
            "environment_blockers": ["PCT native runtime unavailable"],
            "cases": [
                {
                    "route": "terrain_short",
                    "path_safety": {"ok": True},
                    "planning": [
                        {
                            "planner": "pct",
                            "native_backend_used": False,
                            "native_runtime": {
                                "ok": False,
                                "canonical_arch": "x86_64",
                                "python_tag": "py313",
                                "missing": ["ele_planner.cpython-313-x86_64-linux-gnu.so"],
                                "error": "No runnable PCT native modules",
                            },
                        }
                    ],
                }
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_terrain": report},
        required={"large_terrain"},
        include_optional=False,
    )

    assert summary["ok"] is False
    gate = summary["gates"]["large_terrain"]
    assert "PCT native runtime unavailable" in gate["blockers"]
    assert gate["evidence"]["execution_mode"] == "host_guard"
    assert gate["evidence"]["native_runtime"]["python_tag"] == "py313"
    assert gate["evidence"]["environment"]["claim_boundary"] == (
        "environment_blocked_no_algorithm_claim"
    )
    assert "environment_runtime" in summary["algorithm_validation"]["gate_categories"]["large_terrain"]


def test_server_sim_closure_next_actions_separate_runtime_blocker_from_missing_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    report = _write_json(
        tmp_path / "large_terrain_pct_runtime_missing.json",
        {
            "ok": False,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "native_runtime": {
                "ok": False,
                "canonical_arch": "x86_64",
                "python_tag": "py313",
                "missing": ["traj_opt.cpython-313-x86_64-linux-gnu.so"],
                "error": "No runnable PCT native modules",
            },
            "environment_blockers": ["PCT native runtime unavailable"],
            "cases": [],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"large_terrain": report},
        required={"large_terrain", "fastlio2_dynamic_inspection"},
        include_optional=False,
    )

    actions = {action["gate"]: action for action in summary["next_actions"]}
    assert actions["large_terrain"]["category"] == "environment_runtime"
    assert actions["large_terrain"]["action_type"] == "fix_runtime_then_rerun"
    assert "PCT native runtime unavailable" in actions["large_terrain"]["blockers"]
    assert "large_terrain_nav_validation.py" in actions["large_terrain"]["command"]
    assert any(
        "PCT native extension modules" in item
        for item in actions["large_terrain"]["host_requirements"]
    )
    assert actions["large_terrain"]["expected_report_path"] == (
        "artifacts/server_sim_closure/large_terrain/report.json"
    )
    assert "artifacts/large_terrain_nav_validation*/report.json" in actions["large_terrain"][
        "accepted_patterns"
    ]
    assert actions["fastlio2_dynamic_inspection"]["category"] == "artifact_contract"
    assert actions["fastlio2_dynamic_inspection"]["action_type"] == "generate_missing_report"
    assert "mujoco/launch_fastlio2_live.sh" in actions["fastlio2_dynamic_inspection"]["command"]
    assert actions["fastlio2_dynamic_inspection"]["expected_report_path"].startswith(
        "artifacts/server_sim_closure/mujoco_fastlio2_live"
    )
    assert any(
        "inspection" in pattern
        for pattern in actions["fastlio2_dynamic_inspection"]["accepted_patterns"]
    )
    assert any(
        "ROS 2 Humble" in item
        for item in actions["fastlio2_dynamic_inspection"]["host_requirements"]
    )
    assert any(
        "MuJoCo EGL" in item
        for item in actions["fastlio2_dynamic_inspection"]["host_requirements"]
    )
    assert summary["algorithm_validation"]["next_actions"] == summary["next_actions"]


def test_server_sim_closure_saved_map_relocalize_next_action_lists_localizer_host(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"saved_map_relocalize"},
        include_optional=False,
    )

    action = summary["next_actions"][0]
    assert action["gate"] == "saved_map_relocalize"
    assert action["expected_report_path"] == (
        "artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json"
    )
    assert any("MuJoCo/Fast-LIO live feed" in item for item in action["host_requirements"])
    assert any("localizer runtime" in item for item in action["host_requirements"])
    assert not any("PCT native extension modules" in item for item in action["host_requirements"])
    assert summary["host_requirements"]["saved_map_relocalize"] == action["host_requirements"]


def test_server_sim_closure_native_pct_missing_summary_lists_runtime_requirements(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)
    summary = server_sim_closure.summarize(
        report_overrides={},
        required={"native_pct_mujoco"},
        include_optional=False,
    )

    command = summary["missing_required_commands"][0]
    assert command["name"] == "native_pct_mujoco"
    assert "mujoco/native_pct_gate.py" in command["command"]
    assert "mujoco/launch_fastlio2_live.sh" not in command["command"]
    assert "--route terrain_short" in command["command"]
    assert "--timeout-s 80" in command["command"]
    assert "--near-field-stop-distance 0.35" in command["command"]
    assert command["expected_report_path"] == (
        "artifacts/server_sim_closure/native_pct_mujoco/report.json"
    )
    assert (
        "artifacts/server_sim_closure/native_pct_mujoco/report.*.server.json"
        in command["accepted_patterns"]
    )
    assert any("PCT native extension modules" in item for item in command["host_requirements"])
    assert any("CPython 3.10" in item for item in command["host_requirements"])
    assert any("ROS 2 Humble" in item for item in command["host_requirements"])
    assert any("MuJoCo EGL" in item for item in command["host_requirements"])
    assert any("MID-360 scan pattern asset" in item for item in command["host_requirements"])
    assert any(
        "no physical robot drivers or hardware command publishers" in item
        for item in command["host_requirements"]
    )


def test_server_sim_host_preflight_blocks_pct_gate_on_wrong_host():
    report = server_sim_closure.host_preflight(
        required={"large_terrain"},
        platform_system="Windows",
        machine="AMD64",
        python_tag="py313",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda _name: False,
        path_exists=lambda _path: False,
        pct_runtime_report={
            "ok": False,
            "host_platform_supported": False,
            "python_abi_matches_known_good": False,
            "error": "No runnable PCT native modules for arch=x86_64 python=py313",
        },
    )

    assert report["schema_version"] == "lingtu.server_sim_host_preflight.v1"
    assert report["execution_mode"] == "host_preflight_only"
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["ok"] is False
    assert report["blocked_gates"] == ["large_terrain"]
    gate = report["gates"]["large_terrain"]
    assert gate["ok"] is False
    assert gate["failed_checks"] == ["pct_native"]
    assert gate["checks"]["pct_native"]["ok"] is False
    assert "build/source the PCT native runtime" in gate["checks"]["pct_native"]["recommended_action"]
    assert any(
        "pct_runtime_preflight.py" in command
        for command in gate["checks"]["pct_native"]["diagnostic_commands"]
    )
    assert "bash scripts/deploy/setup_server_ros_pct.sh" in gate["diagnostic_commands"]
    assert any("PCT native runtime unavailable" in item for item in gate["blockers"])
    assert any("CPython 3.10" in item for item in gate["blockers"])
    setup_plan = report["host_setup_plan"]
    assert setup_plan["checked"] is True
    assert setup_plan["source"] == "host_preflight_gates"
    assert setup_plan["ok"] is False
    assert setup_plan["blocked_gates"] == ["large_terrain"]
    assert setup_plan["ready_to_run_gates"] == []
    assert setup_plan["failed_check_count"] == 1
    pct_check = setup_plan["failed_checks"][0]
    assert pct_check["check"] == "pct_native"
    assert pct_check["gates"] == ["large_terrain"]
    assert pct_check["diagnostic_commands"] == gate["diagnostic_commands"]
    assert any("PCT native runtime unavailable" in item for item in pct_check["blockers"])
    assert "fix host checks before running blocked DimOS gates" in setup_plan["stop_condition"]
    action = report["next_actions"][0]
    assert action["gate"] == "large_terrain"
    assert action["failed_checks"] == ["pct_native"]
    assert action["diagnostic_commands"] == gate["diagnostic_commands"]
    assert action["recommended_action"] == gate["recommended_action"]


def test_server_sim_host_preflight_accepts_local_non_motion_gate():
    report = server_sim_closure.host_preflight(
        required={"routecheck_preflight"},
        platform_system="Windows",
        machine="AMD64",
        python_tag="py313",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda _name: False,
        path_exists=lambda _path: False,
    )

    assert report["ok"] is True
    assert report["runnable_gates"] == ["routecheck_preflight"]
    assert report["blocked_gates"] == []
    gate = report["gates"]["routecheck_preflight"]
    assert gate["ok"] is True
    assert gate["failed_checks"] == []
    assert gate["diagnostic_commands"] == []
    assert gate["checks"]["local_non_motion"]["ok"] is True
    assert "routecheck_preflight_gate.py" in gate["command"]
    assert report["host_setup_plan"]["checked"] is True
    assert report["host_setup_plan"]["source"] == "host_preflight_gates"
    assert report["host_setup_plan"]["ok"] is True
    assert report["host_setup_plan"]["failed_check_count"] == 0
    assert report["host_setup_plan"]["failed_checks"] == []
    assert report["host_setup_plan"]["stop_condition"] == (
        "host can run the selected DimOS gates"
    )


def test_server_sim_host_preflight_blocks_dynamic_obstacle_on_windows_numpy_runtime():
    report = server_sim_closure.host_preflight(
        required={"dynamic_obstacle_local_planner"},
        platform_system="Windows",
        machine="AMD64",
        python_tag="py313",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda name: name == "numpy",
        path_exists=lambda _path: False,
    )

    assert report["ok"] is False
    assert report["runnable_gates"] == []
    assert report["blocked_gates"] == ["dynamic_obstacle_local_planner"]
    gate = report["gates"]["dynamic_obstacle_local_planner"]
    assert gate["checks"]["local_non_motion"]["ok"] is True
    assert gate["checks"]["local_numeric_nav"]["ok"] is False
    assert gate["failed_checks"] == ["local_numeric_nav"]
    assert any("platform.system" in command.lower() or "platform.system" in command for command in gate["diagnostic_commands"])
    assert report["next_actions"][0]["failed_checks"] == ["local_numeric_nav"]
    setup_plan = report["host_setup_plan"]
    assert setup_plan["failed_check_count"] == 1
    assert setup_plan["failed_checks"][0]["check"] == "local_numeric_nav"
    assert setup_plan["failed_checks"][0]["gates"] == ["dynamic_obstacle_local_planner"]
    assert any("Windows/MINGW NumPy" in blocker for blocker in gate["blockers"])


def test_server_sim_host_preflight_accepts_dynamic_obstacle_on_linux_numpy_runtime():
    report = server_sim_closure.host_preflight(
        required={"dynamic_obstacle_local_planner"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda name: name == "numpy",
        path_exists=lambda _path: False,
    )

    assert report["ok"] is True
    assert report["runnable_gates"] == ["dynamic_obstacle_local_planner"]
    gate = report["gates"]["dynamic_obstacle_local_planner"]
    assert gate["checks"]["local_numeric_nav"]["ok"] is True


def test_server_sim_host_preflight_blocks_local_sim_numeric_gates_on_windows():
    report = server_sim_closure.host_preflight(
        required={"policy_nav", "cmu_unity_sim"},
        platform_system="Windows",
        machine="AMD64",
        python_tag="py313",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda name: name == "numpy",
        path_exists=lambda _path: False,
    )

    assert report["ok"] is False
    assert report["runnable_gates"] == []
    assert report["blocked_gates"] == ["policy_nav", "cmu_unity_sim"]
    setup_plan = report["host_setup_plan"]
    assert setup_plan["failed_check_count"] == 1
    assert setup_plan["failed_checks"][0]["check"] == "local_numeric_nav"
    assert setup_plan["failed_checks"][0]["gates"] == ["policy_nav", "cmu_unity_sim"]
    assert len(setup_plan["failed_checks"][0]["diagnostic_commands"]) == 1
    for name in ("policy_nav", "cmu_unity_sim"):
        gate = report["gates"][name]
        assert gate["checks"]["local_non_motion"]["ok"] is True
        assert gate["checks"]["local_numeric_nav"]["ok"] is False
        assert any("Windows/MINGW NumPy local simulation runtime" in item for item in gate["blockers"])


def test_server_sim_host_preflight_accepts_local_sim_numeric_gates_on_linux():
    report = server_sim_closure.host_preflight(
        required={"policy_nav", "cmu_unity_sim"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda name: name == "numpy",
        path_exists=lambda _path: False,
    )

    assert report["ok"] is True
    assert report["runnable_gates"] == ["policy_nav", "cmu_unity_sim"]


def _native_pct_preflight_path_exists(path) -> bool:
    normalized = str(path).replace("\\", "/")
    return normalized in {
        "/opt/ros/humble/setup.bash",
        str(server_sim_closure.ROOT / server_sim_closure.MID360_PATTERN_REL).replace(
            "\\",
            "/",
        ),
        str(server_sim_closure.ROOT / server_sim_closure.MUJOCO_WORLD_ASSET_REL).replace(
            "\\",
            "/",
        ),
    }


def test_server_sim_host_preflight_requires_isolated_ros_domain_and_hardware_audit():
    common = dict(
        required={"native_pct_mujoco"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "30", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=_native_pct_preflight_path_exists,
        pct_runtime_report={
            "ok": True,
            "host_platform_supported": True,
            "python_abi_matches_known_good": True,
            "lib_dir": "/tmp/pct-native",
            "error": "",
        },
        ros2_package_executables=lambda _package: [
            "local_planner localPlanner",
            "local_planner pathFollower",
        ],
    )

    unsafe = server_sim_closure.host_preflight(
        **common,
        hardware_subscribers=lambda: ["/s100p_driver"],
    )
    assert unsafe["ok"] is False
    assert unsafe["blocked_gates"] == ["native_pct_mujoco"]
    assert any(
        "hardware command subscribers present" in item
        for item in unsafe["gates"]["native_pct_mujoco"]["blockers"]
    )

    safe = server_sim_closure.host_preflight(
        **common,
        hardware_subscribers=lambda: [],
    )
    assert safe["ok"] is True
    assert safe["runnable_gates"] == ["native_pct_mujoco"]
    assert safe["gates"]["native_pct_mujoco"]["checks"]["hardware_subscribers"]["ok"] is True
    assert safe["gates"]["native_pct_mujoco"]["checks"]["ros2_local_planner"]["ok"] is True


def test_server_sim_host_preflight_blocks_missing_ros2_local_planner_runtime():
    report = server_sim_closure.host_preflight(
        required={"native_pct_mujoco"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=_native_pct_preflight_path_exists,
        pct_runtime_report={
            "ok": True,
            "host_platform_supported": True,
            "python_abi_matches_known_good": True,
            "lib_dir": "/tmp/pct-native",
            "error": "",
        },
        hardware_subscribers=lambda: [],
        ros2_package_executables=lambda _package: [],
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["native_pct_mujoco"]
    gate = report["gates"]["native_pct_mujoco"]
    assert gate["failed_checks"] == ["ros2_local_planner"]
    check = gate["checks"]["ros2_local_planner"]
    assert check["ok"] is False
    assert check["evidence"]["missing_executables"] == ["localPlanner", "pathFollower"]
    setup_plan = report["host_setup_plan"]
    assert setup_plan["failed_check_count"] == 1
    assert setup_plan["failed_checks"][0]["check"] == "ros2_local_planner"
    assert any(
        "ros2 pkg executables local_planner" in command
        for command in setup_plan["failed_checks"][0]["diagnostic_commands"]
    )


def test_server_sim_host_preflight_blocks_missing_gazebo_pct_adapter_runtime():
    def package_executables(package: str) -> list[str]:
        if package == "local_planner":
            return ["local_planner localPlanner", "local_planner pathFollower"]
        if package == "pct_adapters":
            return []
        return []

    def path_exists(path: str | Path) -> bool:
        normalized = str(path).replace("\\", "/")
        return normalized == "/opt/ros/humble/setup.bash" or any(
            normalized.endswith(rel) for rel in server_sim_closure.GAZEBO_NAV_SOURCE_RELS
        )

    report = server_sim_closure.host_preflight(
        required={"gazebo_runtime"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75"},
        executable_exists=lambda name: name in {"ros2", "gz"},
        module_available=lambda _name: False,
        path_exists=path_exists,
        pct_runtime_report={"ok": True},
        hardware_subscribers=lambda: [],
        ros2_package_executables=package_executables,
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["gazebo_runtime"]
    gate = report["gates"]["gazebo_runtime"]
    assert gate["checks"]["ros2_local_planner"]["ok"] is True
    assert gate["failed_checks"] == ["ros2_pct_adapters"]
    check = gate["checks"]["ros2_pct_adapters"]
    assert check["ok"] is False
    assert check["evidence"]["missing_executables"] == ["pct_path_adapter"]
    assert any(
        "ros2 pkg executables pct_adapters" in command
        for command in check["diagnostic_commands"]
    )


def test_server_sim_host_preflight_blocks_missing_gazebo_navigation_source():
    def package_executables(package: str) -> list[str]:
        if package == "local_planner":
            return ["local_planner localPlanner", "local_planner pathFollower"]
        if package == "pct_adapters":
            return ["pct_adapters pct_path_adapter"]
        return []

    missing_rel = "sim/planning/gazebo_line_global_planner.py"

    def path_exists(path: str | Path) -> bool:
        normalized = str(path).replace("\\", "/")
        if normalized == "/opt/ros/humble/setup.bash":
            return True
        if normalized.endswith(missing_rel):
            return False
        return any(
            normalized.endswith(rel) for rel in server_sim_closure.GAZEBO_NAV_SOURCE_RELS
        )

    report = server_sim_closure.host_preflight(
        required={"gazebo_runtime"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75"},
        executable_exists=lambda name: name in {"ros2", "gz"},
        module_available=lambda _name: False,
        path_exists=path_exists,
        pct_runtime_report={"ok": True},
        hardware_subscribers=lambda: [],
        ros2_package_executables=package_executables,
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["gazebo_runtime"]
    gate = report["gates"]["gazebo_runtime"]
    assert gate["failed_checks"] == ["gazebo_navigation_sources"]
    check = gate["checks"]["gazebo_navigation_sources"]
    assert check["ok"] is False
    assert check["evidence"]["missing_relative_paths"] == [missing_rel]
    assert any(
        "gazebo_line_global_planner.py" in command
        for command in check["diagnostic_commands"]
    )


def test_server_sim_host_preflight_blocks_missing_ros2_fastlio2_runtime():
    def package_executables(package: str) -> list[str]:
        if package == "fastlio2":
            return []
        return []

    report = server_sim_closure.host_preflight(
        required={"fastlio2_dynamic_inspection"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=_native_pct_preflight_path_exists,
        pct_runtime_report={
            "ok": True,
            "host_platform_supported": True,
            "python_abi_matches_known_good": True,
            "lib_dir": "/tmp/pct-native",
            "error": "",
        },
        hardware_subscribers=lambda: [],
        ros2_package_executables=package_executables,
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["fastlio2_dynamic_inspection"]
    gate = report["gates"]["fastlio2_dynamic_inspection"]
    assert "ros2_local_planner" not in gate["checks"]
    assert gate["failed_checks"] == ["ros2_fastlio2"]
    check = gate["checks"]["ros2_fastlio2"]
    assert check["ok"] is False
    assert check["evidence"]["missing_executables"] == ["lio_node"]
    assert any(
        "ros2 pkg executables fastlio2" in command
        for command in check["diagnostic_commands"]
    )
    setup_plan = report["host_setup_plan"]
    assert setup_plan["failed_check_count"] == 1
    assert setup_plan["failed_checks"][0]["check"] == "ros2_fastlio2"
    assert setup_plan["failed_checks"][0]["gates"] == [
        "fastlio2_dynamic_inspection"
    ]


def test_server_sim_host_preflight_accepts_ros2_fastlio2_runtime():
    def package_executables(package: str) -> list[str]:
        if package == "fastlio2":
            return ["fastlio2 lio_node"]
        return []

    report = server_sim_closure.host_preflight(
        required={"fastlio2_dynamic_inspection"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=_native_pct_preflight_path_exists,
        pct_runtime_report={
            "ok": True,
            "host_platform_supported": True,
            "python_abi_matches_known_good": True,
            "lib_dir": "/tmp/pct-native",
            "error": "",
        },
        hardware_subscribers=lambda: [],
        ros2_package_executables=package_executables,
    )

    assert report["ok"] is True
    assert report["runnable_gates"] == ["fastlio2_dynamic_inspection"]
    gate = report["gates"]["fastlio2_dynamic_inspection"]
    assert "ros2_local_planner" not in gate["checks"]
    assert gate["checks"]["ros2_fastlio2"]["ok"] is True
    assert gate["checks"]["ros2_fastlio2"]["evidence"]["executables"] == [
        "fastlio2 lio_node"
    ]


def test_server_sim_host_preflight_blocks_missing_mid360_pattern_asset():
    world_path = str(
        server_sim_closure.ROOT / server_sim_closure.MUJOCO_WORLD_ASSET_REL
    ).replace("\\", "/")

    report = server_sim_closure.host_preflight(
        required={"native_pct_mujoco"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=lambda path: str(path).replace("\\", "/")
        in {"/opt/ros/humble/setup.bash", world_path},
        pct_runtime_report={
            "ok": True,
            "host_platform_supported": True,
            "python_abi_matches_known_good": True,
            "lib_dir": "/tmp/pct-native",
            "error": "",
        },
        hardware_subscribers=lambda: [],
        ros2_package_executables=lambda _package: [
            "local_planner localPlanner",
            "local_planner pathFollower",
        ],
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["native_pct_mujoco"]
    gate = report["gates"]["native_pct_mujoco"]
    assert gate["failed_checks"] == ["mid360_pattern"]
    check = gate["checks"]["mid360_pattern"]
    assert check["ok"] is False
    assert check["evidence"]["relative_path"] == server_sim_closure.MID360_PATTERN_REL
    assert check["evidence"]["exists"] is False
    assert any(
        "sim/assets/livox/mid360.npy" in command
        for command in check["diagnostic_commands"]
    )


def test_server_sim_host_preflight_blocks_missing_mujoco_world_asset():
    setup_path = "/opt/ros/humble/setup.bash"
    mid360_path = str(
        server_sim_closure.ROOT / server_sim_closure.MID360_PATTERN_REL
    ).replace("\\", "/")

    report = server_sim_closure.host_preflight(
        required={"native_pct_mujoco"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=lambda path: str(path).replace("\\", "/")
        in {setup_path, mid360_path},
        pct_runtime_report={
            "ok": True,
            "host_platform_supported": True,
            "python_abi_matches_known_good": True,
            "lib_dir": "/tmp/pct-native",
            "error": "",
        },
        hardware_subscribers=lambda: [],
        ros2_package_executables=lambda _package: [
            "local_planner localPlanner",
            "local_planner pathFollower",
        ],
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["native_pct_mujoco"]
    gate = report["gates"]["native_pct_mujoco"]
    assert gate["failed_checks"] == ["mujoco_world_asset"]
    check = gate["checks"]["mujoco_world_asset"]
    assert check["ok"] is False
    assert check["evidence"]["relative_path"] == server_sim_closure.MUJOCO_WORLD_ASSET_REL
    assert check["evidence"]["exists"] is False
    assert any(
        "industrial_park_scene.xml" in command
        for command in check["diagnostic_commands"]
    )


def test_server_sim_host_preflight_blocks_missing_localizer_runtime():
    report = server_sim_closure.host_preflight(
        required={"saved_map_relocalize"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=_native_pct_preflight_path_exists,
        pct_runtime_report={"ok": True},
        hardware_subscribers=lambda: [],
        ros2_package_executables=lambda _package: [],
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["saved_map_relocalize"]
    gate = report["gates"]["saved_map_relocalize"]
    assert gate["failed_checks"] == ["localizer_runtime"]
    check = gate["checks"]["localizer_runtime"]
    assert check["ok"] is False
    assert check["evidence"]["missing_executables"] == ["localizer_node"]
    assert any(
        "ros2 pkg executables localizer" in command
        for command in check["diagnostic_commands"]
    )


def test_server_sim_host_preflight_accepts_localizer_runtime():
    report = server_sim_closure.host_preflight(
        required={"saved_map_relocalize"},
        platform_system="Linux",
        machine="x86_64",
        python_tag="py310",
        env={"ROS_DISTRO": "humble", "ROS_DOMAIN_ID": "75", "MUJOCO_GL": "egl"},
        executable_exists=lambda name: name in {"ros2"},
        module_available=lambda name: name == "mujoco",
        path_exists=_native_pct_preflight_path_exists,
        pct_runtime_report={"ok": True},
        hardware_subscribers=lambda: [],
        ros2_package_executables=lambda package: (
            ["localizer localizer_node"] if package == "localizer" else []
        ),
    )

    assert report["ok"] is True
    assert report["runnable_gates"] == ["saved_map_relocalize"]
    check = report["gates"]["saved_map_relocalize"]["checks"]["localizer_runtime"]
    assert check["ok"] is True
    assert check["evidence"]["executables"] == ["localizer localizer_node"]


def test_ros2_package_executables_sources_ros_and_workspace_install(monkeypatch):
    calls = []

    def fake_exists(path):
        normalized = str(path).replace("\\", "/")
        return normalized in {
            "/opt/ros/humble/setup.bash",
            str(server_sim_closure.ROOT / "install" / "setup.bash").replace(
                "\\",
                "/",
            ),
        }

    def fake_which(name, path=None):
        if name == "bash":
            return "/bin/bash"
        if name == "ros2":
            return "/opt/ros/humble/bin/ros2"
        return None

    def fake_run(cmd, **kwargs):
        calls.append((cmd, kwargs))
        return server_sim_closure.subprocess.CompletedProcess(
            cmd,
            0,
            stdout="local_planner localPlanner\nlocal_planner pathFollower\n",
            stderr="",
        )

    monkeypatch.setattr(server_sim_closure.os, "name", "posix")
    monkeypatch.setattr(server_sim_closure.Path, "exists", fake_exists)
    monkeypatch.setattr(server_sim_closure.shutil, "which", fake_which)
    monkeypatch.setattr(server_sim_closure.subprocess, "run", fake_run)

    executables = server_sim_closure._ros2_package_executables(
        "local_planner",
        {"ROS_DISTRO": "humble", "PYTHONPATH": "src:."},
    )

    assert executables == ["local_planner localPlanner", "local_planner pathFollower"]
    assert calls
    command = calls[0][0]
    assert command[:2] == ["bash", "-lc"]
    shell = command[2]
    assert "source /opt/ros/humble/setup.bash" in shell
    assert "source" in shell
    assert "install/setup.bash" in shell.replace("\\", "/")
    assert "ros2 pkg executables local_planner" in shell


def test_server_sim_host_preflight_setup_plan_orders_native_runtime_failures():
    report = server_sim_closure.host_preflight(
        required={"native_pct_mujoco"},
        platform_system="Windows",
        machine="AMD64",
        python_tag="py313",
        env={},
        executable_exists=lambda _name: False,
        module_available=lambda _name: False,
        path_exists=lambda _path: False,
        pct_runtime_report={
            "ok": False,
            "host_platform_supported": False,
            "python_abi_matches_known_good": False,
            "error": "No runnable PCT native modules for arch=x86_64 python=py313",
        },
        hardware_subscribers=lambda: None,
        ros2_package_executables=lambda _package: None,
    )

    assert report["ok"] is False
    assert report["blocked_gates"] == ["native_pct_mujoco"]
    failed_checks = [
        item["check"] for item in report["host_setup_plan"]["failed_checks"]
    ]
    assert failed_checks == [
        "pct_native",
        "ros2_humble",
        "mujoco_headless",
        "mid360_pattern",
        "mujoco_world_asset",
        "isolated_ros_domain",
        "hardware_subscribers",
        "ros2_local_planner",
    ]
    for item in report["host_setup_plan"]["failed_checks"]:
        assert item["gates"] == ["native_pct_mujoco"]
    commands = {
        item["check"]: item["diagnostic_commands"]
        for item in report["host_setup_plan"]["failed_checks"]
    }
    assert any("pct_runtime_preflight.py" in command for command in commands["pct_native"])
    assert any("source /opt/ros/humble/setup.bash" in command for command in commands["ros2_humble"])
    assert any("import mujoco" in command for command in commands["mujoco_headless"])
    assert any("mid360.npy" in command for command in commands["mid360_pattern"])
    assert any(
        "industrial_park_scene.xml" in command
        for command in commands["mujoco_world_asset"]
    )
    assert any("ROS_DOMAIN_ID" in command for command in commands["isolated_ros_domain"])
    assert any("ros2 topic info" in command for command in commands["hardware_subscribers"])
    assert any("local_planner" in command for command in commands["ros2_local_planner"])


def test_server_sim_host_preflight_cli_writes_read_only_report(tmp_path: Path, monkeypatch):
    out = tmp_path / "host_preflight.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--host-preflight",
            "--required",
            "routecheck_preflight",
            "--json-out",
            str(out),
            "--strict",
        ],
    )

    assert server_sim_closure.main() == 0
    report = json.loads(out.read_text(encoding="utf-8"))
    assert report["execution_mode"] == "host_preflight_only"
    assert report["run_missing"] is False
    assert report["gate_runs"] == []
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    assert report["runnable_gates"] == ["routecheck_preflight"]
    assert report["host_setup_plan"]["checked"] is True
    assert report["host_setup_plan"]["source"] == "host_preflight_gates"
    assert report["host_setup_plan"]["ok"] is True


def test_server_sim_host_preflight_cli_json_out_dash_writes_stdout_only(
    tmp_path: Path,
    monkeypatch,
    capsys,
):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--host-preflight",
            "--required",
            "routecheck_preflight",
            "--json-out",
            "-",
            "--strict",
        ],
    )

    assert server_sim_closure.main() == 0
    assert not (tmp_path / "-").exists()
    report = json.loads(capsys.readouterr().out)
    assert report["execution_mode"] == "host_preflight_only"
    assert report["run_missing"] is False
    assert report["gate_runs"] == []
    assert report["runnable_gates"] == ["routecheck_preflight"]
    assert report["host_setup_plan"]["checked"] is True
    assert report["host_setup_plan"]["source"] == "host_preflight_gates"
    assert report["host_setup_plan"]["ok"] is True


def test_server_sim_host_preflight_rejects_run_missing_mix(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--host-preflight",
            "--run-missing",
            "--required",
            "routecheck_preflight",
        ],
    )

    with pytest.raises(SystemExit) as exc:
        server_sim_closure.main()
    assert "--host-preflight cannot be combined with --run-missing" in str(exc.value)


def test_server_sim_closure_rejects_skip_host_blocked_without_run_missing(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--skip-host-blocked",
            "--required",
            "routecheck_preflight",
        ],
    )

    with pytest.raises(SystemExit) as exc:
        server_sim_closure.main()
    assert "--skip-host-blocked requires --run-missing" in str(exc.value)


def test_server_sim_closure_rejects_navigation_replay_report_and_topic_jsonl_mix(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--navigation-replay-deviation-report",
            str(tmp_path / "report.json"),
            "--navigation-replay-deviation-topic-jsonl",
            str(tmp_path / "recorded_topics.jsonl"),
        ],
    )

    with pytest.raises(SystemExit) as exc:
        server_sim_closure.main()
    assert (
        "--navigation-replay-deviation-report cannot be combined with "
        "--navigation-replay-deviation-topic-jsonl"
    ) in str(exc.value)


def test_server_sim_closure_rejects_navigation_replay_topic_jsonl_with_host_preflight(
    tmp_path: Path,
    monkeypatch,
):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--host-preflight",
            "--navigation-replay-deviation-topic-jsonl",
            str(tmp_path / "recorded_topics.jsonl"),
        ],
    )

    with pytest.raises(SystemExit) as exc:
        server_sim_closure.main()
    assert (
        "--navigation-replay-deviation-topic-jsonl cannot be combined with "
        "--host-preflight"
    ) in str(exc.value)


def test_server_sim_closure_cli_passes_skip_host_blocked_to_run_missing(
    monkeypatch,
    capsys,
):
    from runtime.algorithm_gates import DIMOS_BENCHMARK_REQUIRED_GATES

    captured: dict[str, object] = {}

    def fake_run_missing_required_gates(**kwargs: object) -> dict[str, object]:
        captured.update(kwargs)
        return {
            "ok": False,
            "execution_mode": "run_missing",
            "run_missing": True,
            "skip_host_blocked": kwargs.get("skip_host_blocked"),
        }

    monkeypatch.setattr(
        server_sim_closure,
        "run_missing_required_gates",
        fake_run_missing_required_gates,
    )
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "server_sim_closure.py",
            "--preset",
            "dimos_benchmark",
            "--required-only",
            "--run-missing",
            "--skip-host-blocked",
            "--json-out",
            "-",
        ],
    )

    assert server_sim_closure.main() == 0
    report = json.loads(capsys.readouterr().out)
    assert report["skip_host_blocked"] is True
    assert captured["required"] == set(DIMOS_BENCHMARK_REQUIRED_GATES)
    assert captured["include_optional"] is False
    assert captured["skip_host_blocked"] is True


def test_server_sim_closure_fastlio2_dynamic_inspection_command_uses_pct_contract():
    spec = next(
        gate for gate in server_sim_closure.GATES if gate.name == "fastlio2_dynamic_inspection"
    )
    moving_sweep = next(
        gate for gate in server_sim_closure.GATES if gate.name == "moving_obstacle_sweep"
    )
    fastlio_gate = Path("sim/scripts/mujoco/live_gate.py").read_text(
        encoding="utf-8"
    )
    fastlio_report = Path("sim/scripts/mujoco_live/report.py").read_text(
        encoding="utf-8"
    )
    fastlio_launcher = Path("sim/scripts/mujoco/launch_fastlio2_live.sh").read_text(
        encoding="utf-8"
    )

    assert (
        "artifacts/server_sim_closure/mujoco_fastlio2_live*/inspection*/report.json"
        in spec.default_patterns
    )
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_PLANNER:-pct}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT="
        "${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-timed_pointcloud2}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_TOMOGRAM:-"
        "artifacts/server_sim_closure/large_terrain/tomogram.pickle}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_WORLD="
        "${LINGTU_MUJOCO_LIVE_WORLD:-"
        "artifacts/server_sim_closure/large_terrain/large_terrain_scene.xml}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_INSPECTION_GOALS="
        "${LINGTU_MUJOCO_LIVE_INSPECTION_GOALS:-0.5,0.05;1.0,0.1;1.5,0.15}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM="
        "${LINGTU_MUJOCO_LIVE_BUILD_TOMOGRAM:-1}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER="
        "${LINGTU_MUJOCO_LIVE_FASTLIO_IESKF_MAX_ITER:-10}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S="
        "${LINGTU_MUJOCO_LIVE_MOVING_OBSTACLE_START_S:-2}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT="
        "${LINGTU_MUJOCO_LIVE_CMD_VEL_MUX_SOURCE_TIMEOUT:-5.0}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT="
        "${LINGTU_MUJOCO_LIVE_CMD_VEL_ANGULAR_LIMIT:-0.25}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z="
        "${LINGTU_MUJOCO_LIVE_NAV_MAX_ANGULAR_Z:-0.20}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES="
        "${LINGTU_MUJOCO_LIVE_RUNTIME_FAULT_CONFIRM_SAMPLES:-6}"
    ) in spec.command
    assert (
        "LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M="
        "${LINGTU_MUJOCO_LIVE_RUNTIME_MOTION_FAULT_MIN_SIM_M:-1.0}"
    ) in spec.command
    assert "--run-matrix" in moving_sweep.command
    assert "--report-glob" not in moving_sweep.command
    assert (
        "LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT="
        "${LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-timed_pointcloud2}"
    ) in moving_sweep.command
    assert "--fastlio-lidar-input" in fastlio_gate
    assert 'choices=["livox_custom_msg", "timed_pointcloud2"]' in fastlio_gate
    assert '"early_success": early_success' in fastlio_report
    assert "_inspection_gate_evidence_complete(" in fastlio_report
    assert "--runtime-motion-fault-min-sim-m" in fastlio_gate
    assert "LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT=livox_custom_msg" in fastlio_launcher
    assert '"--fastlio-lidar-input"' in fastlio_launcher
    assert "LINGTU_MUJOCO_LIVE_FASTLIO_LIDAR_INPUT:-livox_custom_msg" in fastlio_launcher


def test_server_sim_closure_accepts_fastlio2_dynamic_inspection_report_override(
    tmp_path: Path,
):
    report = tmp_path / "dynamic.json"
    parser = server_sim_closure._build_parser()
    args = parser.parse_args([
        "--fastlio2-dynamic-inspection-report",
        str(report),
    ])

    assert args.fastlio2_dynamic_inspection_report == report


def test_server_sim_closure_rejects_fastlio2_inspection_without_successful_checkpoints(
    tmp_path: Path,
):
    weak = _complete_fastlio2_inspection_report()
    weak["lingtu_inspection"]["verified"] = False
    weak["lingtu_inspection"]["successful_navigation_goal_count"] = 1
    weak["lingtu_inspection"]["global_path_points_max"] = 0
    report = _write_json(tmp_path / "mujoco_inspection_weak.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"fastlio2_live": report},
        required={"fastlio2_live"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "lingtu_inspection.verified is not true" in gaps
    assert "lingtu_inspection.successful_navigation_goal_count below required" in gaps
    assert "lingtu_inspection.global_path_points_max below required" in gaps


def test_server_sim_closure_rejects_mujoco_tare_failed_moving_obstacle_evidence(
    tmp_path: Path,
):
    report_payload = _complete_fastlio2_tare_report()
    report_payload["moving_obstacles"] = {
        "enabled": True,
        "mode": "robot_crossing",
        "ok": False,
        "published_update_count": 0,
        "published_point_count_max": 0,
        "trail_clearance": {
            "checked": True,
            "collision": True,
            "min_clearance_minus_robot_radius_m": -0.02,
        },
    }
    report = _write_json(tmp_path / "mujoco_tare_moving_obstacle_failed.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "moving_obstacles.ok is not true" in gaps
    assert "moving_obstacles.published_update_count missing" in gaps
    assert "moving_obstacles.published_point_count_max missing" in gaps
    assert "moving_obstacles.trail_clearance.collision is true" in gaps
    assert "moving_obstacles.trail_clearance margin is negative" in gaps


def test_server_sim_closure_requires_declared_fastlio2_live_video_evidence(tmp_path: Path):
    report_payload = _complete_fastlio2_tare_report()
    report_payload["video_path"] = str(tmp_path / "tare_moving_obstacle.mp4")
    report_payload["video_frame_count"] = 0
    report_payload["video_sample_count"] = 0
    report = _write_json(tmp_path / "mujoco_tare_video_missing.json", report_payload)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "fastlio2_live video_frame_count missing" in gaps
    assert "fastlio2_live video_sample_count missing" in gaps


def test_server_sim_closure_rejects_mujoco_tare_without_live_nav_map_growth_contract(tmp_path: Path):
    weak = _complete_fastlio2_tare_report()
    weak["map_growth"].pop("accepted_cumulative_growth_source", None)
    weak["map_growth"].pop("nav_map_cloud_area_samples", None)
    weak["map_growth"]["min_map_area_growth_m2"] = 0.0
    report = _write_json(tmp_path / "mujoco_tare_missing_live_map_contract.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "map_growth.accepted_cumulative_growth_source is not /nav/map_cloud" in gaps
    assert "map_growth.min_map_area_growth_m2 is not positive" in gaps
    assert "map_growth.nav_map_cloud_area_samples missing" in gaps


def test_server_sim_closure_rejects_mujoco_tare_without_exploration_grid_signal(tmp_path: Path):
    weak = _complete_fastlio2_tare_report()
    weak["lingtu_tare"]["exploration_grid_samples"] = 0
    weak["lingtu_tare"]["exploration_grid_first"] = {}
    weak["lingtu_tare"]["exploration_grid_last"] = {}
    weak["lingtu_tare"]["exploration_grid_metadata_last"] = {}
    report = _write_json(tmp_path / "mujoco_tare_missing_exploration_grid.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "lingtu_tare.exploration_grid_samples missing" in gaps
    assert "lingtu_tare.exploration_grid source is not raycast_lidar_exploration_grid" in gaps
    assert "lingtu_tare.exploration_grid_first.free missing" in gaps
    assert "lingtu_tare.exploration_grid_last.unknown missing" in gaps


def test_server_sim_closure_rejects_mujoco_tare_without_exploration_progress(tmp_path: Path):
    weak = _complete_fastlio2_tare_report()
    weak["map_growth"]["exploration_area_samples"] = 0
    weak["map_growth"]["exploration_known_area"] = {"growth_m2": 0.0}
    weak["map_growth"]["exploration_coverage"] = {"growth_ratio": 0.0}
    report = _write_json(tmp_path / "mujoco_tare_no_exploration_progress.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "map_growth.exploration_area_samples missing" in gaps
    assert "exploration known area growth below threshold" in gaps
    assert "exploration coverage growth below threshold" in gaps


def test_server_sim_closure_rejects_mujoco_tare_without_successful_goals(tmp_path: Path):
    weak = _complete_fastlio2_tare_report()
    weak["lingtu_tare"]["verified"] = False
    weak["lingtu_tare"]["successful_navigation_goal_count"] = 0
    weak["lingtu_tare"]["global_path_points_max"] = 0
    report = _write_json(tmp_path / "mujoco_tare_weak.json", weak)

    summary = server_sim_closure.summarize(
        report_overrides={"mujoco_tare_exploration": report},
        required={"mujoco_tare_exploration"},
    )

    assert summary["ok"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "lingtu_tare.verified is not true" in gaps
    assert "lingtu_tare.successful_navigation_goal_count below required" in gaps
    assert "lingtu_tare.global_path_points_max below required" in gaps


def test_server_sim_closure_accepts_native_pct_effect_overlay_report(tmp_path: Path, monkeypatch):
    report = _write_json(
        tmp_path / "artifacts" / "native_pct_effect" / "native_pct_omni_scene_overlay_report.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.2,
            "planner": "pct",
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_used": False,
            "global_planner_source": "source_report/native_pct_tomogram",
            "pct_native_backend_used": True,
            "pct_runtime_ok": True,
            "pct_path_count": 8,
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 3,
            "pct_planner_path_mode": "native_astar_raw_path",
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
                "native_backend_used": True,
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
            "map_artifacts": _same_source_map_artifacts("native_pct_effect"),
            "obstacle_aware": {"enabled": True, "metadata_points": 32},
            "obstacle_clearance": {"checked": True, "collision": False},
            "local_path_samples": [{"frame_id": "body", "point_count": 8}],
            "trajectory_quality": {"ok": True},
            "video": {"lidar_source": _mid360_lidar_source()},
        },
    )
    monkeypatch.setattr(server_sim_closure, "ROOT", tmp_path)

    summary = server_sim_closure.summarize(report_overrides={}, required={"native_pct_mujoco"})

    assert summary["ok"] is True
    assert summary["gates"]["native_pct_mujoco"]["path"] == str(report)
    assert summary["verified"]["native_pct_mujoco"] is True


def test_server_sim_closure_rejects_native_pct_bad_mid360_pattern(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_pct_bad_lidar.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "reached_goal": True,
            "final_distance_m": 0.2,
            "planner": "pct",
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_used": False,
            "global_planner_source": "source_report/native_pct_tomogram",
            "pct_native_backend_used": True,
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
                "native_backend_used": True,
                "tomogram_exists": True,
                "tomogram_sha256": "abc123",
            },
            "obstacle_aware": {"enabled": True, "metadata_points": 32},
            "obstacle_clearance": {"checked": True, "collision": False},
            "local_path_samples": [{"frame_id": "body", "point_count": 8}],
            "trajectory_quality": {"ok": True},
            "lidar_source": {
                "kind": "MuJoCo mj_multiRay fallback grid",
                "forced_pattern": True,
                "pattern_path": "/tmp/not_mid360.npy",
                "pattern_sha256": "bad",
                "samples_per_frame": 24000,
            },
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "lidar_source.pattern_sha256 is not official MID-360 asset" in gaps
    assert "lidar_source.pattern_path is not sim/assets/livox/mid360.npy" in gaps


def test_server_sim_closure_reports_native_pct_ros2_runtime_boundary(tmp_path: Path):
    native = _write_json(
        tmp_path / "native_pct_ros2_blocked.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": False,
            "native_gate_skipped": True,
            "claim_boundary": "ros2_runtime_unavailable",
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "planner": "pct",
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_used": False,
            "global_planner_source": "source_report/native_pct_tomogram",
            "pct_native_backend_used": True,
            "pct_runtime_ok": True,
            "pct_path_count": 8,
            "pct_optimizer_enabled": True,
            "pct_optimizer_attempted": True,
            "pct_optimizer_accepted": False,
            "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
            "pct_optimizer_blocked_sample_count": 12,
            "pct_planner_path_mode": "native_astar_raw_path",
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
                "native_backend_used": True,
                "tomogram_exists": True,
                "tomogram_sha256": "abc123",
                "pct_optimizer_attempted": True,
                "pct_optimizer_accepted": False,
                "pct_optimizer_reject_reason": "optimized_trajectory_hard_obstacle",
                "pct_optimizer_blocked_sample_count": 12,
                "pct_planner_path_mode": "native_astar_raw_path",
            },
            "deliverable_contract": {
                "checks": {"same_source_map_artifact": True},
            },
            "map_artifacts": _same_source_map_artifacts("native_pct_ros2_blocked"),
            "environment": {
                "ros2_executable": "",
                "ros_distro": "",
                "diagnostic_commands": ["ros2 pkg executables local_planner"],
            },
            "blockers": [
                "ROS2 runtime unavailable for native local planner gate: ros2 CLI is unavailable"
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    gate = summary["gates"]["native_pct_mujoco"]
    assert gate["evidence"]["claim_boundary"] == "ros2_runtime_unavailable"
    assert gate["evidence"]["pct_optimizer"]["accepted"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "ROS2 runtime unavailable for native local planner gate" in gaps
    assert "reached_goal is not true" not in gaps
    assert "local_path_samples missing" not in gaps


def test_server_sim_closure_reports_native_pct_ros2_boundary_with_bad_pct_evidence(
    tmp_path: Path,
):
    native = _write_json(
        tmp_path / "native_pct_ros2_bad_pct.json",
        {
            "schema_version": "lingtu.native_pct_mujoco_gate.v1",
            "ok": False,
            "native_gate_skipped": True,
            "claim_boundary": "ros2_runtime_unavailable",
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "planner": "pct",
            "primary_planner": "pct",
            "selected_planner": "pct",
            "fallback_used": False,
            "global_planner_source": "source_report/native_pct_tomogram",
            "pct_native_backend_used": True,
            "pct_runtime_ok": False,
            "pct_path_count": 0,
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
                "native_backend_used": True,
                "tomogram_exists": True,
                "tomogram_sha256": "abc123",
            },
            "deliverable_contract": {
                "checks": {"same_source_map_artifact": True},
            },
            "map_artifacts": _same_source_map_artifacts("native_pct_ros2_bad_pct"),
            "environment": {
                "ros2_executable": "",
                "ros_distro": "",
                "diagnostic_commands": ["ros2 pkg executables local_planner"],
            },
            "blockers": [
                "ROS2 runtime unavailable for native local planner gate: ros2 CLI is unavailable"
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"native_pct_mujoco": native},
        required={"native_pct_mujoco"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["native_pct_mujoco"] is False
    gate = summary["gates"]["native_pct_mujoco"]
    assert gate["evidence"]["claim_boundary"] == "ros2_runtime_unavailable"
    assert gate["evidence"]["pct_optimizer"]["enabled"] is None
    gaps = "\n".join(summary["remaining_gaps"])
    assert "pct_runtime_ok is not true" in gaps
    assert "pct_path_count < 2" in gaps
    assert "pct_optimizer_enabled is not recorded" in gaps
    assert "pct_planner_path_mode is not supported" in gaps
    assert "ROS2 runtime unavailable for native local planner gate" in gaps
    assert "reached_goal is not true" not in gaps


def test_server_sim_closure_rejects_policy_nav_direct_fallback(tmp_path: Path):
    policy = _write_json(
        tmp_path / "policy.json",
        {
            "passed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checks": [
                {
                    "mode": "direct_policy",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                },
                {
                    "mode": "full_stack_policy_nav",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "global_planner_backend_status": {
                        "configured_backend": "octoplanner3d",
                        "backend": "octoplanner3d",
                        "degraded": False,
                    },
                    "local_planner_backend_actual": "nanobind",
                    "path_follower_backend_actual": "nav_kernel",
                    "seen": {
                        "direct_fallback": 1,
                        "local_path": 12,
                        "path_follower_cmd": 12,
                        "mux_cmd": 12,
                        "waypoints": 2,
                    },
                },
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"policy_nav": policy},
        required={"policy_nav"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["policy_nav"] is False
    assert any("direct_goal_fallback" in gap for gap in summary["remaining_gaps"])


def test_server_sim_closure_rejects_policy_nav_legacy_local_backends(tmp_path: Path):
    policy = _write_json(
        tmp_path / "policy_legacy_backends.json",
        {
            "passed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checks": [
                {
                    "mode": "direct_policy",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "contacts": {
                        "foot_contact_sample_count": 20,
                        "unique_feet_count": 4,
                        "non_foot_ground_contacts": 0,
                    },
                },
                {
                    "mode": "full_stack_policy_nav",
                    "passed": True,
                    "policy_loaded": True,
                    "policy_path": "/tmp/policy.onnx",
                    "cmd_vel_sent_to_hardware": False,
                    "global_planner_backend_status": {
                        "configured_backend": "astar",
                        "backend": "astar",
                        "degraded": False,
                    },
                    "local_planner_backend_actual": "cmu_py",
                    "path_follower_backend_actual": "pid",
                    "contacts": {
                        "foot_contact_sample_count": 20,
                        "unique_feet_count": 4,
                        "non_foot_ground_contacts": 0,
                    },
                    "seen": {
                        "direct_fallback": 0,
                        "local_path": 12,
                        "path_follower_cmd": 12,
                        "mux_cmd": 12,
                        "waypoints": 2,
                    },
                },
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"policy_nav": policy},
        required={"policy_nav"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["policy_nav"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert (
        "full_stack_policy_nav global_planner_backend_status.configured_backend is not octoplanner3d"
        in gaps
    )
    assert "full_stack_policy_nav local_planner_backend_actual is not nanobind" in gaps
    assert "full_stack_policy_nav path_follower_backend_actual is not nav_kernel" in gaps


def test_server_sim_closure_rejects_policy_nav_without_policy_or_chain_topics(tmp_path: Path):
    policy = _write_json(
        tmp_path / "policy_missing_chain.json",
        {
            "passed": True,
            "simulation_only": True,
            "real_robot_motion": False,
            "cmd_vel_sent_to_hardware": False,
            "checks": [
                {
                    "mode": "direct_policy",
                    "passed": True,
                    "policy_loaded": False,
                    "cmd_vel_sent_to_hardware": False,
                },
                {
                    "mode": "full_stack_policy_nav",
                    "passed": True,
                    "policy_loaded": False,
                    "cmd_vel_sent_to_hardware": False,
                    "seen": {
                        "direct_fallback": 0,
                        "local_path": 0,
                        "path_follower_cmd": 0,
                        "mux_cmd": 0,
                        "waypoints": 0,
                    },
                },
            ],
        },
    )

    summary = server_sim_closure.summarize(
        report_overrides={"policy_nav": policy},
        required={"policy_nav"},
    )

    assert summary["ok"] is False
    assert summary["verified"]["policy_nav"] is False
    gaps = "\n".join(summary["remaining_gaps"])
    assert "no ONNX policy loaded" in gaps
    assert "full_stack_policy_nav missing local_path" in gaps
    assert "full_stack_policy_nav missing path_follower_cmd" in gaps
    assert "full_stack_policy_nav missing mux_cmd" in gaps
    assert "full_stack_policy_nav missing waypoints" in gaps
