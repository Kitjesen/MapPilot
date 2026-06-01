from __future__ import annotations

import pytest

pytestmark = [pytest.mark.sim]

import argparse
import json
import pickle
import sys
from pathlib import Path

import numpy as np

from exploration.native_factories import TARE_REMAPS
from sim.engine.bridge.cmu_unity_lingtu_adapter import required_relay_contract
from sim.scripts import cmu_unity_sim_gate
from sim.scripts import cmu_unity_runtime_gate
from sim.scripts import cmu_unity_tomogram_capture


def _touch(path: Path, text: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _make_fake_cmu_workspace(tmp_path: Path) -> Path:
    workspace = tmp_path / "cmu"
    topic_text = "\n".join(cmu_unity_sim_gate.CMU_TOPIC_CONTRACT)
    for relative in cmu_unity_sim_gate.REQUIRED_CMU_PATHS.values():
        _touch(workspace / relative, topic_text)
    for relative in cmu_unity_sim_gate.UNITY_ENVIRONMENT_PATHS.values():
        path = workspace / relative
        if relative.endswith("Model_Data"):
            path.mkdir(parents=True, exist_ok=True)
        else:
            _touch(path, "binary-placeholder")
    for relative in cmu_unity_sim_gate.BUILD_PATHS.values():
        _touch(workspace / relative, "source install/setup.bash")
    return workspace


def _args(workspace: Path, **overrides: object) -> argparse.Namespace:
    values = {
        "cmu_workspace": str(workspace),
        "require_git": False,
        "require_humble_branch": False,
        "require_unity_model": True,
        "require_build": True,
        "require_ros": False,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def test_cmu_unity_sim_gate_accepts_complete_preflight_workspace(tmp_path: Path):
    workspace = _make_fake_cmu_workspace(tmp_path)

    report = cmu_unity_sim_gate.build_report(_args(workspace))

    assert report["schema_version"] == "lingtu.cmu_unity_sim_gate.v1"
    assert report["ok"] is True
    assert report["simulation_only"] is True
    assert report["real_robot_motion"] is False
    assert report["cmd_vel_sent_to_hardware"] is False
    checks = {item["name"]: item["ok"] for item in report["checks"]}
    assert checks["cmu_required_source_paths"] is True
    assert checks["cmu_unity_environment_assets"] is True
    assert checks["cmu_colcon_build_output"] is True
    assert checks["cmu_topic_contract"] is True
    assert report["cmu_workspace"]["topic_contract"]["/path"] is True
    assert "/global_path" not in report["cmu_workspace"]["topic_contract"]
    assert report["cmu_workspace"]["optional_topic_contract"]["/global_path"] is False
    assert report["cmu_workspace"]["optional_topic_contract"]["/local_path"] is False
    assert checks["lingtu_tare_remap_contract"] is True
    assert checks["lingtu_tare_explore_profile"] is True
    assert checks["lingtu_cmu_tare_profile"] is True
    assert checks["lingtu_cmu_adapter_exists"] is True
    assert checks["lingtu_cmu_adapter_launch_exists"] is True
    assert checks["lingtu_cmu_stack_script_exists"] is True
    assert checks["lingtu_cmu_adapter_relay_contract"] is True
    assert checks["lingtu_cmu_adapter_safety_contract"] is True
    assert checks["lingtu_cmu_stack_simulation_contract"] is True
    assert (
        report["lingtu_contract"]["remaps"]["/state_estimation_at_scan"]
        == "/nav/odometry"
    )
    assert report["lingtu_contract"]["remaps"] == TARE_REMAPS
    assert (
        report["lingtu_contract"]["remaps"]["/runtime_breakdown"]
        == "/exploration/runtime_breakdown"
    )
    assert (
        report["lingtu_contract"]["adapter_required_relays"][
            "/nav/cmd_vel->/cmd_vel"
        ]
        == "geometry_msgs/msg/TwistStamped"
    )
    assert report["lingtu_contract"]["adapter_required_relays"] == required_relay_contract(
        relay_cmd_vel_to_sim=True
    )
    assert (
        report["lingtu_contract"]["remaps"]["/global_path"]
        == "/exploration/global_path"
    )
    assert "/global_path->/exploration/global_path" not in report["lingtu_contract"]["adapter_required_relays"]
    assert "/local_path->/exploration/local_path" not in report["lingtu_contract"]["adapter_required_relays"]


def test_cmu_unity_lingtu_stack_script_is_simulation_only():
    script = cmu_unity_sim_gate.ROOT / "sim/scripts/cmu_unity_lingtu_stack.py"
    text = script.read_text(encoding="utf-8")

    assert "robot=\"sim_ros2\"" in text
    assert "slam_profile=\"none\"" in text
    assert "enable_frontier=args.enable_frontier" in text
    assert "--disable-direct-goal-fallback" in text
    assert "LINGTU_CMU_ALLOW_STATIC_TOMOGRAM" in text
    assert "Refusing CMU Unity PCT without a same-source tomogram" in text
    assert "Refusing CMU Unity PCT with the legacy building2_9 tomogram" in text
    assert "allow_direct_goal_fallback=not args.disable_direct_goal_fallback" in text
    assert "LINGTU_CMU_WAYPOINT_THRESHOLD" in text
    assert "LINGTU_CMU_SAFE_GOAL_TOLERANCE" in text
    assert 'default=_env_float("LINGTU_CMU_WAYPOINT_THRESHOLD", 0.45)' in text
    assert 'default=_env_float("LINGTU_CMU_FINAL_WAYPOINT_THRESHOLD", 0.35)' in text
    assert "waypoint_threshold=args.waypoint_threshold" in text
    assert "safe_goal_tolerance=args.safe_goal_tolerance" in text
    assert "LINGTU_CMU_ACCEPT_PARTIAL_GOAL_PROGRESS" in text
    assert "accept_partial_goal_progress=" in text
    assert "LINGTU_CMU_PREFER_TARE_PATH_STRATEGY" in text
    assert 'default=not _env_bool("LINGTU_CMU_PREFER_TARE_PATH_STRATEGY", True)' in text
    assert "LINGTU_CMU_TARE_PATH_START_TOLERANCE" in text
    assert "prefer_path_strategy=" in text
    assert "path_start_tolerance_m=" in text
    assert "path_goal_min_distance_m=" in text
    assert "external_strategy_start_tolerance_m=" in text
    assert "local_planner_allow_direct_track_fallback=True" in text
    assert "local_planner_ignore_near_field_stop=True" in text
    assert "local_planner_direct_track_fallback_min_distance_m=0.3" in text
    assert "plan_safety_policy=args.plan_safety_policy" in text
    assert "stuck_timeout=args.stuck_timeout" in text
    assert "path_follower_goal_tolerance=args.path_follower_goal_tolerance" in text
    assert (
        "direct_goal_fallback_on_planner_failure=not args.disable_direct_goal_fallback"
        in text
    )
    assert "enable_ros2_bridge=True" in text
    assert "enable_ros2_path_bridge=True" in text
    assert "exploration_backend=\"none\" if args.enable_frontier else \"tare_external\"" in text
    assert "manage_external_services=False" in text
    assert "run_startup_checks=False" in text
    assert "latch_stop_signal=False" in text
    assert "ROS_DOMAIN_ID" in text
    assert "thunder" not in text.lower()


def test_cmu_unity_runtime_wrapper_can_open_cmu_operator_rviz():
    script = cmu_unity_sim_gate.ROOT / "sim/scripts/launch_cmu_unity_lingtu_runtime.sh"
    text = script.read_text(encoding="utf-8")

    assert "start [--gate] [--rviz]" in text
    assert "launch_operator_rviz" in text
    assert "[c]mu_unity_runtime_gate.py" in text
    assert "rviz2 -d" in text
    assert "cmu_unity_lingtu_runtime.rviz" in text
    assert "vehicle_simulator.rviz" in text
    assert "LINGTU_CMU_RVIZ_CONFIG" in text
    assert "LINGTU_CMU_DISABLE_NATIVE_NAV" in text
    assert "stop_cmu_native_navigation" in text
    assert "[l]ocalPlanner" in text
    assert "[p]athFollower" in text
    assert "LINGTU_CMU_TOMOGRAM" in text
    assert "LINGTU_CMU_AUTO_TOMOGRAM" in text
    assert "LINGTU_CMU_TOMOGRAM_TOPICS" in text
    assert "LINGTU_CMU_PLANNER" in text
    assert "LINGTU_CMU_SAFE_GOAL_TOLERANCE" in text
    assert "LINGTU_CMU_PLAN_SAFETY_POLICY" in text
    assert "LINGTU_CMU_LOCAL_SCAN_ENABLE" in text
    assert "LINGTU_CMU_TARE_REGISTERED_SCAN_TOPIC" in text
    assert "LINGTU_CMU_NAV_CLOUD_Z_MIN" in text
    assert "--nav-cloud-z-min" in text
    assert "export LINGTU_CMU_PLAN_SAFETY_POLICY=reject" in text
    assert "LINGTU_CMU_ENABLE_FRONTIER" in text
    assert "LINGTU_CMU_START_CMU_TARE" in text
    assert "LINGTU_CMU_TARE_AUTOSTART" in text
    assert "LINGTU_CMU_STUCK_TIMEOUT" in text
    assert 'export LINGTU_CMU_TARE_AUTOSTART=0' in text
    assert "prepare_tare_scenario" in text
    assert "capture_cmu_unity_tomogram" in text
    assert "cmu_unity_tomogram_capture.py" in text
    assert "/nav/map_cloud" in text
    assert "/nav/terrain_map_ext" in text
    assert "LINGTU_CMU_TOMOGRAM_FLAT_DEFAULT_FREE" in text
    assert "LINGTU_CMU_TOMOGRAM_CAPTURE_RETRIES" in text
    assert "capture_attempt=" in text
    assert "cmu_unity_same_source_tomogram.pickle" in text
    assert "PCT planner requires LINGTU_CMU_TOMOGRAM" in text
    assert "indoor_large" in text
    assert 'run_dir="$(cd "$run_dir" && pwd)"' in text
    assert "record_latest_run_dir" in text
    assert "cmu_unity_tare_pct_visible_latest.txt" in text
    assert "kAutoStart : false" in text
    assert "sub_registered_scan_topic_ : ${tare_scan_topic}" in text
    assert "/lingtu/registered_scan_local" in text
    assert 'exploration_planner_config:="$cmu_tare_scenario"' in text
    assert "--gateway-start-exploration-session" in text
    assert "--require-motion-progress" in text
    assert "--required-progress-topic /nav/way_point" in text
    assert "--late-window-sec" in text
    assert "--min-late-odom-delta-m" in text
    assert "--min-late-cmd-vel-samples" in text
    assert "--min-late-path-samples" in text
    assert "--min-late-map-area-delta-m2" in text
    assert "--allow-flat-late-map-after-total-growth" in text
    assert "LINGTU_CMU_GATE_ALLOW_FLAT_LATE_MAP_AFTER_TOTAL_GROWTH" in text
    assert "--require-planner-diagnostics" in text
    assert "--require-no-planner-fallback" in text
    assert "--require-no-primary-replan" in text
    assert "LINGTU_CMU_GATE_REQUIRE_NO_PRIMARY_REPLAN" in text
    assert 'LINGTU_CMU_GATE_REQUIRE_NO_PRIMARY_REPLAN:-0' in text
    assert "--require-planner-path-safety" in text
    assert "--unique-waypoint-topic /nav/way_point" in text
    assert 'LINGTU_CMU_GATE_MIN_UNIQUE_WAYPOINTS:-2' in text
    assert "--require-path-topic /exploration/global_path" not in text
    assert "--require-path-topic /exploration/local_path" not in text
    assert "LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK" in text
    assert 'FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"' in text
    assert "--disable-direct-goal-fallback" in text
    assert "system_simulation.launch" in text


def test_cmu_unity_runtime_gate_starts_gateway_session_after_subscriptions():
    script = cmu_unity_sim_gate.ROOT / "sim/scripts/cmu_unity_runtime_gate.py"
    text = script.read_text(encoding="utf-8")

    assert "def _post_gateway_json" in text
    assert "--gateway-start-exploration-session" in text
    assert '"/api/v1/session/start"' in text
    assert '{"mode": "exploring", "slam_profile": "none"}' in text
    assert "gateway exploration session start failed" in text
    assert text.index("node.create_subscription") < text.index(
        "gateway_session_start = _post_gateway_json"
    )


def test_cmu_unity_runtime_gate_treats_stuck_navigation_as_failure():
    status = {
        "available": True,
        "data": {
            "state": "STUCK",
            "reason_codes": ["mission_stuck", "failure_stuck_after_max_replans"],
            "failure_reason": "stuck after max replans",
        },
    }

    result = cmu_unity_runtime_gate._navigation_failure_from_gateway(status)

    assert result["failed"] is True
    assert result["state"] == "STUCK"


def test_cmu_tomogram_capture_builds_flat_traversability_map(tmp_path: Path):
    pcd = tmp_path / "terrain.pcd"
    tomogram = tmp_path / "tomogram.pickle"
    points = [
        (0.0, 0.0, 0.0),
        (0.2, 0.0, 0.0),
        (0.4, 0.0, 0.0),
        (0.6, 0.0, 0.0),
    ]
    cmu_unity_tomogram_capture._write_ascii_pcd(
        pcd,
        np.asarray(points, dtype="float32"),
    )

    args = argparse.Namespace(
        tomogram_mode="flat_traversability",
        resolution=0.2,
        ground_h=0.0,
        flat_pad_m=0.4,
        flat_free_dilation_cells=1,
        flat_default_free=False,
        flat_floor_z_max=0.35,
        flat_obstacle_z_min=0.35,
        flat_obstacle_inflation_cells=1,
        flat_border_cells=1,
        flat_slice_dh=1.0,
        flat_ceiling_h=2.2,
        flat_free_cost=1.0,
        flat_obstacle_cost=50.0,
    )

    report = cmu_unity_tomogram_capture._build_tomogram(pcd, tomogram, args)

    assert report["exists"] is True
    assert report["mode"] == "flat_traversability"
    assert report["same_source_input"] is True
    assert report["input_pcd"] == str(pcd)
    assert report["input_pcd_sha256"]
    assert report["sha256"]
    assert report["shape"][0:2] == [5, 1]
    assert report["slice_dh"] == 1.0
    assert report["free_cells"] > 0
    loaded = pickle.loads(tomogram.read_bytes())
    assert loaded["grid_info"]["axis_order"] == "row_y_col_x"
    assert loaded["meta"]["input_pcd_sha256"] == report["input_pcd_sha256"]
    assert loaded["data"].shape[2:] == tuple(loaded["grid_info"]["shape_yx"])


def test_cmu_tomogram_capture_flat_default_free_keeps_obstacles(tmp_path: Path):
    pcd = tmp_path / "terrain.pcd"
    tomogram = tmp_path / "tomogram.pickle"
    points = [
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.5, 0.0, 1.0),
    ]
    cmu_unity_tomogram_capture._write_ascii_pcd(
        pcd,
        np.asarray(points, dtype="float32"),
    )

    args = argparse.Namespace(
        tomogram_mode="flat_traversability",
        resolution=0.25,
        ground_h=0.0,
        flat_pad_m=0.5,
        flat_free_dilation_cells=0,
        flat_default_free=True,
        flat_floor_z_max=0.35,
        flat_obstacle_z_min=0.35,
        flat_obstacle_inflation_cells=0,
        flat_border_cells=0,
        flat_slice_dh=1.0,
        flat_ceiling_h=2.2,
        flat_free_cost=1.0,
        flat_obstacle_cost=50.0,
    )

    report = cmu_unity_tomogram_capture._build_tomogram(pcd, tomogram, args)

    assert report["free_cells"] > report["obstacle_cells"]
    assert report["obstacle_cells"] > 0
    loaded = pickle.loads(tomogram.read_bytes())
    col = int(round((0.5 - loaded["origin"][0]) / loaded["resolution"]))
    row = int(round((0.0 - loaded["origin"][1]) / loaded["resolution"]))
    assert loaded["data"][0, 0, row, col] >= args.flat_obstacle_cost


def test_cmu_tomogram_capture_uses_sensor_qos_for_pointclouds():
    script = cmu_unity_sim_gate.ROOT / "sim/scripts/cmu_unity_tomogram_capture.py"
    text = script.read_text(encoding="utf-8")

    assert "ReliabilityPolicy.BEST_EFFORT" in text
    assert "cloud_qos" in text


def test_cmu_tomogram_capture_defaults_to_lingtu_canonical_map_topics():
    assert cmu_unity_tomogram_capture.DEFAULT_TOPICS == (
        "/nav/map_cloud",
        "/nav/terrain_map_ext",
    )


def test_cmu_unity_sim_gate_rejects_missing_unity_assets(tmp_path: Path):
    workspace = _make_fake_cmu_workspace(tmp_path)
    (workspace / cmu_unity_sim_gate.UNITY_ENVIRONMENT_PATHS["model_binary"]).unlink()

    report = cmu_unity_sim_gate.build_report(_args(workspace))

    assert report["ok"] is False
    assert "cmu_unity_environment_assets" in report["blockers"]


def test_cmu_unity_sim_gate_rejects_missing_workspace(tmp_path: Path):
    report = cmu_unity_sim_gate.build_report(_args(tmp_path / "missing"))

    assert report["ok"] is False
    assert "cmu_workspace_exists" in report["blockers"]


def test_cmu_unity_sim_gate_cli_writes_json_without_hardware_outputs(
    tmp_path: Path,
    monkeypatch,
):
    workspace = _make_fake_cmu_workspace(tmp_path)
    json_out = tmp_path / "report.json"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "cmu_unity_sim_gate.py",
            "--cmu-workspace",
            str(workspace),
            "--json-out",
            str(json_out),
            "--no-require-git",
            "--no-require-humble-branch",
            "--no-require-ros",
            "--strict",
        ],
    )

    assert cmu_unity_sim_gate.main() == 0

    loaded = json.loads(json_out.read_text(encoding="utf-8"))
    assert loaded["cmd_vel_sent_to_hardware"] is False
    assert loaded["runtime_executed"] is False
