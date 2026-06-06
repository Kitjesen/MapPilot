import pytest

pytestmark = [pytest.mark.sim]

import builtins
import types
from pathlib import Path

from sim.validation import full_system as full_system_module
from sim.validation.full_system import (
    BLOCKED,
    FAIL,
    PASS,
    ValidationCheck,
    run_validation,
    _timed,
    validate_navigation_blueprint,
    validate_policy_smoke_runtime,
    validate_scene_catalog,
)


def test_scene_catalog_identifies_multifloor_building_contract():
    repo_root = Path(__file__).resolve().parents[2]
    checks = validate_scene_catalog(repo_root)
    by_name = {check.name: check for check in checks}

    assert by_name["required_worlds_exist"].status == PASS
    multifloor = by_name["building_scene_multifloor_contract"]
    assert multifloor.status == PASS
    assert multifloor.evidence["step_count"] >= 10
    assert multifloor.evidence["second_floor_geom_count"] >= 2
    assert multifloor.evidence["max_geom_z"] >= 3.5


def test_static_full_system_validation_covers_required_capabilities():
    report = run_validation(run_mujoco=False)
    categories = {check.category for check in report.checks}

    assert {
        "scene",
        "lidar",
        "slam_localization",
        "navigation",
        "exploration",
        "tracking",
    } <= categories
    assert not [check for check in report.checks if check.status == "fail"]
    nav = next(check for check in report.checks if check.name == "sim_nav_planning_wiring")
    assert report.passed is (nav.status == PASS)
    lidar = [check for check in report.checks if check.category == "lidar"]
    assert lidar and lidar[0].status == BLOCKED


def test_navigation_blueprint_validation_uses_static_profile_graph(monkeypatch):
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "core.blueprints.full_stack":
            raise AssertionError(f"runtime blueprint import attempted: {name}")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(full_system_module, "_NUMPY_IMPORT_SAFE", False)
    monkeypatch.setattr(builtins, "__import__", guarded_import)

    check = validate_navigation_blueprint(Path(__file__).resolve().parents[2])

    assert check.status == BLOCKED
    assert check.evidence["profile_graph_mode"] == "static"
    assert check.evidence["missing_connections"] == []
    assert check.evidence["runtime_parity"]["status"] == BLOCKED


def test_navigation_blueprint_validation_passes_runtime_parity_when_available(
    monkeypatch,
):
    from core.blueprints.profile_graph import WireEdge

    required_edges = (
        WireEdge("MujocoDriverModule", "odometry", "NavigationModule", "odometry"),
        WireEdge("MujocoDriverModule", "map_cloud", "OccupancyGridModule", "map_cloud"),
        WireEdge("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireEdge("TraversabilityCostModule", "fused_cost", "NavigationModule", "costmap"),
        WireEdge("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
        WireEdge("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
        WireEdge("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        WireEdge("CmdVelMux", "driver_cmd_vel", "MujocoDriverModule", "cmd_vel"),
    )
    graph = types.SimpleNamespace(
        modules=(
            "MujocoDriverModule",
            "NavigationModule",
            "OccupancyGridModule",
            "TraversabilityCostModule",
            "LocalPlannerModule",
            "PathFollowerModule",
            "CmdVelMux",
        ),
        explicit_wires=required_edges,
        as_snapshot=lambda: {
            "modules": sorted(
                (
                    "MujocoDriverModule",
                    "NavigationModule",
                    "OccupancyGridModule",
                    "TraversabilityCostModule",
                    "LocalPlannerModule",
                    "PathFollowerModule",
                    "CmdVelMux",
                )
            ),
            "explicit_wires": sorted(edge.as_snapshot() for edge in required_edges),
        },
    )

    def fake_graph_for_profile(_profile, **_kwargs):
        return graph

    monkeypatch.setattr(full_system_module, "_NUMPY_IMPORT_SAFE", True)
    monkeypatch.setattr(
        "core.blueprints.profile_graph.graph_for_profile",
        fake_graph_for_profile,
    )

    check = validate_navigation_blueprint(Path(__file__).resolve().parents[2])

    assert check.status == PASS
    assert check.evidence["runtime_parity"]["checked"] is True
    assert check.evidence["runtime_parity"]["status"] == PASS


def test_navigation_blueprint_runtime_internal_import_error_fails(monkeypatch):
    from core.blueprints.profile_graph import WireEdge

    required_edges = (
        WireEdge("MujocoDriverModule", "odometry", "NavigationModule", "odometry"),
        WireEdge("MujocoDriverModule", "map_cloud", "OccupancyGridModule", "map_cloud"),
        WireEdge("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireEdge("TraversabilityCostModule", "fused_cost", "NavigationModule", "costmap"),
        WireEdge("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
        WireEdge("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
        WireEdge("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        WireEdge("CmdVelMux", "driver_cmd_vel", "MujocoDriverModule", "cmd_vel"),
    )
    graph = types.SimpleNamespace(
        modules=(
            "MujocoDriverModule",
            "NavigationModule",
            "OccupancyGridModule",
            "TraversabilityCostModule",
            "LocalPlannerModule",
            "PathFollowerModule",
            "CmdVelMux",
        ),
        explicit_wires=required_edges,
        as_snapshot=lambda: {
            "modules": sorted(
                (
                    "MujocoDriverModule",
                    "NavigationModule",
                    "OccupancyGridModule",
                    "TraversabilityCostModule",
                    "LocalPlannerModule",
                    "PathFollowerModule",
                    "CmdVelMux",
                )
            ),
            "explicit_wires": sorted(edge.as_snapshot() for edge in required_edges),
        },
    )

    def fake_graph_for_profile(_profile, **kwargs):
        if kwargs.get("mode") == "runtime":
            raise ModuleNotFoundError(
                "No module named 'core.internal_typo'",
                name="core.internal_typo",
            )
        return graph

    monkeypatch.setattr(full_system_module, "_NUMPY_IMPORT_SAFE", True)
    monkeypatch.setattr(
        "core.blueprints.profile_graph.graph_for_profile",
        fake_graph_for_profile,
    )

    check = validate_navigation_blueprint(Path(__file__).resolve().parents[2])

    assert check.status == FAIL
    assert check.evidence["runtime_parity"]["status"] == FAIL
    assert check.evidence["runtime_parity"]["missing_module"] == "core"


def test_default_validation_requires_navigation_runtime_parity(monkeypatch):
    def passing_check(name: str, category: str) -> ValidationCheck:
        return ValidationCheck(name=name, category=category, status=PASS, summary="ok")

    monkeypatch.setattr(
        full_system_module,
        "validate_scene_catalog",
        lambda _root: [passing_check("required_worlds_exist", "scene")],
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_slam_localization_contract",
        lambda _root: passing_check("slam_localization_metric_gate", "slam_localization"),
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_navigation_blueprint",
        lambda _root: ValidationCheck(
            name="sim_nav_planning_wiring",
            category="navigation",
            status=BLOCKED,
            summary="runtime parity blocked",
            evidence={"runtime_parity": {"status": BLOCKED}},
        ),
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_frontier_exploration_runtime",
        lambda: passing_check("frontier_exploration_to_navigation", "exploration"),
    )
    monkeypatch.setattr(
        full_system_module,
        "validate_person_tracking_runtime",
        lambda: passing_check("person_tracking_behavior_loop", "tracking"),
    )

    report = run_validation(run_mujoco=False)

    assert report.passed is False
    assert any(check.name == "mujoco_lidar_runtime" and check.status == BLOCKED for check in report.checks)


def test_static_full_system_validation_can_treat_blocked_checks_as_failure():
    report = run_validation(run_mujoco=False, require_all=True)

    assert report.passed is False
    assert any(check.status == BLOCKED for check in report.checks)


def test_policy_smoke_reports_missing_policy_as_blocked(monkeypatch):
    from sim.scripts import policy_nav_smoke

    direct = {"policy_loaded": False, "policy_path": "", "moved_m": 0.0}
    nav = {"policy_loaded": False, "policy_path": "", "moved_m": 0.0}
    monkeypatch.setattr(policy_nav_smoke, "run_direct_policy", lambda **_: direct)
    monkeypatch.setattr(policy_nav_smoke, "run_full_stack_nav", lambda **_: nav)
    monkeypatch.setattr(policy_nav_smoke, "_passes_direct", lambda *_args, **_kw: False)
    monkeypatch.setattr(policy_nav_smoke, "_passes_nav", lambda *_args, **_kw: False)
    monkeypatch.setattr(policy_nav_smoke, "_load_policy_metadata", lambda _: {"exists": False})

    check = validate_policy_smoke_runtime(
        world="open_field",
        direct_duration_s=0.1,
        nav_duration_s=0.1,
        goal_distance_m=0.5,
    )

    assert check.status == BLOCKED
    assert "checkpoint" in check.summary


def test_policy_smoke_can_pass_with_explicit_policy(monkeypatch):
    from sim.scripts import policy_nav_smoke

    direct = {"policy_loaded": True, "policy_path": "policy.onnx", "moved_m": 0.3}
    nav = {"policy_loaded": True, "policy_path": "policy.onnx", "moved_m": 0.3}
    monkeypatch.setattr(policy_nav_smoke, "run_direct_policy", lambda **_: direct)
    monkeypatch.setattr(policy_nav_smoke, "run_full_stack_nav", lambda **_: nav)
    monkeypatch.setattr(policy_nav_smoke, "_passes_direct", lambda *_args, **_kw: True)
    monkeypatch.setattr(policy_nav_smoke, "_passes_nav", lambda *_args, **_kw: True)
    monkeypatch.setattr(policy_nav_smoke, "_load_policy_metadata", lambda _: {"exists": True})

    check = validate_policy_smoke_runtime(
        world="open_field",
        direct_duration_s=0.1,
        nav_duration_s=0.1,
        goal_distance_m=0.5,
        policy_path="policy.onnx",
    )

    assert check.status == PASS
    assert check.evidence["policy_path_arg"] == "policy.onnx"


def test_timed_marks_missing_environment_dependency_as_blocked():
    def missing_yaml():
        raise ModuleNotFoundError("No module named 'yaml'", name="yaml")

    check = _timed("yaml_required_check", "environment", missing_yaml)

    assert check.status == BLOCKED
    assert check.summary == "missing Python dependency: yaml"
    assert check.evidence["missing_module"] == "yaml"


def test_timed_keeps_internal_missing_module_as_failure():
    def missing_internal_module():
        raise ModuleNotFoundError(
            "No module named 'core.internal_typo'",
            name="core.internal_typo",
        )

    check = _timed("internal_import_check", "architecture", missing_internal_module)

    assert check.status == FAIL
    assert check.evidence["missing_module"] == "core"
