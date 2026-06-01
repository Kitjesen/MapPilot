import pytest

pytestmark = [pytest.mark.sim]

from pathlib import Path

from sim.validation.full_system import (
    BLOCKED,
    PASS,
    run_validation,
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

    assert report.passed is True
    assert {
        "scene",
        "lidar",
        "slam_localization",
        "navigation",
        "exploration",
        "tracking",
    } <= categories
    assert not [check for check in report.checks if check.status == "fail"]
    lidar = [check for check in report.checks if check.category == "lidar"]
    assert lidar and lidar[0].status == BLOCKED


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
