from __future__ import annotations

from runtime.profiles.planner_backends import (
    normalize_planner_name,
    resolve_planner_runtime_profile,
)
from runtime.profiles.resolver import resolve_profile_config


def test_planner_name_normalization_maps_octplanner_alias() -> None:
    assert normalize_planner_name("octplanner") == "octoplanner3d"
    assert normalize_planner_name("octo") == "octoplanner3d"
    assert normalize_planner_name("octomap") == "octoplanner3d"
    assert normalize_planner_name(" OctoPlanner3D ") == "octoplanner3d"


def test_thunder_lite_resolves_direct_planner_profile() -> None:
    config = resolve_profile_config("thunder-lite")

    assert config["planner"] == "direct"
    assert config["planner_latency_budget_ms"] == 50
    assert config["planner_profile"] == {
        "schema_version": "lingtu.planner_runtime_profile.v1",
        "profile": "lite",
        "primary": "direct",
        "fallback_planners": [],
        "plan_safety_policy": "observe",
        "latency_budget_ms": 50,
    }


def test_thunder_field_navigation_resolves_octoplanner3d_without_fallback_profile() -> None:
    config = resolve_profile_config("thunder-nav")

    assert config["planner"] == "octoplanner3d"
    assert config["tomogram"].endswith((".bt", ".ot", ".octomap", ".pcd"))
    assert config["fallback_planner_name"] == ""
    assert config["planner_profile"]["profile"] == "nav"
    assert config["planner_profile"]["primary"] == "octoplanner3d"
    assert config["planner_profile"]["fallback_planners"] == []
    assert config["planner_profile"]["plan_safety_policy"] == "reject"
    assert config["planner_profile"]["latency_budget_ms"] == 800
    assert config["octoplanner3d_robot_radius"] == 0.60
    assert config["octoplanner3d_require_ground_support"] is True


def test_simulation_profile_uses_octoplanner3d_without_fallback_chain() -> None:
    config = resolve_profile_config("sim")

    assert config["planner"] == "octoplanner3d"
    assert config["fallback_planner_name"] == ""
    assert config["planner_profile"]["profile"] == "sim"
    assert config["planner_profile"]["primary"] == "octoplanner3d"
    assert config["planner_profile"]["fallback_planners"] == []
    assert config["planner_profile"]["latency_budget_ms"] == 250


def test_octoplanner_override_is_runtime_profile_selectable() -> None:
    config = resolve_profile_config(
        "thunder-nav",
        planner="octplanner",
        fallback_planner_name="pct",
        planner_latency_budget_ms=1200,
    )

    assert config["planner"] == "octoplanner3d"
    assert config["fallback_planners"] == ["pct"]
    assert config["planner_profile"]["primary"] == "octoplanner3d"
    assert config["planner_profile"]["fallback_planners"] == ["pct"]
    assert config["planner_profile"]["latency_budget_ms"] == 1200


def test_resolve_planner_runtime_profile_accepts_fallback_list() -> None:
    profile = resolve_planner_runtime_profile(
        "nav",
        {
            "planner": "octplanner",
            "fallback_planners": ["pct", "direct", "pct"],
            "plan_safety_policy": "reject",
        },
    )

    assert profile["primary"] == "octoplanner3d"
    assert profile["fallback_planners"] == ["pct", "direct"]
