from __future__ import annotations

from lingtu.assembly.products import resolve_product_host_config
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


def test_lite_resolves_direct_planner_profile() -> None:
    config = resolve_profile_config("lite")

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


def test_real_navigation_resolves_octoplanner3d_without_fallback_profile() -> None:
    config = resolve_product_host_config("nav", "real")

    assert config["planner"] == "octoplanner3d"
    assert config["map_path"].endswith((".bt", ".ot", ".octomap", ".pcd"))
    assert config["fallback_planner_name"] == ""
    assert config["planner_profile"]["profile"] == "nav"
    assert config["planner_profile"]["primary"] == "octoplanner3d"
    assert config["planner_profile"]["fallback_planners"] == []
    assert config["planner_profile"]["plan_safety_policy"] == "reject"
    assert config["planner_profile"]["latency_budget_ms"] == 800
    assert config["preview_timeout"] == 30.0
    assert config["octoplanner3d_timeout_s"] == 30.0
    assert config["waypoint_threshold"] == 0.20
    assert config["final_waypoint_threshold"] == 0.10
    assert "local_planner_allow_direct_track_fallback" not in config
    assert config["local_planner_direct_track_fallback_min_distance_m"] == 0.05
    assert config["local_planner_min_trackable_local_path_m"] == 0.05
    assert config["path_follower_goal_tolerance"] == 0.05
    assert config["path_follower_lookahead"] == 0.35
    assert config["path_follower_max_speed"] == 0.20
    assert config["path_follower_min_speed"] == 0.08
    assert config["octoplanner3d_robot_radius"] == 0.25
    assert config["octomap_resolution"] == 0.1
    assert config["octomap_free_layers_above"] == 6
    assert config["octoplanner3d_snap_search_radius_cells"] == 24
    assert config["octoplanner3d_require_ground_support"] is True
    assert config["octoplanner3d_strict_direct_ground_support"] is False
    assert config["octoplanner3d_ground_support_xy_radius_cells"] == 2
    assert config["octoplanner3d_ground_support_depth_cells"] == 2
    assert config["octoplanner3d_max_step_height"] == 0.45
    assert config["octoplanner3d_max_slope"] == 0.0


def test_simulation_profile_uses_octoplanner3d_without_fallback_chain() -> None:
    config = resolve_profile_config("sim")

    assert config["planner"] == "octoplanner3d"
    assert config["fallback_planner_name"] == ""
    assert config["planner_profile"]["profile"] == "sim"
    assert config["planner_profile"]["primary"] == "octoplanner3d"
    assert config["planner_profile"]["fallback_planners"] == []
    assert config["planner_profile"]["latency_budget_ms"] == 250


def test_octoplanner_override_is_runtime_profile_selectable() -> None:
    config = resolve_product_host_config(
        "nav",
        "real",
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


def test_retired_tare_product_name_has_no_special_planner_profile() -> None:
    profile = resolve_planner_runtime_profile("tare_explore", {})

    assert profile["latency_budget_ms"] == 500
