"""Core navigation Module composition."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module

_NAVIGATION_CONFIG_KEYS = (
    "obstacle_thr",
    "waypoint_threshold",
    "final_waypoint_threshold",
    "complete_path_on_goal_proximity",
    "goal_proximity_completion_threshold",
    "stuck_timeout",
    "stuck_dist_thre",
    "max_replan_count",
    "downsample_dist",
    "allow_direct_goal_fallback",
    "direct_goal_fallback_on_planner_failure",
    "external_strategy_path_control",
    "external_strategy_start_tolerance_m",
    "goal_update_epsilon",
    "mission_timeout",
    "planning_frame_id",
    "expected_saved_map_frame_id",
    "planning_timeout",
    "preview_timeout",
    "safe_goal_tolerance",
    "plan_safety_policy",
    "fallback_planner_name",
    "accept_partial_goal_progress",
    "partial_goal_repeat_ignore_window_s",
    "defer_empty_path_planning_failure",
    "empty_path_retry_interval_s",
    "empty_path_retry_timeout_s",
    "replan_on_costmap_update",
    "allow_path_start_insert",
    "octoplanner3d_robot_radius",
    "octoplanner3d_max_iterations",
    "octoplanner3d_snap_search_radius_cells",
    "octoplanner3d_require_ground_support",
    "octoplanner3d_strict_direct_ground_support",
    "octoplanner3d_ground_support_xy_radius_cells",
    "octoplanner3d_ground_support_depth_cells",
    "octoplanner3d_enable_preblocked_costmap",
    "octoplanner3d_preblocked_costmap_radius_cells",
    "octoplanner3d_preblocked_costmap_weight",
    "octoplanner3d_lowest_traversable_only",
)


def navigation_config(
    planner_backend: str = "octoplanner3d",
    tomogram: str = "",
    **config,
) -> dict:
    """Return Navigation constructor kwargs without resolving classes."""

    nav_config = {key: config[key] for key in _NAVIGATION_CONFIG_KEYS if key in config}
    return {
        "planner": planner_backend,
        "tomogram": tomogram,
        **nav_config,
    }


def add_navigation_core(
    bp: Blueprint,
    *,
    planner_backend: str = "octoplanner3d",
    tomogram: str = "",
    **config,
) -> Blueprint:
    """Add the mission/navigation Module to a Blueprint."""

    Navigation = stack_module(
        "navigation",
        "default",
        seed_group="navigation",
        fallback="nav.mission.navigation.Navigation",
    )
    bp.add(
        Navigation,
        alias="nav.mission",
        **navigation_config(planner_backend, tomogram, **config),
    )
    return bp
