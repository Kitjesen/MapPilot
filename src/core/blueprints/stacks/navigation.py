"""Navigation stack: NavigationModule + optional Python autonomy chain."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import optional_stack_module, stack_module

logger = logging.getLogger(__name__)

_NAVIGATION_CONFIG_KEYS = (
    "obstacle_thr",
    "waypoint_threshold",
    "final_waypoint_threshold",
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
)


def navigation_module_config(
    planner_backend: str = "astar",
    tomogram: str = "",
    **config,
) -> dict:
    """Return NavigationModule constructor kwargs without resolving classes."""

    nav_config = {key: config[key] for key in _NAVIGATION_CONFIG_KEYS if key in config}
    return {
        "planner": planner_backend,
        "tomogram": tomogram,
        "enable_ros2_bridge": bool(config.get("enable_ros2_bridge", False)),
        **nav_config,
    }


def frontier_module_config(**config) -> dict:
    """Return WavefrontFrontierExplorer constructor kwargs."""

    return {
        "min_frontier_size": config.get("frontier_min_size", 5),
        "safe_distance": config.get("frontier_safe_distance", 1.0),
        "lookahead_distance": config.get("frontier_lookahead", 5.0),
        "max_explored_distance": config.get("frontier_max_dist", 15.0),
        "info_gain_threshold": config.get("frontier_info_gain", 0.03),
        "goal_timeout": config.get("frontier_goal_timeout", 30.0),
        "explore_rate": config.get("frontier_rate", 2.0),
        "blocked_goal_radius": config.get("frontier_blocked_goal_radius", 1.0),
        "blocked_goal_ttl": config.get("frontier_blocked_goal_ttl", 120.0),
        "approach_standoff_m": config.get("frontier_approach_standoff_m", 0.8),
        "approach_max_target_distance_m": config.get(
            "frontier_approach_max_target_distance_m",
            1.5,
        ),
        "approach_goal_max_distance_m": config.get(
            "frontier_approach_goal_max_distance_m",
            3.0,
        ),
        "reachable_goal_radius": config.get("frontier_reachable_goal_radius", 0.8),
        "navigation_failure_grace_s": config.get(
            "frontier_navigation_failure_grace_s",
            2.0,
        ),
        "cost_obstacle_threshold": config.get(
            "frontier_cost_obstacle_threshold",
            49.9,
        ),
    }


def autonomy_stack_config(enable_native: bool = True, **config) -> dict:
    """Return autonomy backend/config selection without importing backends."""

    path_follower_config = {
        param: config[key]
        for key, param in (
            ("path_follower_max_speed", "max_speed"),
            ("path_follower_lookahead", "lookahead"),
            ("path_follower_goal_tolerance", "goal_tolerance"),
            ("path_follower_min_speed", "min_speed"),
            ("path_follower_max_yaw_rate", "max_yaw_rate"),
            ("path_follower_turn_speed_yaw_rate_start", "turn_speed_yaw_rate_start"),
            ("path_follower_turn_speed_min_scale", "turn_speed_min_scale"),
            ("path_follower_two_way_drive", "two_way_drive"),
        )
        if key in config
    }
    local_planner_config = {
        param: config[key]
        for key, param in (
            ("local_planner_corridor_lookahead_m", "corridor_lookahead_m"),
            (
                "local_planner_allow_direct_track_fallback",
                "allow_direct_track_fallback",
            ),
            (
                "local_planner_ignore_near_field_stop",
                "ignore_near_field_stop",
            ),
            (
                "local_planner_direct_track_fallback_min_distance_m",
                "direct_track_fallback_min_distance_m",
            ),
            (
                "local_planner_min_trackable_local_path_m",
                "min_trackable_local_path_m",
            ),
        )
        if key in config
    }
    if enable_native:
        backend = config.get("local_planner_backend", "cmu")
        path_follower_backend = config.get("path_follower_backend", "nav_core")
    else:
        backend = config.get(
            "local_planner_backend",
            config.get("python_autonomy_backend", "nanobind"),
        )
        path_follower_backend = config.get(
            "path_follower_backend",
            config.get("python_path_follower_backend", "nav_core"),
        )

    return {
        "backend": backend,
        "terrain_backend": config.get("terrain_backend"),
        "path_follower_backend": path_follower_backend,
        "local_planner_config": local_planner_config,
        "path_follower_config": path_follower_config,
    }


def navigation(
    planner_backend: str = "astar",
    tomogram: str = "",
    enable_native: bool = True,
    **config,
) -> Blueprint:
    """Global planning + local autonomy (terrain -> local planner -> path follower)."""
    bp = Blueprint()

    NavigationModule = stack_module(
        "navigation",
        "default",
        seed_group="navigation",
        fallback="nav.navigation_module.NavigationModule",
    )
    bp.add(
        NavigationModule,
        alias="NavigationModule",
        **navigation_module_config(planner_backend, tomogram, **config),
    )

    if config.get("enable_ros2_bridge", False):
        ROS2WaypointBridgeModule = optional_stack_module(
            "navigation",
            "ros2_waypoint_bridge",
            seed_group="navigation",
            fallback="nav.ros2_waypoint_bridge_module.ROS2WaypointBridgeModule",
        )
        if ROS2WaypointBridgeModule is not None:
            waypoint_bridge_config = {}
            if "planning_frame_id" in config:
                waypoint_bridge_config["default_frame_id"] = config["planning_frame_id"]
            bp.add(
                ROS2WaypointBridgeModule,
                alias="ROS2WaypointBridgeModule",
                **waypoint_bridge_config,
            )
            bp.wire(
                "NavigationModule",
                "waypoint",
                "ROS2WaypointBridgeModule",
                "waypoint",
            )
        else:
            logger.warning("ROS2 waypoint bridge not available")

    if config.get("enable_ros2_path_bridge", False):
        ROS2PathBridgeModule = optional_stack_module(
            "navigation",
            "ros2_path_bridge",
            seed_group="navigation",
            fallback="nav.ros2_path_bridge_module.ROS2PathBridgeModule",
        )
        if ROS2PathBridgeModule is not None:
            path_bridge_config = {}
            if "planning_frame_id" in config:
                path_bridge_config["default_frame_id"] = config["planning_frame_id"]
            bp.add(
                ROS2PathBridgeModule,
                alias="ROS2PathBridgeModule",
                **path_bridge_config,
            )
        else:
            logger.warning("ROS2 path bridge not available")

    if config.get("enable_frontier", False):
        try:
            WavefrontFrontierExplorer = stack_module(
                "exploration",
                "wavefront_frontier",
                seed_group="exploration",
                fallback="nav.frontier_explorer_module.WavefrontFrontierExplorer",
            )
            bp.add(
                WavefrontFrontierExplorer,
                alias="WavefrontFrontierExplorer",
                **frontier_module_config(**config),
            )
            bp.wire(
                "WavefrontFrontierExplorer",
                "exploration_goal",
                "NavigationModule",
                "goal_pose",
            )
            bp.wire(
                "NavigationModule",
                "mission_status",
                "WavefrontFrontierExplorer",
                "navigation_status",
            )
        except ImportError as e:
            logger.warning("FrontierExplorer not available: %s", e)

    if config.get("enable_traversable_frontier", False):
        try:
            TraversableFrontierModule = stack_module(
                "navigation",
                "traversable_frontier",
                seed_group="navigation",
                fallback="nav.traversable_frontier_module.TraversableFrontierModule",
            )

            bp.add(
                TraversableFrontierModule,
                alias="TraversableFrontierModule",
                min_frontier_size=config.get("traversable_frontier_min_size", 5),
                safe_distance=config.get("traversable_frontier_safe_distance", 1.0),
                lookahead_distance=config.get("traversable_frontier_lookahead", 5.0),
                max_explored_distance=config.get("traversable_frontier_max_dist", 15.0),
                info_gain_threshold=config.get("traversable_frontier_info_gain", 0.03),
                goal_timeout=config.get("traversable_frontier_goal_timeout", 30.0),
                explore_rate=config.get("traversable_frontier_rate", 2.0),
                blocked_goal_radius=config.get(
                    "traversable_frontier_blocked_goal_radius",
                    1.0,
                ),
                blocked_goal_ttl=config.get(
                    "traversable_frontier_blocked_goal_ttl",
                    120.0,
                ),
                max_slope_deg=config.get("traversable_frontier_max_slope_deg", 35.0),
                max_frontier_cost=config.get("traversable_frontier_max_cost", 80.0),
                semantic_prior_weight=config.get(
                    "traversable_frontier_semantic_prior_weight",
                    0.0,
                ),
            )
        except ImportError as e:
            logger.warning("TraversableFrontierModule not available: %s", e)

    try:
        from base_autonomy.modules.autonomy_module import add_autonomy_stack
        autonomy_config = autonomy_stack_config(enable_native, **config)
        path_follower_config = autonomy_config.pop("path_follower_config")
        add_autonomy_stack(bp, **autonomy_config, **path_follower_config)
    except ImportError as e:
        logger.warning("Autonomy stack not available: %s", e)

    return bp
