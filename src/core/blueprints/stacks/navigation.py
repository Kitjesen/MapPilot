"""Navigation stack: NavigationModule + optional Python autonomy chain."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import optional_stack_module, stack_module

logger = logging.getLogger(__name__)


def _enabled(config: dict, name: str, legacy_name: str) -> bool:
    if name in config:
        return bool(config[name])
    return bool(config.get(legacy_name, False))


def _endpoint_egress_uses_lcm(config: dict) -> bool:
    selected = str(
        config.get("endpoint_path_bridge")
        or config.get("endpoint_egress_adapter")
        or ""
    ).lower()
    endpoint_transport = str(
        config.get("_endpoint_transport")
        or config.get("endpoint_transport")
        or ""
    ).lower()

    return selected in {"lcm", "lcm_endpoint", "lcm_path_command_bridge"} or (
        not selected and endpoint_transport == "lcm"
    )


def _endpoint_ingress_uses_lcm(config: dict) -> bool:
    selected = str(
        config.get("endpoint_command_bridge")
        or config.get("endpoint_ingress_adapter")
        or ""
    ).lower()
    endpoint_transport = str(
        config.get("_endpoint_transport")
        or config.get("endpoint_transport")
        or ""
    ).lower()

    return selected in {"lcm", "lcm_endpoint", "lcm_navigation_command_bridge"} or (
        not selected and endpoint_transport == "lcm"
    )


def _endpoint_path_bridge_spec(config: dict) -> tuple[str, str]:
    if _endpoint_egress_uses_lcm(config):
        return (
            "lcm_path_command_bridge",
            "compat.lcm.path_command_adapter.LCMPathCommandBridgeModule",
        )
    return (
        "ros2_path_bridge",
        "compat.ros2.nav.path_bridge.ROS2PathBridgeModule",
    )


def _endpoint_command_bridge_spec(config: dict) -> tuple[str, str]:
    if _endpoint_ingress_uses_lcm(config):
        return (
            "lcm_navigation_command_bridge",
            "compat.lcm.navigation_command_adapter.LCMNavigationCommandBridgeModule",
        )
    return (
        "ros2_navigation_command_bridge",
        "compat.ros2.nav.command_bridge.ROS2NavigationCommandBridgeModule",
    )


def _endpoint_bridge_seed_group(bridge_name: str) -> str:
    if bridge_name.startswith("lcm_"):
        return "navigation_lcm"
    return "navigation_ros2"


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
            ("path_follower_yaw_rate_gain", "yaw_rate_gain"),
            ("path_follower_stop_yaw_rate_gain", "stop_yaw_rate_gain"),
            ("path_follower_dir_diff_thre", "dir_diff_thre"),
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

    if _enabled(config, "enable_endpoint_command_bridge", "enable_ros2_command_bridge"):
        bridge_name, bridge_fallback = _endpoint_command_bridge_spec(config)
        EndpointCommandBridgeModule = optional_stack_module(
            "navigation",
            bridge_name,
            seed_group=_endpoint_bridge_seed_group(bridge_name),
            fallback=bridge_fallback,
        )
        if EndpointCommandBridgeModule is not None:
            command_bridge_config = {}
            if "planning_frame_id" in config:
                command_bridge_config["default_frame_id"] = config["planning_frame_id"]
            if bridge_name == "lcm_navigation_command_bridge":
                endpoint_contract = config.get("_endpoint_contract") or config.get(
                    "endpoint_contract"
                )
                if endpoint_contract:
                    command_bridge_config["endpoint_contract"] = endpoint_contract
            bp.add(
                EndpointCommandBridgeModule,
                alias="EndpointCommandBridgeModule",
                **command_bridge_config,
            )
            bp.wire(
                "EndpointCommandBridgeModule",
                "goal_pose",
                "NavigationModule",
                "goal_pose",
            )
            bp.wire(
                "EndpointCommandBridgeModule",
                "cancel",
                "NavigationModule",
                "cancel",
            )
            bp.wire(
                "EndpointCommandBridgeModule",
                "instruction",
                "NavigationModule",
                "instruction",
            )
        else:
            logger.warning("Endpoint command bridge not available")

    if _enabled(config, "enable_endpoint_waypoint_bridge", "enable_ros2_bridge"):
        if _endpoint_egress_uses_lcm(config):
            logger.debug(
                "Endpoint waypoint egress is handled by the LCM path command bridge"
            )
        else:
            EndpointWaypointBridgeModule = optional_stack_module(
                "navigation",
                "ros2_waypoint_bridge",
                seed_group="navigation_ros2",
                fallback="compat.ros2.nav.waypoint_bridge.ROS2WaypointBridgeModule",
            )
            if EndpointWaypointBridgeModule is not None:
                waypoint_bridge_config = {}
                if "planning_frame_id" in config:
                    waypoint_bridge_config["default_frame_id"] = config[
                        "planning_frame_id"
                    ]
                bp.add(
                    EndpointWaypointBridgeModule,
                    alias="EndpointWaypointBridgeModule",
                    **waypoint_bridge_config,
                )
                bp.wire(
                    "NavigationModule",
                    "waypoint",
                    "EndpointWaypointBridgeModule",
                    "waypoint",
                )
            else:
                logger.warning("Endpoint waypoint bridge not available")

    if _enabled(config, "enable_endpoint_path_bridge", "enable_ros2_path_bridge"):
        bridge_name, bridge_fallback = _endpoint_path_bridge_spec(config)
        EndpointPathBridgeModule = optional_stack_module(
            "navigation",
            bridge_name,
            seed_group=_endpoint_bridge_seed_group(bridge_name),
            fallback=bridge_fallback,
        )
        if EndpointPathBridgeModule is not None:
            path_bridge_config = {}
            if "planning_frame_id" in config:
                path_bridge_config["default_frame_id"] = config["planning_frame_id"]
            if bridge_name == "lcm_path_command_bridge":
                endpoint_contract = config.get("_endpoint_contract") or config.get(
                    "endpoint_contract"
                )
                if endpoint_contract:
                    path_bridge_config["endpoint_contract"] = endpoint_contract
            bp.add(
                EndpointPathBridgeModule,
                alias="EndpointPathBridgeModule",
                **path_bridge_config,
            )
            if bridge_name == "lcm_path_command_bridge":
                bp.wire(
                    "NavigationModule",
                    "waypoint",
                    "EndpointPathBridgeModule",
                    "waypoint",
                )
        else:
            logger.warning("Endpoint path bridge not available")

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
