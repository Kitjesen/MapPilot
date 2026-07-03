"""Local autonomy chain composition."""

from __future__ import annotations

import logging

from runtime.blueprint import Blueprint
from runtime.profiles.binding_policy import resolved_autonomy_backend_selection

logger = logging.getLogger(__name__)


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
            ("path_follower_native_max_accel", "native_max_accel"),
            ("path_follower_yaw_rate_gain", "yaw_rate_gain"),
            ("path_follower_stop_yaw_rate_gain", "stop_yaw_rate_gain"),
            ("path_follower_dir_diff_thre", "dir_diff_thre"),
            ("path_follower_two_way_drive", "two_way_drive"),
            ("path_follower_use_incl_rate_to_slow", "use_incl_rate_to_slow"),
            ("path_follower_incl_rate_thre", "incl_rate_thre"),
            ("path_follower_slow_rate_1", "slow_rate_1"),
            ("path_follower_slow_rate_2", "slow_rate_2"),
            ("path_follower_slow_rate_3", "slow_rate_3"),
            ("path_follower_slow_time_1", "slow_time_1"),
            ("path_follower_slow_time_2", "slow_time_2"),
            ("path_follower_use_incl_to_stop", "use_incl_to_stop"),
            ("path_follower_incl_thre", "incl_thre"),
            ("path_follower_stop_time", "stop_time"),
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
            ("planning_frame_id", "planning_frame_id"),
        )
        if key in config
    }
    backend_selection = resolved_autonomy_backend_selection(
        config,
        enable_native=enable_native,
    )

    return {
        "backend": backend_selection["local_planner_backend"],
        "terrain_backend": backend_selection["terrain_backend"],
        "path_follower_backend": backend_selection["path_follower_backend"],
        "local_planner_config": local_planner_config,
        "path_follower_config": path_follower_config,
    }


def add_autonomy_chain(
    bp: Blueprint,
    *,
    enable_native: bool = True,
    **config,
) -> Blueprint:
    """Add terrain, local-planner, and path-follower Modules."""

    try:
        from nav.local.stack import add_autonomy_stack

        autonomy_config = autonomy_stack_config(enable_native, **config)
        path_follower_config = autonomy_config.pop("path_follower_config")
        add_autonomy_stack(bp, **autonomy_config, **path_follower_config)
    except ImportError as e:
        logger.warning("Autonomy stack not available: %s", e)

    return bp
