"""Runtime endpoint adapter override configs.

Endpoint specs describe where data and commands cross the runtime seam. These
dicts describe adapter-specific overrides for simulator and external benchmark
endpoints, kept separate from product intent profiles.
"""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
    _resolve_octoplanner3d_map,
)
from runtime.runtime_interface import TOPICS

MUJOCO_LIVE_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "octoplanner3d",
    "map_path": _resolve_octoplanner3d_map(),
    "plan_safety_policy": "reject",
    "fallback_planner_name": "",
    "enable_semantic": False,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_map_out": False,
    "enable_camera": False,
    "use_driver_camera": False,
    "cloud_topic": TOPICS.map_cloud,
    "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
    "enable_native": False,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_kernel",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": DEFAULT_GATEWAY_PORT,
}

GAZEBO_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "octoplanner3d",
    "map_path": _resolve_octoplanner3d_map(),
    "plan_safety_policy": "reject",
    "fallback_planner_name": "",
    "enable_semantic": True,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_map_out": False,
    "enable_camera": True,
    "use_driver_camera": True,
    "cloud_topic": TOPICS.map_cloud,
    "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
    "enable_frontier": True,
    "exploration_backend": "none",
    "frontier_safe_distance": 0.80,
    "frontier_max_dist": 20.0,
    "frontier_rate": 2.0,
    "enable_native": False,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_kernel",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": DEFAULT_GATEWAY_PORT,
}

CMU_UNITY_CONFIG: dict[str, Any] = {
    "slam_profile": "none",
    "llm": "mock",
    "planner": "octoplanner3d",
    "map_path": _resolve_octoplanner3d_map(),
    "plan_safety_policy": "reject",
    "fallback_planner_name": "",
    "safe_goal_tolerance": 0.4,
    "waypoint_threshold": 0.45,
    "final_waypoint_threshold": 0.35,
    "stuck_timeout": 25.0,
    "stuck_dist_thre": 0.08,
    "downsample_dist": 0.6,
    "path_follower_goal_tolerance": 0.35,
    "local_planner_allow_direct_track_fallback": True,
    "local_planner_ignore_near_field_stop": True,
    "local_planner_direct_track_fallback_min_distance_m": 0.3,
    "enable_native": False,
    "enable_semantic": False,
    "enable_gateway": True,
    "enable_teleop": False,
    "enable_map_modules": True,
    "enable_frontier": False,
    "enable_traversable_frontier": False,
    "exploration_backend": "tare_external",
    "exploration_auto_start": True,
    "prefer_path_strategy": True,
    "path_start_tolerance_m": 1.5,
    "path_goal_min_distance_m": 1.0,
    "path_goal_spacing_m": 0.75,
    "tare_fallback_timeout_s": 180.0,
    "allow_direct_goal_fallback": True,
    "direct_goal_fallback_on_planner_failure": True,
    "accept_partial_goal_progress": True,
    "partial_goal_repeat_ignore_window_s": 5.0,
    "external_strategy_path_control": False,
    "external_strategy_start_tolerance_m": 1.5,
    "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
    "latch_stop_signal": False,
    "python_autonomy_backend": "nanobind",
    "python_path_follower_backend": "nav_kernel",
    "run_startup_checks": False,
    "manage_external_services": False,
    "gateway_port": DEFAULT_GATEWAY_PORT,
}
