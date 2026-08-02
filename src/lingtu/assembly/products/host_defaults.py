"""Blueprint defaults for Host processes inside Field Products.

Runtime Graph YAML declares the Products. Product assembly owns these Python
Host inputs; the runtime Profile catalog intentionally does not expose them.
"""

from __future__ import annotations

from typing import Any

from runtime.graph.loader import load_runtime_graph
from runtime.profiles.catalog.navigation_defaults import (
    ACTIVE_OCTOPLANNER3D_MAP,
    THUNDER_OCTOPLANNER_DEFAULTS,
)
from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
)

_RUNTIME_GRAPH_PRODUCTS = load_runtime_graph().products
FIELD_PRODUCT_NAMES = tuple(_RUNTIME_GRAPH_PRODUCTS)

FIELD_PRODUCT_HOST_DEFAULTS: dict[str, dict[str, Any]] = {
    "explore": {
        "_desc": "Explore unknown area using native frontier navigation",
        "slam_profile": "fastlio2",
        "llm": "qwen",
        "planner": "octoplanner3d",
        "map_path": ACTIVE_OCTOPLANNER3D_MAP,
        "plan_safety_policy": "reject",
        "fallback_planner_name": "",
        **THUNDER_OCTOPLANNER_DEFAULTS,
        "enable_native": False,
        "enable_semantic": True,
        "enable_gateway": True,
        "enable_frontier": True,
        "enable_traversable_frontier": True,
        "exploration_backend": "none",
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
    "inspection": {
        "_desc": "Run native multi-point inspection over a saved map",
        "slam_profile": "localizer",
        "llm": "qwen",
        "planner": "octoplanner3d",
        "map_path": ACTIVE_OCTOPLANNER3D_MAP,
        "plan_safety_policy": "reject",
        "fallback_planner_name": "",
        **THUNDER_OCTOPLANNER_DEFAULTS,
        "enable_native": False,
        "enable_semantic": True,
        "enable_gateway": True,
        "enable_teleop": False,
        "enable_map_modules": True,
        "enable_goals": True,
        "enable_inspection_evidence": True,
        "enable_patrol_routes": False,
        "enable_scheduler": False,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
    "map": {
        "_desc": "Build a new map of the environment",
        "slam_profile": "fastlio2",
        "llm": "mock",
        "planner": "direct",
        "enable_navigation": False,
        "enable_native": False,
        "enable_semantic": False,
        "enable_gateway": True,
        "enable_teleop": True,
        "enable_map_modules": True,
        "enable_goals": False,
        "enable_patrol_routes": False,
        "enable_scheduler": False,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
    "nav": {
        "_desc": "Navigate using a saved map",
        "llm": "qwen",
        "planner": "octoplanner3d",
        "map_path": ACTIVE_OCTOPLANNER3D_MAP,
        "plan_safety_policy": "reject",
        "fallback_planner_name": "",
        "waypoint_threshold": 0.20,
        "final_waypoint_threshold": 0.10,
        "native_corridor_lookahead_m": 3.0,
        "local_planner_direct_track_fallback_min_distance_m": 0.05,
        "local_planner_min_trackable_local_path_m": 0.05,
        "path_follower_goal_tolerance": 0.05,
        "path_follower_lookahead": 0.35,
        "path_follower_max_speed": 0.20,
        "path_follower_max_accel": 1.0,
        "path_follower_min_speed": 0.08,
        **THUNDER_OCTOPLANNER_DEFAULTS,
        "enable_native": False,
        "enable_semantic": True,
        "enable_gateway": True,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
    "teleop": {
        "_desc": "Remote operator control through Gateway and Teleop",
        "slam_profile": "none",
        "llm": "mock",
        "planner": "direct",
        "enable_navigation": False,
        "enable_native": False,
        "enable_semantic": False,
        "enable_gateway": True,
        "enable_teleop": True,
        "enable_map_modules": False,
        "enable_goals": False,
        "enable_patrol_routes": False,
        "enable_scheduler": False,
        "enable_gnss": False,
        "run_startup_checks": False,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
    "teleop_avoid": {
        "_desc": "Native assisted teleoperation with obstacle avoidance",
        "slam_profile": "fastlio2",
        "llm": "mock",
        "planner": "direct",
        "enable_navigation": False,
        "enable_native": False,
        "enable_semantic": False,
        "enable_gateway": True,
        "enable_teleop": True,
        # Live obstacle maps come from native mapd over HostBus. Assisted
        # teleoperation has no saved-map control/query responsibility.
        "enable_map_modules": False,
        "cmd_vel_mux_collision_monitor": True,
        "enable_goals": False,
        "enable_patrol_routes": False,
        "enable_scheduler": False,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "teleop_planner_horizon_m": 2.0,
        "teleop_planner_max_deviation_deg": 55.0,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
    "tracking": {
        "_desc": "Follow explicit map-frame goals without semantic selection",
        "slam_profile": "localizer",
        "llm": "mock",
        "planner": "octoplanner3d",
        "map_path": ACTIVE_OCTOPLANNER3D_MAP,
        "plan_safety_policy": "reject",
        "fallback_planner_name": "",
        "enable_native": False,
        "enable_semantic": False,
        "enable_gateway": True,
        "enable_teleop": False,
        "enable_map_modules": True,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    },
}

FIELD_PRODUCT_VARIANT_HOST_DEFAULTS: dict[str, dict[str, dict[str, Any]]] = {
    "explore": {
        "live": FIELD_PRODUCT_HOST_DEFAULTS["explore"],
        "map": {
            "_desc": "Explore a saved map using the native TARE-style policy",
            "slam_profile": "localizer",
            "llm": "qwen",
            "planner": "octoplanner3d",
            "map_path": ACTIVE_OCTOPLANNER3D_MAP,
            "map_artifact_gate_required": True,
            "plan_safety_policy": "reject",
            "fallback_planner_name": "",
            **THUNDER_OCTOPLANNER_DEFAULTS,
            "enable_native": False,
            "enable_semantic": True,
            "enable_gateway": True,
            "enable_map_modules": True,
            "enable_frontier": False,
            "enable_traversable_frontier": False,
            "exploration_backend": "tare",
            "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
            "gateway_port": DEFAULT_GATEWAY_PORT,
        },
    }
}

if tuple(FIELD_PRODUCT_HOST_DEFAULTS) != FIELD_PRODUCT_NAMES:
    missing = sorted(set(FIELD_PRODUCT_NAMES) - set(FIELD_PRODUCT_HOST_DEFAULTS))
    extra = sorted(set(FIELD_PRODUCT_HOST_DEFAULTS) - set(FIELD_PRODUCT_NAMES))
    raise RuntimeError(
        "Field Product Host defaults must match Runtime Graph exactly "
        f"(missing={missing}, extra={extra})"
    )


__all__ = [
    "FIELD_PRODUCT_HOST_DEFAULTS",
    "FIELD_PRODUCT_NAMES",
    "FIELD_PRODUCT_VARIANT_HOST_DEFAULTS",
]
