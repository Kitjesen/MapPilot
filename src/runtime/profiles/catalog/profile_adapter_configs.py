"""Local Host Profile adapter override configs.

These dictionaries configure simulator and external-benchmark adapters. They
do not define Product identity or a communication endpoint.
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
    "gateway_port": DEFAULT_GATEWAY_PORT,
}
