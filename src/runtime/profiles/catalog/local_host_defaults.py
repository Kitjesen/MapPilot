"""Blueprint defaults for local, non-Product Host entrypoints."""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
)

LOCAL_HOST_DEFAULTS: dict[str, dict[str, Any]] = {
    "lite": {
        "_desc": "Local Thunder hardware diagnostic Host without SLAM or semantic modules",
        "runtime_mode": "lite",
        "slam_profile": "none",
        "llm": "mock",
        "planner": "direct",
        "enable_native": False,
        "python_autonomy_backend": "simple",
        "python_path_follower_backend": "pid",
        "enable_semantic": False,
        "enable_gateway": False,
        "enable_teleop": False,
        "enable_map_modules": False,
        "enable_gnss": False,
        "run_startup_checks": False,
        "planning_frame_id": DEFAULT_PLANNING_FRAME_ID,
        "gateway_port": DEFAULT_GATEWAY_PORT,
    }
}

LOCAL_PROFILE_NAMES = tuple(LOCAL_HOST_DEFAULTS)


__all__ = ["LOCAL_HOST_DEFAULTS", "LOCAL_PROFILE_NAMES"]
