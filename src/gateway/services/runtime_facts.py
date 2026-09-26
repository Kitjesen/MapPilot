"""One lock-consistent read of Gateway runtime facts."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from gateway.services.native_control import teleop_active as native_teleop_active
from gateway.services.native_status import read_navigation_status


def capture_runtime_facts(gw: Any) -> dict[str, Any]:
    """Capture the mutable facts shared by Gateway status projections."""

    now = time.time()
    # Read the native snapshot once and pass it through the projection.  The
    # status file is independent of the Host lock, so parsing it while holding
    # that lock only makes unrelated callbacks wait.
    native_endpoint_status = read_navigation_status()
    teleop_is_active = native_teleop_active(native_endpoint_status)
    with gw._state_lock:
        navigation_state = getattr(gw, "_navigation_state", None)
        localization_status = getattr(gw, "_localization_status", None)
        visual_servo_status = getattr(gw, "_visual_servo_status", None)
        return {
            "ts": now,
            "odometry": gw._odom,
            "odometry_received_at": (
                float(gw._odom_timestamps[-1])
                if getattr(gw, "_odom_timestamps", None)
                else None
            ),
            "navigation_state": (
                dict(navigation_state) if isinstance(navigation_state, Mapping) else None
            ),
            "localization_status": (
                dict(localization_status) if isinstance(localization_status, Mapping) else None
            ),
            "navigation_goal_status_by_task": dict(
                getattr(gw, "_navigation_goal_status_by_task", {})
            ),
            "navigation_goal_status_by_request": dict(
                getattr(gw, "_navigation_goal_status_by_request", {})
            ),
            "native_endpoint_status": native_endpoint_status,
            "mode": str(getattr(gw, "_mode", "") or "").strip().lower(),
            "teleop_active": bool(teleop_is_active),
            "scene_graph_json": gw._sg_json,
            "path_len": len(gw._last_path),
            "visual_servo_status": (
                dict(visual_servo_status)
                if isinstance(visual_servo_status, Mapping)
                else visual_servo_status
            ),
            "icp_quality": float(getattr(gw, "_icp_quality", 0.0)),
        }


__all__ = ["capture_runtime_facts"]
