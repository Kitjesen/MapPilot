"""Odometry event handling for GatewayModule."""

from __future__ import annotations

import logging
import math
import time
from typing import Any

logger = logging.getLogger(__name__)


def handle_odometry(gw: Any, odom: Any) -> None:
    # SlamBridge applies map->odom TF before publishing to Gateway. Re-applying
    # the TF here would double-transform the frontend cursor.
    data = {
        "x": float(odom.x),
        "y": float(odom.y),
        "z": float(getattr(odom, "z", 0.0)),
        "yaw": float(getattr(odom, "yaw", 0.0)),
        "vx": odom.twist.linear.x if odom.twist else 0.0,
        "wz": odom.twist.angular.z if odom.twist else 0.0,
        "frame_id": str(getattr(odom, "frame_id", "") or "unknown"),
        "child_frame_id": str(getattr(odom, "child_frame_id", "") or "body"),
        "ts": odom.ts,
    }
    numeric_fields = ("x", "y", "z", "yaw", "vx", "wz", "ts")
    invalid_fields = [name for name in numeric_fields if not math.isfinite(float(data.get(name, 0.0)))]
    if invalid_fields:
        invalid_odometry = {
            "reason": "non_finite_odometry",
            "fields": invalid_fields,
            "ts": time.time(),
        }
        with gw._state_lock:
            gw._last_invalid_odometry = invalid_odometry
        logger.warning(
            "GatewayModule: quarantined non-finite odometry fields=%s",
            invalid_fields,
        )
        return

    gw._frame_tree.update_odometry(odom)
    with gw._state_lock:
        gw._runtime_cache.record_odometry(data, ts=time.time(), max_samples=20)
    gw._blackbox.record("odom", data)
    gw.push_event({"type": "odometry", "data": data})

    gw._slam_status_throttle += 1
    if gw._slam_status_throttle % 10 == 0:
        gw.push_event(
            {
                "type": "slam_status",
                "mode": gw._get_slam_profile(),
                "slam_hz": gw._get_slam_hz_cached(),
                "map_points": gw._cloud_viewer.cache_point_count(),
                "degeneracy_count": getattr(gw, "_degeneracy_count", 0),
            }
        )
