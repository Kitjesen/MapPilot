"""Localization transform and reset helpers for GatewayModule."""

from __future__ import annotations

import logging
from typing import Any

from runtime.runtime_interface import map_frame_id, odom_frame_id
from runtime.tf import (
    map_from_odom_transform_from_mapping,
    map_from_odom_transform_to_dict,
)

logger = logging.getLogger(__name__)

def handle_map_odom_tf(gw: Any, tf: dict) -> None:
    """Cache canonical ``T_map_from_odom`` in FrameTree and matrix form."""
    transform = map_from_odom_transform_from_mapping(tf)
    if transform is None:
        gw._T_map_odom = None
        gw._has_map_odom_tf = False
        gw._frame_tree.remove_transform(
            map_frame_id(),
            odom_frame_id(),
            dynamic_only=True,
        )
        return

    try:
        gw._blackbox.record("tf", map_from_odom_transform_to_dict(transform))
        gw._frame_tree.set_transform(transform)
        gw._T_map_odom = transform.to_matrix()
        gw._has_map_odom_tf = True
    except Exception as e:
        gw._T_map_odom = None
        gw._has_map_odom_tf = False
        gw._frame_tree.remove_transform(
            transform.frame_id,
            transform.child_frame_id,
            dynamic_only=True,
        )
        logger.debug("gateway: _on_map_odom_tf parse failed: %s", e)


def clear_localization_runtime_cache(
    gw: Any,
    *,
    reason: str = "localization_restart",
) -> None:
    """Clear stale pose, map-cloud, and odometry UI state after localization reset."""
    with gw._state_lock:
        gw._runtime_cache.clear()
    clear_map_cache = getattr(gw, "clear_map_cloud_cache", None)
    if callable(clear_map_cache):
        clear_map_cache(reason=reason)
    else:
        gw._cloud_viewer.clear(reason=reason)
    gw.push_event(
        {
            "type": "odometry",
            "data": None,
            "reset": True,
            "reason": reason,
        }
    )
