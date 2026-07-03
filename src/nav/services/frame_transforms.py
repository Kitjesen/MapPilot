"""Frame transform helpers shared by navigation execution modules."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from runtime.msgs.geometry import Quaternion, Vector3
from runtime.runtime_interface import normalize_frame_id

MAP_FRAME_JUMP_TRANSLATION_EPS_M = 0.05
MAP_FRAME_JUMP_YAW_EPS_DEG = 2.0


def is_map_frame_jump_event(event: Mapping[str, Any] | None) -> bool:
    """Return true only for explicit map-frame discontinuity events.

    Several runtime status payloads are ``dict`` values and may share a
    transport or loose autoconnect surface with event ports.  Consumers must
    not clear motion state just because a normal map->odom TF/status dict
    arrived on the wrong input.
    """

    if not isinstance(event, Mapping):
        return False
    event_type = str(event.get("type") or event.get("event") or "").strip().lower()

    # Normal TF payloads are status, not events.
    if (
        event.get("valid") is not None
        or event.get("frame_id") is not None
        or event.get("child_frame_id") is not None
        or any(key in event for key in ("tx", "ty", "tz", "qx", "qy", "qz", "qw"))
    ):
        return False

    if any(key in event for key in ("dt_m", "dyaw_deg")):
        try:
            dt_m = abs(float(event.get("dt_m", 0.0) or 0.0))
        except (TypeError, ValueError):
            dt_m = 0.0
        try:
            dyaw_deg = abs(float(event.get("dyaw_deg", 0.0) or 0.0))
        except (TypeError, ValueError):
            dyaw_deg = 0.0
        return (
            dt_m > MAP_FRAME_JUMP_TRANSLATION_EPS_M
            or dyaw_deg > MAP_FRAME_JUMP_YAW_EPS_DEG
        )
    if event_type in {"map_frame_jump", "map_odom_jump", "relocalization_jump"}:
        return True
    if event.get("map_frame_jump") is True:
        return True
    if any(key in event for key in ("prev", "next")):
        return True

    return False


def transform_xyz_yaw_with_map_odom(
    position: list[float] | tuple[float, float, float],
    yaw: float | None,
    *,
    source_frame: str | None,
    target_frame: str | None,
    map_odom_tf: Mapping[str, Any] | None,
) -> tuple[list[float], float | None, str | None, bool, str]:
    """Transform an odom-frame pose into a target frame using map->odom TF.

    Returns ``(position, yaw, output_frame, transformed, reason)``.  The helper
    intentionally supports only the map<->odom edge used by the navigation
    runtime; broader frame-tree lookups belong in ``runtime.tf`` adapters.
    """

    pos = [float(position[0]), float(position[1]), float(position[2])]
    source = normalize_frame_id(source_frame)
    target = normalize_frame_id(target_frame)
    if not source or not target:
        return pos, yaw, source, False, "missing_frame"
    if source == target:
        return pos, yaw, target, False, "same_frame"
    if not isinstance(map_odom_tf, Mapping) or map_odom_tf.get("valid") is False:
        return pos, yaw, source, False, "missing_map_odom_tf"

    parent = normalize_frame_id(map_odom_tf.get("frame_id"))
    child = normalize_frame_id(map_odom_tf.get("child_frame_id"))
    if parent == target and child == source:
        inverse = False
    elif parent == source and child == target:
        inverse = True
    else:
        return pos, yaw, source, False, "map_odom_tf_frame_mismatch"

    try:
        rotation = Quaternion(
            float(map_odom_tf.get("qx", 0.0)),
            float(map_odom_tf.get("qy", 0.0)),
            float(map_odom_tf.get("qz", 0.0)),
            float(map_odom_tf.get("qw", 1.0)),
        ).normalize()
        translation = Vector3(
            float(map_odom_tf.get("tx", 0.0)),
            float(map_odom_tf.get("ty", 0.0)),
            float(map_odom_tf.get("tz", 0.0)),
        )
    except (TypeError, ValueError, ZeroDivisionError):
        return pos, yaw, source, False, "invalid_map_odom_tf"

    vector = Vector3(pos)
    if inverse:
        converted = rotation.inverse().rotate_vector(vector - translation)
        yaw_delta = -rotation.yaw
    else:
        converted = translation + rotation.rotate_vector(vector)
        yaw_delta = rotation.yaw

    converted_yaw = yaw
    if yaw is not None and math.isfinite(float(yaw)):
        converted_yaw = math.atan2(
            math.sin(float(yaw) + yaw_delta),
            math.cos(float(yaw) + yaw_delta),
        )
    return converted.to_list(), converted_yaw, target, True, "map_odom_tf"
