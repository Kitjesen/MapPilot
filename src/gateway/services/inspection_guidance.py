from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any


def build_inspection_guidance(
    task: Mapping[str, Any],
    localization: Mapping[str, Any],
    *,
    active_map_id: str | None,
    active_map_content_epoch: int | None,
) -> dict[str, Any]:
    """Project immutable task and localization facts into read-only HUD guidance."""

    identity = _identity(task)
    payload = {
        "schema_version": "lingtu.inspection.guidance.v1",
        "task_id": identity["task_id"],
        "identity": identity,
        "available": False,
        "reason": "",
        "point_id": "",
        "phase": str(task.get("phase") or ""),
        "goal": None,
        "robot_pose": None,
        "path": {
            "available": False,
            "reason": "execution_path_unbound",
            "points": [],
        },
    }

    point = _current_point(task)
    if point is not None:
        payload["point_id"] = str(point.get("id") or "")
        payload["goal"] = _goal(point)
    payload["robot_pose"] = _robot_pose(localization)

    reason = _unavailable_reason(
        task,
        localization,
        identity=identity,
        point=point,
        goal=payload["goal"],
        robot_pose=payload["robot_pose"],
        active_map_id=active_map_id,
        active_map_content_epoch=active_map_content_epoch,
    )
    payload["available"] = reason == ""
    payload["reason"] = reason
    return payload


def _identity(task: Mapping[str, Any]) -> dict[str, Any]:
    raw = task.get("identity")
    source = raw if isinstance(raw, Mapping) else {}
    return {
        "task_id": str(source.get("task_id") or task.get("task_id") or ""),
        "route_id": str(source.get("route_id") or ""),
        "route_revision": source.get("route_revision"),
        "map_id": str(source.get("map_id") or ""),
        "map_content_epoch": source.get("map_content_epoch"),
    }


def _current_point(task: Mapping[str, Any]) -> Mapping[str, Any] | None:
    route = task.get("route_snapshot")
    event = task.get("latest_event")
    if not isinstance(route, Mapping) or not isinstance(event, Mapping):
        return None
    points = route.get("points")
    point_index = event.get("point_index")
    if (
        not isinstance(points, list)
        or isinstance(point_index, bool)
        or not isinstance(point_index, int)
        or point_index < 0
        or point_index >= len(points)
    ):
        return None
    point = points[point_index]
    if not isinstance(point, Mapping) or point.get("enabled") is False:
        return None
    return point


def _goal(point: Mapping[str, Any]) -> dict[str, Any] | None:
    coordinates = _finite_values(point, ("x", "y", "z"))
    if coordinates is None or str(point.get("frame_id") or "") != "map":
        return None
    return {"frame_id": "map", **coordinates}


def _robot_pose(localization: Mapping[str, Any]) -> dict[str, Any] | None:
    if localization.get("pose_fresh") is not True:
        return None
    if not _localization_tracking(localization):
        return None
    odometry = localization.get("odometry")
    if not isinstance(odometry, Mapping):
        return None
    values = _finite_values(odometry, ("x", "y", "z", "yaw"))
    if values is None or str(odometry.get("frame_id") or "") != "map":
        return None
    return {"frame_id": "map", **values}


def _finite_values(
    source: Mapping[str, Any],
    fields: tuple[str, ...],
) -> dict[str, float] | None:
    values: dict[str, float] = {}
    for field in fields:
        raw = source.get(field)
        if isinstance(raw, bool):
            return None
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        values[field] = value
    return values


def _localization_tracking(localization: Mapping[str, Any]) -> bool:
    state = str(localization.get("state") or "").strip().lower()
    if state == "tracking":
        return True
    reported = str(localization.get("reported_state") or "").strip().upper()
    return state == "ready" and reported in {"TRACKING", "OK", "READY"}


def _unavailable_reason(
    task: Mapping[str, Any],
    localization: Mapping[str, Any],
    *,
    identity: Mapping[str, Any],
    point: Mapping[str, Any] | None,
    goal: object,
    robot_pose: object,
    active_map_id: str | None,
    active_map_content_epoch: int | None,
) -> str:
    if task.get("found") is False:
        return "task_not_found"
    if task.get("execution_confirmed") is not True:
        return "task_execution_unconfirmed"
    delivery = task.get("delivery")
    if not isinstance(delivery, Mapping) or delivery.get("continuity") != "verified":
        return "task_continuity_unverified"
    if task.get("terminal") is True:
        return "task_terminal"
    if task.get("current_state") == "PAUSED" or task.get("phase") == "PAUSED":
        return "task_paused"
    if point is None:
        return "current_point_unavailable"
    if goal is None:
        return "goal_geometry_unavailable"
    if not _localization_tracking(localization):
        return "localization_not_tracking"
    if localization.get("pose_fresh") is not True:
        return "localization_pose_stale"
    odometry = localization.get("odometry")
    if not isinstance(odometry, Mapping):
        return "robot_pose_unavailable"
    if str(odometry.get("frame_id") or "") != "map":
        return "robot_pose_frame_mismatch"
    if robot_pose is None:
        return "robot_pose_unavailable"
    if not active_map_id:
        return "active_map_unavailable"
    if active_map_id != identity.get("map_id"):
        return "active_map_mismatch"
    if active_map_content_epoch is None:
        return "active_map_epoch_unavailable"
    if active_map_content_epoch != identity.get("map_content_epoch"):
        return "active_map_epoch_mismatch"
    return ""
