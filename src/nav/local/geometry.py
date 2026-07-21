"""Geometry helpers for the local planner service."""

from __future__ import annotations

import math
from typing import Any

from runtime.msgs.geometry import PoseStamped
from runtime.msgs.numpy_compat import np


def coerce_path_point(point: Any) -> np.ndarray | None:
    if isinstance(point, PoseStamped):
        return np.array([point.x, point.y, point.z], dtype=float)
    pose = getattr(point, "pose", None)
    if pose is not None and hasattr(pose, "position"):
        pos = pose.position
        return np.array(
            [
                float(getattr(pos, "x", 0.0)),
                float(getattr(pos, "y", 0.0)),
                float(getattr(pos, "z", 0.0)),
            ],
            dtype=float,
        )
    if isinstance(point, dict):
        if "pose" in point and isinstance(point["pose"], dict):
            return coerce_path_point(point["pose"])
        if "position" in point:
            return coerce_path_point(point["position"])
        if "x" in point and "y" in point:
            return np.array(
                [
                    float(point.get("x", 0.0)),
                    float(point.get("y", 0.0)),
                    float(point.get("z", 0.0)),
                ],
                dtype=float,
            )
    try:
        arr = np.asarray(point, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if arr.size < 2 or not np.all(np.isfinite(arr[:2])):
        return None
    out = np.zeros(3, dtype=float)
    out[: min(arr.size, 3)] = arr[: min(arr.size, 3)]
    return out


def planning_origin(
    robot_pos: Any,
    robot_yaw: float,
    sensor_offset_x: float,
    sensor_offset_y: float,
) -> np.ndarray:
    cos_yaw = math.cos(float(robot_yaw))
    sin_yaw = math.sin(float(robot_yaw))
    origin = np.asarray(robot_pos, dtype=float).copy()
    origin[0] = origin[0] - cos_yaw * sensor_offset_x + sin_yaw * sensor_offset_y
    origin[1] = origin[1] - sin_yaw * sensor_offset_x - cos_yaw * sensor_offset_y
    return origin


def select_corridor_goal(
    global_path_points: np.ndarray | None,
    origin: np.ndarray,
    fallback_goal: np.ndarray,
    lookahead_m: float,
) -> np.ndarray:
    points = global_path_points
    if points is None or len(points) < 2:
        return fallback_goal
    robot_xy = origin[:2]
    if (
        points.shape[1] >= 3
        and len(origin) >= 3
        and np.isfinite(float(origin[2]))
        and np.all(np.isfinite(points[:, :3]))
    ):
        dists = np.linalg.norm(points[:, :3] - origin[:3], axis=1)
    else:
        dists = np.linalg.norm(points[:, :2] - robot_xy, axis=1)
    if not np.all(np.isfinite(dists)):
        return fallback_goal
    idx = int(np.argmin(dists))
    if idx >= len(points) - 1:
        return fallback_goal

    remaining = max(0.1, float(lookahead_m))
    cursor = points[idx].astype(float, copy=True)
    for next_idx in range(idx + 1, len(points)):
        target = points[next_idx].astype(float, copy=False)
        segment = target - cursor
        seg_len = float(np.linalg.norm(segment[:2]))
        if seg_len <= 1e-6:
            cursor = target.astype(float, copy=True)
            continue
        if remaining <= seg_len:
            return cursor + segment * (remaining / seg_len)
        remaining -= seg_len
        cursor = target.astype(float, copy=True)
    return fallback_goal


def result_path_xy_metrics(path: list[Any]) -> tuple[float, float]:
    if len(path) < 2:
        return 0.0, 0.0
    raw_xy = np.asarray([[float(v.x), float(v.y)] for v in path], dtype=float)
    xy_length = float(np.sum(np.linalg.norm(np.diff(raw_xy, axis=0), axis=1)))
    xy_span = float(np.linalg.norm(raw_xy[-1] - raw_xy[0]))
    return xy_length, xy_span


def straight_line(start: np.ndarray, goal: np.ndarray, step: float = 0.5) -> list[np.ndarray]:
    diff = goal - start
    dist = float(np.linalg.norm(diff))
    if dist < step:
        return [start, goal]
    n = max(int(dist / step), 2)
    return [start + diff * (i / n) for i in range(1, n + 1)]
