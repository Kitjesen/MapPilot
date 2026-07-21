"""Obstacle cloud assembly for local planning."""

from __future__ import annotations

from typing import Any

from runtime.msgs.numpy_compat import np

BOUNDARY_INTENSITY = 100.0
OBSTACLE_INTENSITY = 200.0
TRAVERSABILITY_OBSTACLE_THRESHOLD = 90.0
MAX_TRAVERSABILITY_OBSTACLES = 5000


def traversability_grid_from_payload(
    traversability: dict[str, Any] | None,
) -> tuple[np.ndarray, float, np.ndarray] | None:
    if not isinstance(traversability, dict):
        return None

    raw = None
    for key in ("grid", "traversability", "cost", "costmap"):
        value = traversability.get(key)
        if value is not None:
            raw = value
            break
    if raw is None:
        return None

    try:
        grid = np.asarray(raw, dtype=np.float32)
        resolution = float(traversability["resolution"])
        origin = np.asarray(traversability["origin"][:2], dtype=np.float32)
    except (KeyError, TypeError, ValueError, IndexError):
        return None

    if grid.ndim != 2 or resolution <= 0.0 or origin.shape[0] < 2:
        return None
    grid = np.nan_to_num(grid, nan=0.0, posinf=100.0, neginf=0.0)
    return np.clip(grid, 0.0, 100.0).astype(np.float32, copy=False), resolution, origin


def traversability_obstacle_points(
    traversability: dict[str, Any] | None,
    *,
    robot_position: Any,
    max_range_m: float,
    threshold: float = TRAVERSABILITY_OBSTACLE_THRESHOLD,
    max_points: int = MAX_TRAVERSABILITY_OBSTACLES,
) -> np.ndarray:
    payload = traversability_grid_from_payload(traversability)
    if payload is None:
        return np.zeros((0, 4), dtype=np.float32)
    grid, resolution, origin = payload

    try:
        robot = np.asarray(robot_position, dtype=np.float32)
    except (TypeError, ValueError):
        return np.zeros((0, 4), dtype=np.float32)

    if robot.shape[0] < 2:
        return np.zeros((0, 4), dtype=np.float32)

    rows, cols = np.where(grid >= float(threshold))
    if rows.size == 0:
        return np.zeros((0, 4), dtype=np.float32)

    xs = origin[0] + (cols.astype(np.float32) + 0.5) * resolution
    ys = origin[1] + (rows.astype(np.float32) + 0.5) * resolution

    if max_range_m > 0.0:
        dx = xs - robot[0]
        dy = ys - robot[1]
        keep = dx * dx + dy * dy <= float(max_range_m) * float(max_range_m)
        xs = xs[keep]
        ys = ys[keep]

    if xs.size == 0:
        return np.zeros((0, 4), dtype=np.float32)

    if max_points > 0 and xs.size > max_points:
        step = int(np.ceil(xs.size / max_points))
        xs = xs[::step][:max_points]
        ys = ys[::step][:max_points]

    z = float(robot[2]) if robot.shape[0] >= 3 else 0.0
    out = np.zeros((xs.size, 4), dtype=np.float32)
    out[:, 0] = xs
    out[:, 1] = ys
    out[:, 2] = z
    out[:, 3] = OBSTACLE_INTENSITY
    return out


def merge_obstacle_clouds(
    *,
    terrain_points: np.ndarray | None,
    terrain_ext_points: np.ndarray | None,
    traversability: dict[str, Any] | None = None,
    robot_position: Any = None,
    traversability_range_m: float = 3.5,
    boundary_points: np.ndarray | None = None,
    added_obstacle_points: np.ndarray | None = None,
    check_obstacle_enabled: bool = True,
) -> np.ndarray:
    if not check_obstacle_enabled:
        return np.zeros((0, 4), dtype=np.float32)

    clouds: list[np.ndarray] = []
    for points in (terrain_points, terrain_ext_points):
        if points is None or points.shape[0] <= 0:
            continue
        pts = points.astype(np.float32)
        if pts.shape[1] == 3:
            buf = np.zeros((len(pts), 4), dtype=np.float32)
            buf[:, :3] = pts
            clouds.append(buf)
        elif pts.shape[1] >= 4:
            clouds.append(pts[:, :4].astype(np.float32, copy=False))

    trav_points = traversability_obstacle_points(
        traversability,
        robot_position=robot_position,
        max_range_m=traversability_range_m,
    )
    if trav_points.shape[0] > 0:
        clouds.append(trav_points)

    if boundary_points is not None and boundary_points.shape[0] > 0:
        bpts = boundary_points.astype(np.float32)
        buf = np.zeros((len(bpts), 4), dtype=np.float32)
        buf[:, : min(3, bpts.shape[1])] = bpts[:, : min(3, bpts.shape[1])]
        buf[:, 3] = BOUNDARY_INTENSITY
        clouds.append(buf)

    if added_obstacle_points is not None and added_obstacle_points.shape[0] > 0:
        apts = added_obstacle_points.astype(np.float32)
        buf = np.zeros((len(apts), 4), dtype=np.float32)
        buf[:, : min(3, apts.shape[1])] = apts[:, : min(3, apts.shape[1])]
        buf[:, 3] = OBSTACLE_INTENSITY
        clouds.append(buf)

    if not clouds:
        return np.zeros((0, 4), dtype=np.float32)
    return np.concatenate(clouds, axis=0)
