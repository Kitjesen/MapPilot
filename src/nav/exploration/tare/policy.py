"""Small portable TARE-style exploration policy.

This is the LingTu-owned algorithm entrypoint. The CMU TARE project stays
outside this package as reference/ROS runtime code; normal modules should call
this file instead of depending on ROS topics, launch files, or RViz helpers.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from runtime.msgs.numpy_compat import np

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

_NEIGH4 = ((-1, 0), (1, 0), (0, -1), (0, 1))
_NEIGH8 = (
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1),
)


@dataclass(frozen=True)
class TAREPolicyConfig:
    min_frontier_size: int = 3
    sensor_range_m: float = 5.0
    candidate_radius_m: float = 2.5
    min_goal_distance_m: float = 0.8
    novelty_radius_m: float = 1.0
    max_candidates: int = 16


@dataclass(frozen=True)
class TAREDecision:
    goal: tuple[float, float, float] | None
    path: list[tuple[float, float, float]] = field(default_factory=list)
    reason: str = ""
    candidates: list[dict[str, Any]] = field(default_factory=list)
    done: bool = False


class PortableTAREPolicy:
    """Select the next exploration viewpoint from a LingTu exploration grid."""

    def __init__(self, config: TAREPolicyConfig | None = None) -> None:
        self.config = config or TAREPolicyConfig()

    def select(
        self,
        *,
        grid_payload: dict[str, Any],
        robot_xy: tuple[float, float],
        robot_yaw: float = 0.0,
        visited_goals: list[tuple[float, float]] | None = None,
    ) -> TAREDecision:
        grid, meta = _parse_grid(grid_payload)
        if grid is None:
            return TAREDecision(None, reason="missing_grid")

        start = _world_to_cell(robot_xy[0], robot_xy[1], meta)
        if start is None or not _free(grid, start[0], start[1]):
            return TAREDecision(None, reason="robot_not_in_free_space")

        reachable = _reachable_free(grid, start)
        frontier_cells = _frontier_cells(grid, reachable)
        if not frontier_cells:
            return TAREDecision(None, reason="no_frontiers", done=True)

        clusters = [
            cluster
            for cluster in _clusters(frontier_cells)
            if len(cluster) >= self.config.min_frontier_size
        ]
        if not clusters:
            return TAREDecision(None, reason="frontiers_below_min_size")

        visited = visited_goals or []
        candidates = [
            candidate
            for cluster in clusters
            if (
                candidate := self._candidate_for_cluster(
                    grid,
                    meta,
                    cluster,
                    reachable,
                    robot_xy,
                    robot_yaw,
                    visited,
                )
            )
            is not None
        ]
        candidates.sort(key=lambda item: item["score"], reverse=True)
        candidates = candidates[: self.config.max_candidates]
        if not candidates:
            return TAREDecision(None, reason="no_reachable_viewpoint")

        best = candidates[0]
        goal = (float(best["x"]), float(best["y"]), 0.0)
        path = [(float(robot_xy[0]), float(robot_xy[1]), 0.0), goal]
        return TAREDecision(goal, path=path, reason="selected_viewpoint", candidates=candidates)

    def _candidate_for_cluster(
        self,
        grid: Any,
        meta: dict[str, float],
        cluster: set[tuple[int, int]],
        reachable: set[tuple[int, int]],
        robot_xy: tuple[float, float],
        robot_yaw: float,
        visited: list[tuple[float, float]],
    ) -> dict[str, Any] | None:
        centroid = _cluster_centroid(cluster)
        frontier_xy = _cell_to_world(*centroid, meta)
        radius_cells = max(1, int(math.ceil(self.config.candidate_radius_m / meta["resolution"])))
        best: dict[str, Any] | None = None

        for row in range(max(0, centroid[0] - radius_cells), min(grid.shape[0], centroid[0] + radius_cells + 1)):
            for col in range(max(0, centroid[1] - radius_cells), min(grid.shape[1], centroid[1] + radius_cells + 1)):
                cell = (row, col)
                if cell not in reachable:
                    continue
                x, y = _cell_to_world(row, col, meta)
                distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
                if distance < self.config.min_goal_distance_m:
                    continue
                if any(math.hypot(x - vx, y - vy) < self.config.novelty_radius_m for vx, vy in visited):
                    continue
                coverage = _nearby_count(cell, cluster, self.config.sensor_range_m, meta["resolution"])
                if coverage <= 0:
                    continue
                heading = math.atan2(y - robot_xy[1], x - robot_xy[0])
                momentum = (1.0 + math.cos(_angle_delta(heading, robot_yaw))) * 0.5
                # ponytail: no LOS raycast here; add it only if field logs show
                # viewpoint choices behind walls/terrain.
                score = (
                    coverage * 1.0
                    - distance * 0.15
                    + momentum * 2.0
                    - math.hypot(x - frontier_xy[0], y - frontier_xy[1]) * 0.05
                )
                item = {
                    "x": x,
                    "y": y,
                    "score": float(score),
                    "frontier_size": int(len(cluster)),
                    "covered_frontier_cells": int(coverage),
                    "distance_m": float(distance),
                }
                if best is None or item["score"] > best["score"]:
                    best = item
        return best


def _parse_grid(payload: dict[str, Any]) -> tuple[Any | None, dict[str, float]]:
    grid = payload.get("grid") if isinstance(payload, dict) else None
    if grid is None:
        return None, {}
    arr = np.asarray(grid)
    if arr.ndim != 2:
        return None, {}
    resolution = float(payload.get("resolution") or 0.0)
    if resolution <= 0.0:
        return None, {}
    origin = payload.get("origin") or [payload.get("origin_x", 0.0), payload.get("origin_y", 0.0)]
    return arr, {
        "resolution": resolution,
        "origin_x": float(origin[0]),
        "origin_y": float(origin[1]),
    }


def _world_to_cell(x: float, y: float, meta: dict[str, float]) -> tuple[int, int] | None:
    res = meta["resolution"]
    row = int(math.floor((y - meta["origin_y"]) / res))
    col = int(math.floor((x - meta["origin_x"]) / res))
    return (row, col)


def _cell_to_world(row: int, col: int, meta: dict[str, float]) -> tuple[float, float]:
    res = meta["resolution"]
    return (
        meta["origin_x"] + (col + 0.5) * res,
        meta["origin_y"] + (row + 0.5) * res,
    )


def _free(grid: Any, row: int, col: int) -> bool:
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] and int(grid[row, col]) == FREE


def _reachable_free(grid: Any, start: tuple[int, int]) -> set[tuple[int, int]]:
    seen = {start}
    q: deque[tuple[int, int]] = deque([start])
    while q:
        row, col = q.popleft()
        for dr, dc in _NEIGH8:
            nxt = (row + dr, col + dc)
            if nxt not in seen and _free(grid, nxt[0], nxt[1]):
                seen.add(nxt)
                q.append(nxt)
    return seen


def _frontier_cells(grid: Any, reachable: set[tuple[int, int]]) -> set[tuple[int, int]]:
    frontiers: set[tuple[int, int]] = set()
    for row, col in reachable:
        for dr, dc in _NEIGH4:
            nr, nc = row + dr, col + dc
            if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1] and int(grid[nr, nc]) == UNKNOWN:
                frontiers.add((row, col))
                break
    return frontiers


def _clusters(cells: set[tuple[int, int]]) -> list[set[tuple[int, int]]]:
    remaining = set(cells)
    out: list[set[tuple[int, int]]] = []
    while remaining:
        start = remaining.pop()
        cluster = {start}
        q = deque([start])
        while q:
            row, col = q.popleft()
            for dr, dc in _NEIGH8:
                nxt = (row + dr, col + dc)
                if nxt in remaining:
                    remaining.remove(nxt)
                    cluster.add(nxt)
                    q.append(nxt)
        out.append(cluster)
    return out


def _cluster_centroid(cluster: set[tuple[int, int]]) -> tuple[int, int]:
    rows = [cell[0] for cell in cluster]
    cols = [cell[1] for cell in cluster]
    center = (sum(rows) / len(rows), sum(cols) / len(cols))
    return min(cluster, key=lambda cell: (cell[0] - center[0]) ** 2 + (cell[1] - center[1]) ** 2)


def _nearby_count(
    cell: tuple[int, int],
    cluster: set[tuple[int, int]],
    sensor_range_m: float,
    resolution: float,
) -> int:
    radius_sq = (sensor_range_m / max(resolution, 1e-6)) ** 2
    return sum((row - cell[0]) ** 2 + (col - cell[1]) ** 2 <= radius_sq for row, col in cluster)


def _angle_delta(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi
