"""Shared path safety checks for global planning and simulation gates."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal, Sequence

from runtime.msgs.numpy_compat import np

GridIndexOrder = Literal["yx", "xy"]
MAX_ELEVATION_NOMINAL_DELTA_M = 10.0


@dataclass(frozen=True)
class PlanSafetyGrid:
    """2D traversability grid with an explicit world-to-cell convention.

    `yx` means `grid[row_y, col_x]`, which is the convention used by the
    planner backends. `xy` means `grid[ix, iy]`, which is used by some raw
    generated tomogram artifacts before backend normalization.
    """

    grid: np.ndarray
    resolution: float
    origin: tuple[float, float]
    index_order: GridIndexOrder = "yx"
    trav_3d: np.ndarray | None = None
    elev_3d: np.ndarray | None = None
    slice_h0: float = 0.0
    slice_dh: float = 0.5
    use_grid_overlay: bool = False

    def cost_at(self, x: float, y: float, z: float | None = None) -> float:
        return float(self.sample_cost(x, y, z)["cost"])

    def sample_cost(self, x: float, y: float, z: float | None = None) -> dict[str, Any]:
        col = int(round((float(x) - self.origin[0]) / self.resolution))
        row = int(round((float(y) - self.origin[1]) / self.resolution))
        if self.trav_3d is not None and z is not None:
            if self.slice_dh == 0:
                layer = 0
            else:
                layer = int(round((float(z) - self.slice_h0) / self.slice_dh))
            if row < 0 or col < 0 or row >= self.trav_3d.shape[1] or col >= self.trav_3d.shape[2]:
                return {
                    "cost": float("inf"),
                    "row": row,
                    "col": col,
                    "layer": layer,
                    "reason": "xy_out_of_bounds",
                }
            elev = self.elev_3d
            if elev is not None and elev.shape == self.trav_3d.shape:
                column_heights = np.asarray(elev[:, row, col], dtype=float)
                nominal_heights = self.slice_h0 + np.arange(self.trav_3d.shape[0]) * self.slice_dh
                finite = (
                    np.isfinite(column_heights)
                    & (np.abs(column_heights - nominal_heights) <= MAX_ELEVATION_NOMINAL_DELTA_M)
                )
                if np.any(finite):
                    finite_indices = np.where(finite)[0]
                    nearest_pos = int(np.argmin(np.abs(column_heights[finite] - float(z))))
                    nearest_layer = int(finite_indices[nearest_pos])
                    nearest_error = abs(float(column_heights[nearest_layer]) - float(z))
                    height_tolerance = max(abs(float(self.slice_dh)), float(self.resolution), 0.5)
                    if nearest_error <= height_tolerance:
                        layer = nearest_layer
            if layer < 0 and float(z) >= self.slice_h0 - abs(self.slice_dh):
                layer = 0
            elif layer >= self.trav_3d.shape[0] and float(z) <= (
                self.slice_h0 + (self.trav_3d.shape[0] - 1) * self.slice_dh + abs(self.slice_dh)
            ):
                layer = self.trav_3d.shape[0] - 1
            if layer < 0 or layer >= self.trav_3d.shape[0]:
                return {
                    "cost": float("inf"),
                    "row": row,
                    "col": col,
                    "layer": layer,
                    "reason": "z_out_of_bounds",
                }
            cost = float(self.trav_3d[layer, row, col])
            if self.use_grid_overlay:
                overlay_cost = self._grid_cost(row, col)
                if math.isfinite(overlay_cost):
                    cost = max(cost, overlay_cost)
            return {
                "cost": cost,
                "row": row,
                "col": col,
                "layer": layer,
                "reason": "ok",
            }
        if self.index_order == "xy":
            ix, iy = col, row
            if ix < 0 or iy < 0 or ix >= self.grid.shape[0] or iy >= self.grid.shape[1]:
                return {
                    "cost": float("inf"),
                    "row": row,
                    "col": col,
                    "layer": None,
                    "reason": "xy_out_of_bounds",
                }
        elif row < 0 or col < 0 or row >= self.grid.shape[0] or col >= self.grid.shape[1]:
            return {
                "cost": float("inf"),
                "row": row,
                "col": col,
                "layer": None,
                "reason": "xy_out_of_bounds",
            }
        return {
            "cost": self._grid_cost(row, col),
            "row": row,
            "col": col,
            "layer": None,
            "reason": "ok",
        }

    def _grid_cost(self, row: int, col: int) -> float:
        if self.index_order == "xy":
            ix, iy = col, row
            if ix < 0 or iy < 0 or ix >= self.grid.shape[0] or iy >= self.grid.shape[1]:
                return float("inf")
            return float(self.grid[ix, iy])
        if row < 0 or col < 0 or row >= self.grid.shape[0] or col >= self.grid.shape[1]:
            return float("inf")
        return float(self.grid[row, col])


def path_distance(path: Sequence[Sequence[float]]) -> float:
    if len(path) < 2:
        return 0.0
    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def path_samples(
    path: Sequence[Sequence[float]],
    *,
    max_step_m: float,
) -> list[tuple[float, float]]:
    if not path:
        return []
    xy = [(float(p[0]), float(p[1])) for p in path]
    samples: list[tuple[float, float]] = [xy[0]]
    for start, end in zip(xy, xy[1:]):
        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(dist / max(max_step_m, 1e-6))))
        for idx in range(1, steps + 1):
            alpha = idx / steps
            samples.append(
                (
                    start[0] + (end[0] - start[0]) * alpha,
                    start[1] + (end[1] - start[1]) * alpha,
                )
            )
    return samples


def _path_samples_xyz(
    path: Sequence[Sequence[float]],
    *,
    max_step_m: float,
) -> list[tuple[float, float, float | None]]:
    if not path:
        return []
    pts = [
        (
            float(p[0]),
            float(p[1]),
            float(p[2]) if len(p) > 2 and math.isfinite(float(p[2])) else None,
        )
        for p in path
    ]
    samples: list[tuple[float, float, float | None]] = [pts[0]]
    for start, end in zip(pts, pts[1:]):
        dist = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, int(math.ceil(dist / max(max_step_m, 1e-6))))
        for idx in range(1, steps + 1):
            alpha = idx / steps
            if start[2] is None or end[2] is None:
                z = None
            else:
                z = start[2] + (end[2] - start[2]) * alpha
            samples.append(
                (
                    start[0] + (end[0] - start[0]) * alpha,
                    start[1] + (end[1] - start[1]) * alpha,
                    z,
                )
            )
    return samples


def evaluate_plan_safety(
    path: Sequence[Sequence[float]],
    grid: PlanSafetyGrid,
    *,
    obstacle_thr: float,
    max_step_m: float | None = None,
    blocked_sample_limit: int = 10,
) -> dict[str, Any]:
    if not path:
        return {
            "ok": False,
            "max_cost": None,
            "blocked_sample_count": 0,
            "blocked_samples": [],
            "sample_count": 0,
            "resolution": float(grid.resolution),
            "index_order": grid.index_order,
        }
    step = float(max_step_m) if max_step_m is not None else max(float(grid.resolution) * 0.5, 0.05)
    samples = _path_samples_xyz(path, max_step_m=step)
    cost_reports = [grid.sample_cost(x, y, z) for x, y, z in samples]
    costs = [float(report["cost"]) for report in cost_reports]
    blocked = [
        {
            "x": round(float(samples[idx][0]), 4),
            "y": round(float(samples[idx][1]), 4),
            "z": (
                round(float(samples[idx][2]), 4)
                if samples[idx][2] is not None
                else None
            ),
            "cost": round(float(cost), 4) if math.isfinite(float(cost)) else float("inf"),
            "row": cost_reports[idx].get("row"),
            "col": cost_reports[idx].get("col"),
            "layer": cost_reports[idx].get("layer"),
            "reason": (
                cost_reports[idx].get("reason")
                if not math.isfinite(float(cost))
                else "cost_ge_threshold"
            ),
        }
        for idx, cost in enumerate(costs)
        if (not math.isfinite(float(cost))) or float(cost) >= obstacle_thr
    ]
    finite_costs = [float(cost) for cost in costs if math.isfinite(float(cost))]
    return {
        "ok": not blocked,
        "max_cost": round(float(max(finite_costs)), 4) if finite_costs else float("inf"),
        "blocked_sample_count": len(blocked),
        "blocked_samples": blocked[:blocked_sample_limit],
        "sample_count": len(samples),
        "resolution": float(grid.resolution),
        "index_order": grid.index_order,
    }


def grid_from_backend(backend: Any) -> PlanSafetyGrid | None:
    grid = getattr(backend, "_grid", None)
    if grid is None or not hasattr(grid, "shape"):
        return None
    arr = np.asarray(grid, dtype=np.float32)
    if arr.ndim != 2:
        return None
    origin = getattr(backend, "_origin", (0.0, 0.0))
    resolution = float(getattr(backend, "_resolution", 0.2))
    trav_3d = getattr(backend, "_trav_3d", None)
    elev_3d = getattr(backend, "_elev_3d", None)
    trav_arr = None
    elev_arr = None
    if trav_3d is not None and hasattr(trav_3d, "shape"):
        arr3 = np.asarray(trav_3d, dtype=np.float32)
        if arr3.ndim == 3:
            trav_arr = arr3
    if elev_3d is not None and hasattr(elev_3d, "shape"):
        elev3 = np.asarray(elev_3d, dtype=np.float32)
        if elev3.ndim == 3:
            elev_arr = elev3
    return PlanSafetyGrid(
        grid=arr,
        resolution=resolution,
        origin=(float(origin[0]), float(origin[1])),
        index_order="yx",
        trav_3d=trav_arr,
        elev_3d=elev_arr,
        slice_h0=float(getattr(backend, "_slice_h0", 0.0)),
        slice_dh=float(getattr(backend, "_slice_dh", 0.5)),
        use_grid_overlay=bool(getattr(backend, "_grid_is_projection", False)),
    )


def grid_from_tomogram(tomogram: dict[str, Any]) -> PlanSafetyGrid:
    grid_info = tomogram.get("grid_info") or {}
    axis_order = str(grid_info.get("axis_order") or "")
    data = tomogram.get("data")
    if data is not None:
        arr = np.asarray(data, dtype=np.float32)
        if arr.ndim == 4:
            grid = np.asarray(arr[0, 0], dtype=np.float32)
        elif arr.ndim == 3:
            grid = np.asarray(arr[0], dtype=np.float32)
        elif arr.ndim == 2:
            grid = arr
        else:
            raise ValueError(f"unsupported tomogram data shape: {arr.shape}")
    else:
        raw_grid = tomogram.get("grid", tomogram.get("traversability"))
        if raw_grid is None:
            raise ValueError("tomogram has no data/grid/traversability")
        grid = np.asarray(raw_grid, dtype=np.float32)
    origin = tomogram.get("origin")
    index_order: GridIndexOrder = "xy"
    if axis_order in {"row_y_col_x", "yx"}:
        index_order = "yx"
    elif axis_order in {"builder_xy", "xy"}:
        index_order = "xy"
    if origin is None:
        center = np.asarray(tomogram.get("center", [0.0, 0.0])[:2], dtype=float)
        res = float(tomogram.get("resolution", 0.2))
        h, w = grid.shape
        origin_arr = center - np.asarray([w * res / 2.0, h * res / 2.0], dtype=float)
        origin = [float(origin_arr[0]), float(origin_arr[1])]
        if axis_order not in {"builder_xy", "xy"}:
            index_order = "yx"
    return PlanSafetyGrid(
        grid=np.asarray(grid, dtype=np.float32),
        resolution=float(tomogram.get("resolution", 0.2)),
        origin=(float(origin[0]), float(origin[1])),
        index_order=index_order,
    )


def evaluate_backend_path_safety(
    path: Sequence[Sequence[float]],
    backend: Any,
    *,
    obstacle_thr: float,
) -> dict[str, Any] | None:
    grid = grid_from_backend(backend)
    if grid is None:
        return None
    return evaluate_plan_safety(path, grid, obstacle_thr=obstacle_thr)
