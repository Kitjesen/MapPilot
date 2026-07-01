"""Thin adapter for C++ L2 map kernels.

The algorithms live in ``nav_kernel/map_layers_core.hpp``. This file only
converts runtime numpy arrays and dict payloads to/from ``lingtu_nav_kernel`` types.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from nav.kernel import try_import_nav_kernel
from runtime.msgs.numpy_compat import np


MAP_KERNEL_SYMBOLS = (
    "Grid2D",
    "ElevationMapResult",
    "EsdfResult",
    "TerrainRiskParams",
    "TraversabilityParams",
    "build_elevation_map",
    "compute_esdf",
    "compute_terrain_risk",
    "fuse_traversability_cost",
)


@dataclass(frozen=True)
class MapKernelBackend:
    runtime: Any


def create_map_kernel_backend() -> MapKernelBackend | None:
    runtime = try_import_nav_kernel(MAP_KERNEL_SYMBOLS)
    return MapKernelBackend(runtime=runtime) if runtime is not None else None


def grid_to_array(grid: Any, *, dtype: Any = np.float32) -> Any:
    return np.asarray(grid.data, dtype=dtype).reshape(int(grid.rows), int(grid.cols))


def valid_to_array(valid: Any, rows: int, cols: int) -> Any:
    return np.asarray(valid, dtype=np.uint8).reshape(int(rows), int(cols)).astype(bool)


def make_grid2d(runtime: Any, array: Any, *, resolution: float, origin: Any) -> Any:
    arr = np.ascontiguousarray(array, dtype=np.float32)
    if arr.ndim != 2:
        raise ValueError("Grid2D payload must be 2-D")
    grid = runtime.Grid2D()
    grid.rows = int(arr.shape[0])
    grid.cols = int(arr.shape[1])
    grid.resolution = float(resolution)
    grid.origin_x = float(origin[0])
    grid.origin_y = float(origin[1])
    grid.data = arr.ravel().tolist()
    return grid


def empty_grid2d(runtime: Any) -> Any:
    return runtime.Grid2D()


def elevation_result_to_payload(result: Any, *, ts: float, frame_id: str) -> dict[str, Any]:
    rows = int(result.max_z.rows)
    cols = int(result.max_z.cols)
    return {
        "min_z": grid_to_array(result.min_z),
        "max_z": grid_to_array(result.max_z),
        "clearance": grid_to_array(result.clearance),
        "valid": valid_to_array(result.valid, rows, cols),
        "resolution": float(result.max_z.resolution),
        "origin": [float(result.max_z.origin_x), float(result.max_z.origin_y)],
        "ts": ts,
        "frame_id": frame_id,
        "backend": "nav_kernel",
    }


def esdf_result_to_payload(result: Any, *, ts: float, frame_id: str) -> dict[str, Any]:
    return {
        "distance_field": grid_to_array(result.distance),
        "grad_x": grid_to_array(result.grad_x),
        "grad_y": grid_to_array(result.grad_y),
        "resolution": float(result.distance.resolution),
        "origin": [float(result.distance.origin_x), float(result.distance.origin_y)],
        "ts": ts,
        "frame_id": frame_id,
        "backend": "nav_kernel",
    }


def elevation_payload_to_result(runtime: Any, payload: dict[str, Any]) -> Any:
    max_z = np.asarray(payload["max_z"], dtype=np.float32)
    valid = np.asarray(payload["valid"], dtype=bool)
    resolution = float(payload.get("resolution", 0.2))
    origin = payload.get("origin", [0.0, 0.0])

    result = runtime.ElevationMapResult()
    result.min_z = make_grid2d(
        runtime,
        np.asarray(payload.get("min_z", max_z), dtype=np.float32),
        resolution=resolution,
        origin=origin,
    )
    result.max_z = make_grid2d(runtime, max_z, resolution=resolution, origin=origin)
    result.clearance = make_grid2d(
        runtime,
        np.asarray(payload.get("clearance", np.zeros_like(max_z)), dtype=np.float32),
        resolution=resolution,
        origin=origin,
    )
    if valid.shape != max_z.shape:
        raise ValueError("elevation valid mask shape does not match max_z")
    result.valid = valid.astype(np.uint8).ravel().tolist()
    return result


def terrain_risk_result_to_payload(result: Any, *, ts: float, frame_id: str) -> dict[str, Any]:
    return {
        "grid": grid_to_array(result.risk),
        "slope_deg": grid_to_array(result.slope_deg),
        "step_height": grid_to_array(result.step_height),
        "roughness": grid_to_array(result.roughness),
        "resolution": float(result.risk.resolution),
        "origin": [float(result.risk.origin_x), float(result.risk.origin_y)],
        "ts": ts,
        "frame_id": frame_id,
        "backend": "nav_kernel",
    }
