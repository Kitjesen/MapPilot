"""ctypes adapter for native C++ map-layer kernels.

The Python Module runtime may still pack/unpack message payloads, but map math
must execute inside ``lingtu_maps``.
"""

from __future__ import annotations

import ctypes
import ctypes.util
import math
import os
import sys
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path
from typing import Any

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


class MapKernelUnavailable(RuntimeError):
    """Raised when the native lingtu_maps grid-layer ABI is unavailable."""


class _GridSpec(ctypes.Structure):
    _fields_ = [
        ("rows", ctypes.c_int32),
        ("cols", ctypes.c_int32),
        ("resolution_m", ctypes.c_double),
        ("origin_x_m", ctypes.c_double),
        ("origin_y_m", ctypes.c_double),
    ]


class _TerrainRiskParams(ctypes.Structure):
    _fields_ = [
        ("max_slope_deg", ctypes.c_float),
        ("soft_slope_start_deg", ctypes.c_float),
        ("critical_step_m", ctypes.c_float),
        ("roughness_critical_m", ctypes.c_float),
    ]


class _TraversabilityParams(ctypes.Structure):
    _fields_ = [
        ("lethal", ctypes.c_float),
        ("inscribed", ctypes.c_float),
        ("max_slope_deg", ctypes.c_float),
        ("soft_slope_start_deg", ctypes.c_float),
        ("safe_distance_m", ctypes.c_float),
        ("proximity_cap", ctypes.c_float),
    ]


class _OccupancyCounts(ctypes.Structure):
    _fields_ = [
        ("unknown", ctypes.c_int64),
        ("free", ctypes.c_int64),
        ("occupied", ctypes.c_int64),
    ]


@dataclass
class Grid2D:
    rows: int = 0
    cols: int = 0
    resolution: float = 0.2
    origin_x: float = 0.0
    origin_y: float = 0.0
    data: Any = field(default_factory=list)

    def empty(self) -> bool:
        return self.rows <= 0 or self.cols <= 0 or len(self.data) == 0


@dataclass
class ElevationMapResult:
    min_z: Grid2D = field(default_factory=Grid2D)
    max_z: Grid2D = field(default_factory=Grid2D)
    clearance: Grid2D = field(default_factory=Grid2D)
    valid: Any = field(default_factory=list)


@dataclass
class EsdfResult:
    distance: Grid2D = field(default_factory=Grid2D)
    grad_x: Grid2D = field(default_factory=Grid2D)
    grad_y: Grid2D = field(default_factory=Grid2D)


@dataclass
class TerrainRiskParams:
    max_slope_deg: float = 35.0
    soft_slope_start_deg: float = 3.0
    critical_step_m: float = 0.22
    roughness_critical_m: float = 0.08


@dataclass
class TerrainRiskResult:
    risk: Grid2D = field(default_factory=Grid2D)
    slope_deg: Grid2D = field(default_factory=Grid2D)
    step_height: Grid2D = field(default_factory=Grid2D)
    roughness: Grid2D = field(default_factory=Grid2D)


@dataclass
class TraversabilityParams:
    lethal: float = 100.0
    inscribed: float = 99.0
    max_slope_deg: float = 35.0
    soft_slope_start_deg: float = 3.0
    safe_distance: float = 1.5
    proximity_cap: float = 50.0


@dataclass(frozen=True)
class MapKernelBackend:
    runtime: Any


def _library_names() -> tuple[str, ...]:
    if sys.platform.startswith("win"):
        return ("lingtu_maps.dll",)
    if sys.platform == "darwin":
        return ("liblingtu_maps.dylib", "lingtu_maps.dylib")
    return ("liblingtu_maps.so", "lingtu_maps.so")


def _native_library_candidates() -> list[Path]:
    env = os.environ.get("LINGTU_MAPS_LIB")
    candidates: list[Path] = [Path(env)] if env else []
    repo = Path(__file__).resolve().parents[4]
    roots = (
        repo / "build" / "maps",
        repo / "build" / "maps" / "Release",
        repo / "build" / "maps" / "Debug",
        repo / ".tmp" / "maps-cmake",
        repo / ".tmp" / "maps-cmake" / "Release",
        repo / ".tmp" / "maps-cmake" / "Debug",
    )
    for root in roots:
        for name in _library_names():
            candidates.append(root / name)
    return candidates


def _ptr(array: Any) -> ctypes.c_void_p:
    return ctypes.c_void_p(array.ctypes.data)


def _check_rc(rc: int, what: str) -> None:
    if int(rc) != 0:
        raise RuntimeError(f"lingtu_maps {what} failed: {int(rc)}")


class _NativeMapKernelRuntime:
    Grid2D = Grid2D
    ElevationMapResult = ElevationMapResult
    EsdfResult = EsdfResult
    TerrainRiskParams = TerrainRiskParams
    TerrainRiskResult = TerrainRiskResult
    TraversabilityParams = TraversabilityParams

    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._configure()

    def _configure(self) -> None:
        self._lib.lingtu_maps_abi_version.argtypes = []
        self._lib.lingtu_maps_abi_version.restype = ctypes.c_uint32

        self._lib.lingtu_maps_build_elevation_map.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.POINTER(_GridSpec),
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        self._lib.lingtu_maps_build_elevation_map.restype = ctypes.c_int32

        self._lib.lingtu_maps_compute_esdf.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_GridSpec),
            ctypes.c_float,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        self._lib.lingtu_maps_compute_esdf.restype = ctypes.c_int32

        self._lib.lingtu_maps_compute_terrain_risk.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(_GridSpec),
            ctypes.POINTER(_TerrainRiskParams),
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        self._lib.lingtu_maps_compute_terrain_risk.restype = ctypes.c_int32

        self._lib.lingtu_maps_fuse_traversability_cost.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(_GridSpec),
            ctypes.POINTER(_TraversabilityParams),
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        self._lib.lingtu_maps_fuse_traversability_cost.restype = ctypes.c_int32

        self._lib.lingtu_maps_build_occupancy_grid.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint32,
            ctypes.c_double,
            ctypes.POINTER(_GridSpec),
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(_OccupancyCounts),
            ctypes.c_uint64,
        ]
        self._lib.lingtu_maps_build_occupancy_grid.restype = ctypes.c_int32

        self._lib.lingtu_maps_resample_grid_bilinear.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_GridSpec),
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_float,
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        self._lib.lingtu_maps_resample_grid_bilinear.restype = ctypes.c_int32

        abi = int(self._lib.lingtu_maps_abi_version())
        if abi != 1:
            raise MapKernelUnavailable(f"unsupported lingtu_maps ABI version: {abi}")

    def resample_grid_bilinear(
        self,
        src: Any,
        *,
        src_origin: Any,
        src_resolution: float,
        dst_shape: tuple[int, int],
        dst_origin: Any,
        dst_resolution: float,
        fill: float = 0.0,
    ) -> Any:
        arr = np.ascontiguousarray(src, dtype=np.float32)
        if arr.ndim != 2:
            raise ValueError("resample source grid must be 2-D")
        dst_rows = int(dst_shape[0])
        dst_cols = int(dst_shape[1])
        if dst_rows <= 0 or dst_cols <= 0:
            raise ValueError("destination shape must be positive")
        spec = _GridSpec(
            int(arr.shape[0]),
            int(arr.shape[1]),
            float(src_resolution),
            float(src_origin[0]),
            float(src_origin[1]),
        )
        out = np.empty(dst_rows * dst_cols, dtype=np.float32)
        rc = self._lib.lingtu_maps_resample_grid_bilinear(
            _ptr(arr),
            ctypes.byref(spec),
            ctypes.c_int32(dst_rows),
            ctypes.c_int32(dst_cols),
            float(dst_resolution),
            float(dst_origin[0]),
            float(dst_origin[1]),
            ctypes.c_float(float(fill)),
            _ptr(out),
            ctypes.c_uint64(out.size),
        )
        _check_rc(rc, "resample_grid_bilinear")
        return out.reshape((dst_rows, dst_cols))

    def build_occupancy_grid(
        self,
        xyz: Any,
        *,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        resolution: float,
        radius: float,
        z_min: float,
        z_max: float,
        inflation_radius: float,
        robot_clear_radius: float,
        robot_clear_forward: float,
        robot_clear_backward: float,
        robot_clear_lateral: float,
        raycast_free_space: bool,
        unknown_as_obstacle_for_costmap: bool,
        raycast_max_rays: int,
        raycast_free_inflation_radius: float,
    ) -> dict[str, Any]:
        pts = np.ascontiguousarray(xyz, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] < 3:
            raise ValueError("occupancy input must be an Nx3 or Nx4 array")
        pts3 = np.ascontiguousarray(pts[:, :3], dtype=np.float32)
        size = int(math.floor((2.0 * float(radius)) / float(resolution)))
        if size <= 0:
            raise ValueError("computed occupancy grid is empty")
        count = size * size
        occupancy = np.empty(count, dtype=np.int8)
        cost = np.empty(count, dtype=np.float32)
        counts = _OccupancyCounts()
        spec = _GridSpec()
        rc = self._lib.lingtu_maps_build_occupancy_grid(
            _ptr(pts3),
            ctypes.c_uint64(pts3.shape[0]),
            float(robot_x),
            float(robot_y),
            float(robot_yaw),
            float(resolution),
            float(radius),
            float(z_min),
            float(z_max),
            float(inflation_radius),
            float(robot_clear_radius),
            float(robot_clear_forward),
            float(robot_clear_backward),
            float(robot_clear_lateral),
            ctypes.c_uint8(1 if raycast_free_space else 0),
            ctypes.c_uint8(1 if unknown_as_obstacle_for_costmap else 0),
            ctypes.c_uint32(max(1, int(raycast_max_rays))),
            float(raycast_free_inflation_radius),
            ctypes.byref(spec),
            _ptr(occupancy),
            _ptr(cost),
            ctypes.byref(counts),
            ctypes.c_uint64(count),
        )
        _check_rc(rc, "build_occupancy_grid")
        shape = (int(spec.rows), int(spec.cols))
        return {
            "occupancy": occupancy.reshape(shape),
            "cost": cost.reshape(shape),
            "resolution": float(spec.resolution_m),
            "origin": [float(spec.origin_x_m), float(spec.origin_y_m)],
            "counts": {
                "unknown": int(counts.unknown),
                "free": int(counts.free),
                "occupied": int(counts.occupied),
            },
        }

    def build_elevation_map(
        self,
        xyz_flat: Any,
        robot_x: float,
        robot_y: float,
        resolution: float,
        radius: float,
        z_floor: float,
        z_ceil: float,
    ) -> ElevationMapResult:
        xyz = np.ascontiguousarray(xyz_flat, dtype=np.float32)
        if xyz.size % 3 != 0:
            raise ValueError("xyz_flat must contain x/y/z triples")
        size = int(math.floor((2.0 * float(radius)) / float(resolution)))
        if size <= 0:
            raise ValueError("computed elevation grid is empty")
        count = size * size
        min_z = np.empty(count, dtype=np.float32)
        max_z = np.empty(count, dtype=np.float32)
        clearance = np.empty(count, dtype=np.float32)
        valid = np.empty(count, dtype=np.uint8)
        spec = _GridSpec()
        rc = self._lib.lingtu_maps_build_elevation_map(
            _ptr(xyz),
            ctypes.c_uint64(xyz.size // 3),
            float(robot_x),
            float(robot_y),
            float(resolution),
            float(radius),
            float(z_floor),
            float(z_ceil),
            ctypes.byref(spec),
            _ptr(min_z),
            _ptr(max_z),
            _ptr(clearance),
            _ptr(valid),
            ctypes.c_uint64(count),
        )
        _check_rc(rc, "build_elevation_map")
        return ElevationMapResult(
            min_z=_grid_from_array(min_z, spec),
            max_z=_grid_from_array(max_z, spec),
            clearance=_grid_from_array(clearance, spec),
            valid=valid,
        )

    def compute_esdf(
        self,
        occupancy: Grid2D,
        obstacle_threshold: float = 50.0,
    ) -> EsdfResult:
        spec = _spec_from_grid(occupancy)
        occ = _grid_array(occupancy)
        count = occ.size
        distance = np.empty(count, dtype=np.float32)
        grad_x = np.empty(count, dtype=np.float32)
        grad_y = np.empty(count, dtype=np.float32)
        rc = self._lib.lingtu_maps_compute_esdf(
            _ptr(occ),
            ctypes.byref(spec),
            ctypes.c_float(float(obstacle_threshold)),
            _ptr(distance),
            _ptr(grad_x),
            _ptr(grad_y),
            ctypes.c_uint64(count),
        )
        _check_rc(rc, "compute_esdf")
        return EsdfResult(
            distance=_grid_from_array(distance, spec),
            grad_x=_grid_from_array(grad_x, spec),
            grad_y=_grid_from_array(grad_y, spec),
        )

    def compute_terrain_risk(
        self,
        elevation: ElevationMapResult,
        params: TerrainRiskParams | None = None,
    ) -> TerrainRiskResult:
        spec = _spec_from_grid(elevation.max_z)
        count = int(spec.rows) * int(spec.cols)
        risk = np.empty(count, dtype=np.float32)
        slope = np.empty(count, dtype=np.float32)
        step = np.empty(count, dtype=np.float32)
        rough = np.empty(count, dtype=np.float32)
        c_params = _terrain_params(params or TerrainRiskParams())
        valid = np.ascontiguousarray(elevation.valid, dtype=np.uint8).reshape(-1)
        rc = self._lib.lingtu_maps_compute_terrain_risk(
            _ptr(_grid_array(elevation.min_z)),
            _ptr(_grid_array(elevation.max_z)),
            _ptr(_grid_array(elevation.clearance)),
            _ptr(valid),
            ctypes.byref(spec),
            ctypes.byref(c_params),
            _ptr(risk),
            _ptr(slope),
            _ptr(step),
            _ptr(rough),
            ctypes.c_uint64(count),
        )
        _check_rc(rc, "compute_terrain_risk")
        return TerrainRiskResult(
            risk=_grid_from_array(risk, spec),
            slope_deg=_grid_from_array(slope, spec),
            step_height=_grid_from_array(step, spec),
            roughness=_grid_from_array(rough, spec),
        )

    def fuse_traversability_cost(
        self,
        costmap: Grid2D,
        slope_deg: Grid2D,
        esdf_distance: Grid2D,
        terrain_risk: Grid2D,
        params: TraversabilityParams | None = None,
    ) -> Grid2D:
        spec = _spec_from_grid(costmap)
        cost = _grid_array(costmap)
        slope = None if slope_deg.empty() else _grid_array(slope_deg)
        esdf = None if esdf_distance.empty() else _grid_array(esdf_distance)
        terrain = None if terrain_risk.empty() else _grid_array(terrain_risk)
        out = np.empty(cost.size, dtype=np.float32)
        c_params = _traversability_params(params or TraversabilityParams())
        rc = self._lib.lingtu_maps_fuse_traversability_cost(
            _ptr(cost),
            _ptr(slope) if slope is not None else ctypes.c_void_p(),
            _ptr(esdf) if esdf is not None else ctypes.c_void_p(),
            _ptr(terrain) if terrain is not None else ctypes.c_void_p(),
            ctypes.byref(spec),
            ctypes.byref(c_params),
            _ptr(out),
            ctypes.c_uint64(out.size),
        )
        _check_rc(rc, "fuse_traversability_cost")
        return _grid_from_array(out, spec)


def _terrain_params(params: TerrainRiskParams) -> _TerrainRiskParams:
    return _TerrainRiskParams(
        float(params.max_slope_deg),
        float(params.soft_slope_start_deg),
        float(params.critical_step_m),
        float(params.roughness_critical_m),
    )


def _traversability_params(params: TraversabilityParams) -> _TraversabilityParams:
    return _TraversabilityParams(
        float(params.lethal),
        float(params.inscribed),
        float(params.max_slope_deg),
        float(params.soft_slope_start_deg),
        float(params.safe_distance),
        float(params.proximity_cap),
    )


def _grid_from_array(values: Any, spec: _GridSpec) -> Grid2D:
    arr = np.ascontiguousarray(values, dtype=np.float32)
    return Grid2D(
        rows=int(spec.rows),
        cols=int(spec.cols),
        resolution=float(spec.resolution_m),
        origin_x=float(spec.origin_x_m),
        origin_y=float(spec.origin_y_m),
        data=arr,
    )


def _spec_from_grid(grid: Grid2D) -> _GridSpec:
    if grid.empty():
        raise ValueError("grid is empty")
    return _GridSpec(
        int(grid.rows),
        int(grid.cols),
        float(grid.resolution),
        float(grid.origin_x),
        float(grid.origin_y),
    )


def _grid_array(grid: Grid2D) -> Any:
    arr = np.ascontiguousarray(grid.data, dtype=np.float32).reshape(-1)
    expected = int(grid.rows) * int(grid.cols)
    if arr.size != expected:
        raise ValueError("grid data size does not match rows*cols")
    return arr


@lru_cache(maxsize=1)
def _load_native_map_kernel() -> _NativeMapKernelRuntime | None:
    for candidate in _native_library_candidates():
        if candidate.is_file():
            try:
                return _NativeMapKernelRuntime(ctypes.CDLL(str(candidate)))
            except (OSError, AttributeError, MapKernelUnavailable):
                continue
    found = ctypes.util.find_library("lingtu_maps")
    if found:
        try:
            return _NativeMapKernelRuntime(ctypes.CDLL(found))
        except (OSError, AttributeError, MapKernelUnavailable):
            return None
    return None


def create_map_kernel_backend() -> MapKernelBackend | None:
    runtime = _load_native_map_kernel()
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
    grid.data = arr.ravel()
    return grid


def empty_grid2d(runtime: Any) -> Any:
    return runtime.Grid2D()


def elevation_result_to_payload(
    result: Any,
    *,
    ts: float,
    frame_id: str,
) -> dict[str, Any]:
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
        "backend": "cpp",
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
        "backend": "cpp",
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
    result.valid = valid.astype(np.uint8).ravel()
    return result


def terrain_risk_result_to_payload(
    result: Any,
    *,
    ts: float,
    frame_id: str,
) -> dict[str, Any]:
    return {
        "grid": grid_to_array(result.risk),
        "slope_deg": grid_to_array(result.slope_deg),
        "step_height": grid_to_array(result.step_height),
        "roughness": grid_to_array(result.roughness),
        "resolution": float(result.risk.resolution),
        "origin": [float(result.risk.origin_x), float(result.risk.origin_y)],
        "ts": ts,
        "frame_id": frame_id,
        "backend": "cpp",
    }
