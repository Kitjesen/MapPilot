"""Thin Python adapter for the ROS-free native FAR planner."""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

from nav.services.plan.contracts import (
    GlobalPlanRequest,
    GlobalPlanResult,
    GlobalPlanningMap,
    coerce_planning_map,
)
from runtime.msgs.numpy_compat import np

_ABI_VERSION = 1


class FarNativeUnavailable(RuntimeError):
    """Raised when the explicitly selected native FAR backend is unavailable."""


class _FarConfig(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("robot_radius_m", ctypes.c_double),
        ("obstacle_clearance_m", ctypes.c_double),
        ("max_visibility_distance_m", ctypes.c_double),
        ("unknown_cost_multiplier", ctypes.c_double),
        ("corner_separation_cells", ctypes.c_int32),
        ("snap_search_radius_cells", ctypes.c_int32),
        ("max_graph_nodes", ctypes.c_uint64),
        ("max_visibility_pairs", ctypes.c_uint64),
        ("max_search_expansions", ctypes.c_uint64),
        ("allow_unknown_fallback", ctypes.c_uint8),
        ("simplify_path", ctypes.c_uint8),
    ]


class _FarMap(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("width", ctypes.c_int32),
        ("height", ctypes.c_int32),
        ("resolution_m", ctypes.c_double),
        ("origin_x_m", ctypes.c_double),
        ("origin_y_m", ctypes.c_double),
        ("frame_id", ctypes.c_char_p),
        ("generation", ctypes.c_uint64),
        ("cells", ctypes.POINTER(ctypes.c_int8)),
        ("cell_count", ctypes.c_uint64),
        ("map_id", ctypes.c_char_p),
        ("map_version", ctypes.c_int64),
        ("artifact_sha256", ctypes.c_char_p),
    ]


class _FarPlanRequest(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("start_x", ctypes.c_double),
        ("start_y", ctypes.c_double),
        ("start_z", ctypes.c_double),
        ("goal_x", ctypes.c_double),
        ("goal_y", ctypes.c_double),
        ("goal_z", ctypes.c_double),
        ("expected_map_generation", ctypes.c_uint64),
        ("max_iterations", ctypes.c_int32),
        ("terminal_goal_tolerance_m", ctypes.c_double),
        ("terminal_goal_xy_tolerance_m", ctypes.c_double),
        ("terminal_goal_z_tolerance_m", ctypes.c_double),
    ]


class _FarPlanResult(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("ok", ctypes.c_uint8),
        ("reached_goal", ctypes.c_uint8),
        ("cancelled", ctypes.c_uint8),
        ("used_unknown_space", ctypes.c_uint8),
        ("start_snapped", ctypes.c_uint8),
        ("goal_snapped", ctypes.c_uint8),
        ("planning_phase", ctypes.c_int32),
        ("map_update_mode", ctypes.c_int32),
        ("map_generation", ctypes.c_uint64),
        ("changed_cells", ctypes.c_uint64),
        ("graph_nodes", ctypes.c_uint64),
        ("visibility_pairs", ctypes.c_uint64),
        ("reusable_edges", ctypes.c_uint64),
        ("recomputed_edges", ctypes.c_uint64),
        ("search_expansions", ctypes.c_uint64),
        ("unknown_cells_traversed", ctypes.c_uint64),
        ("goal_error_m", ctypes.c_double),
        ("goal_xy_error_m", ctypes.c_double),
        ("goal_z_error_m", ctypes.c_double),
        ("elapsed_ms", ctypes.c_double),
    ]


def _bytes(value: str | os.PathLike[str] | None) -> bytes | None:
    if value is None:
        return None
    return os.fsencode(value)


def _library_names() -> tuple[str, ...]:
    if sys.platform.startswith("win"):
        return ("nav_far.dll",)
    if sys.platform == "darwin":
        return ("libnav_far.dylib", "nav_far.dylib")
    return ("libnav_far.so", "nav_far.so")


def _library_candidates() -> list[Path]:
    explicit = os.environ.get("LINGTU_NAV_FAR_LIB", "").strip()
    candidates = [Path(explicit)] if explicit else []
    repo = Path(__file__).resolve().parents[6]
    roots = (
        repo / "build" / "nav",
        repo / "build" / "nav" / "Release",
        repo / "build" / "nav" / "Debug",
        repo / ".tmp" / "nav-product-build",
        repo / ".tmp" / "nav-product-build" / "Release",
        repo / ".tmp" / "nav-product-build" / "Debug",
        Path(sys.prefix) / "lib",
        Path(sys.prefix) / "bin",
    )
    for root in roots:
        for name in _library_names():
            candidates.append(root / name)
    return candidates


class _NativeFarLibrary:
    def __init__(self, library: ctypes.CDLL) -> None:
        self.library = library
        library.lingtu_nav_far_create.argtypes = [ctypes.POINTER(_FarConfig)]
        library.lingtu_nav_far_create.restype = ctypes.c_void_p
        library.lingtu_nav_far_destroy.argtypes = [ctypes.c_void_p]
        library.lingtu_nav_far_destroy.restype = None
        library.lingtu_nav_far_update_map.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_FarMap),
        ]
        library.lingtu_nav_far_update_map.restype = ctypes.c_int32
        library.lingtu_nav_far_plan.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_FarPlanRequest),
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(_FarPlanResult),
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        library.lingtu_nav_far_plan.restype = ctypes.c_int32
        library.lingtu_nav_far_last_error.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        library.lingtu_nav_far_last_error.restype = ctypes.c_int32


def _load_native_library() -> _NativeFarLibrary:
    failures: list[str] = []
    for candidate in _library_candidates():
        if not candidate.is_file():
            continue
        try:
            return _NativeFarLibrary(ctypes.CDLL(str(candidate)))
        except (OSError, AttributeError) as exc:
            failures.append(f"{candidate}: {exc}")
    found = ctypes.util.find_library("nav_far")
    if found:
        try:
            return _NativeFarLibrary(ctypes.CDLL(found))
        except (OSError, AttributeError) as exc:
            failures.append(f"{found}: {exc}")
    detail = f" ({'; '.join(failures)})" if failures else ""
    raise FarNativeUnavailable(
        "native FAR library not found; build src/nav/cpp and set LINGTU_NAV_FAR_LIB" + detail
    )


def _config_values(options: Mapping[str, Any] | None) -> dict[str, Any]:
    values: dict[str, Any] = {
        "robot_radius_m": 0.35,
        "obstacle_clearance_m": 0.10,
        "max_visibility_distance_m": 30.0,
        "unknown_cost_multiplier": 6.0,
        "corner_separation_cells": 1,
        "snap_search_radius_cells": 12,
        "max_graph_nodes": 4096,
        "max_visibility_pairs": 500_000,
        "max_search_expansions": 500_000,
        "allow_unknown_fallback": False,
        "simplify_path": True,
    }
    for key, value in dict(options or {}).items():
        if key in values and value is not None:
            values[key] = value
    return values


class FarPlannerBackend:
    """GlobalPlanner backend backed entirely by the native FAR C++ library."""

    def __init__(
        self,
        map_path: str = "",
        obstacle_thr: float = 49.9,
        *,
        options: Mapping[str, Any] | None = None,
    ) -> None:
        del map_path
        self._native = _load_native_library()
        self._obstacle_thr = float(obstacle_thr)
        self._config = _config_values(options)
        config = _FarConfig()
        config.struct_size = ctypes.sizeof(_FarConfig)
        config.abi_version = _ABI_VERSION
        for key, value in self._config.items():
            setattr(config, key, value)
        self._handle = self._native.library.lingtu_nav_far_create(ctypes.byref(config))
        if not self._handle:
            raise FarNativeUnavailable("native FAR planner rejected its configuration")
        self._grid = None
        self._resolution = 0.2
        self._origin = np.asarray([0.0, 0.0], dtype=float)
        self._frame_id = "map"
        self._generation = 0
        self._last_plan_error = ""
        self._last_plan_reached_goal = False
        self._last_plan_diagnostics: dict[str, Any] = {}

    @property
    def available(self) -> bool:
        return bool(self._handle)

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._native.library.lingtu_nav_far_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def _last_error(self) -> str:
        needed = ctypes.c_uint64(0)
        rc = int(
            self._native.library.lingtu_nav_far_last_error(
                self._handle,
                None,
                0,
                ctypes.byref(needed),
            )
        )
        if rc not in (0, 1) or needed.value == 0:
            return "native FAR operation failed"
        output = ctypes.create_string_buffer(needed.value)
        rc = int(
            self._native.library.lingtu_nav_far_last_error(
                self._handle,
                output,
                len(output),
                ctypes.byref(needed),
            )
        )
        return output.value.decode("utf-8", errors="replace") if rc == 0 else "native FAR operation failed"

    def update_planning_map(self, value: GlobalPlanningMap) -> None:
        planning_map = coerce_planning_map(value)
        grid = np.asarray(planning_map.grid, dtype=np.float32)
        if grid.ndim != 2 or grid.size == 0:
            raise ValueError("FAR planning map must be a non-empty 2-D grid")
        states = np.full(grid.shape, -1, dtype=np.int8)
        finite = np.isfinite(grid)
        states[finite & (grid >= self._obstacle_thr)] = 100
        states[finite & (grid >= 0.0) & (grid < self._obstacle_thr)] = 0
        states = np.ascontiguousarray(states, dtype=np.int8)
        origin = (
            np.asarray(planning_map.origin, dtype=float).reshape(-1)[:2]
            if planning_map.origin is not None
            else np.asarray([0.0, 0.0], dtype=float)
        )
        requested_generation = int(getattr(planning_map, "generation", 0) or 0)
        same_unversioned_map = bool(
            requested_generation == 0
            and self._grid is not None
            and self._grid.shape == grid.shape
            and float(self._resolution) == float(planning_map.resolution)
            and np.array_equal(self._origin, origin)
            and self._frame_id == str(planning_map.frame_id or "map")
            and np.array_equal(self._grid, grid)
        )
        generation = (
            requested_generation
            or (self._generation if same_unversioned_map else self._generation + 1)
        )
        map_value = _FarMap()
        map_value.struct_size = ctypes.sizeof(_FarMap)
        map_value.abi_version = _ABI_VERSION
        map_value.width = int(states.shape[1])
        map_value.height = int(states.shape[0])
        map_value.resolution_m = float(planning_map.resolution)
        map_value.origin_x_m = float(origin[0])
        map_value.origin_y_m = float(origin[1])
        frame_bytes = _bytes(planning_map.frame_id or "map")
        map_value.frame_id = frame_bytes
        map_value.generation = generation
        map_value.cells = states.ctypes.data_as(ctypes.POINTER(ctypes.c_int8))
        map_value.cell_count = int(states.size)
        map_value.map_id = None
        map_value.map_version = 0
        map_value.artifact_sha256 = None
        rc = int(self._native.library.lingtu_nav_far_update_map(self._handle, ctypes.byref(map_value)))
        if rc != 0:
            raise RuntimeError(self._last_error())
        self._grid = grid.copy()
        self._resolution = float(planning_map.resolution)
        self._origin = origin.copy()
        self._frame_id = str(planning_map.frame_id or "map")
        self._generation = generation
        self._last_plan_diagnostics = {
            "planner": "far",
            "stage": "map_updated",
            "map_generation": generation,
            "grid_shape": [int(grid.shape[0]), int(grid.shape[1])],
            "resolution": self._resolution,
            "origin": [float(origin[0]), float(origin[1])],
        }

    def update_map(
        self,
        grid: GlobalPlanningMap | Any,
        resolution: float = 0.2,
        origin: Any | None = None,
    ) -> None:
        self.update_planning_map(
            coerce_planning_map(grid, resolution=resolution, origin=origin)
        )

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        if self._grid is None:
            self._last_plan_error = "FAR planning map is not loaded"
            self._last_plan_reached_goal = False
            return GlobalPlanResult(
                error=self._last_plan_error,
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
                diagnostics={"planner": "far", "stage": "map_unavailable"},
            )
        native_request = _FarPlanRequest()
        native_request.struct_size = ctypes.sizeof(_FarPlanRequest)
        native_request.abi_version = _ABI_VERSION
        start = np.asarray(request.start, dtype=float).reshape(-1)
        goal = np.asarray(request.goal, dtype=float).reshape(-1)
        (
            native_request.start_x,
            native_request.start_y,
            native_request.start_z,
        ) = (float(start[0]), float(start[1]), float(start[2]))
        (
            native_request.goal_x,
            native_request.goal_y,
            native_request.goal_z,
        ) = (float(goal[0]), float(goal[1]), float(goal[2]))
        native_request.expected_map_generation = int(
            getattr(request, "map_generation", 0) or self._generation
        )
        native_request.max_iterations = 500_000
        native_request.terminal_goal_tolerance_m = 0.5
        native_request.terminal_goal_xy_tolerance_m = 0.6
        native_request.terminal_goal_z_tolerance_m = 0.75
        native_result = _FarPlanResult()
        native_result.struct_size = ctypes.sizeof(_FarPlanResult)
        native_result.abi_version = _ABI_VERSION
        path_points = ctypes.c_uint64(0)
        reason_size = ctypes.c_uint64(0)
        rc = int(
            self._native.library.lingtu_nav_far_plan(
                self._handle,
                ctypes.byref(native_request),
                None,
                None,
                None,
                0,
                ctypes.byref(path_points),
                ctypes.byref(native_result),
                None,
                0,
                ctypes.byref(reason_size),
            )
        )
        if rc not in (0, 1):
            raise RuntimeError(self._last_error())
        path_buffer = (ctypes.c_double * max(1, int(path_points.value) * 3))()
        reason_buffer = ctypes.create_string_buffer(max(1, int(reason_size.value)))
        rc = int(
            self._native.library.lingtu_nav_far_plan(
                self._handle,
                ctypes.byref(native_request),
                None,
                None,
                path_buffer,
                path_points.value,
                ctypes.byref(path_points),
                ctypes.byref(native_result),
                reason_buffer,
                len(reason_buffer),
                ctypes.byref(reason_size),
            )
        )
        if rc != 0:
            raise RuntimeError(self._last_error())
        path = [
            np.asarray(path_buffer[index * 3 : index * 3 + 3], dtype=float)
            for index in range(int(path_points.value))
        ]
        phase = {
            1: "known_free",
            2: "unknown_fallback",
            3: "failed",
        }.get(int(native_result.planning_phase), "none")
        update_mode = {
            1: "full",
            2: "incremental",
            3: "noop_same_generation",
        }.get(int(native_result.map_update_mode), "none")
        diagnostics = {
            "planner": "far",
            "stage": "cxx_plan_success" if native_result.ok else "cxx_plan_failed",
            "planning_phase": phase,
            "map_update_mode": update_mode,
            "map_generation": int(native_result.map_generation),
            "changed_cells": int(native_result.changed_cells),
            "graph_nodes": int(native_result.graph_nodes),
            "visibility_pairs": int(native_result.visibility_pairs),
            "reusable_edges": int(native_result.reusable_edges),
            "recomputed_edges": int(native_result.recomputed_edges),
            "search_expansions": int(native_result.search_expansions),
            "unknown_cells_traversed": int(native_result.unknown_cells_traversed),
            "used_unknown_space": bool(native_result.used_unknown_space),
            "start_snapped": bool(native_result.start_snapped),
            "goal_snapped": bool(native_result.goal_snapped),
            "goal_error_m": float(native_result.goal_error_m),
            "goal_xy_error_m": float(native_result.goal_xy_error_m),
            "goal_z_error_m": float(native_result.goal_z_error_m),
            "native_plan_ms": float(native_result.elapsed_ms),
        }
        error = reason_buffer.value.decode("utf-8", errors="replace")
        self._last_plan_error = error
        self._last_plan_reached_goal = bool(native_result.reached_goal)
        self._last_plan_diagnostics = diagnostics
        return GlobalPlanResult(
            path=path,
            plan_ms=float(native_result.elapsed_ms),
            reached_goal=bool(native_result.reached_goal),
            error=error,
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            map_generation=int(native_result.map_generation),
            diagnostics=diagnostics,
        )

    def plan(self, start: Sequence[float], goal: Sequence[float]) -> list[Any]:
        result = self.plan_request(
            GlobalPlanRequest(
                start=np.asarray(start, dtype=float),
                goal=np.asarray(goal, dtype=float),
                frame_id=self._frame_id,
                map_generation=self._generation,
            )
        )
        return result.points()


__all__ = ["FarNativeUnavailable", "FarPlannerBackend"]
