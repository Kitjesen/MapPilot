"""ctypes adapter for the C++ maps voxel layer."""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
from functools import lru_cache
from pathlib import Path
from typing import Any

from runtime.msgs.numpy_compat import np


class VoxelNativeUnavailable(RuntimeError):
    """Raised when the native voxel library is not available."""


class _VoxelConfig(ctypes.Structure):
    _fields_ = [
        ("voxel_size_m", ctypes.c_float),
        ("max_range_m", ctypes.c_float),
        ("min_z_m", ctypes.c_float),
        ("max_z_m", ctypes.c_float),
        ("decay_rate", ctypes.c_float),
        ("prune_below_count", ctypes.c_float),
        ("column_carving", ctypes.c_uint8),
    ]


class _VoxelStats(ctypes.Structure):
    _fields_ = [
        ("input_points", ctypes.c_uint64),
        ("accepted_points", ctypes.c_uint64),
        ("input_voxels", ctypes.c_uint64),
        ("input_columns", ctypes.c_uint64),
        ("carved_columns", ctypes.c_uint64),
        ("carved_voxels", ctypes.c_uint64),
        ("total_voxels", ctypes.c_uint64),
        ("accumulated_cells", ctypes.c_uint64),
        ("accumulated_occupied", ctypes.c_uint64),
        ("accumulated_generation", ctypes.c_uint64),
        ("ray_updates", ctypes.c_uint64),
        ("free_updates", ctypes.c_uint64),
        ("hit_updates", ctypes.c_uint64),
        ("pruned_cells", ctypes.c_uint64),
        ("column_carving", ctypes.c_uint8),
    ]


class _VoxelSceneStats(ctypes.Structure):
    _fields_ = [
        ("live_voxels", ctypes.c_uint64),
        ("accumulated_cells", ctypes.c_uint64),
        ("accumulated_occupied", ctypes.c_uint64),
        ("accumulated_generation", ctypes.c_uint64),
        ("live_points", ctypes.c_uint64),
        ("frame_stamp_ns", ctypes.c_uint64),
        ("voxel_size_m", ctypes.c_float),
        ("localization_ok", ctypes.c_uint8),
        ("map_ok", ctypes.c_uint8),
        ("planner_ok", ctypes.c_uint8),
        ("status_bits", ctypes.c_uint32),
    ]


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


class _NativeVoxelLib:
    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._configure()

    def _configure(self) -> None:
        self._lib.lingtu_maps_abi_version.argtypes = []
        self._lib.lingtu_maps_abi_version.restype = ctypes.c_uint32

        self._lib.lingtu_maps_voxel_create.argtypes = [ctypes.POINTER(_VoxelConfig)]
        self._lib.lingtu_maps_voxel_create.restype = ctypes.c_void_p

        self._lib.lingtu_maps_voxel_destroy.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_voxel_destroy.restype = None

        self._lib.lingtu_maps_voxel_reset.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_voxel_reset.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_decay.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_voxel_decay.restype = ctypes.c_int32

        update_xyz_args = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_char_p,
            ctypes.c_int64,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.lingtu_maps_voxel_update_xyz_interleaved.argtypes = update_xyz_args
        self._lib.lingtu_maps_voxel_update_xyz_interleaved.restype = ctypes.c_int32
        self._lib.lingtu_maps_voxel_update_xyzi_interleaved.argtypes = update_xyz_args
        self._lib.lingtu_maps_voxel_update_xyzi_interleaved.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_update_xyz_soa.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_char_p,
            ctypes.c_int64,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.lingtu_maps_voxel_update_xyz_soa.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_count.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_voxel_count.restype = ctypes.c_uint64

        self._lib.lingtu_maps_voxel_contains.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        self._lib.lingtu_maps_voxel_contains.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_query_count.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_float),
        ]
        self._lib.lingtu_maps_voxel_query_count.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_stats.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_VoxelStats),
        ]
        self._lib.lingtu_maps_voxel_stats.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_snapshot_size.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_voxel_snapshot_size.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_snapshot_xyz_soa.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_voxel_snapshot_xyz_soa.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_snapshot_occupied_xyz_soa.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_voxel_snapshot_occupied_xyz_soa.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_scene_stats.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_VoxelSceneStats),
        ]
        self._lib.lingtu_maps_voxel_scene_stats.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_save_binary.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
        ]
        self._lib.lingtu_maps_voxel_save_binary.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_load_binary.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
        ]
        self._lib.lingtu_maps_voxel_load_binary.restype = ctypes.c_int32

        self._lib.lingtu_maps_voxel_validate_binary.argtypes = [ctypes.c_char_p]
        self._lib.lingtu_maps_voxel_validate_binary.restype = ctypes.c_int32

        abi = int(self._lib.lingtu_maps_abi_version())
        if abi != 1:
            raise VoxelNativeUnavailable(f"unsupported lingtu_maps ABI version: {abi}")


@lru_cache(maxsize=1)
def _load_native_voxel_lib() -> _NativeVoxelLib | None:
    for candidate in _native_library_candidates():
        if candidate.is_file():
            try:
                return _NativeVoxelLib(ctypes.CDLL(str(candidate)))
            except OSError:
                continue
    found = ctypes.util.find_library("lingtu_maps")
    if found:
        try:
            return _NativeVoxelLib(ctypes.CDLL(found))
        except OSError:
            return None
    return None


def native_voxel_available() -> bool:
    return _load_native_voxel_lib() is not None


class NativeVoxelLayer:
    """Python-owned handle for the C++ VoxelLayerCore."""

    def __init__(
        self,
        *,
        voxel_size: float = 0.05,
        max_range: float = 20.0,
        min_z: float = -0.5,
        max_z: float = 3.0,
        decay_rate: float = 0.01,
        column_carving: bool = True,
    ) -> None:
        lib = _load_native_voxel_lib()
        if lib is None:
            raise VoxelNativeUnavailable("lingtu_maps native library not found")
        self._native = lib
        self._config = _VoxelConfig(
            float(voxel_size),
            float(max_range),
            float(min_z),
            float(max_z),
            float(decay_rate),
            1.0,
            1 if column_carving else 0,
        )
        self._handle = self._native._lib.lingtu_maps_voxel_create(ctypes.byref(self._config))
        if not self._handle:
            raise VoxelNativeUnavailable("lingtu_maps voxel create failed")

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._native._lib.lingtu_maps_voxel_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()

    def reset(self) -> None:
        self._check_rc(self._native._lib.lingtu_maps_voxel_reset(self._handle))

    def decay(self) -> None:
        self._check_rc(self._native._lib.lingtu_maps_voxel_decay(self._handle))

    def update(
        self,
        points: Any,
        *,
        frame_id: str = "map",
        stamp_ns: int = 0,
        origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        pts = np.ascontiguousarray(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] not in (3, 4):
            raise ValueError(f"points must be Nx3 or Nx4, got {pts.shape}")
        count = int(pts.shape[0])
        frame = str(frame_id or "map").encode("utf-8")
        origin = tuple(float(v) for v in origin_xyz)
        if pts.shape[1] == 3:
            rc = self._native._lib.lingtu_maps_voxel_update_xyz_interleaved(
                self._handle,
                ctypes.c_void_p(pts.ctypes.data),
                ctypes.c_uint64(count),
                ctypes.c_char_p(frame),
                ctypes.c_int64(int(stamp_ns)),
                ctypes.c_float(origin[0]),
                ctypes.c_float(origin[1]),
                ctypes.c_float(origin[2]),
            )
        else:
            rc = self._native._lib.lingtu_maps_voxel_update_xyzi_interleaved(
                self._handle,
                ctypes.c_void_p(pts.ctypes.data),
                ctypes.c_uint64(count),
                ctypes.c_char_p(frame),
                ctypes.c_int64(int(stamp_ns)),
                ctypes.c_float(origin[0]),
                ctypes.c_float(origin[1]),
                ctypes.c_float(origin[2]),
            )
        self._check_rc(rc)

    def voxel_count(self) -> int:
        return int(self._native._lib.lingtu_maps_voxel_count(self._handle))

    def query_count(self, x: float, y: float, z: float) -> float:
        out = ctypes.c_float()
        rc = self._native._lib.lingtu_maps_voxel_query_count(
            self._handle,
            ctypes.c_float(float(x)),
            ctypes.c_float(float(y)),
            ctypes.c_float(float(z)),
            ctypes.byref(out),
        )
        self._check_rc(rc)
        return float(out.value)

    def stats(self) -> dict[str, Any]:
        out = _VoxelStats()
        rc = self._native._lib.lingtu_maps_voxel_stats(self._handle, ctypes.byref(out))
        self._check_rc(rc)
        return {
            "input_points": int(out.input_points),
            "accepted_points": int(out.accepted_points),
            "input_voxels": int(out.input_voxels),
            "input_columns": int(out.input_columns),
            "carved_columns": int(out.carved_columns),
            "carved_voxels": int(out.carved_voxels),
            "total_voxels": int(out.total_voxels),
            "accumulated_cells": int(out.accumulated_cells),
            "accumulated_occupied": int(out.accumulated_occupied),
            "accumulated_generation": int(out.accumulated_generation),
            "ray_updates": int(out.ray_updates),
            "free_updates": int(out.free_updates),
            "hit_updates": int(out.hit_updates),
            "pruned_cells": int(out.pruned_cells),
            "column_carving": bool(out.column_carving),
        }

    def snapshot_xyz(self) -> Any:
        count = ctypes.c_uint64()
        self._check_rc(
            self._native._lib.lingtu_maps_voxel_snapshot_size(
                self._handle,
                ctypes.byref(count),
            )
        )
        n = int(count.value)
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)
        x = np.empty(n, dtype=np.float32)
        y = np.empty(n, dtype=np.float32)
        z = np.empty(n, dtype=np.float32)
        written = ctypes.c_uint64()
        rc = self._native._lib.lingtu_maps_voxel_snapshot_xyz_soa(
            self._handle,
            ctypes.c_void_p(x.ctypes.data),
            ctypes.c_void_p(y.ctypes.data),
            ctypes.c_void_p(z.ctypes.data),
            ctypes.c_uint64(n),
            ctypes.byref(written),
        )
        self._check_rc(rc)
        return np.column_stack((x[: written.value], y[: written.value], z[: written.value])).astype(
            np.float32,
            copy=False,
        )

    def scene_metadata(self) -> dict[str, Any]:
        out = _VoxelSceneStats()
        rc = self._native._lib.lingtu_maps_voxel_scene_stats(
            self._handle,
            ctypes.byref(out),
        )
        self._check_rc(rc)
        return {
            "live_voxels": int(out.live_voxels),
            "accumulated_cells": int(out.accumulated_cells),
            "accumulated_occupied": int(out.accumulated_occupied),
            "accumulated_generation": int(out.accumulated_generation),
            "live_points": int(out.live_points),
            "frame_stamp_ns": int(out.frame_stamp_ns),
            "voxel_size_m": float(out.voxel_size_m),
            "localization_ok": bool(out.localization_ok),
            "map_ok": bool(out.map_ok),
            "planner_ok": bool(out.planner_ok),
            "status_bits": int(out.status_bits),
        }

    def save_state(self, path: str | os.PathLike[str]) -> None:
        raw_path = os.fsencode(path)
        self._check_rc(
            self._native._lib.lingtu_maps_voxel_save_binary(
                self._handle,
                ctypes.c_char_p(raw_path),
            )
        )

    def load_state(self, path: str | os.PathLike[str]) -> None:
        raw_path = os.fsencode(path)
        self._check_rc(
            self._native._lib.lingtu_maps_voxel_load_binary(
                self._handle,
                ctypes.c_char_p(raw_path),
            )
        )

    def validate_state(self, path: str | os.PathLike[str]) -> bool:
        rc = self._native._lib.lingtu_maps_voxel_validate_binary(ctypes.c_char_p(os.fsencode(path)))
        if int(rc) < 0:
            self._check_rc(rc)
        return int(rc) == 0

    @staticmethod
    def _check_rc(rc: int) -> None:
        if int(rc) != 0:
            raise RuntimeError(f"lingtu_maps voxel native call failed: {rc}")
