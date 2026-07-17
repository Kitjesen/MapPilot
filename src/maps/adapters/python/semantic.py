"""ctypes adapter for the native semantic occupancy C ABI."""

from __future__ import annotations

import ctypes
import ctypes.util
from functools import lru_cache
from typing import Any

from maps.adapters.python.store import _native_library_candidates
from runtime.msgs.numpy_compat import np


class SemanticNativeUnavailable(RuntimeError):
    pass


class SemanticGenerationChanged(RuntimeError):
    pass


class _Config(ctypes.Structure):
    _fields_ = [
        ("voxel_size_m", ctypes.c_float),
        ("max_range_m", ctypes.c_float),
        ("min_z_m", ctypes.c_float),
        ("max_z_m", ctypes.c_float),
        ("hit_log_odds", ctypes.c_float),
        ("miss_log_odds", ctypes.c_float),
        ("min_log_odds", ctypes.c_float),
        ("max_log_odds", ctypes.c_float),
        ("occupied_probability", ctypes.c_float),
        ("raycast_free_space", ctypes.c_uint8),
        ("max_rays_per_update", ctypes.c_uint64),
        ("max_ray_voxels_per_ray", ctypes.c_uint64),
        ("max_voxels", ctypes.c_uint64),
        ("max_query_voxel_checks", ctypes.c_uint64),
        ("max_query_results", ctypes.c_uint64),
        ("unknown_label", ctypes.c_uint16),
    ]


class _UpdateStats(ctypes.Structure):
    _fields_ = [
        ("input_points", ctypes.c_uint64),
        ("accepted_points", ctypes.c_uint64),
        ("hit_voxels", ctypes.c_uint64),
        ("rays_traced", ctypes.c_uint64),
        ("truncated_rays", ctypes.c_uint64),
        ("free_voxel_updates", ctypes.c_uint64),
        ("pruned_voxels", ctypes.c_uint64),
        ("total_voxels", ctypes.c_uint64),
        ("generation_before", ctypes.c_uint64),
        ("generation_after", ctypes.c_uint64),
        ("replaced_full_map", ctypes.c_uint8),
        ("applied", ctypes.c_uint8),
        ("duplicate_sequence", ctypes.c_uint8),
        ("stale_sequence", ctypes.c_uint8),
    ]


class _Query(ctypes.Structure):
    _fields_ = [
        ("center_x_m", ctypes.c_float),
        ("center_y_m", ctypes.c_float),
        ("center_z_m", ctypes.c_float),
        ("radius_m", ctypes.c_float),
        ("min_occupancy_probability", ctypes.c_float),
    ]


class _Metadata(ctypes.Structure):
    _fields_ = [
        ("generation", ctypes.c_uint64),
        ("voxel_count", ctypes.c_uint64),
        ("voxel_size_m", ctypes.c_float),
        ("taxonomy_version", ctypes.c_uint32),
        ("frame_id_bytes", ctypes.c_uint64),
        ("taxonomy_bytes", ctypes.c_uint64),
    ]


class _ChunkBuffers(ctypes.Structure):
    _fields_ = [
        ("index_x", ctypes.c_void_p),
        ("index_y", ctypes.c_void_p),
        ("index_z", ctypes.c_void_p),
        ("center_x_m", ctypes.c_void_p),
        ("center_y_m", ctypes.c_void_p),
        ("center_z_m", ctypes.c_void_p),
        ("occupancy_probability", ctypes.c_void_p),
        ("hit_count", ctypes.c_void_p),
        ("miss_count", ctypes.c_void_p),
        ("point_count", ctypes.c_void_p),
        ("mean_x_m", ctypes.c_void_p),
        ("mean_y_m", ctypes.c_void_p),
        ("mean_z_m", ctypes.c_void_p),
        ("covariance_xx", ctypes.c_void_p),
        ("covariance_xy", ctypes.c_void_p),
        ("covariance_xz", ctypes.c_void_p),
        ("covariance_yy", ctypes.c_void_p),
        ("covariance_yz", ctypes.c_void_p),
        ("covariance_zz", ctypes.c_void_p),
        ("dominant_label", ctypes.c_void_p),
        ("semantic_confidence", ctypes.c_void_p),
    ]


class _NativeLib:
    def __init__(self, lib: ctypes.CDLL) -> None:
        self.lib = lib
        self._configure()

    def _configure(self) -> None:
        lib = self.lib
        lib.lingtu_maps_semantic_abi_version.argtypes = []
        lib.lingtu_maps_semantic_abi_version.restype = ctypes.c_uint32
        if int(lib.lingtu_maps_semantic_abi_version()) != 1:
            raise SemanticNativeUnavailable("unsupported semantic maps ABI")
        lib.lingtu_maps_semantic_create.argtypes = [ctypes.POINTER(_Config)]
        lib.lingtu_maps_semantic_create.restype = ctypes.c_void_p
        lib.lingtu_maps_semantic_destroy.argtypes = [ctypes.c_void_p]
        lib.lingtu_maps_semantic_destroy.restype = None
        lib.lingtu_maps_semantic_reset.argtypes = [ctypes.c_void_p]
        lib.lingtu_maps_semantic_reset.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_generation.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        lib.lingtu_maps_semantic_generation.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_metadata.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Metadata),
        ]
        lib.lingtu_maps_semantic_metadata.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_metadata_strings.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_void_p,
            ctypes.c_uint64,
        ]
        lib.lingtu_maps_semantic_metadata_strings.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_load_file.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint64,
            ctypes.POINTER(_UpdateStats),
        ]
        lib.lingtu_maps_semantic_load_file.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_save_file.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint64,
        ]
        lib.lingtu_maps_semantic_save_file.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_validate_file.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        lib.lingtu_maps_semantic_validate_file.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_update_xyz_soa.argtypes = [
            ctypes.c_void_p,
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
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_uint32,
            ctypes.POINTER(_UpdateStats),
        ]
        lib.lingtu_maps_semantic_update_xyz_soa.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_query_radius_count.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Query),
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint64),
        ]
        lib.lingtu_maps_semantic_query_radius_count.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_query_radius_fill.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Query),
            ctypes.c_uint64,
            ctypes.POINTER(_ChunkBuffers),
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        lib.lingtu_maps_semantic_query_radius_fill.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_snapshot_count.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint64),
        ]
        lib.lingtu_maps_semantic_snapshot_count.restype = ctypes.c_int32
        lib.lingtu_maps_semantic_snapshot_fill.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_uint64,
            ctypes.c_uint64,
            ctypes.POINTER(_ChunkBuffers),
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
            ctypes.POINTER(ctypes.c_uint8),
        ]
        lib.lingtu_maps_semantic_snapshot_fill.restype = ctypes.c_int32


@lru_cache(maxsize=1)
def _load_native() -> _NativeLib | None:
    for candidate in _native_library_candidates():
        if not candidate.is_file():
            continue
        try:
            return _NativeLib(ctypes.CDLL(str(candidate)))
        except (OSError, AttributeError, SemanticNativeUnavailable):
            continue
    found = ctypes.util.find_library("lingtu_maps")
    if found:
        try:
            return _NativeLib(ctypes.CDLL(found))
        except (OSError, AttributeError, SemanticNativeUnavailable):
            return None
    return None


def _pointer(array: Any | None) -> ctypes.c_void_p:
    if array is None or not getattr(array, "size", 0):
        return ctypes.c_void_p()
    return ctypes.c_void_p(int(array.ctypes.data))


def _allocate_chunk(count: int) -> tuple[dict[str, Any], _ChunkBuffers]:
    arrays = {
        "index_x": np.empty(count, dtype=np.int32),
        "index_y": np.empty(count, dtype=np.int32),
        "index_z": np.empty(count, dtype=np.int32),
        "center_x_m": np.empty(count, dtype=np.float32),
        "center_y_m": np.empty(count, dtype=np.float32),
        "center_z_m": np.empty(count, dtype=np.float32),
        "occupancy_probability": np.empty(count, dtype=np.float32),
        "hit_count": np.empty(count, dtype=np.uint32),
        "miss_count": np.empty(count, dtype=np.uint32),
        "point_count": np.empty(count, dtype=np.uint32),
        "mean_x_m": np.empty(count, dtype=np.float32),
        "mean_y_m": np.empty(count, dtype=np.float32),
        "mean_z_m": np.empty(count, dtype=np.float32),
        "covariance_xx": np.empty(count, dtype=np.float32),
        "covariance_xy": np.empty(count, dtype=np.float32),
        "covariance_xz": np.empty(count, dtype=np.float32),
        "covariance_yy": np.empty(count, dtype=np.float32),
        "covariance_yz": np.empty(count, dtype=np.float32),
        "covariance_zz": np.empty(count, dtype=np.float32),
        "dominant_label": np.empty(count, dtype=np.uint16),
        "semantic_confidence": np.empty(count, dtype=np.float32),
    }
    return arrays, _ChunkBuffers(*(_pointer(array) for array in arrays.values()))


class NativeSemanticLayer:
    """Owns a C++ semantic occupancy handle; no Python map implementation exists."""

    def __init__(
        self,
        *,
        voxel_size: float = 0.2,
        max_range: float = 50.0,
        min_z: float = -3.0,
        max_z: float = 5.0,
        raycast_free_space: bool = True,
        max_voxels: int = 2_000_000,
        unknown_label: int = 0,
    ) -> None:
        native = _load_native()
        if native is None:
            raise SemanticNativeUnavailable("lingtu_maps semantic C ABI not found")
        self._native = native
        self._config = _Config(
            voxel_size,
            max_range,
            min_z,
            max_z,
            0.85,
            -0.40,
            -2.0,
            3.5,
            0.50,
            1 if raycast_free_space else 0,
            4000,
            1024,
            max_voxels,
            1_500_000,
            200_000,
            unknown_label,
        )
        self._handle = native.lib.lingtu_maps_semantic_create(ctypes.byref(self._config))
        if not self._handle:
            raise SemanticNativeUnavailable("semantic occupancy create failed")

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._native.lib.lingtu_maps_semantic_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()

    def generation(self) -> int:
        value = ctypes.c_uint64()
        self._check(self._native.lib.lingtu_maps_semantic_generation(self._handle, ctypes.byref(value)))
        return int(value.value)

    def reset(self) -> None:
        self._check(self._native.lib.lingtu_maps_semantic_reset(self._handle))

    def metadata(self) -> dict[str, Any]:
        raw = _Metadata()
        self._check(self._native.lib.lingtu_maps_semantic_metadata(self._handle, ctypes.byref(raw)))
        frame = ctypes.create_string_buffer(max(1, int(raw.frame_id_bytes)))
        taxonomy = ctypes.create_string_buffer(max(1, int(raw.taxonomy_bytes)))
        self._check(
            self._native.lib.lingtu_maps_semantic_metadata_strings(
                self._handle,
                raw.generation,
                frame,
                ctypes.c_uint64(len(frame)),
                taxonomy,
                ctypes.c_uint64(len(taxonomy)),
            )
        )
        return {
            "generation": int(raw.generation),
            "voxel_count": int(raw.voxel_count),
            "voxel_size_m": float(raw.voxel_size_m),
            "frame_id": frame.value.decode("utf-8"),
            "taxonomy": taxonomy.value.decode("utf-8"),
            "taxonomy_version": int(raw.taxonomy_version),
        }

    def load(self, path: str, *, expected_generation: int | None = None) -> dict[str, Any]:
        generation = self.generation() if expected_generation is None else int(expected_generation)
        stats = _UpdateStats()
        self._check(
            self._native.lib.lingtu_maps_semantic_load_file(
                self._handle,
                str(path).encode("utf-8"),
                ctypes.c_uint64(generation),
                ctypes.byref(stats),
            )
        )
        return {name: int(getattr(stats, name)) for name, _ in stats._fields_}

    def save(self, path: str, *, expected_generation: int | None = None) -> None:
        generation = self.generation() if expected_generation is None else int(expected_generation)
        self._check(
            self._native.lib.lingtu_maps_semantic_save_file(
                self._handle,
                str(path).encode("utf-8"),
                ctypes.c_uint64(generation),
            )
        )

    @classmethod
    def validate_file(cls, path: str) -> tuple[bool, str]:
        native = _load_native()
        if native is None:
            raise SemanticNativeUnavailable("lingtu_maps semantic C ABI not found")
        needed = ctypes.c_uint64()
        rc = int(
            native.lib.lingtu_maps_semantic_validate_file(str(path).encode("utf-8"), None, 0, ctypes.byref(needed))
        )
        if rc != 1:
            raise RuntimeError(f"semantic validation probe failed: {rc}")
        output = ctypes.create_string_buffer(max(1, int(needed.value)))
        rc = int(
            native.lib.lingtu_maps_semantic_validate_file(
                str(path).encode("utf-8"),
                output,
                ctypes.c_uint64(len(output)),
                ctypes.byref(needed),
            )
        )
        message = output.value.decode("utf-8")
        if rc == 0:
            return True, message or "semantic_map.bin validated"
        if rc == -2:
            return False, message or "semantic_map.bin validation failed"
        raise RuntimeError(f"semantic validation failed: {rc}")

    def update(
        self,
        points: Any,
        *,
        sequence: int,
        stamp_ns: int,
        frame_id: str,
        origin_xyz: tuple[float, float, float],
        labels: Any | None = None,
        taxonomy: str = "",
        taxonomy_version: int = 0,
        expected_generation: int | None = None,
    ) -> dict[str, Any]:
        xyz = np.ascontiguousarray(points, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] < 3 or xyz.shape[0] <= 0:
            raise ValueError("points must be non-empty Nx3 or wider")
        x = np.ascontiguousarray(xyz[:, 0])
        y = np.ascontiguousarray(xyz[:, 1])
        z = np.ascontiguousarray(xyz[:, 2])
        label_array = None
        if labels is not None:
            label_array = np.ascontiguousarray(labels, dtype=np.uint16)
            if label_array.ndim != 1 or label_array.size != xyz.shape[0]:
                raise ValueError("labels must match point count")
            if not taxonomy or int(taxonomy_version) <= 0:
                raise ValueError("versioned taxonomy is required with labels")
        generation = self.generation() if expected_generation is None else int(expected_generation)
        origin = tuple(float(value) for value in origin_xyz)
        stats = _UpdateStats()
        rc = self._native.lib.lingtu_maps_semantic_update_xyz_soa(
            self._handle,
            _pointer(x),
            _pointer(y),
            _pointer(z),
            _pointer(label_array),
            ctypes.c_uint64(xyz.shape[0]),
            str(frame_id).encode("utf-8"),
            ctypes.c_int64(stamp_ns),
            ctypes.c_float(origin[0]),
            ctypes.c_float(origin[1]),
            ctypes.c_float(origin[2]),
            ctypes.c_uint64(sequence),
            ctypes.c_uint64(generation),
            ctypes.c_uint8(0),
            str(taxonomy).encode("utf-8") if label_array is not None else None,
            ctypes.c_uint32(taxonomy_version if label_array is not None else 0),
            ctypes.byref(stats),
        )
        self._check(rc)
        return {name: int(getattr(stats, name)) for name, _ in stats._fields_}

    def snapshot(self, *, min_occupancy: float = 0.5) -> dict[str, Any]:
        generation = ctypes.c_uint64()
        count = ctypes.c_uint64()
        self._check(
            self._native.lib.lingtu_maps_semantic_snapshot_count(
                self._handle,
                ctypes.c_float(min_occupancy),
                ctypes.byref(generation),
                ctypes.byref(count),
            )
        )
        size = int(count.value)
        arrays, buffers = _allocate_chunk(size)
        if size:
            written = ctypes.c_uint64()
            complete = ctypes.c_uint8()
            self._check(
                self._native.lib.lingtu_maps_semantic_snapshot_fill(
                    self._handle,
                    ctypes.c_float(min_occupancy),
                    generation,
                    ctypes.c_uint64(0),
                    ctypes.byref(buffers),
                    ctypes.c_uint64(size),
                    ctypes.byref(written),
                    ctypes.byref(complete),
                )
            )
            if int(written.value) != size or int(complete.value) != 1:
                raise RuntimeError("semantic snapshot was not complete")
        arrays["generation"] = int(generation.value)
        return arrays

    @staticmethod
    def _check(rc: int) -> None:
        value = int(rc)
        if value == 0:
            return
        if value == 2:
            raise SemanticGenerationChanged("semantic map generation changed")
        raise RuntimeError(f"lingtu_maps semantic call failed: {value}")
