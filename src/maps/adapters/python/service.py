"""ctypes adapter for the native C++ MapsServiceCore."""

from __future__ import annotations

import ctypes
import ctypes.util
import json
from functools import lru_cache
from pathlib import Path
from typing import Any

from maps.adapters.python.store import (
    MapStoreNativeUnavailable,
    _bytes,
    _native_library_candidates,
)


class MapsServiceNativeUnavailable(RuntimeError):
    """Raised when the native lingtu_maps service ABI is unavailable."""


_SAVE_MAP_ABI_VERSION = 1


class _SaveRequirements(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("occupancy", ctypes.c_uint8),
        ("octomap", ctypes.c_uint8),
        ("esdf", ctypes.c_uint8),
        ("traversability", ctypes.c_uint8),
        ("semantic", ctypes.c_uint8),
    ]


class _SaveSourceOptions(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("voxel_size", ctypes.c_double),
        ("dynamic_filter_enabled", ctypes.c_uint8),
        ("dynamic_filter_required", ctypes.c_uint8),
        ("dynamic_filter_command", ctypes.c_char_p),
        ("dynamic_filter_timeout_sec", ctypes.c_double),
        ("optimizer_strategy", ctypes.c_char_p),
        ("optimizer_required", ctypes.c_uint8),
        ("optimizer_command", ctypes.c_char_p),
        ("optimizer_timeout_sec", ctypes.c_double),
    ]


class _SaveOctomapOptions(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("converter_command", ctypes.c_char_p),
        ("build_mode", ctypes.c_char_p),
        ("resolution", ctypes.c_double),
        ("support_dilation_cells", ctypes.c_int32),
        ("free_layers_above", ctypes.c_int32),
        ("free_dilation_cells", ctypes.c_int32),
        ("frame_id", ctypes.c_char_p),
        ("source_profile", ctypes.c_char_p),
        ("data_source", ctypes.c_char_p),
        ("slam_source", ctypes.c_char_p),
        ("localization_source", ctypes.c_char_p),
        ("mapping_source", ctypes.c_char_p),
        ("timeout_sec", ctypes.c_double),
    ]


class _SaveMapRequest(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("request_id", ctypes.c_char_p),
        ("map_id", ctypes.c_char_p),
        ("require", _SaveRequirements),
        ("source", _SaveSourceOptions),
        ("octomap", _SaveOctomapOptions),
        ("activate_on_success", ctypes.c_uint8),
        ("require_slam_healthy", ctypes.c_uint8),
        ("minimum_point_count", ctypes.c_uint64),
    ]


class _MapSnapshot(ctypes.Structure):
    _fields_ = [
        ("struct_size", ctypes.c_uint32),
        ("abi_version", ctypes.c_uint32),
        ("snapshot_id", ctypes.c_char_p),
        ("source_dir", ctypes.c_char_p),
        ("frame_id", ctypes.c_char_p),
        ("captured_at_ns", ctypes.c_int64),
        ("first_sequence", ctypes.c_uint64),
        ("last_sequence", ctypes.c_uint64),
        ("source_sha256", ctypes.c_char_p),
        ("slam_healthy", ctypes.c_uint8),
        ("health_message", ctypes.c_char_p),
    ]


def _new_save_struct(struct_type):
    value = struct_type()
    value.struct_size = ctypes.sizeof(struct_type)
    value.abi_version = _SAVE_MAP_ABI_VERSION
    return value


class _NativeMapsServiceLib:
    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._configure()

    def _configure(self) -> None:
        self._lib.lingtu_maps_abi_version.argtypes = []
        self._lib.lingtu_maps_abi_version.restype = ctypes.c_uint32

        self._lib.lingtu_maps_service_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self._lib.lingtu_maps_service_create.restype = ctypes.c_void_p
        self._lib.lingtu_maps_service_destroy.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_service_destroy.restype = None

        query_args = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        for name in (
            "lingtu_maps_service_list_maps_json",
            "lingtu_maps_service_get_map_types_json",
            "lingtu_maps_service_get_active_json",
            "lingtu_maps_service_clear_active_map_json",
        ):
            fn = getattr(self._lib, name)
            fn.argtypes = query_args
            fn.restype = ctypes.c_int32

        id_args = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        for name in (
            "lingtu_maps_service_get_record_json",
            "lingtu_maps_service_get_health_json",
            "lingtu_maps_service_create_map_json",
            "lingtu_maps_service_delete_map_json",
            "lingtu_maps_service_retire_map_json",
        ):
            fn = getattr(self._lib, name)
            fn.argtypes = id_args
            fn.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_get_bundle_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_get_bundle_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_validate_artifacts_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_validate_artifacts_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_get_map_points_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint64,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_get_map_points_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_save_map_begin_json.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_SaveMapRequest),
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_save_map_begin_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_save_map_provide_snapshot_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.POINTER(_MapSnapshot),
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_save_map_provide_snapshot_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_list_save_map_jobs_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_list_save_map_jobs_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_list_map_versions_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_list_map_versions_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_rollback_map_version_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_int64,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_rollback_map_version_json.restype = ctypes.c_int32
        for name in (
            "lingtu_maps_service_save_map_status_json",
            "lingtu_maps_service_save_map_cancel_json",
            "lingtu_maps_service_save_map_retry_json",
        ):
            fn = getattr(self._lib, name)
            fn.argtypes = [
                ctypes.c_void_p,
                ctypes.c_char_p,
                ctypes.c_void_p,
                ctypes.c_uint64,
                ctypes.POINTER(ctypes.c_uint64),
            ]
            fn.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_list_poi_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_list_poi_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_set_poi_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_set_poi_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_delete_poi_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_delete_poi_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_list_map_graph_json.argtypes = query_args
        self._lib.lingtu_maps_service_list_map_graph_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_list_active_slots_json.argtypes = query_args
        self._lib.lingtu_maps_service_list_active_slots_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_set_map_edge_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_set_map_edge_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_delete_map_edge_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_delete_map_edge_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_shortest_route_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_shortest_route_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_get_active_slot_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_get_active_slot_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_set_active_slot_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_set_active_slot_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_clear_active_slot_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_clear_active_slot_json.restype = ctypes.c_int32

        for name in (
            "lingtu_maps_service_rollback_active_map_json",
            "lingtu_maps_service_get_build_queue_json",
            "lingtu_maps_service_pop_build_queue_json",
        ):
            fn = getattr(self._lib, name)
            fn.argtypes = query_args
            fn.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_enqueue_build_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_enqueue_build_json.restype = ctypes.c_int32

        for name in (
            "lingtu_maps_service_get_artifact_job_json",
            "lingtu_maps_service_cancel_artifact_job_json",
            "lingtu_maps_service_retry_artifact_job_json",
        ):
            fn = getattr(self._lib, name)
            fn.argtypes = id_args
            fn.restype = ctypes.c_int32

        maintenance_args = [
            ctypes.c_void_p,
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        for name in (
            "lingtu_maps_service_audit_versions_json",
            "lingtu_maps_service_quarantine_versions_json",
            "lingtu_maps_service_gc_versions_json",
            "lingtu_maps_service_migrate_versions_json",
        ):
            fn = getattr(self._lib, name)
            fn.argtypes = maintenance_args
            fn.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_export_version_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_int64,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_export_version_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_import_package_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_import_package_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_rename_map_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_rename_map_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_set_active_map_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_set_active_map_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_begin_build_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_begin_build_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_finish_build_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_finish_build_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_get_build_status_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_get_build_status_json.restype = ctypes.c_int32

        pcd_edit_args = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_import_pcd_json.argtypes = pcd_edit_args
        self._lib.lingtu_maps_service_import_pcd_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_commit_saved_source_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_commit_saved_source_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_crop_pcd_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_crop_pcd_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_restore_source_backup_json.argtypes = id_args
        self._lib.lingtu_maps_service_restore_source_backup_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_service_build_occupancy_snapshot_json.argtypes = id_args
        self._lib.lingtu_maps_service_build_occupancy_snapshot_json.restype = ctypes.c_int32
        octomap_build_args = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_int32,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_build_octomap_artifact_json.argtypes = octomap_build_args
        self._lib.lingtu_maps_service_build_octomap_artifact_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_get_voxel_edits_json.argtypes = id_args
        self._lib.lingtu_maps_service_get_voxel_edits_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_edit_octomap_voxels_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_edit_octomap_voxels_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_build_navigation_package_json.argtypes = (
            octomap_build_args[:-3]
            + [
                ctypes.c_uint8,
                ctypes.c_uint8,
            ]
            + octomap_build_args[-3:]
        )
        self._lib.lingtu_maps_service_build_navigation_package_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_build_esdf_artifact_json.argtypes = id_args
        self._lib.lingtu_maps_service_build_esdf_artifact_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_build_traversability_artifact_json.argtypes = id_args
        self._lib.lingtu_maps_service_build_traversability_artifact_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_build_semantic_artifact_json.argtypes = id_args
        self._lib.lingtu_maps_service_build_semantic_artifact_json.restype = ctypes.c_int32

        health_output = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_service_ingest_localization_health_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_char_p,
            *health_output,
        ]
        self._lib.lingtu_maps_service_ingest_localization_health_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_ingest_planning_outcome_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_char_p,
            *health_output,
        ]
        self._lib.lingtu_maps_service_ingest_planning_outcome_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_service_ingest_collision_event_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_char_p,
            ctypes.c_char_p,
            *health_output,
        ]
        self._lib.lingtu_maps_service_ingest_collision_event_json.restype = ctypes.c_int32

        abi = int(self._lib.lingtu_maps_abi_version())
        if abi != 1:
            raise MapsServiceNativeUnavailable(f"unsupported lingtu_maps ABI version: {abi}")


@lru_cache(maxsize=1)
def _load_native_maps_service_lib() -> _NativeMapsServiceLib | None:
    for candidate in _native_library_candidates():
        if candidate.is_file():
            try:
                return _NativeMapsServiceLib(ctypes.CDLL(str(candidate)))
            except (OSError, AttributeError, MapsServiceNativeUnavailable, MapStoreNativeUnavailable):
                continue
    found = ctypes.util.find_library("lingtu_maps")
    if found:
        try:
            return _NativeMapsServiceLib(ctypes.CDLL(found))
        except (OSError, AttributeError, MapsServiceNativeUnavailable, MapStoreNativeUnavailable):
            return None
    return None


class NativeMapsService:
    """Python-owned handle for C++ MapsServiceCore."""

    def __init__(self, root_dir: str | Path, active_state_filename: str = "active_map.txt") -> None:
        lib = _load_native_maps_service_lib()
        if lib is None:
            raise MapsServiceNativeUnavailable("lingtu_maps native service library not found")
        self._native = lib
        self._handle = self._native._lib.lingtu_maps_service_create(
            _bytes(root_dir),
            _bytes(active_state_filename),
        )
        if not self._handle:
            raise MapsServiceNativeUnavailable("lingtu_maps service create failed")

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._native._lib.lingtu_maps_service_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()

    def list_maps(self) -> dict[str, Any]:
        return self._read_json(self._native._lib.lingtu_maps_service_list_maps_json)

    def get_map_types(self) -> dict[str, Any]:
        return self._read_json(self._native._lib.lingtu_maps_service_get_map_types_json)

    def get_record(self, map_id: str) -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_get_record_json,
            map_id,
        )

    def get_active_map(self) -> dict[str, Any]:
        return self._read_json(self._native._lib.lingtu_maps_service_get_active_json)

    def get_health(self, map_id: str = "") -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_get_health_json,
            map_id,
        )

    def validate_artifacts(
        self,
        map_id: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_frame_id: str = "",
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_validate_artifacts_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_uint8(int(bool(require_octomap))),
            ctypes.c_uint8(int(bool(require_occupancy))),
            _bytes(expected_frame_id),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def ingest_localization_health(
        self,
        map_id: str = "",
        *,
        timestamp_s: float,
        localized: bool,
        position_error_m: float = 0.0,
        covariance_trace: float = 0.0,
        quality: float = 1.0,
        source: str = "runtime.localization",
    ) -> dict[str, Any]:
        """Feed one localization observation into the native health model."""
        fn = self._native._lib.lingtu_maps_service_ingest_localization_health_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_double(float(timestamp_s)),
            ctypes.c_uint8(int(bool(localized))),
            ctypes.c_double(float(position_error_m)),
            ctypes.c_double(float(covariance_trace)),
            ctypes.c_double(float(quality)),
            _bytes(source),
        )
        return self._call_health_command(fn, args)

    def ingest_planning_outcome(
        self,
        map_id: str = "",
        *,
        timestamp_s: float,
        success: bool,
        planner: str,
        reason: str = "",
    ) -> dict[str, Any]:
        """Feed one global-planning result into the native health model."""
        fn = self._native._lib.lingtu_maps_service_ingest_planning_outcome_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_double(float(timestamp_s)),
            ctypes.c_uint8(int(bool(success))),
            _bytes(planner),
            _bytes(reason),
        )
        return self._call_health_command(fn, args)

    def ingest_collision_event(
        self,
        map_id: str = "",
        *,
        timestamp_s: float,
        severity: float,
        source: str,
        reason: str = "",
    ) -> dict[str, Any]:
        """Feed one collision or projected-collision event into native health."""
        fn = self._native._lib.lingtu_maps_service_ingest_collision_event_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_double(float(timestamp_s)),
            ctypes.c_double(float(severity)),
            _bytes(source),
            _bytes(reason),
        )
        return self._call_health_command(fn, args)

    def _call_health_command(self, fn, args: tuple[Any, ...]) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def get_bundle(self, map_id: str, capability: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_get_bundle_json
        rc = int(fn(self._handle, _bytes(map_id), _bytes(capability), None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                _bytes(capability),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def get_map_points(self, map_id: str = "", *, max_points: int = 0) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_get_map_points_json
        limit = max(0, int(max_points or 0))
        rc = int(
            fn(
                self._handle,
                _bytes(map_id),
                ctypes.c_uint64(limit),
                None,
                0,
                ctypes.byref(needed),
            )
        )
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                ctypes.c_uint64(limit),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def begin_save_map(
        self,
        request_id: str,
        map_id: str,
        *,
        requirements: dict[str, bool] | None = None,
        source: dict[str, Any] | None = None,
        octomap: dict[str, Any] | None = None,
        activate_on_success: bool = False,
        require_slam_healthy: bool = True,
        minimum_point_count: int = 1,
    ) -> dict[str, Any]:
        requirements = requirements or {}
        source = source or {}
        octomap = octomap or {}

        required = _new_save_struct(_SaveRequirements)
        required.occupancy = int(requirements.get("occupancy", True))
        required.octomap = int(requirements.get("octomap", True))
        required.esdf = int(requirements.get("esdf", True))
        required.traversability = int(requirements.get("traversability", True))
        required.semantic = int(requirements.get("semantic", False))

        source_options = _new_save_struct(_SaveSourceOptions)
        source_options.voxel_size = float(source.get("voxel_size", 0.0))
        source_options.dynamic_filter_enabled = int(source.get("dynamic_filter_enabled", True))
        source_options.dynamic_filter_required = int(source.get("dynamic_filter_required", False))
        source_options.dynamic_filter_command = _bytes(source.get("dynamic_filter_command", ""))
        source_options.dynamic_filter_timeout_sec = float(source.get("dynamic_filter_timeout_sec", 300.0))
        source_options.optimizer_strategy = _bytes(source.get("optimizer_strategy", "none"))
        source_options.optimizer_required = int(source.get("optimizer_required", False))
        source_options.optimizer_command = _bytes(source.get("optimizer_command", ""))
        source_options.optimizer_timeout_sec = float(source.get("optimizer_timeout_sec", 120.0))

        octomap_options = _new_save_struct(_SaveOctomapOptions)
        octomap_options.converter_command = _bytes(octomap.get("converter_command", ""))
        octomap_options.build_mode = _bytes(octomap.get("build_mode", "external_pcl_converter"))
        octomap_options.resolution = float(octomap.get("resolution", 0.20))
        octomap_options.support_dilation_cells = int(octomap.get("support_dilation_cells", 1))
        octomap_options.free_layers_above = int(octomap.get("free_layers_above", 3))
        octomap_options.free_dilation_cells = int(octomap.get("free_dilation_cells", 1))
        octomap_options.frame_id = _bytes(octomap.get("frame_id", "map"))
        octomap_options.source_profile = _bytes(octomap.get("source_profile", "map_pipeline"))
        octomap_options.data_source = _bytes(octomap.get("data_source", octomap.get("source_profile", "map_pipeline")))
        octomap_options.slam_source = _bytes(octomap.get("slam_source", "unknown"))
        octomap_options.localization_source = _bytes(
            octomap.get("localization_source", octomap.get("slam_source", "unknown"))
        )
        octomap_options.mapping_source = _bytes(octomap.get("mapping_source", "lingtu_maps_pipeline"))
        octomap_options.timeout_sec = float(octomap.get("timeout_sec", 60.0))

        request = _new_save_struct(_SaveMapRequest)
        request.request_id = _bytes(request_id)
        request.map_id = _bytes(map_id)
        request.require = required
        request.source = source_options
        request.octomap = octomap_options
        request.activate_on_success = int(activate_on_success)
        request.require_slam_healthy = int(require_slam_healthy)
        request.minimum_point_count = max(1, int(minimum_point_count))

        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_save_map_begin_json
        args = (self._handle, ctypes.byref(request))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def provide_save_map_snapshot(
        self,
        job_id: str,
        source_dir: str | Path,
        *,
        snapshot_id: str = "",
        frame_id: str = "map",
        captured_at_ns: int = 0,
        first_sequence: int = 0,
        last_sequence: int = 0,
        source_sha256: str = "",
        slam_healthy: bool = True,
        health_message: str = "",
    ) -> dict[str, Any]:
        snapshot = _new_save_struct(_MapSnapshot)
        snapshot.snapshot_id = _bytes(snapshot_id or job_id)
        snapshot.source_dir = _bytes(source_dir)
        snapshot.frame_id = _bytes(frame_id or "map")
        snapshot.captured_at_ns = int(captured_at_ns)
        snapshot.first_sequence = max(0, int(first_sequence))
        snapshot.last_sequence = max(0, int(last_sequence))
        snapshot.source_sha256 = _bytes(source_sha256)
        snapshot.slam_healthy = int(slam_healthy)
        snapshot.health_message = _bytes(health_message)

        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_save_map_provide_snapshot_json
        args = (self._handle, _bytes(job_id), ctypes.byref(snapshot))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def get_save_map_status(self, job_id: str) -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_save_map_status_json,
            job_id,
        )

    def list_save_map_jobs(self, *, limit: int = 100) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_list_save_map_jobs_json
        value = max(1, min(int(limit), 1000))
        rc = int(
            fn(
                self._handle,
                ctypes.c_uint64(value),
                None,
                0,
                ctypes.byref(needed),
            )
        )
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(
                self._handle,
                ctypes.c_uint64(value),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def cancel_save_map(self, job_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_save_map_cancel_json,
            job_id,
        )

    def retry_save_map(self, job_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_save_map_retry_json,
            job_id,
        )

    def list_map_versions(self, map_id: str) -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_list_map_versions_json,
            map_id,
        )

    def rollback_map_version(self, map_id: str, version: int) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_rollback_map_version_json
        args = (self._handle, _bytes(map_id), ctypes.c_int64(int(version)))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def list_poi(self, map_id: str = "") -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_list_poi_json
        rc = int(fn(self._handle, _bytes(map_id), None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def set_poi(
        self,
        map_id: str,
        name: str,
        *,
        x: float,
        y: float,
        z: float = 0.0,
        yaw: float | None = None,
        frame_id: str = "map",
        tags: str | dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        import json

        tags_json = json.dumps(tags, sort_keys=True) if isinstance(tags, dict) else str(tags or "{}")
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_set_poi_json
        args = (
            self._handle,
            _bytes(map_id),
            _bytes(name),
            ctypes.c_double(float(x)),
            ctypes.c_double(float(y)),
            ctypes.c_double(float(z)),
            ctypes.c_double(0.0 if yaw is None else float(yaw)),
            ctypes.c_uint8(0 if yaw is None else 1),
            _bytes(frame_id or "map"),
            _bytes(tags_json),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def delete_poi(self, map_id: str, name: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_delete_poi_json
        args = (self._handle, _bytes(map_id), _bytes(name))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def list_map_graph(self) -> dict[str, Any]:
        return self._read_json(self._native._lib.lingtu_maps_service_list_map_graph_json)

    def set_map_edge(
        self,
        from_map_id: str,
        to_map_id: str,
        *,
        edge_type: str = "link",
        bidirectional: bool = True,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_set_map_edge_json
        args = (
            self._handle,
            _bytes(from_map_id),
            _bytes(to_map_id),
            _bytes(edge_type or "link"),
            ctypes.c_uint8(1 if bidirectional else 0),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def delete_map_edge(self, from_map_id: str, to_map_id: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_delete_map_edge_json
        args = (self._handle, _bytes(from_map_id), _bytes(to_map_id))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def shortest_route(self, start_map_id: str, goal_map_id: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_shortest_route_json
        args = (self._handle, _bytes(start_map_id), _bytes(goal_map_id))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def rollback_active_map(self) -> dict[str, Any]:
        return self._read_command(self._native._lib.lingtu_maps_service_rollback_active_map_json)

    def list_active_slots(self) -> dict[str, Any]:
        return self._read_json(self._native._lib.lingtu_maps_service_list_active_slots_json)

    def get_active_slot(self, slot: str) -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_get_active_slot_json,
            slot,
        )

    def set_active_slot(self, slot: str, map_id: str, *, strict: bool = True) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_set_active_slot_json
        args = (
            self._handle,
            _bytes(slot),
            _bytes(map_id),
            ctypes.c_uint8(1 if strict else 0),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def clear_active_slot(self, slot: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_clear_active_slot_json,
            slot,
        )

    def get_build_queue(self) -> dict[str, Any]:
        return self._read_json(self._native._lib.lingtu_maps_service_get_build_queue_json)

    def enqueue_build(self, map_id: str, artifact_type: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_enqueue_build_json
        args = (self._handle, _bytes(map_id), _bytes(artifact_type))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def pop_build_queue(self) -> dict[str, Any]:
        return self._read_command(self._native._lib.lingtu_maps_service_pop_build_queue_json)

    def get_artifact_job(self, request_id: str) -> dict[str, Any]:
        """Return one native artifact worker job."""
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_get_artifact_job_json,
            request_id,
        )

    def cancel_artifact_job(self, request_id: str) -> dict[str, Any]:
        """Request cancellation of one native artifact worker job."""
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_cancel_artifact_job_json,
            request_id,
        )

    def retry_artifact_job(self, request_id: str) -> dict[str, Any]:
        """Retry one failed or cancelled native artifact worker job."""
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_retry_artifact_job_json,
            request_id,
        )

    def _maintenance_command(self, fn, *, dry_run: bool) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        args = (self._handle, ctypes.c_uint8(int(bool(dry_run))))
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def audit_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self._maintenance_command(
            self._native._lib.lingtu_maps_service_audit_versions_json,
            dry_run=dry_run,
        )

    def quarantine_corrupt_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self._maintenance_command(
            self._native._lib.lingtu_maps_service_quarantine_versions_json,
            dry_run=dry_run,
        )

    def garbage_collect_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self._maintenance_command(
            self._native._lib.lingtu_maps_service_gc_versions_json,
            dry_run=dry_run,
        )

    def migrate_versions(self, *, dry_run: bool = True) -> dict[str, Any]:
        return self._maintenance_command(
            self._native._lib.lingtu_maps_service_migrate_versions_json,
            dry_run=dry_run,
        )

    def export_version(
        self,
        map_id: str,
        version: int,
        package_dir: str | Path,
        *,
        dry_run: bool = False,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_export_version_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_int64(int(version)),
            _bytes(package_dir),
            ctypes.c_uint8(int(bool(dry_run))),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def import_package(
        self,
        package_dir: str | Path,
        *,
        requested_map_id: str = "",
        dry_run: bool = False,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_import_package_json
        args = (
            self._handle,
            _bytes(package_dir),
            _bytes(requested_map_id),
            ctypes.c_uint8(int(bool(dry_run))),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def create_map(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_create_map_json,
            map_id,
        )

    def delete_map(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_delete_map_json,
            map_id,
        )

    def retire_map(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_retire_map_json,
            map_id,
        )

    def rename_map(self, map_id: str, new_map_id: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_rename_map_json
        rc = int(fn(self._handle, _bytes(map_id), _bytes(new_map_id), None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                _bytes(new_map_id),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def set_active_map(self, map_id: str, *, strict: bool = True) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_set_active_map_json
        rc = int(
            fn(
                self._handle,
                _bytes(map_id),
                ctypes.c_uint8(1 if strict else 0),
                None,
                0,
                ctypes.byref(needed),
            )
        )
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                ctypes.c_uint8(1 if strict else 0),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def clear_active_map(self) -> dict[str, Any]:
        return self._read_command(self._native._lib.lingtu_maps_service_clear_active_map_json)

    def begin_build(self, map_id: str, artifact_type: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_begin_build_json
        rc = int(fn(self._handle, _bytes(map_id), _bytes(artifact_type), None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                _bytes(artifact_type),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def finish_build(
        self,
        map_id: str,
        build_id: str,
        *,
        success: bool,
        message: str = "",
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_finish_build_json
        rc = int(
            fn(
                self._handle,
                _bytes(map_id),
                _bytes(build_id),
                ctypes.c_uint8(1 if success else 0),
                _bytes(message),
                None,
                0,
                ctypes.byref(needed),
            )
        )
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(
                self._handle,
                _bytes(map_id),
                _bytes(build_id),
                ctypes.c_uint8(1 if success else 0),
                _bytes(message),
                buf,
                size,
                ctypes.byref(needed),
            ),
        )

    def get_build_status(self, map_id: str) -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_get_build_status_json,
            map_id,
        )

    def import_pcd(
        self,
        map_id: str,
        source_path: str | Path,
        *,
        voxel_size: float = 0.0,
        bounds: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        has_bounds, values = self._bounds_values(bounds)
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_import_pcd_json
        args = (
            self._handle,
            _bytes(map_id),
            _bytes(source_path),
            ctypes.c_double(float(voxel_size or 0.0)),
            ctypes.c_uint8(1 if has_bounds else 0),
            *values,
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def commit_saved_source(
        self,
        map_id: str,
        source_dir: str | Path,
        *,
        voxel_size: float = 0.0,
        dynamic_filter_enabled: bool = True,
        dynamic_filter_required: bool = False,
        dynamic_filter_command: str = "",
        dynamic_filter_timeout_sec: float = 300.0,
        optimizer_strategy: str = "pgo",
        optimizer_required: bool = False,
        optimizer_command: str = "",
        optimizer_timeout_sec: float = 120.0,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_commit_saved_source_json
        args = (
            self._handle,
            _bytes(map_id),
            _bytes(source_dir),
            ctypes.c_double(float(voxel_size or 0.0)),
            ctypes.c_uint8(1 if dynamic_filter_enabled else 0),
            ctypes.c_uint8(1 if dynamic_filter_required else 0),
            _bytes(dynamic_filter_command),
            ctypes.c_double(float(dynamic_filter_timeout_sec or 300.0)),
            _bytes(optimizer_strategy or "pgo"),
            ctypes.c_uint8(1 if optimizer_required else 0),
            _bytes(optimizer_command),
            ctypes.c_double(float(optimizer_timeout_sec or 120.0)),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def crop_pcd(
        self,
        map_id: str,
        bounds: dict[str, Any],
        *,
        invert: bool = False,
        voxel_size: float = 0.0,
    ) -> dict[str, Any]:
        has_bounds, values = self._bounds_values(bounds)
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_crop_pcd_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_uint8(1 if invert else 0),
            ctypes.c_double(float(voxel_size or 0.0)),
            ctypes.c_uint8(1 if has_bounds else 0),
            *values,
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def restore_source_backup(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_restore_source_backup_json,
            map_id,
        )

    def build_occupancy_snapshot(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_build_occupancy_snapshot_json,
            map_id,
        )

    def build_octomap_artifact(
        self,
        map_id: str,
        *,
        converter_command: str = "",
        build_mode: str = "external_pcl_converter",
        resolution: float = 0.20,
        support_dilation_cells: int = 1,
        free_layers_above: int = 3,
        free_dilation_cells: int = 1,
        frame_id: str = "map",
        source_profile: str = "maps_pipeline",
        data_source: str = "maps_pipeline",
        slam_source: str = "unknown",
        localization_source: str = "unknown",
        mapping_source: str = "lingtu_maps_pipeline",
        timeout_sec: float = 60.0,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_build_octomap_artifact_json
        args = (
            self._handle,
            _bytes(map_id),
            _bytes(converter_command),
            _bytes(build_mode),
            ctypes.c_double(float(resolution)),
            ctypes.c_int32(int(support_dilation_cells)),
            ctypes.c_int32(int(free_layers_above)),
            ctypes.c_int32(int(free_dilation_cells)),
            _bytes(frame_id),
            _bytes(source_profile),
            _bytes(data_source),
            _bytes(slam_source),
            _bytes(localization_source),
            _bytes(mapping_source),
            ctypes.c_double(float(timeout_sec)),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def get_voxel_edits(self, map_id: str) -> dict[str, Any]:
        return self._read_json_with_id(
            self._native._lib.lingtu_maps_service_get_voxel_edits_json,
            map_id,
        )

    def edit_octomap_voxels(
        self,
        map_id: str,
        *,
        editor_command: str = "",
        state: str,
        shape: str = "sphere",
        x: float,
        y: float,
        z: float = 0.0,
        radius: float = 0.2,
        timeout_sec: float = 15.0,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_edit_octomap_voxels_json
        args = (
            self._handle,
            _bytes(map_id),
            _bytes(editor_command),
            _bytes(state),
            _bytes(shape),
            ctypes.c_double(float(x)),
            ctypes.c_double(float(y)),
            ctypes.c_double(float(z)),
            ctypes.c_double(float(radius)),
            ctypes.c_double(float(timeout_sec)),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def build_navigation_package(
        self,
        map_id: str,
        *,
        converter_command: str = "",
        build_mode: str = "external_pcl_converter",
        resolution: float = 0.20,
        support_dilation_cells: int = 1,
        free_layers_above: int = 3,
        free_dilation_cells: int = 1,
        frame_id: str = "map",
        source_profile: str = "maps_pipeline",
        data_source: str = "maps_pipeline",
        slam_source: str = "unknown",
        localization_source: str = "unknown",
        mapping_source: str = "lingtu_maps_pipeline",
        timeout_sec: float = 60.0,
        include_esdf: bool = True,
        include_traversability: bool = True,
    ) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        fn = self._native._lib.lingtu_maps_service_build_navigation_package_json
        args = (
            self._handle,
            _bytes(map_id),
            _bytes(converter_command),
            _bytes(build_mode),
            ctypes.c_double(float(resolution)),
            ctypes.c_int32(int(support_dilation_cells)),
            ctypes.c_int32(int(free_layers_above)),
            ctypes.c_int32(int(free_dilation_cells)),
            _bytes(frame_id),
            _bytes(source_profile),
            _bytes(data_source),
            _bytes(slam_source),
            _bytes(localization_source),
            _bytes(mapping_source),
            ctypes.c_double(float(timeout_sec)),
            ctypes.c_uint8(1 if include_esdf else 0),
            ctypes.c_uint8(1 if include_traversability else 0),
        )
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(*args, buf, size, ctypes.byref(needed)),
        )

    def build_esdf_artifact(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_build_esdf_artifact_json,
            map_id,
        )

    def build_traversability_artifact(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_build_traversability_artifact_json,
            map_id,
        )

    def build_semantic_artifact(self, map_id: str) -> dict[str, Any]:
        return self._read_command_with_id(
            self._native._lib.lingtu_maps_service_build_semantic_artifact_json,
            map_id,
        )

    def _read_json(self, fn) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        rc = int(fn(self._handle, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(self._handle, buf, size, ctypes.byref(needed)),
        )

    def _read_json_with_id(self, fn, map_id: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        rc = int(fn(self._handle, _bytes(map_id), None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json(
            needed,
            lambda buf, size: fn(self._handle, _bytes(map_id), buf, size, ctypes.byref(needed)),
        )

    def _read_command(self, fn) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        rc = int(fn(self._handle, None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(self._handle, buf, size, ctypes.byref(needed)),
        )

    def _read_command_with_id(self, fn, map_id: str) -> dict[str, Any]:
        needed = ctypes.c_uint64(0)
        rc = int(fn(self._handle, _bytes(map_id), None, 0, ctypes.byref(needed)))
        self._check_probe(rc)
        return self._decode_json_from_command(
            needed,
            lambda buf, size: fn(self._handle, _bytes(map_id), buf, size, ctypes.byref(needed)),
        )

    def _decode_json_from_command(self, needed: ctypes.c_uint64, call) -> dict[str, Any]:
        return self._decode_json(needed, call, command=True)

    def _decode_json(self, needed: ctypes.c_uint64, call, *, command: bool = False) -> dict[str, Any]:
        size = max(2, int(needed.value))
        for _ in range(4):
            buf = ctypes.create_string_buffer(size)
            rc = int(call(buf, size))
            if rc != 1:
                break
            size = max(size + 1, int(needed.value))
        self._check_rc(rc)
        loaded = json.loads(buf.value.decode("utf-8") or "{}")
        return loaded if isinstance(loaded, dict) else {}

    @staticmethod
    def _bounds_values(bounds: dict[str, Any] | None) -> tuple[bool, tuple[ctypes.c_double, ...]]:
        if not bounds:
            return (
                False,
                (
                    ctypes.c_double(0.0),
                    ctypes.c_double(0.0),
                    ctypes.c_double(0.0),
                    ctypes.c_double(0.0),
                    ctypes.c_double(0.0),
                    ctypes.c_double(0.0),
                ),
            )
        if isinstance(bounds.get("min"), (list, tuple)) and isinstance(bounds.get("max"), (list, tuple)):
            lower = list(bounds["min"][:3])
            upper = list(bounds["max"][:3])
        else:
            lower = [
                bounds.get("min_x", bounds.get("x_min")),
                bounds.get("min_y", bounds.get("y_min")),
                bounds.get("min_z", bounds.get("z_min")),
            ]
            upper = [
                bounds.get("max_x", bounds.get("x_max")),
                bounds.get("max_y", bounds.get("y_max")),
                bounds.get("max_z", bounds.get("z_max")),
            ]
        if len(lower) != 3 or len(upper) != 3 or any(value is None for value in lower + upper):
            raise ValueError("bounds must provide min/max xyz values")
        values = [float(value) for value in lower + upper]
        if values[0] > values[3] or values[1] > values[4] or values[2] > values[5]:
            raise ValueError("bounds min must be <= max")
        return True, tuple(ctypes.c_double(value) for value in values)

    @staticmethod
    def _check_probe(rc: int) -> None:
        value = int(rc)
        if value == 1:
            return
        if value == 0:
            return
        raise RuntimeError(f"lingtu_maps service probe failed: {value}")

    @staticmethod
    def _check_rc(rc: int) -> None:
        value = int(rc)
        if value == 0:
            return
        raise RuntimeError(f"lingtu_maps service call failed: {value}")
