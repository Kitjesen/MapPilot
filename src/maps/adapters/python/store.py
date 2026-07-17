"""ctypes adapter for the native C++ MapStore."""

from __future__ import annotations

import ctypes
import ctypes.util
import json
import os
import sys
from functools import lru_cache
from pathlib import Path


class MapStoreNativeUnavailable(RuntimeError):
    """Raised when the native lingtu_maps store ABI is unavailable."""


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
        repo / ".tmp" / "maps-cmake-grid",
        repo / ".tmp" / "maps-cmake-grid" / "Release",
        repo / ".tmp" / "maps-cmake-grid" / "Debug",
        repo / ".tmp" / "maps-cmake",
        repo / ".tmp" / "maps-cmake" / "Release",
        repo / ".tmp" / "maps-cmake" / "Debug",
    )
    for root in roots:
        for name in _library_names():
            candidates.append(root / name)
    return candidates


class _NativeStoreLib:
    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._configure()

    def _configure(self) -> None:
        self._lib.lingtu_maps_abi_version.argtypes = []
        self._lib.lingtu_maps_abi_version.restype = ctypes.c_uint32

        self._lib.lingtu_maps_store_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self._lib.lingtu_maps_store_create.restype = ctypes.c_void_p

        self._lib.lingtu_maps_store_destroy.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_store_destroy.restype = None

        self._lib.lingtu_maps_store_validate_map_id.argtypes = [ctypes.c_char_p]
        self._lib.lingtu_maps_store_validate_map_id.restype = ctypes.c_int32

        string_out_args = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_store_list_map_ids.argtypes = string_out_args
        self._lib.lingtu_maps_store_list_map_ids.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_active_map_id.argtypes = string_out_args
        self._lib.lingtu_maps_store_active_map_id.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_list_records_json.argtypes = string_out_args
        self._lib.lingtu_maps_store_list_records_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_active_record_json.argtypes = string_out_args
        self._lib.lingtu_maps_store_active_record_json.restype = ctypes.c_int32
        record_json_args = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_store_record_json.argtypes = record_json_args
        self._lib.lingtu_maps_store_record_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_bundle_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_store_bundle_json.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_validate_artifacts_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_store_validate_artifacts_json.restype = ctypes.c_int32

        self._lib.lingtu_maps_store_create_map.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._lib.lingtu_maps_store_create_map.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_delete_map.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._lib.lingtu_maps_store_delete_map.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_rename_map.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
        ]
        self._lib.lingtu_maps_store_rename_map.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_set_active_map.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint8,
        ]
        self._lib.lingtu_maps_store_set_active_map.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_clear_active_map.argtypes = [ctypes.c_void_p]
        self._lib.lingtu_maps_store_clear_active_map.restype = ctypes.c_int32
        self._lib.lingtu_maps_store_artifact_count.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_store_artifact_count.restype = ctypes.c_int32

        abi = int(self._lib.lingtu_maps_abi_version())
        if abi != 1:
            raise MapStoreNativeUnavailable(f"unsupported lingtu_maps ABI version: {abi}")


@lru_cache(maxsize=1)
def _load_native_store_lib() -> _NativeStoreLib | None:
    for candidate in _native_library_candidates():
        if candidate.is_file():
            try:
                return _NativeStoreLib(ctypes.CDLL(str(candidate)))
            except (OSError, AttributeError, MapStoreNativeUnavailable):
                continue
    found = ctypes.util.find_library("lingtu_maps")
    if found:
        try:
            return _NativeStoreLib(ctypes.CDLL(found))
        except (OSError, AttributeError, MapStoreNativeUnavailable):
            return None
    return None


def _bytes(value: str | Path | None) -> bytes | None:
    if value is None:
        return None
    return str(value).encode("utf-8")


class NativeMapStore:
    """Python-owned handle for C++ MapStore."""

    def __init__(self, root_dir: str | Path, active_state_filename: str = "active_map.txt") -> None:
        lib = _load_native_store_lib()
        if lib is None:
            raise MapStoreNativeUnavailable("lingtu_maps native library not found")
        self._native = lib
        self._handle = self._native._lib.lingtu_maps_store_create(
            _bytes(root_dir),
            _bytes(active_state_filename),
        )
        if not self._handle:
            raise MapStoreNativeUnavailable("lingtu_maps store create failed")

    @staticmethod
    def validate_map_id(map_id: str) -> bool:
        lib = _load_native_store_lib()
        if lib is None:
            raise MapStoreNativeUnavailable("lingtu_maps native library not found")
        return int(lib._lib.lingtu_maps_store_validate_map_id(_bytes(map_id))) == 1

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if handle:
            self._native._lib.lingtu_maps_store_destroy(handle)
            self._handle = None

    def __del__(self) -> None:
        self.close()

    def list_map_ids(self) -> list[str]:
        payload = self._read_string(self._native._lib.lingtu_maps_store_list_map_ids)
        return [item for item in payload.splitlines() if item]

    def active_map_id(self) -> str:
        return self._read_string(self._native._lib.lingtu_maps_store_active_map_id)

    def list_records(self) -> list[dict]:
        payload = self._read_string(self._native._lib.lingtu_maps_store_list_records_json)
        loaded = json.loads(payload or "[]")
        return loaded if isinstance(loaded, list) else []

    def record(self, map_id: str) -> dict | None:
        payload = self._read_string_with_map_id(
            self._native._lib.lingtu_maps_store_record_json,
            map_id,
            missing_ok=True,
        )
        if not payload:
            return None
        loaded = json.loads(payload)
        return loaded if isinstance(loaded, dict) else None

    def active_record(self) -> dict | None:
        payload = self._read_string(
            self._native._lib.lingtu_maps_store_active_record_json,
            missing_ok=True,
        )
        if not payload:
            return None
        loaded = json.loads(payload)
        return loaded if isinstance(loaded, dict) else None

    def bundle(self, map_id: str, capability: str) -> dict:
        payload = self._read_string_with_two_values(
            self._native._lib.lingtu_maps_store_bundle_json,
            map_id,
            capability,
        )
        loaded = json.loads(payload or "{}")
        return loaded if isinstance(loaded, dict) else {}

    def validate_artifacts(
        self,
        map_id: str,
        *,
        require_octomap: bool = False,
        require_occupancy: bool = False,
        expected_frame_id: str = "",
    ) -> dict:
        """Validate saved-map identity without constructing the full Maps service."""
        fn = self._native._lib.lingtu_maps_store_validate_artifacts_json
        args = (
            self._handle,
            _bytes(map_id),
            ctypes.c_uint8(int(bool(require_octomap))),
            ctypes.c_uint8(int(bool(require_occupancy))),
            _bytes(expected_frame_id),
        )
        needed = ctypes.c_uint64(0)
        rc = int(fn(*args, None, 0, ctypes.byref(needed)))
        if rc < 0:
            self._check_rc(rc)
        size = max(1, int(needed.value))
        buf = ctypes.create_string_buffer(size)
        self._check_rc(
            int(fn(*args, buf, size, ctypes.byref(needed))),
            allow_truncated=False,
        )
        loaded = json.loads(buf.value.decode("utf-8") or "{}")
        return loaded if isinstance(loaded, dict) else {}

    def create_map(self, map_id: str) -> None:
        self._check_rc(self._native._lib.lingtu_maps_store_create_map(self._handle, _bytes(map_id)))

    def delete_map(self, map_id: str) -> None:
        self._check_rc(self._native._lib.lingtu_maps_store_delete_map(self._handle, _bytes(map_id)))

    def rename_map(self, map_id: str, new_map_id: str) -> None:
        self._check_rc(
            self._native._lib.lingtu_maps_store_rename_map(
                self._handle,
                _bytes(map_id),
                _bytes(new_map_id),
            )
        )

    def set_active_map(self, map_id: str, *, strict: bool = True) -> None:
        self._check_rc(
            self._native._lib.lingtu_maps_store_set_active_map(
                self._handle,
                _bytes(map_id),
                ctypes.c_uint8(1 if strict else 0),
            )
        )

    def clear_active_map(self) -> None:
        self._check_rc(self._native._lib.lingtu_maps_store_clear_active_map(self._handle))

    def artifact_count(self, map_id: str) -> int:
        out = ctypes.c_uint64(0)
        self._check_rc(
            self._native._lib.lingtu_maps_store_artifact_count(
                self._handle,
                _bytes(map_id),
                ctypes.byref(out),
            )
        )
        return int(out.value)

    def _read_string(self, fn, *, missing_ok: bool = False) -> str:
        needed = ctypes.c_uint64(0)
        rc = int(fn(self._handle, None, 0, ctypes.byref(needed)))
        if missing_ok and rc == 2:
            return ""
        if rc < 0:
            self._check_rc(rc)
        size = max(1, int(needed.value))
        buf = ctypes.create_string_buffer(size)
        rc = int(fn(self._handle, buf, size, ctypes.byref(needed)))
        if missing_ok and rc == 2:
            return ""
        self._check_rc(rc, allow_truncated=False)
        return buf.value.decode("utf-8")

    def _read_string_with_map_id(self, fn, map_id: str, *, missing_ok: bool = False) -> str:
        needed = ctypes.c_uint64(0)
        rc = int(fn(self._handle, _bytes(map_id), None, 0, ctypes.byref(needed)))
        if missing_ok and rc == 2:
            return ""
        if rc < 0:
            self._check_rc(rc)
        size = max(1, int(needed.value))
        buf = ctypes.create_string_buffer(size)
        rc = int(fn(self._handle, _bytes(map_id), buf, size, ctypes.byref(needed)))
        if missing_ok and rc == 2:
            return ""
        self._check_rc(rc, allow_truncated=False)
        return buf.value.decode("utf-8")

    def _read_string_with_two_values(self, fn, first: str, second: str) -> str:
        needed = ctypes.c_uint64(0)
        rc = int(
            fn(
                self._handle,
                _bytes(first),
                _bytes(second),
                None,
                0,
                ctypes.byref(needed),
            )
        )
        if rc < 0:
            self._check_rc(rc)
        size = max(1, int(needed.value))
        buf = ctypes.create_string_buffer(size)
        rc = int(
            fn(
                self._handle,
                _bytes(first),
                _bytes(second),
                buf,
                size,
                ctypes.byref(needed),
            )
        )
        self._check_rc(rc, allow_truncated=False)
        return buf.value.decode("utf-8")

    @staticmethod
    def _check_rc(rc: int, *, allow_truncated: bool = False) -> None:
        value = int(rc)
        if value == 0:
            return
        if allow_truncated and value == 1:
            return
        raise RuntimeError(f"lingtu_maps store call failed: {value}")
