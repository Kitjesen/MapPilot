"""Shared ctypes adapter for the native C++ inspection route store.

This module performs ABI marshalling only. Route validation, persistence, and
execution remain in C++.
"""

from __future__ import annotations

import ctypes
import json
import os
import platform
from pathlib import Path
from typing import Any

NATIVE_INSPECTION_STORE_ABI_VERSION = 1


class InspectionNativeError(RuntimeError):
    """Raised when the native inspection ABI rejects an operation."""


class _Point(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_char_p),
        ("x_m", ctypes.c_double),
        ("y_m", ctypes.c_double),
        ("z_m", ctypes.c_double),
        ("yaw_rad", ctypes.c_double),
        ("has_yaw", ctypes.c_int32),
        ("position_tolerance_m", ctypes.c_double),
        ("yaw_tolerance_rad", ctypes.c_double),
        ("dwell_s", ctypes.c_double),
        ("action", ctypes.c_char_p),
        ("enabled", ctypes.c_int32),
    ]


class _Route(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_char_p),
        ("name", ctypes.c_char_p),
        ("map_id", ctypes.c_char_p),
        ("map_content_epoch", ctypes.c_int64),
        ("revision", ctypes.c_uint64),
        ("loop_count", ctypes.c_uint32),
        ("failure_policy", ctypes.c_int32),
        ("max_retries", ctypes.c_uint32),
        ("points", ctypes.POINTER(_Point)),
        ("point_count", ctypes.c_uint64),
    ]


def _library_names() -> tuple[str, ...]:
    if platform.system() == "Windows":
        return ("lingtu_inspection.dll",)
    if platform.system() == "Darwin":
        return ("liblingtu_inspection.dylib",)
    return ("liblingtu_inspection.so",)


def _library_candidates() -> list[Path]:
    explicit = os.environ.get("LINGTU_INSPECTION_LIBRARY", "").strip()
    root = Path(__file__).resolve().parents[4]
    candidates = [Path(explicit)] if explicit else []
    for name in _library_names():
        candidates.extend(
            [
                root / "src" / "nav" / "inspection" / "build" / name,
                root / "src" / "nav" / "inspection" / "build" / "Release" / name,
                root / "build" / "nav_endpoint" / "inspection" / name,
                root / "build" / "nav_endpoint" / "inspection" / "Release" / name,
                Path("/opt/lingtu/current/build/nav_endpoint/inspection") / name,
                Path("/opt/lingtu/current/lib") / name,
            ]
        )
    return candidates


def _load_library() -> ctypes.CDLL:
    for candidate in _library_candidates():
        if candidate.is_file():
            return ctypes.CDLL(str(candidate))
    raise InspectionNativeError("native inspection library not found; set LINGTU_INSPECTION_LIBRARY")


class NativeInspectionStore:
    """Typed owner of one native route-store handle."""

    def __init__(self, data_dir: str | os.PathLike[str]) -> None:
        self._lib = _load_library()
        self._configure_core_abi()
        self._validate_abi()
        self._configure_abi()
        self._handle = self._lib.lingtu_inspection_store_create(os.fspath(data_dir).encode("utf-8"))
        if not self._handle:
            raise InspectionNativeError("failed to create native inspection store")

    def _configure_core_abi(self) -> None:
        try:
            self._lib.lingtu_inspection_store_abi_version.argtypes = []
            self._lib.lingtu_inspection_store_abi_version.restype = ctypes.c_uint32
        except AttributeError as exc:
            raise InspectionNativeError(
                "native inspection store ABI metadata is missing; rebuild liblingtu_inspection.so"
            ) from exc

    def _validate_abi(self) -> None:
        version = int(self._lib.lingtu_inspection_store_abi_version())
        if version != NATIVE_INSPECTION_STORE_ABI_VERSION:
            raise InspectionNativeError(
                "native inspection store ABI version mismatch: "
                f"expected {NATIVE_INSPECTION_STORE_ABI_VERSION}, got {version}"
            )

    def _configure_abi(self) -> None:
        lib = self._lib
        lib.lingtu_inspection_store_create.argtypes = [ctypes.c_char_p]
        lib.lingtu_inspection_store_create.restype = ctypes.c_void_p
        lib.lingtu_inspection_store_destroy.argtypes = [ctypes.c_void_p]
        lib.lingtu_inspection_store_destroy.restype = None
        lib.lingtu_inspection_store_put.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Route),
        ]
        lib.lingtu_inspection_store_put.restype = ctypes.c_int32
        lib.lingtu_inspection_store_delete.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
        ]
        lib.lingtu_inspection_store_delete.restype = ctypes.c_int32
        for name in (
            "lingtu_inspection_store_get_json",
            "lingtu_inspection_store_list_json",
        ):
            fn = getattr(lib, name)
            fn.restype = ctypes.c_void_p
        lib.lingtu_inspection_store_get_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
        ]
        lib.lingtu_inspection_store_list_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
        ]
        lib.lingtu_inspection_store_status_json.argtypes = [ctypes.c_void_p]
        lib.lingtu_inspection_store_status_json.restype = ctypes.c_void_p
        lib.lingtu_inspection_store_last_error.argtypes = [ctypes.c_void_p]
        lib.lingtu_inspection_store_last_error.restype = ctypes.c_char_p
        lib.lingtu_inspection_string_free.argtypes = [ctypes.c_void_p]
        lib.lingtu_inspection_string_free.restype = None

    def close(self) -> None:
        if self._handle:
            self._lib.lingtu_inspection_store_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> NativeInspectionStore:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def _error(self) -> str:
        raw = self._lib.lingtu_inspection_store_last_error(self._handle)
        return bytes(raw or b"native inspection operation failed").decode("utf-8", errors="replace")

    def _json_result(self, pointer: int | None) -> dict[str, Any]:
        if not pointer:
            raise InspectionNativeError(self._error())
        try:
            raw = ctypes.string_at(pointer).decode("utf-8")
            result = json.loads(raw)
            if not isinstance(result, dict):
                raise InspectionNativeError("native inspection response is not an object")
            return result
        finally:
            self._lib.lingtu_inspection_string_free(pointer)

    def put(self, route: dict[str, Any]) -> dict[str, Any]:
        raw_points = route.get("points")
        if not isinstance(raw_points, list):
            raise InspectionNativeError("route points must be a list")
        point_array = (_Point * len(raw_points))()
        encoded: list[bytes] = []

        def text(value: Any) -> bytes:
            item = str(value or "").encode("utf-8")
            encoded.append(item)
            return item

        for index, item in enumerate(raw_points):
            if not isinstance(item, dict):
                raise InspectionNativeError("route point must be an object")
            yaw = item.get("yaw")
            point_array[index] = _Point(
                text(item.get("id")),
                float(item.get("x", 0.0)),
                float(item.get("y", 0.0)),
                float(item.get("z", 0.0)),
                float(yaw or 0.0),
                int(yaw is not None),
                float(item.get("position_tolerance_m", 0.35)),
                float(item.get("yaw_tolerance_rad", 0.35)),
                float(item.get("dwell_s", 0.0)),
                text(item.get("action")),
                int(bool(item.get("enabled", True))),
            )
        policy = {"stop": 0, "retry": 1, "skip": 2}.get(str(route.get("failure_policy", "stop")).lower(), 0)
        native = _Route(
            text(route.get("id")),
            text(route.get("name") or route.get("id")),
            text(route.get("map_id")),
            int(route.get("map_content_epoch", 0)),
            int(route.get("revision", 1)),
            int(route.get("loop_count", 1)),
            policy,
            int(route.get("max_retries", 0)),
            point_array,
            len(raw_points),
        )
        if self._lib.lingtu_inspection_store_put(self._handle, ctypes.byref(native)) != 0:
            raise InspectionNativeError(self._error())
        return self.get(str(route.get("map_id", "")), str(route.get("id", "")))

    def get(self, map_id: str, route_id: str) -> dict[str, Any]:
        return self._json_result(
            self._lib.lingtu_inspection_store_get_json(
                self._handle,
                map_id.encode("utf-8"),
                route_id.encode("utf-8"),
            )
        )

    def list(self, map_id: str) -> dict[str, Any]:
        return self._json_result(self._lib.lingtu_inspection_store_list_json(self._handle, map_id.encode("utf-8")))

    def status(self) -> dict[str, Any]:
        return self._json_result(self._lib.lingtu_inspection_store_status_json(self._handle))

    def delete(self, map_id: str, route_id: str) -> None:
        result = self._lib.lingtu_inspection_store_delete(
            self._handle,
            map_id.encode("utf-8"),
            route_id.encode("utf-8"),
        )
        if result != 0:
            raise InspectionNativeError(self._error())
