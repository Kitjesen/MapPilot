"""Compatibility import for the old real camera module path."""

from __future__ import annotations

from .module import (
    _FMT_BGR8,
    _FMT_DEPTH_U16,
    _FMT_RGB8,
    _HEADER,
    _KIND_COLOR,
    _KIND_DEPTH,
    _KIND_INTRINSICS,
    _MAGIC,
    _VERSION,
    OrbbecNativeCameraModule,
    _default_executable,
    _runtime_library_paths,
    orbbec_native_build_dir,
    orbbec_sdk_root,
    orbbec_sdk_source,
)

CameraModule = OrbbecNativeCameraModule

__all__ = [
    "CameraModule",
    "OrbbecNativeCameraModule",
    "orbbec_native_build_dir",
    "orbbec_sdk_root",
    "orbbec_sdk_source",
]
