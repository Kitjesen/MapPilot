"""Optional point-cloud native backend boundary.

The portable runtime must not require PCL.  This module looks for an optional
``lingtu_pcl_ops`` native package and falls back to a deterministic NumPy
implementation when the package is absent.  The public API uses plain arrays so
callers never depend on PCL types, ``pcl_ros``, or ``pcl_conversions``.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Any

from runtime.msgs.numpy_compat import np

_NATIVE_MODULE = "lingtu_pcl_ops"


@dataclass(frozen=True)
class PclOpsBackendInfo:
    """Resolved optional PCL backend state."""

    available: bool
    backend: str
    reason: str = ""


def _load_native() -> Any | None:
    try:
        return importlib.import_module(_NATIVE_MODULE)
    except Exception:
        return None


def pcl_ops_available() -> bool:
    """Return True when the optional native PCL plugin can be imported."""

    return _load_native() is not None


def backend_info() -> PclOpsBackendInfo:
    """Describe the active point-cloud ops backend."""

    native = _load_native()
    if native is None:
        return PclOpsBackendInfo(
            available=False,
            backend="numpy_fallback",
            reason="optional lingtu_pcl_ops native package is not installed",
        )
    return PclOpsBackendInfo(available=True, backend=_NATIVE_MODULE)


def voxel_downsample_xyzi(
    points: Any,
    voxel_size: float,
    *,
    prefer_native: bool = True,
) -> np.ndarray:
    """Downsample ``Nx3``/``Nx4`` points using native PCL when available.

    Parameters
    ----------
    points:
        Point array with XYZ or XYZI columns.
    voxel_size:
        Voxel edge length in metres.  Must be positive.
    prefer_native:
        When true, use ``lingtu_pcl_ops.voxel_downsample_xyzi`` if installed;
        otherwise use the deterministic NumPy fallback.
    """

    pts = _normalize_points(points)
    if prefer_native:
        native = _load_native()
        fn = getattr(native, "voxel_downsample_xyzi", None) if native is not None else None
        if fn is not None:
            return _normalize_points(fn(pts, float(voxel_size)))
    return _numpy_voxel_downsample_xyzi(pts, voxel_size)


def _normalize_points(points: Any) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32)
    if pts.ndim == 1 and pts.size == 0:
        return pts.reshape(0, 4)
    if pts.ndim != 2 or pts.shape[1] not in (3, 4):
        raise ValueError(f"points must be shaped (N,3) or (N,4), got {pts.shape}")
    return np.ascontiguousarray(pts, dtype=np.float32)


def _numpy_voxel_downsample_xyzi(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0:
        raise ValueError(f"voxel_size must be positive, got {voxel_size}")
    if points.size == 0:
        return points.copy()

    coords = np.floor(points[:, :3] / float(voxel_size)).astype(np.int64)
    _voxels, inverse, counts = np.unique(
        coords,
        axis=0,
        return_inverse=True,
        return_counts=True,
    )
    downsampled = np.zeros((int(counts.shape[0]), int(points.shape[1])), dtype=np.float64)
    np.add.at(downsampled, inverse, points.astype(np.float64, copy=False))
    downsampled /= counts[:, None]
    return downsampled.astype(np.float32, copy=False)
