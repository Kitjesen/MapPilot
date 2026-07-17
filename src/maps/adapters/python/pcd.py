"""Native binary PCD writer used by map-source capture adapters."""

from __future__ import annotations

import ctypes
import ctypes.util
from functools import lru_cache
from pathlib import Path

from maps.adapters.python.store import _native_library_candidates
from runtime.msgs.numpy_compat import np


class PcdNativeUnavailable(RuntimeError):
    """Raised when the product maps library cannot be loaded."""


class _NativePcdLib:
    def __init__(self, lib: ctypes.CDLL) -> None:
        self._lib = lib
        self._lib.lingtu_maps_write_xyz_pcd.argtypes = [
            ctypes.c_char_p,
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_uint32,
            ctypes.c_float,
            ctypes.POINTER(ctypes.c_uint64),
        ]
        self._lib.lingtu_maps_write_xyz_pcd.restype = ctypes.c_int32


@lru_cache(maxsize=1)
def _load_native_pcd_lib() -> _NativePcdLib | None:
    for candidate in _native_library_candidates():
        if candidate.is_file():
            try:
                return _NativePcdLib(ctypes.CDLL(str(candidate)))
            except (AttributeError, OSError):
                continue
    found = ctypes.util.find_library("lingtu_maps")
    if found:
        try:
            return _NativePcdLib(ctypes.CDLL(found))
        except (AttributeError, OSError):
            pass
    return None


class NativePcdWriter:
    """Thin array-to-C-ABI adapter; filtering and PCD encoding stay in C++."""

    def __init__(self) -> None:
        self._native = _load_native_pcd_lib()
        if self._native is None:
            raise PcdNativeUnavailable(
                "native lingtu_maps PCD writer is unavailable; build src/maps and set LINGTU_MAPS_LIB"
            )

    def write_xyz(
        self,
        path: str | Path,
        points: np.ndarray,
        *,
        max_abs_m: float = 500.0,
    ) -> int:
        xyz = np.ascontiguousarray(points, dtype=np.float32)
        if xyz.ndim != 2 or xyz.shape[1] < 3:
            raise ValueError("points must have shape Nx3 or wider")
        written = ctypes.c_uint64(0)
        rc = self._native._lib.lingtu_maps_write_xyz_pcd(
            str(Path(path)).encode("utf-8"),
            ctypes.c_void_p(xyz.ctypes.data),
            ctypes.c_uint64(xyz.shape[0]),
            ctypes.c_uint32(xyz.shape[1]),
            ctypes.c_float(max_abs_m),
            ctypes.byref(written),
        )
        if rc != 0:
            raise RuntimeError(f"native PCD write failed with status {rc}")
        return int(written.value)
