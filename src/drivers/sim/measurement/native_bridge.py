"""Owned ctypes reader for native typed DDS measurements; no Python DDS runtime."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
from threading import RLock
from typing import Any

from runtime.endpoints.dds.measurement import measurement_from_record

MAX_PACKET_BYTES = 32 * 1024 * 1024


class MeasurementBridgeError(RuntimeError):
    """Native reader unavailable or violated its bounded C ABI contract."""


def _identity(value: str) -> bytes:
    if not isinstance(value, str) or not value or value != value.strip() or "\0" in value:
        raise ValueError("measurement identity must be nonempty trimmed UTF-8")
    encoded = value.encode("utf-8")
    if len(encoded) > 255:
        raise ValueError("measurement identity exceeds 255 UTF-8 bytes")
    return encoded


def _library_path() -> Path:
    explicit = os.environ.get("LINGTU_MEASUREMENT_BRIDGE_LIBRARY", "")
    if explicit:
        candidates = [Path(explicit)]
    else:
        root = Path(__file__).resolve().parents[4]
        candidates = (
            [root / "build/windows-native-dds-adapter/Release/lingtu_measurement_bridge.dll"]
            if os.name == "nt"
            else [
                root / "build/mujoco_native_dds/liblingtu_measurement_bridge.so",
                Path("/opt/lingtu/current/lib/liblingtu_measurement_bridge.so"),
            ]
        )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise MeasurementBridgeError("native measurement bridge missing; set LINGTU_MEASUREMENT_BRIDGE_LIBRARY")


class NativeMeasurementBridge:
    """Serialize take/close on one reader; buffers never outlive native loans."""

    def __init__(self, *, domain_id: int, product_session_id: str, session_id: str, library: Any = None):
        if type(domain_id) is not int or not 0 <= domain_id <= 232:
            raise ValueError("domain_id must be an integer in 0..232")
        product, source = _identity(product_session_id), _identity(session_id)
        self.product_session_id, self.session_id = product_session_id, session_id
        self._lock = RLock()
        self._handle = None
        self._dll_directory = None
        try:
            if library is None:
                path = _library_path()
                if os.name == "nt":
                    self._dll_directory = os.add_dll_directory(str(path.parent))
                library = ctypes.CDLL(str(path))
            self._lib = library
            library.lt_measurement_create.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p]
            library.lt_measurement_create.restype = ctypes.c_void_p
            library.lt_measurement_take.argtypes = [
                ctypes.c_void_p,
                ctypes.c_void_p,
                ctypes.c_size_t,
                ctypes.POINTER(ctypes.c_size_t),
            ]
            library.lt_measurement_take.restype = ctypes.c_int
            library.lt_measurement_destroy.argtypes = [ctypes.c_void_p]
            library.lt_measurement_destroy.restype = None
            library.lt_measurement_last_error.argtypes = []
            library.lt_measurement_last_error.restype = ctypes.c_char_p
            self._handle = library.lt_measurement_create(domain_id, product, source)
            if not self._handle:
                raise MeasurementBridgeError(self._error())
        except Exception:
            self.close()
            raise

    def _error(self) -> str:
        raw = self._lib.lt_measurement_last_error()
        return raw.decode("utf-8", errors="replace") if raw else "native measurement reader failed"

    def take(self, *, received_at_s: float):
        with self._lock:
            if self._handle is None:
                raise MeasurementBridgeError("native measurement reader is closed")
            needed = ctypes.c_size_t()
            result = self._lib.lt_measurement_take(self._handle, None, 0, ctypes.byref(needed))
            if result == 0 and needed.value == 0:
                return None
            if result == -1:
                raise MeasurementBridgeError(self._error())
            if result != -2 or not 76 <= needed.value <= MAX_PACKET_BYTES:
                raise MeasurementBridgeError("native measurement packet size contract failed")
            size = needed.value
            buffer = ctypes.create_string_buffer(size)
            result = self._lib.lt_measurement_take(self._handle, buffer, size, ctypes.byref(needed))
            if result != 1 or needed.value != size:
                raise MeasurementBridgeError(
                    "native measurement pending sample changed or read failed: " + self._error()
                )
            return measurement_from_record(
                buffer.raw,
                received_at_s=received_at_s,
                expected_product_session_id=self.product_session_id,
                expected_session_id=self.session_id,
            )

    def close(self) -> None:
        with self._lock:
            try:
                if self._handle is not None:
                    self._lib.lt_measurement_destroy(self._handle)
                    self._handle = None
            finally:
                if self._dll_directory is not None:
                    self._dll_directory.close()
                    self._dll_directory = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
