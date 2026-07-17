"""ctypes adapter for the native inspection evidence DDS bridge.

The robot main path must not depend on cyclonedds-python. This adapter only
marshals fixed-size C ABI structs exposed by liblingtu_inspection_evidence_bridge.
"""

from __future__ import annotations

import ctypes
import os
import platform
from pathlib import Path
from typing import Any

ID_CAP = 128
ACTION_CAP = 64
REASON_CAP = 256


class InspectionEvidenceBridgeError(RuntimeError):
    """Raised when the native evidence DDS bridge rejects an operation."""


class _Request(ctypes.Structure):
    _fields_ = [
        ("requested_at_s", ctypes.c_double),
        ("request_id", ctypes.c_char * ID_CAP),
        ("run_id", ctypes.c_char * ID_CAP),
        ("route_id", ctypes.c_char * ID_CAP),
        ("route_revision", ctypes.c_uint64),
        ("map_id", ctypes.c_char * ID_CAP),
        ("map_version", ctypes.c_int64),
        ("point_index", ctypes.c_uint32),
        ("point_id", ctypes.c_char * ID_CAP),
        ("action", ctypes.c_char * ACTION_CAP),
        ("deadline_s", ctypes.c_double),
    ]


class _Result(ctypes.Structure):
    _fields_ = [
        ("result_at_s", ctypes.c_double),
        ("request_id", ctypes.c_char * ID_CAP),
        ("evidence_id", ctypes.c_char * ID_CAP),
        ("persisted", ctypes.c_int32),
        ("reason", ctypes.c_char * REASON_CAP),
        ("analysis_verdict", ctypes.c_char * REASON_CAP),
    ]


def _library_names() -> tuple[str, ...]:
    if platform.system() == "Windows":
        return ("lingtu_inspection_evidence_bridge.dll",)
    if platform.system() == "Darwin":
        return ("liblingtu_inspection_evidence_bridge.dylib",)
    return ("liblingtu_inspection_evidence_bridge.so",)


def _library_candidates() -> list[Path]:
    explicit = os.environ.get("LINGTU_INSPECTION_EVIDENCE_BRIDGE_LIBRARY", "").strip()
    root = Path(__file__).resolve().parents[3]
    candidates = [Path(explicit)] if explicit else []
    for name in _library_names():
        candidates.extend(
            [
                root / "build" / "nav_endpoint" / name,
                root / "build" / "nav_endpoint" / "Release" / name,
                Path("/opt/lingtu/current/build/nav_endpoint") / name,
                Path("/opt/lingtu/current/lib") / name,
            ]
        )
    return candidates


def _load_library() -> ctypes.CDLL:
    for candidate in _library_candidates():
        if candidate.is_file():
            return ctypes.CDLL(str(candidate))
    raise InspectionEvidenceBridgeError(
        "native inspection evidence bridge library not found; "
        "set LINGTU_INSPECTION_EVIDENCE_BRIDGE_LIBRARY"
    )


def _encode_fixed(value: str, capacity: int, field: str) -> bytes:
    raw = str(value or "").encode("utf-8")
    if len(raw) >= capacity:
        raise InspectionEvidenceBridgeError(f"{field} exceeds fixed ABI buffer")
    return raw


def _decode_fixed(raw: bytes | bytearray) -> str:
    return bytes(raw).split(b"\0", 1)[0].decode("utf-8", errors="replace")


class NativeInspectionEvidenceBridge:
    """Non-blocking DDS bridge for inspection evidence request/result messages."""

    def __init__(
        self,
        *,
        domain_id: int = 0,
        library: Any | None = None,
    ) -> None:
        self._lib = library or _load_library()
        self._configure_abi()
        self._handle = self._lib.lingtu_inspection_evidence_bridge_create(int(domain_id))
        if not self._handle:
            raise InspectionEvidenceBridgeError(self._error())

    def _configure_abi(self) -> None:
        lib = self._lib
        lib.lingtu_inspection_evidence_bridge_create.argtypes = [ctypes.c_int32]
        lib.lingtu_inspection_evidence_bridge_create.restype = ctypes.c_void_p
        lib.lingtu_inspection_evidence_bridge_destroy.argtypes = [ctypes.c_void_p]
        lib.lingtu_inspection_evidence_bridge_destroy.restype = None
        lib.lingtu_inspection_evidence_bridge_take_request.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Request),
        ]
        lib.lingtu_inspection_evidence_bridge_take_request.restype = ctypes.c_int32
        lib.lingtu_inspection_evidence_bridge_write_result.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_Result),
        ]
        lib.lingtu_inspection_evidence_bridge_write_result.restype = ctypes.c_int32
        lib.lingtu_inspection_evidence_bridge_last_error.argtypes = [ctypes.c_void_p]
        lib.lingtu_inspection_evidence_bridge_last_error.restype = ctypes.c_char_p

    def close(self) -> None:
        if self._handle:
            self._lib.lingtu_inspection_evidence_bridge_destroy(self._handle)
            self._handle = None

    def __enter__(self) -> NativeInspectionEvidenceBridge:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def _error(self) -> str:
        raw = self._lib.lingtu_inspection_evidence_bridge_last_error(self._handle)
        return bytes(raw or b"native inspection evidence bridge operation failed").decode(
            "utf-8",
            errors="replace",
        )

    def take_request(self) -> dict[str, Any] | None:
        """Return one request, or None when no DDS sample is currently available."""

        request = _Request()
        status = self._lib.lingtu_inspection_evidence_bridge_take_request(
            self._handle,
            ctypes.byref(request),
        )
        if status < 0:
            raise InspectionEvidenceBridgeError(self._error())
        if status == 0:
            return None
        return {
            "run_id": _decode_fixed(request.run_id),
            "route_id": _decode_fixed(request.route_id),
            "route_revision": int(request.route_revision),
            "map_id": _decode_fixed(request.map_id),
            "map_version": int(request.map_version),
            "point_id": _decode_fixed(request.point_id),
            "point_index": int(request.point_index),
            "request_id": _decode_fixed(request.request_id),
            "action": _decode_fixed(request.action),
            "requested_at_s": float(request.requested_at_s),
            "deadline_s": float(request.deadline_s),
        }

    def write_result(
        self,
        *,
        request_id: str,
        evidence_id: str,
        persisted: bool,
        reason: str = "",
        analysis_verdict: str = "",
        result_at_s: float = 0.0,
    ) -> None:
        result = _Result()
        result.result_at_s = float(result_at_s)
        result.request_id = _encode_fixed(request_id, ID_CAP, "request_id")
        result.evidence_id = _encode_fixed(evidence_id, ID_CAP, "evidence_id")
        result.persisted = 1 if persisted else 0
        result.reason = _encode_fixed(reason, REASON_CAP, "reason")
        result.analysis_verdict = _encode_fixed(
            analysis_verdict,
            REASON_CAP,
            "analysis_verdict",
        )
        status = self._lib.lingtu_inspection_evidence_bridge_write_result(
            self._handle,
            ctypes.byref(result),
        )
        if status != 0:
            raise InspectionEvidenceBridgeError(self._error())
