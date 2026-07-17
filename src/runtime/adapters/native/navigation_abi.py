"""Private ctypes session for the native navigation command library.

The process owns one C++ DDS session.  Python command interfaces bind only the
capability they use, so adding an inspection symbol cannot break navigation
goal, teleop, or emergency-stop delivery.
"""

from __future__ import annotations

import atexit
import ctypes
import os
import threading
from pathlib import Path
from typing import Any

NATIVE_COMMAND_ABI_VERSION = 1
NATIVE_COMMAND_CAP_NAVIGATION = 1 << 0
NATIVE_COMMAND_CAP_INSPECTION = 1 << 1


class NativeCommandClientError(RuntimeError):
    """Raised when the native command session cannot be loaded or invoked."""


class NativeCommandSession:
    """Thread-safe owner of one process-local C++ CycloneDDS client handle."""

    def __init__(
        self,
        library_path: str | os.PathLike[str],
        *,
        domain_id: int = 0,
        timeout_ms: int = 1000,
        goal_timeout_ms: int | None = None,
        cancel_timeout_ms: int | None = None,
        teleop_timeout_ms: int | None = None,
        library: Any | None = None,
    ) -> None:
        path = Path(library_path)
        if library is None and not path.is_file():
            raise NativeCommandClientError(f"native navigation client library missing: {path}")
        self.library_path = path
        self.library = library or ctypes.CDLL(str(path))
        self.timeout_ms = max(1, int(timeout_ms))
        self.goal_timeout_ms = max(
            1,
            int(goal_timeout_ms if goal_timeout_ms is not None else self.timeout_ms),
        )
        self.cancel_timeout_ms = max(
            1,
            int(cancel_timeout_ms if cancel_timeout_ms is not None else self.timeout_ms),
        )
        self.teleop_timeout_ms = max(
            1,
            int(teleop_timeout_ms if teleop_timeout_ms is not None else self.timeout_ms),
        )
        self.lock = threading.RLock()
        self._state_changed = threading.Condition(self.lock)
        self._active_calls = 0
        self._navigation_configured = False
        self._inspection_configured = False
        self._configure_core_abi()
        self._validate_abi()
        self.handle = self.library.lingtu_nav_client_create(int(domain_id))
        if not self.handle:
            raise NativeCommandClientError(self.last_error(None))

    def _configure_core_abi(self) -> None:
        try:
            self.library.lingtu_nav_client_abi_version.argtypes = []
            self.library.lingtu_nav_client_abi_version.restype = ctypes.c_uint32
            self.library.lingtu_nav_client_capabilities.argtypes = []
            self.library.lingtu_nav_client_capabilities.restype = ctypes.c_uint64
            self.library.lingtu_nav_client_create.argtypes = [ctypes.c_int]
            self.library.lingtu_nav_client_create.restype = ctypes.c_void_p
            self.library.lingtu_nav_client_destroy.argtypes = [ctypes.c_void_p]
            self.library.lingtu_nav_client_destroy.restype = None
            self.library.lingtu_nav_client_last_error.argtypes = [ctypes.c_void_p]
            self.library.lingtu_nav_client_last_error.restype = ctypes.c_char_p
        except AttributeError as exc:
            raise NativeCommandClientError(
                "native navigation client ABI metadata is missing; rebuild liblingtu_nav_client.so"
            ) from exc

    def _validate_abi(self) -> None:
        version = int(self.library.lingtu_nav_client_abi_version())
        if version != NATIVE_COMMAND_ABI_VERSION:
            raise NativeCommandClientError(
                "native navigation client ABI version mismatch: "
                f"expected {NATIVE_COMMAND_ABI_VERSION}, got {version}"
            )
        self.capabilities = int(self.library.lingtu_nav_client_capabilities())

    def ensure_navigation_abi(self) -> None:
        """Validate and configure navigation-command symbols lazily."""

        with self.lock:
            if self._navigation_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_NAVIGATION,
                "navigation commands",
            )
            lib = self.library
            try:
                lib.lingtu_nav_client_send_goal.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_send_goal.restype = ctypes.c_int
                lib.lingtu_nav_client_send_goal_with_id.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_send_goal_with_id.restype = ctypes.c_int
                lib.lingtu_nav_client_cancel.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_cancel.restype = ctypes.c_int
                lib.lingtu_nav_client_cancel_with_id.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_cancel_with_id.restype = ctypes.c_int
                lib.lingtu_nav_client_send_teleop.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_send_teleop.restype = ctypes.c_int
                lib.lingtu_nav_client_send_teleop_with_id.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_send_teleop_with_id.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_stop",
                    "lingtu_nav_client_estop",
                    "lingtu_nav_client_clear_estop",
                    "lingtu_nav_client_resume_autonomy",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
                    function.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_stop_with_id",
                    "lingtu_nav_client_estop_with_id",
                    "lingtu_nav_client_clear_estop_with_id",
                    "lingtu_nav_client_resume_autonomy_with_id",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_int,
                    ]
                    function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native navigation command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._navigation_configured = True

    def ensure_inspection_abi(self) -> None:
        """Validate and configure inspection-command symbols lazily."""

        with self.lock:
            if self._inspection_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_INSPECTION,
                "inspection commands",
            )
            lib = self.library
            try:
                lib.lingtu_nav_client_start_inspection.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_start_inspection.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_pause_inspection",
                    "lingtu_nav_client_resume_inspection",
                    "lingtu_nav_client_cancel_inspection",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_int,
                    ]
                    function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native inspection command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._inspection_configured = True

    def _require_capability(self, capability: int, label: str) -> None:
        if self.capabilities & capability:
            return
        raise NativeCommandClientError(
            f"native navigation client does not provide {label} capability"
        )

    def call(self, function_name: str, *args: object) -> None:
        """Invoke one C ABI command without serializing other active calls."""

        with self.lock:
            self.require_open()
            handle = self.handle
            function = getattr(self.library, function_name)
            self._active_calls += 1
        try:
            result = function(handle, *args)
            if int(result) != 0:
                raise NativeCommandClientError(self.last_error(handle))
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def last_error(self, handle: int | None = None) -> str:
        """Read the calling thread's latest native command error."""

        resolved = self.handle if handle is None and hasattr(self, "handle") else handle
        raw = self.library.lingtu_nav_client_last_error(resolved)
        if not raw:
            return "native navigation command failed"
        return bytes(raw).decode("utf-8", errors="replace")

    def require_open(self) -> None:
        """Raise when the shared native handle has already been closed."""

        if not self.handle:
            raise NativeCommandClientError("native navigation client is closed")

    def close(self) -> None:
        """Stop new calls, wait for active calls, and destroy the C++ handle."""

        with self.lock:
            if not self.handle:
                return
            handle = self.handle
            self.handle = None
            while self._active_calls:
                self._state_changed.wait()
        self.library.lingtu_nav_client_destroy(handle)


_SESSIONS: dict[tuple[str, int, int, int, int, int], NativeCommandSession] = {}
_SESSIONS_LOCK = threading.Lock()


def get_native_command_session(*, required: bool = False) -> NativeCommandSession | None:
    """Return the process-wide C++ command session configured by the field profile."""

    raw_path = os.environ.get("LINGTU_NAV_CLIENT_LIB", "").strip()
    if not raw_path:
        if required:
            raise NativeCommandClientError("LINGTU_NAV_CLIENT_LIB is not configured")
        return None
    try:
        domain_id = int(os.environ.get("LINGTU_DDS_DOMAIN_ID", "0") or "0")
        timeout_ms = int(os.environ.get("LINGTU_NAV_CLIENT_TIMEOUT_MS", "1000") or "1000")
        goal_timeout_ms = int(os.environ.get("LINGTU_NAV_GOAL_TIMEOUT_MS", "10000") or "10000")
        cancel_timeout_ms = int(os.environ.get("LINGTU_NAV_CANCEL_TIMEOUT_MS", "2000") or "2000")
        teleop_timeout_ms = int(os.environ.get("LINGTU_NAV_TELEOP_TIMEOUT_MS", "1000") or "1000")
    except ValueError as exc:
        raise NativeCommandClientError("invalid native navigation client configuration") from exc
    key = (
        str(Path(raw_path).resolve()),
        domain_id,
        max(1, timeout_ms),
        max(1, goal_timeout_ms),
        max(1, cancel_timeout_ms),
        max(1, teleop_timeout_ms),
    )
    with _SESSIONS_LOCK:
        session = _SESSIONS.get(key)
        if session is None:
            session = NativeCommandSession(
                raw_path,
                domain_id=domain_id,
                timeout_ms=timeout_ms,
                goal_timeout_ms=goal_timeout_ms,
                cancel_timeout_ms=cancel_timeout_ms,
                teleop_timeout_ms=teleop_timeout_ms,
            )
            _SESSIONS[key] = session
        return session


def _close_sessions() -> None:
    with _SESSIONS_LOCK:
        sessions = list(_SESSIONS.values())
        _SESSIONS.clear()
    for session in sessions:
        session.close()


atexit.register(_close_sessions)
