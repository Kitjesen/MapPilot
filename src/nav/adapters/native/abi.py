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

from runtime.msgs import NavigationCommandReceipt

# Field Products package this library with the Host as one immutable release.
# Capability bits keep feature discovery explicit inside the exact ABI version.

NATIVE_COMMAND_ABI_VERSION = 5
NATIVE_COMMAND_CAP_NAVIGATION = 1 << 0
NATIVE_COMMAND_CAP_INSPECTION = 1 << 1
NATIVE_COMMAND_CAP_EXPLORATION = 1 << 2
NATIVE_COMMAND_CAP_DIRECTED_EXPLORATION = 1 << 3
NATIVE_COMMAND_CAP_OPERATOR_MOTION = 1 << 4
NATIVE_COMMAND_CAP_HOST_STATE = 1 << 5
NATIVE_COMMAND_CAP_GOAL_STATUS = 1 << 6
NATIVE_COMMAND_CAP_PATH_TELEMETRY = 1 << 7
NATIVE_COMMAND_CAP_MAP_SCENE = 1 << 8
NATIVE_COMMAND_CAP_OPERATOR_MOTION_RECEIPT = 1 << 9
NATIVE_COMMAND_CAP_NAVIGATION_COMMAND_RECEIPT = 1 << 10
NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS = 1 << 11
NATIVE_COMMAND_CAP_NAVIGATION_STATE_V1 = 1 << 12
NATIVE_COMMAND_CAP_NAVIGATION_GOAL_STATUS_V1 = 1 << 13

NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION = 1
NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION = 1
NATIVE_NAVIGATION_STATE_ABI_VERSION = 1


class _NativeNavigationState(ctypes.Structure):
    """Frozen ABI v4 layout retained for legacy binary compatibility checks."""

    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("boot_id", ctypes.c_char * 128),
        ("sequence", ctypes.c_ulonglong),
        ("control_mode", ctypes.c_int32),
        ("lifecycle_state", ctypes.c_int32),
        ("active_request_id", ctypes.c_char * 128),
        ("goal_epoch", ctypes.c_ulonglong),
        ("map_id", ctypes.c_char * 128),
        ("map_version", ctypes.c_int64),
        ("map_hash", ctypes.c_char * 128),
        ("planning_state", ctypes.c_int32),
        ("execution_state", ctypes.c_int32),
        ("recovery_state", ctypes.c_int32),
        ("progress", ctypes.c_float),
        ("authority", ctypes.c_char * 32),
        ("hold_reason", ctypes.c_char * 128),
        ("failure_code", ctypes.c_char * 128),
    ]


class _NativeNavigationStateV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("boot_id", ctypes.c_char * 128),
        ("sequence", ctypes.c_ulonglong),
        ("control_mode", ctypes.c_int32),
        ("lifecycle_state", ctypes.c_int32),
        ("active_task_id", ctypes.c_char * 128),
        ("active_request_id", ctypes.c_char * 128),
        ("goal_epoch", ctypes.c_ulonglong),
        ("map_id", ctypes.c_char * 128),
        ("map_version", ctypes.c_int64),
        ("map_hash", ctypes.c_char * 128),
        ("planning_state", ctypes.c_int32),
        ("execution_state", ctypes.c_int32),
        ("recovery_state", ctypes.c_int32),
        ("progress", ctypes.c_float),
        ("authority", ctypes.c_char * 32),
        ("hold_reason", ctypes.c_char * 128),
        ("failure_code", ctypes.c_char * 128),
    ]


class _NativeNavigationGoalStatus(ctypes.Structure):
    """Frozen ABI v4 layout retained for legacy binary compatibility checks."""

    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("boot_id", ctypes.c_char * 128),
        ("sequence", ctypes.c_ulonglong),
        ("request_id", ctypes.c_char * 128),
        ("state", ctypes.c_int32),
        ("goal_epoch", ctypes.c_ulonglong),
        ("reason", ctypes.c_char * 256),
    ]


class _NativeNavigationCommandReceiptV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("task_id", ctypes.c_char * 128),
        ("request_id", ctypes.c_char * 128),
        ("accepted", ctypes.c_int32),
        ("kind", ctypes.c_int32),
        ("reason", ctypes.c_char * 256),
        ("endpoint_timestamp_s", ctypes.c_double),
    ]


class _NativeNavigationGoalStatusV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("boot_id", ctypes.c_char * 128),
        ("sequence", ctypes.c_ulonglong),
        ("task_id", ctypes.c_char * 128),
        ("request_id", ctypes.c_char * 128),
        ("state", ctypes.c_int32),
        ("goal_epoch", ctypes.c_ulonglong),
        ("reason", ctypes.c_char * 256),
    ]


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
        self._exploration_configured = False
        self._inspection_configured = False
        self._host_state_configured = False
        self._goal_status_configured = False
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
                f"native navigation client ABI version mismatch: expected {NATIVE_COMMAND_ABI_VERSION}, got {version}"
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
            self._require_capability(
                NATIVE_COMMAND_CAP_NAVIGATION_COMMAND_RECEIPT,
                "navigation command receipt",
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
                lib.lingtu_nav_client_start_task_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeNavigationCommandReceiptV1),
                ]
                lib.lingtu_nav_client_start_task_with_receipt_v1.restype = ctypes.c_int
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
                lib.lingtu_nav_client_cancel_task_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeNavigationCommandReceiptV1),
                ]
                lib.lingtu_nav_client_cancel_task_with_receipt_v1.restype = ctypes.c_int
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

    def ensure_exploration_abi(self) -> None:
        """Validate and configure exploration-session command symbols lazily."""

        with self.lock:
            if self._exploration_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_EXPLORATION,
                "exploration commands",
            )
            lib = self.library
            try:
                lib.lingtu_nav_client_start_exploration.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_start_exploration.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_pause_exploration",
                    "lingtu_nav_client_resume_exploration",
                    "lingtu_nav_client_stop_exploration",
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
                    "native exploration command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._exploration_configured = True

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

    def ensure_host_state_abi(self) -> None:
        """Bind the versioned navigation state reader used by the Host."""

        with self.lock:
            if self._host_state_configured:
                return
            self._require_capability(NATIVE_COMMAND_CAP_HOST_STATE, "Host state")
            self._require_capability(
                NATIVE_COMMAND_CAP_NAVIGATION_STATE_V1,
                "navigation state v1",
            )
            try:
                function = self.library.lingtu_nav_client_read_navigation_state_v1
                function.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeNavigationStateV1),
                ]
                function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native Host state v1 ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._host_state_configured = True

    def ensure_goal_status_abi(self) -> None:
        """Bind versioned lifecycle readers that preserve logical task identity."""

        with self.lock:
            if self._goal_status_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_GOAL_STATUS,
                "navigation goal status",
            )
            self._require_capability(
                NATIVE_COMMAND_CAP_NAVIGATION_GOAL_STATUS_V1,
                "navigation goal status v1",
            )
            try:
                take = self.library.lingtu_nav_client_take_navigation_goal_status_v1
                take.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeNavigationGoalStatusV1),
                ]
                take.restype = ctypes.c_int
                get = self.library.lingtu_nav_client_get_navigation_goal_status_v1
                get.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.POINTER(_NativeNavigationGoalStatusV1),
                ]
                get.restype = ctypes.c_int
                if self.capabilities & NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS:
                    get_task = self.library.lingtu_nav_client_get_navigation_task_status_v1
                    get_task.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.POINTER(_NativeNavigationGoalStatusV1),
                    ]
                    get_task.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native navigation goal status v1 ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._goal_status_configured = True

    def _require_capability(self, capability: int, label: str) -> None:
        if self.capabilities & capability:
            return
        raise NativeCommandClientError(f"native navigation client does not provide {label} capability")

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

    def start_navigation_task(
        self,
        task_id: str,
        request_id: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
    ) -> NavigationCommandReceipt:
        """Submit one logical task and return its correlated business ACK."""

        task, request = self._validate_task_identity(task_id, request_id)
        self.ensure_navigation_abi()
        return self._write_navigation_command_receipt(
            self.library.lingtu_nav_client_start_task_with_receipt_v1,
            task.encode("utf-8"),
            request.encode("utf-8"),
            float(x),
            float(y),
            float(z),
            float(yaw),
            self.goal_timeout_ms,
        )

    def cancel_navigation_task(
        self,
        task_id: str,
        request_id: str,
        reason: str,
    ) -> NavigationCommandReceipt:
        """Cancel one logical task and return its correlated business ACK."""

        task, request = self._validate_task_identity(task_id, request_id)
        self.ensure_navigation_abi()
        return self._write_navigation_command_receipt(
            self.library.lingtu_nav_client_cancel_task_with_receipt_v1,
            task.encode("utf-8"),
            request.encode("utf-8"),
            str(reason or "cancel").encode("utf-8"),
            self.cancel_timeout_ms,
        )

    def read_navigation_state(self) -> dict[str, object] | None:
        """Return the latest task-aware authoritative state snapshot."""

        self.ensure_host_state_abi()
        state = self._read_versioned_snapshot(
            self.library.lingtu_nav_client_read_navigation_state_v1,
            _NativeNavigationStateV1,
            NATIVE_NAVIGATION_STATE_ABI_VERSION,
            "navigation state v1",
        )
        if state is None:
            return None
        return {
            "timestamp_s": float(state.timestamp_s),
            "frame_id": self._decode_fixed_text(state.frame_id),
            "boot_id": self._decode_fixed_text(state.boot_id),
            "sequence": int(state.sequence),
            "control_mode": int(state.control_mode),
            "lifecycle_state": int(state.lifecycle_state),
            "active_task_id": self._decode_fixed_text(state.active_task_id),
            "active_request_id": self._decode_fixed_text(state.active_request_id),
            "goal_epoch": int(state.goal_epoch),
            "map_id": self._decode_fixed_text(state.map_id),
            "map_version": int(state.map_version),
            "map_hash": self._decode_fixed_text(state.map_hash),
            "planning_state": int(state.planning_state),
            "execution_state": int(state.execution_state),
            "recovery_state": int(state.recovery_state),
            "progress": float(state.progress),
            "authority": self._decode_fixed_text(state.authority),
            "hold_reason": self._decode_fixed_text(state.hold_reason),
            "failure_code": self._decode_fixed_text(state.failure_code),
        }

    def take_navigation_goal_status(self) -> dict[str, object] | None:
        """Pop one task-aware lifecycle event from the bounded native queue."""

        self.ensure_goal_status_abi()
        return self._read_navigation_goal_status(
            self.library.lingtu_nav_client_take_navigation_goal_status_v1,
        )

    def get_navigation_goal_status(self, request_id: str) -> dict[str, object] | None:
        """Read retained task-aware lifecycle state for one request attempt."""

        request = str(request_id or "").strip()
        if not request:
            raise ValueError("request_id is required")
        self.ensure_goal_status_abi()
        return self._read_navigation_goal_status(
            self.library.lingtu_nav_client_get_navigation_goal_status_v1,
            request.encode("utf-8"),
        )

    def get_navigation_task_status(self, task_id: str) -> dict[str, object] | None:
        """Read retained lifecycle state for one stable logical task."""

        task = str(task_id or "").strip()
        if not task:
            raise ValueError("task_id is required")
        self.ensure_goal_status_abi()
        self._require_capability(
            NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS,
            "navigation task status",
        )
        return self._read_navigation_goal_status(
            self.library.lingtu_nav_client_get_navigation_task_status_v1,
            task.encode("utf-8"),
        )

    @staticmethod
    def _validate_task_identity(task_id: str, request_id: str) -> tuple[str, str]:
        task = str(task_id or "").strip()
        request = str(request_id or "").strip()
        if not task:
            raise ValueError("task_id is required")
        if not request:
            raise ValueError("request_id is required")
        if task == request:
            raise ValueError("task_id and request_id must be distinct")
        return task, request

    def _read_navigation_goal_status(
        self,
        function: Any,
        *arguments: object,
    ) -> dict[str, object] | None:
        status = self._read_versioned_snapshot(
            function,
            _NativeNavigationGoalStatusV1,
            NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION,
            "navigation goal status v1",
            *arguments,
        )
        if status is None:
            return None
        return {
            "timestamp_s": float(status.timestamp_s),
            "frame_id": self._decode_fixed_text(status.frame_id),
            "boot_id": self._decode_fixed_text(status.boot_id),
            "sequence": int(status.sequence),
            "task_id": self._decode_fixed_text(status.task_id),
            "request_id": self._decode_fixed_text(status.request_id),
            "state": int(status.state),
            "goal_epoch": int(status.goal_epoch),
            "reason": self._decode_fixed_text(status.reason),
        }

    def _read_versioned_snapshot(
        self,
        function: Any,
        structure_type: type[ctypes.Structure],
        abi_version: int,
        label: str,
        *arguments: object,
    ) -> ctypes.Structure | None:
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        snapshot = structure_type()
        snapshot.abi_version = int(abi_version)
        snapshot.struct_size = ctypes.sizeof(structure_type)
        try:
            result = int(function(handle, *arguments, ctypes.byref(snapshot)))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            if result != 1:
                raise NativeCommandClientError(f"native {label} returned unexpected result {result}")
            self._validate_versioned_metadata(
                snapshot,
                expected_abi_version=abi_version,
                structure_type=structure_type,
                label=label,
            )
            return snapshot
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def _write_navigation_command_receipt(
        self,
        function: Any,
        *arguments: object,
    ) -> NavigationCommandReceipt:
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        receipt = _NativeNavigationCommandReceiptV1()
        receipt.abi_version = NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION
        receipt.struct_size = ctypes.sizeof(_NativeNavigationCommandReceiptV1)
        try:
            result = int(function(handle, *arguments, ctypes.byref(receipt)))
            if result != 0:
                raise NativeCommandClientError(self.last_error(handle))
            self._validate_versioned_metadata(
                receipt,
                expected_abi_version=NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION,
                structure_type=_NativeNavigationCommandReceiptV1,
                label="navigation command receipt v1",
            )
            if int(receipt.accepted) not in (0, 1):
                raise NativeCommandClientError("native navigation command receipt has an invalid accepted value")
            return NavigationCommandReceipt(
                accepted=bool(receipt.accepted),
                kind=int(receipt.kind),
                task_id=self._decode_fixed_text(receipt.task_id),
                request_id=self._decode_fixed_text(receipt.request_id),
                endpoint_timestamp_s=float(receipt.endpoint_timestamp_s),
                reason=self._decode_fixed_text(receipt.reason),
            )
        except ValueError as exc:
            raise NativeCommandClientError(f"native navigation command returned an invalid receipt: {exc}") from exc
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    @staticmethod
    def _validate_versioned_metadata(
        value: ctypes.Structure,
        *,
        expected_abi_version: int,
        structure_type: type[ctypes.Structure],
        label: str,
    ) -> None:
        if int(value.abi_version) != int(expected_abi_version):
            raise NativeCommandClientError(
                f"native {label} ABI version mismatch: expected {expected_abi_version}, got {int(value.abi_version)}"
            )
        expected_size = ctypes.sizeof(structure_type)
        if int(value.struct_size) != expected_size:
            raise NativeCommandClientError(
                f"native {label} struct size mismatch: expected {expected_size}, got {int(value.struct_size)}"
            )

    @staticmethod
    def _decode_fixed_text(value: bytes | bytearray | memoryview) -> str:
        return bytes(value).split(b"\0", 1)[0].decode("utf-8", errors="replace")

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
