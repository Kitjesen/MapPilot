"""Private ctypes session for the native navigation command library.

The process owns one C++ DDS session.  Python command interfaces bind only the
capability they use, so adding an inspection symbol cannot break navigation
goal, teleop, or emergency-stop delivery.
"""

from __future__ import annotations

import atexit
import ctypes
import math
import os
import threading
from pathlib import Path
from typing import Any

# Field Products package the Host and native client in one atomically switched
# release. Exact equality intentionally fails closed.
# Append-only extensions inside a release are advertised by capability bits.
NATIVE_COMMAND_ABI_VERSION = 8
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
NATIVE_COMMAND_CAP_INSPECTION_TASK_EVENTS = 1 << 12
NATIVE_COMMAND_CAP_EXPLORATION_RUN_EVENTS = 1 << 13
NATIVE_COMMAND_CAP_TRAVERSABILITY_GRID = 1 << 14
NATIVE_COMMAND_CAP_PLAN_PREVIEW = 1 << 15

NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION = 1
NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION = 1
NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION = 1
NATIVE_INSPECTION_TASK_EVENT_ABI_VERSION = 1
NATIVE_EXPLORATION_COMMAND_RECEIPT_ABI_VERSION = 1
NATIVE_EXPLORATION_RUN_EVENT_ABI_VERSION = 2
NATIVE_PLAN_RESULT_ABI_VERSION = 1

NATIVE_MAP_SCENE_ABI_VERSION = 1
NATIVE_MAP_SCENE_MAX_POINTS_PER_LAYER = 300_000
NATIVE_MAP_SCENE_MAX_TOTAL_POINTS = 800_000
NATIVE_MAP_SCENE_MAX_GRID_CELLS_PER_LAYER = 1_000_000
NATIVE_MAP_SCENE_MAX_TOTAL_GRID_CELLS = 4_000_000
NATIVE_MAP_SCENE_MAX_PAYLOAD_BYTES = 32 * 1024 * 1024
NATIVE_TRAVERSABILITY_GRID_ABI_VERSION = 1
NATIVE_TRAVERSABILITY_MAX_CELLS = 1_000_000


class _NativeNavigationState(ctypes.Structure):
    _fields_ = [
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
        ("map_content_epoch", ctypes.c_int64),
        ("planning_state", ctypes.c_int32),
        ("execution_state", ctypes.c_int32),
        ("recovery_state", ctypes.c_int32),
        ("progress", ctypes.c_float),
        ("authority", ctypes.c_char * 32),
        ("hold_reason", ctypes.c_char * 128),
        ("failure_code", ctypes.c_char * 128),
    ]


class _NativeNavigationGoalStatus(ctypes.Structure):
    _fields_ = [
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


class _NativeInspectionTaskEventV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("boot_id", ctypes.c_char * 128),
        ("event_sequence", ctypes.c_ulonglong),
        ("kind", ctypes.c_int32),
        ("task_id", ctypes.c_char * 128),
        ("request_id", ctypes.c_char * 128),
        ("command_request_id", ctypes.c_char * 128),
        ("state", ctypes.c_int32),
        ("map_id", ctypes.c_char * 128),
        ("map_content_epoch", ctypes.c_int64),
        ("route_id", ctypes.c_char * 128),
        ("route_revision", ctypes.c_ulonglong),
        ("point_index", ctypes.c_uint32),
        ("point_count", ctypes.c_uint32),
        ("loop_index", ctypes.c_uint32),
        ("retry_count", ctypes.c_uint32),
        ("point_id", ctypes.c_char * 128),
        ("action", ctypes.c_char * 128),
        ("action_request_id", ctypes.c_char * 128),
        ("evidence_id", ctypes.c_char * 128),
        ("reason", ctypes.c_char * 256),
    ]


class _NativeExplorationCommandReceiptV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("accepted", ctypes.c_int32),
        ("request_id", ctypes.c_char * 128),
        ("exploration_run_id", ctypes.c_char * 128),
        ("reason", ctypes.c_char * 256),
        ("duplicate", ctypes.c_int32),
    ]


class _NativeExplorationRunEventV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("boot_id", ctypes.c_char * 128),
        ("event_sequence", ctypes.c_ulonglong),
        ("kind", ctypes.c_int32),
        ("exploration_run_id", ctypes.c_char * 128),
        ("start_request_id", ctypes.c_char * 128),
        ("command_request_id", ctypes.c_char * 128),
        ("product_session_id", ctypes.c_char * 128),
        ("state", ctypes.c_int32),
        ("route", ctypes.c_char * 32),
        ("map_id", ctypes.c_char * 128),
        ("map_content_epoch", ctypes.c_int64),
        ("reason", ctypes.c_char * 256),
        ("motion_stop_confirmed", ctypes.c_int32),
        ("motion_stop_reason", ctypes.c_char * 256),
    ]


class _NativePathPoint(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
    ]


class _NativePlanResultV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("request_id", ctypes.c_char * 128),
        ("feasible", ctypes.c_int32),
        ("start_valid", ctypes.c_int32),
        ("reason", ctypes.c_char * 256),
        ("elapsed_ms", ctypes.c_double),
        ("planner", ctypes.c_char * 64),
        ("start", _NativePathPoint),
        ("goal", _NativePathPoint),
        ("point_count", ctypes.c_ulonglong),
    ]


class _NativePathHeader(ctypes.Structure):
    _fields_ = [
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("receive_sequence", ctypes.c_ulonglong),
        ("point_count", ctypes.c_ulonglong),
    ]


class _NativeTraversabilityGridHeaderV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("receive_sequence", ctypes.c_ulonglong),
        ("reset_epoch", ctypes.c_ulonglong),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("resolution", ctypes.c_float),
        ("origin_x", ctypes.c_double),
        ("origin_y", ctypes.c_double),
        ("origin_z", ctypes.c_double),
        ("yaw", ctypes.c_double),
        ("cell_count", ctypes.c_ulonglong),
    ]

class _NativeMapScenePointV1(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("intensity", ctypes.c_float),
    ]

class _NativeOperatorMotionReceiptV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("accepted", ctypes.c_int32),
        ("action", ctypes.c_int32),
        ("request_id", ctypes.c_char * 128),
        ("source_id", ctypes.c_char * 128),
        ("source_epoch", ctypes.c_ulonglong),
        ("source_sequence", ctypes.c_ulonglong),
        ("accepted_sequence", ctypes.c_ulonglong),
        ("final_output_sequence", ctypes.c_ulonglong),
        ("endpoint_timestamp_s", ctypes.c_double),
        ("reason", ctypes.c_char * 256),
    ]



class _NativeMapSceneGridHeaderV1(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("resolution", ctypes.c_float),
        ("origin_x", ctypes.c_double),
        ("origin_y", ctypes.c_double),
        ("origin_z", ctypes.c_double),
        ("origin_qx", ctypes.c_double),
        ("origin_qy", ctypes.c_double),
        ("origin_qz", ctypes.c_double),
        ("origin_qw", ctypes.c_double),
        ("cell_count", ctypes.c_ulonglong),
    ]


class _NativeMapSceneHeaderV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("timestamp_s", ctypes.c_double),
        ("frame_id", ctypes.c_char * 32),
        ("producer_boot_id", ctypes.c_char * 128),
        ("receive_sequence", ctypes.c_ulonglong),
        ("reset_epoch", ctypes.c_ulonglong),
        ("observation_sequence", ctypes.c_ulonglong),
        ("generation", ctypes.c_ulonglong),
        ("live", ctypes.c_int32),
        ("sensor_x", ctypes.c_double),
        ("sensor_y", ctypes.c_double),
        ("sensor_z", ctypes.c_double),
        ("sensor_qx", ctypes.c_double),
        ("sensor_qy", ctypes.c_double),
        ("sensor_qz", ctypes.c_double),
        ("sensor_qw", ctypes.c_double),
        ("payload_bytes", ctypes.c_ulonglong),
        ("live_point_count", ctypes.c_ulonglong),
        ("voxel_point_count", ctypes.c_ulonglong),
        ("accumulated_point_count", ctypes.c_ulonglong),
        ("occupancy", _NativeMapSceneGridHeaderV1),
        ("elevation", _NativeMapSceneGridHeaderV1),
        ("esdf", _NativeMapSceneGridHeaderV1),
    ]


class _NativeMapSceneBuffersV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("live_points", ctypes.POINTER(_NativeMapScenePointV1)),
        ("live_point_capacity", ctypes.c_ulonglong),
        ("voxel_points", ctypes.POINTER(_NativeMapScenePointV1)),
        ("voxel_point_capacity", ctypes.c_ulonglong),
        ("accumulated_points", ctypes.POINTER(_NativeMapScenePointV1)),
        ("accumulated_point_capacity", ctypes.c_ulonglong),
        ("occupancy_cells", ctypes.POINTER(ctypes.c_float)),
        ("occupancy_cell_capacity", ctypes.c_ulonglong),
        ("elevation_cells", ctypes.POINTER(ctypes.c_float)),
        ("elevation_cell_capacity", ctypes.c_ulonglong),
        ("esdf_cells", ctypes.POINTER(ctypes.c_float)),
        ("esdf_cell_capacity", ctypes.c_ulonglong),
    ]


class _NativeMapSceneHealthV1(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_uint32),
        ("struct_size", ctypes.c_uint32),
        ("received_samples", ctypes.c_ulonglong),
        ("valid_samples", ctypes.c_ulonglong),
        ("stale_samples", ctypes.c_ulonglong),
        ("invalid_samples", ctypes.c_ulonglong),
        ("capacity_rejections", ctypes.c_ulonglong),
        ("replaced_samples", ctypes.c_ulonglong),
        ("consumer_buffer_retries", ctypes.c_ulonglong),
        ("last_receive_sequence", ctypes.c_ulonglong),
        ("last_generation", ctypes.c_ulonglong),
        ("last_sample_timestamp_s", ctypes.c_double),
        ("pending", ctypes.c_int32),
        ("last_error", ctypes.c_char * 256),
        ("state_received_samples", ctypes.c_ulonglong),
        ("state_valid_samples", ctypes.c_ulonglong),
        ("state_stale_samples", ctypes.c_ulonglong),
        ("state_invalid_samples", ctypes.c_ulonglong),
        ("state_timestamp_s", ctypes.c_double),
        ("state_producer_boot_id", ctypes.c_char * 128),
        ("state_received", ctypes.c_int32),
        ("state_running", ctypes.c_int32),
        ("state_live", ctypes.c_int32),
        ("state_required_publications_ready", ctypes.c_int32),
        ("state_current_generation_published", ctypes.c_int32),
        ("state_capacity_limited", ctypes.c_int32),
        ("state_reset_epoch", ctypes.c_ulonglong),
        ("state_observation_sequence", ctypes.c_ulonglong),
        ("state_generation", ctypes.c_ulonglong),
        ("state_scene_published_generation", ctypes.c_ulonglong),
        ("state_error", ctypes.c_char * 256),
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
        operator_motion_timeout_ms: int | None = None,
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
        self.operator_motion_timeout_ms = max(
            1,
            int(
                operator_motion_timeout_ms
                if operator_motion_timeout_ms is not None
                else self.teleop_timeout_ms
            ),
        )
        self.lock = threading.RLock()
        self._state_changed = threading.Condition(self.lock)
        self._active_calls = 0
        self._navigation_configured = False
        self._resume_autonomy_receipt_configured = False
        self._exploration_configured = False
        self._directed_exploration_configured = False
        self._inspection_task_configured = False
        self._inspection_task_event_configured = False
        self._exploration_run_event_configured = False
        self._operator_motion_configured = False
        self._host_state_configured = False
        self._goal_status_configured = False
        self._path_telemetry_configured = False
        self._plan_preview_configured = False
        self._map_scene_configured = False
        self._traversability_configured = False
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
                lib.lingtu_nav_client_cancel_task_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeNavigationCommandReceiptV1),
                ]
                lib.lingtu_nav_client_cancel_task_with_receipt_v1.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_pause_task_with_receipt_v1",
                    "lingtu_nav_client_resume_task_with_receipt_v1",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_int,
                        ctypes.POINTER(_NativeNavigationCommandReceiptV1),
                    ]
                    function.restype = ctypes.c_int
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

    def ensure_resume_autonomy_receipt_abi(self) -> None:
        """Configure the additive resume receipt symbol only when requested."""

        self.ensure_navigation_abi()
        with self.lock:
            if self._resume_autonomy_receipt_configured:
                return
            try:
                function = self.library.lingtu_nav_client_resume_autonomy_with_receipt_v1
                function.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeNavigationCommandReceiptV1),
                ]
                function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native resume autonomy receipt ABI is unavailable; rebuild liblingtu_nav_client.so"
                ) from exc
            self._resume_autonomy_receipt_configured = True

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
                lib.lingtu_nav_client_start_exploration_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeExplorationCommandReceiptV1),
                ]
                lib.lingtu_nav_client_start_exploration_with_receipt_v1.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_pause_exploration_with_receipt_v1",
                    "lingtu_nav_client_resume_exploration_with_receipt_v1",
                    "lingtu_nav_client_stop_exploration_with_receipt_v1",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_int,
                        ctypes.POINTER(_NativeExplorationCommandReceiptV1),
                    ]
                    function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native exploration command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._exploration_configured = True

    def ensure_directed_exploration_abi(self) -> None:
        """Validate and configure the typed directed-exploration symbols lazily."""

        with self.lock:
            if self._directed_exploration_configured:
                return
            self.ensure_exploration_abi()
            self._require_capability(
                NATIVE_COMMAND_CAP_DIRECTED_EXPLORATION,
                "directed exploration commands",
            )
            lib = self.library
            try:
                lib.lingtu_nav_client_set_directed_exploration_target_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeExplorationCommandReceiptV1),
                ]
                lib.lingtu_nav_client_set_directed_exploration_target_with_receipt_v1.restype = ctypes.c_int
                lib.lingtu_nav_client_clear_directed_exploration_target_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeExplorationCommandReceiptV1),
                ]
                lib.lingtu_nav_client_clear_directed_exploration_target_with_receipt_v1.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native directed exploration command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._directed_exploration_configured = True

    def ensure_inspection_task_abi(self) -> None:
        """Validate task-addressed inspection symbols without a v1 fallback."""

        with self.lock:
            if self._inspection_task_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_INSPECTION,
                "inspection task commands",
            )
            lib = self.library
            try:
                lib.lingtu_nav_client_start_inspection_task.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_start_inspection_task.restype = ctypes.c_int
                for name in (
                    "lingtu_nav_client_pause_inspection_task",
                    "lingtu_nav_client_resume_inspection_task",
                    "lingtu_nav_client_cancel_inspection_task",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_int,
                    ]
                    function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native inspection task command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._inspection_task_configured = True

    def ensure_inspection_task_event_abi(self) -> None:
        """Validate the native task-event reader used by the Product Host."""

        with self.lock:
            if self._inspection_task_event_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_INSPECTION_TASK_EVENTS,
                "inspection task events",
            )
            try:
                take = self.library.lingtu_nav_client_take_inspection_task_event_v1
                take.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeInspectionTaskEventV1),
                ]
                take.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native inspection task event ABI is incomplete; "
                    "rebuild liblingtu_nav_client.so"
                ) from exc
            self._inspection_task_event_configured = True

    def ensure_exploration_run_event_abi(self) -> None:
        """Validate the ordered native Explore run-event reader."""

        with self.lock:
            if self._exploration_run_event_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_EXPLORATION_RUN_EVENTS,
                "exploration run events",
            )
            try:
                take = self.library.lingtu_nav_client_take_exploration_run_event_v1
                take.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeExplorationRunEventV1),
                ]
                take.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native exploration run event ABI is incomplete; "
                    "rebuild liblingtu_nav_client.so"
                ) from exc
            self._exploration_run_event_configured = True

    def ensure_operator_motion_abi(self) -> None:
        """Validate and configure typed operator-motion command symbols lazily."""

        with self.lock:
            if self._operator_motion_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_OPERATOR_MOTION,
                "operator motion commands",
            )
            self._require_capability(
                NATIVE_COMMAND_CAP_OPERATOR_MOTION_RECEIPT,
                "operator motion receipt",
            )
            lib = self.library
            try:
                lib.lingtu_nav_client_operator_motion_claim.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_ulonglong,
                    ctypes.c_uint,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_operator_motion_claim.restype = ctypes.c_int
                lib.lingtu_nav_client_operator_motion_sample.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_ulonglong,
                    ctypes.c_int,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_uint,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_operator_motion_sample.restype = ctypes.c_int
                lib.lingtu_nav_client_operator_motion_sample_v2.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_ulonglong,
                    ctypes.c_int,
                    ctypes.c_int,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_uint,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_operator_motion_sample_v2.restype = ctypes.c_int
                lib.lingtu_nav_client_operator_motion_hold.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_ulonglong,
                    ctypes.c_char_p,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_operator_motion_hold.restype = ctypes.c_int
                lib.lingtu_nav_client_operator_motion_release.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_ulonglong,
                    ctypes.c_char_p,
                    ctypes.c_int,
                ]
                lib.lingtu_nav_client_operator_motion_release.restype = ctypes.c_int
                lib.lingtu_nav_client_operator_motion_claim_with_receipt_v1.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_char_p,
                    ctypes.c_ulonglong,
                    ctypes.c_ulonglong,
                    ctypes.c_uint,
                    ctypes.c_int,
                    ctypes.POINTER(_NativeOperatorMotionReceiptV1),
                ]
                lib.lingtu_nav_client_operator_motion_claim_with_receipt_v1.restype = (
                    ctypes.c_int
                )
                for name in (
                    "lingtu_nav_client_operator_motion_hold_with_receipt_v1",
                    "lingtu_nav_client_operator_motion_release_with_receipt_v1",
                ):
                    function = getattr(lib, name)
                    function.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.c_char_p,
                        ctypes.c_ulonglong,
                        ctypes.c_ulonglong,
                        ctypes.c_char_p,
                        ctypes.c_int,
                        ctypes.POINTER(_NativeOperatorMotionReceiptV1),
                    ]
                    function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native operator motion command ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._operator_motion_configured = True

    def ensure_host_state_abi(self) -> None:
        """Validate the single native state-reader boundary used by HostBus."""

        with self.lock:
            if self._host_state_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_HOST_STATE,
                "Host state",
            )
            try:
                function = self.library.lingtu_nav_client_read_navigation_state
                function.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeNavigationState),
                ]
                function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native Host state ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._host_state_configured = True

    def ensure_goal_status_abi(self) -> None:
        """Validate the native request-lifecycle reader used by HostBus."""

        with self.lock:
            if self._goal_status_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_GOAL_STATUS,
                "navigation goal status",
            )
            has_task_lookup = bool(
                self.capabilities & NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS
            )
            try:
                take = self.library.lingtu_nav_client_take_navigation_goal_status
                take.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeNavigationGoalStatus),
                ]
                take.restype = ctypes.c_int
                get = self.library.lingtu_nav_client_get_navigation_goal_status
                get.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.POINTER(_NativeNavigationGoalStatus),
                ]
                get.restype = ctypes.c_int
                if has_task_lookup:
                    get_task = self.library.lingtu_nav_client_get_navigation_task_status_v1
                    get_task.argtypes = [
                        ctypes.c_void_p,
                        ctypes.c_char_p,
                        ctypes.POINTER(_NativeNavigationGoalStatusV1),
                    ]
                    get_task.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native navigation goal status ABI is incomplete; "
                    "rebuild liblingtu_nav_client.so"
                ) from exc
            self._goal_status_configured = True

    def ensure_path_telemetry_abi(self) -> None:
        """Validate latest-only global/local path telemetry symbols lazily."""

        with self.lock:
            if self._path_telemetry_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_PATH_TELEMETRY,
                "navigation path telemetry",
            )
            argument_types = [
                ctypes.c_void_p,
                ctypes.POINTER(_NativePathHeader),
                ctypes.POINTER(_NativePathPoint),
                ctypes.c_ulonglong,
            ]
            try:
                for name in (
                    "lingtu_nav_client_take_global_path",
                    "lingtu_nav_client_take_local_path",
                ):
                    function = getattr(self.library, name)
                    function.argtypes = argument_types
                    function.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native navigation path telemetry ABI is incomplete; "
                    "rebuild liblingtu_nav_client.so"
                ) from exc
            self._path_telemetry_configured = True

    def ensure_plan_preview_abi(self) -> None:
        """Validate the read-only native plan preview symbol lazily."""

        with self.lock:
            if self._plan_preview_configured:
                return
            self._require_capability(NATIVE_COMMAND_CAP_PLAN_PREVIEW, "plan preview")
            try:
                preview = self.library.lingtu_nav_client_preview_plan_v1
                preview.argtypes = [
                    ctypes.c_void_p,
                    ctypes.c_char_p,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_double,
                    ctypes.c_int,
                    ctypes.POINTER(_NativePlanResultV1),
                    ctypes.POINTER(_NativePathPoint),
                    ctypes.c_ulonglong,
                ]
                preview.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native plan preview ABI is incomplete; rebuild liblingtu_nav_client.so"
                ) from exc
            self._plan_preview_configured = True

    def ensure_map_scene_abi(self) -> None:
        """Validate bounded latest-only MapScene telemetry symbols lazily."""

        with self.lock:
            if self._map_scene_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_MAP_SCENE,
                "map scene telemetry",
            )
            try:
                take = self.library.lingtu_nav_client_take_map_scene_v1
                take.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeMapSceneHeaderV1),
                    ctypes.POINTER(_NativeMapSceneBuffersV1),
                ]
                take.restype = ctypes.c_int
                health = self.library.lingtu_nav_client_read_map_scene_health_v1
                health.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeMapSceneHealthV1),
                ]
                health.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native map scene telemetry ABI is incomplete; "
                    "rebuild liblingtu_nav_client.so"
                ) from exc
            self._map_scene_configured = True

    def ensure_traversability_abi(self) -> None:
        """Validate bounded latest-only native control-risk grid telemetry."""

        with self.lock:
            if self._traversability_configured:
                return
            self._require_capability(
                NATIVE_COMMAND_CAP_TRAVERSABILITY_GRID,
                "native traversability grid telemetry",
            )
            try:
                take = self.library.lingtu_nav_client_take_traversability_grid_v1
                take.argtypes = [
                    ctypes.c_void_p,
                    ctypes.POINTER(_NativeTraversabilityGridHeaderV1),
                    ctypes.POINTER(ctypes.c_uint8),
                    ctypes.c_ulonglong,
                ]
                take.restype = ctypes.c_int
            except AttributeError as exc:
                raise NativeCommandClientError(
                    "native traversability grid ABI is incomplete; "
                    "rebuild liblingtu_nav_client.so"
                ) from exc
            self._traversability_configured = True

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

    def call_with_exploration_receipt(
        self,
        function_name: str,
        *args: object,
    ) -> dict[str, object]:
        """Invoke one exploration command and return its correlated ACK."""

        with self.lock:
            self.require_open()
            handle = self.handle
            function = getattr(self.library, function_name)
            self._active_calls += 1
        receipt = _NativeExplorationCommandReceiptV1()
        receipt.abi_version = NATIVE_EXPLORATION_COMMAND_RECEIPT_ABI_VERSION
        receipt.struct_size = ctypes.sizeof(_NativeExplorationCommandReceiptV1)
        try:
            result = int(function(handle, *args, ctypes.byref(receipt)))
            if result != 0:
                raise NativeCommandClientError(self.last_error(handle))
            if (
                int(receipt.abi_version)
                != NATIVE_EXPLORATION_COMMAND_RECEIPT_ABI_VERSION
                or int(receipt.struct_size)
                != ctypes.sizeof(_NativeExplorationCommandReceiptV1)
            ):
                raise NativeCommandClientError(
                    "native exploration command returned an incompatible receipt ABI"
                )
            return {
                "accepted": bool(receipt.accepted),
                "request_id": self._decode_fixed_text(receipt.request_id),
                "exploration_run_id": self._decode_fixed_text(
                    receipt.exploration_run_id
                ),
                "reason": self._decode_fixed_text(receipt.reason),
                "duplicate": bool(receipt.duplicate),
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def read_navigation_state(self) -> dict[str, object] | None:
        """Return the latest owning native navigation snapshot, if published."""

        self.ensure_host_state_abi()
        with self.lock:
            self.require_open()
            handle = self.handle
            function = self.library.lingtu_nav_client_read_navigation_state
            self._active_calls += 1
        state = _NativeNavigationState()
        try:
            result = int(function(handle, ctypes.byref(state)))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            return {
                "timestamp_s": float(state.timestamp_s),
                "frame_id": bytes(state.frame_id).split(b"\0", 1)[0].decode(
                    "utf-8", errors="replace"
                ),
                "boot_id": bytes(state.boot_id).split(b"\0", 1)[0].decode(
                    "utf-8", errors="replace"
                ),
                "sequence": int(state.sequence),
                "control_mode": int(state.control_mode),
                "lifecycle_state": int(state.lifecycle_state),
                "active_task_id": bytes(state.active_task_id)
                .split(b"\0", 1)[0]
                .decode("utf-8", errors="replace"),
                "active_request_id": bytes(state.active_request_id)
                .split(b"\0", 1)[0]
                .decode("utf-8", errors="replace"),
                "goal_epoch": int(state.goal_epoch),
                "map_id": bytes(state.map_id).split(b"\0", 1)[0].decode(
                    "utf-8", errors="replace"
                ),
                "map_content_epoch": int(state.map_content_epoch),
                "planning_state": int(state.planning_state),
                "execution_state": int(state.execution_state),
                "recovery_state": int(state.recovery_state),
                "progress": float(state.progress),
                "authority": bytes(state.authority).split(b"\0", 1)[0].decode(
                    "utf-8", errors="replace"
                ),
                "hold_reason": bytes(state.hold_reason).split(b"\0", 1)[0].decode(
                    "utf-8", errors="replace"
                ),
                "failure_code": bytes(state.failure_code).split(b"\0", 1)[0].decode(
                    "utf-8", errors="replace"
                ),
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def take_navigation_goal_status(self) -> dict[str, object] | None:
        """Pop one deduplicated request-lifecycle event from the native client."""

        self.ensure_goal_status_abi()
        return self._read_navigation_goal_status(
            self.library.lingtu_nav_client_take_navigation_goal_status,
        )

    def take_inspection_task_event(self) -> dict[str, object] | None:
        """Pop one native inspection task fact without inventing lifecycle state."""

        self.ensure_inspection_task_event_abi()
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        event = _NativeInspectionTaskEventV1()
        event.abi_version = NATIVE_INSPECTION_TASK_EVENT_ABI_VERSION
        event.struct_size = ctypes.sizeof(_NativeInspectionTaskEventV1)
        try:
            result = int(
                self.library.lingtu_nav_client_take_inspection_task_event_v1(
                    handle,
                    ctypes.byref(event),
                )
            )
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            if (
                int(event.abi_version) != NATIVE_INSPECTION_TASK_EVENT_ABI_VERSION
                or int(event.struct_size) != ctypes.sizeof(_NativeInspectionTaskEventV1)
            ):
                raise NativeCommandClientError(
                    "native inspection task event returned an incompatible event ABI"
                )
            return {
                "timestamp_s": float(event.timestamp_s),
                "frame_id": self._decode_fixed_text(event.frame_id),
                "boot_id": self._decode_fixed_text(event.boot_id),
                "event_sequence": int(event.event_sequence),
                "kind": int(event.kind),
                "task_id": self._decode_fixed_text(event.task_id),
                "request_id": self._decode_fixed_text(event.request_id),
                "command_request_id": self._decode_fixed_text(
                    event.command_request_id
                ),
                "state": int(event.state),
                "map_id": self._decode_fixed_text(event.map_id),
                "map_content_epoch": int(event.map_content_epoch),
                "route_id": self._decode_fixed_text(event.route_id),
                "route_revision": int(event.route_revision),
                "point_index": int(event.point_index),
                "point_count": int(event.point_count),
                "loop_index": int(event.loop_index),
                "retry_count": int(event.retry_count),
                "point_id": self._decode_fixed_text(event.point_id),
                "action": self._decode_fixed_text(event.action),
                "action_request_id": self._decode_fixed_text(
                    event.action_request_id
                ),
                "evidence_id": self._decode_fixed_text(event.evidence_id),
                "reason": self._decode_fixed_text(event.reason),
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def take_exploration_run_event(self) -> dict[str, object] | None:
        """Pop one validated native Explore lifecycle fact."""

        self.ensure_exploration_run_event_abi()
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        event = _NativeExplorationRunEventV1()
        event.abi_version = NATIVE_EXPLORATION_RUN_EVENT_ABI_VERSION
        event.struct_size = ctypes.sizeof(_NativeExplorationRunEventV1)
        try:
            result = int(
                self.library.lingtu_nav_client_take_exploration_run_event_v1(
                    handle,
                    ctypes.byref(event),
                )
            )
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            if (
                int(event.abi_version) != NATIVE_EXPLORATION_RUN_EVENT_ABI_VERSION
                or int(event.struct_size) != ctypes.sizeof(_NativeExplorationRunEventV1)
            ):
                raise NativeCommandClientError(
                    "native exploration run event returned an incompatible event ABI"
                )
            return {
                "timestamp_s": float(event.timestamp_s),
                "frame_id": self._decode_fixed_text(event.frame_id),
                "boot_id": self._decode_fixed_text(event.boot_id),
                "event_sequence": int(event.event_sequence),
                "kind": int(event.kind),
                "exploration_run_id": self._decode_fixed_text(
                    event.exploration_run_id
                ),
                "start_request_id": self._decode_fixed_text(event.start_request_id),
                "command_request_id": self._decode_fixed_text(
                    event.command_request_id
                ),
                "product_session_id": self._decode_fixed_text(
                    event.product_session_id
                ),
                "state": int(event.state),
                "route": self._decode_fixed_text(event.route),
                "map_id": self._decode_fixed_text(event.map_id),
                "map_content_epoch": int(event.map_content_epoch),
                "reason": self._decode_fixed_text(event.reason),
                "motion_stop_confirmed": bool(event.motion_stop_confirmed),
                "motion_stop_reason": self._decode_fixed_text(
                    event.motion_stop_reason
                ),
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def get_navigation_goal_status(self, request_id: str) -> dict[str, object] | None:
        """Return the retained latest lifecycle state for one request."""

        normalized = str(request_id or "").strip()
        if not normalized:
            raise ValueError("request_id is required")
        self.ensure_goal_status_abi()
        return self._read_navigation_goal_status(
            self.library.lingtu_nav_client_get_navigation_goal_status,
            normalized.encode("utf-8"),
        )

    def get_navigation_task_status(self, task_id: str) -> dict[str, object] | None:
        """Return the retained latest lifecycle state for one logical task."""

        normalized = str(task_id or "").strip()
        if not normalized:
            raise ValueError("task_id is required")
        self.ensure_goal_status_abi()
        self._require_capability(
            NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS,
            "navigation task status",
        )
        return self._read_navigation_goal_status_v1(
            self.library.lingtu_nav_client_get_navigation_task_status_v1,
            normalized.encode("utf-8"),
        )

    def start_navigation_task(
        self,
        task_id: str,
        request_id: str,
        x: float,
        y: float,
        z: float,
        yaw: float | None,
    ) -> dict[str, object]:
        """Submit one navigation task and return the correlated native ACK."""

        self.ensure_navigation_abi()
        task = str(task_id or "").strip()
        request = str(request_id or "").strip()
        if not task:
            raise ValueError("task_id is required")
        if not request:
            raise ValueError("request_id is required")
        return self._write_navigation_command_receipt(
            self.library.lingtu_nav_client_start_task_with_receipt_v1,
            task.encode("utf-8"),
            request.encode("utf-8"),
            float(x),
            float(y),
            float(z),
            math.nan if yaw is None else float(yaw),
            self.goal_timeout_ms,
        )

    def cancel_navigation_task(
        self,
        task_id: str,
        request_id: str,
        reason: str,
    ) -> dict[str, object]:
        """Cancel one logical navigation task and return the correlated ACK."""

        self.ensure_navigation_abi()
        task = str(task_id or "").strip()
        request = str(request_id or "").strip()
        if not task:
            raise ValueError("task_id is required")
        if not request:
            raise ValueError("request_id is required")
        return self._write_navigation_command_receipt(
            self.library.lingtu_nav_client_cancel_task_with_receipt_v1,
            task.encode("utf-8"),
            request.encode("utf-8"),
            str(reason or "cancel").encode("utf-8"),
            self.cancel_timeout_ms,
        )

    def preview_plan(
        self,
        request_id: str,
        x: float,
        y: float,
        z: float,
    ) -> dict[str, object]:
        """Run the endpoint planner without creating a navigation task."""

        self.ensure_plan_preview_abi()
        request = str(request_id or "").strip()
        if not request:
            raise ValueError("request_id is required")
        function = self.library.lingtu_nav_client_preview_plan_v1
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        result = _NativePlanResultV1()
        result.abi_version = NATIVE_PLAN_RESULT_ABI_VERSION
        result.struct_size = ctypes.sizeof(_NativePlanResultV1)
        try:
            status = int(
                function(
                    handle,
                    request.encode("utf-8"),
                    float(x),
                    float(y),
                    float(z),
                    self.goal_timeout_ms,
                    ctypes.byref(result),
                    None,
                    0,
                )
            )
            if (
                int(result.abi_version) != NATIVE_PLAN_RESULT_ABI_VERSION
                or int(result.struct_size) < ctypes.sizeof(_NativePlanResultV1)
            ):
                raise NativeCommandClientError(
                    "native plan preview returned an incompatible result ABI"
                )
            point_count = int(result.point_count)
            if status == 1:
                if point_count:
                    raise NativeCommandClientError(
                        "native plan preview skipped the required path buffer probe"
                    )
                path: list[dict[str, float]] = []
            elif status == 2:
                point_buffer = (_NativePathPoint * point_count)()
                status = int(
                    function(
                        handle,
                        request.encode("utf-8"),
                        float(x),
                        float(y),
                        float(z),
                        self.goal_timeout_ms,
                        ctypes.byref(result),
                        point_buffer,
                        point_count,
                    )
                )
                if status != 1:
                    if status < 0:
                        raise NativeCommandClientError(self.last_error(handle))
                    raise NativeCommandClientError(
                        "native plan preview did not complete its buffered copy"
                    )
                path = [
                    {"x": float(point.x), "y": float(point.y), "z": float(point.z)}
                    for point in point_buffer
                ]
            else:
                if status < 0:
                    raise NativeCommandClientError(self.last_error(handle))
                raise NativeCommandClientError(
                    f"native plan preview returned unexpected status {status}"
                )
            return {
                "timestamp_s": float(result.timestamp_s),
                "frame_id": self._decode_fixed_text(result.frame_id),
                "request_id": self._decode_fixed_text(result.request_id),
                "feasible": bool(result.feasible),
                "start_valid": bool(result.start_valid),
                "reason": self._decode_fixed_text(result.reason),
                "elapsed_ms": float(result.elapsed_ms),
                "planner": self._decode_fixed_text(result.planner),
                "start": {
                    "x": float(result.start.x),
                    "y": float(result.start.y),
                    "z": float(result.start.z),
                },
                "goal": {
                    "x": float(result.goal.x),
                    "y": float(result.goal.y),
                    "z": float(result.goal.z),
                },
                "point_count": int(result.point_count),
                "path": path,
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def pause_navigation_task(
        self,
        task_id: str,
        request_id: str,
        reason: str,
    ) -> dict[str, object]:
        """Request a stop-confirmed pause for one logical task."""

        return self._request_navigation_task_lifecycle(
            "lingtu_nav_client_pause_task_with_receipt_v1",
            task_id=task_id,
            request_id=request_id,
            reason=reason or "operator_pause",
        )

    def resume_navigation_task(
        self,
        task_id: str,
        request_id: str,
        reason: str,
    ) -> dict[str, object]:
        """Request continuation of the same paused logical task."""

        return self._request_navigation_task_lifecycle(
            "lingtu_nav_client_resume_task_with_receipt_v1",
            task_id=task_id,
            request_id=request_id,
            reason=reason or "operator_resume",
        )

    def resume_autonomy_with_receipt(
        self,
        request_id: str,
        reason: str,
    ) -> dict[str, object]:
        """Release manual takeover and return the correlated native ACK."""

        self.ensure_resume_autonomy_receipt_abi()
        return self._write_navigation_command_receipt(
            self.library.lingtu_nav_client_resume_autonomy_with_receipt_v1,
            str(request_id or "").encode("utf-8"),
            str(reason or "resume_autonomy").encode("utf-8"),
            self.cancel_timeout_ms,
        )

    def _request_navigation_task_lifecycle(
        self,
        symbol: str,
        *,
        task_id: str,
        request_id: str,
        reason: str,
    ) -> dict[str, object]:
        self.ensure_navigation_abi()
        function = getattr(self.library, symbol)
        task = str(task_id or "").strip()
        request = str(request_id or "").strip()
        if not task:
            raise ValueError("task_id is required")
        if not request:
            raise ValueError("request_id is required")
        if task == request:
            raise ValueError("task_id and request_id must be distinct")
        # Lifecycle controls share cancellation's short synchronous ACK budget.
        return self._write_navigation_command_receipt(
            function,
            task.encode("utf-8"),
            request.encode("utf-8"),
            str(reason).encode("utf-8"),
            self.cancel_timeout_ms,
        )

    def take_global_path(self) -> dict[str, object] | None:
        """Pop the latest complete native global path sample."""

        self.ensure_path_telemetry_abi()
        return self._take_path(self.library.lingtu_nav_client_take_global_path)

    def take_local_path(self) -> dict[str, object] | None:
        """Pop the latest complete native local path sample."""

        self.ensure_path_telemetry_abi()
        return self._take_path(self.library.lingtu_nav_client_take_local_path)

    def _take_path(self, function: Any) -> dict[str, object] | None:
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        header = _NativePathHeader()
        try:
            result = int(function(handle, ctypes.byref(header), None, 0))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            point_count = int(header.point_count)
            if result == 1:
                if point_count != 0:
                    raise NativeCommandClientError(
                        "native path telemetry returned a non-empty path without a buffer probe"
                    )
                points: list[dict[str, float]] = []
            elif result == 2:
                if point_count <= 0:
                    raise NativeCommandClientError(
                        "native path telemetry requested an invalid point buffer"
                    )
                point_buffer = (_NativePathPoint * point_count)()
                result = int(
                    function(
                        handle,
                        ctypes.byref(header),
                        point_buffer,
                        point_count,
                    )
                )
                if result != 1:
                    if result < 0:
                        raise NativeCommandClientError(self.last_error(handle))
                    raise NativeCommandClientError(
                        "native path telemetry did not complete its buffered copy"
                    )
                if int(header.point_count) != point_count:
                    raise NativeCommandClientError(
                        "native path telemetry point count changed during buffered copy"
                    )
                points = [
                    {
                        "x": float(point.x),
                        "y": float(point.y),
                        "z": float(point.z),
                    }
                    for point in point_buffer
                ]
            else:
                raise NativeCommandClientError(
                    f"native path telemetry returned unexpected status {result}"
                )
            return {
                "timestamp_s": float(header.timestamp_s),
                "frame_id": self._decode_fixed_text(header.frame_id),
                "receive_sequence": int(header.receive_sequence),
                "points": points,
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def take_map_scene(self) -> dict[str, object] | None:
        """Pop one coherent MapScene after validating all product capacities."""

        self.ensure_map_scene_abi()
        with self.lock:
            self.require_open()
            handle = self.handle
            function = self.library.lingtu_nav_client_take_map_scene_v1
            self._active_calls += 1
        header = _NativeMapSceneHeaderV1()
        try:
            result = int(function(handle, ctypes.byref(header), None))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            self._validate_map_scene_header(header)
            if result == 1:
                if self._map_scene_payload_count(header) != 0:
                    raise NativeCommandClientError(
                        "native map scene returned payload without a capacity probe"
                    )
                return self._map_scene_result(header, {}, {})
            if result != 2:
                raise NativeCommandClientError(
                    f"native map scene returned unexpected status {result}"
                )

            point_counts = {
                "live": int(header.live_point_count),
                "voxel": int(header.voxel_point_count),
                "accumulated": int(header.accumulated_point_count),
            }
            grid_headers = {
                "occupancy": header.occupancy,
                "elevation": header.elevation,
                "esdf": header.esdf,
            }
            point_buffers = {
                name: (_NativeMapScenePointV1 * count)()
                for name, count in point_counts.items()
                if count
            }
            grid_buffers = {
                name: (ctypes.c_float * int(grid.cell_count))()
                for name, grid in grid_headers.items()
                if int(grid.cell_count)
            }
            buffers = _NativeMapSceneBuffersV1()
            buffers.abi_version = NATIVE_MAP_SCENE_ABI_VERSION
            buffers.struct_size = ctypes.sizeof(_NativeMapSceneBuffersV1)
            for name in point_counts:
                setattr(buffers, f"{name}_points", point_buffers.get(name))
                setattr(buffers, f"{name}_point_capacity", point_counts[name])
            for name, grid in grid_headers.items():
                setattr(buffers, f"{name}_cells", grid_buffers.get(name))
                setattr(
                    buffers,
                    f"{name}_cell_capacity",
                    int(grid.cell_count),
                )
            identity = self._map_scene_header_identity(header)
            result = int(
                function(handle, ctypes.byref(header), ctypes.byref(buffers))
            )
            if result != 1:
                if result < 0:
                    raise NativeCommandClientError(self.last_error(handle))
                raise NativeCommandClientError(
                    "native map scene did not complete its buffered copy"
                )
            self._validate_map_scene_header(header)
            if self._map_scene_header_identity(header) != identity:
                raise NativeCommandClientError(
                    "native map scene identity changed during buffered copy"
                )
            point_bytes = {
                name: ctypes.string_at(
                    ctypes.addressof(buffer),
                    ctypes.sizeof(buffer),
                )
                for name, buffer in point_buffers.items()
            }
            grid_bytes = {
                name: ctypes.string_at(
                    ctypes.addressof(buffer),
                    ctypes.sizeof(buffer),
                )
                for name, buffer in grid_buffers.items()
            }
            return self._map_scene_result(header, point_bytes, grid_bytes)
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def take_traversability_grid(self) -> dict[str, object] | None:
        """Pop one bounded native traversability-grid sample."""

        self.ensure_traversability_abi()
        with self.lock:
            self.require_open()
            handle = self.handle
            function = self.library.lingtu_nav_client_take_traversability_grid_v1
            self._active_calls += 1
        header = _NativeTraversabilityGridHeaderV1()
        try:
            result = int(function(handle, ctypes.byref(header), None, 0))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            self._validate_traversability_header(header)
            if result != 2:
                raise NativeCommandClientError(
                    "native traversability grid did not provide a capacity probe"
                )
            count = int(header.cell_count)
            cells = (ctypes.c_uint8 * count)()
            identity = self._traversability_header_identity(header)
            result = int(function(handle, ctypes.byref(header), cells, count))
            if result != 1:
                if result < 0:
                    raise NativeCommandClientError(self.last_error(handle))
                raise NativeCommandClientError(
                    "native traversability grid buffered copy was incomplete"
                )
            self._validate_traversability_header(header)
            if self._traversability_header_identity(header) != identity:
                raise NativeCommandClientError(
                    "native traversability grid identity changed during copy"
                )
            cell_bytes = bytes(cells)
            if any(value > 100 for value in cell_bytes):
                raise NativeCommandClientError(
                    "native traversability grid cell is outside control-risk range"
                )
            return {
                "timestamp_s": float(header.timestamp_s),
                "frame_id": self._decode_fixed_text(header.frame_id),
                "receive_sequence": int(header.receive_sequence),
                "reset_epoch": int(header.reset_epoch),
                "width": int(header.width),
                "height": int(header.height),
                "resolution": float(header.resolution),
                "origin": {
                    "x": float(header.origin_x),
                    "y": float(header.origin_y),
                    "z": float(header.origin_z),
                },
                "yaw": float(header.yaw),
                "cells_u8": cell_bytes,
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    @staticmethod
    def _traversability_header_identity(
        header: _NativeTraversabilityGridHeaderV1,
    ) -> tuple[int, ...]:
        return (
            int(header.receive_sequence),
            int(header.reset_epoch),
            int(header.width),
            int(header.height),
            int(header.cell_count),
        )

    @classmethod
    def _validate_traversability_header(
        cls,
        header: _NativeTraversabilityGridHeaderV1,
    ) -> None:
        if (
            int(header.abi_version) != NATIVE_TRAVERSABILITY_GRID_ABI_VERSION
            or int(header.struct_size) < ctypes.sizeof(_NativeTraversabilityGridHeaderV1)
        ):
            raise NativeCommandClientError(
                "native traversability grid ABI header is incompatible"
            )
        width = int(header.width)
        height = int(header.height)
        count = int(header.cell_count)
        if (
            width <= 0
            or height <= 0
            or count != width * height
            or count > NATIVE_TRAVERSABILITY_MAX_CELLS
        ):
            raise NativeCommandClientError(
                "native traversability grid dimensions exceed product limit"
            )
        if (
            not math.isfinite(float(header.timestamp_s))
            or float(header.timestamp_s) <= 0.0
            or int(header.receive_sequence) <= 0
            or int(header.reset_epoch) <= 0
            or cls._decode_fixed_text(header.frame_id) != "map"
            or not math.isfinite(float(header.resolution))
            or float(header.resolution) <= 0.0
            or not math.isfinite(float(header.origin_x))
            or not math.isfinite(float(header.origin_y))
            or not math.isfinite(float(header.origin_z))
            or not math.isfinite(float(header.yaw))
            or abs(float(header.yaw)) > 1e-6
        ):
            raise NativeCommandClientError(
                "native traversability grid identity or transform is invalid"
            )

    def read_map_scene_health(self) -> dict[str, object]:
        """Return native MapScene receive, rejection, and capacity counters."""

        self.ensure_map_scene_abi()
        with self.lock:
            self.require_open()
            handle = self.handle
            function = self.library.lingtu_nav_client_read_map_scene_health_v1
            self._active_calls += 1
        health = _NativeMapSceneHealthV1()
        try:
            result = int(function(handle, ctypes.byref(health)))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if (
                int(health.abi_version) != NATIVE_MAP_SCENE_ABI_VERSION
                or int(health.struct_size) < ctypes.sizeof(_NativeMapSceneHealthV1)
            ):
                raise NativeCommandClientError(
                    "native map scene health returned an incompatible v1 struct"
                )
            return {
                "received_samples": int(health.received_samples),
                "valid_samples": int(health.valid_samples),
                "stale_samples": int(health.stale_samples),
                "invalid_samples": int(health.invalid_samples),
                "capacity_rejections": int(health.capacity_rejections),
                "replaced_samples": int(health.replaced_samples),
                "consumer_buffer_retries": int(
                    health.consumer_buffer_retries
                ),
                "last_receive_sequence": int(health.last_receive_sequence),
                "last_generation": int(health.last_generation),
                "last_sample_timestamp_s": float(
                    health.last_sample_timestamp_s
                ),
                "pending": bool(health.pending),
                "last_error": self._decode_fixed_text(health.last_error),
                "state_received_samples": int(
                    health.state_received_samples
                ),
                "state_valid_samples": int(health.state_valid_samples),
                "state_stale_samples": int(health.state_stale_samples),
                "state_invalid_samples": int(
                    health.state_invalid_samples
                ),
                "state_timestamp_s": float(health.state_timestamp_s),
                "state_producer_boot_id": self._decode_fixed_text(
                    health.state_producer_boot_id
                ),
                "state_received": bool(health.state_received),
                "state_running": bool(health.state_running),
                "state_live": bool(health.state_live),
                "state_required_publications_ready": bool(
                    health.state_required_publications_ready
                ),
                "state_current_generation_published": bool(
                    health.state_current_generation_published
                ),
                "state_capacity_limited": bool(
                    health.state_capacity_limited
                ),
                "state_reset_epoch": int(health.state_reset_epoch),
                "state_observation_sequence": int(
                    health.state_observation_sequence
                ),
                "state_generation": int(health.state_generation),
                "state_scene_published_generation": int(
                    health.state_scene_published_generation
                ),
                "state_error": self._decode_fixed_text(
                    health.state_error
                ),
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    @staticmethod
    def _map_scene_payload_count(header: _NativeMapSceneHeaderV1) -> int:
        return sum(
            (
                int(header.live_point_count),
                int(header.voxel_point_count),
                int(header.accumulated_point_count),
                int(header.occupancy.cell_count),
                int(header.elevation.cell_count),
                int(header.esdf.cell_count),
            )
        )

    @staticmethod
    def _map_scene_header_identity(
        header: _NativeMapSceneHeaderV1,
    ) -> tuple[int, ...]:
        return (
            int(header.receive_sequence),
            int(header.reset_epoch),
            int(header.observation_sequence),
            int(header.generation),
            int(header.live_point_count),
            int(header.voxel_point_count),
            int(header.accumulated_point_count),
            int(header.occupancy.cell_count),
            int(header.elevation.cell_count),
            int(header.esdf.cell_count),
            int(header.payload_bytes),
        )

    @classmethod
    def _validate_map_scene_header(
        cls,
        header: _NativeMapSceneHeaderV1,
    ) -> None:
        if (
            int(header.abi_version) != NATIVE_MAP_SCENE_ABI_VERSION
            or int(header.struct_size) < ctypes.sizeof(_NativeMapSceneHeaderV1)
        ):
            raise NativeCommandClientError(
                "native map scene returned an incompatible v1 header"
            )
        point_counts = [
            int(header.live_point_count),
            int(header.voxel_point_count),
            int(header.accumulated_point_count),
        ]
        grid_headers = [
            header.occupancy,
            header.elevation,
            header.esdf,
        ]
        grid_counts = [int(grid.cell_count) for grid in grid_headers]
        if any(
            count < 0 or count > NATIVE_MAP_SCENE_MAX_POINTS_PER_LAYER
            for count in point_counts
        ) or sum(point_counts) > NATIVE_MAP_SCENE_MAX_TOTAL_POINTS:
            raise NativeCommandClientError(
                "native map scene point count exceeds Python product limit"
            )
        if any(
            count < 0 or count > NATIVE_MAP_SCENE_MAX_GRID_CELLS_PER_LAYER
            for count in grid_counts
        ) or sum(grid_counts) > NATIVE_MAP_SCENE_MAX_TOTAL_GRID_CELLS:
            raise NativeCommandClientError(
                "native map scene grid count exceeds Python product limit"
            )
        for grid, count in zip(grid_headers, grid_counts, strict=True):
            if int(grid.width) * int(grid.height) != count:
                raise NativeCommandClientError(
                    "native map scene grid dimensions do not match cell count"
                )
            if count and (
                not math.isfinite(float(grid.resolution))
                or float(grid.resolution) <= 0.0
            ):
                raise NativeCommandClientError(
                    "native map scene grid resolution is invalid"
                )
        expected_bytes = (
            sum(point_counts) * ctypes.sizeof(_NativeMapScenePointV1)
            + sum(grid_counts) * ctypes.sizeof(ctypes.c_float)
        )
        if (
            expected_bytes != int(header.payload_bytes)
            or expected_bytes > NATIVE_MAP_SCENE_MAX_PAYLOAD_BYTES
        ):
            raise NativeCommandClientError(
                "native map scene payload byte count exceeds or violates product contract"
            )
        if (
            int(header.receive_sequence) <= 0
            or int(header.observation_sequence) <= 0
            or int(header.generation) <= 0
            or int(header.live) not in (0, 1)
            or not math.isfinite(float(header.timestamp_s))
            or float(header.timestamp_s) <= 0.0
            or not cls._decode_fixed_text(header.frame_id)
            or not cls._decode_fixed_text(header.producer_boot_id)
        ):
            raise NativeCommandClientError(
                "native map scene identity or timestamp is invalid"
            )
        pose = (
            header.sensor_x,
            header.sensor_y,
            header.sensor_z,
            header.sensor_qx,
            header.sensor_qy,
            header.sensor_qz,
            header.sensor_qw,
        )
        if not all(math.isfinite(float(value)) for value in pose):
            raise NativeCommandClientError(
                "native map scene sensor pose is invalid"
            )

    @classmethod
    def _map_scene_result(
        cls,
        header: _NativeMapSceneHeaderV1,
        point_bytes: dict[str, bytes],
        grid_bytes: dict[str, bytes],
    ) -> dict[str, object]:
        def grid(name: str, value: _NativeMapSceneGridHeaderV1) -> dict[str, object]:
            return {
                "width": int(value.width),
                "height": int(value.height),
                "resolution": float(value.resolution),
                "origin": {
                    "x": float(value.origin_x),
                    "y": float(value.origin_y),
                    "z": float(value.origin_z),
                    "qx": float(value.origin_qx),
                    "qy": float(value.origin_qy),
                    "qz": float(value.origin_qz),
                    "qw": float(value.origin_qw),
                },
                "cell_count": int(value.cell_count),
                "values_f32": grid_bytes.get(name, b""),
            }

        return {
            "timestamp_s": float(header.timestamp_s),
            "frame_id": cls._decode_fixed_text(header.frame_id),
            "producer_boot_id": cls._decode_fixed_text(
                header.producer_boot_id
            ),
            "receive_sequence": int(header.receive_sequence),
            "reset_epoch": int(header.reset_epoch),
            "observation_sequence": int(header.observation_sequence),
            "generation": int(header.generation),
            "live": bool(header.live),
            "sensor_pose": {
                "x": float(header.sensor_x),
                "y": float(header.sensor_y),
                "z": float(header.sensor_z),
                "qx": float(header.sensor_qx),
                "qy": float(header.sensor_qy),
                "qz": float(header.sensor_qz),
                "qw": float(header.sensor_qw),
            },
            "payload_bytes": int(header.payload_bytes),
            "clouds": {
                "live": {
                    "point_count": int(header.live_point_count),
                    "points_xyzi_f32": point_bytes.get("live", b""),
                },
                "voxel": {
                    "point_count": int(header.voxel_point_count),
                    "points_xyzi_f32": point_bytes.get("voxel", b""),
                },
                "accumulated": {
                    "point_count": int(header.accumulated_point_count),
                    "points_xyzi_f32": point_bytes.get("accumulated", b""),
                },
            },
            "grids": {
                "occupancy": grid("occupancy", header.occupancy),
                "elevation": grid("elevation", header.elevation),
                "esdf": grid("esdf", header.esdf),
            },
        }

    def _read_navigation_goal_status(
        self,
        function: Any,
        *arguments: object,
    ) -> dict[str, object] | None:
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        status = _NativeNavigationGoalStatus()
        try:
            result = int(function(handle, *arguments, ctypes.byref(status)))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
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
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def _read_navigation_goal_status_v1(
        self,
        function: Any,
        *arguments: object,
    ) -> dict[str, object] | None:
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        status = _NativeNavigationGoalStatusV1()
        status.abi_version = NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION
        status.struct_size = ctypes.sizeof(_NativeNavigationGoalStatusV1)
        try:
            result = int(function(handle, *arguments, ctypes.byref(status)))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if result == 0:
                return None
            if (
                int(status.abi_version) != NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION
                or int(status.struct_size) != ctypes.sizeof(_NativeNavigationGoalStatusV1)
            ):
                raise NativeCommandClientError(
                    "native navigation task status returned an incompatible receipt ABI"
                )
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
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

    def _write_navigation_command_receipt(
        self,
        function: Any,
        *arguments: object,
    ) -> dict[str, object]:
        with self.lock:
            self.require_open()
            handle = self.handle
            self._active_calls += 1
        receipt = _NativeNavigationCommandReceiptV1()
        receipt.abi_version = NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION
        receipt.struct_size = ctypes.sizeof(_NativeNavigationCommandReceiptV1)
        try:
            result = int(function(handle, *arguments, ctypes.byref(receipt)))
            if result < 0:
                raise NativeCommandClientError(self.last_error(handle))
            if (
                int(receipt.abi_version) != NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION
                or int(receipt.struct_size) != ctypes.sizeof(_NativeNavigationCommandReceiptV1)
            ):
                raise NativeCommandClientError(
                    "native navigation command returned an incompatible receipt ABI"
                )
            return {
                "accepted": bool(receipt.accepted),
                "kind": int(receipt.kind),
                "task_id": self._decode_fixed_text(receipt.task_id),
                "request_id": self._decode_fixed_text(receipt.request_id),
                "endpoint_timestamp_s": float(receipt.endpoint_timestamp_s),
                "reason": self._decode_fixed_text(receipt.reason),
            }
        finally:
            with self.lock:
                self._active_calls -= 1
                if self._active_calls == 0:
                    self._state_changed.notify_all()

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


_SESSIONS: dict[tuple[str, int, int, int, int, int, int], NativeCommandSession] = {}
_SESSIONS_LOCK = threading.Lock()


def get_native_command_session(*, required: bool = False) -> NativeCommandSession | None:
    """Return the process-wide C++ command session configured by the active RunPlan."""

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
        operator_motion_timeout_ms = int(
            os.environ.get("LINGTU_NAV_OPERATOR_MOTION_TIMEOUT_MS", str(teleop_timeout_ms))
            or str(teleop_timeout_ms)
        )
    except ValueError as exc:
        raise NativeCommandClientError("invalid native navigation client configuration") from exc
    key = (
        str(Path(raw_path).resolve()),
        domain_id,
        max(1, timeout_ms),
        max(1, goal_timeout_ms),
        max(1, cancel_timeout_ms),
        max(1, teleop_timeout_ms),
        max(1, operator_motion_timeout_ms),
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
                operator_motion_timeout_ms=operator_motion_timeout_ms,
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
