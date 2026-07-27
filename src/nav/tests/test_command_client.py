from __future__ import annotations

import ctypes
import threading

import pytest

from nav.adapters.native import abi as native_abi
from nav.adapters.native.commands import (
    NativeNavigationClient,
    NavigationClientError,
)
from runtime.msgs import NavigationCommandKind, NavigationCommandReceipt


class _Function:
    def __init__(self, callback):
        self._callback = callback
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self._callback(*args)


class _Library:
    def __init__(self) -> None:
        self.calls = []
        self.error = b""
        self.lingtu_nav_client_abi_version = _Function(lambda: 5)
        self.lingtu_nav_client_capabilities = _Function(lambda: 0x3FFF)
        self.lingtu_nav_client_create = _Function(self._create)
        self.lingtu_nav_client_destroy = _Function(self._destroy)
        self.lingtu_nav_client_send_goal = _Function(self._goal)
        self.lingtu_nav_client_send_goal_with_id = _Function(self._goal_with_id)
        self.lingtu_nav_client_start_task_with_receipt_v1 = _Function(self._start_task_with_receipt)
        self.lingtu_nav_client_cancel = _Function(self._cancel)
        self.lingtu_nav_client_cancel_with_id = _Function(self._cancel_with_id)
        self.lingtu_nav_client_cancel_task_with_receipt_v1 = _Function(self._cancel_task_with_receipt)
        self.lingtu_nav_client_send_teleop = _Function(self._teleop)
        self.lingtu_nav_client_send_teleop_with_id = _Function(self._teleop_with_id)
        self.lingtu_nav_client_stop = _Function(self._stop)
        self.lingtu_nav_client_stop_with_id = _Function(self._stop_with_id)
        self.lingtu_nav_client_estop = _Function(self._estop)
        self.lingtu_nav_client_estop_with_id = _Function(self._estop_with_id)
        self.lingtu_nav_client_clear_estop = _Function(self._clear_estop)
        self.lingtu_nav_client_clear_estop_with_id = _Function(self._clear_estop_with_id)
        self.lingtu_nav_client_resume_autonomy = _Function(self._resume_autonomy)
        self.lingtu_nav_client_resume_autonomy_with_id = _Function(self._resume_autonomy_with_id)
        self.lingtu_nav_client_read_navigation_state_v1 = _Function(lambda *_args: 0)
        self.lingtu_nav_client_take_navigation_goal_status_v1 = _Function(lambda *_args: 0)
        self.lingtu_nav_client_get_navigation_goal_status_v1 = _Function(lambda *_args: 0)
        self.lingtu_nav_client_get_navigation_task_status_v1 = _Function(lambda *_args: 0)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: self.error)

    def _create(self, domain_id):
        self.calls.append(("create", domain_id))
        return 41

    def _destroy(self, handle):
        self.calls.append(("destroy", handle))

    def _goal(self, handle, x, y, z, yaw, timeout_ms):
        self.calls.append(("goal", handle, x, y, z, yaw, timeout_ms))
        return 0

    def _goal_with_id(self, handle, request_id, x, y, z, yaw, timeout_ms):
        self.calls.append(("goal", handle, request_id, x, y, z, yaw, timeout_ms))
        return 0

    def _start_task_with_receipt(
        self,
        handle,
        task_id,
        request_id,
        x,
        y,
        z,
        yaw,
        timeout_ms,
        receipt_pointer,
    ):
        self.calls.append(("start_task", handle, task_id, request_id, x, y, z, yaw, timeout_ms))
        self._fill_command_receipt(
            receipt_pointer,
            task_id=task_id,
            request_id=request_id,
            kind=int(NavigationCommandKind.GOAL),
        )
        return 0

    def _cancel(self, handle, reason, timeout_ms):
        self.calls.append(("cancel", handle, reason, timeout_ms))
        return 0

    def _cancel_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("cancel", handle, request_id, reason, timeout_ms))
        return 0

    def _cancel_task_with_receipt(
        self,
        handle,
        task_id,
        request_id,
        reason,
        timeout_ms,
        receipt_pointer,
    ):
        self.calls.append(("cancel_task", handle, task_id, request_id, reason, timeout_ms))
        self._fill_command_receipt(
            receipt_pointer,
            task_id=task_id,
            request_id=request_id,
            kind=int(NavigationCommandKind.CANCEL),
            reason=reason,
        )
        return 0

    @staticmethod
    def _fill_command_receipt(
        pointer,
        *,
        task_id,
        request_id,
        kind,
        reason=b"accepted",
        accepted=True,
    ):
        receipt = pointer._obj
        assert receipt.abi_version == native_abi.NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION
        assert receipt.struct_size == ctypes.sizeof(native_abi._NativeNavigationCommandReceiptV1)
        receipt.abi_version = native_abi.NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION
        receipt.struct_size = ctypes.sizeof(native_abi._NativeNavigationCommandReceiptV1)
        receipt.task_id = task_id
        receipt.request_id = request_id
        receipt.accepted = int(accepted)
        receipt.kind = int(kind)
        receipt.reason = reason
        receipt.endpoint_timestamp_s = 123.5

    def _teleop(self, handle, vx, vy, wz, timeout_ms):
        self.calls.append(("teleop", handle, vx, vy, wz, timeout_ms))
        return 0

    def _teleop_with_id(self, handle, request_id, vx, vy, wz, timeout_ms):
        self.calls.append(("teleop", handle, request_id, vx, vy, wz, timeout_ms))
        return 0

    def _stop(self, handle, reason, timeout_ms):
        self.calls.append(("stop", handle, reason, timeout_ms))
        return 0

    def _stop_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("stop", handle, request_id, reason, timeout_ms))
        return 0

    def _estop(self, handle, reason, timeout_ms):
        self.calls.append(("estop", handle, reason, timeout_ms))
        return 0

    def _estop_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("estop", handle, request_id, reason, timeout_ms))
        return 0

    def _clear_estop(self, handle, reason, timeout_ms):
        self.calls.append(("clear_estop", handle, reason, timeout_ms))
        return 0

    def _clear_estop_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("clear_estop", handle, request_id, reason, timeout_ms))
        return 0

    def _resume_autonomy(self, handle, reason, timeout_ms):
        self.calls.append(("resume_autonomy", handle, reason, timeout_ms))
        return 0

    def _resume_autonomy_with_id(self, handle, request_id, reason, timeout_ms):
        self.calls.append(("resume_autonomy", handle, request_id, reason, timeout_ms))
        return 0


def test_native_client_reuses_one_cpp_handle_for_commands(tmp_path):
    library = _Library()
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=7,
        timeout_ms=250,
        library=library,
    )

    client.send_goal(1.0, 2.0, 0.3, 0.4, request_id="goal-001")
    client.cancel("operator_cancel", request_id="cancel-001")
    client.send_teleop(0.2, -0.1, 0.5, request_id="teleop-001")
    client.stop("operator_stop", request_id="stop-001")
    client.estop("operator_estop", request_id="estop-001")
    client.clear_estop("operator_reset", request_id="clear-estop-001")
    client.resume_autonomy("operator_resume", request_id="resume-001")
    client.close()

    assert library.calls == [
        ("create", 7),
        ("goal", 41, b"goal-001", 1.0, 2.0, 0.3, 0.4, 250),
        ("cancel", 41, b"cancel-001", b"operator_cancel", 250),
        ("teleop", 41, b"teleop-001", 0.2, -0.1, 0.5, 250),
        ("stop", 41, b"stop-001", b"operator_stop", 250),
        ("estop", 41, b"estop-001", b"operator_estop", 250),
        ("clear_estop", 41, b"clear-estop-001", b"operator_reset", 250),
        ("resume_autonomy", 41, b"resume-001", b"operator_resume", 250),
        ("destroy", 41),
    ]


def test_native_client_surfaces_cpp_delivery_error(tmp_path):
    library = _Library()
    library.error = b"no matched DDS reader for goal_pose"
    library.lingtu_nav_client_send_goal_with_id = _Function(lambda *_args: -1)
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        client.send_goal(1.0, 2.0, 0.0, 0.0)
    except NavigationClientError as exc:
        assert str(exc) == "no matched DDS reader for goal_pose"
    else:
        raise AssertionError("native client error was not raised")
    finally:
        client.close()


def test_native_navigation_client_rejects_incompatible_abi(tmp_path):
    library = _Library()
    library.lingtu_nav_client_abi_version = _Function(lambda: 99)

    try:
        NativeNavigationClient(
            tmp_path / "liblingtu_nav_client.so",
            library=library,
        )
    except NavigationClientError as exc:
        assert "ABI version" in str(exc)
    else:
        raise AssertionError("incompatible native client ABI was accepted")


def test_native_navigation_client_requires_navigation_capability(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: 0x3FFE)

    try:
        NativeNavigationClient(
            tmp_path / "liblingtu_nav_client.so",
            library=library,
        )
    except NavigationClientError as exc:
        assert "navigation commands capability" in str(exc)
    else:
        raise AssertionError("missing navigation capability was accepted")

    assert library.calls == [("create", 0), ("destroy", 41)]


def test_python_session_does_not_serialize_estop_behind_goal(tmp_path):
    library = _Library()
    goal_started = threading.Event()
    release_goal = threading.Event()
    estop_seen = threading.Event()

    def blocked_goal(*_args):
        goal_started.set()
        release_goal.wait(timeout=1.0)
        return 0

    def immediate_estop(*_args):
        estop_seen.set()
        return 0

    library.lingtu_nav_client_send_goal_with_id = _Function(blocked_goal)
    library.lingtu_nav_client_estop_with_id = _Function(immediate_estop)
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )
    goal_thread = threading.Thread(
        target=lambda: client.send_goal(1.0, 2.0, 0.0, 0.0),
    )
    estop_thread = threading.Thread(target=lambda: client.estop("operator_estop"))

    try:
        goal_thread.start()
        assert goal_started.wait(timeout=0.5)
        estop_thread.start()
        assert estop_seen.wait(timeout=0.25)
    finally:
        release_goal.set()
        goal_thread.join(timeout=1.0)
        estop_thread.join(timeout=1.0)
        client.close()

    assert not goal_thread.is_alive()
    assert not estop_thread.is_alive()


def test_python_ctypes_preserves_legacy_navigation_struct_layouts() -> None:
    assert [name for name, _type in native_abi._NativeNavigationState._fields_] == [
        "timestamp_s",
        "frame_id",
        "boot_id",
        "sequence",
        "control_mode",
        "lifecycle_state",
        "active_request_id",
        "goal_epoch",
        "map_id",
        "map_version",
        "map_hash",
        "planning_state",
        "execution_state",
        "recovery_state",
        "progress",
        "authority",
        "hold_reason",
        "failure_code",
    ]
    assert [name for name, _type in native_abi._NativeNavigationGoalStatus._fields_] == [
        "timestamp_s",
        "frame_id",
        "boot_id",
        "sequence",
        "request_id",
        "state",
        "goal_epoch",
        "reason",
    ]
    assert "active_task_id" in {name for name, _type in native_abi._NativeNavigationStateV1._fields_}
    assert "task_id" in {name for name, _type in native_abi._NativeNavigationGoalStatusV1._fields_}


def test_native_session_uses_v1_state_and_goal_status_symbols(tmp_path) -> None:
    library = _Library()
    calls: list[tuple[object, ...]] = []

    def fill_status(pointer, *, sequence=11):
        status = pointer._obj
        assert status.abi_version == native_abi.NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION
        assert status.struct_size == ctypes.sizeof(native_abi._NativeNavigationGoalStatusV1)
        status.abi_version = native_abi.NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION
        status.struct_size = ctypes.sizeof(native_abi._NativeNavigationGoalStatusV1)
        status.timestamp_s = 42.5
        status.frame_id = b"map"
        status.boot_id = b"navd-boot"
        status.sequence = sequence
        status.task_id = b"task-42"
        status.request_id = b"goal-42"
        status.state = 4
        status.goal_epoch = 9
        status.reason = b"goal_reached"

    def read_state(_handle, pointer):
        calls.append(("state_v1",))
        state = pointer._obj
        assert state.abi_version == native_abi.NATIVE_NAVIGATION_STATE_ABI_VERSION
        assert state.struct_size == ctypes.sizeof(native_abi._NativeNavigationStateV1)
        state.abi_version = native_abi.NATIVE_NAVIGATION_STATE_ABI_VERSION
        state.struct_size = ctypes.sizeof(native_abi._NativeNavigationStateV1)
        state.timestamp_s = 40.0
        state.frame_id = b"map"
        state.boot_id = b"navd-boot"
        state.sequence = 10
        state.control_mode = 1
        state.lifecycle_state = 2
        state.active_task_id = b"task-42"
        state.active_request_id = b"goal-42"
        state.goal_epoch = 9
        state.map_id = b"yard"
        state.map_version = 3
        state.map_hash = b"sha256:abc"
        state.planning_state = 2
        state.execution_state = 1
        state.recovery_state = 0
        state.progress = 0.5
        state.authority = b"autonomy"
        return 1

    take_count = 0

    def take_status(_handle, pointer):
        nonlocal take_count
        calls.append(("take_status_v1",))
        if take_count:
            return 0
        take_count += 1
        fill_status(pointer)
        return 1

    def get_status(_handle, request_id, pointer):
        calls.append(("get_status_v1", request_id))
        fill_status(pointer)
        return 1

    def get_task_status(_handle, task_id, pointer):
        calls.append(("get_task_status_v1", task_id))
        fill_status(pointer, sequence=12)
        return 1

    library.lingtu_nav_client_read_navigation_state_v1 = _Function(read_state)
    library.lingtu_nav_client_take_navigation_goal_status_v1 = _Function(take_status)
    library.lingtu_nav_client_get_navigation_goal_status_v1 = _Function(get_status)
    library.lingtu_nav_client_get_navigation_task_status_v1 = _Function(get_task_status)
    session = native_abi.NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    state = session.read_navigation_state()
    event = session.take_navigation_goal_status()

    assert state is not None
    assert state["active_task_id"] == "task-42"
    assert state["active_request_id"] == "goal-42"
    assert event is not None
    assert event["task_id"] == "task-42"
    assert session.take_navigation_goal_status() is None
    assert session.get_navigation_goal_status("goal-42") == event
    task_status = session.get_navigation_task_status("task-42")
    assert task_status is not None
    assert task_status["sequence"] == 12
    assert calls == [
        ("state_v1",),
        ("take_status_v1",),
        ("take_status_v1",),
        ("get_status_v1", b"goal-42"),
        ("get_task_status_v1", b"task-42"),
    ]
    session.close()


def test_native_session_returns_typed_task_command_receipts(tmp_path) -> None:
    library = _Library()
    session = native_abi.NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        goal_timeout_ms=700,
        cancel_timeout_ms=300,
        library=library,
    )

    started = session.start_navigation_task("task-42", "goal-42", 1.0, 2.0, 0.3, 0.4)
    cancelled = session.cancel_navigation_task("task-42", "cancel-42", "operator_cancel")

    assert started == NavigationCommandReceipt(
        accepted=True,
        kind=int(NavigationCommandKind.GOAL),
        task_id="task-42",
        request_id="goal-42",
        endpoint_timestamp_s=123.5,
        reason="accepted",
    )
    assert cancelled == NavigationCommandReceipt(
        accepted=True,
        kind=int(NavigationCommandKind.CANCEL),
        task_id="task-42",
        request_id="cancel-42",
        endpoint_timestamp_s=123.5,
        reason="operator_cancel",
    )
    assert library.calls[-2:] == [
        ("start_task", 41, b"task-42", b"goal-42", 1.0, 2.0, 0.3, 0.4, 700),
        (
            "cancel_task",
            41,
            b"task-42",
            b"cancel-42",
            b"operator_cancel",
            300,
        ),
    ]
    session.close()


def test_native_session_rejects_equal_task_and_request_identity(tmp_path) -> None:
    library = _Library()
    session = native_abi.NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(ValueError, match="must be distinct"):
        session.start_navigation_task("same-id", "same-id", 1.0, 2.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="must be distinct"):
        session.cancel_navigation_task("same-id", "same-id", "cancel")
    assert not any(call[0] in {"start_task", "cancel_task"} for call in library.calls)
    session.close()


@pytest.mark.parametrize(
    ("missing_capability", "operation", "error_pattern"),
    [
        (
            native_abi.NATIVE_COMMAND_CAP_NAVIGATION_STATE_V1,
            "state",
            "navigation state v1 capability",
        ),
        (
            native_abi.NATIVE_COMMAND_CAP_NAVIGATION_GOAL_STATUS_V1,
            "status",
            "navigation goal status v1 capability",
        ),
    ],
)
def test_native_session_requires_versioned_state_and_status_capabilities(
    tmp_path,
    missing_capability,
    operation,
    error_pattern,
) -> None:
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: 0x3FFF & ~missing_capability)
    session = native_abi.NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(native_abi.NativeCommandClientError, match=error_pattern):
        if operation == "state":
            session.read_navigation_state()
        else:
            session.take_navigation_goal_status()
    session.close()


@pytest.mark.parametrize("metadata_field", ["abi_version", "struct_size"])
def test_native_session_rejects_invalid_navigation_receipt_metadata(
    tmp_path,
    metadata_field,
) -> None:
    library = _Library()

    def invalid_receipt(*arguments):
        pointer = arguments[-1]
        library._fill_command_receipt(
            pointer,
            task_id=b"task-42",
            request_id=b"goal-42",
            kind=int(NavigationCommandKind.GOAL),
        )
        setattr(pointer._obj, metadata_field, 99)
        return 0

    library.lingtu_nav_client_start_task_with_receipt_v1 = _Function(invalid_receipt)
    session = native_abi.NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(native_abi.NativeCommandClientError, match="mismatch"):
        session.start_navigation_task("task-42", "goal-42", 1.0, 2.0, 0.0, 0.0)
    session.close()


@pytest.mark.parametrize("surface", ["state", "status"])
def test_native_session_rejects_invalid_v1_snapshot_metadata(
    tmp_path,
    surface,
) -> None:
    library = _Library()

    def invalid_snapshot(_handle, pointer):
        pointer._obj.abi_version = 99
        return 1

    if surface == "state":
        library.lingtu_nav_client_read_navigation_state_v1 = _Function(invalid_snapshot)
    else:
        library.lingtu_nav_client_take_navigation_goal_status_v1 = _Function(invalid_snapshot)
    session = native_abi.NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(native_abi.NativeCommandClientError, match="ABI version mismatch"):
        if surface == "state":
            session.read_navigation_state()
        else:
            session.take_navigation_goal_status()
    session.close()
