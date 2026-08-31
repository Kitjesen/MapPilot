from __future__ import annotations

import ctypes
import threading

import pytest

from nav.adapters.native import abi as native_abi
from nav.adapters.native.abi import (
    NativeCommandClientError,
    NativeCommandSession,
)
from nav.adapters.native.commands import (
    NativeNavigationClient,
    NavigationClientError,
)
from nav.adapters.native.operator_motion import NativeOperatorMotionClient
from runtime.msgs import (
    NavigationCommandKind,
    NavigationGoalState,
    NavigationGoalStatus,
    OperatorMotionAction,
    OperatorMotionReceipt,
)

_DEFAULT_CAPABILITIES = (
    native_abi.NATIVE_COMMAND_CAP_NAVIGATION
    | native_abi.NATIVE_COMMAND_CAP_INSPECTION
    | native_abi.NATIVE_COMMAND_CAP_EXPLORATION
    | native_abi.NATIVE_COMMAND_CAP_DIRECTED_EXPLORATION
    | native_abi.NATIVE_COMMAND_CAP_OPERATOR_MOTION
    | native_abi.NATIVE_COMMAND_CAP_OPERATOR_MOTION_RECEIPT
    | native_abi.NATIVE_COMMAND_CAP_NAVIGATION_COMMAND_RECEIPT
    | native_abi.NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS
    | native_abi.NATIVE_COMMAND_CAP_PLAN_PREVIEW
)


def test_navigation_task_enums_are_append_only_and_paused_is_not_terminal():
    assert [int(kind) for kind in NavigationCommandKind] == [1, 2, 4, 5, 6, 7, 8, 9]
    assert [int(state) for state in NavigationGoalState] == list(range(1, 7))

    terminal_states = {
        state
        for state in NavigationGoalState
        if NavigationGoalStatus(
            ts=1.0,
            frame_id="map",
            boot_id="boot-1",
            sequence=1,
            task_id="task-1",
            request_id="request-1",
            state=int(state),
            goal_epoch=1,
        ).terminal
    }

    assert terminal_states == {
        NavigationGoalState.FAILED,
        NavigationGoalState.REACHED,
        NavigationGoalState.CANCELLED,
    }
    assert NavigationGoalState.PAUSED not in terminal_states


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
        self.operator_motion_accepted = True
        self.operator_motion_reason = b"accepted"
        self.operator_motion_generated_request_id = b"native-generated-request"
        self.operator_motion_endpoint_timestamp_s = 123.5
        self.operator_motion_receipt_abi_version = native_abi.NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION
        self.operator_motion_receipt_struct_size = ctypes.sizeof(native_abi._NativeOperatorMotionReceiptV1)
        self.navigation_command_accepted = True
        self.navigation_command_reason = b"accepted"
        self.navigation_command_endpoint_timestamp_s = 124.5
        self.navigation_command_receipt_abi_version = native_abi.NATIVE_NAVIGATION_COMMAND_RECEIPT_ABI_VERSION
        self.navigation_command_receipt_struct_size = ctypes.sizeof(native_abi._NativeNavigationCommandReceiptV1)
        self.lingtu_nav_client_abi_version = _Function(lambda: native_abi.NATIVE_COMMAND_ABI_VERSION)
        self.lingtu_nav_client_capabilities = _Function(lambda: _DEFAULT_CAPABILITIES)
        self.lingtu_nav_client_create = _Function(self._create)
        self.lingtu_nav_client_destroy = _Function(self._destroy)
        self.lingtu_nav_client_start_task_with_receipt_v1 = _Function(self._start_task_with_receipt)
        self.lingtu_nav_client_cancel_task_with_receipt_v1 = _Function(self._cancel_task_with_receipt)
        self.lingtu_nav_client_pause_task_with_receipt_v1 = _Function(self._pause_task_with_receipt)
        self.lingtu_nav_client_resume_task_with_receipt_v1 = _Function(self._resume_task_with_receipt)
        self.lingtu_nav_client_stop = _Function(self._stop)
        self.lingtu_nav_client_stop_with_id = _Function(self._stop_with_id)
        self.lingtu_nav_client_estop = _Function(self._estop)
        self.lingtu_nav_client_estop_with_id = _Function(self._estop_with_id)
        self.lingtu_nav_client_clear_estop = _Function(self._clear_estop)
        self.lingtu_nav_client_clear_estop_with_id = _Function(self._clear_estop_with_id)
        self.lingtu_nav_client_resume_autonomy = _Function(self._resume_autonomy)
        self.lingtu_nav_client_resume_autonomy_with_id = _Function(self._resume_autonomy_with_id)
        self.lingtu_nav_client_resume_autonomy_with_receipt_v1 = _Function(
            self._resume_autonomy_with_receipt
        )
        self.lingtu_nav_client_preview_plan_v1 = _Function(self._preview_plan)
        self.lingtu_nav_client_operator_motion_claim = _Function(self._operator_motion_claim)
        self.lingtu_nav_client_operator_motion_claim_with_receipt_v1 = _Function(
            self._operator_motion_claim_with_receipt
        )
        self.lingtu_nav_client_operator_motion_hold_with_receipt_v1 = _Function(self._operator_motion_hold_with_receipt)
        self.lingtu_nav_client_operator_motion_release_with_receipt_v1 = _Function(
            self._operator_motion_release_with_receipt
        )
        self.lingtu_nav_client_operator_motion_sample = _Function(self._operator_motion_sample)
        self.lingtu_nav_client_operator_motion_sample_v2 = _Function(
            self._operator_motion_sample_v2
        )
        self.lingtu_nav_client_operator_motion_hold = _Function(self._operator_motion_hold)
        self.lingtu_nav_client_operator_motion_release = _Function(self._operator_motion_release)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: self.error)

    def _create(self, domain_id):
        self.calls.append(("create", domain_id))
        return 41

    def _destroy(self, handle):
        self.calls.append(("destroy", handle))

    def _navigation_command_receipt(
        self,
        pointer,
        *,
        task_id,
        request_id,
        kind,
        reason=None,
    ):
        receipt = pointer._obj
        receipt.abi_version = self.navigation_command_receipt_abi_version
        receipt.struct_size = self.navigation_command_receipt_struct_size
        receipt.task_id = task_id
        receipt.request_id = request_id
        receipt.accepted = 1 if self.navigation_command_accepted else 0
        receipt.kind = kind
        receipt.reason = reason or self.navigation_command_reason
        receipt.endpoint_timestamp_s = self.navigation_command_endpoint_timestamp_s

    def _start_task_with_receipt(self, handle, task_id, request_id, x, y, z, yaw, timeout_ms, receipt):
        self.calls.append(("start_task", handle, task_id, request_id, x, y, z, yaw, timeout_ms))
        self._navigation_command_receipt(
            receipt,
            task_id=task_id,
            request_id=request_id,
            kind=1,
        )
        return 0

    def _cancel_task_with_receipt(self, handle, task_id, request_id, reason, timeout_ms, receipt):
        self.calls.append(("cancel_task", handle, task_id, request_id, reason, timeout_ms))
        self._navigation_command_receipt(
            receipt,
            task_id=task_id,
            request_id=request_id,
            kind=2,
            reason=b"cancel_requested",
        )
        return 0
    def _pause_task_with_receipt(self, handle, task_id, request_id, reason, timeout_ms, receipt):
        self.calls.append(("pause_task", handle, task_id, request_id, reason, timeout_ms))
        self._navigation_command_receipt(
            receipt,
            task_id=task_id,
            request_id=request_id,
            kind=8,
            reason=b"pause_requested",
        )
        return 0

    def _resume_task_with_receipt(self, handle, task_id, request_id, reason, timeout_ms, receipt):
        self.calls.append(("resume_task", handle, task_id, request_id, reason, timeout_ms))
        self._navigation_command_receipt(
            receipt,
            task_id=task_id,
            request_id=request_id,
            kind=9,
            reason=b"resume_requested",
        )
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

    def _resume_autonomy_with_receipt(
        self,
        handle,
        request_id,
        reason,
        timeout_ms,
        receipt,
    ):
        self.calls.append(
            ("resume_autonomy_receipt", handle, request_id, reason, timeout_ms)
        )
        self._navigation_command_receipt(
            receipt,
            task_id=b"",
            request_id=request_id,
            kind=int(NavigationCommandKind.RESUME_AUTONOMY),
        )
        return 0

    def _preview_plan(
        self,
        handle,
        request_id,
        x,
        y,
        z,
        timeout_ms,
        result_pointer,
        points,
        point_capacity,
    ):
        self.calls.append(
            (
                "preview_plan",
                handle,
                request_id,
                x,
                y,
                z,
                timeout_ms,
                point_capacity,
            )
        )
        result = result_pointer._obj
        result.abi_version = native_abi.NATIVE_PLAN_RESULT_ABI_VERSION
        result.struct_size = ctypes.sizeof(native_abi._NativePlanResultV1)
        result.timestamp_s = 125.5
        result.frame_id = b"map"
        result.request_id = request_id
        result.feasible = 1
        result.start_valid = 1
        result.reason = b"planned"
        result.elapsed_ms = 4.25
        result.planner = b"astar"
        result.start.x = 0.5
        result.start.y = 1.0
        result.start.z = 0.0
        result.goal.x = x
        result.goal.y = y
        result.goal.z = z
        result.point_count = 2
        if points is None or point_capacity < 2:
            return 2
        points[0] = native_abi._NativePathPoint(0.5, 1.0, 0.0)
        points[1] = native_abi._NativePathPoint(x, y, z)
        return 1

    def _operator_motion_claim(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        lease_ttl_ms,
        timeout_ms,
    ):
        self.calls.append(
            (
                "operator_claim",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                lease_ttl_ms,
                timeout_ms,
            )
        )
        return 0

    def _operator_motion_sample(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        deadman,
        vx,
        vy,
        wz,
        freshness_budget_ms,
        timeout_ms,
    ):
        self.calls.append(
            (
                "operator_sample",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                deadman,
                vx,
                vy,
                wz,
                freshness_budget_ms,
                timeout_ms,
            )
        )
        return 0

    def _operator_motion_sample_v2(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        deadman,
        manual_mode,
        vx,
        vy,
        wz,
        freshness_budget_ms,
        timeout_ms,
    ):
        self.calls.append(
            (
                "operator_sample",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                deadman,
                manual_mode,
                vx,
                vy,
                wz,
                freshness_budget_ms,
                timeout_ms,
            )
        )
        return 0

    def _operator_motion_hold(self, handle, request_id, source_id, source_epoch, sequence, reason, timeout_ms):
        self.calls.append(("operator_hold", handle, request_id, source_id, source_epoch, sequence, reason, timeout_ms))
        return 0

    def _operator_motion_release(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        reason,
        timeout_ms,
    ):
        self.calls.append(
            (
                "operator_release",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                reason,
                timeout_ms,
            )
        )
        return 0

    def _populate_operator_motion_receipt(
        self,
        receipt_pointer,
        *,
        action,
        request_id,
        source_id,
        source_epoch,
        sequence,
    ):
        receipt = receipt_pointer._obj
        expected_size = ctypes.sizeof(native_abi._NativeOperatorMotionReceiptV1)
        assert receipt.abi_version == native_abi.NATIVE_OPERATOR_MOTION_RECEIPT_ABI_VERSION
        assert receipt.struct_size >= expected_size
        receipt.abi_version = self.operator_motion_receipt_abi_version
        receipt.struct_size = self.operator_motion_receipt_struct_size
        receipt.accepted = 1 if self.operator_motion_accepted else 0
        receipt.action = int(action)
        receipt.request_id = request_id or self.operator_motion_generated_request_id
        receipt.source_id = source_id
        receipt.source_epoch = source_epoch
        receipt.source_sequence = sequence
        receipt.accepted_sequence = sequence if self.operator_motion_accepted else 0
        receipt.final_output_sequence = (
            sequence + 100
            if self.operator_motion_accepted
            and int(action) in {int(OperatorMotionAction.HOLD), int(OperatorMotionAction.RELEASE)}
            else 0
        )
        receipt.endpoint_timestamp_s = self.operator_motion_endpoint_timestamp_s
        receipt.reason = self.operator_motion_reason

    def _operator_motion_claim_with_receipt(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        lease_ttl_ms,
        timeout_ms,
        receipt,
    ):
        self.calls.append(
            (
                "operator_claim_receipt",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                lease_ttl_ms,
                timeout_ms,
            )
        )
        self._populate_operator_motion_receipt(
            receipt,
            action=OperatorMotionAction.CLAIM,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
        )
        return 0

    def _operator_motion_hold_with_receipt(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        reason,
        timeout_ms,
        receipt,
    ):
        self.calls.append(
            (
                "operator_hold_receipt",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                reason,
                timeout_ms,
            )
        )
        self._populate_operator_motion_receipt(
            receipt,
            action=OperatorMotionAction.HOLD,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
        )
        return 0

    def _operator_motion_release_with_receipt(
        self,
        handle,
        request_id,
        source_id,
        source_epoch,
        sequence,
        reason,
        timeout_ms,
        receipt,
    ):
        self.calls.append(
            (
                "operator_release_receipt",
                handle,
                request_id,
                source_id,
                source_epoch,
                sequence,
                reason,
                timeout_ms,
            )
        )
        self._populate_operator_motion_receipt(
            receipt,
            action=OperatorMotionAction.RELEASE,
            request_id=request_id,
            source_id=source_id,
            source_epoch=source_epoch,
            sequence=sequence,
        )
        return 0


def test_native_client_reuses_one_cpp_handle_for_commands(tmp_path):
    library = _Library()
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=7,
        timeout_ms=250,
        library=library,
    )

    goal_receipt = client.start_task(
        1.0,
        2.0,
        0.3,
        0.4,
        task_id="task-001",
        request_id="goal-001",
    )
    cancel_receipt = client.cancel_task(
        "task-001",
        "operator_cancel",
        request_id="cancel-001",
    )
    client.stop("operator_stop", request_id="stop-001")
    client.estop("operator_estop", request_id="estop-001")
    client.clear_estop("operator_reset", request_id="clear-estop-001")
    client.resume_autonomy("operator_resume", request_id="resume-001")
    client.close()

    assert goal_receipt.task_id == "task-001"
    assert goal_receipt.request_id == "goal-001"
    assert cancel_receipt.task_id == "task-001"
    assert cancel_receipt.request_id == "cancel-001"

    assert library.calls == [
        ("create", 7),
        ("start_task", 41, b"task-001", b"goal-001", 1.0, 2.0, 0.3, 0.4, 250),
        ("cancel_task", 41, b"task-001", b"cancel-001", b"operator_cancel", 250),
        ("stop", 41, b"stop-001", b"operator_stop", 250),
        ("estop", 41, b"estop-001", b"operator_estop", 250),
        ("clear_estop", 41, b"clear-estop-001", b"operator_reset", 250),
        ("resume_autonomy", 41, b"resume-001", b"operator_resume", 250),
        ("destroy", 41),

    ]


def test_native_client_previews_plan_without_creating_a_task(tmp_path, monkeypatch):
    library = _Library()
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        timeout_ms=250,
        library=library,
    )
    monkeypatch.setattr("nav.adapters.native.commands.os.getpid", lambda: 23)
    monkeypatch.setattr("nav.adapters.native.commands.time.time_ns", lambda: 456)

    preview = client.preview_plan(3.0, 4.0, 0.2)
    client.close()

    assert preview == {
        "timestamp_s": 125.5,
        "frame_id": "map",
        "request_id": "plan-23-456",
        "feasible": True,
        "start_valid": True,
        "reason": "planned",
        "elapsed_ms": 4.25,
        "planner": "astar",
        "start": {"x": 0.5, "y": 1.0, "z": 0.0},
        "goal": {"x": 3.0, "y": 4.0, "z": 0.2},
        "point_count": 2,
        "path": [
            {"x": 0.5, "y": 1.0, "z": 0.0},
            {"x": 3.0, "y": 4.0, "z": 0.2},
        ],
    }
    assert library.calls == [
        ("create", 0),
        ("preview_plan", 41, b"plan-23-456", 3.0, 4.0, 0.2, 250, 0),
        ("preview_plan", 41, b"plan-23-456", 3.0, 4.0, 0.2, 250, 2),
        ("destroy", 41),
    ]


def test_native_client_resume_autonomy_returns_rejected_endpoint_receipt(tmp_path):
    library = _Library()
    library.navigation_command_accepted = False
    library.navigation_command_reason = b"manual_takeover_still_active"
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        timeout_ms=250,
        library=library,
    )

    receipt = client.resume_autonomy_with_receipt(
        "operator_resume",
        request_id="resume-rejected-001",
    )
    client.close()

    assert receipt == {
        "accepted": False,
        "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
        "task_id": "",
        "request_id": "resume-rejected-001",
        "endpoint_timestamp_s": 124.5,
        "reason": "manual_takeover_still_active",
    }
    assert library.calls == [
        ("create", 0),
        (
            "resume_autonomy_receipt",
            41,
            b"resume-rejected-001",
            b"operator_resume",
            250,
        ),
        ("destroy", 41),
    ]


def test_native_client_legacy_resume_autonomy_does_not_require_receipt_symbol(tmp_path):
    library = _Library()
    del library.lingtu_nav_client_resume_autonomy_with_receipt_v1
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        timeout_ms=250,
        library=library,
    )

    assert client.resume_autonomy(
        "operator_resume",
        request_id="legacy-resume-001",
    ) is None
    with pytest.raises(
        NativeCommandClientError,
        match="resume autonomy receipt ABI is unavailable",
    ):
        client.resume_autonomy_with_receipt(
            "operator_resume",
            request_id="receipt-unavailable",
        )
    client.close()

    assert library.calls == [
        ("create", 0),
        (
            "resume_autonomy",
            41,
            b"legacy-resume-001",
            b"operator_resume",
            250,
        ),
        ("destroy", 41),
    ]


def test_native_client_pauses_and_resumes_the_same_task_with_typed_receipts(tmp_path):
    library = _Library()
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        timeout_ms=250,
        library=library,
    )

    pause_receipt = client.pause_task(
        "task-001",
        "operator_pause",
        request_id="pause-001",
    )
    resume_receipt = client.resume_task(
        "task-001",
        "operator_resume",
        request_id="resume-001",
    )
    client.close()

    assert pause_receipt.task_id == resume_receipt.task_id == "task-001"
    assert pause_receipt.request_id == "pause-001"
    assert resume_receipt.request_id == "resume-001"
    assert pause_receipt.kind == 8
    assert resume_receipt.kind == 9
    assert library.calls == [
        ("create", 0),
        ("pause_task", 41, b"task-001", b"pause-001", b"operator_pause", 250),
        ("resume_task", 41, b"task-001", b"resume-001", b"operator_resume", 250),
        ("destroy", 41),
    ]

def test_native_client_surfaces_cpp_delivery_error(tmp_path):
    library = _Library()
    library.error = b"no matched DDS reader for goal_pose"
    library.lingtu_nav_client_start_task_with_receipt_v1 = _Function(lambda *_args: -1)
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        client.start_task(1.0, 2.0, 0.0, 0.0, task_id="task-error", request_id="goal-error")
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
    library.lingtu_nav_client_capabilities = _Function(lambda: 0x1E)

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


@pytest.mark.parametrize(
    ("method", "symbol"),
    [
        (
            "pause_navigation_task",
            "lingtu_nav_client_pause_task_with_receipt_v1",
        ),
        (
            "resume_navigation_task",
            "lingtu_nav_client_resume_task_with_receipt_v1",
        ),
    ],
)
def test_native_task_lifecycle_reports_incomplete_abi_before_symbol_lookup(
    tmp_path,
    method,
    symbol,
):
    library = _Library()
    delattr(library, symbol)
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        with pytest.raises(
            NativeCommandClientError,
            match="native navigation command ABI is incomplete",
        ):
            getattr(session, method)(
                "task-1",
                f"{method}-request-1",
                "operator_request",
            )
    finally:
        session.close()


def test_native_session_reads_and_looks_up_goal_status(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: 1 << 6)
    take_count = 0

    def populate(pointer, *, sequence):
        status = pointer._obj
        status.timestamp_s = 42.5
        status.frame_id = b"map"
        status.boot_id = b"navd-boot"
        status.sequence = sequence
        status.task_id = b"navigation-task-42"
        status.request_id = b"goal-42"
        status.state = 4
        status.goal_epoch = 9
        status.reason = b"goal_reached"

    def take(_handle, pointer):
        nonlocal take_count
        if take_count:
            return 0
        take_count += 1
        populate(pointer, sequence=11)
        return 1

    def get(_handle, request_id, pointer):
        assert request_id == b"goal-42"
        populate(pointer, sequence=11)
        return 1

    library.lingtu_nav_client_take_navigation_goal_status = _Function(take)
    library.lingtu_nav_client_get_navigation_goal_status = _Function(get)
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    event = session.take_navigation_goal_status()
    assert event == {
        "timestamp_s": 42.5,
        "frame_id": "map",
        "boot_id": "navd-boot",
        "sequence": 11,
        "task_id": "navigation-task-42",
        "request_id": "goal-42",
        "state": 4,
        "goal_epoch": 9,
        "reason": "goal_reached",
    }
    assert session.take_navigation_goal_status() is None
    assert session.get_navigation_goal_status("goal-42") == event
    session.close()


def test_native_session_reads_ordered_inspection_task_event(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(
        lambda: native_abi.NATIVE_COMMAND_CAP_INSPECTION_TASK_EVENTS
    )
    taken = False

    def take(_handle, pointer):
        nonlocal taken
        event = pointer._obj
        assert event.abi_version == native_abi.NATIVE_INSPECTION_TASK_EVENT_ABI_VERSION
        assert event.struct_size == ctypes.sizeof(native_abi._NativeInspectionTaskEventV1)
        if taken:
            return 0
        taken = True
        event.abi_version = native_abi.NATIVE_INSPECTION_TASK_EVENT_ABI_VERSION
        event.struct_size = ctypes.sizeof(native_abi._NativeInspectionTaskEventV1)
        event.timestamp_s = 42.5
        event.frame_id = b"map"
        event.boot_id = b"navd-boot"
        event.event_sequence = 11
        event.kind = 2
        event.task_id = b"inspection-task-42"
        event.request_id = b"start-request-42"
        event.command_request_id = b"pause-request-43"
        event.state = 12
        event.map_id = b"field-map"
        event.map_content_epoch = 7
        event.route_id = b"route-a"
        event.route_revision = 3
        event.point_index = 1
        event.point_count = 4
        event.loop_index = 2
        event.retry_count = 1
        event.point_id = b"checkpoint-b"
        event.action = b"capture:overview"
        event.action_request_id = b"evidence-request-44"
        event.evidence_id = b"evidence-45"
        event.reason = b"operator_pause_waiting_for_stop"
        return 1

    library.lingtu_nav_client_take_inspection_task_event_v1 = _Function(take)
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    assert session.take_inspection_task_event() == {
        "timestamp_s": 42.5,
        "frame_id": "map",
        "boot_id": "navd-boot",
        "event_sequence": 11,
        "kind": 2,
        "task_id": "inspection-task-42",
        "request_id": "start-request-42",
        "command_request_id": "pause-request-43",
        "state": 12,
        "map_id": "field-map",
        "map_content_epoch": 7,
        "route_id": "route-a",
        "route_revision": 3,
        "point_index": 1,
        "point_count": 4,
        "loop_index": 2,
        "retry_count": 1,
        "point_id": "checkpoint-b",
        "action": "capture:overview",
        "action_request_id": "evidence-request-44",
        "evidence_id": "evidence-45",
        "reason": "operator_pause_waiting_for_stop",
    }
    assert session.take_inspection_task_event() is None
    session.close()


def test_native_session_rejects_missing_inspection_task_event_symbol(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(
        lambda: native_abi.NATIVE_COMMAND_CAP_INSPECTION_TASK_EVENTS
    )
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        with pytest.raises(
            NativeCommandClientError,
            match="native inspection task event ABI is incomplete",
        ):
            session.take_inspection_task_event()
    finally:
        session.close()


def test_native_session_looks_up_goal_status_by_task_identity(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(
        lambda: (
            native_abi.NATIVE_COMMAND_CAP_GOAL_STATUS
            | native_abi.NATIVE_COMMAND_CAP_NAVIGATION_TASK_STATUS
        )
    )

    def no_event(*_args):
        return 0

    def get_task(_handle, task_id, pointer):
        assert task_id == b"navigation-task-42"
        status = pointer._obj
        assert (
            status.abi_version
            == native_abi.NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION
        )
        assert status.struct_size == ctypes.sizeof(
            native_abi._NativeNavigationGoalStatusV1
        )
        status.abi_version = native_abi.NATIVE_NAVIGATION_GOAL_STATUS_ABI_VERSION
        status.struct_size = ctypes.sizeof(native_abi._NativeNavigationGoalStatusV1)
        status.timestamp_s = 42.5
        status.frame_id = b"map"
        status.boot_id = b"navd-boot"
        status.sequence = 12
        status.task_id = b"navigation-task-42"
        status.request_id = b"goal-42"
        status.state = 4
        status.goal_epoch = 9
        status.reason = b"goal_reached"
        return 1

    library.lingtu_nav_client_take_navigation_goal_status = _Function(no_event)
    library.lingtu_nav_client_get_navigation_goal_status = _Function(no_event)
    library.lingtu_nav_client_get_navigation_task_status_v1 = _Function(get_task)
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    assert session.get_navigation_task_status("navigation-task-42") == {
        "timestamp_s": 42.5,
        "frame_id": "map",
        "boot_id": "navd-boot",
        "sequence": 12,
        "task_id": "navigation-task-42",
        "request_id": "goal-42",
        "state": 4,
        "goal_epoch": 9,
        "reason": "goal_reached",
    }
    session.close()


def test_native_session_reads_complete_variable_length_paths(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: 1 << 7)

    def path_reader(z_offset):
        consumed = False

        def read(_handle, header_pointer, points, capacity):
            nonlocal consumed
            if consumed:
                return 0
            header = header_pointer._obj
            header.timestamp_s = 52.5
            header.frame_id = b"map"
            header.receive_sequence = 7
            header.point_count = 2
            if int(capacity) < 2:
                return 2
            points[0].x, points[0].y, points[0].z = 1.0, 2.0, z_offset
            points[1].x, points[1].y, points[1].z = 3.0, 4.0, z_offset
            consumed = True
            return 1

        return read

    library.lingtu_nav_client_take_global_path = _Function(path_reader(0.1))
    library.lingtu_nav_client_take_local_path = _Function(path_reader(0.0))
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    global_path = session.take_global_path()
    local_path = session.take_local_path()

    assert global_path == {
        "timestamp_s": 52.5,
        "frame_id": "map",
        "receive_sequence": 7,
        "points": [
            {"x": 1.0, "y": 2.0, "z": 0.1},
            {"x": 3.0, "y": 4.0, "z": 0.1},
        ],
    }
    assert local_path is not None
    assert local_path["points"][1] == {"x": 3.0, "y": 4.0, "z": 0.0}
    assert session.take_global_path() is None
    session.close()


def test_native_session_reads_bounded_map_scene_and_health(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: native_abi.NATIVE_COMMAND_CAP_MAP_SCENE)
    take_calls = 0

    def populate_header(header):
        header.abi_version = native_abi.NATIVE_MAP_SCENE_ABI_VERSION
        header.struct_size = ctypes.sizeof(native_abi._NativeMapSceneHeaderV1)
        header.timestamp_s = 52.5
        header.frame_id = b"map"
        header.producer_boot_id = b"mapd-boot"
        header.receive_sequence = 7
        header.reset_epoch = 3
        header.observation_sequence = 11
        header.generation = 5
        header.live = 1
        header.sensor_qw = 1.0
        header.live_point_count = 1
        header.voxel_point_count = 1
        header.accumulated_point_count = 0
        header.occupancy.width = 2
        header.occupancy.height = 1
        header.occupancy.resolution = 0.1
        header.occupancy.origin_qw = 1.0
        header.occupancy.cell_count = 2
        for name in ("elevation", "esdf"):
            grid = getattr(header, name)
            grid.resolution = 0.1
            grid.origin_qw = 1.0
        header.payload_bytes = 2 * 16 + 2 * 4

    def take(_handle, header_pointer, buffers_pointer):
        nonlocal take_calls
        take_calls += 1
        header = header_pointer._obj
        populate_header(header)
        if buffers_pointer is None:
            return 2
        buffers = buffers_pointer._obj
        assert buffers.live_point_capacity == 1
        assert buffers.voxel_point_capacity == 1
        assert buffers.occupancy_cell_capacity == 2
        buffers.live_points[0].x = 1.0
        buffers.live_points[0].y = 2.0
        buffers.live_points[0].z = 3.0
        buffers.live_points[0].intensity = 4.0
        buffers.voxel_points[0].x = 5.0
        buffers.voxel_points[0].y = 6.0
        buffers.voxel_points[0].z = 7.0
        buffers.voxel_points[0].intensity = 8.0
        buffers.occupancy_cells[0] = 0.25
        buffers.occupancy_cells[1] = 0.75
        return 1

    def health(_handle, health_pointer):
        value = health_pointer._obj
        value.abi_version = native_abi.NATIVE_MAP_SCENE_ABI_VERSION
        value.struct_size = ctypes.sizeof(native_abi._NativeMapSceneHealthV1)
        value.received_samples = 2
        value.valid_samples = 1
        value.consumer_buffer_retries = 1
        value.last_receive_sequence = 7
        value.last_generation = 5
        value.last_sample_timestamp_s = 52.5
        value.state_received_samples = 1
        value.state_valid_samples = 1
        value.state_timestamp_s = 52.5
        value.state_producer_boot_id = b"mapd-boot"
        value.state_received = 1
        value.state_running = 1
        value.state_live = 1
        value.state_required_publications_ready = 1
        value.state_current_generation_published = 1
        value.state_reset_epoch = 3
        value.state_observation_sequence = 11
        value.state_generation = 5
        value.state_scene_published_generation = 5
        return 0

    library.lingtu_nav_client_take_map_scene_v1 = _Function(take)
    library.lingtu_nav_client_read_map_scene_health_v1 = _Function(health)
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    scene = session.take_map_scene()
    assert scene is not None
    assert take_calls == 2
    assert scene["producer_boot_id"] == "mapd-boot"
    assert scene["generation"] == 5
    assert scene["clouds"]["live"]["point_count"] == 1
    assert len(scene["clouds"]["live"]["points_xyzi_f32"]) == 16
    assert scene["grids"]["occupancy"]["cell_count"] == 2
    assert len(scene["grids"]["occupancy"]["values_f32"]) == 8

    map_health = session.read_map_scene_health()
    assert map_health["consumer_buffer_retries"] == 1
    assert map_health["state_received"] is True
    assert map_health["state_scene_published_generation"] == 5
    session.close()


def test_native_session_reads_native_traversability_grid(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(
        lambda: native_abi.NATIVE_COMMAND_CAP_TRAVERSABILITY_GRID
    )
    take_calls = 0

    def take(_handle, header_pointer, cells, capacity):
        nonlocal take_calls
        take_calls += 1
        header = header_pointer._obj
        header.abi_version = native_abi.NATIVE_TRAVERSABILITY_GRID_ABI_VERSION
        header.struct_size = ctypes.sizeof(native_abi._NativeTraversabilityGridHeaderV1)
        header.timestamp_s = 52.5
        header.frame_id = b"map"
        header.receive_sequence = 7
        header.reset_epoch = 2
        header.width = 2
        header.height = 2
        header.resolution = 0.2
        header.origin_x = 1.0
        header.origin_y = -2.0
        header.origin_z = 0.0
        header.yaw = 0.0
        header.cell_count = 4
        if cells is None or int(capacity) < 4:
            return 2
        for index, value in enumerate((0, 25, 80, 100)):
            cells[index] = value
        return 1

    library.lingtu_nav_client_take_traversability_grid_v1 = _Function(take)
    session = NativeCommandSession(tmp_path / "liblingtu_nav_client.so", library=library)

    grid = session.take_traversability_grid()
    assert grid is not None
    assert take_calls == 2
    assert grid["frame_id"] == "map"
    assert grid["reset_epoch"] == 2
    assert grid["width"] == 2
    assert grid["height"] == 2
    assert grid["origin"] == {"x": 1.0, "y": -2.0, "z": 0.0}
    assert grid["cells_u8"] == bytes((0, 25, 80, 100))
    session.close()


def test_native_session_rejects_oversized_map_scene_before_allocation(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: native_abi.NATIVE_COMMAND_CAP_MAP_SCENE)
    take_calls = 0

    def take(_handle, header_pointer, _buffers_pointer):
        nonlocal take_calls
        take_calls += 1
        header = header_pointer._obj
        header.abi_version = native_abi.NATIVE_MAP_SCENE_ABI_VERSION
        header.struct_size = ctypes.sizeof(native_abi._NativeMapSceneHeaderV1)
        header.timestamp_s = 52.5
        header.frame_id = b"map"
        header.producer_boot_id = b"mapd-boot"
        header.receive_sequence = 1
        header.observation_sequence = 1
        header.generation = 1
        header.live = 1
        header.sensor_qw = 1.0
        header.live_point_count = native_abi.NATIVE_MAP_SCENE_MAX_POINTS_PER_LAYER + 1
        header.payload_bytes = int(header.live_point_count) * 16
        return 2

    library.lingtu_nav_client_take_map_scene_v1 = _Function(take)
    library.lingtu_nav_client_read_map_scene_health_v1 = _Function(lambda *_args: 0)
    session = NativeCommandSession(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(
        NativeCommandClientError,
        match="point count exceeds Python product limit",
    ):
        session.take_map_scene()
    assert take_calls == 1
    session.close()


def test_native_operator_motion_client_uses_typed_c_abi(tmp_path):
    library = _Library()
    client = NativeOperatorMotionClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=7,
        timeout_ms=250,
        operator_motion_timeout_ms=600,
        library=library,
    )

    claim_receipt = client.claim("ws:operator-a", 42, 1, lease_ttl_ms=1000, request_id="claim-1")
    assert (
        client.sample(
            "ws:operator-a",
            42,
            2,
            0.2,
            -0.1,
            0.5,
            deadman=True,
            manual_mode=True,
            freshness_budget_ms=350,
            request_id="sample-1",
        )
        is True
    )
    hold_receipt = client.hold("ws:operator-a", 42, 3, reason="manual_hold", request_id="hold-1")
    release_receipt = client.release("ws:operator-a", 42, 4, reason="disconnect", request_id="release-1")
    assert isinstance(claim_receipt, OperatorMotionReceipt)
    assert claim_receipt.action == OperatorMotionAction.CLAIM
    assert claim_receipt.request_id == "claim-1"
    assert claim_receipt.source_id == "ws:operator-a"
    assert claim_receipt.source_epoch == 42
    assert claim_receipt.source_sequence == 1
    assert claim_receipt.accepted_sequence == 1
    assert claim_receipt.final_output_sequence == 0
    assert claim_receipt.endpoint_timestamp_s == 123.5
    assert hold_receipt.final_output_sequence == 103
    assert release_receipt.final_output_sequence == 104

    client.close()

    assert library.calls == [
        ("create", 7),
        ("operator_claim_receipt", 41, b"claim-1", b"ws:operator-a", 42, 1, 1000, 600),
        (
            "operator_sample",
            41,
            b"sample-1",
            b"ws:operator-a",
            42,
            2,
            1,
            1,
            0.2,
            -0.1,
            0.5,
            350,
            600,
        ),
        ("operator_hold_receipt", 41, b"hold-1", b"ws:operator-a", 42, 3, b"manual_hold", 600),
        (
            "operator_release_receipt",
            41,
            b"release-1",
            b"ws:operator-a",
            42,
            4,
            b"disconnect",
            600,
        ),
        ("destroy", 41),
    ]


def test_native_operator_motion_returns_rejected_business_receipt(tmp_path):
    library = _Library()
    library.operator_motion_accepted = False
    library.operator_motion_reason = b"source_busy"
    client = NativeOperatorMotionClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    receipt = client.claim(
        "ws:operator-b",
        7,
        9,
        lease_ttl_ms=500,
        request_id="claim-rejected",
    )
    client.close()

    assert isinstance(receipt, OperatorMotionReceipt)
    assert receipt.accepted is False
    assert receipt.request_id == "claim-rejected"
    assert receipt.source_id == "ws:operator-b"
    assert receipt.source_epoch == 7
    assert receipt.source_sequence == 9
    assert receipt.accepted_sequence == 0
    assert receipt.final_output_sequence == 0
    assert receipt.reason == "source_busy"


def test_native_operator_motion_preserves_endpoint_generated_request_id(tmp_path):
    library = _Library()
    client = NativeOperatorMotionClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    receipt = client.hold("ws:operator-a", 4, 5, reason="disconnect")
    client.close()

    assert receipt.request_id == "native-generated-request"
    assert receipt.action == OperatorMotionAction.HOLD
    assert receipt.final_output_sequence == 105


def test_native_operator_motion_requires_receipt_capability(tmp_path):
    library = _Library()
    library.lingtu_nav_client_capabilities = _Function(lambda: native_abi.NATIVE_COMMAND_CAP_OPERATOR_MOTION)

    with pytest.raises(
        NativeCommandClientError,
        match="operator motion receipt capability",
    ):
        NativeOperatorMotionClient(
            tmp_path / "liblingtu_nav_client.so",
            library=library,
        )

    assert library.calls == [("create", 0), ("destroy", 41)]


@pytest.mark.parametrize(
    ("attribute", "value", "error_pattern"),
    [
        ("operator_motion_receipt_abi_version", 2, "ABI version mismatch"),
        ("operator_motion_receipt_struct_size", 0, "struct size mismatch"),
    ],
)
def test_native_operator_motion_rejects_invalid_receipt_metadata(
    tmp_path,
    attribute,
    value,
    error_pattern,
):
    library = _Library()
    setattr(library, attribute, value)
    client = NativeOperatorMotionClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    try:
        with pytest.raises(NativeCommandClientError, match=error_pattern):
            client.claim(
                "ws:operator-a",
                1,
                1,
                lease_ttl_ms=500,
                request_id="claim-invalid-metadata",
            )
    finally:
        client.close()


def test_python_session_does_not_serialize_estop_behind_goal(tmp_path):
    library = _Library()
    goal_started = threading.Event()
    release_goal = threading.Event()
    estop_seen = threading.Event()

    def blocked_goal(handle, task_id, request_id, x, y, z, yaw, timeout_ms, receipt):
        del handle, x, y, z, yaw, timeout_ms
        goal_started.set()
        release_goal.wait(timeout=1.0)
        library._navigation_command_receipt(
            receipt,
            task_id=task_id,
            request_id=request_id,
            kind=1,
        )
        return 0

    def immediate_estop(*_args):
        estop_seen.set()
        return 0

    library.lingtu_nav_client_start_task_with_receipt_v1 = _Function(blocked_goal)
    library.lingtu_nav_client_estop_with_id = _Function(immediate_estop)
    client = NativeNavigationClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )
    goal_thread = threading.Thread(
        target=lambda: client.start_task(
            1.0,
            2.0,
            0.0,
            0.0,
            task_id="task-blocked",
            request_id="goal-blocked",
        ),
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


@pytest.mark.parametrize("deadman", ["false", "0", 1, None])
def test_native_operator_motion_rejects_non_boolean_deadman(tmp_path, deadman) -> None:
    library = _Library()
    client = NativeOperatorMotionClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(TypeError, match="deadman must be a boolean"):
        client.sample(
            "ws:operator-a",
            42,
            1,
            0.2,
            0.0,
            0.1,
            deadman=deadman,
        )
    client.close()


@pytest.mark.parametrize("manual_mode", ["true", "0", 1, None])
def test_native_operator_motion_rejects_non_boolean_manual_mode(tmp_path, manual_mode) -> None:
    library = _Library()
    client = NativeOperatorMotionClient(
        tmp_path / "liblingtu_nav_client.so",
        library=library,
    )

    with pytest.raises(TypeError, match="manual_mode must be a boolean"):
        client.sample(
            "ws:operator-a",
            42,
            1,
            0.2,
            0.0,
            0.1,
            manual_mode=manual_mode,
        )
    client.close()
