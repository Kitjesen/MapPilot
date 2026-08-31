from __future__ import annotations

import pytest

from nav.commands import module as command_module
from runtime import Module
from runtime.msgs import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    OperatorMotionAction,
    OperatorMotionReceipt,
)

_EXPLORATION_RUN_ID = "01ARZ3NDEKTSV4RRFFQ69G5FAV"


class _NavigationClient:
    def __init__(self) -> None:
        self.calls = []

    def start_task(self, x, y, z, yaw, *, task_id=None, request_id=None):
        self.calls.append(("goal", x, y, z, yaw, task_id, request_id))
        return NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.GOAL),
            task_id=task_id,
            request_id=request_id,
            endpoint_timestamp_s=123.5,
            reason="accepted",
        )

    def preview_plan(self, x, y, z):
        self.calls.append(("preview_plan", x, y, z))
        return {
            "feasible": True,
            "start_valid": True,
            "reason": "planned",
            "path": [{"x": x, "y": y, "z": z}],
        }

    def cancel_task(self, task_id, reason, *, request_id=None):
        self.calls.append(("cancel_task", task_id, reason, request_id))
        return NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.TASK_CANCEL),
            task_id=task_id,
            request_id=request_id,
            endpoint_timestamp_s=124.0,
            reason="cancel_requested",
        )

    def pause_task(self, task_id, reason, *, request_id=None):
        self.calls.append(("pause_task", task_id, reason, request_id))
        return NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.PAUSE_TASK),
            task_id=task_id,
            request_id=request_id,
            endpoint_timestamp_s=125.0,
            reason="pause_requested",
        )

    def resume_task(self, task_id, reason, *, request_id=None):
        self.calls.append(("resume_task", task_id, reason, request_id))
        return NavigationCommandReceipt(
            accepted=True,
            kind=int(NavigationCommandKind.RESUME_TASK),
            task_id=task_id,
            request_id=request_id,
            endpoint_timestamp_s=126.0,
            reason="resume_requested",
        )

    def stop(self, reason, *, request_id=None):
        self.calls.append(("stop", reason, request_id))

    def resume_autonomy(self, reason, *, request_id=None):
        self.calls.append(("resume_autonomy", reason, request_id))

    def resume_autonomy_with_receipt(self, reason, *, request_id=None):
        self.calls.append(("resume_autonomy_with_receipt", reason, request_id))
        return {
            "accepted": False,
            "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
            "task_id": "",
            "request_id": request_id,
            "endpoint_timestamp_s": 127.5,
            "reason": "manual_takeover_still_active",
        }


class _ExplorationClient:
    def __init__(self) -> None:
        self.calls = []

    @staticmethod
    def _receipt(exploration_run_id, request_id):
        return {
            "accepted": True,
            "request_id": request_id,
            "exploration_run_id": exploration_run_id,
            "reason": "admitted",
            "duplicate": False,
        }

    def start(
        self,
        exploration_run_id,
        product_session_id,
        *,
        reason="operator_start",
        request_id=None,
    ):
        self.calls.append(("start", exploration_run_id, product_session_id, reason, request_id))
        return self._receipt(exploration_run_id, request_id)

    def pause(
        self,
        exploration_run_id,
        product_session_id,
        reason="operator_pause",
        *,
        request_id=None,
    ):
        self.calls.append(("pause", exploration_run_id, product_session_id, reason, request_id))
        return self._receipt(exploration_run_id, request_id)

    def resume(
        self,
        exploration_run_id,
        product_session_id,
        reason="operator_resume",
        *,
        request_id=None,
    ):
        self.calls.append(("resume", exploration_run_id, product_session_id, reason, request_id))
        return self._receipt(exploration_run_id, request_id)

    def stop(
        self,
        exploration_run_id,
        product_session_id,
        reason="operator_stop",
        *,
        request_id=None,
    ):
        self.calls.append(("stop", exploration_run_id, product_session_id, reason, request_id))
        return self._receipt(exploration_run_id, request_id)

    def set_directed_target(
        self,
        x,
        y,
        ttl_s,
        *,
        exploration_run_id,
        product_session_id,
        reason="operator_directed_explore",
        request_id=None,
    ):
        self.calls.append(
            (
                "set",
                x,
                y,
                ttl_s,
                exploration_run_id,
                product_session_id,
                reason,
                request_id,
            )
        )
        return self._receipt(exploration_run_id, request_id)

    def clear_directed_target(
        self,
        exploration_run_id,
        product_session_id,
        reason="operator_clear_directed_explore",
        *,
        request_id=None,
    ):
        self.calls.append(("clear", exploration_run_id, product_session_id, reason, request_id))
        return self._receipt(exploration_run_id, request_id)


class _InspectionTaskClient:
    def __init__(self) -> None:
        self.calls = []

    def start(self, task_id, route_id, *, revision=0, request_id=None):
        self.calls.append(("start", task_id, route_id, revision, request_id))

    def pause(self, task_id, reason, *, request_id=None):
        self.calls.append(("pause", task_id, reason, request_id))

    def resume(self, task_id, reason, *, request_id=None):
        self.calls.append(("resume", task_id, reason, request_id))

    def cancel(self, task_id, reason, *, request_id=None):
        self.calls.append(("cancel", task_id, reason, request_id))


_DEFAULT_OPERATOR_RECEIPT = object()


class _OperatorMotionClient:
    def __init__(self, ack=_DEFAULT_OPERATOR_RECEIPT, *, sample_ack=True) -> None:
        self.calls = []
        self.ack = ack
        self.sample_ack = sample_ack

    def _control_result(
        self,
        action,
        source_id,
        source_epoch,
        sequence,
        request_id,
    ):
        if self.ack is not _DEFAULT_OPERATOR_RECEIPT:
            return self.ack
        return OperatorMotionReceipt(
            accepted=True,
            action=int(action),
            request_id=str(request_id or f"generated-{int(action)}-{sequence}"),
            source_id=source_id,
            source_epoch=source_epoch,
            source_sequence=sequence,
            accepted_sequence=sequence,
            final_output_sequence=(
                sequence + 100 if action in {OperatorMotionAction.HOLD, OperatorMotionAction.RELEASE} else 0
            ),
            endpoint_timestamp_s=123.5,
            reason="accepted",
        )

    def claim(
        self,
        source_id,
        source_epoch,
        sequence,
        *,
        lease_ttl_ms,
        request_id=None,
    ):
        self.calls.append(("claim", source_id, source_epoch, sequence, lease_ttl_ms, request_id))
        return self._control_result(
            OperatorMotionAction.CLAIM,
            source_id,
            source_epoch,
            sequence,
            request_id,
        )

    def sample(
        self,
        source_id,
        source_epoch,
        sequence,
        vx,
        vy,
        wz,
        *,
        deadman=True,
        manual_mode=False,
        freshness_budget_ms=350,
        request_id=None,
    ):
        self.calls.append(
            (
                "sample",
                source_id,
                source_epoch,
                sequence,
                vx,
                vy,
                wz,
                deadman,
                manual_mode,
                freshness_budget_ms,
                request_id,
            )
        )
        return self.sample_ack

    def hold(
        self,
        source_id,
        source_epoch,
        sequence,
        *,
        reason="operator_hold",
        request_id=None,
    ):
        self.calls.append(("hold", source_id, source_epoch, sequence, reason, request_id))
        return self._control_result(
            OperatorMotionAction.HOLD,
            source_id,
            source_epoch,
            sequence,
            request_id,
        )

    def release(
        self,
        source_id,
        source_epoch,
        sequence,
        *,
        reason="operator_release",
        request_id=None,
    ):
        self.calls.append(("release", source_id, source_epoch, sequence, reason, request_id))
        return self._control_result(
            OperatorMotionAction.RELEASE,
            source_id,
            source_epoch,
            sequence,
            request_id,
        )


def test_command_rpc_names_do_not_override_module_lifecycle(monkeypatch) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    assert command_module.Commands.stop is Module.stop
    commands.start()
    assert commands.running is True
    commands.stop()
    assert commands.running is False
    assert navigation.calls == []


def test_commands_forward_typed_navigation_requests(monkeypatch) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    goal_receipt = commands.send_goal(
        1,
        2,
        0.3,
        0.4,
        task_id="task-1",
        request_id="goal-1",
    )
    cancel_receipt = commands.cancel_task(
        "task-1",
        "operator",
        request_id="cancel-1",
    )
    assert commands.stop_motion("operator", request_id="stop-1") is True

    assert goal_receipt.task_id == "task-1"
    assert goal_receipt.request_id == "goal-1"
    assert goal_receipt.kind == NavigationCommandKind.GOAL
    assert cancel_receipt.task_id == "task-1"
    assert cancel_receipt.request_id == "cancel-1"
    assert cancel_receipt.kind == NavigationCommandKind.TASK_CANCEL

    assert navigation.calls == [
        ("goal", 1.0, 2.0, 0.3, 0.4, "task-1", "goal-1"),
        ("cancel_task", "task-1", "operator", "cancel-1"),
        ("stop", "operator", "stop-1"),
    ]


def test_commands_expose_native_plan_preview(monkeypatch) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )

    result = command_module.Commands().preview_plan(1, 2, 0.3)

    assert result == {
        "feasible": True,
        "start_valid": True,
        "reason": "planned",
        "path": [{"x": 1.0, "y": 2.0, "z": 0.3}],
    }
    assert navigation.calls == [("preview_plan", 1.0, 2.0, 0.3)]


def test_commands_resume_autonomy_returns_plain_native_receipt(monkeypatch) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    receipt = commands.resume_autonomy_with_receipt(
        "operator_resume",
        request_id="resume-1",
    )

    assert receipt == {
        "accepted": False,
        "kind": int(NavigationCommandKind.RESUME_AUTONOMY),
        "task_id": "",
        "request_id": "resume-1",
        "endpoint_timestamp_s": 127.5,
        "reason": "manual_takeover_still_active",
    }
    assert navigation.calls == [
        ("resume_autonomy_with_receipt", "operator_resume", "resume-1"),
    ]


def test_commands_forward_task_addressed_inspection_requests_without_v1_fallback(monkeypatch) -> None:
    inspection = _InspectionTaskClient()
    monkeypatch.setattr(
        command_module,
        "get_native_inspection_task_client",
        lambda *, required: inspection,
    )
    commands = command_module.Commands()

    assert commands.start_inspection_task("task-42", "route-a", 7, "request-start") is True
    assert commands.pause_inspection_task("task-42", "operator_hold", "request-pause") is True
    assert commands.resume_inspection_task("task-42", "operator_resume", "request-resume") is True
    assert commands.cancel_inspection_task("task-42", "operator_cancel", "request-cancel") is True

    assert inspection.calls == [
        ("start", "task-42", "route-a", 7, "request-start"),
        ("pause", "task-42", "operator_hold", "request-pause"),
        ("resume", "task-42", "operator_resume", "request-resume"),
        ("cancel", "task-42", "operator_cancel", "request-cancel"),
    ]


def test_commands_forward_typed_pause_and_resume_for_one_task(monkeypatch) -> None:
    navigation = _NavigationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_navigation_client",
        lambda *, required: navigation,
    )
    commands = command_module.Commands()

    pause_receipt = commands.pause_task(
        "task-1",
        "operator_pause",
        request_id="pause-1",
    )
    resume_receipt = commands.resume_task(
        "task-1",
        "operator_resume",
        request_id="resume-1",
    )

    assert pause_receipt.kind == NavigationCommandKind.PAUSE_TASK
    assert resume_receipt.kind == NavigationCommandKind.RESUME_TASK
    assert pause_receipt.task_id == resume_receipt.task_id == "task-1"
    assert pause_receipt.request_id == "pause-1"
    assert resume_receipt.request_id == "resume-1"
    assert navigation.calls == [
        ("pause_task", "task-1", "operator_pause", "pause-1"),
        ("resume_task", "task-1", "operator_resume", "resume-1"),
    ]


def test_commands_forward_run_bound_exploration_lifecycle_requests(monkeypatch) -> None:
    exploration = _ExplorationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_exploration_command_client",
        lambda *, required: exploration,
    )
    commands = command_module.Commands()

    receipts = [
        commands.start_exploration(
            _EXPLORATION_RUN_ID, "product-session-a", "web_start", "start-1"
        ),
        commands.pause_exploration(
            _EXPLORATION_RUN_ID, "product-session-a", "web_pause", "pause-1"
        ),
        commands.resume_exploration(
            _EXPLORATION_RUN_ID, "product-session-a", "web_resume", "resume-1"
        ),
        commands.stop_exploration(
            _EXPLORATION_RUN_ID, "product-session-a", "web_stop", "stop-1"
        ),
    ]

    assert [receipt["request_id"] for receipt in receipts] == [
        "start-1",
        "pause-1",
        "resume-1",
        "stop-1",
    ]
    assert all(receipt["exploration_run_id"] == _EXPLORATION_RUN_ID for receipt in receipts)
    assert exploration.calls == [
        ("start", _EXPLORATION_RUN_ID, "product-session-a", "web_start", "start-1"),
        ("pause", _EXPLORATION_RUN_ID, "product-session-a", "web_pause", "pause-1"),
        ("resume", _EXPLORATION_RUN_ID, "product-session-a", "web_resume", "resume-1"),
        ("stop", _EXPLORATION_RUN_ID, "product-session-a", "web_stop", "stop-1"),
    ]


def test_commands_forward_typed_directed_exploration_requests(monkeypatch) -> None:
    exploration = _ExplorationClient()
    monkeypatch.setattr(
        command_module,
        "get_native_exploration_command_client",
        lambda *, required: exploration,
    )
    commands = command_module.Commands()

    assert (
        commands.set_directed_exploration_target(
            12,
            -8.5,
            45,
            _EXPLORATION_RUN_ID,
            "session-a",
            "web_directed_target",
            "directed-set-1",
        )
        == {
            "accepted": True,
            "request_id": "directed-set-1",
            "exploration_run_id": _EXPLORATION_RUN_ID,
            "reason": "admitted",
            "duplicate": False,
        }
    )
    assert (
        commands.clear_directed_exploration_target(
            _EXPLORATION_RUN_ID,
            "session-a",
            "web_clear_directed_target",
            "directed-clear-1",
        )
        == {
            "accepted": True,
            "request_id": "directed-clear-1",
            "exploration_run_id": _EXPLORATION_RUN_ID,
            "reason": "admitted",
            "duplicate": False,
        }
    )

    assert exploration.calls == [
        (
            "set",
            12.0,
            -8.5,
            45.0,
            _EXPLORATION_RUN_ID,
            "session-a",
            "web_directed_target",
            "directed-set-1",
        ),
        (
            "clear",
            _EXPLORATION_RUN_ID,
            "session-a",
            "web_clear_directed_target",
            "directed-clear-1",
        ),
    ]


@pytest.mark.parametrize(
    ("receipt", "error"),
    [
        (True, "invalid receipt"),
        (
            {
                "accepted": True,
                "request_id": "start-1",
                "exploration_run_id": "01ARZ3NDEKTSV4RRFFQ69G5FAA",
                "reason": "admitted",
                "duplicate": False,
            },
            "wrong exploration_run_id",
        ),
        (
            {
                "accepted": True,
                "request_id": "other-request",
                "exploration_run_id": _EXPLORATION_RUN_ID,
                "reason": "admitted",
                "duplicate": False,
            },
            "wrong request_id",
        ),
    ],
)
def test_commands_fail_closed_on_uncorrelated_exploration_receipt(
    monkeypatch, receipt, error
) -> None:
    class UntrustedExplorationClient:
        @staticmethod
        def start(*_args, **_kwargs):
            return receipt

    monkeypatch.setattr(
        command_module,
        "get_native_exploration_command_client",
        lambda *, required: UntrustedExplorationClient(),
    )

    with pytest.raises(RuntimeError, match=error):
        command_module.Commands().start_exploration(
            _EXPLORATION_RUN_ID,
            "product-session-a",
            request_id="start-1",
        )


def test_commands_are_the_typed_operator_motion_adapter(monkeypatch) -> None:
    operator_motion = _OperatorMotionClient()
    required_values = []

    def get_client(*, required):
        required_values.append(required)
        return operator_motion

    monkeypatch.setattr(
        command_module,
        "get_native_operator_motion_client",
        get_client,
    )
    commands = command_module.Commands()

    claim_receipt = commands.claim(
        "ws:operator-a",
        42,
        1,
        lease_ttl_ms=1000,
        request_id="claim-1",
    )
    assert (
        commands.sample(
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
    hold_receipt = commands.hold(
        "ws:operator-a",
        42,
        3,
        reason="manual_hold",
        request_id="hold-1",
    )
    release_receipt = commands.release(
        "ws:operator-a",
        42,
        4,
        reason="disconnect",
        request_id="release-1",
    )
    assert isinstance(claim_receipt, OperatorMotionReceipt)
    assert claim_receipt.request_id == "claim-1"
    assert claim_receipt.final_output_sequence == 0
    assert hold_receipt.final_output_sequence == 103
    assert release_receipt.final_output_sequence == 104

    assert required_values == [True, True, True, True]
    assert operator_motion.calls == [
        ("claim", "ws:operator-a", 42, 1, 1000, "claim-1"),
        (
            "sample",
            "ws:operator-a",
            42,
            2,
            0.2,
            -0.1,
            0.5,
            True,
            True,
            350,
            "sample-1",
        ),
        ("hold", "ws:operator-a", 42, 3, "manual_hold", "hold-1"),
        ("release", "ws:operator-a", 42, 4, "disconnect", "release-1"),
    ]


@pytest.mark.parametrize("ack", [None, False, 1, "true", {"accepted": True}])
def test_commands_reject_ambiguous_operator_motion_sample_ack(monkeypatch, ack) -> None:
    monkeypatch.setattr(
        command_module,
        "get_native_operator_motion_client",
        lambda *, required: _OperatorMotionClient(
            sample_ack=ack,
        ),
    )
    commands = command_module.Commands()

    with pytest.raises(RuntimeError, match="invalid submission"):
        commands.sample("ws:operator-a", 42, 1, 0.2, 0.0, 0.1)


def test_commands_preserve_rejected_operator_motion_receipt(monkeypatch) -> None:
    rejected = OperatorMotionReceipt(
        accepted=False,
        action=int(OperatorMotionAction.CLAIM),
        request_id="claim-rejected",
        source_id="ws:operator-a",
        source_epoch=42,
        source_sequence=1,
        accepted_sequence=0,
        final_output_sequence=0,
        endpoint_timestamp_s=123.5,
        reason="source_busy",
    )
    monkeypatch.setattr(
        command_module,
        "get_native_operator_motion_client",
        lambda *, required: _OperatorMotionClient(rejected),
    )
    commands = command_module.Commands()

    result = commands.claim(
        "ws:operator-a",
        42,
        1,
        lease_ttl_ms=1000,
        request_id="claim-rejected",
    )

    assert result is rejected
    assert result.accepted is False


def test_commands_fail_closed_without_operator_motion_capability(monkeypatch) -> None:
    monkeypatch.setattr(
        command_module,
        "get_native_operator_motion_client",
        lambda *, required: None,
    )
    commands = command_module.Commands()

    with pytest.raises(RuntimeError, match="boundary is unavailable"):
        commands.claim("ws:operator-a", 42, 1, lease_ttl_ms=1000)


def test_commands_reject_invalid_inspection_identity_before_native_call(monkeypatch) -> None:
    inspection = _InspectionTaskClient()
    monkeypatch.setattr(
        command_module,
        "get_native_inspection_task_client",
        lambda *, required: inspection,
    )
    commands = command_module.Commands()

    with pytest.raises(RuntimeError, match="task_id is required"):
        commands.start_inspection_task("", "route-a")
    with pytest.raises(RuntimeError, match="route_id is required"):
        commands.start_inspection_task("task-42", "  ")
    with pytest.raises(ValueError, match="UINT64_MAX"):
        commands.start_inspection_task("task-42", "route-a", -1)

    assert inspection.calls == []


@pytest.mark.parametrize("deadman", ["false", "0", 1, None])
def test_commands_reject_non_boolean_deadman(monkeypatch, deadman) -> None:
    operator_motion = _OperatorMotionClient()
    monkeypatch.setattr(
        command_module,
        "get_native_operator_motion_client",
        lambda *, required: operator_motion,
    )

    with pytest.raises(TypeError, match="deadman must be a boolean"):
        command_module.Commands().sample(
            "ws:operator-a", 42, 1, 0.2, 0.0, 0.1, deadman=deadman
        )

    assert operator_motion.calls == []


def test_commands_setup_validates_shared_native_session_without_sending_motion(
    monkeypatch,
) -> None:
    class Session:
        def __init__(self) -> None:
            self.handle = object()
            self.calls: list[str] = []

        def require_open(self) -> None:
            self.calls.append("require_open")

        def ensure_navigation_abi(self) -> None:
            self.calls.append("ensure_navigation_abi")

        def ensure_operator_motion_abi(self) -> None:
            self.calls.append("ensure_operator_motion_abi")

        def close(self) -> None:
            self.calls.append("close")

    session = Session()
    monkeypatch.setattr(
        command_module,
        "get_native_command_session",
        lambda *, required: session,
    )
    commands = command_module.Commands()

    commands.setup()
    commands.start()

    assert commands.startup_readiness() is None
    assert commands.health() == {
        "ok": True,
        "running": True,
        "readiness": "ready",
        "failure": "",
        "native": {
            "session_acquired": True,
            "handle_open": True,
            "navigation_abi_ready": True,
            "operator_motion_abi_ready": True,
        },
    }
    assert session.calls.count("ensure_navigation_abi") == 1
    assert session.calls.count("ensure_operator_motion_abi") == 1
    assert "require_open" in session.calls
    assert "close" not in session.calls


def test_inspection_product_commands_validate_task_abi_during_setup(monkeypatch) -> None:
    class Session:
        handle = object()

        def __init__(self) -> None:
            self.calls: list[str] = []

        def require_open(self) -> None:
            self.calls.append("require_open")

        def ensure_navigation_abi(self) -> None:
            self.calls.append("ensure_navigation_abi")

        def ensure_operator_motion_abi(self) -> None:
            self.calls.append("ensure_operator_motion_abi")

        def ensure_inspection_task_abi(self) -> None:
            self.calls.append("ensure_inspection_task_abi")

    session = Session()
    monkeypatch.setattr(
        command_module,
        "get_native_command_session",
        lambda *, required: session,
    )
    commands = command_module.Commands(require_inspection_task_commands=True)

    commands.setup()
    commands.start()

    assert commands.startup_readiness() is None
    assert session.calls.count("ensure_inspection_task_abi") == 1


def test_inspection_product_commands_fail_setup_without_task_abi(monkeypatch) -> None:
    class Session:
        handle = object()

        @staticmethod
        def require_open() -> None:
            return None

        @staticmethod
        def ensure_navigation_abi() -> None:
            return None

        @staticmethod
        def ensure_operator_motion_abi() -> None:
            return None

        @staticmethod
        def ensure_inspection_task_abi() -> None:
            raise RuntimeError("missing task symbols")

    monkeypatch.setattr(
        command_module,
        "get_native_command_session",
        lambda *, required: Session(),
    )
    commands = command_module.Commands(require_inspection_task_commands=True)

    with pytest.raises(
        RuntimeError,
        match="native_inspection_task_abi_unavailable:missing task symbols",
    ):
        commands.setup()

    assert commands.startup_readiness() == (
        "native_inspection_task_abi_unavailable:missing task symbols"
    )


def test_commands_report_operator_motion_abi_setup_failure(monkeypatch) -> None:
    class Session:
        handle = object()

        def __init__(self) -> None:
            self.closed = False

        def require_open(self) -> None:
            return None

        def ensure_navigation_abi(self) -> None:
            return None

        def ensure_operator_motion_abi(self) -> None:
            raise RuntimeError("missing operator receipt symbols")

        def close(self) -> None:
            self.closed = True

    session = Session()
    monkeypatch.setattr(
        command_module,
        "get_native_command_session",
        lambda *, required: session,
    )
    commands = command_module.Commands()

    with pytest.raises(
        RuntimeError,
        match="native_operator_motion_abi_unavailable:missing operator receipt symbols",
    ):
        commands.setup()

    reason = commands.startup_readiness()
    assert reason == (
        "native_operator_motion_abi_unavailable:missing operator receipt symbols"
    )
    assert commands.health()["native"] == {
        "session_acquired": True,
        "handle_open": True,
        "navigation_abi_ready": True,
        "operator_motion_abi_ready": False,
    }
    assert session.closed is False


def test_commands_readiness_distinguishes_uninitialized_and_not_running(
    monkeypatch,
) -> None:
    class Session:
        handle = object()

        def require_open(self) -> None:
            return None

        def ensure_navigation_abi(self) -> None:
            return None

        def ensure_operator_motion_abi(self) -> None:
            return None

    commands = command_module.Commands()
    assert commands.startup_readiness() == "native_command_session_unavailable"

    monkeypatch.setattr(
        command_module,
        "get_native_command_session",
        lambda *, required: Session(),
    )
    commands.setup()

    assert commands.startup_readiness() == "not_running"
    assert commands.health()["readiness"] == "not_running"


def test_commands_detect_shared_native_handle_closing_after_start(
    monkeypatch,
) -> None:
    class Session:
        handle = object()

        def __init__(self) -> None:
            self.open = True

        def require_open(self) -> None:
            if not self.open:
                raise RuntimeError("native navigation client is closed")

        def ensure_navigation_abi(self) -> None:
            return None

        def ensure_operator_motion_abi(self) -> None:
            return None

    session = Session()
    monkeypatch.setattr(
        command_module,
        "get_native_command_session",
        lambda *, required: session,
    )
    commands = command_module.Commands()
    commands.setup()
    commands.start()
    session.open = False

    reason = commands.startup_readiness()
    assert reason == (
        "native_command_handle_unavailable:native navigation client is closed"
    )
    health = commands.health()
    assert health["ok"] is False
    assert health["readiness"] == reason
    assert health["native"]["handle_open"] is False
