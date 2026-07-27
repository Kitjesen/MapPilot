from __future__ import annotations

from types import SimpleNamespace

import pytest

from gateway.services import native_control
from gateway.services.command_boundary import (
    CommandBoundaryError,
    submit_cancel,
    submit_goal,
)


class _Client:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str | None]] = []

    def stop_motion(self, reason: str, *, request_id: str | None = None) -> bool:
        self.calls.append(("stop", reason, request_id))
        return True

    def estop(self, reason: str, *, request_id: str | None = None) -> bool:
        self.calls.append(("estop", reason, request_id))
        return True

    def clear_estop(self, reason: str, *, request_id: str | None = None) -> bool:
        self.calls.append(("clear_estop", reason, request_id))
        return True


def test_endpoint_only_control_commands_share_one_native_boundary(monkeypatch):
    client = _Client()
    owner = SimpleNamespace(_nav_commands=client)
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")

    assert native_control.stop(owner, "rest_stop", request_id="stop-1") is True
    assert native_control.estop(owner, "mcp_estop", request_id="estop-1") is True
    assert native_control.clear_estop(owner, "operator_reset", request_id="reset-1") is True
    assert client.calls == [
        ("stop", "rest_stop", "stop-1"),
        ("estop", "mcp_estop", "estop-1"),
        ("clear_estop", "operator_reset", "reset-1"),
    ]


def test_non_endpoint_profile_reports_no_native_delivery_without_client(monkeypatch):
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.delenv("LINGTU_TELEOP_CMD_DDS", raising=False)
    owner = SimpleNamespace(_nav_commands=None, _all_modules={})
    assert native_control.stop(owner, "dev_stop") is False


def test_legacy_native_boundary_is_resolved_by_the_same_policy(monkeypatch):
    client = _Client()
    owner = SimpleNamespace(_nav_commands=client)
    monkeypatch.delenv("LINGTU_COMMAND_OUTPUT_MODE", raising=False)
    monkeypatch.setenv("LINGTU_TELEOP_CMD_DDS", "1")
    assert native_control.stop(owner, "legacy_stop") is True


def test_conflicting_native_boundary_configuration_fails_closed(monkeypatch):
    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_TELEOP_CMD_DDS", "0")
    owner = SimpleNamespace(_nav_commands=_Client())

    try:
        native_control.stop(owner, "must_not_fallback")
    except ValueError as exc:
        assert "conflicts" in str(exc)
    else:
        raise AssertionError("conflicting control-boundary configuration must fail")


def test_invalid_status_age_configuration_fails_closed(monkeypatch):
    monkeypatch.setenv("LINGTU_NAV_STATUS_MAX_AGE_S", "not-a-number")

    assert native_control.status_is_fresh({"stamp_s": 10.0}, now_s=10.1) is False


class _TaskGoalService:
    def __init__(self, response: object) -> None:
        self.response = response
        self.calls: list[tuple[str, object, str | None, str | None]] = []

    def submit_goal(
        self,
        goal: object,
        *,
        task_id: str | None = None,
        request_id: str | None = None,
        action: str = "goal",
    ) -> object:
        self.calls.append((action, goal, task_id, request_id))
        return self.response

    def submit_cancel(
        self,
        reason: str,
        *,
        task_id: str | None = None,
        request_id: str | None = None,
    ) -> object:
        self.calls.append((reason, None, task_id, request_id))
        return self.response


def _native_task_ack(*, task_id: str, request_id: str) -> dict[str, object]:
    native_ack = {
        "accepted": True,
        "task_id": task_id,
        "request_id": request_id,
        "reason": "accepted",
    }
    return {
        "accepted": True,
        "success": True,
        "task_id": task_id,
        "request_id": request_id,
        "native_task_id": task_id,
        "native_request_id": request_id,
        "native_ack": native_ack,
        "sink": "native_dds",
    }


def test_goal_boundary_preserves_goal_service_generated_task_identity():
    service = _TaskGoalService(
        _native_task_ack(task_id="task-generated", request_id="attempt-1")
    )
    owner = SimpleNamespace(_goals=service)
    goal = object()

    receipt = submit_goal(owner, goal, request_id="attempt-1")

    assert receipt["task_id"] == "task-generated"
    assert receipt["request_id"] == "attempt-1"
    assert service.calls == [("goal", goal, None, "attempt-1")]


def test_goal_boundary_rejects_task_or_attempt_identity_mismatch():
    wrong_task = _TaskGoalService(
        _native_task_ack(task_id="other-task", request_id="attempt-1")
    )
    with pytest.raises(CommandBoundaryError, match="wrong task_id"):
        submit_goal(
            SimpleNamespace(_goals=wrong_task),
            object(),
            task_id="expected-task",
            request_id="attempt-1",
        )

    wrong_attempt = _TaskGoalService(
        _native_task_ack(task_id="expected-task", request_id="other-attempt")
    )
    with pytest.raises(CommandBoundaryError, match="wrong request_id"):
        submit_goal(
            SimpleNamespace(_goals=wrong_attempt),
            object(),
            task_id="expected-task",
            request_id="attempt-1",
        )


@pytest.mark.parametrize(
    "response",
    [
        True,
        {"accepted": 1, "task_id": "task-1", "request_id": "attempt-1"},
        {
            "accepted": True,
            "success": False,
            "task_id": "task-1",
            "request_id": "attempt-1",
        },
    ],
)
def test_goal_boundary_requires_an_unambiguous_positive_mapping_ack(response):
    with pytest.raises(CommandBoundaryError):
        submit_goal(
            SimpleNamespace(_goals=_TaskGoalService(response)),
            object(),
            task_id="task-1",
            request_id="attempt-1",
        )


def test_cancel_boundary_forwards_exact_task_and_validates_native_ack():
    service = _TaskGoalService(
        _native_task_ack(task_id="task-7", request_id="cancel-attempt-1")
    )

    receipt = submit_cancel(
        SimpleNamespace(_goals=service),
        "operator_cancel",
        task_id="task-7",
        request_id="cancel-attempt-1",
    )

    assert receipt["task_id"] == "task-7"
    assert receipt["request_id"] == "cancel-attempt-1"
    assert service.calls == [
        ("operator_cancel", None, "task-7", "cancel-attempt-1")
    ]


def test_goal_boundary_accepts_clock_recovery_native_attempt_identity():
    response = _native_task_ack(task_id="task-9", request_id="attempt-9")
    response["native_request_id"] = "attempt-9-clock-retry-1"
    response["native_ack"]["request_id"] = "attempt-9-clock-retry-1"

    receipt = submit_goal(
        SimpleNamespace(_goals=_TaskGoalService(response)),
        object(),
        task_id="task-9",
        request_id="attempt-9",
    )

    assert receipt["request_id"] == "attempt-9"
    assert receipt["native_request_id"] == "attempt-9-clock-retry-1"
