from __future__ import annotations

from collections import UserDict
from types import SimpleNamespace

import pytest

from gateway.navigation.commands import (
    CommandBoundaryError,
    submit_cancel,
    submit_goal,
)


class _GoalService:
    def __init__(self, result) -> None:
        self.result = result

    def submit_goal(self, goal, *, task_id=None, request_id=None, action="goal"):
        del goal, task_id, request_id, action
        return self.result

    def submit_cancel(self, reason, *, task_id=None, request_id=None):
        del reason, task_id, request_id
        return self.result


@pytest.mark.parametrize(
    "result",
    [
        {"accepted": True, "task_id": "task-1"},
        {"success": True, "task_id": "task-1"},
        UserDict({"accepted": True, "task_id": "task-1"}),
    ],
)
@pytest.mark.parametrize("operation", ["goal", "cancel"])
def test_goal_boundary_accepts_only_explicit_positive_mapping_ack(result, operation):
    owner = SimpleNamespace(_goals=_GoalService(result))

    if operation == "goal":
        accepted = submit_goal(
            owner,
            object(),
            task_id="task-1",
            request_id="goal-1",
        )
    else:
        accepted = submit_cancel(
            owner,
            "operator_cancel",
            task_id="task-1",
            request_id="cancel-1",
        )

    assert accepted["task_id"] == "task-1"


@pytest.mark.parametrize(
    "result",
    [
        {"accepted": True},
        {"accepted": True, "task_id": "another-task"},
    ],
)
@pytest.mark.parametrize("operation", ["goal", "cancel"])
def test_goal_boundary_rejects_missing_or_mismatched_task_identity(result, operation):
    owner = SimpleNamespace(_goals=_GoalService(result))

    with pytest.raises(CommandBoundaryError, match="wrong task_id"):
        if operation == "goal":
            submit_goal(
                owner,
                object(),
                task_id="task-1",
                request_id="goal-1",
            )
        else:
            submit_cancel(
                owner,
                "operator_cancel",
                task_id="task-1",
                request_id="cancel-1",
            )


@pytest.mark.parametrize(
    "result",
    [
        None,
        True,
        1,
        "true",
        {},
        {"accepted": False},
        {"accepted": 1},
        {"accepted": "true"},
        {"success": {"value": True}},
    ],
)
@pytest.mark.parametrize("operation", ["goal", "cancel"])
def test_goal_boundary_rejects_ambiguous_or_malformed_ack(result, operation):
    owner = SimpleNamespace(_goals=_GoalService(result))

    with pytest.raises(CommandBoundaryError):
        if operation == "goal":
            submit_goal(owner, object(), request_id="goal-1")
        else:
            submit_cancel(owner, "operator_cancel", request_id="cancel-1")


def test_goal_boundary_does_not_override_explicit_native_rejection():
    owner = SimpleNamespace(_goals=_GoalService({
        "accepted": True, "task_id": "task-1", "message": "native rejected",
        "native_ack": {"accepted": False, "reason": "estop_latched"},
    }))
    with pytest.raises(CommandBoundaryError, match="native rejected"):
        submit_goal(owner, object(), task_id="task-1", request_id="goal-1")


@pytest.mark.parametrize("accepted, task_id, request_id, kind", [
    (False, "task-1", "speed-1", 10),
    (True, "other-task", "speed-1", 10),
    (True, "task-1", "other-request", 10),
    (True, "task-1", "speed-1", 9),
])
def test_speed_limit_rejects_negative_or_mismatched_native_ack(accepted, task_id, request_id, kind):
    from gateway.navigation.commands import submit_speed_limit
    from runtime.msgs import NavigationCommandReceipt

    receipt = NavigationCommandReceipt(accepted, kind, task_id, request_id, 123.0, "task_not_active")
    owner = SimpleNamespace(_nav_commands=SimpleNamespace(set_task_speed_limit=lambda **kwargs: receipt))
    with pytest.raises(CommandBoundaryError):
        submit_speed_limit(owner, 0.3, task_id="task-1", request_id="speed-1")
