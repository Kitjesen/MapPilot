from __future__ import annotations

from collections import UserDict
from types import SimpleNamespace

import pytest

from gateway.services.command_boundary import (
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
