"""Focused tests for semantic inspection lifecycle commands."""

from __future__ import annotations

import json

import pytest

from nav.services.goals import GoalService


class FakeInspectionCommands:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str, str | None]] = []

    def pause_inspection_task(
        self,
        *,
        task_id: str,
        reason: str,
        request_id: str | None = None,
    ) -> bool:
        self.calls.append(("pause_inspection_task", task_id, reason, request_id))
        return True

    def resume_inspection_task(
        self,
        *,
        task_id: str,
        reason: str,
        request_id: str | None = None,
    ) -> bool:
        self.calls.append(("resume_inspection_task", task_id, reason, request_id))
        return True

    def cancel_inspection_task(
        self,
        *,
        task_id: str,
        reason: str,
        request_id: str | None = None,
    ) -> bool:
        self.calls.append(("cancel_inspection_task", task_id, reason, request_id))
        return True


def _native_goal_service(commands: object | None = None) -> tuple[GoalService, list[dict]]:
    service = GoalService(command_module="nav.commands")
    if commands is not None:
        service.on_system_modules({"nav.commands": commands})
    service.setup()
    statuses: list[dict] = []
    service.goal_status.subscribe(statuses.append)
    return service, statuses


def _send(service: GoalService, command: dict) -> None:
    service.goal_command._deliver(json.dumps(command))


@pytest.mark.parametrize(
    ("action", "method", "default_reason"),
    [
        ("inspection_pause", "pause_inspection_task", "operator_pause"),
        ("inspection_resume", "resume_inspection_task", "operator_resume"),
        ("inspection_cancel", "cancel_inspection_task", "operator_cancel"),
    ],
)
def test_inspection_lifecycle_dispatches_native_method_with_reason_and_request_id(
    action: str,
    method: str,
    default_reason: str,
) -> None:
    commands = FakeInspectionCommands()
    service, statuses = _native_goal_service(commands)

    _send(
        service,
        {
            "action": action,
            "task_id": "inspection-task-7",
            "request_id": "semantic-7",
        },
    )

    assert commands.calls == [(method, "inspection-task-7", default_reason, "semantic-7")]
    assert statuses[-1]["success"] is True
    assert statuses[-1]["action"] == action
    assert statuses[-1]["task_id"] == "inspection-task-7"
    assert statuses[-1]["reason"] == default_reason
    assert statuses[-1]["request_id"] == "semantic-7"
    assert statuses[-1]["sink"] == "native_dds"


def test_inspection_lifecycle_preserves_explicit_reason() -> None:
    commands = FakeInspectionCommands()
    service, statuses = _native_goal_service(commands)

    _send(
        service,
        {
            "action": "inspection_pause",
            "task_id": "inspection-task-voice",
            "reason": "voice_pause",
            "request_id": "voice-1",
        },
    )

    assert commands.calls == [
        ("pause_inspection_task", "inspection-task-voice", "voice_pause", "voice-1")
    ]
    assert statuses[-1]["reason"] == "voice_pause"


def test_inspection_lifecycle_rejects_missing_task_id_without_guessing_current_task() -> None:
    commands = FakeInspectionCommands()
    service, statuses = _native_goal_service(commands)

    _send(service, {"action": "inspection_pause", "request_id": "semantic-no-task"})

    assert commands.calls == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["request_id"] == "semantic-no-task"
    assert "task_id is required" in statuses[-1]["message"]


def test_inspection_lifecycle_rejects_when_command_module_not_configured() -> None:
    service = GoalService()
    service.setup()
    statuses: list[dict] = []
    service.goal_status.subscribe(statuses.append)

    _send(
        service,
        {
            "action": "inspection_pause",
            "task_id": "inspection-task-unconfigured",
            "request_id": "semantic-8",
        },
    )

    assert statuses[-1]["success"] is False
    assert statuses[-1]["accepted"] is False
    assert statuses[-1]["request_id"] == "semantic-8"
    assert statuses[-1]["sink"] == "native_dds"
    assert "requires native command capability" in statuses[-1]["message"]


def test_inspection_lifecycle_rejects_when_command_module_unavailable() -> None:
    service, statuses = _native_goal_service()

    _send(
        service,
        {
            "action": "inspection_resume",
            "task_id": "inspection-task-unavailable",
            "request_id": "semantic-9",
        },
    )

    assert statuses[-1]["success"] is False
    assert statuses[-1]["request_id"] == "semantic-9"
    assert "unavailable" in statuses[-1]["message"]


def test_inspection_lifecycle_rejects_missing_native_method() -> None:
    service, statuses = _native_goal_service(object())

    _send(
        service,
        {
            "action": "inspection_cancel",
            "task_id": "inspection-task-missing-method",
            "request_id": "semantic-10",
        },
    )

    assert statuses[-1]["success"] is False
    assert statuses[-1]["request_id"] == "semantic-10"
    assert "does not implement cancel_inspection_task" in statuses[-1]["message"]


@pytest.mark.parametrize(
    "action",
    ["pause_inspection", "resume_inspection", "cancel_inspection"],
)
def test_taskless_inspection_action_aliases_are_retired(action: str) -> None:
    commands = FakeInspectionCommands()
    service, statuses = _native_goal_service(commands)

    _send(
        service,
        {
            "action": action,
            "task_id": "inspection-task-old-alias",
            "request_id": "semantic-old-alias",
        },
    )

    assert commands.calls == []
    assert statuses[-1]["success"] is False
    assert statuses[-1]["message"] == f"unknown action: {action}"
