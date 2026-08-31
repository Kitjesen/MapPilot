"""Gateway-side access to the assembled navigation command capability."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


class CommandBoundaryError(RuntimeError):
    """Raised when a required product command cannot reach its endpoint."""


def _has_positive_mapping_ack(result: Any) -> bool:
    return isinstance(result, Mapping) and (
        result.get("accepted") is True or result.get("success") is True
    )


def navigation_commands(owner: Any) -> Any | None:
    commands = getattr(owner, "_nav_commands", None)
    if commands is not None:
        return commands
    modules = getattr(owner, "_all_modules", None)
    if isinstance(modules, dict):
        return modules.get("nav.commands")
    return None


def submit_goal(
    owner: Any,
    goal: Any,
    *,
    task_id: str | None = None,
    request_id: str | None = None,
) -> dict[str, Any] | bool:
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, "submit_goal", None)
    if not callable(operation):
        return False
    try:
        result = operation(goal, task_id=task_id, request_id=request_id, action="goal")
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if not _has_positive_mapping_ack(result):
        reason = (
            result.get("message")
            if isinstance(result, Mapping)
            else "invalid goal service response"
        )
        raise CommandBoundaryError(str(reason))
    receipt = dict(result)
    expected_task_id = str(task_id or "").strip()
    returned_task_id = str(receipt.get("task_id") or "").strip()
    if expected_task_id and returned_task_id != expected_task_id:
        raise CommandBoundaryError("goal service returned the wrong task_id")
    receipt["task_id"] = returned_task_id or expected_task_id
    return receipt


def submit_cancel(
    owner: Any,
    reason: str,
    *,
    task_id: str | None = None,
    request_id: str | None = None,
) -> dict[str, Any] | bool:
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, "submit_cancel", None)
    if not callable(operation):
        return False
    try:
        result = operation(reason, task_id=task_id, request_id=request_id)
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if not _has_positive_mapping_ack(result):
        message = (
            result.get("message")
            if isinstance(result, Mapping)
            else "invalid goal service response"
        )
        raise CommandBoundaryError(str(message))
    receipt = dict(result)
    expected_task_id = str(task_id or "").strip()
    returned_task_id = str(receipt.get("task_id") or "").strip()
    if expected_task_id and returned_task_id != expected_task_id:
        raise CommandBoundaryError("goal service returned the wrong task_id")
    receipt["task_id"] = returned_task_id or expected_task_id
    return receipt


def _submit_task_control(
    owner: Any,
    reason: str,
    *,
    method: str,
    expected_state: str,
    task_id: str,
    request_id: str,
) -> dict[str, Any] | bool:
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, method, None)
    if not callable(operation):
        return False
    try:
        result = operation(
            reason,
            task_id=task_id,
            request_id=request_id,
        )
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if not _has_positive_mapping_ack(result):
        message = (
            result.get("message")
            if isinstance(result, Mapping)
            else f"invalid {method} service response"
        )
        raise CommandBoundaryError(str(message))

    receipt = dict(result)
    expected_task_id = str(task_id or "").strip()
    returned_task_id = str(receipt.get("task_id") or "").strip()
    if not expected_task_id or returned_task_id != expected_task_id:
        raise CommandBoundaryError(f"goal service returned the wrong task_id for {method}")

    expected_request_id = str(request_id or "").strip()
    returned_request_id = str(receipt.get("request_id") or "").strip()
    if not expected_request_id or returned_request_id != expected_request_id:
        raise CommandBoundaryError(
            f"goal service returned the wrong request_id for {method}"
        )

    returned_state = str(receipt.get("state") or "").strip()
    if returned_state != expected_state:
        raise CommandBoundaryError(
            f"goal service returned the wrong acknowledgement state for {method}"
        )
    return receipt


def submit_pause(
    owner: Any,
    reason: str,
    *,
    task_id: str,
    request_id: str,
) -> dict[str, Any] | bool:
    """Submit a task-specific pause request through nav.goals."""
    return _submit_task_control(
        owner,
        reason,
        method="submit_pause",
        expected_state="pause_requested",
        task_id=task_id,
        request_id=request_id,
    )


def submit_resume(
    owner: Any,
    reason: str,
    *,
    task_id: str,
    request_id: str,
) -> dict[str, Any] | bool:
    """Submit a task-specific resume request through nav.goals."""
    return _submit_task_control(
        owner,
        reason,
        method="submit_resume",
        expected_state="resume_requested",
        task_id=task_id,
        request_id=request_id,
    )


def invoke_navigation_command(
    owner: Any,
    method: str,
    *,
    required: bool,
    accept_receipt: bool = False,
    **kwargs: Any,
) -> dict[str, Any] | bool:
    commands = navigation_commands(owner)
    if commands is None:
        if required:
            raise CommandBoundaryError("native navigation command capability is unavailable")
        return False
    operation = getattr(commands, method, None)
    if not callable(operation):
        raise CommandBoundaryError(f"native navigation command capability does not implement {method}")
    try:
        result = operation(**kwargs)
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if result is True:
        return True
    if accept_receipt and isinstance(result, Mapping):
        if result.get("accepted") is True:
            return dict(result)
        reason = str(result.get("reason") or "was rejected")
        raise CommandBoundaryError(f"native navigation command {method} was rejected: {reason}")
    if result is not True:
        outcome = "was rejected" if result is False else "returned an invalid acknowledgement"
        raise CommandBoundaryError(f"native navigation command {method} {outcome}")
    return True


__all__ = [
    "CommandBoundaryError",
    "invoke_navigation_command",
    "navigation_commands",
    "submit_cancel",
    "submit_goal",
    "submit_pause",
    "submit_resume",
]
