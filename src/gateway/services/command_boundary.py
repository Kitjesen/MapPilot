"""Gateway-side access to the assembled navigation command capability."""

from __future__ import annotations

from typing import Any


class CommandBoundaryError(RuntimeError):
    """Raised when a required product command cannot reach its endpoint."""


def navigation_commands(owner: Any) -> Any | None:
    commands = getattr(owner, "_nav_commands", None)
    if commands is not None:
        return commands
    modules = getattr(owner, "_all_modules", None)
    if isinstance(modules, dict):
        return modules.get("nav.commands")
    return None


def submit_goal(owner: Any, goal: Any, *, request_id: str | None = None) -> bool:
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, "submit_goal", None)
    if not callable(operation):
        return False
    try:
        result = operation(goal, request_id=request_id, action="goal")
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if not isinstance(result, dict) or not bool(result.get("accepted", result.get("success", False))):
        reason = result.get("message") if isinstance(result, dict) else "invalid goal service response"
        raise CommandBoundaryError(str(reason))
    return True


def submit_cancel(
    owner: Any,
    reason: str,
    *,
    request_id: str | None = None,
) -> bool:
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, "submit_cancel", None)
    if not callable(operation):
        return False
    try:
        result = operation(reason, request_id=request_id)
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if not isinstance(result, dict) or not bool(result.get("accepted", result.get("success", False))):
        message = result.get("message") if isinstance(result, dict) else "invalid goal service response"
        raise CommandBoundaryError(str(message))
    return True


def invoke_navigation_command(
    owner: Any,
    method: str,
    *,
    required: bool,
    **kwargs: Any,
) -> bool:
    commands = navigation_commands(owner)
    if commands is None:
        if required:
            raise CommandBoundaryError("native navigation command capability is unavailable")
        return False
    operation = getattr(commands, method, None)
    if not callable(operation):
        raise CommandBoundaryError(f"native navigation command capability does not implement {method}")
    try:
        accepted = operation(**kwargs)
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    if accepted is False:
        raise CommandBoundaryError(f"native navigation command {method} was rejected")
    return True


__all__ = [
    "CommandBoundaryError",
    "invoke_navigation_command",
    "navigation_commands",
    "submit_cancel",
    "submit_goal",
]
