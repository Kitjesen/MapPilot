"""Gateway-side access to the assembled navigation command capability."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


class CommandBoundaryError(RuntimeError):
    """Raised when a required product command cannot reach its endpoint."""


class CommandAdmissionUnconfirmed(CommandBoundaryError):
    """Raised when a stable task exists but native admission is not known."""

    def __init__(self, receipt: Mapping[str, Any]) -> None:
        self.receipt = dict(receipt)
        super().__init__(_response_reason(receipt))


def _clean_identity(value: Any) -> str:
    return str(value or "").strip()


def _matches_native_attempt(logical_request_id: str, native_request_id: str) -> bool:
    return native_request_id == logical_request_id or native_request_id.startswith(
        f"{logical_request_id}-clock-retry-"
    )


def _has_positive_mapping_ack(result: Any) -> bool:
    if not isinstance(result, Mapping):
        return False
    verdicts = [result[key] for key in ("accepted", "success") if key in result]
    return bool(verdicts) and all(verdict is True for verdict in verdicts)


def _response_reason(result: Any) -> str:
    if not isinstance(result, Mapping):
        return "goal service returned a non-mapping acknowledgement"
    return str(result.get("message") or result.get("reason") or "goal service rejected the command")


def _validate_requested_identity(
    *,
    task_id: str | None,
    request_id: str | None,
) -> tuple[str, str]:
    expected_task_id = _clean_identity(task_id)
    expected_request_id = _clean_identity(request_id)
    if expected_task_id and expected_task_id == expected_request_id:
        raise CommandBoundaryError("task_id and request_id must be distinct")
    return expected_task_id, expected_request_id


def _validated_task_receipt(
    result: Any,
    *,
    expected_task_id: str,
    expected_request_id: str,
    task_id_required: bool,
) -> dict[str, Any]:
    if isinstance(result, Mapping) and result.get("admission_unconfirmed") is True:
        receipt = dict(result)
        if receipt.get("admission_confirmed") is not False:
            raise CommandBoundaryError("unconfirmed admission has contradictory confirmation state")
        if receipt.get("accepted") is True or receipt.get("success") is True:
            raise CommandBoundaryError("unconfirmed admission has a positive acceptance verdict")

        returned_task_id = _clean_identity(receipt.get("task_id"))
        returned_request_id = _clean_identity(receipt.get("request_id"))
        if not returned_task_id:
            raise CommandBoundaryError("unconfirmed admission is missing task_id")
        if not returned_request_id:
            raise CommandBoundaryError("unconfirmed admission is missing request_id")
        if expected_task_id and returned_task_id != expected_task_id:
            raise CommandBoundaryError("goal service returned the wrong task_id")
        if expected_request_id and returned_request_id != expected_request_id:
            raise CommandBoundaryError("goal service returned the wrong request_id")
        if returned_task_id and returned_task_id == returned_request_id:
            raise CommandBoundaryError("task_id and request_id must be distinct")
        if receipt.get("native_ack") is not None:
            raise CommandBoundaryError("unconfirmed admission cannot include a native acknowledgement")

        native_request_id = _clean_identity(receipt.get("native_request_id"))
        if native_request_id and not _matches_native_attempt(returned_request_id, native_request_id):
            raise CommandBoundaryError("goal service returned the wrong native_request_id")
        receipt["task_id"] = returned_task_id
        receipt["request_id"] = returned_request_id
        if native_request_id:
            receipt["native_request_id"] = native_request_id
        receipt["admission_confirmed"] = False
        receipt["admission_unconfirmed"] = True
        raise CommandAdmissionUnconfirmed(receipt)

    if not _has_positive_mapping_ack(result):
        raise CommandBoundaryError(_response_reason(result))

    receipt = dict(result)
    returned_task_id = _clean_identity(receipt.get("task_id"))
    returned_request_id = _clean_identity(receipt.get("request_id"))
    if task_id_required and not returned_task_id:
        raise CommandBoundaryError("goal service acknowledgement is missing task_id")
    if not returned_request_id:
        raise CommandBoundaryError("goal service acknowledgement is missing request_id")
    if expected_task_id and returned_task_id != expected_task_id:
        raise CommandBoundaryError("goal service returned the wrong task_id")
    if expected_request_id and returned_request_id != expected_request_id:
        raise CommandBoundaryError("goal service returned the wrong request_id")
    if returned_task_id and returned_task_id == returned_request_id:
        raise CommandBoundaryError("task_id and request_id must be distinct")

    native_task_id = _clean_identity(receipt.get("native_task_id"))
    native_request_id = _clean_identity(receipt.get("native_request_id"))
    if native_task_id and native_task_id != returned_task_id:
        raise CommandBoundaryError("goal service returned the wrong native_task_id")
    if native_request_id and not _matches_native_attempt(returned_request_id, native_request_id):
        raise CommandBoundaryError("goal service returned the wrong native_request_id")

    native_ack = receipt.get("native_ack")
    if native_ack is not None:
        if not _has_positive_mapping_ack(native_ack):
            raise CommandBoundaryError("native endpoint returned an invalid acknowledgement")
        ack_task_id = _clean_identity(native_ack.get("task_id"))
        ack_request_id = _clean_identity(native_ack.get("request_id"))
        if ack_task_id != returned_task_id:
            raise CommandBoundaryError("native acknowledgement returned the wrong task_id")
        if not _matches_native_attempt(returned_request_id, ack_request_id):
            raise CommandBoundaryError("native acknowledgement returned the wrong request_id")
    elif receipt.get("sink") == "native_dds":
        raise CommandBoundaryError("native goal service acknowledgement is missing native_ack")

    receipt["task_id"] = returned_task_id
    receipt["request_id"] = returned_request_id
    return receipt


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
    expected_task_id, expected_request_id = _validate_requested_identity(
        task_id=task_id,
        request_id=request_id,
    )
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, "submit_goal", None)
    if not callable(operation):
        return False
    try:
        result = operation(
            goal,
            task_id=task_id,
            request_id=request_id,
            action="goal",
        )
    except Exception as exc:
        raise CommandBoundaryError(str(exc)) from exc
    return _validated_task_receipt(
        result,
        expected_task_id=expected_task_id,
        expected_request_id=expected_request_id,
        task_id_required=True,
    )


def submit_cancel(
    owner: Any,
    reason: str,
    *,
    task_id: str | None = None,
    request_id: str | None = None,
) -> dict[str, Any] | bool:
    expected_task_id, expected_request_id = _validate_requested_identity(
        task_id=task_id,
        request_id=request_id,
    )
    goals = getattr(owner, "_goals", None)
    if goals is None:
        modules = getattr(owner, "_all_modules", None)
        goals = modules.get("nav.goals") if isinstance(modules, dict) else None
    operation = getattr(goals, "submit_cancel", None)
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
    return _validated_task_receipt(
        result,
        expected_task_id=expected_task_id,
        expected_request_id=expected_request_id,
        task_id_required=bool(expected_task_id),
    )


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
    "CommandAdmissionUnconfirmed",
    "CommandBoundaryError",
    "invoke_navigation_command",
    "navigation_commands",
    "submit_cancel",
    "submit_goal",
]
