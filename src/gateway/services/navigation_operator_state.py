"""Project navigation runtime facts into the operator-facing state model."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, Final

OPERATOR_STATE_SCHEMA_VERSION: Final = 1

_TASK_STATE_BY_NATIVE_NAME: Final = {
    "PLANNING": "PLANNING",
    "PATH_ACTIVE": "EXECUTING",
    "EXECUTING": "EXECUTING",
    "PAUSED": "PAUSED",
    "RECOVERING": "RECOVERING",
    "REACHED": "SUCCESS",
    "SUCCESS": "SUCCESS",
    "FAILED": "FAILED",
    "CANCELLED": "CANCELLED",
}
_TERMINAL_TASK_STATES: Final = {"SUCCESS", "FAILED", "CANCELLED"}
_AUTONOMY_AUTHORITIES: Final = {"autonomy", "recovery", "path_follower"}
_OPERATOR_AUTHORITIES: Final = {"teleop", "manual_hold", "operator"}
_NO_AUTHORITIES: Final = {"none", "estop"}
_STOP_CONFIRMATIONS: Final = {"NOT_REQUESTED", "PENDING", "CONFIRMED", "FAILED"}

_DEFAULT_LINEAR_SPEED_THRESHOLD_MPS: Final = 0.03
_DEFAULT_ANGULAR_SPEED_THRESHOLD_RADPS: Final = 0.08


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _text(value: Any) -> str:
    return str(value or "").strip()


def _finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _exact_goal_status(
    navigation_state: Mapping[str, Any],
    goal_status: Mapping[str, Any],
) -> dict[str, Any]:
    task_id = _text(navigation_state.get("active_task_id"))
    request_id = _text(navigation_state.get("active_request_id"))
    boot_id = _text(navigation_state.get("boot_id"))
    if not task_id or not request_id:
        return {}
    if _text(goal_status.get("task_id")) != task_id:
        return {}
    if _text(goal_status.get("request_id")) != request_id:
        return {}
    goal_boot_id = _text(goal_status.get("boot_id"))
    if boot_id and goal_boot_id != boot_id:
        return {}
    return dict(goal_status)


def _task_projection(
    navigation_state: Mapping[str, Any],
    navigation_state_fresh: bool | None,
    goal_status: Mapping[str, Any],
) -> dict[str, Any]:
    task_id = _text(navigation_state.get("active_task_id"))
    request_id = _text(navigation_state.get("active_request_id"))
    if navigation_state_fresh is not True:
        return {
            "state": "UNKNOWN",
            "task_id": task_id,
            "request_id": request_id,
            "terminal": False,
            "progress": None,
            "reason": ("navigation_state_stale" if navigation_state else "navigation_state_unavailable"),
        }
    exact_status = _exact_goal_status(navigation_state, goal_status)

    if exact_status:
        native_name = _text(
            exact_status.get("state_name") or exact_status.get("lifecycle_state_name") or exact_status.get("phase")
        ).upper()
        state = _TASK_STATE_BY_NATIVE_NAME.get(native_name, "UNKNOWN")
        if state == "EXECUTING" and _text(navigation_state.get("recovery_state_name")).upper() == "ACTIVE":
            state = "RECOVERING"
        progress = _finite_float(navigation_state.get("progress"))
        if progress is not None and not 0.0 <= progress <= 1.0:
            progress = None
        return {
            "state": state,
            "task_id": task_id,
            "request_id": request_id,
            "terminal": state in _TERMINAL_TASK_STATES,
            "progress": progress,
            "reason": _text(exact_status.get("reason")),
        }

    lifecycle = _text(navigation_state.get("lifecycle_state_name")).upper()
    if not task_id and not request_id and lifecycle == "IDLE":
        return {
            "state": "IDLE",
            "task_id": "",
            "request_id": "",
            "terminal": False,
            "progress": None,
            "reason": "",
        }
    return {
        "state": "UNKNOWN",
        "task_id": task_id,
        "request_id": request_id,
        "terminal": False,
        "progress": None,
        "reason": "task_status_unavailable",
    }


def _goal_admission_projection(
    readiness: Mapping[str, Any],
    native_endpoint: Mapping[str, Any],
    odometry_fresh: bool | None,
) -> dict[str, Any]:
    blockers = [_text(code) for code in readiness.get("blockers", []) if _text(code)]
    advisories = [_text(code) for code in readiness.get("advisories", []) if _text(code)]
    endpoint_unknown = native_endpoint.get("required") is True and native_endpoint.get("status_available") is not True
    can_accept_goal = readiness.get("can_accept_goal")
    if endpoint_unknown or odometry_fresh is not True or not isinstance(can_accept_goal, bool):
        state = "UNKNOWN"
    else:
        state = "ACCEPTING" if can_accept_goal else "BLOCKED"
    return {
        "state": state,
        "blockers": blockers,
        "advisories": advisories,
    }


def _control_projection(
    navigation_state: Mapping[str, Any],
    navigation_state_fresh: bool | None,
    control: Mapping[str, Any],
    native_endpoint: Mapping[str, Any],
) -> dict[str, Any]:
    if native_endpoint.get("required") is True and native_endpoint.get("status_available") is not True:
        return {
            "authority": "UNKNOWN",
            "resume_required": control.get("resume_required") is True,
            "reason": "control_state_unknown",
        }
    source = _text(
        (navigation_state.get("authority") if navigation_state_fresh is True else None)
        or native_endpoint.get("active_cmd_source")
        or control.get("active_cmd_source")
        or control.get("command_owner")
    ).lower()
    if source in _AUTONOMY_AUTHORITIES:
        authority = "AUTONOMY"
    elif source in _OPERATOR_AUTHORITIES:
        authority = "OPERATOR"
    elif source in _NO_AUTHORITIES:
        authority = "NONE"
    else:
        authority = "UNKNOWN"

    resume_required = control.get("resume_required") is True
    if control.get("estop_latched") is True or source == "estop":
        reason = "estop_latched"
    elif control.get("operator_takeover_latched") is True:
        reason = "operator_takeover"
    elif resume_required:
        reason = "resume_required"
    elif authority == "AUTONOMY":
        reason = "autonomy_active"
    elif authority == "OPERATOR":
        reason = "operator_control_active"
    elif authority == "NONE":
        reason = "no_control_authority"
    else:
        reason = "control_state_unknown"
    return {
        "authority": authority,
        "resume_required": resume_required,
        "reason": reason,
    }


def _motion_permission(
    navigation_state: Mapping[str, Any],
    navigation_state_fresh: bool | None,
    readiness: Mapping[str, Any],
    control: Mapping[str, Any],
    native_endpoint: Mapping[str, Any],
) -> tuple[str, str]:
    active_source = _text(control.get("active_cmd_source") or native_endpoint.get("active_cmd_source")).lower()
    navigation_authority = _text(navigation_state.get("authority") if navigation_state_fresh is True else None).lower()
    if control.get("estop_latched") is True or active_source == "estop" or navigation_authority == "estop":
        return "ESTOPPED", "estop_latched"

    endpoint_unknown = native_endpoint.get("required") is True and native_endpoint.get("status_available") is not True
    if endpoint_unknown:
        return "UNKNOWN", "native_endpoint_status_unavailable"

    input_gate = _mapping(native_endpoint.get("input_gate"))
    if input_gate.get("ready") is False:
        return "HELD", _text(input_gate.get("reason")) or "input_gate_blocked"
    if control.get("operator_takeover_latched") is True:
        return "HELD", "operator_takeover"
    if control.get("resume_required") is True:
        return "HELD", "resume_required"

    blockers = [_text(code) for code in readiness.get("blockers", []) if _text(code)]
    if blockers:
        return "HELD", blockers[0]
    if native_endpoint.get("status_available") is True or (navigation_state_fresh is True and navigation_state):
        return "CLEAR", "motion_clear"
    return "UNKNOWN", "motion_permission_unknown"


def _motion_observation(
    odometry: Mapping[str, Any],
    odometry_fresh: bool | None,
    stop_evidence: Mapping[str, Any],
) -> tuple[str, float | None, float | None]:
    if odometry_fresh is not True:
        return "UNKNOWN", None, None

    vx = _finite_float(odometry.get("vx"))
    vy = _finite_float(odometry.get("vy"))
    wz = _finite_float(odometry.get("wz"))
    if vx is None or wz is None:
        return "UNKNOWN", None, None
    linear_speed = math.hypot(vx, vy or 0.0)
    angular_speed = abs(wz)
    linear_threshold = _finite_float(stop_evidence.get("linear_speed_threshold_mps"))
    angular_threshold = _finite_float(stop_evidence.get("angular_speed_threshold_radps"))
    if linear_threshold is None or linear_threshold < 0.0:
        linear_threshold = _DEFAULT_LINEAR_SPEED_THRESHOLD_MPS
    if angular_threshold is None or angular_threshold < 0.0:
        angular_threshold = _DEFAULT_ANGULAR_SPEED_THRESHOLD_RADPS
    observation = "MOVING" if linear_speed > linear_threshold or angular_speed > angular_threshold else "QUIET"
    return observation, round(linear_speed, 4), round(angular_speed, 4)


def _summary_projection(
    task: Mapping[str, Any],
    goal_admission: Mapping[str, Any],
    control: Mapping[str, Any],
    motion: Mapping[str, Any],
) -> dict[str, str]:
    stop_confirmation = motion["stop_confirmation"]
    if stop_confirmation == "FAILED":
        return {
            "severity": "CRITICAL",
            "code": "STOP_CONFIRMATION_FAILED",
            "next_action": "inspect_stop_failure",
        }
    if motion["permission"] == "ESTOPPED":
        return {"severity": "CRITICAL", "code": "ESTOPPED", "next_action": "clear_estop"}
    if (
        task["state"] == "UNKNOWN"
        or goal_admission["state"] == "UNKNOWN"
        or control["authority"] == "UNKNOWN"
        or motion["permission"] == "UNKNOWN"
        or motion["observation"] == "UNKNOWN"
        or stop_confirmation == "UNKNOWN"
    ):
        return {
            "severity": "WARNING",
            "code": "STATUS_SOURCE_UNKNOWN",
            "next_action": "check_status_sources",
        }
    if motion["permission"] == "HELD":
        return {
            "severity": "WARNING",
            "code": "MOTION_HELD",
            "next_action": "resolve_motion_hold",
        }
    if goal_admission["state"] == "BLOCKED":
        return {
            "severity": "WARNING",
            "code": "GOAL_ADMISSION_BLOCKED",
            "next_action": "resolve_goal_blockers",
        }
    if stop_confirmation == "PENDING":
        return {
            "severity": "WARNING",
            "code": "STOP_CONFIRMATION_PENDING",
            "next_action": "wait_for_stop_confirmation",
        }
    task_state = task["state"]
    if task_state == "FAILED":
        return {
            "severity": "CRITICAL",
            "code": "TASK_FAILED",
            "next_action": "inspect_task_failure",
        }
    if task_state == "RECOVERING":
        return {
            "severity": "WARNING",
            "code": "TASK_RECOVERING",
            "next_action": "monitor_recovery",
        }
    if task_state == "PLANNING":
        return {"severity": "INFO", "code": "TASK_PLANNING", "next_action": "wait_for_plan"}
    if task_state == "PAUSED":
        return {
            "severity": "INFO",
            "code": "TASK_PAUSED",
            "next_action": "resume_or_cancel",
        }
    if goal_admission["advisories"]:
        return {
            "severity": "INFO",
            "code": "NAVIGATION_ADVISORY",
            "next_action": "review_advisories",
        }
    if task_state == "EXECUTING":
        return {
            "severity": "OK",
            "code": "TASK_EXECUTING",
            "next_action": "monitor_progress",
        }
    if task_state == "SUCCESS":
        return {"severity": "OK", "code": "TASK_SUCCEEDED", "next_action": "choose_goal"}
    if task_state == "CANCELLED":
        return {"severity": "INFO", "code": "TASK_CANCELLED", "next_action": "choose_goal"}
    return {"severity": "OK", "code": "READY_FOR_GOAL", "next_action": "choose_goal"}


def project_navigation_operator_state(
    *,
    navigation_state: Mapping[str, Any] | None,
    navigation_state_fresh: bool | None,
    goal_status: Mapping[str, Any] | None,
    readiness: Mapping[str, Any] | None,
    control: Mapping[str, Any] | None,
    native_endpoint: Mapping[str, Any] | None,
    odometry: Mapping[str, Any] | None,
    odometry_fresh: bool | None,
) -> dict[str, Any]:
    """Return the single public projection used by REST and SSE clients."""

    native_state = _mapping(navigation_state)
    exact_goal_status = _mapping(goal_status)
    readiness_state = _mapping(readiness)
    control_state = _mapping(control)
    endpoint_state = _mapping(native_endpoint)
    odometry_state = _mapping(odometry)
    stop_evidence = _mapping(endpoint_state.get("motion_stop_evidence"))

    task = _task_projection(native_state, navigation_state_fresh, exact_goal_status)
    if navigation_state_fresh is not True:
        source_reason = _text(task.get("reason")) or "navigation_state_unavailable"
        goal_admission = {
            "state": "UNKNOWN",
            "blockers": [_text(code) for code in readiness_state.get("blockers", []) if _text(code)],
            "advisories": [_text(code) for code in readiness_state.get("advisories", []) if _text(code)],
        }
        control_projection = {
            "authority": "UNKNOWN",
            "resume_required": control_state.get("resume_required") is True,
            "reason": source_reason,
        }
        motion = {
            "permission": "UNKNOWN",
            "observation": "UNKNOWN",
            "stop_confirmation": "UNKNOWN",
            "linear_speed_mps": None,
            "angular_speed_radps": None,
            "reason": source_reason,
        }
        return {
            "schema_version": OPERATOR_STATE_SCHEMA_VERSION,
            "task": task,
            "goal_admission": goal_admission,
            "control": control_projection,
            "motion": motion,
            "summary": _summary_projection(task, goal_admission, control_projection, motion),
        }

    goal_admission = _goal_admission_projection(
        readiness_state,
        endpoint_state,
        odometry_fresh,
    )
    control_projection = _control_projection(
        native_state,
        navigation_state_fresh,
        control_state,
        endpoint_state,
    )
    permission, permission_reason = _motion_permission(
        native_state,
        navigation_state_fresh,
        readiness_state,
        control_state,
        endpoint_state,
    )
    observation, linear_speed, angular_speed = _motion_observation(
        odometry_state,
        odometry_fresh,
        stop_evidence,
    )
    stop_confirmation = _text(stop_evidence.get("state")).upper()
    if stop_confirmation not in _STOP_CONFIRMATIONS:
        stop_confirmation = "UNKNOWN"
    stop_reason = _text(stop_evidence.get("reason"))
    stop_reason_is_operator_relevant = stop_confirmation in {"PENDING", "CONFIRMED", "FAILED"} or (
        stop_confirmation == "NOT_REQUESTED" and stop_reason == "nonzero_output_published"
    )
    motion_reason = stop_reason if stop_reason_is_operator_relevant and stop_reason else permission_reason
    motion = {
        "permission": permission,
        "observation": observation,
        "stop_confirmation": stop_confirmation,
        "linear_speed_mps": linear_speed,
        "angular_speed_radps": angular_speed,
        "reason": motion_reason,
    }
    return {
        "schema_version": OPERATOR_STATE_SCHEMA_VERSION,
        "task": task,
        "goal_admission": goal_admission,
        "control": control_projection,
        "motion": motion,
        "summary": _summary_projection(task, goal_admission, control_projection, motion),
    }
