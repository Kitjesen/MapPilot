"""Project normalized navigation facts into the four public status axes."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, Final

from gateway.navigation.tasks import (
    NavigationTaskProjectionError,
    project_navigation_goal_status,
)

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
            "reason": ("navigation_state_stale" if navigation_state else "navigation_state_unavailable"),
        }
    exact_status = _exact_goal_status(navigation_state, goal_status)

    if exact_status:
        try:
            projected = project_navigation_goal_status(exact_status)
        except NavigationTaskProjectionError:
            projected = {}
        state = _text(projected.get("lifecycle_state_name")).upper() or "UNKNOWN"
        if state == "EXECUTING" and _text(navigation_state.get("recovery_state_name")).upper() == "ACTIVE":
            state = "RECOVERING"
        return {
            "state": state,
            "task_id": task_id,
            "reason": _text(exact_status.get("reason")),
        }

    lifecycle = _text(navigation_state.get("lifecycle_state_name")).upper()
    if not task_id and not request_id and lifecycle == "IDLE":
        return {
            "state": "IDLE",
            "task_id": "",
            "reason": "",
        }
    return {
        "state": "UNKNOWN",
        "task_id": task_id,
        "reason": "task_status_unavailable",
    }


def _goal_admission_projection(
    readiness: Mapping[str, Any],
    native_endpoint: Mapping[str, Any],
) -> dict[str, Any]:
    blockers = [_text(code) for code in readiness.get("blockers", []) if _text(code)]
    endpoint_unknown = native_endpoint.get("required") is True and native_endpoint.get("status_available") is not True
    can_accept_goal = readiness.get("can_accept_goal")
    if endpoint_unknown or not isinstance(can_accept_goal, bool):
        state = "UNKNOWN"
    else:
        state = "ACCEPTING" if can_accept_goal else "BLOCKED"
    if state == "UNKNOWN":
        reason = _text(readiness.get("reason")) or "goal_admission_unknown"
    elif state == "BLOCKED":
        reason = blockers[0] if blockers else "navigation_not_ready"
    else:
        reason = ""
    return {
        "state": state,
        "reason": reason,
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
        (native_endpoint.get("active_cmd_source") if native_endpoint.get("status_available") is True else None)
        or (navigation_state.get("authority") if navigation_state_fresh is True else None)
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

    # The public flag answers whether the operator must use the motion-resume
    # boundary.  Native takeover and the independent runtime hold are separate
    # implementation facts, but either one requires that same operator action.
    resume_required = (
        control.get("resume_required") is True
        or control.get("operator_takeover_latched") is True
    )
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
    if not isinstance(readiness.get("can_accept_goal"), bool):
        return "UNKNOWN", _text(readiness.get("reason")) or "motion_permission_unknown"
    if native_endpoint.get("status_available") is True or (navigation_state_fresh is True and navigation_state):
        return "CLEAR", "motion_clear"
    return "UNKNOWN", "motion_permission_unknown"


def _motion_observation(
    odometry: Mapping[str, Any],
    odometry_fresh: bool | None,
    stop_evidence: Mapping[str, Any],
) -> str:
    if odometry_fresh is not True:
        return "UNKNOWN"

    vx = _finite_float(odometry.get("vx"))
    vy = _finite_float(odometry.get("vy"))
    wz = _finite_float(odometry.get("wz"))
    if vx is None or wz is None:
        return "UNKNOWN"
    linear_speed = math.hypot(vx, vy or 0.0)
    angular_speed = abs(wz)
    linear_threshold = _finite_float(stop_evidence.get("linear_speed_threshold_mps"))
    angular_threshold = _finite_float(stop_evidence.get("angular_speed_threshold_radps"))
    if linear_threshold is None or linear_threshold < 0.0:
        linear_threshold = _DEFAULT_LINEAR_SPEED_THRESHOLD_MPS
    if angular_threshold is None or angular_threshold < 0.0:
        angular_threshold = _DEFAULT_ANGULAR_SPEED_THRESHOLD_RADPS
    return "MOVING" if linear_speed > linear_threshold or angular_speed > angular_threshold else "QUIET"


def project_navigation_status(gate: Mapping[str, Any]) -> dict[str, Any]:
    """Project one evaluated navigation snapshot into the four public axes."""

    native_state = _mapping(gate.get("navigation_state"))
    navigation_state_fresh = gate.get("navigation_state_fresh")
    exact_goal_status = _mapping(gate.get("goal_status"))
    readiness_state = gate
    control_state = _mapping(gate.get("control"))
    endpoint_state = _mapping(gate.get("native_endpoint"))
    odometry_state = _mapping(gate.get("odometry"))
    odometry_fresh = gate.get("odometry_fresh")
    stop_evidence = _mapping(endpoint_state.get("motion_stop_evidence"))

    task = _task_projection(native_state, navigation_state_fresh, exact_goal_status)
    goal_admission = _goal_admission_projection(
        readiness_state,
        endpoint_state,
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
    observation = _motion_observation(
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
    # When motion is held, explain the hold first.  Stop confirmation is an
    # independent observation and must not hide why the robot cannot move.
    motion_reason = (
        permission_reason
        if permission != "CLEAR"
        else stop_reason
        if stop_reason_is_operator_relevant and stop_reason
        else permission_reason
    )
    motion = {
        "permission": permission,
        "observation": observation,
        "stop_confirmation": stop_confirmation,
        "reason": motion_reason,
    }
    return {
        "task": task,
        "goal_admission": goal_admission,
        "control": control_projection,
        "motion": motion,
    }
