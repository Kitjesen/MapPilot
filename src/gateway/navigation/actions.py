"""Shared navigation operations for external transports."""

from __future__ import annotations

import contextlib
import time
from typing import Any

from gateway.navigation.commands import CommandBoundaryError, submit_cancel, submit_goal
from gateway.navigation.goals import construct_goal_from_request
from gateway.schemas import CancelRequest, GoalRequest
from gateway.services.control_commands import ControlCommandService


def _publish_goal(
    gw: Any,
    goal: Any,
    *,
    ts: float,
    task_id: str,
    request_id: str | None = None,
) -> dict[str, Any]:
    typed_goal = goal.pose_stamped(ts=ts)
    result = submit_goal(gw, typed_goal, task_id=task_id, request_id=request_id,
                         max_speed_mps=goal.max_speed_mps,
                         acceptance_radius_m=goal.acceptance_radius_m)
    if isinstance(result, dict):
        return dict(result)
    if result is True:
        return {"accepted": True, "success": True, "task_id": task_id}
    raise CommandBoundaryError("goal service is unavailable")

def _new_navigation_task_id() -> str:
    return f"nav-task-{time.time_ns()}"

def _active_navigation_task_id(gw: Any) -> str | None:
    with getattr(gw, "_state_lock", contextlib.nullcontext()):
        native_state = getattr(gw, "_navigation_state", None)
        if isinstance(native_state, dict):
            task_id = str(native_state.get("active_task_id") or "").strip()
            if task_id:
                return task_id
    return None


def go(gw: Any, body: GoalRequest):
    command_service = ControlCommandService(gw)
    def _publish() -> dict[str, Any]:
        goal = construct_goal_from_request(
            body,
            gw=gw,
            default_source="coordinate",
            default_target_type="coordinate",
        )
        ts = time.time()
        task_id = str(body.task_id or "").strip() or _new_navigation_task_id()
        ack = _publish_goal(
            gw,
            goal,
            ts=ts,
            task_id=task_id,
            request_id=body.request_id,
        )
        return {
            **goal.command_payload(
                status="ok",
                instruction=body.instruction,
                ts=ts,
            ),
            "task_id": ack.get("task_id") or task_id,
            "native_request_id": ack.get("native_request_id"),
            "native_ack": ack.get("native_ack"),
            "accepted": True,
            "stage": "submitted",
            "execution_confirmed": False,
        }

    return command_service.run_planned_goal_command(
        "goal",
        body,
        _publish,
    )


def cancel(gw: Any, body: CancelRequest, task_id: str | None = None):
    command_service = ControlCommandService(gw)
    route_task_id = task_id
    resolved_route_task_id = str(route_task_id or "").strip()
    body_task_id = str(body.task_id or "").strip()
    if route_task_id is not None and not resolved_route_task_id:
        return command_service.rejected_response(
            "navigation_cancel",
            body,
            error="task_identity_invalid",
            message="The navigation task route requires a non-empty task_id.",
            detail=command_service.command_error_detail(
                reason_code="task_identity_invalid",
                reason="route task_id must not be blank",
                source="gateway_navigation_task_api",
                blockers=["task_identity_invalid"],
            ),
        )
    if resolved_route_task_id and body_task_id and body_task_id != resolved_route_task_id:
        return command_service.rejected_response(
            "navigation_cancel",
            body,
            error="task_identity_conflict",
            message="The request body task_id does not match the task route.",
            detail=command_service.command_error_detail(
                reason_code="task_identity_conflict",
                reason="body task_id must match route task_id",
                source="gateway_navigation_task_api",
                blockers=["task_identity_conflict"],
            ),
        )
    command_body = body.model_copy(update={"task_id": resolved_route_task_id}) if resolved_route_task_id else body

    def _publish() -> dict[str, Any]:
        task_id = str(command_body.task_id or "").strip()
        if not task_id:
            task_id = _active_navigation_task_id(gw)
        wrote_native = submit_cancel(
            gw,
            command_body.reason,
            task_id=task_id,
            request_id=command_body.request_id,
        )
        if not wrote_native:
            raise CommandBoundaryError("goal service is unavailable")
        ack = dict(wrote_native) if isinstance(wrote_native, dict) else {}
        return {
            "accepted": True,
            "status": "cancel_requested",
            "stage": "native_acknowledged",
            "reason": command_body.reason,
            "task_id": ack.get("task_id") or task_id,
            "native_request_id": ack.get("native_request_id"),
            "native_ack": ack.get("native_ack"),
            "execution_confirmed": False,
        }

    try:
        return gw._run_control_command(
            "navigation_cancel",
            command_body,
            _publish,
        )
    except CommandBoundaryError as exc:
        reason = str(exc)
        return command_service.rejected_response(
            "navigation_cancel",
            command_body,
            error="native_command_rejected",
            message="Native navigation endpoint rejected the cancel request.",
            detail=command_service.command_error_detail(
                reason_code="native_command_rejected",
                reason=reason,
                source="native_navigation_command_ack",
                blockers=[reason],
            ),
        )
