"""Shared navigation operations for external transports."""

from __future__ import annotations

import contextlib
import time
from typing import Any

from gateway.navigation.commands import (
    CommandBoundaryError,
    submit_cancel,
    submit_goal,
    submit_pause,
    submit_resume,
    submit_speed_limit,
)
from gateway.navigation.goals import construct_goal_from_request
from gateway.schemas import (
    CancelRequest,
    ClickNavRequest,
    GoalRequest,
    NavigationTaskPauseRequest,
    NavigationTaskResumeRequest,
    NavigationTaskSpeedLimitRequest,
)
from gateway.services.control_commands import ControlCommandService


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


def go(gw: Any, body: GoalRequest | ClickNavRequest, *, command: str = "goal"):
    command_service = ControlCommandService(gw)
    def _publish() -> dict[str, Any]:
        goal = construct_goal_from_request(
            body,
            gw=gw,
            default_source=body.source,
            default_target_type=body.target_type,
        )
        ts = time.time()
        task_id = str(body.task_id or "").strip() or _new_navigation_task_id()
        ack = submit_goal(
            gw,
            goal.pose_stamped(ts=ts),
            task_id=task_id,
            request_id=body.request_id,
            max_speed_mps=goal.max_speed_mps,
            acceptance_radius_m=goal.acceptance_radius_m,
        )
        return {
            **goal.command_payload(
                status="ok",
                instruction=getattr(body, "instruction", None),
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
        command,
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
        return command_service.native_error("navigation_cancel", command_body, exc, "native_command_rejected")


def control_task(
    gw: Any,
    body: NavigationTaskPauseRequest | NavigationTaskResumeRequest | NavigationTaskSpeedLimitRequest,
    *,
    route_task_id: str,
    action: str,
) -> Any:
    command_service = ControlCommandService(gw)
    task_id = str(route_task_id or "").strip()
    body_task_id = str(body.task_id or "").strip()
    command_name = f"navigation_task_{action}"
    if not task_id:
        return command_service.rejected_response(
            command_name,
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
    if body_task_id and body_task_id != task_id:
        return command_service.rejected_response(
            command_name,
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

    request_id = str(body.request_id or "").strip() or f"nav-{action}-{time.time_ns()}"
    if request_id == task_id:
        return command_service.rejected_response(
            command_name,
            body,
            error="request_identity_conflict",
            message="request_id and task_id must identify different things.",
            detail=command_service.command_error_detail(
                reason_code="request_identity_conflict",
                reason="request_id must differ from task_id",
                source="gateway_navigation_task_api",
                blockers=["request_identity_conflict"],
            ),
        )
    command_body = body.model_copy(update={"task_id": task_id, "request_id": request_id})
    status = f"{action}_requested"

    def _publish() -> dict[str, Any]:
        if action == "speed_limit":
            result = submit_speed_limit(
                gw, command_body.max_speed_mps, task_id=task_id, request_id=request_id,
            )
        else:
            operation = submit_pause if action == "pause" else submit_resume
            result = operation(
                gw, command_body.reason, task_id=task_id, request_id=request_id,
            )
        if not isinstance(result, dict):
            raise CommandBoundaryError(f"navigation task {action} capability is unavailable")
        payload = {
            "accepted": True,
            "status": status,
            "stage": "request_acknowledged",
            "reason": result["native_ack"]["reason"] if action == "speed_limit" else command_body.reason,
            "task_id": task_id,
            "native_request_id": result.get("native_request_id"),
            "native_ack": result.get("native_ack"),
            "execution_confirmed": False,
            "final_output_confirmed": False,
        }
        if action == "resume":
            payload["goal_reissue_required"] = False
        return payload

    try:
        if action in {"resume", "speed_limit"}:
            return command_service.run_motion_guarded_command(
                command_name,
                command_body,
                _publish,
                success_status_code=202,
            )
        return gw._run_control_command(
            command_name,
            command_body,
            _publish,
            success_status_code=202,
        )
    except CommandBoundaryError as exc:
        return command_service.native_error(command_name, command_body, exc, "native_command_rejected")
