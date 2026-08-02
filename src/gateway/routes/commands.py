"""Control command routes for GatewayModule."""

from __future__ import annotations

import asyncio
import contextlib
import os
import time
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import (
    CancelRequest,
    ClickNavRequest,
    CmdVelRequest,
    ControlCommandResponse,
    GatewayErrorResponse,
    GoalCandidateRequest,
    GoalCandidateResponse,
    GoalRequest,
    InstructionRequest,
    LeaseRequest,
    LeaseResponse,
    ModeRequest,
    NavigationTaskPauseRequest,
    NavigationTaskResumeRequest,
    PlanPreviewRequest,
    PlanPreviewResponse,
    StopRequest,
    VisualServoRequest,
)
from gateway.services.command_boundary import (
    CommandBoundaryError,
    submit_cancel,
    submit_goal,
    submit_pause,
    submit_resume,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.goal_builder import construct_goal_from_request
from gateway.services.native_control import (
    clear_estop as native_clear_estop,
)
from gateway.services.native_control import (
    endpoint_only_enabled,
)
from gateway.services.native_control import (
    estop as native_estop,
)
from gateway.services.native_control import (
    resume_autonomy as native_resume_autonomy,
)
from runtime.msgs.geometry import Twist, Vector3

CONTROL_COMMAND_ERROR_RESPONSES = {
    409: {"model": GatewayErrorResponse},
}

LEASE_ERROR_RESPONSES = {
    403: {"model": GatewayErrorResponse},
    409: {"model": GatewayErrorResponse},
}


def _native_commands_required(gw: Any | None = None) -> bool:
    required_by_env = os.environ.get("LINGTU_NAV_COMMANDS_REQUIRED", "").strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    try:
        required = required_by_env or endpoint_only_enabled(gw)
    except ValueError as exc:
        raise CommandBoundaryError(str(exc)) from exc
    return required


def _publish_goal(
    gw: Any,
    goal: Any,
    *,
    ts: float,
    task_id: str,
    request_id: str | None = None,
) -> dict[str, Any]:
    typed_goal = goal.pose_stamped(ts=ts)
    result = submit_goal(gw, typed_goal, task_id=task_id, request_id=request_id)
    if isinstance(result, dict):
        return dict(result)
    if result is True:
        return {"accepted": True, "success": True, "task_id": task_id}
    if _native_commands_required(gw):
        raise CommandBoundaryError("goal service is unavailable")
    gw.goal_pose.publish(typed_goal)
    return {"accepted": True, "success": True, "task_id": task_id}


def _new_navigation_task_id() -> str:
    return f"nav-task-{time.time_ns()}"


def _new_navigation_request_id(action: str) -> str:
    return f"nav-{action}-{time.time_ns()}"


def _active_navigation_task_id(gw: Any) -> str | None:
    with getattr(gw, "_state_lock", contextlib.nullcontext()):
        mission = getattr(gw, "_mission", None)
        if isinstance(mission, dict):
            task_id = str(mission.get("active_task_id") or mission.get("task_id") or "").strip()
            if task_id:
                return task_id
        native_state = getattr(gw, "_navigation_state", None)
        if isinstance(native_state, dict):
            task_id = str(native_state.get("active_task_id") or "").strip()
            if task_id:
                return task_id
    return None


def register_command_routes(app, gw) -> None:
    """Register Gateway navigation and direct-control command routes."""

    command_service = ControlCommandService(gw)

    @app.post(
        "/api/v1/navigation/plan",
        summary="Preview navigation plan without publishing a goal",
        response_model=PlanPreviewResponse,
    )
    async def post_navigation_plan(body: PlanPreviewRequest):
        return await asyncio.to_thread(command_service.preview_navigation_plan, body)

    @app.post(
        "/api/v1/navigation/goal_candidate",
        summary="Construct and optionally preview a navigation goal without publishing it",
        response_model=GoalCandidateResponse,
    )
    async def post_navigation_goal_candidate(body: GoalCandidateRequest):
        ts = time.time()
        try:
            goal = construct_goal_from_request(
                body,
                gw=gw,
                default_source=body.source,
                default_target_type=body.target_type,
            )
        except ValueError as exc:
            return {
                "schema_version": 1,
                "ok": False,
                "status": "invalid",
                "target": None,
                "preview": None,
                "reasons": [str(exc)],
                "error": str(exc),
                "ts": ts,
            }

        preview = None
        reasons: list[str] = []
        status = "constructed"
        if body.preview:
            preview = await asyncio.to_thread(
                command_service.preview_navigation_plan,
                goal.preview_request(client_id=body.client_id),
            )
            reasons = list(preview.get("reasons") or [])
            status = "preview_feasible" if bool(preview.get("feasible", False)) else "preview_infeasible"

        return {
            "schema_version": 1,
            "ok": True,
            "status": status,
            "target": goal.target_payload(ts=ts),
            "preview": preview,
            "reasons": reasons,
            "error": None,
            "ts": ts,
        }

    @app.post(
        "/api/v1/goal",
        summary="Send navigation goal",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_goal(body: GoalRequest):
        goal = construct_goal_from_request(
            body,
            gw=gw,
            default_source="coordinate",
            default_target_type="coordinate",
        )

        def _publish() -> dict[str, Any]:
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

        return await asyncio.to_thread(
            command_service.run_planned_goal_command,
            "goal",
            body,
            _publish,
        )

    @app.post(
        "/api/v1/navigate/click",
        summary="Navigate to map-viewer click point",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigate_click(body: ClickNavRequest):
        goal = construct_goal_from_request(
            body,
            gw=gw,
            default_source="map_click",
            default_target_type="map_point",
        )

        def _publish() -> dict[str, Any]:
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
                **goal.command_payload(status="ok", ts=ts),
                "task_id": ack.get("task_id") or task_id,
                "native_request_id": ack.get("native_request_id"),
                "native_ack": ack.get("native_ack"),
                "accepted": True,
                "stage": "submitted",
                "execution_confirmed": False,
            }

        return await asyncio.to_thread(
            command_service.run_planned_goal_command,
            "navigate_click",
            body,
            _publish,
        )

    @app.post(
        "/api/v1/cmd_vel",
        summary="Direct velocity command",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_cmd_vel(body: CmdVelRequest):
        def _publish() -> dict[str, Any]:
            twist = Twist(
                linear=Vector3(body.vx, body.vy, 0),
                angular=Vector3(0, 0, body.wz),
            )
            adapter_reported_write = False
            if hasattr(gw, "publish_remote_velocity_request"):
                adapter_reported_write = bool(
                    gw.publish_remote_velocity_request(
                        twist,
                        request_id=body.request_id,
                    )
                )
            else:
                gw.cmd_vel.publish(twist)
            return {
                "accepted": True,
                "status": "ok",
                "stage": "source_request_accepted",
                "source_request_accepted": True,
                "adapter_reported_write": adapter_reported_write,
                "endpoint_submission_confirmed": False,
                "execution_confirmed": False,
                "final_output_confirmed": False,
                "dds": False,
                "teleop_cmd_vel_dds": False,
            }

        return await asyncio.to_thread(
            command_service.run_motion_guarded_command,
            "cmd_vel",
            body,
            _publish,
        )

    @app.post(
        "/api/v1/stop",
        summary="Emergency stop",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_stop(body: StopRequest | None = None):
        def _publish() -> dict[str, Any]:
            request_id = body.request_id if body is not None else None
            wrote_dds = native_estop(
                gw,
                "rest_emergency_stop",
                request_id=request_id,
            )
            if not wrote_dds and bool(getattr(gw, "_teleop_dds_enabled", False)):
                raise CommandBoundaryError("native stop command boundary is unavailable")
            if not wrote_dds:
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            return {
                "accepted": True,
                "status": "stopped",
                "stage": "native_acknowledged" if wrote_dds else "local_published",
                "dds": wrote_dds,
                "native_control": "estop" if wrote_dds else "local_compat",
            }

        try:
            return await asyncio.to_thread(
                gw._run_control_command,
                "stop",
                body,
                _publish,
            )
        except CommandBoundaryError as exc:
            reason = str(exc)
            return command_service.rejected_response(
                "stop",
                body,
                error="native_command_rejected",
                message="Native navigation endpoint did not acknowledge the stop command.",
                detail=command_service.command_error_detail(
                    reason_code="native_command_rejected",
                    reason=reason,
                    source="native_navigation_command_ack",
                    blockers=[reason],
                ),
            )

    async def _submit_navigation_cancel(
        body: CancelRequest,
        *,
        route_task_id: str | None = None,
    ):
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
        command_body = (
            body.model_copy(update={"task_id": resolved_route_task_id})
            if resolved_route_task_id
            else body
        )

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
                if _native_commands_required(gw):
                    raise CommandBoundaryError("goal service is unavailable")
                gw.cancel.publish(command_body.reason)
            ack = dict(wrote_native) if isinstance(wrote_native, dict) else {}
            return {
                "accepted": True,
                "status": "cancel_requested",
                "stage": "native_acknowledged" if wrote_native else "local_published",
                "reason": command_body.reason,
                "task_id": ack.get("task_id") or task_id,
                "native_request_id": ack.get("native_request_id"),
                "native_ack": ack.get("native_ack"),
                "execution_confirmed": False,
            }

        try:
            return await asyncio.to_thread(
                gw._run_control_command,
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

    @app.post(
        "/api/v1/navigation/cancel",
        summary="Compatibility cancel for the current navigation mission",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_cancel(body: CancelRequest):
        return await _submit_navigation_cancel(body)

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/cancel",
        summary="Request cancellation of one navigation task",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_task_cancel(task_id: str, body: CancelRequest):
        return await _submit_navigation_cancel(body, route_task_id=task_id)

    async def _submit_navigation_task_control(
        body: NavigationTaskPauseRequest | NavigationTaskResumeRequest,
        *,
        route_task_id: str,
        action: str,
    ) -> Any:
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

        request_id = (
            str(body.request_id or "").strip()
            or _new_navigation_request_id(action)
        )
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
        command_body = body.model_copy(
            update={"task_id": task_id, "request_id": request_id}
        )
        status = f"{action}_requested"

        def _publish() -> dict[str, Any]:
            operation = submit_pause if action == "pause" else submit_resume
            result = operation(
                gw,
                command_body.reason,
                task_id=task_id,
                request_id=request_id,
            )
            if not isinstance(result, dict):
                raise CommandBoundaryError(
                    f"navigation task {action} capability is unavailable"
                )
            payload = {
                "accepted": True,
                "status": status,
                "stage": "request_acknowledged",
                "reason": command_body.reason,
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
            if action == "resume":
                return await asyncio.to_thread(
                    command_service.run_motion_guarded_command,
                    command_name,
                    command_body,
                    _publish,
                    success_status_code=202,
                )
            return await asyncio.to_thread(
                gw._run_control_command,
                command_name,
                command_body,
                _publish,
                success_status_code=202,
            )
        except CommandBoundaryError as exc:
            reason = str(exc)
            return command_service.rejected_response(
                command_name,
                command_body,
                error="native_command_rejected",
                message=f"Native navigation endpoint rejected task {action}.",
                detail=command_service.command_error_detail(
                    reason_code="native_command_rejected",
                    reason=reason,
                    source="native_navigation_command_ack",
                    blockers=[reason],
                ),
            )

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/pause",
        summary="Request a stop-confirmed pause of one navigation task",
        response_model=ControlCommandResponse,
        status_code=202,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_task_pause(
        task_id: str,
        body: NavigationTaskPauseRequest,
    ):
        return await _submit_navigation_task_control(
            body,
            route_task_id=task_id,
            action="pause",
        )

    @app.post(
        "/api/v1/navigation/tasks/{task_id}/resume",
        summary="Request continuation of the same paused navigation task",
        response_model=ControlCommandResponse,
        status_code=202,
        responses=LEASE_ERROR_RESPONSES,
    )
    async def post_navigation_task_resume(
        task_id: str,
        body: NavigationTaskResumeRequest,
    ):
        return await _submit_navigation_task_control(
            body,
            route_task_id=task_id,
            action="resume",
        )

    @app.post(
        "/api/v1/navigation/resume",
        summary="Release manual takeover and require a fresh navigation goal/path",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_resume(body: StopRequest | None = None):
        client_id = body.client_id if body is not None else "unknown"
        if not gw._lease.check(client_id):
            return command_service.rejected_response(
                "navigation_resume",
                body,
                error="control_lease",
                message="Only the active control owner may resume autonomy.",
                detail=command_service.command_error_detail(
                    reason_code="control_lease",
                    reason="Only the active control owner may resume autonomy.",
                    source="control_lease",
                    path="/api/v1/lease",
                    blockers=["control_lease"],
                    lease=gw._lease.to_dict(),
                ),
                status_code=403,
            )

        def _publish() -> dict[str, Any]:
            request_id = body.request_id if body is not None else None
            wrote_dds = native_resume_autonomy(
                gw,
                "operator_resume",
                request_id=request_id,
            )
            if not wrote_dds and bool(getattr(gw, "_teleop_dds_enabled", False)):
                raise CommandBoundaryError("native autonomy resume boundary is unavailable")
            return {
                "accepted": True,
                "status": "autonomy_resume_ready",
                "stage": "native_acknowledged" if wrote_dds else "local_compat_ready",
                "dds": wrote_dds,
                "goal_reissue_required": True,
            }

        try:
            return await asyncio.to_thread(
                gw._run_control_command,
                "navigation_resume",
                body,
                _publish,
            )
        except CommandBoundaryError as exc:
            reason = str(exc)
            return command_service.rejected_response(
                "navigation_resume",
                body,
                error="native_command_rejected",
                message="Native navigation endpoint rejected autonomy resume.",
                detail=command_service.command_error_detail(
                    reason_code="native_command_rejected",
                    reason=reason,
                    source="native_navigation_command_ack",
                    blockers=[reason],
                ),
            )

    @app.post(
        "/api/v1/instruction",
        summary="Natural language navigation instruction",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_instruction(body: InstructionRequest):
        def _publish() -> dict[str, Any]:
            gw.instruction.publish(body.text)
            return {
                "accepted": True,
                "status": "ok",
                "stage": "submitted",
                "execution_confirmed": False,
                "instruction": body.text,
            }

        return command_service.run_motion_guarded_command(
            "instruction",
            body,
            _publish,
        )

    @app.post(
        "/api/v1/visual_servo",
        summary="Hot-switch visual servo target",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_visual_servo(body: VisualServoRequest):
        modules = getattr(gw, "_all_modules", {}) or {}
        if modules.get("VisualServoModule") is None:
            return command_service.rejected_response(
                "visual_servo",
                body,
                error="visual_servo_unavailable",
                message="VisualServoModule is not loaded in the current runtime profile.",
                detail=command_service.command_error_detail(
                    reason_code="visual_servo_unavailable",
                    reason="VisualServoModule is not loaded in the current runtime profile.",
                    source="gateway_modules",
                    path="/api/v1/runtime/switch-plan",
                    blockers=["visual_servo_unavailable"],
                ),
            )

        servo_target = "stop" if body.mode == "stop" else f"{body.mode}:{body.target}"

        def _publish() -> dict[str, Any]:
            gw.servo_target.publish(servo_target)
            return {
                "accepted": True,
                "status": "ok",
                "stage": "submitted",
                "execution_confirmed": False,
                "mode": body.mode,
                "visual_target": body.target,
                "servo_target": servo_target,
            }

        if body.mode == "stop":
            return gw._run_control_command("visual_servo", body, _publish)
        return command_service.run_motion_guarded_command(
            "visual_servo",
            body,
            _publish,
        )

    @app.post(
        "/api/v1/mode",
        summary="Switch operating mode",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_mode(body: ModeRequest):
        def _publish() -> dict[str, Any]:
            if body.mode == "estop":
                wrote_dds = native_estop(
                    gw,
                    "mode_estop",
                    request_id=body.request_id,
                )
                if not wrote_dds and bool(getattr(gw, "_teleop_dds_enabled", False)):
                    raise CommandBoundaryError("native estop command boundary is unavailable")
                if not wrote_dds:
                    gw.stop_cmd.publish(2)
                    gw.cmd_vel.publish(Twist())
            with gw._state_lock:
                gw._mode = body.mode
            gw.mode_cmd.publish(body.mode)
            return {
                "accepted": True,
                "status": "ok",
                "stage": "published",
                "mode": body.mode,
            }

        try:
            return await asyncio.to_thread(
                gw._run_control_command,
                "mode",
                body,
                _publish,
            )
        except CommandBoundaryError as exc:
            reason = str(exc)
            return command_service.rejected_response(
                "mode",
                body,
                error="native_command_rejected",
                message="Native navigation endpoint did not acknowledge emergency stop.",
                detail=command_service.command_error_detail(
                    reason_code="native_command_rejected",
                    reason=reason,
                    source="native_navigation_command_ack",
                    blockers=[reason],
                ),
            )

    @app.post(
        "/api/v1/estop/reset",
        summary="Explicitly release the native software emergency-stop latch",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_estop_reset(body: StopRequest | None = None):
        def _publish() -> dict[str, Any]:
            request_id = body.request_id if body is not None else None
            wrote_dds = native_clear_estop(
                gw,
                "operator_reset",
                request_id=request_id,
            )
            if not wrote_dds and bool(getattr(gw, "_teleop_dds_enabled", False)):
                raise CommandBoundaryError("native estop reset boundary is unavailable")
            with gw._state_lock:
                if gw._mode == "estop":
                    gw._mode = "manual"
            return {
                "accepted": True,
                "status": "estop_cleared",
                "stage": "native_acknowledged" if wrote_dds else "local_state_updated",
                "dds": wrote_dds,
                "mode": gw._mode,
            }

        try:
            return await asyncio.to_thread(
                gw._run_control_command,
                "estop_reset",
                body,
                _publish,
            )
        except CommandBoundaryError as exc:
            reason = str(exc)
            return command_service.rejected_response(
                "estop_reset",
                body,
                error="native_command_rejected",
                message="Native navigation endpoint did not acknowledge emergency-stop reset.",
                detail=command_service.command_error_detail(
                    reason_code="native_command_rejected",
                    reason=reason,
                    source="native_navigation_command_ack",
                    blockers=[reason],
                ),
            )

    @app.post(
        "/api/v1/lease",
        summary="Acquire/release/renew control lease",
        response_model=LeaseResponse,
        responses=LEASE_ERROR_RESPONSES,
    )
    async def post_lease(body: LeaseRequest):
        def _publish_lease_event(payload: dict[str, Any]) -> None:
            if hasattr(gw, "push_event"):
                gw.push_event({"type": "lease", "data": payload})

        def _apply() -> dict[str, Any]:
            if body.action == "acquire":
                ok = gw._lease.acquire(body.client_id, body.ttl)
                if not ok:
                    raise PermissionError("lease_conflict")
                result = {"ok": True, "status": "acquired", **gw._lease.to_dict()}
                _publish_lease_event(result)
                return result

            if body.action == "release":
                if not gw._lease.release(body.client_id):
                    raise PermissionError("not_lease_holder")
                result = {"ok": True, "status": "released", **gw._lease.to_dict()}
                _publish_lease_event(result)
                return result

            ok = gw._lease.renew(body.client_id, body.ttl)
            if not ok:
                raise PermissionError("not_lease_holder")
            result = {"ok": True, "status": "renewed", **gw._lease.to_dict()}
            _publish_lease_event(result)
            return result

        try:
            return gw._run_control_command("lease", body, _apply)
        except PermissionError as exc:
            error = str(exc)
            status_code = 409 if error == "lease_conflict" else 403
            message = (
                "control lease is held by another client"
                if error == "lease_conflict"
                else "client does not hold the active control lease"
            )
            detail = command_service.command_error_detail(
                reason_code=error,
                reason=message,
                source="control_lease",
                path="/api/v1/lease",
                blockers=[error],
                lease=gw._lease.to_dict(),
            )
            content = {
                "schema_version": 1,
                "ok": False,
                "error": error,
                "message": message,
                "command": {
                    "name": "lease",
                    "request_id": body.request_id,
                    "client_id": body.client_id,
                    "accepted": False,
                    "replay": False,
                    "ts": time.time(),
                },
                "detail": detail,
            }
            if hasattr(gw, "_publish_command_ack"):
                gw._publish_command_ack(content, status_code=status_code)
            _publish_lease_event(
                {
                    "status": "rejected",
                    "error": error,
                    **gw._lease.to_dict(),
                }
            )
            return JSONResponse(
                status_code=status_code,
                content=content,
            )
