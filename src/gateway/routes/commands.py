"""Control command routes for GatewayModule."""

from __future__ import annotations

import os
import subprocess
import time
from typing import Any

from fastapi.responses import JSONResponse

from runtime.msgs.geometry import Twist, Vector3
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
    PlanPreviewRequest,
    PlanPreviewResponse,
    StopRequest,
    VisualServoRequest,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.goal_builder import construct_goal_from_request


CONTROL_COMMAND_ERROR_RESPONSES = {
    409: {"model": GatewayErrorResponse},
}

LEASE_ERROR_RESPONSES = {
    403: {"model": GatewayErrorResponse},
    409: {"model": GatewayErrorResponse},
}


def _native_nav_control_bin() -> str:
    return os.environ.get("LINGTU_NAV_CONTROL_BIN", "").strip()


def _publish_native_nav_control(command: str, *args: str) -> bool:
    binary = _native_nav_control_bin()
    if not binary:
        return False
    if not os.path.isfile(binary):
        raise RuntimeError(f"native nav control binary missing: {binary}")
    domain_id = os.environ.get("LINGTU_DDS_DOMAIN_ID", "0").strip() or "0"
    subprocess.run(
        [binary, command, *args, "--domain-id", domain_id],
        check=True,
        timeout=5.0,
    )
    return True


def _publish_goal(gw: Any, goal: Any, *, ts: float) -> None:
    if _publish_native_nav_control(
        "goal",
        f"{goal.x:.9g}",
        f"{goal.y:.9g}",
        f"{goal.z:.9g}",
        f"{goal.yaw:.9g}",
    ):
        return
    gw.goal_pose.publish(goal.pose_stamped(ts=ts))


def register_command_routes(app, gw) -> None:
    command_service = ControlCommandService(gw)

    @app.post(
        "/api/v1/navigation/plan",
        summary="Preview navigation plan without publishing a goal",
        response_model=PlanPreviewResponse,
    )
    async def post_navigation_plan(body: PlanPreviewRequest):
        return command_service.preview_navigation_plan(body)

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
            preview = command_service.preview_navigation_plan(
                goal.preview_request(client_id=body.client_id)
            )
            reasons = list(preview.get("reasons") or [])
            status = (
                "preview_feasible"
                if bool(preview.get("feasible", False))
                else "preview_infeasible"
            )

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
            _publish_goal(gw, goal, ts=ts)
            if body.instruction:
                if not _publish_native_nav_control("instruction", body.instruction):
                    gw.instruction.publish(body.instruction)
            return goal.command_payload(
                status="ok",
                instruction=body.instruction,
                ts=ts,
            )

        return command_service.run_planned_goal_command("goal", body, _publish)

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
            _publish_goal(gw, goal, ts=ts)
            return goal.command_payload(status="ok", ts=ts)

        return command_service.run_planned_goal_command(
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
            gw.cmd_vel.publish(
                Twist(
                    linear=Vector3(body.vx, body.vy, 0),
                    angular=Vector3(0, 0, body.wz),
                )
            )
            return {"status": "ok"}

        return command_service.run_motion_guarded_command("cmd_vel", body, _publish)

    @app.post(
        "/api/v1/stop",
        summary="Emergency stop",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_stop(body: StopRequest | None = None):
        def _publish() -> dict[str, Any]:
            gw.stop_cmd.publish(2)
            gw.cmd_vel.publish(Twist())
            return {"status": "stopped"}

        return gw._run_control_command("stop", body, _publish)

    @app.post(
        "/api/v1/navigation/cancel",
        summary="Gracefully cancel current navigation mission",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_cancel(body: CancelRequest):
        def _publish() -> dict[str, Any]:
            if not _publish_native_nav_control("cancel", body.reason):
                gw.cancel.publish(body.reason)
            return {"status": "cancelled", "reason": body.reason}

        return gw._run_control_command("navigation_cancel", body, _publish)

    @app.post(
        "/api/v1/instruction",
        summary="Natural language navigation instruction",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_instruction(body: InstructionRequest):
        def _publish() -> dict[str, Any]:
            if not _publish_native_nav_control("instruction", body.text):
                gw.instruction.publish(body.text)
            return {"status": "ok", "instruction": body.text}

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

        servo_target = (
            "stop"
            if body.mode == "stop"
            else f"{body.mode}:{body.target}"
        )

        def _publish() -> dict[str, Any]:
            gw.servo_target.publish(servo_target)
            return {
                "status": "ok",
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
            with gw._state_lock:
                gw._mode = body.mode
            gw.mode_cmd.publish(body.mode)
            if body.mode == "estop":
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            return {"status": "ok", "mode": body.mode}

        return gw._run_control_command("mode", body, _publish)

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
                result = {"status": "acquired", **gw._lease.to_dict()}
                _publish_lease_event(result)
                return result

            if body.action == "release":
                gw._lease.release(body.client_id)
                result = {"status": "released", **gw._lease.to_dict()}
                _publish_lease_event(result)
                return result

            ok = gw._lease.renew(body.client_id, body.ttl)
            if not ok:
                raise PermissionError("not_lease_holder")
            result = {"status": "renewed", **gw._lease.to_dict()}
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
            _publish_lease_event({
                "status": "rejected",
                "error": error,
                **gw._lease.to_dict(),
            })
            return JSONResponse(
                status_code=status_code,
                content=content,
            )
