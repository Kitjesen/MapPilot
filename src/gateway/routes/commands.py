"""Control command routes for GatewayModule."""

from __future__ import annotations

import asyncio
import time
from typing import Any

from fastapi.responses import JSONResponse

from gateway.navigation.commands import (
    CommandBoundaryError,
)
from gateway.schemas import (
    ControlCommandResponse,
    GatewayErrorResponse,
    InstructionRequest,
    LeaseRequest,
    LeaseResponse,
    ModeRequest,
    StopRequest,
    VisualServoRequest,
)
from gateway.services.control_commands import ControlCommandService
from gateway.services.native_control import (
    clear_estop as native_clear_estop,
)
from gateway.services.native_control import (
    estop as native_estop,
)

CONTROL_COMMAND_ERROR_RESPONSES = {
    409: {"model": GatewayErrorResponse},
}

LEASE_ERROR_RESPONSES = {
    403: {"model": GatewayErrorResponse},
    409: {"model": GatewayErrorResponse},
}


def register_command_routes(app, gw) -> None:
    """Register Gateway navigation and direct-control command routes."""

    command_service = ControlCommandService(gw)


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
            if not wrote_dds:
                raise CommandBoundaryError("native stop command boundary is unavailable")
            return {
                "accepted": True,
                "status": "stopped",
                "stage": "native_acknowledged",
                "dds": wrote_dds,
                "native_control": "estop",
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
        summary="Set visual servo target",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_visual_servo(body: VisualServoRequest):
        modules = getattr(gw, "_all_modules", {}) or {}
        visual_servo = modules.get("VisualServoModule")
        if visual_servo is None:
            return command_service.rejected_response(
                "visual_servo",
                body,
                error="visual_servo_unavailable",
                message="VisualServoModule is not loaded in the current runtime profile.",
                detail=command_service.command_error_detail(
                    reason_code="visual_servo_unavailable",
                    reason="VisualServoModule is not loaded in the current runtime profile.",
                    source="gateway_modules",
                    blockers=["visual_servo_unavailable"],
                ),
            )

        if body.mode != "stop":
            perception = modules.get("PerceptionModule")
            perception_health = (
                perception.health()
                if perception is not None and callable(getattr(perception, "health", None))
                else {}
            )
            if perception_health.get("detector_ready") is not True:
                return command_service.rejected_response(
                    "visual_servo",
                    body,
                    error="visual_perception_unavailable",
                    message="Current perception detector is unavailable.",
                    detail=command_service.command_error_detail(
                        reason_code="visual_perception_unavailable",
                        reason="Current perception detector is unavailable.",
                        source="perception_health",
                        blockers=["visual_perception_unavailable"],
                    ),
                )
            if body.mode == "follow" and not body.target_id:
                can_select = getattr(visual_servo, "can_select_follow_target", None)
                if not callable(can_select) or not can_select():
                    return command_service.rejected_response(
                        "visual_servo",
                        body,
                        error="target_selection_unavailable",
                        message="Descriptive person selection is unavailable.",
                        detail=command_service.command_error_detail(
                            reason_code="target_selection_unavailable",
                            reason="Descriptive person selection is unavailable.",
                            source="visual_servo",
                            blockers=["target_selection_unavailable"],
                        ),
                    )

        if body.mode == "stop":
            servo_target = "stop"
        elif body.target_id:
            servo_target = f"follow_id:{body.target_id}"
        else:
            servo_target = f"{body.mode}:{body.target}"

        def _publish() -> dict[str, Any]:
            gw.servo_target.publish(servo_target)
            return {
                "accepted": True,
                "status": "ok",
                "stage": "submitted",
                "execution_confirmed": False,
                "mode": body.mode,
                "visual_target": body.target,
                "visual_target_id": body.target_id,
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
                if not wrote_dds:
                    raise CommandBoundaryError("native estop command boundary is unavailable")
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
            if not wrote_dds:
                raise CommandBoundaryError("native estop reset boundary is unavailable")
            with gw._state_lock:
                if gw._mode == "estop":
                    gw._mode = "manual"
            return {
                "accepted": True,
                "status": "estop_cleared",
                "stage": "native_acknowledged",
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
