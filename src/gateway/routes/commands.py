"""Control command routes for GatewayModule."""

from __future__ import annotations

import asyncio
import time
from typing import Any

from fastapi import Query
from fastapi.responses import JSONResponse, Response

from gateway.navigation.commands import (
    CommandBoundaryError,
)
from gateway.schemas import (
    AgentTaskCancelRequest,
    CancelRequest,
    ControlCommandResponse,
    ConversationApprovalRequest,
    ConversationAttachmentRequest,
    ConversationForkRequest,
    ConversationMessageRequest,
    ConversationQueueEditRequest,
    ConversationUpdateRequest,
    FollowTaskCancelRequest,
    GatewayErrorResponse,
    InstructionRequest,
    LeaseRequest,
    LeaseResponse,
    ModeRequest,
    StopRequest,
    VisualServoRequest,
)
from gateway.services import conversations
from gateway.services.control_commands import ControlCommandService
from gateway.services.instructions import cancel_run, get_run, send
from gateway.services.native_control import (
    estop as native_estop,
)
from gateway.services.stop import clear_emergency_stop, stop

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

    @app.get("/api/v1/agent/options")
    async def get_conversation_options():
        return await asyncio.to_thread(conversations.call, gw, "get_conversation_options")

    @app.get("/api/v1/agent/connections/codex")
    async def codex_connection_status():
        result = await asyncio.to_thread(conversations.call, gw, "codex_connection")
        return JSONResponse(result, headers={"Cache-Control": "no-store"})

    @app.post("/api/v1/agent/connections/codex/login")
    async def codex_login():
        result = await asyncio.to_thread(conversations.call, gw, "codex_connection", "start")
        return JSONResponse(result, headers={"Cache-Control": "no-store"})

    @app.post("/api/v1/agent/connections/codex/login/{login_id}/poll")
    async def codex_login_status(login_id: str):
        result = await asyncio.to_thread(conversations.call, gw, "codex_connection", "poll", login_id)
        return JSONResponse(result, headers={"Cache-Control": "no-store"})

    @app.delete("/api/v1/agent/connections/codex/login/{login_id}")
    async def cancel_codex_login(login_id: str):
        return await asyncio.to_thread(conversations.call, gw, "codex_connection", "cancel", login_id)

    @app.delete("/api/v1/agent/connections/codex")
    async def disconnect_codex():
        return await asyncio.to_thread(conversations.call, gw, "codex_connection", "disconnect")

    @app.post("/api/v1/agent/conversations", status_code=201)
    async def create_conversation():
        return await asyncio.to_thread(conversations.call, gw, "create_conversation")

    @app.get("/api/v1/agent/conversations")
    async def list_conversations(archived: bool = False):
        return await asyncio.to_thread(conversations.call, gw, "list_conversations", archived)

    @app.get("/api/v1/agent/conversations/search")
    async def search_conversations(q: str = Query(min_length=1, max_length=200), archived: bool = False):
        return await asyncio.to_thread(conversations.call, gw, "search_conversations", q, archived)

    @app.get("/api/v1/agent/shares/{token}")
    async def get_shared_conversation(token: str):
        return await asyncio.to_thread(conversations.call, gw, "get_shared_conversation", token)

    @app.get("/api/v1/agent/conversations/{conversation_id}")
    async def get_conversation(conversation_id: str, before_seq: int | None = Query(default=None, ge=1)):
        if before_seq is None:
            return await asyncio.to_thread(conversations.call, gw, "get_conversation", conversation_id)
        return await asyncio.to_thread(conversations.call, gw, "get_conversation", conversation_id, before_seq)

    @app.patch("/api/v1/agent/conversations/{conversation_id}")
    async def update_conversation(conversation_id: str, body: ConversationUpdateRequest):
        return await asyncio.to_thread(conversations.call, gw, "update_conversation", conversation_id,
                                       title=body.title, archived=body.archived)

    @app.post("/api/v1/agent/conversations/{conversation_id}/fork", status_code=201)
    async def fork_conversation(conversation_id: str, body: ConversationForkRequest):
        return await asyncio.to_thread(conversations.call, gw, "fork_conversation", conversation_id, body.message_id)

    @app.post("/api/v1/agent/conversations/{conversation_id}/share")
    async def share_conversation(conversation_id: str):
        return await asyncio.to_thread(conversations.call, gw, "share_conversation", conversation_id)

    @app.delete("/api/v1/agent/conversations/{conversation_id}/share")
    async def revoke_conversation_share(conversation_id: str):
        return await asyncio.to_thread(conversations.call, gw, "revoke_conversation_share", conversation_id)

    @app.post("/api/v1/agent/conversations/{conversation_id}/attachments", status_code=201)
    async def upload_conversation_attachment(conversation_id: str, body: ConversationAttachmentRequest):
        return await asyncio.to_thread(
            conversations.call, gw, "upload_conversation_attachment", conversation_id,
            body.attachment_id, body.name, body.mime_type, body.data_base64, model=body.model)

    @app.get("/api/v1/agent/conversations/{conversation_id}/attachments/{attachment_id}")
    async def download_conversation_attachment(conversation_id: str, attachment_id: str):
        attachment = await asyncio.to_thread(conversations.call, gw, "get_conversation_attachment",
                                             conversation_id, attachment_id)
        filename = attachment["name"].replace('"', "").replace("\r", "").replace("\n", "")
        return Response(content=attachment["data"], media_type=attachment["mime_type"],
                        headers={"Content-Disposition": f'inline; filename="{filename}"'})

    @app.post("/api/v1/agent/conversations/{conversation_id}/messages", status_code=202)
    async def post_conversation_message(conversation_id: str, body: ConversationMessageRequest):
        return await asyncio.to_thread(conversations.submit, gw, conversation_id, body)

    @app.patch("/api/v1/agent/conversations/{conversation_id}/queue/{turn_id}")
    async def edit_queued_message(conversation_id: str, turn_id: str, body: ConversationQueueEditRequest):
        return await asyncio.to_thread(conversations.call, gw, "edit_queued_conversation", conversation_id, turn_id, body.text)

    @app.post("/api/v1/agent/conversations/{conversation_id}/queue/{turn_id}/promote")
    async def promote_queued_message(conversation_id: str, turn_id: str):
        return await asyncio.to_thread(conversations.call, gw, "promote_queued_conversation", conversation_id, turn_id)

    @app.delete("/api/v1/agent/conversations/{conversation_id}/queue/{turn_id}")
    async def cancel_queued_message(conversation_id: str, turn_id: str):
        return await asyncio.to_thread(conversations.call, gw, "edit_queued_conversation", conversation_id, turn_id, None)

    @app.post("/api/v1/agent/conversations/{conversation_id}/approvals/{approval_id}")
    async def post_conversation_approval(conversation_id: str, approval_id: str, body: ConversationApprovalRequest):
        return await asyncio.to_thread(conversations.call, gw, "decide_conversation_approval",
                                       conversation_id, approval_id, body.approved)

    @app.get("/api/v1/agent/conversations/{conversation_id}/turns/{turn_id}")
    async def get_conversation_turn(conversation_id: str, turn_id: str):
        return await asyncio.to_thread(conversations.call, gw, "get_conversation_turn", conversation_id, turn_id)


    @app.post(
        "/api/v1/stop",
        summary="Emergency stop",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_stop(body: StopRequest | None = None):
        return await asyncio.to_thread(stop, gw, body)


    @app.post(
        "/api/v1/instruction",
        summary="Natural language navigation instruction",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_instruction(body: InstructionRequest):
        return await asyncio.to_thread(send, gw, body)

    @app.get("/api/v1/agent/runs/{run_id}", summary="Read the current Agent run by identity")
    async def get_agent_run(run_id: str):
        return await asyncio.to_thread(get_run, gw, run_id)

    @app.post(
        "/api/v1/agent/runs/{run_id}/cancel",
        summary="Cancel only the referenced Agent run",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def cancel_agent_run(run_id: str, body: AgentTaskCancelRequest):
        return await asyncio.to_thread(cancel_run, gw, run_id, body)

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

        if body.mode == "follow":
            start_follow = getattr(visual_servo, "start_follow", None)
            if not callable(start_follow):
                return command_service.rejected_response(
                    "visual_servo",
                    body,
                    error="follow_lifecycle_unavailable",
                    message="VisualServoModule does not expose the Follow lifecycle API.",
                    detail=command_service.command_error_detail(
                        reason_code="follow_lifecycle_unavailable",
                        reason="VisualServoModule does not expose the Follow lifecycle API.",
                        source="visual_servo",
                        blockers=["follow_lifecycle_unavailable"],
                    ),
                )

            def _start_follow() -> dict[str, Any]:
                result = start_follow(
                    description=body.target or "",
                    target_id=body.target_id or "",
                    request_id=body.request_id or "",
                    client_id=body.client_id,
                )
                payload = dict(result) if isinstance(result, dict) else {"accepted": False}
                payload.setdefault("status", str(payload.get("state") or "rejected"))
                payload.setdefault("stage", "submitted" if payload.get("accepted") is True else "rejected")
                payload.setdefault("execution_confirmed", False)
                payload.setdefault("mode", "follow")
                if payload.get("follow_task_id"):
                    payload.setdefault("task_id", payload["follow_task_id"])
                return payload

            return command_service.run_motion_guarded_command(
                "visual_servo",
                body,
                _start_follow,
            )

        if body.mode == "stop" and body.follow_task_id:
            cancel_follow = getattr(visual_servo, "cancel_follow_task", None)
            if not callable(cancel_follow):
                return command_service.rejected_response(
                    "visual_servo",
                    body,
                    error="follow_lifecycle_unavailable",
                    message="VisualServoModule does not expose the Follow lifecycle API.",
                    detail=command_service.command_error_detail(
                        reason_code="follow_lifecycle_unavailable",
                        reason="VisualServoModule does not expose the Follow lifecycle API.",
                        source="visual_servo",
                        blockers=["follow_lifecycle_unavailable"],
                    ),
                )

            def _cancel_follow() -> dict[str, Any]:
                result = cancel_follow(
                    body.follow_task_id,
                    body.product_session_id or "",
                    reason=body.reason,
                    request_id=body.request_id or "",
                    client_id=body.client_id,
                )
                payload = dict(result) if isinstance(result, dict) else {"accepted": False}
                payload.setdefault("status", str(payload.get("state") or "rejected"))
                payload.setdefault("stage", "submitted" if payload.get("accepted") is True else "rejected")
                payload.setdefault("mode", "follow")
                payload.setdefault("task_id", body.follow_task_id)
                return payload

            return gw._run_control_command("visual_servo", body, _cancel_follow)

        if body.mode == "stop":
            get_status = getattr(visual_servo, "get_servo_status", None)
            current = get_status() if callable(get_status) else {}
            active_follow = isinstance(current, dict) and (
                current.get("mode") == "follow"
                or (
                    bool(current.get("follow_task_id"))
                    and current.get("task_terminal") is False
                )
            )
            if active_follow:
                return command_service.rejected_response(
                    "visual_servo",
                    body,
                    error="follow_task_ref_required",
                    message="Stopping Follow requires follow_task_id and product_session_id.",
                    detail=command_service.command_error_detail(
                        reason_code="follow_task_ref_required",
                        reason="Stopping Follow requires follow_task_id and product_session_id.",
                        source="visual_servo",
                        blockers=["follow_task_ref_required"],
                    ),
                )
            servo_target = "stop"
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

    @app.get(
        "/api/v1/follow/{follow_task_id}",
        summary="Read one Follow execution",
    )
    async def get_follow_task(follow_task_id: str, product_session_id: str):
        modules = getattr(gw, "_all_modules", {}) or {}
        visual_servo = modules.get("VisualServoModule")
        query = getattr(visual_servo, "get_follow_task", None)
        if not callable(query):
            return {
                "found": False,
                "follow_task_id": follow_task_id,
                "product_session_id": product_session_id,
                "reason": "follow_lifecycle_unavailable",
            }
        result = query(follow_task_id, product_session_id)
        return dict(result) if isinstance(result, dict) else {
            "found": False,
            "follow_task_id": follow_task_id,
            "product_session_id": product_session_id,
            "reason": "invalid_follow_task_view",
        }

    @app.post(
        "/api/v1/follow/{follow_task_id}/cancel",
        summary="Cancel one Follow execution",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def cancel_follow_task(follow_task_id: str, body: FollowTaskCancelRequest):
        modules = getattr(gw, "_all_modules", {}) or {}
        visual_servo = modules.get("VisualServoModule")
        cancel_follow = getattr(visual_servo, "cancel_follow_task", None)
        if not callable(cancel_follow):
            return command_service.rejected_response(
                "follow_cancel",
                body,
                error="follow_lifecycle_unavailable",
                message="VisualServoModule does not expose the Follow lifecycle API.",
                detail=command_service.command_error_detail(
                    reason_code="follow_lifecycle_unavailable",
                    reason="VisualServoModule does not expose the Follow lifecycle API.",
                    source="visual_servo",
                    blockers=["follow_lifecycle_unavailable"],
                ),
            )

        def _cancel() -> dict[str, Any]:
            result = cancel_follow(
                follow_task_id,
                body.product_session_id,
                reason=body.reason,
                request_id=body.request_id or "",
                client_id=body.client_id,
            )
            payload = dict(result) if isinstance(result, dict) else {"accepted": False}
            payload.setdefault("status", str(payload.get("state") or "rejected"))
            payload.setdefault("stage", "submitted" if payload.get("accepted") is True else "rejected")
            payload.setdefault("mode", "follow")
            payload.setdefault("task_id", follow_task_id)
            return payload

        journal_body = CancelRequest(
            reason=body.reason,
            task_id=f"{body.product_session_id}:{follow_task_id}",
            request_id=body.request_id,
            client_id=body.client_id,
        )
        return gw._run_control_command("follow_cancel", journal_body, _cancel)

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
        return await asyncio.to_thread(clear_emergency_stop, gw, body)

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
