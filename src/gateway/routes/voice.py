"""AskMe voice and safety compatibility routes."""

from __future__ import annotations

import json
import math
import time
from collections.abc import Mapping
from typing import Any

from fastapi import Header, HTTPException
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ValidationError

from gateway.schemas import (
    GatewayErrorResponse,
    SafetyEstopRequest,
    SafetyEstopResponse,
    VoiceTurnRequest,
    VoiceTurnResponse,
)
from gateway.services import native_control
from gateway.services.command_boundary import CommandBoundaryError
from gateway.services.control_commands import ControlCommandService
from gateway.services.safety_status import safety_stop_active
from runtime.msgs.geometry import Twist


def _clean_header(value: Any) -> str | None:
    if not isinstance(value, str):
        return None
    cleaned = value.strip()
    return cleaned or None


def _validated_update(model: BaseModel, updates: dict[str, Any]) -> Any:
    try:
        return type(model).model_validate({**model.model_dump(), **updates})
    except ValidationError as exc:
        raise RequestValidationError(exc.errors()) from exc


def _response_payload(response: Any) -> dict[str, Any]:
    if isinstance(response, dict):
        return response
    body = getattr(response, "body", b"")
    if isinstance(body, bytes):
        try:
            payload = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return {}
        return payload if isinstance(payload, dict) else {}
    return {}


def _spoken_reply(payload: dict[str, Any], *, submitted: bool, replay: bool) -> str:
    error = str(payload.get("error") or "").strip()
    if error == "safety_stop":
        return "The robot safety stop is active, so I did not send that instruction."
    if error == "control_lease":
        return "Another operator owns motion control, so I did not send that instruction."
    if error:
        message = str(payload.get("message") or error).strip()
        return f"I could not send that instruction: {message}"
    if replay:
        return "That instruction was already received; no duplicate command was sent."
    if not submitted:
        return "Preview received; no robot command was sent."
    return "Instruction received and sent to LingTu."


def _estop_state(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        safety = getattr(gw, "_safety", None)
        mode = str(getattr(gw, "_mode", "") or "").strip().lower()
    active = safety_stop_active(safety) or mode == "estop"
    timestamp: Any = None
    if isinstance(safety, Mapping):
        timestamp = safety.get("timestamp", safety.get("_ts", safety.get("ts")))
    if timestamp is None:
        timestamp = getattr(gw, "_askme_estop_timestamp", None)
    try:
        timestamp = float(timestamp)
    except (TypeError, ValueError):
        timestamp = time.time()
    if not math.isfinite(timestamp):
        timestamp = time.time()
    return {"active": active, "enabled": active, "timestamp": timestamp}


def register_voice_routes(app, gw) -> None:
    """Register AskMe compatibility routes."""

    command_service = ControlCommandService(gw)

    @app.post(
        "/api/v1/voice/turns",
        summary="Submit an AskMe voice turn",
        response_model=VoiceTurnResponse,
        responses={409: {"model": GatewayErrorResponse}},
    )
    def post_voice_turn(
        body: VoiceTurnRequest,
        x_request_id: str | None = Header(
            default=None,
            alias="X-Request-Id",
            max_length=128,
        ),
        idempotency_key: str | None = Header(
            default=None,
            alias="Idempotency-Key",
            max_length=128,
        ),
        x_operator_id: str | None = Header(
            default=None,
            alias="X-Operator-Id",
            max_length=128,
        ),
    ):
        request_id = _clean_header(x_request_id) or _clean_header(idempotency_key) or body.request_id
        header_operator = _clean_header(x_operator_id)
        if header_operator and body.operator_id and header_operator != body.operator_id:
            raise HTTPException(
                status_code=422,
                detail="X-Operator-Id does not match body.operator_id",
            )
        operator_id = header_operator or body.operator_id
        client_id = operator_id or body.client_id or "unknown"
        command_body = _validated_update(
            body,
            {
                "request_id": request_id,
                "client_id": client_id,
                "operator_id": operator_id,
            },
        )

        def _apply() -> dict[str, Any]:
            if command_body.submit:
                gw.instruction.publish(command_body.text)
            return {
                "accepted": True,
                "status": "submitted" if command_body.submit else "preview",
                "stage": "submitted" if command_body.submit else "preview",
                "execution_confirmed": False,
                "instruction": command_body.text,
                "submitted": command_body.submit,
            }

        result = command_service.run_motion_guarded_command(
            "instruction",
            command_body,
            _apply,
        )
        payload = _response_payload(result)
        if (
            isinstance(result, JSONResponse)
            and payload.get("error") == "idempotency_conflict"
        ):
            return result
        command = payload.get("command")
        command = command if isinstance(command, dict) else {}
        accepted = payload.get("ok") is True
        replay = bool(command.get("replay", False))
        submitted = payload.get("submitted") is True if accepted else False
        status = str(payload.get("status") or ("rejected" if not accepted else "submitted"))
        return {
            "handled": True,
            "turn": {
                "action_type": "runtime",
                "spoken_reply": _spoken_reply(
                    payload,
                    submitted=submitted,
                    replay=replay,
                ),
                "text": command_body.text,
                "status": status,
                "submitted": submitted,
                "accepted": accepted,
                "replay": replay,
                "request_id": command.get("request_id", request_id),
                "client_id": command.get("client_id", client_id),
                "operator_id": command_body.operator_id,
                "session_id": command_body.session_id,
                "channel": command_body.channel,
                "robot_id": command_body.robot_id,
                "site_id": command_body.site_id,
                "metadata": command_body.metadata,
                "command": command or None,
                "reason": payload.get("error") or payload.get("message"),
            },
        }

    @app.get(
        "/api/v1/safety/modes/estop",
        summary="Read AskMe-compatible emergency-stop state",
        response_model=SafetyEstopResponse,
    )
    def get_safety_estop():
        state = _estop_state(gw)
        return {
            **state,
            "accepted": True,
            "message": ("Emergency stop is active." if state["active"] else "Emergency stop is clear."),
        }

    @app.post(
        "/api/v1/safety/modes/estop",
        summary="Activate the emergency stop through the AskMe compatibility route",
        response_model=SafetyEstopResponse,
        responses={409: {"model": SafetyEstopResponse}},
    )
    def post_safety_estop(
        body: SafetyEstopRequest,
        x_request_id: str | None = Header(
            default=None,
            alias="X-Request-Id",
            max_length=128,
        ),
        idempotency_key: str | None = Header(
            default=None,
            alias="Idempotency-Key",
            max_length=128,
        ),
        x_operator_id: str | None = Header(
            default=None,
            alias="X-Operator-Id",
            max_length=128,
        ),
    ):
        request_id = _clean_header(x_request_id) or _clean_header(idempotency_key) or body.request_id
        client_id = _clean_header(x_operator_id) or body.client_id or "unknown"
        command_body = _validated_update(body, {"request_id": request_id, "client_id": client_id})
        if not command_body.enabled:
            return JSONResponse(
                status_code=409,
                content={
                    **_estop_state(gw),
                    "accepted": False,
                    "replay": False,
                    "request_id": request_id,
                    "client_id": client_id,
                    "message": "This compatibility route can activate E-STOP but cannot clear it.",
                },
            )

        def _activate() -> dict[str, Any]:
            wrote_native = native_control.estop(
                gw,
                "askme_compat_estop",
                request_id=request_id,
            )
            endpoint_only = bool(getattr(gw, "_teleop_dds_enabled", False))
            if not wrote_native:
                endpoint_only = endpoint_only or native_control.endpoint_only_enabled(gw)
                if endpoint_only:
                    raise CommandBoundaryError("native emergency-stop command boundary is unavailable")
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            timestamp = time.time()
            with gw._state_lock:
                gw._mode = "estop"
                gw._askme_estop_timestamp = timestamp
            return {
                "active": True,
                "enabled": True,
                "timestamp": timestamp,
                "accepted": True,
                "stage": "native_acknowledged" if wrote_native else "local_published",
                "control_boundary": "native_estop" if wrote_native else "local_compat",
                "message": "Emergency stop activated.",
            }

        try:
            payload = gw._run_control_command(
                "safety_estop",
                command_body,
                _activate,
            )
        except (CommandBoundaryError, ValueError) as exc:
            return JSONResponse(
                status_code=409,
                content={
                    **_estop_state(gw),
                    "accepted": False,
                    "replay": False,
                    "request_id": request_id,
                    "client_id": client_id,
                    "message": str(exc),
                },
            )
        if isinstance(payload, JSONResponse):
            return payload
        command = payload.get("command")
        command = command if isinstance(command, dict) else {}
        return {
            "active": bool(payload.get("active", True)),
            "enabled": bool(payload.get("enabled", True)),
            "timestamp": float(payload.get("timestamp", time.time())),
            "accepted": True,
            "replay": bool(command.get("replay", False)),
            "request_id": command.get("request_id", request_id),
            "client_id": command.get("client_id", client_id),
            "control_boundary": payload.get("control_boundary"),
            "message": str(payload.get("message") or "Emergency stop activated."),
            "command": command or None,
        }


__all__ = ["register_voice_routes"]
