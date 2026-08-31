"""Emergency-stop routes."""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from typing import Any

from fastapi import Header
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ValidationError

from gateway.schemas import SafetyEstopRequest, SafetyEstopResponse
from gateway.services import native_control
from gateway.services.command_boundary import CommandBoundaryError
from gateway.services.safety_status import safety_stop_active


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


def _estop_state(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        safety = getattr(gw, "_navigation_state", None)
        mode = str(getattr(gw, "_mode", "") or "").strip().lower()
    active = safety_stop_active(safety) or mode == "estop"
    timestamp: Any = None
    if isinstance(safety, Mapping):
        timestamp = safety.get("timestamp", safety.get("_ts", safety.get("ts")))
    if timestamp is None:
        timestamp = getattr(gw, "_estop_timestamp", None)
    try:
        timestamp = float(timestamp)
    except (TypeError, ValueError):
        timestamp = time.time()
    if not math.isfinite(timestamp):
        timestamp = time.time()
    return {"active": active, "enabled": active, "timestamp": timestamp}


def register_safety_routes(app, gw) -> None:
    @app.get(
        "/api/v1/safety/modes/estop",
        summary="Read emergency-stop state",
        response_model=SafetyEstopResponse,
    )
    def get_safety_estop():
        state = _estop_state(gw)
        return {
            **state,
            "accepted": True,
            "message": "Emergency stop is active." if state["active"] else "Emergency stop is clear.",
        }

    @app.post(
        "/api/v1/safety/modes/estop",
        summary="Activate emergency stop",
        response_model=SafetyEstopResponse,
        responses={409: {"model": SafetyEstopResponse}},
    )
    def post_safety_estop(
        body: SafetyEstopRequest,
        x_request_id: str | None = Header(default=None, alias="X-Request-Id", max_length=128),
        idempotency_key: str | None = Header(default=None, alias="Idempotency-Key", max_length=128),
        x_operator_id: str | None = Header(default=None, alias="X-Operator-Id", max_length=128),
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
                    "message": "This endpoint can activate E-STOP but cannot clear it.",
                },
            )

        def _activate() -> dict[str, Any]:
            if not native_control.estop(gw, "gateway_estop", request_id=request_id):
                raise CommandBoundaryError("native emergency-stop command boundary is unavailable")
            timestamp = time.time()
            with gw._state_lock:
                gw._mode = "estop"
                gw._estop_timestamp = timestamp
            return {
                "active": True,
                "enabled": True,
                "timestamp": timestamp,
                "accepted": True,
                "control_boundary": "native_estop",
                "message": "Emergency stop activated.",
            }

        try:
            payload = gw._run_control_command("safety_estop", command_body, _activate)
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


__all__ = ["register_safety_routes"]
