"""Control command gating and receipts for Gateway routes."""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from itertools import pairwise
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import PlanPreviewRequest
from gateway.services.command_boundary import CommandBoundaryError, navigation_commands
from gateway.services.safety_status import safety_stop_active, safety_summary
from runtime.runtime_interface import map_frame_id

CONTROL_MAP_FRAME_ID = map_frame_id()


class ControlCommandService:
    """Keep motion-command safety gates out of HTTP route handlers."""

    def __init__(self, gateway: Any) -> None:
        self._gw = gateway

    def rejected_response(
        self,
        command: str,
        body: Any,
        *,
        error: str,
        message: str,
        detail: dict[str, Any],
        status_code: int = 409,
    ) -> JSONResponse:
        content = {
            "schema_version": 1,
            "ok": False,
            "error": error,
            "message": message,
            "command": {
                "name": command,
                "task_id": getattr(body, "task_id", None),
                "request_id": getattr(body, "request_id", None),
                "client_id": getattr(body, "client_id", "unknown"),
                "accepted": False,
                "replay": False,
                "ts": time.time(),
            },
            "detail": detail,
        }
        if hasattr(self._gw, "_publish_command_ack"):
            self._gw._publish_command_ack(content, status_code=status_code)
        return JSONResponse(status_code=status_code, content=content)

    def command_error_detail(
        self,
        *,
        reason_code: str,
        reason: str | None = None,
        source: str | None = None,
        path: str | None = None,
        blockers: list[Any] | None = None,
        advisories: list[Any] | None = None,
        **extra: Any,
    ) -> dict[str, Any]:
        detail: dict[str, Any] = {
            "reason_code": reason_code,
            "reason": reason or reason_code,
            "source": source,
            "path": path,
            "blockers": [str(item) for item in (blockers or [])],
            "advisories": [str(item) for item in (advisories or [])],
        }
        detail.update({key: value for key, value in extra.items() if value is not None})
        return detail

    def motion_safety_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        client_id = getattr(body, "client_id", "unknown")
        lease = getattr(self._gw, "_lease", None)
        if lease is not None and not lease.check(client_id):
            return self.rejected_response(
                command,
                body,
                error="control_lease",
                message="Another operator currently owns motion control.",
                detail=self.command_error_detail(
                    reason_code="control_lease",
                    reason="Another operator currently owns motion control.",
                    source="control_lease",
                    path="/api/v1/lease",
                    blockers=["control_lease"],
                    lease=lease.to_dict(),
                ),
                status_code=403,
            )
        try:
            with self._gw._state_lock:
                safety = getattr(self._gw, "_navigation_state", None)
        except Exception:
            safety = None

        if not safety_stop_active(safety):
            return None
        return self.rejected_response(
            command,
            body,
            error="safety_stop",
            message="Safety STOP is active; motion commands are not accepted.",
            detail=self.command_error_detail(
                reason_code="safety_stop",
                reason="Safety STOP is active; motion commands are not accepted.",
                source="safety",
                path="/api/v1/state",
                blockers=["safety_stop"],
                safety=safety_summary(safety),
            ),
        )

    def preview_plan(self, body: PlanPreviewRequest) -> dict[str, Any]:
        commands = navigation_commands(self._gw)
        operation = getattr(commands, "preview_plan", None)
        if not callable(operation):
            return self._unavailable(body)
        try:
            result = operation(float(body.x), float(body.y), float(body.z))
        except Exception as exc:
            return self._unavailable(body, error=str(exc))
        if not isinstance(result, Mapping):
            return self._unavailable(body, error="native preview returned an invalid response")

        ts = float(result.get("timestamp_s") or time.time())
        frame_id = str(result.get("frame_id") or body.frame_id)
        goal = _native_point(result.get("goal"), frame_id=frame_id, ts=ts)
        if goal is None:
            goal = _point_payload(body.x, body.y, body.z, frame_id=frame_id, ts=ts)
        path = [
            point
            for raw in (result.get("path") or [])
            if (point := _native_point(raw, frame_id=frame_id, ts=ts)) is not None
        ]
        feasible = bool(result.get("feasible", False))
        reason = str(result.get("reason") or "").strip()
        distance_m = sum(
            math.dist(
                (left["x"], left["y"], left["z"]),
                (right["x"], right["y"], right["z"]),
            )
            for left, right in pairwise(path)
        )
        return {
            "schema_version": 1,
            "ok": True,
            "feasible": feasible,
            "frame_id": frame_id,
            "start": (
                _native_point(result.get("start"), frame_id=frame_id, ts=ts)
                if result.get("start_valid") is True
                else None
            ),
            "goal": goal,
            "path": path,
            "count": len(path),
            "distance_m": distance_m if path else None,
            "plan_ms": result.get("elapsed_ms"),
            "planner": result.get("planner"),
            "source": "native_nav",
            "reasons": [reason] if reason and not feasible else [],
            "error": None,
            "ts": ts,
        }

    def run_planned_goal_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any] | JSONResponse:
        def _execute() -> dict[str, Any] | JSONResponse:
            for check in (
                self.motion_safety_rejection,
                self._goal_readiness_rejection,
                self._goal_map_identity_rejection,
            ):
                rejection = check(command, body)
                if rejection is not None:
                    return rejection
            try:
                return action()
            except CommandBoundaryError as exc:
                reason = str(exc)
                return self.rejected_response(
                    command,
                    body,
                    error="native_command_rejected",
                    message="Native navigation endpoint rejected the command.",
                    detail=self.command_error_detail(
                        reason_code="native_command_rejected",
                        reason=reason,
                        source="native_navigation_command_ack",
                        blockers=[reason],
                    ),
                )

        return self._gw._run_control_command(command, body, _execute)

    def run_motion_guarded_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
        *,
        success_status_code: int = 200,
    ) -> dict[str, Any] | JSONResponse:
        def _execute() -> dict[str, Any] | JSONResponse:
            rejection = self.motion_safety_rejection(command, body)
            if rejection is not None:
                return rejection
            try:
                return action()
            except CommandBoundaryError as exc:
                reason = str(exc)
                return self.rejected_response(
                    command,
                    body,
                    error="native_command_rejected",
                    message="Native navigation endpoint rejected the command.",
                    detail=self.command_error_detail(
                        reason_code="native_command_rejected",
                        reason=reason,
                        source="native_navigation_command_ack",
                        blockers=[reason],
                    ),
                )

        return self._gw._run_control_command(
            command, body, _execute, success_status_code=success_status_code
        )

    def _goal_readiness_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        from gateway.services.runtime_status import build_navigation_status

        status = build_navigation_status(self._gw)
        readiness = status.get("readiness", {})
        blockers = list(readiness.get("blockers") or [])
        if bool(status.get("can_accept_goal", False)) and not blockers:
            return None
        return self.rejected_response(
            command,
            body,
            error="navigation_not_ready",
            message="Navigation cannot accept a goal in the current state.",
            detail=self.command_error_detail(
                reason_code="navigation_not_ready",
                reason="Navigation cannot accept a goal in the current state.",
                source="gateway_readiness",
                path="/api/v1/navigation/status",
                blockers=blockers,
                advisories=list(readiness.get("advisories") or []),
                state=status.get("state"),
                has_odometry=status.get("has_odometry"),
                session_mode=getattr(self._gw, "_session_mode", None),
                localization=status.get("localization", {}),
            ),
        )

    def _goal_map_identity_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        metadata = getattr(body, "metadata", None)
        if not isinstance(metadata, dict):
            return None
        requested_map = str(metadata.get("map_name") or "").strip()
        if not requested_map:
            return None

        active_map = str(getattr(self._gw, "_session_map", None) or "").strip()
        if not active_map:
            active_map_name = getattr(self._gw, "_session_active_map_name", None)
            if callable(active_map_name):
                active_map = str(active_map_name() or "").strip()
        if active_map == requested_map:
            return None
        return self.rejected_response(
            command,
            body,
            error="active_map_mismatch",
            message="The selected map does not match the active navigation map.",
            detail=self.command_error_detail(
                reason_code="active_map_mismatch",
                reason="The selected map does not match the active navigation map.",
                source="goal_metadata",
                blockers=["active_map_mismatch"],
                requested_map=requested_map,
                active_map=active_map or None,
            ),
        )

    def _unavailable(
        self,
        body: PlanPreviewRequest,
        *,
        error: str | None = None,
    ) -> dict[str, Any]:
        ts = time.time()
        reason = str(error or "native navigation plan preview is unavailable").strip()
        return {
            "schema_version": 1,
            "ok": False,
            "feasible": False,
            "frame_id": body.frame_id,
            "start": None,
            "goal": _point_payload(
                body.x,
                body.y,
                body.z,
                ts=ts,
                frame_id=body.frame_id,
            ),
            "path": [],
            "count": 0,
            "distance_m": None,
            "plan_ms": None,
            "planner": None,
            "source": "native_nav",
            "reasons": [reason],
            "error": reason,
            "ts": ts,
        }


def _point_payload(
    x: float,
    y: float,
    z: float,
    *,
    ts: float,
    frame_id: str = CONTROL_MAP_FRAME_ID,
) -> dict[str, Any]:
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "frame_id": frame_id,
        "ts": ts,
        "metadata": {},
    }


def _native_point(
    value: Any,
    *,
    frame_id: str,
    ts: float,
) -> dict[str, Any] | None:
    if not isinstance(value, Mapping):
        return None
    try:
        return _point_payload(
            float(value["x"]),
            float(value["y"]),
            float(value.get("z", 0.0)),
            frame_id=frame_id,
            ts=ts,
        )
    except (KeyError, TypeError, ValueError):
        return None
