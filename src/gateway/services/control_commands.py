"""Control command gating and receipts for Gateway routes."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import PlanPreviewRequest
from gateway.services.safety_status import safety_stop_active, safety_summary
from runtime.adapters.native.navigation import NavigationClientError
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
                safety = getattr(self._gw, "_safety", None)
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

    def preview_navigation_plan(
        self,
        body: PlanPreviewRequest,
        *,
        ignore_blockers: set[str] | None = None,
        map_only: bool = False,
    ) -> dict[str, Any]:
        from gateway.services.runtime_status import build_navigation_status

        status = build_navigation_status(self._gw)
        readiness = status.get("readiness", {})
        ignored = ignore_blockers or set()
        blockers = [str(blocker) for blocker in (readiness.get("blockers") or []) if str(blocker) not in ignored]
        if blockers or not bool(status.get("has_odometry", False)):
            reasons = ["navigation_not_ready", *blockers]
            if not bool(status.get("has_odometry", False)):
                reasons.append("odometry_missing")
            return self._plan_preview_unavailable(
                body,
                reasons=reasons,
                source="gateway_readiness",
                ignored_blockers=sorted(ignored),
            )

        nav = (getattr(self._gw, "_all_modules", {}) or {}).get("nav.mission")
        if nav is None or not hasattr(nav, "preview_plan"):
            return self._plan_preview_unavailable(
                body,
                reasons=["nav_mission_unavailable"],
                source="gateway_modules",
            )

        try:
            constraints = getattr(body, "planner_constraints", {}) or {}
            try:
                return nav.preview_plan(
                    body.x,
                    body.y,
                    body.z,
                    map_only=map_only,
                    planner_constraints=constraints,
                )
            except TypeError:
                return nav.preview_plan(body.x, body.y, body.z)
        except Exception as exc:
            return self._plan_preview_unavailable(
                body,
                reasons=["planning_preview_failed"],
                source="gateway_exception",
                error=str(exc),
            )

    def run_planned_goal_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any] | JSONResponse:
        request_id = getattr(body, "request_id", None) if body is not None else None
        client_id = getattr(body, "client_id", None) if body is not None else None
        rejection = self.motion_safety_rejection(command, body)
        if rejection is not None:
            return rejection
        replay = self._gw._command_journal.replay(command, request_id)
        if replay is not None:
            if hasattr(self._gw, "_publish_command_ack"):
                self._gw._publish_command_ack(replay, status_code=200)
            return replay

        rejection = self._goal_readiness_rejection(command, body)
        if rejection is not None:
            return rejection
        rejection = self._goal_map_identity_rejection(command, body)
        if rejection is not None:
            return rejection
        rejection = self._goal_plan_preview_rejection(command, body)
        if rejection is not None:
            return rejection

        try:
            native_response = action()
        except NavigationClientError as exc:
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
        response = self._gw._command_journal.accept(
            command,
            request_id,
            client_id,
            native_response,
        )
        if hasattr(self._gw, "_publish_command_ack"):
            self._gw._publish_command_ack(response, status_code=200)
        return response

    def run_motion_guarded_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any] | JSONResponse:
        rejection = self.motion_safety_rejection(command, body)
        if rejection is not None:
            return rejection
        try:
            return self._gw._run_control_command(command, body, action)
        except NavigationClientError as exc:
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

    def _goal_plan_preview_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        try:
            preview_body = PlanPreviewRequest(
                x=body.x,
                y=body.y,
                z=body.z,
                frame_id=getattr(body, "frame_id", CONTROL_MAP_FRAME_ID),
                client_id=getattr(body, "client_id", "unknown"),
            )
        except Exception as exc:
            return self.rejected_response(
                command,
                body,
                error="navigation_plan_invalid",
                message="Navigation goal cannot be previewed.",
                detail=self.command_error_detail(
                    reason_code="navigation_plan_invalid",
                    reason="Navigation goal cannot be previewed.",
                    source="gateway_request",
                    error=str(exc),
                ),
            )

        preview = self.preview_navigation_plan(preview_body)
        if (
            bool(preview.get("ok", True))
            and bool(preview.get("feasible", False))
            and not _path_safety_blocks_motion(preview)
        ):
            return None
        return self.rejected_response(
            command,
            body,
            error="navigation_plan_infeasible",
            message="Navigation plan preview is not feasible.",
            detail=self.command_error_detail(
                reason_code="navigation_plan_infeasible",
                reason="Navigation plan preview is not feasible.",
                source=str(preview.get("source") or "navigation_preview"),
                path="/api/v1/navigation/plan",
                blockers=list(preview.get("reasons") or []),
                preview=preview,
            ),
        )

    def _plan_preview_unavailable(
        self,
        body: PlanPreviewRequest,
        *,
        reasons: list[str],
        source: str,
        error: str | None = None,
        ignored_blockers: list[str] | None = None,
    ) -> dict[str, Any]:
        ts = time.time()
        return {
            "schema_version": 1,
            "ok": error is None,
            "feasible": False,
            "frame_id": body.frame_id,
            "start": self._current_start_payload(ts=ts),
            "goal": _point_payload(
                body.x,
                body.y,
                body.z,
                ts=ts,
                frame_id=body.frame_id,
            ),
            "adjusted_goal": None,
            "path": [],
            "count": 0,
            "distance_m": None,
            "plan_ms": None,
            "planner": None,
            "selected_planner": None,
            "plan_safety_policy": None,
            "path_safety": None,
            "fallback_reason": "",
            "rejected_plans": [],
            "source": source,
            "reasons": list(dict.fromkeys(reasons)),
            "ignored_blockers": list(ignored_blockers or []),
            "error": error,
            "ts": ts,
        }

    def _current_start_payload(self, *, ts: float) -> dict[str, Any] | None:
        with self._gw._state_lock:
            odom = dict(self._gw._odom) if self._gw._odom else None
        if not odom:
            return None
        try:
            x = float(odom.get("x", 0.0))
            y = float(odom.get("y", 0.0))
            z = float(odom.get("z", 0.0))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (x, y, z)):
            return None
        frame_id = str(odom.get("frame_id") or odom.get("frame") or "").strip()
        if not frame_id:
            header = odom.get("header")
            if isinstance(header, dict):
                frame_id = str(header.get("frame_id") or header.get("frame") or "").strip()
        return _point_payload(x, y, z, ts=ts, frame_id=frame_id or CONTROL_MAP_FRAME_ID)


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


def _path_safety_blocks_motion(preview: dict[str, Any]) -> bool:
    policy = str(preview.get("plan_safety_policy") or "").lower()
    if policy != "reject":
        return False
    path_safety = preview.get("path_safety")
    if not isinstance(path_safety, dict):
        return False
    return path_safety.get("ok") is False
