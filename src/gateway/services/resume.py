"""Resume the native motion hold through the shared command journal."""

from __future__ import annotations

from typing import Any

from fastapi import HTTPException
from fastapi.responses import JSONResponse

from gateway.navigation.commands import CommandBoundaryError
from gateway.schemas import StopRequest
from gateway.services.control_commands import ControlCommandService
from gateway.services.native_control import (
    motion_resume_context as native_motion_resume_context,
)
from gateway.services.native_control import (
    motion_resume_result as native_motion_resume_result,
)
from gateway.services.native_control import (
    resume_control as native_resume_control,
)


def resume(gw: Any, body: StopRequest | None = None) -> dict[str, Any] | JSONResponse:
    """Release motion hold after native admission, preserving request replay."""
    command_service = ControlCommandService(gw)
    client_id = body.client_id if body is not None else "unknown"
    if not gw._lease.check(client_id):
        return command_service.rejected_response(
            "navigation_resume",
            body,
            error="control_lease",
            message="Only the active control owner may resume motion control.",
            detail=command_service.command_error_detail(
                reason_code="control_lease",
                reason="Only the active control owner may resume motion control.",
                source="control_lease",
                path="/api/v1/lease",
                blockers=["control_lease"],
                lease=gw._lease.to_dict(),
            ),
            status_code=403,
        )

    def _publish() -> dict[str, Any]:
        request_id = body.request_id if body is not None else None
        resume_context = native_motion_resume_context()
        resume_delivery = native_resume_control(
            gw,
            "operator_resume",
            request_id=request_id,
        )
        if resume_delivery is False:
            raise CommandBoundaryError("native motion resume boundary is unavailable")
        return {
            "accepted": True,
            "status": "motion_resume_acknowledged",
            "stage": "native_acknowledged",
            "dds": True,
            **native_motion_resume_result(resume_delivery, resume_context),
            "previous_motion_restored": False,
        }

    try:
        return gw._run_control_command("navigation_resume", body, _publish)
    except CommandBoundaryError as exc:
        reason = str(exc)
        if exc.receipt is None or exc.receipt.get("accepted") is not False:
            raise HTTPException(503, detail={
                "reason_code": "motion_resume_ack_unknown",
                "request_id": body.request_id if body is not None else None,
                "reason": reason,
            }) from exc
        return command_service.rejected_response(
            "navigation_resume",
            body,
            error="native_command_rejected",
            message=f"Native motion resume was rejected: {reason}",
            detail=command_service.command_error_detail(
                reason_code="native_command_rejected",
                reason=reason,
                source="native_navigation_command_ack",
                blockers=[reason],
            ),
        )
