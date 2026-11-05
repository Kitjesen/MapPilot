"""Shared emergency-stop operation for external transports."""

from typing import Any

from gateway.navigation.commands import CommandBoundaryError
from gateway.schemas import StopRequest
from gateway.services.control_commands import ControlCommandService
from gateway.services.native_control import estop


def stop(gw: Any, body: StopRequest | None = None):
    def publish() -> dict[str, Any]:
        request_id = body.request_id if body is not None else None
        wrote_dds = estop(gw, "rest_emergency_stop", request_id=request_id)
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
        return gw._run_control_command("stop", body, publish)
    except CommandBoundaryError as exc:
        reason = str(exc)
        commands = ControlCommandService(gw)
        return commands.rejected_response(
            "stop",
            body,
            error="native_command_rejected",
            message="Native navigation endpoint did not acknowledge the stop command.",
            detail=commands.command_error_detail(
                reason_code="native_command_rejected",
                reason=reason,
                source="native_navigation_command_ack",
                blockers=[reason],
            ),
        )
