"""Shared motion-stop operations for external transports."""

from collections.abc import Callable
from typing import Any

from gateway.navigation.commands import CommandBoundaryError
from gateway.schemas import StopRequest
from gateway.services.control_commands import ControlCommandService
from gateway.services.native_control import clear_estop, estop
from gateway.services.native_control import stop as native_stop


def _run(
    gw: Any,
    name: str,
    body: StopRequest | None,
    publish: Callable[[], dict[str, Any]],
    message: str,
):
    try:
        return gw._run_control_command(name, body, publish)
    except CommandBoundaryError as exc:
        reason = str(exc)
        commands = ControlCommandService(gw)
        return commands.rejected_response(
            name,
            body,
            error="native_command_rejected",
            message=message,
            detail=commands.command_error_detail(
                reason_code="native_command_rejected",
                reason=reason,
                source="native_navigation_command_ack",
                blockers=[reason],
            ),
        )


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

    return _run(gw, "stop", body, publish, "Native navigation endpoint did not acknowledge the stop command.")


def stop_motion(gw: Any, body: StopRequest):
    def publish() -> dict[str, Any]:
        wrote_dds = native_stop(gw, "operator_stop", request_id=body.request_id)
        if not wrote_dds:
            raise CommandBoundaryError("native motion stop boundary is unavailable")
        return {
            "accepted": True,
            "status": "stop_requested",
            "stage": "native_acknowledged",
            "dds": wrote_dds,
            "native_control": "stop_motion",
        }

    return _run(gw, "motion_stop", body, publish, "Native navigation endpoint did not acknowledge the motion stop.")


def clear_emergency_stop(gw: Any, body: StopRequest | None = None):
    def publish() -> dict[str, Any]:
        request_id = body.request_id if body is not None else None
        wrote_dds = clear_estop(gw, "operator_reset", request_id=request_id)
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

    return _run(gw, "estop_reset", body, publish, "Native navigation endpoint did not acknowledge emergency-stop reset.")
