"""Body commands share the command journal and native motion arbitration."""

from collections.abc import Mapping

from fastapi import HTTPException

from gateway.navigation.commands import CommandBoundaryError, navigation_commands
from gateway.services.control_commands import ControlCommandService


def change(gateway, body, action):
    unknown_code = "posture_ack_unknown" if action in {"stand", "sit"} else f"{action}_ack_unknown"

    def publish():
        operation = getattr(navigation_commands(gateway), action, None)
        if not callable(operation):
            raise CommandBoundaryError(f"native {action} capability is unavailable")
        try:
            arguments = {"reason": f"operator_{action}", "request_id": body.request_id}
            if action == "switch_profile":
                arguments["name"] = body.name
            result = operation(**arguments)
        except Exception as exc:
            # Missing ACK does not prove the driver rejected the submitted command.
            raise HTTPException(503, detail={
                "reason_code": unknown_code, "reason": str(exc),
                "request_id": body.request_id,
            }) from exc
        if not isinstance(result, Mapping) or not isinstance(result.get("accepted"), bool):
            raise HTTPException(503, detail={"reason_code": unknown_code, "reason": "invalid native receipt"})
        if result.get("reason") in {
            "posture_ack_timeout", "posture_cancel_requested", "posture_action_ack_unknown", "posture_publish_failed",
        }:
            raise HTTPException(503, detail={
                "reason_code": unknown_code, "reason": result["reason"],
                "request_id": body.request_id, "native_ack": dict(result),
            })
        if result.get("accepted") is not True:
            raise CommandBoundaryError(str(result.get("reason") or "body command rejected"))
        return {
            "accepted": True,
            "stage": {
                "stand": "posture_requested", "sit": "posture_requested",
                "enable_motors": "motor_power_acknowledged", "disable_motors": "motor_power_acknowledged",
                "switch_profile": "profile_switch_acknowledged",
            }[action],
            "execution_confirmed": False,
            "native_ack": dict(result),
        }

    return ControlCommandService(gateway).run_motion_guarded_command(action, body, publish)
