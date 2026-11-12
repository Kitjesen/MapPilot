"""Project known robot facts without inferring health or hardware-stop state."""

from gateway.services.native_control import status_is_fresh
from gateway.services.native_status import read_navigation_status
from gateway.services.readiness import build_readiness_snapshot

from .generated import system_pb2 as pb


def robot_status(gateway):
    readiness, _ = build_readiness_snapshot(gateway)
    result = pb.RobotStatus(
        mode=getattr(gateway, "_mode", "") or "",
        reason="; ".join(readiness.get("reasons") or []),
    )
    if isinstance(readiness.get("ready"), bool):
        result.ready = readiness["ready"]
    native = read_navigation_status()
    if status_is_fresh(native):
        authority = native.get("control_authority")
        if isinstance(authority, dict) and isinstance(authority.get("estop_latched"), bool):
            result.software_emergency_stop = authority["estop_latched"]
    return result
