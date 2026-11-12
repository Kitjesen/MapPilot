"""Project current native fault facts without retaining a second fault history."""

import math
import os
import time

from gateway.services.native_control import status_is_fresh
from gateway.services.native_status import read_json_snapshot, read_navigation_status

from .generated import system_pb2 as pb


def _driver():
    path = os.environ.get("LINGTU_DRIVER_STATUS_FILE", "").strip() or "/dev/shm/lingtu/driver_status.json"
    value = read_json_snapshot(path)
    if value is None:
        return None
    try:
        max_age = float(os.environ.get("LINGTU_DRIVER_STATUS_MAX_AGE_S", "1.5") or "1.5")
        age = time.time() - float(value["stamp_s"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(max_age) or max_age < 0 or not -0.05 <= age <= max_age:
        return None
    return value


def _fault(source, code, message, severity):
    return pb.Fault(source=source, code=code, message=message, severity=severity, active=True)


def read():
    result = pb.FaultReport()
    driver = _driver() or {}
    connected = driver.get("connected") is True
    control = driver.get("control")
    critical = control.get("critical_fault") if isinstance(control, dict) else None
    control_available = connected and isinstance(critical, bool)
    if control_available:
        result.observed_sources.append("driver.control")
        if critical:
            result.faults.append(_fault(
                "driver.control", "critical_motor_fault", "Native driver reports a critical motor fault",
                pb.FAULT_SEVERITY_ERROR,
            ))
    else:
        result.unavailable_sources.append("driver.control")

    health = driver.get("health")
    if connected and isinstance(health, dict) and health.get("fresh") is True and isinstance(health.get("healthy"), bool):
        result.observed_sources.append("driver.health")
        reason = health.get("reason") if isinstance(health.get("reason"), str) else ""
        if not health["healthy"] and reason != "motors_disabled" and not (
            control_available and critical and reason == "critical_motor_fault"
        ):
            result.faults.append(_fault(
                "driver.health", reason or "unhealthy", reason or "Native driver reports unhealthy state",
                pb.FAULT_SEVERITY_ERROR,
            ))
    else:
        result.unavailable_sources.append("driver.health")

    navigation = read_navigation_status()
    loop = navigation.get("control_loop_health") if isinstance(navigation, dict) else None
    if status_is_fresh(navigation) and isinstance(loop, dict) and loop.get("ready") is True and isinstance(loop.get("healthy"), bool):
        result.observed_sources.append("navigation.control_loop")
        if not loop["healthy"]:
            reason = loop.get("reason") if isinstance(loop.get("reason"), str) else ""
            result.faults.append(_fault(
                "navigation.control_loop", reason or "unhealthy", reason or "Native control loop reports unhealthy state",
                pb.FAULT_SEVERITY_WARNING,
            ))
    else:
        result.unavailable_sources.append("navigation.control_loop")

    if any(fault.severity == pb.FAULT_SEVERITY_ERROR for fault in result.faults):
        result.health = pb.HEALTH_STATUS_ERROR
    elif result.faults:
        result.health = pb.HEALTH_STATUS_WARNING
    return result
