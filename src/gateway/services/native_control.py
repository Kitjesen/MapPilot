"""Mode-independent native motion control seam for external interfaces.

The JSON status snapshot in this module is telemetry only. Command delivery
authority comes solely from the configured typed native command boundary.
"""

from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Mapping
from typing import Any

from gateway.services.command_boundary import invoke_navigation_command
from gateway.services.teleop import resolve_native_command_boundary


def endpoint_only_enabled(owner: Any | None = None) -> bool:
    """Resolve the compiled command boundary."""

    command_output_mode = ""
    if owner is not None:
        command_output_mode = str(
            getattr(owner, "_compiled_command_output_mode", "") or ""
        ).strip()
        if not command_output_mode:
            modules = getattr(owner, "_all_modules", None)
            if isinstance(modules, dict):
                for module in modules.values():
                    command_output_mode = str(
                        getattr(module, "_compiled_command_output_mode", "") or ""
                    ).strip()
                    if command_output_mode:
                        break

    return resolve_native_command_boundary(
        command_output_mode=(
            command_output_mode
            or os.environ.get("LINGTU_COMMAND_OUTPUT_MODE", "")
        ),
    )


def read_status() -> dict[str, Any] | None:
    """Read a best-effort, non-authoritative endpoint telemetry snapshot."""
    path = os.environ.get("LINGTU_NAV_STATUS_FILE", "").strip() or ("/dev/shm/lingtu/nav_endpoint_status.json")
    try:
        with open(path, encoding="utf-8") as handle:
            payload = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def status_is_fresh(
    payload: Mapping[str, Any] | None,
    *,
    now_s: float | None = None,
) -> bool:
    """Return whether a telemetry snapshot is recent and time-valid."""

    if not isinstance(payload, Mapping):
        return False
    try:
        stamp_s = float(payload.get("stamp_s"))
    except (TypeError, ValueError):
        return False
    try:
        max_age_s = float(os.environ.get("LINGTU_NAV_STATUS_MAX_AGE_S", "1.0") or "1.0")
    except ValueError:
        return False
    if not math.isfinite(max_age_s) or max_age_s < 0.0:
        return False
    age_s = (time.time() if now_s is None else float(now_s)) - stamp_s
    return -0.05 <= age_s <= max_age_s


def teleop_active(payload: Mapping[str, Any] | None = None) -> bool:
    """Report native teleop activity from non-authoritative telemetry."""

    snapshot = payload if payload is not None else read_status()
    if not status_is_fresh(snapshot):
        return False
    control_mode = str(snapshot.get("control_mode") or "")
    authority = snapshot.get("control_authority")
    takeover_latched = bool(isinstance(authority, Mapping) and authority.get("operator_takeover_latched") is True)
    if control_mode not in {"teleop", "teleop_avoid"} and not (
        control_mode == "autonomy" and takeover_latched and snapshot.get("active_cmd_source") == "teleop"
    ):
        return False
    teleop = snapshot.get("teleop")
    return bool(isinstance(teleop, Mapping) and teleop.get("fresh") is True and teleop.get("published") is True)


def _deliver(owner: Any, method: str, **kwargs: Any) -> bool:
    return invoke_navigation_command(
        owner,
        method,
        required=endpoint_only_enabled(owner),
        **kwargs,
    )


def stop(owner: Any, reason: str = "stop", *, request_id: str | None = None) -> bool:
    """Clear native motion in every control mode without latching estop."""

    return _deliver(owner, "stop_motion", reason=reason or "stop", request_id=request_id)


def estop(owner: Any, reason: str = "estop", *, request_id: str | None = None) -> bool:
    """Latch the native software emergency stop."""

    return _deliver(owner, "estop", reason=reason or "estop", request_id=request_id)


def clear_estop(
    owner: Any,
    reason: str = "clear_estop",
    *,
    request_id: str | None = None,
) -> bool:
    """Explicitly release the native estop latch without restoring motion."""

    return _deliver(
        owner,
        "clear_estop",
        reason=reason or "clear_estop",
        request_id=request_id,
    )


def resume_autonomy(
    owner: Any,
    reason: str = "resume_autonomy",
    *,
    request_id: str | None = None,
) -> bool:
    """Release the native manual-takeover latch without restoring an old path."""

    return _deliver(
        owner,
        "resume_autonomy",
        reason=reason or "resume_autonomy",
        request_id=request_id,
    )
