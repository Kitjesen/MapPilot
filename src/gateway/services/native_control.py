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

from gateway.services.command_boundary import (
    CommandBoundaryError,
    invoke_navigation_command,
    navigation_commands,
)
from runtime.msgs import NavigationCommandKind


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

    mode = (
        command_output_mode
        or os.environ.get("LINGTU_COMMAND_OUTPUT_MODE", "")
    ).strip().lower()
    if mode == "endpoint_only":
        return True
    if mode in {"", "local_driver"}:
        return False
    raise ValueError(f"unsupported command_output_mode: {mode!r}")


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


def motion_resume_context(
    payload: Mapping[str, Any] | None = None,
    *,
    now_s: float | None = None,
) -> dict[str, Any]:
    """Describe what a native motion resume means without guessing from stale status."""

    snapshot = payload if payload is not None else read_status()
    unknown = {
        "status_fresh": False,
        "observed_control_mode": None,
        "resume_was_required": None,
        "goal_reissue_required": None,
        "fresh_operator_command_required": None,
    }
    if not status_is_fresh(snapshot, now_s=now_s):
        return unknown

    control_mode = str(snapshot.get("control_mode") or "").strip()
    if control_mode not in {"autonomy", "teleop", "teleop_avoid"}:
        return {**unknown, "status_fresh": True}

    authority = snapshot.get("control_authority")
    resume_required_value = (
        authority.get("resume_required") if isinstance(authority, Mapping) else None
    )
    resume_required = (
        resume_required_value if isinstance(resume_required_value, bool) else None
    )
    if control_mode == "autonomy":
        goal_reissue_required = resume_required
        fresh_operator_command_required = False
    else:
        goal_reissue_required = False
        fresh_operator_command_required = resume_required
    return {
        "status_fresh": True,
        "observed_control_mode": control_mode,
        "resume_was_required": resume_required,
        "goal_reissue_required": goal_reissue_required,
        "fresh_operator_command_required": fresh_operator_command_required,
    }


def motion_resume_result(
    delivery: Mapping[str, Any] | bool,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    """Prefer the correlated native ACK over a pre-command status observation."""

    result = {
        "status_fresh": context.get("status_fresh") is True,
        "observed_control_mode": context.get("observed_control_mode"),
        "resume_was_required": context.get("resume_was_required"),
        "goal_reissue_required": context.get("goal_reissue_required"),
        "fresh_operator_command_required": context.get(
            "fresh_operator_command_required"
        ),
    }
    if not isinstance(delivery, Mapping):
        return result

    reason = str(delivery.get("reason") or "")
    observed_mode = result["observed_control_mode"]
    if reason == "autonomy_already_ready":
        result.update(
            observed_control_mode="autonomy",
            resume_was_required=False,
            goal_reissue_required=False,
            fresh_operator_command_required=False,
        )
    elif reason == "autonomy_resume_ready_reissue_goal":
        result.update(
            observed_control_mode="autonomy",
            resume_was_required=True,
            goal_reissue_required=True,
            fresh_operator_command_required=False,
        )
    elif reason == "teleop_already_ready":
        result.update(
            observed_control_mode=(
                observed_mode
                if observed_mode in {"teleop", "teleop_avoid"}
                else None
            ),
            resume_was_required=False,
            goal_reissue_required=False,
            fresh_operator_command_required=False,
        )
    elif reason == "teleop_resume_ready_reassert_command":
        result.update(
            observed_control_mode=(
                observed_mode
                if observed_mode in {"teleop", "teleop_avoid"}
                else None
            ),
            resume_was_required=True,
            goal_reissue_required=False,
            fresh_operator_command_required=True,
        )
    result.update(
        native_reason=reason or None,
        native_request_id=str(delivery.get("request_id") or "") or None,
        native_endpoint_timestamp_s=delivery.get("endpoint_timestamp_s"),
    )
    return result


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


def resume_control(
    owner: Any,
    reason: str = "resume_control",
    *,
    request_id: str | None = None,
) -> dict[str, Any] | bool:
    """Release a native motion hold without replaying an old motion request."""

    commands = navigation_commands(owner)
    receipt_method = getattr(commands, "resume_autonomy_with_receipt", None)
    method = (
        "resume_autonomy_with_receipt"
        if callable(receipt_method)
        else "resume_autonomy"
    )
    result = invoke_navigation_command(
        owner,
        method,
        required=endpoint_only_enabled(owner),
        accept_receipt=method == "resume_autonomy_with_receipt",
        reason=reason or "resume_control",
        request_id=request_id,
    )
    if not isinstance(result, Mapping):
        return result

    kind = result.get("kind")
    if isinstance(kind, bool) or kind != int(NavigationCommandKind.RESUME_AUTONOMY):
        raise CommandBoundaryError("native motion resume returned the wrong command kind")
    if str(result.get("task_id") or ""):
        raise CommandBoundaryError("native motion resume unexpectedly addressed a task")
    returned_request_id = str(result.get("request_id") or "")
    expected_request_id = str(request_id or "")
    if not returned_request_id:
        raise CommandBoundaryError("native motion resume returned no request_id")
    if expected_request_id and not (
        returned_request_id == expected_request_id
        or returned_request_id.startswith(f"{expected_request_id}-clock-retry-")
    ):
        raise CommandBoundaryError("native motion resume returned the wrong request_id")
    reason_value = result.get("reason")
    if not isinstance(reason_value, str) or not reason_value:
        raise CommandBoundaryError("native motion resume returned no reason")
    try:
        endpoint_timestamp_s = float(result.get("endpoint_timestamp_s"))
    except (TypeError, ValueError) as exc:
        raise CommandBoundaryError(
            "native motion resume returned an invalid endpoint timestamp"
        ) from exc
    if not math.isfinite(endpoint_timestamp_s) or endpoint_timestamp_s < 0.0:
        raise CommandBoundaryError(
            "native motion resume returned an invalid endpoint timestamp"
        )
    return dict(result)
