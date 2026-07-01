"""Gateway status helpers for monitor bots.

This module intentionally uses only the Python standard library so monitor
entrypoints can run without ROS2, rclpy, or robot-side Python overlays.
"""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from typing import Any


def _get_json(gateway_url: str, path: str, timeout_sec: float = 3.0) -> tuple[dict[str, Any], str | None]:
    url = f"{gateway_url.rstrip('/')}/{path.lstrip('/')}"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            payload = json.loads(resp.read(2_000_000).decode("utf-8", "replace") or "{}")
        return payload if isinstance(payload, dict) else {}, None
    except urllib.error.HTTPError as exc:
        return {}, f"http {exc.code}: {exc.reason}"
    except Exception as exc:
        return {}, repr(exc)


def collect_gateway_status(gateway_url: str, timeout_sec: float = 3.0) -> dict[str, Any]:
    """Collect a compact operator-facing status snapshot from Gateway REST."""
    nav, nav_error = _get_json(gateway_url, "/api/v1/navigation/status", timeout_sec)
    health, health_error = _get_json(gateway_url, "/api/v1/health", timeout_sec)
    loc, loc_error = _get_json(gateway_url, "/api/v1/localization/status", timeout_sec)
    state, state_error = _get_json(gateway_url, "/api/v1/state", timeout_sec)

    mission = nav.get("mission") if isinstance(nav.get("mission"), dict) else {}
    readiness = nav.get("readiness") if isinstance(nav.get("readiness"), dict) else {}
    control = nav.get("control") if isinstance(nav.get("control"), dict) else {}
    slam = (health.get("sensors") or {}).get("slam") if isinstance(health.get("sensors"), dict) else {}
    odom = state.get("odometry") if isinstance(state.get("odometry"), dict) else {}

    blockers = readiness.get("blockers") or nav.get("reason_codes") or []
    if not isinstance(blockers, list):
        blockers = [str(blockers)]

    active_source = control.get("active_cmd_source") or control.get("active_source") or "none"
    if isinstance(active_source, dict):
        active_source = active_source.get("name") or active_source.get("source") or active_source.get("owner") or "none"

    errors = {
        "navigation": nav_error,
        "health": health_error,
        "localization": loc_error,
        "state": state_error,
    }
    errors = {key: value for key, value in errors.items() if value}

    return {
        "timestamp": time.time(),
        "gateway": gateway_url.rstrip("/"),
        "state": nav.get("state") or mission.get("state") or "UNKNOWN",
        "target": mission.get("target") or nav.get("target") or "None",
        "distance": mission.get("distance_to_goal") or nav.get("distance_to_goal") or 0.0,
        "elapsed_time": mission.get("elapsed_time") or nav.get("elapsed_time") or 0.0,
        "success_rate": nav.get("success_rate") or health.get("success_rate") or 0.0,
        "can_accept_goal": nav.get("can_accept_goal", readiness.get("can_execute_autonomy")),
        "active_cmd_source": active_source,
        "blockers": blockers,
        "health_status": health.get("status") or "unknown",
        "slam_status": (slam or {}).get("status") or health.get("slam_status") or "unknown",
        "slam_hz": (slam or {}).get("hz") or health.get("slam_hz") or 0.0,
        "localization_state": loc.get("state") or loc.get("reported_state") or "unknown",
        "localization_backend": loc.get("backend") or loc.get("localization_backend") or "unknown",
        "pose": {
            "x": odom.get("x"),
            "y": odom.get("y"),
            "yaw": odom.get("yaw"),
        },
        "errors": errors,
    }


def format_status(status: dict[str, Any], *, title: str = "LingTu Status Update") -> str:
    """Format a Gateway status snapshot for chat apps."""
    blockers = status.get("blockers") or []
    errors = status.get("errors") or {}
    pose = status.get("pose") if isinstance(status.get("pose"), dict) else {}

    def _float(value: Any, digits: int = 2) -> str:
        try:
            return f"{float(value):.{digits}f}"
        except Exception:
            return "-"

    return (
        f"{title}\n\n"
        f"State:    {status.get('state', 'UNKNOWN')}\n"
        f"Health:   {status.get('health_status', 'unknown')}\n"
        f"SLAM:     {status.get('slam_status', 'unknown')} @ {_float(status.get('slam_hz'), 1)} Hz\n"
        f"Loc:      {status.get('localization_state', 'unknown')} ({status.get('localization_backend', 'unknown')})\n"
        f"Target:   {status.get('target', 'None')}\n"
        f"Distance: {_float(status.get('distance'))} m\n"
        f"Pose:     x={_float(pose.get('x'))} y={_float(pose.get('y'))} yaw={_float(pose.get('yaw'))}\n"
        f"Control:  {status.get('active_cmd_source', 'none')}\n"
        f"Ready:    {status.get('can_accept_goal')}\n"
        f"Blockers: {', '.join(map(str, blockers)) if blockers else '-'}\n"
        f"Errors:   {', '.join(f'{k}={v}' for k, v in errors.items()) if errors else '-'}"
    )
