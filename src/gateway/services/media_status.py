"""Normalize media and camera runtime status for App/Web clients."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any

from runtime.contracts import CAMERA_ROLE

MEDIA_STATUS_SCHEMA_VERSION = 1
CAMERA_STREAM_STALE_MS = 5000.0


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _num(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _find_module(gw: Any, token: str) -> Any | None:
    modules = getattr(gw, "_all_modules", {}) or {}
    token_l = token.lower()
    for name, module in modules.items():
        if token_l in str(name).lower():
            return module
        if token_l in module.__class__.__name__.lower():
            return module
    return None


def _find_camera(gw: Any) -> Any | None:
    modules = getattr(gw, "_all_modules", {}) or {}
    return modules.get(CAMERA_ROLE)


def _safe_health(module: Any) -> tuple[dict[str, Any], str | None]:
    if module is None:
        return {}, None
    health = getattr(module, "health", None)
    if not callable(health):
        return {}, None
    try:
        raw = health()
    except Exception as exc:
        return {}, str(exc)
    return _mapping(raw), None


def _port(summary: Mapping[str, Any], group: str, name: str) -> dict[str, Any]:
    return _mapping(_mapping(summary.get(group)).get(name))


def _stale_ms(port: Mapping[str, Any]) -> float | None:
    value = port.get("stale_ms")
    if value is None:
        return None
    stale = _num(value, -1.0)
    return None if stale < 0.0 else stale


def _camera_status_from_health(health: Mapping[str, Any]) -> dict[str, Any]:
    color = _port(health, "ports_out", "color_image")
    depth = _port(health, "ports_out", "depth_image")
    info = _port(health, "ports_out", "camera_info")

    color_frames = _int(color.get("msg_count"))
    depth_frames = _int(depth.get("msg_count"))
    info_frames = _int(info.get("msg_count"))
    color_rate = round(_num(color.get("rate_hz")), 1)
    depth_rate = round(_num(depth.get("rate_hz")), 1)
    color_stale_ms = _stale_ms(color)
    depth_stale_ms = _stale_ms(depth)

    color_fresh = color_frames > 0 and color_stale_ms is not None and color_stale_ms <= CAMERA_STREAM_STALE_MS
    if color_fresh:
        status = "streaming"
        reason = None
        available = True
    elif color_frames > 0:
        status = "stale"
        reason = "camera_frames_stale"
        available = False
    else:
        status = "idle"
        reason = "no_color_frames"
        available = False

    return {
        "schema_version": MEDIA_STATUS_SCHEMA_VERSION,
        "available": available,
        "status": status,
        "reason": reason,
        "backend": health.get("backend"),
        "fps": color_rate,
        "frames": color_frames,
        "color": {
            "frames": color_frames,
            "fps": color_rate,
            "stale_ms": color_stale_ms,
        },
        "depth": {
            "frames": depth_frames,
            "fps": depth_rate,
            "stale_ms": depth_stale_ms,
        },
        "camera_info": {
            "frames": info_frames,
            "active_topic": health.get("camera_info_active_topic"),
            "preferred_topic": health.get("camera_info_preferred_topic"),
            "topics": list(health.get("camera_info_topics") or []),
        },
        "reconnect_count": _int(health.get("reconnect_count")),
        "service_recovery_allowed": bool(health.get("service_recovery_allowed", False)),
        "service_recovery_suppressed": bool(health.get("service_recovery_suppressed", False)),
    }


def _jpeg_status(gw: Any) -> dict[str, Any]:
    jpeg_lock = getattr(gw, "_jpeg_lock", None) or _NullLock()
    with jpeg_lock:
        frame = getattr(gw, "_latest_jpeg", None)
        seq = _int(getattr(gw, "_latest_jpeg_seq", 0))
    return {
        "cached": bool(frame),
        "seq": seq,
        "bytes": len(frame) if isinstance(frame, (bytes, bytearray)) else 0,
    }


def _teleop_stream_clients(gw: Any) -> int:
    teleop = _find_module(gw, "Teleop")
    teleop_health, _ = _safe_health(teleop)
    return _int(teleop_health.get("stream_clients"))


def build_camera_status(gw: Any) -> dict[str, Any]:
    """Return a stable camera status without probing systemd, ROS, or hardware."""
    camera = _find_camera(gw)
    if camera is None:
        return {
            "schema_version": MEDIA_STATUS_SCHEMA_VERSION,
            "role": CAMERA_ROLE,
            "available": False,
            "status": "not_loaded",
            "reason": "camera_not_loaded",
            "fps": 0.0,
            "frames": 0,
            "color": {"frames": 0, "fps": 0.0, "stale_ms": None},
            "depth": {"frames": 0, "fps": 0.0, "stale_ms": None},
            "camera_info": {
                "frames": 0,
                "active_topic": None,
                "preferred_topic": None,
                "topics": [],
            },
            "reconnect_count": 0,
            "service_recovery_allowed": False,
            "service_recovery_suppressed": False,
            "jpeg": _jpeg_status(gw),
            "teleop_stream_clients": _teleop_stream_clients(gw),
            "ts": time.time(),
        }

    health, error = _safe_health(camera)
    if error is not None:
        status = {
            "schema_version": MEDIA_STATUS_SCHEMA_VERSION,
            "role": CAMERA_ROLE,
            "available": False,
            "status": "error",
            "reason": "camera_health_error",
            "error": error[:160],
            "fps": 0.0,
            "frames": 0,
            "color": {"frames": 0, "fps": 0.0, "stale_ms": None},
            "depth": {"frames": 0, "fps": 0.0, "stale_ms": None},
            "camera_info": {
                "frames": 0,
                "active_topic": None,
                "preferred_topic": None,
                "topics": [],
            },
            "reconnect_count": 0,
            "service_recovery_allowed": False,
            "service_recovery_suppressed": False,
        }
    else:
        status = _camera_status_from_health(health)

    status.setdefault("role", CAMERA_ROLE)
    status["jpeg"] = _jpeg_status(gw)
    status["teleop_stream_clients"] = _teleop_stream_clients(gw)
    status["ts"] = time.time()
    return status


def build_media_status(gw: Any) -> dict[str, Any]:
    return {
        "schema_version": MEDIA_STATUS_SCHEMA_VERSION,
        "camera": build_camera_status(gw),
    }


class _NullLock:
    def __enter__(self) -> _NullLock:
        return self

    def __exit__(self, *args: Any) -> None:
        return None
