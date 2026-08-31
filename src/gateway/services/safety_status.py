"""Gateway helpers for interpreting safety state snapshots."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from typing import Any

SAFETY_STOP_BLOCKER = "safety_stop"

def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    if is_dataclass(value):
        try:
            return asdict(value)
        except Exception:
            return {}
    if hasattr(value, "model_dump"):
        try:
            data = value.model_dump()
            return dict(data) if isinstance(data, Mapping) else {}
        except Exception:
            return {}
    return {}


def normalized_safety_level(safety: Any) -> int | str | None:
    """Derive the public safety level from the native navigation state."""
    raw = _mapping(safety)
    if not raw:
        return None
    return 2 if str(raw.get("authority") or "").strip().lower() == "estop" else 0


def safety_stop_active(safety: Any) -> bool:
    """Return True only for safety states that must block motion commands."""
    level = normalized_safety_level(safety)
    return level == 2


def safety_clear_for_motion(safety: Any) -> bool:
    return not safety_stop_active(safety)


def safety_summary(safety: Any) -> dict[str, Any]:
    raw = _mapping(safety)
    level = normalized_safety_level(safety)
    stop_active = safety_stop_active(safety)
    return {
        "level": level,
        "ok": level in (None, 0),
        "stop_active": stop_active,
        "motion_allowed": not stop_active,
        "raw": raw,
    }
