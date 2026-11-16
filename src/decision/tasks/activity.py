"""Shared activity labels for conversation and robot task events."""

from __future__ import annotations

from typing import Literal

ActivityState = Literal[
    "started",
    "running",
    "awaiting",
    "completed",
    "failed",
    "cancelled",
]

ACTIVITY_STATES = frozenset(
    {"started", "running", "awaiting", "completed", "failed", "cancelled"}
)


def activity_type(name: str, *, role: str = "tool") -> str:
    """Classify an event without changing who owns its execution."""

    if role == "assistant":
        return "model"
    if role != "tool":
        return "other"
    if name in {"follow_person", "stop_servo"}:
        return "tracking"
    if name == "start_inspection_route":
        return "inspection"
    if name.startswith("navigate_") or name == "find_object":
        return "navigation"
    return "tool"


def normalize_activity_state(value: object, *, awaiting: bool = False) -> ActivityState:
    """Convert provider, task, and legacy UI states to one display state."""

    if awaiting or value in {"awaiting", "waiting", "pending_approval"}:
        return "awaiting"
    if value in {"failed", "error"}:
        return "failed"
    if value in {"cancelled", "canceled", "interrupted"}:
        return "cancelled"
    if value in {"completed", "done", "success", "succeeded"}:
        return "completed"
    if value in {"running", "streaming"}:
        return "running"
    return "started"

