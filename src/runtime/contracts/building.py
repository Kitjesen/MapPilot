"""Transport-neutral building infrastructure observations."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class LiftState:
    """Normalized lift observation shared by drivers and navigation."""

    lift_id: str
    available: bool
    current_floor_id: str
    destination_floor_id: str
    door_state: str
    motion_state: str
    session_id: str
    stamp_s: float
