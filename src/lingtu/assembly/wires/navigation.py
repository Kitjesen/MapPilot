"""Host navigation command wires."""

from __future__ import annotations

from runtime.wiring import WireSpec


def navigation_support_specs() -> tuple[WireSpec, ...]:
    """Connect the AI skill adapter to the canonical native command service."""
    return (
        WireSpec("nav.skills", "goal_command", "nav.goals", "goal_command"),
        WireSpec("nav.goals", "goal_status", "nav.skills", "goal_status"),
    )
