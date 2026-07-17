"""Small mission policy helpers.

Keep policy decisions out of the Navigation control flow where possible.
"""

from __future__ import annotations


def should_use_direct_goal_fallback(
    exc: Exception,
    *,
    allow: bool,
    on_planner_failure: bool,
) -> bool:
    if not allow or not on_planner_failure:
        return False
    text = str(exc).lower()
    blocked = (
        "artifact gate",
        "path_safety",
        "path safety",
        "unsafe",
        "collision",
        "frame",
    )
    if any(term in text for term in blocked):
        return False
    return "empty path" in text or "no path" in text
