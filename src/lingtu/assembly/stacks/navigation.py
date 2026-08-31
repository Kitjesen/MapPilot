"""Host-side navigation command skills."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import stack_module


def navigation() -> Blueprint:
    """Expose navigation skills; native endpoints own planning and motion."""

    bp = Blueprint()
    NavSkills = stack_module(
        "navigation_skills",
        "default",
        seed_group="navigation",
        fallback="nav.skills.skills_module.NavSkills",
    )
    bp.add(NavSkills, alias="nav.skills")
    return bp


__all__ = ["navigation"]
