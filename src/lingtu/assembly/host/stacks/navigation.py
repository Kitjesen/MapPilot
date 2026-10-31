"""Host-side navigation command skills."""

from __future__ import annotations

from runtime.host import HostBuilder
from runtime.plugin_resolution import stack_module


def navigation() -> HostBuilder:
    """Expose navigation skills; native endpoints own planning and motion."""

    bp = HostBuilder()
    NavSkills = stack_module(
        "navigation_skills",
        "default",
        seed_group="navigation",
        fallback="nav.skills.skills_module.NavSkills",
    )
    bp.add(NavSkills, alias="nav.skills")
    return bp


__all__ = ["navigation"]
