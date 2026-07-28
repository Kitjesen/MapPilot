"""Navigation stack: Navigation + optional adapters and autonomy chain."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import stack_module
from lingtu.assembly.stacks.autonomy_chain import (
    add_autonomy_chain,
    autonomy_stack_config,
)
from lingtu.assembly.stacks.exploration_goal_sources import (
    add_exploration_goal_sources,
    frontier_module_config,
)
from lingtu.assembly.stacks.navigation_core import (
    add_navigation_core,
    navigation_config,
)


def navigation(
    planner_backend: str = "octoplanner3d",
    map_path: str = "",
    enable_native: bool = False,
    **config,
) -> Blueprint:
    """Global planning plus local autonomy unless a native endpoint owns it."""

    bp = Blueprint()
    add_navigation_core(
        bp,
        planner_backend=planner_backend,
        map_path=map_path,
        **config,
    )
    add_exploration_goal_sources(bp, **config)
    if not config.get("native_navigation_endpoint"):
        add_autonomy_chain(bp, enable_native=enable_native, **config)

    # -- Supporting modules ------------------------------------------------

    # L6 MCP/agent adapter. Product commands still enter through nav.goals.
    NavSkills = stack_module(
        "navigation_skills",
        "default",
        seed_group="navigation",
        fallback="nav.skills.skills_module.NavSkills",
    )
    bp.add(NavSkills, alias="nav.skills")

    # LocalizationMonitorModule: localization health monitoring (always included)
    LocalizationMonitorModule = stack_module(
        "localization_monitor",
        "default",
        seed_group="navigation",
        fallback="nav.localization_monitor.monitor_module.LocalizationMonitorModule",
    )
    bp.add(LocalizationMonitorModule, alias="nav.localization_monitor")

    return bp


__all__ = [
    "autonomy_stack_config",
    "frontier_module_config",
    "navigation",
    "navigation_config",
]
