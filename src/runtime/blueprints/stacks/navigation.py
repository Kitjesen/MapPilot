"""Navigation stack: Navigation + optional adapters and autonomy chain."""

from __future__ import annotations

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks.autonomy_chain import (
    add_autonomy_chain,
    autonomy_stack_config,
)
from runtime.blueprints.stacks.exploration_goal_sources import (
    add_exploration_goal_sources,
    frontier_module_config,
)
from runtime.blueprints.stacks.navigation_core import (
    add_navigation_core,
    navigation_config,
)
from runtime.blueprints.stacks.navigation_io import (
    add_navigation_io_adapter_stack,
    wire_navigation_output_adapter_stack,
)


def navigation(
    planner_backend: str = "octoplanner3d",
    tomogram: str = "",
    enable_native: bool = False,
    **config,
) -> Blueprint:
    """Global planning plus local autonomy unless a native endpoint owns it."""

    bp = Blueprint()
    add_navigation_core(
        bp,
        planner_backend=planner_backend,
        tomogram=tomogram,
        **config,
    )
    add_navigation_io_adapter_stack(bp, **config)
    add_exploration_goal_sources(bp, **config)
    if not config.get("native_navigation_endpoint"):
        add_autonomy_chain(bp, enable_native=enable_native, **config)
    wire_navigation_output_adapter_stack(bp)
    return bp


__all__ = [
    "autonomy_stack_config",
    "frontier_module_config",
    "navigation",
    "navigation_config",
]
