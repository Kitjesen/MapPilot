"""Composable blueprint factory exports."""

from __future__ import annotations

import importlib
from typing import Any


_EXPORTS = {
    # Import concrete stack modules directly. If a stack submodule was imported
    # first, Python sets ``core.blueprints.stacks.<name>`` to that module object;
    # routing through the package would then return the module instead of the
    # factory function and break the top-level lazy API.
    "driver": ("core.blueprints.stacks.driver", "driver"),
    "exploration": ("core.blueprints.stacks.exploration", "exploration"),
    "full_stack_blueprint": ("core.blueprints.full_stack", "full_stack_blueprint"),
    "gateway": ("core.blueprints.stacks.gateway", "gateway"),
    "lidar": ("core.blueprints.stacks.lidar", "lidar"),
    "maps": ("core.blueprints.stacks.maps", "maps"),
    "memory": ("core.blueprints.stacks.memory", "memory"),
    "multi_robot_blueprint": ("core.blueprints.multi_robot", "multi_robot_blueprint"),
    "navigation": ("core.blueprints.stacks.navigation", "navigation"),
    "perception": ("core.blueprints.stacks.perception", "perception"),
    "planner": ("core.blueprints.stacks.planner", "planner"),
    "safety": ("core.blueprints.stacks.safety", "safety"),
    "sim_lidar": ("core.blueprints.stacks.sim_lidar", "sim_lidar"),
    "slam": ("core.blueprints.stacks.slam", "slam"),
    "stub_blueprint": ("core.blueprints.stub", "stub_blueprint"),
    "thunder_basic_blueprint": ("core.blueprints.products", "thunder_basic_blueprint"),
    "thunder_explore_blueprint": (
        "core.blueprints.products",
        "thunder_explore_blueprint",
    ),
    "thunder_map_blueprint": ("core.blueprints.products", "thunder_map_blueprint"),
    "thunder_nav_blueprint": ("core.blueprints.products", "thunder_nav_blueprint"),
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    if name not in _EXPORTS:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attr_name = _EXPORTS[name]
    module = importlib.import_module(module_name)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value
