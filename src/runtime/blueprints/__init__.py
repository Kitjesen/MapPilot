"""Composable blueprint factory exports."""

from __future__ import annotations

import importlib
from typing import Any


_EXPORTS = {
    # Import concrete stack modules directly. If a stack submodule was imported
    # first, Python sets ``runtime.blueprints.stacks.<name>`` to that module object;
    # routing through the package would then return the module instead of the
    # factory function and break the top-level lazy API.
    "driver": ("runtime.blueprints.stacks.driver", "driver"),
    "exploration": ("runtime.blueprints.stacks.exploration", "exploration"),
    "full_stack_blueprint": ("runtime.blueprints.full_stack", "full_stack_blueprint"),
    "gateway": ("runtime.blueprints.stacks.gateway", "gateway"),
    "lidar": ("runtime.blueprints.stacks.lidar", "lidar"),
    "maps": ("runtime.blueprints.stacks.maps", "maps"),
    "memory": ("runtime.blueprints.stacks.memory", "memory"),
    "multi_robot_blueprint": ("runtime.blueprints.multi_robot", "multi_robot_blueprint"),
    "navigation": ("runtime.blueprints.stacks.navigation", "navigation"),
    "perception": ("runtime.blueprints.stacks.perception", "perception"),
    "planner": ("runtime.blueprints.stacks.planner", "planner"),
    "safety": ("runtime.blueprints.stacks.safety", "safety"),
    "sim_lidar": ("runtime.blueprints.stacks.sim_lidar", "sim_lidar"),
    "slam": ("runtime.blueprints.stacks.slam", "slam"),
    "stub_blueprint": ("runtime.blueprints.stub", "stub_blueprint"),
    "thunder_basic_blueprint": ("runtime.blueprints.products", "thunder_basic_blueprint"),
    "thunder_explore_blueprint": (
        "runtime.blueprints.products",
        "thunder_explore_blueprint",
    ),
    "thunder_map_blueprint": ("runtime.blueprints.products", "thunder_map_blueprint"),
    "thunder_nav_blueprint": ("runtime.blueprints.products", "thunder_nav_blueprint"),
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
