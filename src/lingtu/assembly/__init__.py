"""Composable blueprint factory exports."""

from __future__ import annotations

import importlib
from typing import Any


_EXPORTS = {
    # Import concrete stack modules directly. If a stack submodule was imported
    # first, Python sets ``lingtu.assembly.stacks.<name>`` to that module object;
    # routing through the package would then return the module instead of the
    # factory function and break the top-level lazy API.
    "driver": ("lingtu.assembly.stacks.driver", "driver"),
    "exploration": ("lingtu.assembly.stacks.exploration", "exploration"),
    "gateway": ("lingtu.assembly.stacks.gateway", "gateway"),
    "lidar": ("lingtu.assembly.stacks.lidar", "lidar"),
    "maps": ("lingtu.assembly.stacks.maps", "maps"),
    "memory": ("lingtu.assembly.stacks.memory", "memory"),
    "navigation": ("lingtu.assembly.stacks.navigation", "navigation"),
    "perception": ("lingtu.assembly.stacks.perception", "perception"),
    "planner": ("lingtu.assembly.stacks.planner", "planner"),
    "safety": ("lingtu.assembly.stacks.safety", "safety"),
    "sim_lidar": ("lingtu.assembly.stacks.sim_lidar", "sim_lidar"),
    "slam": ("lingtu.assembly.stacks.slam", "slam"),
    "stub_blueprint": ("lingtu.assembly.stub", "stub_blueprint"),
    "thunder_basic_blueprint": ("lingtu.assembly.products", "thunder_basic_blueprint"),
    "thunder_explore_blueprint": (
        "lingtu.assembly.products",
        "thunder_explore_blueprint",
    ),
    "thunder_map_blueprint": ("lingtu.assembly.products", "thunder_map_blueprint"),
    "thunder_nav_blueprint": ("lingtu.assembly.products", "thunder_nav_blueprint"),
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
