"""Composable blueprint factory exports."""

from __future__ import annotations

import importlib
from typing import Any


_EXPORTS = {
    "driver": ("core.blueprints.stacks", "driver"),
    "exploration": ("core.blueprints.stacks", "exploration"),
    "full_stack_blueprint": ("core.blueprints.full_stack", "full_stack_blueprint"),
    "gateway": ("core.blueprints.stacks", "gateway"),
    "lidar": ("core.blueprints.stacks", "lidar"),
    "maps": ("core.blueprints.stacks", "maps"),
    "memory": ("core.blueprints.stacks", "memory"),
    "multi_robot_blueprint": ("core.blueprints.multi_robot", "multi_robot_blueprint"),
    "navigation": ("core.blueprints.stacks", "navigation"),
    "perception": ("core.blueprints.stacks", "perception"),
    "planner": ("core.blueprints.stacks", "planner"),
    "safety": ("core.blueprints.stacks", "safety"),
    "sim_lidar": ("core.blueprints.stacks", "sim_lidar"),
    "slam": ("core.blueprints.stacks", "slam"),
    "stub_blueprint": ("core.blueprints.stub", "stub_blueprint"),
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
