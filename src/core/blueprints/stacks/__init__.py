"""Lazy exports for composable blueprint factory functions."""

from __future__ import annotations

import importlib
from typing import Any

_EXPORTS = {
    "compose_full_stack_modules": ("core.blueprints.stacks.composition", "compose_full_stack_modules"),
    "device_manager": ("core.blueprints.stacks.system", "device_manager"),
    "driver": ("core.blueprints.stacks.driver", "driver"),
    "exploration": ("core.blueprints.stacks.exploration", "exploration"),
    "external_services": ("core.blueprints.stacks.system", "external_services"),
    "gateway": ("core.blueprints.stacks.gateway", "gateway"),
    "gnss": ("core.blueprints.stacks.system", "gnss"),
    "lidar": ("core.blueprints.stacks.lidar", "lidar"),
    "maps": ("core.blueprints.stacks.maps", "maps"),
    "memory": ("core.blueprints.stacks.memory", "memory"),
    "navigation": ("core.blueprints.stacks.navigation", "navigation"),
    "perception": ("core.blueprints.stacks.perception", "perception"),
    "planner": ("core.blueprints.stacks.planner", "planner"),
    "safety": ("core.blueprints.stacks.safety", "safety"),
    "sim_lidar": ("core.blueprints.stacks.sim_lidar", "sim_lidar"),
    "slam": ("core.blueprints.stacks.slam", "slam"),
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
