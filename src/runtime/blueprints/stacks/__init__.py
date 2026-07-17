"""Lazy exports for composable blueprint factory functions."""

from __future__ import annotations

import importlib
from typing import Any

_EXPORTS = {
    "compose_full_stack_modules": ("runtime.blueprints.stacks.composition", "compose_full_stack_modules"),
    "driver": ("runtime.blueprints.stacks.driver", "driver"),
    "exploration": ("runtime.blueprints.stacks.exploration", "exploration"),
    "external_services": ("runtime.blueprints.stacks.system", "external_services"),
    "gateway": ("runtime.blueprints.stacks.gateway", "gateway"),
    "gnss": ("runtime.blueprints.stacks.system", "gnss"),
    "hw": ("runtime.blueprints.stacks.system", "hw"),
    "imu": ("runtime.blueprints.stacks.imu", "imu"),
    "lidar": ("runtime.blueprints.stacks.lidar", "lidar"),
    "maps": ("runtime.blueprints.stacks.maps", "maps"),
    "memory": ("runtime.blueprints.stacks.memory", "memory"),
    "navigation": ("runtime.blueprints.stacks.navigation", "navigation"),
    "perception": ("runtime.blueprints.stacks.perception", "perception"),
    "planner": ("runtime.blueprints.stacks.planner", "planner"),
    "safety": ("runtime.blueprints.stacks.safety", "safety"),
    "services": ("runtime.blueprints.stacks.services", "services"),
    "sim_lidar": ("runtime.blueprints.stacks.sim_lidar", "sim_lidar"),
    "slam": ("runtime.blueprints.stacks.slam", "slam"),
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
