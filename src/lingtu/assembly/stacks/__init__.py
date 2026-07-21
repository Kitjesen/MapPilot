"""Lazy exports for composable blueprint factory functions."""

from __future__ import annotations

import importlib
from typing import Any

_EXPORTS = {
    "compose_full_stack_modules": ("lingtu.assembly.stacks.composition", "compose_full_stack_modules"),
    "driver": ("lingtu.assembly.stacks.driver", "driver"),
    "exploration": ("lingtu.assembly.stacks.exploration", "exploration"),
    "external_services": ("lingtu.assembly.stacks.system", "external_services"),
    "gateway": ("lingtu.assembly.stacks.gateway", "gateway"),
    "gnss": ("lingtu.assembly.stacks.system", "gnss"),
    "hw": ("lingtu.assembly.stacks.system", "hw"),
    "imu": ("lingtu.assembly.stacks.imu", "imu"),
    "lidar": ("lingtu.assembly.stacks.lidar", "lidar"),
    "maps": ("lingtu.assembly.stacks.maps", "maps"),
    "memory": ("lingtu.assembly.stacks.memory", "memory"),
    "navigation": ("lingtu.assembly.stacks.navigation", "navigation"),
    "perception": ("lingtu.assembly.stacks.perception", "perception"),
    "planner": ("lingtu.assembly.stacks.planner", "planner"),
    "safety": ("lingtu.assembly.stacks.safety", "safety"),
    "services": ("lingtu.assembly.stacks.services", "services"),
    "sim_lidar": ("lingtu.assembly.stacks.sim_lidar", "sim_lidar"),
    "slam": ("lingtu.assembly.stacks.slam", "slam"),
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
