"""Local navigation Modules: terrain, local planning, and path following."""

from __future__ import annotations

from importlib import import_module
from typing import Any

from .stack import add_autonomy_stack

__all__ = [
    "LocalPlanner",
    "PathFollower",
    "Terrain",
    "add_autonomy_stack",
]


_EXPORTS = {
    "Terrain": "nav.local.terrain",
    "LocalPlanner": "nav.local.local_planner",
    "PathFollower": "nav.local.path_follower",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
