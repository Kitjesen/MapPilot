"""Global planning service boundary."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = ["GlobalPlanner", "SavedMapArtifacts"]

_EXPORTS = {
    "GlobalPlanner": "nav.services.plan.global_planner.service",
    "SavedMapArtifacts": "nav.services.plan.global_planner.artifacts",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
