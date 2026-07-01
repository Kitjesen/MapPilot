"""Mission execution support."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "Navigation",
    "MissionMode",
    "MissionState",
]

_EXPORTS = {
    "Navigation": "nav.mission.navigation",
    "MissionMode": "nav.mission.model.state",
    "MissionState": "nav.mission.model.state",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
