"""Navigation package: mission execution, planning services, safety, exploration, and native C++ kernels."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "MissionMode",
    "MissionState",
    "Navigation",
]

_EXPORTS = {
    "Navigation": "nav.navigation",
    "MissionMode": "nav.model.state",
    "MissionState": "nav.model.state",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
