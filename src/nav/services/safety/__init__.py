"""Safety services: safety reflexes, plan safety, and velocity arbitration."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = ["SafetyRing", "VelocityMux"]

_EXPORTS = {
    "SafetyRing": "nav.services.safety.safety_ring",
    "VelocityMux": "nav.services.safety.velocity_mux",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
