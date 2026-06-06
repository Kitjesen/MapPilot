"""lingtu.memory -- unified memory layer.

Spatial memory (where), knowledge (what), storage (how to persist),
scheduling (when to look), logging (what happened).

Heavy memory implementations are imported lazily so importing the package does
not load numerical runtimes during blueprint/plugin discovery.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS = {
    "EpisodicMemory": "memory.spatial.episodic",
    "EpisodicMemoryModule": "memory.modules.episodic_module",
    "TaggedLocationStore": "memory.spatial.tagged_locations",
    "TaggedLocationsModule": "memory.modules.tagged_locations_module",
    "TopologicalMemory": "memory.spatial.topological",
    "TopologicalMemoryModule": "memory.modules.topological_module",
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(module_name), name)
    globals()[name] = value
    return value
