"""LingTu persistent map domain.

The package root stays dependency-light so path and artifact contracts can be
used by diagnostics and profile resolution without loading runtime Modules.
Application services are imported lazily through the public API below.
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS = {
    "ArtifactHandle": ("maps.client", "ArtifactHandle"),
    "InvalidMapName": ("maps.services", "InvalidMapName"),
    "MapAPIService": ("maps.services", "MapAPIService"),
    "MapClient": ("maps.client", "MapClient"),
    "MapClientError": ("maps.client", "MapClientError"),
    "MapControlService": ("maps.services", "MapControlService"),
    "MapPipelineService": ("maps.services", "MapPipelineService"),
    "MapRuntimeBridge": ("maps.services", "MapRuntimeBridge"),
    "MapStorageService": ("maps.services", "MapStorageService"),
    "safe_map_name": ("maps.services", "safe_map_name"),
    "validate_map_name": ("maps.services", "validate_map_name"),
    "active_map_dir": ("maps.paths", "active_map_dir"),
    "active_map_name": ("maps.paths", "active_map_name"),
    "map_root_candidates": ("maps.paths", "map_root_candidates"),
    "nav_map_root": ("maps.paths", "nav_map_root"),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    target = _EXPORTS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute = target
    value = getattr(import_module(module_name), attribute)
    globals()[name] = value
    return value
