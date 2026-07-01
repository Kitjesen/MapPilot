"""Compatibility import for ROS relocalization service-call adapters."""

from __future__ import annotations

from importlib import import_module

__all__ = [
    "RELOCALIZE_SERVICE",
    "GLOBAL_RELOCALIZE_SERVICE",
    "GLOBAL_RELOCALIZE_STATUS_SERVICE",
    "RelocalizationResult",
    "subprocess",
    "parse_quality",
    "global_relocalize_command",
    "global_relocalize_status_command",
    "saved_map_relocalize_command",
    "saved_map_relocalize_env",
    "saved_map_relocalize_env_command",
    "trigger_global_relocalize",
    "query_global_relocalize_status",
    "relocalize_saved_map",
    "relocalize_saved_map_with_env",
]

_COMPAT_MODULE = "localization.adapters.ros2.relocalization_service"


def _compat_symbols():
    module = import_module(_COMPAT_MODULE)
    return {name: getattr(module, name) for name in __all__}


def __getattr__(name: str):
    if name in __all__:
        return _compat_symbols()[name]
    raise AttributeError(name)
