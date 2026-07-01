"""Compatibility import for the ROS-backed localization bridge.

The implementation lives in :mod:`localization.adapters.ros2.slam_bridge` so
the localization package keeps ROS runtime code behind an adapter boundary.
"""

from importlib import import_module

__all__ = [
    "SlamBridgeModule",
    "LOC_UNINIT",
    "LOC_TRACKING",
    "LOC_DEGRADED",
    "LOC_FALLBACK_GNSS_ONLY",
    "LOC_LOST",
    "LOC_DIVERGED",
    "DEGEN_NONE",
    "DEGEN_MILD",
    "DEGEN_SEVERE",
    "DEGEN_CRITICAL",
    "LOCALIZER_ODOM_GRACE_HEALTH",
    "LOCALIZER_ODOM_LOSS_RECOVERY_SIGNAL",
    "LOCALIZER_ODOM_LOSS_RECOVERY_ACTION",
    "MAP_ODOM_TF_BACKENDS",
]

_COMPAT_MODULE = "localization.adapters.ros2.slam_bridge"


def _compat_symbols():
    module = import_module(_COMPAT_MODULE)
    return {name: getattr(module, name) for name in __all__}


def __getattr__(name: str):
    if name in __all__:
        return _compat_symbols()[name]
    raise AttributeError(name)
