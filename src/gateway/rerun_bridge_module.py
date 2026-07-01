"""Compatibility import for the ROS-backed Rerun visualization bridge.

The implementation lives in :mod:`gateway.visualization.rerun_bridge` so the gateway
package does not own ROS runtime code.
"""

from importlib import import_module

__all__ = ["RerunBridgeModule"]

_COMPAT_MODULE = "gateway.visualization.rerun_bridge"


def __getattr__(name: str):
    if name == "RerunBridgeModule":
        return getattr(import_module(_COMPAT_MODULE), name)
    raise AttributeError(name)
