"""Compatibility hook for the official Livox ROS2 driver process."""

from __future__ import annotations

from importlib import import_module
from typing import Any

_COMPAT_MODULE = "drivers.adapters.ros2.livox_driver"


def livox_driver_process(cfg: Any):
    """Build the explicit ROS2 Livox driver process adapter.

    This factory is imported only when ``Lidar(start_driver=True)`` is used.
    Product profiles normally let systemd own the ``lidar`` service.
    """

    adapter = import_module(_COMPAT_MODULE)
    return adapter.livox_driver_process(cfg)

__all__ = ["livox_driver_process"]
