"""LiDAR — compatibility shim. Use drivers.real.lidar.Lidar directly."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from drivers.real.lidar import Lidar as LiDAR

__all__ = ["LiDAR"]


def __getattr__(name: str):
    """Lazy re-export from drivers.real.lidar to keep lingtu/ importable."""
    if name == "LiDAR":
        from drivers.real.lidar import Lidar as _LiDAR

        return _LiDAR
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
