"""LiDAR short role package.

New code should use the short role ``LidarModule`` or the native source
contract. Legacy Python DDS readback stays under ``compat/`` and is exposed as
old submodule aliases for one compatibility window.
"""

from __future__ import annotations

import sys
from importlib import import_module

from .api.frames import LivoxPointFrame
from .compat.lidar import Lidar, LidarHealth, LidarState
from .impl.livox import Sdk2Source
from .module import LidarModule
from .native.sdk import LidarSource

_SUBMODULE_ALIASES = {
    "_dds": ".compat.dds",
    "dds_adapter": ".compat.dds_adapter",
    "frame_stream": ".api.frame_stream",
    "frames": ".api.frames",
    "lidar": ".compat.lidar",
    "lidar_module": ".module",
    "sdk2_stream_source": ".impl.livox.sdk2_stream_source",
    "source": ".native.sdk",
}

for _name, _target in _SUBMODULE_ALIASES.items():
    _module = import_module(_target, __name__)
    sys.modules[f"{__name__}.{_name}"] = _module
    globals()[_name] = _module

__all__ = [
    "Lidar",
    "LidarHealth",
    "LidarModule",
    "LidarSource",
    "LidarState",
    "LivoxPointFrame",
    "Sdk2Source",
]
