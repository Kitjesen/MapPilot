"""Compatibility exports for runtime profile data.

Catalog ownership now lives under ``runtime.profiles.catalog``. This module keeps
the old import path stable for CLI and existing tooling.
"""

from __future__ import annotations

from runtime.profiles.catalog.products import PROFILES
from runtime.profiles.catalog.robots import ROBOT_PRESETS
from runtime.profiles.catalog.runtime_paths import (
    RUNTIME_MAP_FRAME_ID,
    _default_map_dir,
)

__all__ = [
    "PROFILES",
    "ROBOT_PRESETS",
    "RUNTIME_MAP_FRAME_ID",
    "_default_map_dir",
]
