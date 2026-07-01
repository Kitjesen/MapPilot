"""Compatibility exports for runtime profile data.

Catalog ownership now lives under ``core.blueprints.catalog``:

- ``catalog.products`` owns product and simulation profiles.
- ``catalog.robots`` owns robot presets and driver defaults.
- ``catalog.runtime_paths`` owns map/tomogram path resolution.

This module keeps the old import path stable for CLI and existing tooling.
"""

from __future__ import annotations

from core.blueprints.catalog.products import PROFILES
from core.blueprints.catalog.robots import ROBOT_PRESETS
from core.blueprints.catalog.runtime_paths import (
    RUNTIME_MAP_FRAME_ID,
    _default_map_dir,
    _resolve_tomogram,
)

__all__ = [
    "PROFILES",
    "ROBOT_PRESETS",
    "RUNTIME_MAP_FRAME_ID",
    "_default_map_dir",
    "_resolve_tomogram",
]
