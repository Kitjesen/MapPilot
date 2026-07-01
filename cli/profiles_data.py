"""Compatibility exports for runtime profile data.

Profile ownership lives under :mod:`core.blueprints.catalog`. The
``core.runtime_profiles`` module keeps legacy imports stable for CLI code.
"""

from __future__ import annotations

from core.runtime_profiles import (
    PROFILES,
    ROBOT_PRESETS,
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
