"""OctoPlanner3D registry entry for GlobalPlanner."""

from __future__ import annotations

from runtime.registry import get_metadata, register

from .octoplanner3d_planner import OctoPlanner3DPlanner
from .octoplanner3d_protocol import (
    PLANNER_INPUT_SCHEMA,
    PLANNER_OUTPUT_SCHEMA,
    SUPPORTED_MAP_EXTENSIONS,
    SUPPORTED_MAP_FORMATS,
)


@register(
    "planner_backend",
    "octoplanner3d",
    description=(
        "OctoPlanner3D constrained global planner via ROS-free C++ headless "
        "executable; internal search is OctoMap 3D A*, ROS2 wrapper is "
        "adapter/debug only"
    ),
)
class OctoPlanner3DBackend(OctoPlanner3DPlanner):
    """Compatibility registry shim for the existing planner_backend contract."""


get_metadata("planner_backend", "octoplanner3d").update(
    {
        "supported_map_formats": list(SUPPORTED_MAP_FORMATS),
        "supported_map_extensions": list(SUPPORTED_MAP_EXTENSIONS),
        "input_schema": PLANNER_INPUT_SCHEMA,
        "output_schema": PLANNER_OUTPUT_SCHEMA,
    }
)


__all__ = [
    "OctoPlanner3DBackend",
    "OctoPlanner3DPlanner",
    "PLANNER_INPUT_SCHEMA",
    "PLANNER_OUTPUT_SCHEMA",
    "SUPPORTED_MAP_EXTENSIONS",
    "SUPPORTED_MAP_FORMATS",
]
