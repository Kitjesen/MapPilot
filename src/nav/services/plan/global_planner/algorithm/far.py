"""Registry entry for the optional ROS-free native FAR global planner."""

from __future__ import annotations

from nav.services.plan.contracts import global_plan_result_schema
from runtime.registry import get_metadata, register

from .far_planner import FarPlannerBackend

SUPPORTED_MAP_FORMATS = ("occupancy_2d", "rolling_occupancy_2d")
SUPPORTED_MAP_EXTENSIONS = (".npz",)

PLANNER_INPUT_SCHEMA = {
    "type": "object",
    "required": ["start", "goal", "map_generation"],
    "properties": {
        "start": {"type": "array", "minItems": 3, "maxItems": 3},
        "goal": {"type": "array", "minItems": 3, "maxItems": 3},
        "map_generation": {"type": "integer", "minimum": 1},
    },
}


@register(
    "planner_backend",
    "far",
    description=(
        "Optional ROS-free native FAR visibility-graph planner for trinary "
        "2-D occupancy maps; known-free planning is attempted before the "
        "explicitly enabled unknown-space fallback"
    ),
)
class FarBackend(FarPlannerBackend):
    """Registry type; native library loading occurs only on construction."""


get_metadata("planner_backend", "far").update(
    {
        "optional": True,
        "supported_map_formats": list(SUPPORTED_MAP_FORMATS),
        "supported_map_extensions": list(SUPPORTED_MAP_EXTENSIONS),
        "input_schema": PLANNER_INPUT_SCHEMA,
        "output_schema": global_plan_result_schema(),
    }
)


__all__ = [
    "FarBackend",
    "FarPlannerBackend",
    "PLANNER_INPUT_SCHEMA",
    "SUPPORTED_MAP_EXTENSIONS",
    "SUPPORTED_MAP_FORMATS",
]
