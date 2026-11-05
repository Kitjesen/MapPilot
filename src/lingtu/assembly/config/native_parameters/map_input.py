"""Map and local collision input parameters."""

import math

from .schema import ParameterSpec, _integer

COLLISION_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec("local_collision.max_age_s", "LINGTU_NAV_LOCAL_COLLISION_MAX_AGE_S", float, 0.10, math.inf),
)

MAP_INPUT_SPECS: tuple[ParameterSpec, ...] = (
    ParameterSpec(
        "map_input.max_cells",
        "LINGTU_NAV_SEGMENT_MAX_GRID_CELLS",
        _integer,
        64,
        1_048_576,
    ),
    ParameterSpec(
        "map_input.max_age_s",
        "LINGTU_NAV_SEGMENT_MAP_MAX_AGE_S",
        float,
        0.02,
        10.0,
    ),
    ParameterSpec(
        "map_input.future_tolerance_s",
        "LINGTU_NAV_SEGMENT_FUTURE_TOLERANCE_S",
        float,
        0.0,
        2.0,
    ),
)
