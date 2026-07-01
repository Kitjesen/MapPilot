"""Runtime catalog surfaces for products, robots, and endpoints.

The catalog package is the product-grade source for named runtime choices.
Compatibility modules such as ``core.runtime_profiles`` may re-export these
objects while CLI and deployment code migrate gradually.
"""

from __future__ import annotations

from .endpoints import (
    RUNTIME_ENDPOINTS,
    RuntimeEndpointError,
    RuntimeEndpointSpec,
    RuntimeRunSpec,
)
from .products import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    PROFILES,
    SIMULATION_PROFILES,
)
from .robots import (
    CANONICAL_ROBOT_DRIVER_PROFILES,
    CANONICAL_ROBOT_PRESETS,
    COMPAT_ROBOT_DRIVER_PROFILES,
    COMPAT_ROBOT_PRESETS,
    ROBOT_DRIVER_PROFILES,
    ROBOT_PRESETS,
    robot_driver_module_name,
    robot_driver_profile,
    robot_driver_profile_names,
    robot_preset,
    robot_preset_names,
)
from .runtime_paths import RUNTIME_MAP_FRAME_ID, _default_map_dir, _resolve_tomogram

__all__ = [
    "CANONICAL_ROBOT_DRIVER_PROFILES",
    "CANONICAL_ROBOT_PRESETS",
    "COMPAT_ROBOT_DRIVER_PROFILES",
    "COMPAT_ROBOT_PRESETS",
    "ROBOT_DRIVER_PROFILES",
    "ROBOT_PRESETS",
    "RUNTIME_MAP_FRAME_ID",
    "RUNTIME_ENDPOINTS",
    "LIGHTWEIGHT_PRODUCT_PROFILES",
    "OPTIONAL_NATIVE_PRODUCT_PROFILES",
    "PRODUCT_PROFILES",
    "PROFILE_SNAPSHOT_TARGETS",
    "PROFILES",
    "RuntimeEndpointError",
    "RuntimeEndpointSpec",
    "RuntimeRunSpec",
    "SIMULATION_PROFILES",
    "robot_driver_module_name",
    "robot_driver_profile",
    "robot_driver_profile_names",
    "robot_preset",
    "robot_preset_names",
    "_default_map_dir",
    "_resolve_tomogram",
]
