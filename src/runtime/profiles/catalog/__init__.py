"""Runtime catalog surfaces for products, robots, and endpoints.

The catalog package is the product-grade source for named runtime choices.
Compatibility modules such as ``runtime.runtime_profiles`` may re-export these
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
from .robot_archives import (
    ROBOT_ARCHIVE_DIR,
    ROBOT_ARCHIVE_SCHEMA_VERSION,
    robot_archive,
    robot_archive_path,
)
from .robot_runtime_defaults import (
    CANONICAL_ROBOT_RUNTIME_DEFAULTS,
    COMPAT_ROBOT_RUNTIME_DEFAULTS,
    ROBOT_RUNTIME_DEFAULTS,
    robot_runtime_defaults,
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
from .runtime_paths import (
    DEFAULT_GATEWAY_PORT,
    DEFAULT_PLANNING_FRAME_ID,
    DEFAULT_SAMPLE_OCTOPLANNER3D_MAP,
    RUNTIME_MAP_FRAME_ID,
    RUNTIME_ODOM_FRAME_ID,
    _default_map_dir,
    _resolve_octoplanner3d_map,
)

__all__ = [
    "CANONICAL_ROBOT_DRIVER_PROFILES",
    "CANONICAL_ROBOT_PRESETS",
    "CANONICAL_ROBOT_RUNTIME_DEFAULTS",
    "COMPAT_ROBOT_DRIVER_PROFILES",
    "COMPAT_ROBOT_PRESETS",
    "COMPAT_ROBOT_RUNTIME_DEFAULTS",
    "DEFAULT_GATEWAY_PORT",
    "DEFAULT_PLANNING_FRAME_ID",
    "DEFAULT_SAMPLE_OCTOPLANNER3D_MAP",
    "LIGHTWEIGHT_PRODUCT_PROFILES",
    "OPTIONAL_NATIVE_PRODUCT_PROFILES",
    "PRODUCT_PROFILES",
    "PROFILES",
    "PROFILE_SNAPSHOT_TARGETS",
    "ROBOT_ARCHIVE_DIR",
    "ROBOT_ARCHIVE_SCHEMA_VERSION",
    "ROBOT_DRIVER_PROFILES",
    "ROBOT_PRESETS",
    "ROBOT_RUNTIME_DEFAULTS",
    "RUNTIME_ENDPOINTS",
    "RUNTIME_MAP_FRAME_ID",
    "RUNTIME_ODOM_FRAME_ID",
    "SIMULATION_PROFILES",
    "RuntimeEndpointError",
    "RuntimeEndpointSpec",
    "RuntimeRunSpec",
    "_default_map_dir",
    "_resolve_octoplanner3d_map",
    "robot_archive",
    "robot_archive_path",
    "robot_driver_module_name",
    "robot_driver_profile",
    "robot_driver_profile_names",
    "robot_preset",
    "robot_preset_names",
    "robot_runtime_defaults",
]
