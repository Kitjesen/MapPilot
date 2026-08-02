"""Runtime catalog surfaces for Host Profiles, driver backends, and adapters.

Field Product declarations and Host defaults are intentionally owned outside
this package by Runtime Graph and :mod:`lingtu.assembly.products`.
"""

from __future__ import annotations

from .profile_adapters import (
    PROFILE_ADAPTERS,
    ProfileAdapterError,
    ProfileAdapterSpec,
    RuntimeRunSpec,
    profile_adapter_names_for_profile,
)
from .host_defaults import (
    HOST_PROFILE_DEFAULTS,
    HOST_PROFILE_SNAPSHOT_NAMES,
)
from .local_host_defaults import (
    LOCAL_HOST_DEFAULTS,
    LOCAL_PROFILE_NAMES,
)
from .navigation_defaults import (
    THUNDER_MAP_ARTIFACT_DEFAULTS,
    THUNDER_OCTOPLANNER_DEFAULTS,
)
from .driver_backends import (
    CANONICAL_DRIVER_BACKENDS,
    CANONICAL_DRIVER_MODULES,
    CANONICAL_DRIVER_PROTOCOLS,
    DRIVER_BACKENDS,
    DRIVER_MODULES,
    DRIVER_PROTOCOLS,
    driver_backend_defaults,
    driver_backend_module_name,
    driver_backend_names,
    driver_backend_protocol,
)
from .driver_catalog import (
    DRIVER_CATALOG_DIR,
    DRIVER_CATALOG_SCHEMA_VERSION,
    driver_catalog,
    driver_catalog_path,
)
from .driver_runtime_defaults import (
    CANONICAL_DRIVER_RUNTIME_DEFAULTS,
    DRIVER_RUNTIME_DEFAULTS,
    driver_runtime_defaults,
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
from .simulation_profiles import (
    SIMULATION_PROFILE_CONFIGS,
    SIMULATION_PROFILES,
)

__all__ = [
    "CANONICAL_DRIVER_BACKENDS",
    "CANONICAL_DRIVER_MODULES",
    "CANONICAL_DRIVER_PROTOCOLS",
    "CANONICAL_DRIVER_RUNTIME_DEFAULTS",
    "DEFAULT_GATEWAY_PORT",
    "DEFAULT_PLANNING_FRAME_ID",
    "DEFAULT_SAMPLE_OCTOPLANNER3D_MAP",
    "HOST_PROFILE_DEFAULTS",
    "HOST_PROFILE_SNAPSHOT_NAMES",
    "LOCAL_HOST_DEFAULTS",
    "LOCAL_PROFILE_NAMES",
    "DRIVER_BACKENDS",
    "DRIVER_CATALOG_DIR",
    "DRIVER_CATALOG_SCHEMA_VERSION",
    "DRIVER_MODULES",
    "DRIVER_PROTOCOLS",
    "DRIVER_RUNTIME_DEFAULTS",
    "PROFILE_ADAPTERS",
    "RUNTIME_MAP_FRAME_ID",
    "RUNTIME_ODOM_FRAME_ID",
    "SIMULATION_PROFILES",
    "SIMULATION_PROFILE_CONFIGS",
    "THUNDER_MAP_ARTIFACT_DEFAULTS",
    "THUNDER_OCTOPLANNER_DEFAULTS",
    "ProfileAdapterError",
    "ProfileAdapterSpec",
    "RuntimeRunSpec",
    "_default_map_dir",
    "_resolve_octoplanner3d_map",
    "driver_backend_defaults",
    "driver_backend_module_name",
    "driver_backend_names",
    "driver_backend_protocol",
    "driver_catalog",
    "driver_catalog_path",
    "driver_runtime_defaults",
    "profile_adapter_names_for_profile",
]
