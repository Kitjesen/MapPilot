"""Named Blueprint inputs for local, development, and simulation Profiles.

Field Product Host inputs are owned by :mod:`lingtu.assembly.products` and are
deliberately absent from this Profile catalog.
"""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.local_host_defaults import (
    LOCAL_HOST_DEFAULTS,
    LOCAL_PROFILE_NAMES,
)
from runtime.profiles.catalog.simulation_profiles import (
    SIMULATION_PROFILE_CONFIGS,
    SIMULATION_PROFILES,
)

_HOST_DEFAULT_GROUPS = (
    LOCAL_HOST_DEFAULTS,
    SIMULATION_PROFILE_CONFIGS,
)

for _index, _group in enumerate(_HOST_DEFAULT_GROUPS):
    for _other in _HOST_DEFAULT_GROUPS[_index + 1 :]:
        _overlap = sorted(set(_group) & set(_other))
        if _overlap:
            raise RuntimeError(
                "Host default categories must not overlap: " + ", ".join(_overlap)
            )

HOST_PROFILE_DEFAULTS: dict[str, dict[str, Any]] = {
    **LOCAL_HOST_DEFAULTS,
    **SIMULATION_PROFILE_CONFIGS,
}

HOST_PROFILE_SNAPSHOT_NAMES = SIMULATION_PROFILES


__all__ = [
    "HOST_PROFILE_DEFAULTS",
    "HOST_PROFILE_SNAPSHOT_NAMES",
    "LOCAL_PROFILE_NAMES",
    "SIMULATION_PROFILES",
]
