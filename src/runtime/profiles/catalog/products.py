"""Profile catalog compatibility facade.

Product intent lives in :mod:`product_intents`; simulation/dev entrypoints live
in :mod:`simulation_profiles`. This module preserves the historical import path
for CLI and tooling that still expects a single ``PROFILES`` mapping.
"""

from __future__ import annotations

from typing import Any

from runtime.profiles.catalog.product_intents import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PRODUCT_INTENT_PROFILES,
    PRODUCT_MODE_PROFILES,
    PRODUCT_PROFILE_CONFIGS,
    PRODUCT_PROFILES,
)
from runtime.profiles.catalog.simulation_profiles import (
    SIMULATION_ENTRYPOINT_PROFILES,
    SIMULATION_PROFILE_CONFIGS,
    SIMULATION_PROFILES,
)

PROFILE_NAME_OVERLAP = frozenset(
    set(PRODUCT_PROFILE_CONFIGS) & set(SIMULATION_PROFILE_CONFIGS)
)
if PROFILE_NAME_OVERLAP:
    names = ", ".join(sorted(PROFILE_NAME_OVERLAP))
    raise RuntimeError(
        "Product and simulation profile catalogs must not share names: "
        f"{names}"
    )

PROFILES: dict[str, dict[str, Any]] = {
    **PRODUCT_PROFILE_CONFIGS,
    **SIMULATION_PROFILE_CONFIGS,
}

PROFILE_SNAPSHOT_TARGETS = (
    *SIMULATION_PROFILES,
    *(
        profile
        for profile in PRODUCT_PROFILES
        if profile not in OPTIONAL_NATIVE_PRODUCT_PROFILES
        and profile not in LIGHTWEIGHT_PRODUCT_PROFILES
    ),
)


def product_profile(name: str) -> dict[str, Any]:
    """Return a copy of a named product, simulation, or development profile."""

    return dict(PROFILES[name])


def is_product_profile(name: str) -> bool:
    """Return True when ``name`` is a Thunder product intent profile."""

    return name in PRODUCT_INTENT_PROFILES


def is_simulation_profile(name: str) -> bool:
    """Return True when ``name`` is a simulation or development entrypoint."""

    return name in SIMULATION_ENTRYPOINT_PROFILES
