"""Runtime resolution helpers for product, robot, and endpoint selection."""

from __future__ import annotations

from .resolver import (
    PROFILE_ALIASES,
    ResolvedRuntimeConfig,
    canonical_profile_name,
    resolve_profile_config,
    resolve_runtime_config,
)

__all__ = [
    "PROFILE_ALIASES",
    "ResolvedRuntimeConfig",
    "canonical_profile_name",
    "resolve_profile_config",
    "resolve_runtime_config",
]
