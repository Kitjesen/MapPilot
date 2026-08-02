"""Local runtime entry points for building LingTu systems.

This module is the package-facing boundary for local execution. It resolves a
Host Profile, applies a Profile adapter, and builds the
Blueprint system without exposing those lower-level layers to facade classes.
"""

from __future__ import annotations

from typing import Any, Mapping

from lingtu.assembly.profile_builder import build_system_from_resolved_profile
from runtime.profiles.resolver import ResolvedRuntimeConfig, resolve_runtime_config


def resolve_runtime(
    profile: str,
    *,
    profile_adapter: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> ResolvedRuntimeConfig:
    """Resolve a local Host Profile into a runtime-ready configuration."""

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    return resolve_runtime_config(
        profile,
        profile_adapter_name=profile_adapter,
        overrides=merged_overrides,
        include_profile_metadata=include_profile_metadata,
    )


def build_system(
    profile: str,
    *,
    profile_adapter: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> Any:
    """Resolve *profile* and build the corresponding local Blueprint system."""

    resolved = resolve_runtime(
        profile,
        profile_adapter=profile_adapter,
        overrides=overrides,
        include_profile_metadata=include_profile_metadata,
        **inline_overrides,
    )
    return build_system_from_resolved_profile(
        resolved.profile,
        resolved.config,
    )
