"""Local runtime entry points for building LingTu systems.

This module is the package-facing boundary for local execution.  It resolves a
product profile, applies robot/runtime endpoint overrides, and builds the
Blueprint system without exposing those lower-level layers to facade classes.
"""

from __future__ import annotations

from typing import Any, Mapping

from lingtu.assembly.profile_builder import compile_product
from runtime.profiles.resolver import ResolvedRuntimeConfig, resolve_runtime_config


def resolve_runtime(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    robot_preset: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> ResolvedRuntimeConfig:
    """Resolve a LingTu product profile into a runtime-ready configuration."""

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    return resolve_runtime_config(
        profile,
        runtime_endpoint_name=runtime_endpoint,
        robot_preset=robot_preset,
        overrides=merged_overrides,
        include_profile_metadata=include_profile_metadata,
    )


def build_system(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    robot_preset: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> Any:
    """Resolve *profile* and build the corresponding local Blueprint system."""

    resolved = resolve_runtime(
        profile,
        runtime_endpoint=runtime_endpoint,
        robot_preset=robot_preset,
        overrides=overrides,
        include_profile_metadata=include_profile_metadata,
        **inline_overrides,
    )
    product = compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )
    return product.build()
