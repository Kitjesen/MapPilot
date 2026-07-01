"""Build Blueprints from resolved runtime profile configuration."""

from __future__ import annotations

from typing import Any, Mapping

from runtime.blueprint import Blueprint
from runtime.profiles.resolver import canonical_profile_name


def blueprint_for_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Return the product or compatibility Blueprint for a resolved profile."""

    canonical_profile = canonical_profile_name(profile)

    from runtime.blueprints.products import product_blueprint_for_profile

    product_blueprint = product_blueprint_for_profile(canonical_profile, config)
    if product_blueprint is not None:
        return product_blueprint

    from runtime.blueprints.full_stack import full_stack_blueprint

    return full_stack_blueprint(**dict(config))


def module_transport_name(config: Mapping[str, Any]) -> str:
    """Return the effective ModulePort transport strategy for a resolved config."""

    return str(
        config.get("module_transport")
        or config.get("_module_transport")
        or "local"
    ).strip().lower()


def module_transport_for_resolved_config(config: Mapping[str, Any]) -> Any | None:
    """Create the build transport for a resolved profile config.

    ``None`` means Blueprint keeps its default fresh LocalTransport instance.
    Non-local strategies are wrapped so Out/In ports can use the simple
    publish/subscribe transport protocol.
    """

    strategy = module_transport_name(config)
    if strategy in ("", "local"):
        return None

    from runtime.transport.factory import create_transport_adapter

    return create_transport_adapter(strategy)


def build_system_from_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Any:
    """Build a system from a resolved profile config and its transport contract."""

    bp = blueprint_for_resolved_profile(profile, config)
    transport = module_transport_for_resolved_config(config)
    if transport is None:
        return bp.build()
    return bp.build(transport=transport)


def build_system_for_profile(
    profile: str,
    overrides: Mapping[str, Any] | None = None,
    **inline_overrides: Any,
) -> Any:
    """Resolve a profile, select product/compat Blueprint, then build System."""

    from runtime.profiles.resolver import resolve_profile_config

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    config = resolve_profile_config(profile, overrides=merged_overrides)
    return build_system_from_resolved_profile(profile, config)
