"""Build Blueprints from resolved runtime profile configuration."""

from __future__ import annotations

from typing import Any, Mapping

from runtime.blueprint import Blueprint
from runtime.profiles.resolver import canonical_profile_name


def blueprint_for_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Blueprint:
    """Return the product Blueprint for a resolved profile."""

    canonical_profile = canonical_profile_name(profile)

    from runtime.blueprints.products import product_blueprint_for_profile

    return product_blueprint_for_profile(canonical_profile, config)


def module_transport_name(config: Mapping[str, Any]) -> str:
    """Return the effective ModulePort transport strategy for a resolved config."""

    return str(config.get("module_transport") or config.get("_module_transport") or "local").strip().lower()


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


def route_contract_name_for_resolved_config(config: Mapping[str, Any]) -> str | None:
    """Return the external route contract selected by a resolved config.

    Route contracts are boundary metadata. They validate endpoint topics and
    adapter ownership, but they do not change ModulePort delivery unless a
    Blueprint explicitly calls ``Blueprint.routed_delivery(...)``.
    """

    from runtime.profiles.endpoints import route_contract_for_config

    return route_contract_for_config(config)


def validate_route_contract_for_resolved_config(config: Mapping[str, Any]) -> None:
    """Fail fast when a resolved endpoint references an invalid route contract."""

    route_contract = route_contract_name_for_resolved_config(config)
    if not route_contract:
        return

    from runtime.route_contract import load_route_contract, validate_route_contract

    contract = load_route_contract(route_contract)
    issues = validate_route_contract(contract)
    if issues:
        detail = "; ".join(f"{issue.code}:{issue.topic}" for issue in issues)
        raise ValueError(f"invalid route contract '{route_contract}': {detail}")


def build_system_from_resolved_profile(
    profile: str,
    config: Mapping[str, Any],
) -> Any:
    """Build a system from a resolved profile config and its transport contract."""

    validate_route_contract_for_resolved_config(config)
    route_contract = route_contract_name_for_resolved_config(config)
    bp = blueprint_for_resolved_profile(profile, config)
    if route_contract:
        bp = bp.route_contract(route_contract)
    transport = module_transport_for_resolved_config(config)
    if transport is None:
        return bp.build()
    return bp.build(transport=transport)


def build_system_for_profile(
    profile: str,
    overrides: Mapping[str, Any] | None = None,
    **inline_overrides: Any,
) -> Any:
    """Resolve a profile, select the product Blueprint, then build System."""

    from runtime.profiles.resolver import resolve_profile_config

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    config = resolve_profile_config(profile, overrides=merged_overrides)
    return build_system_from_resolved_profile(profile, config)
