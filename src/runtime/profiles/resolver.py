"""Resolve product profiles into runtime-ready blueprint config.

The resolver keeps the startup layers explicit:

1. Product profile chooses the task and high-level behavior.
2. Robot preset supplies hardware defaults without overriding product behavior.
3. Runtime endpoint applies compatibility or simulation adaptations.
4. User overrides win last.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from runtime.profiles.catalog.endpoints import (
    PRODUCT_PROFILE_ENDPOINTS,
    RUNTIME_ENDPOINTS,
    RuntimeEndpointError,
    RuntimeEndpointSpec,
)
from runtime.profiles.catalog.products import PRODUCT_PROFILES, PROFILES
from runtime.profiles.catalog.robot_runtime_defaults import ROBOT_RUNTIME_DEFAULTS
from runtime.profiles.catalog.robots import ROBOT_PRESETS
from runtime.profiles.endpoint_config import (
    endpoint_config_for_profile,
    merge_runtime_endpoint_config,
)
from runtime.profiles.endpoints import runtime_endpoint
from runtime.profiles.planner_backends import (
    planner_fallback_chain,
    resolve_planner_runtime_profile,
)
from runtime.runtime_interface import DATA_SOURCE_CONTRACTS, profile_data_source

PROFILE_ALIASES: dict[str, str] = {
    "thunder-basic": "lite",
    "thunder-lite": "lite",
    "remote": "teleop",
    "remote-control": "teleop",
    "teleop-avoid": "teleop_avoid",
    "remote-avoid": "teleop_avoid",
    "mapping": "map",
    "thunder-map": "map",
    "navigation": "nav",
    "thunder-nav": "nav",
    "inspect": "inspection",
    "patrol": "inspection",
    "thunder-explore": "tare_explore",
    "thunder-tare": "tare_explore",
    "thunder-tare-explore": "tare_explore",
}


@dataclass(frozen=True)
class ResolvedRuntimeConfig:
    """Resolved config plus the layers that produced it."""

    profile: str
    robot_preset: str
    runtime_endpoint: str | None
    product_config: Mapping[str, Any]
    robot_config: Mapping[str, Any]
    robot_runtime_config: Mapping[str, Any]
    endpoint_config: Mapping[str, Any]
    overrides: Mapping[str, Any]
    config: Mapping[str, Any]


@dataclass(frozen=True)
class _RuntimeConfigLayers:
    """Resolver-owned layers, ordered from product intent to operator input."""

    product_intent: dict[str, Any]
    robot_preset: dict[str, Any]
    robot_runtime_defaults: dict[str, Any]
    endpoint_adapter: dict[str, Any]
    user_overrides: dict[str, Any]


def resolve_runtime_config(
    profile: str,
    *,
    runtime_endpoint_name: str | None = None,
    robot_preset: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
) -> ResolvedRuntimeConfig:
    """Resolve profile config through product, robot, endpoint, override layers."""

    profile = canonical_profile_name(profile)
    if profile not in PROFILES:
        raise KeyError(f"unknown profile: {profile}")

    product_source = PROFILES[profile]
    endpoint = (
        runtime_endpoint(runtime_endpoint_name)
        if runtime_endpoint_name
        else _default_runtime_endpoint(profile, robot_preset=robot_preset)
    )
    if endpoint is not None:
        endpoint.require_profile(profile)
        _require_product_endpoint(profile, endpoint)

    selected_robot = _selected_robot_preset(
        profile,
        product_source,
        endpoint=endpoint,
        robot_preset=robot_preset,
    )
    layers = _resolve_runtime_layers(
        profile,
        product_source,
        selected_robot,
        endpoint=endpoint,
        overrides=overrides,
        include_profile_metadata=include_profile_metadata,
        allow_custom_robot=robot_preset is not None,
    )
    config = _merge_runtime_layers(profile, layers)

    return ResolvedRuntimeConfig(
        profile=profile,
        robot_preset=selected_robot,
        runtime_endpoint=endpoint.name if endpoint is not None else None,
        product_config=layers.product_intent,
        robot_config=layers.robot_preset,
        robot_runtime_config=layers.robot_runtime_defaults,
        endpoint_config=layers.endpoint_adapter,
        overrides=layers.user_overrides,
        config=config,
    )


def resolve_profile_config(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    robot_preset: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> dict[str, Any]:
    """Return resolved blueprint kwargs for a profile."""

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    resolved = resolve_runtime_config(
        profile,
        runtime_endpoint_name=runtime_endpoint,
        robot_preset=robot_preset,
        overrides=merged_overrides,
        include_profile_metadata=include_profile_metadata,
    )
    return dict(resolved.config)


def canonical_profile_name(profile: str) -> str:
    """Return the canonical catalog profile for a CLI/product alias."""

    return PROFILE_ALIASES.get(profile, profile)


def _require_product_endpoint(profile: str, endpoint: RuntimeEndpointSpec) -> None:
    allowed = PRODUCT_PROFILE_ENDPOINTS.get(profile)
    if allowed is None or endpoint.name in allowed:
        return
    choices = ", ".join(allowed)
    raise RuntimeEndpointError(
        f"endpoint '{endpoint.name}' is not a product endpoint for profile "
        f"'{profile}' (allowed: {choices})"
    )


def _resolve_runtime_layers(
    profile: str,
    product_source: Mapping[str, Any],
    selected_robot: str,
    *,
    endpoint: RuntimeEndpointSpec | None,
    overrides: Mapping[str, Any] | None,
    include_profile_metadata: bool,
    allow_custom_robot: bool,
) -> _RuntimeConfigLayers:
    """Build explicit resolver layers without merging them."""

    return _RuntimeConfigLayers(
        product_intent=_product_intent_layer(
            product_source,
            include_profile_metadata=include_profile_metadata,
        ),
        robot_preset=_robot_preset_layer(
            profile,
            selected_robot,
            allow_custom_robot=allow_custom_robot,
        ),
        robot_runtime_defaults=_robot_runtime_defaults_layer(
            profile,
            selected_robot,
            allow_custom_robot=allow_custom_robot,
        ),
        endpoint_adapter=(
            endpoint_config_for_profile(endpoint, profile)
            if endpoint is not None
            else {}
        ),
        user_overrides=dict(overrides or {}),
    )


def _merge_runtime_layers(profile: str, layers: _RuntimeConfigLayers) -> dict[str, Any]:
    """Merge layers in the resolver contract order."""

    config = dict(layers.product_intent)
    _apply_robot_defaults(config, layers.robot_preset)
    _apply_robot_defaults(config, layers.robot_runtime_defaults)
    if layers.endpoint_adapter:
        config = merge_runtime_endpoint_config(config, layers.endpoint_adapter)
    config.update(layers.user_overrides)
    config.setdefault(
        "cmd_vel_mux_collision_monitor",
        bool(
            config.get("enable_teleop", True)
            and config.get("enable_map_modules", True)
        ),
    )
    planner_profile = resolve_planner_runtime_profile(profile, config)
    config["planner"] = planner_profile["primary"]
    config["fallback_planners"] = list(planner_profile["fallback_planners"])
    configured_fallbacks = planner_fallback_chain(config.get("fallback_planner_name"))
    if configured_fallbacks:
        config["fallback_planner_name"] = configured_fallbacks[0]
    elif planner_profile["fallback_planners"]:
        config["fallback_planner_name"] = planner_profile["fallback_planners"][0]
    config["planner_latency_budget_ms"] = planner_profile["latency_budget_ms"]
    config["planner_profile"] = planner_profile
    return config


def _selected_robot_preset(
    profile: str,
    product_config: Mapping[str, Any],
    *,
    endpoint: RuntimeEndpointSpec | None,
    robot_preset: str | None,
) -> str:
    if robot_preset:
        return robot_preset
    if endpoint is not None:
        return endpoint.robot_preset
    return str(product_config.get("_default_robot", "stub"))


def _default_runtime_endpoint(
    profile: str,
    *,
    robot_preset: str | None,
) -> RuntimeEndpointSpec | None:
    """Return the default hardware endpoint for product profiles.

    Simulation and dev profiles keep their historical profile-only resolution.
    Product profiles that bind to a hardware data source get the single matching
    non-simulation endpoint, so endpoint-owned compatibility choices are applied
    before the module graph is built.
    """

    if robot_preset is not None or profile not in PRODUCT_PROFILES:
        return None
    try:
        data_source = profile_data_source(profile).data_source
        source = DATA_SOURCE_CONTRACTS[data_source]
    except (KeyError, ValueError):
        return None
    if source.provider != "hardware":
        return None

    candidates = [
        endpoint
        for endpoint in RUNTIME_ENDPOINTS.values()
        if endpoint.data_source == data_source
        and profile in endpoint.supported_profiles
        and not endpoint.simulation_only
    ]
    return candidates[0] if len(candidates) == 1 else None


def _product_intent_layer(
    product_config: Mapping[str, Any],
    *,
    include_profile_metadata: bool,
) -> dict[str, Any]:
    if include_profile_metadata:
        return {
            key: value
            for key, value in product_config.items()
            if key != "_default_robot"
        }
    return {
        key: value
        for key, value in product_config.items()
        if not key.startswith("_")
    }


def _robot_preset_layer(
    profile: str,
    selected_robot: str,
    *,
    allow_custom_robot: bool,
) -> dict[str, Any]:
    if selected_robot in ROBOT_PRESETS:
        return dict(ROBOT_PRESETS[selected_robot])
    if allow_custom_robot:
        return {"robot": selected_robot}
    raise KeyError(f"unknown robot preset for profile {profile}: {selected_robot}")


def _robot_runtime_defaults_layer(
    profile: str,
    selected_robot: str,
    *,
    allow_custom_robot: bool,
) -> dict[str, Any]:
    if selected_robot in ROBOT_RUNTIME_DEFAULTS:
        return dict(ROBOT_RUNTIME_DEFAULTS[selected_robot])
    if allow_custom_robot:
        return {}
    raise KeyError(
        f"unknown robot runtime defaults for profile {profile}: {selected_robot}"
    )


def _apply_robot_defaults(
    config: dict[str, Any],
    robot_config: Mapping[str, Any],
) -> None:
    for key, value in robot_config.items():
        if key == "robot" or key not in config:
            config[key] = value
