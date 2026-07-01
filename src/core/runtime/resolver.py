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

from core.blueprints.catalog.endpoints import RUNTIME_ENDPOINTS, RuntimeEndpointSpec
from core.blueprints.catalog.products import PRODUCT_PROFILES, PROFILES
from core.blueprints.catalog.robots import ROBOT_PRESETS
from core.blueprints.runtime_endpoint import apply_runtime_endpoint_config, runtime_endpoint
from core.runtime_interface import DATA_SOURCE_CONTRACTS, profile_data_source


PROFILE_ALIASES: dict[str, str] = {
    "thunder-basic": "lite",
    "thunder-lite": "lite",
    "thunder-map": "map",
    "thunder-nav": "nav",
    "thunder-explore": "explore",
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
    endpoint_config: Mapping[str, Any]
    overrides: Mapping[str, Any]
    config: Mapping[str, Any]


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

    selected_robot = _selected_robot_preset(
        profile,
        product_source,
        endpoint=endpoint,
        robot_preset=robot_preset,
    )
    product_config = _product_config_layer(
        product_source,
        include_profile_metadata=include_profile_metadata,
    )
    robot_config = _robot_config_layer(
        profile,
        selected_robot,
        allow_custom_robot=robot_preset is not None,
    )

    config = dict(product_config)
    _apply_robot_defaults(config, robot_config)

    endpoint_config: dict[str, Any] = {}
    if endpoint is not None:
        endpoint_config = endpoint.config_for_profile(profile)
        config = apply_runtime_endpoint_config(profile, config, endpoint.name)

    clean_overrides = dict(overrides or {})
    config.update(clean_overrides)

    return ResolvedRuntimeConfig(
        profile=profile,
        robot_preset=selected_robot,
        runtime_endpoint=endpoint.name if endpoint is not None else None,
        product_config=product_config,
        robot_config=robot_config,
        endpoint_config=endpoint_config,
        overrides=clean_overrides,
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
    """Return full_stack_blueprint kwargs for a profile."""

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


def _product_config_layer(
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


def _robot_config_layer(
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


def _apply_robot_defaults(
    config: dict[str, Any],
    robot_config: Mapping[str, Any],
) -> None:
    for key, value in robot_config.items():
        if key == "robot" or key not in config:
            config[key] = value
