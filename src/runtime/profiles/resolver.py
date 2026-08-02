"""Resolve named Host defaults into runtime-ready Blueprint config.

The resolver keeps the startup layers explicit:

1. Host defaults choose the in-process Python graph.
2. The Profile's internal driver backend supplies local driver defaults.
3. A Profile adapter applies hardware, data-source, or simulation adaptations.
4. User overrides win last.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from runtime.profiles.catalog.driver_backends import DRIVER_BACKENDS
from runtime.profiles.catalog.driver_runtime_defaults import DRIVER_RUNTIME_DEFAULTS
from runtime.profiles.catalog.host_defaults import HOST_PROFILE_DEFAULTS
from runtime.profiles.catalog.profile_adapters import (
    PROFILE_ADAPTERS,
    ProfileAdapterError,
    ProfileAdapterSpec,
    profile_adapter_names_for_profile,
)
from runtime.profiles.planner_backends import (
    planner_fallback_chain,
    resolve_planner_runtime_profile,
)
from runtime.profiles.profile_adapter_config import (
    merge_profile_adapter_config,
    profile_adapter_config_for_profile,
)
from runtime.profiles.profile_adapters import profile_adapter
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    profile_data_source,
)

_RESERVED_PROFILE_OVERRIDE_KEYS = frozenset(
    {"robot", "driver_backend", "_driver_backend"}
)


@dataclass(frozen=True)
class ResolvedRuntimeConfig:
    """Resolved config plus the layers that produced it."""

    profile: str
    driver_backend: str
    profile_adapter: str | None
    host_defaults: Mapping[str, Any]
    driver_config: Mapping[str, Any]
    driver_runtime_config: Mapping[str, Any]
    adapter_config: Mapping[str, Any]
    overrides: Mapping[str, Any]
    config: Mapping[str, Any]


@dataclass(frozen=True)
class _RuntimeConfigLayers:
    """Resolver-owned layers, ordered from Host defaults to operator input."""

    host_defaults: dict[str, Any]
    driver_backend_defaults: dict[str, Any]
    driver_runtime_defaults: dict[str, Any]
    profile_adapter: dict[str, Any]
    user_overrides: dict[str, Any]


def resolve_runtime_config(
    profile: str,
    *,
    profile_adapter_name: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
) -> ResolvedRuntimeConfig:
    """Resolve a local/development Host Profile.

    Field Products are intentionally rejected here. Product assembly uses
    ``lingtu.assembly.products.resolve_product_host_config`` instead.
    """

    profile = canonical_profile_name(profile)
    return resolve_named_host_config(
        profile,
        HOST_PROFILE_DEFAULTS,
        profile_adapter_name=profile_adapter_name,
        overrides=overrides,
        include_profile_metadata=include_profile_metadata,
    )


def resolve_named_host_config(
    name: str,
    defaults: Mapping[str, Mapping[str, Any]],
    *,
    profile_adapter_name: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
) -> ResolvedRuntimeConfig:
    """Apply local Host hardware and communication layers."""

    if name not in defaults:
        raise KeyError(f"unknown profile: {name}")

    host_defaults = defaults[name]
    adapter = (
        profile_adapter(profile_adapter_name)
        if profile_adapter_name
        else _default_profile_adapter(name)
    )
    if adapter is not None:
        if name not in adapter.supported_profiles:
            supported = ", ".join(adapter.supported_profiles)
            raise ProfileAdapterError(
                f"profile adapter '{adapter.name}' does not support profile '{name}' "
                f"(supported: {supported})"
            )
        _require_named_adapter(
            name,
            adapter,
        )

    selected_driver = _selected_driver_backend(
        host_defaults,
        adapter=adapter,
    )
    layers = _resolve_runtime_layers(
        name,
        host_defaults,
        selected_driver,
        adapter=adapter,
        overrides=overrides,
        include_profile_metadata=include_profile_metadata,
    )
    config = _merge_runtime_layers(name, layers)
    config["_selection_kind"] = "profile"

    return ResolvedRuntimeConfig(
        profile=name,
        driver_backend=selected_driver,
        profile_adapter=adapter.name if adapter is not None else None,
        host_defaults=layers.host_defaults,
        driver_config=layers.driver_backend_defaults,
        driver_runtime_config=layers.driver_runtime_defaults,
        adapter_config=layers.profile_adapter,
        overrides=layers.user_overrides,
        config=config,
    )


def resolve_profile_config(
    profile: str,
    *,
    profile_adapter: str | None = None,
    overrides: Mapping[str, Any] | None = None,
    include_profile_metadata: bool = False,
    **inline_overrides: Any,
) -> dict[str, Any]:
    """Return resolved blueprint kwargs for a profile."""

    merged_overrides = dict(overrides or {})
    merged_overrides.update(inline_overrides)
    resolved = resolve_runtime_config(
        profile,
        profile_adapter_name=profile_adapter,
        overrides=merged_overrides,
        include_profile_metadata=include_profile_metadata,
    )
    return dict(resolved.config)


def canonical_profile_name(profile: str) -> str:
    """Return a normalized local Host Profile name."""

    return profile


def _require_named_adapter(
    name: str,
    adapter: ProfileAdapterSpec,
) -> None:
    allowed = profile_adapter_names_for_profile(name)
    if not allowed or adapter.name in allowed:
        return
    choices = ", ".join(allowed)
    raise ProfileAdapterError(
        f"profile adapter '{adapter.name}' does not accept profile '{name}' "
        f"(allowed: {choices})"
    )


def _resolve_runtime_layers(
    profile: str,
    host_defaults: Mapping[str, Any],
    selected_driver: str,
    *,
    adapter: ProfileAdapterSpec | None,
    overrides: Mapping[str, Any] | None,
    include_profile_metadata: bool,
) -> _RuntimeConfigLayers:
    """Build explicit resolver layers without merging them."""

    return _RuntimeConfigLayers(
        host_defaults=_host_defaults_layer(
            host_defaults,
            include_profile_metadata=include_profile_metadata,
        ),
        driver_backend_defaults=_driver_backend_layer(
            profile,
            selected_driver,
        ),
        driver_runtime_defaults=_driver_runtime_defaults_layer(
            profile,
            selected_driver,
        ),
        profile_adapter=(
            profile_adapter_config_for_profile(adapter, profile)
            if adapter is not None
            else {}
        ),
        user_overrides=_validated_profile_overrides(overrides),
    )


def _merge_runtime_layers(profile: str, layers: _RuntimeConfigLayers) -> dict[str, Any]:
    """Merge layers in the resolver contract order."""

    config = dict(layers.host_defaults)
    _apply_driver_defaults(config, layers.driver_backend_defaults)
    _apply_driver_defaults(config, layers.driver_runtime_defaults)
    if layers.profile_adapter:
        config = merge_profile_adapter_config(config, layers.profile_adapter)
    config.update(layers.user_overrides)
    return finalize_host_config(profile, config)


def finalize_host_config(name: str, config: Mapping[str, Any]) -> dict[str, Any]:
    """Apply shared derived Host settings after all owning layers are merged."""

    config = dict(config)
    config.setdefault(
        "cmd_vel_mux_collision_monitor",
        bool(config.get("enable_teleop", True) and config.get("enable_map_modules", True)),
    )
    planner_profile = resolve_planner_runtime_profile(name, config)
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


def _selected_driver_backend(
    host_defaults: Mapping[str, Any],
    *,
    adapter: ProfileAdapterSpec | None,
) -> str:
    if adapter is not None:
        return adapter.driver_backend
    return str(host_defaults.get("_driver_backend", "stub"))


def _default_profile_adapter(profile: str) -> ProfileAdapterSpec | None:
    """Return the single default hardware adapter for a Host Profile.

    Simulation and dev profiles keep their historical profile-only resolution.
    Host profiles that bind to a hardware data source get the single matching
    non-simulation adapter, so adapter-owned compatibility choices are applied
    before the module graph is built.
    """

    try:
        data_source = profile_data_source(profile).data_source
        source = DATA_SOURCE_CONTRACTS[data_source]
    except (KeyError, ValueError):
        return None
    if source.provider != "hardware":
        return None

    candidates = [
        adapter
        for adapter in PROFILE_ADAPTERS.values()
        if adapter.data_source == data_source
        and profile in adapter.supported_profiles
        and not adapter.simulation_only
    ]
    return candidates[0] if len(candidates) == 1 else None


def _host_defaults_layer(
    host_defaults: Mapping[str, Any],
    *,
    include_profile_metadata: bool,
) -> dict[str, Any]:
    if include_profile_metadata:
        return {
            key: value
            for key, value in host_defaults.items()
            if key != "_driver_backend"
        }
    return {key: value for key, value in host_defaults.items() if not key.startswith("_")}


def _driver_backend_layer(
    profile: str,
    selected_driver: str,
) -> dict[str, Any]:
    if selected_driver in DRIVER_BACKENDS:
        return dict(DRIVER_BACKENDS[selected_driver])
    raise KeyError(f"unknown driver backend for profile {profile}: {selected_driver}")


def _driver_runtime_defaults_layer(
    profile: str,
    selected_driver: str,
) -> dict[str, Any]:
    if selected_driver in DRIVER_RUNTIME_DEFAULTS:
        return dict(DRIVER_RUNTIME_DEFAULTS[selected_driver])
    raise KeyError(f"unknown driver runtime defaults for profile {profile}: {selected_driver}")


def _apply_driver_defaults(
    config: dict[str, Any],
    robot_config: Mapping[str, Any],
) -> None:
    for key, value in robot_config.items():
        if key == "robot" or key not in config:
            config[key] = value


def _validated_profile_overrides(
    overrides: Mapping[str, Any] | None,
) -> dict[str, Any]:
    values = dict(overrides or {})
    reserved = sorted(
        key
        for key in values
        if key in _RESERVED_PROFILE_OVERRIDE_KEYS or key.endswith("_preset")
    )
    if reserved:
        raise ValueError(
            "Profile overrides cannot select a driver backend: " + ", ".join(reserved)
        )
    return values
