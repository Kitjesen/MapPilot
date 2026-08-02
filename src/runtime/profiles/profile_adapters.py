"""Profile adapter resolver for the local Host/connection split.

Adapter definitions live in :mod:`runtime.profiles.catalog.profile_adapters`.
This module applies them to Host Profiles and compiles runtime run
specs for CLI/audit tooling.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import Any, Mapping

from runtime.profiles.binding_policy import (
    endpoint_contract_for_config,
    endpoint_transport_for_config,
    localization_adapter_for_config,
    module_transport_for_config,
    resolved_autonomy_backend_selection,
)
from runtime.profiles.catalog.profile_adapters import (
    PROFILE_ADAPTERS,
    ProfileAdapterError,
    ProfileAdapterSpec,
    RuntimeRunSpec,
)
from runtime.profiles.profile_adapter_config import (
    merge_profile_adapter_config,
    normalized_profile_semantic_overrides,
    profile_adapter_config_for_profile,
)
from runtime.runtime_interface import (
    DATA_SOURCE_CONTRACTS,
    FRAME_LINKS,
    FRAMES,
    RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES,
    canonical_data_source_name,
    profile_data_source,
    resolved_runtime_data_flow,
    runtime_topic_allowed_frame_ids,
    runtime_topic_default_frame_ids,
)

_RESOLVER_OWNED_ENV_KEYS = frozenset(
    {
        "LINGTU_PROFILE",
        "LINGTU_PROFILE_ADAPTER",
        "LINGTU_DATA_SOURCE",
        "LINGTU_MODULE_TRANSPORT",
        "LINGTU_ENDPOINT_TRANSPORT",
        "LINGTU_RUNTIME_CONTRACT",
        "LINGTU_ENDPOINT_CONTRACT",
        "LINGTU_ROUTE_CONTRACT",
        "LINGTU_NAV_GLOBAL_PLANNER",
        "LINGTU_LOCALIZATION_ADAPTER",
        "LINGTU_ENABLE_ROBOT_DRIVER",
        "LINGTU_COMMAND_OUTPUT_MODE",
        "LINGTU_HARDWARE_CONTROL_BOUNDARY",
        "LINGTU_COMMAND_SINK",
        "LINGTU_SIMULATION_ONLY",
    }
)


def canonical_profile_adapter_name(name: str) -> str:
    """Return a normalized local Host Profile adapter name."""

    return str(name)


def profile_adapter(name: str) -> ProfileAdapterSpec:
    adapter_name = canonical_profile_adapter_name(name)
    try:
        return PROFILE_ADAPTERS[adapter_name]
    except KeyError as exc:
        choices = ", ".join(profile_adapter_names())
        raise ProfileAdapterError(f"unknown profile adapter '{name}' (choices: {choices})") from exc


def profile_adapter_names() -> tuple[str, ...]:
    return tuple(PROFILE_ADAPTERS)


def apply_profile_adapter_config(
    profile: str,
    config: Mapping[str, Any],
    adapter_name: str,
) -> dict[str, Any]:
    adapter = profile_adapter(adapter_name)
    adapter_config = profile_adapter_config_for_profile(adapter, profile)
    return merge_profile_adapter_config(config, adapter_config)


def _launcher_args_for_config(
    config: Mapping[str, Any],
    *,
    profile: str,
    adapter_name: str | None,
    launcher: Any,
    record: bool,
    extra_args: tuple[str, ...],
) -> tuple[str, ...]:
    if extra_args:
        return extra_args
    explicit_default_args = config.get("_external_default_args")
    explicit_record_args = config.get("_external_record_args")
    if record:
        if explicit_record_args:
            return tuple(explicit_record_args)
        adapter = _adapter_for_launcher_args(config, adapter_name, launcher)
        if adapter is not None and adapter.record_actions:
            if profile not in adapter.record_actions:
                raise ProfileAdapterError(
                    f"profile adapter '{adapter.name}' record_actions missing profile '{profile}'"
                )
            return adapter.record_actions[profile]
        if explicit_default_args:
            return tuple(explicit_default_args)
        if adapter is not None and adapter.default_actions:
            if profile not in adapter.default_actions:
                raise ProfileAdapterError(
                    f"profile adapter '{adapter.name}' default_actions missing profile '{profile}'"
                )
            return adapter.default_actions[profile]
        raise ProfileAdapterError(f"external launcher args missing for profile '{profile}'")

    if explicit_default_args:
        return tuple(explicit_default_args)
    adapter = _adapter_for_launcher_args(config, adapter_name, launcher)
    if adapter is not None and adapter.default_actions:
        if profile not in adapter.default_actions:
            raise ProfileAdapterError(
                f"profile adapter '{adapter.name}' default_actions missing profile '{profile}'"
            )
        return adapter.default_actions[profile]
    raise ProfileAdapterError(f"external launcher args missing for profile '{profile}'")


def _adapter_for_launcher_args(
    config: Mapping[str, Any],
    adapter_name: str | None,
    launcher: Any,
) -> ProfileAdapterSpec | None:
    if not adapter_name:
        return None
    canonical_adapter_name = canonical_profile_adapter_name(adapter_name)
    if canonical_adapter_name not in PROFILE_ADAPTERS:
        return None
    adapter = PROFILE_ADAPTERS[canonical_adapter_name]
    if not adapter.external_launcher:
        return None
    if config.get("_profile_adapter") == adapter.name:
        return adapter
    if str(launcher or "") == adapter.external_launcher:
        return adapter
    return None


def _adapter_name_for_config(
    config: Mapping[str, Any],
    data_source_name: str,
) -> str | None:
    adapter_name = config.get("_profile_adapter")
    if adapter_name:
        return canonical_profile_adapter_name(str(adapter_name))

    runtime_contract = config.get("_runtime_contract")
    launcher = config.get("_external_launcher")
    for name, adapter in PROFILE_ADAPTERS.items():
        if runtime_contract and adapter.runtime_contract == runtime_contract:
            return name
        if launcher and adapter.external_launcher == launcher:
            return name
    for name, adapter in PROFILE_ADAPTERS.items():
        if adapter.data_source == data_source_name:
            return name
    return None


def route_contract_for_config(
    config: Mapping[str, Any],
    *,
    adapter_name: str | None = None,
    data_source_name: str | None = None,
    endpoint_transport: str | None = None,
    endpoint_contract: str | None = None,
) -> str | None:
    """Return the route-contract preset for a resolved runtime boundary.

    This is deliberately separate from ``module_transport``. A route contract
    describes the external bus/topic boundary. It does not force normal Module
    ports to use DDS unless the Blueprint explicitly opts into routed delivery.
    """

    raw_adapter = adapter_name or str(config.get("_profile_adapter") or "").strip()
    resolved_adapter = canonical_profile_adapter_name(raw_adapter) if raw_adapter else ""
    resolved_adapter = resolved_adapter or None
    resolved_transport = (
        str(
            endpoint_transport
            if endpoint_transport is not None
            else config.get("endpoint_transport") or config.get("_endpoint_transport") or ""
        )
        .strip()
        .lower()
    )
    resolved_contract = str(
        endpoint_contract
        if endpoint_contract is not None
        else config.get("endpoint_contract") or config.get("_endpoint_contract") or ""
    ).strip()
    if resolved_transport == "dds" and resolved_contract == "thunder_dds_v1":
        return "robot"

    resolved_data_source = str(
        data_source_name
        if data_source_name is not None
        else config.get("_profile_adapter_data_source") or config.get("_runtime_contract") or ""
    ).strip()
    if "replay" in resolved_data_source:
        return "replay"
    if resolved_adapter == "mujoco_live":
        return "sim"
    return None


def resolve_runtime_run_spec(
    profile: str,
    config: Mapping[str, Any],
    *,
    record: bool = False,
    extra_args: tuple[str, ...] = (),
) -> RuntimeRunSpec:
    selection_kind = str(
        config.get("_selection_kind")
        or config.get("_profile_adapter_selection_kind")
        or "profile"
    ).lower()
    if selection_kind != "profile":
        raise ProfileAdapterError(
            "Product RunPlans cannot be resolved through Profile adapters"
        )

    binding = profile_data_source(profile)
    data_source_name = str(
        config.get("_profile_adapter_data_source")
        or config.get("_runtime_contract")
        or binding.data_source
    )
    data_source_name = canonical_data_source_name(data_source_name) or data_source_name
    source = DATA_SOURCE_CONTRACTS[data_source_name]
    adapter_name = _adapter_name_for_config(config, data_source_name)
    runtime_contract = config.get("_runtime_contract")
    if not runtime_contract and adapter_name in PROFILE_ADAPTERS:
        runtime_contract = PROFILE_ADAPTERS[adapter_name].runtime_contract
    launcher = config.get("_external_launcher")
    module_transport = module_transport_for_config(
        config,
        default=_default_module_transport(adapter_name),
    )
    endpoint_transport = endpoint_transport_for_config(
        config,
        default=_default_endpoint_transport(adapter_name),
    )
    endpoint_contract = endpoint_contract_for_config(
        config,
        default=_default_endpoint_contract(adapter_name),
    )
    route_contract = route_contract_for_config(
        config,
        adapter_name=adapter_name,
        data_source_name=data_source_name,
        endpoint_transport=endpoint_transport,
        endpoint_contract=endpoint_contract,
    )
    localization_adapter = localization_adapter_for_config(
        config,
        endpoint_transport=endpoint_transport,
        endpoint_contract=endpoint_contract,
    )
    planner_profile = _planner_profile_for_config(config)
    autonomy_backends = resolved_autonomy_backend_selection(
        config,
        enable_native=bool(config.get("enable_native", True)),
    )

    env = {
        "LINGTU_PROFILE": profile,
        "LINGTU_DATA_SOURCE": data_source_name,
        "LINGTU_MODULE_TRANSPORT": module_transport,
        "LINGTU_ENDPOINT_TRANSPORT": endpoint_transport,
        "LINGTU_SIMULATION_ONLY": "1" if source.provider != "hardware" else "0",
    }
    if adapter_name:
        env["LINGTU_PROFILE_ADAPTER"] = str(adapter_name)
    if runtime_contract:
        env["LINGTU_RUNTIME_CONTRACT"] = str(runtime_contract)
    if endpoint_contract:
        env["LINGTU_ENDPOINT_CONTRACT"] = endpoint_contract
    if route_contract:
        env["LINGTU_ROUTE_CONTRACT"] = route_contract
    if localization_adapter:
        env["LINGTU_LOCALIZATION_ADAPTER"] = localization_adapter
    if "enable_robot_driver" in config:
        env["LINGTU_ENABLE_ROBOT_DRIVER"] = "1" if bool(config["enable_robot_driver"]) else "0"
    command_output_mode = str(config.get("command_output_mode") or "").strip()
    if command_output_mode:
        env["LINGTU_COMMAND_OUTPUT_MODE"] = command_output_mode
    hardware_control_boundary = str(config.get("hardware_control_boundary") or "").strip()
    if hardware_control_boundary:
        env["LINGTU_HARDWARE_CONTROL_BOUNDARY"] = hardware_control_boundary
    env["LINGTU_COMMAND_SINK"] = source.command_sink
    if planner_profile["primary"]:
        env["LINGTU_NAV_GLOBAL_PLANNER"] = str(planner_profile["primary"])
    if adapter_name in PROFILE_ADAPTERS:
        _merge_profile_adapter_env_overrides(env, PROFILE_ADAPTERS[adapter_name])

    return RuntimeRunSpec(
        profile=profile,
        adapter=str(adapter_name) if adapter_name else None,
        data_source=data_source_name,
        runtime_contract=str(runtime_contract) if runtime_contract else None,
        module_transport=module_transport,
        endpoint_transport=endpoint_transport,
        endpoint_contract=endpoint_contract or None,
        route_contract=route_contract,
        localization_adapter=localization_adapter or None,
        simulation_only=source.provider != "hardware",
        command_sink=source.command_sink,
        slam_source=source.slam_source,
        localization_source=source.localization_source,
        mapping_source=source.mapping_source,
        lidar_extrinsic_profile=source.lidar_extrinsic_profile,
        frames=asdict(FRAMES),
        frame_links={name: asdict(link) for name, link in FRAME_LINKS.items()},
        topic_allowed_frame_ids=runtime_topic_allowed_frame_ids(
            str(runtime_contract) if runtime_contract else data_source_name
        ),
        topic_default_frame_ids=runtime_topic_default_frame_ids(
            str(runtime_contract) if runtime_contract else data_source_name
        ),
        resolved_runtime_data_flow=tuple(asdict(stage) for stage in resolved_runtime_data_flow(data_source_name)),
        runtime_data_flow_stage_algorithm_interfaces={
            name: tuple(interfaces) for name, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
        },
        launcher=str(launcher) if launcher else None,
        launcher_args=(
            _launcher_args_for_config(
                config,
                profile=profile,
                adapter_name=adapter_name,
                launcher=launcher,
                record=record,
                extra_args=extra_args,
            )
            if launcher
            else ()
        ),
        env=env,
        profile_semantic_overrides=normalized_profile_semantic_overrides(config.get("_profile_semantic_overrides")),
        global_planner=planner_profile["primary"],
        fallback_global_planners=planner_profile["fallback_planners"],
        planner_latency_budget_ms=planner_profile["latency_budget_ms"],
        plan_safety_policy=planner_profile["plan_safety_policy"],
        autonomy_backends=autonomy_backends,
    )



def _planner_profile_for_config(config: Mapping[str, Any]) -> dict[str, Any]:
    raw_profile = config.get("planner_profile")
    if isinstance(raw_profile, Mapping):
        primary = str(raw_profile.get("primary") or config.get("planner") or "").strip()
        fallback_planners = _string_tuple(raw_profile.get("fallback_planners"))
        latency_budget = raw_profile.get(
            "latency_budget_ms",
            config.get("planner_latency_budget_ms"),
        )
        safety_policy = str(raw_profile.get("plan_safety_policy") or config.get("plan_safety_policy") or "").strip()
    else:
        primary = str(config.get("planner") or "").strip()
        fallback_planners = _string_tuple(config.get("fallback_planners"))
        if not fallback_planners:
            fallback_planners = _string_tuple(config.get("fallback_planner_name"))
        latency_budget = config.get("planner_latency_budget_ms")
        safety_policy = str(config.get("plan_safety_policy") or "").strip()

    return {
        "primary": primary or None,
        "fallback_planners": fallback_planners,
        "latency_budget_ms": _optional_int(latency_budget),
        "plan_safety_policy": safety_policy or None,
    }


def _string_tuple(value: Any) -> tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value.strip(),) if value.strip() else ()
    try:
        values = tuple(value)
    except TypeError:
        values = (value,)
    return tuple(str(item).strip() for item in values if str(item).strip())


def _optional_int(value: Any) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _default_module_transport(adapter_name: str | None) -> str:
    if adapter_name in PROFILE_ADAPTERS:
        return PROFILE_ADAPTERS[str(adapter_name)].module_transport
    return "local"


def _default_endpoint_transport(adapter_name: str | None) -> str:
    if adapter_name in PROFILE_ADAPTERS:
        return PROFILE_ADAPTERS[str(adapter_name)].endpoint_transport
    return "local"


def _default_endpoint_contract(adapter_name: str | None) -> str:
    if adapter_name and adapter_name in PROFILE_ADAPTERS:
        return PROFILE_ADAPTERS[str(adapter_name)].endpoint_contract or ""
    return ""


def _merge_profile_adapter_env_overrides(
    env: dict[str, str],
    adapter: ProfileAdapterSpec,
) -> None:
    for raw_key, raw_value in adapter.env_overrides.items():
        key = str(raw_key)
        if key in _RESOLVER_OWNED_ENV_KEYS:
            raise ProfileAdapterError(
                f"profile adapter '{adapter.name}' env_overrides cannot override "
                f"resolver-owned env key '{key}'"
            )
        env[key] = str(raw_value)
