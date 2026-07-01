"""Runtime endpoint resolver for the product/connection split.

Endpoint definitions live in :mod:`runtime.profiles.catalog.endpoints`. This
module applies those definitions to product profiles and compiles runtime run
specs for CLI/audit tooling.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import Any, Mapping

from runtime.profiles.catalog.endpoints import (
    COMPAT_RUNTIME_ENDPOINT_ALIASES,
    PRODUCT_PROFILE_ENDPOINTS,
    PRODUCT_RUNTIME_ENDPOINT_ALIASES,
    RUNTIME_ENDPOINT_ALIASES,
    RUNTIME_ENDPOINTS,
    RuntimeEndpointError,
    RuntimeEndpointSpec,
    RuntimeRunSpec,
)
from runtime.profiles.binding_policy import (
    endpoint_contract_for_config,
    endpoint_transport_for_config,
    localization_adapter_for_config,
    module_transport_for_config,
    resolved_autonomy_backend_selection,
)
from runtime.profiles.endpoint_config import (
    endpoint_config_for_profile,
    merge_runtime_endpoint_config,
    normalized_product_semantic_overrides,
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
        "LINGTU_ENDPOINT",
        "LINGTU_DATA_SOURCE",
        "LINGTU_MODULE_TRANSPORT",
        "LINGTU_ENDPOINT_TRANSPORT",
        "LINGTU_RUNTIME_CONTRACT",
        "LINGTU_ENDPOINT_CONTRACT",
        "LINGTU_LOCALIZATION_ADAPTER",
        "LINGTU_NAV_IN_ADAPTER",
        "LINGTU_NAV_OUT_ADAPTER",
        "LINGTU_ENABLE_ROBOT_DRIVER",
        "LINGTU_COMMAND_OUTPUT_MODE",
        "LINGTU_HARDWARE_CONTROL_BOUNDARY",
        "LINGTU_COMMAND_SINK",
        "LINGTU_SIMULATION_ONLY",
    }
)


def canonical_runtime_endpoint_name(name: str) -> str:
    return RUNTIME_ENDPOINT_ALIASES.get(str(name), str(name))


def runtime_endpoint(name: str) -> RuntimeEndpointSpec:
    endpoint_name = canonical_runtime_endpoint_name(name)
    try:
        return RUNTIME_ENDPOINTS[endpoint_name]
    except KeyError as exc:
        choices = ", ".join(runtime_endpoint_names(include_aliases=True))
        raise RuntimeEndpointError(
            f"unknown runtime endpoint '{name}' (choices: {choices})"
        ) from exc


def runtime_endpoint_names(
    *,
    include_aliases: bool = False,
    include_compat_aliases: bool = False,
) -> tuple[str, ...]:
    names = list(RUNTIME_ENDPOINTS.keys())
    if include_aliases:
        names.extend(
            alias
            for alias in PRODUCT_RUNTIME_ENDPOINT_ALIASES
            if alias not in RUNTIME_ENDPOINTS
        )
    if include_compat_aliases:
        names.extend(
            alias
            for alias in COMPAT_RUNTIME_ENDPOINT_ALIASES
            if alias not in RUNTIME_ENDPOINTS and alias not in names
        )
    return tuple(names)


def runtime_endpoint_robot_preset(profile: str, endpoint_name: str) -> str:
    endpoint = runtime_endpoint(endpoint_name)
    endpoint.require_profile(profile)
    return endpoint.robot_preset


def apply_runtime_endpoint_config(
    profile: str,
    config: Mapping[str, Any],
    endpoint_name: str,
) -> dict[str, Any]:
    endpoint = runtime_endpoint(endpoint_name)
    endpoint_config = endpoint_config_for_profile(endpoint, profile)
    return merge_runtime_endpoint_config(config, endpoint_config)


def _launcher_args_for_config(
    config: Mapping[str, Any],
    *,
    profile: str,
    endpoint_name: str | None,
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
        endpoint = _endpoint_for_launcher_args(config, endpoint_name, launcher)
        if endpoint is not None and endpoint.record_actions:
            if profile not in endpoint.record_actions:
                raise RuntimeEndpointError(
                    f"endpoint '{endpoint.name}' record_actions missing profile "
                    f"'{profile}'"
                )
            return endpoint.record_actions[profile]
        if explicit_default_args:
            return tuple(explicit_default_args)
        if endpoint is not None and endpoint.default_actions:
            if profile not in endpoint.default_actions:
                raise RuntimeEndpointError(
                    f"endpoint '{endpoint.name}' default_actions missing profile "
                    f"'{profile}'"
                )
            return endpoint.default_actions[profile]
        raise RuntimeEndpointError(
            f"external launcher args missing for profile '{profile}'"
        )

    if explicit_default_args:
        return tuple(explicit_default_args)
    endpoint = _endpoint_for_launcher_args(config, endpoint_name, launcher)
    if endpoint is not None and endpoint.default_actions:
        if profile not in endpoint.default_actions:
            raise RuntimeEndpointError(
                f"endpoint '{endpoint.name}' default_actions missing profile "
                f"'{profile}'"
            )
        return endpoint.default_actions[profile]
    raise RuntimeEndpointError(
        f"external launcher args missing for profile '{profile}'"
    )


def _endpoint_for_launcher_args(
    config: Mapping[str, Any],
    endpoint_name: str | None,
    launcher: Any,
) -> RuntimeEndpointSpec | None:
    if not endpoint_name:
        return None
    canonical_endpoint_name = canonical_runtime_endpoint_name(endpoint_name)
    if canonical_endpoint_name not in RUNTIME_ENDPOINTS:
        return None
    endpoint = RUNTIME_ENDPOINTS[canonical_endpoint_name]
    if not endpoint.external_launcher:
        return None
    if config.get("_runtime_endpoint") == endpoint.name:
        return endpoint
    if str(launcher or "") == endpoint.external_launcher:
        return endpoint
    return None


def _endpoint_name_for_config(
    config: Mapping[str, Any],
    data_source_name: str,
) -> str | None:
    endpoint_name = config.get("_runtime_endpoint")
    if endpoint_name:
        return canonical_runtime_endpoint_name(str(endpoint_name))

    runtime_contract = config.get("_runtime_contract")
    launcher = config.get("_external_launcher")
    for name, endpoint in RUNTIME_ENDPOINTS.items():
        if runtime_contract and endpoint.runtime_contract == runtime_contract:
            return name
        if launcher and endpoint.external_launcher == launcher:
            return name
    for name, endpoint in RUNTIME_ENDPOINTS.items():
        if endpoint.data_source == data_source_name:
            return name
    return None


def resolve_runtime_run_spec(
    profile: str,
    config: Mapping[str, Any],
    *,
    record: bool = False,
    extra_args: tuple[str, ...] = (),
) -> RuntimeRunSpec:
    data_source_name = str(
        config.get("_endpoint_data_source")
        or config.get("_runtime_contract")
        or profile_data_source(profile).data_source
    )
    data_source_name = canonical_data_source_name(data_source_name) or data_source_name
    source = DATA_SOURCE_CONTRACTS[data_source_name]
    endpoint_name = _endpoint_name_for_config(config, data_source_name)
    runtime_contract = config.get("_runtime_contract")
    if not runtime_contract and endpoint_name in RUNTIME_ENDPOINTS:
        runtime_contract = RUNTIME_ENDPOINTS[endpoint_name].runtime_contract
    launcher = config.get("_external_launcher")
    robot_preset = (
        RUNTIME_ENDPOINTS[endpoint_name].robot_preset
        if endpoint_name in RUNTIME_ENDPOINTS
        else str(config.get("robot") or "")
    )
    module_transport = module_transport_for_config(
        config,
        default=_default_module_transport(endpoint_name),
    )
    endpoint_transport = endpoint_transport_for_config(
        config,
        default=_default_endpoint_transport(endpoint_name),
    )
    endpoint_contract = endpoint_contract_for_config(
        config,
        default=_default_endpoint_contract(endpoint_name),
    )
    localization_adapter = localization_adapter_for_config(
        config,
        endpoint_transport=endpoint_transport,
        endpoint_contract=endpoint_contract,
    )
    nav_in_adapter = str(config.get("nav_in_adapter") or "").strip()
    nav_out_adapter = str(config.get("nav_out_adapter") or "").strip()
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
    if endpoint_name:
        env["LINGTU_ENDPOINT"] = str(endpoint_name)
    if runtime_contract:
        env["LINGTU_RUNTIME_CONTRACT"] = str(runtime_contract)
    if endpoint_contract:
        env["LINGTU_ENDPOINT_CONTRACT"] = endpoint_contract
    if localization_adapter:
        env["LINGTU_LOCALIZATION_ADAPTER"] = localization_adapter
    if nav_in_adapter:
        env["LINGTU_NAV_IN_ADAPTER"] = nav_in_adapter
    if nav_out_adapter:
        env["LINGTU_NAV_OUT_ADAPTER"] = nav_out_adapter
    if "enable_robot_driver" in config:
        env["LINGTU_ENABLE_ROBOT_DRIVER"] = (
            "1" if bool(config["enable_robot_driver"]) else "0"
        )
    command_output_mode = str(config.get("command_output_mode") or "").strip()
    if command_output_mode:
        env["LINGTU_COMMAND_OUTPUT_MODE"] = command_output_mode
    hardware_control_boundary = str(config.get("hardware_control_boundary") or "").strip()
    if hardware_control_boundary:
        env["LINGTU_HARDWARE_CONTROL_BOUNDARY"] = hardware_control_boundary
    env["LINGTU_COMMAND_SINK"] = source.command_sink
    if endpoint_name in RUNTIME_ENDPOINTS:
        _merge_endpoint_env_overrides(env, RUNTIME_ENDPOINTS[endpoint_name])

    return RuntimeRunSpec(
        profile=profile,
        endpoint=str(endpoint_name) if endpoint_name else None,
        data_source=data_source_name,
        runtime_contract=str(runtime_contract) if runtime_contract else None,
        robot_preset=robot_preset or None,
        module_transport=module_transport,
        endpoint_transport=endpoint_transport,
        endpoint_contract=endpoint_contract or None,
        localization_adapter=localization_adapter or None,
        nav_in_adapter=nav_in_adapter or None,
        nav_out_adapter=nav_out_adapter or None,
        simulation_only=source.provider != "hardware",
        command_sink=source.command_sink,
        slam_source=source.slam_source,
        localization_source=source.localization_source,
        mapping_source=source.mapping_source,
        lidar_extrinsic_profile=source.lidar_extrinsic_profile,
        frames=asdict(FRAMES),
        frame_links={
            name: asdict(link)
            for name, link in FRAME_LINKS.items()
        },
        topic_allowed_frame_ids=runtime_topic_allowed_frame_ids(
            str(runtime_contract) if runtime_contract else data_source_name
        ),
        topic_default_frame_ids=runtime_topic_default_frame_ids(
            str(runtime_contract) if runtime_contract else data_source_name
        ),
        resolved_runtime_data_flow=tuple(
            asdict(stage)
            for stage in resolved_runtime_data_flow(data_source_name)
        ),
        runtime_data_flow_stage_algorithm_interfaces={
            name: tuple(interfaces)
            for name, interfaces in RUNTIME_DATA_FLOW_STAGE_ALGORITHM_INTERFACES.items()
        },
        launcher=str(launcher) if launcher else None,
        launcher_args=(
            _launcher_args_for_config(
                config,
                profile=profile,
                endpoint_name=endpoint_name,
                launcher=launcher,
                record=record,
                extra_args=extra_args,
            )
            if launcher
            else ()
        ),
        env=env,
        product_semantic_overrides=normalized_product_semantic_overrides(
            config.get("_product_semantic_overrides")
        ),
        global_planner=planner_profile["primary"],
        fallback_global_planners=planner_profile["fallback_planners"],
        planner_latency_budget_ms=planner_profile["latency_budget_ms"],
        plan_safety_policy=planner_profile["plan_safety_policy"],
        autonomy_backends=autonomy_backends,
    )


compile_runtime_run_spec = resolve_runtime_run_spec


def _planner_profile_for_config(config: Mapping[str, Any]) -> dict[str, Any]:
    raw_profile = config.get("planner_profile")
    if isinstance(raw_profile, Mapping):
        primary = str(raw_profile.get("primary") or config.get("planner") or "").strip()
        fallback_planners = _string_tuple(raw_profile.get("fallback_planners"))
        latency_budget = raw_profile.get(
            "latency_budget_ms",
            config.get("planner_latency_budget_ms"),
        )
        safety_policy = str(
            raw_profile.get("plan_safety_policy")
            or config.get("plan_safety_policy")
            or ""
        ).strip()
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


def _default_module_transport(endpoint_name: str | None) -> str:
    if endpoint_name in RUNTIME_ENDPOINTS:
        return RUNTIME_ENDPOINTS[str(endpoint_name)].module_transport
    return "local"


def _default_endpoint_transport(endpoint_name: str | None) -> str:
    if endpoint_name in RUNTIME_ENDPOINTS:
        return RUNTIME_ENDPOINTS[str(endpoint_name)].endpoint_transport
    return "local"


def _default_endpoint_contract(endpoint_name: str | None) -> str:
    if endpoint_name and endpoint_name in RUNTIME_ENDPOINTS:
        return RUNTIME_ENDPOINTS[str(endpoint_name)].endpoint_contract or ""
    return ""


def _merge_endpoint_env_overrides(
    env: dict[str, str],
    endpoint: RuntimeEndpointSpec,
) -> None:
    for raw_key, raw_value in endpoint.env_overrides.items():
        key = str(raw_key)
        if key in _RESOLVER_OWNED_ENV_KEYS:
            raise RuntimeEndpointError(
                f"endpoint '{endpoint.name}' env_overrides cannot override "
                f"resolver-owned env key '{key}'"
            )
        env[key] = str(raw_value)
