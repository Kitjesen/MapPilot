"""Runtime endpoint resolver for the product/connection split.

Endpoint definitions live in :mod:`core.blueprints.catalog.endpoints`. This
module applies those definitions to product profiles and compiles runtime run
specs for CLI/audit tooling.
"""

from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
from typing import Any, Mapping

from core.blueprints.catalog.endpoints import (
    COMPAT_RUNTIME_ENDPOINT_ALIASES,
    PRODUCT_RUNTIME_ENDPOINT_ALIASES,
    RUNTIME_ENDPOINT_ALIASES,
    RUNTIME_ENDPOINTS,
    RuntimeEndpointError,
    RuntimeEndpointSpec,
    RuntimeRunSpec,
)
from core.runtime_interface import (
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

PRODUCT_SEMANTIC_CONFIG_KEYS: tuple[str, ...] = (
    "slam_profile",
    "planner",
    "planner_backend",
    "llm",
    "tomogram",
    "plan_safety_policy",
    "fallback_planner_name",
    "enable_semantic",
    "enable_map_modules",
    "enable_frontier",
    "exploration_backend",
    "frontier_safe_distance",
    "frontier_max_dist",
    "frontier_rate",
    "tare_scenario",
    "exploration_auto_start",
    "safe_goal_tolerance",
    "waypoint_threshold",
    "final_waypoint_threshold",
    "stuck_timeout",
    "stuck_dist_thre",
    "downsample_dist",
    "path_follower_goal_tolerance",
    "local_planner_allow_direct_track_fallback",
    "local_planner_ignore_near_field_stop",
    "local_planner_direct_track_fallback_min_distance_m",
    "allow_direct_goal_fallback",
    "direct_goal_fallback_on_planner_failure",
    "accept_partial_goal_progress",
    "prefer_path_strategy",
    "path_start_tolerance_m",
    "path_goal_min_distance_m",
    "path_goal_spacing_m",
    "external_strategy_path_control",
    "external_strategy_start_tolerance_m",
    "hold_active_goal_until_terminal",
    "max_waypoint_distance_m",
    "waypoint_odometry_timeout_s",
    "defer_empty_path_planning_failure",
    "empty_path_retry_interval_s",
    "empty_path_retry_timeout_s",
    "planning_frame_id",
    "goal_frame_id",
    "occupancy_frame_id",
)

_UNSET = object()


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
    merged = dict(config)
    endpoint_config = endpoint.config_for_profile(profile)
    merged.update(endpoint_config)
    merged["_product_semantic_overrides"] = _product_semantic_overrides(
        config,
        merged,
    )
    return merged


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
    module_transport = _module_transport_for_config(config, endpoint_name)
    endpoint_transport = _endpoint_transport_for_config(config, endpoint_name)
    endpoint_contract = _endpoint_contract_for_config(config, endpoint_name)
    localization_adapter = _localization_adapter_for_config(
        config,
        endpoint_transport=endpoint_transport,
        endpoint_contract=endpoint_contract,
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
        product_semantic_overrides=_normalized_product_semantic_overrides(
            config.get("_product_semantic_overrides")
        ),
    )


compile_runtime_run_spec = resolve_runtime_run_spec


def _module_transport_for_config(
    config: Mapping[str, Any],
    endpoint_name: str | None,
) -> str:
    value = config.get("module_transport") or config.get("_module_transport")
    if value:
        return str(value)
    if endpoint_name in RUNTIME_ENDPOINTS:
        return RUNTIME_ENDPOINTS[str(endpoint_name)].module_transport
    return "local"


def _endpoint_transport_for_config(
    config: Mapping[str, Any],
    endpoint_name: str | None,
) -> str:
    value = config.get("endpoint_transport") or config.get("_endpoint_transport")
    if value:
        return str(value)
    if endpoint_name in RUNTIME_ENDPOINTS:
        return RUNTIME_ENDPOINTS[str(endpoint_name)].endpoint_transport
    return "local"


def _endpoint_contract_for_config(
    config: Mapping[str, Any],
    endpoint_name: str | None,
) -> str:
    value = config.get("endpoint_contract") or config.get("_endpoint_contract")
    if value:
        return str(value).strip()
    if endpoint_name and endpoint_name in RUNTIME_ENDPOINTS:
        return RUNTIME_ENDPOINTS[str(endpoint_name)].endpoint_contract or ""
    return ""


def _localization_adapter_for_config(
    config: Mapping[str, Any],
    *,
    endpoint_transport: str,
    endpoint_contract: str,
) -> str:
    value = config.get("localization_adapter") or config.get("_localization_adapter")
    if value:
        return str(value).strip()
    if endpoint_transport == "lcm" and endpoint_contract:
        return "lcm_endpoint"
    return ""


def _product_semantic_overrides(
    product_config: Mapping[str, Any],
    endpoint_config: Mapping[str, Any],
) -> tuple[dict[str, object], ...]:
    overrides: list[dict[str, object]] = []
    for key in PRODUCT_SEMANTIC_CONFIG_KEYS:
        product_value = product_config.get(key, _UNSET)
        endpoint_value = endpoint_config.get(key, _UNSET)
        if product_value is _UNSET and endpoint_value is _UNSET:
            continue
        if product_value == endpoint_value:
            continue
        overrides.append(
            {
                "field": key,
                "override_scope": "compatibility_override",
                "product_value": _json_config_value(product_value),
                "endpoint_value": _json_config_value(endpoint_value),
            }
        )
    return tuple(overrides)


def _normalized_product_semantic_overrides(
    value: Any,
) -> tuple[Mapping[str, Any], ...]:
    if not isinstance(value, (list, tuple)):
        return ()
    normalized: list[Mapping[str, Any]] = []
    for item in value:
        if not isinstance(item, Mapping):
            continue
        field = item.get("field")
        if not field:
            continue
        normalized.append(
            {
                "field": str(field),
                "override_scope": str(
                    item.get("override_scope") or "compatibility_override"
                ),
                "product_value": _json_config_value(item.get("product_value")),
                "endpoint_value": _json_config_value(item.get("endpoint_value")),
            }
        )
    return tuple(normalized)


def _json_config_value(value: Any) -> object:
    if value is _UNSET:
        return "<unset>"
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, tuple):
        return [_json_config_value(item) for item in value]
    if isinstance(value, list):
        return [_json_config_value(item) for item in value]
    if isinstance(value, Mapping):
        return {str(key): _json_config_value(item) for key, item in value.items()}
    return value
