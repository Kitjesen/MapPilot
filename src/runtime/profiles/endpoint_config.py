"""Endpoint adapter config composition helpers.

Runtime endpoint catalog entries describe where data and commands cross the
runtime boundary. This module owns how those endpoint entries are applied to a
resolved product/robot config.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Protocol


class RuntimeEndpointConfigSpec(Protocol):
    """Endpoint fields required to build a profile-specific config layer."""

    name: str
    data_source: str
    module_transport: str
    endpoint_transport: str
    endpoint_contract: str | None
    external_launcher: str | None
    runtime_contract: str | None
    config_overrides: Mapping[str, Any]
    profile_overrides: Mapping[str, Mapping[str, Any]]
    default_actions: Mapping[str, tuple[str, ...]]
    record_actions: Mapping[str, tuple[str, ...]]

    def require_profile(self, profile: str) -> None:
        """Raise when the endpoint does not support the profile."""
        ...


PRODUCT_SEMANTIC_CONFIG_KEYS: tuple[str, ...] = (
    "slam_profile",
    "planner",
    "planner_backend",
    "planner_latency_budget_ms",
    "llm",
    "tomogram",
    "plan_safety_policy",
    "fallback_planner_name",
    "fallback_planners",
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
    "path_follower_native_max_accel",
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


def endpoint_config_for_profile(
    endpoint: RuntimeEndpointConfigSpec,
    profile: str,
) -> dict[str, Any]:
    """Return endpoint adapter config for one supported product/sim profile."""

    endpoint.require_profile(profile)
    merged = dict(endpoint.config_overrides)
    merged.update(endpoint.profile_overrides.get(profile, {}))
    merged["_runtime_endpoint"] = endpoint.name
    merged["_endpoint_data_source"] = endpoint.data_source
    merged["_module_transport"] = endpoint.module_transport
    merged["_endpoint_transport"] = endpoint.endpoint_transport
    if endpoint.endpoint_contract:
        merged["_endpoint_contract"] = endpoint.endpoint_contract
    if endpoint.external_launcher:
        merged["_external_launcher"] = endpoint.external_launcher
    if endpoint.runtime_contract:
        merged["_runtime_contract"] = endpoint.runtime_contract
    if endpoint.default_actions:
        if profile not in endpoint.default_actions:
            from runtime.profiles.catalog.endpoints import RuntimeEndpointError

            raise RuntimeEndpointError(
                f"endpoint '{endpoint.name}' default_actions missing profile "
                f"'{profile}'"
            )
        merged["_external_default_args"] = endpoint.default_actions[profile]
    if endpoint.record_actions:
        if profile not in endpoint.record_actions:
            from runtime.profiles.catalog.endpoints import RuntimeEndpointError

            raise RuntimeEndpointError(
                f"endpoint '{endpoint.name}' record_actions missing profile "
                f"'{profile}'"
            )
        merged["_external_record_args"] = endpoint.record_actions[profile]
    return merged


def merge_runtime_endpoint_config(
    product_config: Mapping[str, Any],
    endpoint_config: Mapping[str, Any],
) -> dict[str, Any]:
    """Apply an endpoint adapter layer to product/robot config."""

    merged = dict(product_config)
    merged.update(endpoint_config)
    merged["_product_semantic_overrides"] = product_semantic_overrides(
        product_config,
        merged,
    )
    return merged


def product_semantic_overrides(
    product_config: Mapping[str, Any],
    endpoint_config: Mapping[str, Any],
) -> tuple[dict[str, object], ...]:
    """Return endpoint overrides that affect product-level behavior."""

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
                "product_value": json_config_value(product_value),
                "endpoint_value": json_config_value(endpoint_value),
            }
        )
    return tuple(overrides)


def normalized_product_semantic_overrides(
    value: Any,
) -> tuple[Mapping[str, Any], ...]:
    """Normalize persisted product semantic override diagnostics."""

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
                "product_value": json_config_value(item.get("product_value")),
                "endpoint_value": json_config_value(item.get("endpoint_value")),
            }
        )
    return tuple(normalized)


def json_config_value(value: Any) -> object:
    """Return a JSON-friendly representation of a config value."""

    if value is _UNSET:
        return "<unset>"
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, tuple):
        return [json_config_value(item) for item in value]
    if isinstance(value, list):
        return [json_config_value(item) for item in value]
    if isinstance(value, Mapping):
        return {str(key): json_config_value(item) for key, item in value.items()}
    return value
