"""Local Host Profile adapter composition helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Protocol


class ProfileAdapterConfigSpec(Protocol):
    """Adapter fields required to build a Profile-specific config layer."""

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
        """Raise when the adapter does not support the Profile."""
        ...

PROFILE_SEMANTIC_CONFIG_KEYS: tuple[str, ...] = (
    "slam_profile",
    "planner",
    "planner_backend",
    "planner_latency_budget_ms",
    "llm",
    "map_path",
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


def profile_adapter_config_for_profile(
    adapter: ProfileAdapterConfigSpec,
    profile: str,
) -> dict[str, Any]:
    """Return adapter config for one supported Host Profile."""

    adapter.require_profile(profile)
    return _profile_adapter_config(
        adapter,
        profile,
        overrides=adapter.profile_overrides,
        subject="profile",
    )


def _profile_adapter_config(
    adapter: ProfileAdapterConfigSpec,
    name: str,
    *,
    overrides: Mapping[str, Mapping[str, Any]],
    subject: str,
) -> dict[str, Any]:
    merged = dict(adapter.config_overrides)
    merged.update(overrides.get(name, {}))
    merged["_profile_adapter"] = adapter.name
    merged["_profile_adapter_selection_kind"] = subject.lower()
    merged["_profile_adapter_data_source"] = adapter.data_source
    merged["_module_transport"] = adapter.module_transport
    merged["_endpoint_transport"] = adapter.endpoint_transport
    if adapter.endpoint_contract:
        merged["_endpoint_contract"] = adapter.endpoint_contract
    if adapter.external_launcher:
        merged["_external_launcher"] = adapter.external_launcher
    if adapter.runtime_contract:
        merged["_runtime_contract"] = adapter.runtime_contract
    if adapter.default_actions:
        if name not in adapter.default_actions:
            from runtime.profiles.catalog.profile_adapters import ProfileAdapterError

            raise ProfileAdapterError(
                f"profile adapter '{adapter.name}' default_actions missing {subject} '{name}'"
            )
        merged["_external_default_args"] = adapter.default_actions[name]
    if adapter.record_actions:
        if name not in adapter.record_actions:
            from runtime.profiles.catalog.profile_adapters import ProfileAdapterError

            raise ProfileAdapterError(
                f"profile adapter '{adapter.name}' record_actions missing {subject} '{name}'"
            )
        merged["_external_record_args"] = adapter.record_actions[name]
    return merged


def merge_profile_adapter_config(
    profile_config: Mapping[str, Any],
    adapter_config: Mapping[str, Any],
) -> dict[str, Any]:
    """Apply an adapter layer to a local Host Profile config."""

    merged = dict(profile_config)
    merged.update(adapter_config)
    merged["_profile_semantic_overrides"] = profile_semantic_overrides(
        profile_config,
        merged,
    )
    return merged


def profile_semantic_overrides(
    profile_config: Mapping[str, Any],
    adapter_config: Mapping[str, Any],
) -> tuple[dict[str, object], ...]:
    """Return adapter overrides that affect Profile-level behavior."""

    overrides: list[dict[str, object]] = []
    for key in PROFILE_SEMANTIC_CONFIG_KEYS:
        profile_value = profile_config.get(key, _UNSET)
        adapter_value = adapter_config.get(key, _UNSET)
        if profile_value is _UNSET and adapter_value is _UNSET:
            continue
        if profile_value == adapter_value:
            continue
        overrides.append(
            {
                "field": key,
                "override_scope": "profile_adapter_override",
                "profile_value": json_config_value(profile_value),
                "adapter_value": json_config_value(adapter_value),
            }
        )
    return tuple(overrides)


def normalized_profile_semantic_overrides(
    value: Any,
) -> tuple[Mapping[str, Any], ...]:
    """Normalize persisted Profile adapter override diagnostics."""

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
                "override_scope": str(item.get("override_scope") or "profile_adapter_override"),
                "profile_value": json_config_value(item.get("profile_value")),
                "adapter_value": json_config_value(item.get("adapter_value")),
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
