"""Runtime binding policy for endpoint transports, adapters, and backends.

Blueprint stacks add Modules. Adapter modules implement external protocols.
This L1 runtime model owns the small set of rules that decides which runtime
binding family a resolved config is asking for.
"""

from __future__ import annotations

from typing import Any, Mapping

from runtime.profiles.planner_backends import (
    normalize_planner_name,
    planner_fallback_chain,
)

DDS_MAP_OUTPUT_ADAPTERS = frozenset({"dds", "dds_endpoint", "dds_map_output"})
NAV_IN_ENABLE_KEYS = ("enable_nav_in",)
NAV_OUT_ENABLE_KEYS = ("enable_nav_out",)
LEGACY_NAV_IN_ENABLE_KEYS = (
    "enable_endpoint_command_bridge",
    "enable_ros2_command_bridge",
)
LEGACY_NAV_OUT_ENABLE_KEYS = (
    "enable_endpoint_path_bridge",
    "enable_endpoint_waypoint_bridge",
    "enable_ros2_bridge",
)
MAP_OUT_ENABLE_KEYS = ("enable_map_out",)
LEGACY_MAP_OUT_ENABLE_KEYS = (
    "enable_endpoint_grid_bridge",
    "enable_ros2_grid_bridge",
)
IO_ADAPTER_ENABLE_KEYS = (
    *NAV_IN_ENABLE_KEYS,
    *NAV_OUT_ENABLE_KEYS,
    *LEGACY_NAV_IN_ENABLE_KEYS,
    *LEGACY_NAV_OUT_ENABLE_KEYS,
    *MAP_OUT_ENABLE_KEYS,
    *LEGACY_MAP_OUT_ENABLE_KEYS,
)
ENDPOINT_ADAPTER_ENABLE_KEYS = IO_ADAPTER_ENABLE_KEYS
ROS2_ADAPTER_NAMES = frozenset(
    {
        "ros2",
        "ros2_slam_bridge",
        "ros2_map_output",
        "ros2_grid_bridge",
        "ros2_nav_input",
        "ros2_nav_output",
        # Legacy selector value resolves to ros2_nav_input; do not register a module for it.
        "ros2_navigation_command_bridge",
    }
)
ROS2_DRIVER_RUNTIMES = frozenset(
    {
        "ros2",
        "sim_ros2",
        "ros2_bridge",
        "ros2simdrivermodule",
    }
)
LIDAR_LEGACY_DRIVER_START_KEYS = (
    "lidar_start_driver",
    "start_lidar_driver",
)
LEGACY_SENSOR_BINDING_KEYS = (
    "use_driver_camera",
    "use_driver_lidar",
    "use_driver_imu",
    "legacy_driver_sensor_fallback",
    "enable_legacy_sim_lidar",
)
ROS2_CAMERA_BRIDGE_ENABLE_KEYS = ("enable_ros2_camera_bridge",)
ROS2_RERUN_BRIDGE_ENABLE_KEYS = ("enable_ros2_rerun_bridge",)
LOCALIZATION_ADAPTER_KEYS = (
    "localization_adapter",
    "_localization_adapter",
)
NAV_OUT_ADAPTER_KEYS = ("nav_out_adapter",)
LEGACY_NAV_OUT_ADAPTER_KEYS = (
    "endpoint_path_bridge",
    "endpoint_egress_adapter",
)
NAV_IN_ADAPTER_KEYS = ("nav_in_adapter",)
LEGACY_NAV_IN_ADAPTER_KEYS = (
    "endpoint_command_bridge",
    "endpoint_ingress_adapter",
)
MAP_OUT_ADAPTER_KEYS = ("map_out_adapter",)
LEGACY_MAP_OUT_ADAPTER_KEYS = (
    "endpoint_grid_bridge",
    "endpoint_grid_adapter",
)
IO_ADAPTER_KEYS = (
    *NAV_OUT_ADAPTER_KEYS,
    *LEGACY_NAV_OUT_ADAPTER_KEYS,
    *NAV_IN_ADAPTER_KEYS,
    *LEGACY_NAV_IN_ADAPTER_KEYS,
    *MAP_OUT_ADAPTER_KEYS,
    *LEGACY_MAP_OUT_ADAPTER_KEYS,
)
ENDPOINT_ADAPTER_KEYS = IO_ADAPTER_KEYS
DRIVER_RUNTIME_KEYS = (
    "robot",
    "_default_robot",
    "driver",
    "driver_runtime",
    "driver_module",
)
AUTONOMY_TERRAIN_BACKENDS = frozenset({"nanobind", "simple"})
ROS2_TERRAIN_BACKENDS = frozenset({"native", "cmu"})
ROS2_LOCAL_PLANNER_BACKENDS = frozenset({"cmu"})
ROS2_PATH_FOLLOWER_BACKENDS = frozenset({"pure_pursuit"})
NAV_KERNEL_TERRAIN_BACKENDS = frozenset({"nanobind"})
NAV_KERNEL_LOCAL_PLANNER_BACKENDS = frozenset({"nanobind"})
NAV_KERNEL_PATH_FOLLOWER_BACKENDS = frozenset({"nav_kernel"})
ROS2_EXPLORATION_BACKENDS = frozenset(
    {
        "ros2_tare",
        "tare_native",
        "tare_native_module",
        "tare_ros2",
        "tare_ros2_bridge",
    }
)
ROS2_GLOBAL_PLANNER_BACKENDS = frozenset(
    {
        "ros2",
        "ros2_global_planner",
        "ros2_octoplanner3d",
        "octoplanner3d_ros2",
    }
)
DEFAULT_TERRAIN_BACKEND = "nanobind"


def _string_value(config: Mapping[str, Any], *keys: str) -> str:
    for key in keys:
        value = config.get(key)
        if value:
            return str(value).strip()
    return ""


def module_transport_for_config(
    config: Mapping[str, Any],
    *,
    default: str = "local",
) -> str:
    return _string_value(config, "module_transport", "_module_transport") or default


def endpoint_transport_for_config(
    config: Mapping[str, Any],
    *,
    default: str = "local",
) -> str:
    return _string_value(config, "endpoint_transport", "_endpoint_transport") or default


def endpoint_contract_for_config(
    config: Mapping[str, Any],
    *,
    default: str = "",
) -> str:
    return _string_value(config, "endpoint_contract", "_endpoint_contract") or default


def localization_adapter_for_config(
    config: Mapping[str, Any],
    *,
    endpoint_transport: str | None = None,
    endpoint_contract: str | None = None,
) -> str:
    explicit = _string_value(config, "localization_adapter", "_localization_adapter")
    if explicit:
        return explicit
    resolved_transport = (
        endpoint_transport if endpoint_transport is not None else endpoint_transport_for_config(config, default="")
    )
    if str(resolved_transport).lower() == "dds":
        return "dds_endpoint"
    return ""


def navigation_output_uses_dds(config: Mapping[str, Any]) -> bool:
    return False


def navigation_output_uses_ros2(config: Mapping[str, Any]) -> bool:
    selected = _string_value(
        config,
        *NAV_OUT_ADAPTER_KEYS,
        *LEGACY_NAV_OUT_ADAPTER_KEYS,
    ).lower()
    return (
        selected in ROS2_ADAPTER_NAMES or selected.startswith("ros2_") or bool(config.get("enable_ros2_bridge", False))
    )


def navigation_input_uses_dds(config: Mapping[str, Any]) -> bool:
    return False


def navigation_input_uses_ros2(config: Mapping[str, Any]) -> bool:
    selected = _string_value(
        config,
        *NAV_IN_ADAPTER_KEYS,
        *LEGACY_NAV_IN_ADAPTER_KEYS,
    ).lower()
    return (
        selected in ROS2_ADAPTER_NAMES
        or selected.startswith("ros2_")
        or bool(config.get("enable_ros2_command_bridge", False))
    )


def map_output_uses_ros2(config: Mapping[str, Any]) -> bool:
    selected = _string_value(
        config,
        *MAP_OUT_ADAPTER_KEYS,
        *LEGACY_MAP_OUT_ADAPTER_KEYS,
    ).lower()
    return (
        selected in ROS2_ADAPTER_NAMES
        or selected.startswith("ros2_")
        or bool(config.get("enable_ros2_grid_bridge", False))
    )


def map_output_uses_dds(config: Mapping[str, Any]) -> bool:
    selected = _string_value(
        config,
        *MAP_OUT_ADAPTER_KEYS,
        *LEGACY_MAP_OUT_ADAPTER_KEYS,
    ).lower()
    endpoint_transport = endpoint_transport_for_config(config, default="").lower()
    return selected in DDS_MAP_OUTPUT_ADAPTERS or (not selected and endpoint_transport == "dds")


def map_output_adapter_enabled(config: Mapping[str, Any]) -> bool:
    return any(bool(config.get(key)) for key in (*MAP_OUT_ENABLE_KEYS, *LEGACY_MAP_OUT_ENABLE_KEYS))


def autonomy_backend_selection(
    config: Mapping[str, Any],
    *,
    enable_native: bool = True,
) -> dict[str, Any]:
    """Resolve runtime-selected autonomy backends without importing Modules."""

    if enable_native:
        local_planner_backend = config.get("local_planner_backend", "nanobind")
        path_follower_backend = config.get("path_follower_backend", "nav_kernel")
    else:
        local_planner_backend = config.get(
            "local_planner_backend",
            config.get("python_autonomy_backend", "nanobind"),
        )
        path_follower_backend = config.get(
            "path_follower_backend",
            config.get("python_path_follower_backend", "nav_kernel"),
        )

    return {
        "terrain_backend": config.get("terrain_backend"),
        "local_planner_backend": local_planner_backend,
        "path_follower_backend": path_follower_backend,
    }


def global_planner_backend_selection(config: Mapping[str, Any]) -> dict[str, list[str] | str]:
    """Resolve the configured global planner backends without importing plugins."""

    primary = normalize_planner_name(_string_value(config, "planner", "planner_backend") or "octoplanner3d")
    fallback_value = config.get("fallback_planners")
    if fallback_value is None:
        fallback_value = config.get("fallback_planner_name")
    return {
        "primary": primary,
        "fallback_planners": planner_fallback_chain(fallback_value),
    }


def resolved_autonomy_backend_selection(
    config: Mapping[str, Any],
    *,
    enable_native: bool = True,
) -> dict[str, str]:
    """Resolve the effective terrain/local/path-follower backend names."""

    selected = autonomy_backend_selection(config, enable_native=enable_native)
    local_planner_backend = str(selected["local_planner_backend"] or "").strip()
    path_follower_backend = str(selected["path_follower_backend"] or "").strip()
    terrain_backend = selected.get("terrain_backend")
    terrain_name = str(terrain_backend or "").strip()
    if not terrain_name:
        terrain_name = (
            local_planner_backend if local_planner_backend in AUTONOMY_TERRAIN_BACKENDS else DEFAULT_TERRAIN_BACKEND
        )
    return {
        "terrain_backend": terrain_name,
        "local_planner_backend": local_planner_backend,
        "path_follower_backend": path_follower_backend,
    }


def nav_kernel_backend_required(
    config: Mapping[str, Any],
    *,
    enable_native: bool = True,
) -> bool:
    """Return true when the resolved autonomy chain needs the native nav kernel."""

    selected = resolved_autonomy_backend_selection(
        config,
        enable_native=enable_native,
    )
    return (
        selected["terrain_backend"] in NAV_KERNEL_TERRAIN_BACKENDS
        or selected["local_planner_backend"] in NAV_KERNEL_LOCAL_PLANNER_BACKENDS
        or selected["path_follower_backend"] in NAV_KERNEL_PATH_FOLLOWER_BACKENDS
    )


def exploration_backend_for_config(config: Mapping[str, Any]) -> str:
    """Return the selected exploration backend name."""

    return _adapter_name(config.get("exploration_backend") or "none")


def ros2_autonomy_backend_violations(
    config: Mapping[str, Any],
    *,
    enable_native: bool = True,
) -> list[str]:
    """Return selected autonomy backends that still require ROS2 NativeModule."""

    selected = resolved_autonomy_backend_selection(
        config,
        enable_native=enable_native,
    )
    violations: list[str] = []
    if selected["terrain_backend"] in ROS2_TERRAIN_BACKENDS:
        violations.append(f"terrain_backend={selected['terrain_backend']} requires ROS2 NativeModule")
    if selected["local_planner_backend"] in ROS2_LOCAL_PLANNER_BACKENDS:
        violations.append("local_planner_backend=cmu requires ROS2 NativeModule")
    if selected["path_follower_backend"] in ROS2_PATH_FOLLOWER_BACKENDS:
        violations.append("path_follower_backend=pure_pursuit requires ROS2 NativeModule")
    return violations


def ros2_driver_runtime_violations(config: Mapping[str, Any]) -> list[str]:
    """Return selected driver runtimes that cross the Module graph through ROS2."""

    violations: list[str] = []
    for key in DRIVER_RUNTIME_KEYS:
        selected = _adapter_name(config.get(key))
        if not selected:
            continue
        if selected in ROS2_DRIVER_RUNTIMES or selected.startswith("ros2_"):
            violations.append(f"{key}={selected} selects a ROS2 driver runtime")
    return violations


def ros2_lidar_driver_violations(config: Mapping[str, Any]) -> list[str]:
    """Return LiDAR acquisition settings that start legacy ROS2 driver processes."""

    violations: list[str] = []
    for key in LIDAR_LEGACY_DRIVER_START_KEYS:
        if bool(config.get(key)):
            violations.append(f"{key}=true starts the legacy local Livox ROS2 driver")
    return violations


def legacy_sensor_binding_violations(config: Mapping[str, Any]) -> list[str]:
    """Return settings that route sensor streams through legacy driver paths."""

    violations: list[str] = []
    for key in LEGACY_SENSOR_BINDING_KEYS:
        if bool(config.get(key)):
            violations.append(f"{key}=true enables a legacy driver sensor path")
    return violations


def ros2_camera_bridge_violations(config: Mapping[str, Any]) -> list[str]:
    """Return camera acquisition settings that select ROS2 compatibility bridges."""

    violations: list[str] = []
    for key in ROS2_CAMERA_BRIDGE_ENABLE_KEYS:
        if bool(config.get(key)):
            violations.append(f"{key}=true enables a ROS2 camera bridge")
    return violations


def ros2_rerun_bridge_violations(config: Mapping[str, Any]) -> list[str]:
    """Return visualization settings that select ROS2 compatibility bridges."""

    violations: list[str] = []
    for key in ROS2_RERUN_BRIDGE_ENABLE_KEYS:
        if bool(config.get(key)):
            violations.append(f"{key}=true enables a ROS2 Rerun bridge")
    return violations


def _is_ros2_planner_backend(name: str) -> bool:
    normalized = normalize_planner_name(name)
    return normalized in ROS2_GLOBAL_PLANNER_BACKENDS or normalized.startswith("ros2_") or normalized.endswith("_ros2")


def ros2_global_planner_backend_violations(
    config: Mapping[str, Any],
) -> list[str]:
    """Return selected global planner backends that would bind to ROS2 wrappers."""

    selected = global_planner_backend_selection(config)
    violations: list[str] = []
    primary = str(selected["primary"])
    if _is_ros2_planner_backend(primary):
        violations.append(f"planner={primary} selects a ROS2 global planner wrapper")
    for fallback in selected["fallback_planners"]:
        if _is_ros2_planner_backend(fallback):
            violations.append(f"fallback_planner={fallback} selects a ROS2 global planner wrapper")
    return violations


def ros2_runtime_binding_violations(
    config: Mapping[str, Any],
    *,
    enable_native: bool = True,
) -> list[str]:
    """Return selected runtime bindings that would pull ROS2 into the graph.

    This is an audit helper, not a hard ban. Compatibility endpoints can still
    choose ROS2 explicitly, but product/portable profiles should be able to
    prove they do not rely on implicit ROS2 fallbacks.
    """

    violations = ros2_autonomy_backend_violations(
        config,
        enable_native=enable_native,
    )
    violations.extend(ros2_driver_runtime_violations(config))
    violations.extend(ros2_lidar_driver_violations(config))
    violations.extend(ros2_camera_bridge_violations(config))
    violations.extend(ros2_rerun_bridge_violations(config))
    violations.extend(ros2_global_planner_backend_violations(config))
    exploration_backend = exploration_backend_for_config(config)
    if exploration_backend in ROS2_EXPLORATION_BACKENDS:
        violations.append(f"exploration_backend={exploration_backend} requires ROS2 NativeModule")
    for key in IO_ADAPTER_ENABLE_KEYS:
        if key in LEGACY_MAP_OUT_ENABLE_KEYS:
            continue
        if key.startswith("enable_ros2") and bool(config.get(key)):
            violations.append(f"{key}=true enables a ROS2 IO adapter")
    if map_output_uses_ros2(config):
        violations.append("map output adapter selects a ROS2 IO adapter")
    if (
        any(bool(config.get(key, False)) for key in (*MAP_OUT_ENABLE_KEYS, *LEGACY_MAP_OUT_ENABLE_KEYS))
        and not map_output_uses_ros2(config)
        and not map_output_uses_dds(config)
    ):
        key = "enable_map_out" if bool(config.get("enable_map_out", False)) else "enable_endpoint_grid_bridge"
        violations.append(f"{key}=true without explicit non-ROS map output adapter has no safe default")
    nav_in_key = "enable_nav_in" if bool(config.get("enable_nav_in", False)) else "enable_endpoint_command_bridge"
    if (
        bool(config.get(nav_in_key, False))
        and not navigation_input_uses_dds(config)
        and not navigation_input_uses_ros2(config)
    ):
        violations.append(f"{nav_in_key}=true without explicit non-ROS navigation input adapter has no safe default")
    nav_out_key = "enable_nav_out" if bool(config.get("enable_nav_out", False)) else "enable_endpoint_path_bridge"
    if (
        bool(config.get(nav_out_key, False))
        and not navigation_output_uses_dds(config)
        and not navigation_output_uses_ros2(config)
    ):
        violations.append(f"{nav_out_key}=true without explicit non-ROS navigation output adapter has no safe default")
    if (
        bool(config.get("enable_endpoint_waypoint_bridge", False))
        and not navigation_output_uses_dds(config)
        and not navigation_output_uses_ros2(config)
    ):
        violations.append(
            "enable_endpoint_waypoint_bridge=true without explicit non-ROS navigation output adapter has no safe default"
        )

    for key in LOCALIZATION_ADAPTER_KEYS:
        selected = _adapter_name(config.get(key))
        if selected in ROS2_ADAPTER_NAMES:
            violations.append(f"{key}={selected} selects a ROS2 localization adapter")

    for key in IO_ADAPTER_KEYS:
        if key in (*MAP_OUT_ADAPTER_KEYS, *LEGACY_MAP_OUT_ADAPTER_KEYS):
            continue
        selected = _adapter_name(config.get(key))
        if selected in ROS2_ADAPTER_NAMES or selected.startswith("ros2_"):
            violations.append(f"{key}={selected} selects a ROS2 IO adapter")

    return violations


def _adapter_name(value: Any) -> str:
    return str(value or "").strip().lower()
