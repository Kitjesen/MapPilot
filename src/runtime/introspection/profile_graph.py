"""Profile graph compilation helpers.

This is the first engineering boundary around profiles: a profile can be
compiled into a stable module/wire graph without constructing runtime modules.
The graph is intentionally read-only and snapshot-friendly.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from runtime.blueprint import Blueprint
from runtime.profiles.catalog.products import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    SIMULATION_PROFILES,
)
from runtime.profiles.catalog.robots import robot_driver_module_name
from runtime.blueprints.full_stack_wiring import full_stack_wire_specs
from runtime.blueprints.wires.context import (
    MAP_OUT,
    NAV_IN,
    NAV_OUT,
)
from runtime.blueprints.stacks.slam import (
    normalize_slam_profile,
    slam_adapter_module_name,
    slam_module_name,
)
from runtime.blueprints.stacks.stack_config import needs_lidar_for_slam
from runtime.profiles.binding_policy import (
    localization_adapter_for_config,
    map_output_adapter_enabled,
    map_output_uses_dds,
    map_output_uses_ros2,
    navigation_input_adapter_enabled,
    navigation_input_uses_dds,
    navigation_input_uses_lcm,
    navigation_input_uses_ros2,
    navigation_output_adapter_enabled,
    navigation_output_uses_dds,
    navigation_output_uses_lcm,
    navigation_output_uses_ros2,
)
from runtime.profiles.resolver import resolve_profile_config

__all__ = [
    "LIGHTWEIGHT_PRODUCT_PROFILES",
    "OPTIONAL_NATIVE_PRODUCT_PROFILES",
    "PRODUCT_PROFILES",
    "PROFILE_SNAPSHOT_TARGETS",
    "SIMULATION_PROFILES",
    "ProfileGraph",
    "WireEdge",
    "blueprint_for_profile",
    "graph_for_profile",
    "resolve_profile_config",
]


@dataclass(frozen=True, order=True)
class WireEdge:
    """A directed edge in the profile dependency graph between two module ports."""
    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: str | None = None
    topic: str | None = None

    @classmethod
    def from_blueprint_spec(cls, spec: Any) -> "WireEdge":
        transport = getattr(spec, "transport", None)
        if isinstance(transport, str):
            pass
        elif transport is not None:
            transport = getattr(transport, "__class__", type(transport)).__name__
        return cls(
            out_module=spec.out_module,
            out_port=spec.out_port,
            in_module=spec.in_module,
            in_port=spec.in_port,
            transport=transport,
            topic=_topic_name(getattr(spec, "topic", None)),
        )

    @classmethod
    def from_graph_spec(cls, spec: Any) -> "WireEdge":
        return cls(
            out_module=spec.out_module,
            out_port=spec.out_port,
            in_module=spec.in_module,
            in_port=spec.in_port,
            transport=spec.transport,
            topic=getattr(spec, "topic", None),
        )

    def as_snapshot(self) -> str:
        wire = f"{self.out_module}.{self.out_port}->{self.in_module}.{self.in_port}"
        if self.transport:
            wire = f"{wire}[{self.transport}]"
        if self.topic:
            wire = f"{wire}@{self.topic}"
        return wire


@dataclass(frozen=True)
class ProfileGraph:
    """DAG representation of a runtime profile: which modules and explicit wires it includes."""
    profile: str
    modules: tuple[str, ...]
    explicit_wires: tuple[WireEdge, ...]

    def as_snapshot(self) -> dict[str, list[str]]:
        return {
            "modules": sorted(self.modules),
            "explicit_wires": sorted(wire.as_snapshot() for wire in self.explicit_wires),
        }

    def dangling_wires(self) -> tuple[WireEdge, ...]:
        module_set = set(self.modules)
        return tuple(
            wire
            for wire in self.explicit_wires
            if wire.out_module not in module_set or wire.in_module not in module_set
        )


_NATIVE_CAMERA_DRIVERS = {"MujocoDriverModule"}
_REPO_ROOT = Path(__file__).resolve().parents[3]


def _dedupe(names: list[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    unique: list[str] = []
    for name in names:
        if name and name not in seen:
            unique.append(name)
            seen.add(name)
    return tuple(unique)


def _topic_name(topic: Any) -> str | None:
    if topic is None:
        return None
    value = str(topic).strip()
    return value or None


def _bridge_enabled(config: dict[str, Any], *names: str) -> bool:
    for name in names:
        if name in config:
            return bool(config[name])
    return False


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() not in {"0", "false", "no", "off", ""}
    return bool(value)


def _static_driver_module(robot: str) -> str:
    return robot_driver_module_name(robot)


def _needs_camera_bridge(config: dict[str, Any], *, driver_module: str) -> bool:
    return bool(config.get("enable_camera", True)) and (
        bool(config.get("force_camera_bridge"))
        or (
            driver_module not in _NATIVE_CAMERA_DRIVERS
            and not bool(config.get("use_driver_camera", False))
        )
    )


def _camera_bridge_module_enabled(config: dict[str, Any], *, driver_module: str) -> bool:
    return bool(config.get("enable_camera", True)) or bool(
        config.get("enable_ros2_camera_bridge")
    )


def _static_device_manager_modules() -> tuple[str, ...]:
    """Mirror stacks.system.device_manager without importing devices."""

    return ("DeviceManager",) if (_REPO_ROOT / "config" / "devices.yaml").exists() else ()


def _static_gnss_module_names(config: dict[str, Any]) -> tuple[str, ...]:
    """Mirror stacks.system.gnss from config data only.

    Static profile graphs must not import SLAM/GNSS runtime modules, but they
    should still include the module names that full_stack_blueprint() adds when
    a profile allows GNSS and config/robot_config.yaml enables it. This keeps
    runtime parity checks from reporting expected hardware support modules as
    graph drift while letting lightweight profiles explicitly opt out.
    """

    if _optional_bool(config.get("enable_gnss")) is False:
        return ()
    try:
        from runtime.config import get_config

        gnss_cfg = get_config().raw.get("gnss", {})
    except Exception:
        return ()
    if not gnss_cfg.get("enabled", False):
        return ()
    serial_port = gnss_cfg.get("device") or gnss_cfg.get("serial_port")
    use_device_manager_bridge = _optional_bool(gnss_cfg.get("device_manager_bridge"))
    if use_device_manager_bridge is None:
        use_device_manager_bridge = not bool(serial_port)
    modules = ["GnssModule"]
    if use_device_manager_bridge:
        modules.append("GnssBridgeModule")
    if (gnss_cfg.get("rtcm") or {}).get("enabled", False):
        modules.append("NtripClientModule")
    return tuple(modules)


def _static_module_names(config: dict[str, Any]) -> tuple[str, ...]:
    robot = str(config.get("robot", "thunder"))
    driver_module = _static_driver_module(robot)
    slam_profile = normalize_slam_profile(str(config.get("slam_profile", "fastlio2")))
    enable_semantic = bool(config.get("enable_semantic", True))
    enable_gateway = bool(config.get("enable_gateway", True))
    enable_teleop = bool(config.get("enable_teleop", True))
    enable_navigation = bool(config.get("enable_navigation", True))
    enable_device_manager = bool(config.get("enable_device_manager", True))
    enable_robot_driver = bool(config.get("enable_robot_driver", True))

    modules: list[str] = []
    if enable_device_manager:
        modules.extend(_static_device_manager_modules())
    if enable_robot_driver:
        modules.append(driver_module)

    enable_lidar = _optional_bool(config.get("enable_lidar"))
    if enable_lidar is None:
        enable_lidar = needs_lidar_for_slam(slam_profile)
    if enable_lidar:
        modules.append("LidarModule")
    if config.get("scene_xml", ""):
        modules.append("SimPointCloudProvider")

    slam_module = _static_slam_module_name(config, slam_profile)
    if slam_module:
        modules.append(slam_module)
        if bool(config.get("enable_visual_backup", True)):
            modules.append("DepthVisualOdomModule")
    modules.extend(_static_gnss_module_names(config))

    if bool(config.get("enable_map_modules", True)):
        modules.append("OccupancyGridModule")
        if (
            map_output_adapter_enabled(config)
            and (map_output_uses_dds(config) or map_output_uses_ros2(config))
        ):
            modules.append(MAP_OUT)
        modules.extend([
            "VoxelGridModule",
            "ESDFModule",
            "ElevationMapModule",
            "TraversabilityCostModule",
            "nav.maps",
        ])

    if enable_semantic:
        if _needs_camera_bridge(
            config,
            driver_module=driver_module,
        ) and _camera_bridge_module_enabled(config, driver_module=driver_module):
            modules.append("CameraBridgeModule")
        modules.extend(["PerceptionModule", "ReconstructionModule"])
        if config.get("recon_save_dir", ""):
            modules.append("DatasetRecorderModule")
        if config.get("recon_server_url", ""):
            modules.append("ReconKeyframeExporterModule")
        modules.extend([
            "SemanticMapperModule",
            "EpisodicMemoryModule",
            "TaggedLocationsModule",
            "VectorMemoryModule",
            "TemporalMemoryModule",
            "MissionLoggerModule",
            "SemanticPlannerModule",
            "LLMModule",
            "VisualServoModule",
        ])

    if bool(config.get("enable_goals", True)):
        modules.append("nav.goals")
    if bool(config.get("enable_patrol_routes", True)):
        modules.append("PatrolManagerModule")
    if bool(config.get("enable_scheduler", False)):
        modules.append("TaskSchedulerModule")

    if (
        navigation_output_adapter_enabled(config)
        and (
            navigation_output_uses_dds(config)
            or navigation_output_uses_lcm(config)
            or navigation_output_uses_ros2(config)
        )
    ):
        modules.append(NAV_OUT)

    if enable_navigation:
        modules.append("nav.mission")
        if (
            navigation_input_adapter_enabled(config)
            and (
                navigation_input_uses_dds(config)
                or navigation_input_uses_lcm(config)
                or navigation_input_uses_ros2(config)
            )
        ):
            modules.append(NAV_IN)
        if bool(config.get("enable_frontier", False)):
            modules.append("WavefrontFrontierExplorer")
        if bool(config.get("enable_traversable_frontier", False)):
            modules.append("TraversableFrontierModule")
        modules.extend(["nav.terrain", "nav.local_planner", "nav.path_follower"])

    exploration_backend = str(config.get("exploration_backend", "none") or "none")
    if exploration_backend in {"tare", "tare_external"}:
        modules.extend(["TAREExplorerModule", "ExplorationSupervisorModule"])
    elif exploration_backend != "none":
        raise ValueError(
            f"unknown exploration backend for static profile graph: {exploration_backend}"
        )

    modules.extend(["nav.safety", "nav.velocity_mux", "GeofenceManagerModule"])

    if enable_gateway:
        modules.extend(["GatewayModule", "MCPServerModule"])
        if enable_teleop:
            modules.append("TeleopModule")
        if bool(config.get("enable_rerun", False)):
            modules.append("RerunBridgeModule")

    return _dedupe(modules)


def _static_slam_module_name(config: Mapping[str, Any], slam_profile: str) -> str:
    if not slam_profile or slam_profile == "none":
        return ""
    adapter = str(localization_adapter_for_config(config) or "").strip().lower()
    if adapter and adapter not in {"native", "native_slam", "slam"}:
        return slam_adapter_module_name(adapter)
    return slam_module_name(slam_profile)


def _static_stack_wire_specs(config: dict[str, Any]) -> tuple[WireEdge, ...]:
    modules = set(_static_module_names(config))
    specs: list[WireEdge] = []
    if "WavefrontFrontierExplorer" in modules:
        specs.extend([
            WireEdge(
                "WavefrontFrontierExplorer",
                "exploration_goal",
                "nav.mission",
                "goal_pose",
            ),
            WireEdge(
                "nav.mission",
                "mission_status",
                "WavefrontFrontierExplorer",
                "navigation_status",
            ),
        ])
    return tuple(specs)


def blueprint_for_profile(
    profile: str,
    *,
    run_startup_checks: bool = False,
    manage_external_services: bool = False,
    **overrides: Any,
) -> Blueprint:
    """Compile a profile into a Blueprint without starting modules."""

    config = resolve_profile_config(profile, **overrides)
    config["run_startup_checks"] = run_startup_checks
    config["manage_external_services"] = manage_external_services

    from runtime.blueprints.profile_builder import blueprint_for_resolved_profile

    return blueprint_for_resolved_profile(profile, config)


def graph_for_profile(
    profile: str,
    *,
    run_startup_checks: bool = False,
    manage_external_services: bool = False,
    mode: str = "static",
    **overrides: Any,
) -> ProfileGraph:
    """Compile a profile into a stable module/wire graph."""

    if mode == "runtime":
        bp = blueprint_for_profile(
            profile,
            run_startup_checks=run_startup_checks,
            manage_external_services=manage_external_services,
            **overrides,
        )
        graph = bp.export_graph(profile=profile)
        modules = graph.module_names
        wires = tuple(WireEdge.from_graph_spec(spec) for spec in graph.explicit_wires)
        return ProfileGraph(profile=profile, modules=modules, explicit_wires=wires)

    if mode != "static":
        raise ValueError(f"unknown profile graph mode: {mode}")

    config = resolve_profile_config(profile, **overrides)
    config["run_startup_checks"] = run_startup_checks
    config["manage_external_services"] = manage_external_services
    modules = _static_module_names(config)
    module_names = set(modules)
    driver_module = _static_driver_module(str(config.get("robot", "thunder")))
    slam_profile = normalize_slam_profile(str(config.get("slam_profile", "fastlio2")))
    specs = [
        *_static_stack_wire_specs(config),
        *(
            WireEdge.from_blueprint_spec(spec)
            for spec in full_stack_wire_specs(
                module_names,
                robot=str(config.get("robot", "thunder")),
                driver_module=driver_module,
                slam_profile=slam_profile,
                scene_xml=str(config.get("scene_xml", "")),
                enable_semantic=bool(config.get("enable_semantic", True)),
                safety_stop_wiring=bool(config.get("safety_stop_wiring", True)),
                cmd_vel_mux_collision_monitor=bool(
                    config.get("cmd_vel_mux_collision_monitor", False)
                ),
                nav_plan_transport=(
                    config.get("nav_plan_transport")
                    if "nav_plan_transport" in config
                    else config.get("local_planner_transport")
                ),
            )
        ),
    ]
    wires = tuple(sorted(set(specs)))
    return ProfileGraph(profile=profile, modules=modules, explicit_wires=wires)


def snapshot_profile_graphs(
    profiles: tuple[str, ...] = PROFILE_SNAPSHOT_TARGETS,
) -> dict[str, dict[str, list[str]]]:
    return {profile: graph_for_profile(profile).as_snapshot() for profile in profiles}
