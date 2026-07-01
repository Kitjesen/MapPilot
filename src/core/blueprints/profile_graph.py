"""Profile graph compilation helpers.

This is the first engineering boundary around profiles: a profile can be
compiled into a stable module/wire graph without constructing runtime modules.
The graph is intentionally read-only and snapshot-friendly.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from core.blueprint import Blueprint
from core.blueprints.catalog.products import (
    LIGHTWEIGHT_PRODUCT_PROFILES,
    OPTIONAL_NATIVE_PRODUCT_PROFILES,
    PRODUCT_PROFILES,
    PROFILE_SNAPSHOT_TARGETS,
    SIMULATION_PROFILES,
)
from core.blueprints.catalog.robots import robot_driver_module_name
from core.blueprints.full_stack_wiring import full_stack_wire_specs
from core.blueprints.stacks.slam import normalize_slam_profile, slam_module_name
from core.blueprints.stacks.stack_config import needs_lidar_for_slam
from core.runtime.resolver import resolve_profile_config

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

    @classmethod
    def from_blueprint_spec(cls, spec: Any) -> "WireEdge":
        transport = getattr(spec, "transport", None)
        if transport is not None:
            transport = getattr(transport, "__class__", type(transport)).__name__
        return cls(
            out_module=spec.out_module,
            out_port=spec.out_port,
            in_module=spec.in_module,
            in_port=spec.in_port,
            transport=transport,
        )

    def as_snapshot(self) -> str:
        wire = f"{self.out_module}.{self.out_port}->{self.in_module}.{self.in_port}"
        if self.transport:
            wire = f"{wire}[{self.transport}]"
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


def _bridge_enabled(config: dict[str, Any], name: str, legacy_name: str) -> bool:
    if name in config:
        return bool(config[name])
    return bool(config.get(legacy_name, False))


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() not in {"0", "false", "no", "off", ""}
    return bool(value)


def _endpoint_egress_uses_lcm(config: dict[str, Any]) -> bool:
    selected = str(
        config.get("endpoint_path_bridge")
        or config.get("endpoint_egress_adapter")
        or ""
    ).lower()
    endpoint_transport = str(
        config.get("_endpoint_transport")
        or config.get("endpoint_transport")
        or ""
    ).lower()
    return selected in {"lcm", "lcm_endpoint", "lcm_path_command_bridge"} or (
        not selected and endpoint_transport == "lcm"
    )


def _endpoint_ingress_uses_lcm(config: dict[str, Any]) -> bool:
    selected = str(
        config.get("endpoint_command_bridge")
        or config.get("endpoint_ingress_adapter")
        or ""
    ).lower()
    endpoint_transport = str(
        config.get("_endpoint_transport")
        or config.get("endpoint_transport")
        or ""
    ).lower()
    return selected in {"lcm", "lcm_endpoint", "lcm_navigation_command_bridge"} or (
        not selected and endpoint_transport == "lcm"
    )


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
        from core.config import get_config

        gnss_cfg = get_config().raw.get("gnss", {})
    except Exception:
        return ()
    if not gnss_cfg.get("enabled", False):
        return ()
    modules = ["GnssModule", "GnssBridgeModule"]
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
    enable_robot_driver = bool(config.get("enable_robot_driver", True))

    modules: list[str] = [*_static_device_manager_modules()]
    if enable_robot_driver:
        modules.append(driver_module)

    if needs_lidar_for_slam(slam_profile):
        modules.append("LidarModule")
    if config.get("scene_xml", ""):
        modules.append("SimPointCloudProvider")

    slam_module = slam_module_name(slam_profile)
    if slam_module:
        modules.extend([slam_module, "DepthVisualOdomModule"])
    modules.extend(_static_gnss_module_names(config))

    if bool(config.get("enable_map_modules", True)):
        modules.append("OccupancyGridModule")
        if _bridge_enabled(config, "enable_endpoint_grid_bridge", "enable_ros2_grid_bridge"):
            modules.append("EndpointGridBridgeModule")
        modules.extend([
            "VoxelGridModule",
            "ESDFModule",
            "ElevationMapModule",
            "TraversabilityCostModule",
            "MapManagerModule",
        ])

    if enable_semantic:
        if _needs_camera_bridge(config, driver_module=driver_module):
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

    modules.append("NavigationModule")
    if (
        _bridge_enabled(
            config,
            "enable_endpoint_command_bridge",
            "enable_ros2_command_bridge",
        )
        and _endpoint_ingress_uses_lcm(config)
    ):
        modules.append("EndpointCommandBridgeModule")
    if (
        _bridge_enabled(config, "enable_endpoint_waypoint_bridge", "enable_ros2_bridge")
        and not _endpoint_egress_uses_lcm(config)
    ):
        modules.append("EndpointWaypointBridgeModule")
    if _bridge_enabled(config, "enable_endpoint_path_bridge", "enable_ros2_path_bridge"):
        modules.append("EndpointPathBridgeModule")
    if bool(config.get("enable_frontier", False)):
        modules.append("WavefrontFrontierExplorer")
    if bool(config.get("enable_traversable_frontier", False)):
        modules.append("TraversableFrontierModule")
    modules.extend(["TerrainModule", "LocalPlannerModule", "PathFollowerModule"])

    exploration_backend = str(config.get("exploration_backend", "none") or "none")
    if exploration_backend == "tare":
        modules.extend([
            "TAREPlannerNativeModule",
            "TAREExplorerModule",
            "ExplorationSupervisorModule",
        ])
    elif exploration_backend == "tare_external":
        modules.extend(["TAREExplorerModule", "ExplorationSupervisorModule"])
    elif exploration_backend != "none":
        raise ValueError(
            f"unknown exploration backend for static profile graph: {exploration_backend}"
        )

    modules.extend(["SafetyRingModule", "CmdVelMux", "GeofenceManagerModule"])

    if enable_gateway:
        modules.extend(["GatewayModule", "MCPServerModule"])
        if enable_teleop:
            modules.append("TeleopModule")
        if bool(config.get("enable_rerun", False)):
            modules.append("RerunBridgeModule")

    return _dedupe(modules)


def _static_stack_wire_specs(config: dict[str, Any]) -> tuple[WireEdge, ...]:
    modules = set(_static_module_names(config))
    specs: list[WireEdge] = []
    if "EndpointCommandBridgeModule" in modules:
        specs.extend([
            WireEdge(
                "EndpointCommandBridgeModule",
                "goal_pose",
                "NavigationModule",
                "goal_pose",
            ),
            WireEdge(
                "EndpointCommandBridgeModule",
                "cancel",
                "NavigationModule",
                "cancel",
            ),
            WireEdge(
                "EndpointCommandBridgeModule",
                "instruction",
                "NavigationModule",
                "instruction",
            ),
        ])
    if "EndpointWaypointBridgeModule" in modules:
        specs.append(WireEdge(
            "NavigationModule",
            "waypoint",
            "EndpointWaypointBridgeModule",
            "waypoint",
        ))
    if "EndpointPathBridgeModule" in modules and _endpoint_egress_uses_lcm(config):
        specs.append(WireEdge(
            "NavigationModule",
            "waypoint",
            "EndpointPathBridgeModule",
            "waypoint",
        ))
    if "WavefrontFrontierExplorer" in modules:
        specs.extend([
            WireEdge(
                "WavefrontFrontierExplorer",
                "exploration_goal",
                "NavigationModule",
                "goal_pose",
            ),
            WireEdge(
                "NavigationModule",
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

    from core.blueprints.profile_builder import blueprint_for_resolved_profile

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
        modules = tuple(entry.name for entry in bp._entries)
        wires = tuple(WireEdge.from_blueprint_spec(spec) for spec in bp._wires)
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
            )
        ),
    ]
    wires = tuple(sorted(set(specs)))
    return ProfileGraph(profile=profile, modules=modules, explicit_wires=wires)


def snapshot_profile_graphs(
    profiles: tuple[str, ...] = PROFILE_SNAPSHOT_TARGETS,
) -> dict[str, dict[str, list[str]]]:
    return {profile: graph_for_profile(profile).as_snapshot() for profile in profiles}
