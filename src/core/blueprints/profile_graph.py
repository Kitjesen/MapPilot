"""Profile graph compilation helpers.

This is the first engineering boundary around profiles: a profile can be
compiled into a stable module/wire graph without constructing runtime modules.
The graph is intentionally read-only and snapshot-friendly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.blueprint import Blueprint
from core.blueprints.full_stack_wiring import full_stack_wire_specs
from core.blueprints.runtime_endpoint import (
    apply_runtime_endpoint_config,
    runtime_endpoint_robot_preset,
)
from core.blueprints.stacks.slam import normalize_slam_profile, slam_module_name
from core.runtime_profiles import PROFILES, ROBOT_PRESETS


SIMULATION_PROFILES = (
    "stub",
    "dev",
    "sim",
    "sim_mujoco_live",
    "sim_mujoco_pct_live",
    "sim_gazebo",
    "sim_industrial",
    "sim_cmu_tare",
    "sim_nav",
)

PRODUCT_PROFILES = (
    "map",
    "nav",
    "explore",
    "tare_explore",
    "super_lio",
    "super_lio_relocation",
)

# Offline graph snapshots exclude profiles that require a locally built native
# external binary at blueprint construction time. They remain product profiles
# and are checked through profile-level contracts.
OPTIONAL_NATIVE_PRODUCT_PROFILES = ("tare_explore",)

PROFILE_SNAPSHOT_TARGETS = (
    *SIMULATION_PROFILES,
    *(profile for profile in PRODUCT_PROFILES if profile not in OPTIONAL_NATIVE_PRODUCT_PROFILES),
)


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


_DRIVER_MODULES = {
    "auto": "StubDogModule",
    "stub": "StubDogModule",
    "sim": "MujocoDriverModule",
    "sim_mujoco": "MujocoDriverModule",
    "sim_gazebo": "ROS2SimDriverModule",
    "sim_ros2": "ROS2SimDriverModule",
    "ros2": "ROS2SimDriverModule",
    "s100p": "ThunderDriver",
    "thunder": "ThunderDriver",
    "navigate": "ThunderDriver",
}

_NATIVE_CAMERA_DRIVERS = {"MujocoDriverModule"}


def _dedupe(names: list[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    unique: list[str] = []
    for name in names:
        if name and name not in seen:
            unique.append(name)
            seen.add(name)
    return tuple(unique)


def _static_driver_module(robot: str) -> str:
    try:
        return _DRIVER_MODULES[robot]
    except KeyError as exc:
        raise KeyError(
            f"unknown static driver mapping for robot preset: {robot}"
        ) from exc


def _needs_lidar_for_slam(slam_profile: str) -> bool:
    return slam_profile not in (
        "",
        "none",
        "bridge",
        "super_lio",
        "super_lio_relocation",
    )


def _needs_camera_bridge(config: dict[str, Any], *, driver_module: str) -> bool:
    return bool(config.get("force_camera_bridge")) or (
        driver_module not in _NATIVE_CAMERA_DRIVERS
        and not bool(config.get("use_driver_camera", False))
    )


def _static_module_names(config: dict[str, Any]) -> tuple[str, ...]:
    robot = str(config.get("robot", "thunder"))
    driver_module = _static_driver_module(robot)
    slam_profile = normalize_slam_profile(str(config.get("slam_profile", "fastlio2")))
    enable_semantic = bool(config.get("enable_semantic", True))
    enable_gateway = bool(config.get("enable_gateway", True))
    enable_teleop = bool(config.get("enable_teleop", True))

    modules: list[str] = [driver_module]

    if _needs_lidar_for_slam(slam_profile):
        modules.append("LidarModule")
    if config.get("scene_xml", ""):
        modules.append("SimPointCloudProvider")

    slam_module = slam_module_name(slam_profile)
    if slam_module:
        modules.extend([slam_module, "DepthVisualOdomModule"])

    if bool(config.get("enable_map_modules", True)):
        modules.append("OccupancyGridModule")
        if bool(config.get("enable_ros2_grid_bridge", False)):
            modules.append("ROS2GridBridgeModule")
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
    if bool(config.get("enable_ros2_bridge", False)):
        modules.append("ROS2WaypointBridgeModule")
    if bool(config.get("enable_ros2_path_bridge", False)):
        modules.append("ROS2PathBridgeModule")
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
    if "ROS2WaypointBridgeModule" in modules:
        specs.append(WireEdge(
            "NavigationModule",
            "waypoint",
            "ROS2WaypointBridgeModule",
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


def resolve_profile_config(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    **overrides: Any,
) -> dict[str, Any]:
    """Return the full full_stack_blueprint kwargs for a CLI profile."""

    if profile not in PROFILES:
        raise KeyError(f"unknown profile: {profile}")
    profile_data = PROFILES[profile]
    if runtime_endpoint:
        preset_name = runtime_endpoint_robot_preset(profile, runtime_endpoint)
    else:
        preset_name = profile_data.get("_default_robot", "stub")
    if preset_name not in ROBOT_PRESETS:
        raise KeyError(f"unknown robot preset for profile {profile}: {preset_name}")

    config = dict(ROBOT_PRESETS[preset_name])
    config.update({k: v for k, v in profile_data.items() if not k.startswith("_")})
    if runtime_endpoint:
        config = apply_runtime_endpoint_config(profile, config, runtime_endpoint)
    config.update(overrides)
    return config


def blueprint_for_profile(
    profile: str,
    *,
    run_startup_checks: bool = False,
    manage_external_services: bool = False,
    **overrides: Any,
) -> Blueprint:
    """Compile a profile into a Blueprint without starting modules."""

    from core.blueprints.full_stack import full_stack_blueprint

    config = resolve_profile_config(profile, **overrides)
    config["run_startup_checks"] = run_startup_checks
    config["manage_external_services"] = manage_external_services
    return full_stack_blueprint(**config)


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
