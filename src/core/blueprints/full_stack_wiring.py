"""Explicit cross-stack wiring for the full LingTu profile.

The module keeps high-risk fan-in/fan-out rules out of the profile factory so
profile graph tests can lock the actual contract of the assembled system.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from core.blueprint import Blueprint

from .stacks.slam import slam_module_name

_MAP_CLOUD_CONSUMERS = (
    "OccupancyGridModule",
    "ElevationMapModule",
    "TerrainModule",
    "VoxelGridModule",
    "RerunBridgeModule",
    "GatewayModule",
)

_SEMANTIC_CAMERA_CONSUMERS = (
    "PerceptionModule",
    "ReconstructionModule",
    "VisualServoModule",
)


@dataclass(frozen=True)
class WireSpec:
    """Blueprint wire specification — connects an output port on one module to an input port on another.

    Used to define explicit cross-module connections that would otherwise
    require runtime dispatch or manual wiring.
    """
    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: Any = None

    def label(self) -> str:
        return (
            f"{self.out_module}.{self.out_port}"
            f"->{self.in_module}.{self.in_port}"
        )

    def apply(self, bp: Blueprint) -> None:
        bp.wire(
            self.out_module,
            self.out_port,
            self.in_module,
            self.in_port,
            transport=self.transport,
        )


def _wire_if_present(bp: Blueprint, names: set[str], spec: WireSpec) -> None:
    if (
        spec.out_module in names
        and spec.in_module in names
        and _declares_port(bp, spec.out_module, spec.out_port)
        and _declares_port(bp, spec.in_module, spec.in_port)
    ):
        spec.apply(bp)


def _apply_if_present(bp: Blueprint, names: set[str], specs: list[WireSpec]) -> None:
    for spec in specs:
        _wire_if_present(bp, names, spec)


def _wire_contract_issues(bp: Blueprint, names: set[str], spec: WireSpec) -> list[str]:
    issues: list[str] = []
    if spec.out_module not in names:
        issues.append(f"missing source module {spec.out_module}")
    elif not _declares_port(bp, spec.out_module, spec.out_port):
        issues.append(f"missing source port {spec.out_module}.{spec.out_port}")

    if spec.in_module not in names:
        issues.append(f"missing destination module {spec.in_module}")
    elif not _declares_port(bp, spec.in_module, spec.in_port):
        issues.append(f"missing destination port {spec.in_module}.{spec.in_port}")
    return issues


def _require_wire(bp: Blueprint, names: set[str], spec: WireSpec) -> None:
    issues = _wire_contract_issues(bp, names, spec)
    if issues:
        raise ValueError(
            "Required full-stack wire unavailable: "
            f"{spec.label()} ({'; '.join(issues)})"
        )
    spec.apply(bp)


def _declares_port(bp: Blueprint, module_name: str, port_name: str) -> bool:
    for entry in bp._entries:
        if entry.name != module_name:
            continue
        if entry.instance is not None:
            ports = getattr(entry.instance, "all_ports", {})
            if port_name in ports:
                return True
        for cls in entry.module_cls.__mro__:
            if port_name in getattr(cls, "__annotations__", {}):
                return True
        return hasattr(entry.module_cls, port_name)
    return False


def _apply_required_safety_stop_wires(
    bp: Blueprint,
    names: set[str],
    *,
    driver_module: str,
) -> None:
    required = [
        WireSpec("SafetyRingModule", "stop_cmd", driver_module, "stop_signal"),
        WireSpec("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal"),
    ]
    if "GeofenceManagerModule" in names:
        required.extend([
            WireSpec("GeofenceManagerModule", "stop_cmd", driver_module, "stop_signal"),
            WireSpec("GeofenceManagerModule", "stop_cmd", "NavigationModule", "stop_signal"),
        ])
    for spec in required:
        _require_wire(bp, names, spec)


def _camera_source(names: set[str], *, driver_module: str) -> tuple[str, str]:
    camera_src = "CameraBridgeModule" if "CameraBridgeModule" in names else driver_module
    color_out = "color_image" if camera_src == "CameraBridgeModule" else "camera_image"
    return camera_src, color_out


def _apply_map_cloud_wires(
    bp: Blueprint,
    names: set[str],
    *,
    driver_module: str,
    slam_module: str,
    scene_xml: str,
) -> None:
    if slam_module:
        for consumer in _MAP_CLOUD_CONSUMERS:
            _wire_if_present(
                bp,
                names,
                WireSpec(slam_module, "map_cloud", consumer, "map_cloud"),
            )
        return

    if scene_xml and "SimPointCloudProvider" in names:
        for consumer in _MAP_CLOUD_CONSUMERS:
            _wire_if_present(
                bp,
                names,
                WireSpec("SimPointCloudProvider", "map_cloud", consumer, "map_cloud"),
            )
        _wire_if_present(
            bp,
            names,
            WireSpec(driver_module, "odometry", "SimPointCloudProvider", "odometry"),
        )
        return

    driver_map_port = ""
    if driver_module == "ROS2SimDriverModule":
        driver_map_port = "map_cloud"
    elif driver_module == "MujocoDriverModule":
        driver_map_port = "map_cloud"
    if not driver_map_port:
        return

    for consumer in _MAP_CLOUD_CONSUMERS:
        _wire_if_present(
            bp,
            names,
            WireSpec(driver_module, driver_map_port, consumer, "map_cloud"),
        )


def _apply_semantic_camera_wires(
    bp: Blueprint,
    names: set[str],
    *,
    camera_src: str,
    color_out: str,
) -> None:
    for consumer in _SEMANTIC_CAMERA_CONSUMERS:
        _apply_if_present(
            bp,
            names,
            [
                WireSpec(camera_src, color_out, consumer, "color_image"),
                WireSpec(camera_src, "depth_image", consumer, "depth_image"),
                WireSpec(camera_src, "camera_info", consumer, "camera_info"),
            ],
        )


def full_stack_wire_specs(
    module_names: set[str] | frozenset[str],
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    scene_xml: str = "",
    enable_semantic: bool = True,
    safety_stop_wiring: bool = True,
) -> tuple[WireSpec, ...]:
    """Return module-name-filtered full-stack wire specs.

    This is intentionally free of runtime module class inspection so profile
    graph tooling can compile architecture snapshots without importing driver,
    perception, or message implementations. ``robot`` is accepted for parity
    with ``apply_full_stack_wires``; dynamic driver-specific ports remain a
    runtime-only concern.
    """

    del robot
    names = set(module_names)
    drv = driver_module
    slam_module = slam_module_name(slam_profile)
    camera_src, color_out = _camera_source(names, driver_module=drv)
    nav_odom_src = slam_module if (slam_module and slam_module in names) else drv

    specs: list[WireSpec] = []

    if slam_module:
        specs.extend(
            WireSpec(slam_module, "map_cloud", consumer, "map_cloud")
            for consumer in _MAP_CLOUD_CONSUMERS
            if consumer in names
        )
    elif scene_xml and "SimPointCloudProvider" in names:
        specs.extend(
            WireSpec("SimPointCloudProvider", "map_cloud", consumer, "map_cloud")
            for consumer in _MAP_CLOUD_CONSUMERS
            if consumer in names
        )
        specs.append(WireSpec(drv, "odometry", "SimPointCloudProvider", "odometry"))
    else:
        driver_map_port = ""
        if drv in {"ROS2SimDriverModule", "MujocoDriverModule"}:
            driver_map_port = "map_cloud"
        if driver_map_port:
            specs.extend(
                WireSpec(drv, driver_map_port, consumer, "map_cloud")
                for consumer in _MAP_CLOUD_CONSUMERS
                if consumer in names
            )

    if enable_semantic:
        for consumer in _SEMANTIC_CAMERA_CONSUMERS:
            specs.extend([
                WireSpec(camera_src, color_out, consumer, "color_image"),
                WireSpec(camera_src, "depth_image", consumer, "depth_image"),
                WireSpec(camera_src, "camera_info", consumer, "camera_info"),
            ])

    if safety_stop_wiring:
        specs.extend([
            WireSpec("SafetyRingModule", "stop_cmd", drv, "stop_signal"),
            WireSpec("SafetyRingModule", "stop_cmd", "NavigationModule", "stop_signal"),
        ])
        if "GeofenceManagerModule" in names:
            specs.extend([
                WireSpec("GeofenceManagerModule", "stop_cmd", drv, "stop_signal"),
                WireSpec("GeofenceManagerModule", "stop_cmd", "NavigationModule", "stop_signal"),
            ])

    specs.extend([
        WireSpec("GatewayModule", "stop_cmd", drv, "stop_signal"),
        WireSpec("GatewayModule", "stop_cmd", "NavigationModule", "stop_signal"),
        WireSpec("GatewayModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        WireSpec("MCPServerModule", "stop_cmd", drv, "stop_signal"),
        WireSpec("MCPServerModule", "stop_cmd", "NavigationModule", "stop_signal"),
        WireSpec("MCPServerModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        WireSpec("SlamBridgeModule", "localization_status", "SafetyRingModule", "localization_status"),
        WireSpec("SlamBridgeModule", "localization_status", "NavigationModule", "localization_status"),
        WireSpec("SlamBridgeModule", "localization_status", "DepthVisualOdomModule", "localization_status"),
        WireSpec("SlamBridgeModule", "localization_status", "GatewayModule", "localization_status"),
        WireSpec("SlamBridgeModule", "map_frame_jump_event", "NavigationModule", "map_frame_jump_event"),
        WireSpec("SlamBridgeModule", "map_frame_jump_event", "LocalPlannerModule", "map_frame_jump_event"),
        WireSpec("SlamBridgeModule", "map_frame_jump_event", "PathFollowerModule", "map_frame_jump_event"),
        WireSpec("DepthVisualOdomModule", "visual_odometry", "SlamBridgeModule", "visual_odom"),
        WireSpec(camera_src, color_out, "DepthVisualOdomModule", "color_image"),
        WireSpec(camera_src, "depth_image", "DepthVisualOdomModule", "depth_image"),
        WireSpec(camera_src, "camera_info", "DepthVisualOdomModule", "camera_info"),
        WireSpec("GatewayModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("GatewayModule", "instruction", "NavigationModule", "instruction"),
        WireSpec("MCPServerModule", "instruction", "NavigationModule", "instruction"),
        WireSpec("GatewayModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("GatewayModule", "cancel", "NavigationModule", "cancel"),
        WireSpec("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("TAREExplorerModule", "exploration_goal", "NavigationModule", "goal_pose"),
        WireSpec("TAREExplorerModule", "exploration_path", "NavigationModule", "patrol_goals"),
        WireSpec(nav_odom_src, "odometry", "TAREExplorerModule", "odometry"),
        WireSpec("NavigationModule", "mission_status", "TAREExplorerModule", "navigation_status"),
        WireSpec("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections"),
        WireSpec(nav_odom_src, "odometry", "NavigationModule", "odometry"),
        WireSpec(nav_odom_src, "odometry", "PerceptionModule", "odometry"),
    ])

    for consumer in [
        "OccupancyGridModule",
        "ElevationMapModule",
        "TerrainModule",
        "VoxelGridModule",
        "WavefrontFrontierExplorer",
        "TraversableFrontierModule",
        "RerunBridgeModule",
        "LocalPlannerModule",
        "PathFollowerModule",
        "SemanticMapperModule",
        "EpisodicMemoryModule",
        "TaggedLocationsModule",
        "VectorMemoryModule",
        "TemporalMemoryModule",
        "MissionLoggerModule",
        "SemanticPlannerModule",
        "VisualServoModule",
        "ReconstructionModule",
        "SafetyRingModule",
        "GeofenceManagerModule",
        "MCPServerModule",
    ]:
        specs.append(WireSpec(nav_odom_src, "odometry", consumer, "odometry"))
    specs.append(WireSpec(nav_odom_src, "odometry", "GatewayModule", "odometry"))

    specs.extend([
        WireSpec("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireSpec("OccupancyGridModule", "exploration_grid", "WavefrontFrontierExplorer", "exploration_grid"),
        WireSpec("OccupancyGridModule", "exploration_grid", "TraversableFrontierModule", "exploration_grid"),
        WireSpec("OccupancyGridModule", "exploration_grid", "ROS2GridBridgeModule", "exploration_grid"),
        WireSpec("ElevationMapModule", "elevation_map", "TraversabilityCostModule", "elevation_map"),
        WireSpec("ElevationMapModule", "elevation_map", "TraversableFrontierModule", "elevation_map"),
        WireSpec("ESDFModule", "esdf", "TraversabilityCostModule", "esdf"),
        WireSpec("TerrainModule", "traversability", "TraversabilityCostModule", "traversability"),
        WireSpec("TraversabilityCostModule", "fused_cost", "NavigationModule", "costmap"),
        WireSpec("TraversabilityCostModule", "esdf_field", "LocalPlannerModule", "esdf"),
        WireSpec("TraversabilityCostModule", "fused_cost", "TraversableFrontierModule", "costmap"),
        WireSpec("TraversabilityCostModule", "fused_cost", "TraversableFrontierModule", "fused_cost"),
        WireSpec("TraversabilityCostModule", "slope_grid", "TraversableFrontierModule", "slope_grid"),
        WireSpec("TraversabilityCostModule", "esdf_field", "TraversableFrontierModule", "esdf_field"),
        WireSpec("TraversableFrontierModule", "traversable_frontiers", "GatewayModule", "traversable_frontiers"),
        WireSpec("TraversableFrontierModule", "frontier_candidate", "GatewayModule", "frontier_candidate"),
        WireSpec("TraversabilityCostModule", "fused_cost", "GatewayModule", "costmap"),
        WireSpec("TraversabilityCostModule", "slope_grid", "GatewayModule", "slope_grid"),
        WireSpec("NavigationModule", "mission_status", "TraversableFrontierModule", "navigation_status"),
        WireSpec("NavigationModule", "mission_status", "MissionLoggerModule", "mission_status"),
        WireSpec("PerceptionModule", "scene_graph", "GatewayModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "MCPServerModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "ReconstructionModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "TraversableFrontierModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "SemanticMapperModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "EpisodicMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "VectorMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "TemporalMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "VisualServoModule", "scene_graph"),
    ])

    for recorder in ["DatasetRecorderModule", "ReconKeyframeExporterModule"]:
        specs.extend([
            WireSpec(camera_src, color_out, recorder, "color_image"),
            WireSpec(camera_src, "depth_image", recorder, "depth_image"),
            WireSpec(camera_src, "camera_info", recorder, "camera_info"),
            WireSpec(nav_odom_src, "odometry", recorder, "odometry"),
        ])

    specs.extend([
        WireSpec("SafetyRingModule", "safety_state", "GatewayModule", "safety_state"),
        WireSpec("SafetyRingModule", "safety_state", "MCPServerModule", "safety_state"),
        WireSpec("NavigationModule", "mission_status", "GatewayModule", "mission_status"),
        WireSpec("NavigationModule", "mission_status", "MCPServerModule", "mission_status"),
        WireSpec("SafetyRingModule", "execution_eval", "GatewayModule", "execution_eval"),
        WireSpec("SafetyRingModule", "dialogue_state", "GatewayModule", "dialogue_state"),
        WireSpec("NavigationModule", "global_path", "GatewayModule", "global_path"),
        WireSpec("NavigationModule", "global_path", "LocalPlannerModule", "global_path"),
        WireSpec("NavigationModule", "global_path", "ROS2PathBridgeModule", "global_path"),
        WireSpec("LocalPlannerModule", "local_path", "GatewayModule", "local_path"),
        WireSpec("LocalPlannerModule", "local_path", "ROS2PathBridgeModule", "local_path"),
        WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
        WireSpec("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
        WireSpec("NavigationModule", "clear_path", "LocalPlannerModule", "clear_path"),
        WireSpec("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map"),
        WireSpec("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
        WireSpec("LocalPlannerModule", "local_path", "SafetyRingModule", "path"),
        WireSpec("LocalPlannerModule", "control_hint", "PathFollowerModule", "control_hint"),
        WireSpec("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target"),
        WireSpec("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal"),
        WireSpec(camera_src, color_out, "TeleopModule", "color_image"),
        WireSpec("PerceptionModule", "scene_graph", "TeleopModule", "scene_graph"),
        WireSpec("TeleopModule", "teleop_active", "NavigationModule", "teleop_active"),
        WireSpec(camera_src, color_out, "WebRTCStreamModule", "color_image"),
        WireSpec("TeleopModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        WireSpec("VisualServoModule", "cmd_vel", "CmdVelMux", "visual_servo_cmd_vel"),
        WireSpec("NavigationModule", "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel"),
        WireSpec("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        WireSpec("CmdVelMux", "driver_cmd_vel", drv, "cmd_vel"),
        WireSpec("CmdVelMux", "driver_cmd_vel", "SafetyRingModule", "cmd_vel"),
    ])

    return tuple(
        spec
        for spec in specs
        if spec.out_module in names and spec.in_module in names
    )


def apply_full_stack_wires(
    bp: Blueprint,
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    scene_xml: str = "",
    enable_semantic: bool = True,
    safety_stop_wiring: bool = True,
) -> Blueprint:
    """Apply explicit cross-stack wires to a composed full-stack Blueprint."""

    drv = driver_module
    slam_module = slam_module_name(slam_profile)
    names = {entry.name for entry in bp._entries}
    camera_src, color_out = _camera_source(names, driver_module=drv)

    _apply_map_cloud_wires(
        bp,
        names,
        driver_module=drv,
        slam_module=slam_module,
        scene_xml=scene_xml,
    )
    if enable_semantic:
        _apply_semantic_camera_wires(
            bp,
            names,
            camera_src=camera_src,
            color_out=color_out,
        )

    if safety_stop_wiring:
        _apply_required_safety_stop_wires(bp, names, driver_module=drv)
    nav_odom_src = slam_module if (slam_module and slam_module in names) else drv

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("GatewayModule", "stop_cmd", drv, "stop_signal"),
            WireSpec("GatewayModule", "stop_cmd", "NavigationModule", "stop_signal"),
            WireSpec("GatewayModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
            WireSpec("MCPServerModule", "stop_cmd", drv, "stop_signal"),
            WireSpec("MCPServerModule", "stop_cmd", "NavigationModule", "stop_signal"),
            WireSpec("MCPServerModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        ],
    )

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("SlamBridgeModule", "localization_status", "SafetyRingModule", "localization_status"),
            WireSpec("SlamBridgeModule", "localization_status", "NavigationModule", "localization_status"),
            WireSpec("SlamBridgeModule", "localization_status", "DepthVisualOdomModule", "localization_status"),
            WireSpec("SlamBridgeModule", "localization_status", "GatewayModule", "localization_status"),
            WireSpec("SlamBridgeModule", "map_frame_jump_event", "NavigationModule", "map_frame_jump_event"),
            WireSpec("SlamBridgeModule", "map_frame_jump_event", "LocalPlannerModule", "map_frame_jump_event"),
            WireSpec("SlamBridgeModule", "map_frame_jump_event", "PathFollowerModule", "map_frame_jump_event"),
            WireSpec("DepthVisualOdomModule", "visual_odometry", "SlamBridgeModule", "visual_odom"),
            WireSpec(camera_src, color_out, "DepthVisualOdomModule", "color_image"),
            WireSpec(camera_src, "depth_image", "DepthVisualOdomModule", "depth_image"),
            WireSpec(camera_src, "camera_info", "DepthVisualOdomModule", "camera_info"),
            WireSpec("GatewayModule", "instruction", "SemanticPlannerModule", "instruction"),
            WireSpec("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction"),
            WireSpec("GatewayModule", "instruction", "NavigationModule", "instruction"),
            WireSpec("MCPServerModule", "instruction", "NavigationModule", "instruction"),
            WireSpec("GatewayModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("GatewayModule", "cancel", "NavigationModule", "cancel"),
            WireSpec("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("TAREExplorerModule", "exploration_goal", "NavigationModule", "goal_pose"),
            WireSpec("TAREExplorerModule", "exploration_path", "NavigationModule", "patrol_goals"),
            WireSpec(nav_odom_src, "odometry", "TAREExplorerModule", "odometry"),
            WireSpec("NavigationModule", "mission_status", "TAREExplorerModule", "navigation_status"),
            WireSpec("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections"),
        ],
    )

    _apply_if_present(
        bp,
        names,
        [
            WireSpec(nav_odom_src, "odometry", "NavigationModule", "odometry"),
            WireSpec(nav_odom_src, "odometry", "PerceptionModule", "odometry"),
        ],
    )
    # Navigation-state consumers must use the same odometry frame as
    # NavigationModule/Gateway. On real profiles this is the SLAM bridge or
    # managed SLAM output; falling back to driver odometry only applies when no
    # localization module exists.
    for consumer in [
        "OccupancyGridModule",
        "ElevationMapModule",
        "TerrainModule",
        "VoxelGridModule",
        "WavefrontFrontierExplorer",
        "TraversableFrontierModule",
        "RerunBridgeModule",
        "LocalPlannerModule",
        "PathFollowerModule",
        "SemanticMapperModule",
        "EpisodicMemoryModule",
        "TaggedLocationsModule",
        "VectorMemoryModule",
        "TemporalMemoryModule",
        "MissionLoggerModule",
        "SemanticPlannerModule",
        "VisualServoModule",
        "ReconstructionModule",
        "SafetyRingModule",
        "GeofenceManagerModule",
        "MCPServerModule",
    ]:
        _wire_if_present(
            bp,
            names,
            WireSpec(nav_odom_src, "odometry", consumer, "odometry"),
        )
    _wire_if_present(bp, names, WireSpec(nav_odom_src, "odometry", "GatewayModule", "odometry"))

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
            WireSpec("OccupancyGridModule", "exploration_grid", "WavefrontFrontierExplorer", "exploration_grid"),
            WireSpec("OccupancyGridModule", "exploration_grid", "TraversableFrontierModule", "exploration_grid"),
            WireSpec("OccupancyGridModule", "exploration_grid", "ROS2GridBridgeModule", "exploration_grid"),
            WireSpec("ElevationMapModule", "elevation_map", "TraversabilityCostModule", "elevation_map"),
            WireSpec("ElevationMapModule", "elevation_map", "TraversableFrontierModule", "elevation_map"),
            WireSpec("ESDFModule", "esdf", "TraversabilityCostModule", "esdf"),
            WireSpec("TerrainModule", "traversability", "TraversabilityCostModule", "traversability"),
            # NAV CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md) L2 安全门控:
            # fused_cost = 风险栅格 (occupancy+slope+ESDF). 它只喂全局安全门控与可视化,
            # 【绝不】作为局部规划主评分输入. 局部主输入是 terrain_map (见下方 L2 局部规划线).
            WireSpec("TraversabilityCostModule", "fused_cost", "NavigationModule", "costmap"),
            # ESDF 预留口: LocalPlannerCore (nav_core) 目前无 set_esdf 绑定, 该线缓存而不参与
            # 局部主评分. 升级前不得声称"局部规划用了 ESDF". 详见契约 §5.
            WireSpec("TraversabilityCostModule", "esdf_field", "LocalPlannerModule", "esdf"),
            WireSpec("TraversabilityCostModule", "fused_cost", "TraversableFrontierModule", "costmap"),
            WireSpec("TraversabilityCostModule", "fused_cost", "TraversableFrontierModule", "fused_cost"),
            WireSpec("TraversabilityCostModule", "slope_grid", "TraversableFrontierModule", "slope_grid"),
            WireSpec("TraversabilityCostModule", "esdf_field", "TraversableFrontierModule", "esdf_field"),
            WireSpec("TraversableFrontierModule", "traversable_frontiers", "GatewayModule", "traversable_frontiers"),
            WireSpec("TraversableFrontierModule", "frontier_candidate", "GatewayModule", "frontier_candidate"),
            WireSpec("TraversabilityCostModule", "fused_cost", "GatewayModule", "costmap"),
            WireSpec("TraversabilityCostModule", "slope_grid", "GatewayModule", "slope_grid"),
            WireSpec("NavigationModule", "mission_status", "TraversableFrontierModule", "navigation_status"),
            WireSpec("NavigationModule", "mission_status", "MissionLoggerModule", "mission_status"),
            WireSpec("PerceptionModule", "scene_graph", "GatewayModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "MCPServerModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "ReconstructionModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "TraversableFrontierModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "SemanticMapperModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "EpisodicMemoryModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "VectorMemoryModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "TemporalMemoryModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph"),
            WireSpec("PerceptionModule", "scene_graph", "VisualServoModule", "scene_graph"),
        ],
    )

    for recorder in ["DatasetRecorderModule", "ReconKeyframeExporterModule"]:
        _apply_if_present(
            bp,
            names,
            [
                WireSpec(camera_src, color_out, recorder, "color_image"),
                WireSpec(camera_src, "depth_image", recorder, "depth_image"),
                WireSpec(camera_src, "camera_info", recorder, "camera_info"),
                WireSpec(nav_odom_src, "odometry", recorder, "odometry"),
            ],
        )

    _apply_if_present(
        bp,
        names,
        [
            WireSpec("SafetyRingModule", "safety_state", "GatewayModule", "safety_state"),
            WireSpec("SafetyRingModule", "safety_state", "MCPServerModule", "safety_state"),
            WireSpec("NavigationModule", "mission_status", "GatewayModule", "mission_status"),
            WireSpec("NavigationModule", "mission_status", "MCPServerModule", "mission_status"),
            WireSpec("SafetyRingModule", "execution_eval", "GatewayModule", "execution_eval"),
            WireSpec("SafetyRingModule", "dialogue_state", "GatewayModule", "dialogue_state"),
            WireSpec("NavigationModule", "global_path", "GatewayModule", "global_path"),
            WireSpec("NavigationModule", "global_path", "LocalPlannerModule", "global_path"),
            WireSpec("NavigationModule", "global_path", "ROS2PathBridgeModule", "global_path"),
            WireSpec("LocalPlannerModule", "local_path", "GatewayModule", "local_path"),
            WireSpec("LocalPlannerModule", "local_path", "ROS2PathBridgeModule", "local_path"),
            WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
        ],
    )

    try:
        from core.registry import get as get_plugin

        if hasattr(get_plugin("driver", robot), "goal_pose"):
            WireSpec(drv, "goal_pose", "NavigationModule", "goal_pose").apply(bp)
    except Exception:
        pass

    _apply_if_present(
        bp,
        names,
        [
            # NAV CONTRACT L2 局部规划: terrain_map (局部几何障碍点云) 是 LocalPlanner 的
            # 主输入, global_path/waypoint 是 L5 下发的阶段目标 (指令派发, 非依赖反转).
            # LocalPlanner 用 CMU/nav_core 点云体素评分产出 local_path (非 costmap 滚动优化).
            WireSpec("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint"),
            WireSpec("NavigationModule", "clear_path", "LocalPlannerModule", "clear_path"),
            WireSpec("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map"),
            # NAV CONTRACT L2 控制跟踪: PathFollower 把 local_path 跟成 cmd_vel.
            WireSpec("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path"),
            WireSpec("LocalPlannerModule", "local_path", "SafetyRingModule", "path"),
            WireSpec("LocalPlannerModule", "control_hint", "PathFollowerModule", "control_hint"),
            WireSpec("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target"),
            WireSpec("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose"),
            WireSpec("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal"),
            WireSpec(camera_src, color_out, "TeleopModule", "color_image"),
            WireSpec("PerceptionModule", "scene_graph", "TeleopModule", "scene_graph"),
            WireSpec("TeleopModule", "teleop_active", "NavigationModule", "teleop_active"),
            WireSpec(camera_src, color_out, "WebRTCStreamModule", "color_image"),
            WireSpec("TeleopModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
            WireSpec("VisualServoModule", "cmd_vel", "CmdVelMux", "visual_servo_cmd_vel"),
            WireSpec("NavigationModule", "recovery_cmd_vel", "CmdVelMux", "recovery_cmd_vel"),
            WireSpec("PathFollowerModule", "cmd_vel", "CmdVelMux", "path_follower_cmd_vel"),
        ],
    )
    _wire_if_present(bp, names, WireSpec("CmdVelMux", "driver_cmd_vel", drv, "cmd_vel"))
    _wire_if_present(bp, names, WireSpec("CmdVelMux", "driver_cmd_vel", "SafetyRingModule", "cmd_vel"))

    return bp
