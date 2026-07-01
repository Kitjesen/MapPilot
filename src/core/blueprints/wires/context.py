"""Shared full-stack wiring context."""

from __future__ import annotations

from dataclasses import dataclass

from core.blueprints.stacks.slam import slam_module_name


MAP_CLOUD_CONSUMERS = (
    "OccupancyGridModule",
    "ElevationMapModule",
    "TerrainModule",
    "VoxelGridModule",
    "RerunBridgeModule",
    "GatewayModule",
)

SEMANTIC_CAMERA_CONSUMERS = (
    "PerceptionModule",
    "ReconstructionModule",
    "VisualServoModule",
)

ODOMETRY_CONSUMERS = (
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
)

RECON_RECORDERS = (
    "DatasetRecorderModule",
    "ReconKeyframeExporterModule",
)


@dataclass(frozen=True)
class WiringContext:
    names: frozenset[str]
    robot: str
    driver_module: str
    slam_profile: str
    slam_module: str
    scene_xml: str
    enable_semantic: bool
    camera_src: str
    color_out: str
    nav_odom_src: str


def camera_source(names: set[str] | frozenset[str], *, driver_module: str) -> tuple[str, str]:
    camera_src = "CameraBridgeModule" if "CameraBridgeModule" in names else driver_module
    color_out = "color_image" if camera_src == "CameraBridgeModule" else "camera_image"
    return camera_src, color_out


def build_wiring_context(
    module_names: set[str] | frozenset[str],
    *,
    robot: str,
    driver_module: str,
    slam_profile: str,
    scene_xml: str = "",
    enable_semantic: bool = True,
) -> WiringContext:
    names = frozenset(module_names)
    slam_module = slam_module_name(slam_profile) or ""
    camera_src, color_out = camera_source(names, driver_module=driver_module)
    nav_odom_src = slam_module if (slam_module and slam_module in names) else driver_module
    return WiringContext(
        names=names,
        robot=robot,
        driver_module=driver_module,
        slam_profile=slam_profile,
        slam_module=slam_module,
        scene_xml=scene_xml,
        enable_semantic=enable_semantic,
        camera_src=camera_src,
        color_out=color_out,
        nav_odom_src=nav_odom_src,
    )
