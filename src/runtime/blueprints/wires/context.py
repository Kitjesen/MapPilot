"""Shared full-stack wiring context."""

from __future__ import annotations

from dataclasses import dataclass

from runtime.runtime_interface import TOPICS

MAP_CLOUD_CONSUMERS = (
    "OccupancyGridModule",
    "ElevationMapModule",
    "nav.terrain",
    "VoxelGridModule",
    "RerunBridgeModule",
    "GatewayModule",
)

MAP_CLOUD_FRAME_CONSUMERS = (
    "OccupancyGridModule",
    "ElevationMapModule",
    "nav.terrain",
    "VoxelGridModule",
    "nav.maps",
)

SEMANTIC_CAMERA_CONSUMERS = (
    "PerceptionModule",
    "ReconstructionModule",
    "VisualServoModule",
)

ODOMETRY_CONSUMERS = (
    "OccupancyGridModule",
    "ElevationMapModule",
    "nav.terrain",
    "VoxelGridModule",
    "WavefrontFrontierExplorer",
    "TraversableFrontierModule",
    "RerunBridgeModule",
    "nav.local_planner",
    "nav.path_follower",
    "SemanticMapperModule",
    "EpisodicMemoryModule",
    "TaggedLocationsModule",
    "VectorMemoryModule",
    "TemporalMemoryModule",
    "MissionLoggerModule",
    "SemanticPlannerModule",
    "VisualServoModule",
    "ReconstructionModule",
    "nav.safety",
    "GeofenceManagerModule",
    "MCPServerModule",
)

RECON_RECORDERS = (
    "DatasetRecorderModule",
    "ReconKeyframeExporterModule",
)

NAV_IN = "nav.in"
NAV_OUT = "nav.out"
MAP_OUT = "map.out"

TOPIC_NAV_GLOBAL_PATH = TOPICS.global_path
TOPIC_NAV_LOCAL_PATH = TOPICS.local_path
TOPIC_NAV_WAYPOINT = TOPICS.nav_way_point
TOPIC_NAV_CMD_VEL = TOPICS.cmd_vel
TOPIC_NAV_GOAL_POSE = TOPICS.goal_pose
TOPIC_NAV_CANCEL = TOPICS.cancel
TOPIC_NAV_INSTRUCTION = TOPICS.semantic_instruction
TOPIC_NAV_TERRAIN_MAP = TOPICS.terrain_map
TOPIC_NAV_TERRAIN_MAP_EXT = TOPICS.terrain_map_ext
TOPIC_NAV_TRAVERSABILITY = TOPICS.traversability
TOPIC_NAV_LOCAL_PLANNER_CLEAR_PATH = TOPICS.local_planner_clear_path
TOPIC_NAV_LOCAL_PLANNER_CONTROL_HINT = TOPICS.local_planner_control_hint
TOPIC_MAP_EXPLORATION_GRID = TOPICS.exploration_grid
TOPIC_SLAM_ODOMETRY = TOPICS.odometry
TOPIC_SLAM_MAP_CLOUD = TOPICS.map_cloud
TOPIC_SLAM_LOCALIZATION_HEALTH = TOPICS.localization_health
TOPIC_SLAM_LOCALIZATION_QUALITY = TOPICS.localization_quality


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
    slam_module = _selected_slam_module(names, slam_profile=slam_profile)
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


def _selected_slam_module(names: frozenset[str], *, slam_profile: str) -> str:
    if not slam_profile or slam_profile == "none":
        return ""
    if "SlamModule" in names:
        return "SlamModule"
    if "SlamAdapterModule" in names:
        return "SlamAdapterModule"
    if "SlamBridgeModule" in names:
        return "SlamBridgeModule"
    return ""
