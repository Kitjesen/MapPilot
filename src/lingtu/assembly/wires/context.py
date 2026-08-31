"""Shared full-stack wiring context."""

from __future__ import annotations

from dataclasses import dataclass

from runtime.contracts import CAMERA_ROLE
from runtime.runtime_interface import TOPICS

MAP_OBSERVATION_CONSUMERS = (
    "ReconstructionModule",
)

SEMANTIC_CAMERA_CONSUMERS = (
    "PerceptionModule",
    "InspectionEvidenceModule",
    "ReconstructionModule",
)

ODOMETRY_CONSUMERS = (
    "SemanticMapperModule",
    "EpisodicMemoryModule",
    "TaggedLocationsModule",
    "VectorMemoryModule",
    "TemporalMemoryModule",
    "MissionLoggerModule",
    "SemanticPlannerModule",
    "AgentPlannerModule",
    "InspectionEvidenceModule",
    "ReconstructionModule",
    "MCPServerModule",
)

RECON_RECORDERS = (
    "DatasetRecorderModule",
    "ReconKeyframeExporterModule",
)

MAP_OUT = "map.out"

TOPIC_NAV_TERRAIN_MAP = TOPICS.terrain_map
TOPIC_NAV_TERRAIN_MAP_EXT = TOPICS.terrain_map_ext
TOPIC_NAV_TRAVERSABILITY = TOPICS.traversability
TOPIC_NAV_LOCAL_PLANNER_CLEAR_PATH = TOPICS.local_planner_clear_path
TOPIC_NAV_LOCAL_PLANNER_CONTROL_HINT = TOPICS.local_planner_control_hint
TOPIC_MAP_EXPLORATION_GRID = TOPICS.exploration_grid
TOPIC_MAPS_SCENE = TOPICS.maps_scene
TOPIC_MAPS_VOXEL_CLOUD = TOPICS.maps_voxel_cloud
TOPIC_SLAM_ODOMETRY = TOPICS.odometry
TOPIC_SLAM_LOCALIZATION_HEALTH = TOPICS.localization_health
TOPIC_SLAM_LOCALIZATION_QUALITY = TOPICS.localization_quality


@dataclass(frozen=True)
class WiringContext:
    names: frozenset[str]
    driver_module: str
    slam_module: str
    camera_src: str
    color_out: str
    nav_odom_src: str


def camera_source(names: set[str] | frozenset[str], *, driver_module: str) -> tuple[str, str]:
    if CAMERA_ROLE in names:
        camera_src = CAMERA_ROLE
    else:
        camera_src = driver_module
    color_out = "color_image" if camera_src == CAMERA_ROLE else "camera_image"
    return camera_src, color_out


def build_wiring_context(
    module_names: set[str] | frozenset[str],
    *,
    driver_module: str,
    slam_profile: str,
) -> WiringContext:
    names = frozenset(module_names)
    slam_module = _selected_slam_module(names, slam_profile=slam_profile)
    camera_src, color_out = camera_source(names, driver_module=driver_module)
    nav_odom_src = slam_module if (slam_module and slam_module in names) else driver_module
    return WiringContext(
        names=names,
        driver_module=driver_module,
        slam_module=slam_module,
        camera_src=camera_src,
        color_out=color_out,
        nav_odom_src=nav_odom_src,
    )


def _selected_slam_module(names: frozenset[str], *, slam_profile: str) -> str:
    if not slam_profile or slam_profile == "none":
        return ""
    if "SlamAdapterModule" in names:
        return "SlamAdapterModule"
    return ""
