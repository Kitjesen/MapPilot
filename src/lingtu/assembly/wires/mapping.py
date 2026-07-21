"""Map, traversability, and frontier support wires."""

from __future__ import annotations

from .context import (
    MAP_OUT,
    TOPIC_MAP_EXPLORATION_GRID,
    TOPIC_MAPS_SCENE,
    TOPIC_MAPS_VOXEL_CLOUD,
    TOPIC_NAV_TRAVERSABILITY,
)
from .types import WireSpec


def map_output_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec(
            "OccupancyGridModule",
            "exploration_grid",
            MAP_OUT,
            "exploration_grid",
            topic=TOPIC_MAP_EXPLORATION_GRID,
        ),
        WireSpec(
            "TraversabilityCostModule",
            "fused_cost",
            MAP_OUT,
            "traversability",
            topic=TOPIC_NAV_TRAVERSABILITY,
        ),
    )


def traversability_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec(
            "ReconstructionModule",
            "semantic_labels",
            "SemanticMapModule",
            "semantic_labels",
        ),
        WireSpec("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireSpec("OccupancyGridModule", "exploration_grid", "WavefrontFrontierExplorer", "exploration_grid"),
        WireSpec("OccupancyGridModule", "exploration_grid", "TraversableFrontierModule", "exploration_grid"),
        WireSpec("OccupancyGridModule", "exploration_grid", "TAREExplorerModule", "exploration_grid"),
        WireSpec("ElevationMapModule", "elevation_map", "TraversabilityCostModule", "elevation_map"),
        WireSpec("ElevationMapModule", "elevation_map", "TraversableFrontierModule", "elevation_map"),
        WireSpec("ESDFModule", "esdf", "TraversabilityCostModule", "esdf"),
        WireSpec("nav.terrain", "traversability", "TraversabilityCostModule", "traversability"),
        WireSpec("nav.terrain", "traversability", "nav.mission", "traversability"),
        # fused_cost is a global risk gate and visualization product. It must
        # not become the local planner's primary scoring input.
        WireSpec("TraversabilityCostModule", "fused_cost", "nav.mission", "costmap"),
        WireSpec("TraversabilityCostModule", "esdf_field", "nav.local_planner", "esdf"),
        WireSpec("TraversabilityCostModule", "fused_cost", "TraversableFrontierModule", "costmap"),
        WireSpec("TraversabilityCostModule", "fused_cost", "TraversableFrontierModule", "fused_cost"),
        WireSpec("TraversabilityCostModule", "slope_grid", "TraversableFrontierModule", "slope_grid"),
        WireSpec("TraversabilityCostModule", "esdf_field", "TraversableFrontierModule", "esdf_field"),
        WireSpec("TraversableFrontierModule", "traversable_frontiers", "GatewayModule", "traversable_frontiers"),
        WireSpec("TraversableFrontierModule", "frontier_candidate", "GatewayModule", "frontier_candidate"),
        WireSpec(
            "VoxelGridModule",
            "scene",
            "GatewayModule",
            "map_scene",
            topic=TOPIC_MAPS_SCENE,
        ),
        WireSpec(
            "SemanticMapModule",
            "scene",
            "GatewayModule",
            "map_scene",
            topic=TOPIC_MAPS_SCENE,
        ),
        WireSpec(
            "maps.service",
            "semantic_save_request",
            "SemanticMapModule",
            "semantic_save_request",
        ),
        WireSpec(
            "SemanticMapModule",
            "semantic_save_result",
            "maps.service",
            "semantic_save_result",
        ),
        WireSpec(
            "VoxelGridModule",
            "voxel_cloud",
            "GatewayModule",
            "voxel_cloud",
            topic=TOPIC_MAPS_VOXEL_CLOUD,
        ),
        WireSpec("TraversabilityCostModule", "fused_cost", "GatewayModule", "costmap"),
        WireSpec("TraversabilityCostModule", "slope_grid", "GatewayModule", "slope_grid"),
        WireSpec("nav.mission", "mission_status", "TraversableFrontierModule", "navigation_status"),
        WireSpec("nav.mission", "mission_status", "MissionLoggerModule", "mission_status"),
    )
