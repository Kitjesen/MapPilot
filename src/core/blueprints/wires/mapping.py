"""Map, traversability, and frontier support wires."""

from __future__ import annotations

from .types import WireSpec


def traversability_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("OccupancyGridModule", "costmap", "TraversabilityCostModule", "costmap"),
        WireSpec("OccupancyGridModule", "exploration_grid", "WavefrontFrontierExplorer", "exploration_grid"),
        WireSpec("OccupancyGridModule", "exploration_grid", "TraversableFrontierModule", "exploration_grid"),
        WireSpec("OccupancyGridModule", "exploration_grid", "EndpointGridBridgeModule", "exploration_grid"),
        WireSpec("ElevationMapModule", "elevation_map", "TraversabilityCostModule", "elevation_map"),
        WireSpec("ElevationMapModule", "elevation_map", "TraversableFrontierModule", "elevation_map"),
        WireSpec("ESDFModule", "esdf", "TraversabilityCostModule", "esdf"),
        WireSpec("TerrainModule", "traversability", "TraversabilityCostModule", "traversability"),
        # fused_cost is a global risk gate and visualization product. It must
        # not become the local planner's primary scoring input.
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
    )
