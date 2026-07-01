"""Map stack: runtime map layers plus MapService."""

from __future__ import annotations

import logging
import os

from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module
from runtime.blueprints.adapters.mapping_slam import map_output_adapter_module
from runtime.blueprints.wires.mapping import map_output_specs
from runtime.blueprints.wires.context import MAP_OUT
from runtime.blueprints.wires.types import wire_present_specs
from runtime.profiles.binding_policy import (
    map_output_adapter_enabled,
    map_output_uses_dds,
    map_output_uses_ros2,
)

logger = logging.getLogger(__name__)


def maps(**config) -> Blueprint:
    """Real-time map layers from LiDAR point cloud, plus map lifecycle management.

    Params sourced from robot_config.yaml (occupancy_grid / voxel_grid sections).
    MapService is always included 閳?it handles list/save/use/delete/build
    commands arriving via its map_command In port.
    """
    bp = Blueprint()
    try:
        from runtime.config import get_config

        OccupancyGridModule = stack_module(
            "map",
            "occupancy_grid",
            seed_group="map",
            fallback="nav.services.map_layers.occupancy_grid_module.OccupancyGridModule",
        )
        VoxelGridModule = stack_module(
            "map",
            "voxel",
            seed_group="map",
            fallback="nav.services.map_layers.voxel_grid_module.VoxelGridModule",
        )
        ESDFModule = stack_module(
            "map",
            "esdf",
            seed_group="map",
            fallback="nav.services.map_layers.esdf_module.ESDFModule",
        )
        ElevationMapModule = stack_module(
            "map",
            "elevation",
            seed_group="map",
            fallback="nav.services.map_layers.elevation_map_module.ElevationMapModule",
        )
        TraversabilityCostModule = stack_module(
            "map",
            "traversability_cost",
            seed_group="map",
            fallback="nav.services.map_layers.traversability_cost_module.TraversabilityCostModule",
        )
        cfg = get_config()
        og = cfg.raw.get("occupancy_grid", {})

        bp.add(
            OccupancyGridModule,
            alias="OccupancyGridModule",
            resolution=config.get("grid_resolution", og.get("resolution", 0.2)),
            map_radius=config.get("grid_radius", og.get("map_radius", 30.0)),
            inflation_radius=config.get("inflation_radius", og.get("inflation_radius", 0.5)),
            z_min=og.get("z_min", 0.10),
            z_max=og.get("z_max", 2.00),
            publish_hz=og.get("publish_hz", 2.0),
            frame_id=config.get("occupancy_frame_id", "map"),
            raycast_free_space=config.get("occupancy_raycast_free_space", False),
            unknown_as_obstacle_for_costmap=config.get(
                "occupancy_unknown_as_obstacle_for_costmap",
                False,
            ),
            raycast_max_rays=config.get("occupancy_raycast_max_rays", 1800),
            raycast_free_inflation_radius=config.get(
                "occupancy_raycast_free_inflation_radius",
                og.get("raycast_free_inflation_radius", 0.0),
            ),
        )
        if map_output_adapter_enabled(config):
            if map_output_uses_dds(config):
                GridAdapterModule = map_output_adapter_module(enable_dds=True)
            elif map_output_uses_ros2(config):
                GridAdapterModule = map_output_adapter_module(enable_ros2=True)
            else:
                logger.warning(
                    "Map output adapter requested without an explicit adapter; "
                    "skipping instead of selecting a ROS2 adapter by default"
                )
                GridAdapterModule = None
            if GridAdapterModule is not None:
                bp.add(GridAdapterModule, alias=MAP_OUT)
                wire_present_specs(bp, map_output_specs())
            else:
                logger.warning("Map output adapter not available")

        vg = cfg.raw.get("voxel_grid", {})
        bp.add(
            VoxelGridModule,
            alias="VoxelGridModule",
            voxel_size=config.get("voxel_size", vg.get("voxel_size", 0.05)),
            max_range=config.get("voxel_max_range", vg.get("max_range", 20.0)),
            min_z=vg.get("min_z", -0.5),
            max_z=vg.get("max_z", 3.0),
            decay_rate=vg.get("decay_rate", 0.01),
            publish_interval=vg.get("publish_interval", 2.0),
        )

        bp.add(ESDFModule, alias="ESDFModule")
        bp.add(
            ElevationMapModule,
            alias="ElevationMapModule",
            resolution=config.get("elev_resolution", og.get("resolution", 0.2)),
            map_radius=config.get("elev_radius", 15.0),
        )

        tc = cfg.raw.get("traversability_cost", {})
        tc_kw: dict = dict(
            safe_distance=tc.get("safe_distance", 1.5),
            proximity_cap=tc.get("proximity_cap", 50.0),
            publish_hz=tc.get("publish_hz", 2.0),
        )
        if "max_slope_deg" in tc:
            tc_kw["max_slope_deg"] = tc["max_slope_deg"]
        # else: auto-read from terrain_analysis.slope_max (single source of truth)
        bp.add(TraversabilityCostModule, alias="TraversabilityCostModule", **tc_kw)
    except ImportError as e:
        logger.warning("Map modules not available: %s", e)

    # MapService: map lifecycle (list/save/use/build/delete).
    # Always included so the REPL and Gateway can issue map_command messages
    # without needing a direct subprocess call.
    try:
        MapService = stack_module(
            "map",
            "manager",
            seed_group="map",
            fallback="nav.services.maps.MapService",
        )
        map_dir = config.get(
            "map_dir",
            os.environ.get(
                "NAV_MAP_DIR",
                # Canonical sunrise map dir 閳?see note in
                # nav.services.map_layers.__init__ for why this must match.
                os.path.expanduser("~/data/nova/maps"),
            ),
        )
        bp.add(MapService, alias="nav.maps", map_dir=map_dir)
    except ImportError as e:
        logger.warning("MapService not available: %s", e)

    return bp
