"""Map stack: runtime map layers plus native maps service."""

from __future__ import annotations

import logging

from maps.adapters.resolver import map_output_adapter_module
from maps.paths import nav_map_root_str
from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import stack_module
from runtime.blueprints.wires.context import MAP_OUT
from runtime.blueprints.wires.mapping import map_output_specs
from runtime.blueprints.wires.types import wire_present_specs
from runtime.profiles.binding_policy import (
    map_output_adapter_enabled,
    map_output_uses_dds,
)

logger = logging.getLogger(__name__)


def maps(**config) -> Blueprint:
    """Real-time map layers from LiDAR point cloud, plus map lifecycle management.

    Params sourced from robot_config.yaml (occupancy_grid / voxel_grid sections).
    MapsModule is always included; it handles list/save/use/delete/build
    commands arriving via its map_command In port.
    """
    bp = Blueprint()
    try:
        from runtime.config import get_config

        OccupancyGridModule = stack_module(
            "map",
            "occupancy_grid",
            seed_group="map",
            fallback="maps.modules.occupancy.OccupancyGridModule",
        )
        VoxelGridModule = stack_module(
            "map",
            "voxel",
            seed_group="map",
            fallback="maps.modules.voxel_grid.VoxelGridModule",
        )
        SemanticMapModule = stack_module(
            "map",
            "semantic",
            seed_group="map",
            fallback="maps.modules.semantic.SemanticMapModule",
        )
        ESDFModule = stack_module(
            "map",
            "esdf",
            seed_group="map",
            fallback="maps.modules.esdf.ESDFModule",
        )
        ElevationMapModule = stack_module(
            "map",
            "elevation",
            seed_group="map",
            fallback="maps.modules.elevation.ElevationMapModule",
        )
        TraversabilityCostModule = stack_module(
            "map",
            "traversability_cost",
            seed_group="map",
            fallback="maps.modules.traversability.TraversabilityCostModule",
        )
        cfg = get_config()
        og = cfg.raw.get("occupancy_grid", {})

        bp.add(
            OccupancyGridModule,
            alias="OccupancyGridModule",
            resolution=config.get("grid_resolution", og.get("resolution", 0.2)),
            map_radius=config.get("grid_radius", og.get("map_radius", 30.0)),
            inflation_radius=config.get("inflation_radius", og.get("inflation_radius", 0.5)),
            robot_clear_radius=config.get(
                "robot_clear_radius",
                og.get("robot_clear_radius", 0.60),
            ),
            robot_clear_forward=config.get(
                "robot_clear_forward",
                og.get("robot_clear_forward", og.get("robot_clear_forward_m", 0.0)),
            ),
            robot_clear_backward=config.get(
                "robot_clear_backward",
                og.get("robot_clear_backward", og.get("robot_clear_backward_m", 0.0)),
            ),
            robot_clear_lateral=config.get(
                "robot_clear_lateral",
                og.get("robot_clear_lateral", og.get("robot_clear_lateral_m", 0.0)),
            ),
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
            else:
                logger.warning(
                    "Map output adapter requested without an explicit adapter; only native DDS map output is supported"
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
            backend=config.get("voxel_backend", vg.get("backend", "cpp")),
            column_carving=config.get(
                "voxel_column_carving",
                vg.get("column_carving", True),
            ),
        )
        bp.add(
            SemanticMapModule,
            alias="SemanticMapModule",
            voxel_size=config.get("semantic_voxel_size", 0.2),
            publish_interval=config.get("semantic_publish_interval", 1.0),
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

    # MapsModule: map lifecycle (list/save/use/build/delete).
    # Always included so the REPL and Gateway can issue map_command messages
    # without needing a direct subprocess call.
    try:
        MapsModule = stack_module(
            "map",
            "service",
            seed_group="map",
            fallback="maps.modules.service.MapsModule",
        )
        map_dir = config.get(
            "map_dir",
            nav_map_root_str(),
        )
        maps_service_config = {"map_dir": map_dir}
        for key in (
            "map_artifact_converter_command",
            "octomap_converter_command",
            "octomap_build_mode",
            "octomap_resolution",
            "octomap_free_layers_above",
            "octomap_free_dilation_cells",
            "octomap_build_timeout_sec",
            "build_octomap_on_save",
            "map_prune_command",
            "dynamic_filter_command",
            "map_opt",
            "map_opt_command",
            "map_opt_timeout_sec",
            "map_opt_required",
        ):
            if key in config and config[key] is not None:
                maps_service_config[key] = config[key]
        bp.add(MapsModule, alias="maps.service", **maps_service_config)
    except ImportError as e:
        logger.warning("MapsModule not available: %s", e)

    return bp
