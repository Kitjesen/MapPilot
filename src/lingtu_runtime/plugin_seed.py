"""LingTu product/runtime plugin catalog.

The core registry is generic. This module owns the concrete LingTu modules,
including optional ROS/LCM compatibility adapters.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from core.plugin_seed import (
    register_plugin_module_catalog,
    seed_registered_plugins,
)

CATALOG_NAME = "lingtu_builtin"

BUILTIN_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = {
    "device": (
        "core.devices.manager",
    ),
    "driver": (
        "core.blueprints.stub",
        "drivers.real.thunder.han_dog_module",
        "drivers.real.thunder.connection",
        "drivers.sim.mujoco_driver_module",
    ),
    # Co-located with "driver": each module above also carries a
    # @register("driver_protocol", ...) decorator for protocol-based lookup.
    "driver_protocol": (
        "core.blueprints.stub",
        "drivers.real.thunder.han_dog_module",
        "drivers.real.thunder.connection",
        "drivers.sim.mujoco_driver_module",
    ),
    "driver_ros2": (
        "compat.ros2.sim_driver",
    ),
    "lidar": (
        "drivers.real.lidar.lidar_module",
    ),
    "camera_ros2": (
        "compat.ros2.camera_bridge",
    ),
    "teleop": (
        "drivers.teleop_module",
    ),
    "map_save_adapter": (
        "compat.ros2.map_save",
    ),
    "map": (
        "nav.occupancy_grid_module",
        "nav.voxel_grid_module",
        "nav.esdf_module",
        "nav.elevation_map_module",
        "nav.traversability_cost_module",
        "nav.services.map_manager_module",
    ),
    "map_ros2": (
        "compat.ros2.nav.grid_bridge",
    ),
    "safety": (
        "nav.safety_ring_module",
        "nav.cmd_vel_mux_module",
        "nav.services.geofence_manager_module",
    ),
    "planner_backend": (
        "global_planning.pct_adapters.global_planner_module",
    ),
    "navigation": (
        "nav.navigation_module",
        "nav.frontier_explorer_module",
        "nav.traversable_frontier_module",
    ),
    "navigation_ros2": (
        "compat.ros2.nav.waypoint_bridge",
        "compat.ros2.nav.path_bridge",
    ),
    "navigation_lcm": (
        "compat.lcm.navigation_command_adapter",
        "compat.lcm.path_command_adapter",
    ),
    "autonomy": (
        "base_autonomy.modules.terrain_module",
        "base_autonomy.modules.local_planner_module",
        "base_autonomy.modules.path_follower_module",
    ),
    "slam": (
        "slam.slam_module",
        "slam.depth_visual_odom_module",
        "slam.gnss_module",
        "slam.gnss_bridge",
        "slam.ntrip_client_module",
    ),
    "slam_ros2": (
        "compat.ros2.slam_bridge",
    ),
    "slam_lcm": (
        "compat.lcm.localization_adapter",
    ),
    "sim_lidar": (
        "drivers.sim.sim_pointcloud_provider",
    ),
    "exploration": (
        "exploration.tare_explorer_module",
        "exploration.exploration_supervisor_module",
    ),
    "exploration_ros2": (
        "compat.ros2.tare_bridge",
    ),
    "perception": (
        "semantic.perception.perception_module",
        "semantic.perception.detector_module",
        "semantic.perception.encoder_module",
        "semantic.perception.api.factory",
    ),
    "reconstruction": (
        "semantic.reconstruction.reconstruction_module",
        "semantic.reconstruction.dataset_recorder_module",
        "semantic.reconstruction.keyframe_exporter_module",
    ),
    "semantic": (
        "semantic.planner.semantic_planner_module",
        "semantic.planner.visual_servo_module",
    ),
    "llm": (
        "semantic.planner.llm_client",
        "semantic.planner.llm_module",
    ),
    "memory": (
        "memory.modules.semantic_mapper_module",
        "memory.modules.episodic_module",
        "memory.modules.tagged_locations_module",
        "memory.modules.vector_memory_module",
        "memory.modules.temporal_memory_module",
        "memory.modules.mission_logger_module",
        "memory.modules.topological_module",
    ),
    "gateway": (
        "gateway.gateway_module",
        "gateway.mcp_server",
    ),
    "visualization": (
        "core.rerun_module",
    ),
    "visualization_ros2": (
        "compat.ros2.rerun_bridge",
    ),
    "webrtc": (
        "webrtc.webrtc_stream_module",
    ),
}


DEFAULT_BUILTIN_PLUGIN_GROUPS: tuple[str, ...] = (
    "device",
    "driver",
    "lidar",
    "teleop",
    "map",
    "safety",
    "planner_backend",
    "navigation",
    "autonomy",
    "slam",
    "exploration",
    "perception",
    "reconstruction",
    "semantic",
    "llm",
    "memory",
)


def install_builtin_plugin_catalog() -> None:
    """Register the LingTu product/runtime plugin catalog with core."""

    register_plugin_module_catalog(
        CATALOG_NAME,
        BUILTIN_PLUGIN_MODULES,
        default_groups=DEFAULT_BUILTIN_PLUGIN_GROUPS,
        replace=True,
    )


def seed_builtin_plugins(
    groups: Iterable[str] | None = None,
    *,
    reload_loaded: bool = False,
    strict: bool = False,
) -> dict[str, dict[str, list[str]]]:
    """Import LingTu built-in plugin modules."""

    install_builtin_plugin_catalog()
    return seed_registered_plugins(
        groups,
        catalog_names=(CATALOG_NAME,),
        reload_loaded=reload_loaded,
        strict=strict,
    )


install_builtin_plugin_catalog()
