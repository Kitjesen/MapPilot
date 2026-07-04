"""LingTu product plugin catalog.

This file is the product-owned bridge between importable module files and the
generic runtime registry. Importing a module listed here runs its ``@register``
decorators, which makes the plugin visible to factories and services through
``runtime.registry``.

It is deliberately not a planner, profile, or module orchestrator:

- profiles decide which capabilities a launch wants;
- ``runtime.plugin_seed`` owns the generic import/seed mechanics;
- this catalog owns LingTu's concrete built-in module list;
- optional ROS2 compatibility groups live in ``lingtu.ros2_plugin_seed``.

The product global-planner backend group is intentionally narrow: production
startup registers OctoPlanner3D only. Legacy PCT/A*/direct code may still exist
as archived tools or benchmarks, but it is not a product planner backend.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from runtime.plugin_seed import (
    register_plugin_module_catalog,
    seed_registered_plugins,
)

CATALOG_NAME = "lingtu_builtin"

BASE_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = {
    "device": (
        "runtime.devices.manager",
    ),
    "driver": (
        "runtime.blueprints.stub",
        "drivers.real.thunder.han_dog_module",
    ),
    # Co-located with "driver": each module above also carries a
    # @register("driver_protocol", ...) decorator for protocol-based lookup.
    "driver_protocol": (
        "runtime.blueprints.stub",
        "drivers.real.thunder.han_dog_module",
    ),
    "driver_sim": (
        "drivers.sim.mujoco.driver",
        "drivers.sim.endpoint",
    ),
    "driver_legacy": (
        "drivers.real.thunder.connection",
    ),
    "lidar": (
        "drivers.real.lidar.lidar_module",
    ),
    "teleop": (
        "drivers.real.teleop_module",
    ),
    "camera": (
        "drivers.real.camera.native_camera_module",
    ),
    "map": (
        "nav.services.map_layers.occupancy_grid_module",
        "nav.services.map_layers.voxel_grid_module",
        "nav.services.map_layers.esdf_module",
        "nav.services.map_layers.elevation_map_module",
        "nav.services.map_layers.traversability_cost_module",
        "nav.services.maps",
    ),
    "map_save_adapter": (
        "runtime.adapters.native.map_save",
    ),
    "safety": (
        "nav.services.safety.safety_ring",
        "nav.services.safety.velocity_mux",
        "nav.services.geofence",
    ),
    "planner_backend": (
        "nav.services.plan.global_planner.algorithm.octoplanner3d",
    ),
    "navigation": (
        "nav.mission.navigation",
        "nav.exploration.frontier_explorer_module",
        "nav.exploration.traversable_frontier_module",
    ),
    "map_dds": (
        "runtime.adapters.dds.map_output",
    ),
    "autonomy": (
        "nav.local.terrain",
        "nav.services.plan.local_planner.service",
        "nav.local.path_follower",
    ),
    "slam": (
        "localization.profiles",
        "localization.slam.module",
        "localization.depth_visual_odom_module",
        "localization.gnss_module",
        "localization.gnss_bridge",
        "localization.ntrip_client_module",
        "runtime.adapters.native.localization_adapter",
    ),
    "slam_lcm": (
        "runtime.adapters.lcm.localization_adapter",
    ),
    "sim_lidar": (
        "drivers.sim.pointcloud",
    ),
    "exploration": (
        "nav.exploration.tare.module",
        "nav.exploration.tare.supervisor",
    ),
    "perception": (
        "perception.perception_module",
        "perception.detection.detector_module",
        "perception.encoding.encoder_module",
        "perception.api.factory",
    ),
    "reconstruction": (
        "perception.reconstruction.reconstruction_module",
        "perception.reconstruction.dataset_recorder_module",
        "perception.reconstruction.keyframe_exporter_module",
    ),
    "decision": (
        "decision.modules.semantic_planner_module",
        "decision.modules.visual_servo_module",
    ),
    "llm": (
        "decision.llm.llm_client",
        "decision.modules.llm_module",
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
        "runtime.rerun_module",
    ),
    "webrtc": (
        "gateway.media.webrtc_stream",
    ),
}


def _optional_ros2_plugin_modules() -> Mapping[str, tuple[str, ...]]:
    try:
        from lingtu.ros2_plugin_seed import ROS2_PLUGIN_MODULES
    except ModuleNotFoundError as exc:
        if exc.name != "lingtu.ros2_plugin_seed":
            raise
        return {}
    return ROS2_PLUGIN_MODULES


def _merge_plugin_modules(
    base: Mapping[str, tuple[str, ...]],
    optional: Mapping[str, tuple[str, ...]],
) -> Mapping[str, tuple[str, ...]]:
    merged: dict[str, tuple[str, ...]] = {key: tuple(value) for key, value in base.items()}
    for key, modules in optional.items():
        merged[key] = tuple(dict.fromkeys((*merged.get(key, ()), *modules)))
    return merged


BUILTIN_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = _merge_plugin_modules(
    BASE_PLUGIN_MODULES,
    _optional_ros2_plugin_modules(),
)


DEFAULT_BUILTIN_PLUGIN_GROUPS: tuple[str, ...] = (
    "device",
    "driver",
    "lidar",
    "teleop",
    "camera",
    "map",
    "safety",
    "planner_backend",
    "navigation",
    "autonomy",
    "slam",
    "exploration",
    "perception",
    "reconstruction",
    "decision",
    "llm",
    "memory",
)


def install_builtin_plugin_catalog() -> None:
    """Register the LingTu product catalog with the runtime seed hook."""

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
    """Import selected LingTu built-in plugin groups.

    This function is the normal startup entrypoint for product plugin
    registration. It imports only the requested groups; callers that need just
    the global planner should pass ``groups=("planner_backend",)`` instead of
    importing unrelated stacks.
    """

    install_builtin_plugin_catalog()
    return seed_registered_plugins(
        groups,
        catalog_names=(CATALOG_NAME,),
        reload_loaded=reload_loaded,
        strict=strict,
    )


install_builtin_plugin_catalog()
