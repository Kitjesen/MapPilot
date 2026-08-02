"""LingTu product plugin catalog.

This file is the product-owned bridge between importable module files and the
generic runtime registry. Importing a module listed here runs its ``@register``
decorators, which makes the plugin visible to factories and services through
``runtime.registry``.

It is deliberately not a planner, profile, or module orchestrator:

- profiles decide which capabilities a launch wants;
- ``runtime.plugin_seed`` owns the generic import/seed mechanics;
- this catalog owns LingTu's concrete built-in module list;
- transport adapters are native DDS or process-local implementations.

The product global-planner backend group is intentionally narrow: production
startup registers OctoPlanner3D plus the explicit native FAR option. FAR does
not silently replace the default. Legacy PCT/A*/direct code may still exist as
archived tools or benchmarks, but it is not a product planner backend.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from runtime.plugin_seed import (
    register_plugin_module_catalog,
    seed_registered_plugins,
)

CATALOG_NAME = "lingtu_builtin"
BASE_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = {
    "device": ("runtime.devices.manager",),
    "driver": (
        "drivers.sim.stub",
        "drivers.real.thunder.han_dog_module",
    ),
    # Co-located with "driver": each module above also carries a
    # @register("driver_protocol", ...) decorator for protocol-based lookup.
    "driver_protocol": (
        "drivers.sim.stub",
        "drivers.real.thunder.han_dog_module",
    ),
    "driver_sim": (
        "drivers.sim.mujoco.driver",
        "drivers.sim.endpoint",
    ),
    "lidar": (
        "drivers.real.lidar.module",
        "drivers.sim.lidar.module",
    ),
    "imu": (
        "drivers.real.imu.module",
        "drivers.real.imu.dds_module",
        "drivers.sim.imu.module",
    ),
    "teleop": ("drivers.real.teleop_module",),
    "camera": (
        "drivers.real.camera.module",
        "drivers.real.camera.dds_module",
    ),
    "camera_sim": ("drivers.sim.camera.module",),
    "map": (
        "maps.modules.occupancy",
        "maps.modules.voxel_grid",
        "maps.modules.semantic",
        "maps.modules.esdf",
        "maps.modules.elevation",
        "maps.modules.traversability",
        "maps.modules.service",
    ),
    "map_save_adapter": ("maps.adapters.native.map_save",),
    "safety": (
        "nav.services.safety.safety_ring",
        "nav.services.safety.velocity_mux",
        "nav.services.geofence",
    ),
    "planner_backend": (
        "nav.services.plan.global_planner.algorithm.far",
        "nav.services.plan.global_planner.algorithm.octoplanner3d",
    ),
    "navigation": (
        "nav.navigation",
        "nav.commands.module",
        "nav.inspection.service",
        "explore.frontier",
        "explore.traversable_frontier",
    ),
    "map_dds": ("maps.adapters.dds.output",),
    "autonomy": (
        "nav.local.terrain",
        "nav.local.local_planner",
        "nav.local.path_follower",
    ),
    "slam": (
        "localization.profiles",
        "localization.slam.module",
        "localization.depth_visual_odom_module",
        "localization.gnss_module",
        "localization.gnss_bridge",
        "localization.ntrip_client_module",
        "localization.adapters.status",
    ),
    "sim_lidar": ("drivers.sim.pointcloud",),
    "exploration": (
        "explore.tare.module",
        "explore.tare.supervisor",
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
        "decision.modules.semantic_planner",
        "decision.modules.visual_servo",
    ),
    "llm": (
        "decision.llm.client",
        "decision.modules.llm",
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
    "visualization": ("runtime.rerun_module",),
}


BUILTIN_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = BASE_PLUGIN_MODULES


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
