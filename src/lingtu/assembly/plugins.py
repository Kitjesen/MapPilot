"""LingTu built-in plugin catalog.

This file is the product-owned bridge between importable module files and the
generic runtime registry. Importing a module listed here runs its ``@register``
decorators, which makes the plugin visible to factories and services through
``runtime.registry``.

It is deliberately not a planner, Product, or module orchestrator:

- Products decide which capabilities a launch wants;
- ``runtime.plugin_seed`` owns the generic import/seed mechanics;
- this catalog owns LingTu's concrete built-in module list;
- transport adapters are native DDS or process-local implementations.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

from runtime.plugin_seed import (
    register_plugin_module_catalog,
    seed_registered_plugins,
)

CATALOG_NAME = "lingtu_builtin"
BASE_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = {
    "driver": ("drivers.sim.stub",),
    "driver_protocol": ("drivers.sim.stub",),
    "driver_sim": (
        "drivers.sim.mujoco.driver",
        "drivers.sim.endpoint",
    ),
    "lidar": (
        "drivers.real.lidar.module",
        "drivers.sim.lidar.module",
    ),
    "teleop": ("drivers.real.camera_jpeg_relay",),
    "camera": (
        "drivers.real.camera.module",
        "drivers.real.camera.dds_module",
    ),
    "camera_sim": ("drivers.sim.camera.module",),
    "navigation": (
        "nav.commands.module",
        "nav.inspection.service",
        "nav.skills.skills_module",
    ),
    "slam": (
        "localization.profiles",
        "localization.adapters.status",
    ),
    "exploration": (
        "explore.tare.module",
        "explore.tare.supervisor",
    ),
    "perception": (
        "perception.perception_module",
        "perception.detection.detector_module",
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
}


BUILTIN_PLUGIN_MODULES: Mapping[str, tuple[str, ...]] = BASE_PLUGIN_MODULES


DEFAULT_BUILTIN_PLUGIN_GROUPS: tuple[str, ...] = (
    "driver",
    "lidar",
    "teleop",
    "camera",
    "navigation",
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
    registration. It imports only the requested groups.
    """

    install_builtin_plugin_catalog()
    return seed_registered_plugins(
        groups,
        catalog_names=(CATALOG_NAME,),
        reload_loaded=reload_loaded,
        strict=strict,
    )


install_builtin_plugin_catalog()
