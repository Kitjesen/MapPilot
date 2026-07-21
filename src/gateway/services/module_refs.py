"""Gateway runtime module reference discovery."""

from __future__ import annotations

from typing import Any

from localization.service import RelocalizationService
from gateway.services.teleop import bind_navigation_commands

_BACKEND_RECONFIGURE_TARGETS = {
    "detector": ("PerceptionModule",),
    "encoder": ("PerceptionModule",),
    "llm": ("LLMModule",),
    "llm_client": ("LLMModule",),
    "planner": ("nav.mission",),
    "local_planner": ("nav.local_planner",),
    "path_follower": ("nav.path_follower",),
    "terrain": ("nav.terrain",),
    "slam": ("SlamAdapterModule", "SlamModule"),
}


def attach_module_refs(gw: Any, modules: dict[str, Any]) -> None:
    """Attach module references discovered by Blueprint/System startup."""
    gw._map_mgr = modules.get("maps.service")
    gw._all_modules = dict(modules)
    gw._navigation = modules.get("nav.mission")
    gw._goals = modules.get("nav.goals")
    gw._nav_commands = modules.get("nav.commands")
    gw._inspection = modules.get("nav.inspection")
    bind_navigation_commands(gw, gw._nav_commands)
    gw._cmd_vel_mux = modules.get("nav.velocity_mux")
    gw._backend_reconfigure_modules = {
        module_name: modules.get(module_name)
        for module_names in _BACKEND_RECONFIGURE_TARGETS.values()
        for module_name in module_names
        if modules.get(module_name) is not None
    }
    relocalization_backend = next(
        (
            module
            for name in (
                "SlamAdapterModule",
                "SlamModule",
            )
            for module in (modules.get(name),)
            if module is not None and _has_relocalization_capability(module)
        ),
        None,
    )
    if relocalization_backend is None:
        relocalization_backend = _native_relocalization_service()
    gw.localization.bind(relocalization_backend)
    gw._frontier_explorer = next(
        (module for module in modules.values() if module.__class__.__name__ == "WavefrontFrontierExplorer"),
        None,
    )
    gw._tare_explorer = next(
        (module for module in modules.values() if module.__class__.__name__ == "TAREExplorerModule"),
        None,
    )
    gw._tagged_loc_module = next(
        (module for module in modules.values() if module.__class__.__name__ == "TaggedLocationsModule"),
        None,
    )

    # SwapManager is created after on_system_modules() returns, so it cannot be
    # discovered here. It is injected later by Blueprint.build() via
    # _set_swap_manager().
    gw._swap_manager = None


def backend_reconfigure_targets() -> dict[str, tuple[str, ...]]:
    return dict(_BACKEND_RECONFIGURE_TARGETS)


def _has_relocalization_capability(module: Any) -> bool:
    return all(
        callable(getattr(module, method, None))
        for method in (
            "trigger_global_relocalize",
            "relocalize_saved_map",
            "relocalize_saved_map_with_env",
        )
    )


def _native_relocalization_service() -> RelocalizationService | None:
    try:
        from localization.adapters.relocalization import (
            NativeSlamRelocalizationService,
        )

        service = NativeSlamRelocalizationService()
        return service if service.available() else None
    except Exception:
        return None
