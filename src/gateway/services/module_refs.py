"""Gateway runtime module reference discovery."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from gateway.services.teleop import bind_navigation_commands
from localization.service import RelocalizationService


def attach_module_refs(gw: Any, modules: dict[str, Any]) -> None:
    """Attach module references discovered by Blueprint/System startup."""
    gw._all_modules = dict(modules)
    gw._goals = modules.get("nav.goals")
    gw._nav_commands = modules.get("nav.commands")
    gw._inspection = modules.get("nav.inspection")
    bind_navigation_commands(gw, gw._nav_commands)
    relocalization_backend = next(
        (
            module
            for name in ("SlamAdapterModule",)
            for module in (modules.get(name),)
            if module is not None and _has_relocalization_capability(module)
        ),
        None,
    )
    if relocalization_backend is None:
        relocalization_backend = _native_relocalization_service()
    gw.localization.bind(relocalization_backend)
    gw._tagged_loc_module = next(
        (module for module in modules.values() if module.__class__.__name__ == "TaggedLocationsModule"),
        None,
    )


def navigation_state(nav: Any) -> str:
    """Return the uppercase navigation FSM state from a nav module reference.

    Shared by GatewayModule and MCPServerModule to avoid duplication.
    """
    if nav is None:
        return ""
    if isinstance(nav, Mapping):
        return str(nav.get("lifecycle_state_name") or nav.get("state") or "").upper()
    health: dict[str, Any] = {}
    if hasattr(nav, "health"):
        try:
            raw_health = nav.health() or {}
            if isinstance(raw_health, dict):
                health = raw_health
        except Exception:
            return "UNKNOWN"
    state = health.get("state")
    nested = health.get("navigation")
    if state is None and isinstance(nested, dict):
        state = nested.get("state")
    if hasattr(state, "value"):
        state = state.value
    return str(state or "").upper()


def _has_relocalization_capability(module: Any) -> bool:
    return all(
        callable(getattr(module, method, None))
        for method in (
            "trigger_global_relocalize",
            "relocalize_saved_map",
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
