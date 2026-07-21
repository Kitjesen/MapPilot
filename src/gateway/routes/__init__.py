"""Lazy route registration exports for GatewayModule."""

from __future__ import annotations

from importlib import import_module
from typing import Any

_EXPORTS = {
    "mount_dashboard_assets": ("gateway.routes.assets", "mount_dashboard_assets"),
    "register_app_routes": ("gateway.routes.app", "register_app_routes"),
    "register_auth_routes": ("gateway.routes.auth", "register_auth_routes"),
    "register_camera_routes": ("gateway.routes.camera", "register_camera_routes"),
    "register_command_routes": ("gateway.routes.commands", "register_command_routes"),
    "register_diagnostic_routes": (
        "gateway.routes.diagnostics",
        "register_diagnostic_routes",
    ),
    "register_inspection_routes": (
        "gateway.routes.inspection",
        "register_inspection_routes",
    ),
    "map_lifecycle_payload": ("gateway.routes.maps", "map_lifecycle_payload"),
    "register_map_routes": ("gateway.routes.maps", "register_map_routes"),
    "register_operation_routes": (
        "gateway.routes.operations",
        "register_operation_routes",
    ),
    "register_place_routes": ("gateway.routes.places", "register_place_routes"),
    "register_realtime_routes": ("gateway.routes.realtime", "register_realtime_routes"),
    "register_session_routes": ("gateway.routes.session", "register_session_routes"),
    "register_status_routes": ("gateway.routes.status", "register_status_routes"),
    "register_voice_routes": ("gateway.routes.voice", "register_voice_routes"),
}

__all__ = list(_EXPORTS)


def __getattr__(name: str) -> Any:
    target = _EXPORTS.get(name)
    if target is None:
        raise AttributeError(name)
    module_name, attr = target
    module = import_module(module_name)
    value = getattr(module, attr)
    globals()[name] = value
    return value
