"""Route registration exports for GatewayModule."""

from gateway.routes.app import register_app_routes
from gateway.routes.assets import mount_dashboard_assets, register_asset_routes
from gateway.routes.auth import register_auth_routes
from gateway.routes.camera import register_camera_routes
from gateway.routes.commands import register_command_routes
from gateway.routes.diagnostics import register_diagnostic_routes
from gateway.routes.inspection import register_inspection_routes
from gateway.routes.maps import map_lifecycle_payload, register_map_routes
from gateway.routes.operations import register_operation_routes
from gateway.routes.places import register_place_routes
from gateway.routes.realtime import register_realtime_routes
from gateway.routes.recordings import register_recording_routes
from gateway.routes.session import register_session_routes
from gateway.routes.status import register_status_routes
from gateway.routes.safety import register_safety_routes

__all__ = [
    "map_lifecycle_payload",
    "mount_dashboard_assets",
    "register_asset_routes",
    "register_app_routes",
    "register_auth_routes",
    "register_camera_routes",
    "register_command_routes",
    "register_diagnostic_routes",
    "register_inspection_routes",
    "register_map_routes",
    "register_operation_routes",
    "register_place_routes",
    "register_realtime_routes",
    "register_recording_routes",
    "register_session_routes",
    "register_status_routes",
    "register_safety_routes",
]
