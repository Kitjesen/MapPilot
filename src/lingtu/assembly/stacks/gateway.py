"""Gateway stack: HTTP/WebSocket/SSE + MCP server + camera relay.

External interfaces are assembled into one Host:
  GatewayModule: REST, SSE, and teleop WebSocket on port 5050.
  MCPServerModule: JSON-RPC on port 8090.
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.blueprint import Blueprint
from runtime.plugin_resolution import optional_stack_module, stack_module

logger = logging.getLogger(__name__)


def gateway(
    port: int = 5050,
    mcp_port: int = 8090,
    *,
    enable_teleop: bool = True,
    enable_camera: bool = False,
    command_output_mode: str | None = None,
    hardware_control_boundary: str | None = None,
    product: str | None = None,
    run_plan: Any | None = None,
) -> Blueprint:
    """Build gateway stack: REST+SSE+WS teleop on one port, MCP on another."""
    bp = Blueprint()

    GatewayModule = stack_module(
        "gateway",
        "fastapi",
        seed_group="gateway",
        fallback="gateway.gateway_module.GatewayModule",
    )
    gateway_config: dict[str, Any] = {
        key: value
        for key, value in {
            "port": port,
            "command_output_mode": command_output_mode,
            "hardware_control_boundary": hardware_control_boundary,
            "product": product,
            "run_plan": run_plan,
        }.items()
        if value is not None
    }
    bp.add(GatewayModule, alias="GatewayModule", **gateway_config)

    MCPServerModule = optional_stack_module(
        "mcp",
        "server",
        seed_group="gateway",
        fallback="gateway.mcp_server.MCPServerModule",
    )
    if MCPServerModule is not None:
        bp.add(MCPServerModule, alias="MCPServerModule", port=mcp_port)
    else:
        logger.warning("MCPServerModule not available")

    if enable_teleop and enable_camera:
        CameraJpegRelayModule = optional_stack_module(
            "media",
            "jpeg_relay",
            seed_group="teleop",
            fallback="drivers.real.camera_jpeg_relay.CameraJpegRelayModule",
        )
        if CameraJpegRelayModule is not None:
            bp.add(CameraJpegRelayModule, alias="CameraJpegRelayModule")
    return bp
