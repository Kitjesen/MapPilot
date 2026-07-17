"""Gateway stack: HTTP/WebSocket/SSE + MCP server + Teleop + Rerun.

All external interfaces share a single uvicorn process on port 5050:
  GatewayModule  â€?REST /api/v1/* + SSE /api/v1/events + WS /ws/teleop
  TeleopModule   â€?camera encoder (pushes JPEG to GatewayModule)
  MCPServerModule â€?JSON-RPC 2.0 at http://host:8090/mcp (separate port)
  RerunBridgeModule â€?optional Rerun 3D viz (separate port)
"""

from __future__ import annotations

import logging

from runtime.adapters.perception_gateway import rerun_bridge_module
from runtime.blueprint import Blueprint
from runtime.blueprints.stacks._registry import optional_stack_module, stack_module

logger = logging.getLogger(__name__)


def gateway(
    port: int = 5050,
    mcp_port: int = 8090,
    *,
    enable_teleop: bool = True,
    enable_rerun: bool = False,
    enable_ros2_rerun_bridge: bool = False,
    rerun_port: int = 9090,
    manage_session_services: bool = True,
    # teleop_port kept for backwards compat but ignored â€?teleop is on /ws/teleop
    teleop_port: int = 5050,
) -> Blueprint:
    """Build gateway stack: REST+SSE+WS teleop on one port, MCP on another."""
    bp = Blueprint()

    try:
        GatewayModule = stack_module(
            "gateway",
            "fastapi",
            seed_group="gateway",
            fallback="gateway.gateway_module.GatewayModule",
        )
        bp.add(
            GatewayModule,
            alias="GatewayModule",
            port=port,
            manage_session_services=manage_session_services,
        )
    except ImportError:
        logger.warning("GatewayModule not available")

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

    if enable_teleop:
        TeleopModule = optional_stack_module(
            "teleop",
            "default",
            seed_group="teleop",
            fallback="drivers.real.teleop_module.TeleopModule",
        )
        if TeleopModule is not None:
            bp.add(
                TeleopModule,
                alias="TeleopModule",
                port=port,
            )  # informational â€?same port as Gateway
    if enable_rerun:
        RerunBridgeModule = rerun_bridge_module(enable_ros2=enable_ros2_rerun_bridge)
        if RerunBridgeModule is not None:
            bp.add(RerunBridgeModule, alias="RerunBridgeModule", web_port=rerun_port)
        else:
            logger.warning("RerunBridgeModule not available")

    return bp
