"""Gateway stack: HTTP/WebSocket/SSE + MCP server + Teleop + Rerun.

All external interfaces share a single uvicorn process on port 5050:
  GatewayModule  — REST /api/v1/* + SSE /api/v1/events + WS /ws/teleop
  TeleopModule   — camera encoder (pushes JPEG to GatewayModule)
  MCPServerModule — JSON-RPC 2.0 at http://host:8090/mcp (separate port)
  RerunBridgeModule — optional Rerun 3D viz (separate port)
"""

from __future__ import annotations

import logging

from core.blueprint import Blueprint
from core.blueprints.stacks._registry import optional_stack_module, stack_module

logger = logging.getLogger(__name__)


def gateway(
    port: int = 5050,
    mcp_port: int = 8090,
    *,
    enable_teleop: bool = True,
    enable_rerun: bool = False,
    rerun_port: int = 9090,
    # teleop_port kept for backwards compat but ignored — teleop is on /ws/teleop
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
        bp.add(GatewayModule, alias="GatewayModule", port=port)
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
            fallback="drivers.teleop_module.TeleopModule",
        )
        if TeleopModule is not None:
            bp.add(
                TeleopModule,
                alias="TeleopModule",
                port=port,
            )  # informational — same port as Gateway
        # WebRTC module: shares the camera stream with TeleopModule but
        # encodes it as H.264 and speaks SDP on /api/v1/webrtc/offer for
        # ~100 ms glass-to-glass latency.  Imports are wrapped because
        # aiortc is an optional dependency on development machines.
        WebRTCStreamModule = optional_stack_module(
            "webrtc",
            "aiortc",
            seed_group="webrtc",
            fallback="webrtc.webrtc_stream_module.WebRTCStreamModule",
        )
        if WebRTCStreamModule is not None:
            bp.add(WebRTCStreamModule, alias="WebRTCStreamModule")
        else:
            logger.info("WebRTCStreamModule unavailable (aiortc not installed)")

    if enable_rerun:
        RerunBridgeModule = optional_stack_module(
            "visualization",
            "rerun",
            seed_group="visualization",
            fallback="compat.ros2.rerun_bridge.RerunBridgeModule",
        )
        if RerunBridgeModule is not None:
            bp.add(RerunBridgeModule, alias="RerunBridgeModule", web_port=rerun_port)
        else:
            logger.warning("RerunBridgeModule not available")

    return bp
