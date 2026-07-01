"""Gateway, MCP, teleop, and WebRTC boundary wires."""

from __future__ import annotations

from .context import WiringContext
from .types import WireSpec


def gateway_command_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec("GatewayModule", "stop_cmd", ctx.driver_module, "stop_signal"),
        WireSpec("GatewayModule", "stop_cmd", "NavigationModule", "stop_signal"),
        WireSpec("GatewayModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        WireSpec("MCPServerModule", "stop_cmd", ctx.driver_module, "stop_signal"),
        WireSpec("MCPServerModule", "stop_cmd", "NavigationModule", "stop_signal"),
        WireSpec("MCPServerModule", "cmd_vel", "CmdVelMux", "teleop_cmd_vel"),
        WireSpec("GatewayModule", "instruction", "NavigationModule", "instruction"),
        WireSpec("MCPServerModule", "instruction", "NavigationModule", "instruction"),
        WireSpec("GatewayModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("GatewayModule", "cancel", "NavigationModule", "cancel"),
        WireSpec("MCPServerModule", "goal_pose", "NavigationModule", "goal_pose"),
    )


def gateway_status_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("NavigationModule", "mission_status", "GatewayModule", "mission_status"),
        WireSpec("NavigationModule", "mission_status", "MCPServerModule", "mission_status"),
        WireSpec("NavigationModule", "global_path", "GatewayModule", "global_path"),
        WireSpec("LocalPlannerModule", "local_path", "GatewayModule", "local_path"),
        WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
    )


def teleop_media_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec(ctx.camera_src, ctx.color_out, "TeleopModule", "color_image"),
        WireSpec("PerceptionModule", "scene_graph", "TeleopModule", "scene_graph"),
        WireSpec("TeleopModule", "teleop_active", "NavigationModule", "teleop_active"),
        WireSpec(ctx.camera_src, ctx.color_out, "WebRTCStreamModule", "color_image"),
    )
