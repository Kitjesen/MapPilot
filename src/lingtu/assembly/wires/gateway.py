"""Gateway, MCP, and teleop boundary wires."""

from __future__ import annotations

from .context import WiringContext
from .types import WireSpec


def gateway_command_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs = [
        WireSpec("GatewayModule", "stop_cmd", ctx.driver_module, "stop_signal"),
        WireSpec("GatewayModule", "stop_cmd", "nav.mission", "stop_signal"),
        WireSpec("GatewayModule", "cmd_vel", "nav.velocity_mux", "teleop_cmd_vel"),
        WireSpec("MCPServerModule", "stop_cmd", ctx.driver_module, "stop_signal"),
        WireSpec("MCPServerModule", "stop_cmd", "nav.mission", "stop_signal"),
        WireSpec("MCPServerModule", "cmd_vel", "nav.velocity_mux", "teleop_cmd_vel"),
    ]
    if "nav.goals" in ctx.names:
        specs.extend(
            [
                WireSpec(
                    "GatewayModule",
                    "goal_pose",
                    "nav.goals",
                    "goal_request",
                ),
                WireSpec(
                    "GatewayModule",
                    "cancel",
                    "nav.goals",
                    "cancel_request",
                ),
                WireSpec(
                    "MCPServerModule",
                    "goal_pose",
                    "nav.goals",
                    "goal_request",
                ),
            ]
        )
    else:
        specs.extend(
            [
                WireSpec("GatewayModule", "goal_pose", "nav.mission", "goal_pose"),
                WireSpec("GatewayModule", "cancel", "nav.mission", "cancel"),
                WireSpec("MCPServerModule", "goal_pose", "nav.mission", "goal_pose"),
            ]
        )
    return tuple(specs)


def gateway_status_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("nav.mission", "mission_status", "GatewayModule", "mission_status"),
        WireSpec("nav.mission", "mission_status", "MCPServerModule", "mission_status"),
        WireSpec("nav.mission", "mission_status", "AgentPlannerModule", "mission_status"),
        WireSpec("maps.service", "map_event", "GatewayModule", "map_event"),
        WireSpec("nav.mission", "global_path", "GatewayModule", "global_path"),
        WireSpec("nav.local_planner", "local_path", "GatewayModule", "local_path"),
        WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
        WireSpec("AgentPlannerModule", "agent_message", "GatewayModule", "agent_message"),
    )


def teleop_media_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    return (
        WireSpec(ctx.camera_src, ctx.color_out, "TeleopModule", "color_image"),
        WireSpec("PerceptionModule", "scene_graph", "TeleopModule", "scene_graph"),
        WireSpec("TeleopModule", "teleop_active", "nav.mission", "teleop_active"),
    )
