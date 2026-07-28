"""Gateway, MCP, and teleop boundary wires."""

from __future__ import annotations

from .context import WiringContext
from .types import WireSpec


def gateway_command_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    native_host = "host.bus" in ctx.names
    specs: list[WireSpec] = []
    if not native_host:
        specs.extend(
            [
                WireSpec("GatewayModule", "stop_cmd", ctx.driver_module, "stop_signal"),
                WireSpec("GatewayModule", "stop_cmd", "nav.mission", "stop_signal"),
                WireSpec("GatewayModule", "cmd_vel", "nav.velocity_mux", "teleop_cmd_vel"),
                WireSpec("MCPServerModule", "stop_cmd", ctx.driver_module, "stop_signal"),
                WireSpec("MCPServerModule", "stop_cmd", "nav.mission", "stop_signal"),
                WireSpec("MCPServerModule", "cmd_vel", "nav.velocity_mux", "teleop_cmd_vel"),
            ]
        )
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


def gateway_status_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs = [
        WireSpec("host.bus", "navigation_state", "GatewayModule", "navigation_state"),
        WireSpec(
            "host.bus",
            "navigation_goal_status",
            "GatewayModule",
            "navigation_goal_status",
        ),
        WireSpec(
            "host.bus",
            "inspection_task_event",
            "GatewayModule",
            "inspection_task_event",
        ),
        WireSpec("host.bus", "navigation_state", "MCPServerModule", "navigation_state"),
        WireSpec(
            "host.bus",
            "navigation_goal_status",
            "MCPServerModule",
            "navigation_goal_status",
        ),
        WireSpec("host.bus", "navigation_state", "AgentPlannerModule", "navigation_state"),
        WireSpec(
            "host.bus",
            "navigation_goal_status",
            "AgentPlannerModule",
            "navigation_goal_status",
        ),
        WireSpec("host.bus", "navigation_state", "nav.skills", "navigation_state"),
        WireSpec(
            "host.bus",
            "navigation_goal_status",
            "nav.skills",
            "navigation_goal_status",
        ),
        WireSpec("maps.service", "map_event", "GatewayModule", "map_event"),
        WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
        WireSpec("AgentPlannerModule", "agent_message", "GatewayModule", "agent_message"),
    ]
    if "host.bus" in ctx.names:
        specs.extend(
            [
                WireSpec("host.bus", "global_path", "GatewayModule", "global_path"),
                WireSpec("host.bus", "local_path", "GatewayModule", "local_path"),
                WireSpec("host.bus", "map_scene", "GatewayModule", "map_scene"),
            ]
        )
        if "nav.goals" in ctx.names:
            specs.append(
                WireSpec(
                    "host.bus",
                    "navigation_goal_status",
                    "nav.goals",
                    "navigation_goal_status",
                )
            )
    else:
        specs.extend(
            [
                WireSpec("nav.mission", "mission_status", "GatewayModule", "mission_status"),
                WireSpec("nav.mission", "mission_status", "MCPServerModule", "mission_status"),
                WireSpec("nav.mission", "mission_status", "AgentPlannerModule", "mission_status"),
                WireSpec("nav.mission", "global_path", "GatewayModule", "global_path"),
                WireSpec("nav.local_planner", "local_path", "GatewayModule", "local_path"),
            ]
        )
    return tuple(specs)


def teleop_media_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    media_module = (
        "CameraJpegRelayModule"
        if "CameraJpegRelayModule" in ctx.names
        else "TeleopModule"
    )
    specs = [
        WireSpec(ctx.camera_src, ctx.color_out, media_module, "color_image"),
        WireSpec("PerceptionModule", "scene_graph", media_module, "scene_graph"),
    ]
    if media_module == "TeleopModule":
        specs.append(WireSpec("TeleopModule", "teleop_active", "nav.mission", "teleop_active"))
    return tuple(specs)
