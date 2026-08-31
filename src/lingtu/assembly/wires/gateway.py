"""Gateway, MCP, and teleop boundary wires."""

from __future__ import annotations

from runtime.wiring import WireSpec

from .context import WiringContext


def gateway_status_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Connect native Host status streams to Gateway-facing Modules."""

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
        WireSpec(
            "host.bus",
            "exploration_run_event",
            "GatewayModule",
            "exploration_run_event",
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
        WireSpec("SemanticPlannerModule", "agent_message", "GatewayModule", "agent_message"),
        WireSpec("AgentPlannerModule", "agent_message", "GatewayModule", "agent_message"),
    ]
    if "host.bus" in ctx.names:
        specs.extend(
            [
                WireSpec("host.bus", "global_path", "GatewayModule", "global_path"),
                WireSpec("host.bus", "local_path", "GatewayModule", "local_path"),
                WireSpec("host.bus", "map_scene", "GatewayModule", "map_scene"),
                WireSpec(
                    "host.bus",
                    "traversability",
                    "GatewayModule",
                    "native_traversability",
                ),
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
    return tuple(specs)


def teleop_media_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Connect camera and perception media to the JPEG relay."""

    specs = [
        WireSpec(ctx.camera_src, ctx.color_out, "CameraJpegRelayModule", "color_image"),
        WireSpec("PerceptionModule", "scene_graph", "CameraJpegRelayModule", "scene_graph"),
    ]
    return tuple(specs)
