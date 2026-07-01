"""Semantic perception, memory, planner, and visual-servo wires."""

from __future__ import annotations

from .context import RECON_RECORDERS, SEMANTIC_CAMERA_CONSUMERS, WiringContext
from .types import WireSpec


def semantic_camera_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs: list[WireSpec] = []
    for consumer in SEMANTIC_CAMERA_CONSUMERS:
        specs.extend([
            WireSpec(ctx.camera_src, ctx.color_out, consumer, "color_image"),
            WireSpec(ctx.camera_src, "depth_image", consumer, "depth_image"),
            WireSpec(ctx.camera_src, "camera_info", consumer, "camera_info"),
        ])
    return tuple(specs)


def semantic_command_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("GatewayModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("EndpointCommandBridgeModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("SemanticPlannerModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections"),
    )


def semantic_scene_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("PerceptionModule", "scene_graph", "GatewayModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "MCPServerModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "ReconstructionModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "TraversableFrontierModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "SemanticMapperModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "EpisodicMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "VectorMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "TemporalMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "VisualServoModule", "scene_graph"),
        WireSpec("SemanticMapperModule", "topo_summary", "SemanticPlannerModule", "topo_summary"),
        WireSpec("SemanticMapperModule", "room_graph", "SemanticPlannerModule", "room_graph"),
    )


def recorder_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    specs: list[WireSpec] = []
    for recorder in RECON_RECORDERS:
        specs.extend([
            WireSpec(ctx.camera_src, ctx.color_out, recorder, "color_image"),
            WireSpec(ctx.camera_src, "depth_image", recorder, "depth_image"),
            WireSpec(ctx.camera_src, "camera_info", recorder, "camera_info"),
            WireSpec(ctx.nav_odom_src, "odometry", recorder, "odometry"),
        ])
    return tuple(specs)


def visual_servo_specs() -> tuple[WireSpec, ...]:
    return (
        WireSpec("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target"),
        WireSpec("VisualServoModule", "goal_pose", "NavigationModule", "goal_pose"),
        WireSpec("VisualServoModule", "nav_stop", "NavigationModule", "stop_signal"),
        WireSpec("VisualServoModule", "cmd_vel", "CmdVelMux", "visual_servo_cmd_vel"),
    )
