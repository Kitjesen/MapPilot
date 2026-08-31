"""Semantic perception, memory, planner, and visual-servo wires."""

from __future__ import annotations

from runtime.wiring import WireSpec

from .context import (
    RECON_RECORDERS,
    SEMANTIC_CAMERA_CONSUMERS,
    WiringContext,
)


def semantic_camera_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Wire camera outputs to semantic consumers."""
    specs: list[WireSpec] = []
    for consumer in SEMANTIC_CAMERA_CONSUMERS:
        specs.extend(
            [
                WireSpec(ctx.camera_src, ctx.color_out, consumer, "color_image"),
                WireSpec(ctx.camera_src, "depth_image", consumer, "depth_image"),
                WireSpec(ctx.camera_src, "camera_info", consumer, "camera_info"),
            ]
        )
    return tuple(specs)


def semantic_command_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Wire commands and planner outputs for semantic planning."""
    specs = [
        WireSpec("GatewayModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("MCPServerModule", "instruction", "SemanticPlannerModule", "instruction"),
        WireSpec("host.bus", "navigation_state", "SemanticPlannerModule", "navigation_state"),
        WireSpec("SemanticPlannerModule", "goal_pose", "nav.goals", "goal_request"),
        WireSpec("AgentPlannerModule", "goal_pose", "nav.goals", "goal_request"),
        WireSpec("PerceptionModule", "detections_3d", "SemanticPlannerModule", "detections"),
        WireSpec("PerceptionModule", "detections_3d", "InspectionEvidenceModule", "detections_3d"),
    ]
    specs.append(WireSpec("SemanticPlannerModule", "nav_command", "nav.goals", "goal_command"))
    if "LLMModule" in ctx.names:
        specs.extend(
            [
                WireSpec("SemanticPlannerModule", "llm_request", "LLMModule", "request"),
                WireSpec("LLMModule", "response", "SemanticPlannerModule", "llm_response"),
            ]
        )
    return tuple(specs)


def semantic_scene_specs() -> tuple[WireSpec, ...]:
    """Wire scene graph outputs to semantic consumers."""
    return (
        WireSpec("PerceptionModule", "scene_graph", "GatewayModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "MCPServerModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "ReconstructionModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "SemanticMapperModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "EpisodicMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "VectorMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "TemporalMemoryModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "SemanticPlannerModule", "scene_graph"),
        WireSpec("PerceptionModule", "scene_graph", "AgentPlannerModule", "scene_graph"),
        WireSpec("SemanticMapperModule", "topo_summary", "SemanticPlannerModule", "topo_summary"),
        WireSpec("SemanticMapperModule", "room_graph", "SemanticPlannerModule", "room_graph"),
    )


def recorder_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Wire sensor feeds to reconstruction recorders."""
    specs: list[WireSpec] = []
    for recorder in RECON_RECORDERS:
        specs.extend(
            [
                WireSpec(ctx.camera_src, ctx.color_out, recorder, "color_image"),
                WireSpec(ctx.camera_src, "depth_image", recorder, "depth_image"),
                WireSpec(ctx.camera_src, "camera_info", recorder, "camera_info"),
                WireSpec(ctx.nav_odom_src, "odometry", recorder, "odometry"),
            ]
        )
    return tuple(specs)


def visual_servo_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """Wire visual servo inputs and outputs."""
    specs = [
        WireSpec("PerceptionModule", "detections_3d", "VisualServoModule", "detections_3d"),
        WireSpec("PerceptionModule", "robot_pose", "VisualServoModule", "robot_pose"),
        WireSpec(ctx.camera_src, ctx.color_out, "VisualServoModule", "color_image"),
        WireSpec("GatewayModule", "servo_target", "VisualServoModule", "servo_target"),
        WireSpec("SemanticPlannerModule", "servo_target", "VisualServoModule", "servo_target"),
        WireSpec("AgentPlannerModule", "servo_target", "VisualServoModule", "servo_target"),
        WireSpec("VisualServoModule", "goal_pose", "nav.goals", "visual_goal_request"),
        WireSpec("VisualServoModule", "goal_cancel", "nav.goals", "visual_cancel_request"),
        WireSpec("nav.goals", "goal_status", "VisualServoModule", "goal_status"),
        WireSpec("VisualServoModule", "servo_status", "GatewayModule", "visual_servo_status"),
    ]
    return tuple(specs)


def vla_specs(ctx: WiringContext) -> tuple[WireSpec, ...]:
    """VLA module input/output wiring."""
    return (
        WireSpec(ctx.camera_src, ctx.color_out, "VLAModule", "color_image"),
        WireSpec(ctx.camera_src, "depth_image", "VLAModule", "depth_image"),
        WireSpec(ctx.camera_src, "camera_info", "VLAModule", "camera_info"),
        WireSpec(ctx.nav_odom_src, "odometry", "VLAModule", "odometry"),
        WireSpec("PerceptionModule", "scene_graph", "VLAModule", "scene_graph"),
        WireSpec("GatewayModule", "instruction", "VLAModule", "instruction"),
        WireSpec("MCPServerModule", "instruction", "VLAModule", "instruction"),
        WireSpec("VLAModule", "goal_pose", "nav.goals", "goal_request"),
    )
