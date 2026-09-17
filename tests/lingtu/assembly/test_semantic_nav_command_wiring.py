"""Focused semantic nav command wire tests."""

from __future__ import annotations

from lingtu.assembly.wires.context import WiringContext
from lingtu.assembly.wires.gateway import gateway_status_specs
from lingtu.assembly.wires.semantic import semantic_command_specs
from runtime.wiring import wire_key


def _ctx(*names: str) -> WiringContext:
    return WiringContext(
        names=frozenset(names),
        driver_module="Driver",
        slam_module="",
        camera_src="Driver",
        color_out="camera_image",
        nav_odom_src="Driver",
    )


def test_semantic_nav_command_wires_to_goal_service_when_present() -> None:
    specs = semantic_command_specs(
        _ctx("host.bus", "GatewayModule", "MCPServerModule", "SemanticPlannerModule", "nav.goals")
    )

    keys = {wire_key(spec) for spec in specs}
    assert ("SemanticPlannerModule", "nav_command", "nav.goals", "goal_command") in keys
    assert ("host.bus", "navigation_state", "SemanticPlannerModule", "navigation_state") in keys
    assert ("nav.goals", "goal_status", "SemanticPlannerModule", "goal_status") in keys
    assert ("nav.goals", "task_status", "SemanticPlannerModule", "navigation_goal_status") in keys
    assert ("SemanticPlannerModule", "goal_pose", "nav.goals", "goal_request") not in keys
    assert ("PerceptionModule", "robot_pose", "SemanticPlannerModule", "robot_pose") in keys
    assert ("PerceptionModule", "robot_pose", "AgentPlannerModule", "robot_pose") in keys
    assert ("PerceptionModule", "observation_image", "SemanticPlannerModule", "observation_image") in keys
    assert ("AgentPlannerModule", "goal_pose", "nav.goals", "goal_request") not in keys


def test_mcp_uses_gateway_status_projection_but_keeps_goal_events() -> None:
    keys = {
        wire_key(spec)
        for spec in gateway_status_specs(_ctx("host.bus", "GatewayModule", "MCPServerModule"))
    }

    assert ("host.bus", "navigation_state", "MCPServerModule", "navigation_state") not in keys
    assert (
        "host.bus",
        "navigation_goal_status",
        "MCPServerModule",
        "navigation_goal_status",
    ) in keys


def test_semantic_planner_symbolic_llm_wires_when_llm_module_present() -> None:
    specs = semantic_command_specs(
        _ctx("GatewayModule", "MCPServerModule", "SemanticPlannerModule", "nav.goals", "LLMModule")
    )
    keys = {wire_key(spec) for spec in specs}

    assert (
        "SemanticPlannerModule",
        "llm_request",
        "LLMModule",
        "request",
    ) in keys
    assert (
        "LLMModule",
        "response",
        "SemanticPlannerModule",
        "llm_response",
    ) in keys


def test_semantic_planner_symbolic_llm_wires_are_conditional() -> None:
    specs = semantic_command_specs(_ctx("GatewayModule", "MCPServerModule", "SemanticPlannerModule", "nav.goals"))
    keys = {wire_key(spec) for spec in specs}

    assert (
        "SemanticPlannerModule",
        "llm_request",
        "LLMModule",
        "request",
    ) not in keys
