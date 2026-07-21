"""Focused semantic nav command wire tests."""

from __future__ import annotations

from lingtu.assembly.wires.context import WiringContext
from lingtu.assembly.wires.semantic import semantic_command_specs
from runtime.wiring import wire_key


def _ctx(*names: str) -> WiringContext:
    return WiringContext(
        names=frozenset(names),
        robot="stub",
        driver_module="Driver",
        slam_profile="none",
        slam_module="",
        scene_xml="",
        enable_semantic=True,
        camera_src="Driver",
        color_out="camera_image",
        nav_odom_src="Driver",
    )


def test_semantic_nav_command_wires_to_goal_service_when_present() -> None:
    specs = semantic_command_specs(_ctx("GatewayModule", "MCPServerModule", "SemanticPlannerModule", "nav.goals"))

    assert (
        "SemanticPlannerModule",
        "nav_command",
        "nav.goals",
        "goal_command",
    ) in {wire_key(spec) for spec in specs}


def test_semantic_nav_command_does_not_wire_to_nav_mission_without_goal_service() -> None:
    specs = semantic_command_specs(_ctx("GatewayModule", "MCPServerModule", "SemanticPlannerModule", "nav.mission"))
    keys = {wire_key(spec) for spec in specs}

    assert (
        "SemanticPlannerModule",
        "nav_command",
        "nav.goals",
        "goal_command",
    ) not in keys


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
    assert (
        "SemanticPlannerModule",
        "nav_command",
        "nav.mission",
        "goal_command",
    ) not in keys
