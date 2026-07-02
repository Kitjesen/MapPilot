from __future__ import annotations

import pytest

from runtime.adapters.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from runtime.adapters.lcm.endpoint_codec import dumps_endpoint_message
from runtime.adapters.lcm.nav_input import LCMNavInModule
from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
from runtime.blueprints.profile_graph import graph_for_profile
from runtime.blueprints.stacks.navigation import navigation
from runtime.msgs.geometry import Pose, PoseStamped
from runtime.profiles.resolver import resolve_profile_config
from runtime.runtime_interface import TOPICS

class _FakeLCMTransport:
    def __init__(self) -> None:
        self.callbacks = {}
        self.closed = False

    def subscribe(self, channel, callback):
        self.callbacks[channel] = callback
        return _FakeSubscription(channel)

    def emit(self, channel, payload) -> None:
        self.callbacks[channel](payload)

    def close(self) -> None:
        self.closed = True


class _FakeSubscription:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.closed = False

    def close(self) -> None:
        self.closed = True


def test_lcm_nav_in_publishes_goal_cancel_and_instruction() -> None:
    transport = _FakeLCMTransport()
    nav_in = LCMNavInModule(transport=transport)
    goals = []
    cancels = []
    instructions = []
    nav_in.goal_pose.subscribe(goals.append)
    nav_in.cancel.subscribe(cancels.append)
    nav_in.instruction.subscribe(instructions.append)
    nav_in.setup()

    goal_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.goal_pose)
    cancel_binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, TOPICS.cancel)
    instruction_binding = binding_for_topic(
        THUNDER_FIELD_LCM_CONTRACT_NAME,
        TOPICS.semantic_instruction,
    )

    transport.emit(
        goal_binding.channel,
        dumps_endpoint_message(
            goal_binding,
            PoseStamped(Pose(1.0, 2.0, 0.0), frame_id="map"),
        ),
    )
    transport.emit(
        cancel_binding.channel,
        dumps_endpoint_message(cancel_binding, "operator_cancel"),
    )
    transport.emit(
        instruction_binding.channel,
        dumps_endpoint_message(instruction_binding, "go to the loading dock"),
    )

    assert goals[-1].x == 1.0
    assert goals[-1].y == 2.0
    assert goals[-1].frame_id == "map"
    assert cancels == ["operator_cancel"]
    assert instructions == ["go to the loading dock"]
    health = nav_in.health()
    assert health["configured_backend"] == "lcm_nav_input"
    assert health["backend"] == "lcm_nav_input"
    assert health["message_counts"] == {
        TOPICS.goal_pose: 1,
        TOPICS.cancel: 1,
        TOPICS.semantic_instruction: 1,
    }

    nav_in.stop()
    assert transport.closed is False


def test_navigation_stack_selects_lcm_nav_in_for_lcm_endpoint() -> None:
    bp = navigation(
        enable_nav_in=True,
        nav_in_adapter="lcm_endpoint",
        _endpoint_transport="lcm",
        _endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
        enable_native=False,
        planning_frame_id="map",
    )
    nav_in_entry = next(
        entry for entry in bp._entries if entry.name == "nav.in"
    )
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }
    topics = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}": wire.topic
        for wire in bp._wires
    }

    assert nav_in_entry.module_cls is LCMNavInModule
    assert nav_in_entry.config == {
        "default_frame_id": "map",
        "endpoint_contract": THUNDER_FIELD_LCM_CONTRACT_NAME,
    }
    assert "nav.in.goal_pose->nav.mission.goal_pose" in wires
    assert "nav.in.cancel->nav.mission.cancel" in wires
    assert "nav.in.instruction->nav.mission.instruction" in wires
    assert topics["nav.in.goal_pose->nav.mission.goal_pose"] == TOPICS.goal_pose
    assert topics["nav.in.cancel->nav.mission.cancel"] == TOPICS.cancel
    assert (
        topics["nav.in.instruction->nav.mission.instruction"]
        == TOPICS.semantic_instruction
    )


def test_navigation_stack_selects_ros2_nav_in_for_explicit_ros2_ingress() -> None:
    from nav.adapters.ros2.nav.nav_in import ROS2NavInModule

    bp = navigation(
        enable_nav_in=True,
        nav_in_adapter="ros2_nav_input",
        enable_native=False,
        planning_frame_id="map",
    )
    nav_in_entry = next(
        entry for entry in bp._entries if entry.name == "nav.in"
    )
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert nav_in_entry.module_cls is ROS2NavInModule
    assert nav_in_entry.config == {"default_frame_id": "map"}
    assert "nav.in.goal_pose->nav.mission.goal_pose" in wires
    assert "nav.in.cancel->nav.mission.cancel" in wires
    assert "nav.in.instruction->nav.mission.instruction" in wires


def test_thunder_field_nav_blueprint_accepts_commands_from_lcm_endpoint() -> None:
    # The "nav" profile defaults to DDS ingress (see
    # docs/architecture/LINGTU_RUNTIME_BUS_DECISION.md); LCM remains an
    # explicit opt-in for smoke/replay bridges, so force it here.
    config = resolve_profile_config("nav", nav_in_adapter="lcm_nav_input")
    bp = blueprint_for_resolved_profile("nav", config)
    nav_in_entry = next(
        entry for entry in bp._entries if entry.name == "nav.in"
    )
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }
    topics = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}": wire.topic
        for wire in bp._wires
    }

    assert config["nav_in_adapter"] == "lcm_nav_input"
    assert config["enable_nav_in"] is True
    assert nav_in_entry.module_cls is LCMNavInModule
    assert "nav.in.goal_pose->nav.mission.goal_pose" in wires
    assert "nav.in.cancel->nav.mission.cancel" in wires
    assert "nav.in.instruction->nav.mission.instruction" in wires
    assert "nav.in.instruction->SemanticPlannerModule.instruction" in wires
    assert topics["nav.in.goal_pose->nav.mission.goal_pose"] == TOPICS.goal_pose
    assert topics["nav.in.cancel->nav.mission.cancel"] == TOPICS.cancel
    assert (
        topics["nav.in.instruction->nav.mission.instruction"]
        == TOPICS.semantic_instruction
    )
    assert (
        topics["nav.in.instruction->SemanticPlannerModule.instruction"]
        == TOPICS.semantic_instruction
    )


def test_static_thunder_nav_graph_includes_endpoint_command_ingress() -> None:
    graph = graph_for_profile("nav")
    wires = {wire.as_snapshot() for wire in graph.explicit_wires}

    assert "nav.in" in graph.modules
    assert (
        f"nav.in.goal_pose->nav.mission.goal_pose@{TOPICS.goal_pose}"
        in wires
    )
    assert f"nav.in.cancel->nav.mission.cancel@{TOPICS.cancel}" in wires
    assert (
        "nav.in.instruction->nav.mission.instruction"
        f"@{TOPICS.semantic_instruction}"
        in wires
    )
    assert (
        "nav.in.instruction->SemanticPlannerModule.instruction"
        f"@{TOPICS.semantic_instruction}"
        in wires
    )
