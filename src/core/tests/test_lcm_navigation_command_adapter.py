from __future__ import annotations

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from compat.lcm.endpoint_codec import dumps_endpoint_message
from compat.lcm.navigation_command_adapter import LCMNavigationCommandBridgeModule
from core.blueprints.profile_builder import blueprint_for_resolved_profile
from core.blueprints.profile_graph import graph_for_profile
from core.blueprints.stacks.navigation import navigation
from core.msgs.geometry import Pose, PoseStamped
from core.runtime.resolver import resolve_profile_config
from core.runtime_interface import TOPICS


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


def test_lcm_navigation_command_bridge_publishes_goal_cancel_and_instruction() -> None:
    transport = _FakeLCMTransport()
    bridge = LCMNavigationCommandBridgeModule(transport=transport)
    goals = []
    cancels = []
    instructions = []
    bridge.goal_pose.subscribe(goals.append)
    bridge.cancel.subscribe(cancels.append)
    bridge.instruction.subscribe(instructions.append)
    bridge.setup()

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
    assert bridge.health()["message_counts"] == {
        TOPICS.goal_pose: 1,
        TOPICS.cancel: 1,
        TOPICS.semantic_instruction: 1,
    }

    bridge.stop()
    assert transport.closed is False


def test_navigation_stack_selects_lcm_command_bridge_for_lcm_endpoint() -> None:
    bp = navigation(
        enable_endpoint_command_bridge=True,
        endpoint_ingress_adapter="lcm_endpoint",
        _endpoint_transport="lcm",
        _endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
        enable_native=False,
        planning_frame_id="map",
    )
    bridge_entry = next(
        entry for entry in bp._entries if entry.name == "EndpointCommandBridgeModule"
    )
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert bridge_entry.module_cls is LCMNavigationCommandBridgeModule
    assert bridge_entry.config == {
        "default_frame_id": "map",
        "endpoint_contract": THUNDER_FIELD_LCM_CONTRACT_NAME,
    }
    assert "EndpointCommandBridgeModule.goal_pose->NavigationModule.goal_pose" in wires
    assert "EndpointCommandBridgeModule.cancel->NavigationModule.cancel" in wires
    assert "EndpointCommandBridgeModule.instruction->NavigationModule.instruction" in wires


def test_thunder_field_nav_blueprint_accepts_commands_from_lcm_endpoint() -> None:
    config = resolve_profile_config("nav")
    bp = blueprint_for_resolved_profile("nav", config)
    bridge_entry = next(
        entry for entry in bp._entries if entry.name == "EndpointCommandBridgeModule"
    )
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert config["endpoint_ingress_adapter"] == "lcm_endpoint"
    assert config["enable_endpoint_command_bridge"] is True
    assert bridge_entry.module_cls is LCMNavigationCommandBridgeModule
    assert bridge_entry.config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert "EndpointCommandBridgeModule.goal_pose->NavigationModule.goal_pose" in wires
    assert "EndpointCommandBridgeModule.cancel->NavigationModule.cancel" in wires
    assert "EndpointCommandBridgeModule.instruction->NavigationModule.instruction" in wires
    assert "EndpointCommandBridgeModule.instruction->SemanticPlannerModule.instruction" in wires


def test_static_thunder_nav_graph_includes_lcm_command_ingress() -> None:
    graph = graph_for_profile("nav")
    wires = {wire.as_snapshot() for wire in graph.explicit_wires}

    assert "EndpointCommandBridgeModule" in graph.modules
    assert "EndpointWaypointBridgeModule" not in graph.modules
    assert "EndpointCommandBridgeModule.goal_pose->NavigationModule.goal_pose" in wires
    assert "EndpointCommandBridgeModule.cancel->NavigationModule.cancel" in wires
    assert "EndpointCommandBridgeModule.instruction->NavigationModule.instruction" in wires
    assert "EndpointCommandBridgeModule.instruction->SemanticPlannerModule.instruction" in wires
