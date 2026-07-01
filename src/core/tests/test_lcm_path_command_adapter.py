from __future__ import annotations

from compat.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from compat.lcm.endpoint_codec import loads_endpoint_message
from compat.lcm.path_command_adapter import LCMPathCommandBridgeModule
from core.blueprints.profile_builder import blueprint_for_resolved_profile
from core.blueprints.stacks.navigation import navigation
from core.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from core.msgs.nav import Path
from core.runtime.resolver import resolve_profile_config
from core.runtime_interface import TOPICS


class _FakeLCMTransport:
    def __init__(self) -> None:
        self.published: list[tuple[str, bytes]] = []
        self.closed = False

    def publish(self, channel: str, payload: bytes) -> None:
        self.published.append((channel, payload))

    def close(self) -> None:
        self.closed = True


def _decoded_by_topic(transport: _FakeLCMTransport, topic: str):
    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, topic)
    for channel, payload in reversed(transport.published):
        if channel == binding.channel:
            return loads_endpoint_message(binding, payload)
    raise AssertionError(f"missing payload for {topic}")


def test_lcm_path_command_bridge_publishes_paths_and_muxed_cmd_vel() -> None:
    transport = _FakeLCMTransport()
    bridge = LCMPathCommandBridgeModule(transport=transport)
    bridge.setup()

    bridge.global_path._deliver([(1.0, 2.0, 0.0), {"x": 3.0, "y": 4.0}])
    bridge.local_path._deliver(
        Path(
            poses=[PoseStamped(Pose(5.0, 6.0, 0.0), frame_id="map")],
            frame_id="map",
        )
    )
    bridge.waypoint._deliver(PoseStamped(Pose(7.0, 8.0, 0.0), frame_id="map"))
    bridge.cmd_vel._deliver(Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)))

    global_path = _decoded_by_topic(transport, TOPICS.global_path)
    local_path = _decoded_by_topic(transport, TOPICS.local_path)
    waypoint = _decoded_by_topic(transport, TOPICS.nav_way_point)
    cmd_vel = _decoded_by_topic(transport, TOPICS.cmd_vel)

    assert isinstance(global_path, Path)
    assert [pose.x for pose in global_path.poses] == [1.0, 3.0]
    assert global_path.frame_id == "map"
    assert isinstance(local_path, Path)
    assert local_path.poses[0].y == 6.0
    assert isinstance(waypoint, PoseStamped)
    assert waypoint.x == 7.0
    assert waypoint.y == 8.0
    assert waypoint.frame_id == "map"
    assert isinstance(cmd_vel, Twist)
    assert cmd_vel.linear.x == 0.3
    assert cmd_vel.angular.z == 0.2
    assert bridge.health()["publish_counts"] == {
        TOPICS.global_path: 1,
        TOPICS.local_path: 1,
        TOPICS.nav_way_point: 1,
        TOPICS.cmd_vel: 1,
    }

    bridge.stop()
    assert transport.closed is False


def test_navigation_stack_selects_lcm_path_command_bridge_for_lcm_endpoint() -> None:
    bp = navigation(
        enable_endpoint_path_bridge=True,
        endpoint_egress_adapter="lcm_endpoint",
        _endpoint_transport="lcm",
        _endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
        enable_native=False,
        planning_frame_id="map",
    )
    bridge_entry = next(entry for entry in bp._entries if entry.name == "EndpointPathBridgeModule")
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert bridge_entry.module_cls is LCMPathCommandBridgeModule
    assert bridge_entry.config == {
        "default_frame_id": "map",
        "endpoint_contract": THUNDER_FIELD_LCM_CONTRACT_NAME,
    }
    assert "NavigationModule.waypoint->EndpointPathBridgeModule.waypoint" in wires


def test_thunder_field_nav_blueprint_publishes_path_and_cmd_vel_to_lcm_endpoint() -> None:
    config = resolve_profile_config("nav")
    bp = blueprint_for_resolved_profile("nav", config)
    bridge_entry = next(entry for entry in bp._entries if entry.name == "EndpointPathBridgeModule")
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert config["endpoint_egress_adapter"] == "lcm_endpoint"
    assert config["enable_robot_driver"] is False
    assert all(entry.name != "ThunderDriver" for entry in bp._entries)
    assert config["enable_endpoint_path_bridge"] is True
    assert all(entry.name != "EndpointWaypointBridgeModule" for entry in bp._entries)
    assert bridge_entry.module_cls is LCMPathCommandBridgeModule
    assert bridge_entry.config["endpoint_contract"] == THUNDER_FIELD_LCM_CONTRACT_NAME
    assert "NavigationModule.global_path->EndpointPathBridgeModule.global_path" in wires
    assert "LocalPlannerModule.local_path->EndpointPathBridgeModule.local_path" in wires
    assert "NavigationModule.waypoint->EndpointPathBridgeModule.waypoint" in wires
    assert "CmdVelMux.driver_cmd_vel->EndpointPathBridgeModule.cmd_vel" in wires
    assert "CmdVelMux.driver_cmd_vel->ThunderDriver.cmd_vel" not in wires
