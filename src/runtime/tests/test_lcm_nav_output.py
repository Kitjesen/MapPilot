from __future__ import annotations

import json

from runtime.adapters.dds.nav import DDSNavOutModule
from runtime.adapters.lcm.contracts import THUNDER_FIELD_LCM_CONTRACT_NAME, binding_for_topic
from runtime.adapters.lcm.endpoint_codec import loads_endpoint_message
from runtime.adapters.lcm.nav_output import LCMNavOutModule
from runtime.blueprints.profile_builder import blueprint_for_resolved_profile
from runtime.blueprints.stacks.navigation import navigation
from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Path
from runtime.profiles.resolver import resolve_profile_config
from runtime.runtime_interface import TOPICS

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


def _envelope_by_topic(transport: _FakeLCMTransport, topic: str) -> dict:
    binding = binding_for_topic(THUNDER_FIELD_LCM_CONTRACT_NAME, topic)
    for channel, payload in reversed(transport.published):
        if channel == binding.channel:
            return json.loads(payload)
    raise AssertionError(f"missing payload for {topic}")


def test_lcm_nav_out_publishes_paths_and_muxed_cmd_vel() -> None:
    transport = _FakeLCMTransport()
    nav_out = LCMNavOutModule(transport=transport)
    nav_out.setup()

    nav_out.global_path._deliver([(1.0, 2.0, 0.0), {"x": 3.0, "y": 4.0}])
    nav_out.local_path._deliver(
        Path(
            poses=[PoseStamped(Pose(5.0, 6.0, 0.0), frame_id="map")],
            frame_id="map",
        )
    )
    nav_out.waypoint._deliver(PoseStamped(Pose(7.0, 8.0, 0.0), frame_id="map"))
    nav_out.cmd_vel._deliver(Twist(Vector3(0.3, 0.0, 0.0), Vector3(0.0, 0.0, 0.2)))

    global_path = _decoded_by_topic(transport, TOPICS.global_path)
    local_path = _decoded_by_topic(transport, TOPICS.local_path)
    waypoint = _decoded_by_topic(transport, TOPICS.nav_way_point)
    cmd_vel = _decoded_by_topic(transport, TOPICS.cmd_vel)
    cmd_vel_envelope = _envelope_by_topic(transport, TOPICS.cmd_vel)
    global_path_envelope = _envelope_by_topic(transport, TOPICS.global_path)

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
    assert global_path_envelope["schema_version"] == 1
    assert global_path_envelope["schema"] == "lingtu.nav.path.v1"
    assert global_path_envelope["frame_id"] == "map"
    assert isinstance(global_path_envelope["ts"], float)
    assert cmd_vel_envelope["schema"] == "lingtu.geometry.twist.v1"
    assert cmd_vel_envelope["frame_id"] == "body"
    health = nav_out.health()
    assert health["configured_backend"] == "lcm_nav_output"
    assert health["backend"] == "lcm_nav_output"
    assert health["publish_counts"] == {
        TOPICS.global_path: 1,
        TOPICS.local_path: 1,
        TOPICS.nav_way_point: 1,
        TOPICS.cmd_vel: 1,
    }

    nav_out.stop()
    assert transport.closed is False


def test_navigation_stack_selects_lcm_nav_out_for_lcm_endpoint() -> None:
    bp = navigation(
        enable_nav_out=True,
        nav_out_adapter="lcm_endpoint",
        _endpoint_transport="lcm",
        _endpoint_contract=THUNDER_FIELD_LCM_CONTRACT_NAME,
        enable_native=False,
        planning_frame_id="map",
    )
    nav_out_entry = next(entry for entry in bp._entries if entry.name == "nav.out")
    wires = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    }

    assert nav_out_entry.module_cls is LCMNavOutModule
    assert nav_out_entry.config == {
        "default_frame_id": "map",
        "endpoint_contract": THUNDER_FIELD_LCM_CONTRACT_NAME,
    }
    assert "nav.mission.global_path->nav.out.global_path" in wires
    assert "nav.local_planner.local_path->nav.out.local_path" in wires
    assert "nav.mission.waypoint->nav.out.waypoint" in wires


def test_thunder_field_nav_blueprint_publishes_path_and_cmd_vel_to_dds_endpoint() -> None:
    config = resolve_profile_config("nav")
    bp = blueprint_for_resolved_profile("nav", config)
    nav_out_entry = next(entry for entry in bp._entries if entry.name == "nav.out")
    wire_labels = [
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}"
        for wire in bp._wires
    ]
    wires = {
        label
        for label in wire_labels
    }
    topics = {
        f"{wire.out_module}.{wire.out_port}->{wire.in_module}.{wire.in_port}": wire.topic
        for wire in bp._wires
    }

    assert config["nav_out_adapter"] == "dds_nav_output"
    assert config["enable_robot_driver"] is False
    assert all(entry.name != "ThunderDriver" for entry in bp._entries)
    assert config["enable_nav_out"] is True
    assert nav_out_entry.module_cls is DDSNavOutModule
    assert "nav.mission.global_path->nav.out.global_path" in wires
    assert "nav.local_planner.local_path->nav.out.local_path" in wires
    assert "nav.mission.waypoint->nav.out.waypoint" in wires
    assert "nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel" in wires
    assert wire_labels.count("nav.mission.waypoint->nav.out.waypoint") == 1
    assert topics["nav.mission.global_path->nav.out.global_path"] == TOPICS.global_path
    assert topics["nav.local_planner.local_path->nav.out.local_path"] == TOPICS.local_path
    assert topics["nav.mission.waypoint->nav.out.waypoint"] == TOPICS.nav_way_point
    assert topics["nav.velocity_mux.driver_cmd_vel->nav.out.cmd_vel"] == TOPICS.cmd_vel
    assert "nav.velocity_mux.driver_cmd_vel->ThunderDriver.cmd_vel" not in wires
