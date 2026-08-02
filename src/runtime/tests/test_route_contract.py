from __future__ import annotations

from dataclasses import replace

import pytest

from runtime.blueprint import Blueprint
from runtime.module import Module
from runtime.route_contract import (
    RouteContract,
    RouteSpec,
    load_route_contract,
    render_route_mermaid,
    validate_route_contract,
)
from runtime.routes import replay, robot, sim
from runtime.runtime_interface import TOPICS
from runtime.stream import In, Out


class RouteModule(Module):
    pass


class CmdProducer(Module):
    cmd_vel: Out[object]


class CmdConsumer(Module):
    cmd_vel: In[object]


class FakeRouteTransport:
    name = "dds"

    def __init__(self) -> None:
        self.subscriptions: list[tuple[str, object]] = []

    def publish(self, _topic: str, _msg: object) -> None:
        pass

    def subscribe(self, topic: str, cb) -> None:
        self.subscriptions.append((topic, cb))

    def close(self) -> None:
        pass


def test_robot_route_validates_against_typed_dds_contract() -> None:
    contract = load_route_contract(robot())

    assert validate_route_contract(contract) == []
    assert contract.route.endpoint_contract == "thunder_dds_v1"
    assert contract.route_for(TOPICS.lidar_scan) == "dds"
    assert contract.route_for(TOPICS.goal_pose) == "dds"
    assert contract.route_for(TOPICS.cancel) == "dds"
    assert contract.route_for(TOPICS.semantic_instruction) == "local"
    assert contract.route_for(TOPICS.nav_way_point) == "dds"
    assert contract.route_for(TOPICS.cmd_vel) == "dds"
    assert contract.binding_for(TOPICS.cmd_vel)["qos"] == "command"


def test_robot_route_matches_thunder_dds_endpoint_topics() -> None:
    from runtime.endpoints.dds.contracts import THUNDER_DDS_CONTRACT

    contract = load_route_contract(robot())

    assert set(contract.route.routes) == set(THUNDER_DDS_CONTRACT.topics)
    assert validate_route_contract(contract) == []


def test_robot_route_includes_native_exploration_dds_boundaries() -> None:
    contract = load_route_contract(robot())
    expected_qos = {
        TOPICS.exploration_grid: "state",
        TOPICS.exploration_snapshot: "state",
        TOPICS.exploration_execution_snapshot: "state",
        TOPICS.exploration_segment_request: "command",
        TOPICS.exploration_segment_ack: "event",
        TOPICS.exploration_segment_status: "event",
    }

    for topic, qos in expected_qos.items():
        assert contract.route_for(topic) == "dds"
        assert contract.binding_for(topic)["qos"] == qos


def test_robot_route_exposes_module_and_endpoint_port_bindings() -> None:
    contract = load_route_contract(robot())
    odometry_bindings = {
        (binding.owner, binding.port, binding.direction, binding.boundary)
        for binding in contract.topic(TOPICS.odometry).port_bindings
    }
    cmd_vel_bindings = {
        (binding.owner, binding.port, binding.direction, binding.boundary)
        for binding in contract.topic(TOPICS.cmd_vel).port_bindings
    }

    assert ("SlamAdapterModule", "odometry", "out", "module") in odometry_bindings
    assert ("nav.mission", "odometry", "in", "module") in odometry_bindings
    assert ("native_slam_runtime", "odometry", "out", "endpoint") in odometry_bindings
    assert ("native_nav_endpoint", "cmd_vel", "out", "native") in cmd_vel_bindings
    assert ("nav.velocity_mux", "driver_cmd_vel", "out", "module") in cmd_vel_bindings
    assert ("driver", "cmd_vel", "in", "native") in cmd_vel_bindings


def test_replay_route_is_protocol_neutral_and_valid() -> None:
    contract = load_route_contract(replay())

    assert validate_route_contract(contract) == []
    assert contract.route_for(TOPICS.odometry) == "lcm"
    assert contract.binding_for(TOPICS.odometry)["channel"] == "LT_SLAM_ODOMETRY"
    assert contract.route_for(TOPICS.global_path) == "local"


def test_route_rejects_dds_topic_without_typed_spec() -> None:
    contract = load_route_contract(robot())
    route = RouteSpec(
        name="bad",
        default="local",
        routes={**dict(contract.route.routes), "/debug/not_registered": "dds"},
        bindings=contract.route.bindings,
    )
    broken = RouteContract(
        name=contract.name,
        topics={
            **dict(contract.topics),
            "/debug/not_registered": replace(
                contract.topic(TOPICS.cmd_vel),
                topic="/debug/not_registered",
                role="debug",
            ),
        },
        route=route,
        native_contract_topics=contract.native_contract_topics,
    )

    issues = validate_route_contract(broken)

    assert any(issue.code == "dds_topic_spec_missing" for issue in issues)


def test_route_rejects_dds_topic_without_port_bindings() -> None:
    contract = load_route_contract(robot())
    broken = RouteContract(
        name=contract.name,
        topics={
            **dict(contract.topics),
            TOPICS.cmd_vel: replace(
                contract.topic(TOPICS.cmd_vel),
                port_bindings=(),
            ),
        },
        route=contract.route,
        native_contract_topics=contract.native_contract_topics,
    )

    issues = validate_route_contract(broken)

    assert any(issue.code == "dds_port_bindings_missing" for issue in issues)


def test_route_allows_explicit_external_status_without_in_repo_consumer() -> None:
    contract = load_route_contract(robot())
    status = contract.topic(TOPICS.operator_motion_status)

    assert status.consumers == ()
    assert status.external_diagnostics_subscribable is True
    assert not any(
        issue.code == "topic_consumers_missing" and issue.scope == TOPICS.operator_motion_status
        for issue in validate_route_contract(contract)
    )


def test_route_registers_host_bus_as_the_task_event_consumer() -> None:
    contract = load_route_contract(robot())
    event = contract.topic(TOPICS.inspection_task_event)

    assert event.consumers == ("host_bus",)
    assert event.external_diagnostics_subscribable is True
    assert not any(
        issue.code == "topic_consumers_missing" and issue.scope == TOPICS.inspection_task_event
        for issue in validate_route_contract(contract)
    )


def test_external_diagnostics_flag_cannot_hide_consumerless_control_topic() -> None:
    contract = load_route_contract(robot())
    control = contract.topic(TOPICS.operator_motion_control)
    broken = RouteContract(
        name=contract.name,
        topics={
            **dict(contract.topics),
            TOPICS.operator_motion_control: replace(
                control,
                consumers=(),
                external_diagnostics_subscribable=True,
            ),
        },
        route=contract.route,
        native_contract_topics=contract.native_contract_topics,
    )

    issues = validate_route_contract(broken)

    assert any(
        issue.code == "topic_consumers_missing" and issue.scope == TOPICS.operator_motion_control
        for issue in issues
    )


def test_route_mermaid_renders_topic_ownership_and_route() -> None:
    mermaid = render_route_mermaid(load_route_contract(robot()))

    assert mermaid.startswith("flowchart LR")
    assert "/lidar/raw_frame" in mermaid
    assert "route=dds" in mermaid
    assert "native_slam_runtime" in mermaid
    assert "SlamAdapterModule.odometry" in mermaid
    assert "driver.cmd_vel" in mermaid


def test_blueprint_records_route_metadata() -> None:
    graph = Blueprint("nav").add(RouteModule).route_contract(robot()).export_graph(profile="unit")

    manifest = graph.to_manifest()

    assert "flow" not in manifest
    assert manifest["route"] == "robot"
    assert manifest["route_contract"] == "robot"
    assert manifest["routed_delivery"] is False


def test_blueprint_route_contract_does_not_change_internal_delivery(monkeypatch) -> None:
    created: list[str] = []

    def fake_route_transport(strategy: str):
        created.append(strategy)
        return FakeRouteTransport()

    monkeypatch.setattr(
        "runtime.transport.factory.create_route_transport_adapter",
        fake_route_transport,
        raising=False,
    )

    graph = (
        Blueprint("nav")
        .add(CmdProducer, alias="producer")
        .add(CmdConsumer, alias="consumer")
        .wire("producer", "cmd_vel", "consumer", "cmd_vel", topic=TOPICS.cmd_vel)
        .route_contract(robot())
    )

    handle = graph.build()

    assert created == []
    assert handle.connection_metadata[("producer", "cmd_vel", "consumer", "cmd_vel")] == {
        "delivery": "callback",
        "transport": "callback",
        "topic": TOPICS.cmd_vel,
    }


def test_blueprint_routed_delivery_drives_typed_dds_transport(monkeypatch) -> None:
    created: list[str] = []

    def fake_route_transport(strategy: str):
        created.append(strategy)
        return FakeRouteTransport()

    monkeypatch.setattr(
        "runtime.transport.factory.create_route_transport_adapter",
        fake_route_transport,
        raising=False,
    )

    graph = (
        Blueprint("nav")
        .add(CmdProducer, alias="producer")
        .add(CmdConsumer, alias="consumer")
        .wire("producer", "cmd_vel", "consumer", "cmd_vel", topic=TOPICS.cmd_vel)
        .routed_delivery(robot())
    )

    handle = graph.build()

    assert created == ["dds"]
    assert handle.connection_metadata[("producer", "cmd_vel", "consumer", "cmd_vel")] == {
        "delivery": "transport",
        "transport": "dds",
        "topic": TOPICS.cmd_vel,
    }


def test_blueprint_robot_route_leaves_unknown_topic_local(monkeypatch) -> None:
    created: list[str] = []

    def fake_route_transport(strategy: str):
        created.append(strategy)
        return FakeRouteTransport()

    monkeypatch.setattr(
        "runtime.transport.factory.create_route_transport_adapter",
        fake_route_transport,
        raising=False,
    )

    graph = (
        Blueprint("nav")
        .add(CmdProducer, alias="producer")
        .add(CmdConsumer, alias="consumer")
        .wire("producer", "cmd_vel", "consumer", "cmd_vel", topic="/debug/custom")
        .routed_delivery(robot())
    )

    handle = graph.build()

    assert created == []
    assert handle.connection_metadata[("producer", "cmd_vel", "consumer", "cmd_vel")] == {
        "delivery": "callback",
        "transport": "callback",
        "topic": "/debug/custom",
    }


def test_blueprint_merge_rejects_route_mismatch() -> None:
    left = Blueprint().add(RouteModule, alias="left").route_contract(robot())
    right = Blueprint().add(RouteModule, alias="right").route_contract(sim())

    try:
        left.merge(right)
    except ValueError as exc:
        assert "route contract mismatch" in str(exc)
    else:
        raise AssertionError("route mismatch should fail")


def test_route_name_loads_builtin_preset() -> None:
    contract = load_route_contract("robot")

    assert validate_route_contract(contract) == []
    assert contract.route.name == "robot"
