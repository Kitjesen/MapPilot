from __future__ import annotations

import re
from dataclasses import replace
from pathlib import Path

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
    assert contract.route.endpoint_contract == "thunder_field_dds_v1"
    assert contract.route_for(TOPICS.lidar_scan) == "dds"
    assert contract.route_for(TOPICS.goal_pose) == "dds"
    assert contract.route_for(TOPICS.cancel) == "dds"
    assert contract.route_for(TOPICS.semantic_instruction) == "local"
    assert contract.route_for(TOPICS.nav_way_point) == "dds"
    assert contract.route_for(TOPICS.cmd_vel) == "dds"
    assert contract.binding_for(TOPICS.cmd_vel)["qos"] == "command"


def test_robot_route_matches_thunder_field_dds_endpoint_topics() -> None:
    from runtime.endpoints.dds.contracts import THUNDER_FIELD_DDS_CONTRACT

    contract = load_route_contract(robot())

    assert set(contract.route.routes) == set(THUNDER_FIELD_DDS_CONTRACT.topics)
    assert validate_route_contract(contract) == []


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


def test_route_name_compatibility_still_loads_builtin_preset() -> None:
    contract = load_route_contract("robot")

    assert validate_route_contract(contract) == []
    assert contract.route.name == "robot"


def test_blueprint_route_legacy_alias_still_enables_routed_delivery() -> None:
    with pytest.warns(DeprecationWarning, match="Blueprint.route"):
        graph = Blueprint("nav").add(RouteModule).route(robot()).export_graph(profile="unit")
    manifest = graph.to_manifest()

    assert manifest["route_contract"] == "robot"
    assert manifest["routed_delivery"] is True


def test_product_builders_do_not_call_legacy_blueprint_route() -> None:
    root = Path(__file__).resolve().parents[3]
    legacy_call = re.compile(r"\.route\s*\(")
    checked_dirs = (
        root / "cli",
        root / "src" / "lingtu",
        root / "src" / "runtime" / "blueprints",
        root / "src" / "runtime" / "profiles",
    )
    offenders: list[str] = []
    for base in checked_dirs:
        if not base.exists():
            continue
        for path in base.rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            for line_no, line in enumerate(text.splitlines(), start=1):
                if legacy_call.search(line):
                    offenders.append(f"{path.relative_to(root)}:{line_no}")

    assert offenders == []
