from __future__ import annotations

import asyncio
import ctypes
import json
import time
from types import SimpleNamespace

import pytest

import gateway.gateway_module as gateway_module
import lingtu.assembly.host_bus as bus_module
from gateway.gateway_module import GatewayModule
from gateway.services.runtime_dataflow import build_runtime_dataflow_topic_detail
from gateway.services.sse import subscribe, unsubscribe
from gateway.services.subscriptions import setup_subscriptions
from gateway.services.traffic import format_sse_message, prepare_sse_delivery, put_latest
from lingtu.assembly.compiler import blueprint_from_run_plan, compile_run_plan
from lingtu.assembly.graph import ProcessArtifact
from lingtu.assembly.host_bus import HostBus
from lingtu.assembly.wires.gateway import gateway_status_specs
from message.topics import TOPICS
from nav.adapters.native import abi
from runtime.endpoints.dds.adapters import joint_state_from_dds
from runtime.msgs import JointState


def payload(stamp=100.25):
    return {
        "timestamp_s": stamp,
        "robot_model": "go2",
        "names": [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")],
        "position": [index * 0.1 for index in range(12)],
        "velocity": [index * 0.2 for index in range(12)],
        "effort": [index * 0.3 for index in range(12)],
    }


def test_joint_message_preserves_source_timestamp_names_and_all_vectors():
    data = payload()
    message = joint_state_from_dds(data)
    decoded = JointState.decode(message.encode())
    assert decoded.to_dict() == message.to_dict()
    assert decoded.ts == 100.25
    assert decoded.names == data["names"]
    assert decoded.position == data["position"]
    assert decoded.velocity == data["velocity"]
    assert decoded.effort == data["effort"]


@pytest.mark.parametrize("changes", [
    {"timestamp_s": float("nan")}, {"timestamp_s": 0}, {"robot_model": ""},
    {"names": ["FR_hip_joint"] * 12}, {"names": [""] * 12},
    {"position": [0.0] * 11}, {"effort": [float("inf")] * 12},
    {"names": [f"joint_{i}" for i in range(13)]},
])
def test_invalid_joint_payload_is_rejected(changes):
    with pytest.raises(ValueError):
        joint_state_from_dds({**payload(), **changes})


def test_native_joint_abi_reads_fixed_bounded_snapshot_and_consumes_once(tmp_path):
    pending = [payload()]

    def take(_handle, pointer):
        if not pending:
            return 0
        data = pending.pop()
        target = pointer._obj
        assert target.abi_version == 1
        assert target.struct_size == ctypes.sizeof(abi._NativeJointStateV1)
        target.timestamp_s = data["timestamp_s"]
        target.robot_model = data["robot_model"].encode()
        target.joint_count = len(data["names"])
        for index, name in enumerate(data["names"]):
            target.names[index].value = name.encode()
            target.position[index] = data["position"][index]
            target.velocity[index] = data["velocity"][index]
            target.effort[index] = data["effort"][index]
        return 1

    library = SimpleNamespace(
        lingtu_nav_client_abi_version=lambda: abi.NATIVE_COMMAND_ABI_VERSION,
        lingtu_nav_client_capabilities=lambda: abi.NATIVE_COMMAND_CAP_JOINT_STATE,
        lingtu_nav_client_create=lambda _domain: 41,
        lingtu_nav_client_destroy=lambda _handle: None,
        lingtu_nav_client_last_error=lambda _handle: b"",
        lingtu_nav_client_take_joint_state_v1=take,
    )
    session = abi.NativeCommandSession(tmp_path / "native.dll", library=library)
    try:
        assert session.take_joint_state() == payload()
        assert session.take_joint_state() is None
        library.lingtu_nav_client_take_joint_state_v1 = lambda _handle, pointer: setattr(pointer._obj, "joint_count", 13) or 1
        with pytest.raises(abi.NativeCommandClientError, match="header"):
            session.take_joint_state()
    finally:
        session.close()


def test_host_bus_discards_stale_and_invalid_joint_samples_without_gating_readiness(monkeypatch):
    bus = HostBus()
    bus._running = True
    bus._received = True
    bus._nav_received_monotonic = time.monotonic()
    observed = []
    bus.joint_state._add_callback(observed.append)
    monkeypatch.setattr(bus_module.time, "time", lambda: 101.0)
    pending = [payload(100.25), payload(98.0), payload(103.0), {**payload(), "position": []}]
    bus._session = SimpleNamespace(take_joint_state=lambda: pending.pop(0))
    assert bus.startup_readiness() is None
    for _ in range(4):
        bus._poll_joint_state()
    assert [state.ts for state in observed] == [100.25]
    assert bus.startup_readiness() is None
    assert bus._failure == ""


def test_joint_wire_is_latest_and_gateway_emits_flat_source_aged_sse(monkeypatch):
    specs = gateway_status_specs(SimpleNamespace(names={"host.bus", "GatewayModule"}))
    joint_wires = [spec for spec in specs if spec.out_port == "joint_state"]
    assert len(joint_wires) == 1
    assert joint_wires[0].in_module == "GatewayModule"
    assert joint_wires[0].in_port == "joint_state"
    gateway = GatewayModule()
    gateway._build_app = lambda: None
    setup_subscriptions(gateway)
    assert gateway.joint_state.policy == "latest"
    monkeypatch.setattr(gateway_module.time, "time", lambda: 101.0)
    queue = subscribe(gateway)
    try:
        gateway.joint_state._deliver(joint_state_from_dds(payload()))
        event = queue.get_nowait()
        assert event["type"] == "joint_state"
        assert "data" not in event
        assert event["ts"] == event["stamp"] == 100.25
        assert event["source_age_s"] == 0.75
        assert event["names"] == payload()["names"]
        ready = prepare_sse_delivery(event, now=101.75)
        assert ready["source_age_s"] == 1.5
        encoded = format_sse_message(ready)
        wire = json.loads(next(line[6:] for line in encoded.splitlines() if line.startswith("data: ")))
        assert wire["stamp"] == 100.25
        assert wire["source_age_s"] == 1.5
        assert prepare_sse_delivery(event, now=103.0) is None
        for stamp in (100.25, 98.0, 103.0):
            gateway.joint_state._deliver(joint_state_from_dds(payload(stamp)))
        assert queue.empty()
        later_subscriber = subscribe(gateway)
        assert later_subscriber.empty()
        unsubscribe(gateway, later_subscriber)
    finally:
        unsubscribe(gateway, queue)


def test_joint_sse_backlog_coalesces_and_never_evicts_command_events():
    queue = asyncio.Queue(maxsize=3)
    put_latest(queue, {"type": "command_ack", "id": 1})
    put_latest(queue, {"type": "joint_state", "stamp": 1})
    put_latest(queue, {"type": "command_ack", "id": 2})
    assert put_latest(queue, {"type": "joint_state", "stamp": 2})
    assert [queue.get_nowait() for _ in range(3)] == [
        {"type": "command_ack", "id": 1}, {"type": "command_ack", "id": 2},
        {"type": "joint_state", "stamp": 2},
    ]
    for index in range(3):
        put_latest(queue, {"type": "command_ack", "id": index})
    assert put_latest(queue, {"type": "joint_state", "stamp": 3})
    assert [queue.get_nowait()["type"] for _ in range(3)] == ["command_ack"] * 3


@pytest.mark.parametrize(("product", "env"), [("map", "real"), ("nav", "real"), ("teleop", "real"), ("nav", "sim")])
def test_joint_topic_subscription_is_optional_and_uses_product_host_bus(product, env, monkeypatch):
    if env == "sim":
        monkeypatch.setattr(ProcessArtifact, "from_repository_path", classmethod(lambda cls, root, path: cls(str(path))))
    plan = compile_run_plan(
        product, env, robot="unitree/go2" if env == "real" else "doso/thunder_v4",
        env_config={"backend": "mujoco"} if env == "sim" else None,
    )
    blueprint = blueprint_from_run_plan(plan)
    assert {"host.bus", "GatewayModule"} <= set(blueprint.module_names)
    assert any(
        wire.out_module == "host.bus" and wire.out_port == "joint_state"
        and wire.in_module == "GatewayModule" and wire.in_port == "joint_state"
        for wire in blueprint._wires
    )
    assert TOPICS.robot_joint_states not in plan.required_topics
    gateway = GatewayModule(run_plan=plan)
    gateway.setup()
    detail = build_runtime_dataflow_topic_detail(gateway, TOPICS.robot_joint_states)
    assert detail["ok"] is True
    assert detail["topic"]["required_by_product"] is False
    assert detail["topic"]["data_flow_stages"] == []
    assert detail["inspection"]["live"] is False
    assert detail["inspection"]["payload_sample_available"] is False
    assert detail["inspection"]["communicate"] is False

    def decode(chunk):
        if isinstance(chunk, bytes):
            chunk = chunk.decode()
        return json.loads(next(line[6:] for line in chunk.splitlines() if line.startswith("data: ")))

    async def read_subscription():
        route = next(route for route in gateway._app.routes if route.path == "/api/v1/events")
        response = await route.endpoint(topic=TOPICS.robot_joint_states)
        iterator = response.body_iterator
        try:
            initial = decode(await iterator.__anext__())
            assert initial["data"]["ok"] is True
            assert initial["data"]["event_types"] == ["joint_state"]
            if env == "sim":
                assert all(queue.empty() for queue in gateway._sse_queues)
                return
            source_stamp = time.time() - 0.25
            gateway.push_event({"type": "command_ack", "data": {"ignored": True}})
            gateway.joint_state._deliver(joint_state_from_dds(payload(source_stamp)))
            event = decode(await asyncio.wait_for(iterator.__anext__(), timeout=0.5))
            assert event["type"] == "joint_state"
            assert event["ts"] == event["stamp"] == source_stamp
            assert 0.25 <= event["source_age_s"] < 2.0
            assert event["names"] == payload()["names"]
        finally:
            await iterator.aclose()

    asyncio.run(read_subscription())
    assert gateway._sse_queues == []
