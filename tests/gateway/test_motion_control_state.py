"""Read the actual source control status through both local gRPC boundaries."""

import socket
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest
from google.protobuf.empty_pb2 import Empty

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import motion_pb2_grpc as motion_rpc
from gateway.rpc.server import start
from runtime.adapters.brainstem.generated.brainstem_api import cms_pb2 as source_pb
from runtime.adapters.brainstem.source import Observations


@contextmanager
def gateway(source):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(SimpleNamespace(_observations=source), f"127.0.0.1:{port}")
    try:
        with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
            yield motion_rpc.MotionStub(channel)
    finally:
        server.stop(0).wait(3)


@contextmanager
def upstream(handler):
    server = grpc.server(ThreadPoolExecutor(max_workers=2))
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
        "brainstem.api.v1.RobotControl", {
            "GetControlStatus": grpc.unary_unary_rpc_method_handler(
                handler, request_deserializer=Empty.FromString,
                response_serializer=lambda event: event.SerializeToString(),
            ),
        },
    ),))
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        yield Observations({"LINGTU_DRIVER_TARGET": f"127.0.0.1:{port}"})
    finally:
        server.stop(0).wait(3)


def test_control_query_preserves_source_owner_and_status_without_control_requests():
    calls = []
    status = source_pb.ControlStatus(
        fsm=source_pb.CmsState(kind=999, transition=777, gesture_name="future"),
        motor_output_enabled=True, owner=2, owner_id="physical-remote",
        critical_motor_fault=True, grpc_lease_active=False,
        last_accepted_sequence=17, last_decision=9, ready_for_walk=False,
        accepted_count=21, rejected_count=3,
    )

    def read(request, context):
        calls.append("read")
        return status

    with upstream(read) as source, gateway(source) as stub:
        result = stub.ControlStatus(common_pb.Empty(), timeout=3)
        assert (result.source_owner, result.source_owner_id) == (2, "physical-remote")
        assert (result.body.kind, result.body.transition, result.body.gesture_name) == (999, 777, "future")
        assert result.motor_output_enabled and result.critical_motor_fault
        assert not result.grpc_lease_active and not result.source_ready_for_walk
        assert (result.last_accepted_sequence, result.accepted_count, result.rejected_count) == (17, 21, 3)
        status.ClearField("fsm")
        status.owner = 999
        result = stub.ControlStatus(common_pb.Empty(), timeout=3)
        assert not result.HasField("body")
        assert result.source_owner == 999
    assert calls == ["read", "read"]


def test_control_query_preserves_source_failure():
    def denied(request, context):
        context.set_trailing_metadata((("source-error", "read-denied"),))
        context.abort(grpc.StatusCode.PERMISSION_DENIED, "source control observation denied")

    with upstream(denied) as source, gateway(source) as stub:
        with pytest.raises(grpc.RpcError) as caught:
            stub.ControlStatus(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert caught.value.details() == "source control observation denied"
    assert ("source-error", "read-denied") in caught.value.trailing_metadata()


def test_control_query_requires_a_source():
    with gateway(None) as stub:
        with pytest.raises(grpc.RpcError) as caught:
            stub.ControlStatus(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE
