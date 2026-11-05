"""Body-state queries and events through both real local gRPC boundaries."""

import socket
import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest
from google.protobuf.empty_pb2 import Empty

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import motion_pb2_grpc as motion_rpc
from gateway.rpc.generated import navigation_pb2 as navigation_pb
from gateway.rpc.generated import navigation_pb2_grpc as navigation_rpc
from gateway.rpc.generated import observations_pb2_grpc as observations_rpc
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.server import start
from runtime.adapters.brainstem.generated.brainstem_api import cms_pb2 as source_pb
from runtime.adapters.brainstem.source import Observations


@contextmanager
def upstream(query, events):
    server = grpc.server(ThreadPoolExecutor(max_workers=12))
    options = {
        "request_deserializer": Empty.FromString,
        "response_serializer": lambda event: event.SerializeToString(),
    }
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
        "brainstem.api.v1.RobotControl", {
            "GetCmsState": grpc.unary_unary_rpc_method_handler(query, **options),
            "ListenCmsState": grpc.unary_stream_rpc_method_handler(events, **options),
        },
    ),))
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        yield Observations({"LINGTU_DRIVER_TARGET": f"127.0.0.1:{port}"})
    finally:
        server.stop(0).wait(3)


@contextmanager
def gateway(source):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(SimpleNamespace(_observations=source), f"127.0.0.1:{port}")
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        try:
            yield channel, server
        finally:
            server.stop(0).wait(3)


def test_body_query_and_events_preserve_order_and_unknown_values():
    states = [
        source_pb.CmsState(kind=1), source_pb.CmsState(kind=4, transition=1),
        source_pb.CmsState(kind=2, transition=3, gesture_name="bow"),
        source_pb.CmsState(kind=999, transition=777, gesture_name="future"),
    ]
    with upstream(lambda request, context: states[-1], lambda request, context: iter(states)) as source:
        with gateway(source) as (channel, _):
            stub = motion_rpc.MotionStub(channel)
            current = stub.BodyState(common_pb.Empty(), timeout=3)
            received = list(stub.WatchBodyState(common_pb.Empty(), timeout=3))
    assert (current.kind, current.transition, current.gesture_name) == (999, 777, "future")
    assert [(state.kind, state.transition, state.gesture_name) for state in received] == [
        (1, 0, ""), (4, 1, ""), (2, 3, "bow"), (999, 777, "future"),
    ]


@pytest.mark.parametrize("stream", [False, True])
def test_body_source_failures_are_preserved(stream):
    def failure(request, context):
        context.set_trailing_metadata((("source-error", "denied"),))
        context.abort(grpc.StatusCode.PERMISSION_DENIED, "body observation denied")

    with upstream(failure, failure) as source, gateway(source) as (channel, _):
        stub = motion_rpc.MotionStub(channel)
        with pytest.raises(grpc.RpcError) as caught:
            if stream:
                next(stub.WatchBodyState(common_pb.Empty(), timeout=3))
            else:
                stub.BodyState(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert caught.value.details() == "body observation denied"
    assert ("source-error", "denied") in caught.value.trailing_metadata()


def test_body_midstream_failure_is_preserved():
    def events(request, context):
        yield source_pb.CmsState(kind=2)
        context.abort(grpc.StatusCode.UNAVAILABLE, "body source stopped")

    with upstream(lambda request, context: source_pb.CmsState(), events) as source:
        with gateway(source) as (channel, _):
            call = motion_rpc.MotionStub(channel).WatchBodyState(common_pb.Empty(), timeout=3)
            assert next(call).kind == 2
            with pytest.raises(grpc.RpcError) as caught:
                next(call)
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE
    assert caught.value.details() == "body source stopped"


@pytest.mark.parametrize("stream", [False, True])
def test_body_missing_source_is_unavailable(stream):
    with gateway(None) as (channel, _):
        stub = motion_rpc.MotionStub(channel)
        with pytest.raises(grpc.RpcError) as caught:
            if stream:
                next(stub.WatchBodyState(common_pb.Empty(), timeout=3))
            else:
                stub.BodyState(common_pb.Empty(), timeout=3)
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE


def test_body_cancel_shutdown_and_shared_watch_slots():
    closed = threading.Semaphore(0)

    def events(request, context):
        stopped = threading.Event()
        context.add_callback(stopped.set)
        try:
            yield source_pb.CmsState(kind=1)
            stopped.wait(10)
        finally:
            closed.release()

    with upstream(lambda request, context: source_pb.CmsState(kind=1), events) as source:
        with gateway(source) as (channel, server):
            motion = motion_rpc.MotionStub(channel)
            calls = [motion.WatchBodyState(common_pb.Empty(), timeout=10) for _ in range(8)]
            try:
                for call in calls:
                    assert next(call).kind == 1
                for call in (
                    motion.WatchBodyState(common_pb.Empty(), timeout=3),
                    motion.WatchState(common_pb.Empty(), timeout=3),
                    observations_rpc.ObservationsStub(channel).WatchImu(common_pb.Empty(), timeout=3),
                    navigation_rpc.NavigationStub(channel).Watch(navigation_pb.TaskQuery(task_id="test"), timeout=3),
                ):
                    with pytest.raises(grpc.RpcError) as caught:
                        next(call)
                    assert caught.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
                assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"
                assert motion.BodyState(common_pb.Empty(), timeout=3).kind == 1
                calls[0].cancel()
                assert closed.acquire(timeout=3)
                replacement = motion.WatchBodyState(common_pb.Empty(), timeout=10)
                calls.append(replacement)
                assert next(replacement).kind == 1
                server.stop(0).wait(3)
                for _ in range(8):
                    assert closed.acquire(timeout=3)
            finally:
                for call in calls:
                    call.cancel()
