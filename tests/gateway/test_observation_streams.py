"""Read real protobuf events through both local gRPC boundaries."""

import threading
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest
from google.protobuf.duration_pb2 import Duration
from google.protobuf.empty_pb2 import Empty
from tools.sdk.observe import start as start_observation_listener

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import motion_pb2_grpc as motion_rpc
from gateway.rpc.generated import navigation_pb2 as navigation_pb
from gateway.rpc.generated import navigation_pb2_grpc as navigation_rpc
from gateway.rpc.generated import observations_pb2_grpc as observations_rpc
from gateway.rpc.generated import system_pb2 as system_pb
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.observations import imu_sample, joint_sample
from gateway.rpc.server import start
from runtime.adapters.brainstem.generated.brainstem_api import cms_pb2 as source_pb
from runtime.adapters.brainstem.generated.brainstem_api import common_pb2 as types_pb
from runtime.adapters.brainstem.source import Observations


def imu():
    return source_pb.Imu(
        gyroscope=types_pb.Vector3(x=1, y=2, z=3), quaternion=types_pb.Quaternion(w=1),
        timestamp=Duration(seconds=8, nanos=9),
    )


@contextmanager
def upstream(handler):
    server = grpc.server(ThreadPoolExecutor(max_workers=12))
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(
        "brainstem.api.v1.RobotControl",
        {name: grpc.unary_stream_rpc_method_handler(
            handler, request_deserializer=Empty.FromString, response_serializer=lambda event: event.SerializeToString(),
        ) for name in ("ListenImu", "ListenJoint")},
    ),))
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    try:
        yield Observations({"LINGTU_DRIVER_TARGET": f"127.0.0.1:{port}"})
    finally:
        server.stop(0).wait(3)


@contextmanager
def gateway(source):
    import socket

    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(SimpleNamespace(_observations=source), f"127.0.0.1:{port}")
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        try:
            yield channel, server
        finally:
            server.stop(0).wait(3)


def test_imu_preserves_source_fields_and_optional_acceleration():
    event = imu()
    sample = imu_sample(event)
    assert sample.angular_velocity_rad_s.y == 2
    assert sample.orientation_world_to_body.w == 1
    assert sample.source_elapsed == event.timestamp
    assert not sample.HasField("linear_acceleration_m_s2")
    event.linear_acceleration.z = 9.81
    assert imu_sample(event).linear_acceleration_m_s2.z == 9.81
    event.ClearField("timestamp")
    assert not imu_sample(event).HasField("source_elapsed")


def test_joints_preserve_single_axis_events_and_full_frames():
    event = source_pb.Joint(
        single_joint=source_pb.SingleJoint(id=12, position=1, velocity=2, torque=3, status=4),
        timestamp=Duration(seconds=5),
    )
    sample = joint_sample(event)
    assert len(sample.joints) == 1
    assert (sample.joints[0].id, sample.joints[0].position_rad, sample.joints[0].velocity_rad_s,
            sample.joints[0].torque_nm, sample.joints[0].status_code) == (12, 1, 2, 3, 4)
    assert sample.source_elapsed.seconds == 5
    frame = source_pb.AllJoints(
        position=types_pb.Matrix4(values=range(16)), velocity=types_pb.Matrix4(values=range(16)),
        torque=types_pb.Matrix4(values=range(16)), status=types_pb.Matrix4Int32(values=range(16)),
    )
    sample = joint_sample(source_pb.Joint(all_joints=frame))
    assert [joint.id for joint in sample.joints] == list(range(16))
    assert sample.joints[15].torque_nm == 15
    assert not sample.HasField("source_elapsed")


@pytest.mark.parametrize("event,convert", [
    (source_pb.Imu(), imu_sample),
    (source_pb.Joint(), joint_sample),
    (source_pb.Joint(single_joint=source_pb.SingleJoint(id=16)), joint_sample),
    (source_pb.Joint(all_joints=source_pb.AllJoints()), joint_sample),
])
def test_malformed_source_is_rejected(event, convert):
    with pytest.raises(ValueError):
        convert(event)


@pytest.mark.parametrize("event,method", [
    (imu(), "WatchImu"),
    (source_pb.Joint(single_joint=source_pb.SingleJoint(id=3, position=1.5)), "WatchJoints"),
])
def test_events_pass_both_grpc_boundaries(event, method):
    def events(request, context):
        yield event

    with upstream(events) as source, gateway(source) as (channel, _):
        samples = list(getattr(observations_rpc.ObservationsStub(channel), method)(common_pb.Empty(), timeout=3))
    assert len(samples) == 1
    assert samples[0] == (imu_sample(event) if method == "WatchImu" else joint_sample(event))


def test_upstream_error_preserves_status_details_and_metadata():
    def events(request, context):
        context.set_trailing_metadata((("source-error", "denied"),))
        context.abort(grpc.StatusCode.PERMISSION_DENIED, "source denied observation")
        yield

    with upstream(events) as source, gateway(source) as (channel, _):
        with pytest.raises(grpc.RpcError) as caught:
            next(observations_rpc.ObservationsStub(channel).WatchImu(common_pb.Empty(), timeout=3))
    assert caught.value.code() == grpc.StatusCode.PERMISSION_DENIED
    assert caught.value.details() == "source denied observation"
    assert ("source-error", "denied") in caught.value.trailing_metadata()


@pytest.mark.parametrize("source", [None, Observations({}), Observations({
    "LINGTU_DRIVER_TARGET": "192.168.66.64:13145",
})])
def test_missing_source_or_tls_is_unavailable(source):
    with gateway(source) as (channel, _):
        with pytest.raises(grpc.RpcError) as caught:
            next(observations_rpc.ObservationsStub(channel).WatchImu(common_pb.Empty(), timeout=3))
        assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE


def test_tls_uses_resolved_files_and_server_name(monkeypatch, tmp_path):
    environment = {"LINGTU_DRIVER_TARGET": "192.168.66.64:13145", "LINGTU_DRIVER_TLS_SERVER_NAME": "brainstem"}
    for name in ("CA", "CERT", "KEY"):
        path = tmp_path / name
        path.write_bytes(name.encode())
        environment[f"LINGTU_DRIVER_TLS_{name}_FILE"] = str(path)
    captured = {}

    def credentials(ca, key, certificate):
        captured["files"] = ca, key, certificate
        return "credentials"

    def channel(target, credentials, options):
        captured["channel"] = target, credentials, options
        return "channel"

    monkeypatch.setattr(grpc, "ssl_channel_credentials", credentials)
    monkeypatch.setattr(grpc, "secure_channel", channel)
    assert Observations(environment)._channel() == "channel"
    assert captured["files"] == (b"CA", b"KEY", b"CERT")
    assert captured["channel"] == (
        "192.168.66.64:13145", "credentials", [
            (grpc.experimental.ChannelOptions.SingleThreadedUnaryStream, 1),
            ("grpc.ssl_target_name_override", "brainstem"),
        ],
    )


@pytest.mark.parametrize("configuration", [
    {"LINGTU_DRIVER_TLS_SERVER_NAME": "brainstem"},
    {"LINGTU_DRIVER_TLS_CA_FILE": "ca.pem"},
    {"LINGTU_DRIVER_TLS_CA_FILE": "ca.pem", "LINGTU_DRIVER_TLS_CERT_FILE": "cert.pem",
     "LINGTU_DRIVER_TLS_KEY_FILE": "missing-key.pem"},
])
def test_invalid_tls_is_not_downgraded(configuration):
    source = Observations({"LINGTU_DRIVER_TARGET": "localhost:13145", **configuration})
    with gateway(source) as (channel, _), pytest.raises(grpc.RpcError) as caught:
        next(observations_rpc.ObservationsStub(channel).WatchImu(common_pb.Empty(), timeout=3))
    assert caught.value.code() == grpc.StatusCode.UNAVAILABLE


def test_cancel_shutdown_and_watch_limit_release_upstream():
    closed = threading.Semaphore(0)

    def events(request, context):
        stopped = threading.Event()
        context.add_callback(stopped.set)
        try:
            yield imu()
            stopped.wait(10)
        finally:
            closed.release()

    with upstream(events) as source, gateway(source) as (channel, server):
        stub = observations_rpc.ObservationsStub(channel)
        watches = [stub.WatchImu(common_pb.Empty(), timeout=10) for _ in range(8)]
        try:
            for watch in watches:
                next(watch)
            with pytest.raises(grpc.RpcError) as caught:
                next(stub.WatchImu(common_pb.Empty(), timeout=3))
            assert caught.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            for call in (
                motion_rpc.MotionStub(channel).WatchState(common_pb.Empty(), timeout=3),
                navigation_rpc.NavigationStub(channel).Watch(navigation_pb.TaskQuery(task_id="test"), timeout=3),
            ):
                with pytest.raises(grpc.RpcError) as caught:
                    next(call)
                assert caught.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"
            watches[0].cancel()
            assert closed.acquire(timeout=3)
            replacement = stub.WatchImu(common_pb.Empty(), timeout=10)
            watches.append(replacement)
            next(replacement)
            server.stop(0).wait(3)
            for _ in range(8):
                assert closed.acquire(timeout=3)
        finally:
            for watch in watches:
                watch.cancel()


@pytest.fixture
def observation_listener(monkeypatch):
    for name in ("CA_FILE", "CERT_FILE", "KEY_FILE", "SERVER_NAME"):
        monkeypatch.delenv(f"LINGTU_DRIVER_TLS_{name}", raising=False)
    monkeypatch.delenv("LINGTU_OBSERVATIONS_TRANSPORT", raising=False)

    @contextmanager
    def listener(source):
        server, port = start_observation_listener(source._environment["LINGTU_DRIVER_TARGET"], port=0,
                                                  lan_development=True)
        with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
            try:
                yield channel, server
            finally:
                server.stop(0).wait(3)

    return listener


@pytest.mark.parametrize("event,method", [
    (imu(), "WatchImu"),
    (source_pb.Joint(single_joint=source_pb.SingleJoint(id=3, position=1.5)), "WatchJoints"),
])
def test_read_only_listener_streams_and_preserves_disconnect(observation_listener, event, method):
    def events(request, context):
        yield event
        context.abort(grpc.StatusCode.UNAVAILABLE, "upstream disconnected")

    with upstream(events) as source, observation_listener(source) as (channel, _):
        reply = system_rpc.SystemStub(channel).Handshake(
            system_pb.Handshake(protocol_version=2, client_id="observation-test"), timeout=3,
        )
        assert reply.protocol_version == 2
        watch = getattr(observations_rpc.ObservationsStub(channel), method)(common_pb.Empty(), timeout=3)
        assert next(watch) == (imu_sample(event) if method == "WatchImu" else joint_sample(event))
        with pytest.raises(grpc.RpcError) as caught:
            next(watch)
        assert caught.value.code() == grpc.StatusCode.UNAVAILABLE
        assert caught.value.details() == "upstream disconnected"
        for method_name in ("System/Ping", "Motion/Stop", "Maps/List", "Observations/Motors"):
            with pytest.raises(grpc.RpcError) as caught:
                channel.unary_unary(f"/doso.v1.{method_name}")(b"", timeout=3)
            assert caught.value.code() == grpc.StatusCode.UNIMPLEMENTED


@pytest.mark.parametrize("stop_server", [False, True])
def test_read_only_listener_cancel_and_shutdown_release_upstream(observation_listener, stop_server):
    closed = threading.Event()

    def events(request, context):
        stopped = threading.Event()
        context.add_callback(stopped.set)
        try:
            yield imu()
            stopped.wait(10)
        finally:
            closed.set()

    with upstream(events) as source, observation_listener(source) as (channel, server):
        watch = observations_rpc.ObservationsStub(channel).WatchImu(common_pb.Empty(), timeout=10)
        try:
            next(watch)
            if stop_server:
                server.stop(0).wait(3)
            else:
                watch.cancel()
            assert closed.wait(3)
        finally:
            watch.cancel()
