"""Camera gRPC shares the JPEG source, viewing policy and stream lifecycle."""

import socket
import threading
import time
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc import camera
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import observations_pb2_grpc as observations_rpc
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.server import start


class Relay:
    def __init__(self, gateway, *, cold=False):
        self.gateway = gateway
        self.clients = 0
        self.cold = cold

    def on_camera_client_connect(self):
        self.clients += 1
        if self.cold:
            threading.Timer(0.05, self.publish).start()

    def on_camera_client_disconnect(self):
        self.clients -= 1

    def publish(self):
        with self.gateway._jpeg_lock:
            self.gateway._latest_jpeg = b"jpeg-second"
            self.gateway._latest_jpeg_seq += 1


@contextmanager
def endpoint(*, jpeg=b"jpeg-first", relay=False, cold=False):
    gateway = SimpleNamespace(_jpeg_lock=threading.Lock(), _latest_jpeg=jpeg, _latest_jpeg_seq=1, _all_modules={})
    if relay:
        gateway._camera_module = Relay(gateway, cold=cold)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(gateway, f"127.0.0.1:{port}")
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        try:
            yield gateway, observations_rpc.ObservationsStub(channel), system_rpc.SystemStub(channel)
        finally:
            server.stop(0).wait(3)


@pytest.fixture(autouse=True)
def development_policy(monkeypatch):
    monkeypatch.delenv("LINGTU_MEDIA_REQUIRE_TOKEN", raising=False)
    monkeypatch.delenv("LINGTU_MEDIA_VIEWER_TOKEN", raising=False)


@pytest.mark.parametrize("method", ["CameraFrame", "WatchCamera"])
@pytest.mark.parametrize("token,header,code", [
    (None, None, grpc.StatusCode.UNAVAILABLE),
    ("viewer", None, grpc.StatusCode.UNAUTHENTICATED),
    ("viewer", "Bearer wrong", grpc.StatusCode.PERMISSION_DENIED),
    ("viewer", "Bearer viewer", None),
])
def test_camera_uses_media_bearer(monkeypatch, method, token, header, code):
    monkeypatch.setenv("LINGTU_MEDIA_REQUIRE_TOKEN", "1")
    if token:
        monkeypatch.setenv("LINGTU_MEDIA_VIEWER_TOKEN", token)
    metadata = [("authorization", header)] if header else [("x-api-key", "viewer")]
    with endpoint() as (_, stub, _):
        def read():
            result = getattr(stub, method)(common_pb.Empty(), metadata=metadata, timeout=2)
            if method == "WatchCamera":
                try:
                    return next(result)
                finally:
                    result.cancel()
            return result
        if code:
            with pytest.raises(grpc.RpcError) as error:
                read()
            assert error.value.code() == code
        else:
            assert read().jpeg == b"jpeg-first"


def test_snapshot_reuses_jpeg_and_missing_frame_is_unavailable():
    with endpoint() as (_, stub, _):
        assert stub.CameraFrame(common_pb.Empty(), timeout=2).jpeg == b"jpeg-first"
    with endpoint(jpeg=None) as (_, stub, _):
        with pytest.raises(grpc.RpcError) as error:
            stub.CameraFrame(common_pb.Empty(), timeout=2)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE


def test_cold_stream_waits_for_encoder_then_sends_changed_frames():
    with endpoint(jpeg=None, relay=True, cold=True) as (gateway, stub, _):
        call = stub.WatchCamera(common_pb.Empty(), timeout=3)
        assert next(call).jpeg == b"jpeg-second"
        with gateway._jpeg_lock:
            gateway._latest_jpeg = b"jpeg-third"
            gateway._latest_jpeg_seq += 1
        assert next(call).jpeg == b"jpeg-third"
        call.cancel()
        deadline = time.monotonic() + 2
        while gateway._camera_module.clients and time.monotonic() < deadline:
            time.sleep(0.01)
        assert gateway._camera_module.clients == 0


def test_empty_or_stalled_stream_ends_and_unregisters(monkeypatch):
    monkeypatch.setattr(camera, "CAMERA_STREAM_STALE_MS", 100)
    with endpoint(jpeg=None, relay=True) as (gateway, stub, _):
        with pytest.raises(grpc.RpcError) as error:
            next(stub.WatchCamera(common_pb.Empty(), timeout=2))
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert gateway._camera_module.clients == 0
    with endpoint(relay=True) as (gateway, stub, _):
        call = stub.WatchCamera(common_pb.Empty(), timeout=2)
        assert next(call).jpeg == b"jpeg-first"
        with pytest.raises(grpc.RpcError) as error:
            next(call)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert gateway._camera_module.clients == 0


def test_camera_streams_share_limit_and_leave_ping_available():
    with endpoint(relay=True) as (gateway, stub, system):
        calls = [stub.WatchCamera(common_pb.Empty(), timeout=3) for _ in range(8)]
        try:
            assert all(next(call).jpeg == b"jpeg-first" for call in calls)
            with pytest.raises(grpc.RpcError) as error:
                next(stub.WatchCamera(common_pb.Empty(), timeout=2))
            assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            assert system.Ping(common_pb.Empty(), timeout=2).status == "ok"
        finally:
            for call in calls:
                call.cancel()
        deadline = time.monotonic() + 2
        while gateway._camera_module.clients and time.monotonic() < deadline:
            time.sleep(0.01)
        assert gateway._camera_module.clients == 0
