"""Camera gRPC shares the JPEG source, viewing policy and stream lifecycle."""

import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError
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


@pytest.mark.parametrize("method", ["CameraFrame", "WatchCamera", "CameraSample", "WatchCameraSamples"])
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
    with endpoint(relay=True, cold=True) as (_, stub, _):
        def read():
            result = getattr(stub, method)(common_pb.Empty(), metadata=metadata, timeout=2)
            if method.startswith("Watch"):
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
            result = read()
            expected = b"jpeg-second" if method.startswith("Watch") else b"jpeg-first"
            assert (result.frame.jpeg if "Sample" in method else result.jpeg) == expected


def test_snapshot_reuses_jpeg_and_missing_frame_is_unavailable():
    with endpoint() as (_, stub, _):
        assert stub.CameraFrame(common_pb.Empty(), timeout=2).jpeg == b"jpeg-first"
    with endpoint(jpeg=None) as (_, stub, _):
        with pytest.raises(grpc.RpcError) as error:
            stub.CameraFrame(common_pb.Empty(), timeout=2)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE


def test_camera_samples_preserve_source_sequence_and_unknown_metadata():
    with endpoint(relay=True) as (gateway, stub, _):
        snapshot = stub.CameraSample(common_pb.Empty(), timeout=2)
        assert snapshot.frame.jpeg == b"jpeg-first"
        assert snapshot.info.encoding == "jpeg"
        assert not snapshot.info.HasField("camera_id")
        assert not snapshot.info.HasField("width_px")
        assert not snapshot.info.HasField("height_px")
        assert not snapshot.info.HasField("frame_id")
        assert not snapshot.HasField("sequence")
        assert not snapshot.HasField("time")

        call = stub.WatchCameraSamples(common_pb.Empty(), timeout=2)
        call.initial_metadata()
        gateway._camera_module.publish()
        first = next(call)
        assert first.frame.jpeg == b"jpeg-second"
        assert first.info.encoding == "jpeg"
        assert first.sequence == 2
        assert not first.HasField("time")
        with gateway._jpeg_lock:
            gateway._latest_jpeg = b"jpeg-third"
            gateway._latest_jpeg_seq += 1
        second = next(call)
        assert second.frame.jpeg == b"jpeg-third"
        assert second.sequence == 3
        call.cancel()


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
    with endpoint(relay=True, cold=True) as (gateway, stub, _):
        call = stub.WatchCamera(common_pb.Empty(), timeout=2)
        assert next(call).jpeg == b"jpeg-second"
        with pytest.raises(grpc.RpcError) as error:
            next(call)
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
        assert gateway._camera_module.clients == 0


def test_camera_streams_share_limit_and_leave_ping_available():
    with endpoint(relay=True) as (gateway, stub, system):
        calls = [stub.WatchCamera(common_pb.Empty(), timeout=3) for _ in range(8)]
        try:
            for call in calls:
                call.initial_metadata()
            gateway._camera_module.publish()
            assert all(next(call).jpeg == b"jpeg-second" for call in calls)
            with pytest.raises(grpc.RpcError) as error:
                next(stub.WatchCameraSamples(common_pb.Empty(), timeout=2))
            assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            assert system.Ping(common_pb.Empty(), timeout=2).status == "ok"
        finally:
            for call in calls:
                call.cancel()
        deadline = time.monotonic() + 2
        while gateway._camera_module.clients and time.monotonic() < deadline:
            time.sleep(0.01)
        assert gateway._camera_module.clients == 0


@pytest.mark.parametrize("method", ["WatchCamera", "WatchCameraSamples"])
def test_cached_frame_waits_for_publication_after_subscription(method):
    with endpoint(relay=True) as (gateway, stub, _):
        call = getattr(stub, method)(common_pb.Empty(), timeout=3)
        call.initial_metadata()
        with ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(next, call)
            try:
                with pytest.raises(TimeoutError):
                    pending.result(timeout=0.1)
                gateway._camera_module.publish()
                received = pending.result(timeout=1)
                assert (received.frame.jpeg if "Samples" in method else received.jpeg) == b"jpeg-second"
            finally:
                call.cancel()


def test_cancel_before_first_frame_unregisters_and_releases_slot():
    with endpoint(relay=True) as (gateway, stub, _):
        for _ in range(9):
            call = stub.WatchCamera(common_pb.Empty(), timeout=2)
            call.initial_metadata()
            call.cancel()
            deadline = time.monotonic() + 1
            while gateway._camera_module.clients and time.monotonic() < deadline:
                time.sleep(0.01)
            assert gateway._camera_module.clients == 0


def test_cancelled_request_does_not_register_camera_client():
    gateway = SimpleNamespace()
    gateway._camera_module = Relay(gateway)
    context = SimpleNamespace(invocation_metadata=lambda: (), add_callback=lambda callback: False)
    slots = threading.BoundedSemaphore(1)

    assert list(camera.watch(gateway, context, slots)) == []
    assert gateway._camera_module.clients == 0
    assert slots.acquire(blocking=False)
