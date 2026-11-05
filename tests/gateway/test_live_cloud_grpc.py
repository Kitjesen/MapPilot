"""Live cloud RPCs consume the same preview owner as WebSocket viewers."""

import asyncio
import socket
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import numpy as np
import pytest

from gateway.rpc.cloud import MAX_FRAME_BYTES
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import observations_pb2 as pb
from gateway.rpc.generated import observations_pb2_grpc as rpc
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.server import start
from gateway.services.cloud_viewer import CloudViewerService
from gateway.services.sse import call_queue_put_latest, running_loop_or_none
from runtime.msgs.sensor import PointCloud2
from runtime.utils.binary_codec import decode_pointcloud_frame, encode_pointcloud


def viewer():
    service = CloudViewerService(
        queue_put_latest=call_queue_put_latest,
        current_loop=running_loop_or_none,
        push_event=lambda event: None,
        session_mode=lambda: "navigating",
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )
    service.configure(scan_viewer_min_interval_s=0, cloud_viewer_min_interval_s=0)
    return service


@contextmanager
def endpoint(service):
    gateway = SimpleNamespace(
        _cloud_viewer=service, _jpeg_lock=threading.Lock(), _latest_jpeg=b"jpeg", _latest_jpeg_seq=1,
    )
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(gateway, f"127.0.0.1:{port}")
    with grpc.insecure_channel(
        f"127.0.0.1:{port}", options=[("grpc.max_receive_message_length", MAX_FRAME_BYTES)],
    ) as channel:
        try:
            yield rpc.ObservationsStub(channel), system_rpc.SystemStub(channel), server
        finally:
            server.stop(0).wait(3)


def wait_clients(service, kind, count):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if service.traffic_snapshot()[kind]["sync_clients"] == count:
            return
        time.sleep(0.01)
    assert service.traffic_snapshot()[kind]["sync_clients"] == count


def frame(sequence=1, *, kind="map", points=None, epoch=3):
    return encode_pointcloud(
        np.array([[1, 2, 3]], dtype=np.float32) if points is None else points,
        frame_id="map", epoch=epoch, sequence=sequence, stamp_s=123.25, stream_kind=kind,
    )


def test_sync_subscriber_keeps_latest_and_close_wakes_waiter():
    service = viewer()
    subscription = service.cloud_subscribe_sync()
    assert service._cloud_ws.has_subscribers()
    try:
        assert subscription.read(timeout=0.01) is None
        for sequence in range(1, 100):
            service.publish_cloud_frame(frame(sequence))
        assert decode_pointcloud_frame(subscription.read(timeout=0.1)).sequence == 99
        with ThreadPoolExecutor(max_workers=1) as pool:
            waiting = pool.submit(subscription.read, 10)
            subscription.close()
            assert waiting.result(timeout=1) is None
    finally:
        subscription.close()
    assert not service._cloud_ws.has_subscribers()


def test_async_and_sync_consumers_share_cross_thread_publish():
    async def run():
        service = viewer()
        queue, _ = service.cloud_subscribe()
        subscription = service.cloud_subscribe_sync()
        data = frame()
        producer = threading.Thread(target=lambda: service.publish_cloud_frame(data))
        producer.start()
        try:
            assert await asyncio.wait_for(queue.get(), timeout=1) == data
            assert subscription.read(timeout=1) == data
        finally:
            producer.join(1)
            service.cloud_unsubscribe(queue)
            subscription.close()
        assert service.traffic_snapshot()["cloud"]["clients"] == 0
    asyncio.run(run())


def test_cached_map_then_changed_and_reset_frames_are_preserved():
    service = viewer()
    first = frame()
    service.publish_cloud_frame(first)
    with endpoint(service) as (stub, _, _):
        call = stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_MAP), timeout=3)
        assert next(call).pcld == first
        changed = frame(2)
        threading.Thread(target=lambda: service.publish_cloud_frame(changed)).start()
        assert next(call).pcld == changed
        reset = frame(3, kind="reset", points=np.empty((0, 3), dtype=np.float32), epoch=4)
        service.publish_cloud_frame(reset)
        assert next(call).pcld == reset
        call.cancel()
        wait_clients(service, "cloud", 0)


def test_cold_scan_subscription_activates_existing_lidar_source():
    service = viewer()
    scan = PointCloud2(points=np.array([[1, 2, 3]], dtype=np.float32), frame_id="map", ts=123.25)
    service.on_lidar_scan(scan)
    assert service.traffic_snapshot()["scan"]["published_frames"] == 0
    with endpoint(service) as (stub, _, _):
        call = stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_SCAN), timeout=3)
        call.initial_metadata()
        wait_clients(service, "scan", 1)
        producer = threading.Thread(target=lambda: service.on_lidar_scan(scan))
        producer.start()
        decoded = decode_pointcloud_frame(next(call).pcld)
        producer.join(1)
        np.testing.assert_allclose(decoded.points, scan.points, atol=0.01)
        assert (decoded.frame_id, decoded.stream_kind, decoded.stamp_s) == ("map", "scan", 123.25)
        call.cancel()
        wait_clients(service, "scan", 0)


def test_supported_scan_larger_than_four_mib_is_complete():
    service = viewer()
    data = frame(kind="scan", points=np.zeros((800_000, 3), dtype=np.float32))
    assert len(data) > 4 * 1024 * 1024
    service.publish_scan_frame(data)
    with endpoint(service) as (stub, _, _):
        call = stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_SCAN), timeout=3)
        assert next(call).pcld == data
        call.cancel()
        wait_clients(service, "scan", 0)


@pytest.mark.parametrize("kind", [pb.CLOUD_KIND_UNSPECIFIED, 99])
def test_invalid_kind_is_rejected(kind):
    with endpoint(viewer()) as (stub, _, _):
        with pytest.raises(grpc.RpcError) as error:
            next(stub.WatchPointCloud(pb.CloudRequest(kind=kind), timeout=2))
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT


def test_missing_source_and_oversized_frame_are_explicit():
    with endpoint(None) as (stub, _, _):
        with pytest.raises(grpc.RpcError) as error:
            next(stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_MAP), timeout=2))
        assert error.value.code() == grpc.StatusCode.UNAVAILABLE
    service = viewer()
    service.publish_cloud_frame(b"x" * (MAX_FRAME_BYTES + 1))
    with endpoint(service) as (stub, _, _):
        with pytest.raises(grpc.RpcError) as error:
            next(stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_MAP), timeout=2))
        assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
        wait_clients(service, "cloud", 0)


def test_clouds_share_stream_slots_and_server_stop_releases_subscriptions():
    service = viewer()
    with endpoint(service) as (stub, system, server):
        calls = [stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_MAP), timeout=4) for _ in range(8)]
        try:
            for call in calls:
                call.initial_metadata()
            wait_clients(service, "cloud", 8)
            with pytest.raises(grpc.RpcError) as error:
                next(stub.WatchCamera(common_pb.Empty(), timeout=2))
            assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
            assert system.Ping(common_pb.Empty(), timeout=2).status == "ok"
            calls[0].cancel()
            wait_clients(service, "cloud", 7)
            replacement = stub.WatchPointCloud(pb.CloudRequest(kind=pb.CLOUD_KIND_SCAN), timeout=2)
            replacement.initial_metadata()
            wait_clients(service, "scan", 1)
            replacement.cancel()
            server.stop(0).wait(2)
            wait_clients(service, "cloud", 0)
            wait_clients(service, "scan", 0)
        finally:
            for call in calls:
                call.cancel()
