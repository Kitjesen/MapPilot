"""Verify typed cache projections and continuous subscriptions without hardware."""

import socket
import threading
import time
from types import SimpleNamespace

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc import telemetry
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import localization_pb2_grpc as localization_rpc
from gateway.rpc.generated import navigation_pb2_grpc as navigation_rpc
from gateway.rpc.server import start


class Abort(Exception):
    def __init__(self, code):
        self.code = code


def abort(code, _message):
    raise Abort(code)


def test_map_pose_preserves_cache_fields():
    source = {"frame_id": "map", "x": 1, "y": 2, "z": 3, "yaw": 0.4, "ts": 123.0}
    gateway = SimpleNamespace(_state_lock=threading.Lock(), _odom=source)
    sample = telemetry.map_pose(gateway, SimpleNamespace(abort=abort))
    assert (sample.pose.x, sample.pose.y, sample.pose.z, sample.pose.yaw) == (1, 2, 3, 0.4)
    assert sample.frame_id == "map"
    assert sample.details["ts"] == 123.0
    assert not sample.HasField("received_age_ms")
    assert not sample.HasField("fresh")
    assert source == gateway._odom


def test_pose_freshness_uses_service_receipt_time(monkeypatch):
    monkeypatch.setattr(telemetry.time, "time", lambda: 100.0)
    source = {"frame_id": "map", "x": 1, "y": 2, "z": 3, "yaw": 0.4, "ts": 1.0}
    gateway = SimpleNamespace(_state_lock=threading.Lock(), _odom=source, _odom_timestamps=[99.0])
    context = SimpleNamespace(abort=abort)
    sample = telemetry.map_pose(gateway, context, require_fresh=True)
    assert sample.received_age_ms == 1000
    assert sample.fresh is True
    assert sample.details["ts"] == 1.0
    gateway._odom_timestamps = [97.0]
    assert telemetry.map_pose(gateway, context).fresh is False
    with pytest.raises(Abort) as error:
        telemetry.map_pose(gateway, context, require_fresh=True)
    assert error.value.code == grpc.StatusCode.UNAVAILABLE
    gateway._odom_timestamps = [99.0]
    gateway._localization_status = {"state": "LOST"}
    with pytest.raises(Abort):
        telemetry.map_pose(gateway, context, require_fresh=True)


@pytest.mark.parametrize("source", [None, {"frame_id": "odom"}, {"frame_id": "map", "x": float("nan")}])
def test_map_pose_unavailable(source):
    gateway = SimpleNamespace(_state_lock=threading.Lock(), _odom=source)
    with pytest.raises(Abort) as error:
        telemetry.map_pose(gateway, SimpleNamespace(abort=abort))
    assert error.value.code == grpc.StatusCode.UNAVAILABLE


def test_route_preserves_xyz_and_empty_clear():
    gateway = SimpleNamespace(_state_lock=threading.Lock(), _last_path=[{"x": 1, "y": 2, "z": 3, "frame_id": "map"}])
    route = telemetry.navigation_path(gateway, SimpleNamespace(abort=abort))
    assert [(point.x, point.y, point.z) for point in route.points] == [(1, 2, 3)]
    assert route.frame_id == "map"
    assert not route.HasField("time")
    gateway._last_path = []
    assert not telemetry.navigation_path(gateway, SimpleNamespace(abort=abort)).points


def test_route_rejects_mixed_frames():
    gateway = SimpleNamespace(_state_lock=threading.Lock(), _last_path=[
        {"x": 1, "y": 2, "frame_id": "map"}, {"x": 3, "y": 4, "frame_id": "odom"},
    ])
    with pytest.raises(Abort) as error:
        telemetry.navigation_path(gateway, SimpleNamespace(abort=abort))
    assert error.value.code == grpc.StatusCode.UNAVAILABLE


@pytest.mark.parametrize("point", [
    {"frame_id": "map", "x": 1, "y": 2},
    {"frame_id": "map", "x": 1, "y": 2, "z": float("nan")},
    {"x": 1, "y": 2, "z": 3},
])
def test_route_rejects_incomplete_xyz_or_frame(point):
    gateway = SimpleNamespace(_state_lock=threading.Lock(), _last_path=[point])
    with pytest.raises(Abort) as error:
        telemetry.navigation_path(gateway, SimpleNamespace(abort=abort))
    assert error.value.code == grpc.StatusCode.UNAVAILABLE


def test_watch_releases_slot_and_accounts_for_processing(monkeypatch):
    waits = []
    stop = SimpleNamespace(set=lambda: None, wait=lambda interval: waits.append(interval) or True)
    ticks = iter([0.0, 0.004])
    monkeypatch.setattr(telemetry, "threading", SimpleNamespace(Event=lambda: stop))
    monkeypatch.setattr(telemetry, "time", SimpleNamespace(monotonic=lambda: next(ticks)))
    slots = threading.BoundedSemaphore(1)
    context = SimpleNamespace(add_callback=lambda callback: True, is_active=lambda: True)
    assert list(telemetry.watch(context, slots, lambda: "sample")) == ["sample"]
    assert waits == [pytest.approx(0.006)]
    assert slots.acquire(blocking=False)


def test_cancelled_watch_does_not_read():
    slots = threading.BoundedSemaphore(1)
    context = SimpleNamespace(add_callback=lambda callback: False)
    assert list(telemetry.watch(context, slots, lambda: pytest.fail("cancelled watch read cache"))) == []
    assert slots.acquire(blocking=False)


def test_rpc_streams_deliver_cache_updates(monkeypatch):
    from gateway.rpc import services

    def full_snapshot(_gateway):
        pytest.fail("pose subscription read full system snapshot")

    monkeypatch.setattr(services, "build_state_snapshot", full_snapshot)
    gateway = GatewayModule()
    gateway._odom = {"frame_id": "map", "x": 1, "y": 2, "z": 3, "yaw": 0.4}
    gateway._last_path = [{"frame_id": "map", "x": 1, "y": 2, "z": 3}]
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(gateway, address)
    channel = grpc.insecure_channel(address)
    pose_stream = path_stream = None
    try:
        grpc.channel_ready_future(channel).result(timeout=3)
        pose_stream = localization_rpc.LocalizationStub(channel).WatchPose(common_pb.Empty(), timeout=3)
        assert next(pose_stream).pose.x == 1
        with gateway._state_lock:
            gateway._odom = {**gateway._odom, "x": 4}
        deadline = time.monotonic() + 1
        while next(pose_stream).pose.x != 4:
            assert time.monotonic() < deadline
        navigation = navigation_rpc.NavigationStub(channel)
        assert navigation.Route(common_pb.Empty(), timeout=3).points[0].z == 3
        path_stream = navigation.WatchPath(common_pb.Empty(), timeout=3)
        assert len(next(path_stream).points) == 1
        with gateway._state_lock:
            gateway._last_path = []
        deadline = time.monotonic() + 1
        while next(path_stream).points:
            assert time.monotonic() < deadline
    finally:
        if pose_stream is not None:
            pose_stream.cancel()
        if path_stream is not None:
            path_stream.cancel()
        channel.close()
        server.stop(0).wait(3)
