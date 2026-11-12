"""Exercise geographic SDK RPCs against the existing GNSS snapshot reader."""

import json
import socket
import threading
import time
from types import SimpleNamespace

import grpc
import pytest

from gateway.gateway_module import GatewayModule
from gateway.rpc import server as rpc_server
from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import localization_pb2 as pb
from gateway.rpc.generated import localization_pb2_grpc as rpc
from gateway.services import geography

KEY = (("x-api-key", "location-test-key"),)
NOW = 1_793_700_100.0


def write_fix(path, *, timestamp=NOW - 0.125, accuracy=None, fix_type=4):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps({"location": {
        "lat": 31.2304, "lon": 121.4737, "fix_type": fix_type,
        "ts": timestamp, "accuracy_m": accuracy,
    }}), encoding="utf-8")
    temporary.replace(path)


@pytest.fixture
def source(tmp_path, monkeypatch):
    path = tmp_path / "gnss.json"
    monkeypatch.setenv("LINGTU_GNSS_STATUS_FILE", str(path))
    monkeypatch.setattr(geography, "time", SimpleNamespace(time=lambda: NOW))
    return path


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setenv("LINGTU_GRPC_API_KEY", KEY[0][1])
    monkeypatch.delenv("LINGTU_GRPC_TLS_CERT_FILE", raising=False)
    monkeypatch.delenv("LINGTU_GRPC_TLS_KEY_FILE", raising=False)
    slots = threading.BoundedSemaphore(1)
    monkeypatch.setattr(rpc_server, "BoundedSemaphore", lambda _count: slots)
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = rpc_server.start(GatewayModule(), address)
    with grpc.insecure_channel(address) as channel:
        try:
            grpc.channel_ready_future(channel).result(timeout=3)
            yield rpc.LocalizationStub(channel), slots
        finally:
            server.stop(0).wait(3)


@pytest.mark.parametrize("accuracy", [None, 0.0, 0.25])
def test_location_preserves_wgs84_source_time_and_optional_accuracy(source, client, accuracy):
    write_fix(source, accuracy=accuracy)
    stub, _slots = client
    result = stub.Location(common_pb.Empty(), metadata=KEY, timeout=3)
    assert result.state == pb.GEOGRAPHIC_STATE_READY
    assert result.age_s == 0.125
    assert (result.fix.latitude, result.fix.longitude, result.fix.fix_type) == (31.2304, 121.4737, 4)
    assert (result.fix.observed_at.seconds, result.fix.observed_at.nanos) == (int(NOW) - 1, 875_000_000)
    assert result.fix.HasField("accuracy_m") is (accuracy is not None)
    if accuracy is not None:
        assert result.fix.accuracy_m == accuracy


@pytest.mark.parametrize(("scenario", "state", "age"), [
    ("missing", pb.GEOGRAPHIC_STATE_UNAVAILABLE, None),
    ("no_fix", pb.GEOGRAPHIC_STATE_NO_FIX, None),
    ("stale", pb.GEOGRAPHIC_STATE_STALE, 4.0),
    ("invalid", pb.GEOGRAPHIC_STATE_INVALID, None),
])
def test_location_reports_unavailable_fix_without_coordinates(source, client, scenario, state, age):
    if scenario == "no_fix":
        write_fix(source, fix_type=0)
    elif scenario == "stale":
        write_fix(source, timestamp=NOW - 4.0)
    elif scenario == "invalid":
        source.write_text("invalid JSON", encoding="utf-8")
    stub, _slots = client
    result = stub.Location(common_pb.Empty(), metadata=KEY, timeout=3)
    assert result.state == state
    assert not result.HasField("fix")
    assert result.HasField("age_s") is (age is not None)
    if age is not None:
        assert result.age_s == age


def test_location_watch_repeats_at_one_hz_and_recovers_after_lost_fix(source, client):
    write_fix(source)
    stub, _slots = client
    stream = stub.WatchLocation(common_pb.Empty(), metadata=KEY, timeout=8)
    try:
        first = next(stream)
        assert first.state == pb.GEOGRAPHIC_STATE_READY
        started = time.monotonic()
        second = next(stream)
        assert second == first
        assert time.monotonic() - started >= 0.75
        for timestamp, fix_type, state in [
            (NOW - 0.125, 0, pb.GEOGRAPHIC_STATE_NO_FIX),
            (NOW - 4.0, 4, pb.GEOGRAPHIC_STATE_STALE),
            (NOW - 0.125, 4, pb.GEOGRAPHIC_STATE_READY),
        ]:
            write_fix(source, timestamp=timestamp, fix_type=fix_type)
            result = next(stream)
            assert result.state == state
            assert result.HasField("fix") is (state == pb.GEOGRAPHIC_STATE_READY)
    finally:
        stream.cancel()


def test_watch_cancellation_releases_shared_slot(source, client):
    write_fix(source)
    stub, slots = client
    stream = stub.WatchLocation(common_pb.Empty(), metadata=KEY, timeout=5)
    try:
        assert next(stream).state == pb.GEOGRAPHIC_STATE_READY
        denied = stub.WatchPose(common_pb.Empty(), metadata=KEY, timeout=3)
        with pytest.raises(grpc.RpcError) as error:
            next(denied)
        assert error.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
        assert stub.Location(common_pb.Empty(), metadata=KEY, timeout=3).HasField("fix")
    finally:
        stream.cancel()
    deadline = time.monotonic() + 2
    while not slots.acquire(blocking=False):
        assert time.monotonic() < deadline
        time.sleep(0.01)
    slots.release()
    recovered = stub.WatchLocation(common_pb.Empty(), metadata=KEY, timeout=3)
    try:
        assert next(recovered).state == pb.GEOGRAPHIC_STATE_READY
    finally:
        recovered.cancel()


@pytest.mark.parametrize("method", ["Location", "WatchLocation"])
def test_location_requires_shared_rpc_credentials(source, client, monkeypatch, method):
    def forbidden_read():
        pytest.fail("unauthenticated location call read GNSS")

    monkeypatch.setattr(geography, "geographic_location", forbidden_read)
    stub, _slots = client
    for metadata in [(), (("x-api-key", "wrong"),)]:
        with pytest.raises(grpc.RpcError) as error:
            call = getattr(stub, method)(common_pb.Empty(), metadata=metadata, timeout=3)
            if method == "WatchLocation":
                next(call)
        assert error.value.code() == grpc.StatusCode.UNAUTHENTICATED
