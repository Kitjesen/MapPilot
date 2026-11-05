"""Transfer mapd artifacts through the SDK listener without loading whole files."""

import os
import socket
import threading
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import pytest

from gateway.rpc.generated import common_pb2 as common_pb
from gateway.rpc.generated import error_pb2 as error_pb
from gateway.rpc.generated import maps_pb2 as pb
from gateway.rpc.generated import maps_pb2_grpc as rpc
from gateway.rpc.generated import system_pb2_grpc as system_rpc
from gateway.rpc.server import start
from runtime.endpoints.mapd import ArtifactHandle, MapClientError


@contextmanager
def serve(gateway):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]
    server = start(gateway, f"127.0.0.1:{port}")
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        try:
            yield channel
        finally:
            server.stop(0).wait(3)


class TrackedChunks:
    def __init__(self, chunks, closed):
        self.chunks = chunks
        self.closed = closed

    def __iter__(self):
        return self

    def __next__(self):
        return next(self.chunks)

    def close(self):
        self.chunks.close()
        self.closed.set()


class TrackedArtifact(ArtifactHandle):
    def __init__(self, *, closed, **kwargs):
        super().__init__(**kwargs)
        self.closed = closed

    def iter_bytes(self):
        return TrackedChunks(super().iter_bytes(), self.closed)


class Client:
    def __init__(self, path, *, metadata=None, declared_size=None):
        self.path = path
        self.metadata = metadata if metadata is not None else {"map_id": "yard", "content_epoch": 17, "frame_id": "map"}
        self.declared_size = declared_size
        self.calls = []
        self.closed = []
        self.descriptors = []

    def open_artifact(self, map_id, capability):
        self.calls.append((map_id, capability))
        closed = threading.Event()
        self.closed.append(closed)
        descriptor = os.open(self.path, os.O_RDONLY)
        self.descriptors.append(descriptor)
        return TrackedArtifact(
            closed=closed,
            metadata=self.metadata,
            size_bytes=self.path.stat().st_size if self.declared_size is None else self.declared_size,
            _descriptor=descriptor,
        )


def reason(error):
    return error_pb.ErrorDetail.FromString(dict(error.trailing_metadata())["doso-error-bin"]).reason_code


def test_large_map_preserves_identity_bytes_and_closes_descriptor(tmp_path):
    payload = b"VERSION .7\nDATA binary\n" + bytes(range(256)) * 20000
    path = tmp_path / "map.pcd"
    path.write_bytes(payload)
    client = Client(path)
    with serve(SimpleNamespace(_map_client=client)) as channel:
        replies = list(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name="yard"), timeout=5))
    assert replies[0].WhichOneof("value") == "info"
    assert replies[0].info == pb.MapContentInfo(map_id="yard", content_epoch=17, frame_id="map", size_bytes=len(payload))
    assert all(reply.WhichOneof("value") == "chunk" and len(reply.chunk) <= 1024 * 1024 for reply in replies[1:])
    assert b"".join(reply.chunk for reply in replies[1:]) == payload
    assert len(payload) > 4 * 1024 * 1024
    assert client.calls == [("yard", "source_pointcloud")]
    assert client.closed[0].is_set()
    with pytest.raises(OSError):
        os.fstat(client.descriptors[0])


def test_empty_map_has_info_and_absent_optional_metadata(tmp_path):
    path = tmp_path / "empty.pcd"
    path.touch()
    client = Client(path, metadata={"map_id": "yard"})
    with serve(SimpleNamespace(_map_client=client)) as channel:
        replies = list(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name="yard"), timeout=3))
    assert len(replies) == 1
    assert replies[0].info.size_bytes == 0
    assert not replies[0].info.HasField("content_epoch")
    assert not replies[0].info.HasField("frame_id")
    assert client.closed[0].is_set()


@pytest.mark.parametrize("metadata", [
    {"map_id": "another-map"},
    {"map_id": "yard", "content_epoch": -1},
])
def test_invalid_identity_or_metadata_fails_before_content(tmp_path, metadata):
    path = tmp_path / "map.pcd"
    path.write_bytes(b"map bytes")
    client = Client(path, metadata=metadata)
    with serve(SimpleNamespace(_map_client=client)) as channel:
        with pytest.raises(grpc.RpcError) as failure:
            next(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name="yard"), timeout=3))
    assert failure.value.code() == grpc.StatusCode.DATA_LOSS
    assert reason(failure.value) == "invalid_map_artifact"
    with pytest.raises(OSError):
        os.fstat(client.descriptors[0])


@pytest.mark.parametrize("source_reason,code", [
    ("map_not_found", grpc.StatusCode.NOT_FOUND),
    ("missing_artifact", grpc.StatusCode.NOT_FOUND),
    ("map_write_in_progress", grpc.StatusCode.FAILED_PRECONDITION),
    ("mapd_unavailable", grpc.StatusCode.UNAVAILABLE),
])
def test_owner_errors_keep_reason(source_reason, code):
    class FailingClient:
        def open_artifact(self, map_id, capability):
            raise MapClientError(source_reason, "map source failure")

    with serve(SimpleNamespace(_map_client=FailingClient())) as channel:
        with pytest.raises(grpc.RpcError) as failure:
            list(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name="yard"), timeout=3))
    assert failure.value.code() == code
    assert reason(failure.value) == source_reason


@pytest.mark.parametrize("name", ["", "../yard"])
def test_invalid_map_name_is_rejected(name):
    with serve(SimpleNamespace()) as channel:
        with pytest.raises(grpc.RpcError) as failure:
            list(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name=name), timeout=3))
    assert failure.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert reason(failure.value) == "invalid_map_id"


@pytest.mark.parametrize("delta", [-1, 1])
def test_size_mismatch_fails_and_closes(tmp_path, delta):
    path = tmp_path / "map.pcd"
    path.write_bytes(b"map bytes")
    client = Client(path, declared_size=path.stat().st_size + delta)
    with serve(SimpleNamespace(_map_client=client)) as channel:
        with pytest.raises(grpc.RpcError) as failure:
            list(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name="yard"), timeout=3))
    assert failure.value.code() == grpc.StatusCode.DATA_LOSS
    assert reason(failure.value) == "artifact_size_mismatch"
    assert client.closed[0].is_set()


def test_cancellation_releases_shared_slots_and_files(tmp_path):
    path = tmp_path / "large.pcd"
    with path.open("wb") as output:
        output.truncate(64 * 1024 * 1024)
    client = Client(path)
    with serve(SimpleNamespace(_map_client=client)) as channel:
        stub = rpc.MapsStub(channel)
        calls = [stub.PointCloud(pb.MapContentRequest(map_name="yard"), timeout=10) for _ in range(8)]
        for call in calls:
            assert next(call).WhichOneof("value") == "info"
        with pytest.raises(grpc.RpcError) as failure:
            next(stub.PointCloud(pb.MapContentRequest(map_name="yard"), timeout=3))
        assert failure.value.code() == grpc.StatusCode.RESOURCE_EXHAUSTED
        assert system_rpc.SystemStub(channel).Ping(common_pb.Empty(), timeout=3).status == "ok"
        for call in calls:
            call.cancel()
        for closed in client.closed:
            assert closed.wait(3)
        followup = stub.PointCloud(pb.MapContentRequest(map_name="yard"), timeout=3)
        assert next(followup).WhichOneof("value") == "info"
        followup.cancel()
        assert client.closed[-1].wait(3)
    for descriptor in client.descriptors:
        with pytest.raises(OSError):
            os.fstat(descriptor)


def test_map_client_uses_current_deadline_despite_cached_query_client(monkeypatch, tmp_path):
    from gateway.rpc import map_content

    path = tmp_path / "map.pcd"
    path.touch()
    fake = Client(path)
    selected = map_content.MapClient("/saved/mapd.sock", timeout_s=360)
    gateway = SimpleNamespace(_map_query_client=selected)
    calls = []

    def open_artifact(client, name, capability):
        calls.append((client.socket_path, client.timeout_s))
        return fake.open_artifact(name, capability)

    monkeypatch.setattr(map_content.MapClient, "open_artifact", open_artifact)
    with serve(gateway) as channel:
        list(rpc.MapsStub(channel).PointCloud(pb.MapContentRequest(map_name="yard"), timeout=0.5))
    assert calls[0][0] == "/saved/mapd.sock"
    assert 0 < calls[0][1] <= 0.5
