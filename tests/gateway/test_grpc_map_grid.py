"""Read real native-format grid files through mapd handles and the SDK listener."""

import io
import os
import socket
import zipfile
from contextlib import contextmanager
from types import SimpleNamespace

import grpc
import numpy as np
import pytest

from gateway.rpc import map_grid
from gateway.rpc.generated import maps_pb2 as pb
from gateway.rpc.generated import maps_pb2_grpc as rpc
from gateway.rpc.server import start
from runtime.endpoints.mapd import ArtifactHandle


class Client:
    def __init__(self, path, *, metadata=None, declared_size=None):
        self.path = path
        self.metadata = metadata or {"map_id": "yard", "content_epoch": 17, "frame_id": "map"}
        self.declared_size = declared_size
        self.calls = []
        self.descriptors = []

    def open_artifact(self, map_id, capability):
        self.calls.append((map_id, capability))
        descriptor = os.open(self.path, os.O_RDONLY)
        self.descriptors.append(descriptor)
        return ArtifactHandle(
            metadata=self.metadata,
            size_bytes=self.path.stat().st_size if self.declared_size is None else self.declared_size,
            _descriptor=descriptor,
        )


@contextmanager
def serve(client):
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        address = f"127.0.0.1:{listener.getsockname()[1]}"
    server = start(SimpleNamespace(_map_client=client), address)
    with grpc.insecure_channel(address, options=[("grpc.max_receive_message_length", map_grid.MAX_ARTIFACT_BYTES)]) as channel:
        try:
            yield rpc.MapsStub(channel)
        finally:
            server.stop(0).wait(3)


def save(path, grid=None, *, resolution=0.2, origin=None, compressed=False):
    writer = np.savez_compressed if compressed else np.savez
    writer(
        path,
        grid=np.array([[-1, 0, 100]], dtype=np.int8) if grid is None else grid,
        resolution=np.array(resolution, dtype="<f8"),
        origin=np.array([-3.0, 4.0] if origin is None else origin, dtype="<f8"),
    )


def test_grid_preserves_identity_geometry_cells_and_closes_fd(tmp_path):
    path = tmp_path / "occupancy.npz"
    save(path)
    client = Client(path)
    with serve(client) as maps:
        grid = maps.Grid(pb.MapContentRequest(map_name="yard"), timeout=5)
    assert (grid.map_id, grid.content_epoch, grid.frame_id) == ("yard", 17, "map")
    assert (grid.width, grid.height, grid.resolution_m) == (3, 1, 0.2)
    assert (grid.origin.x, grid.origin.y, grid.origin.z, grid.origin.yaw) == (-3, 4, 0, 0)
    assert grid.cells == bytes([255, 0, 100])
    assert not grid.HasField("time")
    assert client.calls == [("yard", "path_planning_2d")]
    with pytest.raises(OSError):
        os.fstat(client.descriptors[0])


def test_native_maximum_grid_passes_the_grpc_boundary(tmp_path):
    path = tmp_path / "occupancy.npz"
    save(path, np.full((5000, 5000), -1, dtype=np.int8))
    with serve(Client(path)) as maps:
        grid = maps.Grid(pb.MapContentRequest(map_name="yard"), timeout=15)
    assert (grid.width, grid.height) == (5000, 5000)
    assert len(grid.cells) == map_grid.MAX_CELLS
    assert grid.cells[0] == grid.cells[-1] == 255


@pytest.mark.parametrize("parameters", [
    {"grid": np.array([[0.0]], dtype=np.float32)},
    {"grid": np.array([[-2]], dtype=np.int8)},
    {"grid": np.array([[101]], dtype=np.int8)},
    {"resolution": 0.0},
    {"origin": [float("nan"), 0]},
    {"compressed": True},
])
def test_invalid_native_grid_is_data_loss_and_closes_fd(tmp_path, parameters):
    path = tmp_path / "occupancy.npz"
    save(path, **parameters)
    client = Client(path)
    with serve(client) as maps:
        with pytest.raises(grpc.RpcError) as error:
            maps.Grid(pb.MapContentRequest(map_name="yard"), timeout=3)
    assert error.value.code() == grpc.StatusCode.DATA_LOSS
    with pytest.raises(OSError):
        os.fstat(client.descriptors[0])


def test_declared_array_size_is_checked_before_payload_allocation(tmp_path):
    path = tmp_path / "occupancy.npz"
    save(path)
    with zipfile.ZipFile(path) as archive:
        entries = {name: archive.read(name) for name in archive.namelist()}
    header = io.BytesIO()
    np.lib.format.write_array_header_1_0(header, {"descr": "|i1", "fortran_order": False, "shape": (50000, 50000)})
    entries["grid.npy"] = header.getvalue()
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED) as archive:
        for name, value in entries.items():
            archive.writestr(name, value)
    with serve(Client(path)) as maps:
        with pytest.raises(grpc.RpcError) as error:
            maps.Grid(pb.MapContentRequest(map_name="yard"), timeout=3)
    assert error.value.code() == grpc.StatusCode.DATA_LOSS


@pytest.mark.parametrize("declared_size,code", [
    (map_grid.MAX_ARTIFACT_BYTES + 1, grpc.StatusCode.RESOURCE_EXHAUSTED),
    (1, grpc.StatusCode.DATA_LOSS),
])
def test_artifact_size_is_bounded(tmp_path, declared_size, code):
    path = tmp_path / "occupancy.npz"
    save(path)
    client = Client(path, declared_size=declared_size)
    with serve(client) as maps:
        with pytest.raises(grpc.RpcError) as error:
            maps.Grid(pb.MapContentRequest(map_name="yard"), timeout=3)
    assert error.value.code() == code
    with pytest.raises(OSError):
        os.fstat(client.descriptors[0])


def test_invalid_identity_and_map_name(tmp_path):
    path = tmp_path / "occupancy.npz"
    save(path)
    client = Client(path, metadata={"map_id": "other", "frame_id": "map"})
    with serve(client) as maps:
        with pytest.raises(grpc.RpcError) as error:
            maps.Grid(pb.MapContentRequest(map_name="yard"), timeout=3)
        assert error.value.code() == grpc.StatusCode.DATA_LOSS
        with pytest.raises(grpc.RpcError) as error:
            maps.Grid(pb.MapContentRequest(map_name="../yard"), timeout=3)
        assert error.value.code() == grpc.StatusCode.INVALID_ARGUMENT
    assert len(client.calls) == 1
