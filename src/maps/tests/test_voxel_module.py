from __future__ import annotations

import json

import numpy as np
import pytest

from maps.modules import voxel_grid
from maps.modules.voxel_grid import VoxelGridModule
from runtime.msgs.sensor import PointCloud2


class _FakeNativeVoxelLayer:
    instances: list[_FakeNativeVoxelLayer] = []

    def __init__(self, **config):
        self.config = dict(config)
        self.closed = False
        self.decayed = False
        self.reset_called = False
        self.updates: list[dict[str, object]] = []
        self._count = 2
        _FakeNativeVoxelLayer.instances.append(self)

    def close(self) -> None:
        self.closed = True

    def reset(self) -> None:
        self.reset_called = True
        self._count = 0

    def decay(self) -> None:
        self.decayed = True

    def update(self, points, *, frame_id, stamp_ns, origin_xyz) -> None:
        self.updates.append(
            {
                "shape": tuple(points.shape),
                "frame_id": frame_id,
                "stamp_ns": stamp_ns,
                "origin_xyz": tuple(origin_xyz),
            }
        )

    def voxel_count(self) -> int:
        return self._count

    def query_count(self, x: float, y: float, z: float) -> float:
        return 7.0

    def stats(self) -> dict[str, object]:
        return {
            "input_points": 3,
            "accepted_points": 2,
            "input_voxels": 2,
            "input_columns": 2,
            "carved_columns": 2,
            "carved_voxels": 1,
            "total_voxels": self._count,
            "accumulated_cells": self._count,
            "accumulated_occupied": self._count,
            "accumulated_generation": 3,
            "ray_updates": 2,
            "free_updates": 1,
            "hit_updates": 2,
            "pruned_cells": 0,
            "column_carving": bool(self.config["column_carving"]),
        }

    def snapshot_xyz(self):
        return np.array(
            [
                [0.5, 0.5, 0.5],
                [1.5, 0.5, 0.5],
            ],
            dtype=np.float32,
        )

    def scene_metadata(self) -> dict[str, object]:
        return {
            "live_voxels": self._count,
            "accumulated_cells": self._count,
            "accumulated_occupied": self._count,
            "accumulated_generation": 3,
            "live_points": self._count,
            "frame_stamp_ns": 1_250_000_000,
            "voxel_size_m": float(self.config["voxel_size"]),
            "localization_ok": True,
            "map_ok": True,
            "planner_ok": True,
            "status_bits": 0,
        }

    def validate_state(self, path) -> bool:
        return True

    def load_state(self, path) -> None:
        self.loaded_path = str(path)

    def save_state(self, path) -> None:
        self.saved_path = str(path)


def test_voxel_grid_cpp_backend_uses_native_adapter(monkeypatch: pytest.MonkeyPatch):
    _FakeNativeVoxelLayer.instances.clear()
    monkeypatch.setattr(voxel_grid, "NativeVoxelLayer", _FakeNativeVoxelLayer)

    module = VoxelGridModule(
        backend="cpp",
        voxel_size=1.0,
        max_range=10.0,
        min_z=-1.0,
        max_z=2.0,
        decay_rate=0.0,
        publish_interval=0.0,
        column_carving=True,
    )
    stats_payloads = []
    cloud_payloads = []
    module.voxel_map._add_callback(stats_payloads.append)
    module.voxel_cloud._add_callback(cloud_payloads.append)

    module.setup()
    points = np.array(
        [
            [0.1, 0.1, 0.1],
            [1.1, 0.1, 0.1],
            [100.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    module.map_cloud._deliver(PointCloud2.from_numpy(points, frame_id="/odom", ts=1.25))

    native = _FakeNativeVoxelLayer.instances[0]
    assert module._backend == "cpp"
    assert native.config["voxel_size"] == 1.0
    assert native.updates == [
        {
            "shape": (3, 3),
            "frame_id": "odom",
            "stamp_ns": 1_250_000_000,
            "origin_xyz": (0.0, 0.0, 0.0),
        }
    ]
    assert native.decayed is True

    stats = json.loads(module.get_voxel_stats())
    assert stats["backend"] == "cpp"
    assert stats["total_voxels"] == 2
    assert stats["accumulated_occupied"] == 2
    assert stats["ray_updates"] == 2
    assert stats["last_update"]["accepted_points"] == 2
    assert stats_payloads[0]["backend"] == "cpp"
    assert stats_payloads[0]["frame_id"] == "odom"
    assert stats_payloads[0]["accumulated_generation"] == 3
    assert cloud_payloads[0].frame_id == "odom"
    assert cloud_payloads[0].points.shape == (2, 3)

    query = json.loads(module.query_voxel(0.1, 0.1, 0.1))
    assert query["occupied"] is True
    assert query["count"] == 7.0

    assert json.loads(module.clear_voxels()) == {"cleared": 2}
    assert native.reset_called is True
    module.stop()
    assert native.closed is True


def test_voxel_grid_loads_and_checkpoints_native_state(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
):
    _FakeNativeVoxelLayer.instances.clear()
    monkeypatch.setattr(voxel_grid, "NativeVoxelLayer", _FakeNativeVoxelLayer)
    state_path = tmp_path / "voxel.ltbg"
    state_path.write_bytes(b"placeholder")

    module = VoxelGridModule(backend="cpp", voxel_size=1.0, state_path=str(state_path))
    module.setup()
    native = _FakeNativeVoxelLayer.instances[0]
    assert native.loaded_path == str(state_path)

    assert json.loads(module.checkpoint_voxels()) == {
        "saved": True,
        "path": str(state_path),
    }
    assert native.saved_path == str(state_path)
    module.stop()
    assert native.saved_path == str(state_path)


@pytest.mark.parametrize("backend", ["auto", "python"])
def test_voxel_grid_rejects_non_native_backends(backend: str):
    module = VoxelGridModule(backend=backend, voxel_size=1.0)
    with pytest.raises(ValueError, match="native-only"):
        module.setup()


def test_voxel_grid_cpp_backend_requires_native(
    monkeypatch: pytest.MonkeyPatch,
):
    class FailingNativeVoxelLayer:
        def __init__(self, **config):
            raise voxel_grid.VoxelNativeUnavailable("native test missing")

    monkeypatch.setattr(voxel_grid, "NativeVoxelLayer", FailingNativeVoxelLayer)

    module = VoxelGridModule(backend="cpp")
    with pytest.raises(RuntimeError, match="requires the native lingtu_maps"):
        module.setup()
