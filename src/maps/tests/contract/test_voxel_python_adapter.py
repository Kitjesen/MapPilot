from __future__ import annotations

import ctypes
import os
from pathlib import Path

import pytest

from maps.adapters.python import voxel as voxel_adapter


def test_native_voxel_adapter_reports_missing_library(monkeypatch: pytest.MonkeyPatch):
    missing = Path("missing").resolve()
    monkeypatch.setenv("LINGTU_MAPS_LIB", str(missing))
    monkeypatch.setattr(voxel_adapter, "_native_library_candidates", lambda: [missing])
    monkeypatch.setattr(voxel_adapter.ctypes.util, "find_library", lambda name: None)
    voxel_adapter._load_native_voxel_lib.cache_clear()
    try:
        assert voxel_adapter.native_voxel_available() is False
        with pytest.raises(voxel_adapter.VoxelNativeUnavailable):
            voxel_adapter.NativeVoxelLayer()
    finally:
        voxel_adapter._load_native_voxel_lib.cache_clear()


def test_native_voxel_adapter_ctypes_signature_contract(monkeypatch: pytest.MonkeyPatch):
    class FakeFunc:
        def __init__(self, result=0):
            self.argtypes = None
            self.restype = None
            self.result = result

        def __call__(self, *args):
            return self.result

    class FakeLib:
        def __init__(self):
            self.lingtu_maps_abi_version = FakeFunc(1)
            self.lingtu_maps_voxel_create = FakeFunc(123)
            self.lingtu_maps_voxel_destroy = FakeFunc(None)
            self.lingtu_maps_voxel_reset = FakeFunc(0)
            self.lingtu_maps_voxel_decay = FakeFunc(0)
            self.lingtu_maps_voxel_update_xyz_interleaved = FakeFunc(0)
            self.lingtu_maps_voxel_update_xyzi_interleaved = FakeFunc(0)
            self.lingtu_maps_voxel_update_xyz_soa = FakeFunc(0)
            self.lingtu_maps_voxel_count = FakeFunc(0)
            self.lingtu_maps_voxel_contains = FakeFunc(0)
            self.lingtu_maps_voxel_query_count = FakeFunc(0)
            self.lingtu_maps_voxel_stats = FakeFunc(0)
            self.lingtu_maps_voxel_snapshot_size = FakeFunc(0)
            self.lingtu_maps_voxel_snapshot_xyz_soa = FakeFunc(0)
            self.lingtu_maps_voxel_snapshot_occupied_xyz_soa = FakeFunc(0)
            self.lingtu_maps_voxel_scene_stats = FakeFunc(0)
            self.lingtu_maps_voxel_save_binary = FakeFunc(0)
            self.lingtu_maps_voxel_load_binary = FakeFunc(0)
            self.lingtu_maps_voxel_validate_binary = FakeFunc(0)

    fake = FakeLib()
    monkeypatch.setattr(ctypes, "CDLL", lambda path: fake)
    monkeypatch.setattr(voxel_adapter, "_native_library_candidates", lambda: [Path(__file__)])
    voxel_adapter._load_native_voxel_lib.cache_clear()
    try:
        assert voxel_adapter.native_voxel_available() is True
        assert fake.lingtu_maps_voxel_create.argtypes == [ctypes.POINTER(voxel_adapter._VoxelConfig)]
        assert fake.lingtu_maps_voxel_update_xyz_interleaved.restype is ctypes.c_int32
        assert fake.lingtu_maps_voxel_snapshot_xyz_soa.argtypes[-1] == ctypes.POINTER(ctypes.c_uint64)
        assert fake.lingtu_maps_voxel_save_binary.argtypes == [
            ctypes.c_void_p,
            ctypes.c_char_p,
        ]
        assert fake.lingtu_maps_voxel_scene_stats.argtypes == [
            ctypes.c_void_p,
            ctypes.POINTER(voxel_adapter._VoxelSceneStats),
        ]
    finally:
        voxel_adapter._load_native_voxel_lib.cache_clear()


def test_native_voxel_adapter_roundtrip_with_real_library(
    monkeypatch: pytest.MonkeyPatch,
):
    lib_path = os.environ.get("LINGTU_MAPS_LIB")
    if not lib_path or not Path(lib_path).is_file():
        pytest.skip("LINGTU_MAPS_LIB is not set to a built lingtu_maps library")

    voxel_adapter._load_native_voxel_lib.cache_clear()
    try:
        layer = voxel_adapter.NativeVoxelLayer(
            voxel_size=1.0,
            max_range=10.0,
            min_z=-1.0,
            max_z=2.0,
            decay_rate=0.0,
            column_carving=True,
        )
        points = voxel_adapter.np.array(
            [
                [0.1, 0.1, 0.1],
                [0.2, 0.2, 0.2],
                [1.2, 0.1, 0.1],
            ],
            dtype=voxel_adapter.np.float32,
        )
        layer.update(points, frame_id="map", stamp_ns=123, origin_xyz=(0, 0, 0))

        stats = layer.stats()
        assert stats["accepted_points"] == 3
        assert stats["total_voxels"] == 2
        assert stats["accumulated_occupied"] >= 2
        assert layer.query_count(0.1, 0.1, 0.1) == 2.0
        assert layer.snapshot_xyz().shape == (2, 3)
        metadata = layer.scene_metadata()
        assert metadata["accumulated_occupied"] >= 2
    finally:
        if "layer" in locals():
            layer.close()
        voxel_adapter._load_native_voxel_lib.cache_clear()
