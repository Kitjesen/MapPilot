from __future__ import annotations

import pytest

from runtime.msgs.map import (
    MAP_CLOUD_FRAME_SCHEMA,
    MapCloudFrame,
)
from runtime.msgs.numpy_compat import np
from runtime.msgs.protocol import resolve_msg_type
from runtime.msgs.sensor import PointCloud2


def test_map_cloud_frame_encode_round_trip():
    frame = MapCloudFrame(
        points=np.array([[1.0, 2.0, 0.1, 7.0]], dtype=np.float32),
        mode="keyframe",
        frame_id="map",
        map_id="field_01",
        source="slam",
        sequence=3,
        metadata={"quality": "ok"},
    )

    decoded = MapCloudFrame.decode(frame.encode())

    assert decoded.schema_version == MAP_CLOUD_FRAME_SCHEMA
    assert decoded.mode == "KEYFRAME"
    assert decoded.frame_id == "map"
    assert decoded.map_id == "field_01"
    assert decoded.source == "slam"
    assert decoded.sequence == 3
    assert decoded.metadata == {"quality": "ok"}
    assert decoded.points.shape == (1, 4)
    assert decoded.points.dtype == np.float32


def test_map_cloud_frame_rejects_unknown_mode():
    with pytest.raises(ValueError, match="mode must be one of"):
        MapCloudFrame(points=[[0.0, 0.0, 0.0]], mode="delta")


def test_map_cloud_frame_from_pointcloud2_and_resolve_type():
    cloud = PointCloud2.from_numpy(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        frame_id="map",
    )

    frame = MapCloudFrame.from_pointcloud2(cloud, mode="incremental")

    assert frame.mode == "INCREMENTAL"
    assert frame.frame_id == "map"
    assert frame.finite_xyz().shape == (1, 3)
    assert resolve_msg_type("map.MapCloudFrame") is MapCloudFrame
