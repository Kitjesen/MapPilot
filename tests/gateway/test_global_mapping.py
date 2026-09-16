import json
import os
import time

import numpy as np

from gateway.services.global_mapping import global_mapping_points
from runtime.msgs.sensor import PointCloud2


def setup_snapshot(tmp_path, monkeypatch):
    status = tmp_path / "status.json"
    monkeypatch.setenv("LINGTU_SLAM_STATUS_JSON", str(status))
    monkeypatch.setenv("LINGTU_SLAM_CLOUD_SNAPSHOT_DIR", str(tmp_path))
    metadata = dict(state="accumulating", source_epoch=42, revision=3, points=4, stamp_s=12.0, frame_id="map")
    status.write_text(json.dumps(dict(source_epoch=42, global_mapping=metadata)))
    (tmp_path / "global_map_cloud.meta.json").write_text(json.dumps(metadata))
    points = np.array([[0, 0, 0], [1, 2, 3], [2, 4, -2], [3, 6, 8]], dtype=np.float32)
    cloud = PointCloud2(points=points, ts=12.0, frame_id="map")
    (tmp_path / "global_map_cloud.bin").write_bytes(cloud.encode())
    return status, metadata


def test_whole_map_sampling_retains_full_bounds(tmp_path, monkeypatch):
    setup_snapshot(tmp_path, monkeypatch)
    result = global_mapping_points(2)
    assert result["count"] == 2
    assert result["bounds"] == {"min": [0, 0, -2], "max": [3, 6, 8]}
    assert 0 < result["epoch"] <= 0xFFFFFFFF
    assert result["sequence"] == 3
    assert result["stream_kind"] == "map"


def test_restart_or_partial_snapshot_is_not_shown(tmp_path, monkeypatch):
    status, metadata = setup_snapshot(tmp_path, monkeypatch)
    status.write_text(json.dumps(dict(source_epoch=43, global_mapping=metadata)))
    assert global_mapping_points()["count"] == 0
    metadata["source_epoch"] = 43
    metadata["stamp_s"] = 13.0
    (tmp_path / "global_map_cloud.meta.json").write_text(json.dumps(metadata))
    status.write_text(json.dumps(dict(source_epoch=43, global_mapping=metadata)))
    assert global_mapping_points()["count"] == 0


def test_shutdown_snapshot_is_not_live(tmp_path, monkeypatch):
    status, _ = setup_snapshot(tmp_path, monkeypatch)
    os.utime(status, (time.time() - 10, time.time() - 10))
    assert global_mapping_points()["count"] == 0


def test_new_status_does_not_blank_previous_complete_revision(tmp_path, monkeypatch):
    status, metadata = setup_snapshot(tmp_path, monkeypatch)
    metadata["revision"] = 4
    metadata["stamp_s"] = 13.0
    status.write_text(json.dumps(dict(source_epoch=42, global_mapping=metadata)))
    result = global_mapping_points()
    assert result["count"] == 4
    assert result["sequence"] == 3
