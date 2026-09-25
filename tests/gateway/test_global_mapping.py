import json
import os
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from starlette.responses import JSONResponse

from gateway.maps import routes as map_routes
from gateway.services.global_mapping import global_mapping_points
from runtime.msgs.sensor import PointCloud2
from runtime.utils.binary_codec import decode_pointcloud_frame


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


@pytest.mark.parametrize("endpoint", ["/api/v1/map/global/points", "/api/v1/map/points"])
def test_map_response_encodes_off_the_teleop_event_loop(monkeypatch, endpoint):
    app = FastAPI()
    render_threads = []
    read_limits = []
    payload = {"source": "global_mapping_preview", "count": 2, "points": [[0., 0., -1.], [1., 2., 3.]]}

    def read_points(limit):
        read_limits.append(limit)
        return payload

    render = JSONResponse.render

    def traced_render(self, content):
        if isinstance(content, dict) and content.get("source") == "global_mapping_preview":
            render_threads.append(threading.get_ident())
        return render(self, content)

    @app.get("/event-loop-thread")
    async def event_loop_thread():
        return {"thread": threading.get_ident()}

    monkeypatch.setattr(map_routes, "global_mapping_points", read_points)
    monkeypatch.setattr(JSONResponse, "render", traced_render)
    gateway = SimpleNamespace(_cloud_viewer=SimpleNamespace(map_points_snapshot=lambda *, max_points: read_points(max_points)))
    map_routes.register_map_routes(app, gateway)
    with TestClient(app) as client:
        loop_thread = client.get("/event-loop-thread").json()["thread"]
        response = client.get(f"{endpoint}?max_points=120000")
    assert response.status_code == 200
    assert response.json() == payload
    assert read_limits == [120000]
    assert len(render_threads) == 1
    assert render_threads[0] != loop_thread


def test_binary_preview_preserves_coordinates_identity_and_quality(tmp_path, monkeypatch):
    status, metadata = setup_snapshot(tmp_path, monkeypatch)
    metadata.update(rejected_keyframes=7, loops=2)
    status.write_text(json.dumps(dict(source_epoch=42, global_mapping=metadata)))
    (tmp_path / "global_map_cloud.meta.json").write_text(json.dumps(metadata))
    app = FastAPI()
    map_routes.register_map_routes(app, SimpleNamespace())
    with TestClient(app) as client:
        response = client.get("/api/v1/map/global/points?max_points=2&format=binary")
        json_response = client.get("/api/v1/map/global/points?max_points=2")
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/octet-stream"
    assert response.headers["cache-control"] == "no-store"
    frame = decode_pointcloud_frame(response.content)
    np.testing.assert_allclose(frame.points, [[0, 0, 0], [3, 6, 8]], atol=.003)
    assert frame.frame_id == "map"
    assert frame.epoch == json_response.json()["epoch"]
    assert frame.sequence == 3
    assert frame.stamp_s == 12
    assert frame.stream_kind == "map"
    assert json.loads(response.headers["x-lingtu-global-mapping"])["rejected_keyframes"] == 7


def test_binary_preview_rejects_stale_snapshot_with_empty_frame(tmp_path, monkeypatch):
    status, _ = setup_snapshot(tmp_path, monkeypatch)
    os.utime(status, (time.time() - 10, time.time() - 10))
    frame, _ = map_routes.global_mapping_frame(120000)
    assert len(decode_pointcloud_frame(frame).points) == 0
