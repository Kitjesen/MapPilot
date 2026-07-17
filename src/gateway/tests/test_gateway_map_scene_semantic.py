from __future__ import annotations

import asyncio

from gateway.services.cloud_viewer import CloudViewerService
from runtime.msgs.map import MapSceneFrame
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.utils.binary_codec import decode_pointcloud


def _queue_put_latest(q, buf, loop, record_delivery):
    try:
        q.put_nowait(buf)
        record_delivery(False, q.qsize())
    except asyncio.QueueFull:
        _ = q.get_nowait()
        q.put_nowait(buf)
        record_delivery(True, q.qsize())


def _service(events: list[dict]) -> CloudViewerService:
    svc = CloudViewerService(
        queue_put_latest=_queue_put_latest,
        current_loop=lambda: None,
        push_event=events.append,
        session_mode=lambda: "navigating",
        product_session=lambda: "navigation",
        active_session_map=lambda: "field",
        saved_active_map=lambda: "field",
    )
    svc.configure(
        map_voxel_size=0.02,
        cloud_viewer_min_interval_s=0.0,
        scan_viewer_min_interval_s=0.0,
    )
    return svc


def _latest_cloud_bytes(svc: CloudViewerService) -> bytes:
    q, latest = svc.cloud_subscribe()
    svc.cloud_unsubscribe(q)
    assert latest is not None
    return latest


def test_map_scene_semantic_labels_color_pointcloud_and_emit_metadata() -> None:
    events: list[dict] = []
    svc = _service(events)
    cloud = PointCloud2.from_numpy(
        np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
            ],
            dtype=np.float32,
        ),
        frame_id="map",
    )
    frame = MapSceneFrame(
        frame_id="map",
        source="maps.semantic",
        sequence=9,
        layers=[
            {
                "id": "maps.semantic_occupancy",
                "type": "pointcloud",
                "source": "maps.semantic",
                "payload": cloud,
                "labels": [1, 2, 1],
                "confidence": [0.9, 0.7, 0.8],
                "taxonomy": "lingtu.semantic",
                "taxonomy_version": 1,
                "palette": {
                    "1": {"name": "ground", "color": "#102030"},
                    "2": {"name": "vegetation", "color": "#40a020"},
                },
            }
        ],
    )

    svc.on_map_scene(frame)

    _points, colors = decode_pointcloud(_latest_cloud_bytes(svc))
    assert colors is not None
    assert colors.tolist() == [[16, 32, 48], [64, 160, 32], [16, 32, 48]]
    assert svc.latest_cloud_metadata()["semantic_colors"] is True
    assert events[-1]["type"] == "map_scene"
    assert events[-1]["consumed_pointcloud_layers"] == 1
    layer = events[-1]["layers"][0]
    assert "payload" not in layer
    assert layer["labels"] == [1, 2, 1]
    assert layer["confidence"] == [0.9, 0.7, 0.8]
    assert layer["taxonomy"] == "lingtu.semantic"
    assert layer["palette"][1]["color"] == "#102030"


def test_map_scene_without_labels_remains_geometry_only() -> None:
    events: list[dict] = []
    svc = _service(events)
    cloud = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32),
        frame_id="map",
    )

    svc.on_map_scene(
        MapSceneFrame(
            frame_id="map",
            source="maps.voxel",
            layers=[
                {
                    "id": "maps.voxel_cloud",
                    "type": "pointcloud",
                    "payload": cloud,
                }
            ],
        )
    )

    _points, colors = decode_pointcloud(_latest_cloud_bytes(svc))
    assert colors is None
    assert svc.latest_cloud_metadata()["semantic_colors"] is False
    assert events[-1]["layers"][0]["has_labels"] is False


def test_map_scene_keeps_semantic_colors_aligned_when_invalid_points_are_filtered() -> None:
    events: list[dict] = []
    svc = _service(events)
    cloud = PointCloud2.from_numpy(
        np.array(
            [[0.0, 0.0, 0.0], [np.nan, 1.0, 0.0], [2.0, 0.0, 0.0]],
            dtype=np.float32,
        ),
        frame_id="map",
    )

    svc.on_map_scene(
        MapSceneFrame(
            frame_id="map",
            source="maps.semantic",
            layers=[
                {
                    "id": "maps.semantic_occupancy",
                    "type": "pointcloud",
                    "payload": cloud,
                    "labels": [1, 2, 3],
                    "palette": {
                        "1": {"color": "#102030"},
                        "2": {"color": "#405060"},
                        "3": {"color": "#708090"},
                    },
                }
            ],
        )
    )

    points, colors = decode_pointcloud(_latest_cloud_bytes(svc))
    assert points.shape == (2, 3)
    assert colors is not None
    assert colors.tolist() == [[16, 32, 48], [112, 128, 144]]


def test_map_scene_keeps_semantic_colors_aligned_through_voxel_downsampling() -> None:
    events: list[dict] = []
    svc = _service(events)
    cloud = PointCloud2.from_numpy(
        np.array(
            [[0.001, 0.0, 0.0], [0.009, 0.0, 0.0], [1.0, 0.0, 0.0]],
            dtype=np.float32,
        ),
        frame_id="map",
    )

    svc.on_map_scene(
        MapSceneFrame(
            frame_id="map",
            source="maps.semantic",
            layers=[
                {
                    "id": "maps.semantic_occupancy",
                    "type": "pointcloud",
                    "payload": cloud,
                    "labels": [1, 2, 3],
                    "palette": {
                        "1": {"color": "#102030"},
                        "2": {"color": "#405060"},
                        "3": {"color": "#708090"},
                    },
                }
            ],
        )
    )

    points, colors = decode_pointcloud(_latest_cloud_bytes(svc))
    assert points.shape == (2, 3)
    assert colors is not None
    assert colors.tolist() == [[16, 32, 48], [112, 128, 144]]
