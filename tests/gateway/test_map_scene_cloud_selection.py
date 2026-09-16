from __future__ import annotations

import numpy as np
import pytest

from gateway.services.cloud_viewer import CloudViewerService
from runtime.msgs.map import MapSceneFrame
from runtime.msgs.sensor import PointCloud2
from runtime.utils.binary_codec import decode_pointcloud_frame


def _service(events: list[dict], *, mode: str = "mapping") -> CloudViewerService:
    service = CloudViewerService(
        queue_put_latest=lambda queue, data, loop, record: queue.put_nowait(data),
        current_loop=lambda: None,
        push_event=events.append,
        session_mode=lambda: mode,
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )
    service.configure(map_voxel_size=0.02)
    return service


def _cloud_layer(name: str, points: list[list[float]], *, frame_id: str = "map") -> dict:
    return {
        "id": f"maps.{name}_cloud",
        "type": "pointcloud",
        "source": "mapd",
        "payload": PointCloud2(points=np.asarray(points, dtype=np.float32).reshape(-1, 3), frame_id=frame_id, ts=100.0),
    }


def _frame(*layers: dict) -> MapSceneFrame:
    return MapSceneFrame(layers=list(layers), frame_id="map", source="mapd", ts=100.0)


def _latest(service: CloudViewerService):
    queue, latest = service.cloud_subscribe()
    service.cloud_unsubscribe(queue)
    assert latest is not None
    return decode_pointcloud_frame(latest)


@pytest.mark.parametrize("mode", ["mapping", "exploring"])
def test_native_scene_publishes_one_voxel_surface_snapshot_and_keeps_all_metadata(mode: str) -> None:
    events: list[dict] = []
    service = _service(events, mode=mode)
    surface = [[-3.0, 0.0, 0.0], [3.0, 0.0, 1.0]]
    service.on_map_scene(_frame(
        _cloud_layer("live", [[9.0, 0.0, 0.0]]),
        _cloud_layer("voxel", surface),
        _cloud_layer("accumulated", [[7.0, 0.0, 0.0]]),
    ))

    np.testing.assert_allclose(_latest(service).points, surface, atol=0.011)
    assert service.cache_point_count() == 2
    assert service.cloud_published_frames() == 1
    assert service.latest_cloud_metadata()["source"] == "maps.voxel_cloud"
    assert events[-1]["consumed_pointcloud_layers"] == 1
    assert len(events[-1]["layers"]) == 3


def test_native_snapshot_replaces_previous_mapping_points_without_merging_columns() -> None:
    service = _service([])
    service.configure(cloud_viewer_min_interval_s=0.0, cloud_viewer_min_point_delta=0)
    service.on_map_scene(_frame(_cloud_layer("voxel", [[-3.0, 0.0, 1.0], [3.0, 0.0, 0.0]])))
    service.on_map_scene(_frame(_cloud_layer("voxel", [[-3.0, 0.0, 0.0], [8.0, 0.0, 0.0]])))

    assert service.cache_point_count() == 2
    np.testing.assert_allclose(_latest(service).points, [[-3.0, 0.0, 0.0], [8.0, 0.0, 0.0]], atol=0.011)


def test_navigating_scene_displays_live_cloud_when_extended_layers_are_empty() -> None:
    events: list[dict] = []
    service = _service(events, mode="navigating")
    live = [[-2.0, 0.0, 0.0], [3.0, 0.0, 1.0]]
    service.on_map_scene(_frame(
        _cloud_layer("live", live),
        _cloud_layer("voxel", []),
        _cloud_layer("accumulated", []),
    ))

    np.testing.assert_allclose(_latest(service).points, live, atol=0.011)
    assert service.cache_point_count() == 2
    assert service.cloud_published_frames() == 1
    assert service.latest_cloud_metadata()["source"] == "maps.live_cloud"
    assert events[-1]["consumed_pointcloud_layers"] == 1
    assert len(events[-1]["layers"]) == 3


def test_native_snapshot_replaces_cache_even_when_publication_is_rate_limited() -> None:
    service = _service([])
    service.configure(cloud_viewer_min_interval_s=1000.0)
    service.on_map_scene(_frame(_cloud_layer("accumulated", [[-3.0, 0.0, 1.0]])))
    service.on_map_scene(_frame(_cloud_layer("accumulated", [[8.0, 0.0, 0.0]])))

    assert service.cloud_published_frames() == 1
    assert service.map_points_snapshot()["points"] == [[8.0, 0.0, 0.0]]
    assert service.cache_point_count() == 1


@pytest.mark.parametrize("selected", ["voxel", "accumulated"])
def test_empty_native_snapshot_clears_cache_and_display_without_falling_back(selected: str) -> None:
    events: list[dict] = []
    service = _service(events)
    service.configure(cloud_viewer_min_interval_s=1000.0)
    service.on_map_scene(_frame(_cloud_layer(selected, [[-3.0, 0.0, 1.0]])))
    layers = [_cloud_layer("live", [[9.0, 0.0, 0.0]]), _cloud_layer(selected, [])]
    if selected == "voxel":
        layers.append(_cloud_layer("accumulated", [[7.0, 0.0, 0.0]]))
    service.on_map_scene(_frame(*layers))

    assert service.cache_point_count() == 0
    assert len(_latest(service).points) == 0
    assert service.cloud_published_frames() == 2
    assert service.latest_cloud_metadata()["source"] == f"maps.{selected}_cloud"
    assert events[-1]["consumed_pointcloud_layers"] == 1


@pytest.mark.parametrize("unavailable", ["missing", "no_payload", "wrong_frame"])
def test_native_scene_uses_accumulated_when_voxel_unavailable(unavailable: str) -> None:
    service = _service([])
    layers = [_cloud_layer("live", [[9.0, 0.0, 0.0]]), _cloud_layer("accumulated", [[7.0, 0.0, 0.0]])]
    if unavailable == "no_payload":
        layers.append({"id": "maps.voxel_cloud", "type": "pointcloud", "source": "mapd"})
    elif unavailable == "wrong_frame":
        layers.append(_cloud_layer("voxel", [[3.0, 0.0, 0.0]], frame_id="odom"))
    service.on_map_scene(_frame(*layers))

    assert service.latest_cloud_metadata()["source"] == "maps.accumulated_cloud"
    np.testing.assert_allclose(_latest(service).points, [[7.0, 0.0, 0.0]], atol=0.011)


def test_native_scene_uses_live_only_as_last_fallback() -> None:
    service = _service([])
    service.on_map_scene(_frame(_cloud_layer("live", [[9.0, 0.0, 0.0]])))
    assert service.latest_cloud_metadata()["source"] == "maps.live_cloud"
    np.testing.assert_allclose(_latest(service).points, [[9.0, 0.0, 0.0]], atol=0.011)


@pytest.mark.parametrize("count", [100, 65000, 130000])
def test_native_surface_preserves_resolution_and_aligned_colors_with_bounded_sampling(count: int) -> None:
    service = _service([])
    service.configure(map_voxel_size=0.15)
    index = np.arange(count)
    points = np.column_stack((index % 400, index // 400, np.zeros(count))) * 0.05 + 0.025
    layer = _cloud_layer("voxel", points.tolist())
    layer["labels"] = (index % 3).tolist()
    layer["palette"] = {"0": {"color": "#102030"}, "1": {"color": "#405060"}, "2": {"color": "#708090"}}
    service.on_map_scene(_frame(layer))

    expected_indices = np.linspace(0, count - 1, 120000, dtype=np.int64) if count > 120000 else index
    expected_cache_count = len(expected_indices)
    if len(expected_indices) > 60000:
        expected_indices = expected_indices[np.linspace(0, len(expected_indices) - 1, 60000, dtype=np.int64)]
    decoded = _latest(service)
    assert service.cache_point_count() == expected_cache_count
    assert len(decoded.points) == min(count, 60000)
    np.testing.assert_allclose(decoded.points, points[expected_indices], atol=0.011)
    palette = np.array([[16, 32, 48], [64, 80, 96], [112, 128, 144]])
    np.testing.assert_array_equal(decoded.colors, palette[expected_indices % 3])


def test_native_surface_filters_nonfinite_points_without_losing_adjacent_valid_surface() -> None:
    service = _service([])
    service.configure(map_voxel_size=0.15)
    points = [[0.025, 0.025, 0.025], [float("nan"), 0.0, 0.0], [0.075, 0.025, 0.025]]
    service.on_map_scene(_frame(_cloud_layer("voxel", points)))
    np.testing.assert_allclose(_latest(service).points, [points[0], points[2]], atol=0.011)


def test_http_snapshot_sampling_is_stable_and_spans_the_cached_cloud() -> None:
    service = _service([])
    points = np.column_stack((np.arange(100, dtype=np.float32), np.zeros((100, 2), dtype=np.float32)))
    service.replace_map_points(points)
    first = service.map_points_snapshot(max_points=7)
    second = service.map_points_snapshot(max_points=7)
    assert first["points"] == second["points"]
    assert first["points"][0] == points[0].tolist()
    assert first["points"][-1] == points[-1].tolist()
    assert len(first["points"]) == 7


@pytest.mark.parametrize("replacement", [[[8.0, 0.0, 0.0], [9.0, 0.0, 0.0]], [[8.0, 0.0, 0.0]]])
def test_native_snapshot_publishes_changed_geometry_without_point_count_growth(replacement, monkeypatch) -> None:
    clock = [1000.0]
    monkeypatch.setattr("gateway.services.cloud_viewer.time.time", lambda: clock[0])
    service = _service([])
    service.configure(cloud_viewer_min_interval_s=0.25, cloud_viewer_min_point_delta=100,
                      cloud_viewer_force_interval_s=2.0)
    service.on_map_scene(_frame(_cloud_layer("voxel", [[-3.0, 0.0, 1.0], [3.0, 0.0, 0.0]])))
    clock[0] += 0.3
    service.on_map_scene(_frame(_cloud_layer("voxel", replacement)))
    assert service.cloud_published_frames() == 2
    np.testing.assert_allclose(_latest(service).points, replacement, atol=0.011)


def test_http_cloud_retains_source_time_even_when_reads_and_ws_throttling_continue(monkeypatch) -> None:
    clock = [1000.0]
    monkeypatch.setattr("gateway.services.cloud_viewer.time.time", lambda: clock[0])
    service = _service([])
    service.configure(cloud_viewer_min_interval_s=1000.0)
    service.on_map_scene(_frame(_cloud_layer("voxel", [[1.0, 0.0, 0.0]])))
    clock[0] += 10.0
    old = service.map_points_snapshot()
    assert old["stamp_s"] == 100.0
    assert old["ts"] == 1010.0
    replacement = _cloud_layer("voxel", [[2.0, 0.0, 0.0]])
    replacement["payload"].ts = 105.0
    service.on_map_scene(_frame(replacement))
    updated = service.map_points_snapshot()
    assert updated["stamp_s"] == 105.0
    assert updated["points"] == [[2.0, 0.0, 0.0]]
    assert service.cloud_published_frames() == 1
    service.clear()
    reset_stamp = service.map_points_snapshot()["stamp_s"]
    clock[0] += 10.0
    assert service.map_points_snapshot()["stamp_s"] == reset_stamp
