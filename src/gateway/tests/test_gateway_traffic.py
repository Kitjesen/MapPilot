from __future__ import annotations

import asyncio
import json
import threading
import time

import pytest

pytest.importorskip("fastapi")
from runtime.tests.numpy_guard import numpy_safe_skip_mark


def disable_cloud_publish_throttle(gateway) -> None:
    gateway.configure_cloud_viewer(
        cloud_viewer_min_interval_s=0.0,
        cloud_viewer_force_interval_s=0.0,
        cloud_viewer_min_point_delta=0,
    )


def configure_map_viewer(
    gateway,
    *,
    map_voxel_size: float = 0.1,
    voxel_min_hits: int | None = None,
    map_viewer_stale_grace: int | None = None,
    clean_map_layer_prefer_s: float | None = None,
    cloud_viewer_min_interval_s: float | None = None,
    cloud_viewer_force_interval_s: float | None = None,
    cloud_viewer_min_point_delta: int | None = None,
    scan_viewer_min_interval_s: float | None = None,
    slam_map_scan_prefer_s: float | None = None,
) -> None:
    kwargs = {
        "map_voxel_size": map_voxel_size,
        "voxel_min_hits": voxel_min_hits,
        "map_viewer_stale_grace": map_viewer_stale_grace,
        "clean_map_layer_prefer_s": clean_map_layer_prefer_s,
        "cloud_viewer_min_interval_s": cloud_viewer_min_interval_s,
        "cloud_viewer_force_interval_s": cloud_viewer_force_interval_s,
        "cloud_viewer_min_point_delta": cloud_viewer_min_point_delta,
        "scan_viewer_min_interval_s": scan_viewer_min_interval_s,
        "slam_map_scan_prefer_s": slam_map_scan_prefer_s,
    }
    gateway.configure_cloud_viewer(**{k: v for k, v in kwargs.items() if v is not None})


def cached_cloud_points(gateway):
    import numpy as np

    pts = gateway.map_cloud_points_array()
    if pts is None:
        return np.empty((0, 3), dtype=np.float32)
    return np.asarray(pts, dtype=np.float32)


def test_sse_slow_client_keeps_latest_events_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY, SSE_EVENT_SCHEMA_VERSION

    gateway = GatewayModule()
    queue = gateway._sse_subscribe()

    for seq in range(gateway._sse_queue_maxsize + 3):
        gateway.push_event({"type": "tick", "seq": seq})

    assert queue.qsize() == gateway._sse_queue_maxsize
    retained = queue.get_nowait()
    assert retained["seq"] == 3
    assert retained["event_id"] == 4
    assert retained["schema_version"] == SSE_EVENT_SCHEMA_VERSION
    assert retained["ts"] > 0

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["clients"] == 1
    assert stats["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert stats["sse"]["latest_event_id"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["published_events"] == gateway._sse_queue_maxsize + 3
    assert stats["sse"]["dropped_events"] == 3
    assert stats["sse"]["drop_policy"] == DROP_OLDEST_POLICY


@numpy_safe_skip_mark()
def test_sse_raster_events_are_client_bound_and_throttled():
    import numpy as np

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._sse_raster_min_interval_s = 60.0
    grid = np.full((2, 3), 42, dtype=np.uint8)

    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})
    assert gateway._traffic_stats_snapshot()["sse"]["published_events"] == 0

    queue = gateway._sse_subscribe()
    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})

    event = queue.get_nowait()
    assert event["type"] == "costmap"
    assert event["rows"] == 2
    assert event["cols"] == 3
    assert event["grid_b64"]

    gateway._costmap_throttle = 4
    gateway._on_costmap({"grid": grid, "resolution": 0.1, "origin": [1.0, 2.0]})
    assert queue.empty()

    stats = gateway._traffic_stats_snapshot()
    assert stats["sse"]["suppressed_events"]["costmap"] == 1
    assert stats["sse"]["raster_min_interval_s"] == 60.0


@numpy_safe_skip_mark()
def test_slope_grid_defaults_to_metadata_and_can_enable_inline_payload():
    import numpy as np

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._sse_raster_min_interval_s = 0.0
    queue = gateway._sse_subscribe()

    gateway._on_slope_grid({"grid": np.ones((2, 2)), "resolution": 0.2, "origin": [0.0, 0.0]})
    event = queue.get_nowait()

    assert event["type"] == "slope_grid"
    assert event["payload"] == "omitted"
    assert event["available"] is True
    assert event["rows"] == 2
    assert event["cols"] == 2
    assert "grid_b64" not in event

    gateway._sse_slope_payload_enabled = True
    gateway._on_slope_grid({"grid": np.ones((2, 2)), "resolution": 0.2, "origin": [0.0, 0.0]})
    event = queue.get_nowait()

    assert event["type"] == "slope_grid"
    assert event["payload"] == "inline"
    assert event["grid_b64"]


def test_gateway_visual_raster_inputs_keep_latest_only():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    assert gateway.costmap._policy == "latest"
    assert gateway.slope_grid._policy == "latest"


def test_cloud_slow_client_keeps_latest_frames_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue, latest = gateway._cloud_subscribe()

    assert latest is None

    cloud_queue_maxsize = gateway.cloud_queue_maxsize()
    for seq in range(cloud_queue_maxsize + 2):
        gateway._publish_cloud_frame(bytes([seq]))

    assert queue.qsize() == cloud_queue_maxsize
    assert queue.get_nowait() == bytes([2])

    stats = gateway._traffic_stats_snapshot()
    assert stats["cloud"]["clients"] == 1
    assert stats["cloud"]["queue_maxsize"] == cloud_queue_maxsize
    assert stats["cloud"]["published_frames"] == cloud_queue_maxsize + 2
    assert stats["cloud"]["dropped_frames"] == 2
    assert stats["cloud"]["drop_policy"] == DROP_OLDEST_POLICY
    assert stats["cloud"]["latest_seq"] == cloud_queue_maxsize + 2


def test_cloud_stats_include_latest_frame_metadata():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    seq = gateway._publish_cloud_frame(
        b"PCLD",
        metadata={
            "point_count": 12,
            "source": "slam_map_cloud",
            "z_min": -6.5,
            "z_max": -2.8,
        },
    )

    latest = gateway._traffic_stats_snapshot()["cloud"]["latest_frame"]
    assert latest["seq"] == seq
    assert latest["bytes"] == 4
    assert latest["point_count"] == 12
    assert latest["source"] == "slam_map_cloud"
    assert latest["z_min"] == pytest.approx(-6.5)
    assert latest["z_max"] == pytest.approx(-2.8)
    assert latest["age_s"] >= 0.0


def test_scan_slow_client_keeps_latest_frames_and_drops_oldest():
    from gateway.gateway_module import GatewayModule
    from gateway.services.traffic import DROP_OLDEST_POLICY

    gateway = GatewayModule()
    queue, latest = gateway._scan_subscribe()

    assert latest is None

    scan_queue_maxsize = gateway.scan_queue_maxsize()
    for seq in range(scan_queue_maxsize + 2):
        gateway._publish_scan_frame(bytes([seq]))

    assert queue.qsize() == scan_queue_maxsize
    assert queue.get_nowait() == bytes([2])

    stats = gateway._traffic_stats_snapshot()
    assert stats["scan"]["clients"] == 1
    assert stats["scan"]["queue_maxsize"] == scan_queue_maxsize
    assert stats["scan"]["published_frames"] == scan_queue_maxsize + 2
    assert stats["scan"]["dropped_frames"] == 2
    assert stats["scan"]["drop_policy"] == DROP_OLDEST_POLICY
    assert stats["scan"]["latest_seq"] == scan_queue_maxsize + 2


def test_scan_stats_include_latest_frame_metadata():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    seq = gateway._publish_scan_frame(
        b"PCLD",
        metadata={
            "point_count": 8,
            "source": "slam_map_cloud",
            "z_min": -1.0,
            "z_max": 2.0,
        },
    )

    latest = gateway._traffic_stats_snapshot()["scan"]["latest_frame"]
    assert latest["seq"] == seq
    assert latest["bytes"] == 4
    assert latest["point_count"] == 8
    assert latest["source"] == "slam_map_cloud"
    assert latest["z_min"] == pytest.approx(-1.0)
    assert latest["z_max"] == pytest.approx(2.0)
    assert latest["age_s"] >= 0.0


def test_map_viewer_voxel_size_can_be_configured(monkeypatch):
    from gateway.gateway_module import GatewayModule

    monkeypatch.setenv("LINGTU_MAP_VIEWER_VOXEL_SIZE", "0.05")
    gateway = GatewayModule()

    config = gateway.cloud_viewer_config()
    assert config["map_voxel_size"] == pytest.approx(0.05)
    assert config["inv_map_voxel_size"] == pytest.approx(20.0)


@numpy_safe_skip_mark()
def test_external_mapping_profile_recovers_session_before_first_cloud(monkeypatch):
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    monkeypatch.setenv("LINGTU_COMMAND_OUTPUT_MODE", "endpoint_only")
    monkeypatch.setenv("LINGTU_PROFILE", "map")
    gateway = GatewayModule(manage_session_services=False)
    gateway.setup()
    configure_map_viewer(gateway, map_voxel_size=0.1, voxel_min_hits=1)
    disable_cloud_publish_throttle(gateway)

    gateway._on_voxel_cloud(
        PointCloud2.from_numpy(
            np.asarray([[0.21, 0.21, 0.10], [1.21, 1.21, 0.10]], dtype=np.float32),
            frame_id="map",
        )
    )
    gateway._on_voxel_cloud(
        PointCloud2.from_numpy(
            np.asarray([[5.21, 5.21, 0.10], [6.21, 6.21, 0.10]], dtype=np.float32),
            frame_id="map",
        )
    )

    assert gateway._session_mode == "mapping"
    assert gateway._session_product_profile == "map"
    assert gateway._session_product_session == "mapping"
    assert gateway.map_cloud_point_count() == 4
    assert gateway.latest_cloud_metadata()["session_mode"] == "mapping"


@numpy_safe_skip_mark()
def test_map_viewer_cache_has_a_hard_point_limit(monkeypatch):
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    monkeypatch.setenv("LINGTU_MAP_VIEWER_MAX_CACHE_POINTS", "8")
    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    disable_cloud_publish_throttle(gateway)

    for frame in range(4):
        points = np.asarray(
            [[frame * 10.0 + index, 0.0, 0.0] for index in range(6)],
            dtype=np.float32,
        )
        gateway._on_map_cloud(PointCloud2(points=points, frame_id="map"))

    assert gateway.map_cloud_point_count() <= 8
    snapshot = gateway.map_cloud_points_snapshot(max_points=100)
    assert snapshot["count"] <= 8


@numpy_safe_skip_mark()
def test_mapping_clean_voxel_cloud_preserves_accumulated_viewer_context():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(gateway, map_voxel_size=0.1, voxel_min_hits=1, map_viewer_stale_grace=2)
    disable_cloud_publish_throttle(gateway)

    raw_with_stale_dynamic = np.asarray(
        [
            [0.21, 0.21, 0.10],
            [0.21, 0.21, 1.10],
            [1.21, 1.21, 1.10],
        ],
        dtype=np.float32,
    )
    cleaned_voxel_layer = np.asarray(
        [
            [0.21, 0.21, 0.15],
            [1.21, 1.21, 1.10],
        ],
        dtype=np.float32,
    )

    gateway._on_map_cloud(PointCloud2.from_numpy(raw_with_stale_dynamic, frame_id="map"))
    gateway._on_voxel_cloud(PointCloud2.from_numpy(cleaned_voxel_layer, frame_id="map"))

    pts = cached_cloud_points(gateway)

    clean_floor_observation = np.asarray([0.21, 0.21, 0.15], dtype=np.float32)
    untouched = np.asarray([1.21, 1.21, 1.10], dtype=np.float32)
    raw_high_observation = np.asarray([0.21, 0.21, 1.10], dtype=np.float32)

    assert np.any(np.linalg.norm(pts - clean_floor_observation, axis=1) < 0.02)
    assert np.any(np.linalg.norm(pts - untouched, axis=1) < 0.02)
    assert np.any(np.linalg.norm(pts - raw_high_observation, axis=1) < 0.02)

    gateway._on_voxel_cloud(PointCloud2.from_numpy(cleaned_voxel_layer, frame_id="map"))
    pts_after_decay = cached_cloud_points(gateway)

    assert np.any(np.linalg.norm(pts_after_decay - clean_floor_observation, axis=1) < 0.02)
    assert np.any(np.linalg.norm(pts_after_decay - untouched, axis=1) < 0.02)
    assert not np.any(np.linalg.norm(pts_after_decay - raw_high_observation, axis=1) < 0.02)


@numpy_safe_skip_mark()
def test_mapping_clean_voxel_cloud_accumulates_disjoint_snapshots_without_flicker():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(gateway, map_voxel_size=0.1, voxel_min_hits=1)
    disable_cloud_publish_throttle(gateway)

    first_clean_snapshot = np.asarray(
        [
            [0.21, 0.21, 0.10],
            [1.21, 1.21, 0.10],
        ],
        dtype=np.float32,
    )
    next_clean_snapshot = np.asarray(
        [
            [5.21, 5.21, 0.10],
            [6.21, 6.21, 0.10],
        ],
        dtype=np.float32,
    )

    gateway._on_voxel_cloud(PointCloud2.from_numpy(first_clean_snapshot, frame_id="map"))
    gateway._on_voxel_cloud(PointCloud2.from_numpy(next_clean_snapshot, frame_id="map"))

    pts = cached_cloud_points(gateway)
    latest = gateway.latest_cloud_metadata()

    assert len(pts) == 4
    assert latest["point_count"] == 4
    assert latest["cache_points"] == 4
    for expected in np.concatenate([first_clean_snapshot, next_clean_snapshot], axis=0):
        assert np.any(np.linalg.norm(pts - expected, axis=1) < 0.02)


@numpy_safe_skip_mark()
def test_mapping_raw_viewer_publishes_accumulated_cache_not_hit_filtered_subset():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(gateway, map_voxel_size=0.1, voxel_min_hits=3)
    disable_cloud_publish_throttle(gateway)

    frames = [
        [[0.21, 0.21, 0.10]],
        [[0.21, 0.21, 0.10], [5.21, 5.21, 0.10]],
        [[0.21, 0.21, 0.10], [6.21, 6.21, 0.10]],
        [[0.21, 0.21, 0.10], [7.21, 7.21, 0.10]],
    ]
    for frame in frames:
        gateway._on_map_cloud(PointCloud2.from_numpy(np.asarray(frame, dtype=np.float32), frame_id="map"))

    pts = cached_cloud_points(gateway)
    latest = gateway.latest_cloud_metadata()

    assert len(pts) == 4
    assert latest["cache_points"] == 4
    assert latest["point_count"] == 4


@numpy_safe_skip_mark()
def test_mapping_cloud_viewer_throttles_full_frame_republish():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(
        gateway,
        map_voxel_size=0.1,
        cloud_viewer_min_interval_s=60.0,
        cloud_viewer_min_point_delta=0,
    )

    gateway._on_voxel_cloud(PointCloud2.from_numpy(np.asarray([[0.21, 0.21, 0.10]], dtype=np.float32), frame_id="map"))
    first_published = gateway.cloud_published_frames()
    gateway._on_voxel_cloud(PointCloud2.from_numpy(np.asarray([[5.21, 5.21, 0.10]], dtype=np.float32), frame_id="map"))

    pts = cached_cloud_points(gateway)

    assert len(pts) == 2
    assert gateway.cloud_published_frames() == first_published


@numpy_safe_skip_mark()
def test_mapping_cloud_viewer_suppresses_small_delta_until_force_interval():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(
        gateway,
        map_voxel_size=0.1,
        cloud_viewer_min_interval_s=0.0,
        cloud_viewer_min_point_delta=10,
        cloud_viewer_force_interval_s=60.0,
    )

    gateway._on_voxel_cloud(PointCloud2.from_numpy(np.asarray([[0.21, 0.21, 0.10]], dtype=np.float32), frame_id="map"))
    first_published = gateway.cloud_published_frames()
    gateway._on_voxel_cloud(PointCloud2.from_numpy(np.asarray([[1.21, 1.21, 0.10]], dtype=np.float32), frame_id="map"))
    assert gateway.cloud_published_frames() == first_published

    gateway.adjust_cloud_viewer_last_publish_ts(-61.0)
    gateway._on_voxel_cloud(PointCloud2.from_numpy(np.asarray([[2.21, 2.21, 0.10]], dtype=np.float32), frame_id="map"))

    assert gateway.cloud_published_frames() == first_published + 1
    latest = gateway.latest_cloud_metadata()
    assert latest["cache_points"] == 3


@numpy_safe_skip_mark()
def test_recent_clean_voxel_layer_suppresses_slam_map_cloud_fallback():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(gateway, map_voxel_size=0.1, voxel_min_hits=1, clean_map_layer_prefer_s=60.0)
    disable_cloud_publish_throttle(gateway)

    clean = np.asarray([[0.21, 0.21, 0.15]], dtype=np.float32)
    stale_raw = np.asarray([[0.21, 0.21, 1.10]], dtype=np.float32)

    gateway._on_voxel_cloud(PointCloud2.from_numpy(clean, frame_id="map"))
    gateway._on_map_cloud(PointCloud2.from_numpy(stale_raw, frame_id="map"))

    pts = cached_cloud_points(gateway)

    assert np.any(np.linalg.norm(pts - clean[0], axis=1) < 0.02)
    assert not np.any(pts[:, 2] > 0.8)
    assert gateway._traffic_stats_snapshot()["scan"]["latest_frame"]["source"] == "slam_map_cloud"


@numpy_safe_skip_mark()
def test_default_clean_layer_ownership_spans_voxel_publish_interval(monkeypatch):
    """The field voxel layer publishes every 2 s, so raw map frames
    must not take ownership between two clean accumulated snapshots.
    """
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    monkeypatch.delenv("LINGTU_CLEAN_MAP_LAYER_PREFER_S", raising=False)
    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(gateway, map_voxel_size=0.1, voxel_min_hits=1)
    disable_cloud_publish_throttle(gateway)

    clean = np.asarray([[0.21, 0.21, 0.15]], dtype=np.float32)
    raw_between_clean_frames = np.asarray([[9.21, 9.21, 1.10]], dtype=np.float32)
    gateway._on_voxel_cloud(PointCloud2.from_numpy(clean, frame_id="map"))
    gateway.mark_clean_map_layer_recent(time.time() - 1.5)
    gateway._on_map_cloud(PointCloud2.from_numpy(raw_between_clean_frames, frame_id="map"))

    pts = cached_cloud_points(gateway)
    latest = gateway.latest_cloud_metadata()

    assert latest["source"] == "voxel_cloud"
    assert len(pts) == 1
    assert np.linalg.norm(pts[0] - clean[0]) < 0.02


@numpy_safe_skip_mark()
def test_raw_cloud_still_publishes_scan_when_clean_layer_owns_map_view():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway.mark_clean_map_layer_recent(time.time())
    gateway.configure_cloud_viewer(clean_map_layer_prefer_s=60.0)
    pts = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.2],
            [2.0, 0.0, 0.4],
        ],
        dtype=np.float32,
    )

    gateway._on_map_cloud(PointCloud2(points=pts))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["published_frames"] == 1
    assert traffic["scan"]["latest_frame"]["source"] == "slam_map_cloud"
    assert traffic["scan"]["latest_frame"]["point_count"] == 3
    assert traffic["cloud"]["published_frames"] == 0


@numpy_safe_skip_mark()
def test_slam_map_cloud_is_primary_scan_source_over_raw_lidar():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    lidar_pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    slam_pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)

    gateway._on_lidar_scan(PointCloud2(points=lidar_pts, frame_id="livox_frame"))
    gateway._on_map_cloud(PointCloud2(points=slam_pts, frame_id="map"))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["published_frames"] == 1
    assert traffic["scan"]["latest_frame"]["source"] == "slam_map_cloud"
    assert traffic["scan"]["incompatible_frame_drops"] == 1
    assert traffic["scan"]["last_incompatible_frame_id"] == "livox_frame"


@numpy_safe_skip_mark()
def test_recent_slam_map_cloud_suppresses_raw_lidar_scan_overlay():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway.configure_cloud_viewer(scan_viewer_min_interval_s=10.0)
    slam_pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
    lidar_pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)

    gateway._on_map_cloud(PointCloud2(points=slam_pts, frame_id="map"))
    gateway._on_lidar_scan(PointCloud2(points=lidar_pts, frame_id="livox_frame"))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["published_frames"] == 1
    assert traffic["scan"]["latest_frame"]["source"] == "slam_map_cloud"


@numpy_safe_skip_mark()
def test_lidar_scan_fallback_when_slam_map_cloud_is_stale():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway.configure_cloud_viewer(scan_viewer_min_interval_s=0.0, slam_map_scan_prefer_s=0.1)
    slam_pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
    lidar_pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)

    gateway._on_map_cloud(PointCloud2(points=slam_pts, frame_id="map"))
    gateway.adjust_slam_map_scan_ts(-1.0)
    gateway._on_lidar_scan(PointCloud2(points=lidar_pts, frame_id="map"))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["published_frames"] == 2
    assert traffic["scan"]["latest_frame"]["source"] == "lidar_scan"
    assert traffic["scan"]["latest_frame"]["frame_id"] == "map"


@numpy_safe_skip_mark()
def test_scan_overlay_rejects_frame_that_differs_from_accumulated_map():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway.configure_cloud_viewer(
        scan_viewer_min_interval_s=0.0,
        slam_map_scan_prefer_s=0.1,
    )
    gateway._on_map_cloud(
        PointCloud2(
            points=np.asarray([[5.0, 0.0, 0.0]], dtype=np.float32),
            frame_id="map",
        )
    )
    gateway.adjust_slam_map_scan_ts(-1.0)

    gateway._on_lidar_scan(
        PointCloud2(
            points=np.asarray([[50.0, 0.0, 0.0]], dtype=np.float32),
            frame_id="odom",
        )
    )

    traffic = gateway._traffic_stats_snapshot()["scan"]
    assert traffic["published_frames"] == 1
    assert traffic["incompatible_frame_drops"] == 1
    assert traffic["last_incompatible_frame_id"] == "odom"
    assert traffic["latest_frame"]["frame_id"] == "map"


@numpy_safe_skip_mark()
def test_accumulated_map_rejects_points_from_a_different_frame():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    disable_cloud_publish_throttle(gateway)
    gateway._session_mode = "mapping"
    gateway._on_map_cloud(
        PointCloud2(
            points=np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
            frame_id="map",
        )
    )
    gateway._on_map_cloud(
        PointCloud2(
            points=np.asarray([[100.0, 0.0, 0.0]], dtype=np.float32),
            frame_id="odom",
        )
    )

    snapshot = gateway.map_cloud_points_snapshot(max_points=10)
    assert snapshot["frame_id"] == "map"
    assert snapshot["count"] == 1
    np.testing.assert_allclose(snapshot["points"], [[1.0, 0.0, 0.0]], atol=1e-6)
    scan = gateway._traffic_stats_snapshot()["scan"]
    assert scan["map_incompatible_frame_drops"] == 1
    assert scan["last_incompatible_map_frame_id"] == "odom"


@numpy_safe_skip_mark()
def test_cloud_and_scan_wire_frames_share_frame_epoch_and_reset_subscribers():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2
    from runtime.utils.binary_codec import decode_pointcloud_frame

    gateway = GatewayModule()
    disable_cloud_publish_throttle(gateway)
    gateway.configure_cloud_viewer(scan_viewer_min_interval_s=0.0)
    cloud_queue, _ = gateway._cloud_subscribe()
    scan_queue, _ = gateway._scan_subscribe()

    gateway._on_voxel_cloud(
        PointCloud2.from_numpy(
            np.asarray([[1.0, 2.0, 0.5]], dtype=np.float32),
            frame_id="map",
            ts=12.5,
        )
    )

    cloud_frame = decode_pointcloud_frame(cloud_queue.get_nowait())
    scan_frame = decode_pointcloud_frame(scan_queue.get_nowait())
    assert cloud_frame.frame_id == scan_frame.frame_id == "map"
    assert cloud_frame.epoch == scan_frame.epoch == 1
    assert cloud_frame.stream_kind == "map"
    assert scan_frame.stream_kind == "scan"
    http_snapshot = gateway.map_cloud_points_snapshot(max_points=10)
    assert http_snapshot["protocol_version"] == 2
    assert http_snapshot["frame_id"] == "map"
    assert http_snapshot["epoch"] == 1
    assert http_snapshot["stream_kind"] == "map"

    gateway.clear_map_cloud_cache(reason="slam_runtime_changed")

    cloud_reset = decode_pointcloud_frame(cloud_queue.get_nowait())
    scan_reset = decode_pointcloud_frame(scan_queue.get_nowait())
    assert cloud_reset.stream_kind == scan_reset.stream_kind == "reset"
    assert cloud_reset.epoch == scan_reset.epoch == 2
    assert cloud_reset.points.shape == scan_reset.points.shape == (0, 3)
    http_reset = gateway.map_cloud_points_snapshot(max_points=10)
    assert http_reset["epoch"] == 2
    assert http_reset["stream_kind"] == "reset"


@numpy_safe_skip_mark()
def test_map_publish_cannot_overtake_scene_epoch_reset(monkeypatch):
    import numpy as np

    import gateway.services.cloud_viewer as cloud_viewer_module
    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2
    from runtime.utils.binary_codec import decode_pointcloud_frame

    gateway = GatewayModule()
    disable_cloud_publish_throttle(gateway)
    cloud_queue, _ = gateway._cloud_subscribe()
    map_encode_started = threading.Event()
    allow_map_encode = threading.Event()
    original_encode = cloud_viewer_module.encode_pointcloud

    def blocking_encode(*args, **kwargs):
        if kwargs.get("stream_kind") == "map" and not map_encode_started.is_set():
            map_encode_started.set()
            assert allow_map_encode.wait(timeout=2.0)
        return original_encode(*args, **kwargs)

    monkeypatch.setattr(cloud_viewer_module, "encode_pointcloud", blocking_encode)
    map_thread = threading.Thread(
        target=lambda: gateway._on_voxel_cloud(
            PointCloud2.from_numpy(
                np.asarray([[1.0, 0.0, 0.0]], dtype=np.float32),
                frame_id="map",
            )
        )
    )
    map_thread.start()
    assert map_encode_started.wait(timeout=2.0)

    reset_thread = threading.Thread(target=lambda: gateway.clear_map_cloud_cache(reason="slam_runtime_changed"))
    reset_thread.start()
    time.sleep(0.05)
    assert reset_thread.is_alive()

    allow_map_encode.set()
    map_thread.join(timeout=2.0)
    reset_thread.join(timeout=2.0)
    assert not map_thread.is_alive()
    assert not reset_thread.is_alive()

    first = decode_pointcloud_frame(cloud_queue.get_nowait())
    second = decode_pointcloud_frame(cloud_queue.get_nowait())
    assert (first.stream_kind, first.epoch) == ("map", 1)
    assert (second.stream_kind, second.epoch) == ("reset", 2)


@numpy_safe_skip_mark()
def test_lidar_scan_with_sensor_frame_is_not_used_as_viewer_overlay():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    gateway.configure_cloud_viewer(scan_viewer_min_interval_s=0.0, slam_map_scan_prefer_s=0.1)
    slam_pts = np.array([[5.0, 0.0, 0.0]], dtype=np.float32)
    lidar_pts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)

    gateway._on_map_cloud(PointCloud2(points=slam_pts, frame_id="map"))
    gateway.adjust_slam_map_scan_ts(-1.0)
    gateway._on_lidar_scan(PointCloud2(points=lidar_pts, frame_id="livox_frame"))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["published_frames"] == 1
    assert traffic["scan"]["latest_frame"]["source"] == "slam_map_cloud"
    assert traffic["scan"]["incompatible_frame_drops"] == 1
    assert traffic["scan"]["last_incompatible_frame_id"] == "livox_frame"


@numpy_safe_skip_mark()
def test_current_scan_allows_kilometer_scale_odom_frame_coordinates():
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from runtime.msgs.sensor import PointCloud2

    gateway = GatewayModule()
    pts = np.array(
        [
            [-2421.8, 1072.8, -21.8],
            [-2421.4, 1073.1, -21.6],
        ],
        dtype=np.float32,
    )

    gateway._on_map_cloud(PointCloud2(points=pts))

    traffic = gateway._traffic_stats_snapshot()
    assert traffic["scan"]["latest_frame"]["point_count"] == 2
    assert traffic["scan"]["latest_frame"]["source"] == "slam_map_cloud"


@numpy_safe_skip_mark()
def test_voxel_map_layer_to_gateway_clears_stale_dynamic_column(monkeypatch):
    import numpy as np

    from gateway.gateway_module import GatewayModule
    from maps.modules import voxel_grid
    from maps.modules.voxel_grid import VoxelGridModule
    from runtime.msgs.sensor import PointCloud2

    class FakeNativeVoxelLayer:
        def __init__(self, **config):
            self.voxel_size = float(config["voxel_size"])
            self.column_carving = bool(config["column_carving"])
            self._points = np.empty((0, 3), dtype=np.float32)
            self._stats = {
                "input_points": 0,
                "accepted_points": 0,
                "input_voxels": 0,
                "input_columns": 0,
                "carved_columns": 0,
                "carved_voxels": 0,
                "total_voxels": 0,
                "column_carving": self.column_carving,
            }

        def close(self):
            pass

        def reset(self):
            self._points = np.empty((0, 3), dtype=np.float32)

        def decay(self):
            pass

        def update(self, points, *, frame_id, stamp_ns, origin_xyz):
            pts = np.asarray(points[:, :3], dtype=np.float32)
            old = self._points
            carved_voxels = 0
            if self.column_carving and old.size:
                cols = {tuple(v) for v in np.floor(pts[:, :2] / self.voxel_size).astype(np.int32).tolist()}
                old_cols = np.floor(old[:, :2] / self.voxel_size).astype(np.int32)
                keep = np.asarray(
                    [tuple(col) not in cols for col in old_cols.tolist()],
                    dtype=bool,
                )
                carved_voxels = int((~keep).sum())
                old = old[keep]
            self._points = np.vstack([old, pts]) if old.size else pts
            self._stats = {
                "input_points": int(points.shape[0]),
                "accepted_points": int(pts.shape[0]),
                "input_voxels": int(pts.shape[0]),
                "input_columns": int(
                    len({tuple(v) for v in np.floor(pts[:, :2] / self.voxel_size).astype(np.int32).tolist()})
                ),
                "carved_columns": 1 if carved_voxels else 0,
                "carved_voxels": carved_voxels,
                "total_voxels": int(self._points.shape[0]),
                "column_carving": self.column_carving,
            }

        def voxel_count(self):
            return int(self._points.shape[0])

        def query_count(self, x, y, z):
            return 0.0

        def stats(self):
            return dict(self._stats)

        def scene_metadata(self):
            return {}

        def snapshot_xyz(self):
            if self._points.size == 0:
                return self._points.copy()
            return (np.floor(self._points / self.voxel_size).astype(np.float32) + 0.5) * self.voxel_size

    monkeypatch.setattr(voxel_grid, "NativeVoxelLayer", FakeNativeVoxelLayer)

    gateway = GatewayModule()
    gateway._session_mode = "mapping"
    configure_map_viewer(gateway, map_voxel_size=0.1, map_viewer_stale_grace=2)

    voxel_layer = VoxelGridModule(
        voxel_size=0.1,
        min_z=-1.0,
        max_z=3.0,
        decay_rate=0.0,
        publish_interval=0.0,
        column_carving=True,
    )
    voxel_layer.setup()
    voxel_layer.voxel_cloud._add_callback(gateway._on_voxel_cloud)

    first = np.asarray(
        [
            [0.21, 0.21, 0.10],
            [0.21, 0.21, 1.10],
            [1.21, 1.21, 1.10],
        ],
        dtype=np.float32,
    )
    second = np.asarray([[0.21, 0.21, 0.15]], dtype=np.float32)

    voxel_layer.map_cloud._deliver(PointCloud2.from_numpy(first, frame_id="map"))
    voxel_layer.map_cloud._deliver(PointCloud2.from_numpy(second, frame_id="map"))

    pts = cached_cloud_points(gateway)

    same_column = (np.abs(pts[:, 0] - 0.25) < 0.02) & (np.abs(pts[:, 1] - 0.25) < 0.02)
    assert np.any(same_column & (np.abs(pts[:, 2] - 0.15) < 0.08))
    assert np.any(same_column & (pts[:, 2] > 0.8))
    assert np.any((np.abs(pts[:, 0] - 1.25) < 0.02) & (np.abs(pts[:, 1] - 1.25) < 0.02))

    voxel_layer.map_cloud._deliver(PointCloud2.from_numpy(second, frame_id="map"))
    pts_after_decay = cached_cloud_points(gateway)

    same_column_after_decay = (np.abs(pts_after_decay[:, 0] - 0.25) < 0.02) & (
        np.abs(pts_after_decay[:, 1] - 0.25) < 0.02
    )
    assert np.any(same_column_after_decay & (np.abs(pts_after_decay[:, 2] - 0.15) < 0.08))
    assert not np.any(same_column_after_decay & (pts_after_decay[:, 2] > 0.8))
    assert np.any((np.abs(pts_after_decay[:, 0] - 1.25) < 0.02) & (np.abs(pts_after_decay[:, 1] - 1.25) < 0.02))


def test_gateway_health_and_bootstrap_expose_traffic_policy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap, build_app_traffic

    gateway = GatewayModule()
    gateway._sse_subscribe()
    gateway._cloud_subscribe()

    health = gateway.health()
    bootstrap = build_app_bootstrap(gateway)
    traffic = build_app_traffic(gateway)

    assert health["gateway"]["traffic"]["sse"]["clients"] == 1
    assert health["gateway"]["traffic"]["cloud"]["clients"] == 1
    assert bootstrap["traffic"]["sse"]["queue_maxsize"] == gateway._sse_queue_maxsize
    assert bootstrap["traffic"]["cloud"]["queue_maxsize"] == gateway.cloud_queue_maxsize()
    assert bootstrap["traffic"]["recommended_client_rates_hz"]["state"] == 1.0
    assert bootstrap["traffic"]["client_policy"]["large_event_policy"]["slope_grid_payload"] == "metadata_sse"
    assert traffic["schema_version"] == 1
    assert traffic["status"] == "ok"
    assert traffic["sse"]["clients"] == 1
    assert traffic["cloud"]["clients"] == 1
    assert traffic["client_policy"]["usage"] == "low_frequency_monitoring"
    assert traffic["client_policy"]["traffic_endpoint"] == "/api/v1/app/traffic"


def test_app_traffic_reports_backpressure_warnings():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_traffic

    gateway = GatewayModule()
    gateway._sse_queue_maxsize = 2
    gateway.configure_cloud_viewer(cloud_queue_maxsize=2)
    gateway._sse_subscribe()
    gateway._cloud_subscribe()

    for seq in range(5):
        gateway.push_event({"type": "tick", "seq": seq})
        gateway._publish_cloud_frame(bytes([seq]))

    traffic = build_app_traffic(gateway)

    assert traffic["status"] == "degraded"
    assert traffic["sse"]["dropped_events"] == 3
    assert traffic["cloud"]["dropped_frames"] == 3
    assert "sse_events_dropped" in traffic["warnings"]
    assert "sse_queue_pressure" in traffic["warnings"]
    assert "cloud_frames_dropped_latest_only" in traffic["warnings"]
    assert "cloud_queue_pressure" in traffic["warnings"]


def test_sse_message_format_keeps_eventsource_onmessage_contract():
    from gateway.services.traffic import (
        SSE_EVENT_SCHEMA_VERSION,
        format_sse_message,
        normalize_sse_event,
    )

    event = normalize_sse_event({"type": "mission_status", "data": {"state": "IDLE"}}, event_id=7, now=123.0)
    text = format_sse_message(event, retry_ms=3000)

    assert text.startswith("retry: 3000\nid: 7\ndata: ")
    assert "\nevent:" not in text
    assert '"schema_version":1' in text
    assert '"event_id":7' in text
    assert '"ts":123.0' in text
    assert event["schema_version"] == SSE_EVENT_SCHEMA_VERSION


def test_sse_subscribe_with_event_id_reserves_snapshot_before_push():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    queue, snapshot_id = gateway._sse_subscribe_with_event_id()

    gateway.push_event({"type": "later"})

    event = queue.get_nowait()
    assert snapshot_id == 1
    assert event["type"] == "later"
    assert event["event_id"] == 2
    assert gateway._traffic_stats_snapshot()["sse"]["latest_event_id"] == 2


def test_sse_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue = gateway._sse_subscribe()

        thread = threading.Thread(target=lambda: gateway.push_event({"type": "threaded", "data": 1}))
        thread.start()
        try:
            event = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._sse_unsubscribe(queue)

        assert event["type"] == "threaded"
        assert event["event_id"] == 1
        assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0

    asyncio.run(_run())


def test_cloud_subscriber_receives_threaded_publish_on_endpoint_loop():
    from gateway.gateway_module import GatewayModule

    async def _run():
        gateway = GatewayModule()
        queue, latest = gateway._cloud_subscribe()
        assert latest is None

        thread = threading.Thread(target=lambda: gateway._publish_cloud_frame(b"PCL-threaded"))
        thread.start()
        try:
            frame = await asyncio.wait_for(queue.get(), timeout=1.0)
        finally:
            thread.join(timeout=1.0)
            gateway._cloud_unsubscribe(queue)

        assert frame == b"PCL-threaded"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0

    asyncio.run(_run())


def test_camera_websocket_is_camera_only_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.camera_clients = 0
            self.teleop_clients = 0

        def on_camera_client_connect(self):
            self.camera_clients += 1

        def on_camera_client_disconnect(self):
            self.camera_clients -= 1

        def on_client_connect(self):
            self.teleop_clients += 1

        def on_client_disconnect(self):
            self.teleop_clients -= 1

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    gateway.push_jpeg(b"\xff\xd8\xffcamera")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/camera") as ws:
        assert ws.receive_bytes() == b"\xff\xd8\xffcamera"
        assert tracker.camera_clients == 1
        assert tracker.teleop_clients == 0
        assert gateway._teleop_clients == 0

    assert tracker.camera_clients == 0
    assert tracker.teleop_clients == 0


def test_cloud_websocket_sends_latest_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._publish_cloud_frame(b"PCL0")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/cloud") as ws:
        assert ws.receive_bytes() == b"PCL0"
        assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 1
        gateway._publish_cloud_frame(b"PCL1")
        assert ws.receive_bytes() == b"PCL1"

    assert gateway._traffic_stats_snapshot()["cloud"]["clients"] == 0


def test_scan_websocket_sends_latest_and_cleans_up():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._publish_scan_frame(b"SCAN0")

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/scan") as ws:
        assert ws.receive_bytes() == b"SCAN0"
        assert gateway._traffic_stats_snapshot()["scan"]["clients"] == 1
        gateway._publish_scan_frame(b"SCAN1")
        assert ws.receive_bytes() == b"SCAN1"

    assert gateway._traffic_stats_snapshot()["scan"]["clients"] == 0


def test_teleop_websocket_ignores_malformed_frames_without_motion():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.clients = 0

        def on_client_connect(self):
            self.clients += 1

        def on_client_disconnect(self):
            self.clients -= 1

        def force_release(self):
            pass

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    sent_cmds = []
    gateway.cmd_vel._add_callback(sent_cmds.append)

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop") as ws:
        ws.send_bytes(b"\xff")
        ws.send_text("[1, 2, 3]")
        ws.send_text("not json")
        ws.send_text('{"type":"joy","lx":"bad","ly":0,"az":0,"deadman":true}')
        ws.send_text('{"type":"unknown"}')
        assert gateway._teleop_clients == 1
        assert tracker.clients == 1
        assert sent_cmds == []

    assert gateway._teleop_clients == 0
    assert tracker.clients == 0
    assert sent_cmds == []


def test_teleop_websocket_rejects_joy_when_safety_stop_active():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    class TeleopTracker:
        def __init__(self):
            self.clients = 0
            self.joy_calls = 0

        def on_client_connect(self):
            self.clients += 1

        def on_client_disconnect(self):
            self.clients -= 1

        def force_release(self):
            pass

    gateway = GatewayModule()
    gateway.setup()
    tracker = TeleopTracker()
    gateway._teleop_module = tracker
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    sent_cmds = []
    gateway.cmd_vel._add_callback(sent_cmds.append)

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop") as ws:
        ws.send_text('{"type":"joy","lx":0.2,"ly":0,"az":0.1,"deadman":true}')
        payload = json.loads(ws.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "safety_stop"
        assert gateway._teleop_clients == 1
        assert tracker.clients == 1
        assert tracker.joy_calls == 0
        assert sent_cmds == []

    assert gateway._teleop_clients == 0
    assert tracker.clients == 0
    assert tracker.joy_calls == 0
    assert sent_cmds == []


def test_teleop_websocket_rejects_second_control_owner():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-a"):
        with client.websocket_connect("/ws/teleop?client_id=operator-b") as second:
            payload = json.loads(second.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "lease_conflict"


def test_teleop_websocket_rejects_duplicate_client_id_connection():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-a"):
        with client.websocket_connect("/ws/teleop?client_id=operator-a") as second:
            payload = json.loads(second.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "lease_conflict"


def test_teleop_websocket_requires_explicit_deadman_for_motion():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    sent_cmds = []
    gateway.cmd_vel._add_callback(sent_cmds.append)
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-deadman") as ws:
        ws.send_text('{"type":"joy","lx":0.4,"ly":0,"az":0.1}')
        payload = json.loads(ws.receive_text())

        assert payload == {
            "type": "control_ack",
            "action": "manual_hold",
            "accepted": True,
        }
        assert sent_cmds
        assert sent_cmds[-1].linear.x == 0.0
        assert sent_cmds[-1].linear.y == 0.0
        assert sent_cmds[-1].angular.z == 0.0


def test_teleop_websocket_reports_unconfirmed_manual_hold(monkeypatch):
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule
    from nav.adapters.native.commands import NavigationClientError

    class FailingPublisher:
        def quiesce_and_send_zero(self, *, timeout_s):
            raise NavigationClientError("zero command rejected")

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_dds_enabled = True
    gateway._teleop_native_publisher = FailingPublisher()
    events = []
    gateway.push_event = events.append
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-deadman") as ws:
        ws.send_text('{"type":"joy","lx":0,"ly":0,"az":0,"deadman":false}')
        payload = json.loads(ws.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "manual_hold_unconfirmed"

    assert any(
        event["type"] == "control_rejected" and event["data"]["error"] == "disconnect_zero_unconfirmed"
        for event in events
    )


def test_teleop_websocket_reports_disconnect_zero_when_release_raises():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_dds_enabled = True
    gateway._teleop_release = lambda: (_ for _ in ()).throw(RuntimeError("release crashed"))
    events = []
    gateway.push_event = events.append
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-release-error"):
        pass

    assert any(
        event["type"] == "control_rejected" and event["data"]["error"] == "disconnect_zero_unconfirmed"
        for event in events
    )


def test_teleop_websocket_reports_unavailable_native_command_queue():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._teleop_dds_enabled = True
    gateway._teleop_native_publisher = None
    client = TestClient(gateway._app)

    with client.websocket_connect("/ws/teleop?client_id=operator-native") as ws:
        ws.send_text('{"type":"joy","lx":0.2,"ly":0,"az":0,"deadman":true}')
        payload = json.loads(ws.receive_text())

        assert payload["type"] == "control_rejected"
        assert payload["error"] == "native_command_unavailable"


def test_teleop_camera_frame_task_is_awaited_on_disconnect(monkeypatch):
    from fastapi.testclient import TestClient

    import gateway.routes.realtime as realtime
    from gateway.gateway_module import GatewayModule

    class FakeTask:
        def __init__(self):
            self.cancelled = False
            self.awaited = False

        def cancel(self):
            self.cancelled = True

        def __await__(self):
            async def _complete():
                self.awaited = True
                raise asyncio.CancelledError()

            return _complete().__await__()

    fake_task = FakeTask()

    def fake_create_task(coro):
        coro.close()
        return fake_task

    monkeypatch.setattr(realtime.asyncio, "create_task", fake_create_task)

    gateway = GatewayModule()
    gateway.setup()

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop?camera=1"):
        assert gateway._teleop_clients == 1

    assert fake_task.cancelled is True
    assert fake_task.awaited is True
    assert gateway._teleop_clients == 0


def test_teleop_camera_frame_task_error_still_releases_client(monkeypatch):
    from fastapi.testclient import TestClient

    import gateway.routes.realtime as realtime
    from gateway.gateway_module import GatewayModule

    class FakeTask:
        def __init__(self):
            self.cancelled = False
            self.awaited = False

        def cancel(self):
            self.cancelled = True

        def __await__(self):
            async def _complete():
                self.awaited = True
                raise RuntimeError("camera stream failed during cleanup")

            return _complete().__await__()

    fake_task = FakeTask()

    def fake_create_task(coro):
        coro.close()
        return fake_task

    monkeypatch.setattr(realtime.asyncio, "create_task", fake_create_task)

    gateway = GatewayModule()
    gateway.setup()

    client = TestClient(gateway._app)
    with client.websocket_connect("/ws/teleop?camera=1"):
        assert gateway._teleop_clients == 1

    assert fake_task.cancelled is True
    assert fake_task.awaited is True
    assert gateway._teleop_clients == 0


def test_teleop_client_counter_helpers_are_thread_safe():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    def connect_and_disconnect_many():
        for _ in range(1000):
            gateway._teleop_client_connected()
            gateway._teleop_client_disconnected()

    threads = [threading.Thread(target=connect_and_disconnect_many) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert gateway._teleop_client_count() == 0
    assert gateway._teleop_client_disconnected() == 0


def test_gateway_run_server_reports_failure_without_configured_app():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()

    assert gateway._run_server() is False


def test_gateway_run_server_reports_clean_uvicorn_shutdown(monkeypatch):
    import sys
    import types

    from gateway.gateway_module import GatewayModule

    class FakeConfig:
        def __init__(self, *args, **kwargs):
            pass

    class FakeServer:
        def __init__(self, config):
            self.should_exit = False
            self.force_exit = False

        def run(self):
            self.should_exit = True

    fake_uvicorn = types.ModuleType("uvicorn")
    fake_uvicorn.Config = FakeConfig
    fake_uvicorn.Server = FakeServer
    monkeypatch.setitem(sys.modules, "uvicorn", fake_uvicorn)

    gateway = GatewayModule()
    gateway.setup()

    assert gateway._run_server() is True
    assert gateway._server is None


def test_gateway_run_server_reports_unexpected_uvicorn_return(monkeypatch):
    import sys
    import types

    from gateway.gateway_module import GatewayModule

    class FakeConfig:
        def __init__(self, *args, **kwargs):
            pass

    class FakeServer:
        def __init__(self, config):
            self.should_exit = False
            self.force_exit = False

        def run(self):
            pass

    fake_uvicorn = types.ModuleType("uvicorn")
    fake_uvicorn.Config = FakeConfig
    fake_uvicorn.Server = FakeServer
    monkeypatch.setitem(sys.modules, "uvicorn", fake_uvicorn)

    gateway = GatewayModule()
    gateway.setup()

    assert gateway._run_server() is False
    assert gateway._server is None


def test_gateway_stop_signals_background_threads_without_server_start():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = True
    gateway._drift_watchdog_interval = 60.0
    gateway.setup()
    gateway.start()

    saved_thread = gateway._saved_map_loader_thread
    drift_thread = gateway._drift_watchdog_thread
    assert saved_thread is not None
    assert drift_thread is not None
    assert saved_thread.is_alive()
    assert drift_thread.is_alive()

    gateway.stop()

    assert gateway._saved_map_loader_thread is None
    assert gateway._drift_watchdog_thread is None
    assert not saved_thread.is_alive()
    assert not drift_thread.is_alive()


def test_gateway_start_runs_client_http_prewarm_in_background(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._drift_watchdog_enabled = False
    gateway.setup()

    server_started = threading.Event()
    prewarm_started = threading.Event()

    def fake_run_server(stop_event=None):
        server_started.set()
        (stop_event or gateway._stop_event).wait(1.0)
        return True

    def fake_prewarm(stop_event=None, *, timeout_s=15.0):
        assert timeout_s == 15.0
        prewarm_started.set()
        (stop_event or gateway._stop_event).wait(1.0)
        return False

    monkeypatch.setattr(gateway, "_run_server", fake_run_server)
    monkeypatch.setattr(gateway, "_prewarm_client_http_routes", fake_prewarm)

    started_at = time.perf_counter()
    gateway.start()
    elapsed_s = time.perf_counter() - started_at

    assert elapsed_s < 0.5
    assert server_started.wait(timeout=0.5)
    assert prewarm_started.wait(timeout=0.5)
    assert gateway._client_http_prewarm_thread is not None
    assert gateway._client_http_prewarm_thread.is_alive()

    gateway.stop()

    assert gateway._client_http_prewarm_thread is None


def test_gateway_deferred_mode_can_start_client_http_prewarm(monkeypatch):
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = False
    gateway.setup()

    prewarm_started = threading.Event()

    def fake_prewarm(stop_event=None, *, timeout_s=30.0):
        assert timeout_s == 30.0
        prewarm_started.set()
        (stop_event or gateway._stop_event).wait(1.0)
        return False

    monkeypatch.setattr(gateway, "_prewarm_client_http_routes", fake_prewarm)

    assert gateway._start_client_http_prewarm(timeout_s=30.0) is True
    assert prewarm_started.wait(timeout=0.5)
    assert gateway._client_http_prewarm_thread is not None
    assert gateway._client_http_prewarm_thread.is_alive()

    gateway.stop()

    assert gateway._client_http_prewarm_thread is None


def test_gateway_http_prewarm_does_not_skip_deferred_server(monkeypatch):
    import urllib.request

    from gateway.gateway_module import GatewayModule

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def read(self):
            return b"{}"

    calls = []

    def fake_urlopen(request, timeout=None):
        calls.append((request.full_url, timeout))
        return FakeResponse()

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway.setup()

    assert gateway._prewarm_client_http_routes(threading.Event(), timeout_s=1.0) is True
    assert calls
    assert calls[0][0].endswith("/api/v1/app/capabilities")


def test_gateway_stop_retains_background_thread_when_join_times_out():
    from gateway.gateway_module import GatewayModule

    class StuckThread:
        name = "saved_map_loader"

        def __init__(self):
            self.join_timeouts = []

        def is_alive(self):
            return True

        def join(self, timeout=None):
            self.join_timeouts.append(timeout)

    gateway = GatewayModule()
    old_event = gateway._stop_event
    stuck = StuckThread()
    gateway._saved_map_loader_thread = stuck

    gateway.stop()

    assert old_event.is_set()
    assert stuck.join_timeouts == [2.0]
    assert gateway._saved_map_loader_thread is stuck


def test_gateway_start_replaces_stopped_event_without_duplicate_stuck_loader():
    from gateway.gateway_module import GatewayModule

    class StuckThread:
        name = "saved_map_loader"

        def is_alive(self):
            return True

        def join(self, timeout=None):
            pass

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = False
    gateway.setup()
    old_event = gateway._stop_event
    old_event.set()
    stuck = StuckThread()
    gateway._saved_map_loader_thread = stuck

    gateway.start()

    assert gateway._stop_event is not old_event
    assert old_event.is_set()
    assert gateway._saved_map_loader_thread is stuck

    gateway.stop()


def test_gateway_start_does_not_duplicate_background_threads():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway._defer_server = True
    gateway._drift_watchdog_enabled = True
    gateway._drift_watchdog_interval = 60.0
    gateway.setup()
    gateway.start()

    saved_thread = gateway._saved_map_loader_thread
    drift_thread = gateway._drift_watchdog_thread

    gateway.start()

    assert gateway._saved_map_loader_thread is saved_thread
    assert gateway._drift_watchdog_thread is drift_thread

    gateway.stop()


def test_gateway_stop_signals_uvicorn_server():
    from gateway.gateway_module import GatewayModule

    class FakeServer:
        should_exit = False

    gateway = GatewayModule()
    fake_server = FakeServer()
    gateway._server = fake_server

    gateway.stop()

    assert fake_server.should_exit is True
