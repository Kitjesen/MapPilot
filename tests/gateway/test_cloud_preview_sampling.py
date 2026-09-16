from __future__ import annotations

import numpy as np
import pytest

from gateway.services.cloud_viewer import CloudViewerService
from runtime.msgs.sensor import PointCloud2
from runtime.utils.binary_codec import decode_pointcloud_frame


@pytest.fixture
def viewer(monkeypatch):
    monkeypatch.setenv("LINGTU_SCAN_VIEWER_MAX_POINTS", "1000")
    service = CloudViewerService(
        queue_put_latest=lambda queue, data, loop, record: queue.put_nowait(data),
        current_loop=lambda: None,
        push_event=lambda event: None,
        session_mode=lambda: "navigating",
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )
    service.configure(scan_viewer_min_interval_s=0.0, cloud_viewer_min_interval_s=0.0)
    return service


@pytest.mark.parametrize("source", ["lidar_scan", "registered_scan"])
def test_scan_point_budget_preserves_both_ends_of_scan(viewer, source):
    points = np.indices((5, 21, 15)).reshape(3, -1).T.astype(np.float32) * 0.3
    queue, _ = viewer.scan_subscribe()

    viewer.handle_scan_cloud(PointCloud2(points=points, frame_id="map"), source=source)

    decoded = decode_pointcloud_frame(queue.get_nowait())
    assert len(decoded.points) == 1000
    np.testing.assert_allclose(decoded.points.min(axis=0), points.min(axis=0), atol=0.011)
    np.testing.assert_allclose(decoded.points.max(axis=0), points.max(axis=0), atol=0.011)


def test_map_point_budget_keeps_regions_after_first_sixty_thousand_points(viewer):
    points = np.indices((41, 41, 41)).reshape(3, -1).T.astype(np.float32) * 0.3
    queue, _ = viewer.cloud_subscribe()

    viewer.handle_view_cloud(
        PointCloud2(points=points, frame_id="map"), source="accumulated", authoritative=True,
    )

    decoded = decode_pointcloud_frame(queue.get_nowait())
    assert len(decoded.points) == 60_000
    assert viewer.cache_point_count() == len(points)
    np.testing.assert_allclose(decoded.points.min(axis=0), points.min(axis=0), atol=0.011)
    np.testing.assert_allclose(decoded.points.max(axis=0), points.max(axis=0), atol=0.011)
