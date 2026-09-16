from __future__ import annotations

import math

import numpy as np
import pytest

from gateway.services import cloud_viewer as cloud_viewer_module
from gateway.services.cloud_viewer import CloudViewerService
from runtime.msgs.geometry import Pose, Quaternion, Transform, Vector3
from runtime.msgs.map import MapObservationFrame
from runtime.msgs.sensor import PointCloud2
from runtime.utils.binary_codec import decode_pointcloud_frame


@pytest.fixture
def viewer(monkeypatch):
    monkeypatch.setattr(cloud_viewer_module.time, "time", lambda: 100.0)
    events = []

    def deliver(queue, data, loop, record):
        if queue.full():
            queue.get_nowait()
        queue.put_nowait(data)

    service = CloudViewerService(
        queue_put_latest=deliver,
        current_loop=lambda: None,
        push_event=events.append,
        session_mode=lambda: "navigating",
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )
    service.configure(scan_viewer_min_interval_s=0.0)
    queue, _ = service.scan_subscribe()
    return service, queue, events


def observation(*, epoch=10, sequence=1, stamp=99.9, frame="map"):
    translation = Vector3(10.0, -2.0, 3.0)
    rotation = Quaternion.from_euler(0.0, 0.0, math.pi / 2)
    return MapObservationFrame(
        points=np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 1.0]], dtype=np.float32),
        reset_epoch=epoch,
        sequence=sequence,
        ts=stamp,
        frame_id=frame,
        sensor_frame_id="body",
        sensor_origin=translation,
        map_sensor_pose=Pose(translation, rotation),
        map_sensor_transform=Transform(
            translation=translation,
            rotation=rotation,
            frame_id=frame,
            child_frame_id="body",
            ts=stamp,
        ),
    )


def test_registered_scan_uses_scan_time_rotation_and_translation_without_making_a_map(viewer):
    service, queue, _ = viewer
    service.on_map_observation(observation())
    decoded = decode_pointcloud_frame(queue.get_nowait())
    points = decoded.points[np.argsort(decoded.points[:, 0])]
    np.testing.assert_allclose(points, [[8.0, -2.0, 4.0], [10.0, -1.0, 3.0]], atol=0.011)
    assert decoded.frame_id == "map"
    assert decoded.stream_kind == "scan"
    assert decoded.stamp_s == 99.9
    assert service.cache_point_count() == 0
    assert service.debug_snapshot()["has_latest_binary_frame"] is False
    metadata = service.debug_snapshot()["latest_scan_frame"]
    assert metadata["source"] == "registered_scan"
    assert metadata["source_epoch"] == 10
    assert metadata["observation_sequence"] == 1
    assert metadata["sensor_frame_id"] == "body"


def test_untransformed_sensor_frame_is_not_relabelled_as_map(viewer):
    service, queue, _ = viewer
    service.on_lidar_scan(PointCloud2(points=np.array([[1.0, 0.0, 0.0]]), frame_id="lidar_link"))
    service.on_map_observation(observation(frame="lidar_link"))
    assert queue.empty()
    assert service.debug_snapshot()["scan_incompatible_frame_drops"] == 2


@pytest.mark.parametrize("stamp", [97.9, 101.1])
def test_registered_scan_rejects_old_or_future_source_timestamps(viewer, stamp):
    service, queue, _ = viewer
    service.on_map_observation(observation(stamp=stamp))
    assert queue.empty()
    assert service.debug_snapshot()["stale_observation_drops"] == 1


def test_registered_scan_epoch_reset_clears_old_scene_and_rejects_old_samples(viewer):
    service, queue, events = viewer
    service.on_map_observation(observation(sequence=20))
    before = decode_pointcloud_frame(queue.get_nowait())
    service.on_map_observation(observation(epoch=11, sequence=1))
    reset = decode_pointcloud_frame(queue.get_nowait())
    current = decode_pointcloud_frame(queue.get_nowait())
    assert reset.stream_kind == "reset"
    assert current.epoch == reset.epoch != before.epoch
    assert any(event.get("reason") == "slam_observation_epoch_changed" for event in events)
    service.on_map_observation(observation(epoch=10, sequence=21))
    service.on_map_observation(observation(epoch=11, sequence=1))
    assert queue.empty()

    service.clear("localization_restart")
    assert decode_pointcloud_frame(queue.get_nowait()).stream_kind == "reset"
    service.on_map_observation(observation(epoch=10, sequence=22))
    assert queue.empty()
    service.on_map_observation(observation(epoch=11, sequence=2))
    assert decode_pointcloud_frame(queue.get_nowait()).stream_kind == "scan"


def test_unobserved_scan_does_not_encode_but_next_subscriber_gets_new_sample(monkeypatch):
    monkeypatch.setattr(cloud_viewer_module.time, "time", lambda: 100.0)
    service = CloudViewerService(
        queue_put_latest=lambda queue, data, loop, record: queue.put_nowait(data),
        current_loop=lambda: None,
        push_event=lambda event: None,
        session_mode=lambda: "navigating",
        active_session_map=lambda: None,
        saved_active_map=lambda: None,
    )
    service.on_map_observation(observation())
    queue, latest = service.scan_subscribe()
    assert latest is None
    service.on_map_observation(observation(sequence=2))
    assert decode_pointcloud_frame(queue.get_nowait()).frame_id == "map"
