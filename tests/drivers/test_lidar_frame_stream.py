from __future__ import annotations

from types import SimpleNamespace

from runtime.msgs.numpy_compat import np


def test_lidar_frame_stream_tracks_clouds_and_isolates_callback_errors() -> None:
    from drivers.real.lidar.frame_stream import LidarFrameStream

    stream = LidarFrameStream()
    received = []

    def broken_callback(_pts) -> None:
        raise RuntimeError("callback failed")

    stream.on_cloud(broken_callback)
    stream.on_cloud(received.append)

    pts = np.asarray([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)

    assert stream.ingest_cloud(pts) is True
    assert stream.get_cloud() is pts
    assert stream.wait_for_cloud(timeout=0.01) is pts
    assert len(received) == 1
    assert received[0] is pts
    assert stream.metrics.total_frames == 1
    assert stream.metrics.total_points == 1
    assert stream.metrics.last_frame_points == 1
    assert stream.metrics.last_frame_time > 0
    assert stream.fps >= 1.0


def test_lidar_frame_stream_ignores_empty_cloud_conversion() -> None:
    from drivers.real.lidar.frame_stream import LidarFrameStream

    stream = LidarFrameStream()

    assert stream.ingest_cloud(None) is False
    assert stream.get_cloud() is None
    assert stream.metrics.total_frames == 0


def test_lidar_frame_stream_tracks_imu_callbacks() -> None:
    from drivers.real.lidar.frame_stream import LidarFrameStream

    stream = LidarFrameStream()
    received = []
    imu = object()

    stream.on_imu(received.append)
    stream.ingest_imu(imu)

    assert stream.get_imu() is imu
    assert received == [imu]


def test_lidar_scan_and_imu_callbacks_use_frame_stream(monkeypatch) -> None:
    from drivers.real.lidar import _dds as livox_dds
    from drivers.real.lidar import lidar as lidar_module

    lidar = lidar_module.Lidar()
    cloud_callbacks = []
    raw_callbacks = []
    imu_callbacks = []
    cloud = np.asarray([[1.0, 2.0, 3.0, 9.0], [4.0, 5.0, 6.0, 8.0]], dtype=np.float32)
    imu = object()
    raw = SimpleNamespace(to_xyzi=lambda: cloud)

    monkeypatch.setattr(livox_dds, "livox_msg_to_frame", lambda _msg: raw)
    monkeypatch.setattr(livox_dds, "dds_imu_to_imu", lambda _msg: imu)

    lidar.on_cloud(cloud_callbacks.append)
    lidar.on_raw_cloud(raw_callbacks.append)
    lidar.on_imu(imu_callbacks.append)

    lidar._on_scan(object())
    lidar._on_imu(object())

    assert lidar.get_cloud() is cloud
    assert lidar.get_raw_cloud() is raw
    assert lidar.wait_for_cloud(timeout=0.01) is cloud
    assert lidar.get_imu() is imu
    assert raw_callbacks == [raw]
    assert len(cloud_callbacks) == 1
    assert cloud_callbacks[0] is cloud
    assert imu_callbacks == [imu]

    health = lidar.health
    assert health.total_frames == 1
    assert health.total_points == 2
    assert health.last_frame_points == 2
    assert health.last_frame_time > 0
