from __future__ import annotations

from runtime.msgs.numpy_compat import np


def test_lidar_frame_stream_tracks_clouds_and_isolates_callback_errors() -> None:
    from drivers.real.lidar.api.frame_stream import LidarFrameStream

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
    from drivers.real.lidar.api.frame_stream import LidarFrameStream

    stream = LidarFrameStream()

    assert stream.ingest_cloud(None) is False
    assert stream.get_cloud() is None
    assert stream.metrics.total_frames == 0


def test_lidar_frame_stream_tracks_imu_callbacks() -> None:
    from drivers.real.lidar.api.frame_stream import LidarFrameStream

    stream = LidarFrameStream()
    received = []
    imu = object()

    stream.on_imu(received.append)
    stream.ingest_imu(imu)

    assert stream.get_imu() is imu
    assert received == [imu]
