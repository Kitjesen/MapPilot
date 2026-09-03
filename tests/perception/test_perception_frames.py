from __future__ import annotations

from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from perception.frames import FrameDrop, FrameSynchronizer, SynchronizedFrame
from perception.tracking.projection import project_to_3d
from runtime.msgs.geometry import Pose
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


def _color(ts: float, *, frame_id: str = "camera_link") -> Image:
    return Image(
        data=np.zeros((2, 2, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        ts=ts,
        frame_id=frame_id,
    )


def _depth(ts: float, *, frame_id: str = "camera_link") -> Image:
    return Image(
        data=np.ones((2, 2), dtype=np.uint16),
        format=ImageFormat.DEPTH_U16,
        ts=ts,
        frame_id=frame_id,
    )


def _intrinsics(ts: float = 1.0, *, frame_id: str = "camera_link") -> CameraIntrinsics:
    return CameraIntrinsics(
        fx=100.0,
        fy=100.0,
        cx=1.0,
        cy=1.0,
        width=2,
        height=2,
        depth_scale=0.001,
        ts=ts,
        frame_id=frame_id,
    )


def _odom(ts: float, *, x: float = 0.0, frame_id: str = "map") -> Odometry:
    return Odometry(
        pose=Pose(x, 0.0, 0.0),
        ts=ts,
        frame_id=frame_id,
        child_frame_id="body",
    )


def _map_odom(ts: float, *, x: float = 0.0) -> dict[str, object]:
    return {
        "valid": True,
        "frame_id": "map",
        "child_frame_id": "odom",
        "tx": x,
        "ty": 0.0,
        "tz": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
        "ts": ts,
    }


def _synchronizer(**overrides: object) -> FrameSynchronizer:
    options: dict[str, object] = {
        "camera_to_body": np.eye(4),
        "skip_frames": 1,
        "max_rgbd_skew_s": 0.05,
        "max_odom_age_s": 0.1,
        "max_map_odom_age_s": 0.5,
    }
    options.update(overrides)
    return FrameSynchronizer(**options)


def _prime(sync: FrameSynchronizer, *, ts: float = 10.0) -> None:
    assert sync.push_camera_info(_intrinsics(ts)) == ()
    assert sync.push_odometry(_odom(ts)) == ()


def test_exact_rgbd_timestamp_pairs_immediately_and_preserves_messages() -> None:
    sync = _synchronizer()
    info = _intrinsics()
    odom = _odom(10.0, x=2.0)
    color = _color(10.0)
    depth = _depth(10.0)

    assert sync.push_camera_info(info) == ()
    assert sync.push_odometry(odom) == ()
    assert sync.push_depth(depth) == ()
    results = sync.push_color(color)

    frames = [result for result in results if isinstance(result, SynchronizedFrame)]
    assert len(frames) == 1
    frame = frames[0]
    assert isinstance(frame, SynchronizedFrame)
    assert frame.color is color
    assert frame.depth is depth
    assert frame.intrinsics is info
    assert frame.odometry is odom
    assert frame.map_odom is None
    assert frame.timestamp == 10.0
    assert frame.color_ts == 10.0
    assert frame.frame_id == "map"
    np.testing.assert_allclose(frame.map_from_body[:3, 3], [2.0, 0.0, 0.0])
    np.testing.assert_allclose(frame.map_from_camera, frame.map_from_body)


def test_color_before_exact_depth_pairs_when_depth_arrives() -> None:
    sync = _synchronizer()
    _prime(sync)

    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_depth(_depth(10.0))

    assert len(results) == 1
    assert isinstance(results[0], SynchronizedFrame)


def test_non_exact_pair_waits_for_depth_watermark_then_uses_closest_depth() -> None:
    sync = _synchronizer()
    _prime(sync)
    earlier = _depth(9.97)
    later = _depth(10.04)

    assert sync.push_depth(earlier) == ()
    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_depth(later)

    frames = [result for result in results if isinstance(result, SynchronizedFrame)]
    assert len(frames) == 1
    frame = frames[0]
    assert isinstance(frame, SynchronizedFrame)
    assert frame.depth is earlier


def test_exact_pair_has_priority_and_depth_is_consumed_once() -> None:
    sync = _synchronizer(max_rgbd_skew_s=0.05)
    _prime(sync, ts=10.02)
    exact_color = _color(10.02)

    assert sync.push_color(_color(10.0)) == ()
    assert sync.push_color(exact_color) == ()
    results = sync.push_depth(_depth(10.02))

    assert len(results) == 2
    frames = [result for result in results if isinstance(result, SynchronizedFrame)]
    drops = [result for result in results if isinstance(result, FrameDrop)]
    assert [frame.color for frame in frames] == [exact_color]
    assert [(drop.reason, drop.timestamp) for drop in drops] == [
        ("rgb_depth_time_skew", 10.0)
    ]


def test_skip_counts_color_inputs_and_camera_info_can_update() -> None:
    sync = _synchronizer(skip_frames=2)
    first_info = _intrinsics(ts=9.0)
    second_info = _intrinsics(ts=10.0)
    sync.push_camera_info(first_info)
    sync.push_odometry(_odom(10.0))

    assert sync.push_color(_color(9.0)) == ()
    assert sync.push_camera_info(second_info) == ()
    assert sync.push_depth(_depth(10.0)) == ()
    results = sync.push_color(_color(10.0))

    assert len(results) == 1
    frame = results[0]
    assert isinstance(frame, SynchronizedFrame)
    assert frame.intrinsics is second_info
    assert sync.health()["color_frames"] == 2


def test_rgbd_waits_for_late_camera_info_and_odometry() -> None:
    sync = _synchronizer()
    color = _color(10.0)
    depth = _depth(10.0)

    assert sync.push_color(color) == ()
    assert sync.push_depth(depth) == ()
    assert sync.push_camera_info(_intrinsics(ts=10.0)) == ()
    results = sync.push_odometry(_odom(10.0))

    assert len(results) == 1
    assert isinstance(results[0], SynchronizedFrame)


@pytest.mark.parametrize(
    "info",
    [
        CameraIntrinsics(
            fx=0.0, fy=100.0, cx=1.0, cy=1.0, width=2, height=2
        ),
        CameraIntrinsics(
            fx=float("nan"), fy=100.0, cx=1.0, cy=1.0, width=2, height=2
        ),
        CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            cx=1.0,
            cy=1.0,
            width=2,
            height=2,
            dist_k1=float("inf"),
        ),
        CameraIntrinsics(
            fx=100.0, fy=100.0, cx=1.0, cy=1.0, width=3, height=2
        ),
    ],
)
def test_invalid_camera_intrinsics_drop_the_rgbd_frame(
    info: CameraIntrinsics,
) -> None:
    sync = _synchronizer()
    sync.push_camera_info(info)
    sync.push_odometry(_odom(10.0))
    sync.push_depth(_depth(10.0))

    results = sync.push_color(_color(10.0))

    assert results == (
        FrameDrop(reason="invalid_camera_intrinsics", timestamp=10.0),
    )
    assert sync.health()["matched_frames"] == 0
    assert sync.health()["drop_reasons"]["invalid_camera_intrinsics"] == 1


@pytest.mark.parametrize(
    "image",
    [
        Image(
            data=np.zeros((2,), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=10.0,
        ),
        Image(
            data=np.zeros((2, 2), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=10.0,
        ),
        Image(
            data=np.zeros((2, 2, 3), dtype=np.uint8),
            format=ImageFormat.RGBA,
            ts=10.0,
        ),
        Image(
            data=np.zeros((2, 2), dtype=np.uint16),
            format=ImageFormat.DEPTH_U16,
            ts=10.0,
        ),
    ],
)
def test_malformed_color_images_are_dropped_before_buffering(image: Image) -> None:
    sync = _synchronizer()

    assert sync.push_color(image) == (
        FrameDrop(reason="invalid_color_image", timestamp=10.0),
    )
    assert sync.health()["buffer_sizes"]["color"] == 0


@pytest.mark.parametrize(
    "image",
    [
        Image(
            data=np.zeros((2, 2, 3), dtype=np.uint16),
            format=ImageFormat.DEPTH_U16,
            ts=10.0,
        ),
        Image(
            data=np.zeros((2, 2), dtype=np.uint8),
            format=ImageFormat.GRAY,
            ts=10.0,
        ),
    ],
)
def test_malformed_depth_images_are_dropped_before_buffering(image: Image) -> None:
    sync = _synchronizer()

    assert sync.push_depth(image) == (
        FrameDrop(reason="invalid_depth_image", timestamp=10.0),
    )
    assert sync.health()["buffer_sizes"]["depth"] == 0


def test_projection_rejects_invalid_focal_lengths() -> None:
    invalid = CameraIntrinsics(
        fx=0.0,
        fy=100.0,
        cx=1.0,
        cy=1.0,
        width=2,
        height=2,
    )

    with pytest.raises(ValueError, match="focal"):
        project_to_3d(1.0, 1.0, 2.0, invalid)


def test_rgbd_waits_for_matching_odometry_when_an_old_sample_exists() -> None:
    sync = _synchronizer(max_odom_age_s=0.1)
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(9.8))
    sync.push_depth(_depth(10.0))

    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_odometry(_odom(10.0, x=2.0))

    frames = [result for result in results if isinstance(result, SynchronizedFrame)]
    assert len(frames) == 1
    frame = frames[0]
    assert frame.odometry.ts == 10.0
    np.testing.assert_allclose(frame.map_from_body[:3, 3], [2.0, 0.0, 0.0])


def test_rgbd_waits_for_matching_map_transform_when_an_old_sample_exists() -> None:
    sync = _synchronizer(max_map_odom_age_s=0.5)
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(10.0, frame_id="odom"))
    sync.push_map_odom(_map_odom(9.0))
    sync.push_depth(_depth(10.0))

    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_map_odom(_map_odom(10.0, x=4.0))

    frames = [result for result in results if isinstance(result, SynchronizedFrame)]
    assert len(frames) == 1
    frame = frames[0]
    assert frame.map_odom is not None
    assert frame.map_odom.ts == 10.0
    np.testing.assert_allclose(frame.map_from_body[:3, 3], [4.0, 0.0, 0.0])


def test_future_odometry_does_not_finalize_missing_map_transform() -> None:
    sync = _synchronizer(max_odom_age_s=0.1, max_map_odom_age_s=0.5)
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(10.0, frame_id="odom"))
    sync.push_depth(_depth(10.0))
    sync.push_color(_color(10.0))

    assert sync.push_odometry(_odom(10.6, frame_id="odom")) == ()
    results = sync.push_map_odom(_map_odom(10.0, x=4.0))

    frames = [result for result in results if isinstance(result, SynchronizedFrame)]
    assert len(frames) == 1
    np.testing.assert_allclose(frames[0].map_from_body[:3, 3], [4.0, 0.0, 0.0])


def test_depth_watermark_finalizes_impossible_color_without_pose_support() -> None:
    sync = _synchronizer(max_rgbd_skew_s=0.05)

    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_depth(_depth(10.2))

    assert FrameDrop(reason="rgb_depth_time_skew", timestamp=10.0) in results
    assert sync.health()["buffer_sizes"]["color"] == 0


@pytest.mark.parametrize(
    ("color_frame", "depth_frame", "info_frame", "reason"),
    [
        ("camera_link", "other_camera", "camera_link", "rgb_depth_frame_mismatch"),
        ("camera_link", "camera_link", "other_camera", "camera_info_frame_mismatch"),
    ],
)
def test_camera_frame_mismatches_are_diagnostic_drops(
    color_frame: str,
    depth_frame: str,
    info_frame: str,
    reason: str,
) -> None:
    sync = _synchronizer()
    sync.push_camera_info(_intrinsics(frame_id=info_frame))
    sync.push_odometry(_odom(10.0))
    sync.push_depth(_depth(10.0, frame_id=depth_frame))

    results = sync.push_color(_color(10.0, frame_id=color_frame))

    assert results == (FrameDrop(reason=reason, timestamp=10.0),)


def test_empty_camera_info_frame_is_accepted() -> None:
    sync = _synchronizer()
    sync.push_camera_info(_intrinsics(frame_id=""))
    sync.push_odometry(_odom(10.0))
    sync.push_depth(_depth(10.0))

    assert isinstance(sync.push_color(_color(10.0))[0], SynchronizedFrame)


def test_closest_odometry_is_selected_and_aged_against_color() -> None:
    sync = _synchronizer(max_odom_age_s=0.1)
    sync.push_camera_info(_intrinsics())
    older = _odom(9.94, x=1.0)
    closer = _odom(10.03, x=2.0)
    sync.push_odometry(older)
    sync.push_odometry(closer)
    sync.push_depth(_depth(10.0))

    result = sync.push_color(_color(10.0))[0]

    assert isinstance(result, SynchronizedFrame)
    assert result.odometry is closer
    np.testing.assert_allclose(result.map_from_body[:3, 3], [2.0, 0.0, 0.0])


def test_stale_odometry_is_dropped() -> None:
    sync = _synchronizer(max_odom_age_s=0.1)
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(9.8))
    sync.push_depth(_depth(10.0))

    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_odometry(_odom(10.2))

    assert FrameDrop(reason="odom_time_skew", timestamp=10.0) in results


def test_odom_frame_requires_fresh_map_transform_and_composes_camera_pose() -> None:
    camera_to_body = np.eye(4)
    camera_to_body[0, 3] = 0.5
    sync = _synchronizer(camera_to_body=camera_to_body)
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(10.0, x=2.0, frame_id="odom"))
    sync.push_map_odom(_map_odom(10.0, x=10.0))
    sync.push_depth(_depth(10.0))

    result = sync.push_color(_color(10.0))[0]

    assert isinstance(result, SynchronizedFrame)
    assert result.map_odom is not None
    np.testing.assert_allclose(result.map_from_body[:3, 3], [12.0, 0.0, 0.0])
    np.testing.assert_allclose(result.map_from_camera[:3, 3], [12.5, 0.0, 0.0])


def test_odom_frame_rejects_stale_map_transform_after_map_watermark() -> None:
    sync = _synchronizer(max_map_odom_age_s=0.5)
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(10.0, frame_id="odom"))
    sync.push_map_odom(_map_odom(9.0))
    sync.push_depth(_depth(10.0))

    assert sync.push_color(_color(10.0)) == ()
    results = sync.push_map_odom(_map_odom(10.6))

    assert FrameDrop(reason="map_odom_time_skew", timestamp=10.0) in results


def test_invalid_map_transform_is_reported_without_entering_buffer() -> None:
    sync = _synchronizer()

    assert sync.push_map_odom({"valid": False, "ts": 7.0}) == (
        FrameDrop(reason="invalid_map_odom_transform", timestamp=7.0),
    )
    assert sync.health()["buffer_sizes"]["map_odom"] == 0


@pytest.mark.parametrize(
    ("stream", "limit", "reason"),
    [
        ("color", 8, "color_buffer_overflow"),
        ("depth", 8, "depth_buffer_overflow"),
        ("odometry", 64, "odometry_buffer_overflow"),
        ("map_odom", 16, "map_odom_buffer_overflow"),
    ],
)
def test_buffers_have_hard_caps(stream: str, limit: int, reason: str) -> None:
    sync = _synchronizer()
    push = {
        "color": lambda index: sync.push_color(_color(10.0)),
        "depth": lambda index: sync.push_depth(_depth(10.0)),
        "odometry": lambda index: sync.push_odometry(_odom(10.0)),
        "map_odom": lambda index: sync.push_map_odom(_map_odom(10.0)),
    }[stream]

    results: tuple[object, ...] = ()
    for index in range(limit + 1):
        results = push(index)

    assert FrameDrop(reason=reason, timestamp=10.0) in results
    assert sync.health()["buffer_sizes"][stream] == limit


@pytest.mark.parametrize(
    ("stream", "expected_reason"),
    [
        ("depth", "depth_expired"),
        ("odometry", "odometry_expired"),
        ("map_odom", "map_odom_expired"),
    ],
)
def test_support_buffers_are_pruned_by_their_time_window(
    stream: str,
    expected_reason: str,
) -> None:
    sync = _synchronizer(
        max_rgbd_skew_s=0.05,
        max_odom_age_s=0.1,
        max_map_odom_age_s=0.5,
    )
    push = {
        "depth": lambda ts: sync.push_depth(_depth(ts)),
        "odometry": lambda ts: sync.push_odometry(_odom(ts)),
        "map_odom": lambda ts: sync.push_map_odom(_map_odom(ts)),
    }[stream]

    assert push(10.0) == ()
    results = push(11.0)

    assert FrameDrop(reason=expected_reason, timestamp=10.0) in results
    assert sync.health()["buffer_sizes"][stream] == 1


@pytest.mark.parametrize("stream", ["color", "depth", "odometry"])
def test_invalid_sample_timestamps_are_dropped_before_buffering(stream: str) -> None:
    sync = _synchronizer()
    push = {
        "color": lambda: sync.push_color(_color(float("nan"))),
        "depth": lambda: sync.push_depth(_depth(float("nan"))),
        "odometry": lambda: sync.push_odometry(_odom(float("nan"))),
    }[stream]

    results = push()

    reason = "invalid_odometry_timestamp" if stream == "odometry" else "invalid_sample_timestamp"
    assert results == (FrameDrop(reason=reason, timestamp=0.0),)
    assert sync.health()["buffer_sizes"][stream] == 0


def test_unsupported_odometry_frame_is_dropped() -> None:
    sync = _synchronizer()
    sync.push_camera_info(_intrinsics())
    sync.push_odometry(_odom(10.0, frame_id="world"))
    sync.push_depth(_depth(10.0))

    assert sync.push_color(_color(10.0)) == (
        FrameDrop(reason="odometry_frame_unsupported", timestamp=10.0),
    )


def test_result_is_frozen_and_pose_matrices_are_read_only() -> None:
    sync = _synchronizer()
    _prime(sync)
    sync.push_depth(_depth(10.0))
    result = sync.push_color(_color(10.0))[0]
    assert isinstance(result, SynchronizedFrame)

    with pytest.raises(FrozenInstanceError):
        result.timestamp = 11.0  # type: ignore[misc]
    with pytest.raises(ValueError):
        result.map_from_body[0, 0] = 2.0


def test_health_is_a_thread_safe_detached_snapshot() -> None:
    sync = _synchronizer()
    sync.push_map_odom({"valid": False, "ts": 7.0})

    first = sync.health()
    first["drop_reasons"]["tampered"] = 99
    first["buffer_sizes"]["color"] = 99
    second = sync.health()

    assert "tampered" not in second["drop_reasons"]
    assert second["buffer_sizes"]["color"] == 0


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("max_rgbd_skew_s", -0.01),
        ("max_odom_age_s", float("nan")),
        ("max_map_odom_age_s", float("inf")),
    ],
)
def test_time_windows_must_be_finite_and_non_negative(name: str, value: float) -> None:
    with pytest.raises(ValueError, match=name):
        _synchronizer(**{name: value})
