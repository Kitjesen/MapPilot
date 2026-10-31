from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from perception.backends import RgbdObservationSource
from perception.detection.detector_base import Detection2D
from perception.tracking.projection import person_depth_position, person_depth_sample
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat

INTRINSICS = CameraIntrinsics(
    fx=100.0,
    fy=100.0,
    cx=50.0,
    cy=50.0,
    width=100,
    height=100,
    depth_scale=0.001,
    ts=1.0,
    frame_id="camera",
)


def _person_depth_with_center_background() -> np.ndarray:
    depth = np.full((100, 100), 4.0, dtype=np.float32)
    depth[22:70, 26:74] = 2.0
    depth[45:56, 45:56] = 4.0
    return depth


def test_person_depth_uses_supported_torso_layer_not_center_background() -> None:
    depth = _person_depth_with_center_background()

    sample = person_depth_sample(
        depth,
        np.array([10.0, 10.0, 90.0, 90.0]),
        depth_scale=1.0,
    )

    assert sample is not None
    assert sample[2] == pytest.approx(2.0)


def test_person_depth_mask_excludes_valid_background() -> None:
    depth = np.full((100, 100), 4.0, dtype=np.float32)
    mask = np.zeros((100, 100), dtype=bool)
    mask[20:80, 35:65] = True
    depth[mask] = 1.5

    sample = person_depth_sample(
        depth,
        np.array([10.0, 10.0, 90.0, 90.0]),
        mask=mask,
        depth_scale=1.0,
    )

    assert sample is not None
    assert sample[2] == pytest.approx(1.5)


def test_person_depth_returns_none_without_supported_target_depth() -> None:
    depth = np.full((100, 100), 4.0, dtype=np.float32)
    mask = np.zeros((100, 100), dtype=bool)
    mask[20:80, 35:65] = True
    depth[mask] = 0.0

    assert person_depth_sample(
        depth,
        np.array([10.0, 10.0, 90.0, 90.0]),
        mask=mask,
        depth_scale=1.0,
    ) is None


def test_person_depth_fixed_anchor_does_not_follow_layer_pixel_row() -> None:
    bbox = np.array([10.0, 10.0, 90.0, 90.0])
    upper = np.zeros((100, 100), dtype=np.float32)
    lower = np.zeros((100, 100), dtype=np.float32)
    upper[20:39, 40:61] = 2.0
    lower[39:57, 40:61] = 2.0

    upper_sample = person_depth_sample(upper, bbox, depth_scale=1.0)
    lower_sample = person_depth_sample(lower, bbox, depth_scale=1.0)

    assert upper_sample == pytest.approx((50.0, 38.0, 2.0))
    assert lower_sample == pytest.approx(upper_sample)


def test_person_depth_returns_none_when_anchor_patch_has_no_depth() -> None:
    depth = np.full((100, 100), 2.0, dtype=np.float32)
    depth[28:49, 40:61] = 0.0

    assert person_depth_sample(
        depth,
        np.array([10.0, 10.0, 90.0, 90.0]),
        depth_scale=1.0,
    ) is None


def test_person_depth_anchor_is_safe_for_box_clipped_at_image_edge() -> None:
    depth = np.full((100, 100), 2.0, dtype=np.float32)

    sample = person_depth_sample(
        depth,
        np.array([-40.0, -20.0, 20.0, 30.0]),
        depth_scale=1.0,
    )

    assert sample == pytest.approx((10.0, 10.5, 2.0))


def test_person_depth_position_projects_selected_layer() -> None:
    point = person_depth_position(
        _person_depth_with_center_background(),
        np.array([10.0, 10.0, 90.0, 90.0]),
        INTRINSICS,
        depth_scale=1.0,
    )

    assert point is not None
    assert point[2] == pytest.approx(2.0)


def test_rgbd_person_observation_uses_person_depth_path() -> None:
    class Detector:
        def detect(self, _image: np.ndarray, _prompt: str) -> list[Detection2D]:
            return [
                Detection2D(
                    bbox=np.array([10.0, 10.0, 90.0, 90.0]),
                    score=0.9,
                    label="person",
                )
            ]

    source = RgbdObservationSource(
        Detector(),
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
    )
    source.load()
    frame = SimpleNamespace(
        color=Image(
            data=np.zeros((100, 100, 3), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=1.0,
            frame_id="camera",
        ),
        depth=Image(
            data=_person_depth_with_center_background(),
            format=ImageFormat.DEPTH_F32,
            ts=1.0,
            frame_id="camera",
        ),
        intrinsics=INTRINSICS,
        map_from_camera=np.eye(4),
    )

    result = source.observe(frame, "person")

    assert len(result) == 1
    assert result[0].depth == pytest.approx(2.0)
    assert result[0].position[2] == pytest.approx(2.0)
