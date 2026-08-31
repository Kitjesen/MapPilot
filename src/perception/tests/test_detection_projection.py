"""Depth projection behavior for 2D detections."""

from types import SimpleNamespace

import numpy as np
import pytest

from perception.tracking.projection import CameraIntrinsics, bbox_median_depth_to_detection3d


INTRINSICS = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)


def _project(depth: np.ndarray):
    detection = SimpleNamespace(
        bbox=np.array([100, 100, 200, 200], dtype=np.float32),
        label="chair",
        score=0.9,
        features=np.array([]),
    )
    return bbox_median_depth_to_detection3d(
        detection,
        depth_image=depth,
        tf_camera_to_world=np.eye(4),
        intrinsics=INTRINSICS,
        depth_scale=0.001,
        min_depth=0.3,
        max_depth=6.0,
        min_valid_pixels=20,
    )


def test_valid_bbox_uses_median_depth():
    result = _project(np.full((480, 640), 2000, dtype=np.uint16))

    assert result is not None
    assert result.depth == pytest.approx(2.0)
    assert result.label == "chair"
    assert result.score == pytest.approx(0.9)
    assert result.confidence_3d == pytest.approx(1.0)
    assert float(result.position[2]) == pytest.approx(2.0, abs=0.02)


def test_sparse_depth_returns_no_detection():
    depth = np.zeros((480, 640), dtype=np.uint16)
    for offset in range(10):
        depth[110 + offset, 110] = 2000

    assert _project(depth) is None


def test_median_depth_rejects_out_of_range_pixels():
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    depth[100:105, 100:115] = 8000

    result = _project(depth)
    assert result is not None
    assert result.depth == pytest.approx(2.0)


def test_confidence_reflects_valid_pixel_ratio():
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    depth[100:200, 150:200] = 0

    result = _project(depth)
    assert result is not None
    assert result.confidence_3d == pytest.approx(0.5, abs=0.02)
