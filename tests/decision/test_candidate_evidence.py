"""Measured geometry and expiring review labels, without a model or robot."""

from dataclasses import replace

import numpy as np
import pytest

from decision.vision.candidates import PersonReviewCache, candidate_geometry
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D, SemanticObservation
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


def observation(depth, scale):
    h, w = depth.shape
    transform = np.eye(4)
    transform[:3, 3] = [1, 2, 0]
    return SemanticObservation(
        source_ts=100, map_id="map-a", map_from_camera=transform,
        image=Image(np.zeros((h, w, 3), dtype=np.uint8), ImageFormat.BGR, ts=100),
        depth=Image(depth, ImageFormat.DEPTH_U16 if depth.dtype == np.uint16 else ImageFormat.DEPTH_F32, ts=100.02),
        depth_scale_m=scale, depth_range_m=(.3, 6),
        camera_info=CameraIntrinsics(fx=60, fy=60, cx=w / 2, cy=h / 2, width=w, height=h),
    )


@pytest.mark.parametrize("raw,scale", [(np.full((64, 64), 2500, np.uint16), .001),
                                      (np.full((64, 64), 2.5, np.float32), 1.)])
def test_units_and_coordinate_frames_are_explicit(raw, scale):
    result = candidate_geometry(observation(raw, scale), [16, 16, 48, 48], [1, 2, 2.5])
    assert result["position_m"] == [1, 2, 2.5]
    assert result["camera_position_m"] == [0, 0, 2.5]
    assert result["distance_m"] == 2.5
    depth = result["depth"]
    assert depth["median_m"] == depth["background_median_m"] == 2.5
    assert depth["foreground_gap_m"] == depth["plane_rms_m"] == 0
    assert depth["valid_fraction"] == 1
    assert depth["source_skew_s"] == pytest.approx(.02)
    assert "kind" not in result


def test_background_and_invalid_depth_do_not_become_person_distance():
    raw = np.full((64, 64), 5., np.float32)
    raw[16:48, 16:48] = 2
    raw[16:24, 16:48] = np.nan
    raw[24:32, 16:48] = 0
    raw[32:40, 16:48] = 20
    result = candidate_geometry(observation(raw, 1.), [16, 16, 48, 48], [1, 2, 2])
    depth = result["depth"]
    assert depth["valid_fraction"] == .25
    assert depth["median_m"] == 2 and depth["background_median_m"] == 5
    assert depth["foreground_gap_m"] == 3
    assert result["distance_m"] == 2


@pytest.mark.parametrize("case,reason", [("missing", "missing_depth"), ("units", "depth_units_unknown"),
    ("shape", "depth_not_aligned"), ("empty", "no_valid_depth")])
def test_missing_geometry_remains_unknown(case, reason):
    obs = observation(np.zeros((64, 64), np.float32), 1.)
    if case == "missing":
        obs = replace(obs, depth=None)
    elif case == "units":
        obs = replace(obs, depth_scale_m=None)
    elif case == "shape":
        obs.depth.data = np.ones((16, 16), np.float32)
    result = candidate_geometry(obs, [16, 16, 48, 48], [1, 2, 2])
    assert result["depth"]["available"] is False and result["depth"]["reason"] == reason
    assert candidate_geometry(None, [0, 0, 10, 10], [0, 0, 0])["position_m"] is None


def test_plane_residual_uses_calibrated_rays_with_distortion():
    import cv2

    obs = observation(np.full((64, 64), 2., np.float32), 1.)
    obs.camera_info.dist_k1 = .3
    u, v = np.meshgrid(np.arange(64), np.arange(64))
    pixels = np.column_stack((u.ravel(), v.ravel())).astype(float)
    rays = cv2.undistortPoints(pixels[:, None, :], obs.camera_info.K_matrix, obs.camera_info.D_vector)[:, 0, :]
    obs.depth.data = (2 / (1 + .5 * rays[:, 0] + .2 * rays[:, 1])).reshape(64, 64).astype(np.float32)
    depth = candidate_geometry(obs, [1, 1, 63, 63], [1, 2, 2])["depth"]
    assert depth["p90_m"] - depth["p10_m"] > .2
    assert depth["plane_rms_m"] < 1e-6


def detection(identity="poster", x=1., box=(8, 8, 40, 56)):
    return Detection3D(id=identity, label="person", position=Vector3(x, 0, 2), bbox_2d=list(box))


def reviewed_cache():
    cache = PersonReviewCache(max_gap_s=5)
    image = np.full((64, 64, 3), 80, np.uint8)
    cache.update([detection()], image, 100, map_valid=True, now=1)
    captured = cache.capture()
    cache.remember({"poster": {"kind": "non_person", "reason": "advertisement"}}, captured, now=1)
    return cache, image, captured


def test_negative_review_is_reused_only_for_the_original_track():
    cache, image, _ = reviewed_cache()
    items = [detection(), detection("new-person")]
    cache.update(items, image, 100.1, map_valid=True, now=2)
    assert [item.id for item in cache.filter(items)] == ["new-person"]
    assert cache.verdict("poster")["review_source"] == "track_cache"
    assert len(items) == 2  # Raw perception is retained.
    cache.remember({"new-person": {"kind": "unknown"}}, cache.capture(), now=2)
    assert cache.verdict("new-person") is None


@pytest.mark.parametrize("change", ["position", "bbox", "appearance", "gap", "expired", "missing_image"])
def test_changed_evidence_or_expiry_removes_the_negative_label(change):
    cache, image, _ = reviewed_cache()
    item = detection()
    source_ts, now = 100.1, 2
    if change == "position":
        item.position.x = 1.31
    elif change == "bbox":
        item.bbox_2d = [32, 8, 63, 56]
    elif change == "appearance":
        image[:] = 200
    elif change == "gap":
        source_ts = 106
    elif change == "expired":
        now = 32
    else:
        image = None
    cache.update([item], image, source_ts, map_valid=True, now=now)
    assert cache.verdict("poster") is None
    assert cache.filter([item]) == [item]


def test_slow_drift_is_compared_to_the_reviewed_position_not_last_frame():
    cache, image, _ = reviewed_cache()
    for index, x in enumerate([1.1, 1.2, 1.31], 1):
        cache.update([detection(x=x)], image, 100 + index * .1, map_valid=True, now=1 + index)
    assert cache.verdict("poster") is None


def test_disappearance_prevents_late_review_from_labelling_a_reused_id():
    cache, image, captured = reviewed_cache()
    cache.update([], image, 100.1, map_valid=True, now=2)
    cache.update([detection()], image, 100.2, map_valid=True, now=3)
    cache.remember({"poster": {"kind": "non_person"}}, captured, now=3)
    assert cache.verdict("poster") is None
