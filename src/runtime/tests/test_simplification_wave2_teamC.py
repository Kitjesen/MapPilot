"""Wave 2 Team C — tests for perception algorithm upgrades.

W2-1: bbox_median_depth_to_detection3d — masked-depth median algorithm
W2-3: BPUDetector._generate_masks — proper proto decoding with missing-proto guard
"""

from __future__ import annotations

import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from perception.tracking.projection import (
    CameraIntrinsics,
    bbox_median_depth_to_detection3d,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _det2d(bbox, label="chair", score=0.9):
    """Return a minimal duck-typed Detection2D."""
    d = types.SimpleNamespace()
    d.bbox = np.array(bbox, dtype=np.float32)
    d.label = label
    d.score = score
    d.features = np.array([])
    return d


def _identity_tf():
    return np.eye(4, dtype=np.float64)


def _project_fallback(dets2d, depth, tf, intrinsics=None):
    """Wrapper around bbox_median_depth_to_detection3d for list input."""
    if intrinsics is None:
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)
    results = []
    for det2d in dets2d:
        d3d = bbox_median_depth_to_detection3d(
            det2d,
            depth_image=depth,
            tf_camera_to_world=tf,
            intrinsics=intrinsics,
            depth_scale=0.001,
            min_depth=0.3,
            max_depth=6.0,
            min_valid_pixels=20,
        )
        if d3d is not None:
            results.append(d3d)
    return results


# ---------------------------------------------------------------------------
# W2-1 Tests — bbox_median_depth_to_detection3d
# ---------------------------------------------------------------------------


class TestProjectTo3dFallback(unittest.TestCase):
    def setUp(self):
        self.tf = _identity_tf()
        self.intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0, width=640, height=480)

    def _depth_image(self, h, w, fill_raw=2000):
        """Return uint16-ish depth image where raw * 0.001 = fill_raw * 0.001 metres."""
        return np.full((h, w), fill_raw, dtype=np.uint16)

    # -- W2-1-A: valid bbox with plenty of depth pixels --
    def test_valid_bbox_returns_det3d_with_median_depth(self):
        """A bbox with >20 valid depth pixels should return a detection with median depth."""
        depth = self._depth_image(480, 640, fill_raw=2000)  # 2.0 m everywhere
        det2d = _det2d([100, 100, 200, 200])
        results = _project_fallback([det2d], depth, self.tf, self.intrinsics)

        self.assertEqual(len(results), 1)
        r = results[0]
        self.assertAlmostEqual(r.depth, 2.0, places=3)
        self.assertEqual(r.label, "chair")
        self.assertAlmostEqual(r.score, 0.9, places=5)
        # confidence_3d should be 1.0 because all pixels are valid
        self.assertAlmostEqual(r.confidence_3d, 1.0, places=5)
        # 3D position z should equal depth (identity transform, ray pointing along Z)
        self.assertAlmostEqual(float(r.position[2]), 2.0, places=2)

    # -- W2-1-B: too few valid depth pixels -> None (not added to results) --
    def test_sparse_depth_returns_no_detection(self):
        """Bbox with fewer than 20 valid depth pixels must be skipped."""
        h, w = 480, 640
        depth = np.zeros((h, w), dtype=np.uint16)  # all zeros -> invalid
        # Plant exactly 10 valid pixels in the bbox interior
        for px in range(10):
            depth[110 + px, 110] = 2000
        det2d = _det2d([100, 100, 200, 200])
        results = _project_fallback([det2d], depth, self.tf, self.intrinsics)
        self.assertEqual(len(results), 0, "Sparse depth must produce no detection")

    # -- W2-1-C: median is robust to outliers --
    def test_median_robust_to_outlier_pixels(self):
        """Median depth must not be pulled toward extreme outlier values."""
        h, w = 480, 640
        depth = np.full((h, w), 2000, dtype=np.uint16)  # 2.0 m background
        # Inject a cluster of very-far pixels (8 m -> raw=8000) in the bbox corner
        depth[100:105, 100:115] = 8000  # 75 pixels at 8 m (beyond max_depth=6 -> filtered)
        det2d = _det2d([100, 100, 200, 200])
        results = _project_fallback([det2d], depth, self.tf, self.intrinsics)
        self.assertEqual(len(results), 1)
        # Remaining valid pixels are all 2.0 m; median must be ~2.0 m
        self.assertAlmostEqual(results[0].depth, 2.0, places=2)

    # -- W2-1-D: confidence_3d proportional to valid pixel fraction --
    def test_confidence_3d_reflects_valid_pixel_ratio(self):
        """confidence_3d should be valid_pixels / total_pixels (between 0 and 1)."""
        h, w = 480, 640
        depth = np.full((h, w), 2000, dtype=np.uint16)
        bbox = [100, 100, 200, 200]  # 100x100 = 10000 pixels total in roi
        # Zero out the right half -> ~50% valid
        depth[100:200, 150:200] = 0
        det2d = _det2d(bbox)
        results = _project_fallback([det2d], depth, self.tf, self.intrinsics)
        self.assertEqual(len(results), 1)
        c = results[0].confidence_3d
        self.assertGreater(c, 0.0)
        self.assertLessEqual(c, 1.0)
        # Roughly half the pixels are valid
        self.assertAlmostEqual(c, 0.5, delta=0.02)


# ---------------------------------------------------------------------------
# W2-3 Tests — BPUDetector._generate_masks
# ---------------------------------------------------------------------------


def _make_bpu_detector(**kw):
    """Return a BPUDetector without loading any .hbm model."""
    from perception.detection.bpu_detector import BPUDetector

    det = BPUDetector.__new__(BPUDetector)
    det._conf_thr = 0.25
    det._iou_thr = 0.45
    det._max_detections = 64
    det._min_box_size_px = 12
    det._proto_tensor_name_override = kw.get("proto_tensor_name", "proto")
    det._proto_name = kw.get("proto_name", None)
    det._proto_missing_logged = False
    det._dfl_weights = np.arange(16, dtype=np.float32)
    det._bbox_scales = {}
    det._output_map = []
    det.has_seg = False
    det._model_name_short = "test"
    det._prompt_cache_key = ""
    det._allowed_cids = set()
    det._custom_vocab = None
    det._num_classes = 80
    det._is_yoloe = False
    det._yoloe_det_name = None
    det._yoloe_proto_name = None
    det._rt = None
    det._mname = None
    return det


def _synthetic_outputs(proto_key="proto", proto_shape=(160, 160, 32)):
    """Build a minimal outputs dict with deterministic proto and mask-coeff tensors."""
    rng = np.random.default_rng(42)
    proto = rng.standard_normal(proto_shape).astype(np.float32)
    outputs = {proto_key: np.expand_dims(proto, 0)}  # (1, *proto_shape)
    return outputs


def _make_raw_and_mc(n=2):
    """Return (raw, kept_mc) as _postprocess would output."""
    raw = []
    for i in range(n):
        box = np.array([50 + i * 60, 50, 100 + i * 60, 100], dtype=np.float32)
        raw.append((box, 0.9, 0))
    rng = np.random.default_rng(7)
    kept_mc = rng.standard_normal((n, 32)).astype(np.float32)
    return raw, kept_mc


class TestGenerateMasks(unittest.TestCase):
    # -- W2-3-A: normal path produces boolean masks with bbox intersection --
    def test_masks_are_bool_and_intersect_bbox(self):
        """Each returned mask crop must be a bool array and within bbox bounds."""
        det = _make_bpu_detector(proto_name="proto")
        raw, kept_mc = _make_raw_and_mc(n=2)
        outputs = _synthetic_outputs(proto_key="proto", proto_shape=(160, 160, 32))

        scale, pad_x, pad_y = 1.0, 0, 0
        masks = det._generate_masks(raw, kept_mc, outputs, scale, pad_x, pad_y)

        self.assertEqual(len(masks), 2)
        for i, m in enumerate(masks):
            self.assertIsNotNone(m, f"mask {i} should not be None")
            mask_crop, _ox1, _oy1 = m
            self.assertEqual(mask_crop.dtype, bool, "mask dtype must be bool")
            # Crop dimensions match bbox pixel size
            box = raw[i][0].astype(int)
            expected_h = max(1, box[3] - box[1])
            expected_w = max(1, box[2] - box[0])
            self.assertEqual(mask_crop.shape, (expected_h, expected_w))

    # -- W2-3-B: missing proto logs ERROR once and returns None masks --
    def test_missing_proto_logs_error_once_and_returns_none(self):
        """When proto tensor is absent, log exactly one ERROR and return None masks."""
        det = _make_bpu_detector(proto_name=None, proto_tensor_name="proto")
        raw, kept_mc = _make_raw_and_mc(n=3)
        outputs = {}  # no proto key

        with self.assertLogs("perception.detection.bpu_detector", level="ERROR") as log_ctx:
            masks1 = det._generate_masks(raw, kept_mc, outputs, 1.0, 0, 0)
            # Second call must NOT emit another log (logged once guard)
            masks2 = det._generate_masks(raw, kept_mc, outputs, 1.0, 0, 0)

        # Only one ERROR record should be emitted across both calls
        self.assertEqual(len(log_ctx.records), 1)
        self.assertIn("proto", log_ctx.records[0].message.lower())

        for m in masks1:
            self.assertIsNone(m)
        for m in masks2:
            self.assertIsNone(m)

    # -- W2-3-C: None kept_mc short-circuits immediately (no crash) --
    def test_none_kept_mc_returns_all_none(self):
        """_generate_masks must handle kept_mc=None without error."""
        det = _make_bpu_detector(proto_name="proto")
        raw, _ = _make_raw_and_mc(n=2)
        outputs = _synthetic_outputs()

        masks = det._generate_masks(raw, None, outputs, 1.0, 0, 0)
        self.assertEqual(len(masks), 2)
        for m in masks:
            self.assertIsNone(m)

    # -- W2-3-D: proto_tensor_name_override fallback is used when proto_name absent --
    def test_override_proto_name_is_used_as_fallback(self):
        """When _proto_name is None but override key exists in outputs, masks decode."""
        det = _make_bpu_detector(proto_name=None, proto_tensor_name="my_proto")
        raw, kept_mc = _make_raw_and_mc(n=1)
        outputs = _synthetic_outputs(proto_key="my_proto", proto_shape=(160, 160, 32))

        masks = det._generate_masks(raw, kept_mc, outputs, 1.0, 0, 0)
        self.assertEqual(len(masks), 1)
        self.assertIsNotNone(masks[0])


if __name__ == "__main__":
    unittest.main()
