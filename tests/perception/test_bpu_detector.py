import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from perception.detection.bpu_detector import BPUDetector
from perception.detection.detector_base import Detection2D


class TestBPUDetectorRecallTuning(unittest.TestCase):
    def test_system_prompt_free_yoloe_is_a_default_candidate(self):
        self.assertIn(
            "/opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm",
            BPUDetector.MODEL_CANDIDATES,
        )

    def test_loads_newline_vocabulary_for_system_prompt_free_yoloe(self):
        detector = BPUDetector()
        with tempfile.TemporaryDirectory() as tmpdir:
            vocab_path = Path(tmpdir) / "coco_extended.names"
            vocab_path.write_text("door\nfire extinguisher\nvalve\n", encoding="utf-8")
            with patch.object(detector, "SYSTEM_YOLOE_VOCAB", str(vocab_path)):
                vocab = detector._load_custom_vocab(
                    "/opt/hobot/model/s100/basic/yoloe_11s_seg_pf_nashe_640x640_nv12.hbm",
                    __import__("glob"),
                )

        self.assertEqual(vocab, {0: "door", 1: "fire extinguisher", 2: "valve"})

    def test_small_box_threshold_is_configurable(self):
        detector = BPUDetector(min_box_size_px=12)

        self.assertTrue(detector._is_box_large_enough(12, 24))
        self.assertFalse(detector._is_box_large_enough(11, 24))
        self.assertFalse(detector._is_box_large_enough(24, 11))

    def test_limit_detection_results_prefers_highest_scores(self):
        detector = BPUDetector(max_detections=2)
        results = [
            Detection2D(bbox=np.array([0, 0, 10, 10], dtype=np.float32), score=0.30, label="person"),
            Detection2D(bbox=np.array([0, 0, 10, 10], dtype=np.float32), score=0.92, label="person"),
            Detection2D(bbox=np.array([0, 0, 10, 10], dtype=np.float32), score=0.61, label="person"),
        ]

        limited = detector._limit_detection_results(results)

        self.assertEqual([round(det.score, 2) for det in limited], [0.92, 0.61])

    def test_sort_and_limit_indices_uses_global_score_order(self):
        detector = BPUDetector(max_detections=2)
        scores = np.array([0.25, 0.91, 0.58], dtype=np.float32)

        keep = detector._sort_and_limit_indices(scores, [0, 1, 2])

        self.assertEqual(keep, [1, 2])


def _mask_detector(proto_name="proto", proto_tensor_name="proto"):
    detector = BPUDetector.__new__(BPUDetector)
    detector._proto_tensor_name_override = proto_tensor_name
    detector._proto_name = proto_name
    detector._proto_missing_logged = False
    return detector


def _raw_detections(count=2):
    raw = [
        (np.array([50 + index * 60, 50, 100 + index * 60, 100], dtype=np.float32), 0.9, 0)
        for index in range(count)
    ]
    coefficients = np.random.default_rng(7).standard_normal((count, 32)).astype(np.float32)
    return raw, coefficients


def _proto_outputs(name="proto"):
    proto = np.random.default_rng(42).standard_normal((160, 160, 32)).astype(np.float32)
    return {name: np.expand_dims(proto, 0)}


class TestGenerateMasks(unittest.TestCase):
    def test_masks_are_boolean_bbox_crops(self):
        detector = _mask_detector()
        raw, coefficients = _raw_detections()
        masks = detector._generate_masks(raw, coefficients, _proto_outputs(), 1.0, 0, 0)

        self.assertEqual(len(masks), 2)
        for mask, detection in zip(masks, raw, strict=True):
            self.assertIsNotNone(mask)
            crop, _x, _y = mask
            box = detection[0].astype(int)
            self.assertEqual(crop.dtype, bool)
            self.assertEqual(crop.shape, (box[3] - box[1], box[2] - box[0]))

    def test_missing_proto_logs_once_and_returns_none_masks(self):
        detector = _mask_detector(proto_name=None)
        raw, coefficients = _raw_detections(3)

        with self.assertLogs("perception.detection.bpu_detector", level="ERROR") as logs:
            first = detector._generate_masks(raw, coefficients, {}, 1.0, 0, 0)
            second = detector._generate_masks(raw, coefficients, {}, 1.0, 0, 0)

        self.assertEqual(len(logs.records), 1)
        self.assertIn("proto", logs.records[0].message.lower())
        self.assertEqual(first, [None, None, None])
        self.assertEqual(second, [None, None, None])

    def test_none_coefficients_return_none_masks(self):
        detector = _mask_detector()
        raw, _coefficients = _raw_detections()
        self.assertEqual(detector._generate_masks(raw, None, _proto_outputs(), 1.0, 0, 0), [None, None])

    def test_proto_name_override_is_used_as_fallback(self):
        detector = _mask_detector(proto_name=None, proto_tensor_name="my_proto")
        raw, coefficients = _raw_detections(1)
        masks = detector._generate_masks(raw, coefficients, _proto_outputs("my_proto"), 1.0, 0, 0)
        self.assertIsNotNone(masks[0])


if __name__ == "__main__":
    unittest.main()
