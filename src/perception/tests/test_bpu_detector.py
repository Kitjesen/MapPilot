import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from perception.detection.bpu_detector import BPUDetector
from perception.detection.detector_base import Detection2D


class TestBPUDetectorRecallTuning(unittest.TestCase):
    def test_small_box_threshold_is_configurable(self):
        detector = BPUDetector(min_box_size_px=12)

        self.assertTrue(detector._is_box_large_enough(12, 24))
        self.assertFalse(detector._is_box_large_enough(11, 24))
        self.assertFalse(detector._is_box_large_enough(24, 11))

    def test_limit_detection_results_prefers_highest_scores(self):
        detector = BPUDetector(max_detections=2)
        results = [
            Detection2D(bbox=np.array([0, 0, 10, 10], dtype=np.float32), score=0.30, label='person'),
            Detection2D(bbox=np.array([0, 0, 10, 10], dtype=np.float32), score=0.92, label='person'),
            Detection2D(bbox=np.array([0, 0, 10, 10], dtype=np.float32), score=0.61, label='person'),
        ]

        limited = detector._limit_detection_results(results)

        self.assertEqual([round(det.score, 2) for det in limited], [0.92, 0.61])

    def test_sort_and_limit_indices_uses_global_score_order(self):
        detector = BPUDetector(max_detections=2)
        scores = np.array([0.25, 0.91, 0.58], dtype=np.float32)

        keep = detector._sort_and_limit_indices(scores, [0, 1, 2])

        self.assertEqual(keep, [1, 2])


if __name__ == '__main__':
    unittest.main()
