import sys
import types
import unittest
from dataclasses import dataclass
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

try:
    from perception.tracking import bpu_qp_bridge as bridge

    _HAS_BPU_QP_BRIDGE = True
except (ImportError, ModuleNotFoundError):
    bridge = None
    _HAS_BPU_QP_BRIDGE = False

pytestmark = pytest.mark.skipif(
    not _HAS_BPU_QP_BRIDGE,
    reason="bpu_qp_bridge removed (dead code, Phase 1 cleanup)",
)


@dataclass
class _FakeBoundingBox:
    x: float
    y: float
    w: float
    h: float


@dataclass
class _FakeDetection:
    bbox: _FakeBoundingBox
    confidence: float
    class_id: str
    timestamp: float


@dataclass
class _FakeTrack:
    track_id: int
    bbox: _FakeBoundingBox
    confidence: float
    class_id: str
    first_seen_ts: float = 0.0
    last_seen_ts: float = 0.0


class _FakeFusionMOTConfig:
    pass


class _FakeFusionMOT:
    def __init__(self, config, feature_dim):
        self.config = config
        self.feature_dim = feature_dim

    def update(self, bboxes, confs, features, timestamp):
        return []

    def update_selective(self, bboxes, confs, timestamp, reid_extractor, frame):
        return []


class _FakeReIDConfig:
    def __init__(self, backbone, device=""):
        self.backbone = backbone
        self.device = device


class _FakeReIDExtractor:
    def __init__(self, config):
        self.config = config
        self.feature_dim = 512


class _FakeSelectorConfig:
    def __init__(self, preferred_classes=None):
        self.preferred_classes = preferred_classes or {}


class _FakeWeightedTargetSelector:
    def __init__(self, frame_width, frame_height, config):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.config = config

    def select(self, tracks, timestamp):
        return tracks[0] if tracks else None


def _fake_qp_modules():
    qp_pkg = types.ModuleType("qp_perception")
    qp_pkg.__path__ = []

    tracking_pkg = types.ModuleType("qp_perception.tracking")
    tracking_pkg.__path__ = []
    fusion_mod = types.ModuleType("qp_perception.tracking.fusion")
    fusion_mod.FusionMOT = _FakeFusionMOT
    fusion_mod.FusionMOTConfig = _FakeFusionMOTConfig

    reid_pkg = types.ModuleType("qp_perception.reid")
    reid_pkg.__path__ = []
    extractor_mod = types.ModuleType("qp_perception.reid.extractor")
    extractor_mod.ReIDConfig = _FakeReIDConfig
    extractor_mod.ReIDExtractor = _FakeReIDExtractor

    selection_pkg = types.ModuleType("qp_perception.selection")
    selection_pkg.__path__ = []
    weighted_mod = types.ModuleType("qp_perception.selection.weighted")
    weighted_mod.WeightedTargetSelector = _FakeWeightedTargetSelector

    config_mod = types.ModuleType("qp_perception.config")
    config_mod.SelectorConfig = _FakeSelectorConfig

    types_mod = types.ModuleType("qp_perception.types")
    types_mod.BoundingBox = _FakeBoundingBox
    types_mod.Detection = _FakeDetection
    types_mod.Track = _FakeTrack

    return {
        "qp_perception": qp_pkg,
        "qp_perception.tracking": tracking_pkg,
        "qp_perception.tracking.fusion": fusion_mod,
        "qp_perception.reid": reid_pkg,
        "qp_perception.reid.extractor": extractor_mod,
        "qp_perception.selection": selection_pkg,
        "qp_perception.selection.weighted": weighted_mod,
        "qp_perception.config": config_mod,
        "qp_perception.types": types_mod,
    }


class TestCreatePerceptionPipeline(unittest.TestCase):
    def test_defaults_build_person_high_recall_pipeline(self):
        fake_bpu = MagicMock()

        with patch.dict(sys.modules, _fake_qp_modules()):
            with patch(
                "perception.detection.bpu_detector.BPUDetector",
                return_value=fake_bpu,
            ) as bpu_cls:
                pipeline = bridge.create_perception_pipeline()

        bpu_cls.assert_called_once_with(
            confidence=0.22,
            iou_threshold=0.45,
            model_path=None,
            max_detections=64,
            min_box_size_px=12,
        )
        fake_bpu.load_model.assert_called_once_with()
        self.assertIsInstance(pipeline, bridge.PerceptionPipeline)
        self.assertEqual(pipeline._detector._whitelist, {"person"})
        self.assertEqual(pipeline._detector._prompt, "person")
        self.assertEqual(pipeline._selector.frame_width, 640)
        self.assertEqual(pipeline._selector.frame_height, 480)

    def test_explicit_pipeline_params_are_forwarded(self):
        fake_bpu = MagicMock()

        with patch.dict(sys.modules, _fake_qp_modules()):
            with patch(
                "perception.detection.bpu_detector.BPUDetector",
                return_value=fake_bpu,
            ) as bpu_cls:
                pipeline = bridge.create_perception_pipeline(
                    bpu_confidence=0.18,
                    bpu_iou_threshold=0.40,
                    bpu_max_detections=96,
                    bpu_min_box_size_px=10,
                    bpu_model_path="D:/models/person.hbm",
                    class_whitelist=["person", "worker"],
                    frame_width=1280,
                    frame_height=720,
                    reid_backbone="mobilenet",
                )

        bpu_cls.assert_called_once_with(
            confidence=0.18,
            iou_threshold=0.40,
            model_path="D:/models/person.hbm",
            max_detections=96,
            min_box_size_px=10,
        )
        self.assertIsInstance(pipeline, bridge.PerceptionPipeline)
        self.assertEqual(pipeline._detector._whitelist, {"person", "worker"})
        self.assertEqual(pipeline._detector._prompt, "person . worker")
        self.assertEqual(pipeline._selector.frame_width, 1280)
        self.assertEqual(pipeline._selector.frame_height, 720)
        self.assertEqual(pipeline._tracker._reid.config.backbone, "mobilenet")


class TestBPUDetectorAdapter(unittest.TestCase):
    def test_detect_filters_invalid_boxes_and_non_whitelisted_classes(self):
        raw_person = type("RawDet", (), {"label": "Person", "bbox": np.array([10, 20, 30, 60]), "score": 0.91})()
        raw_dog = type("RawDet", (), {"label": "Dog", "bbox": np.array([5, 5, 15, 20]), "score": 0.8})()
        raw_bad = type("RawDet", (), {"label": "Person", "bbox": np.array([8, 8, 8, 14]), "score": 0.7})()
        fake_bpu = MagicMock()
        fake_bpu.detect.return_value = [raw_person, raw_dog, raw_bad]

        with patch.dict(sys.modules, _fake_qp_modules()):
            adapter = bridge.BPUDetectorAdapter(fake_bpu, class_whitelist=["person"])
            detections = adapter.detect(np.zeros((80, 80, 3), dtype=np.uint8), 123.0)

        fake_bpu.detect.assert_called_once()
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].class_id, "person")
        self.assertEqual(detections[0].confidence, 0.91)
        self.assertEqual(detections[0].bbox.x, 10.0)
        self.assertEqual(detections[0].bbox.y, 20.0)
        self.assertEqual(detections[0].bbox.w, 20.0)
        self.assertEqual(detections[0].bbox.h, 40.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
