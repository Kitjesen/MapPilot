"""Tests for decoupled perception modules — DetectorModule + EncoderModule.

Verifies pluggable backends, Blueprint wiring, and swap-ability.
No GPU required — uses mock backends.
"""

import os
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from runtime import Blueprint, In, Module, Out
from runtime.registry import register
from perception.detection.detector_module import (
    DetectionResult,
    DetectorModule,
)
from perception.encoding.encoder_module import (
    EncoderModule,
    FeatureResult,
)

# ---------------------------------------------------------------------------
# Mock backends (no GPU needed) — imported from shared test utilities
# ---------------------------------------------------------------------------

from runtime.tests._test_utils import (_MockDetectorBackend,
                                    _MockEncoderBackend)


# ---------------------------------------------------------------------------
# DetectorModule tests
# ---------------------------------------------------------------------------

class TestDetectorModule(unittest.TestCase):

    def test_ports_declared(self):
        mod = DetectorModule()
        self.assertIn("image", mod.ports_in)
        self.assertIn("detections", mod.ports_out)

    def test_layer_is_3(self):
        self.assertEqual(DetectorModule._layer, 3)

    @patch.object(DetectorModule, '_create_backend', return_value=_MockDetectorBackend())
    def test_detect_publishes_result(self, _mock):
        mod = DetectorModule(detector="yoloe")
        mod.setup()

        received = []
        mod.detections._add_callback(received.append)

        fake_img = np.zeros((480, 640, 3), dtype=np.uint8)
        mod.image._deliver(fake_img)

        self.assertEqual(len(received), 1)
        self.assertIsInstance(received[0], DetectionResult)
        self.assertEqual(len(received[0].detections), 1)
        self.assertGreater(received[0].inference_ms, 0)

    @patch.object(DetectorModule, '_create_backend', return_value=_MockDetectorBackend())
    def test_health_report(self, _mock):
        mod = DetectorModule(detector="yoloe")
        mod.setup()
        mod.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))
        h = mod.health()
        self.assertEqual(h["detector"]["frames"], 1)
        self.assertTrue(h["detector"]["loaded"])

    def test_unknown_backend_raises(self):
        mod = DetectorModule(detector="nonexistent")
        with self.assertRaises(ValueError):
            mod.setup()

    def test_create_backend_uses_perception_detector_registry(self):
        seen = []

        @register("detector", "standalone_registry_detector")
        class _DetectorProvider:
            @staticmethod
            def create(module):
                seen.append(module)
                return _MockDetectorBackend()

        mod = DetectorModule(detector="standalone_registry_detector")

        backend = mod._create_backend()

        self.assertIsInstance(backend, _MockDetectorBackend)
        self.assertEqual(seen, [mod])

    @patch.object(DetectorModule, '_create_backend', return_value=None)
    def test_no_backend_no_crash(self, _mock):
        """If backend fails to load, image delivery doesn't crash."""
        mod = DetectorModule()
        mod._backend = None
        mod.setup()
        mod.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))
        self.assertEqual(mod.detections.msg_count, 0)


# ---------------------------------------------------------------------------
# EncoderModule tests
# ---------------------------------------------------------------------------

class TestEncoderModule(unittest.TestCase):

    def test_ports_declared(self):
        mod = EncoderModule()
        self.assertIn("image", mod.ports_in)
        self.assertIn("feature", mod.ports_out)

    def test_create_backend_uses_perception_encoder_registry(self):
        seen = []

        @register("encoder", "standalone_registry_encoder")
        class _EncoderProvider:
            @staticmethod
            def create(module):
                seen.append(module)
                return _MockEncoderBackend()

        mod = EncoderModule(encoder="standalone_registry_encoder")

        backend = mod._create_backend()

        self.assertIsInstance(backend, _MockEncoderBackend)
        self.assertEqual(seen, [mod])

    @patch.object(EncoderModule, '_create_backend', return_value=_MockEncoderBackend())
    def test_encode_publishes_feature(self, _mock):
        mod = EncoderModule(encoder="clip")
        mod.setup()

        received = []
        mod.feature._add_callback(received.append)

        fake_img = np.zeros((224, 224, 3), dtype=np.uint8)
        mod.image._deliver(fake_img)

        self.assertEqual(len(received), 1)
        self.assertIsInstance(received[0], FeatureResult)
        self.assertEqual(received[0].dim, 512)

    @patch.object(EncoderModule, '_create_backend', return_value=_MockEncoderBackend())
    def test_encode_text(self, _mock):
        mod = EncoderModule()
        mod.setup()
        feat = mod.encode_text("a red chair")
        self.assertIsNotNone(feat)
        self.assertEqual(feat.shape[0], 512)


# ---------------------------------------------------------------------------
# Blueprint swap test
# ---------------------------------------------------------------------------

class TestPluggableSwap(unittest.TestCase):
    """Prove that swapping detector = changing one Blueprint argument."""

    class Consumer(Module, layer=4):
        detections: In[DetectionResult]

        def setup(self):
            self.received = []
            self.detections.subscribe(self.received.append)

    @patch.object(DetectorModule, '_create_backend', return_value=_MockDetectorBackend())
    def test_swap_detector_same_consumer(self, _mock):
        """Same Consumer works with any detector backend."""
        for backend_name in ["yoloe", "yolo_world"]:
            bp = Blueprint()
            bp.add(DetectorModule, detector=backend_name)
            bp.add(self.Consumer)
            bp.auto_wire()
            handle = bp.build()
            handle.start()

            det = handle.get_module("DetectorModule")
            cons = handle.get_module("Consumer")

            det.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))
            self.assertEqual(len(cons.received), 1,
                             f"Failed with backend={backend_name}")
            handle.stop()

    @patch.object(DetectorModule, '_create_backend', return_value=_MockDetectorBackend())
    @patch.object(EncoderModule, '_create_backend', return_value=_MockEncoderBackend())
    def test_detector_encoder_pipeline(self, _enc, _det):
        """Detector + Encoder wired as independent modules in one Blueprint."""

        class ImageSource(Module, layer=1):
            image: Out[np.ndarray]

        class FusionSink(Module, layer=4):
            detections: In[DetectionResult]
            feature: In[FeatureResult]

        bp = Blueprint()
        bp.add(ImageSource)
        bp.add(DetectorModule)
        bp.add(EncoderModule)
        bp.add(FusionSink)
        bp.auto_wire()
        handle = bp.build()
        handle.start()

        src = handle.get_module("ImageSource")
        sink = handle.get_module("FusionSink")

        fake_img = np.zeros((224, 224, 3), dtype=np.uint8)
        src.image.publish(fake_img)

        self.assertEqual(sink.detections.msg_count, 1)
        self.assertEqual(sink.feature.msg_count, 1)

        handle.stop()




class TestDetectorBackendConfiguration(unittest.TestCase):

    @patch("perception.detection.yoloe_detector.YOLOEDetector")
    def test_yoloe_backend_receives_recall_parameters(self, yoloe_cls):
        mod = DetectorModule(
            detector="yoloe",
            confidence=0.18,
            iou_threshold=0.41,
            max_detections=96,
            model_size="m",
            device="cuda:0",
        )

        backend = mod._create_backend()

        yoloe_cls.assert_called_once_with(
            model_size="m",
            confidence=0.18,
            iou_threshold=0.41,
            device="cuda:0",
            max_detections=96,
        )
        self.assertIs(backend, yoloe_cls.return_value)

    @patch("perception.detection.yolo_world_detector.YOLOWorldDetector")
    def test_yolo_world_backend_receives_iou_parameters(self, yolo_world_cls):
        mod = DetectorModule(
            detector="yolo_world",
            confidence=0.20,
            iou_threshold=0.44,
            model_size="s",
            device="cpu",
        )

        backend = mod._create_backend()

        yolo_world_cls.assert_called_once_with(
            model_size="s",
            confidence=0.20,
            iou_threshold=0.44,
            device="cpu",
        )
        self.assertIs(backend, yolo_world_cls.return_value)

    @patch("perception.detection.bpu_detector.BPUDetector")
    def test_bpu_backend_receives_recall_parameters(self, bpu_cls):
        mod = DetectorModule(
            detector="bpu",
            model_path="D:/models/person.hbm",
            confidence=0.22,
            iou_threshold=0.45,
            max_detections=72,
            min_box_size_px=10,
        )

        backend = mod._create_backend()

        bpu_cls.assert_called_once_with(
            model_path="D:/models/person.hbm",
            confidence=0.22,
            iou_threshold=0.45,
            max_detections=72,
            min_box_size_px=10,
        )
        self.assertIs(backend, bpu_cls.return_value)

    @patch("perception.detection.grounding_dino_detector.GroundingDINODetector")
    def test_grounding_dino_backend_receives_confidence_as_box_threshold(self, dino_cls):
        mod = DetectorModule(
            detector="grounding_dino",
            confidence=0.27,
            device="cpu",
        )

        backend = mod._create_backend()

        dino_cls.assert_called_once_with(
            box_threshold=0.27,
            device="cpu",
        )
        self.assertIs(backend, dino_cls.return_value)


if __name__ == "__main__":
    unittest.main(verbosity=2)
