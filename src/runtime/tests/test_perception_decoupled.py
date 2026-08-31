"""Tests for the standalone detector development module."""

from __future__ import annotations

from unittest.mock import patch

import numpy as np
import pytest

from perception.detection.detector_module import DetectionResult, DetectorModule
from runtime import Blueprint, In, Module
from runtime.registry import register
from runtime.tests._test_utils import _MockDetectorBackend


class TestDetectorModule:
    def test_ports_declared(self) -> None:
        mod = DetectorModule()
        assert "image" in mod.ports_in
        assert "detections" in mod.ports_out

    def test_layer_is_3(self) -> None:
        assert DetectorModule._layer == 3

    @patch.object(DetectorModule, "_create_backend", return_value=_MockDetectorBackend())
    def test_detect_publishes_result(self, _mock: object) -> None:
        mod = DetectorModule(detector="yoloe")
        mod.setup()
        received: list[DetectionResult] = []
        mod.detections._add_callback(received.append)

        mod.image._deliver(np.zeros((480, 640, 3), dtype=np.uint8))

        assert len(received) == 1
        assert isinstance(received[0], DetectionResult)
        assert len(received[0].detections) == 1
        assert received[0].inference_ms >= 0

    @patch.object(DetectorModule, "_create_backend", return_value=_MockDetectorBackend())
    def test_health_report(self, _mock: object) -> None:
        mod = DetectorModule(detector="yoloe")
        mod.setup()
        mod.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))

        health = mod.health()
        assert health["detector"]["frames"] == 1
        assert health["detector"]["loaded"] is True

    def test_unknown_backend_raises(self) -> None:
        with pytest.raises(ValueError):
            DetectorModule(detector="nonexistent").setup()

    def test_create_backend_uses_perception_detector_registry(self) -> None:
        seen: list[DetectorModule] = []

        @register("detector", "standalone_registry_detector")
        class _DetectorProvider:
            @staticmethod
            def create(module: DetectorModule) -> _MockDetectorBackend:
                seen.append(module)
                return _MockDetectorBackend()

        mod = DetectorModule(detector="standalone_registry_detector")

        assert isinstance(mod._create_backend(), _MockDetectorBackend)
        assert seen == [mod]

    @patch.object(DetectorModule, "_create_backend", return_value=None)
    def test_no_backend_no_crash(self, _mock: object) -> None:
        mod = DetectorModule()
        mod.setup()
        mod.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))
        assert mod.detections.msg_count == 0


class TestPluggableDetector:
    class Consumer(Module, layer=4):
        detections: In[DetectionResult]

        def setup(self) -> None:
            self.received: list[DetectionResult] = []
            self.detections.subscribe(self.received.append)

    @patch.object(DetectorModule, "_create_backend", return_value=_MockDetectorBackend())
    def test_swap_detector_same_consumer(self, _mock: object) -> None:
        for backend_name in ("yoloe", "yolo_world"):
            bp = Blueprint()
            bp.add(DetectorModule, detector=backend_name)
            bp.add(self.Consumer)
            bp.auto_wire()
            handle = bp.build()
            handle.start()
            try:
                detector = handle.get_module("DetectorModule")
                consumer = handle.get_module("Consumer")
                detector.image._deliver(np.zeros((100, 100, 3), dtype=np.uint8))
                assert len(consumer.received) == 1
            finally:
                handle.stop()
