from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from perception.detection.bpu_detector import BPUDetector
from perception.detection.yolo_world_detector import YOLOWorldDetector
from perception.detection.yoloe_detector import YOLOEDetector

_IMAGE = np.zeros((16, 16, 3), dtype=np.uint8)


class _FailingModel:
    def set_classes(self, _classes: list[str]) -> None:
        pass

    def predict(self, *_args: Any, **_kwargs: Any) -> list[Any]:
        raise RuntimeError("inference transport failed")


class _EmptyModel:
    def set_classes(self, _classes: list[str]) -> None:
        pass

    def predict(self, *_args: Any, **_kwargs: Any) -> list[Any]:
        return []


@pytest.mark.parametrize("detector_type", [YOLOEDetector, YOLOWorldDetector])
def test_yolo_inference_failure_is_not_reported_as_zero_detections(
    detector_type: type[YOLOEDetector] | type[YOLOWorldDetector],
) -> None:
    detector = detector_type()
    detector._model = _FailingModel()

    with pytest.raises(RuntimeError, match="inference transport failed"):
        detector.detect(_IMAGE, "chair")


@pytest.mark.parametrize("detector_type", [YOLOEDetector, YOLOWorldDetector])
def test_yolo_successful_zero_detection_remains_empty(
    detector_type: type[YOLOEDetector] | type[YOLOWorldDetector],
) -> None:
    detector = detector_type()
    detector._model = _EmptyModel()

    assert detector.detect(_IMAGE, "chair") == []


def test_bpu_model_not_loaded_is_not_reported_as_zero_detections() -> None:
    detector = BPUDetector()

    with pytest.raises(RuntimeError, match="Model not loaded"):
        detector.detect(_IMAGE, "chair")


def test_bpu_successful_filtering_to_empty_remains_empty(monkeypatch: pytest.MonkeyPatch) -> None:
    class _Runtime:
        def run(self, _inputs: dict[str, np.ndarray]) -> dict[str, dict[str, np.ndarray]]:
            return {"model": {}}

    detector = BPUDetector()
    detector._rt = _Runtime()
    detector._mname = "model"
    detector._custom_vocab = None
    detector._is_yoloe = False
    monkeypatch.setattr(
        detector,
        "_preprocess",
        lambda _image: (
            np.empty((0,), dtype=np.uint8),
            np.empty((0,), dtype=np.uint8),
            1.0,
            0,
            0,
        ),
    )
    monkeypatch.setattr(detector, "_postprocess", lambda *_args: ([], []))

    assert detector.detect(_IMAGE, "chair") == []
