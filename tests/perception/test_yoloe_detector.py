"""Label contracts for prompted YOLO-E and its fixed-class fallback."""

import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from perception.detection.yoloe_detector import YOLOEDetector


class _CpuArray:
    def __init__(self, values):
        self.values = np.asarray(values)

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def _result(class_ids, names):
    boxes = [
        SimpleNamespace(
            xyxy=[_CpuArray([1, 2, 3, 4])], conf=[0.9], cls=[class_id]
        )
        for class_id in class_ids
    ]
    return SimpleNamespace(boxes=boxes, masks=None, names=names)


def test_coco_fallback_keeps_original_labels_and_filters_prompt(monkeypatch):
    result = _result([2, 0, 56], {0: "person", 2: "car", 56: "chair"})
    model = Mock(names=result.names)
    model.predict.return_value = [result]
    load = Mock(side_effect=[RuntimeError("YOLO-E unavailable"), model])
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=load))
    detector = YOLOEDetector(model_size="s")
    detector.load_model()

    detections = detector.detect(np.zeros((8, 8, 3)), "door . chair . person")

    assert [item.label for item in detections] == ["person", "chair"]
    assert [item.class_id for item in detections] == [0, 56]
    model.set_classes.assert_not_called()
    assert [call.args[0] for call in load.call_args_list] == [
        "yoloe-11s-seg.pt", "yolov8s-seg.pt"
    ]


def test_prompted_model_keeps_prompt_indices_and_class_cache():
    model = Mock()
    model.predict.return_value = [_result([2, 0], {0: "door", 2: "person"})]
    detector = YOLOEDetector()
    detector._model = model

    for _ in range(2):
        detections = detector.detect(np.zeros((8, 8, 3)), "door . chair . person")
        assert [item.label for item in detections] == ["person", "door"]

    model.set_classes.assert_called_once_with(["door", "chair", "person"])


def test_model_without_prompt_support_uses_original_names():
    result = _result([0, 2], ["person", "bicycle", "car"])
    detector = YOLOEDetector()
    detector._model = SimpleNamespace(predict=Mock(return_value=[result]))

    detections = detector.detect(np.zeros((8, 8, 3)), "person")

    assert [(item.label, item.class_id) for item in detections] == [("person", 0)]


@pytest.mark.parametrize("error_type", [RuntimeError, AttributeError])
def test_prompt_setup_failure_propagates_without_committing_cache(error_type):
    detector = YOLOEDetector()
    detector._model = Mock()
    detector._model.set_classes.side_effect = error_type("prompt encoding failed")

    with pytest.raises(error_type, match="prompt encoding failed"):
        detector.detect(np.zeros((8, 8, 3)), "person")

    assert detector._current_classes is None
    detector._model.predict.assert_not_called()


def test_detection_failure_propagates():
    detector = YOLOEDetector()
    detector._model = Mock()
    detector._model.predict.side_effect = RuntimeError("inference failed")

    with pytest.raises(RuntimeError, match="inference failed"):
        detector.detect(np.zeros((8, 8, 3)), "person")


def test_loading_failure_propagates_when_both_models_fail(monkeypatch):
    load = Mock(side_effect=[RuntimeError("YOLO-E unavailable"), RuntimeError("COCO unavailable")])
    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(YOLO=load))
    detector = YOLOEDetector()

    with pytest.raises(RuntimeError, match="COCO unavailable"):
        detector.load_model()

    assert detector._model is None
