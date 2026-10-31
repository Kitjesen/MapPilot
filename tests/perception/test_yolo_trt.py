"""TensorRT fixed-person detector preprocessing, tracking, and provider contracts."""

from __future__ import annotations

import ctypes

import numpy as np
import pytest

from perception.backends import DetectorSpec
from perception.detection.yolo_trt import YoloTensorRTDetector
from runtime.registry import get, list_plugins


class _Function:
    def __init__(self, implementation):
        self.implementation = implementation
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.implementation(*args)


class _Library:
    def __init__(self) -> None:
        self.created: list[bytes] = []
        self.destroyed: list[int] = []
        self.inputs: list[np.ndarray] = []
        self.predictions = [
            [200.0, 305.0, 200.0, 350.0, 0.90],
            [202.0, 307.0, 200.0, 350.0, 0.80],
            [500.0, 280.0, 100.0, 200.0, 0.70],
        ]
        self.yolo_create = _Function(self._create)
        self.yolo_infer = _Function(self._infer)
        self.yolo_destroy = _Function(self._destroy)

    def _create(self, path: bytes) -> int:
        self.created.append(path)
        return 73

    def _infer(self, handle: int, pixels, output) -> int:
        assert handle == 73
        tensor = np.ctypeslib.as_array(pixels, shape=(3 * 640 * 640,)).reshape(1, 3, 640, 640)
        self.inputs.append(tensor.copy())
        predictions = np.ctypeslib.as_array(output, shape=(5 * 8400,)).reshape(5, 8400)
        predictions.fill(0.0)
        # A 640x480 source is letterboxed with 80 pixels above and below.
        for index, prediction in enumerate(self.predictions):
            predictions[:, index] = prediction
        return 0

    def _destroy(self, handle: int) -> None:
        self.destroyed.append(handle)


def _loaded_detector(monkeypatch, tmp_path):
    engine = tmp_path / "person.engine"
    library_file = tmp_path / "liblingtu_yolo_trt.so"
    engine.write_bytes(b"engine")
    library_file.write_bytes(b"library")
    library = _Library()
    monkeypatch.setattr(ctypes, "CDLL", lambda _path: library)
    return YoloTensorRTDetector(str(engine)), library


def test_letterbox_nms_coordinate_restore_and_native_ids(monkeypatch, tmp_path) -> None:
    detector, library = _loaded_detector(monkeypatch, tmp_path)
    frame = np.full((480, 640, 3), [10, 20, 30], dtype=np.uint8)

    first = detector.detect(frame, "person")
    second = detector.detect(frame, "person")

    assert len(first) == 2
    assert [item.track_id for item in first] == [1, 2]
    assert [item.track_id for item in second] == [1, 2]
    assert np.allclose(first[0].bbox, [100.0, 50.0, 300.0, 400.0])
    assert np.allclose(first[1].bbox, [450.0, 100.0, 550.0, 300.0])
    assert [item.label for item in first] == ["person", "person"]
    assert library.created == [str(tmp_path / "person.engine").encode()]
    tensor = library.inputs[0]
    assert tensor.shape == (1, 3, 640, 640)
    assert tensor[0, :, 0, 0] == pytest.approx([114 / 255] * 3)
    assert tensor[0, :, 100, 100] == pytest.approx([30 / 255, 20 / 255, 10 / 255])

    detector.shutdown()
    assert library.destroyed == [73]


def test_fixed_person_backend_does_not_answer_other_prompts(monkeypatch, tmp_path) -> None:
    detector, library = _loaded_detector(monkeypatch, tmp_path)

    assert detector.detect(np.zeros((480, 640, 3), np.uint8), "chair . bottle") == []
    assert library.created == []


def test_low_confidence_box_recovers_existing_track_but_cannot_create_one(
    monkeypatch, tmp_path
) -> None:
    detector, library = _loaded_detector(monkeypatch, tmp_path)
    frame = np.zeros((480, 640, 3), np.uint8)
    library.predictions = [[200.0, 305.0, 200.0, 350.0, 0.90]]

    initial = detector.detect(frame, "person")
    library.predictions = [[200.0, 305.0, 200.0, 350.0, 0.20]]
    recovered = detector.detect(frame, "person")

    assert [item.track_id for item in initial] == [1]
    assert [item.track_id for item in recovered] == [1]
    assert recovered[0].score == pytest.approx(0.20)

    detector.shutdown()
    standalone_path = tmp_path / "standalone"
    standalone_path.mkdir()
    standalone, standalone_library = _loaded_detector(monkeypatch, standalone_path)
    standalone_library.predictions = [[200.0, 305.0, 200.0, 350.0, 0.20]]

    assert standalone.detect(frame, "person") == []


def test_invalid_image_and_inference_failure_are_explicit(monkeypatch, tmp_path) -> None:
    detector, library = _loaded_detector(monkeypatch, tmp_path)
    with pytest.raises(ValueError, match="HxWx3 uint8 BGR"):
        detector.detect(np.zeros((480, 640), np.uint8), "person")
    library.yolo_infer = _Function(lambda *_args: 3)
    with pytest.raises(RuntimeError, match="TensorRT inference failed: 3"):
        detector.detect(np.zeros((480, 640, 3), np.uint8), "person")


def test_engine_requires_sibling_native_library(tmp_path) -> None:
    engine = tmp_path / "person.engine"
    engine.write_bytes(b"engine")
    detector = YoloTensorRTDetector(str(engine))

    with pytest.raises(RuntimeError, match="TensorRT library is missing"):
        detector.load_model()


def test_provider_registers_and_preserves_bounded_detection_options(tmp_path) -> None:
    engine = str(tmp_path / "person.engine")
    detector = get("detector", "yolo_trt").create(
        DetectorSpec(
            name="yolo_trt",
            model_path=engine,
            confidence=0.42,
            iou_threshold=0.51,
            max_detections=7,
            min_box_size_px=18,
        )
    )

    assert "yolo_trt" in set(list_plugins("detector"))
    assert detector.model_path == engine
    assert detector.confidence == pytest.approx(0.42)
    assert detector.iou_threshold == pytest.approx(0.51)
    assert detector.max_detections == 7
    assert detector.min_box_size_px == 18


def test_native_source_contract_names_fixed_tensor_shapes() -> None:
    source = (
        __import__("pathlib").Path(__file__).parents[2]
        / "src/perception/detection/yolo_trt.cpp"
    ).read_text(encoding="utf-8")

    assert "1LL * 3 * 640 * 640" in source
    assert "1LL * 5 * 8400" in source
    assert "unexpected YOLO TensorRT input contract" in source
    assert "unexpected YOLO TensorRT output contract" in source
