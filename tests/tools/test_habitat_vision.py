"""Per-frame Habitat vision contracts using synthetic geometry and model fixtures."""

import sys
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
from PIL import Image
from tools.datasets.habitat_vision import (
    DEFAULT_OBJECT_LABELS,
    Vision,
    build_observation_prompt,
    clean_depth_mask,
)

from perception.detection.detector_base import Detection2D
from runtime.msgs.sensor import CameraIntrinsics
from runtime.registry import get


class _Tensor:
    def __init__(self, value):
        self.value = value

    def cpu(self):
        return self

    def numpy(self):
        return self.value


def test_observation_prompt_adds_generic_objects_and_deduplicates_target():
    prompt = build_observation_prompt("bottle . plant", ("chair", "bottle", "table"))

    assert prompt == "bottle . plant . chair . table"
    assert build_observation_prompt("chair", ()) == "chair"
    assert {"chair", "table", "cabinet", "shelf", "sofa", "bed", "bottle", "cup", "door"} <= set(
        DEFAULT_OBJECT_LABELS,
    )


def test_clean_depth_mask_removes_far_background_and_keeps_main_object():
    mask = np.ones((20, 20), dtype=bool)
    depth = np.full(mask.shape, 6.0)
    depth[5:15, 5:15] = np.linspace(1.9, 2.1, 10)[:, None]
    depth[0, 0] = np.nan

    cleaned = clean_depth_mask(mask, depth)

    assert cleaned.raw_pixels == 400
    assert cleaned.valid_pixels == 399
    assert cleaned.component_count == 2
    assert cleaned.mask.sum() == 100
    assert cleaned.mask[5:15, 5:15].all()
    assert not cleaned.mask[:5].any()


def test_infer_keeps_generic_object_order_and_matching_clip_features():
    prompts = []
    bottle_mask = np.zeros((20, 20), dtype=bool)
    bottle_mask[2:6, 2:6] = True
    table_mask = np.zeros((20, 20), dtype=bool)
    table_mask[12:16, 12:16] = True
    boxes = [
        Detection2D(np.array([12, 12, 16, 16]), 0.7, "table"),
        Detection2D(np.array([2, 2, 6, 6]), 0.9, "bottle"),
    ]

    class _Segmenter:
        predictor = SimpleNamespace()

        @staticmethod
        def predict(image, **kwargs):
            return [SimpleNamespace()]

    def prompt(results, bboxes):
        mask = bottle_mask if bboxes[0][0] == 2 else table_mask
        return [_Selected(SimpleNamespace(data=[_Tensor(mask)]))]

    class _Selected:
        def __init__(self, masks):
            self.masks = masks

        def __len__(self):
            return 1

    vision = Vision.__new__(Vision)
    vision.prompt = build_observation_prompt("bottle", ("chair", "table", "cabinet"))
    vision.device = "cpu"
    vision.min_depth = 0.3
    vision.max_depth = 15.0
    vision.detector = SimpleNamespace(
        detect=lambda image, prompt_value: prompts.append(prompt_value) or boxes,
    )
    vision.segmenter = _Segmenter()
    vision.segmenter.predictor.prompt = prompt
    vision.encoder = SimpleNamespace(
        encode_image_crops=lambda image, bboxes, use_cache: [
            np.array([1.0, 0.0]),
            np.array([0.0, 2.0]),
        ],
    )
    rgb = np.full((20, 20, 3), 127, dtype=np.uint8)
    depth = np.full((20, 20), 6.0)
    depth[bottle_mask] = 2.0
    depth[table_mask] = 3.0
    intrinsics = CameraIntrinsics(fx=100, fy=100, cx=10, cy=10, width=20, height=20)

    result = vision.infer(rgb, depth, intrinsics, np.eye(4))

    assert prompts == ["bottle . chair . table . cabinet"]
    assert [item.label for item in result.detections] == ["bottle", "table"]
    np.testing.assert_allclose(result.detections[0].features, [1.0, 0.0])
    np.testing.assert_allclose(result.detections[1].features, [0.0, 1.0])
    assert [item.detection.label for item in result.observations] == ["bottle", "table"]


def test_infer_reuses_loaded_models_passes_bgr_and_reports_projection(monkeypatch, tmp_path):
    detector_calls = []
    encoder_calls = []
    encode_calls = []
    thread_calls = []
    detect_count = 0

    def detect(bgr, prompt):
        nonlocal detect_count
        detect_count += 1
        detector_calls.append((bgr[0, 0].tolist(), prompt))
        if detect_count == 3:
            return []
        return [
            Detection2D(np.array([2, 1, 10, 11]), 0.8, "chair"),
            Detection2D(np.array([1, 1, 11, 11]), 0.6, "seat"),
        ]

    detector = SimpleNamespace(
        load_model=lambda: detector_calls.append("loaded"),
        detect=detect,
        shutdown=lambda: detector_calls.append("closed"),
    )
    monkeypatch.setattr(get("detector", "yolo_world"), "create", lambda spec: detector)

    def encode_image_crops(bgr, bboxes, *, use_cache):
        assert bgr[0, 0].tolist() == [10, 40, 230]
        assert len(bboxes) == 1
        assert use_cache is False
        np.testing.assert_array_equal(bboxes[0], [2, 1, 10, 11])
        rgb_crop = bgr[1:11, 2:10, ::-1]
        assert rgb_crop.shape == (10, 8, 3)
        assert rgb_crop[0, 0].tolist() == [230, 40, 10]
        encode_calls.append(rgb_crop)
        return [np.array([3.0, 4.0])]

    encoder = SimpleNamespace(
        load_model=lambda: encoder_calls.append("loaded"),
        encode_image_crops=encode_image_crops,
        shutdown=lambda: encoder_calls.append("closed"),
    )

    def create_encoder(*, model_path, device):
        encoder_calls.append(("created", model_path, device))
        return encoder

    monkeypatch.setattr(get("encoder", "clip"), "create", create_encoder)

    mask = np.zeros((12, 12), dtype=bool)
    mask[2:10, 2:10] = True

    class FastSAM:
        instances = 0
        predict_calls = 0

        def __init__(self, weights):
            FastSAM.instances += 1
            self.predictor = SimpleNamespace(prompt=self.prompt)

        def predict(self, image, **kwargs):
            FastSAM.predict_calls += 1
            assert isinstance(image, Image.Image)
            assert image.getpixel((0, 0)) == (230, 40, 10)
            return [SimpleNamespace()]

        def prompt(self, results, bboxes):
            selected = SimpleNamespace(masks=SimpleNamespace(data=[_Tensor(mask)]))
            selected.__len__ = lambda: 1
            return [_Selected(selected.masks)]

    class _Selected:
        def __init__(self, masks):
            self.masks = masks

        def __len__(self):
            return 1

    monkeypatch.setitem(sys.modules, "ultralytics", SimpleNamespace(FastSAM=FastSAM))
    monkeypatch.setitem(sys.modules, "torch", SimpleNamespace(set_num_threads=thread_calls.append))
    vision = Vision(
        "chair",
        tmp_path / "yolo.pt",
        tmp_path / "fastsam.pt",
        tmp_path / "clip.pt",
        observation_labels=(),
    )
    intrinsics = CameraIntrinsics(fx=100, fy=100, cx=6, cy=6, width=12, height=12)
    depth = np.full((12, 12), 6.0)
    depth[mask] = 2.0
    rgb = np.full((12, 12, 3), [230, 40, 10], dtype=np.uint8)

    first = vision.infer(rgb, depth, intrinsics, np.eye(4))
    second = vision.infer(Image.fromarray(rgb), depth, intrinsics, np.eye(4))
    empty = vision.infer(rgb, depth, intrinsics, np.eye(4))
    failed = vision.infer(rgb, np.zeros_like(depth), intrinsics, np.eye(4))
    vision.close()

    assert FastSAM.instances == 1
    assert FastSAM.predict_calls == 3
    assert thread_calls == [4]
    assert detector_calls == [
        "loaded",
        ([10, 40, 230], "chair"),
        ([10, 40, 230], "chair"),
        ([10, 40, 230], "chair"),
        ([10, 40, 230], "chair"),
        "closed",
    ]
    assert encoder_calls == [("created", str(tmp_path / "clip.pt"), "cpu"), "loaded", "closed"]
    assert len(encode_calls) == 2
    assert len(first.detections) == len(second.detections) == 1
    assert empty.detections == []
    assert empty.observations == []
    assert failed.detections == []
    assert all(item.detection is None for item in failed.observations)
    observation = first.observations[0]
    assert observation.confidence == 0.8
    assert observation.quality.raw_mask_pixels == 64
    assert observation.quality.cleaned_pixels == 64
    assert observation.quality.projected_points == 64
    assert observation.quality.rejected_reason is None
    assert asdict(observation.quality)["cleaned_pixels"] == 64
    assert first.observations[1].confidence == 0.6
    assert first.observations[1].detection is None
    assert first.observations[1].quality.rejected_reason == "duplicate_mask"
    np.testing.assert_allclose(first.detections[0].features, [0.6, 0.8])
    np.testing.assert_allclose(np.linalg.norm(first.detections[0].features), 1.0)
    projected = [item.detection for item in first.observations if item.detection is not None]
    assert len(projected) == len(first.detections)
    assert all(observation is detection for observation, detection in zip(projected, first.detections))
    np.testing.assert_allclose(first.detections[0].position, [-0.01, -0.01, 2.0])
