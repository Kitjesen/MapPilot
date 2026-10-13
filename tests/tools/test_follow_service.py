from __future__ import annotations

import asyncio
import base64

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient
from tools.inference.follow_service import (
    FollowInferenceService,
    _decode_image,
    _valid_track_crops,
    create_app,
)

from perception.detection.detector_base import Detection2D


def _jpeg_b64(image: np.ndarray) -> str:
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return base64.b64encode(encoded).decode("ascii")


class FakeDetector:
    confidence = 0.1

    def __init__(self) -> None:
        self.closed = False
        self.prompts = []

    def detect(self, _image, prompt):
        self.prompts.append(prompt)
        return [
            Detection2D(
                bbox=np.asarray([1.8, 2.9, 16.7, 19.2], dtype=np.float32),
                score=0.9,
                label="person",
            )
        ]

    def shutdown(self) -> None:
        self.closed = True


class FakeTracker:
    def update(self, _detections, _image):
        return np.asarray([[1.8, 2.9, 16.7, 19.2, 7, 0.9, 0, 0]], dtype=np.float32)


class FakeDino:
    def __init__(self) -> None:
        self.crop_shapes = []

    def encode_batch(self, crops):
        self.crop_shapes.append([crop.shape for crop in crops])
        feature = np.ones(384, dtype=np.float32)
        feature /= np.linalg.norm(feature)
        return [feature.copy() for _ in crops]


class FakeClip:
    def __init__(self) -> None:
        self.closed = False

    def encode_image(self, _image):
        return np.asarray([1.0, 0.0, 0.0], dtype=np.float32)

    def encode_text(self, texts):
        return np.asarray([[0.0, 1.0, 0.0] for _ in texts], dtype=np.float32)

    def shutdown(self) -> None:
        self.closed = True


class FakeChat:
    async def chat(self, _messages):
        return "MATCH"


def _service() -> FollowInferenceService:
    return FollowInferenceService(
        detector=FakeDetector(),
        clip=FakeClip(),
        dino=FakeDino(),
        chat=FakeChat(),
        tracker=FakeTracker(),
        warmup_ms={"yolo": 1.0, "clip": 2.0, "dino": 3.0, "qwen": 4.0},
    )


def test_person_crop_uses_host_truncation_clamp_and_minimum_size() -> None:
    image = np.zeros((30, 40, 3), dtype=np.uint8)
    tracks = np.asarray(
        [
            [-2.8, 3.9, 15.7, 20.6, 1, 0.9, 0, 0],
            [10.2, 10.8, 18.9, 25.4, 2, 0.8, 0, 1],
        ],
        dtype=np.float32,
    )

    rows, crops = _valid_track_crops(image, tracks)

    assert [int(row[4]) for row in rows] == [1]
    assert crops[0].shape == (17, 15, 3)


def test_observe_returns_matching_track_and_finite_384d_feature() -> None:
    service = _service()
    image = np.full((24, 24, 3), 80, dtype=np.uint8)

    result = asyncio.run(
        service.observe(
            type(
                "Payload",
                (),
                {
                    "image_b64": _jpeg_b64(image),
                    "prompt": "person",
                    "confidence": 0.2,
                    "source_ts": 123456,
                },
            )()
        )
    )

    assert result["source_ts"] == 123456
    assert len(result["detections"]) == 1
    detection = result["detections"][0]
    assert detection["track_id"] == 7
    assert detection["label"] == "person"
    assert detection["bbox_xyxy"] == pytest.approx([1.8, 2.9, 16.7, 19.2])
    assert len(detection["dino_feature"]) == 384
    assert np.isfinite(detection["dino_feature"]).all()
    assert np.linalg.norm(detection["dino_feature"]) == pytest.approx(1.0)
    assert service.dino.crop_shapes == [[(17, 15, 3)]]


def test_http_contract_health_observe_embed_and_chat() -> None:
    loaded = _service()
    app = create_app(lambda: loaded)
    image_b64 = _jpeg_b64(np.full((24, 24, 3), 120, dtype=np.uint8))

    with TestClient(app) as client:
        health = client.get("/health")
        assert health.status_code == 200
        assert health.json()["ready"] is True
        assert all(health.json()["models"].values())

        observe = client.post(
            "/observe",
            json={
                "image_b64": image_b64,
                "prompt": "person",
                "confidence": 0.1,
                "source_ts": "frame-9",
            },
        )
        assert observe.status_code == 200
        assert observe.json()["source_ts"] == "frame-9"
        assert len(observe.json()["detections"][0]["dino_feature"]) == 384

        clip_image = client.post(
            "/embed", json={"kind": "clip_image", "images": [image_b64]}
        )
        assert clip_image.status_code == 200
        assert clip_image.json()["features"] == [[1.0, 0.0, 0.0]]

        clip_text = client.post(
            "/embed", json={"kind": "clip_text", "texts": ["black shirt"]}
        )
        assert clip_text.status_code == 200
        assert clip_text.json()["features"] == [[0.0, 1.0, 0.0]]

        dino = client.post("/embed", json={"kind": "dino", "images": [image_b64]})
        assert dino.status_code == 200
        assert len(dino.json()["features"][0]) == 384

        chat = client.post(
            "/chat",
            json={
                "messages": [{"role": "user", "content": "same person?"}],
                "temperature": 0.0,
            },
        )
        assert chat.status_code == 200
        assert chat.json()["text"] == "MATCH"

    assert loaded.detector.closed is True
    assert loaded.clip.closed is True


def test_invalid_image_and_missing_embed_input_return_400() -> None:
    app = create_app(_service)
    with TestClient(app) as client:
        invalid_image = client.post(
            "/observe",
            json={"image_b64": "not-base64", "prompt": "person", "confidence": 0.1},
        )
        assert invalid_image.status_code == 400
        assert invalid_image.json()["detail"] == "invalid_base64_image"

        missing_text = client.post("/embed", json={"kind": "clip_text"})
        assert missing_text.status_code == 400
        assert missing_text.json()["detail"] == "clip_text_requires_texts"
