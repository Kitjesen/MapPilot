"""Single-process GPU inference service for online person following.

This service owns vision inference only.  It never projects depth, authorizes
motion, or sends robot commands.  Run it with one Uvicorn worker because the
native tracker and model instances are intentionally process-local state.
"""

from __future__ import annotations

import asyncio
import base64
import os
import time
from collections.abc import Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException, Request
from pydantic import BaseModel, Field
from tools.datasets.person_follow import TrackerInput
from tools.datasets.qwen_chat import QwenChat

from perception.backends import DetectorSpec
from perception.encoding.clip_encoder import CLIPEncoder
from perception.encoding.dino import DinoEncoder
from perception.tracking.native_byte_tracker import NativeByteTracker
from runtime.registry import get


class ObserveRequest(BaseModel):
    image_b64: str
    prompt: str = "person"
    confidence: float = Field(default=0.1, ge=0.0, le=1.0)
    source_ts: int | float | str | None = None


class EmbedRequest(BaseModel):
    kind: Literal["clip_image", "clip_text", "dino"]
    images: list[str] = Field(default_factory=list)
    texts: list[str] = Field(default_factory=list)


class ChatRequest(BaseModel):
    messages: list[dict[str, Any]]
    temperature: float = 0.0


@dataclass(frozen=True)
class FollowServiceConfig:
    yolo_model: Path
    clip_model: Path
    dino_model: Path
    qwen_model: Path
    device: str = "cuda"
    detector_confidence: float = 0.1

    @classmethod
    def from_environment(cls) -> FollowServiceConfig:
        required = {
            "yolo_model": "LINGTU_FOLLOW_YOLO_MODEL",
            "clip_model": "LINGTU_FOLLOW_CLIP_MODEL",
            "dino_model": "LINGTU_FOLLOW_DINO_MODEL",
            "qwen_model": "LINGTU_FOLLOW_QWEN_MODEL",
        }
        missing = [name for name in required.values() if not os.environ.get(name)]
        if missing:
            raise RuntimeError(f"missing_follow_service_environment:{','.join(missing)}")
        return cls(
            **{key: Path(os.environ[name]) for key, name in required.items()},
            device=os.environ.get("LINGTU_FOLLOW_DEVICE", "cuda"),
        )


@dataclass
class ServiceStatistics:
    observe_ms: list[float] = field(default_factory=list)
    embed_ms: list[float] = field(default_factory=list)
    chat_ms: list[float] = field(default_factory=list)

    @staticmethod
    def _summary(values: list[float]) -> dict[str, float | int | None]:
        if not values:
            return {"count": 0, "mean_ms": None, "p95_ms": None, "max_ms": None}
        array = np.asarray(values, dtype=np.float64)
        return {
            "count": len(values),
            "mean_ms": float(np.mean(array)),
            "p95_ms": float(np.percentile(array, 95)),
            "max_ms": float(np.max(array)),
        }

    def snapshot(self) -> dict[str, dict[str, float | int | None]]:
        return {
            "observe": self._summary(self.observe_ms),
            "embed": self._summary(self.embed_ms),
            "chat": self._summary(self.chat_ms),
        }


def _decode_image(image_b64: str) -> np.ndarray:
    encoded = image_b64.split(",", 1)[1] if image_b64.startswith("data:image/") else image_b64
    try:
        payload = base64.b64decode(encoded, validate=True)
    except ValueError as exc:
        raise ValueError("invalid_base64_image") from exc
    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("invalid_jpeg_image")
    return image


def _feature_list(feature: np.ndarray, *, expected_size: int | None = None) -> list[float]:
    array = np.asarray(feature, dtype=np.float32).reshape(-1)
    if expected_size is not None and array.size != expected_size:
        raise RuntimeError(f"invalid_feature_size:{array.size}")
    if array.size == 0 or not np.isfinite(array).all() or float(np.linalg.norm(array)) <= 0.0:
        raise RuntimeError("invalid_model_feature")
    return [float(value) for value in array]


def _yolo_device(device: str) -> str:
    return "0" if device in {"cuda", "cuda:0"} else device


def _valid_track_crops(
    image: np.ndarray,
    tracks: np.ndarray,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Apply the Host person-crop contract and discard unusably small boxes."""
    height, width = image.shape[:2]
    kept_rows = []
    crops = []
    for row in tracks:
        x1, y1, x2, y2 = (int(value) for value in row[:4])
        x1, x2 = np.clip([x1, x2], 0, width)
        y1, y2 = np.clip([y1, y2], 0, height)
        if x2 - x1 < 10 or y2 - y1 < 10:
            continue
        kept_rows.append(row)
        crops.append(image[y1:y2, x1:x2].copy())
    return kept_rows, crops


class FollowInferenceService:
    """Loaded models, native tracker, and serialized inference operations."""

    def __init__(
        self,
        *,
        detector: Any,
        clip: Any,
        dino: Any,
        chat: Any,
        tracker: NativeByteTracker | Any | None = None,
        warmup_ms: dict[str, float] | None = None,
    ) -> None:
        self.detector = detector
        self.clip = clip
        self.dino = dino
        self.chat_model = chat
        self.tracker = tracker or NativeByteTracker(frame_rate=30)
        self.lock = asyncio.Lock()
        self.statistics = ServiceStatistics()
        self.warmup_ms = warmup_ms or {}
        self.ready = True

    @classmethod
    def load(cls, config: FollowServiceConfig) -> FollowInferenceService:
        warmup_ms: dict[str, float] = {}
        detector = get("detector", "yolo_world").create(
            DetectorSpec(
                name="yolo_world",
                model_path=str(config.yolo_model),
                confidence=config.detector_confidence,
                device=_yolo_device(config.device),
            )
        )
        detector.load_model()
        started = time.perf_counter()
        detector.detect(np.zeros((480, 640, 3), dtype=np.uint8), "person")
        warmup_ms["yolo"] = (time.perf_counter() - started) * 1000.0

        clip = CLIPEncoder(
            model_name="ViT-B/32",
            model_path=str(config.clip_model),
            device=config.device,
        )
        clip.load_model()
        started = time.perf_counter()
        clip.encode_image(np.zeros((128, 64, 3), dtype=np.uint8))
        clip.encode_text(["person"])
        warmup_ms["clip"] = (time.perf_counter() - started) * 1000.0

        dino = DinoEncoder(model_path=str(config.dino_model), device=config.device)
        dino.load_model()
        started = time.perf_counter()
        dino.encode_batch([np.zeros((256, 128, 3), dtype=np.uint8)] * 3)
        warmup_ms["dino"] = (time.perf_counter() - started) * 1000.0

        chat = QwenChat(config.qwen_model, device=config.device)
        started = time.perf_counter()
        synthetic = np.zeros((256, 128, 3), dtype=np.uint8)
        ok, encoded = cv2.imencode(".jpg", synthetic)
        if not ok:
            raise RuntimeError("cannot_encode_qwen_warmup_image")
        url = "data:image/jpeg;base64," + base64.b64encode(encoded).decode("ascii")
        chat._generate(
            [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "Reply only UNKNOWN."},
                        *[
                            {"type": "image_url", "image_url": {"url": url}}
                            for _ in range(4)
                        ],
                    ],
                }
            ]
        )
        warmup_ms["qwen"] = (time.perf_counter() - started) * 1000.0
        chat.calls.clear()
        return cls(
            detector=detector,
            clip=clip,
            dino=dino,
            chat=chat,
            warmup_ms=warmup_ms,
        )

    def close(self) -> None:
        self.ready = False
        self.detector.shutdown()
        self.clip.shutdown()

    async def observe(self, request: ObserveRequest) -> dict[str, Any]:
        started = time.perf_counter()
        image = _decode_image(request.image_b64)
        async with self.lock:
            self.detector.confidence = request.confidence
            detections = self.detector.detect(image, request.prompt or "person")
            tracks = self.tracker.update(TrackerInput(detections), image)
            kept_rows, crops = _valid_track_crops(image, tracks)
            features = self.dino.encode_batch(crops)
        serialized = []
        for row, feature in zip(kept_rows, features, strict=True):
            detection_index = int(row[7])
            label = detections[detection_index].label
            serialized.append(
                {
                    "bbox_xyxy": [float(value) for value in row[:4]],
                    "score": float(row[5]),
                    "label": label,
                    "track_id": int(row[4]),
                    "dino_feature": _feature_list(feature, expected_size=384),
                }
            )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.statistics.observe_ms.append(elapsed_ms)
        return {
            "detections": serialized,
            "source_ts": request.source_ts,
            "elapsed_ms": elapsed_ms,
        }

    async def embed(self, request: EmbedRequest) -> dict[str, Any]:
        started = time.perf_counter()
        async with self.lock:
            if request.kind == "clip_text":
                if not request.texts:
                    raise ValueError("clip_text_requires_texts")
                features = self.clip.encode_text(request.texts)
            else:
                if not request.images:
                    raise ValueError(f"{request.kind}_requires_images")
                images = [_decode_image(value) for value in request.images]
                if request.kind == "clip_image":
                    features = [self.clip.encode_image(image) for image in images]
                else:
                    features = self.dino.encode_batch(images)
        serialized = [_feature_list(feature) for feature in features]
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.statistics.embed_ms.append(elapsed_ms)
        return {"features": serialized, "elapsed_ms": elapsed_ms}

    async def chat(self, request: ChatRequest) -> dict[str, Any]:
        started = time.perf_counter()
        async with self.lock:
            text = await self.chat_model.chat(request.messages)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.statistics.chat_ms.append(elapsed_ms)
        return {"text": str(text), "elapsed_ms": elapsed_ms}

    def health(self) -> dict[str, Any]:
        return {
            "ready": self.ready,
            "models": {
                "yolo": self.detector is not None,
                "clip": self.clip is not None,
                "dino": self.dino is not None,
                "qwen": self.chat_model is not None,
            },
            "warmup_ms": self.warmup_ms,
            "statistics": self.statistics.snapshot(),
        }


ServiceLoader = Callable[[], FollowInferenceService]


def create_app(loader: ServiceLoader | None = None) -> FastAPI:
    service_loader = loader or (lambda: FollowInferenceService.load(FollowServiceConfig.from_environment()))

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.follow_service = service_loader()
        try:
            yield
        finally:
            app.state.follow_service.close()

    application = FastAPI(title="LingTu Follow Vision", lifespan=lifespan)

    def service(request: Request) -> FollowInferenceService:
        return request.app.state.follow_service

    @application.get("/health")
    async def health(request: Request) -> dict[str, Any]:
        return service(request).health()

    @application.post("/observe")
    async def observe(payload: ObserveRequest, request: Request) -> dict[str, Any]:
        try:
            return await service(request).observe(payload)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @application.post("/embed")
    async def embed(payload: EmbedRequest, request: Request) -> dict[str, Any]:
        try:
            return await service(request).embed(payload)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @application.post("/chat")
    async def chat(payload: ChatRequest, request: Request) -> dict[str, Any]:
        return await service(request).chat(payload)

    return application


app = create_app()
