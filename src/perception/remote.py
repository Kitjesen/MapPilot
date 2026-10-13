"""HTTP clients for the shared remote vision service."""

from __future__ import annotations

import asyncio
import base64
import json
from typing import Any
from urllib import error, request

import cv2
import numpy as np

from perception.detection.detector_base import Detection2D, DetectorBase


class _RemoteClient:
    def __init__(self, server_url: str, *, timeout: float = 5.0) -> None:
        self.server_url = str(server_url).rstrip("/")
        if not self.server_url:
            raise ValueError("vision service URL is required")
        self.timeout = float(timeout)
        if self.timeout <= 0:
            raise ValueError("vision service timeout must be positive")
        self._health: dict[str, Any] = {}

    def _request_json(self, path: str, payload: dict[str, Any] | None = None) -> Any:
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        req = request.Request(
            f"{self.server_url}{path}",
            data=data,
            method="GET" if data is None else "POST",
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(req, timeout=self.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except (error.HTTPError, error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"remote_vision_request_failed:{path}:{exc}") from exc

    def load_model(self) -> None:
        health = self._request_json("/health")
        if not isinstance(health, dict) or health.get("ready") is not True:
            raise RuntimeError("remote_vision_not_ready")
        self._health = dict(health)

    def health(self) -> dict[str, Any]:
        return {"backend": "remote", "ready": bool(self._health.get("ready"))}

    def shutdown(self) -> None:
        """The shared server owns model lifetime."""

    close = shutdown


def _jpeg_b64(image_bgr: np.ndarray) -> str:
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("remote vision input must be an HxWx3 BGR image")
    ok, encoded = cv2.imencode(".jpg", image_bgr)
    if not ok:
        raise RuntimeError("remote_vision_jpeg_encode_failed")
    return base64.b64encode(encoded).decode("ascii")


def _features(response: Any) -> list[np.ndarray]:
    values = response.get("features") if isinstance(response, dict) else None
    if not isinstance(values, list):
        raise RuntimeError("remote_vision_invalid_features")
    result = []
    for value in values:
        feature = np.asarray(value, dtype=np.float32)
        if feature.ndim != 1 or not np.isfinite(feature).all() or feature.size == 0:
            raise RuntimeError("remote_vision_invalid_feature")
        result.append(feature)
    return result


class RemoteDetector(_RemoteClient, DetectorBase):
    """Send RGB only to the server and keep depth projection on the Host."""

    def __init__(self, server_url: str, *, confidence: float = 0.3, timeout: float = 5.0) -> None:
        super().__init__(server_url, timeout=timeout)
        self.confidence = float(confidence)
        self._source_ts: float | None = None

    def set_source_ts(self, source_ts: float) -> None:
        self._source_ts = float(source_ts)

    def detect(self, rgb: np.ndarray, text_prompt: str) -> list[Detection2D]:
        payload: dict[str, Any] = {
            "image_b64": _jpeg_b64(rgb),
            "prompt": str(text_prompt),
            "confidence": self.confidence,
        }
        if self._source_ts is not None:
            payload["source_ts"] = self._source_ts
        response = self._request_json(
            "/observe",
            payload,
        )
        rows = response.get("detections") if isinstance(response, dict) else response
        if not isinstance(rows, list):
            raise RuntimeError("remote_vision_invalid_detections")
        detections: list[Detection2D] = []
        for row in rows:
            if not isinstance(row, dict):
                raise RuntimeError("remote_vision_invalid_detection")
            bbox = np.asarray(row.get("bbox_xyxy"), dtype=np.float32)
            score = float(row.get("score", 0.0))
            if bbox.shape != (4,) or not np.isfinite(bbox).all() or not np.isfinite(score):
                raise RuntimeError("remote_vision_invalid_detection")
            detection = Detection2D(
                bbox=bbox,
                score=score,
                label=str(row.get("label") or "person"),
            )
            track_id = row.get("track_id")
            detection.track_id = None if track_id is None else int(track_id)
            dino = row.get("dino_feature")
            detection.dino_feature = None if dino is None else np.asarray(dino, dtype=np.float32)
            detections.append(detection)
        return detections


class RemoteCLIPEncoder(_RemoteClient):
    def encode_image(self, image: np.ndarray) -> np.ndarray:
        result = _features(
            self._request_json(
                "/embed",
                {"kind": "clip_image", "images": [_jpeg_b64(image)], "texts": []},
            )
        )
        if len(result) != 1:
            raise RuntimeError("remote_vision_feature_count_mismatch")
        return result[0]

    def encode_text(self, texts: list[str]) -> np.ndarray:
        if not texts:
            return np.array([])
        result = _features(
            self._request_json(
                "/embed",
                {"kind": "clip_text", "images": [], "texts": list(texts)},
            )
        )
        if len(result) != len(texts):
            raise RuntimeError("remote_vision_feature_count_mismatch")
        return np.stack(result)


class RemoteDinoEncoder(_RemoteClient):
    def encode(self, crop_bgr: np.ndarray) -> np.ndarray:
        return self.encode_batch([crop_bgr])[0]

    def encode_batch(self, crops_bgr: list[np.ndarray]) -> list[np.ndarray]:
        if not crops_bgr:
            return []
        result = _features(
            self._request_json(
                "/embed",
                {
                    "kind": "dino",
                    "images": [_jpeg_b64(crop) for crop in crops_bgr],
                    "texts": [],
                },
            )
        )
        if len(result) != len(crops_bgr):
            raise RuntimeError("remote_vision_feature_count_mismatch")
        return result


class RemoteVisionClient(_RemoteClient):
    supports_vision = True

    def is_available(self) -> bool:
        return bool(self._health.get("ready"))

    async def chat(self, messages: list[dict[str, Any]], **kwargs: Any) -> str:
        response = await asyncio.to_thread(
            self._request_json,
            "/chat",
            {"messages": messages, "temperature": float(kwargs.get("temperature", 0.3))},
        )
        if not isinstance(response, dict) or not isinstance(response.get("text"), str):
            raise RuntimeError("remote_vision_invalid_chat_response")
        return str(response["text"])
