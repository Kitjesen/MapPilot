from __future__ import annotations

import asyncio
import base64
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from decision.vision.person import PersonTracker, verify_candidate
from lingtu.assembly.stacks.perception import _perception_runtime_config
from perception.backends import DetectorSpec, RgbdObservationSource
from perception.pipeline import to_runtime_detections
from perception.remote import (
    RemoteCLIPEncoder,
    RemoteDetector,
    RemoteDinoEncoder,
    RemoteVisionClient,
)
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D as RuntimeDetection3D
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


@pytest.fixture
def vision_service():
    requests: list[tuple[str, dict | None]] = []
    clip_image_index: int | None = None

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *args) -> None:
            pass

        def _write(self, payload: dict) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            requests.append((self.path, None))
            self._write({"ready": True, "models": {"dino": True}})

        def do_POST(self) -> None:
            nonlocal clip_image_index
            length = int(self.headers["Content-Length"])
            payload = json.loads(self.rfile.read(length))
            requests.append((self.path, payload))
            if self.path == "/observe":
                self._write(
                    {
                        "detections": [
                            {
                                "bbox_xyxy": [4, 4, 16, 16],
                                "score": 0.91,
                                "label": "person",
                                "track_id": 7,
                                "dino_feature": [1.0, 0.0, 0.5],
                            }
                        ],
                        "source_ts": payload.get("source_ts"),
                    }
                )
            elif self.path == "/embed":
                count = len(payload["texts"]) or len(payload["images"])
                if payload["kind"] == "clip_text" and payload["texts"] == ["person in blue"]:
                    clip_image_index = 0
                    self._write({"features": [[1.0, 0.0]]})
                elif (
                    payload["kind"] == "clip_image"
                    and clip_image_index is not None
                    and clip_image_index < 2
                ):
                    features = ([0.0, 1.0], [1.0, 0.0])[clip_image_index]
                    clip_image_index += 1
                    self._write({"features": [features]})
                else:
                    self._write({"features": [[1.0, 0.5] for _ in range(count)]})
            elif self.path == "/chat":
                self._write({"text": "MATCH"})
            else:
                self.send_error(404)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", requests
    finally:
        server.shutdown()
        thread.join(timeout=2.0)
        server.server_close()


def _rgbd_frame() -> SimpleNamespace:
    return SimpleNamespace(
        color=Image(
            data=np.zeros((20, 20, 3), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=12.5,
            frame_id="camera",
        ),
        depth=Image(
            data=np.full((20, 20), 1500, dtype=np.uint16),
            format=ImageFormat.DEPTH_U16,
            ts=12.5,
            frame_id="camera",
        ),
        intrinsics=CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            cx=10.0,
            cy=10.0,
            width=20,
            height=20,
            depth_scale=0.001,
            ts=12.5,
            frame_id="camera",
        ),
        map_from_camera=np.eye(4),
    )


def test_remote_detector_keeps_depth_projection_and_dino_feature_on_host(
    vision_service,
) -> None:
    server_url, requests = vision_service
    source = RgbdObservationSource(
        RemoteDetector(server_url, confidence=0.42),
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
    )
    source.load()

    result = source.observe(_rgbd_frame(), "person")

    assert len(result) == 1
    assert result[0].track_id == 7
    assert result[0].depth == pytest.approx(1.5)
    np.testing.assert_allclose(result[0].dino_feature, [1.0, 0.0, 0.5])
    runtime_detection = to_runtime_detections(result, source_ts=12.5)[0]
    assert runtime_detection.id == "track_7"
    np.testing.assert_allclose(runtime_detection.dino_feature, [1.0, 0.0, 0.5])
    observe_payload = next(payload for path, payload in requests if path == "/observe")
    assert observe_payload["prompt"] == "person"
    assert observe_payload["confidence"] == pytest.approx(0.42)
    assert observe_payload["source_ts"] == pytest.approx(12.5)
    assert isinstance(observe_payload["image_b64"], str)


def test_remote_encoders_and_chat_use_shared_service_contract(vision_service) -> None:
    server_url, requests = vision_service
    image = np.zeros((12, 12, 3), dtype=np.uint8)
    clip = RemoteCLIPEncoder(server_url)
    dino = RemoteDinoEncoder(server_url)
    chat = RemoteVisionClient(server_url)
    clip.load_model()
    dino.load_model()
    chat.load_model()

    np.testing.assert_allclose(clip.encode_image(image), [1.0, 0.5])
    np.testing.assert_allclose(
        clip.encode_text(["person", "chair"]),
        [[1.0, 0.5], [1.0, 0.5]],
    )
    embed_count = sum(path == "/embed" for path, _payload in requests)
    assert clip.encode_text([]).shape == (0,)
    assert sum(path == "/embed" for path, _payload in requests) == embed_count
    assert len(dino.encode_batch([image, image])) == 2
    assert asyncio.run(chat.chat([{"role": "user", "content": "same person?"}])) == "MATCH"

    kinds = [payload["kind"] for path, payload in requests if path == "/embed"]
    assert kinds == ["clip_image", "clip_text", "dino"]
    assert chat.is_available()


def test_remote_clip_preserves_batch_contract_for_person_selection(vision_service) -> None:
    server_url, requests = vision_service
    tracker = PersonTracker()
    tracker.set_clip_encoder(RemoteCLIPEncoder(server_url))
    crops = [
        np.zeros((12, 12, 3), dtype=np.uint8),
        np.full((12, 12, 3), 255, dtype=np.uint8),
    ]
    people = [
        {"id": "first", "label": "person", "position": [1.0, 0.0, 0.0]},
        {"id": "intended", "label": "person", "position": [2.0, 0.0, 0.0]},
    ]

    assert tracker.select_by_clip("person in blue", crops, people) == 1
    assert tracker.status()["id"] == "intended"

    clip_text_payloads = [
        payload
        for path, payload in requests
        if path == "/embed" and payload is not None and payload["kind"] == "clip_text"
    ]
    assert [payload["texts"] for payload in clip_text_payloads] == [["person in blue"]]


def test_remote_chat_keeps_identity_evidence_images_in_order(vision_service) -> None:
    server_url, requests = vision_service
    client = RemoteVisionClient(server_url)

    def image(value: int) -> np.ndarray:
        return np.full((12, 12, 3), value, dtype=np.uint8)

    verdict, candidate_id = asyncio.run(
        verify_candidate(
            "person in blue",
            image(200),
            "candidate-7",
            client.chat,
            reference_crop_bgr=image(20),
            reference_crops_bgr=[image(40), image(60)],
            candidate_crops_bgr=[image(100), image(120), image(140)],
        )
    )

    assert (verdict, candidate_id) == ("MATCH", "candidate-7")
    chat_payload = next(payload for path, payload in requests if path == "/chat")
    content = chat_payload["messages"][0]["content"]
    image_urls = [
        item["image_url"]["url"]
        for item in content
        if item.get("type") == "image_url"
    ]
    assert len(image_urls) == 6
    decoded_means = []
    for image_url in image_urls:
        encoded = base64.b64decode(image_url.split(",", 1)[1])
        decoded = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        assert decoded is not None
        decoded_means.append(float(decoded.mean()))
    np.testing.assert_allclose(decoded_means, [20, 40, 60, 100, 120, 140], atol=2.0)


def test_runtime_detection_dino_tail_roundtrips_and_old_payload_stays_valid() -> None:
    legacy = RuntimeDetection3D(
        id="track_7",
        label="person",
        confidence=0.9,
        position=Vector3(1.0, 2.0, 3.0),
        bbox_2d=[1.0, 2.0, 3.0, 4.0],
        clip_feature=np.array([0.2, 0.4], dtype=np.float32),
        ts=12.5,
    )
    restored_legacy = RuntimeDetection3D.decode(legacy.encode())
    assert restored_legacy.dino_feature is None

    current = RuntimeDetection3D(**{**legacy.__dict__, "dino_feature": np.array([0.1, 0.3, 0.9], dtype=np.float32)})
    restored = RuntimeDetection3D.decode(current.encode())
    np.testing.assert_allclose(restored.clip_feature, current.clip_feature)
    np.testing.assert_allclose(restored.dino_feature, current.dino_feature)
    assert restored.to_dict()["has_dino_feature"] is True
    assert "dino_feature" not in restored.to_dict()


def test_perception_stack_passes_remote_service_url_to_detector_spec() -> None:
    detector_config = SimpleNamespace(
        model_size="l",
        confidence_threshold=0.3,
        iou_threshold=0.45,
        max_detections=64,
        min_box_size_px=12,
        device="",
        model_path="",
    )
    tracking_config = SimpleNamespace(merge_distance=0.5, iou_threshold=0.3, max_objects=100)
    perception_config = SimpleNamespace(
        detector=detector_config,
        tracking=tracking_config,
        default_classes="person",
        min_depth=0.3,
        max_depth=6.0,
        laplacian_threshold=0.0,
        skip_frames=0,
        max_rgbd_skew_s=0.05,
        max_odom_age_s=0.2,
        max_map_odom_age_s=0.2,
    )
    robot_config = SimpleNamespace(
        perception=perception_config,
        camera=SimpleNamespace(depth_scale=0.001),
    )

    _, detector, _ = _perception_runtime_config(
        robot_config,
        "remote",
        {"vision_service_url": "http://vision:8080"},
    )

    assert detector == DetectorSpec(name="remote", server_url="http://vision:8080")
