from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

import perception.backends as perception_backends
from perception.backends import (
    CLIPEncoderProvider,
    DetectorSpec,
    RgbdObservationSource,
)
from perception.detection.detector_base import Detection2D
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from runtime.registry import get, list_plugins


class _OneBoxDetector:
    def __init__(self) -> None:
        self.detect_calls = 0
        self.shutdown_calls = 0

    def load_model(self) -> None:
        pass

    def detect(self, _image: np.ndarray, _prompt: str) -> list[Detection2D]:
        self.detect_calls += 1
        return [
            Detection2D(
                bbox=np.array([4, 4, 16, 16], dtype=np.float32),
                score=0.9,
                label="chair",
            )
        ]

    def shutdown(self) -> None:
        self.shutdown_calls += 1


def _frame(depth: Image) -> SimpleNamespace:
    return SimpleNamespace(
        color=Image(
            data=np.zeros((20, 20, 3), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=10.0,
            frame_id="camera",
        ),
        depth=depth,
        intrinsics=CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            cx=10.0,
            cy=10.0,
            width=20,
            height=20,
            depth_scale=0.001,
            ts=10.0,
            frame_id="camera",
        ),
        map_from_camera=np.eye(4),
    )


def test_rgbd_observation_keeps_float_depth_in_metres() -> None:
    source = RgbdObservationSource(
        _OneBoxDetector(),
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
    )
    source.load()

    detections = source.observe(
        _frame(
            Image(
                data=np.full((20, 20), 2.0, dtype=np.float32),
                format=ImageFormat.DEPTH_F32,
                ts=10.0,
                frame_id="camera",
            )
        ),
        "chair",
    )

    assert len(detections) == 1
    assert detections[0].depth == pytest.approx(2.0)
    assert detections[0].position[2] == pytest.approx(2.0)


@pytest.mark.parametrize(
    ("camera_scale", "fallback", "expected_depth"),
    [
        (0.002, 0.001, 2.0),
        (1.0, 0.001, 1.0),
    ],
)
def test_uint16_depth_uses_camera_scale_then_configured_fallback(
    camera_scale: float,
    fallback: float,
    expected_depth: float,
) -> None:
    source = RgbdObservationSource(
        _OneBoxDetector(),
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=fallback,
    )
    source.load()
    frame = _frame(
        Image(
            data=np.full((20, 20), 1000, dtype=np.uint16),
            format=ImageFormat.DEPTH_U16,
            ts=10.0,
            frame_id="camera",
        )
    )
    frame.intrinsics.depth_scale = camera_scale

    detections = source.observe(frame, "chair")

    assert detections[0].depth == pytest.approx(expected_depth)


def test_unsupported_depth_format_is_rejected_before_detection() -> None:
    detector = _OneBoxDetector()
    source = RgbdObservationSource(
        detector,
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
    )
    source.load()

    with pytest.raises(ValueError, match="unsupported_depth_format:GRAY"):
        source.observe(
            _frame(
                Image(
                    data=np.zeros((20, 20), dtype=np.uint8),
                    format=ImageFormat.GRAY,
                    ts=10.0,
                    frame_id="camera",
                )
            ),
            "chair",
        )

    assert detector.detect_calls == 0


def test_2d_tracker_failure_uses_raw_detector_output() -> None:
    class BrokenTracker:
        def track(self, _image: np.ndarray, _prompt: str) -> list[Detection2D]:
            raise RuntimeError("tracker unavailable")

    detector = _OneBoxDetector()
    source = RgbdObservationSource(
        detector,
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
        detector_tracker=BrokenTracker(),
    )
    source.load()

    detections = source.observe(
        _frame(
            Image(
                data=np.full((20, 20), 1000, dtype=np.uint16),
                format=ImageFormat.DEPTH_U16,
                ts=10.0,
                frame_id="camera",
            )
        ),
        "chair",
    )

    assert len(detections) == 1
    assert detector.detect_calls == 1
    assert source.health()["detector_tracker_fallbacks"] == 1


def test_mask_projection_receives_runtime_camera_distortion(monkeypatch) -> None:
    mask = np.ones((20, 20), dtype=bool)

    class MaskedDetector(_OneBoxDetector):
        def detect(self, _image: np.ndarray, _prompt: str) -> list[Detection2D]:
            return [
                Detection2D(
                    bbox=np.array([4, 4, 16, 16], dtype=np.float32),
                    score=0.9,
                    label="chair",
                    mask=mask,
                )
            ]

    captured: dict[str, np.ndarray] = {}

    def project_mask(**kwargs):
        captured["K"] = kwargs["K"]
        captured["D"] = kwargs["D"]
        return np.tile(np.array([[0.0, 0.0, 2.0]]), (10, 1))

    monkeypatch.setattr(perception_backends, "mask_to_pointcloud", project_mask)
    source = RgbdObservationSource(
        MaskedDetector(),
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
    )
    source.load()
    frame = _frame(
        Image(
            data=np.full((20, 20), 2.0, dtype=np.float32),
            format=ImageFormat.DEPTH_F32,
            ts=10.0,
            frame_id="camera",
        )
    )
    frame.intrinsics.dist_k1 = 0.1
    frame.intrinsics.dist_k2 = -0.02

    detections = source.observe(frame, "chair")

    assert len(detections) == 1
    np.testing.assert_allclose(captured["K"], frame.intrinsics.K_matrix)
    np.testing.assert_allclose(captured["D"], frame.intrinsics.D_vector)


def test_backend_resource_is_closed_once_by_identity() -> None:
    detector = _OneBoxDetector()
    source = RgbdObservationSource(
        detector,
        min_depth=0.3,
        max_depth=6.0,
        u16_depth_scale=0.001,
        detector_tracker=detector,
    )
    source.load()

    source.close()
    source.close()

    assert detector.shutdown_calls == 1


def test_canonical_registry_has_all_supported_detectors() -> None:
    expected = {"yoloe", "yolo_world", "bpu", "grounding_dino", "sim_scene"}

    assert expected <= set(list_plugins("detector"))
    assert get("detector", "yoloe").create(DetectorSpec()).model_size == "l"


def test_encoder_provider_needs_no_module_object() -> None:
    encoder = CLIPEncoderProvider.create()

    assert callable(encoder.encode_text)
    assert callable(encoder.encode_image)
