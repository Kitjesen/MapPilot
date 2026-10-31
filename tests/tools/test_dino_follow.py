"""Narrow contracts for the DINO person-follow A/B replay."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import tools.datasets.dino_follow as dino_follow
from tools.datasets.dino_follow import (
    _apply_identity_result,
    _dino_verifier_strata,
    _draw_state,
    _future_result_at_arrival,
    _identity_changes,
    _identity_metrics,
    _is_hf_clip_dir,
    _live_video_frames,
    _make_trackers,
    _positions_for_rgbd,
    _resolve_yolo_device,
    _select_target,
    _TimedEncoder,
    _tum_frames,
    _update_tracker,
    _variant_metrics,
    _verify_pending_identity,
    _visibility_episodes,
    _warm_identity_chat,
    detections_from_record,
    select_initial_index,
)

from decision.vision.person import PersonTracker
from perception.detection.detector_base import Detection2D


class _FakeClip:
    def encode_image(self, _crop):
        return np.ones(4, dtype=np.float32) / 2.0


class _FakeDino:
    backend = "test"

    def encode(self, _crop):
        feature = np.zeros(384, dtype=np.float32)
        feature[0] = 1.0
        return feature


class _FakeIdentityChat:
    def __init__(self, response: str) -> None:
        self.response = response
        self.calls = []

    async def chat(self, messages):
        self.calls.append({"model": "fake-vlm", "response": self.response, "messages": messages})
        return self.response


def test_cached_detector_and_native_tracker_rows_are_replayed_exactly() -> None:
    restored = detections_from_record(
        {
            "frame": 4,
            "detections": [
                {
                    "bbox_xyxy": [10.0, 20.0, 30.0, 60.0],
                    "score": 0.72,
                    "class_id": 0,
                    "detection_index": 7,
                }
            ],
            "tracks": [
                {
                    "track_id": 99,
                    "bbox_xyxy": [0, 0, 1, 1],
                    "score": 0.61,
                    "class_id": 0,
                    "detection_index": 7,
                }
            ],
        }
    )

    np.testing.assert_allclose(restored.xyxy, [[10.0, 20.0, 30.0, 60.0]])
    np.testing.assert_allclose(restored.conf, [0.72])
    np.testing.assert_array_equal(restored._orig_indices, [7])
    np.testing.assert_allclose(restored._cached_tracks, [[0, 0, 1, 1, 99, 0.61, 0, 7]])


def test_visibility_episodes_report_real_leave_and_return_without_inventing_frames() -> None:
    records = [
        {"frame": 10, "source_frame": 30, "time_s": 1.0, "target": {"track_id": 4}},
        {"frame": 11, "source_frame": 31, "time_s": 1.1, "target": {"track_id": 4}},
        {"frame": 12, "source_frame": 32, "time_s": 1.2, "target": None},
        {"frame": 13, "source_frame": 33, "time_s": 1.3, "target": None},
        {"frame": 14, "source_frame": 34, "time_s": 1.4, "target": {"track_id": 9}},
        {"frame": 15, "source_frame": 35, "time_s": 1.5, "target": {"track_id": 9}},
    ]

    result = _visibility_episodes(records)

    assert result[0] == {
        "state": "visible",
        "start_frame": 10,
        "end_frame": 11,
        "start_source_frame": 30,
        "end_source_frame": 31,
        "start_time_s": 1.0,
        "end_time_s": 1.1,
        "frames": 2,
        "track_ids": [4],
    }
    assert result[1]["state"] == "missing"
    assert result[1]["frames"] == 2
    assert result[2]["track_ids"] == [9]


def test_identity_proposal_is_verified_once_and_queued_for_next_frame(tmp_path) -> None:
    class Tracker:
        resolved = []

        def identity_candidate(self):
            crop = np.full((16, 10, 3), 80, dtype=np.uint8)
            return {
                "object": {"id": "17", "ts": 12.5},
                "crop": crop,
                "reference_crop": np.full((18, 12, 3), 120, dtype=np.uint8),
            }

        def resolve_identity(self, candidate_id, source_ts, verdict):
            self.resolved.append((candidate_id, source_ts, verdict))
            return verdict == "MATCH"

    tracker = Tracker()
    chat = _FakeIdentityChat("MATCH")
    seen = set()

    first = _verify_pending_identity(
        tracker,
        chat,
        "person wearing a plaid shirt",
        tmp_path,
        0,
        seen,
    )
    repeated = _verify_pending_identity(
        tracker,
        chat,
        "person wearing a plaid shirt",
        tmp_path,
        1,
        seen,
    )

    assert first["verdict"] == "MATCH"
    assert first["authorization_queued"]
    assert tracker.resolved == [("17", 12.5, "MATCH")]
    assert repeated is None
    assert len(chat.calls) == 1
    assert (tmp_path / "attempt-000-id-17-candidate.jpg").is_file()
    assert (tmp_path / "attempt-000-id-17-reference.jpg").is_file()


def test_identity_verification_preserves_all_temporal_and_reference_evidence(
    monkeypatch, tmp_path
) -> None:
    candidate_crops = [
        np.full((12, 8, 3), value, dtype=np.uint8)
        for value in (30, 60, 90)
    ]
    extra_references = [
        np.full((14, 9, 3), value, dtype=np.uint8)
        for value in (130, 160)
    ]

    class Tracker:
        def identity_candidate(self):
            return {
                "object": {"id": "28", "ts": 8.3},
                "crop": candidate_crops[-1],
                "reference_crop": np.full((16, 10, 3), 110, dtype=np.uint8),
                "crops": candidate_crops,
                "crop_timestamps": [8.1, 8.2, 8.3],
                "reference_crops": extra_references,
                "reference_timestamps": [2.5, 5.7],
            }

        def resolve_identity(self, candidate_id, source_ts, verdict):
            assert (candidate_id, source_ts, verdict) == ("28", 8.3, "MATCH")
            return True

    captured = {}

    async def verify(
        description,
        crop,
        candidate_id,
        _chat,
        *,
        reference_crop_bgr,
        reference_crops_bgr,
        candidate_crops_bgr,
    ):
        captured.update(
            description=description,
            crop=crop,
            candidate_id=candidate_id,
            reference_crop=reference_crop_bgr,
            reference_crops=reference_crops_bgr,
            candidate_crops=candidate_crops_bgr,
        )
        return "MATCH", candidate_id

    monkeypatch.setattr(dino_follow, "verify_candidate", verify)
    chat = _FakeIdentityChat("unused")
    chat.calls.append(
        {
            "model": "fake-vlm",
            "response": "MATCH",
            "elapsed_s": 0.125,
            "image_decode_s": 0.005,
            "base64_decode_s": 0.001,
            "image_pil_decode_s": 0.004,
            "processor_s": 0.030,
            "processor_cpu_s": 0.024,
            "input_transfer_s": 0.006,
            "generate_s": 0.085,
            "response_decode_s": 0.005,
            "input_tokens": 900,
            "output_tokens": 1,
            "image_count": 6,
        }
    )

    result = _verify_pending_identity(
        Tracker(), chat, "plaid shirt", tmp_path, 4, set()
    )

    assert captured["candidate_id"] == 28
    assert len(captured["candidate_crops"]) == 3
    assert len(captured["reference_crops"]) == 2
    assert result["candidate_crop_timestamps"] == [8.1, 8.2, 8.3]
    assert len(result["candidate_crops"]) == 3
    assert len(result["reference_crops"]) == 3
    assert len(result["extra_reference_crops"]) == 2
    assert result["reference_crop_timestamps"] == [2.5, 5.7]
    assert result["model_elapsed_ms"] == pytest.approx(125.0)
    assert result["model_timing_ms"] == {
        "image_decode_s": pytest.approx(5.0),
        "base64_decode_s": pytest.approx(1.0),
        "image_pil_decode_s": pytest.approx(4.0),
        "processor_s": pytest.approx(30.0),
        "processor_cpu_s": pytest.approx(24.0),
        "input_transfer_s": pytest.approx(6.0),
        "generate_s": pytest.approx(85.0),
        "response_decode_s": pytest.approx(5.0),
    }
    assert result["model_input_tokens"] == 900
    assert result["model_output_tokens"] == 1
    assert result["model_image_count"] == 6
    assert all((tmp_path / Path(path).name).is_file() for path in result["candidate_crops"])
    assert all((tmp_path / Path(path).name).is_file() for path in result["reference_crops"])


def test_identity_metrics_report_call_count_verdicts_and_total_model_time() -> None:
    result = _identity_metrics(
        [
            {
                "verdict": "MATCH",
                "elapsed_ms": 130.0,
                "model_elapsed_ms": 120.0,
                "model_timing_ms": {"processor_s": 35.0, "generate_s": 80.0},
                "model_input_tokens": 900,
                "model_output_tokens": 1,
                "model_image_count": 6,
            },
            {
                "verdict": "UNKNOWN",
                "elapsed_ms": 90.0,
                "model_elapsed_ms": 80.0,
                "model_timing_ms": {"processor_s": 25.0, "generate_s": 50.0},
                "model_input_tokens": 880,
                "model_output_tokens": 1,
                "model_image_count": 6,
            },
            {
                "verdict": "OTHER",
                "elapsed_ms": 70.0,
                "model_elapsed_ms": None,
                "model_timing_ms": {"processor_s": 15.0, "generate_s": 40.0},
                "model_input_tokens": 860,
                "model_output_tokens": 1,
                "model_image_count": 6,
            },
        ]
    )

    assert result == {
        "attempt_count": 3,
        "verdict_counts": {"MATCH": 1, "UNKNOWN": 1, "OTHER": 1},
        "total_elapsed_ms": 290.0,
        "mean_elapsed_ms": pytest.approx(290.0 / 3.0),
        "total_model_elapsed_ms": 200.0,
        "mean_model_elapsed_ms": 100.0,
        "model_timing_total_ms": {"generate_s": 170.0, "processor_s": 75.0},
        "mean_model_input_tokens": 880.0,
        "mean_model_output_tokens": 1.0,
        "mean_model_image_count": 6.0,
    }


def test_timed_encoder_reports_actual_calls_items_and_elapsed_time(monkeypatch) -> None:
    clock = iter((1.0, 1.004, 2.0, 2.006, 3.0, 3.009))
    monkeypatch.setattr(dino_follow.time, "perf_counter", lambda: next(clock))

    class Encoder:
        backend = "test"

        def encode_image(self, image):
            return np.asarray(image)

        def encode_text(self, texts):
            return np.asarray(texts)

        def encode_batch(self, crops):
            return np.asarray(crops)

    encoder = _TimedEncoder(Encoder(), ("encode_batch", "encode_image", "encode_text"))
    before = encoder.snapshot()
    encoder.encode_image(np.zeros((2, 2, 3), dtype=np.uint8))
    encoder.encode_text(["person", "background"])
    encoder.encode_batch([
        np.zeros((2, 2, 3), dtype=np.uint8),
        np.ones((2, 2, 3), dtype=np.uint8),
        np.full((2, 2, 3), 2, dtype=np.uint8),
    ])

    assert encoder.backend == "test"
    assert encoder.timing() == {
        "encode_batch": {
            "calls": 1,
            "items": 3,
            "total_ms": pytest.approx(9.0),
            "mean_ms": pytest.approx(9.0),
            "p50_ms": pytest.approx(9.0),
            "p95_ms": pytest.approx(9.0),
            "max_ms": pytest.approx(9.0),
        },
        "encode_image": {
            "calls": 1,
            "items": 1,
            "total_ms": pytest.approx(4.0),
            "mean_ms": pytest.approx(4.0),
            "p50_ms": pytest.approx(4.0),
            "p95_ms": pytest.approx(4.0),
            "max_ms": pytest.approx(4.0),
        },
        "encode_text": {
            "calls": 1,
            "items": 2,
            "total_ms": pytest.approx(6.0),
            "mean_ms": pytest.approx(6.0),
            "p50_ms": pytest.approx(6.0),
            "p95_ms": pytest.approx(6.0),
            "max_ms": pytest.approx(6.0),
        },
    }
    assert encoder.delta(before) == {
        "encode_batch": {"calls": 1, "items": 3, "total_ms": pytest.approx(9.0)},
        "encode_image": {"calls": 1, "items": 1, "total_ms": pytest.approx(4.0)},
        "encode_text": {"calls": 1, "items": 2, "total_ms": pytest.approx(6.0)},
    }


def test_current_unknown_identity_is_applied_without_being_labeled_obsolete() -> None:
    class Tracker:
        def identity_candidate(self):
            return {"object": {"id": "17", "ts": 13.3}}

        def resolve_identity(self, candidate_id, source_ts, verdict):
            assert (candidate_id, source_ts, verdict) == ("17", 13.3, "UNKNOWN")
            return False

    result = _apply_identity_result(
        Tracker(),
        {"candidate_id": "17", "source_ts": 13.3, "verdict": "UNKNOWN"},
        390,
        13.4,
    )

    assert result["request_still_current"] is True
    assert result["verdict_applied"] is True
    assert result["authorization_queued"] is False
    assert result["obsolete_on_arrival"] is False


def test_identity_result_completing_during_load_waits_for_next_arrival() -> None:
    class Future:
        complete = False

        def done(self):
            return self.complete

        def result(self):
            return ({"verdict": "MATCH"}, 2.0)

    future = Future()
    assert _future_result_at_arrival(future) is None
    future.complete = True  # deferred RGB/depth load completed after the arrival poll
    assert _future_result_at_arrival(future) == ({"verdict": "MATCH"}, 2.0)


def test_completed_match_resolves_before_fresh_update() -> None:
    events = []

    class Tracker:
        def identity_candidate(self):
            return {"object": {"id": "4", "ts": 7.7}}

        def resolve_identity(self, *_args):
            events.append("resolve")
            return True

        def update(self):
            events.append("fresh_update")

    tracker = Tracker()
    result = _apply_identity_result(
        tracker,
        {"candidate_id": "4", "source_ts": 7.7, "verdict": "MATCH"},
        224,
        7.8,
    )
    tracker.update()

    assert result["authorization_queued"] is True
    assert events == ["resolve", "fresh_update"]


def test_dino_verifier_strata_excludes_pending_frames_without_dino_work() -> None:
    records = [
        {
            "identity_future_running_at_update": True,
            "encoder_timing": {"dino": {"encode": {"calls": 0, "total_ms": 0.0}}},
        },
        {
            "identity_future_running_at_update": True,
            "encoder_timing": {
                "dino": {
                    "encode": {"calls": 0, "total_ms": 0.0},
                    "encode_batch": {"calls": 1, "total_ms": 9.0},
                }
            },
        },
        {
            "identity_future_running_at_update": False,
            "encoder_timing": {"dino": {"encode": {"calls": 1, "total_ms": 5.0}}},
        },
    ]

    strata = _dino_verifier_strata(records)

    assert strata["verifier_running"]["frames_with_dino"] == 1
    assert strata["verifier_running"]["dino_p50_ms"] == 9.0
    assert strata["verifier_idle"]["frames_with_dino"] == 1
    assert strata["verifier_idle"]["dino_p50_ms"] == 5.0


def test_hugging_face_clip_directory_is_detected_without_loading_weights(tmp_path) -> None:
    model = tmp_path / "clip"
    model.mkdir()
    assert not _is_hf_clip_dir(model)
    (model / "config.json").write_text("{}", encoding="utf-8")
    assert _is_hf_clip_dir(model)


def test_auto_device_is_resolved_only_for_ultralytics(monkeypatch) -> None:
    fake_torch = SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: True))
    monkeypatch.setitem(sys.modules, "torch", fake_torch)

    assert _resolve_yolo_device("auto") == "cuda:0"
    assert _resolve_yolo_device("cpu") == "cpu"


def test_lock_frame_is_recorded_without_replaying_the_same_timestamp() -> None:
    class Tracker:
        calls = 0

        def update(self, _objects, _frame):
            self.calls += 1
            return False

    tracker = Tracker()
    updated = _update_tracker(
        tracker,
        [{"ts": 4.0}],
        np.zeros((2, 2, 3), dtype=np.uint8),
        locked=True,
        just_locked=True,
    )

    assert updated
    assert tracker.calls == 0


def test_live_video_runs_yolo_world_and_returns_real_detector_input(monkeypatch, tmp_path) -> None:
    import cv2

    import runtime.registry

    frame = np.zeros((12, 20, 3), dtype=np.uint8)

    class Capture:
        released = False

        def isOpened(self):
            return True

        def get(self, _key):
            return 25.0

        def read(self):
            if self.released:
                return False, None
            self.released = True
            return True, frame

        def release(self):
            self.released = True

    class Detector:
        loaded = False
        stopped = False

        def load_model(self):
            self.loaded = True

        def detect(self, image, prompt):
            assert self.loaded
            assert image is frame
            assert prompt == "person"
            return [Detection2D(np.asarray([1, 2, 10, 11]), 0.8, "person")]

        def shutdown(self):
            self.stopped = True

    detector = Detector()
    received_specs = []

    def create(spec):
        received_specs.append(spec)
        return detector

    backend = SimpleNamespace(create=create)
    monkeypatch.setattr(runtime.registry, "get", lambda *_args: backend)
    monkeypatch.setattr(cv2, "VideoCapture", lambda _path: Capture())
    monkeypatch.setitem(
        sys.modules,
        "torch",
        SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: True)),
    )
    args = SimpleNamespace(
        video=tmp_path / "source.mp4",
        yolo_weights=tmp_path / "yolo.pt",
        device="auto",
        max_frames=2,
    )

    frames = list(_live_video_frames(args))

    assert len(frames) == 1
    assert frames[0][1] == 0.0
    np.testing.assert_allclose(frames[0][5].xyxy, [[1, 2, 10, 11]])
    assert args.device == "auto"
    assert received_specs[0].device == "cuda:0"
    assert detector.stopped


def test_tum_cache_replays_source_frame_without_loading_yolo(monkeypatch, tmp_path) -> None:
    import tools.datasets.tum_rgbd_probe as tum_probe
    from PIL import Image

    import runtime.registry

    cache = tmp_path / "detector-cache.json"
    cache.write_text(
        json.dumps(
            [
                {
                    "frame": 0,
                    "source_frame": 1,
                    "time_s": 20.0,
                    "detections": [
                        {
                            "bbox_xyxy": [1, 2, 8, 10],
                            "score": 0.8,
                            "class_id": 0,
                            "detection_index": 3,
                        }
                    ],
                }
            ]
        ),
        encoding="utf-8",
    )
    pairs = [
        {"rgb_timestamp": 10.0, "rgb": tmp_path / "rgb0.png", "depth": tmp_path / "depth0.png"},
        {"rgb_timestamp": 20.0, "rgb": tmp_path / "rgb1.png", "depth": tmp_path / "depth1.png"},
    ]
    monkeypatch.setattr(tum_probe, "_indexed", lambda _path: [])
    monkeypatch.setattr(tum_probe, "_associate", lambda *_args: pairs)
    monkeypatch.setattr(tum_probe, "_poses", lambda _path: [(20.0, np.eye(4))])
    monkeypatch.setattr(tum_probe, "_nearest_pose", lambda *_args: (20.0, np.eye(4)))
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[..., 0], rgb[..., 1], rgb[..., 2] = 17, 89, 201
    depth_values = np.ones((12, 16), dtype=np.uint16)
    depth_values[0, :3] = [0, 5000, 65535]
    Image.fromarray(rgb[:, :, ::-1]).save(pairs[1]["rgb"])
    Image.fromarray(depth_values).save(pairs[1]["depth"])
    monkeypatch.setattr(runtime.registry, "get", lambda *_args: pytest.fail("YOLO must not load"))
    args = SimpleNamespace(
        rgbd_sequence=tmp_path,
        tracks=cache,
        yolo_weights=None,
        device="auto",
        max_frames=10,
    )

    frames = list(_tum_frames(args))

    assert len(frames) == 1
    assert frames[0][0] == 1
    assert frames[0][1] == 20.0
    frame, depth, detector_input, timing = frames[0][2]()
    assert frame.shape == (12, 16, 3)
    assert depth.shape == (12, 16)
    assert depth.dtype == np.uint16
    np.testing.assert_array_equal(depth[0, :3], [0, 5000, 65535])
    np.testing.assert_allclose(detector_input.xyxy, [[1, 2, 8, 10]])
    np.testing.assert_array_equal(detector_input._orig_indices, [3])
    assert timing["rgb_decode_ms"] >= 0.0
    assert timing["rgb_file_read_ms"] >= 0.0
    assert timing["rgb_imdecode_ms"] >= 0.0
    assert timing["depth_decode_ms"] >= 0.0
    assert timing["depth_file_read_ms"] >= 0.0
    assert timing["depth_imdecode_ms"] >= 0.0
    assert timing["cache_restore_ms"] >= 0.0
    assert timing["detector_ms"] is None


def test_recorded_camera_rgbd_uses_real_calibration_and_camera_local_frame(
    monkeypatch, tmp_path,
) -> None:
    from PIL import Image

    import runtime.registry

    (tmp_path / "rgb").mkdir()
    (tmp_path / "depth").mkdir()
    rgb = np.zeros((12, 16, 3), dtype=np.uint8)
    rgb[..., 0], rgb[..., 1], rgb[..., 2] = 17, 89, 201
    depth = np.full((12, 16), 1000, dtype=np.uint16)
    Image.fromarray(rgb[:, :, ::-1]).save(tmp_path / "rgb" / "42.png")
    Image.fromarray(depth).save(tmp_path / "depth" / "42.png")
    (tmp_path / "frames.json").write_text(
        json.dumps(
            [
                {
                    "frame": 0,
                    "source_timestamp_ns": 42,
                    "source_timestamp_s": 4.2e-8,
                    "rgb_depth_skew_ns": 0,
                    "rgb": "rgb/42.png",
                    "depth": "depth/42.png",
                }
            ]
        ),
        encoding="utf-8",
    )
    (tmp_path / "camera.json").write_text(
        json.dumps(
            {
                "width": 16,
                "height": 12,
                "frame_id": "camera_color_optical_frame",
                "fx": 10.0,
                "fy": 10.0,
                "cx": 8.0,
                "cy": 6.0,
                "depth_scale": 0.001,
            }
        ),
        encoding="utf-8",
    )
    (tmp_path / "manifest.json").write_text(
        json.dumps(
            {
                "schema": "lingtu.camera.shm_frame.v1",
                "map_pose_available": False,
                "camera_to_body_extrinsics_verified": False,
            }
        ),
        encoding="utf-8",
    )
    cache = tmp_path / "detector-cache.json"
    cache.write_text(
        json.dumps(
            [
                {
                    "frame": 0,
                    "source_frame": 0,
                    "time_s": 4.2e-8,
                    "detections": [
                        {
                            "bbox_xyxy": [4, 2, 12, 10],
                            "score": 0.8,
                            "class_id": 0,
                            "detection_index": 0,
                        }
                    ],
                }
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(runtime.registry, "get", lambda *_args: pytest.fail("YOLO must not load"))
    args = SimpleNamespace(
        rgbd_sequence=tmp_path,
        tracks=cache,
        yolo_weights=None,
        device="auto",
        max_frames=10,
    )

    frames = list(_tum_frames(args))
    frame, loaded_depth, _detector_input, _ = frames[0][2]()
    positions, invalid = _positions_for_rgbd(
        np.asarray([[4, 2, 12, 10, 1, 0.8, 0, 0]], dtype=np.float32),
        loaded_depth,
        frames[0][4],
        depth_method="center",
        intrinsics=args._rgbd_intrinsics,
        depth_scale=args._rgbd_depth_scale,
    )

    np.testing.assert_array_equal(frame, rgb)
    np.testing.assert_array_equal(loaded_depth, depth)
    np.testing.assert_allclose(positions, [[1.0, 0.0, 0.0]], atol=1e-6)
    assert invalid == 0
    assert frames[0][0:2] == (0, 4.2e-8)
    assert args._rgbd_source_format == "lingtu.camera.shm_frame.v1"
    assert args._rgbd_position_frame == "camera_local_forward_left_up"
    assert args._rgbd_map_pose_available is False
    assert args._rgbd_camera_to_body_extrinsics_verified is False


def test_tum_live_observation_labels_yolo_separately_from_decode(
    monkeypatch, tmp_path,
) -> None:
    import tools.datasets.tum_rgbd_probe as tum_probe
    from PIL import Image

    import runtime.registry

    pair = {
        "rgb_timestamp": 20.0,
        "rgb": tmp_path / "rgb.png",
        "depth": tmp_path / "depth.png",
    }
    monkeypatch.setattr(tum_probe, "_indexed", lambda _path: [])
    monkeypatch.setattr(tum_probe, "_associate", lambda *_args: [pair])
    monkeypatch.setattr(tum_probe, "_poses", lambda _path: [(20.0, np.eye(4))])
    monkeypatch.setattr(tum_probe, "_nearest_pose", lambda *_args: (20.0, np.eye(4)))
    frame = np.zeros((12, 16, 3), dtype=np.uint8)
    frame[..., 0], frame[..., 1], frame[..., 2] = 17, 89, 201
    Image.fromarray(frame[:, :, ::-1]).save(pair["rgb"])
    Image.fromarray(np.ones((12, 16), dtype=np.uint16)).save(pair["depth"])

    class Detector:
        def load_model(self):
            pass

        def detect(self, image, prompt):
            np.testing.assert_array_equal(image, frame)
            assert prompt == "person"
            return [Detection2D(np.asarray([1, 2, 8, 10]), 0.8, "person")]

        def shutdown(self):
            pass

    backend = SimpleNamespace(create=lambda _spec: Detector())
    monkeypatch.setattr(runtime.registry, "get", lambda *_args: backend)
    args = SimpleNamespace(
        rgbd_sequence=tmp_path,
        tracks=None,
        yolo_weights=tmp_path / "yolo.pt",
        device="cpu",
        max_frames=1,
        pace_source_time=False,
    )

    source = _tum_frames(args)
    first = next(source)
    try:
        _, _, detector_input, timing = first[2]()
    finally:
        source.close()

    np.testing.assert_allclose(detector_input.xyxy, [[1, 2, 8, 10]])
    assert timing["detector_ms"] >= 0.0
    assert timing["rgb_file_read_ms"] >= 0.0
    assert timing["rgb_imdecode_ms"] >= 0.0
    assert timing["depth_decode_ms"] >= 0.0
    assert timing["depth_file_read_ms"] >= 0.0
    assert timing["depth_imdecode_ms"] >= 0.0
    assert timing["cache_restore_ms"] is None


def test_external_bbox_selects_overlap_and_never_reads_identity_labels() -> None:
    tracks = np.asarray(
        [
            [0, 0, 20, 40, 3, 0.8, 0, 0],
            [50, 10, 90, 80, 8, 0.9, 0, 1],
        ],
        dtype=np.float32,
    )

    assert select_initial_index(tracks, target_track_id=8, target_bbox=None) == 1
    assert select_initial_index(tracks, target_track_id=None, target_bbox=[55, 12, 88, 75]) == 1
    assert select_initial_index(tracks, target_track_id=42, target_bbox=None) == -1


@pytest.mark.parametrize(
    ("scores", "expected"),
    [
        ((0.3, 0.8), 1),
        ((0.1, 0.19), -1),
    ],
)
def test_text_selection_ranks_candidates_without_locking_temporary_tracker(
    monkeypatch, scores, expected,
) -> None:
    class Clip:
        def __init__(self):
            self.image_calls = 0

        def encode_text(self, _texts):
            return np.asarray([[1.0, 0.0]], dtype=np.float32)

        def encode_image(self, crop):
            self.image_calls += 1
            score = float(crop[0])
            return np.asarray([score, np.sqrt(1.0 - score * score)], dtype=np.float32)

    crops = [np.asarray([score], dtype=np.float32) for score in scores]
    monkeypatch.setattr(dino_follow, "crop_tracks", lambda *_args: crops)
    clip = Clip()
    args = SimpleNamespace(
        target_track_id=None,
        target_bbox=None,
        target_description="plaid shirt",
    )

    selected = _select_target(
        args,
        np.zeros((4, 4, 3), dtype=np.uint8),
        np.zeros((2, 8), dtype=np.float32),
        clip,
    )

    assert selected == expected
    assert clip.image_calls == 2


def test_identity_warmup_uses_four_synthetic_images_and_keeps_calls_clean() -> None:
    class Chat:
        def __init__(self):
            self.calls = []

        async def chat(self, messages):
            image_count = sum(
                part.get("type") == "image_url"
                for message in messages
                for part in message["content"]
            )
            self.calls.append(
                {
                    "image_count": image_count,
                    "max_new_tokens": 512,
                    "input_tokens": 123,
                    "output_tokens": 1,
                    "base64_decode_s": 0.001,
                    "image_pil_decode_s": 0.002,
                    "processor_cpu_s": 0.003,
                    "input_transfer_s": 0.004,
                    "generate_s": 0.005,
                    "response_decode_s": 0.006,
                }
            )
            return "UNKNOWN"

    chat = Chat()
    result = _warm_identity_chat(chat)

    assert result["verdict"] == "UNKNOWN"
    assert result["image_count"] == 4
    assert result["max_new_tokens"] == 512
    assert result["model_timing_ms"]["generate_s"] == pytest.approx(5.0)
    assert chat.calls == []


def test_rgbd_projection_uses_explicit_freiburg3_intrinsics_and_world_pose() -> None:
    tracks = np.asarray([[310.1, 237.6, 330.1, 257.6, 1, 0.9, 0, 0]], dtype=np.float32)
    depth = np.full((480, 640), 10_000, dtype=np.uint16)
    pose = np.eye(4, dtype=np.float64)
    pose[:3, 3] = [1.0, 2.0, 3.0]

    positions, invalid = _positions_for_rgbd(tracks, depth, pose, depth_method="center")

    assert invalid == 0
    np.testing.assert_allclose(positions[0], [1.0, 2.0, 5.0], atol=1e-6)


def test_person_depth_method_uses_supported_torso_layer_instead_of_center_patch() -> None:
    tracks = np.asarray([[100, 100, 200, 300, 1, 0.9, 0, 0]], dtype=np.float32)
    depth = np.full((480, 640), 10_000, dtype=np.uint16)
    depth[195:206, 145:156] = 25_000
    pose = np.eye(4, dtype=np.float64)

    person, invalid = _positions_for_rgbd(tracks, depth, pose, depth_method="person")
    center, _ = _positions_for_rgbd(tracks, depth, pose, depth_method="center")

    assert invalid == 0
    assert person[0][2] == pytest.approx(2.0)
    assert center[0][2] == pytest.approx(5.0)


def test_rgb_metrics_keep_xyz_absent_and_report_observed_id_changes() -> None:
    records = [
        {"selected_track_id": 4, "target": {"position_xyz": None}},
        {"selected_track_id": 9, "target": {"position_xyz": None}},
        {"selected_track_id": 9, "target": None},
    ]

    result = _variant_metrics(records, [1.0, 2.0, 3.0])

    assert result["visible_target_frames"] == 2
    assert result["missing_target_frames"] == 1
    assert result["selected_track_id_changes"] == 1
    assert result["leave_return_events"] == []
    assert result["position_stability"] == {
        "valid_positions": 0,
        "adjacent_step_samples": 0,
        "observation_gaps": 0,
        "step_median_m": None,
        "step_p95_m": None,
        "step_max_m": None,
        "adjacent_jump_count_over_0_5m": 0,
        "speed_samples": 0,
        "speed_p95_mps": None,
        "speed_max_mps": None,
    }


def test_variant_metrics_expose_leave_return_reacquisition() -> None:
    records = [
        {"frame": 0, "source_frame": 0, "time_s": 0.0, "selected_track_id": 4, "target": {"track_id": 4}},
        {"frame": 1, "source_frame": 1, "time_s": 0.1, "selected_track_id": 4, "target": None},
        {"frame": 2, "source_frame": 2, "time_s": 0.2, "selected_track_id": 8, "target": None},
        {"frame": 3, "source_frame": 3, "time_s": 0.3, "selected_track_id": 8, "target": {"track_id": 8}},
    ]

    result = _variant_metrics(records, [])

    assert result["leave_return_events"] == [
        {
            "left_after_frame": 0,
            "missing_start_frame": 1,
            "missing_end_frame": 2,
            "returned_at_frame": 3,
            "missing_frames": 2,
            "previous_track_id": 4,
            "returned_track_id": 8,
            "track_id_changed": True,
        }
    ]


def test_position_stability_excludes_steps_across_missing_frames_and_separates_ema() -> None:
    records = [
        {
            "frame": 0,
            "time_s": 0.0,
            "target": {"raw_position_xyz": [0.0, 0.0, 0.0], "ema_position_xyz": [0.0, 0.0, 0.0]},
            "selected_track_id": 1,
        },
        {"frame": 1, "time_s": 0.1, "target": None, "selected_track_id": 1},
        {
            "frame": 2,
            "time_s": 0.2,
            "target": {"raw_position_xyz": [2.0, 0.0, 0.0], "ema_position_xyz": [0.2, 0.0, 0.0]},
            "selected_track_id": 1,
        },
        {
            "frame": 3,
            "time_s": 0.3,
            "target": {"raw_position_xyz": [2.2, 0.0, 0.0], "ema_position_xyz": [0.3, 0.0, 0.0]},
            "selected_track_id": 1,
        },
    ]

    metrics = _variant_metrics(records, [])
    raw = metrics["raw_position_stability"]
    ema = metrics["ema_position_stability"]

    assert raw["observation_gaps"] == 1
    assert raw["adjacent_step_samples"] == 1
    assert raw["step_max_m"] == pytest.approx(0.2)
    assert raw["speed_samples"] == 2
    assert raw["speed_max_mps"] == pytest.approx(10.0)
    assert ema["observation_gaps"] == 1
    assert ema["step_max_m"] == pytest.approx(0.1)


def test_visual_state_never_highlights_stale_selected_id_when_target_is_missing() -> None:
    selected, lines = _draw_state(
        {
            "frame": 12,
            "time_s": 0.4,
            "selected_track_id": 7,
            "target": None,
            "geometry_used": False,
            "dino_event": "confirming",
            "dino_similarity": 0.81,
        },
        "CLIP + DINOv3",
    )

    assert selected is None
    assert "TARGET LOST/UNCONFIRMED" in lines[0]
    assert lines[1] == "frame=12"
    assert lines[2] == "RGB only"


def test_visual_state_uses_relative_time_and_separate_short_xyz_lines() -> None:
    selected, lines = _draw_state(
        {
            "frame": 8,
            "time_s": 1_700_000_000.0,
            "elapsed_s": 0.27,
            "target": {
                "track_id": 3,
                "raw_position_xyz": [1.234, -2.345, 3.456],
                "ema_position_xyz": [1.111, -2.222, 3.333],
            },
            "geometry_used": True,
            "dino_event": "matched",
        },
        "CLIP + DINOv3",
    )

    assert selected == 3
    assert lines[1] == "frame=8 t=0.27s"
    assert lines[2] == "raw XYZ=(1.23,-2.35,3.46)"
    assert lines[3] == "EMA XYZ=(1.11,-2.22,3.33)"
    assert all("1700000000" not in line for line in lines)


def test_ab_trackers_are_real_person_trackers_and_only_variant_gets_dino() -> None:
    args = SimpleNamespace(dino_min_similarity=0.65, dino_margin=0.03)
    trackers = _make_trackers(args, _FakeClip(), _FakeDino())
    target = {
        "id": "5",
        "label": "person",
        "bbox": [0, 0, 20, 40],
        "position": [0.0, 0.0, 0.0],
        "confidence": 0.9,
        "ts": 0.0,
    }
    crop = np.zeros((40, 20, 3), dtype=np.uint8)
    for tracker in trackers.values():
        tracker.lock_target(target, crop)

    assert all(isinstance(tracker, PersonTracker) for tracker in trackers.values())
    assert trackers["baseline"].status()["appearance_backend"] == "clip_osnet"
    assert trackers["dino"].status()["appearance_backend"] == "dino"


def test_real_identity_changes_are_derived_only_from_post_replay_attribution() -> None:
    evaluation = {
        "frame_evidence": [
            {"attributed_gt_id": 21},
            {"attributed_gt_id": None},
            {"attributed_gt_id": 21},
            {"attributed_gt_id": 9},
        ]
    }

    assert _identity_changes(evaluation) == 1
