#!/usr/bin/env python3
"""Replay the real PersonTracker with and without DINOv3 appearance features.

Both variants consume the same RGB frames and NativeByteTracker detections. CLIP
remains attached to both trackers for text selection and the existing baseline.
MOT labels, when supplied, are read only after replay for evaluation.
"""

from __future__ import annotations

import argparse
import asyncio
import io
import json
import sys
import time
from concurrent.futures import Future, ThreadPoolExecutor
from itertools import pairwise
from pathlib import Path
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from tools.datasets.person_follow import TrackerInput, crop_tracks

from decision.vision.person import PersonTracker, verify_candidate
from perception.detection.detector_base import Detection2D
from perception.encoding.clip_encoder import CLIPEncoder
from perception.encoding.dino import DinoEncoder
from perception.tracking.native_byte_tracker import NativeByteTracker
from perception.tracking.projection import (
    bbox_center_depth,
    person_depth_position,
    project_to_3d,
)
from runtime.msgs.sensor import CameraIntrinsics

TUM_INTRINSICS = CameraIntrinsics(
    fx=535.4,
    fy=539.2,
    cx=320.1,
    cy=247.6,
    depth_scale=1.0 / 5000.0,
)

CAMERA_LOCAL_FROM_OPTICAL = np.asarray(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


class _HFClipEncoder:
    """Minimal local Hugging Face CLIP adapter matching PersonTracker's API."""

    def __init__(self, model_path: Path, device: str) -> None:
        import torch
        from transformers import CLIPModel, CLIPProcessor

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self._device = device
        self._model = CLIPModel.from_pretrained(
            str(model_path), local_files_only=True
        ).to(device).eval()
        self._processor = CLIPProcessor.from_pretrained(
            str(model_path), local_files_only=True
        )

    @staticmethod
    def _normalise(value):
        value = value.float()
        return value / value.norm(dim=-1, keepdim=True).clamp_min(1e-8)

    def encode_image(self, image: np.ndarray) -> np.ndarray:
        import torch
        from PIL import Image

        inputs = self._processor(
            images=Image.fromarray(image[:, :, ::-1]), return_tensors="pt"
        )
        with torch.inference_mode():
            feature = self._model.get_image_features(
                pixel_values=inputs["pixel_values"].to(self._device)
            )
        return self._normalise(feature).cpu().numpy()[0]

    def encode_text(self, texts: list[str]) -> np.ndarray:
        import torch

        inputs = self._processor(text=texts, padding=True, return_tensors="pt")
        with torch.inference_mode():
            feature = self._model.get_text_features(
                input_ids=inputs["input_ids"].to(self._device),
                attention_mask=inputs["attention_mask"].to(self._device),
            )
        return self._normalise(feature).cpu().numpy()

    def shutdown(self) -> None:
        self._model = None
        self._processor = None


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


class _TimedEncoder:
    """Measure actual encoder work without changing the encoder contract."""

    def __init__(self, encoder: Any, methods: tuple[str, ...]) -> None:
        self._encoder = encoder
        self._methods = methods
        self._timings = {
            name: {"calls": 0, "total_ms": 0.0, "items": 0, "samples_ms": []}
            for name in methods
        }

    def __getattr__(self, name: str) -> Any:
        return getattr(self._encoder, name)

    def _call(self, name: str, *args: Any, **kwargs: Any) -> Any:
        started = time.perf_counter()
        try:
            return getattr(self._encoder, name)(*args, **kwargs)
        finally:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            timing = self._timings[name]
            timing["calls"] += 1
            timing["total_ms"] += elapsed_ms
            timing["samples_ms"].append(elapsed_ms)
            first = args[0] if args else None
            timing["items"] += (
                len(first)
                if name in {"encode_batch", "encode_text"} and isinstance(first, list)
                else 1
            )

    def encode(self, crop: np.ndarray) -> np.ndarray:
        return self._call("encode", crop)

    def encode_batch(self, crops: list[np.ndarray]) -> list[np.ndarray]:
        return self._call("encode_batch", crops)

    def encode_image(self, image: np.ndarray) -> np.ndarray:
        return self._call("encode_image", image)

    def encode_text(self, texts: list[str]) -> np.ndarray:
        return self._call("encode_text", texts)

    def timing(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for name, timing in self._timings.items():
            calls = int(timing["calls"])
            total_ms = float(timing["total_ms"])
            result[name] = {
                "calls": calls,
                "items": int(timing["items"]),
                "total_ms": total_ms,
                "mean_ms": total_ms / calls if calls else None,
                "p50_ms": float(np.percentile(timing["samples_ms"], 50)) if calls else None,
                "p95_ms": float(np.percentile(timing["samples_ms"], 95)) if calls else None,
                "max_ms": float(max(timing["samples_ms"])) if calls else None,
            }
        return result

    def snapshot(self) -> dict[str, tuple[int, float, int]]:
        return {
            name: (
                int(timing["calls"]),
                float(timing["total_ms"]),
                int(timing["items"]),
            )
            for name, timing in self._timings.items()
        }

    def delta(self, before: dict[str, tuple[int, float, int]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for name, timing in self.snapshot().items():
            previous = before[name]
            result[name] = {
                "calls": timing[0] - previous[0],
                "total_ms": timing[1] - previous[1],
                "items": timing[2] - previous[2],
            }
        return result


def _verify_pending_identity(
    tracker: PersonTracker,
    chat: Any,
    description: str,
    artifact_dir: Path,
    attempt_index: int,
    seen: set[tuple[str, float | None]],
) -> dict[str, Any] | None:
    """Verify one new DINO proposal and queue its verdict for the next frame."""
    candidate = _take_identity_candidate(tracker, seen)
    if candidate is None:
        return None
    result = _run_identity_request(
        candidate,
        chat,
        description,
        artifact_dir,
        attempt_index,
    )
    result["authorization_queued"] = bool(
        tracker.resolve_identity(result["candidate_id"], result["source_ts"], result["verdict"])
    )
    return result


def _take_identity_candidate(
    tracker: PersonTracker,
    seen: set[tuple[str, float | None]],
) -> dict[str, Any] | None:
    """Snapshot one unseen proposal so model work cannot observe later tracker mutation."""
    candidate = tracker.identity_candidate()
    if candidate is None:
        return None
    obj = candidate["object"]
    candidate_id = str(obj["id"])
    source_ts = float(obj["ts"]) if obj.get("ts") is not None else None
    key = (candidate_id, source_ts)
    if key in seen:
        return None
    seen.add(key)
    return candidate


def _run_identity_request(
    candidate: dict[str, Any],
    chat: Any,
    description: str,
    artifact_dir: Path,
    attempt_index: int,
) -> dict[str, Any]:
    """Run one immutable identity snapshot; tracker resolution stays on the frame thread."""
    obj = candidate["object"]
    candidate_id = str(obj["id"])
    source_ts = float(obj["ts"]) if obj.get("ts") is not None else None

    crop = np.asarray(candidate["crop"])
    reference = np.asarray(candidate["reference_crop"])
    candidate_crops = [np.asarray(value) for value in candidate.get("crops", [crop])]
    crop_timestamps = [
        None if value is None else float(value)
        for value in candidate.get("crop_timestamps", [source_ts])
    ]
    reference_crops = [np.asarray(value) for value in candidate.get("reference_crops", [])]
    reference_crop_timestamps = [
        None if value is None else float(value)
        for value in candidate.get("reference_timestamps", [])
    ]
    started = time.perf_counter()
    verdict, matched_id = asyncio.run(
        verify_candidate(
            description,
            crop,
            int(candidate_id),
            chat.chat,
            reference_crop_bgr=reference,
            reference_crops_bgr=reference_crops,
            candidate_crops_bgr=candidate_crops,
        )
    )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    import cv2

    artifact_dir.mkdir(parents=True, exist_ok=True)
    stem = f"attempt-{attempt_index:03d}-id-{candidate_id}"
    candidate_path = artifact_dir / f"{stem}-candidate.jpg"
    reference_path = artifact_dir / f"{stem}-reference.jpg"
    cv2.imwrite(str(candidate_path), crop)
    cv2.imwrite(str(reference_path), reference)
    candidate_paths = []
    for evidence_index, evidence_crop in enumerate(candidate_crops):
        path = artifact_dir / f"{stem}-candidate-{evidence_index:02d}.jpg"
        cv2.imwrite(str(path), evidence_crop)
        candidate_paths.append(path.as_posix())
    extra_reference_paths = []
    for reference_index, reference_crop in enumerate(reference_crops):
        path = artifact_dir / f"{stem}-reference-{reference_index + 1:02d}.jpg"
        cv2.imwrite(str(path), reference_crop)
        extra_reference_paths.append(path.as_posix())
    call = chat.calls[-1] if getattr(chat, "calls", None) else {}
    return {
        "attempt": int(attempt_index),
        "candidate_id": candidate_id,
        "source_ts": source_ts,
        "verdict": verdict,
        "matched_id": matched_id,
        "authorization_queued": False,
        "elapsed_ms": elapsed_ms,
        "model": call.get("model"),
        "model_response": call.get("response"),
        "model_elapsed_ms": (
            float(call["elapsed_s"]) * 1000.0
            if call.get("elapsed_s") is not None
            else None
        ),
        "model_timing_ms": {
            name: float(call[name]) * 1000.0
            for name in (
                "image_decode_s",
                "base64_decode_s",
                "image_pil_decode_s",
                "processor_s",
                "processor_cpu_s",
                "input_transfer_s",
                "generate_s",
                "response_decode_s",
            )
            if call.get(name) is not None
        },
        "model_input_tokens": call.get("input_tokens"),
        "model_output_tokens": call.get("output_tokens"),
        "model_image_count": call.get("image_count"),
        "candidate_crop": candidate_path.as_posix(),
        "reference_crop": reference_path.as_posix(),
        "candidate_crops": candidate_paths,
        "candidate_crop_timestamps": crop_timestamps,
        "reference_crops": [reference_path.as_posix(), *extra_reference_paths],
        "extra_reference_crops": extra_reference_paths,
        "reference_crop_timestamps": reference_crop_timestamps,
    }


def _warm_identity_chat(chat: Any) -> dict[str, Any]:
    """Warm the real four-image identity path without consuming replay evidence."""
    y, x = np.indices((256, 128))

    def crop(offset: int) -> np.ndarray:
        return np.stack(
            (
                (x + offset) % 256,
                (y + 2 * offset) % 256,
                ((x + y) // 2 + 3 * offset) % 256,
            ),
            axis=-1,
        ).astype(np.uint8)

    anchor = crop(11)
    candidates = [crop(value) for value in (37, 73, 109)]
    call_count = len(getattr(chat, "calls", []))
    started = time.perf_counter()
    verdict, _ = asyncio.run(
        verify_candidate(
            "a person wearing a plaid shirt",
            candidates[-1],
            "synthetic",
            chat.chat,
            reference_crop_bgr=anchor,
            candidate_crops_bgr=candidates,
        )
    )
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    calls = getattr(chat, "calls", [])
    call = calls[-1] if len(calls) > call_count else {}
    result = {
        "synthetic_inputs": True,
        "elapsed_ms": elapsed_ms,
        "verdict": verdict,
        "model": call.get("model"),
        "model_response": call.get("response"),
        "model_elapsed_ms": (
            float(call["elapsed_s"]) * 1000.0
            if call.get("elapsed_s") is not None
            else None
        ),
        "image_count": call.get("image_count"),
        "max_new_tokens": call.get("max_new_tokens"),
        "input_tokens": call.get("input_tokens"),
        "output_tokens": call.get("output_tokens"),
        "model_timing_ms": {
            name: float(call[name]) * 1000.0
            for name in (
                "base64_decode_s",
                "image_pil_decode_s",
                "processor_cpu_s",
                "input_transfer_s",
                "generate_s",
                "response_decode_s",
            )
            if call.get(name) is not None
        },
    }
    del calls[call_count:]
    return result


def _identity_key(candidate: dict[str, Any] | None) -> tuple[str, float | None] | None:
    if candidate is None:
        return None
    obj = candidate["object"]
    source_ts = float(obj["ts"]) if obj.get("ts") is not None else None
    return str(obj["id"]), source_ts


def _apply_identity_result(
    tracker: PersonTracker,
    result: dict[str, Any],
    source_frame: int,
    source_ts: float,
) -> dict[str, Any]:
    """Apply a completed verdict before one new frame; distinguish rejection from staleness."""
    current_key = _identity_key(tracker.identity_candidate())
    request_key = (str(result["candidate_id"]), result["source_ts"])
    still_current = current_key == request_key
    resolved = tracker.resolve_identity(
        result["candidate_id"], result["source_ts"], result["verdict"]
    )
    result["resolve_source_frame"] = int(source_frame)
    result["resolve_source_ts"] = float(source_ts)
    result["request_still_current"] = still_current
    result["verdict_applied"] = still_current
    result["authorization_queued"] = bool(resolved)
    result["obsolete_on_arrival"] = not still_current
    return result


def _future_result_at_arrival(future: Future | None) -> Any | None:
    """Poll once at frame arrival; later completion must wait for the next frame."""
    if future is None or not future.done():
        return None
    return future.result()


def _identity_metrics(attempts: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize verifier cost without discarding per-call audit evidence."""
    verdict_counts: dict[str, int] = {}
    for attempt in attempts:
        verdict = str(attempt.get("verdict", "UNKNOWN"))
        verdict_counts[verdict] = verdict_counts.get(verdict, 0) + 1
    wrapper_times = [
        float(attempt["elapsed_ms"])
        for attempt in attempts
        if attempt.get("elapsed_ms") is not None
    ]
    model_times = [
        float(attempt["model_elapsed_ms"])
        for attempt in attempts
        if attempt.get("model_elapsed_ms") is not None
    ]
    timing_names = {
        name
        for attempt in attempts
        for name in attempt.get("model_timing_ms", {})
    }
    timing_totals = {
        name: float(sum(attempt.get("model_timing_ms", {}).get(name, 0.0) for attempt in attempts))
        for name in sorted(timing_names)
    }
    return {
        "attempt_count": len(attempts),
        "verdict_counts": verdict_counts,
        "total_elapsed_ms": float(sum(wrapper_times)),
        "mean_elapsed_ms": float(np.mean(wrapper_times)) if wrapper_times else None,
        "total_model_elapsed_ms": float(sum(model_times)) if model_times else None,
        "mean_model_elapsed_ms": float(np.mean(model_times)) if model_times else None,
        "model_timing_total_ms": timing_totals,
        "mean_model_input_tokens": (
            float(np.mean([attempt["model_input_tokens"] for attempt in attempts]))
            if attempts and all(attempt.get("model_input_tokens") is not None for attempt in attempts)
            else None
        ),
        "mean_model_output_tokens": (
            float(np.mean([attempt["model_output_tokens"] for attempt in attempts]))
            if attempts and all(attempt.get("model_output_tokens") is not None for attempt in attempts)
            else None
        ),
        "mean_model_image_count": (
            float(np.mean([attempt["model_image_count"] for attempt in attempts]))
            if attempts and all(attempt.get("model_image_count") is not None for attempt in attempts)
            else None
        ),
    }


def _iou(left: Iterable[float], right: Iterable[float]) -> float:
    a = np.asarray(list(left), dtype=np.float32)
    b = np.asarray(list(right), dtype=np.float32)
    top_left = np.maximum(a[:2], b[:2])
    bottom_right = np.minimum(a[2:], b[2:])
    intersection = float(np.prod(np.maximum(0.0, bottom_right - top_left)))
    union = float(np.prod(np.maximum(0.0, a[2:] - a[:2]))) + float(
        np.prod(np.maximum(0.0, b[2:] - b[:2]))
    ) - intersection
    return intersection / union if union > 0.0 else 0.0


def detections_from_record(record: dict[str, Any]) -> TrackerInput:
    """Restore the detector rows recorded before NativeByteTracker execution."""
    rows = record.get("detections", record.get("detector_input"))
    if rows is None:
        raise ValueError(f"frame {record.get('frame')} has no recorded detector input")
    detections = [
        Detection2D(
            bbox=np.asarray(row["bbox_xyxy"], dtype=np.float32),
            score=float(row["score"]),
            label="person",
            class_id=int(row.get("class_id", 0)),
        )
        for row in rows
    ]
    result = TrackerInput(detections)
    result._orig_indices = np.asarray(
        [int(row.get("detection_index", index)) for index, row in enumerate(rows)],
        dtype=np.int32,
    )
    cached_tracks = record.get("tracks")
    if cached_tracks is not None:
        result._cached_tracks = np.asarray(
            [
                [
                    *[float(value) for value in row["bbox_xyxy"]],
                    float(row["track_id"]),
                    float(row["score"]),
                    float(row.get("class_id", 0)),
                    float(row.get("detection_index", index)),
                ]
                for index, row in enumerate(cached_tracks)
            ],
            dtype=np.float32,
        ).reshape(-1, 8)
    return result


def _load_detection_records(path: Path) -> list[dict[str, Any]]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(value, dict):
        value = value.get("frames")
    if not isinstance(value, list):
        raise ValueError(f"detector cache must contain a frame list: {path}")
    return value


def _track_rows(tracks: np.ndarray) -> list[dict[str, Any]]:
    return [
        {
            "bbox_xyxy": [float(value) for value in row[:4]],
            "track_id": int(row[4]),
            "score": float(row[5]),
            "class_id": int(row[6]),
            "detection_index": int(row[7]),
        }
        for row in tracks
    ]


def select_initial_index(
    tracks: np.ndarray,
    *,
    target_track_id: int | None,
    target_bbox: list[float] | None,
) -> int:
    """Resolve an externally selected target without using evaluation labels."""
    if target_track_id is not None:
        for index, row in enumerate(tracks):
            if int(row[4]) == target_track_id:
                return index
        return -1
    if target_bbox is None or len(tracks) == 0:
        return -1
    overlaps = [_iou(row[:4], target_bbox) for row in tracks]
    best = int(np.argmax(overlaps))
    return best if overlaps[best] > 0.0 else -1


def _track_object(
    row: np.ndarray,
    timestamp: float,
    *,
    position: list[float] | None,
) -> dict[str, Any]:
    return {
        "id": str(int(row[4])),
        "label": "person",
        "bbox": [float(value) for value in row[:4]],
        "confidence": float(row[5]),
        "position": position if position is not None else [0.0, 0.0, 0.0],
        "ts": float(timestamp),
    }


def _find_current_track(tracks: np.ndarray, tracker_status: dict[str, Any] | None) -> np.ndarray | None:
    if tracker_status is None:
        return None
    target_id = str(tracker_status.get("id", ""))
    for row in tracks:
        if str(int(row[4])) == target_id:
            return row
    return None


def _update_tracker(
    tracker: PersonTracker,
    objects: list[dict[str, Any]],
    frame: np.ndarray,
    *,
    locked: bool,
    just_locked: bool,
) -> bool:
    if just_locked:
        return True
    return tracker.update(objects, frame) if locked else False


def _target_record(
    tracks: np.ndarray,
    status: dict[str, Any] | None,
    *,
    updated: bool,
    geometry_used: bool,
) -> dict[str, Any] | None:
    row = _find_current_track(tracks, status) if updated else None
    if row is None:
        return None
    raw_position = status.get("raw_position") if geometry_used else None
    ema_position = status.get("position") if geometry_used else None
    return {
        "track_id": int(row[4]),
        "bbox_xyxy": [round(float(value), 3) for value in row[:4]],
        "score": round(float(row[5]), 6),
        "position_xyz": [float(value) for value in raw_position] if raw_position is not None else None,
        "raw_position_xyz": [float(value) for value in raw_position] if raw_position is not None else None,
        "ema_position_xyz": [float(value) for value in ema_position] if ema_position is not None else None,
    }


def _draw_state(record: dict[str, Any], title: str) -> tuple[int | None, list[str]]:
    target = record.get("target")
    event = record.get("dino_event", "baseline")
    similarity = record.get("dino_similarity")
    suffix = f" sim={similarity:.3f}" if isinstance(similarity, (int, float)) else ""
    if target is None:
        headline = f"{title} | TARGET LOST/UNCONFIRMED | event={event}{suffix}"
        target_id = None
    else:
        target_id = int(target["track_id"])
        headline = f"{title} | ID {target_id} | event={event}{suffix}"
    frame = int(record.get("frame", -1))
    elapsed = record.get("elapsed_s")
    timing = f"frame={frame} t={float(elapsed):.2f}s" if elapsed is not None else f"frame={frame}"
    if not record.get("geometry_used"):
        lines = [headline, timing, "RGB only"]
    else:
        raw = target.get("raw_position_xyz") if target is not None else None
        ema = target.get("ema_position_xyz") if target is not None else None
        raw_text = ",".join(f"{value:.2f}" for value in raw) if raw is not None else "-"
        ema_text = ",".join(f"{value:.2f}" for value in ema) if ema is not None else "-"
        lines = [headline, timing, f"raw XYZ=({raw_text})", f"EMA XYZ=({ema_text})"]
    return target_id, lines


def _draw(frame: np.ndarray, tracks: np.ndarray, record: dict[str, Any], title: str) -> np.ndarray:
    import cv2

    output = frame.copy()
    selected, lines = _draw_state(record, title)
    for row in tracks:
        track_id = int(row[4])
        x1, y1, x2, y2 = np.rint(row[:4]).astype(int)
        color = (0, 180, 255) if str(track_id) == str(selected) else (70, 190, 70)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        cv2.putText(output, f"ID {track_id}", (x1, max(22, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.rectangle(output, (0, 0), (output.shape[1], min(110, output.shape[0])), (18, 22, 26), -1)
    for line_index, line in enumerate(lines):
        cv2.putText(
            output,
            line,
            (10, 23 + line_index * 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return output


def _positions_for_rgbd(
    tracks: np.ndarray,
    depth: np.ndarray,
    world_from_camera: np.ndarray,
    *,
    depth_method: str = "person",
    intrinsics: CameraIntrinsics = TUM_INTRINSICS,
    depth_scale: float = 1.0 / 5000.0,
) -> tuple[list[list[float] | None], int]:
    positions: list[list[float] | None] = []
    invalid = 0
    for row in tracks:
        if depth_method == "person":
            world = person_depth_position(
                depth,
                np.asarray(row[:4]),
                intrinsics,
                map_from_camera=world_from_camera,
                depth_scale=depth_scale,
                K=intrinsics.K_matrix if intrinsics.has_distortion else None,
                D=intrinsics.D_vector if intrinsics.has_distortion else None,
            )
        elif depth_method == "center":
            depth_m = bbox_center_depth(
                depth,
                np.asarray(row[:4]),
                depth_scale=depth_scale,
            )
            if depth_m is None:
                world = None
            else:
                u = float((row[0] + row[2]) * 0.5)
                v = float((row[1] + row[3]) * 0.5)
                camera = project_to_3d(
                    u,
                    v,
                    depth_m,
                    intrinsics,
                    K=intrinsics.K_matrix if intrinsics.has_distortion else None,
                    D=intrinsics.D_vector if intrinsics.has_distortion else None,
                )
                world = world_from_camera @ np.asarray([*camera, 1.0], dtype=np.float64)
        else:
            raise ValueError(f"unsupported depth method: {depth_method}")
        if world is None:
            positions.append(None)
            invalid += 1
            continue
        positions.append([float(value) for value in world[:3]])
    return positions, invalid


def _position_metrics(records: list[dict[str, Any]], field: str = "raw_position_xyz") -> dict[str, Any]:
    observations = []
    for record in records:
        target = record.get("target")
        if target is None:
            continue
        value = target.get(field)
        if value is None and field == "raw_position_xyz":
            value = target.get("position_xyz")
        if value is not None:
            observations.append(
                (
                    int(record["frame"]),
                    float(record["time_s"]),
                    np.asarray(value, dtype=np.float64),
                )
            )

    adjacent_steps = []
    speeds = []
    gap_count = 0
    for (left_frame, left_time, left), (right_frame, right_time, right) in pairwise(observations):
        distance = float(np.linalg.norm(right - left))
        frame_delta = right_frame - left_frame
        if frame_delta == 1:
            adjacent_steps.append(distance)
        elif frame_delta > 1:
            gap_count += 1
        elapsed = right_time - left_time
        if elapsed > 0:
            speeds.append(distance / elapsed)

    return {
        "valid_positions": len(observations),
        "adjacent_step_samples": len(adjacent_steps),
        "observation_gaps": gap_count,
        "step_median_m": float(np.median(adjacent_steps)) if adjacent_steps else None,
        "step_p95_m": float(np.percentile(adjacent_steps, 95)) if adjacent_steps else None,
        "step_max_m": float(np.max(adjacent_steps)) if adjacent_steps else None,
        "adjacent_jump_count_over_0_5m": int(sum(step > 0.5 for step in adjacent_steps)),
        "speed_samples": len(speeds),
        "speed_p95_mps": float(np.percentile(speeds, 95)) if speeds else None,
        "speed_max_mps": float(np.max(speeds)) if speeds else None,
    }


def _visibility_episodes(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Summarize observed target presence without filling real missing frames."""
    episodes: list[dict[str, Any]] = []
    for offset, record in enumerate(records):
        state = "visible" if record.get("target") is not None else "missing"
        track_id = (
            int(record["target"].get("track_id", record.get("selected_track_id")))
            if record.get("target") is not None
            and record["target"].get("track_id", record.get("selected_track_id")) is not None
            else None
        )
        frame = int(record.get("frame", offset))
        source_frame = int(record.get("source_frame", frame))
        timestamp = float(record.get("time_s", frame))
        if not episodes or episodes[-1]["state"] != state:
            episodes.append(
                {
                    "state": state,
                    "start_frame": frame,
                    "end_frame": frame,
                    "start_source_frame": source_frame,
                    "end_source_frame": source_frame,
                    "start_time_s": timestamp,
                    "end_time_s": timestamp,
                    "frames": 1,
                    "track_ids": [] if track_id is None else [track_id],
                }
            )
            continue
        episode = episodes[-1]
        episode["end_frame"] = frame
        episode["end_source_frame"] = source_frame
        episode["end_time_s"] = timestamp
        episode["frames"] += 1
        if track_id is not None and track_id not in episode["track_ids"]:
            episode["track_ids"].append(track_id)
    return episodes


def _leave_return_events(episodes: list[dict[str, Any]]) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    for index in range(1, len(episodes) - 1):
        missing = episodes[index]
        before = episodes[index - 1]
        after = episodes[index + 1]
        if missing["state"] != "missing" or before["state"] != "visible" or after["state"] != "visible":
            continue
        previous_id = before["track_ids"][-1] if before["track_ids"] else None
        returned_id = after["track_ids"][0] if after["track_ids"] else None
        events.append(
            {
                "left_after_frame": int(before["end_frame"]),
                "missing_start_frame": int(missing["start_frame"]),
                "missing_end_frame": int(missing["end_frame"]),
                "returned_at_frame": int(after["start_frame"]),
                "missing_frames": int(missing["frames"]),
                "previous_track_id": previous_id,
                "returned_track_id": returned_id,
                "track_id_changed": previous_id != returned_id,
            }
        )
    return events


def _variant_metrics(records: list[dict[str, Any]], latencies_ms: list[float]) -> dict[str, Any]:
    visible_ids = [record["selected_track_id"] for record in records if record.get("target") is not None]
    changes = sum(left != right for left, right in pairwise(visible_ids))
    raw_stability = _position_metrics(records, "raw_position_xyz")
    episodes = _visibility_episodes(records)
    encoder_names = {
        (encoder, method)
        for record in records
        for encoder, methods in record.get("encoder_timing", {}).items()
        for method in methods
    }
    encoder_timing = {}
    for encoder, method in sorted(encoder_names):
        rows = [
            record.get("encoder_timing", {}).get(encoder, {}).get(method, {})
            for record in records
        ]
        calls = int(sum(row.get("calls", 0) for row in rows))
        total_ms = float(sum(row.get("total_ms", 0.0) for row in rows))
        encoder_timing.setdefault(encoder, {})[method] = {
            "calls": calls,
            "items": int(sum(row.get("items", 0) for row in rows)),
            "total_ms": total_ms,
            "mean_ms": total_ms / calls if calls else None,
        }
    return {
        "frames": len(records),
        "visible_target_frames": len(visible_ids),
        "missing_target_frames": len(records) - len(visible_ids),
        "selected_track_id_changes": int(changes),
        "mean_update_ms": float(np.mean(latencies_ms)) if latencies_ms else None,
        "p50_update_ms": float(np.percentile(latencies_ms, 50)) if latencies_ms else None,
        "p95_update_ms": float(np.percentile(latencies_ms, 95)) if latencies_ms else None,
        "max_update_ms": float(max(latencies_ms)) if latencies_ms else None,
        "encoder_timing": encoder_timing,
        "position_stability": raw_stability,
        "raw_position_stability": raw_stability,
        "ema_position_stability": _position_metrics(records, "ema_position_xyz"),
        "visibility_episodes": episodes,
        "leave_return_events": _leave_return_events(episodes),
        "real_identity_changes": None,
    }


def _dino_verifier_strata(records: list[dict[str, Any]]) -> dict[str, Any]:
    """Describe DINO frames by verifier-future state without claiming kernel overlap."""
    result = {}
    for running in (False, True):
        rows = []
        for row in records:
            methods = row.get("encoder_timing", {}).get("dino", {})
            calls = sum(
                methods.get(name, {}).get("calls", 0)
                for name in ("encode", "encode_batch")
            )
            if bool(row.get("identity_future_running_at_update")) is running and calls > 0:
                rows.append(row)
        dino_ms = [
            sum(
                float(row["encoder_timing"]["dino"].get(name, {}).get("total_ms", 0.0))
                for name in ("encode", "encode_batch")
            )
            for row in rows
        ]
        result["verifier_running" if running else "verifier_idle"] = {
            "frames_with_dino": len(rows),
            "dino_p50_ms": float(np.percentile(dino_ms, 50)) if dino_ms else None,
            "dino_p95_ms": float(np.percentile(dino_ms, 95)) if dino_ms else None,
            "dino_max_ms": float(max(dino_ms)) if dino_ms else None,
        }
    return result


def _numeric_summary(rows: list[dict[str, Any]], name: str) -> dict[str, float] | None:
    values = [float(row[name]) for row in rows if row.get(name) is not None]
    if not values:
        return None
    return {
        "mean_ms": float(np.mean(values)),
        "p50_ms": float(np.percentile(values, 50)),
        "p95_ms": float(np.percentile(values, 95)),
        "max_ms": float(max(values)),
    }


def _identity_changes(evaluation: dict[str, Any]) -> int:
    identities = [
        row["attributed_gt_id"]
        for row in evaluation["frame_evidence"]
        if row.get("attributed_gt_id") is not None
    ]
    return int(sum(left != right for left, right in pairwise(identities)))


def _is_hf_clip_dir(path: Path) -> bool:
    return path.is_dir() and (path / "config.json").is_file()


def _load_clip(args: argparse.Namespace):
    if _is_hf_clip_dir(args.clip_model_path):
        return _HFClipEncoder(args.clip_model_path, args.device)
    encoder = CLIPEncoder(
        model_name=args.clip_model_name,
        model_path=str(args.clip_model_path),
        device=args.device,
        multi_scale=False,
    )
    encoder.load_model()
    return encoder


def _resolve_yolo_device(device: str) -> str:
    if device != "auto":
        return device
    try:
        import torch

        return "cuda:0" if torch.cuda.is_available() else "cpu"
    except ImportError:
        return "cpu"


def _make_trackers(args: argparse.Namespace, clip: CLIPEncoder, dino: DinoEncoder) -> dict[str, PersonTracker]:
    baseline = PersonTracker()
    baseline.set_clip_encoder(clip)
    with_dino = PersonTracker(
        dino_min_similarity=args.dino_min_similarity,
        dino_margin=args.dino_margin,
    )
    with_dino.set_clip_encoder(clip)
    with_dino.set_dino_encoder(dino)
    if getattr(args, "dino_only", False):
        return {"dino": with_dino}
    return {"baseline": baseline, "dino": with_dino}


def _select_target(
    args: argparse.Namespace,
    frame: np.ndarray,
    tracks: np.ndarray,
    clip: CLIPEncoder,
) -> int:
    index = select_initial_index(
        tracks,
        target_track_id=args.target_track_id,
        target_bbox=args.target_bbox,
    )
    if index >= 0:
        return index
    if not args.target_description:
        return -1
    selector = PersonTracker()
    selector.set_clip_encoder(clip)
    scores = selector._clip_prescore(
        args.target_description,
        crop_tracks(frame, tracks),
    )
    if not scores:
        return -1
    best_index = int(np.argmax(scores))
    return (
        best_index
        if scores[best_index] > PersonTracker._CLIP_SELECT_MIN_SIM
        else -1
    )


def _cached_frames(args: argparse.Namespace):
    import cv2

    records = _load_detection_records(Path(args.tracks))
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise RuntimeError(f"cannot_open_video:{args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS)) or 30.0
    try:
        for index, record in enumerate(records):
            if index >= args.max_frames:
                break
            ok, frame = capture.read()
            if not ok:
                break
            yield index, float(record.get("time_s", index / fps)), frame, None, None, detections_from_record(record), fps
    finally:
        capture.release()


def _live_video_frames(args: argparse.Namespace):
    import cv2

    from perception.backends import DetectorSpec
    from runtime.registry import get

    detector = get("detector", "yolo_world").create(
        DetectorSpec(
            name="yolo_world",
            model_path=str(args.yolo_weights),
            confidence=0.1,
            device=_resolve_yolo_device(args.device),
        )
    )
    detector.load_model()
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        detector.shutdown()
        raise RuntimeError(f"cannot_open_video:{args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS)) or 30.0
    try:
        for index in range(args.max_frames):
            ok, frame = capture.read()
            if not ok:
                break
            detections = detector.detect(frame, "person")
            yield index, index / fps, frame, None, None, TrackerInput(detections), fps
    finally:
        capture.release()
        detector.shutdown()


def _tum_frames(args: argparse.Namespace):
    import cv2
    from PIL import Image
    from tools.datasets.tum_rgbd_probe import _associate, _indexed, _nearest_pose, _poses

    sequence = Path(args.rgbd_sequence)
    capture_frames = sequence / "frames.json"
    expected_size = None
    captured = capture_frames.is_file()
    if captured:
        frame_records = json.loads(capture_frames.read_text(encoding="utf-8"))
        camera = json.loads((sequence / "camera.json").read_text(encoding="utf-8"))
        manifest = json.loads((sequence / "manifest.json").read_text(encoding="utf-8"))
        intrinsics = CameraIntrinsics(
            fx=float(camera["fx"]),
            fy=float(camera["fy"]),
            cx=float(camera["cx"]),
            cy=float(camera["cy"]),
            width=int(camera["width"]),
            height=int(camera["height"]),
            depth_scale=float(camera["depth_scale"]),
            dist_k1=float(camera.get("dist_k1", 0.0)),
            dist_k2=float(camera.get("dist_k2", 0.0)),
            dist_p1=float(camera.get("dist_p1", 0.0)),
            dist_p2=float(camera.get("dist_p2", 0.0)),
            dist_k3=float(camera.get("dist_k3", 0.0)),
            frame_id=str(camera.get("frame_id", "camera_optical")),
        )
        if (
            intrinsics.width <= 0
            or intrinsics.height <= 0
            or intrinsics.fx <= 0
            or intrinsics.fy <= 0
            or not 0.0 < intrinsics.depth_scale <= 0.1
        ):
            raise ValueError("invalid_recorded_rgbd_camera")
        pairs = [
            {
                "source_frame": int(record["frame"]),
                "rgb_timestamp": float(record["source_timestamp_s"]),
                "rgb": sequence / str(record["rgb"]),
                "depth": sequence / str(record["depth"]),
            }
            for record in frame_records
        ]
        if not pairs:
            raise ValueError("empty_recorded_rgbd_capture")
        poses = None
        expected_size = (intrinsics.height, intrinsics.width)
        args._rgbd_intrinsics = intrinsics
        args._rgbd_depth_scale = intrinsics.depth_scale
        args._rgbd_position_transform = CAMERA_LOCAL_FROM_OPTICAL
        args._rgbd_source_format = str(manifest.get("schema", "lingtu.camera.shm_frame.v1"))
        args._rgbd_position_frame = "camera_local_forward_left_up"
        args._rgbd_map_pose_available = False
        args._rgbd_camera_to_body_extrinsics_verified = bool(
            manifest.get("camera_to_body_extrinsics_verified", False)
        )
    else:
        pairs = _associate(
            _indexed(sequence / "rgb.txt"),
            _indexed(sequence / "depth.txt"),
            0.02,
        )
        poses = _poses(sequence / "groundtruth.txt")
        args._rgbd_intrinsics = TUM_INTRINSICS
        args._rgbd_depth_scale = 1.0 / 5000.0
        args._rgbd_position_transform = None
        args._rgbd_source_format = "tum_rgbd_with_groundtruth_pose"
        args._rgbd_position_frame = "dataset_map"
        args._rgbd_map_pose_available = True
        args._rgbd_camera_to_body_extrinsics_verified = None
    cached = _load_detection_records(Path(args.tracks)) if args.tracks else None
    detector = None
    if cached is None:
        from perception.backends import DetectorSpec
        from runtime.registry import get

        detector = get("detector", "yolo_world").create(
            DetectorSpec(
                name="yolo_world",
                model_path=str(args.yolo_weights),
                confidence=0.1,
                device=_resolve_yolo_device(args.device),
            )
        )
        detector.load_model()
        if getattr(args, "pace_source_time", False):
            started = time.perf_counter()
            detector.detect(np.zeros((480, 640, 3), dtype=np.uint8), "person")
            args._detector_warmup_ms = (time.perf_counter() - started) * 1000.0
            detector.reset_statistics()
    try:
        previous_timestamp = None
        deltas = []
        source_rows = cached[: args.max_frames] if cached is not None else None
        iterator = (
            (
                int(row.get("source_frame", row.get("frame", offset))),
                pairs[int(row.get("source_frame", row.get("frame", offset)))],
                row,
            )
            for offset, row in enumerate(source_rows)
        ) if source_rows is not None else (
            (int(pair.get("source_frame", index)), pair, None)
            for index, pair in enumerate(pairs[: args.max_frames])
        )
        for index, pair, cached_row in iterator:
            timestamp = float(pair["rgb_timestamp"])
            pose = (
                (timestamp, args._rgbd_position_transform)
                if captured
                else _nearest_pose(timestamp, poses, 0.05)
            )
            if pose is None:
                continue
            if previous_timestamp is not None:
                deltas.append(timestamp - previous_timestamp)
            previous_timestamp = timestamp
            fps = 1.0 / float(np.median(deltas)) if deltas and np.median(deltas) > 0 else 30.0

            def load_observation(pair=pair, cached_row=cached_row):
                timing: dict[str, float | None] = {
                    "rgb_decode_ms": None,
                    "rgb_file_read_ms": None,
                    "rgb_imdecode_ms": None,
                    "depth_decode_ms": None,
                    "depth_file_read_ms": None,
                    "depth_imdecode_ms": None,
                    "detector_ms": None,
                    "cache_restore_ms": None,
                }
                started = time.perf_counter()
                rgb_bytes = Path(pair["rgb"]).read_bytes()
                rgb_read_complete = time.perf_counter()
                frame = cv2.imdecode(
                    np.frombuffer(rgb_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if frame is None:
                    raise RuntimeError(f"cannot_read_rgb:{pair['rgb']}")
                rgb_complete = time.perf_counter()
                timing["rgb_decode_ms"] = (rgb_complete - started) * 1000.0
                timing["rgb_file_read_ms"] = (
                    rgb_read_complete - started
                ) * 1000.0
                timing["rgb_imdecode_ms"] = (
                    rgb_complete - rgb_read_complete
                ) * 1000.0
                depth_started = rgb_complete
                depth_bytes = Path(pair["depth"]).read_bytes()
                depth_read_complete = time.perf_counter()
                with Image.open(io.BytesIO(depth_bytes)) as depth_image:
                    depth = np.asarray(depth_image).copy()
                if expected_size is not None:
                    if frame.shape[:2] != expected_size:
                        raise ValueError("recorded_rgb_size_mismatch")
                    if depth.shape != expected_size or depth.dtype != np.uint16:
                        raise ValueError("recorded_depth_format_mismatch")
                depth_complete = time.perf_counter()
                timing["depth_decode_ms"] = (
                    depth_complete - depth_started
                ) * 1000.0
                timing["depth_file_read_ms"] = (
                    depth_read_complete - depth_started
                ) * 1000.0
                timing["depth_imdecode_ms"] = (
                    depth_complete - depth_read_complete
                ) * 1000.0
                detector_started = depth_complete
                if cached_row is not None:
                    detector_input = detections_from_record(cached_row)
                    detector_complete = time.perf_counter()
                    timing["cache_restore_ms"] = (
                        detector_complete - detector_started
                    ) * 1000.0
                else:
                    detector_input = TrackerInput(detector.detect(frame, "person"))
                    detector_complete = time.perf_counter()
                    timing["detector_ms"] = (
                        detector_complete - detector_started
                    ) * 1000.0
                timing.update(
                    {
                        "rgb_decode_complete_wall": rgb_complete,
                        "depth_decode_complete_wall": depth_complete,
                        "detector_complete_wall": detector_complete,
                        "observation_input_complete_wall": time.perf_counter(),
                    }
                )
                return frame, depth, detector_input, timing

            yield index, timestamp, load_observation, None, pose[1], None, fps
    finally:
        if detector is not None:
            detector.shutdown()


def run(args: argparse.Namespace) -> dict[str, Any]:
    """Run the A/B replay and write comparable visual and JSON evidence."""
    import cv2

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    pace_source_time = bool(getattr(args, "pace_source_time", False))
    async_identity = bool(getattr(args, "async_identity", False))
    no_render = bool(getattr(args, "no_render", False))
    warmup_timing = {
        "clip_ms": None,
        "dino_ms": None,
        "dino_batch_ms": None,
        "dino_batch_items": 0,
        "detector_ms": None,
        "identity_ms": None,
    }
    identity_warmup = None
    clip_backend = _load_clip(args)
    if pace_source_time:
        started = time.perf_counter()
        clip_backend.encode_text([args.target_description or "person"])
        clip_backend.encode_image(np.zeros((128, 64, 3), dtype=np.uint8))
        warmup_timing["clip_ms"] = (time.perf_counter() - started) * 1000.0
    clip = _TimedEncoder(clip_backend, ("encode_image", "encode_text"))
    dino_backend = DinoEncoder(
        str(args.dino_model),
        device=args.device,
        input_size=args.dino_input_size,
        cpu_threads=args.dino_cpu_threads,
    )
    dino_backend.load_model()
    if pace_source_time:
        # Warm CUDA kernels before establishing the source-time wall clock.
        started = time.perf_counter()
        dino_backend.encode(np.zeros((args.dino_input_size, args.dino_input_size, 3), dtype=np.uint8))
        warmup_timing["dino_ms"] = (time.perf_counter() - started) * 1000.0
        batch = [
            np.full(
                (args.dino_input_size, args.dino_input_size, 3),
                value,
                dtype=np.uint8,
            )
            for value in (0, 64, 128)
        ]
        started = time.perf_counter()
        dino_backend.encode_batch(batch)
        warmup_timing["dino_batch_ms"] = (time.perf_counter() - started) * 1000.0
        warmup_timing["dino_batch_items"] = len(batch)
    dino = _TimedEncoder(dino_backend, ("encode", "encode_batch"))
    identity_chat = None
    if args.identity_model:
        from tools.datasets.qwen_chat import QwenChat

        identity_device = args.device
        if identity_device == "auto":
            try:
                import torch

                identity_device = "cuda" if torch.cuda.is_available() else "cpu"
            except ImportError:
                identity_device = "cpu"
        identity_chat = QwenChat(args.identity_model, device=identity_device)
        if pace_source_time:
            identity_warmup = _warm_identity_chat(identity_chat)
            warmup_timing["identity_ms"] = identity_warmup["elapsed_ms"]
            (output / "identity-warmup.json").write_text(
                json.dumps(identity_warmup, indent=2), encoding="utf-8",
            )
    person_trackers = _make_trackers(args, clip, dino)
    native = NativeByteTracker(frame_rate=30, track_high_thresh=0.25, track_low_thresh=0.1, new_track_thresh=0.25, track_buffer=30)
    records = {name: [] for name in person_trackers}
    latencies = {name: [] for name in person_trackers}
    writer = None
    preview_written = False
    locked = False
    invalid_depth_boxes = 0
    geometry_used = bool(args.rgbd_sequence)
    selection_evidence = None
    detector_cache: list[dict[str, Any]] = []
    identity_attempts: list[dict[str, Any]] = []
    seen_identity_candidates: set[tuple[str, float | None]] = set()
    identity_executor = (
        ThreadPoolExecutor(max_workers=1, thread_name_prefix="identity_verify")
        if async_identity and identity_chat is not None
        else None
    )
    identity_future: Future | None = None
    identity_submit: dict[str, Any] | None = None
    identity_submissions = 0
    paced_frames: list[dict[str, Any]] = []
    pace_wall_start: float | None = None
    previous_finished_wall: float | None = None
    first_timestamp = None
    source = (
        _tum_frames(args)
        if geometry_used
        else _cached_frames(args)
        if args.tracks
        else _live_video_frames(args)
    )

    try:
        for index, timestamp, frame, depth, pose, detector_input, fps in source:
            loop_entry_wall = time.perf_counter()
            interframe_prepare_ms = (
                (loop_entry_wall - previous_finished_wall) * 1000.0
                if previous_finished_wall is not None
                else None
            )
            if first_timestamp is None:
                first_timestamp = float(timestamp)
            elapsed_s = float(timestamp) - first_timestamp
            if pace_source_time:
                if pace_wall_start is None:
                    pace_wall_start = time.perf_counter()
                scheduled_wall = pace_wall_start + elapsed_s
                remaining = scheduled_wall - time.perf_counter()
                idle_sleep_ms = 0.0
                if remaining > 0:
                    sleep_started = time.perf_counter()
                    time.sleep(remaining)
                    idle_sleep_ms = (time.perf_counter() - sleep_started) * 1000.0
                arrival_wall = time.perf_counter()
            else:
                scheduled_wall = None
                arrival_wall = time.perf_counter()
                idle_sleep_ms = 0.0

            completed_attempt: dict[str, Any] | None = None
            arrived_result: Any | None = None
            arrived_error: Exception | None = None
            try:
                arrived_result = _future_result_at_arrival(identity_future)
            except Exception as exc:
                arrived_error = exc
            if arrived_result is not None or arrived_error is not None:
                if arrived_error is None:
                    completed_attempt, completed_wall = arrived_result
                else:
                    completed_wall = time.perf_counter()
                    completed_attempt = {
                        "attempt": identity_submit["attempt"],
                        "candidate_id": identity_submit["candidate_id"],
                        "source_ts": identity_submit["candidate_ts"],
                        "verdict": "UNKNOWN",
                        "matched_id": None,
                        "request_error": (
                            f"{type(arrived_error).__name__}: {arrived_error}"
                        ),
                    }
                completed_attempt["submitted_source_frame"] = identity_submit["source_frame"]
                completed_attempt["submitted_source_ts"] = identity_submit["source_ts"]
                completed_attempt["submitted_wall_s"] = identity_submit["wall"] - pace_wall_start
                completed_attempt["completed_wall_s"] = completed_wall - pace_wall_start
                completed_attempt["wall_latency_ms"] = (
                    completed_wall - identity_submit["wall"]
                ) * 1000.0
                _apply_identity_result(
                    person_trackers["dino"], completed_attempt, index, timestamp
                )
                identity_attempts.append(completed_attempt)
                identity_future = None
                identity_submit = None

            observation_started = time.perf_counter()
            observation_timing: dict[str, float | None] = {}
            if callable(frame):
                frame, depth, detector_input, observation_timing = frame()
            observation_load_ms = (time.perf_counter() - observation_started) * 1000.0
            identity_future_running_at_update = (
                identity_future is not None and not identity_future.done()
            )
            just_locked = False
            target_selection_ms = None
            target_lock_ms = None
            target_selection_encoder_timing = None
            target_lock_encoder_timing = None
            detector_rows = detector_input.to_records()
            native_tracking_started = time.perf_counter()
            cached_tracks = getattr(detector_input, "_cached_tracks", None)
            tracks = (
                np.asarray(cached_tracks, dtype=np.float32).reshape(-1, 8)
                if cached_tracks is not None
                else native.update(detector_input, frame)
            )
            native_tracking_complete = time.perf_counter()
            native_tracking_ms = (
                native_tracking_complete - native_tracking_started
            ) * 1000.0
            detector_cache.append(
                {
                    "frame": len(detector_cache),
                    "source_frame": int(index),
                    "source_pair_index": int(index) if geometry_used else None,
                    "time_s": float(timestamp),
                    "detections": detector_rows,
                    "tracks": _track_rows(tracks),
                }
            )
            position_started = time.perf_counter()
            if depth is not None and pose is not None:
                positions, invalid = _positions_for_rgbd(
                    tracks,
                    depth,
                    pose,
                    depth_method=args.depth_method,
                    intrinsics=getattr(args, "_rgbd_intrinsics", TUM_INTRINSICS),
                    depth_scale=float(
                        getattr(args, "_rgbd_depth_scale", 1.0 / 5000.0)
                    ),
                )
                invalid_depth_boxes += invalid
                valid_rows = [(row, position) for row, position in zip(tracks, positions) if position is not None]
                objects = [_track_object(row, timestamp, position=position) for row, position in valid_rows]
                association_tracks = (
                    np.asarray([row for row, _ in valid_rows], dtype=np.float32)
                    if valid_rows
                    else np.empty((0, 8), dtype=np.float32)
                )
            else:
                objects = [_track_object(row, timestamp, position=None) for row in tracks]
                association_tracks = tracks
            position_complete = time.perf_counter()
            position_ms = (position_complete - position_started) * 1000.0

            if not locked and index >= args.selection_frame:
                clip_before_selection = clip.snapshot()
                dino_before_selection = dino.snapshot()
                selection_started = time.perf_counter()
                selected = _select_target(args, frame, association_tracks, clip)
                target_selection_ms = (
                    time.perf_counter() - selection_started
                ) * 1000.0
                target_selection_encoder_timing = {
                    "clip": clip.delta(clip_before_selection),
                    "dino": dino.delta(dino_before_selection),
                }
                if selected < 0:
                    raise RuntimeError("initial_target_not_found")
                selected_row = association_tracks[selected]
                selected_object = next((obj for obj in objects if obj["id"] == str(int(selected_row[4]))), None)
                if selected_object is None:
                    raise RuntimeError("selected_target_has_no_valid_depth")
                crop = crop_tracks(frame, np.asarray([selected_row]))[0]
                clip_before_lock = clip.snapshot()
                dino_before_lock = dino.snapshot()
                lock_started = time.perf_counter()
                for tracker in person_trackers.values():
                    tracker.lock_target(selected_object, crop)
                    tracker._description = args.target_description or ""
                target_lock_ms = (time.perf_counter() - lock_started) * 1000.0
                target_lock_encoder_timing = {
                    "clip": clip.delta(clip_before_lock),
                    "dino": dino.delta(dino_before_lock),
                }
                selection_evidence = {
                    "frame": len(next(iter(records.values()))),
                    "source_frame": int(index),
                    "time_s": float(timestamp),
                    "track_id": int(selected_row[4]),
                    "bbox_xyxy": [float(value) for value in selected_row[:4]],
                    "target_selection_ms": target_selection_ms,
                    "target_lock_ms": target_lock_ms,
                    "selection_encoder_timing": target_selection_encoder_timing,
                    "lock_encoder_timing": target_lock_encoder_timing,
                }
                locked = True
                just_locked = True

            panels = []
            for name, tracker in person_trackers.items():
                clip_before = clip.snapshot()
                dino_before = dino.snapshot()
                started = time.perf_counter()
                updated = _update_tracker(
                    tracker,
                    objects,
                    frame,
                    locked=locked,
                    just_locked=just_locked,
                )
                update_ms = (time.perf_counter() - started) * 1000.0
                latencies[name].append(update_ms)
                encoder_timing = {
                    "clip": clip.delta(clip_before),
                    "dino": dino.delta(dino_before),
                }
                status = tracker.status()
                target = _target_record(tracks, status, updated=updated, geometry_used=geometry_used)
                identity_verification = None
                if name == "dino" and identity_chat is not None:
                    if identity_executor is None:
                        identity_verification = _verify_pending_identity(
                            tracker,
                            identity_chat,
                            args.target_description or "",
                            output / "identity-attempts",
                            len(identity_attempts),
                            seen_identity_candidates,
                        )
                        if identity_verification is not None:
                            identity_verification["frame"] = len(records[name])
                            identity_verification["source_frame"] = int(index)
                            identity_verification["time_s"] = float(timestamp)
                            identity_attempts.append(identity_verification)
                    elif identity_future is None:
                        candidate = _take_identity_candidate(
                            tracker, seen_identity_candidates
                        )
                        if candidate is not None:
                            submitted_wall = time.perf_counter()
                            attempt_index = identity_submissions
                            identity_submissions += 1

                            def run_request(snapshot=candidate, number=attempt_index):
                                result = _run_identity_request(
                                    snapshot,
                                    identity_chat,
                                    args.target_description or "",
                                    output / "identity-attempts",
                                    number,
                                )
                                return result, time.perf_counter()

                            identity_future = identity_executor.submit(run_request)
                            identity_submit = {
                                "attempt": attempt_index,
                                "candidate_id": str(candidate["object"]["id"]),
                                "candidate_ts": (
                                    float(candidate["object"]["ts"])
                                    if candidate["object"].get("ts") is not None
                                    else None
                                ),
                                "source_frame": int(index),
                                "source_ts": float(timestamp),
                                "wall": submitted_wall,
                            }
                record = {
                    "frame": len(records[name]),
                    "source_frame": int(index),
                    "source_pair_index": int(index) if geometry_used else None,
                    "time_s": float(timestamp),
                    "elapsed_s": elapsed_s,
                    "selected_track_id": int(status["id"]) if status and status.get("id") is not None else None,
                    "target": target,
                    "geometry_used": geometry_used,
                    "position_xyz": target["position_xyz"] if target is not None else None,
                    "dino_similarity": status.get("dino_similarity") if status else None,
                    "dino_event": status.get("dino_event") if status else ("disabled" if name == "baseline" else "ready"),
                    "reid_stats": status.get("reid_stats", {}) if status else {},
                    "identity_retry_until": status.get("identity_retry_until", {}) if status else {},
                    "identity_verification": identity_verification,
                    "identity_completed_before_update": (
                        completed_attempt
                        if name == "dino" and completed_attempt is not None
                        else None
                    ),
                    "tracker_update_ms": update_ms,
                    "identity_future_running_at_update": (
                        identity_future_running_at_update if name == "dino" else False
                    ),
                    "encoder_timing": encoder_timing,
                }
                records[name].append(record)
                if (
                    name == "dino"
                    and completed_attempt is not None
                    and completed_attempt["authorization_queued"]
                    and target is not None
                ):
                    completed_attempt["first_accepted_source_frame"] = int(index)
                    completed_attempt["first_accepted_source_ts"] = float(timestamp)
                if not no_render:
                    panels.append(_draw(frame, tracks, record, "CLIP baseline" if name == "baseline" else "CLIP + DINOv3"))

            tracker_complete = time.perf_counter()

            if not no_render:
                combined = np.hstack(panels)
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(output / "annotated.mp4"),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        float(fps),
                        (combined.shape[1], combined.shape[0]),
                    )
                    if not writer.isOpened():
                        raise RuntimeError("cannot_open_output_video")
                writer.write(combined)
                if locked and not preview_written:
                    cv2.imwrite(str(output / "preview.png"), combined)
                    preview_written = True
            if pace_source_time:
                finished_wall = time.perf_counter()
                paced_frames.append(
                    {
                        "source_frame": int(index),
                        "source_ts": float(timestamp),
                        "source_elapsed_s": elapsed_s,
                        "arrival_wall_s": arrival_wall - pace_wall_start,
                        "schedule_lag_ms": (arrival_wall - scheduled_wall) * 1000.0,
                        "finish_wall_s": finished_wall - pace_wall_start,
                        "finish_lag_ms": (finished_wall - scheduled_wall) * 1000.0,
                        "processing_ms": (finished_wall - arrival_wall) * 1000.0,
                        "interframe_prepare_ms": interframe_prepare_ms,
                        "idle_sleep_ms": idle_sleep_ms,
                        "observation_load_ms": observation_load_ms,
                        "rgb_decode_ms": observation_timing.get("rgb_decode_ms"),
                        "rgb_file_read_ms": observation_timing.get("rgb_file_read_ms"),
                        "rgb_imdecode_ms": observation_timing.get("rgb_imdecode_ms"),
                        "depth_decode_ms": observation_timing.get("depth_decode_ms"),
                        "depth_file_read_ms": observation_timing.get("depth_file_read_ms"),
                        "depth_imdecode_ms": observation_timing.get("depth_imdecode_ms"),
                        "detector_ms": observation_timing.get("detector_ms"),
                        "cache_restore_ms": observation_timing.get("cache_restore_ms"),
                        "native_tracking_ms": native_tracking_ms,
                        "native_tracking_source": (
                            "cached_tracks" if cached_tracks is not None else "native_tracker"
                        ),
                        "position_ms": position_ms,
                        "target_selection_ms": target_selection_ms,
                        "target_lock_ms": target_lock_ms,
                        "target_selection_encoder_timing": (
                            target_selection_encoder_timing
                        ),
                        "target_lock_encoder_timing": target_lock_encoder_timing,
                        "rgb_decode_complete_wall_s": (
                            observation_timing["rgb_decode_complete_wall"] - pace_wall_start
                            if observation_timing.get("rgb_decode_complete_wall") is not None
                            else None
                        ),
                        "rgb_decode_complete_lag_ms": (
                            (observation_timing["rgb_decode_complete_wall"] - scheduled_wall)
                            * 1000.0
                            if observation_timing.get("rgb_decode_complete_wall") is not None
                            else None
                        ),
                        "depth_decode_complete_wall_s": (
                            observation_timing["depth_decode_complete_wall"] - pace_wall_start
                            if observation_timing.get("depth_decode_complete_wall") is not None
                            else None
                        ),
                        "depth_decode_complete_lag_ms": (
                            (observation_timing["depth_decode_complete_wall"] - scheduled_wall)
                            * 1000.0
                            if observation_timing.get("depth_decode_complete_wall") is not None
                            else None
                        ),
                        "observation_input_complete_wall_s": (
                            observation_timing["observation_input_complete_wall"] - pace_wall_start
                            if observation_timing.get("observation_input_complete_wall") is not None
                            else None
                        ),
                        "detector_complete_wall_s": (
                            observation_timing["detector_complete_wall"] - pace_wall_start
                            if observation_timing.get("detector_complete_wall") is not None
                            else None
                        ),
                        "detector_complete_lag_ms": (
                            (observation_timing["detector_complete_wall"] - scheduled_wall)
                            * 1000.0
                            if observation_timing.get("detector_complete_wall") is not None
                            else None
                        ),
                        "observation_input_complete_lag_ms": (
                            (observation_timing["observation_input_complete_wall"] - scheduled_wall)
                            * 1000.0
                            if observation_timing.get("observation_input_complete_wall") is not None
                            else None
                        ),
                        "native_tracking_complete_wall_s": (
                            native_tracking_complete - pace_wall_start
                        ),
                        "native_tracking_complete_lag_ms": (
                            native_tracking_complete - scheduled_wall
                        ) * 1000.0,
                        "position_complete_wall_s": position_complete - pace_wall_start,
                        "position_complete_lag_ms": (
                            position_complete - scheduled_wall
                        ) * 1000.0,
                        "tracker_complete_wall_s": tracker_complete - pace_wall_start,
                        "tracker_complete_lag_ms": (
                            tracker_complete - scheduled_wall
                        ) * 1000.0,
                        "identity_in_flight": identity_future is not None,
                    }
                )
                previous_finished_wall = finished_wall
    finally:
        if identity_future is not None:
            try:
                tail_attempt, completed_wall = identity_future.result()
            except Exception as exc:
                completed_wall = time.perf_counter()
                tail_attempt = {
                    "attempt": identity_submit["attempt"],
                    "candidate_id": identity_submit["candidate_id"],
                    "source_ts": identity_submit["candidate_ts"],
                    "verdict": "UNKNOWN",
                    "matched_id": None,
                    "request_error": f"{type(exc).__name__}: {exc}",
                }
            tail_attempt["submitted_source_frame"] = identity_submit["source_frame"]
            tail_attempt["submitted_source_ts"] = identity_submit["source_ts"]
            tail_attempt["submitted_wall_s"] = identity_submit["wall"] - pace_wall_start
            tail_attempt["completed_wall_s"] = completed_wall - pace_wall_start
            tail_attempt["wall_latency_ms"] = (
                completed_wall - identity_submit["wall"]
            ) * 1000.0
            tail_attempt["authorization_queued"] = False
            tail_attempt["obsolete_on_arrival"] = True
            tail_attempt["tail_result_not_authorized"] = True
            identity_attempts.append(tail_attempt)
        if identity_executor is not None:
            identity_executor.shutdown(wait=True, cancel_futures=False)
        if writer is not None:
            writer.release()
        clip.shutdown()

    if not locked:
        raise RuntimeError("selection_frame_not_reached")
    try:
        import torch

        torch_intraop_threads = int(torch.get_num_threads())
        torch_interop_threads = int(torch.get_num_interop_threads())
    except ImportError:
        torch_intraop_threads = None
        torch_interop_threads = None
    warmup_timing["detector_ms"] = getattr(args, "_detector_warmup_ms", None)
    for name, rows in records.items():
        _write_json(output / f"tracks_{name}.json", rows)
    _write_json(output / "detector_cache.json", detector_cache)
    metrics: dict[str, Any] = {
        "schema_version": "lingtu.dino_follow.v1",
        "source": str(args.rgbd_sequence or args.video),
        "detector_input": str(args.tracks) if args.tracks else "live_yolo_world",
        "same_frames_and_detector_input_for_ab": len(records) > 1,
        "fixed_cached_detector_input": bool(args.tracks),
        "selection_used_ground_truth": False,
        "selection_source": (
            "cached_native_track_id"
            if args.target_track_id is not None
            else "external_reference_bbox"
            if args.target_bbox is not None
            else "clip_text_image_similarity"
        ),
        "selection": selection_evidence,
        "geometry_used": geometry_used,
        "source_frame_semantics": (
            "synchronized_rgb_depth_pair_index"
            if geometry_used
            else "video_frame_index"
        ),
        "rgbd_source_format": (
            getattr(args, "_rgbd_source_format", None) if geometry_used else None
        ),
        "rgb_position_representation": (
            getattr(args, "_rgbd_position_frame", "dataset_map")
            if geometry_used
            else "all xyz values are null"
        ),
        "map_pose_available": (
            getattr(args, "_rgbd_map_pose_available", None) if geometry_used else None
        ),
        "camera_to_body_extrinsics_verified": (
            getattr(args, "_rgbd_camera_to_body_extrinsics_verified", None)
            if geometry_used
            else None
        ),
        "invalid_depth_boxes": invalid_depth_boxes if geometry_used else None,
        "depth_method": args.depth_method if geometry_used else None,
        "dino_backend": dino.backend,
        "dino_model": str(args.dino_model),
        "identity_verification": {
            "enabled": identity_chat is not None,
            "model": str(args.identity_model) if args.identity_model else None,
            "attempts": identity_attempts,
            **_identity_metrics(identity_attempts),
        },
        "paced_replay": {
            "enabled": pace_source_time,
            "async_identity": async_identity,
            "frames": paced_frames,
            "single_physical_identity_job": async_identity,
            "summary": {
                "processed_frames": len(paced_frames),
                "unique_source_frames": len(
                    {row["source_frame"] for row in paced_frames}
                ),
                "source_frames_strictly_in_order": all(
                    left["source_frame"] < right["source_frame"]
                    for left, right in pairwise(paced_frames)
                ),
                "source_duration_s": (
                    paced_frames[-1]["source_elapsed_s"] if paced_frames else None
                ),
                "first_source_frame": (
                    paced_frames[0]["source_frame"] if paced_frames else None
                ),
                "last_source_frame": (
                    paced_frames[-1]["source_frame"] if paced_frames else None
                ),
                "source_cadence_ms": (
                    float(
                        np.median(
                            [
                                (right["source_ts"] - left["source_ts"]) * 1000.0
                                for left, right in pairwise(paced_frames)
                            ]
                        )
                    )
                    if len(paced_frames) > 1
                    else None
                ),
                "source_cadence_p95_ms": (
                    float(
                        np.percentile(
                            [
                                (right["source_ts"] - left["source_ts"]) * 1000.0
                                for left, right in pairwise(paced_frames)
                            ],
                            95,
                        )
                    )
                    if len(paced_frames) > 1
                    else None
                ),
                "source_cadence_max_ms": (
                    float(
                        max(
                            (right["source_ts"] - left["source_ts"]) * 1000.0
                            for left, right in pairwise(paced_frames)
                        )
                    )
                    if len(paced_frames) > 1
                    else None
                ),
                "wall_to_last_frame_s": (
                    paced_frames[-1]["finish_wall_s"] if paced_frames else None
                ),
                "identity_in_flight_frames": sum(
                    bool(row["identity_in_flight"]) for row in paced_frames
                ),
                "pending_tail_recorded": any(
                    bool(row.get("tail_result_not_authorized"))
                    for row in identity_attempts
                ),
                "final_schedule_lag_ms": (
                    paced_frames[-1]["schedule_lag_ms"] if paced_frames else None
                ),
                "final_finish_lag_ms": (
                    paced_frames[-1]["finish_lag_ms"] if paced_frames else None
                ),
                **{
                    name: (
                        {
                            "mean_ms": float(np.mean([row[name] for row in paced_frames])),
                            "p50_ms": float(np.percentile([row[name] for row in paced_frames], 50)),
                            "p95_ms": float(np.percentile([row[name] for row in paced_frames], 95)),
                            "max_ms": float(max(row[name] for row in paced_frames)),
                        }
                        if paced_frames
                        else None
                    )
                    for name in (
                        "schedule_lag_ms",
                        "finish_lag_ms",
                        "processing_ms",
                        "observation_load_ms",
                    )
                },
                "stage_timing_ms": {
                    name: _numeric_summary(paced_frames, name)
                    for name in (
                        "interframe_prepare_ms",
                        "idle_sleep_ms",
                        "rgb_decode_ms",
                        "rgb_file_read_ms",
                        "rgb_imdecode_ms",
                        "depth_decode_ms",
                        "depth_file_read_ms",
                        "depth_imdecode_ms",
                        "detector_ms",
                        "cache_restore_ms",
                        "native_tracking_ms",
                        "position_ms",
                        "target_selection_ms",
                        "target_lock_ms",
                    )
                },
                "stage_completion_lag_ms": {
                    name: _numeric_summary(paced_frames, name)
                    for name in (
                        "rgb_decode_complete_lag_ms",
                        "depth_decode_complete_lag_ms",
                        "detector_complete_lag_ms",
                        "observation_input_complete_lag_ms",
                        "native_tracking_complete_lag_ms",
                        "position_complete_lag_ms",
                        "tracker_complete_lag_ms",
                    )
                },
            },
        },
        "warmup_timing_ms": warmup_timing,
        "identity_warmup": identity_warmup,
        "inference_timing": {
            "clip": clip.timing(),
            "dino": dino.timing(),
        },
        "parameters": {
            "dino_min_similarity": float(args.dino_min_similarity),
            "dino_margin": float(args.dino_margin),
            "input_size": int(dino.input_size),
            "dino_cpu_threads": int(args.dino_cpu_threads),
            "torch_intraop_threads": torch_intraop_threads,
            "torch_interop_threads": torch_interop_threads,
            "clip_model_path": str(args.clip_model_path),
            "clip_model_name": str(args.clip_model_name),
            "requested_device": str(args.device),
            "clip_device_actual": str(getattr(clip, "_device", "unknown")),
            "yolo_device_resolved": (
                _resolve_yolo_device(args.device)
                if not args.tracks
                else None
            ),
            "selection": {
                "selection_frame": int(args.selection_frame),
                "target_track_id": args.target_track_id,
                "target_bbox": (
                    [float(value) for value in args.target_bbox]
                    if args.target_bbox is not None
                    else None
                ),
                "target_description": args.target_description,
            },
        },
    }
    for name in records:
        metrics[name] = _variant_metrics(records[name], latencies[name])
        if name == "dino":
            metrics[name]["dino_by_verifier_future_state"] = _dino_verifier_strata(
                records[name]
            )
    if args.gt:
        from tools.datasets.person_eval import evaluate

        annotations = np.loadtxt(args.gt, delimiter=",", ndmin=2)
        for name in records:
            evaluation = evaluate(records[name], annotations)
            metrics[name]["mot_attribution"] = evaluation
            metrics[name]["real_identity_changes"] = _identity_changes(evaluation)
    _write_json(output / "metrics.json", metrics)
    return metrics


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--video", type=Path)
    source.add_argument("--rgbd-sequence", type=Path)
    parser.add_argument("--tracks", type=Path, help="Optional person_follow detection cache for --video")
    parser.add_argument("--yolo-weights", type=Path, help="YOLO-World weights for live video or RGB-D")
    parser.add_argument("--clip-model-path", type=Path, required=True)
    parser.add_argument("--clip-model-name", default="ViT-B/32")
    parser.add_argument("--dino-model", required=True, help="Local ONNX file or local/Hugging Face model path")
    parser.add_argument(
        "--identity-model",
        type=Path,
        help="Optional local Qwen3-VL model used only for DINO reacquisition proposals",
    )
    selector = parser.add_mutually_exclusive_group(required=True)
    selector.add_argument("--target-track-id", type=int)
    selector.add_argument("--target-bbox", type=float, nargs=4, metavar=("X1", "Y1", "X2", "Y2"))
    selector.add_argument("--target-description")
    parser.add_argument("--selection-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=300)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--dino-input-size", type=int, default=336)
    parser.add_argument("--dino-cpu-threads", type=int, default=4)
    parser.add_argument("--dino-min-similarity", type=float, default=0.65)
    parser.add_argument("--dino-margin", type=float, default=0.03)
    parser.add_argument(
        "--dino-only",
        action="store_true",
        help="Run only the product DINO tracker; retain CLIP for initial/text selection",
    )
    parser.add_argument(
        "--pace-source-time",
        action="store_true",
        help="Schedule frames by recorded source timestamps after model warmup",
    )
    parser.add_argument(
        "--async-identity",
        action="store_true",
        help="Run at most one physical identity request while source-paced frames continue",
    )
    parser.add_argument(
        "--no-render",
        action="store_true",
        help="Exclude drawing and video encoding from the paced critical path",
    )
    parser.add_argument(
        "--depth-method",
        choices=("person", "center"),
        default="person",
        help="RGB-D person position sampler; center preserves the previous baseline",
    )
    parser.add_argument("--gt", type=Path, help="Optional MOT gt.txt used only after tracking")
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.video and not args.tracks and not args.yolo_weights:
        parser.error("--video requires either --tracks or --yolo-weights")
    if args.rgbd_sequence and not args.tracks and not args.yolo_weights:
        parser.error("--rgbd-sequence requires either --tracks or --yolo-weights")
    if args.max_frames <= 0 or args.selection_frame < 0:
        parser.error("frame limits must be non-negative and max-frames positive")
    if args.dino_cpu_threads <= 0:
        parser.error("--dino-cpu-threads must be positive")
    if args.async_identity and not args.pace_source_time:
        parser.error("--async-identity requires --pace-source-time")
    metrics = run(args)
    summary = {"output": str(args.output), "dino_backend": metrics["dino_backend"]}
    for variant in ("baseline", "dino"):
        if variant not in metrics:
            continue
        summary[variant] = {
            key: value for key, value in metrics[variant].items() if key != "mot_attribution"
        }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
