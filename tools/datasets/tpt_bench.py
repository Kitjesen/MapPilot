#!/usr/bin/env python3
"""Score and replay LingTu target-person tracking in the TPT-Bench layout.

TPT-Bench (Ye et al., arXiv 2505.07446; data CC BY 4.0) stores one ground-truth
file per sequence, ``GTs/<seq>.json``, keyed by frame timestamp with
``{"is_exist": bool, "bbox": [x, y, w, h]}``. A method writes
``<results>/<seq>/<method>.json`` with ``{"target_info": [x, y, w, h, conf]}``
per frame, ``[0, 0, 0, 0, -1]`` meaning "target not reported".

``score`` reproduces the long-term metrics of the official
``tpt_bench_tools/evaluate_baselines.py`` at MedlarTea/TPT-BENCH-TOOLS
2488f8b: AO, best F-score, AP and AMR, averaged over sequences. The reported
precision/recall belong to the best F-score threshold; the official script's
unprinted precision/recall columns are overwritten by its last AMR sweep and
are not reproduced. ``replay`` runs the production PersonTracker and
NativeByteTracker on recorded detections. The ground truth is used only to
choose the initial target, as in the benchmark's single-target protocol.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from itertools import pairwise
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
for _path in (ROOT, ROOT / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import numpy as np
from tools.datasets.dino_follow import select_initial_index
from tools.datasets.person_follow import TrackerInput

import decision.vision.person as person_module
from decision.vision.person import PersonTracker
from perception.detection.detector_base import Detection2D
from perception.tracking.native_byte_tracker import NativeByteTracker

ABSENT = [0, 0, 0, 0, -1]
AMR_IOU_THRESHOLDS = (0.01, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.99)
# Mirrors the tracking Product detector (`yolo_trt`, confidence 0.3).
DETECTOR_CONFIDENCE = 0.3
PERSON_HEIGHT_M = 1.7


def _tpt_iou(box: list[float], other: list[float]) -> float:
    """Official TPT-Bench IoU on xyxy boxes, including its +1 pixel convention."""
    x1, y1, x2, y2 = box
    u1, v1, u2, v2 = other
    width = max(0.0, min(x2, u2) - max(x1, u1) + 1)
    height = max(0.0, min(y2, v2) - max(y1, v1) + 1)
    intersection = width * height
    union = (x2 - x1 + 1) * (y2 - y1 + 1) + (u2 - u1 + 1) * (v2 - v1 + 1) - intersection
    return intersection / float(union)


def _thresholds(scores: np.ndarray, resolution: int) -> list[float]:
    ordered = sorted((float(value) for value in scores if not math.isnan(value)), reverse=True)
    if len(ordered) > resolution - 2:
        delta = math.floor(len(ordered) / (resolution - 2))
        indices = np.round(np.linspace(delta, len(ordered) - delta, num=resolution - 2)).astype(int)
        ordered = [ordered[index] for index in indices]
    return [math.inf, *ordered, -math.inf]


def _precision_recall(rows: np.ndarray, values: np.ndarray, threshold: float, visible: int) -> tuple[float, float]:
    subset = rows[:, 1] >= threshold
    if not subset.any():
        return 1.0, 0.0
    return float(np.mean(values[subset])), float(np.sum(values[subset]) / visible)


def score_sequence(ground_truth: dict[str, dict], prediction: dict[str, dict]) -> dict[str, float]:
    """Return the official per-sequence long-term tracking metrics."""
    if set(prediction) != set(ground_truth):
        missing = len(set(ground_truth) - set(prediction))
        extra = len(set(prediction) - set(ground_truth))
        raise ValueError(f"prediction frames differ from ground truth: {missing} missing, {extra} extra")
    rows = []
    for frame_id in prediction:
        exists = bool(ground_truth[frame_id]["is_exist"])
        x, y, w, h = ground_truth[frame_id]["bbox"]
        truth = [x, y, x + w, y + h]
        px, py, pw, ph, confidence = prediction[frame_id]["target_info"]
        box = [px, py, px + pw, py + ph]
        reported = sum(box) != 0
        if not reported:
            tracks = prediction[frame_id].get("tracks_target_conf_bbox", [])
            best = max(tracks, key=lambda track: track[5], default=None)
            confidence = best[5] if best is not None and best[5] > 0 else 0
            box = None if best is None or best[5] <= 0 else [best[1], best[2], best[1] + best[3], best[2] + best[4]]
        elif confidence == -1 and exists:
            confidence = 1.0  # official: initialization frames of MOT+ReID methods
        iou = _tpt_iou(truth, box) if exists and box is not None else 0.0
        rows.append([iou, confidence, exists, reported])
    result = np.asarray(rows, dtype=np.float64)
    visible = int(np.sum(result[:, 2] == 1))
    if visible == 0:
        raise ValueError("sequence has no frame with a visible target")
    thresholds = _thresholds(result[:, 1], 100)

    precision, recall, f_score, ap = [], [], [], 0.0
    for index, threshold in enumerate(thresholds):
        p, r = _precision_recall(result, result[:, 0], threshold, visible)
        precision.append(p)
        recall.append(r)
        f_score.append(2 * p * r / (p + r + 1e-6))
        if index > 1:  # official accumulation starts at the third threshold
            ap += (recall[index] - recall[index - 1]) * p

    max_recalls = []
    for iou_threshold in AMR_IOU_THRESHOLDS:
        hits = (result[:, 0] >= iou_threshold).astype(np.float64)
        pairs = [_precision_recall(result, hits, threshold, visible) for threshold in thresholds]
        perfect = [r for p, r in pairs if p == 1.0]
        max_recalls.append(max(perfect) if perfect else -1.0)  # official keeps -1 in the mean

    best = int(np.argmax(f_score))
    return {
        "ao": float(np.mean(result[result[:, 2] == 1][:, 0])),
        "f_score": float(f_score[best]),
        "precision": float(precision[best]),
        "recall": float(recall[best]),
        "ap": float(ap),
        "amr": float(np.mean(max_recalls)),
        "frames": int(len(result)),
        "visible_frames": visible,
    }


def score_results(
    ground_truth_dir: Path, results_dir: Path, methods: list[str], sequences: list[str],
) -> dict[str, Any]:
    """Score ``<results>/<seq>/<method>.json`` files and average like the official script."""
    per_sequence: dict[str, dict[str, dict[str, float]]] = {}
    for sequence in sequences:
        truth = json.loads((ground_truth_dir / f"{sequence}.json").read_text(encoding="utf-8"))
        per_sequence[sequence] = {
            method: score_sequence(
                truth, json.loads((results_dir / sequence / f"{method}.json").read_text(encoding="utf-8")),
            )
            for method in methods
        }
    keys = ("ao", "f_score", "precision", "recall", "ap", "amr")
    mean = {
        method: {key: float(np.mean([per_sequence[s][method][key] for s in sequences])) for key in keys}
        for method in methods
    }
    return {"mean": mean, "sequences": per_sequence}


@contextmanager
def source_clock() -> Iterator[Callable[[float], None]]:
    """Drive PersonTracker's wall-clock timeouts from source timestamps during replay."""
    current = [0.0]

    class _Clock:
        @staticmethod
        def time() -> float:
            return current[0]

    original = person_module.time
    person_module.time = _Clock
    try:
        yield lambda value: current.__setitem__(0, float(value))
    finally:
        person_module.time = original


def panoramic_position(box: list[float], width: int, height: int) -> list[float]:
    """Approximate a bearing/range from an equirectangular box when no depth is recorded."""
    azimuth = (0.5 - 0.5 * (box[0] + box[2]) / width) * 2.0 * math.pi
    angle = max(1e-3, (box[3] - box[1]) / height * math.pi)
    distance = PERSON_HEIGHT_M / (2.0 * math.tan(min(angle, 3.0) / 2.0))
    return [distance * math.cos(azimuth), distance * math.sin(azimuth), 0.0]


def load_detections(path: Path) -> dict[str, dict[str, Any]]:
    """Read ``{"ts": key, "boxes": [[x1,y1,x2,y2,score]], "positions"?: [[x,y,z]]}`` lines."""
    frames = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            record = json.loads(line)
            frames[str(record["ts"])] = record
    return frames


def _read_image(path: Path) -> np.ndarray:
    import cv2

    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(path)
    return image


Verifier = Callable[[dict[str, Any], str], str]


def oracle_verifier(ground_truth: dict[str, dict]) -> Verifier:
    """Answer identity checks from the label: an upper bound for a perfect VLM verifier."""
    def verify(candidate: dict[str, Any], key: str) -> str:
        truth = ground_truth[key]
        if not truth["is_exist"]:
            return "OTHER"
        x, y, w, h = truth["bbox"]
        return "MATCH" if _tpt_iou(candidate["bbox"], [x, y, x + w, y + h]) >= 0.5 else "OTHER"
    return verify


def replay_sequence(
    image_dir: Path,
    ground_truth: dict[str, dict],
    detections: dict[str, dict[str, Any]],
    *,
    encoder: Any | None,
    verifier: Verifier | None = None,
    verifier_latency_s: float = 0.0,
    seconds_per_tick: float = 1e-9,
) -> tuple[dict[str, dict], dict[str, Any]]:
    """Replay one sequence and return TPT predictions plus replay diagnostics.

    With an appearance encoder, PersonTracker accepts a changed track only after
    ``resolve_identity`` (VisualServo asks the VLM). ``verifier`` stands in for
    that single in-flight request; its answer is applied ``verifier_latency_s``
    of source time later.
    """
    byte_tracker = NativeByteTracker(
        track_high_thresh=DETECTOR_CONFIDENCE,
        track_low_thresh=min(0.1, DETECTOR_CONFIDENCE),
        new_track_thresh=DETECTOR_CONFIDENCE,
    )
    tracker = PersonTracker()
    if encoder is not None:
        tracker.set_dino_encoder(encoder)
    predictions: dict[str, dict] = {}
    selected_ids: list[str | None] = []
    heuristic_positions = 0
    locked = False
    keys_by_time: dict[float, str] = {}
    request: tuple[float, str, float, str] | None = None  # due time, candidate id, source ts, verdict
    verdicts = {"MATCH": 0, "OTHER": 0}
    with source_clock() as set_time:
        for key in sorted(ground_truth, key=int):
            timestamp = int(key) * seconds_per_tick
            keys_by_time[timestamp] = key
            set_time(timestamp)
            if request is not None and timestamp >= request[0]:
                tracker.resolve_identity(request[1], request[2], request[3])
                request = None
            frame = _read_image(next(image_dir.glob(f"{key}.*")))
            record = detections.get(key, {"boxes": []})
            boxes = record.get("boxes", [])
            boxes = [box for box in boxes if box[4] >= byte_tracker.track_low_thresh]
            positions = record.get("positions")
            tracks = byte_tracker.update(TrackerInput([
                Detection2D(bbox=np.asarray(box[:4], dtype=np.float32), score=float(box[4]), label="person")
                for box in boxes
            ]))
            objects = []
            for row in tracks:
                box = [float(value) for value in row[:4]]
                index = int(row[7])
                position = positions[index] if positions is not None and positions[index] is not None else None
                if position is None:
                    heuristic_positions += 1
                    position = panoramic_position(box, frame.shape[1], frame.shape[0])
                objects.append({
                    "id": str(int(row[4])), "label": "person", "bbox": box,
                    "confidence": float(row[5]), "position": list(position), "ts": timestamp,
                })

            target = None
            if not locked and ground_truth[key]["is_exist"]:
                x, y, w, h = ground_truth[key]["bbox"]
                index = select_initial_index(tracks, target_track_id=None, target_bbox=[x, y, x + w, y + h])
                if index >= 0:
                    tracker.lock_target(objects[index], tracker._crop_person(frame, objects[index]))
                    locked = True
                    target = (objects[index], 1.0)
            elif locked:
                prepared = tracker.prepare_observation(objects, frame)
                if tracker.update(prepared, frame):
                    status = tracker.status() or {}
                    match = next((obj for obj in objects if obj["id"] == str(status.get("id"))), None)
                    if match is not None:
                        similarity = status.get("dino_similarity")
                        target = (match, float(similarity) if similarity is not None else match["confidence"])
                elif verifier is not None and request is None:
                    candidate = tracker.identity_candidate()
                    if candidate is not None:
                        obj = candidate["object"]
                        verdict = verifier(obj, keys_by_time[float(obj["ts"])])
                        verdicts[verdict] += 1
                        request = (timestamp + verifier_latency_s, str(obj["id"]), float(obj["ts"]), verdict)
            if target is None:
                predictions[key] = {"target_info": list(ABSENT), "tracks_target_conf_bbox": []}
                selected_ids.append(None)
            else:
                (x1, y1, x2, y2), confidence = target[0]["bbox"], target[1]
                predictions[key] = {"target_info": [x1, y1, x2 - x1, y2 - y1, confidence]}
                selected_ids.append(target[0]["id"])
    reported = [value for value in selected_ids if value is not None]
    diagnostics = {
        "initialized": locked,
        "reported_frames": len(reported),
        "selected_track_changes": sum(1 for a, b in pairwise(reported) if a != b),
        "heuristic_position_objects": heuristic_positions,
        "verifier_verdicts": verdicts,
        "selected_track_ids": selected_ids,
    }
    return predictions, diagnostics


def _encoder(name: str, args: argparse.Namespace) -> Any | None:
    if name == "none":
        return None
    if name == "dino":
        from perception.encoding.dino import DinoEncoder

        encoder = DinoEncoder(str(args.dino_model), device=args.device)
        encoder.load_model()
        return encoder
    raise ValueError(f"unknown encoder {name}")


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=1), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    score = commands.add_parser("score", help="score result files with the official TPT-Bench metrics")
    score.add_argument("--dataset-dir", type=Path, required=True)
    score.add_argument("--results-dir", type=Path, help="defaults to <dataset-dir>/evaluation_results")
    score.add_argument("--methods", nargs="+", required=True)
    score.add_argument("--sequences", nargs="+", required=True)
    score.add_argument("--output", type=Path)

    replay = commands.add_parser("replay", help="replay PersonTracker on recorded detections")
    replay.add_argument("--dataset-dir", type=Path, required=True)
    replay.add_argument("--sequence", required=True)
    replay.add_argument("--detections", type=Path, required=True, help="JSONL detector output for the sequence")
    replay.add_argument("--method", required=True, help="result file name, e.g. lingtu-dino")
    replay.add_argument("--results-dir", type=Path, required=True)
    replay.add_argument("--encoder", choices=("none", "dino"), default="none")
    replay.add_argument("--dino-model", type=Path)
    replay.add_argument("--device", default="cpu")
    replay.add_argument("--verifier", choices=("none", "oracle"), default="none",
                        help="oracle answers identity checks from the label (perfect-VLM upper bound)")
    replay.add_argument("--verifier-latency-s", type=float, default=0.0)
    args = parser.parse_args()

    if args.command == "score":
        result = score_results(
            args.dataset_dir / "GTs", args.results_dir or args.dataset_dir / "evaluation_results",
            args.methods, args.sequences,
        )
        if args.output:
            _write_json(args.output, result)
        print(json.dumps(result["mean"], indent=1))
        return 0

    truth = json.loads((args.dataset_dir / "GTs" / f"{args.sequence}.json").read_text(encoding="utf-8"))
    predictions, diagnostics = replay_sequence(
        args.dataset_dir / "panoramic_images" / args.sequence, truth,
        load_detections(args.detections), encoder=_encoder(args.encoder, args),
        verifier=oracle_verifier(truth) if args.verifier == "oracle" else None,
        verifier_latency_s=args.verifier_latency_s,
    )
    _write_json(args.results_dir / args.sequence / f"{args.method}.json", predictions)
    _write_json(args.results_dir / args.sequence / f"{args.method}.diagnostics.json", diagnostics)
    print(json.dumps(score_sequence(truth, predictions), indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
