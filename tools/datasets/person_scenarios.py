#!/usr/bin/env python3
"""Deterministic target-person scenarios written in the TPT-Bench layout.

Each scenario renders flat-coloured people in front of a grey, world-fixed
background texture seen by a pinhole camera, detector boxes with noise,
map-frame positions and per-person annotations. The replay
uses the production NativeByteTracker and PersonTracker. Appearance comes from
``MarkerEncoder``: every clothing colour maps to a fixed unit vector with small
per-crop noise, so "similar clothes" is an explicit cosine similarity.

This is a logic benchmark for association, identity and timing behaviour. It
does not measure detector, DINO or ReID model accuracy, and its absolute
scores are not comparable with TPT-Bench results.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from tools.datasets.tpt_bench import load_detections, oracle_verifier, replay_sequence, score_sequence

WIDTH, HEIGHT = 640, 360
FOCAL, CX, CY = 320.0, 320.0, 180.0
CAMERA_HEIGHT_M, PERSON_HEIGHT_M, PERSON_WIDTH_M = 0.6, 1.7, 0.5
EMBEDDING_DIM = 64
# Clothing colours (BGR) and the cosine similarity of their base embeddings to the target.
COLOURS = {
    "target": ((40, 40, 200), 1.0),
    "similar": ((60, 40, 190), 0.9),
    "other_a": ((200, 120, 40), 0.2),
    "other_b": ((40, 180, 60), 0.1),
}


def _base_embeddings() -> dict[tuple[int, int, int], np.ndarray]:
    rng = np.random.default_rng(7)
    target = rng.standard_normal(EMBEDDING_DIM)
    target /= np.linalg.norm(target)
    vectors = {}
    for colour, similarity in COLOURS.values():
        orthogonal = rng.standard_normal(EMBEDDING_DIM)
        orthogonal -= orthogonal.dot(target) * target
        orthogonal /= np.linalg.norm(orthogonal)
        vectors[colour] = similarity * target + math.sqrt(max(0.0, 1.0 - similarity**2)) * orthogonal
    return vectors


class MarkerEncoder:
    """Map a crop's dominant clothing colour to its embedding plus deterministic noise."""

    def __init__(self, noise: float = 0.15) -> None:
        self._vectors = _base_embeddings()
        self._colours = np.asarray(list(self._vectors), dtype=np.float64)
        self._noise = noise

    def encode(self, crop: np.ndarray) -> np.ndarray:
        pixels = np.asarray(crop, dtype=np.float64).reshape(-1, 3)
        pixels = pixels[np.ptp(pixels, axis=1) > 40]  # ignore the grey background
        if len(pixels) == 0:
            return np.zeros(EMBEDDING_DIM, dtype=np.float32)
        nearest = np.argmin(((pixels[:, None, :] - self._colours[None]) ** 2).sum(axis=2), axis=1)
        dominant = tuple(int(v) for v in self._colours[np.bincount(nearest).argmax()])
        seed = int.from_bytes(hashlib.sha1(np.ascontiguousarray(crop).tobytes()).digest()[:8], "little")
        noise = np.random.default_rng(seed).standard_normal(EMBEDDING_DIM) * self._noise / math.sqrt(EMBEDDING_DIM)
        vector = self._vectors[dominant] + noise
        return (vector / np.linalg.norm(vector)).astype(np.float32)

    def encode_batch(self, crops: list[np.ndarray]) -> list[np.ndarray]:
        return [self.encode(crop) for crop in crops]


Path2D = Callable[[float], tuple[float, float] | None]


@dataclass(frozen=True)
class Person:
    name: str
    clothes: str
    path: Path2D


@dataclass(frozen=True)
class Scenario:
    name: str
    purpose: str
    people: tuple[Person, ...]
    times: tuple[float, ...]
    camera_yaw: Callable[[float], float] = lambda _t: 0.0


def _line(start: tuple[float, float], end: tuple[float, float], t0: float, t1: float) -> Path2D:
    def position(t: float) -> tuple[float, float] | None:
        if t < t0 or t > t1:
            return None
        s = (t - t0) / (t1 - t0)
        return (start[0] + s * (end[0] - start[0]), start[1] + s * (end[1] - start[1]))
    return position


def _uniform(duration: float, rate: float = 10.0) -> tuple[float, ...]:
    return tuple(round(i / rate, 4) for i in range(int(duration * rate) + 1))


def _irregular(duration: float, seed: int = 3) -> tuple[float, ...]:
    """Jittered 5-20 Hz source with a 0.8 s gap every 3 s."""
    rng, times, t = np.random.default_rng(seed), [], 0.0
    while t <= duration:
        if not (t % 3.0 > 2.2):
            times.append(round(t, 4))
        t += float(rng.uniform(0.05, 0.2))
    return tuple(times)


def scenarios() -> list[Scenario]:
    return [
        Scenario("walk_away", "baseline: one target walking away, static bystander",
                 (Person("target", "target", _line((2.5, 0.0), (7.0, 0.5), 0, 15)),
                  Person("bystander", "other_a", _line((4.0, -2.5), (4.0, -2.5), 0, 15))),
                 _uniform(15)),
        Scenario("crossing_similar", "similar clothes crosses in front and occludes the target",
                 (Person("target", "target", _line((4.6, 2.0), (4.6, -2.0), 0, 12)),
                  Person("distractor", "similar", _line((4.0, -2.0), (4.0, 2.0), 0, 12))),
                 _uniform(12)),
        Scenario("occlusion_reentry", "target passes 2 s behind a standing person, then re-enters",
                 (Person("target", "target", _line((5.0, 2.0), (5.0, -2.0), 0, 14)),
                  Person("occluder", "other_a", _line((3.0, 0.0), (3.0, 0.0), 0, 14))),
                 _uniform(14)),
        Scenario("leave_impostor", "target leaves; a differently dressed person enters at the exit",
                 (Person("target", "target", _line((4.0, 0.0), (4.0, -5.0), 0, 6)),
                  Person("impostor", "other_b", _line((4.0, -5.0), (4.0, 0.0), 8, 14))),
                 _uniform(14)),
        Scenario("similar_elsewhere", "target leaves; similar clothes enters far from its exit",
                 (Person("target", "target", _line((4.0, 0.0), (4.0, 5.0), 0, 5)),
                  Person("lookalike", "similar", _line((4.0, -5.0), (4.0, -0.5), 8, 14))),
                 _uniform(14)),
        Scenario("lookalike_reentry", "target stays hidden; similar clothes emerges where it vanished",
                 (Person("target", "target", _line((5.0, 2.0), (5.0, 0.0), 0, 7)),
                  Person("occluder", "other_a", _line((3.0, 0.0), (3.0, 0.0), 0, 16)),
                  Person("lookalike", "similar", _line((5.0, 0.0), (5.0, -2.0), 9, 16))),
                 _uniform(16)),
        Scenario("dropped_frames", "side-by-side walkers under 5-20 Hz jitter and 0.8 s gaps",
                 (Person("target", "target", _line((4.0, 2.5), (4.0, -2.5), 0, 6)),
                  Person("companion", "other_a", _line((4.0, 3.2), (4.0, -1.8), 0, 6))),
                 _irregular(6)),
        Scenario("ego_rotation", "camera yaws +/-0.6 rad at 1/3 Hz while the target walks slowly",
                 (Person("target", "target", _line((4.0, 0.5), (5.0, 0.0), 0, 12)),
                  Person("bystander", "other_a", _line((4.5, -1.5), (4.5, -1.5), 0, 12))),
                 _uniform(12), lambda t: 0.6 * math.sin(2.0 * math.pi * t / 3.0)),
    ]


def _project(point: tuple[float, float], yaw: float) -> tuple[float, list[float]] | None:
    """Return depth and the clipped xyxy box, or None when mostly outside the image."""
    c, s = math.cos(yaw), math.sin(yaw)
    depth = c * point[0] + s * point[1]
    lateral = -s * point[0] + c * point[1]
    if depth < 0.5:
        return None
    u = CX - FOCAL * lateral / depth
    half = FOCAL * PERSON_WIDTH_M / 2.0 / depth
    box = [u - half, CY - FOCAL * (PERSON_HEIGHT_M - CAMERA_HEIGHT_M) / depth, u + half,
           CY + FOCAL * CAMERA_HEIGHT_M / depth]
    clipped = [max(0.0, box[0]), max(0.0, box[1]), min(WIDTH - 1.0, box[2]), min(HEIGHT - 1.0, box[3])]
    if (clipped[2] - clipped[0]) < 0.5 * (box[2] - box[0]):
        return None
    return depth, clipped


def _background_panorama() -> np.ndarray:
    """A grey cylindrical texture at fixed world bearings, so camera yaw moves it."""
    rng = np.random.default_rng(11)
    width = int(2 * math.pi * FOCAL)
    coarse = rng.integers(20, 120, size=(HEIGHT // 8 + 1, width // 8 + 1), dtype=np.uint8)
    texture = np.kron(coarse, np.ones((8, 8), dtype=np.uint8))[:HEIGHT, :width]
    return np.repeat(texture[:, :, None], 3, axis=2)


def _background(panorama: np.ndarray, yaw: float) -> np.ndarray:
    bearings = np.arctan((CX - np.arange(WIDTH)) / FOCAL) + yaw
    columns = np.round(-bearings * FOCAL).astype(int) % panorama.shape[1]
    return panorama[:, columns].copy()


def _visible_fraction(box: list[float], occluders: list[list[float]]) -> float:
    mask = np.ones((int(box[3] - box[1]) + 1, int(box[2] - box[0]) + 1), dtype=bool)
    for other in occluders:
        x1, y1 = max(0, int(other[0] - box[0])), max(0, int(other[1] - box[1]))
        x2, y2 = min(mask.shape[1], int(other[2] - box[0]) + 1), min(mask.shape[0], int(other[3] - box[1]) + 1)
        if x2 > x1 and y2 > y1:
            mask[y1:y2, x1:x2] = False
    return float(mask.mean())


def write_scenario(scenario: Scenario, dataset_dir: Path, seed: int = 0) -> None:
    """Render one scenario into ``panoramic_images``, ``GTs``, ``detections`` and ``annotations``."""
    import cv2

    rng = np.random.default_rng(seed)
    panorama = _background_panorama()
    image_dir = dataset_dir / "panoramic_images" / scenario.name
    image_dir.mkdir(parents=True, exist_ok=True)
    truth, annotations, detection_lines = {}, {}, []
    for t in scenario.times:
        key = str(int(round(t * 1e9)) + 1_000_000_000)
        yaw = scenario.camera_yaw(t)
        visible = []
        for person in scenario.people:
            point = person.path(t)
            projected = None if point is None else _project(point, yaw)
            if projected is not None:
                visible.append((projected[0], person, point, projected[1]))
        visible.sort(key=lambda item: -item[0])  # paint far to near
        image = _background(panorama, yaw)
        frame_annotations, boxes, positions = {}, [], []
        for index, (_depth, person, point, box) in enumerate(visible):
            x1, y1, x2, y2 = (int(round(v)) for v in box)
            image[y1:y2 + 1, x1:x2 + 1] = COLOURS[person.clothes][0]
            nearer = [item[3] for item in visible[index + 1:]]
            fraction = _visible_fraction(box, nearer)
            frame_annotations[person.name] = {"bbox_xyxy": box, "visible_fraction": fraction}
            if fraction < 0.3 or rng.random() < 0.03:
                continue  # fully occluded or a detector miss
            score = float(np.clip(rng.normal(0.8, 0.06), 0.31, 0.99)) if fraction > 0.7 else float(rng.uniform(0.12, 0.29))
            noisy = [float(v + rng.normal(0.0, 2.0)) for v in box]
            boxes.append([*noisy, score])
            positions.append([float(point[0] + rng.normal(0, 0.05)), float(point[1] + rng.normal(0, 0.05)), 0.0])
        cv2.imwrite(str(image_dir / f"{key}.png"), image)
        target = frame_annotations.get("target")
        exists = target is not None and target["visible_fraction"] >= 0.3
        x1, y1, x2, y2 = target["bbox_xyxy"] if exists else (0, 0, 0, 0)
        truth[key] = {"is_exist": exists, "bbox": [x1, y1, x2 - x1, y2 - y1]}
        annotations[key] = frame_annotations
        detection_lines.append(json.dumps({"ts": key, "boxes": boxes, "positions": positions}))
    for folder, value in (("GTs", truth), ("annotations", annotations)):
        (dataset_dir / folder).mkdir(parents=True, exist_ok=True)
        (dataset_dir / folder / f"{scenario.name}.json").write_text(json.dumps(value), encoding="utf-8")
    (dataset_dir / "detections").mkdir(parents=True, exist_ok=True)
    (dataset_dir / "detections" / f"{scenario.name}.jsonl").write_text("\n".join(detection_lines), encoding="utf-8")


def identity_diagnostics(predictions: dict, annotations: dict, truth: dict) -> dict[str, Any]:
    """Attribute each reported box to an annotated person (IoU >= 0.5) after replay."""
    from tools.datasets.dino_follow import _iou

    counts = {"correct": 0, "wrong_person": 0, "unattributed": 0, "missed_visible": 0, "reported_while_absent": 0}
    recovery, waiting_since = [], None
    for key in sorted(truth, key=int):
        x, y, w, h, _ = predictions[key]["target_info"]
        reported = w > 0 and h > 0
        exists = truth[key]["is_exist"]
        owner = None
        if reported:
            overlaps = {name: _iou([x, y, x + w, y + h], value["bbox_xyxy"]) for name, value in annotations[key].items()}
            best = max(overlaps, key=overlaps.get, default=None)
            owner = best if best is not None and overlaps[best] >= 0.5 else None
        if reported and owner == "target":
            counts["correct"] += 1
        elif reported and owner is not None:
            counts["wrong_person"] += 1
        elif reported:
            counts["unattributed"] += 1
        if exists and owner != "target":
            counts["missed_visible"] += 1
        if reported and not exists:
            counts["reported_while_absent"] += 1
        seconds = (int(key) - 1_000_000_000) * 1e-9
        if exists and owner != "target" and waiting_since is None:
            waiting_since = seconds
        elif owner == "target" and waiting_since is not None:
            recovery.append(seconds - waiting_since)
            waiting_since = None
    counts["longest_recovery_s"] = round(max(recovery), 3) if recovery else 0.0
    counts["unrecovered"] = waiting_since is not None
    return counts


# (appearance encoder, oracle verifier, verifier latency s). 7.88 s is the median
# Kimi identity call measured on 2026-11-16 (person-follow.md).
VARIANTS = {
    "motion": (False, False, 0.0),
    "appearance-no-verifier": (True, False, 0.0),
    "appearance-oracle-0s": (True, True, 0.0),
    "appearance-oracle-7.9s": (True, True, 7.88),
    "appearance-provisional-oracle-7.9s": (True, True, 7.88),
}


def run_suite(dataset_dir: Path, results_dir: Path) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for scenario in scenarios():
        write_scenario(scenario, dataset_dir)
        truth = json.loads((dataset_dir / "GTs" / f"{scenario.name}.json").read_text(encoding="utf-8"))
        annotations = json.loads((dataset_dir / "annotations" / f"{scenario.name}.json").read_text(encoding="utf-8"))
        detections = load_detections(dataset_dir / "detections" / f"{scenario.name}.jsonl")
        summary[scenario.name] = {"purpose": scenario.purpose}
        for method, (appearance, oracle, latency) in VARIANTS.items():
            predictions, diagnostics = replay_sequence(
                dataset_dir / "panoramic_images" / scenario.name, truth, detections,
                encoder=MarkerEncoder() if appearance else None,
                verifier=oracle_verifier(truth) if oracle else None, verifier_latency_s=latency,
                provisional_min_similarity=0.75 if method.startswith("appearance-provisional") else None,
            )
            out = results_dir / scenario.name
            out.mkdir(parents=True, exist_ok=True)
            (out / f"{method}.json").write_text(json.dumps(predictions), encoding="utf-8")
            summary[scenario.name][method] = {
                **{k: round(v, 4) for k, v in score_sequence(truth, predictions).items()},
                **identity_diagnostics(predictions, annotations, truth),
                "selected_track_changes": diagnostics["selected_track_changes"],
                "verifier_verdicts": diagnostics["verifier_verdicts"],
                "provisional_revocations": diagnostics["provisional_revocations"],
            }
    return summary


def _markdown(summary: dict[str, Any]) -> str:
    lines = ["| scenario | method | AO | F | AMR | wrong person | reported while absent | missed visible | longest recovery s |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for name, rows in summary.items():
        for method, row in rows.items():
            if method == "purpose":
                continue
            lines.append(f"| {name} | {method} | {row['ao']:.3f} | {row['f_score']:.3f} | {row['amr']:.3f} | "
                         f"{row['wrong_person']} | {row['reported_while_absent']} | {row['missed_visible']} | "
                         f"{row['longest_recovery_s']}{' (unrecovered)' if row['unrecovered'] else ''} |")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset-dir", type=Path, default=ROOT / "build/person-bench/synthetic")
    parser.add_argument("--results-dir", type=Path, default=ROOT / "build/person-bench/results")
    args = parser.parse_args()
    summary = run_suite(args.dataset_dir, args.results_dir)
    args.results_dir.mkdir(parents=True, exist_ok=True)
    (args.results_dir / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    table = _markdown(summary)
    (args.results_dir / "summary.md").write_text(table + "\n", encoding="utf-8")
    print(table)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
