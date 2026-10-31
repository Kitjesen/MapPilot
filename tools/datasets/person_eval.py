"""Attribute selected video tracks to MOT annotations after the experiment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _iou(box: np.ndarray, truth: np.ndarray) -> float:
    left = np.maximum(box[:2], truth[:2])
    right = np.minimum(box[2:], truth[2:])
    intersection = float(np.prod(np.maximum(0, right - left)))
    area = float(np.prod(np.maximum(0, box[2:] - box[:2])))
    truth_area = float(np.prod(np.maximum(0, truth[2:] - truth[:2])))
    union = area + truth_area - intersection
    return intersection / union if union > 0 else 0.0


def evaluate(records: list[dict], annotations: np.ndarray, minimum_iou: float = 0.5) -> dict:
    """Report target identity evidence without feeding labels into tracking."""
    reference_id = None
    by_frame = {}
    for row in np.asarray(annotations).reshape(-1, 9):
        if row[6] != 1 or row[7] != 1:
            continue
        box = np.array([row[2], row[3], row[2] + row[4], row[3] + row[5]])
        by_frame.setdefault(int(row[0]) - 1, []).append((int(row[1]), box))
    details = []
    previous_track_id = None
    track_changes = 0
    for record in records:
        target = record["target"]
        identity, overlap = None, 0.0
        if target is not None:
            box = np.asarray(target["bbox_xyxy"])
            for candidate_id, truth in by_frame.get(record["frame"], []):
                candidate_overlap = _iou(box, truth)
                if candidate_overlap > overlap:
                    identity, overlap = candidate_id, candidate_overlap
            if overlap < minimum_iou:
                identity = None
            if identity is not None and reference_id is None:
                reference_id = identity
            track_id = target["track_id"]
            if previous_track_id is not None and track_id != previous_track_id:
                track_changes += 1
            previous_track_id = track_id
        details.append({
            "frame": record["frame"], "selected_track_id": record["selected_track_id"],
            "target_present": target is not None, "attributed_gt_id": identity,
            "best_iou": overlap,
        })
    correct = sum(row["attributed_gt_id"] == reference_id for row in details) if reference_id is not None else 0
    wrong = sum(row["attributed_gt_id"] is not None and row["attributed_gt_id"] != reference_id for row in details)
    ambiguous = sum(row["target_present"] and row["attributed_gt_id"] is None for row in details)
    missing = sum(not row["target_present"] for row in details)
    return {
        "metric": "selected_target_iou_attribution", "minimum_iou": minimum_iou,
        "reference_gt_id": reference_id, "frames": len(records),
        "correct_reference_frames": correct, "wrong_identity_frames": wrong,
        "unattributed_target_frames": ambiguous, "missing_target_frames": missing,
        "selected_track_id_changes": track_changes,
        "reference_annotated_frames": sum(
            any(identity == reference_id for identity, _ in by_frame.get(record["frame"], []))
            for record in records
        ) if reference_id is not None else 0,
        "ground_truth_used_for_execution": False, "standard_mot_benchmark_score": False,
        "frame_evidence": details,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracks", type=Path, required=True)
    parser.add_argument("--gt", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = evaluate(json.loads(args.tracks.read_text()), np.loadtxt(args.gt, delimiter=",", ndmin=2))
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "frame_evidence"}))


if __name__ == "__main__":
    main()
