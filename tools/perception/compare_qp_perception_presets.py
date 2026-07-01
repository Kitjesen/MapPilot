"""Compare qp_perception YoloSegTracker presets on the same video frames."""

from __future__ import annotations

import argparse
import inspect
import json
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)
QP_PERCEPTION_SRC = REPO_ROOT / "modules" / "perception" / "src"

if QP_PERCEPTION_SRC.exists():
    sys.path.insert(0, str(QP_PERCEPTION_SRC))

PRESETS: dict[str, dict[str, Any]] = {
    "baseline": {
        "confidence_threshold": 0.25,
        "nms_iou_threshold": 0.45,
        "img_size": 640,
        "max_detections": 30,
    },
    "balanced": {
        "confidence_threshold": 0.18,
        "nms_iou_threshold": 0.45,
        "img_size": 960,
        "max_detections": 64,
    },
    "aggressive": {
        "confidence_threshold": 0.15,
        "nms_iou_threshold": 0.45,
        "img_size": 1280,
        "max_detections": 96,
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True, help="Path to the source video.")
    parser.add_argument(
        "--preset",
        action="append",
        choices=sorted(PRESETS),
        help="Preset to evaluate. Repeat to compare multiple presets. Default: all presets.",
    )
    parser.add_argument(
        "--model-path",
        default="yolo11n-seg.pt",
        help="Ultralytics segmentation checkpoint passed to YoloSegTracker.",
    )
    parser.add_argument("--device", default="", help="Inference device, for example cpu or cuda:0.")
    parser.add_argument("--max-frames", type=int, default=300, help="Maximum frames to evaluate.")
    parser.add_argument("--frame-stride", type=int, default=1, help="Use every Nth frame.")
    parser.add_argument(
        "--small-box-threshold-px",
        type=float,
        default=24.0,
        help="Boxes with min(width, height) below this threshold count as small.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional JSON output path. Defaults to tools/figures/qp_perception_preset_report.json.",
    )
    parser.add_argument(
        "--include-frame-stats",
        action="store_true",
        help="Include per-frame metrics in the JSON output.",
    )
    return parser.parse_args()


def build_tracker(preset_name: str, model_path: str, device: str):
    from qp_perception.tracking.yolo_seg import YoloSegTracker

    preset = dict(PRESETS[preset_name])
    kwargs = {
        "model_path": model_path,
        "confidence_threshold": preset["confidence_threshold"],
        "nms_iou_threshold": preset["nms_iou_threshold"],
        "class_whitelist": ["person"],
        "device": device,
        "img_size": preset["img_size"],
    }
    signature = inspect.signature(YoloSegTracker)
    if "max_detections" in signature.parameters:
        kwargs["max_detections"] = preset["max_detections"]

    return YoloSegTracker(**kwargs), preset


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values, dtype=np.float32), percentile))


def _summarize_tracks(tracks: list[Any], small_box_threshold_px: float) -> dict[str, Any]:
    confidences: list[float] = []
    track_ids: set[int] = set()
    small_box_count = 0

    for track in tracks:
        bbox = getattr(track, "bbox", None)
        if bbox is None:
            continue
        width = float(getattr(bbox, "w", 0.0))
        height = float(getattr(bbox, "h", 0.0))
        if min(width, height) < small_box_threshold_px:
            small_box_count += 1
        confidences.append(float(getattr(track, "confidence", 0.0)))
        track_id = getattr(track, "track_id", None)
        if track_id is not None:
            track_ids.add(int(track_id))

    return {
        "count": len(tracks),
        "small_box_count": small_box_count,
        "confidences": confidences,
        "track_ids": track_ids,
    }


def evaluate_preset(
    video_path: Path,
    preset_name: str,
    model_path: str,
    device: str,
    max_frames: int,
    frame_stride: int,
    small_box_threshold_px: float,
    include_frame_stats: bool,
) -> dict[str, Any]:
    tracker, preset = build_tracker(preset_name, model_path, device)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Failed to open video: {video_path}")

    source_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    processed_frames = 0
    raw_frame_index = 0
    frame_rows: list[dict[str, Any]] = []
    latencies_ms: list[float] = []
    all_confidences: list[float] = []
    unique_track_ids: set[int] = set()

    try:
        while processed_frames < max_frames:
            ok, frame = capture.read()
            if not ok:
                break

            if frame_stride > 1 and raw_frame_index % frame_stride != 0:
                raw_frame_index += 1
                continue

            timestamp = (
                raw_frame_index / source_fps
                if source_fps > 0.0
                else time.perf_counter()
            )
            t0 = time.perf_counter()
            tracks = tracker.detect_and_track(frame, timestamp)
            latency_ms = (time.perf_counter() - t0) * 1000.0
            summary = _summarize_tracks(tracks, small_box_threshold_px)

            latencies_ms.append(latency_ms)
            all_confidences.extend(summary["confidences"])
            unique_track_ids.update(summary["track_ids"])

            frame_rows.append(
                {
                    "frame_index": raw_frame_index,
                    "timestamp": round(timestamp, 4),
                    "latency_ms": round(latency_ms, 3),
                    "track_count": summary["count"],
                    "small_box_count": summary["small_box_count"],
                    "mean_confidence": round(
                        statistics.mean(summary["confidences"]), 4
                    )
                    if summary["confidences"]
                    else 0.0,
                }
            )

            processed_frames += 1
            raw_frame_index += 1
    finally:
        capture.release()

    if not frame_rows:
        raise RuntimeError(f"No frames were processed for preset {preset_name!r}.")

    track_counts = [row["track_count"] for row in frame_rows]
    small_box_counts = [row["small_box_count"] for row in frame_rows]

    avg_latency_ms = statistics.mean(latencies_ms)
    summary = {
        "preset": preset_name,
        "source_video": str(video_path),
        "frames_processed": len(frame_rows),
        "frame_stride": frame_stride,
        "avg_latency_ms": round(avg_latency_ms, 3),
        "max_latency_ms": round(max(latencies_ms), 3),
        "approx_fps": round(1000.0 / avg_latency_ms, 3) if avg_latency_ms > 0 else 0.0,
        "avg_tracks_per_frame": round(statistics.mean(track_counts), 3),
        "max_tracks_in_frame": max(track_counts),
        "avg_small_boxes_per_frame": round(statistics.mean(small_box_counts), 3),
        "max_small_boxes_in_frame": max(small_box_counts),
        "confidence_mean": round(statistics.mean(all_confidences), 4) if all_confidences else 0.0,
        "confidence_p50": round(_percentile(all_confidences, 50.0), 4),
        "confidence_p90": round(_percentile(all_confidences, 90.0), 4),
        "unique_track_ids": len(unique_track_ids),
        "class_whitelist": ["person"],
        "preset_config": preset,
    }

    result = {"summary": summary}
    if include_frame_stats:
        result["frames"] = frame_rows
    return result


def main() -> int:
    args = parse_args()
    video_path = Path(args.video).expanduser().resolve()
    if not video_path.exists():
        raise FileNotFoundError(f"Video not found: {video_path}")

    preset_names = args.preset or list(PRESETS)
    output_path = Path(args.output).expanduser() if args.output else (
        Path(__file__).resolve().parent / "figures" / "qp_perception_preset_report.json"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    report = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "video": str(video_path),
        "presets": {},
    }

    for preset_name in preset_names:
        report["presets"][preset_name] = evaluate_preset(
            video_path=video_path,
            preset_name=preset_name,
            model_path=args.model_path,
            device=args.device,
            max_frames=args.max_frames,
            frame_stride=max(args.frame_stride, 1),
            small_box_threshold_px=args.small_box_threshold_px,
            include_frame_stats=args.include_frame_stats,
        )
        summary = report["presets"][preset_name]["summary"]
        print(
            f"{preset_name}: "
            f"tracks/frame={summary['avg_tracks_per_frame']:.2f}, "
            f"small/frame={summary['avg_small_boxes_per_frame']:.2f}, "
            f"latency={summary['avg_latency_ms']:.2f}ms"
        )

    output_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"Wrote report to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
