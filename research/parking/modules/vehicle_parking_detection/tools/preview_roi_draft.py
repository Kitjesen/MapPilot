from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

MODULES_ROOT = Path(__file__).resolve().parents[2]
if str(MODULES_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULES_ROOT))

from vehicle_parking_detection.config import load_yaml, normalize_polygon


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Preview no-parking ROI draft on extracted keyframes.")
    parser.add_argument("--draft", required=True)
    parser.add_argument("--keyframes-dir", default="", help="Defaults to <draft-dir>/keyframes.")
    parser.add_argument("--output", default="", help="Defaults to <draft-dir>/roi_preview.jpg.")
    parser.add_argument("--thumb-width", type=int, default=640)
    return parser.parse_args()


def draw_polygon(frame, label: str, polygon: list[tuple[int, int]]) -> None:
    if len(polygon) < 3:
        return
    color = (0, 255, 0)
    arr = np.array(polygon, dtype=np.int32)
    cv2.polylines(frame, [arr], True, color, 3)
    x, y = polygon[0]
    cv2.putText(frame, label, (max(5, x), max(24, y + 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)


def resize(frame, width: int):
    if width <= 0 or frame.shape[1] <= width:
        return frame
    height = int(round(frame.shape[0] * width / frame.shape[1]))
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def draft_rois(draft: dict[str, Any]) -> list[tuple[str, list[tuple[int, int]]]]:
    output = []
    for roi in draft.get("rois", []) or []:
        label = str(roi.get("name") or roi.get("id") or "no_parking_roi")
        output.append((label, normalize_polygon(roi.get("polygon"))))
    return output


def main() -> None:
    args = parse_args()
    draft_path = Path(args.draft)
    draft = load_yaml(draft_path)
    keyframes_dir = Path(args.keyframes_dir) if args.keyframes_dir else draft_path.parent / "keyframes"
    output = Path(args.output) if args.output else draft_path.parent / "roi_preview.jpg"
    image_paths = sorted(keyframes_dir.glob("*.jpg"))
    latest = draft_path.parent / "latest_source_rotated.jpg"
    if latest.exists():
        image_paths = [latest] + image_paths
    if not image_paths:
        raise SystemExit(f"no jpg keyframes found under: {keyframes_dir}")

    frames = []
    rois = draft_rois(draft)
    for path in image_paths[:8]:
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        cv2.putText(frame, path.name, (20, 46), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
        for label, polygon in rois:
            draw_polygon(frame, label, polygon)
        frames.append(resize(frame, args.thumb_width))
    if not frames:
        raise SystemExit("failed to read keyframe images")
    while len(frames) % 2:
        frames.append(frames[-1].copy())
    rows = [cv2.hconcat(frames[index : index + 2]) for index in range(0, len(frames), 2)]
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), cv2.vconcat(rows))
    print(output.resolve())


if __name__ == "__main__":
    main()
