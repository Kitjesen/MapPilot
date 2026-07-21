from __future__ import annotations

import argparse
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2

MODULES_ROOT = Path(__file__).resolve().parents[2]
if str(MODULES_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULES_ROOT))

from vehicle_parking_detection.config import DEFAULT_LOCATION, DEFAULT_POINT_ID, dump_yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_BOARD_OUTPUT = PROJECT_ROOT / "board_return" / "vehicle_parking_detection" / "output"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "validation" / "vehicle_parking_roi"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract keyframes for marking a no-parking ROI.")
    parser.add_argument("--run-dir", default="latest", help="Fetched run directory, or latest.")
    parser.add_argument("--point-id", default=DEFAULT_POINT_ID)
    parser.add_argument("--location", default=DEFAULT_LOCATION)
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--video-name", default="raw_rotated.mp4")
    parser.add_argument("--seconds", default="", help="Comma-separated extra seconds to sample.")
    parser.add_argument("--sample-count", type=int, default=7)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def resolve_run_dir(raw: str) -> Path:
    if raw != "latest":
        return Path(raw).resolve()
    if not DEFAULT_BOARD_OUTPUT.exists():
        raise SystemExit(f"no fetched board output directory: {DEFAULT_BOARD_OUTPUT}")
    candidates = sorted(
        [path for path in DEFAULT_BOARD_OUTPUT.iterdir() if path.is_dir()],
        key=lambda item: item.stat().st_mtime,
    )
    if not candidates:
        raise SystemExit(f"no fetched RDK runs under: {DEFAULT_BOARD_OUTPUT}")
    return candidates[-1].resolve()


def video_info(video_path: Path) -> dict[str, Any]:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"failed to open video: {video_path}")
    try:
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    finally:
        cap.release()
    return {
        "width": width,
        "height": height,
        "fps": fps,
        "frames": frames,
        "duration_sec": frames / fps if fps > 0 else 0.0,
    }


def parse_seconds(raw: str) -> list[float]:
    values = []
    for item in raw.split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    return values


def uniform_seconds(duration: float, count: int) -> list[float]:
    if duration <= 0 or count <= 0:
        return []
    if count == 1:
        return [duration / 2.0]
    margin = min(1.0, duration * 0.05)
    start = margin
    end = max(start, duration - margin)
    return [start + (end - start) * index / max(1, count - 1) for index in range(count)]


def extract_frames(video_path: Path, keyframe_dir: Path, seconds: list[float], overwrite: bool) -> list[dict[str, Any]]:
    keyframe_dir.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"failed to open video: {video_path}")
    output = []
    try:
        fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
        for second in sorted({round(max(0.0, item), 3) for item in seconds}):
            file_name = f"t{second:010.3f}".replace(".", "_") + ".jpg"
            path = keyframe_dir / file_name
            if path.exists() and not overwrite:
                output.append({"second": second, "path": str(path), "status": "exists"})
                continue
            if fps > 0:
                cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(second * fps)))
            else:
                cap.set(cv2.CAP_PROP_POS_MSEC, second * 1000.0)
            ok, frame = cap.read()
            if not ok:
                output.append({"second": second, "path": str(path), "status": "failed"})
                continue
            cv2.putText(frame, f"t={second:.1f}s", (20, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 3)
            cv2.imwrite(str(path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            output.append({"second": second, "path": str(path), "status": "written"})
    finally:
        cap.release()
    return output


def draft(point_id: str, location: str, info: dict[str, Any], run_dir: Path) -> dict[str, Any]:
    return {
        "schema_version": "vehicle-parking-roi-draft/v1",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "source_run_dir": str(run_dir),
        "coordinate_frame": "rdk_recorder_rotated_frame",
        "point_id": point_id,
        "location": location,
        "frame_width": info["width"],
        "frame_height": info["height"],
        "rois": [
            {
                "id": "no_parking_zone_01",
                "name": location,
                "polygon": [],
                "dwell_seconds": 2,
                "cooldown_seconds": 300,
            }
        ],
    }


def main() -> None:
    args = parse_args()
    run_dir = resolve_run_dir(args.run_dir)
    video_path = run_dir / args.video_name
    if not video_path.exists():
        raise SystemExit(f"video not found: {video_path}")
    info = video_info(video_path)
    out_dir = Path(args.output_root) / args.point_id / run_dir.name
    keyframe_dir = out_dir / "keyframes"
    out_dir.mkdir(parents=True, exist_ok=True)

    seconds = uniform_seconds(info["duration_sec"], args.sample_count) + parse_seconds(args.seconds)
    frames = extract_frames(video_path, keyframe_dir, seconds, args.overwrite)
    latest_source = run_dir / "latest_source_rotated.jpg"
    if latest_source.exists():
        shutil.copy2(latest_source, out_dir / "latest_source_rotated.jpg")

    manifest = {
        "schema_version": "vehicle-parking-roi-keyframes/v1",
        "source_run_dir": str(run_dir),
        "video": str(video_path),
        "video_info": info,
        "keyframes": frames,
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    dump_yaml(out_dir / "roi_draft.yaml", draft(args.point_id, args.location, info, run_dir))
    print(f"roi_draft={out_dir / 'roi_draft.yaml'}")
    print(f"keyframes={keyframe_dir}")


if __name__ == "__main__":
    main()
