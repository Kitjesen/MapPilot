from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

MODULES_ROOT = Path(__file__).resolve().parents[2]
TOOLS_ROOT = Path(__file__).resolve().parent
if str(MODULES_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULES_ROOT))
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from apply_roi_draft import apply_draft
from vehicle_parking_detection.config import DEFAULT_LOCATION, DEFAULT_POINT_ID, dump_yaml, load_yaml, normalize_polygon
from vehicle_parking_detection.scene_matching import (
    active_rois_for_scene,
    anchor_for_frame,
    best_scene_match,
    scene_anchors_for_roi,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_BOARD_OUTPUT = PROJECT_ROOT / "board_return" / "vehicle_parking_detection" / "output"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "validation" / "vehicle_parking_roi"
DEFAULT_SITE_CONFIG = PROJECT_ROOT / "modules" / "vehicle_parking_detection" / "configs" / "site_rois.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactively mark no-parking polygon ROI from a fetched RDK video.")
    parser.add_argument("--run-dir", default="latest", help="Fetched run directory, or latest.")
    parser.add_argument("--video-name", default="raw_rotated.mp4")
    parser.add_argument("--draft", default="", help="Defaults to validation/vehicle_parking_roi/<point_id>/<run_id>/roi_draft.yaml.")
    parser.add_argument("--point-id", default=DEFAULT_POINT_ID)
    parser.add_argument("--location", default=DEFAULT_LOCATION)
    parser.add_argument("--roi-prefix", default="no_parking_zone")
    parser.add_argument("--dwell-seconds", type=float, default=2.0)
    parser.add_argument("--cooldown-seconds", type=float, default=300.0)
    parser.add_argument("--site-config", default=str(DEFAULT_SITE_CONFIG))
    parser.add_argument("--apply", action="store_true", help="Also write non-empty ROI polygons into site_rois.yaml.")
    parser.add_argument("--replace", action="store_true", help="Clear existing ROI polygons in the draft before marking.")
    parser.add_argument("--max-window-width", type=int, default=1200)
    parser.add_argument("--max-window-height", type=int, default=1100)
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


def default_draft_path(run_dir: Path, point_id: str) -> Path:
    return DEFAULT_OUTPUT_ROOT / point_id / run_dir.name / "roi_draft.yaml"


def initial_draft(point_id: str, location: str, info: dict[str, Any], run_dir: Path, roi_prefix: str, dwell: float, cooldown: float) -> dict[str, Any]:
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
                "id": f"{roi_prefix}_01",
                "name": f"{roi_prefix}_01",
                "polygon": [],
                "dwell_seconds": dwell,
                "cooldown_seconds": cooldown,
            }
        ],
    }


def site_rois_for_point(site_config_path: str | Path, point_id: str) -> list[dict[str, Any]]:
    path = Path(site_config_path)
    if not path.exists():
        return []
    config = load_yaml(path)
    for point in config.get("points", []) or []:
        if str(point.get("point_id") or point.get("patrol_point_id") or "") == point_id:
            return [copy.deepcopy(roi) for roi in point.get("rois", []) or [] if isinstance(roi, dict)]
    return []


def merge_site_rois_into_draft(draft: dict[str, Any], args: argparse.Namespace) -> None:
    existing = site_rois_for_point(args.site_config, str(draft.get("point_id") or args.point_id))
    if not existing:
        return
    rois = draft.setdefault("rois", [])
    by_id = {str(roi.get("id") or ""): roi for roi in rois if isinstance(roi, dict)}
    for source in existing:
        roi_id = str(source.get("id") or "").strip()
        if not roi_id:
            continue
        target = by_id.get(roi_id)
        if target is None:
            rois.append(source)
            by_id[roi_id] = source
            continue
        if not normalize_polygon(target.get("polygon")):
            target.clear()
            target.update(source)


def ensure_draft(args: argparse.Namespace, run_dir: Path, info: dict[str, Any]) -> Path:
    draft_path = Path(args.draft).resolve() if args.draft else default_draft_path(run_dir, args.point_id)
    draft_path.parent.mkdir(parents=True, exist_ok=True)
    if draft_path.exists():
        draft = load_yaml(draft_path)
    else:
        draft = initial_draft(args.point_id, args.location, info, run_dir, args.roi_prefix, args.dwell_seconds, args.cooldown_seconds)

    draft["point_id"] = str(draft.get("point_id") or args.point_id)
    draft["location"] = str(draft.get("location") or args.location)
    draft["source_run_dir"] = str(draft.get("source_run_dir") or run_dir)
    draft["frame_width"] = int(draft.get("frame_width") or info["width"])
    draft["frame_height"] = int(draft.get("frame_height") or info["height"])
    draft.setdefault("coordinate_frame", "rdk_recorder_rotated_frame")
    draft.setdefault("schema_version", "vehicle-parking-roi-draft/v1")
    rois = draft.setdefault("rois", [])
    if args.replace:
        rois.clear()
    else:
        merge_site_rois_into_draft(draft, args)
        rois = draft.setdefault("rois", [])
    if not rois:
        rois.append(
            {
                "id": f"{args.roi_prefix}_01",
                "name": f"{args.roi_prefix}_01",
                "polygon": [],
                "dwell_seconds": args.dwell_seconds,
                "cooldown_seconds": args.cooldown_seconds,
            }
        )
    for roi in rois:
        roi["dwell_seconds"] = float(roi.get("dwell_seconds", args.dwell_seconds))
        roi["cooldown_seconds"] = float(roi.get("cooldown_seconds", args.cooldown_seconds))
    dump_yaml(draft_path, draft)
    return draft_path


def scale_for_window(frame: np.ndarray, max_width: int, max_height: int) -> float:
    height, width = frame.shape[:2]
    if width <= 0 or height <= 0:
        return 1.0
    return min(1.0, max_width / float(width), max_height / float(height))


def to_display(frame: np.ndarray, scale: float) -> np.ndarray:
    if scale >= 0.999:
        return frame.copy()
    height, width = frame.shape[:2]
    return cv2.resize(frame, (max(1, int(width * scale)), max(1, int(height * scale))), interpolation=cv2.INTER_AREA)


def is_window_open(window: str) -> bool:
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def scaled_polygon(polygon: list[tuple[int, int]], scale: float) -> np.ndarray:
    return np.array([(int(round(x * scale)), int(round(y * scale))) for x, y in polygon], dtype=np.int32)


def draw_existing_rois(frame: np.ndarray, draft: dict[str, Any], scale: float) -> None:
    colors = [(0, 255, 0), (0, 200, 255), (255, 120, 0), (255, 0, 255), (0, 128, 255)]
    for index, roi in enumerate(active_draft_rois(frame, draft)):
        polygon = normalize_polygon(roi.get("polygon"))
        if len(polygon) < 3:
            continue
        color = colors[index % len(colors)]
        arr = scaled_polygon(polygon, scale)
        cv2.polylines(frame, [arr], True, color, 2)
        label = str(roi.get("name") or roi.get("id") or f"roi_{index + 1}")
        x, y = arr[0]
        cv2.putText(frame, label, (max(5, x), max(24, y + 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def active_draft_rois(frame: np.ndarray, draft: dict[str, Any]) -> list[dict[str, Any]]:
    rois = [roi for roi in draft.get("rois", []) or [] if isinstance(roi, dict)]
    if not any(scene_anchors_for_roi(roi) for roi in rois):
        return rois
    match = best_scene_match(frame, rois, min_score=0.6, min_margin=0.08)
    return active_rois_for_scene(rois, match)


def draw_help(frame: np.ndarray, lines: list[str]) -> None:
    y = 26
    for line in lines:
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        y += 24


def suffix_number(roi_id: str, prefix: str) -> int:
    match = re.fullmatch(re.escape(prefix) + r"_(\d+)", roi_id)
    return int(match.group(1)) if match else 0


def next_roi_id(draft: dict[str, Any], prefix: str) -> str:
    used = [suffix_number(str(item.get("id") or ""), prefix) for item in draft.get("rois", []) or []]
    return f"{prefix}_{max(used or [0]) + 1:02d}"


def upsert_polygon(draft: dict[str, Any], polygon: list[tuple[int, int]], args: argparse.Namespace) -> str:
    rois = draft.setdefault("rois", [])
    target = None
    for roi in rois:
        if not normalize_polygon(roi.get("polygon")):
            target = roi
            break
    if target is None:
        roi_id = next_roi_id(draft, args.roi_prefix)
        target = {"id": roi_id, "name": roi_id}
        rois.append(target)
    roi_id = str(target.get("id") or next_roi_id(draft, args.roi_prefix))
    target["id"] = roi_id
    target["name"] = str(target.get("name") or roi_id)
    target["polygon"] = [[int(x), int(y)] for x, y in polygon]
    target["dwell_seconds"] = float(target.get("dwell_seconds", args.dwell_seconds))
    target["cooldown_seconds"] = float(target.get("cooldown_seconds", args.cooldown_seconds))
    draft["updated_at"] = datetime.now().isoformat(timespec="seconds")
    return roi_id


def update_roi_anchor(
    draft: dict[str, Any],
    roi_id: str,
    frame: np.ndarray,
    keyframe_path: Path,
    draft_path: Path,
    frame_index: int,
    second: float,
) -> None:
    relative = keyframe_path.relative_to(draft_path.parent).as_posix()
    scene_id = f"scene_f{int(frame_index):06d}"
    height, width = frame.shape[:2]
    for roi in draft.get("rois", []) or []:
        if str(roi.get("id") or "") != roi_id:
            continue
        roi["scene_id"] = scene_id
        roi["anchor_frame_index"] = int(frame_index)
        roi["anchor_second"] = float(round(second, 3))
        roi["anchor_keyframe_image"] = relative
        roi["scene_anchor"] = anchor_for_frame(
            frame,
            keyframe_image=relative,
            frame_index=frame_index,
            second=second,
            original_width=width,
            original_height=height,
        )
        roi["scene_anchor_count"] = 1 + len(roi.get("scene_anchors") or [])
        return


def append_roi_scene_anchor(
    draft: dict[str, Any],
    roi_id: str,
    frame: np.ndarray,
    keyframe_path: Path,
    draft_path: Path,
    frame_index: int,
    second: float,
) -> bool:
    relative = keyframe_path.relative_to(draft_path.parent).as_posix()
    height, width = frame.shape[:2]
    for roi in draft.get("rois", []) or []:
        if str(roi.get("id") or "") != roi_id:
            continue
        roi.setdefault("scene_id", f"scene_{roi_id}")
        anchor = anchor_for_frame(
            frame,
            keyframe_image=relative,
            frame_index=frame_index,
            second=second,
            original_width=width,
            original_height=height,
        )
        if not isinstance(roi.get("scene_anchor"), dict):
            roi["anchor_frame_index"] = int(frame_index)
            roi["anchor_second"] = float(round(second, 3))
            roi["anchor_keyframe_image"] = relative
            roi["scene_anchor"] = anchor
            roi["scene_anchor_count"] = 1 + len(roi.get("scene_anchors") or [])
            draft["updated_at"] = datetime.now().isoformat(timespec="seconds")
            return True
        anchors = roi.setdefault("scene_anchors", [])
        anchors[:] = [item for item in anchors if int(item.get("frame_index", -1)) != int(frame_index)]
        anchors.append(anchor)
        roi["scene_anchor_count"] = 1 + len(anchors)
        draft["updated_at"] = datetime.now().isoformat(timespec="seconds")
        return True
    return False


def roi_scene_id(draft: dict[str, Any], roi_id: str) -> str:
    for roi in draft.get("rois", []) or []:
        if str(roi.get("id") or "") == roi_id:
            return str(roi.get("scene_id") or "")
    return ""


def write_manifest(draft_path: Path, selected_frames: list[dict[str, Any]]) -> None:
    manifest_path = draft_path.parent / "manual_keyframes_manifest.json"
    manifest = {
        "schema_version": "vehicle-parking-manual-roi-keyframes/v1",
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "selected_keyframes": selected_frames,
    }
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_roi_preview(draft_path: Path, frame: np.ndarray, draft: dict[str, Any]) -> Path:
    scale = 1.0
    preview = frame.copy()
    draw_existing_rois(preview, draft, scale)
    output = draft_path.parent / "roi_interactive_preview.jpg"
    cv2.imwrite(str(output), preview, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
    return output


def detect_sampled_freeze_start(video_path: Path, frame_count: int, sample_step: int = 250, threshold: float = 0.05, consecutive: int = 3) -> int | None:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return None
    previous = None
    streak = 0
    streak_start = 0
    try:
        for position in range(0, max(1, frame_count), sample_step):
            cap.set(cv2.CAP_PROP_POS_FRAMES, position)
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            small = cv2.resize(gray, (64, 128), interpolation=cv2.INTER_AREA)
            if previous is None:
                previous = small
                continue
            diff = float(np.mean(cv2.absdiff(previous, small)))
            if diff <= threshold:
                if streak == 0:
                    streak_start = max(0, position - sample_step)
                streak += 1
                if streak >= consecutive:
                    return streak_start
            else:
                streak = 0
            previous = small
    finally:
        cap.release()
    return None


class InteractiveMarker:
    def __init__(self, args: argparse.Namespace, video_path: Path, draft_path: Path, info: dict[str, Any], freeze_start_frame: int | None = None) -> None:
        self.args = args
        self.video_path = video_path
        self.draft_path = draft_path
        self.info = info
        self.freeze_start_frame = freeze_start_frame
        self.cap = cv2.VideoCapture(str(video_path))
        if not self.cap.isOpened():
            raise SystemExit(f"failed to open video: {video_path}")
        self.fps = float(info["fps"] or 15.0)
        self.frame_count = int(info["frames"] or 1)
        self.frame_index = 0
        self.paused = False
        self.current_frame: np.ndarray | None = None
        self.selected_frames: list[dict[str, Any]] = []
        self.saved_roi_count = 0
        self.selected_roi_id = ""
        self.window = "vehicle parking ROI - video"
        self.updating_trackbar = False

    def close(self) -> None:
        self.cap.release()
        cv2.destroyAllWindows()

    def load_frame(self, index: int) -> np.ndarray:
        index = max(0, min(index, max(0, self.frame_count - 1)))
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read frame {index}")
        self.frame_index = index
        self.current_frame = frame
        return frame

    def read_next_frame(self) -> np.ndarray | None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.paused = True
            return None
        actual_next = int(round(self.cap.get(cv2.CAP_PROP_POS_FRAMES) or 0))
        if actual_next > 0:
            self.frame_index = min(max(0, actual_next - 1), max(0, self.frame_count - 1))
        else:
            self.frame_index = min(self.frame_index + 1, max(0, self.frame_count - 1))
        self.current_frame = frame
        if self.frame_index >= self.frame_count - 1:
            self.paused = True
        return frame

    def on_trackbar(self, value: int) -> None:
        if self.updating_trackbar:
            return
        self.frame_index = max(0, min(value, max(0, self.frame_count - 1)))
        self.paused = True
        try:
            self.load_frame(self.frame_index)
        except RuntimeError:
            pass

    def update_trackbar(self) -> None:
        self.updating_trackbar = True
        try:
            cv2.setTrackbarPos("frame", self.window, int(self.frame_index))
        finally:
            self.updating_trackbar = False

    def show_current(self) -> None:
        if self.current_frame is None:
            self.load_frame(self.frame_index)
        assert self.current_frame is not None
        frame = self.current_frame.copy()
        scale = scale_for_window(frame, self.args.max_window_width, self.args.max_window_height)
        display = to_display(frame, scale)
        draft = load_yaml(self.draft_path)
        draw_existing_rois(display, draft, scale)
        seconds = self.frame_index / self.fps if self.fps > 0 else 0.0
        self.ensure_selected_roi(draft)
        anchor_count = self.selected_roi_anchor_count(draft)
        draw_help(
            display,
            [
                f"frame {self.frame_index}/{max(0, self.frame_count - 1)}  t={seconds:.2f}s  rois_saved={self.saved_roi_count}",
                f"selected_roi={self.selected_roi_id or '-'} anchors={anchor_count}",
                "SPACE play/pause | A/D frame | J/L 1 sec | R draw ROI | H add anchor | 1-9/[ ] select | Q finish",
            ]
            + (
                [f"WARNING: sampled video appears frozen after frame {self.freeze_start_frame}; re-record if this area is needed."]
                if self.freeze_start_frame is not None and self.frame_index >= self.freeze_start_frame
                else []
            ),
        )
        cv2.imshow(self.window, display)
        self.update_trackbar()

    def roi_ids(self, draft: dict[str, Any]) -> list[str]:
        return [
            str(roi.get("id") or "")
            for roi in draft.get("rois", []) or []
            if isinstance(roi, dict) and str(roi.get("id") or "")
        ]

    def ensure_selected_roi(self, draft: dict[str, Any]) -> None:
        ids = self.roi_ids(draft)
        if ids and self.selected_roi_id not in ids:
            self.selected_roi_id = ids[0]

    def selected_roi_anchor_count(self, draft: dict[str, Any]) -> int:
        for roi in draft.get("rois", []) or []:
            if isinstance(roi, dict) and str(roi.get("id") or "") == self.selected_roi_id:
                return len(scene_anchors_for_roi(roi))
        return 0

    def select_roi_by_offset(self, offset: int) -> None:
        draft = load_yaml(self.draft_path)
        ids = self.roi_ids(draft)
        if not ids:
            self.selected_roi_id = ""
            return
        if self.selected_roi_id not in ids:
            self.selected_roi_id = ids[0]
            return
        index = ids.index(self.selected_roi_id)
        self.selected_roi_id = ids[(index + offset) % len(ids)]
        print(f"selected ROI: {self.selected_roi_id}")

    def select_roi_by_number(self, number: int) -> None:
        draft = load_yaml(self.draft_path)
        ids = self.roi_ids(draft)
        if 1 <= number <= len(ids):
            self.selected_roi_id = ids[number - 1]
            print(f"selected ROI: {self.selected_roi_id}")
        else:
            print(f"no ROI at index {number}; current ROI count={len(ids)}")

    def add_anchor_to_selected_roi(self) -> None:
        if self.current_frame is None:
            self.load_frame(self.frame_index)
        assert self.current_frame is not None
        draft = load_yaml(self.draft_path)
        self.ensure_selected_roi(draft)
        if not self.selected_roi_id:
            print("No ROI selected. Draw an ROI first, or press 1-9 to select one.")
            return
        keyframe_path = self.save_keyframe(label=f"anchor_{self.selected_roi_id}_extra")
        ok = append_roi_scene_anchor(
            draft,
            self.selected_roi_id,
            self.current_frame,
            keyframe_path,
            self.draft_path,
            self.frame_index,
            self.frame_index / self.fps if self.fps > 0 else 0.0,
        )
        if not ok:
            print(f"ROI not found: {self.selected_roi_id}")
            return
        dump_yaml(self.draft_path, draft)
        preview = write_roi_preview(self.draft_path, self.current_frame, draft)
        print(f"added scene anchor to {self.selected_roi_id}: frame={self.frame_index}")
        print(f"anchor_count={self.selected_roi_anchor_count(draft)}")
        print(f"updated draft: {self.draft_path}")
        print(f"preview: {preview}")

    def step(self, frames: int) -> None:
        self.paused = True
        self.load_frame(self.frame_index + frames)

    def save_keyframe(self, label: str = "manual") -> Path:
        if self.current_frame is None:
            self.load_frame(self.frame_index)
        assert self.current_frame is not None
        seconds = self.frame_index / self.fps if self.fps > 0 else 0.0
        keyframe_dir = self.draft_path.parent / "keyframes"
        keyframe_dir.mkdir(parents=True, exist_ok=True)
        safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "_", label).strip("_") or "manual"
        name = f"{safe_label}_f{self.frame_index:06d}_t{seconds:010.3f}".replace(".", "_") + ".jpg"
        path = keyframe_dir / name
        cv2.imwrite(str(path), self.current_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        record = {
            "frame_index": self.frame_index,
            "second": round(seconds, 3),
            "path": str(path),
            "created_at": datetime.now().isoformat(timespec="seconds"),
        }
        self.selected_frames.append(record)
        write_manifest(self.draft_path, self.selected_frames)
        print(f"saved keyframe: {path}")
        return path

    def draw_roi(self) -> None:
        if self.current_frame is None:
            self.load_frame(self.frame_index)
        assert self.current_frame is not None
        base = self.current_frame.copy()
        polygon: list[tuple[int, int]] = []
        scale = scale_for_window(base, self.args.max_window_width, self.args.max_window_height)
        window = "vehicle parking ROI - draw polygon"

        def mouse_callback(event: int, x: int, y: int, _flags: int, _param: Any) -> None:
            if event == cv2.EVENT_LBUTTONDOWN:
                polygon.append((int(round(x / scale)), int(round(y / scale))))
            elif event == cv2.EVENT_RBUTTONDOWN and polygon:
                polygon.pop()

        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window, mouse_callback)
        while True:
            display = to_display(base, scale)
            if polygon:
                arr = scaled_polygon(polygon, scale)
                cv2.polylines(display, [arr], False, (0, 255, 255), 2)
                for index, (x, y) in enumerate(arr):
                    cv2.circle(display, (int(x), int(y)), 5, (0, 255, 255), -1)
                    cv2.putText(display, str(index + 1), (int(x) + 7, int(y) - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            draw_help(
                display,
                [
                    "Left click add point | Right click/U/Z/Backspace undo | C clear | ENTER save | ESC/Q/close cancel",
                    f"points={len(polygon)}",
                ],
            )
            cv2.imshow(window, display)
            key_raw = cv2.waitKey(30)
            if not is_window_open(window):
                print("ROI drawing canceled.")
                break
            key = key_raw & 0xFF
            if key in (13, 10):
                if len(polygon) < 3:
                    print("ROI needs at least 3 points.")
                    continue
                draft = load_yaml(self.draft_path)
                roi_id = upsert_polygon(draft, polygon, self.args)
                keyframe_path = self.save_keyframe(label=f"anchor_{roi_id}")
                update_roi_anchor(
                    draft,
                    roi_id,
                    base,
                    keyframe_path,
                    self.draft_path,
                    self.frame_index,
                    self.frame_index / self.fps if self.fps > 0 else 0.0,
                )
                dump_yaml(self.draft_path, draft)
                preview = write_roi_preview(self.draft_path, base, draft)
                self.saved_roi_count += 1
                self.selected_roi_id = roi_id
                print(f"saved ROI {roi_id}: {polygon}")
                print(f"scene anchor: {roi_scene_id(draft, roi_id)}")
                print(f"updated draft: {self.draft_path}")
                print(f"preview: {preview}")
                break
            if key in (27, ord("q")):
                print("ROI drawing canceled.")
                break
            if key in (8, 127, ord("u"), ord("U"), ord("z"), ord("Z")) and polygon:
                polygon.pop()
            if key in (ord("c"), ord("C")):
                polygon.clear()
        try:
            cv2.destroyWindow(window)
        except cv2.error:
            pass

    def run(self) -> None:
        cv2.namedWindow(self.window, cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar("frame", self.window, 0, max(1, self.frame_count - 1), self.on_trackbar)
        self.load_frame(0)
        try:
            while True:
                self.show_current()
                if not is_window_open(self.window):
                    break
                delay = 30 if self.paused else max(1, int(round(1000.0 / max(self.fps, 1.0))))
                key_raw = cv2.waitKey(delay)
                if not is_window_open(self.window):
                    break
                key = key_raw & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" "):
                    self.paused = not self.paused
                elif key in (ord("a"), ord("A")):
                    self.step(-1)
                elif key in (ord("d"), ord("D")):
                    self.step(1)
                elif key in (ord("j"), ord("J")):
                    self.step(-max(1, int(round(self.fps))))
                elif key in (ord("l"), ord("L")):
                    self.step(max(1, int(round(self.fps))))
                elif key in (ord("k"), ord("K")):
                    self.paused = True
                    self.save_keyframe()
                elif key in (ord("r"), ord("R")):
                    self.paused = True
                    self.draw_roi()
                elif key in (ord("h"), ord("H")):
                    self.paused = True
                    self.add_anchor_to_selected_roi()
                elif ord("1") <= key <= ord("9"):
                    self.paused = True
                    self.select_roi_by_number(key - ord("0"))
                elif key in (ord("["), ord(",")):
                    self.paused = True
                    self.select_roi_by_offset(-1)
                elif key in (ord("]"), ord(".")):
                    self.paused = True
                    self.select_roi_by_offset(1)
                elif not self.paused:
                    self.read_next_frame()
        finally:
            self.close()


def apply_to_site(draft_path: Path, site_config_path: Path) -> None:
    updated, changes = apply_draft(load_yaml(site_config_path), load_yaml(draft_path))
    if not changes:
        print("No non-empty ROI polygons to apply.")
        return
    backup = site_config_path.with_name(site_config_path.name + f".bak_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    backup.write_text(site_config_path.read_text(encoding="utf-8"), encoding="utf-8")
    dump_yaml(site_config_path, updated)
    print("Applied ROI memory:")
    for change in changes:
        print(f"  - {change}")
    print(f"site_config={site_config_path}")
    print(f"backup={backup}")


def main() -> None:
    args = parse_args()
    run_dir = resolve_run_dir(args.run_dir)
    video_path = run_dir / args.video_name
    if not video_path.exists():
        raise SystemExit(f"video not found: {video_path}")
    info = video_info(video_path)
    draft_path = ensure_draft(args, run_dir, info)
    print(f"video={video_path}")
    print(f"draft={draft_path}")
    print("Controls: SPACE play/pause, K save keyframe, R draw ROI, Q finish.")
    freeze_start = detect_sampled_freeze_start(video_path, int(info["frames"] or 0))
    if freeze_start is not None:
        seconds = freeze_start / float(info["fps"] or 15.0)
        print(f"WARNING: sampled video appears frozen after frame {freeze_start} (~{seconds:.1f}s).")
        print("Use frames before that point, or re-record after deploying the updated recorder.")
    marker = InteractiveMarker(args, video_path, draft_path, info, freeze_start)
    marker.run()
    if args.apply:
        apply_to_site(draft_path, Path(args.site_config))
    print("ROI marking done.")


if __name__ == "__main__":
    main()
