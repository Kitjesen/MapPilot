#!/usr/bin/env python3
"""Serve a read-only YOLO11n preview from the LingTu camera snapshot API."""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import cv2
import numpy as np

from perception.detection.bpu_detector import BPUDetector

LOGGER = logging.getLogger("yolo11n-preview")
BOUNDARY = b"lingtu-yolo-frame"


@dataclass(frozen=True)
class PersonLockResult:
    state: str
    track_id: int | None
    bbox: tuple[int, int, int, int] | None
    score: float
    lost_frames: int


class PersonLockTracker:
    """Dependency-free single-person lock for a visual-only demo.

    This intentionally does not claim ReID behavior. It keeps the selected
    person while detections overlap or move within a plausible local radius,
    then reports a bounded LOST state before allowing a fresh lock.
    """

    def __init__(self, *, max_lost_frames: int = 15, min_iou: float = 0.1) -> None:
        self.max_lost_frames = max(0, max_lost_frames)
        self.min_iou = min_iou
        self._next_track_id = 1
        self._track_id: int | None = None
        self._bbox: tuple[int, int, int, int] | None = None
        self._score = 0.0
        self._lost_frames = 0

    @staticmethod
    def _iou(
        left: tuple[int, int, int, int],
        right: tuple[int, int, int, int],
    ) -> float:
        x1 = max(left[0], right[0])
        y1 = max(left[1], right[1])
        x2 = min(left[2], right[2])
        y2 = min(left[3], right[3])
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        left_area = max(0, left[2] - left[0]) * max(0, left[3] - left[1])
        right_area = max(0, right[2] - right[0]) * max(0, right[3] - right[1])
        return intersection / max(left_area + right_area - intersection, 1)

    @staticmethod
    def _center_distance(
        left: tuple[int, int, int, int],
        right: tuple[int, int, int, int],
    ) -> float:
        left_x = (left[0] + left[2]) / 2.0
        left_y = (left[1] + left[3]) / 2.0
        right_x = (right[0] + right[2]) / 2.0
        right_y = (right[1] + right[3]) / 2.0
        return ((left_x - right_x) ** 2 + (left_y - right_y) ** 2) ** 0.5

    @staticmethod
    def _association_radius(bbox: tuple[int, int, int, int]) -> float:
        width = max(1, bbox[2] - bbox[0])
        height = max(1, bbox[3] - bbox[1])
        return max(48.0, ((width * width + height * height) ** 0.5) * 0.65)

    def _result(self, state: str) -> PersonLockResult:
        return PersonLockResult(
            state=state,
            track_id=self._track_id,
            bbox=self._bbox,
            score=self._score,
            lost_frames=self._lost_frames,
        )

    def update(self, detections: list[Any]) -> PersonLockResult:
        people = [detection for detection in detections if detection.label == "person"]
        if self._bbox is None:
            if not people:
                return self._result("searching")
            selected = max(people, key=lambda detection: float(detection.score))
            self._track_id = self._next_track_id
            self._next_track_id += 1
            self._bbox = tuple(int(value) for value in selected.bbox)
            self._score = float(selected.score)
            self._lost_frames = 0
            return self._result("locked")

        ranked = sorted(
            people,
            key=lambda detection: self._iou(
                self._bbox,
                tuple(int(value) for value in detection.bbox),
            ),
            reverse=True,
        )
        if ranked:
            candidate = ranked[0]
            candidate_bbox = tuple(int(value) for value in candidate.bbox)
            overlaps = self._iou(self._bbox, candidate_bbox) >= self.min_iou
            nearby = self._center_distance(self._bbox, candidate_bbox) <= self._association_radius(self._bbox)
            if overlaps or nearby:
                self._bbox = candidate_bbox
                self._score = float(candidate.score)
                self._lost_frames = 0
                return self._result("locked")

        self._lost_frames += 1
        if self._lost_frames <= self.max_lost_frames:
            return self._result("lost")

        self._track_id = None
        self._bbox = None
        self._score = 0.0
        self._lost_frames = 0
        return self._result("searching")


class PreviewState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.jpeg: bytes | None = None
        self.stats: dict[str, Any] = {
            "status": "starting",
            "frames": 0,
            "detections": [],
        }

    def update(self, jpeg: bytes, stats: dict[str, Any]) -> None:
        with self._lock:
            self.jpeg = jpeg
            self.stats = stats

    def snapshot(self) -> tuple[bytes | None, dict[str, Any]]:
        with self._lock:
            return self.jpeg, dict(self.stats)


class WebSocketFrameSource:
    """Consume fresh JPEG frames from Gateway's dedicated camera WebSocket."""

    def __init__(self, url: str, *, connector=None) -> None:
        self.url = url
        self._connector = connector
        self._connection = None

    def _connect(self):
        if self._connector is None:
            from websockets.sync.client import connect

            self._connector = connect
        self._connection = self._connector(self.url, open_timeout=5.0)

    def close(self) -> None:
        connection, self._connection = self._connection, None
        if connection is not None:
            connection.close()

    def next_frame(self) -> bytes:
        while True:
            if self._connection is None:
                self._connect()
            try:
                payload = self._connection.recv(timeout=5.0)
            except Exception:
                self.close()
                time.sleep(0.2)
                continue
            if isinstance(payload, bytes) and payload:
                return payload


def _rotate(frame: np.ndarray, rotation: str) -> np.ndarray:
    if rotation == "cw90":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if rotation == "ccw90":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if rotation == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    return frame


def _draw(
    frame: np.ndarray,
    detections: list[Any],
    person_lock: PersonLockResult | None = None,
) -> np.ndarray:
    output = frame.copy()
    for detection in detections:
        x1, y1, x2, y2 = [int(value) for value in detection.bbox]
        color = (185, 185, 185) if person_lock is not None else (0, 255, 136)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
        text = f"{detection.label} {detection.score:.2f}"
        cv2.putText(
            output,
            text,
            (x1, max(18, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    if person_lock is not None:
        state_color = {
            "locked": (0, 255, 80),
            "lost": (0, 180, 255),
            "searching": (160, 160, 160),
        }[person_lock.state]
        track_text = "-" if person_lock.track_id is None else str(person_lock.track_id)
        hud = f"PERSON LOCK  {person_lock.state.upper()}  ID:{track_text}"
        cv2.rectangle(output, (8, 8), (500, 45), (12, 12, 12), -1)
        cv2.putText(
            output,
            hud,
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            state_color,
            2,
            cv2.LINE_AA,
        )
        if person_lock.bbox is not None:
            x1, y1, x2, y2 = person_lock.bbox
            cv2.rectangle(output, (x1, y1), (x2, y2), state_color, 4)
            cv2.putText(
                output,
                f"LOCK #{track_text}",
                (x1, min(output.shape[0] - 8, y2 + 24)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                state_color,
                2,
                cv2.LINE_AA,
            )
    return output


def _worker(args: argparse.Namespace, state: PreviewState) -> None:
    detector = BPUDetector(
        confidence=args.confidence,
        iou_threshold=args.iou,
        model_path=args.model,
        max_detections=args.max_detections,
        min_box_size_px=args.min_box_size,
    )
    detector.load_model()
    source = WebSocketFrameSource(args.camera_ws_url)
    person_tracker = PersonLockTracker(max_lost_frames=args.lost_frames) if args.person_lock else None
    detection_classes = "person" if args.person_lock else args.classes
    interval = 1.0 / max(args.fps, 0.1)
    frame_count = 0
    try:
        while True:
            loop_started = time.perf_counter()
            try:
                payload = source.next_frame()
                frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise RuntimeError("snapshot JPEG decode failed")
                frame = _rotate(frame, args.rotate)
                inference_started = time.perf_counter()
                detections = detector.detect(frame, detection_classes)
                inference_ms = (time.perf_counter() - inference_started) * 1000.0
                person_lock = person_tracker.update(detections) if person_tracker else None
                annotated = _draw(frame, detections, person_lock)
                ok, encoded = cv2.imencode(
                    ".jpg",
                    annotated,
                    [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality],
                )
                if not ok:
                    raise RuntimeError("annotated JPEG encode failed")
                frame_count += 1
                state.update(
                    encoded.tobytes(),
                    {
                        "status": "running",
                        "model": args.model,
                        "frames": frame_count,
                        "inference_ms": round(inference_ms, 2),
                        "fps": round(1000.0 / max(inference_ms, 0.001), 2),
                        "source_shape": list(frame.shape),
                        "confidence": args.confidence,
                        "classes": detection_classes or "COCO-80",
                        "mode": "person_lock" if person_tracker else "detection",
                        "lock_state": person_lock.state if person_lock else None,
                        "locked_track_id": person_lock.track_id if person_lock else None,
                        "lost_frames": person_lock.lost_frames if person_lock else 0,
                        "detections": [
                            {
                                "label": detection.label,
                                "score": round(float(detection.score), 4),
                                "bbox": [int(value) for value in detection.bbox],
                            }
                            for detection in detections
                        ],
                        "ts": time.time(),
                    },
                )
            except Exception as exc:
                LOGGER.warning("preview frame failed: %s", exc)
                jpeg, stats = state.snapshot()
                stats.update({"status": "error", "error": str(exc), "ts": time.time()})
                if jpeg is not None:
                    state.update(jpeg, stats)
            remaining = interval - (time.perf_counter() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        source.close()
        detector.shutdown()


def _handler(state: PreviewState):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: object) -> None:
            return

        def do_GET(self) -> None:
            if self.path == "/":
                body = (
                    "<!doctype html><meta charset='utf-8'><title>LingTu YOLO11n Person Lock</title>"
                    "<style>body{margin:0;background:#080b12;color:#d7ffe8;font:14px monospace}"
                    "header{padding:10px 16px}img{display:block;max-width:100vw;max-height:calc(100vh - 44px);margin:auto}"
                    "</style><header>LingTu · Orbbec · YOLO11n · 视觉锁定（不驱动机器人） · "
                    "<a href='/stats.json' style='color:#00ff88'>stats</a></header>"
                    "<img src='/stream.mjpg' alt='YOLO11n preview'>"
                ).encode()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/stats.json":
                _, stats = state.snapshot()
                body = json.dumps(stats, ensure_ascii=False).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path == "/frame.jpg":
                jpeg, _ = state.snapshot()
                if jpeg is None:
                    self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "No frame available")
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
                return

            if self.path == "/stream.mjpg":
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type",
                    f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}",
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last_frame = -1
                while True:
                    jpeg, stats = state.snapshot()
                    frame_number = int(stats.get("frames", 0))
                    if jpeg is None or frame_number == last_frame:
                        time.sleep(0.02)
                        continue
                    last_frame = frame_number
                    try:
                        self.wfile.write(b"--" + BOUNDARY + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        return

            self.send_error(HTTPStatus.NOT_FOUND)

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5052)
    parser.add_argument("--camera-ws-url", default="ws://127.0.0.1:5050/ws/camera")
    parser.add_argument(
        "--model",
        default="/opt/hobot/model/s100/basic/yolo11n_detect_nashe_640x640_nv12.hbm",
    )
    parser.add_argument("--classes", default="", help="Dot-separated COCO labels; empty enables all 80 classes")
    parser.add_argument("--confidence", type=float, default=0.45)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-detections", type=int, default=32)
    parser.add_argument("--min-box-size", type=int, default=10)
    parser.add_argument(
        "--person-lock",
        action="store_true",
        help="Auto-lock one detected person for a motion-free demo",
    )
    parser.add_argument(
        "--lost-frames",
        type=int,
        default=15,
        help="Frames to retain a lost visual lock",
    )
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--rotate", choices=("none", "cw90", "ccw90", "180"), default="cw90")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    state = PreviewState()
    threading.Thread(target=_worker, args=(args, state), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.port), _handler(state))
    LOGGER.info("YOLO11n preview listening on http://%s:%d", args.host, args.port)
    server.serve_forever()


if __name__ == "__main__":
    main()
