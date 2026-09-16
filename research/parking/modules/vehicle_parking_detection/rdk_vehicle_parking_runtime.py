from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from .alarm_store import AlarmStore
from .config import (
    DEFAULT_EVENT_NAME,
    DEFAULT_POINT_ID,
    camera_config,
    load_yaml,
    point_location,
    roi_configs,
    runtime_config,
)
from .models import Detection
from .parking_engine import VehicleParkingEngine
from .scene_matching import SceneMatcher, scene_anchors_for_roi
from .tracker import IouTracker


CLASSES = ["person", "vehicle", "trash_bin", "garbage", "fire", "smoke"]
ROTATIONS = {
    "none": None,
    "cw90": cv2.ROTATE_90_CLOCKWISE,
    "ccw90": cv2.ROTATE_90_COUNTERCLOCKWISE,
    "rot180": cv2.ROTATE_180,
}


class LatestFrameSubscriber(Node):
    def __init__(self, topic: str, msg_type: str):
        super().__init__("vehicle_parking_detection")
        self.latest_frame: np.ndarray | None = None
        self.frame_id = 0
        self.received_at = 0.0
        if msg_type == "sensor_msgs/msg/CompressedImage":
            self.subscription = self.create_subscription(CompressedImage, topic, self._compressed_cb, 1)
        elif msg_type == "sensor_msgs/msg/Image":
            self.subscription = self.create_subscription(Image, topic, self._image_cb, 1)
        else:
            raise RuntimeError(f"unsupported topic type: {msg_type}")

    def _compressed_cb(self, msg: CompressedImage) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frame = frame
            self.frame_id += 1
            self.received_at = time.time()

    def _image_cb(self, msg: Image) -> None:
        height = int(msg.height)
        width = int(msg.width)
        encoding = str(msg.encoding).lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in {"bgr8", "rgb8"}:
            frame = data.reshape((height, int(msg.step)))[:, : width * 3].reshape((height, width, 3))
            if encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding in {"mono8", "8uc1"}:
            frame = data.reshape((height, int(msg.step)))[:, :width]
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            raise RuntimeError(f"unsupported raw image encoding: {msg.encoding}")
        self.latest_frame = frame.copy()
        self.frame_id += 1
        self.received_at = time.time()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone RDK vehicle no-parking detector.")
    parser.add_argument("--mode", choices=["record", "detect"], default="detect")
    parser.add_argument("--config", default="configs/site_rois.yaml")
    parser.add_argument("--point-id", default=DEFAULT_POINT_ID)
    parser.add_argument("--topic", default="")
    parser.add_argument("--type", default="")
    parser.add_argument("--hbm", default="")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--view", choices=["auto", "top", "bottom", "left", "right", "full"], default="")
    parser.add_argument("--rotation", choices=sorted(ROTATIONS), default="")
    parser.add_argument("--record-fps", type=float, default=0.0)
    parser.add_argument("--process-interval", type=float, default=0.0)
    parser.add_argument("--infer-interval", type=float, default=0.0)
    parser.add_argument("--input-format", choices=["rgb_u8", "bgr_u8", "rgb_i8_centered", "rgb_i8_norm127"], default="")
    parser.add_argument("--input-height", type=int, default=0)
    parser.add_argument("--input-width", type=int, default=0)
    parser.add_argument("--hbm-backend", choices=["hbm_runtime"], default="hbm_runtime")
    parser.add_argument("--conf", type=float, default=-1.0)
    parser.add_argument("--iou", type=float, default=-1.0)
    parser.add_argument("--detection-ttl", type=float, default=0.0)
    parser.add_argument("--track-iou-threshold", type=float, default=-1.0)
    parser.add_argument("--track-center-distance-ratio", type=float, default=-1.0)
    parser.add_argument("--track-max-missed", type=int, default=-1)
    parser.add_argument("--track-max-age-seconds", type=float, default=-1.0)
    return parser.parse_args()


def query_topic_type(topic: str) -> str:
    proc = subprocess.run(["ros2", "topic", "type", topic], text=True, capture_output=True, check=False)
    return proc.stdout.strip().splitlines()[0] if proc.stdout.strip() else ""


def choose_model_view(frame: np.ndarray, view: str) -> np.ndarray:
    height, width = frame.shape[:2]
    if view == "full":
        return frame
    if view == "top":
        return frame[: height // 2, :] if height >= width else frame
    if view == "bottom":
        return frame[height // 2 :, :] if height >= width else frame
    if view == "left":
        return frame[:, : width // 2] if width >= height else frame
    if view == "right":
        return frame[:, width // 2 :] if width >= height else frame
    if height == width and height >= 1000:
        return frame[: height // 2, :]
    if height >= int(width * 1.8):
        return frame[: height // 2, :]
    if width >= int(height * 2.4):
        return frame[:, : width // 2]
    return frame


def rotate_image(image: np.ndarray, rotation: str) -> np.ndarray:
    code = ROTATIONS[rotation]
    return image.copy() if code is None else cv2.rotate(image, code)


def letterbox(image: np.ndarray, target_height: int, target_width: int, color: int = 114) -> tuple[np.ndarray, dict[str, Any]]:
    height, width = image.shape[:2]
    scale = min(target_height / height, target_width / width)
    new_width = int(round(width * scale))
    new_height = int(round(height * scale))
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((target_height, target_width, 3), color, dtype=np.uint8)
    pad_left = (target_width - new_width) // 2
    pad_top = (target_height - new_height) // 2
    canvas[pad_top : pad_top + new_height, pad_left : pad_left + new_width] = resized
    return canvas, {
        "scale": float(scale),
        "pad_left": int(pad_left),
        "pad_top": int(pad_top),
        "input_width": int(width),
        "input_height": int(height),
    }


def xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    out = np.empty_like(boxes)
    out[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.0
    out[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.0
    out[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.0
    out[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.0
    return out


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[order[1:]] - inter
        iou = inter / np.maximum(union, 1e-9)
        order = order[1:][iou <= iou_threshold]
    return keep


def decode_detections(output: np.ndarray, meta: dict[str, Any], conf: float, iou: float) -> list[dict[str, Any]]:
    channels = 4 + len(CLASSES)
    if output.shape == (1, channels, 8400) or output.shape == (1, channels, 4200):
        pred = output[0].T
    elif output.ndim == 3 and output.shape[0] == 1 and output.shape[2] == channels:
        pred = output[0]
    elif output.ndim == 2 and output.shape[0] == channels:
        pred = output.T
    elif output.ndim == 2 and output.shape[1] == channels:
        pred = output
    else:
        pred = output.reshape(channels, -1).T

    pred = pred.astype(np.float32)
    boxes = xywh_to_xyxy(pred[:, :4])
    class_scores = pred[:, 4 : 4 + len(CLASSES)]
    class_ids = class_scores.argmax(axis=1)
    scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
    mask = scores >= conf
    boxes = boxes[mask]
    scores = scores[mask]
    class_ids = class_ids[mask]

    detections: list[dict[str, Any]] = []
    for class_id in sorted(set(int(value) for value in class_ids.tolist())):
        class_name = CLASSES[class_id]
        if class_name != "vehicle":
            continue
        idx = np.where(class_ids == class_id)[0]
        keep = nms(boxes[idx], scores[idx], iou)
        for local_index in keep:
            i = int(idx[local_index])
            box = boxes[i].copy()
            box[[0, 2]] = (box[[0, 2]] - float(meta["pad_left"])) / float(meta["scale"])
            box[[1, 3]] = (box[[1, 3]] - float(meta["pad_top"])) / float(meta["scale"])
            box[[0, 2]] = np.clip(box[[0, 2]], 0, float(meta["input_width"]))
            box[[1, 3]] = np.clip(box[[1, 3]], 0, float(meta["input_height"]))
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": float(scores[i]),
                    "x1": float(box[0]),
                    "y1": float(box[1]),
                    "x2": float(box[2]),
                    "y2": float(box[3]),
                }
            )
    detections.sort(key=lambda item: item["confidence"], reverse=True)
    return detections


def build_model_input(letterboxed_bgr: np.ndarray, input_format: str) -> np.ndarray:
    if input_format == "rgb_u8":
        return cv2.cvtColor(letterboxed_bgr, cv2.COLOR_BGR2RGB).astype(np.uint8)
    if input_format == "bgr_u8":
        return letterboxed_bgr.astype(np.uint8)
    rgb = cv2.cvtColor(letterboxed_bgr, cv2.COLOR_BGR2RGB)
    if input_format == "rgb_i8_centered":
        return (rgb.astype(np.int16) - 128).astype(np.int8)
    return np.round(rgb.astype(np.float32) / 255.0 * 127.0).astype(np.int8)


def first_numpy_output(value) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if isinstance(value, dict):
        for item in value.values():
            try:
                return first_numpy_output(item)
            except TypeError:
                continue
    if isinstance(value, (list, tuple)):
        for item in value:
            try:
                return first_numpy_output(item)
            except TypeError:
                continue
    if hasattr(value, "numpy"):
        return value.numpy()
    raise TypeError(f"no numpy output tensor found in type={type(value).__name__}")


def numeric_shape(value) -> tuple[int, ...] | None:
    if value is None:
        return None
    if isinstance(value, dict):
        for key in ("shape", "dims", "input_shape"):
            if key in value:
                shape = numeric_shape(value[key])
                if shape:
                    return shape
        for item in value.values():
            shape = numeric_shape(item)
            if shape:
                return shape
        return None
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if isinstance(value, (list, tuple)):
        try:
            shape = tuple(int(item) for item in value)
        except (TypeError, ValueError):
            return None
        return shape if shape and all(item > 0 for item in shape) else None
    return None


def runtime_input_array(model_input: np.ndarray, expected_shape) -> np.ndarray:
    arr = np.ascontiguousarray(model_input)
    shape = numeric_shape(expected_shape)
    if shape is None or int(np.prod(shape)) != int(arr.size):
        return arr
    return np.ascontiguousarray(arr.reshape(shape))


class HbmRuntimeBackend:
    def __init__(self, hbm: Path):
        from hbm_runtime import HB_HBMRuntime

        self.model = HB_HBMRuntime(str(hbm))
        self.model_name = None
        self.input_name = None
        try:
            model_names = list(self.model.model_names)
            self.model_name = str(model_names[0]) if model_names else None
        except Exception:
            self.model_name = None
        self.input_shape = self._first_input_shape()

    def _first_input_shape(self):
        for attr in ("input_shapes", "inputs"):
            try:
                value = getattr(self.model, attr)
            except Exception:
                continue
            if isinstance(value, dict) and value:
                self.input_name = str(next(iter(value.keys())))
                return next(iter(value.values()))
            if isinstance(value, (list, tuple)) and value:
                first = value[0]
                if isinstance(first, dict):
                    for name_key in ("name", "input_name"):
                        if name_key in first:
                            self.input_name = str(first[name_key])
                    for candidate in ("shape", "dims"):
                        if candidate in first:
                            return first[candidate]
                return first
        return None

    def infer(self, model_input: np.ndarray) -> tuple[float, np.ndarray]:
        input_array = runtime_input_array(model_input, self.input_shape)
        started = time.time()
        try:
            outputs = self.model.run(input_array)
        except Exception:
            if not self.input_name:
                raise
            outputs = self.model.run({self.input_name: input_array})
        elapsed_ms = (time.time() - started) * 1000.0
        if self.model_name and isinstance(outputs, dict) and self.model_name in outputs:
            return elapsed_ms, first_numpy_output(outputs[self.model_name])
        return elapsed_ms, first_numpy_output(outputs)


def detection_from_dict(item: dict[str, Any]) -> Detection:
    return Detection(
        class_id=int(item.get("class_id", -1)),
        class_name=str(item.get("class_name", "")),
        confidence=float(item.get("confidence", 0.0)),
        box=(float(item["x1"]), float(item["y1"]), float(item["x2"]), float(item["y2"])),
        track_id=item.get("track_id"),
    )


def detection_to_row(det: Detection) -> dict[str, Any]:
    x1, y1, x2, y2 = det.box
    return {
        "class_id": det.class_id,
        "class_name": det.class_name,
        "confidence": det.confidence,
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "track_id": det.track_id,
    }


def draw_overlay(
    image: np.ndarray,
    detections: list[Detection],
    rois,
    status: str,
) -> np.ndarray:
    out = image.copy()
    for roi in rois:
        if len(roi.polygon) >= 3:
            pts = np.array(roi.polygon, dtype=np.int32)
            cv2.polylines(out, [pts], True, (0, 255, 0), 2)
            x, y = roi.polygon[0]
            cv2.putText(out, roi.location, (max(5, x), max(24, y + 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    for det in detections:
        x1, y1, x2, y2 = [int(round(value)) for value in det.box]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 255), 2)
        label = f"vehicle {det.confidence:.2f}"
        if det.track_id is not None:
            label += f" #{det.track_id}"
        cv2.putText(out, label, (x1, max(22, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
    cv2.putText(out, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (50, 255, 50), 2, cv2.LINE_AA)
    return out


def open_video_writer(path: Path, fps: float, frame_shape: tuple[int, ...]) -> cv2.VideoWriter:
    height, width = frame_shape[:2]
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise SystemExit(f"failed to open VideoWriter: {path}")
    return writer


def write_json(path: Path, payload: dict[str, Any] | list[Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def pick(value, default):
    if value in {"", None}:
        return default
    if isinstance(value, (int, float)) and value <= 0:
        return default
    return value


def main() -> int:
    args = parse_args()
    site_config = load_yaml(args.config)
    camera = camera_config(site_config)
    runtime = runtime_config(site_config)
    point_id = args.point_id
    rois = roi_configs(site_config, point_id)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    topic = str(args.topic or camera["topic"])
    view = str(args.view or camera["view"])
    rotation = str(args.rotation or camera["rotation"])
    record_fps = float(pick(args.record_fps, runtime["record_fps"]))
    infer_interval = float(pick(args.infer_interval, runtime["infer_interval"]))
    process_interval = float(pick(args.process_interval, runtime["process_interval"]))
    input_format = str(args.input_format or runtime["input_format"])
    input_height = int(pick(args.input_height, runtime["input_height"]))
    input_width = int(pick(args.input_width, runtime["input_width"]))
    conf = float(pick(args.conf, runtime["conf"]))
    iou = float(pick(args.iou, runtime["iou"]))
    detection_ttl = float(pick(args.detection_ttl, runtime["detection_ttl"]))
    vehicle_class = str(runtime.get("vehicle_class", "vehicle"))
    event_name = str(runtime.get("event_name", DEFAULT_EVENT_NAME))
    scene_match_enabled = bool(runtime.get("scene_match_enabled", True))
    scene_match_min_score = float(runtime.get("scene_match_min_score", 0.65))
    scene_match_min_margin = float(runtime.get("scene_match_min_margin", 0.08))
    scene_match_hold_seconds = float(runtime.get("scene_match_hold_seconds", 8.0))

    record_fps = max(1.0, min(30.0, record_fps))
    infer_interval = max(0.0, infer_interval or process_interval)
    raw_video_path = output_dir / "raw_rotated.mp4"
    annotated_video_path = output_dir / "annotated_detect.mp4"

    backend = None
    if args.mode == "detect":
        if not args.hbm:
            raise SystemExit("--hbm is required in detect mode")
        backend = HbmRuntimeBackend(Path(args.hbm))

    tracker = IouTracker(
        iou_threshold=float(pick(args.track_iou_threshold, runtime["track_iou_threshold"])),
        center_distance_ratio_threshold=float(pick(args.track_center_distance_ratio, runtime["track_center_distance_ratio"])),
        max_missed=int(pick(args.track_max_missed, runtime["track_max_missed"])),
        max_age_seconds=float(pick(args.track_max_age_seconds, runtime["track_max_age_seconds"])),
    )
    engine = VehicleParkingEngine(
        rois=rois,
        event_name=event_name,
        vehicle_class=vehicle_class,
        min_confidence=conf,
    )
    scene_matcher = SceneMatcher(
        rois,
        enabled=scene_match_enabled,
        min_score=scene_match_min_score,
        min_margin=scene_match_min_margin,
        hold_seconds=scene_match_hold_seconds,
    )
    alarm_store = AlarmStore(output_dir)

    msg_type = args.type.strip() or query_topic_type(topic)
    if msg_type not in {"sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/Image"}:
        raise SystemExit(f"unsupported or missing topic type for {topic}: {msg_type}")

    print("=== vehicle parking detection runtime ===", flush=True)
    print(f"mode={args.mode}", flush=True)
    print(f"output_dir={output_dir}", flush=True)
    print(f"topic={topic} type={msg_type}", flush=True)
    print(f"point_id={point_id} location={point_location(site_config, point_id)}", flush=True)
    print(f"roi_count={len(rois)} configured_roi_with_polygon={sum(1 for roi in rois if roi.polygon)}", flush=True)
    print(f"vehicle_confidence_threshold={conf} min_roi_dwell_seconds={min((roi.dwell_seconds for roi in rois), default=0.0)}", flush=True)
    print(
        f"scene_match_enabled={scene_match_enabled} scene_anchors={sum(len(scene_anchors_for_roi(roi)) for roi in rois)} "
        f"scene_match_min_score={scene_match_min_score} scene_match_min_margin={scene_match_min_margin} "
        f"scene_match_hold_seconds={scene_match_hold_seconds}",
        flush=True,
    )
    print(f"record_fps={record_fps} infer_interval={infer_interval} target_infer_fps={1.0 / infer_interval if infer_interval > 0 else 'unlimited'}", flush=True)
    print(f"input={input_height}x{input_width} {input_format} hbm_backend={args.hbm_backend}", flush=True)

    rclpy.init()
    node = LatestFrameSubscriber(topic, msg_type)
    stop_event = threading.Event()
    frame_lock = threading.Lock()
    result_lock = threading.Lock()
    frame_buffer: dict[str, Any] = {"frame": None, "frame_id": -1, "received_at": 0.0}
    result: dict[str, Any] = {
        "frame_index": 0,
        "detections": [],
        "alarm_count": 0,
        "infer_ms": 0.0,
        "decode_ms": 0.0,
        "tracker_ms": 0.0,
        "event_ms": 0.0,
        "loop_ms": 0.0,
        "completed_at": 0.0,
        "error": "",
        "active_scene_id": "",
        "active_scene_score": 0.0,
        "active_scene_margin": 0.0,
        "active_scene_reason": "",
        "active_roi_count": len(rois),
        "active_roi_ids": [roi.id for roi in rois],
    }
    metrics = {
        "captured_topic_frames": 0,
        "video_frames_written": 0,
        "annotated_video_frames_written": 0,
        "skipped_stale_record_ticks": 0,
        "latest_source_frame_id": -1,
        "source_stale_seconds": 0.0,
        "processed_inference_frames": 0,
        "alarm_count": 0,
    }
    start_time = time.time()

    def status_payload(now: float) -> dict[str, Any]:
        elapsed = max(1e-9, now - start_time)
        with result_lock:
            current = dict(result)
        return {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": args.mode,
            "point_id": point_id,
            "location": point_location(site_config, point_id),
            "roi_count": len(rois),
            "configured_roi_with_polygon": sum(1 for roi in rois if roi.polygon),
            "vehicle_confidence_threshold": conf,
            "min_roi_dwell_seconds": min((roi.dwell_seconds for roi in rois), default=0.0),
            "scene_match_enabled": scene_match_enabled,
            "scene_anchor_count": sum(len(scene_anchors_for_roi(roi)) for roi in rois),
            "scene_match_min_score": scene_match_min_score,
            "scene_match_min_margin": scene_match_min_margin,
            "scene_match_hold_seconds": scene_match_hold_seconds,
            "active_scene_id": current.get("active_scene_id", ""),
            "active_scene_score": current.get("active_scene_score", 0.0),
            "active_scene_margin": current.get("active_scene_margin", 0.0),
            "active_scene_reason": current.get("active_scene_reason", ""),
            "active_roi_count": current.get("active_roi_count", len(rois)),
            "record_fps": record_fps,
            "target_infer_fps": 1.0 / infer_interval if infer_interval > 0 else 0,
            "infer_fps": metrics["processed_inference_frames"] / elapsed,
            "capture_fps": metrics["captured_topic_frames"] / elapsed,
            "recorded_video_fps": metrics["video_frames_written"] / elapsed,
            "video_frames_written": metrics["video_frames_written"],
            "skipped_stale_record_ticks": metrics["skipped_stale_record_ticks"],
            "latest_source_frame_id": metrics["latest_source_frame_id"],
            "source_stale_seconds": metrics["source_stale_seconds"],
            "processed_inference_frames": metrics["processed_inference_frames"],
            "alarm_count": metrics["alarm_count"],
            "latest_frame_index": current.get("frame_index", 0),
            "detections": len(current.get("detections") or []),
            "infer_ms": current.get("infer_ms", 0.0),
            "decode_ms": current.get("decode_ms", 0.0),
            "tracker_ms": current.get("tracker_ms", 0.0),
            "event_ms": current.get("event_ms", 0.0),
            "loop_ms": current.get("loop_ms", 0.0),
            "error": current.get("error", ""),
            "raw_video": str(raw_video_path),
            "annotated_video": str(annotated_video_path),
            "annotated_video_frames_written": metrics["annotated_video_frames_written"],
            "alarm_events_jsonl": str(output_dir / "alarm_events.jsonl"),
            "latest_alarm_event": str(output_dir / "latest_alarm_event.json"),
        }

    def write_status() -> None:
        write_json(output_dir / "latest_status.json", status_payload(time.time()))

    def inference_worker() -> None:
        assert backend is not None
        processed = 0
        last_frame_id = -1
        next_infer_at = 0.0
        while not stop_event.is_set():
            now = time.time()
            if now < next_infer_at:
                stop_event.wait(min(0.01, next_infer_at - now))
                continue
            with frame_lock:
                frame = frame_buffer["frame"]
                frame_id = int(frame_buffer["frame_id"])
                frame = None if frame is None else frame.copy()
            if frame is None or frame_id == last_frame_id:
                stop_event.wait(0.005)
                continue
            last_frame_id = frame_id
            processed += 1
            loop_start = time.time()
            error = ""
            detections: list[Detection] = []
            infer_ms = 0.0
            decode_ms = 0.0
            tracker_ms = 0.0
            event_ms = 0.0
            active_rois = rois
            scene_match = None
            try:
                rotated = rotate_image(choose_model_view(frame, view), rotation)
                active_rois, scene_match = scene_matcher.select(rotated)
                engine.rois = active_rois
                letterboxed, meta = letterbox(rotated, input_height, input_width)
                model_input = build_model_input(letterboxed, input_format)
                infer_ms, output = backend.infer(model_input)
                decode_start = time.time()
                raw_detections = decode_detections(output, meta, conf, iou)
                detections = [detection_from_dict(item) for item in raw_detections]
                decode_ms = (time.time() - decode_start) * 1000.0
                tracker_start = time.time()
                detections = tracker.update(detections, now=time.time())
                tracker_ms = (time.time() - tracker_start) * 1000.0
                event_start = time.time()
                triggers = engine.evaluate(detections, now=time.time())
                status = (
                    f"infer={processed} vehicles={len(detections)} alarms={len(triggers)} "
                    f"scene={scene_match.scene_id or 'none'} score={scene_match.score:.2f} "
                    f"margin={scene_match.margin:.2f} rois={len(active_rois)}"
                )
                annotated = draw_overlay(rotated, detections, active_rois, status)
                for trigger in triggers:
                    alarm_store.save(annotated, trigger)
                    metrics["alarm_count"] += 1
                event_ms = (time.time() - event_start) * 1000.0
                cv2.imwrite(str(output_dir / "latest_annotated.jpg"), annotated)
                write_json(output_dir / "latest_detections.json", [detection_to_row(det) for det in detections])
                write_json(
                    output_dir / "latest_scene_match.json",
                    {
                        "scene_id": scene_match.scene_id,
                        "score": scene_match.score,
                        "margin": scene_match.margin,
                        "matched": scene_match.matched,
                        "reason": scene_match.reason,
                        "active_roi_ids": [roi.id for roi in active_rois],
                    },
                )
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"

            loop_ms = (time.time() - loop_start) * 1000.0
            with result_lock:
                result.update(
                    {
                        "frame_index": processed,
                        "detections": [detection_to_row(det) for det in detections],
                        "alarm_count": metrics["alarm_count"],
                        "infer_ms": infer_ms,
                        "decode_ms": decode_ms,
                        "tracker_ms": tracker_ms,
                        "event_ms": event_ms,
                        "loop_ms": loop_ms,
                        "completed_at": time.time(),
                        "error": error,
                        "active_scene_id": scene_match.scene_id if scene_match is not None else "",
                        "active_scene_score": scene_match.score if scene_match is not None else 0.0,
                        "active_scene_margin": scene_match.margin if scene_match is not None else 0.0,
                        "active_scene_reason": scene_match.reason if scene_match is not None else "",
                        "active_roi_count": len(active_rois),
                        "active_roi_ids": [roi.id for roi in active_rois],
                    }
                )
            metrics["processed_inference_frames"] = processed
            write_status()
            if error:
                print(f"infer_frame={processed} error={error}", flush=True)
            else:
                print(f"infer_frame={processed} vehicles={len(detections)} infer_ms={infer_ms:.1f} loop_ms={loop_ms:.1f}", flush=True)
            next_infer_at = time.time() + infer_interval

    infer_thread = None
    if args.mode == "detect":
        infer_thread = threading.Thread(target=inference_worker, name="vehicle-parking-inference", daemon=True)
        infer_thread.start()

    writer: cv2.VideoWriter | None = None
    annotated_writer: cv2.VideoWriter | None = None
    record_period = 1.0 / record_fps
    next_record_at = time.time()
    next_latest_write_at = 0.0
    last_buffered_frame_id = -1
    last_recorded_frame_id = -1
    try:
        while rclpy.ok() and not stop_event.is_set():
            now = time.time()
            if args.seconds > 0 and now - start_time >= args.seconds:
                break

            try:
                rclpy.spin_once(node, timeout_sec=0.002)
            except ExternalShutdownException:
                print("ROS external shutdown requested; stopping runtime.", flush=True)
                break
            except Exception as exc:
                if not rclpy.ok() or stop_event.is_set():
                    print(f"ROS context closed; stopping runtime: {type(exc).__name__}: {exc}", flush=True)
                    break
                with result_lock:
                    result["error"] = f"{type(exc).__name__}: {exc}"
                write_status()
                print(
                    f"warning: rclpy spin_once failed, keeping runtime alive: {type(exc).__name__}: {exc}",
                    flush=True,
                )
                time.sleep(0.2)
                continue
            if node.latest_frame is not None and node.frame_id != last_buffered_frame_id:
                with frame_lock:
                    frame_buffer["frame"] = node.latest_frame.copy()
                    frame_buffer["frame_id"] = node.frame_id
                    frame_buffer["received_at"] = node.received_at
                metrics["captured_topic_frames"] += 1
                metrics["latest_source_frame_id"] = int(node.frame_id)
                last_buffered_frame_id = node.frame_id

            now = time.time()
            if now < next_record_at:
                continue
            with frame_lock:
                frame = frame_buffer["frame"]
                frame_id = int(frame_buffer["frame_id"])
                received_at = float(frame_buffer["received_at"] or 0.0)
                frame = None if frame is None else frame.copy()
            if frame is None:
                next_record_at = now + 0.01
                continue
            if frame_id == last_recorded_frame_id:
                metrics["skipped_stale_record_ticks"] += 1
                metrics["source_stale_seconds"] = max(0.0, now - received_at) if received_at > 0 else 0.0
                if now >= next_latest_write_at:
                    write_status()
                    next_latest_write_at = now + 1.0
                next_record_at += record_period
                if next_record_at < now - record_period:
                    next_record_at = now + record_period
                continue

            rotated = rotate_image(choose_model_view(frame, view), rotation)
            if writer is None:
                writer = open_video_writer(raw_video_path, record_fps, rotated.shape)
            writer.write(np.ascontiguousarray(rotated))
            metrics["video_frames_written"] += 1
            metrics["source_stale_seconds"] = 0.0
            last_recorded_frame_id = frame_id

            if args.mode == "detect":
                with result_lock:
                    current = dict(result)
                result_age = max(0.0, now - float(current.get("completed_at") or 0.0))
                if result_age <= max(1.0, detection_ttl * 2.0):
                    detections_for_overlay = []
                    for row in current.get("detections") or []:
                        try:
                            detections_for_overlay.append(detection_from_dict(row))
                        except Exception:
                            continue
                    active_ids = {str(item) for item in current.get("active_roi_ids") or []}
                    rois_for_overlay = [roi for roi in rois if roi.id in active_ids]
                else:
                    detections_for_overlay = []
                    rois_for_overlay = []
                overlay_status = (
                    f"src={frame_id} infer={current.get('frame_index', 0)} "
                    f"vehicles={len(detections_for_overlay)} alarms={current.get('alarm_count', 0)} "
                    f"scene={current.get('active_scene_id') or 'none'} "
                    f"score={float(current.get('active_scene_score') or 0.0):.2f} "
                    f"rois={len(rois_for_overlay)}"
                )
                annotated = draw_overlay(rotated, detections_for_overlay, rois_for_overlay, overlay_status)
                if annotated_writer is None:
                    annotated_writer = open_video_writer(annotated_video_path, record_fps, annotated.shape)
                annotated_writer.write(np.ascontiguousarray(annotated))
                metrics["annotated_video_frames_written"] += 1

            if now >= next_latest_write_at:
                cv2.imwrite(str(output_dir / "latest_source_rotated.jpg"), rotated)
                write_status()
                next_latest_write_at = now + 1.0

            next_record_at += record_period
            if next_record_at < now - record_period:
                next_record_at = now + record_period
    finally:
        stop_event.set()
        if infer_thread is not None:
            infer_thread.join(timeout=5.0)
        if writer is not None:
            writer.release()
        if annotated_writer is not None:
            annotated_writer.release()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            print(f"warning: rclpy shutdown skipped: {type(exc).__name__}: {exc}", flush=True)

    summary = status_payload(time.time())
    summary["duration_sec"] = time.time() - start_time
    write_json(output_dir / "recording_summary.json", summary)
    write_status()
    print("VEHICLE_PARKING_RUNTIME_DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
