"""Fixed-person TensorRT YOLO detector for the robot-side runtime."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from perception.detection.detector_base import Detection2D
from perception.tracking.native_byte_tracker import NativeByteTracker


@dataclass
class _TrackedDetection(Detection2D):
    track_id: int | None = None


class _TrackerInput:
    def __init__(self, detections: list[Detection2D]) -> None:
        self.conf = np.asarray([item.score for item in detections], dtype=np.float32)
        self.cls = np.zeros(len(detections), dtype=np.float32)
        self.xyxy = (
            np.stack([item.bbox for item in detections]).astype(np.float32, copy=False)
            if detections
            else np.empty((0, 4), dtype=np.float32)
        )
        self._orig_indices = np.arange(len(detections), dtype=np.int32)


class YoloTensorRTDetector:
    """Run one fixed-shape person model and preserve native track IDs."""

    INPUT_SIZE = 640
    OUTPUT_SHAPE = (5, 8400)

    def __init__(
        self,
        model_path: str,
        *,
        confidence: float = 0.3,
        iou_threshold: float = 0.45,
        max_detections: int = 64,
        min_box_size_px: int = 12,
    ) -> None:
        if not model_path:
            raise ValueError("YOLO TensorRT model_path is required")
        self.model_path = model_path
        self.confidence = float(confidence)
        self.iou_threshold = float(iou_threshold)
        self.max_detections = int(max_detections)
        self.min_box_size_px = int(min_box_size_px)
        self._library = None
        self._handle = None
        self._tracker = NativeByteTracker(
            track_high_thresh=self.confidence,
            track_low_thresh=min(0.1, self.confidence),
            new_track_thresh=self.confidence,
        )

    def load_model(self) -> None:
        if self._handle is not None:
            return
        engine_path = Path(self.model_path)
        if engine_path.suffix != ".engine" or not engine_path.is_file():
            raise RuntimeError(f"YOLO TensorRT engine is missing: {engine_path}")
        library_path = engine_path.with_name("liblingtu_yolo_trt.so")
        if not library_path.is_file():
            raise RuntimeError(f"YOLO TensorRT library is missing: {library_path}")
        library = ctypes.CDLL(str(library_path))
        library.yolo_create.argtypes = [ctypes.c_char_p]
        library.yolo_create.restype = ctypes.c_void_p
        library.yolo_infer.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        library.yolo_infer.restype = ctypes.c_int
        library.yolo_destroy.argtypes = [ctypes.c_void_p]
        library.yolo_destroy.restype = None
        handle = library.yolo_create(str(engine_path).encode())
        if not handle:
            raise RuntimeError("YOLO TensorRT engine could not be loaded")
        self._library = library
        self._handle = handle

    def detect(self, bgr: np.ndarray, text_prompt: str) -> list[Detection2D]:
        if not self._requests_person(text_prompt):
            return []
        if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
            raise ValueError("YOLO TensorRT requires an HxWx3 uint8 BGR image")
        self.load_model()
        pixels, scale, left, top = self._letterbox(bgr)
        output = np.empty(self.OUTPUT_SHAPE, dtype=np.float32)
        status = self._library.yolo_infer(
            self._handle,
            pixels.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
            output.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
        )
        if status != 0:
            raise RuntimeError(f"YOLO TensorRT inference failed: {status}")
        detections = self._postprocess(
            output,
            image_width=int(bgr.shape[1]),
            image_height=int(bgr.shape[0]),
            scale=scale,
            left=left,
            top=top,
        )
        tracks = self._tracker.update(_TrackerInput(detections), bgr)
        tracked: list[Detection2D] = []
        for row in tracks:
            detection_index = int(row[7])
            source = detections[detection_index]
            tracked.append(
                _TrackedDetection(
                    bbox=np.asarray(row[:4], dtype=np.float32),
                    score=float(row[5]),
                    label=source.label,
                    class_id=source.class_id,
                    track_id=int(row[4]),
                )
            )
        return tracked

    def shutdown(self) -> None:
        if self._handle is not None:
            self._library.yolo_destroy(self._handle)
            self._handle = None
        self._library = None
        self._tracker.reset()

    @staticmethod
    def _requests_person(text_prompt: str) -> bool:
        return "person" in {part.strip().casefold() for part in text_prompt.split(".")}

    @classmethod
    def _letterbox(cls, bgr: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        height, width = bgr.shape[:2]
        scale = min(cls.INPUT_SIZE / height, cls.INPUT_SIZE / width)
        resized_width = int(round(width * scale))
        resized_height = int(round(height * scale))
        resized = cv2.resize(bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        horizontal = (cls.INPUT_SIZE - resized_width) / 2.0
        vertical = (cls.INPUT_SIZE - resized_height) / 2.0
        left = int(round(horizontal - 0.1))
        right = int(round(horizontal + 0.1))
        top = int(round(vertical - 0.1))
        bottom = int(round(vertical + 0.1))
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        rgb = padded[:, :, ::-1].astype(np.float32) / 255.0
        return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None]), scale, left, top

    def _postprocess(
        self,
        output: np.ndarray,
        *,
        image_width: int,
        image_height: int,
        scale: float,
        left: int,
        top: int,
    ) -> list[Detection2D]:
        scores = output[4]
        indices = np.flatnonzero(
            np.isfinite(scores) & (scores >= self._tracker.track_low_thresh)
        )
        if len(indices) == 0:
            return []
        xywh = output[:4, indices].T
        boxes = np.column_stack(
            (
                xywh[:, 0] - xywh[:, 2] / 2.0,
                xywh[:, 1] - xywh[:, 3] / 2.0,
                xywh[:, 0] + xywh[:, 2] / 2.0,
                xywh[:, 1] + xywh[:, 3] / 2.0,
            )
        )
        boxes[:, [0, 2]] = (boxes[:, [0, 2]] - left) / scale
        boxes[:, [1, 3]] = (boxes[:, [1, 3]] - top) / scale
        boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, image_width)
        boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, image_height)
        finite = np.isfinite(boxes).all(axis=1)
        size = (boxes[:, 2] - boxes[:, 0] >= self.min_box_size_px) & (
            boxes[:, 3] - boxes[:, 1] >= self.min_box_size_px
        )
        boxes = boxes[finite & size]
        filtered_scores = scores[indices][finite & size]
        keep = self._nms(boxes, filtered_scores)
        return [
            Detection2D(
                bbox=boxes[index].astype(np.float32, copy=True),
                score=float(filtered_scores[index]),
                label="person",
                class_id=0,
            )
            for index in keep[: self.max_detections]
        ]

    def _nms(self, boxes: np.ndarray, scores: np.ndarray) -> list[int]:
        order = np.argsort(-scores, kind="stable")
        keep: list[int] = []
        while len(order):
            current = int(order[0])
            keep.append(current)
            if len(order) == 1:
                break
            remaining = order[1:]
            top_left = np.maximum(boxes[current, :2], boxes[remaining, :2])
            bottom_right = np.minimum(boxes[current, 2:], boxes[remaining, 2:])
            intersection_wh = np.maximum(0.0, bottom_right - top_left)
            intersection = intersection_wh[:, 0] * intersection_wh[:, 1]
            current_area = np.prod(np.maximum(0.0, boxes[current, 2:] - boxes[current, :2]))
            remaining_area = np.prod(
                np.maximum(0.0, boxes[remaining, 2:] - boxes[remaining, :2]), axis=1
            )
            union = current_area + remaining_area - intersection
            iou = np.divide(intersection, union, out=np.zeros_like(intersection), where=union > 0)
            order = remaining[iou <= self.iou_threshold]
        return keep
