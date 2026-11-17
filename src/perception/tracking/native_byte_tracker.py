"""Dependency-light, detector-agnostic multi-object tracking for the BPU path.

The association strategy follows the central idea from ByteTrack: associate
high-confidence detections first, then use low-confidence detections to recover
existing tracks without allowing those detections to create new tracks.  This
is a clean LingTu implementation built on NumPy and the project's existing
SciPy dependency; it does not require Ultralytics, PyTorch, CUDA, or ROS.

Reference: Zhang et al., "ByteTrack: Multi-Object Tracking by Associating Every
Detection Box" (ECCV 2022), https://github.com/FoundationVision/ByteTrack

Motion is predicted over the elapsed source time when callers supply frame
timestamps, so irregular and dropped frames do not stall the prediction. When a
frame is supplied, global camera motion is removed before association, as in
BoT-SORT's camera-motion compensation (Aharon et al., 2022,
https://github.com/NirAharon/BoT-SORT); a rotating robot otherwise moves every
box off its prediction and fragments track IDs.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment


def _iou_matrix(track_boxes: np.ndarray, detection_boxes: np.ndarray) -> np.ndarray:
    """Return pairwise IoU for two xyxy box arrays."""
    if len(track_boxes) == 0 or len(detection_boxes) == 0:
        return np.empty((len(track_boxes), len(detection_boxes)), dtype=np.float32)

    top_left = np.maximum(track_boxes[:, None, :2], detection_boxes[None, :, :2])
    bottom_right = np.minimum(track_boxes[:, None, 2:], detection_boxes[None, :, 2:])
    intersection_wh = np.maximum(0.0, bottom_right - top_left)
    intersection = intersection_wh[..., 0] * intersection_wh[..., 1]

    track_wh = np.maximum(0.0, track_boxes[:, 2:] - track_boxes[:, :2])
    detection_wh = np.maximum(0.0, detection_boxes[:, 2:] - detection_boxes[:, :2])
    track_area = track_wh[:, 0] * track_wh[:, 1]
    detection_area = detection_wh[:, 0] * detection_wh[:, 1]
    union = track_area[:, None] + detection_area[None, :] - intersection
    return np.divide(
        intersection,
        union,
        out=np.zeros_like(intersection, dtype=np.float32),
        where=union > 1e-6,
    )


@dataclass
class _NativeTrack:
    track_id: int
    bbox: np.ndarray
    score: float
    class_id: int
    det_idx: int
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float32))  # pixels/s
    unobserved_s: float = 0.0
    hits: int = 1
    is_activated: bool = True
    _observed_bbox: np.ndarray | None = field(default=None, repr=False)

    def __post_init__(self) -> None:
        if self._observed_bbox is None:
            self._observed_bbox = self.bbox.copy()

    def predict(self, dt: float) -> None:
        self.bbox = self.bbox + self.velocity * dt
        self.unobserved_s += dt
        self.is_activated = False

    def apply_camera_motion(self, transform: np.ndarray) -> None:
        """Move the box, its last observation and its velocity with the camera image."""
        rotation, translation = transform[:, :2], transform[:, 2]

        def warp(box: np.ndarray) -> np.ndarray:
            corners = box.reshape(2, 2) @ rotation.T + translation
            return np.concatenate([corners.min(axis=0), corners.max(axis=0)]).astype(np.float32)

        self.bbox = warp(self.bbox)
        self._observed_bbox = warp(self._observed_bbox)
        self.velocity = (self.velocity.reshape(2, 2) @ rotation.T).reshape(4).astype(np.float32)

    def update(self, bbox: np.ndarray, score: float, det_idx: int) -> None:
        if self.unobserved_s > 0.0:
            measured_velocity = (bbox - self._observed_bbox) / self.unobserved_s
            self.velocity = 0.7 * self.velocity + 0.3 * measured_velocity
        self.bbox = bbox.astype(np.float32, copy=True)
        self._observed_bbox = self.bbox.copy()
        self.score = float(score)
        self.det_idx = int(det_idx)
        self.unobserved_s = 0.0
        self.hits += 1
        self.is_activated = True


class _CameraMotion:
    """Global image translation between consecutive frames by phase correlation.

    Robot yaw and pitch move the whole image almost uniformly. Phase correlation
    recovers that shift beyond the range of local optical flow; the detected
    people are blanked so that their own motion does not dominate.
    """

    DOWNSCALE = 2
    MIN_RESPONSE = 0.3  # correlation peak; unrelated images measure about 0.07

    def __init__(self) -> None:
        self._previous: np.ndarray | None = None

    def estimate(self, frame: np.ndarray, boxes: np.ndarray) -> np.ndarray:
        """Return the 2x3 transform from the previous frame; identity when unknown."""
        import cv2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.resize(gray, (gray.shape[1] // self.DOWNSCALE, gray.shape[0] // self.DOWNSCALE))
        gray = gray.astype(np.float32)
        fill = float(gray.mean())
        for x1, y1, x2, y2 in (np.asarray(boxes, dtype=np.float32) / self.DOWNSCALE).astype(int):
            gray[max(0, y1):max(0, y2 + 1), max(0, x1):max(0, x2 + 1)] = fill
        transform = np.eye(2, 3, dtype=np.float32)
        if self._previous is not None and self._previous.shape == gray.shape:
            window = cv2.createHanningWindow((gray.shape[1], gray.shape[0]), cv2.CV_32F)
            (dx, dy), response = cv2.phaseCorrelate(self._previous, gray, window)
            if response >= self.MIN_RESPONSE:
                transform[:, 2] = (dx * self.DOWNSCALE, dy * self.DOWNSCALE)
        self._previous = gray
        return transform

    def reset(self) -> None:
        self._previous = None


class NativeByteTracker:
    """Small ByteTrack-style tracker with a BOTSORT-compatible update surface."""

    def __init__(
        self,
        *,
        frame_rate: int = 30,
        track_high_thresh: float = 0.25,
        track_low_thresh: float = 0.1,
        new_track_thresh: float = 0.25,
        track_buffer: int = 30,
        match_thresh: float = 0.8,
        low_match_thresh: float = 0.5,
    ) -> None:
        if not 0.0 <= track_low_thresh <= track_high_thresh <= 1.0:
            raise ValueError("expected 0 <= track_low_thresh <= track_high_thresh <= 1")
        if not 0.0 <= new_track_thresh <= 1.0:
            raise ValueError("new_track_thresh must be between 0 and 1")
        if track_buffer < 0:
            raise ValueError("track_buffer must be non-negative")

        self.track_high_thresh = float(track_high_thresh)
        self.track_low_thresh = float(track_low_thresh)
        self.new_track_thresh = float(new_track_thresh)
        self.match_thresh = float(match_thresh)
        self.low_match_thresh = float(low_match_thresh)
        self.frame_period_s = 1.0 / float(frame_rate)
        # ByteTrack keeps a lost track for `track_buffer` frames at 30 FPS.
        self.max_unobserved_s = max(0, int(round(track_buffer * frame_rate / 30.0))) * self.frame_period_s

        self.tracked_stracks: list[_NativeTrack] = []
        self._next_track_id = 1
        self._last_timestamp: float | None = None
        self._camera_motion = _CameraMotion()

    def update(self, results, frame=None, *, timestamp: float | None = None) -> np.ndarray:
        """Update tracker and return rows compatible with Ultralytics trackers.

        Each returned row is ``[x1, y1, x2, y2, track_id, score, class_id,
        original_detection_index]``. Only tracks observed in the current frame
        are returned; temporarily lost tracks remain internal for recovery.
        ``timestamp`` is the frame's source time in seconds; without it each
        update advances one nominal frame period.
        """
        boxes = np.asarray(results.xyxy, dtype=np.float32).reshape(-1, 4)
        scores = np.asarray(results.conf, dtype=np.float32).reshape(-1)
        classes = np.asarray(results.cls, dtype=np.int32).reshape(-1)
        original_indices = np.asarray(getattr(results, "_orig_indices", np.arange(len(scores))), dtype=np.int32)

        dt = self.frame_period_s
        if timestamp is not None:
            if self._last_timestamp is not None:
                dt = max(0.0, float(timestamp) - self._last_timestamp)
            self._last_timestamp = float(timestamp)
        transform = None if frame is None else self._camera_motion.estimate(frame, boxes)
        for track in self.tracked_stracks:
            track.predict(dt)
            if transform is not None:
                track.apply_camera_motion(transform)

        high_indices = np.flatnonzero(scores >= self.track_high_thresh)
        low_indices = np.flatnonzero((scores >= self.track_low_thresh) & (scores < self.track_high_thresh))

        unmatched_track_indices = list(range(len(self.tracked_stracks)))
        high_matches, unmatched_track_indices, unmatched_high = self._associate(
            unmatched_track_indices,
            high_indices.tolist(),
            boxes,
            classes,
            max_cost=self.match_thresh,
        )
        self._apply_matches(high_matches, boxes, scores, original_indices)

        low_matches, unmatched_track_indices, _ = self._associate(
            unmatched_track_indices,
            low_indices.tolist(),
            boxes,
            classes,
            max_cost=self.low_match_thresh,
        )
        self._apply_matches(low_matches, boxes, scores, original_indices)

        for detection_index in unmatched_high:
            if scores[detection_index] < self.new_track_thresh:
                continue
            self.tracked_stracks.append(
                _NativeTrack(
                    track_id=self._next_track_id,
                    bbox=boxes[detection_index].copy(),
                    score=float(scores[detection_index]),
                    class_id=int(classes[detection_index]),
                    det_idx=int(original_indices[detection_index]),
                )
            )
            self._next_track_id += 1

        self.tracked_stracks = [
            track for track in self.tracked_stracks if track.unobserved_s <= self.max_unobserved_s + 1e-9
        ]

        active = sorted(
            (track for track in self.tracked_stracks if track.is_activated),
            key=lambda track: track.track_id,
        )
        if not active:
            return np.empty((0, 8), dtype=np.float32)
        return np.asarray(
            [
                [
                    *track.bbox.tolist(),
                    float(track.track_id),
                    track.score,
                    float(track.class_id),
                    float(track.det_idx),
                ]
                for track in active
            ],
            dtype=np.float32,
        )

    def _associate(
        self,
        track_indices: list[int],
        detection_indices: list[int],
        boxes: np.ndarray,
        classes: np.ndarray,
        *,
        max_cost: float,
    ) -> tuple[list[tuple[int, int]], list[int], list[int]]:
        if not track_indices or not detection_indices:
            return [], list(track_indices), list(detection_indices)

        track_boxes = np.asarray(
            [self.tracked_stracks[index].bbox for index in track_indices],
            dtype=np.float32,
        )
        detection_boxes = boxes[detection_indices]
        cost = 1.0 - _iou_matrix(track_boxes, detection_boxes)

        track_classes = np.asarray(
            [self.tracked_stracks[index].class_id for index in track_indices],
            dtype=np.int32,
        )
        detection_classes = classes[detection_indices]
        cost[track_classes[:, None] != detection_classes[None, :]] = 1e6

        # ByteTrack's cost-limited assignment allows either side to stay unmatched.
        track_count, detection_count = cost.shape
        assignment_cost = np.full(
            (track_count + detection_count, track_count + detection_count),
            max_cost / 2.0,
            dtype=np.float64,
        )
        assignment_cost[:track_count, :detection_count] = cost
        assignment_cost[track_count:, detection_count:] = 0.0
        row_indices, column_indices = linear_sum_assignment(assignment_cost)
        matches: list[tuple[int, int]] = []
        matched_tracks: set[int] = set()
        matched_detections: set[int] = set()
        for row, column in zip(row_indices.tolist(), column_indices.tolist()):
            if row >= track_count or column >= detection_count:
                continue
            if float(cost[row, column]) > max_cost:
                continue
            track_index = track_indices[row]
            detection_index = detection_indices[column]
            matches.append((track_index, detection_index))
            matched_tracks.add(track_index)
            matched_detections.add(detection_index)

        return (
            matches,
            [index for index in track_indices if index not in matched_tracks],
            [index for index in detection_indices if index not in matched_detections],
        )

    def _apply_matches(
        self,
        matches: list[tuple[int, int]],
        boxes: np.ndarray,
        scores: np.ndarray,
        original_indices: np.ndarray,
    ) -> None:
        for track_index, detection_index in matches:
            self.tracked_stracks[track_index].update(
                boxes[detection_index],
                float(scores[detection_index]),
                int(original_indices[detection_index]),
            )

    def reset(self) -> None:
        self.tracked_stracks.clear()
        self._next_track_id = 1
        self._last_timestamp = None
        self._camera_motion.reset()
