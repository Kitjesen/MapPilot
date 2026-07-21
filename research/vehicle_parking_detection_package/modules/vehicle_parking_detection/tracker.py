from __future__ import annotations

from time import time

from .geometry import box_iou, center_distance_ratio
from .models import Detection, Track


class IouTracker:
    def __init__(
        self,
        iou_threshold: float = 0.10,
        max_missed: int = 12,
        center_distance_ratio_threshold: float = 0.85,
        max_age_seconds: float = 30.0,
    ):
        self.iou_threshold = float(iou_threshold)
        self.max_missed = int(max_missed)
        self.center_distance_ratio_threshold = float(center_distance_ratio_threshold)
        self.max_age_seconds = float(max_age_seconds)
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, detections: list[Detection], now: float | None = None) -> list[Detection]:
        now = time() if now is None else now
        unmatched_tracks = set(self.tracks)

        for det in detections:
            best_track_id = None
            best_score = float("-inf")
            for track_id in list(unmatched_tracks):
                track = self.tracks[track_id]
                if track.class_name != det.class_name:
                    continue
                iou = box_iou(track.box, det.box)
                distance_ratio = center_distance_ratio(track.box, det.box)
                if iou < self.iou_threshold and distance_ratio > self.center_distance_ratio_threshold:
                    continue
                score = iou - min(distance_ratio, 10.0) * 0.05
                if score > best_score:
                    best_score = score
                    best_track_id = track_id

            if best_track_id is None:
                track_id = self.next_id
                self.next_id += 1
                self.tracks[track_id] = Track(
                    id=track_id,
                    class_name=det.class_name,
                    box=det.box,
                    confidence=det.confidence,
                    first_seen=now,
                    last_seen=now,
                )
                det.track_id = track_id
                continue

            track = self.tracks[best_track_id]
            track.box = det.box
            track.confidence = det.confidence
            track.last_seen = now
            track.missed = 0
            det.track_id = best_track_id
            unmatched_tracks.discard(best_track_id)

        for track_id in list(unmatched_tracks):
            track = self.tracks[track_id]
            track.missed += 1
            if track.missed > self.max_missed or now - track.last_seen > self.max_age_seconds:
                del self.tracks[track_id]

        return detections

    def active_tracks(self) -> list[Track]:
        return sorted(self.tracks.values(), key=lambda item: item.id)
