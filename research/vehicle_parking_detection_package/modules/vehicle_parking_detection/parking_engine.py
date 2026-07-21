from __future__ import annotations

from dataclasses import dataclass
from time import time

from .config import DEFAULT_EVENT_NAME, RoiConfig
from .geometry import box_center_bottom, point_in_polygon
from .models import Detection


@dataclass
class ParkingTrigger:
    event_name: str
    location: str
    roi_id: str
    duration_seconds: float
    detection: Detection
    timestamp_epoch: float


@dataclass
class RoiTrackState:
    first_seen: float
    last_seen: float
    last_box: tuple[float, float, float, float]


class VehicleParkingEngine:
    def __init__(
        self,
        rois: list[RoiConfig],
        event_name: str = DEFAULT_EVENT_NAME,
        vehicle_class: str = "vehicle",
        min_confidence: float = 0.20,
        reconnect_seconds: float = 5.0,
    ):
        self.rois = rois
        self.event_name = event_name
        self.vehicle_class = vehicle_class
        self.min_confidence = float(min_confidence)
        self.reconnect_seconds = float(reconnect_seconds)
        self.track_states: dict[tuple[str, int | str], RoiTrackState] = {}
        self.last_alarm: dict[str, float] = {}

    def evaluate(self, detections: list[Detection], now: float | None = None) -> list[ParkingTrigger]:
        now = time() if now is None else now
        triggers: list[ParkingTrigger] = []
        active_keys: set[tuple[str, int | str]] = set()
        vehicles = [
            det
            for det in detections
            if det.class_name == self.vehicle_class and det.confidence >= self.min_confidence
        ]

        for roi in self.rois:
            if not roi.polygon:
                continue
            for det in vehicles:
                if not point_in_polygon(box_center_bottom(det.box), roi.polygon):
                    continue
                key = (roi.id, det.track_id if det.track_id is not None else self._untracked_key(det))
                active_keys.add(key)
                state = self.track_states.get(key)
                if state is None:
                    state = RoiTrackState(first_seen=now, last_seen=now, last_box=det.box)
                    self.track_states[key] = state
                else:
                    state.last_seen = now
                    state.last_box = det.box

                duration = now - state.first_seen
                if duration < roi.dwell_seconds:
                    continue
                last_alarm = self.last_alarm.get(roi.id, 0.0)
                if now - last_alarm < roi.cooldown_seconds:
                    continue
                self.last_alarm[roi.id] = now
                triggers.append(
                    ParkingTrigger(
                        event_name=self.event_name,
                        location=roi.location,
                        roi_id=roi.id,
                        duration_seconds=duration,
                        detection=det,
                        timestamp_epoch=now,
                    )
                )

        self._prune(now, active_keys)
        return triggers

    def _prune(self, now: float, active_keys: set[tuple[str, int | str]]) -> None:
        for key, state in list(self.track_states.items()):
            if key in active_keys:
                continue
            if now - state.last_seen > self.reconnect_seconds:
                del self.track_states[key]

    @staticmethod
    def _untracked_key(det: Detection) -> str:
        x1, y1, x2, y2 = det.box
        return f"untracked:{round((x1 + x2) / 20)}:{round((y1 + y2) / 20)}"
