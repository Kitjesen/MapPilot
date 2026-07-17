"""Person occupancy and line-crossing metrics over stable 2D track IDs."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

_PERSON_LABELS = {"person", "people", "human", "pedestrian"}


@dataclass
class _CrossingState:
    stable_side: int
    last_crossing_frame: int = -1_000_000


class PersonTrackCounter:
    """Count active, unique, and directed person crossings.

    ``unique_tracks`` means unique tracker IDs observed since reset. It is not a
    biometric count: a person who leaves longer than the tracker buffer and
    later returns may receive a new ID and be counted again.
    """

    def __init__(
        self,
        *,
        line_y_ratio: float = 0.5,
        hysteresis_px: float = 4.0,
        min_crossing_gap_frames: int = 2,
    ) -> None:
        if not 0.0 <= line_y_ratio <= 1.0:
            raise ValueError("line_y_ratio must be between 0 and 1")
        self.line_y_ratio = float(line_y_ratio)
        self.hysteresis_px = max(0.0, float(hysteresis_px))
        self.min_crossing_gap_frames = max(0, int(min_crossing_gap_frames))
        self.reset()

    def update(self, tracks: Iterable, *, frame_height: int) -> dict[str, int]:
        self._frame_index += 1
        people = [track for track in tracks if self._is_person(track)]
        self._unique_track_ids.update(int(track.track_id) for track in people)

        line_y = max(0.0, float(frame_height)) * self.line_y_ratio
        for track in people:
            track_id = int(track.track_id)
            bottom_y = float(track.bbox[3])
            side = self._side(bottom_y, line_y)
            if side == 0:
                continue

            state = self._crossing_states.get(track_id)
            if state is None:
                self._crossing_states[track_id] = _CrossingState(stable_side=side)
                continue
            if side == state.stable_side:
                continue

            enough_gap = self._frame_index - state.last_crossing_frame >= self.min_crossing_gap_frames
            if enough_gap:
                if state.stable_side < 0 and side > 0:
                    self._entered += 1
                elif state.stable_side > 0 and side < 0:
                    self._exited += 1
                state.last_crossing_frame = self._frame_index
            state.stable_side = side

        self._latest = {
            "active": len(people),
            "unique_tracks": len(self._unique_track_ids),
            "entered": self._entered,
            "exited": self._exited,
        }
        return dict(self._latest)

    def snapshot(self) -> dict[str, int]:
        return dict(self._latest)

    def reset(self) -> None:
        self._frame_index = 0
        self._unique_track_ids: set[int] = set()
        self._crossing_states: dict[int, _CrossingState] = {}
        self._entered = 0
        self._exited = 0
        self._latest = {
            "active": 0,
            "unique_tracks": 0,
            "entered": 0,
            "exited": 0,
        }

    def _side(self, bottom_y: float, line_y: float) -> int:
        if bottom_y < line_y - self.hysteresis_px:
            return -1
        if bottom_y > line_y + self.hysteresis_px:
            return 1
        return 0

    @staticmethod
    def _is_person(track) -> bool:
        label = str(getattr(track, "label", "")).strip().lower()
        return label in _PERSON_LABELS or int(getattr(track, "class_id", -1)) == 0
