from __future__ import annotations

import numpy as np

from perception.tracking.bpu_tracker import TrackedDetection
from perception.tracking.person_counting import PersonTrackCounter


def _person(track_id: int, bottom_y: float) -> TrackedDetection:
    return TrackedDetection(
        track_id=track_id,
        bbox=np.asarray([10, bottom_y - 40, 30, bottom_y], dtype=np.float32),
        score=0.9,
        label="person",
        class_id=0,
    )


def test_counts_active_and_unique_person_tracks() -> None:
    counter = PersonTrackCounter(line_y_ratio=0.5)

    first = counter.update([_person(1, 40), _person(2, 60)], frame_height=100)
    second = counter.update([_person(2, 65)], frame_height=100)

    assert first["active"] == 2
    assert first["unique_tracks"] == 2
    assert second["active"] == 1
    assert second["unique_tracks"] == 2


def test_counts_directional_line_crossing_with_hysteresis() -> None:
    counter = PersonTrackCounter(line_y_ratio=0.5, hysteresis_px=3)

    counter.update([_person(1, 40)], frame_height=100)
    near_line = counter.update([_person(1, 51)], frame_height=100)
    crossed_in = counter.update([_person(1, 60)], frame_height=100)
    counter.update([_person(1, 62)], frame_height=100)
    crossed_out = counter.update([_person(1, 40)], frame_height=100)

    assert near_line["entered"] == 0
    assert crossed_in["entered"] == 1
    assert crossed_in["exited"] == 0
    assert crossed_out["exited"] == 1


def test_non_person_tracks_are_ignored() -> None:
    counter = PersonTrackCounter()
    bicycle = TrackedDetection(
        track_id=7,
        bbox=np.asarray([0, 0, 20, 20], dtype=np.float32),
        score=0.9,
        label="bicycle",
        class_id=1,
    )

    counts = counter.update([bicycle], frame_height=100)

    assert counts == {
        "active": 0,
        "unique_tracks": 0,
        "entered": 0,
        "exited": 0,
    }
