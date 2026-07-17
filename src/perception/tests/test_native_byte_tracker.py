from __future__ import annotations

import numpy as np

from perception.detection.detector_base import Detection2D
from perception.tracking.bpu_tracker import BPUTracker, _DetectionResults
from perception.tracking.native_byte_tracker import NativeByteTracker


def _results(*items: tuple[list[float], float, int]) -> _DetectionResults:
    detections = [
        Detection2D(
            bbox=np.asarray(bbox, dtype=np.float32),
            score=score,
            label="person",
            class_id=class_id,
        )
        for bbox, score, class_id in items
    ]
    return _DetectionResults(detections)


def _by_left_edge(rows: np.ndarray) -> list[int]:
    return [int(row[4]) for row in sorted(rows, key=lambda row: float(row[0]))]


def test_ids_remain_stable_when_detection_order_changes() -> None:
    tracker = NativeByteTracker(track_buffer=3)

    first = tracker.update(
        _results(
            ([10, 10, 50, 110], 0.9, 0),
            ([150, 10, 190, 110], 0.9, 0),
        )
    )
    second = tracker.update(
        _results(
            ([145, 10, 185, 110], 0.9, 0),
            ([15, 10, 55, 110], 0.9, 0),
        )
    )

    assert _by_left_edge(first) == _by_left_edge(second)


def test_low_confidence_detection_recovers_track_but_does_not_start_one() -> None:
    tracker = NativeByteTracker(
        track_high_thresh=0.5,
        track_low_thresh=0.1,
        new_track_thresh=0.5,
        track_buffer=3,
    )

    first = tracker.update(_results(([10, 10, 50, 110], 0.9, 0)))
    recovered = tracker.update(_results(([12, 10, 52, 110], 0.2, 0)))
    tracker.reset()
    low_only = tracker.update(_results(([10, 10, 50, 110], 0.2, 0)))

    assert int(first[0, 4]) == int(recovered[0, 4])
    assert low_only.shape == (0, 8)


def test_track_recovers_within_buffer_and_expires_after_buffer() -> None:
    tracker = NativeByteTracker(track_buffer=2)

    first = tracker.update(_results(([10, 10, 50, 110], 0.9, 0)))
    first_id = int(first[0, 4])
    tracker.update(_results())
    recovered = tracker.update(_results(([11, 10, 51, 110], 0.9, 0)))
    assert int(recovered[0, 4]) == first_id

    tracker.update(_results())
    tracker.update(_results())
    tracker.update(_results())
    restarted = tracker.update(_results(([11, 10, 51, 110], 0.9, 0)))

    assert int(restarted[0, 4]) != first_id


def test_class_mismatch_does_not_reuse_track_id() -> None:
    tracker = NativeByteTracker(track_buffer=3)

    person = tracker.update(_results(([10, 10, 50, 110], 0.9, 0)))
    bicycle = tracker.update(_results(([11, 10, 51, 110], 0.9, 1)))

    assert int(person[0, 4]) != int(bicycle[0, 4])


class _Detector:
    def detect(self, _frame, _prompt):
        return [
            Detection2D(
                bbox=np.asarray([10, 10, 50, 110], dtype=np.float32),
                score=0.9,
                label="person",
                class_id=0,
            )
        ]


def test_bpu_tracker_can_explicitly_use_native_backend() -> None:
    tracker = BPUTracker(_Detector(), tracker_type="native_bytetrack")

    result = tracker.track(np.zeros((120, 200, 3), dtype=np.uint8), "person")

    assert tracker.backend_name == "native_bytetrack"
    assert len(result) == 1
    assert result[0].label == "person"
    assert result[0].track_id > 0
