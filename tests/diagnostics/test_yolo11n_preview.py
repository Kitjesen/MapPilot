from __future__ import annotations

from types import SimpleNamespace

import pytest

pytest.importorskip("cv2")

from tools.perception.yolo11n_preview import PersonLockTracker, WebSocketFrameSource


class _FakeConnection:
    def __init__(self, frames: list[bytes]) -> None:
        self._frames = iter(frames)
        self.closed = False

    def recv(self, *, timeout: float) -> bytes:
        assert timeout == 5.0
        return next(self._frames)

    def close(self) -> None:
        self.closed = True


def test_websocket_frame_source_consumes_fresh_binary_frames_without_reconnecting():
    connection = _FakeConnection([b"frame-1", b"frame-2"])
    calls: list[str] = []

    def connect(url: str, *, open_timeout: float):
        calls.append(url)
        assert open_timeout == 5.0
        return connection

    source = WebSocketFrameSource("ws://camera/ws", connector=connect)

    assert source.next_frame() == b"frame-1"
    assert source.next_frame() == b"frame-2"
    assert calls == ["ws://camera/ws"]

    source.close()
    assert connection.closed is True


def _detection(label: str, score: float, bbox: tuple[int, int, int, int]):
    return SimpleNamespace(label=label, score=score, bbox=bbox)


def test_person_lock_keeps_target_when_a_more_confident_person_appears_far_away():
    tracker = PersonLockTracker(max_lost_frames=2)

    first = tracker.update([_detection("person", 0.8, (10, 10, 110, 210))])
    second = tracker.update(
        [
            _detection("person", 0.75, (20, 10, 120, 210)),
            _detection("person", 0.99, (400, 10, 500, 210)),
        ]
    )

    assert first.state == "locked"
    assert first.track_id == 1
    assert second.state == "locked"
    assert second.track_id == 1
    assert second.bbox == (20, 10, 120, 210)


def test_person_lock_reports_lost_then_allocates_a_new_id_after_timeout():
    tracker = PersonLockTracker(max_lost_frames=1)

    locked = tracker.update([_detection("person", 0.8, (10, 10, 110, 210))])
    lost = tracker.update([])
    searching = tracker.update([])
    relocked = tracker.update([_detection("person", 0.9, (300, 10, 400, 210))])

    assert locked.track_id == 1
    assert lost.state == "lost"
    assert lost.track_id == 1
    assert searching.state == "searching"
    assert searching.track_id is None
    assert relocked.state == "locked"
    assert relocked.track_id == 2


def test_person_lock_ignores_non_person_detections():
    tracker = PersonLockTracker()

    result = tracker.update([_detection("cup", 0.99, (10, 10, 50, 80))])

    assert result.state == "searching"
    assert result.track_id is None
