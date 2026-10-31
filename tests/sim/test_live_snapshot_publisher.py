
from __future__ import annotations

import json
from typing import Any

import pytest
from sim.runtime.coordinator import CoordinatorError, RuntimeState
from sim.runtime.coordinator.live_snapshot import (
    MAX_LOOPBACK_DATAGRAM_BYTES,
    UdpLoopbackSnapshotPublisher,
    encode_truth_snapshot,
    stream_live_snapshots,
    truth_snapshot_document,
)


def _snapshot(sequence: int = 1, sim_time_ns: int = 16_000_000) -> dict[str, Any]:
    return {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": 2,
        "sequence": sequence,
        "physics_step": sequence * 8,
        "sim_time_ns": sim_time_ns,
        "bodies": [
            {
                "stable_id": "thunder_01/base_link",
                "instance_id": "thunder_01",
                "frame_id": "base_link",
                "position_m": [1.0, 2.0, 3.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        ],
    }


def test_truth_snapshot_projection_is_strict_and_compact() -> None:
    document = truth_snapshot_document(_snapshot())
    assert document["schema"] == "lingtu.sim.truth-snapshot.v1"
    assert "event" not in document
    payload = encode_truth_snapshot(_snapshot())
    assert len(payload) < MAX_LOOPBACK_DATAGRAM_BYTES
    assert json.loads(payload)["bodies"][0]["stable_id"] == "thunder_01/base_link"

    with pytest.raises(CoordinatorError, match="snapshot events only"):
        truth_snapshot_document({"event": "ready"})
    invalid = _snapshot()
    invalid["position"] = float("nan")
    with pytest.raises(CoordinatorError, match="not JSON-serializable"):
        encode_truth_snapshot(invalid)


class _FakeSocket:
    def __init__(self, *_: object) -> None:
        self.blocking: bool | None = None
        self.sent: list[tuple[bytes, tuple[str, int]]] = []
        self.closed = False

    def setblocking(self, value: bool) -> None:
        self.blocking = value

    def sendto(self, payload: bytes, destination: tuple[str, int]) -> int:
        self.sent.append((payload, destination))
        return len(payload)

    def close(self) -> None:
        self.closed = True


def test_udp_publisher_is_loopback_only_and_sends_one_whole_datagram() -> None:
    fake = _FakeSocket()
    publisher = UdpLoopbackSnapshotPublisher(
        25101, socket_factory=lambda *_: fake  # type: ignore[arg-type]
    )
    assert publisher.publish(_snapshot()) > 0
    assert fake.blocking is False
    assert fake.sent[0][1] == ("127.0.0.1", 25101)
    publisher.close()
    assert fake.closed

    with pytest.raises(ValueError, match="restricted"):
        UdpLoopbackSnapshotPublisher(25101, host="192.0.2.1")


def test_visual_udp_omits_contact_telemetry_without_mutating_recorded_truth() -> None:
    fake = _FakeSocket()
    event = _snapshot()
    # Contact details can grow independently of the fixed robot pose payload.
    contacts = [{"geom1_name": "ground", "geom2_name": "wheel", "normal_force_n": 50.0}] * 1000
    event.update(contact_count=len(contacts), contacts=contacts)
    publisher = UdpLoopbackSnapshotPublisher(
        25101, socket_factory=lambda *_: fake  # type: ignore[arg-type]
    )
    try:
        assert publisher.publish(event) < MAX_LOOPBACK_DATAGRAM_BYTES
    finally:
        publisher.close()
    visual = json.loads(fake.sent[0][0])
    assert "contacts" not in visual
    assert "contact_count" not in visual
    assert visual["bodies"] == event["bodies"]
    assert visual["sequence"] == event["sequence"]
    assert event["contacts"] is contacts
    assert truth_snapshot_document(event)["contacts"] == contacts


class _FakeCoordinator:
    def __init__(self) -> None:
        self.state = RuntimeState.NEW
        self.sequence = 0
        self.calls: list[object] = []

    def prepare(self) -> dict[str, Any]:
        self.calls.append("prepare")
        self.state = RuntimeState.READY
        return {"event": "ready"}

    def start(self) -> dict[str, Any]:
        self.calls.append("start")
        self.state = RuntimeState.RUNNING
        return {"event": "running"}

    def advance(self, steps: int) -> dict[str, Any]:
        self.calls.append(("advance", steps))
        self.sequence += 1
        return _snapshot(self.sequence, self.sequence * 16_000_000)

    def pause(self) -> dict[str, Any]:
        self.calls.append("pause")
        self.state = RuntimeState.PAUSED
        return {"event": "paused"}

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        self.state = RuntimeState.STOPPED
        return {"event": "stopped"}


class _Collector:
    def __init__(self) -> None:
        self.sequences: list[int] = []

    def publish(self, event: dict[str, Any]) -> int:
        self.sequences.append(event["sequence"])
        return 1


def test_bounded_live_loop_keeps_physics_authority_and_stops_cleanly() -> None:
    coordinator = _FakeCoordinator()
    collector = _Collector()
    frames = stream_live_snapshots(
        coordinator,  # type: ignore[arg-type]
        collector,
        steps_per_frame=8,
        frame_limit=3,
        pace=False,
    )
    assert frames == 3
    assert collector.sequences == [1, 2, 3]
    assert coordinator.calls == [
        "prepare",
        "start",
        ("advance", 8),
        ("advance", 8),
        ("advance", 8),
        "pause",
        "stop",
    ]
    assert coordinator.state is RuntimeState.STOPPED


@pytest.mark.parametrize("advance_seconds", [0.0, 0.005, 0.030])
def test_live_loop_waits_before_publishing_and_does_not_sleep_when_behind(
    monkeypatch: pytest.MonkeyPatch, advance_seconds: float,
) -> None:
    coordinator = _FakeCoordinator()
    clock = [100.0]
    published: list[tuple[int, float]] = []
    sleeps: list[float] = []
    advance = coordinator.advance

    def timed_advance(steps: int) -> dict[str, Any]:
        clock[0] += advance_seconds
        return advance(steps)

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        clock[0] += seconds

    class TimedCollector:
        def publish(self, event: dict[str, Any]) -> int:
            published.append((event["sim_time_ns"], clock[0]))
            return 1

    monkeypatch.setattr(coordinator, "advance", timed_advance)
    frames = stream_live_snapshots(
        coordinator, TimedCollector(), frame_limit=4,  # type: ignore[arg-type]
        monotonic=lambda: clock[0], sleep=sleep,
    )

    assert frames == 4
    assert [sim_ns for sim_ns, _ in published] == [16_000_000, 32_000_000, 48_000_000, 64_000_000]
    assert [wall for _, wall in published] == pytest.approx([
        100.0 + advance_seconds + index * max(0.016, advance_seconds) for index in range(4)
    ])
    if advance_seconds > 0.016:
        assert sleeps == []
    assert coordinator.state is RuntimeState.STOPPED


def test_live_loop_stops_when_start_fails_with_pending_readiness() -> None:
    error = CoordinatorError("operation requires READY, current state is PREPARING")

    class PreparingCoordinator(_FakeCoordinator):
        def prepare(self) -> dict[str, Any]:
            event = super().prepare()
            self.state = RuntimeState.PREPARING
            return event

        def start(self) -> dict[str, Any]:
            self.calls.append("start")
            raise error

    coordinator = PreparingCoordinator()
    collector = _Collector()
    with pytest.raises(CoordinatorError) as raised:
        stream_live_snapshots(
            coordinator, collector, frame_limit=1, pace=False  # type: ignore[arg-type]
        )

    assert raised.value is error
    assert coordinator.calls == ["prepare", "start", "stop"]
    assert coordinator.state is RuntimeState.STOPPED
    assert collector.sequences == []


def test_live_loop_stops_even_when_pause_fails() -> None:
    error = CoordinatorError("physics pause failed")

    class PauseFailingCoordinator(_FakeCoordinator):
        def pause(self) -> dict[str, Any]:
            self.calls.append("pause")
            raise error

    coordinator = PauseFailingCoordinator()
    collector = _Collector()
    with pytest.raises(CoordinatorError) as raised:
        stream_live_snapshots(
            coordinator, collector, frame_limit=1, pace=False  # type: ignore[arg-type]
        )

    assert raised.value is error
    assert coordinator.calls == ["prepare", "start", ("advance", 8), "pause", "stop"]
    assert coordinator.state is RuntimeState.STOPPED
    assert collector.sequences == [1]


@pytest.mark.parametrize("state", [RuntimeState.PREPARING, RuntimeState.FAILED])
def test_live_loop_cleans_up_prepare_failure_without_repeating_owned_abort(
    state: RuntimeState,
) -> None:
    error = CoordinatorError("physics preparation failed")

    class PrepareFailingCoordinator(_FakeCoordinator):
        def prepare(self) -> dict[str, Any]:
            self.calls.append("prepare")
            self.state = state
            raise error

    coordinator = PrepareFailingCoordinator()
    collector = _Collector()
    with pytest.raises(CoordinatorError) as raised:
        stream_live_snapshots(
            coordinator, collector, frame_limit=1, pace=False  # type: ignore[arg-type]
        )

    assert raised.value is error
    if state is RuntimeState.FAILED:
        assert coordinator.calls == ["prepare"]
        assert coordinator.state is RuntimeState.FAILED
    else:
        assert coordinator.calls == ["prepare", "stop"]
        assert coordinator.state is RuntimeState.STOPPED
    assert collector.sequences == []
