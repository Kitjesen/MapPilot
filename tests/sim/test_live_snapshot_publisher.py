# ruff: noqa: S101

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
