# ruff: noqa: S101

from __future__ import annotations

import pytest

from sim.runtime.coordinator.multiplayer_room import (
    MultiplayerJoinRequest,
    MultiplayerRole,
    MultiplayerRoomAuthority,
    MultiplayerRoomError,
    MultiplayerRoomErrorCode,
    MultiplayerRoomIdentity,
)
from sim.runtime.coordinator.multiplayer_snapshot import MultiplayerSnapshotFanout


def _room() -> MultiplayerRoomAuthority:
    return MultiplayerRoomAuthority(
        MultiplayerRoomIdentity(
            run_id="run-multiplayer-001",
            session_id="a" * 64,
            boot_id="boot-multiplayer-001",
        )
    )


def _join(
    room: MultiplayerRoomAuthority,
    member_id: str,
    role: MultiplayerRole,
) -> tuple[str, int]:
    source_id = f"robotsimue.{member_id}"
    room.join(
        MultiplayerJoinRequest(
            member_id=member_id,
            role=role,
            identity=room.identity,
            source_id=source_id,
            source_epoch=1,
        )
    )
    return source_id, 1


def _snapshot(sequence: int, *, reset_generation: int = 2) -> dict[str, object]:
    return {
        "event": "snapshot",
        "session_id": "a" * 64,
        "model_generation": 3,
        "reset_generation": reset_generation,
        "sequence": sequence,
        "physics_step": sequence * 8,
        "sim_time_ns": sequence * 20_000_000,
        "bodies": {
            "thunder_01/base": {
                "position_m": [float(sequence), 0.0, 0.4],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            }
        },
    }


def test_driver_and_observer_receive_independent_copies_of_same_truth() -> None:
    room = _room()
    driver_source = _join(room, "operator.alpha", MultiplayerRole.DRIVER)
    observer_source = _join(room, "observer.bravo", MultiplayerRole.OBSERVER)
    fanout = MultiplayerSnapshotFanout(room)
    fanout.subscribe("operator.alpha", source_id=driver_source[0], source_epoch=1)
    fanout.subscribe("observer.bravo", source_id=observer_source[0], source_epoch=1)

    assert fanout.publish(_snapshot(7)) == 2
    driver = fanout.take_latest(
        "operator.alpha", source_id=driver_source[0], source_epoch=1
    )
    observer = fanout.take_latest(
        "observer.bravo", source_id=observer_source[0], source_epoch=1
    )

    assert driver == observer
    assert driver is not observer
    assert driver == {
        "schema": "lingtu.sim.multiplayer-truth-envelope.v1",
        "run_id": "run-multiplayer-001",
        "session_id": "a" * 64,
        "boot_id": "boot-multiplayer-001",
        "truth": {
            "schema": "lingtu.sim.truth-snapshot.v1",
            **{key: value for key, value in _snapshot(7).items() if key != "event"},
        },
    }
    assert driver is not None
    driver["truth"]["bodies"].clear()  # type: ignore[index,union-attr]
    assert observer["truth"]["bodies"]  # type: ignore[index,union-attr]


def test_slow_observer_drops_intermediate_snapshot_without_blocking_driver() -> None:
    room = _room()
    driver_source = _join(room, "operator.alpha", MultiplayerRole.DRIVER)
    observer_source = _join(room, "observer.bravo", MultiplayerRole.OBSERVER)
    fanout = MultiplayerSnapshotFanout(room)
    fanout.subscribe("operator.alpha", source_id=driver_source[0], source_epoch=1)
    fanout.subscribe("observer.bravo", source_id=observer_source[0], source_epoch=1)

    fanout.publish(_snapshot(7))
    first_driver = fanout.take_latest(
        "operator.alpha", source_id=driver_source[0], source_epoch=1
    )
    fanout.publish(_snapshot(8))
    second_driver = fanout.take_latest(
        "operator.alpha", source_id=driver_source[0], source_epoch=1
    )
    observer = fanout.take_latest(
        "observer.bravo", source_id=observer_source[0], source_epoch=1
    )

    assert first_driver["truth"]["sequence"] == 7  # type: ignore[index,union-attr]
    assert second_driver["truth"]["sequence"] == 8  # type: ignore[index,union-attr]
    assert observer["truth"]["sequence"] == 8  # type: ignore[index,union-attr]
    assert fanout.subscriber_state("observer.bravo").dropped_snapshots == 1
    assert fanout.subscriber_state("operator.alpha").dropped_snapshots == 0


def test_late_subscriber_may_replay_latest_without_reordering_future_truth() -> None:
    room = _room()
    source = _join(room, "observer.alpha", MultiplayerRole.OBSERVER)
    fanout = MultiplayerSnapshotFanout(room)
    fanout.publish(_snapshot(7))

    fanout.subscribe(
        "observer.alpha",
        source_id=source[0],
        source_epoch=1,
        replay_latest=True,
    )
    replay = fanout.take_latest(
        "observer.alpha", source_id=source[0], source_epoch=1
    )
    fanout.publish(_snapshot(8))
    current = fanout.take_latest(
        "observer.alpha", source_id=source[0], source_epoch=1
    )

    assert replay["truth"]["sequence"] == 7  # type: ignore[index,union-attr]
    assert current["truth"]["sequence"] == 8  # type: ignore[index,union-attr]


def test_stale_reconnected_source_cannot_receive_or_remove_current_subscription() -> None:
    room = _room()
    source = _join(room, "observer.alpha", MultiplayerRole.OBSERVER)
    fanout = MultiplayerSnapshotFanout(room)
    fanout.subscribe("observer.alpha", source_id=source[0], source_epoch=1)
    room.join(
        MultiplayerJoinRequest(
            member_id="observer.alpha",
            role=MultiplayerRole.OBSERVER,
            identity=room.identity,
            source_id=source[0],
            source_epoch=2,
        )
    )
    fanout.subscribe("observer.alpha", source_id=source[0], source_epoch=2)
    fanout.publish(_snapshot(7))

    with pytest.raises(MultiplayerRoomError) as stale:
        fanout.take_latest("observer.alpha", source_id=source[0], source_epoch=1)

    assert stale.value.code is MultiplayerRoomErrorCode.SOURCE_EPOCH_STALE
    current = fanout.take_latest("observer.alpha", source_id=source[0], source_epoch=2)
    assert current["truth"]["sequence"] == 7  # type: ignore[index,union-attr]


def test_truth_sequence_regression_is_rejected_before_subscriber_mutation() -> None:
    room = _room()
    source = _join(room, "observer.alpha", MultiplayerRole.OBSERVER)
    fanout = MultiplayerSnapshotFanout(room)
    fanout.subscribe("observer.alpha", source_id=source[0], source_epoch=1)
    fanout.publish(_snapshot(8))

    with pytest.raises(ValueError, match="strictly advance"):
        fanout.publish(_snapshot(7))

    current = fanout.take_latest("observer.alpha", source_id=source[0], source_epoch=1)
    assert current["truth"]["sequence"] == 8  # type: ignore[index,union-attr]
