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


class _Clock:
    def __init__(self) -> None:
        self.now_ns = 1_000_000_000

    def __call__(self) -> int:
        return self.now_ns

    def advance_ms(self, milliseconds: int) -> None:
        self.now_ns += milliseconds * 1_000_000


def _identity(**overrides: object) -> MultiplayerRoomIdentity:
    values: dict[str, object] = {
        "run_id": "run-multiplayer-001",
        "session_id": "a" * 64,
        "boot_id": "boot-multiplayer-001",
    }
    values.update(overrides)
    return MultiplayerRoomIdentity(**values)  # type: ignore[arg-type]


def _join(
    member_id: str,
    *,
    role: MultiplayerRole = MultiplayerRole.DRIVER,
    source_id: str | None = None,
    source_epoch: int = 1,
    identity: MultiplayerRoomIdentity | None = None,
) -> MultiplayerJoinRequest:
    return MultiplayerJoinRequest(
        member_id=member_id,
        role=role,
        identity=identity or _identity(),
        source_id=source_id or f"robotsimue.{member_id}",
        source_epoch=source_epoch,
    )


def test_exact_room_identity_join_is_idempotent_and_projects_public_state() -> None:
    clock = _Clock()
    room = MultiplayerRoomAuthority(_identity(), monotonic_ns=clock)
    request = _join("operator.alpha")

    first = room.join(request)
    retried = room.join(request)

    assert retried is first
    assert first.member_id == "operator.alpha"
    assert first.role is MultiplayerRole.DRIVER
    assert first.source_id == "robotsimue.operator.alpha"
    assert first.source_epoch == 1
    assert room.snapshot() == {
        "schema": "lingtu.sim.multiplayer-room-state.v1",
        "run_id": "run-multiplayer-001",
        "session_id": "a" * 64,
        "boot_id": "boot-multiplayer-001",
        "members": [
            {
                "member_id": "operator.alpha",
                "role": "driver",
                "source_id": "robotsimue.operator.alpha",
                "source_epoch": 1,
            }
        ],
        "control_lease": None,
    }


@pytest.mark.parametrize(
    "identity",
    [
        _identity(run_id="run-other"),
        _identity(session_id="b" * 64),
        _identity(boot_id="boot-other"),
    ],
)
def test_join_rejects_any_room_identity_mismatch_without_mutation(
    identity: MultiplayerRoomIdentity,
) -> None:
    room = MultiplayerRoomAuthority(_identity())

    with pytest.raises(MultiplayerRoomError) as raised:
        room.join(_join("operator.alpha", identity=identity))

    assert raised.value.code is MultiplayerRoomErrorCode.IDENTITY_MISMATCH
    assert room.members == ()


def test_observer_cannot_claim_control_and_one_driver_lease_is_exclusive() -> None:
    clock = _Clock()
    room = MultiplayerRoomAuthority(_identity(), monotonic_ns=clock)
    room.join(_join("operator.alpha"))
    room.join(_join("operator.bravo"))
    room.join(_join("observer.charlie", role=MultiplayerRole.OBSERVER))

    lease = room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=500,
    )

    assert lease.holder_member_id == "operator.alpha"
    with pytest.raises(MultiplayerRoomError) as observer_error:
        room.claim_control(
            "observer.charlie",
            source_id="robotsimue.observer.charlie",
            source_epoch=1,
            ttl_ms=500,
        )
    assert observer_error.value.code is MultiplayerRoomErrorCode.ROLE_DENIED
    with pytest.raises(MultiplayerRoomError) as conflict:
        room.claim_control(
            "operator.bravo",
            source_id="robotsimue.operator.bravo",
            source_epoch=1,
            ttl_ms=500,
        )
    assert conflict.value.code is MultiplayerRoomErrorCode.LEASE_HELD


def test_lease_expiry_rejects_old_holder_and_allows_explicit_new_claim() -> None:
    clock = _Clock()
    room = MultiplayerRoomAuthority(_identity(), monotonic_ns=clock)
    room.join(_join("operator.alpha"))
    room.join(_join("operator.bravo"))
    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=100,
    )

    room.require_control_authority(
        member_id="operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
    )
    clock.advance_ms(101)

    with pytest.raises(MultiplayerRoomError) as expired:
        room.require_control_authority(
            member_id="operator.alpha",
            source_id="robotsimue.operator.alpha",
            source_epoch=1,
        )
    assert expired.value.code is MultiplayerRoomErrorCode.LEASE_REQUIRED
    assert (
        room.claim_control(
            "operator.bravo",
            source_id="robotsimue.operator.bravo",
            source_epoch=1,
            ttl_ms=100,
        ).holder_member_id
        == "operator.bravo"
    )


def test_higher_epoch_reconnect_revokes_lease_and_stale_source_is_rejected() -> None:
    room = MultiplayerRoomAuthority(_identity())
    room.join(_join("operator.alpha", source_epoch=1))
    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=500,
    )

    replacement = room.join(_join("operator.alpha", source_epoch=2))

    assert replacement.source_epoch == 2
    assert room.control_lease is None
    with pytest.raises(MultiplayerRoomError) as stale:
        room.require_control_authority(
            member_id="operator.alpha",
            source_id="robotsimue.operator.alpha",
            source_epoch=1,
        )
    assert stale.value.code is MultiplayerRoomErrorCode.SOURCE_EPOCH_STALE

    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=2,
        ttl_ms=500,
    )
    admitted = room.require_control_authority(
        member_id="operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=2,
    )
    assert admitted is replacement


def test_leave_by_driver_releases_control_but_keeps_other_members() -> None:
    room = MultiplayerRoomAuthority(_identity())
    room.join(_join("operator.alpha"))
    observer = room.join(_join("observer.bravo", role=MultiplayerRole.OBSERVER))
    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=500,
    )

    assert room.leave(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
    ) is True

    assert room.control_lease is None
    assert room.members == (observer,)
    assert room.leave(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
    ) is False


def test_stale_connection_cannot_release_or_remove_reconnected_member() -> None:
    room = MultiplayerRoomAuthority(_identity())
    room.join(_join("operator.alpha", source_epoch=1))
    current = room.join(_join("operator.alpha", source_epoch=2))
    room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=2,
        ttl_ms=500,
    )

    with pytest.raises(MultiplayerRoomError) as stale_release:
        room.release_control(
            "operator.alpha",
            source_id="robotsimue.operator.alpha",
            source_epoch=1,
        )
    assert stale_release.value.code is MultiplayerRoomErrorCode.SOURCE_EPOCH_STALE
    with pytest.raises(MultiplayerRoomError) as stale_leave:
        room.leave(
            "operator.alpha",
            source_id="robotsimue.operator.alpha",
            source_epoch=1,
        )
    assert stale_leave.value.code is MultiplayerRoomErrorCode.SOURCE_EPOCH_STALE
    assert room.members == (current,)
    assert room.control_lease is not None


def test_current_source_can_renew_then_explicitly_release_without_implicit_handoff() -> None:
    clock = _Clock()
    room = MultiplayerRoomAuthority(_identity(), monotonic_ns=clock)
    room.join(_join("operator.alpha"))
    room.join(_join("operator.bravo"))
    original = room.claim_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=100,
    )
    clock.advance_ms(80)

    renewed = room.renew_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
        ttl_ms=100,
    )
    clock.advance_ms(30)

    assert renewed.acquired_at_monotonic_ns == original.acquired_at_monotonic_ns
    assert renewed.expires_at_monotonic_ns > original.expires_at_monotonic_ns
    assert room.require_control_authority(
        member_id="operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
    ).member_id == "operator.alpha"
    assert room.release_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
    ) is True
    assert room.release_control(
        "operator.alpha",
        source_id="robotsimue.operator.alpha",
        source_epoch=1,
    ) is False
    with pytest.raises(MultiplayerRoomError) as no_implicit_handoff:
        room.require_control_authority(
            member_id="operator.bravo",
            source_id="robotsimue.operator.bravo",
            source_epoch=1,
        )
    assert no_implicit_handoff.value.code is MultiplayerRoomErrorCode.LEASE_REQUIRED


def test_source_id_is_unique_across_members() -> None:
    room = MultiplayerRoomAuthority(_identity())
    room.join(_join("operator.alpha", source_id="robotsimue.shared"))

    with pytest.raises(MultiplayerRoomError) as conflict:
        room.join(_join("operator.bravo", source_id="robotsimue.shared"))

    assert conflict.value.code is MultiplayerRoomErrorCode.SOURCE_CONFLICT
    assert [member.member_id for member in room.members] == ["operator.alpha"]
