"""Admission adapter from multiplayer room authority to playable control."""

from __future__ import annotations

from sim.runtime.coordinator.control_intent_udp import (
    OperatorMotionIntent,
    OperatorRuntimeRequest,
)

from .multiplayer_room import (
    MAX_CONTROL_LEASE_TTL_MS,
    MultiplayerRoomAuthority,
    MultiplayerRoomError,
    MultiplayerRoomIdentity,
)


class RoomMotionAdmission:
    """Check motion intent against one exact room and driver lease."""

    def __init__(self, room: MultiplayerRoomAuthority) -> None:
        if not isinstance(room, MultiplayerRoomAuthority):
            raise TypeError("room must be a MultiplayerRoomAuthority")
        self._room = room

    def rejection_reason(self, intent: OperatorMotionIntent) -> str | None:
        """Return ``None`` when admitted, otherwise a stable rejection reason."""

        if not isinstance(intent, OperatorMotionIntent):
            raise TypeError("intent must be an OperatorMotionIntent")
        identity = intent.identity
        wire_room_identity = MultiplayerRoomIdentity(
            run_id=identity.run_id,
            session_id=identity.session_id,
            boot_id=identity.boot_id,
        )
        if wire_room_identity != self._room.identity:
            return "multiplayer:identity_mismatch"
        try:
            self._room.require_control_source(
                source_id=identity.source_id,
                source_epoch=intent.source_epoch,
            )
        except MultiplayerRoomError as exc:
            return f"multiplayer:{exc.code.value}"
        return None


class RoomRuntimeRequestAdmission:
    """Apply multiplayer role and lease policy to runtime requests."""

    def __init__(
        self,
        room: MultiplayerRoomAuthority,
        *,
        lease_ttl_ms: int,
    ) -> None:
        if not isinstance(room, MultiplayerRoomAuthority):
            raise TypeError("room must be a MultiplayerRoomAuthority")
        if (
            isinstance(lease_ttl_ms, bool)
            or not isinstance(lease_ttl_ms, int)
            or not 1 <= lease_ttl_ms <= MAX_CONTROL_LEASE_TTL_MS
        ):
            raise ValueError(
                "lease_ttl_ms must be an integer from 1 through "
                f"{MAX_CONTROL_LEASE_TTL_MS}"
            )
        self._room = room
        self._lease_ttl_ms = lease_ttl_ms

    def rejection_reason(self, request: OperatorRuntimeRequest) -> str | None:
        """Apply request policy and return a stable rejection reason, if any."""

        if not isinstance(request, OperatorRuntimeRequest):
            raise TypeError("request must be an OperatorRuntimeRequest")
        identity = request.identity
        wire_room_identity = MultiplayerRoomIdentity(
            run_id=identity.run_id,
            session_id=identity.session_id,
            boot_id=identity.boot_id,
        )
        if wire_room_identity != self._room.identity:
            return "multiplayer:identity_mismatch"
        try:
            member = self._room.require_source_member(
                source_id=identity.source_id,
                source_epoch=request.source_epoch,
            )
            if request.request == "ui_state_update":
                return None
            if request.request == "control_claim":
                self._room.claim_control(
                    member.member_id,
                    source_id=member.source_id,
                    source_epoch=member.source_epoch,
                    ttl_ms=self._lease_ttl_ms,
                )
                return None
            if request.request in {"control_release", "safe_stop"}:
                self._room.require_control_source(
                    source_id=identity.source_id,
                    source_epoch=request.source_epoch,
                )
                released = self._room.release_control(
                    member.member_id,
                    source_id=member.source_id,
                    source_epoch=member.source_epoch,
                )
                if not released:
                    return "multiplayer:lease_required"
                return None
            self._room.require_control_source(
                source_id=identity.source_id,
                source_epoch=request.source_epoch,
            )
        except MultiplayerRoomError as exc:
            return f"multiplayer:{exc.code.value}"
        return None


__all__ = ["RoomMotionAdmission", "RoomRuntimeRequestAdmission"]
