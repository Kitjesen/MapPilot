"""Transport-free multiplayer room membership and control authority.

The room binds ephemeral multiplayer state to one exact RunAllocation identity.
It does not own sockets, simulation advancement, or controller submission.
"""

from __future__ import annotations

import re
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any

MULTIPLAYER_ROOM_STATE_SCHEMA = "lingtu.sim.multiplayer-room-state.v1"
MAX_CONTROL_LEASE_TTL_MS = 30_000

_SAFE_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")


class MultiplayerRole(str, Enum):
    """Authority role assigned to one room member."""

    DRIVER = "driver"
    OBSERVER = "observer"


class MultiplayerRoomErrorCode(str, Enum):
    """Stable rejection codes at the room authority seam."""

    IDENTITY_MISMATCH = "identity_mismatch"
    MEMBER_UNKNOWN = "member_unknown"
    MEMBER_CONFLICT = "member_conflict"
    SOURCE_CONFLICT = "source_conflict"
    SOURCE_EPOCH_STALE = "source_epoch_stale"
    ROLE_DENIED = "role_denied"
    LEASE_HELD = "lease_held"
    LEASE_REQUIRED = "lease_required"


class MultiplayerRoomError(RuntimeError):
    """Machine-readable room admission or control failure."""

    def __init__(self, code: MultiplayerRoomErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message

    def to_dict(self) -> dict[str, str]:
        """Return the stable public error projection."""

        return {
            "schema": "lingtu.sim.multiplayer-room-error.v1",
            "code": self.code.value,
            "message": self.message,
        }


@dataclass(frozen=True, slots=True)
class MultiplayerRoomIdentity:
    """Exact run identity shared by every admitted member."""

    run_id: str
    session_id: str
    boot_id: str

    def __post_init__(self) -> None:
        _safe_id(self.run_id, "run_id")
        _safe_id(self.session_id, "session_id")
        _safe_id(self.boot_id, "boot_id")


@dataclass(frozen=True, slots=True)
class MultiplayerJoinRequest:
    """One transport-authenticated request to enter a room."""

    member_id: str
    role: MultiplayerRole
    identity: MultiplayerRoomIdentity
    source_id: str
    source_epoch: int

    def __post_init__(self) -> None:
        _safe_id(self.member_id, "member_id")
        if not isinstance(self.role, MultiplayerRole):
            raise TypeError("role must be a MultiplayerRole")
        if not isinstance(self.identity, MultiplayerRoomIdentity):
            raise TypeError("identity must be a MultiplayerRoomIdentity")
        _safe_id(self.source_id, "source_id")
        _positive_int(self.source_epoch, "source_epoch")


@dataclass(frozen=True, slots=True)
class MultiplayerMembership:
    """Immutable admitted member state."""

    member_id: str
    role: MultiplayerRole
    source_id: str
    source_epoch: int

    def to_dict(self) -> dict[str, str | int]:
        """Return the stable public membership projection."""

        return {
            "member_id": self.member_id,
            "role": self.role.value,
            "source_id": self.source_id,
            "source_epoch": self.source_epoch,
        }


@dataclass(frozen=True, slots=True)
class MultiplayerControlLease:
    """One explicit exclusive driver lease."""

    holder_member_id: str
    source_id: str
    source_epoch: int
    acquired_at_monotonic_ns: int
    expires_at_monotonic_ns: int


class MultiplayerRoomAuthority:
    """Serialize room membership and exclusive control policy for one run."""

    def __init__(
        self,
        identity: MultiplayerRoomIdentity,
        *,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(identity, MultiplayerRoomIdentity):
            raise TypeError("identity must be a MultiplayerRoomIdentity")
        if not callable(monotonic_ns):
            raise TypeError("monotonic_ns must be callable")
        self._identity = identity
        self._monotonic_ns = monotonic_ns
        self._members: dict[str, MultiplayerMembership] = {}
        self._lease: MultiplayerControlLease | None = None
        self._lock = threading.RLock()

    @property
    def identity(self) -> MultiplayerRoomIdentity:
        """Return the immutable room identity."""

        return self._identity

    @property
    def members(self) -> tuple[MultiplayerMembership, ...]:
        """Return members in deterministic member-id order."""

        with self._lock:
            return tuple(self._members[key] for key in sorted(self._members))

    @property
    def control_lease(self) -> MultiplayerControlLease | None:
        """Return the current active lease, expiring it when necessary."""

        with self._lock:
            now_ns = self._now_ns()
            self._expire_lease_locked(now_ns)
            return self._lease

    def join(self, request: MultiplayerJoinRequest) -> MultiplayerMembership:
        """Admit an exact member join or reconnect request."""

        if not isinstance(request, MultiplayerJoinRequest):
            raise TypeError("request must be a MultiplayerJoinRequest")
        with self._lock:
            if request.identity != self._identity:
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.IDENTITY_MISMATCH,
                    "join identity does not match the room allocation",
                )

            requested = MultiplayerMembership(
                member_id=request.member_id,
                role=request.role,
                source_id=request.source_id,
                source_epoch=request.source_epoch,
            )
            existing = self._members.get(request.member_id)
            if existing is not None:
                if existing == requested:
                    return existing
                if request.source_id != existing.source_id or request.role is not existing.role:
                    raise MultiplayerRoomError(
                        MultiplayerRoomErrorCode.MEMBER_CONFLICT,
                        "member_id is already bound to a different source or role",
                    )
                if request.source_epoch <= existing.source_epoch:
                    raise MultiplayerRoomError(
                        MultiplayerRoomErrorCode.SOURCE_EPOCH_STALE,
                        "reconnect source_epoch must be strictly greater than the active epoch",
                    )
                self._members[request.member_id] = requested
                if (
                    self._lease is not None
                    and self._lease.holder_member_id == request.member_id
                ):
                    self._lease = None
                return requested

            conflicting_member = next(
                (
                    member.member_id
                    for member in self._members.values()
                    if member.source_id == request.source_id
                ),
                None,
            )
            if conflicting_member is not None:
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.SOURCE_CONFLICT,
                    f"source_id is already bound to member {conflicting_member!r}",
                )
            self._members[request.member_id] = requested
            return requested

    def leave(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
    ) -> bool:
        """Remove one member and release its lease, if present."""

        _safe_id(member_id, "member_id")
        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        with self._lock:
            member = self._members.get(member_id)
            if member is None:
                return False
            self._require_source_locked(
                member,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            self._members.pop(member_id)
            if self._lease is not None and self._lease.holder_member_id == member_id:
                self._lease = None
            return True

    def claim_control(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
        ttl_ms: int,
    ) -> MultiplayerControlLease:
        """Claim the exclusive lease for an admitted driver."""

        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        ttl_ns = _lease_ttl_ns(ttl_ms)
        with self._lock:
            now_ns = self._now_ns()
            self._expire_lease_locked(now_ns)
            member = self._require_member_locked(member_id)
            self._require_source_locked(
                member,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            if member.role is not MultiplayerRole.DRIVER:
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.ROLE_DENIED,
                    "observer members cannot claim control",
                )
            if self._lease is not None:
                if self._lease.holder_member_id == member_id:
                    return self._lease
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.LEASE_HELD,
                    f"control is held by member {self._lease.holder_member_id!r}",
                )
            lease = MultiplayerControlLease(
                holder_member_id=member.member_id,
                source_id=member.source_id,
                source_epoch=member.source_epoch,
                acquired_at_monotonic_ns=now_ns,
                expires_at_monotonic_ns=now_ns + ttl_ns,
            )
            self._lease = lease
            return lease

    def renew_control(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
        ttl_ms: int,
    ) -> MultiplayerControlLease:
        """Extend an active lease held by the same current member source."""

        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        ttl_ns = _lease_ttl_ns(ttl_ms)
        with self._lock:
            now_ns = self._now_ns()
            self._expire_lease_locked(now_ns)
            member = self._require_member_locked(member_id)
            self._require_source_locked(
                member,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            lease = self._lease
            if (
                lease is None
                or lease.holder_member_id != member.member_id
                or lease.source_id != member.source_id
                or lease.source_epoch != member.source_epoch
            ):
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.LEASE_REQUIRED,
                    "member does not hold the current control lease",
                )
            renewed = MultiplayerControlLease(
                holder_member_id=lease.holder_member_id,
                source_id=lease.source_id,
                source_epoch=lease.source_epoch,
                acquired_at_monotonic_ns=lease.acquired_at_monotonic_ns,
                expires_at_monotonic_ns=now_ns + ttl_ns,
            )
            self._lease = renewed
            return renewed

    def release_control(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
    ) -> bool:
        """Release control only when ``member_id`` is the current holder."""

        _safe_id(member_id, "member_id")
        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        with self._lock:
            member = self._require_member_locked(member_id)
            self._require_source_locked(
                member,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            self._expire_lease_locked(self._now_ns())
            if self._lease is None or self._lease.holder_member_id != member_id:
                return False
            self._lease = None
            return True

    def require_control_authority(
        self,
        *,
        member_id: str,
        source_id: str,
        source_epoch: int,
    ) -> MultiplayerMembership:
        """Return the current member or reject an unauthorised control source."""

        _safe_id(member_id, "member_id")
        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        with self._lock:
            member = self._require_member_locked(member_id)
            self._require_source_locked(
                member,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            self._expire_lease_locked(self._now_ns())
            lease = self._lease
            if (
                lease is None
                or lease.holder_member_id != member.member_id
                or lease.source_id != member.source_id
                or lease.source_epoch != member.source_epoch
            ):
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.LEASE_REQUIRED,
                    "member does not hold an active control lease",
                )
            return member

    def require_control_source(
        self,
        *,
        source_id: str,
        source_epoch: int,
    ) -> MultiplayerMembership:
        """Return the lease-holding driver bound to one transport source."""

        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        with self._lock:
            member = self._require_source_member_locked(
                source_id=source_id,
                source_epoch=source_epoch,
            )
            if member.role is not MultiplayerRole.DRIVER:
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.ROLE_DENIED,
                    "observer members cannot submit control",
                )
            self._expire_lease_locked(self._now_ns())
            lease = self._lease
            if (
                lease is None
                or lease.holder_member_id != member.member_id
                or lease.source_id != member.source_id
                or lease.source_epoch != member.source_epoch
            ):
                raise MultiplayerRoomError(
                    MultiplayerRoomErrorCode.LEASE_REQUIRED,
                    "control source does not hold an active lease",
                )
            return member

    def require_source_member(
        self,
        *,
        source_id: str,
        source_epoch: int,
    ) -> MultiplayerMembership:
        """Return the current room member bound to a transport source."""

        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        with self._lock:
            return self._require_source_member_locked(
                source_id=source_id,
                source_epoch=source_epoch,
            )

    def require_member_source(
        self,
        *,
        member_id: str,
        source_id: str,
        source_epoch: int,
    ) -> MultiplayerMembership:
        """Return a current member bound to the exact connection source."""

        _safe_id(member_id, "member_id")
        _safe_id(source_id, "source_id")
        _positive_int(source_epoch, "source_epoch")
        with self._lock:
            member = self._require_member_locked(member_id)
            self._require_source_locked(
                member,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            return member

    def snapshot(self) -> dict[str, Any]:
        """Return a deterministic, read-only public room projection."""

        with self._lock:
            now_ns = self._now_ns()
            self._expire_lease_locked(now_ns)
            lease = self._lease
            lease_document: dict[str, str | int] | None = None
            if lease is not None:
                lease_document = {
                    "holder_member_id": lease.holder_member_id,
                    "source_id": lease.source_id,
                    "source_epoch": lease.source_epoch,
                    "expires_in_ms": max(
                        0,
                        (lease.expires_at_monotonic_ns - now_ns) // 1_000_000,
                    ),
                }
            return {
                "schema": MULTIPLAYER_ROOM_STATE_SCHEMA,
                "run_id": self._identity.run_id,
                "session_id": self._identity.session_id,
                "boot_id": self._identity.boot_id,
                "members": [
                    self._members[key].to_dict() for key in sorted(self._members)
                ],
                "control_lease": lease_document,
            }

    def _require_member_locked(self, member_id: str) -> MultiplayerMembership:
        _safe_id(member_id, "member_id")
        member = self._members.get(member_id)
        if member is None:
            raise MultiplayerRoomError(
                MultiplayerRoomErrorCode.MEMBER_UNKNOWN,
                f"room member is unknown: {member_id!r}",
            )
        return member

    def _require_source_locked(
        self,
        member: MultiplayerMembership,
        *,
        source_id: str,
        source_epoch: int,
    ) -> None:
        if source_id != member.source_id:
            raise MultiplayerRoomError(
                MultiplayerRoomErrorCode.SOURCE_CONFLICT,
                "source_id does not match the admitted member",
            )
        if source_epoch != member.source_epoch:
            raise MultiplayerRoomError(
                MultiplayerRoomErrorCode.SOURCE_EPOCH_STALE,
                "source_epoch does not match the active member epoch",
            )

    def _require_source_member_locked(
        self,
        *,
        source_id: str,
        source_epoch: int,
    ) -> MultiplayerMembership:
        member = next(
            (
                candidate
                for candidate in self._members.values()
                if candidate.source_id == source_id
            ),
            None,
        )
        if member is None:
            raise MultiplayerRoomError(
                MultiplayerRoomErrorCode.MEMBER_UNKNOWN,
                "source is not an admitted room member",
            )
        self._require_source_locked(
            member,
            source_id=source_id,
            source_epoch=source_epoch,
        )
        return member

    def _expire_lease_locked(self, now_ns: int) -> None:
        if self._lease is not None and now_ns >= self._lease.expires_at_monotonic_ns:
            self._lease = None

    def _now_ns(self) -> int:
        value = self._monotonic_ns()
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise RuntimeError("monotonic_ns must return a non-negative integer")
        return value


def _safe_id(value: object, field: str) -> str:
    if not isinstance(value, str) or _SAFE_ID_RE.fullmatch(value) is None:
        raise ValueError(f"{field} must be a safe non-empty identifier")
    return value


def _positive_int(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _lease_ttl_ns(ttl_ms: object) -> int:
    value = _positive_int(ttl_ms, "ttl_ms")
    if value > MAX_CONTROL_LEASE_TTL_MS:
        raise ValueError(
            f"ttl_ms must be no greater than {MAX_CONTROL_LEASE_TTL_MS}"
        )
    return value * 1_000_000
