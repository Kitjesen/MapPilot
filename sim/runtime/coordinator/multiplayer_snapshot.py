"""Latest-wins authoritative truth fan-out for multiplayer room members."""

from __future__ import annotations

import copy
import threading
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .live_snapshot import truth_snapshot_document
from .multiplayer_room import MultiplayerRoomAuthority, MultiplayerRoomError

MULTIPLAYER_TRUTH_ENVELOPE_SCHEMA = "lingtu.sim.multiplayer-truth-envelope.v1"


@dataclass(frozen=True, slots=True)
class MultiplayerSnapshotSubscriberState:
    """Read-only delivery counters for one room member."""

    member_id: str
    source_id: str
    source_epoch: int
    pending_snapshot: bool
    dropped_snapshots: int
    delivered_snapshots: int


@dataclass(slots=True)
class _Subscriber:
    member_id: str
    source_id: str
    source_epoch: int
    pending: dict[str, Any] | None = None
    dropped_snapshots: int = 0
    delivered_snapshots: int = 0


class MultiplayerSnapshotFanout:
    """Publish immutable truth to independent latest-value member slots."""

    def __init__(self, room: MultiplayerRoomAuthority) -> None:
        if not isinstance(room, MultiplayerRoomAuthority):
            raise TypeError("room must be a MultiplayerRoomAuthority")
        self._room = room
        self._subscribers: dict[str, _Subscriber] = {}
        self._latest: dict[str, Any] | None = None
        self._last_order: tuple[int, int, int] | None = None
        self._lock = threading.RLock()

    def subscribe(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
        replay_latest: bool = True,
    ) -> MultiplayerSnapshotSubscriberState:
        """Bind one current room source to a latest-value delivery slot."""

        self._room.require_member_source(
            member_id=member_id,
            source_id=source_id,
            source_epoch=source_epoch,
        )
        if not isinstance(replay_latest, bool):
            raise TypeError("replay_latest must be bool")
        with self._lock:
            subscriber = _Subscriber(
                member_id=member_id,
                source_id=source_id,
                source_epoch=source_epoch,
                pending=(
                    None
                    if not replay_latest or self._latest is None
                    else copy.deepcopy(self._latest)
                ),
            )
            self._subscribers[member_id] = subscriber
            return _subscriber_state(subscriber)

    def unsubscribe(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
    ) -> bool:
        """Remove only the subscription owned by the exact current source."""

        self._room.require_member_source(
            member_id=member_id,
            source_id=source_id,
            source_epoch=source_epoch,
        )
        with self._lock:
            subscriber = self._subscribers.get(member_id)
            if subscriber is None:
                return False
            _require_subscription_source(
                subscriber,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            self._subscribers.pop(member_id)
            return True

    def publish(self, event: Mapping[str, Any]) -> int:
        """Publish one strictly newer truth snapshot without blocking consumers."""

        truth = truth_snapshot_document(event)
        identity = self._room.identity
        if truth.get("session_id") != identity.session_id:
            raise ValueError("truth session_id does not match the multiplayer room")
        order = (
            _nonnegative_int(truth.get("model_generation"), "model_generation"),
            _nonnegative_int(truth.get("reset_generation"), "reset_generation"),
            _nonnegative_int(truth.get("sequence"), "sequence"),
        )
        envelope = {
            "schema": MULTIPLAYER_TRUTH_ENVELOPE_SCHEMA,
            "run_id": identity.run_id,
            "session_id": identity.session_id,
            "boot_id": identity.boot_id,
            "truth": copy.deepcopy(truth),
        }
        with self._lock:
            if self._last_order is not None and order <= self._last_order:
                raise ValueError("truth order must strictly advance")
            active_subscribers: list[_Subscriber] = []
            stale_member_ids: list[str] = []
            for member_id, subscriber in self._subscribers.items():
                try:
                    self._room.require_member_source(
                        member_id=member_id,
                        source_id=subscriber.source_id,
                        source_epoch=subscriber.source_epoch,
                    )
                except MultiplayerRoomError:
                    stale_member_ids.append(member_id)
                    continue
                active_subscribers.append(subscriber)
            for member_id in stale_member_ids:
                self._subscribers.pop(member_id, None)

            self._last_order = order
            self._latest = envelope
            for subscriber in active_subscribers:
                if subscriber.pending is not None:
                    subscriber.dropped_snapshots += 1
                subscriber.pending = envelope
            return len(active_subscribers)

    def take_latest(
        self,
        member_id: str,
        *,
        source_id: str,
        source_epoch: int,
    ) -> dict[str, Any] | None:
        """Take and clear one member's latest pending truth envelope."""

        self._room.require_member_source(
            member_id=member_id,
            source_id=source_id,
            source_epoch=source_epoch,
        )
        with self._lock:
            subscriber = self._subscribers.get(member_id)
            if subscriber is None:
                raise ValueError(f"member has no snapshot subscription: {member_id!r}")
            _require_subscription_source(
                subscriber,
                source_id=source_id,
                source_epoch=source_epoch,
            )
            pending = subscriber.pending
            if pending is None:
                return None
            subscriber.pending = None
            subscriber.delivered_snapshots += 1
            return copy.deepcopy(pending)

    def subscriber_state(
        self,
        member_id: str,
    ) -> MultiplayerSnapshotSubscriberState:
        """Return counters for one subscribed member."""

        with self._lock:
            subscriber = self._subscribers.get(member_id)
            if subscriber is None:
                raise ValueError(f"member has no snapshot subscription: {member_id!r}")
            return _subscriber_state(subscriber)


def _subscriber_state(
    subscriber: _Subscriber,
) -> MultiplayerSnapshotSubscriberState:
    return MultiplayerSnapshotSubscriberState(
        member_id=subscriber.member_id,
        source_id=subscriber.source_id,
        source_epoch=subscriber.source_epoch,
        pending_snapshot=subscriber.pending is not None,
        dropped_snapshots=subscriber.dropped_snapshots,
        delivered_snapshots=subscriber.delivered_snapshots,
    )


def _require_subscription_source(
    subscriber: _Subscriber,
    *,
    source_id: str,
    source_epoch: int,
) -> None:
    if subscriber.source_id != source_id or subscriber.source_epoch != source_epoch:
        raise ValueError("snapshot subscription belongs to a different source")


def _nonnegative_int(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"truth {field} must be a non-negative integer")
    return value


__all__ = [
    "MULTIPLAYER_TRUTH_ENVELOPE_SCHEMA",
    "MultiplayerSnapshotFanout",
    "MultiplayerSnapshotSubscriberState",
]
