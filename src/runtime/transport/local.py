"""Transport protocol and in-process LocalTransport bus."""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from typing import Any, Protocol, runtime_checkable

logger = logging.getLogger(__name__)


@runtime_checkable
class Transport(Protocol):
    """Minimal protocol used by module ports."""

    def publish(self, topic: str, msg: Any) -> None:
        """Publish a message to a topic."""
        ...

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Subscribe to a topic."""
        ...

    def close(self) -> None:
        """Release resources."""
        ...


class LocalTransport:
    """Synchronous zero-copy bus for single-process mode and tests."""

    def __init__(self) -> None:
        self._bus: dict[str, list[Callable[[Any], None]]] = {}
        self._lock = threading.Lock()

    def publish(self, topic: str, msg: Any) -> None:
        """Publish a message, synchronously invoking all subscriber callbacks."""
        with self._lock:
            callbacks = list(self._bus.get(topic, []))
        for cb in callbacks:
            try:
                cb(msg)
            except Exception:
                logger.exception("LocalTransport callback error on topic '%s'", topic)

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Subscribe to a topic."""
        with self._lock:
            self._bus.setdefault(topic, []).append(cb)

    def unsubscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Unsubscribe from a topic."""
        with self._lock:
            cbs = self._bus.get(topic)
            if cbs and cb in cbs:
                cbs.remove(cb)

    def close(self) -> None:
        """Clear all subscriptions."""
        with self._lock:
            self._bus.clear()

    @property
    def topics(self) -> list[str]:
        """Return a list of currently active topics."""
        with self._lock:
            return list(self._bus.keys())

    @property
    def name(self) -> str:
        return "local"

    def subscriber_count(self, topic: str) -> int:
        """Return the number of subscribers on a topic."""
        with self._lock:
            return len(self._bus.get(topic, []))

    def __repr__(self) -> str:
        return f"LocalTransport(topics={len(self._bus)})"
