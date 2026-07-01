"""Optional LCM transport backend.

LCM is treated as an external IPC boundary. Modules continue to publish and
receive normal Python objects through Module ports; serialization happens in
``TransportAdapter`` before data reaches this backend.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

from .abc import Publisher, Subscriber, TopicConfig, TransportABC

logger = logging.getLogger(__name__)


try:
    import lcm as _lcm

    _LCM_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised through monkeypatch tests
    _lcm = None
    _LCM_AVAILABLE = False


class LCMPublisher(Publisher):
    """LCM publisher for serialized byte payloads."""

    def __init__(self, topic: TopicConfig, client: Any):
        super().__init__(topic)
        self._client = client

    def publish(self, msg: Any) -> None:
        if not isinstance(msg, (bytes, bytearray)):
            raise TypeError("LCMPublisher expects bytes; use TransportAdapter")
        self._client.publish(self._topic.name, bytes(msg))


class LCMSubscriber(Subscriber):
    """LCM subscriber wrapper."""

    def __init__(self, topic: TopicConfig, callback: Callable, client: Any):
        super().__init__(topic, callback)
        self._client = client
        self._subscription = None

    def start(self) -> None:
        if self._subscription is not None:
            return
        self._subscription = self._client.subscribe(
            self._topic.name,
            self._on_lcm_message,
        )

    def close(self) -> None:
        if self._subscription is None:
            return
        unsubscribe = getattr(self._client, "unsubscribe", None)
        if callable(unsubscribe):
            try:
                unsubscribe(self._subscription)
            except (RuntimeError, OSError, ValueError):
                logger.debug("LCM unsubscribe failed", exc_info=True)
        self._subscription = None

    def _on_lcm_message(self, channel: str, data: bytes) -> None:
        del channel
        self._callback(bytes(data), time.time())


class LCMTransport(TransportABC):
    """LCM transport backend.

    Parameters
    ----------
    url:
        Optional LCM URL, for example ``udpm://239.255.76.67:7667?ttl=1``.
        ``None`` uses the LCM package default.
    poll_timeout_ms:
        Timeout passed to ``handle_timeout`` in the background receive loop.
    """

    def __init__(self, url: str | None = None, poll_timeout_ms: int = 20):
        if not _LCM_AVAILABLE:
            raise ImportError(
                "lcm is not installed. Install it only for LCM IPC deployments."
            )
        self._client = _lcm.LCM(url) if url else _lcm.LCM()
        self._poll_timeout_ms = max(1, int(poll_timeout_ms))
        self._publishers: list[LCMPublisher] = []
        self._subscribers: list[LCMSubscriber] = []
        self._closed = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

    def create_publisher(self, topic: TopicConfig) -> LCMPublisher:
        pub = LCMPublisher(topic, self._client)
        self._publishers.append(pub)
        return pub

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> LCMSubscriber:
        sub = LCMSubscriber(topic, callback, self._client)
        sub.start()
        self._subscribers.append(sub)
        self._ensure_thread()
        return sub

    def close(self) -> None:
        self._closed.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)
        for sub in self._subscribers:
            sub.close()
        self._subscribers.clear()
        self._publishers.clear()

    @property
    def name(self) -> str:
        return "lcm"

    def _ensure_thread(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._thread = threading.Thread(
                target=self._poll_loop,
                daemon=True,
                name="lcm-transport",
            )
            self._thread.start()

    def _poll_loop(self) -> None:
        while not self._closed.is_set():
            try:
                handled = self._client.handle_timeout(self._poll_timeout_ms)
            except (RuntimeError, OSError) as exc:
                logger.warning("LCM handle loop error: %s", exc)
                time.sleep(0.05)
                continue
            if handled == 0:
                time.sleep(0.001)
