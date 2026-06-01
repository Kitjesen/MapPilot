"""core.transport.adapter — Bridge TransportABC backends to Transport Protocol.

Solves the two-API problem: Out[T]/In[T] ports use the simple Transport
Protocol (publish/subscribe/close with string topics), while DDS/SHM backends
implement TransportABC (factory pattern with TopicConfig objects).

This adapter wraps any TransportABC and presents it as a Transport Protocol
implementation, so modules can use DDS or SHM without knowing the difference.

Usage::

    from core.transport.adapter import TransportAdapter
    from core.transport.shm import SHMTransport

    # Wrap SHM backend as simple Transport
    shm = TransportAdapter(SHMTransport())

    # Now usable by Out[T] ports
    out_port._bind_transport(shm, "/nav/odometry")
    shm.subscribe("/nav/odometry", callback)
"""

import logging
import pickle
from collections.abc import Callable
from typing import Any

from .abc import Publisher, Subscriber, TopicConfig, TransportABC

logger = logging.getLogger(__name__)


class TransportAdapter:
    """Adapts a TransportABC backend to the Transport Protocol interface.

    Maintains a registry of publishers and subscribers keyed by topic name.
    Lazily creates Publisher/Subscriber instances on first use per topic.
    """

    def __init__(
        self,
        backend: TransportABC,
        default_qos_depth: int = 10,
        default_reliable: bool = False,
        serializer: Callable[[Any], bytes] | None = None,
        deserializer: Callable[[bytes], Any] | None = None,
    ) -> None:
        self._backend = backend
        self._default_qos_depth = default_qos_depth
        self._default_reliable = default_reliable
        self._serializer = serializer or pickle.dumps
        self._deserializer = deserializer or pickle.loads
        self._publishers: dict[str, Publisher] = {}
        self._subscribers: dict[str, list[Subscriber]] = {}

    def publish(self, topic: str, msg: Any) -> None:
        """Publish a message to a topic.

        Creates a Publisher on first use for this topic.
        If the backend expects raw bytes (SHM), the message is serialized.
        """
        pub = self._publishers.get(topic)
        if pub is None:
            config = TopicConfig(
                name=topic,
                qos_depth=self._default_qos_depth,
                reliable=self._default_reliable,
            )
            pub = self._backend.create_publisher(config)
            self._publishers[topic] = pub

        # SHM expects bytes; DDS expects typed messages.
        # Try publishing as-is first; if it fails with bytes requirement,
        # serialize and retry.
        try:
            pub.publish(msg)
        except (TypeError, AttributeError):
            pub.publish(self._serializer(msg))

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Subscribe to a topic.

        Creates a Subscriber that forwards messages to *cb*.
        For SHM backends, received bytes are deserialized before delivery.
        """
        def _on_message(*args):
            # SHM callback: (data: bytes, ts: float)
            # DDS callback: (msg,)
            if len(args) == 2 and isinstance(args[0], (bytes, bytearray)):
                data, _ts = args
                try:
                    msg = self._deserializer(data)
                except Exception:
                    msg = data  # pass raw bytes if deserialization fails
            elif len(args) == 1:
                msg = args[0]
            else:
                msg = args[0] if args else None
            try:
                cb(msg)
            except Exception:
                logger.exception("TransportAdapter callback error on '%s'", topic)

        config = TopicConfig(
            name=topic,
            qos_depth=self._default_qos_depth,
            reliable=self._default_reliable,
        )
        sub = self._backend.create_subscriber(config, _on_message)
        if hasattr(sub, 'start'):
            sub.start()
        self._subscribers.setdefault(topic, []).append(sub)

    def close(self) -> None:
        """Close all publishers/subscribers and the backend."""
        for pub in self._publishers.values():
            try:
                pub.close()
            except (RuntimeError, OSError):
                pass
        for subs in self._subscribers.values():
            for sub in subs:
                try:
                    sub.close()
                except (RuntimeError, OSError):
                    pass
        self._publishers.clear()
        self._subscribers.clear()
        try:
            self._backend.close()
        except (RuntimeError, OSError):
            pass

    @property
    def backend_name(self) -> str:
        """Name of the wrapped backend."""
        return self._backend.name

    def __repr__(self) -> str:
        return (
            f"TransportAdapter({self._backend.name}, "
            f"pubs={len(self._publishers)}, "
            f"subs={sum(len(s) for s in self._subscribers.values())})"
        )
