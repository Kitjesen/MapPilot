"""runtime.transport.adapter — Bridge TransportABC backends to Transport Protocol.

Solves the two-API problem: Out[T]/In[T] ports use the simple Transport
Protocol (publish/subscribe/close with string topics), while DDS/SHM backends
implement TransportABC (factory pattern with TopicConfig objects).

This adapter wraps any TransportABC and presents it as a Transport Protocol
implementation, so modules can use DDS or SHM without knowing the difference.

Usage::

    from runtime.transport.adapter import TransportAdapter
    from runtime.transport.shm import SHMTransport

    # Wrap SHM backend as simple Transport
    shm = TransportAdapter(SHMTransport())

    # Now usable by Out[T] ports
    out_port._bind_transport(shm, "/slam/odometry")
    shm.subscribe("/slam/odometry", callback)
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
        topic_serializer: Callable[[str, Any], bytes] | None = None,
        deserializer: Callable[[bytes], Any] | None = None,
        backend_msg_type: Any = None,
        topic_msg_type: Callable[[str], Any] | None = None,
        topic_encoder: Callable[[str, Any], Any] | None = None,
        topic_decoder: Callable[[str, Any], Any] | None = None,
        forbidden_topics: set[str] | frozenset[str] | None = None,
    ) -> None:
        self._backend = backend
        self._default_qos_depth = default_qos_depth
        self._default_reliable = default_reliable
        self._backend_msg_type = backend_msg_type
        self._topic_msg_type = topic_msg_type
        self._topic_encoder = topic_encoder
        self._topic_decoder = topic_decoder
        self._forbidden_topics = frozenset(forbidden_topics or ())
        self._serializer = serializer or pickle.dumps
        self._topic_serializer = topic_serializer
        self._force_serialize = serializer is not None or topic_serializer is not None
        self._deserializer = deserializer or pickle.loads
        self._publishers: dict[str, Publisher] = {}
        self._subscribers: dict[str, list[Subscriber]] = {}

    def publish(self, topic: str, msg: Any) -> None:
        """Publish a message to a topic.

        Creates a Publisher on first use for this topic.
        If the backend expects raw bytes (SHM), the message is serialized.
        """
        self._check_topic(topic)
        pub = self._publishers.get(topic)
        if pub is None:
            config = TopicConfig(
                name=topic,
                msg_type=self._msg_type_for_topic(topic),
                qos_depth=self._default_qos_depth,
                reliable=self._default_reliable,
            )
            pub = self._backend.create_publisher(config)
            self._publishers[topic] = pub

        if self._topic_encoder is not None:
            msg_type = self._msg_type_for_topic(topic)
            if msg_type is not None and msg_type is not self._backend_msg_type:
                pub.publish(self._topic_encoder(topic, msg))
                return

        if self._force_serialize:
            pub.publish(self._encode(topic, msg))
            return

        # Legacy mode: try publishing as-is first; if it fails with bytes
        # requirement, serialize and retry.
        try:
            pub.publish(msg)
        except (TypeError, AttributeError):
            pub.publish(self._encode(topic, msg))

    def subscribe(self, topic: str, cb: Callable[[Any], None]) -> None:
        """Subscribe to a topic.

        Creates a Subscriber that forwards messages to *cb*.
        For SHM backends, received bytes are deserialized before delivery.
        """
        self._check_topic(topic)
        def _on_message(*args):
            # SHM callback: (data: bytes, ts: float)
            # DDS callback: (msg,)
            if args and isinstance(args[0], (bytes, bytearray)):
                data = args[0]
                try:
                    msg = self._deserializer(data)
                except Exception:
                    if self._force_serialize:
                        logger.warning(
                            "TransportAdapter dropped invalid payload on '%s'",
                            topic,
                            exc_info=True,
                        )
                        return
                    msg = data  # legacy mode keeps raw bytes for old debug paths
            elif len(args) == 1:
                msg = args[0]
            else:
                msg = args[0] if args else None
            if self._topic_decoder is not None:
                msg_type = self._msg_type_for_topic(topic)
                if msg_type is not None and msg_type is not self._backend_msg_type:
                    try:
                        msg = self._topic_decoder(topic, msg)
                    except Exception:
                        logger.warning(
                            "TransportAdapter dropped invalid typed payload on '%s'",
                            topic,
                            exc_info=True,
                        )
                        return
            try:
                cb(msg)
            except Exception:
                logger.exception("TransportAdapter callback error on '%s'", topic)

        config = TopicConfig(
            name=topic,
            msg_type=self._msg_type_for_topic(topic),
            qos_depth=self._default_qos_depth,
            reliable=self._default_reliable,
        )
        sub = self._backend.create_subscriber(config, _on_message)
        if hasattr(sub, 'start'):
            sub.start()
        self._subscribers.setdefault(topic, []).append(sub)

    def _encode(self, topic: str, msg: Any) -> bytes:
        if self._topic_serializer is not None:
            return self._topic_serializer(topic, msg)
        return self._serializer(msg)

    def _check_topic(self, topic: str) -> None:
        if str(topic) not in self._forbidden_topics:
            return
        raise ValueError(
            f"{self._backend.name} generic transport adapter cannot carry "
            f"registered product topic {topic}; use a typed adapter or typed "
            "publisher/subscriber"
        )

    def _msg_type_for_topic(self, topic: str) -> Any:
        if self._topic_msg_type is None:
            return self._backend_msg_type
        return self._topic_msg_type(topic) or self._backend_msg_type

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
