"""core.transport.dds — Pure CycloneDDS transport backend.

Uses the cyclonedds-python bindings directly (no rclpy / ROS2 dependency).
Messages are serialized with pickle, wrapped in a RawMessage IDL struct, and
exchanged over a DDS DomainParticipant.

Install:
    pip install cyclonedds

Classes:
    DDSTransport    — creates DomainParticipant, factory for pub/sub
    DDSPublisher    — wraps DataWriter on a RawMessage topic
    DDSSubscriber   — wraps DataReader with a Listener callback
"""

import logging
import pickle
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .abc import Publisher, Subscriber, TopicConfig, TransportABC

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Optional import guard — clear error if cyclonedds is missing
# ---------------------------------------------------------------------------

try:
    from cyclonedds.core import Listener
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.idl import IdlStruct
    from cyclonedds.pub import DataWriter
    from cyclonedds.sub import DataReader
    from cyclonedds.topic import Topic
    _CYCLONE_AVAILABLE = True
except ImportError:  # pragma: no cover
    _CYCLONE_AVAILABLE = False
    DomainParticipant = DataWriter = DataReader = Listener = None
    IdlStruct = object  # fallback so the dataclass below is definable


# ---------------------------------------------------------------------------
# IDL message type — generic bytes envelope
# ---------------------------------------------------------------------------

if _CYCLONE_AVAILABLE:
    from cyclonedds.idl.types import sequence, uint8

    @dataclass
    class RawMessage(IdlStruct, typename="lingtu.RawMessage"):
        """Generic bytes envelope for DDS transport.

        Uses sequence[uint8] instead of raw bytes because cyclonedds IDL
        doesn't support the Python bytes type directly.
        """
        data: sequence[uint8]
        timestamp: float = 0.0
else:
    @dataclass
    class RawMessage:  # type: ignore[no-redef]
        """Stub used when cyclonedds is not installed."""
        data: bytes
        timestamp: float = 0.0


# ---------------------------------------------------------------------------
# DDSPublisher
# ---------------------------------------------------------------------------

class DDSPublisher(Publisher):
    """DDS publisher — serializes messages with pickle and writes RawMessage."""

    def __init__(self, topic: TopicConfig, participant: Any, dds_topic: Any):
        super().__init__(topic)
        self._writer = DataWriter(participant, dds_topic)
        logger.info(f"[DDS-Pub] created on topic '{topic.name}'")

    def publish(self, msg: Any) -> None:
        """Publish *msg* to the DDS topic.

        Non-bytes values are serialized with pickle.  Raw bytes are wrapped
        without an extra pickle layer so subscribers can optionally skip
        deserialization.
        """
        if isinstance(msg, bytes):
            data = msg
        else:
            data = pickle.dumps(msg)
        # cyclonedds IDL uses sequence[uint8], so convert bytes → list[int]
        raw = RawMessage(data=list(data), timestamp=time.time())
        self._writer.write(raw)

    def close(self) -> None:
        try:
            self._writer.close()
        except (RuntimeError, OSError):
            pass


# ---------------------------------------------------------------------------
# DDSSubscriber
# ---------------------------------------------------------------------------

if _CYCLONE_AVAILABLE:
    class _RawMessageListener(Listener):
        """DDS Listener that forwards RawMessage arrivals to a user callback."""

        def __init__(self, callback: Callable):
            super().__init__()
            self._callback = callback

        def on_data_available(self, reader) -> None:  # type: ignore[override]
            for sample in reader.take():
                if sample is None:
                    continue
                # Convert sequence[uint8] back to bytes
                raw_bytes = bytes(sample.data)
                try:
                    msg = pickle.loads(raw_bytes)  # noqa: S301  # trusted intra-robot DDS
                except Exception:
                    # Deliver raw bytes if deserialisation fails
                    msg = raw_bytes
                try:
                    self._callback(msg)
                except Exception as exc:
                    logger.error(f"[DDS-Sub] callback error: {exc}")
else:
    _RawMessageListener = None  # type: ignore[assignment,misc]


class DDSSubscriber(Subscriber):
    """DDS subscriber — uses a Listener for push-style delivery."""

    def __init__(self, topic: TopicConfig, callback: Callable,
                 participant: Any, dds_topic: Any):
        super().__init__(topic, callback)
        self._listener = _RawMessageListener(callback)
        self._reader = DataReader(participant, dds_topic, listener=self._listener)
        logger.info(f"[DDS-Sub] created on topic '{topic.name}'")

    def start(self) -> None:
        # Listener is already active; nothing more to start.
        pass

    def close(self) -> None:
        try:
            self._reader.close()
        except (RuntimeError, OSError):
            pass


# ---------------------------------------------------------------------------
# DDSTransport
# ---------------------------------------------------------------------------

class DDSTransport(TransportABC):
    """CycloneDDS transport layer (no ROS2 required).

    Parameters
    ----------
    domain_id : int
        DDS domain identifier.  Default 0 matches the ROS2 default domain.
    """

    def __init__(self, domain_id: int = 0):
        if not _CYCLONE_AVAILABLE:
            raise ImportError(
                "cyclonedds-python is not installed. "
                "Run: pip install cyclonedds"
            )
        self._participant = DomainParticipant(domain_id)
        self._publishers: list = []
        self._subscribers: list = []
        # Cache DDS topics so pub and sub on the same name share one Topic object
        self._topics: dict = {}
        logger.info(f"[DDSTransport] domain_id={domain_id}")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _get_or_create_topic(self, topic_name: str) -> Any:
        """Return a cached DDS Topic for *topic_name*, creating if absent."""
        if topic_name not in self._topics:
            self._topics[topic_name] = Topic(
                self._participant, topic_name, RawMessage
            )
        return self._topics[topic_name]

    # ------------------------------------------------------------------
    # TransportABC interface
    # ------------------------------------------------------------------

    def create_publisher(self, topic: TopicConfig) -> DDSPublisher:
        dds_topic = self._get_or_create_topic(topic.name)
        pub = DDSPublisher(topic, self._participant, dds_topic)
        self._publishers.append(pub)
        return pub

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> DDSSubscriber:
        dds_topic = self._get_or_create_topic(topic.name)
        sub = DDSSubscriber(topic, callback, self._participant, dds_topic)
        sub.start()
        self._subscribers.append(sub)
        return sub

    def close(self) -> None:
        for sub in self._subscribers:
            sub.close()
        for pub in self._publishers:
            pub.close()
        self._subscribers.clear()
        self._publishers.clear()
        try:
            self._participant.close()
        except (RuntimeError, OSError):
            pass

    @property
    def name(self) -> str:
        return "dds"
