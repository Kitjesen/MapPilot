"""Pure CycloneDDS transport backend.

Registered product topics use typed DDS classes from ``src/message``.
Unregistered topics keep the RawMessage JSON envelope as a legacy test/debug
path.
"""

import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from message.dds import dds_topic_name, dds_type_for_topic, topic_spec

from .abc import Publisher, Subscriber, TopicConfig, TransportABC
from .json_codec import dumps_message, loads_message

logger = logging.getLogger(__name__)

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
    IdlStruct = object


if _CYCLONE_AVAILABLE:
    from cyclonedds.idl.types import sequence, uint8

    @dataclass
    class RawMessage(IdlStruct, typename="lingtu.RawMessage"):
        data: sequence[uint8]
        timestamp: float = 0.0
else:

    @dataclass
    class RawMessage:  # type: ignore[no-redef]
        data: bytes
        timestamp: float = 0.0


class DDSPublisher(Publisher):
    """DDS publisher for typed product topics and legacy RawMessage topics."""

    def __init__(self, topic: TopicConfig, participant: Any, dds_topic: Any, dds_type: Any):
        super().__init__(topic)
        self._dds_type = dds_type
        self._writer = DataWriter(participant, dds_topic)
        logger.info("[DDS-Pub] created on topic '%s'", topic.name)

    def publish(self, msg: Any) -> None:
        if self._dds_type is not RawMessage:
            if not isinstance(msg, self._dds_type):
                expected = getattr(self._dds_type, "__name__", str(self._dds_type))
                raise TypeError(f"{self.topic_name} expects typed DDS {expected}")
            self._writer.write(msg)
            return

        data = msg if isinstance(msg, bytes) else dumps_message(msg, topic=self.topic_name)
        self._writer.write(RawMessage(data=list(data), timestamp=time.time()))

    def close(self) -> None:
        try:
            self._writer.close()
        except (RuntimeError, OSError):
            pass


if _CYCLONE_AVAILABLE:

    class _MessageListener(Listener):
        def __init__(self, callback: Callable, *, legacy_raw: bool):
            super().__init__()
            self._callback = callback
            self._legacy_raw = legacy_raw

        def on_data_available(self, reader) -> None:  # type: ignore[override]
            for sample in reader.take():
                if sample is None:
                    continue
                if self._legacy_raw:
                    raw_bytes = bytes(sample.data)
                    try:
                        msg = loads_message(raw_bytes)
                    except Exception:
                        msg = raw_bytes
                else:
                    msg = sample
                try:
                    self._callback(msg)
                except Exception:
                    logger.exception("[DDS-Sub] callback error")
else:
    _MessageListener = None  # type: ignore[assignment,misc]


class DDSSubscriber(Subscriber):
    """DDS subscriber using a Listener for push-style delivery."""

    def __init__(
        self,
        topic: TopicConfig,
        callback: Callable,
        participant: Any,
        dds_topic: Any,
        dds_type: Any,
    ):
        super().__init__(topic, callback)
        self._listener = _MessageListener(callback, legacy_raw=dds_type is RawMessage)
        self._reader = DataReader(participant, dds_topic, listener=self._listener)
        logger.info("[DDS-Sub] created on topic '%s'", topic.name)

    def start(self) -> None:
        pass

    def close(self) -> None:
        try:
            self._reader.close()
        except (RuntimeError, OSError):
            pass


class DDSTransport(TransportABC):
    """CycloneDDS transport layer."""

    def __init__(self, domain_id: int = 0):
        if not _CYCLONE_AVAILABLE:
            raise ImportError(
                "cyclonedds-python is not installed. Run: pip install cyclonedds"
            )
        self._participant = DomainParticipant(domain_id)
        self._publishers: list[DDSPublisher] = []
        self._subscribers: list[DDSSubscriber] = []
        self._topics: dict[tuple[str, Any], Any] = {}
        logger.info("[DDSTransport] domain_id=%s", domain_id)

    def _topic_type(self, topic: TopicConfig) -> Any:
        if topic.msg_type is not None:
            return topic.msg_type
        if topic_spec(topic.name) is None:
            return RawMessage
        dds_type = dds_type_for_topic(topic.name)
        if dds_type is None:
            raise ImportError(f"typed DDS contract for {topic.name} is unavailable")
        return dds_type

    def _get_or_create_topic(self, topic: TopicConfig) -> tuple[Any, Any]:
        dds_type = self._topic_type(topic)
        key = (topic.name, dds_type)
        if key not in self._topics:
            self._topics[key] = Topic(
                self._participant,
                dds_topic_name(topic.name, typed=dds_type is not RawMessage),
                dds_type,
            )
        return self._topics[key], dds_type

    def create_publisher(self, topic: TopicConfig) -> DDSPublisher:
        dds_topic, dds_type = self._get_or_create_topic(topic)
        pub = DDSPublisher(topic, self._participant, dds_topic, dds_type)
        self._publishers.append(pub)
        return pub

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> DDSSubscriber:
        dds_topic, dds_type = self._get_or_create_topic(topic)
        sub = DDSSubscriber(topic, callback, self._participant, dds_topic, dds_type)
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
