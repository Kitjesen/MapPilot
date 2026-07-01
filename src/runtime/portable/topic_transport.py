"""Bind portable topic contracts to LingTu transport backends.

This is the production path for ROS-topic replacement at adapter boundaries:
``PortableTopic`` supplies the canonical topic name/payload type, while the
existing mature transport layer supplies delivery (local, SHM, LCM, DDS).

Do not add a new broker here.  This module is a thin adapter over
``runtime.transport.factory``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from runtime.transport.abc import TransportStrategy
from runtime.transport.factory import create_transport_adapter

from .contracts import PortableCommandFrame, PortablePlanningFrame, PortableSensorFrame
from .topics import COMMAND_TOPICS, PLANNING_TOPICS, SENSOR_TOPICS, PortableTopic, topic_name


class PortableTopicTransport:
    """Publish/subscribe portable topics through an existing transport backend."""

    def __init__(self, strategy: TransportStrategy | str = TransportStrategy.LOCAL) -> None:
        self.strategy = strategy
        self._transport = create_transport_adapter(strategy)

    def publish(self, topic: str | PortableTopic[Any], payload: Any) -> None:
        self._transport.publish(topic_name(topic), payload)

    def subscribe(self, topic: str | PortableTopic[Any], callback: Callable[[Any], None]) -> None:
        self._transport.subscribe(topic_name(topic), callback)

    def publish_sensor_frame(self, frame: PortableSensorFrame) -> None:
        for spec in SENSOR_TOPICS.values():
            payload = getattr(frame, spec.field)
            if payload is not None:
                self.publish(spec, payload)

    def publish_command_frame(self, frame: PortableCommandFrame) -> None:
        for spec in COMMAND_TOPICS.values():
            payload = getattr(frame, spec.field)
            if payload is not None:
                self.publish(spec, payload)

    def publish_planning_frame(self, frame: PortablePlanningFrame) -> None:
        for spec in PLANNING_TOPICS.values():
            payload = getattr(frame, spec.field)
            if payload is not None:
                self.publish(spec, payload)

    def close(self) -> None:
        close = getattr(self._transport, "close", None)
        if callable(close):
            close()


def create_portable_topic_transport(
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
) -> PortableTopicTransport:
    """Create a portable topic transport over local/SHM/LCM/DDS."""

    return PortableTopicTransport(strategy)


def create_portable_publisher(
    topic: str | PortableTopic[Any],
    *,
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
) -> Callable[[Any], None]:
    """Return a simple publisher callable backed by the selected transport."""

    transport = create_portable_topic_transport(strategy)

    def publish(payload: Any) -> None:
        transport.publish(topic, payload)

    setattr(publish, "transport", transport)
    return publish


def create_portable_subscriber(
    topic: str | PortableTopic[Any],
    callback: Callable[[Any], None],
    *,
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
) -> PortableTopicTransport:
    """Subscribe to a portable topic through an existing transport backend."""

    transport = create_portable_topic_transport(strategy)
    transport.subscribe(topic, callback)
    return transport
