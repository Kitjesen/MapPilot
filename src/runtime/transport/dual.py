"""runtime.transport.dual — SHM + DDS dual-write transport.

Writes to both SHM (low latency) and DDS (tool compatibility) on publish.
Use when real-time performance and ROS2 tool access are both required.
"""

import logging
from collections.abc import Callable
from typing import Any

from .abc import Publisher, Subscriber, TopicConfig, TransportABC

logger = logging.getLogger(__name__)


class DualPublisher(Publisher):
    """Dual-write publisher: SHM + DDS."""

    def __init__(self, shm_pub: Publisher, dds_pub: Publisher):
        super().__init__(shm_pub._topic)
        self._shm = shm_pub
        self._dds = dds_pub

    def publish(self, msg: Any) -> None:
        # SHM fast path (writes bytes/numpy directly)
        self._shm.publish(msg)
        # DDS compatibility path (sends ROS2 msg as-is)
        self._dds.publish(msg)

    def close(self) -> None:
        self._shm.close()
        self._dds.close()


class DualSubscriber(Subscriber):
    """Dual-read subscriber: prefer SHM, DDS as fallback."""

    def __init__(self, shm_sub: Subscriber, dds_sub: Subscriber):
        super().__init__(shm_sub._topic, shm_sub._callback)
        self._shm = shm_sub
        self._dds = dds_sub
        self._use_shm = True

    def start(self) -> None:
        self._shm.start()
        # DDS also started but only used if SHM fails
        self._dds.start()

    def close(self) -> None:
        self._shm.close()
        self._dds.close()


class DualTransport(TransportABC):
    """SHM + DDS dual-channel transport."""

    def __init__(self, shm_transport: TransportABC, dds_transport: TransportABC):
        self._shm = shm_transport
        self._dds = dds_transport

    def create_publisher(self, topic: TopicConfig) -> DualPublisher:
        return DualPublisher(
            self._shm.create_publisher(topic),
            self._dds.create_publisher(topic),
        )

    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> DualSubscriber:
        return DualSubscriber(
            self._shm.create_subscriber(topic, callback),
            self._dds.create_subscriber(topic, callback),
        )

    def close(self) -> None:
        self._shm.close()
        self._dds.close()

    @property
    def name(self) -> str:
        return "dual(shm+dds)"
