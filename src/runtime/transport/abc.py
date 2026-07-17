"""Abstract base classes for pluggable transport backends.

Provides the ABC hierarchy used by Local, SHM, and DDS transport
backends. Upper layers communicate through Publisher / Subscriber without
knowing the underlying implementation.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from typing import Any


class TransportStrategy(Enum):
    """Transport strategy selection."""

    LOCAL = "local"
    DDS = "dds"
    SHM = "shm"
    AUTO = "auto"


@dataclass
class TopicConfig:
    """Topic configuration."""

    name: str
    msg_type: Any = None
    strategy: TransportStrategy = TransportStrategy.LOCAL
    buffer_size: int = 0
    qos_depth: int = 10
    reliable: bool = False
    # Optional named QoS profile from config/qos_profiles.yaml. When set it
    # takes priority over qos_depth/reliable on DDS backends. None keeps the
    # existing default behavior (opt-in).
    qos_profile: str | None = None


class Publisher(ABC):
    """Message publisher."""

    def __init__(self, topic: TopicConfig):
        self._topic = topic

    @property
    def topic_name(self) -> str:
        return self._topic.name

    @abstractmethod
    def publish(self, msg: Any) -> None: ...

    def close(self) -> None:
        pass


class Subscriber(ABC):
    """Message subscriber."""

    def __init__(self, topic: TopicConfig, callback: Callable):
        self._topic = topic
        self._callback = callback

    @property
    def topic_name(self) -> str:
        return self._topic.name

    @abstractmethod
    def start(self) -> None: ...

    def close(self) -> None:
        pass


class TransportABC(ABC):
    """Transport layer factory."""

    @abstractmethod
    def create_publisher(self, topic: TopicConfig) -> Publisher: ...

    @abstractmethod
    def create_subscriber(self, topic: TopicConfig, callback: Callable) -> Subscriber: ...

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def name(self) -> str: ...
