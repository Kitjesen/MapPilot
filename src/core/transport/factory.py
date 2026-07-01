"""Transport factory for LingTu module communication.

Default creation is product-neutral and ROS-free: LocalTransport for in-process
graphs, SHM for same-host IPC, optional LCM for lightweight cross-process IPC,
and DDS only for compatibility windows.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

from .abc import (
    Publisher,
    Subscriber,
    TopicConfig,
    TransportStrategy,
)
from .local import LocalTransport

logger = logging.getLogger(__name__)

# The process-local bus is intentionally shared. It makes the default factory
# shortcuts behave like a single in-process message fabric without ROS/DDS.
_DEFAULT_LOCAL_TRANSPORT = LocalTransport()


def create_transport(
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
    ros_node=None,
) -> Any:
    """Create a transport instance by strategy.

    ``ros_node`` is kept for old call sites. DDS transport currently uses the
    native CycloneDDS backend and does not require rclpy.
    """

    strategy = _coerce_strategy(strategy)

    if strategy == TransportStrategy.LOCAL:
        return _DEFAULT_LOCAL_TRANSPORT

    if strategy == TransportStrategy.SHM:
        from .shm import SHMTransport

        return SHMTransport()

    if strategy == TransportStrategy.LCM:
        from .lcm import LCMTransport

        return LCMTransport()

    if strategy == TransportStrategy.DDS:
        from .dds import DDSTransport

        return DDSTransport()

    if strategy == TransportStrategy.DUAL:
        from .dds import DDSTransport
        from .dual import DualTransport
        from .shm import SHMTransport

        return DualTransport(SHMTransport(), DDSTransport())

    if strategy == TransportStrategy.AUTO:
        from .shm import SHMTransport

        try:
            return SHMTransport()
        except Exception:
            logger.debug("AUTO transport could not create SHM", exc_info=True)
            if ros_node is None:
                return _DEFAULT_LOCAL_TRANSPORT
            from .dds import DDSTransport

            return DDSTransport()

    raise ValueError(f"Unknown strategy: {strategy}")


def create_transport_adapter(
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
    ros_node=None,
) -> Any:
    """Create a simple pub/sub adapter for a TransportABC backend."""

    strategy = _coerce_strategy(strategy)
    transport = create_transport(strategy, ros_node)
    if hasattr(transport, "publish") and hasattr(transport, "subscribe"):
        return transport

    from .adapter import TransportAdapter

    if strategy == TransportStrategy.LCM:
        from .json_codec import dumps_message, loads_message

        return TransportAdapter(
            transport,
            serializer=dumps_message,
            deserializer=loads_message,
        )
    return TransportAdapter(transport)


def create_publisher(
    topic_name: str,
    msg_type: Any = None,
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
    ros_node=None,
    **kwargs,
) -> Publisher:
    """Shortcut to create a publisher."""

    strategy = _coerce_strategy(strategy)
    topic = TopicConfig(
        name=topic_name,
        msg_type=msg_type,
        strategy=strategy,
        **kwargs,
    )
    transport = create_transport(strategy, ros_node)
    return transport.create_publisher(topic)


def create_subscriber(
    topic_name: str,
    msg_type: Any = None,
    callback: Callable | None = None,
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
    ros_node=None,
    **kwargs,
) -> Subscriber:
    """Shortcut to create a subscriber."""

    strategy = _coerce_strategy(strategy)
    topic = TopicConfig(
        name=topic_name,
        msg_type=msg_type,
        strategy=strategy,
        **kwargs,
    )
    transport = create_transport(strategy, ros_node)
    return transport.create_subscriber(topic, callback or (lambda _msg: None))


def _coerce_strategy(strategy: TransportStrategy | str) -> TransportStrategy:
    if isinstance(strategy, TransportStrategy):
        return strategy
    try:
        return TransportStrategy(strategy)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Unknown strategy: {strategy}") from exc
