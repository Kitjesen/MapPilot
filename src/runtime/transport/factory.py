"""Transport factory for LingTu module communication.

Default creation is product-neutral and ROS-free: LocalTransport for in-process
graphs, SHM for same-host IPC, and DDS only for compatibility windows.
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
    *,
    domain_id: int | None = None,
) -> Any:
    """Create a transport instance by strategy.

    ``ros_node`` is kept for old call sites. DDS transport currently uses the
    native CycloneDDS backend and does not require rclpy.

    ``domain_id`` overrides the DDS domain (only relevant for DDS strategy).
    When ``None`` the process-wide default is used.
    """

    strategy = _coerce_strategy(strategy)

    if strategy == TransportStrategy.LOCAL:
        return _DEFAULT_LOCAL_TRANSPORT

    if strategy == TransportStrategy.SHM:
        from .shm import SHMTransport

        return SHMTransport()

    if strategy == TransportStrategy.DDS:
        from .dds import DDSTransport

        return DDSTransport(domain_id=domain_id)

    if strategy == TransportStrategy.AUTO:
        from .shm import SHMTransport

        try:
            return SHMTransport()
        except Exception:
            logger.debug("AUTO transport could not create SHM", exc_info=True)
            if ros_node is None:
                return _DEFAULT_LOCAL_TRANSPORT
            from .dds import DDSTransport

            return DDSTransport(domain_id=domain_id)

    raise ValueError(f"Unknown strategy: {strategy}")


def create_transport_adapter(
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
    ros_node=None,
    *,
    domain_id: int | None = None,
) -> Any:
    """Create a simple pub/sub adapter for a TransportABC backend."""

    strategy = _coerce_strategy(strategy)
    transport = create_transport(strategy, ros_node, domain_id=domain_id)
    if hasattr(transport, "publish") and hasattr(transport, "subscribe"):
        return transport

    from .adapter import TransportAdapter

    if strategy == TransportStrategy.SHM:
        from .json_codec import dumps_topic_message, loads_message

        return TransportAdapter(
            transport,
            topic_serializer=dumps_topic_message,
            deserializer=loads_message,
        )
    if strategy == TransportStrategy.DDS:
        from message.dds import (
            TOPIC_SPECS,
            dds_type_for_topic,
            from_dds_message,
            to_dds_message,
        )

        from .dds import RawMessage
        from .json_codec import dumps_topic_message, loads_message

        return TransportAdapter(
            transport,
            topic_serializer=dumps_topic_message,
            deserializer=loads_message,
            backend_msg_type=RawMessage,
            topic_msg_type=dds_type_for_topic,
            topic_encoder=to_dds_message,
            topic_decoder=from_dds_message,
            forbidden_topics=frozenset(TOPIC_SPECS),
        )
    return TransportAdapter(transport)


def create_route_transport_adapter(
    strategy: TransportStrategy | str = TransportStrategy.LOCAL,
    ros_node=None,
    *,
    domain_id: int | None = None,
) -> Any:
    """Create a transport adapter for a validated runtime route.

    Unlike per-wire ``wire(delivery="dds")``, route-selected DDS is
    allowed to carry registered product topics because the route contract has
    already declared the topic/backend pair.
    """

    strategy = _coerce_strategy(strategy)
    if strategy == TransportStrategy.LOCAL:
        return _DEFAULT_LOCAL_TRANSPORT
    if strategy == TransportStrategy.SHM:
        return create_transport_adapter(strategy, ros_node, domain_id=domain_id)
    if strategy == TransportStrategy.DDS:
        from message.dds import (
            dds_type_for_topic,
            from_dds_message,
            to_dds_message,
        )

        from .dds import RawMessage
        from .json_codec import dumps_topic_message, loads_message

        return TransportAdapter(
            create_transport(strategy, ros_node, domain_id=domain_id),
            topic_serializer=dumps_topic_message,
            deserializer=loads_message,
            backend_msg_type=RawMessage,
            topic_msg_type=dds_type_for_topic,
            topic_encoder=to_dds_message,
            topic_decoder=from_dds_message,
        )
    raise ValueError(f"Runtime route backend {strategy.value!r} is not supported")


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
