"""RemoteOut / RemoteIn — serializable port proxies for cross-process wiring.

When a Module is deployed to a Worker, its real Out/In ports exist in the worker
process. RemoteOut/RemoteIn act as references that can be used by the main process
for transport-based wiring (DDS/SHM).

Usage:
    # In main process
    remote_out = RemoteOut("odometry", Odometry, transport_topic="/slam/odometry")
    remote_in = RemoteIn("cmd_vel", Twist, transport_topic="/nav/cmd_vel")

    # Bind a transport so data flows across process boundaries
    remote_out.bind_transport(dds_transport, topic="/slam/odometry")
    remote_in.bind_transport(dds_transport, topic="/nav/cmd_vel")
"""
import logging
from collections.abc import Callable
from typing import Any

logger = logging.getLogger(__name__)


class RemoteOut:
    """Proxy for an Out port that lives in a remote Worker process.

    Holds the transport binding so the main process can publish or
    forward messages without holding a reference to the real port.
    """

    def __init__(
        self,
        name: str,
        msg_type: type | None = None,
        transport_topic: str = "",
    ) -> None:
        self.name = name
        self.msg_type = msg_type
        self.transport_topic = transport_topic or f"/{name}"
        self._transport: Any | None = None

    def bind_transport(self, transport: Any, topic: str = "") -> None:
        """Bind to a transport for cross-process delivery.

        Args:
            transport: Transport instance (DDS, SHM, Local, …).
            topic: Override the default transport topic.
        """
        self._transport = transport
        if topic:
            self.transport_topic = topic

    def publish(self, msg: Any) -> None:
        """Publish via transport (no local callbacks in remote proxy)."""
        if self._transport:
            try:
                self._transport.publish(self.transport_topic, msg)
            except Exception:
                logger.exception(
                    "RemoteOut[%s] transport publish error", self.name
                )

    def __repr__(self) -> str:
        return f"RemoteOut({self.name}, topic={self.transport_topic})"


class RemoteIn:
    """Proxy for an In port that lives in a remote Worker process.

    Holds the transport subscription so the main process can receive
    messages and forward them into the worker without holding a reference
    to the real port.
    """

    def __init__(
        self,
        name: str,
        msg_type: type | None = None,
        transport_topic: str = "",
    ) -> None:
        self.name = name
        self.msg_type = msg_type
        self.transport_topic = transport_topic or f"/{name}"
        self._transport: Any | None = None
        self._callback: Callable | None = None

    def bind_transport(self, transport: Any, topic: str = "") -> None:
        """Bind to a transport for cross-process delivery.

        If a callback was already registered via subscribe(), it is
        immediately wired to the transport subscription.

        Args:
            transport: Transport instance (DDS, SHM, Local, …).
            topic: Override the default transport topic.
        """
        self._transport = transport
        if topic:
            self.transport_topic = topic
        if self._callback:
            try:
                transport.subscribe(self.transport_topic, self._callback)
            except Exception:
                logger.exception(
                    "RemoteIn[%s] transport subscribe error", self.name
                )

    def subscribe(self, callback: Callable) -> None:
        """Register callback for incoming messages.

        If a transport is already bound, the callback is wired immediately.
        """
        self._callback = callback
        if self._transport:
            try:
                self._transport.subscribe(self.transport_topic, callback)
            except Exception:
                logger.exception(
                    "RemoteIn[%s] transport subscribe error", self.name
                )

    def __repr__(self) -> str:
        status = "connected" if self._callback else "idle"
        return f"RemoteIn({self.name}, topic={self.transport_topic}, {status})"
