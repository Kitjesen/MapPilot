"""Standalone LCM endpoint service for Thunder field adapters.

This is the process-side counterpart to the in-graph LCM adapter modules. It is
meant for external Thunder endpoint processes that publish normalized
sensor/localization inputs to LingTu and consume LingTu navigation outputs.
"""

from __future__ import annotations

import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from runtime.runtime_interface import TOPICS
from runtime.transport.abc import TopicConfig

from .contracts import (
    THUNDER_FIELD_LCM_CONTRACT_NAME,
    LCMEndpointBinding,
    LCMEndpointContract,
    endpoint_contract,
)
from .endpoint_codec import dumps_endpoint_message, loads_endpoint_message

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LCMEndpointEvent:
    """Decoded message received from LingTu over an endpoint LCM channel."""

    topic: str
    channel: str
    schema: str
    message: Any
    ts: float


class LCMEndpointService:
    """Publish and consume the Thunder LCM endpoint contract outside LingTu.

    External field services should use this class instead of importing the
    Python ``lcm`` package directly. It keeps transport creation in
    ``runtime.transport`` and keeps product topic/schema binding in
    ``runtime.adapters.lcm.contracts``.
    """

    def __init__(
        self,
        endpoint_contract_name: str = THUNDER_FIELD_LCM_CONTRACT_NAME,
        *,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        on_lingtu_message: Callable[[LCMEndpointEvent], None] | None = None,
        subscribe_lingtu_outputs: bool = True,
        qos_depth: int = 10,
        reliable: bool = False,
        transport_strategy: str | None = None,
    ) -> None:
        self._contract = self._resolve_contract(endpoint_contract_name)
        self._transport = transport
        self._transport_factory = transport_factory
        self._transport_strategy = str(
            transport_strategy or self._contract.transport or "lcm"
        )
        self._owns_transport = transport is None
        self._on_lingtu_message = on_lingtu_message
        self._subscribe_lingtu_outputs = bool(subscribe_lingtu_outputs)
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._publishers: dict[str, Any] = {}
        self._subscriptions: list[Any] = []
        self._publish_counts: Counter[str] = Counter()
        self._receive_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_publish_ts = 0.0
        self._last_receive_ts = 0.0
        self._started = False

    @property
    def contract(self) -> LCMEndpointContract:
        """Return the endpoint contract served by this process."""

        return self._contract

    def start(self) -> None:
        """Start endpoint subscriptions for LingTu-to-endpoint outputs."""

        if self._started:
            return
        self._transport = self._transport or self._create_default_transport()
        if self._subscribe_lingtu_outputs:
            for binding in self._contract.bindings:
                if binding.direction == "lingtu_to_endpoint":
                    self._subscriptions.append(self._subscribe_binding(binding))
        self._started = True

    def stop(self) -> None:
        """Stop endpoint subscriptions and close owned transport resources."""

        for sub in self._subscriptions:
            close = getattr(sub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("LCM endpoint subscription close failed", exc_info=True)
        self._subscriptions.clear()
        self._publishers.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("LCM endpoint transport close failed", exc_info=True)
        self._transport = None
        self._started = False

    def publish_to_lingtu(self, topic: str, msg: Any) -> None:
        """Publish one endpoint-to-LingTu message using the contract schema."""

        binding = self._binding(topic)
        if binding.direction != "endpoint_to_lingtu":
            raise ValueError(f"{topic} is not an endpoint-to-LingTu LCM binding")
        self._publish_binding(binding, msg)

    def publish_sensor_snapshot(
        self,
        *,
        lidar_scan: Any | None = None,
        imu: Any | None = None,
    ) -> int:
        """Publish available normalized sensor inputs for the field endpoint."""

        published = 0
        if lidar_scan is not None:
            self.publish_to_lingtu(TOPICS.lidar_scan, lidar_scan)
            published += 1
        if imu is not None:
            self.publish_to_lingtu(TOPICS.imu, imu)
            published += 1
        return published

    def publish_localization_snapshot(
        self,
        *,
        odometry: Any | None = None,
        registered_cloud: Any | None = None,
        map_cloud: Any | None = None,
        localization_health: Mapping[str, Any] | None = None,
        localization_quality: float | None = None,
    ) -> int:
        """Publish available localization/map outputs for the field endpoint."""

        published = 0
        if odometry is not None:
            self.publish_to_lingtu(TOPICS.odometry, odometry)
            published += 1
        if registered_cloud is not None:
            self.publish_to_lingtu(TOPICS.registered_cloud, registered_cloud)
            published += 1
        if map_cloud is not None:
            self.publish_to_lingtu(TOPICS.map_cloud, map_cloud)
            published += 1
        if localization_health is not None:
            self.publish_to_lingtu(TOPICS.localization_health, dict(localization_health))
            published += 1
        if localization_quality is not None:
            self.publish_to_lingtu(TOPICS.localization_quality, float(localization_quality))
            published += 1
        return published

    def health(self) -> dict[str, Any]:
        """Return service status and endpoint contract channel coverage."""

        return {
            "service": "lcm_endpoint_service",
            "endpoint_contract": self._contract.name,
            "runtime_contract": self._contract.runtime_contract,
            "transport": self._transport_strategy,
            "contract_transport": self._contract.transport,
            "started": self._started,
            "endpoint_to_lingtu_channels": {
                binding.topic: binding.channel
                for binding in self._contract.bindings
                if binding.direction == "endpoint_to_lingtu"
            },
            "lingtu_to_endpoint_channels": {
                binding.topic: binding.channel
                for binding in self._contract.bindings
                if binding.direction == "lingtu_to_endpoint"
            },
            "publish_counts": dict(self._publish_counts),
            "receive_counts": dict(self._receive_counts),
            "decode_errors": dict(self._decode_errors),
            "last_publish_ts": self._last_publish_ts,
            "last_receive_ts": self._last_receive_ts,
        }

    @staticmethod
    def _resolve_contract(contract_name: str) -> LCMEndpointContract:
        return endpoint_contract(str(contract_name or THUNDER_FIELD_LCM_CONTRACT_NAME))

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()

        from runtime.transport.factory import create_transport

        return create_transport("lcm")

    def _ensure_transport(self) -> Any:
        self._transport = self._transport or self._create_default_transport()
        return self._transport

    def _binding(self, topic: str) -> LCMEndpointBinding:
        return self._contract.binding_for_topic(topic)

    def _publish_binding(self, binding: LCMEndpointBinding, msg: Any) -> None:
        payload = dumps_endpoint_message(binding, msg)
        self._publish_payload(binding, payload)
        self._publish_counts[binding.topic] += 1
        self._last_publish_ts = time.time()

    def _publish_payload(self, binding: LCMEndpointBinding, payload: bytes) -> None:
        transport = self._ensure_transport()
        publish = getattr(transport, "publish", None)
        if callable(publish):
            publish(binding.channel, payload)
            return

        publisher = self._publishers.get(binding.topic)
        if publisher is None:
            create_publisher = getattr(transport, "create_publisher", None)
            if not callable(create_publisher):
                raise TypeError(
                    "LCM endpoint service transport must expose publish() "
                    "or create_publisher()"
                )
            publisher = create_publisher(
                TopicConfig(
                    name=binding.channel,
                    qos_depth=self._qos_depth,
                    reliable=self._reliable,
                )
            )
            self._publishers[binding.topic] = publisher
        publisher.publish(payload)

    def _subscribe_binding(self, binding: LCMEndpointBinding) -> Any:
        transport = self._ensure_transport()

        def callback(*args: Any) -> None:
            payload = args[0] if args else None
            self._on_binding_payload(binding, payload)

        subscribe = getattr(transport, "subscribe", None)
        if callable(subscribe):
            return subscribe(binding.channel, callback)

        create_subscriber = getattr(transport, "create_subscriber", None)
        if callable(create_subscriber):
            sub = create_subscriber(TopicConfig(name=binding.channel), callback)
            start = getattr(sub, "start", None)
            if callable(start):
                start()
            return sub

        raise TypeError(
            "LCM endpoint service transport must expose subscribe() "
            "or create_subscriber()"
        )

    def _on_binding_payload(self, binding: LCMEndpointBinding, payload: Any) -> None:
        try:
            msg = loads_endpoint_message(binding, payload)
        except Exception:
            self._decode_errors[binding.topic] += 1
            logger.exception("Failed to decode LingTu endpoint payload on %s", binding.channel)
            return

        self._receive_counts[binding.topic] += 1
        self._last_receive_ts = time.time()
        if self._on_lingtu_message is not None:
            self._on_lingtu_message(
                LCMEndpointEvent(
                    topic=binding.topic,
                    channel=binding.channel,
                    schema=binding.schema,
                    message=msg,
                    ts=self._last_receive_ts,
                )
            )
