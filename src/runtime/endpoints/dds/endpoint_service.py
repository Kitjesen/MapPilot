"""Standalone typed DDS endpoint service for Thunder field adapters."""

from __future__ import annotations

import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from message.dds_codec import (
    from_dds_message,
    to_dds_message,
)
from runtime.endpoints.dds.contracts import (
    THUNDER_FIELD_DDS_CONTRACT_NAME,
    DDSEndpointBinding,
    DDSEndpointContract,
    endpoint_contract,
)
from runtime.runtime_interface import TOPICS
from runtime.transport.abc import TopicConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class DDSEndpointEvent:
    topic: str
    channel: str
    schema: str
    message: Any
    ts: float


class DDSEndpointService:
    """Publish and consume the Thunder endpoint contract over typed DDS."""

    def __init__(
        self,
        endpoint_contract_name: str = THUNDER_FIELD_DDS_CONTRACT_NAME,
        *,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        on_lingtu_message: Callable[[DDSEndpointEvent], None] | None = None,
        subscribe_lingtu_outputs: bool = True,
        qos_depth: int = 10,
        reliable: bool = True,
        transport_strategy: str | None = None,
    ) -> None:
        self._contract = endpoint_contract(str(endpoint_contract_name or THUNDER_FIELD_DDS_CONTRACT_NAME))
        self._transport = transport
        self._transport_factory = transport_factory
        self._transport_strategy = str(transport_strategy or "dds")
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
    def contract(self) -> DDSEndpointContract:
        return self._contract

    def start(self) -> None:
        if self._started:
            return
        self._transport = self._transport or self._create_default_transport()
        if self._subscribe_lingtu_outputs:
            for binding in self._contract.bindings:
                if binding.direction == "lingtu_to_endpoint":
                    self._subscriptions.append(self._subscribe_binding(binding))
        self._started = True

    def stop(self) -> None:
        for sub in self._subscriptions:
            close = getattr(sub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS endpoint subscription close failed", exc_info=True)
        self._subscriptions.clear()
        for pub in self._publishers.values():
            close = getattr(pub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS endpoint publisher close failed", exc_info=True)
        self._publishers.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS endpoint transport close failed", exc_info=True)
        self._transport = None
        self._started = False

    def publish_to_lingtu(self, topic: str, msg: Any) -> None:
        binding = self._binding(topic)
        if binding.direction != "endpoint_to_lingtu":
            raise ValueError(f"{topic} is not an endpoint-to-LingTu DDS binding")
        payload = (
            msg
            if self._transport_strategy == "local"
            else to_dds_message(topic, msg)
        )
        self._publisher(topic).publish(payload)
        self._publish_counts[topic] += 1
        self._last_publish_ts = time.time()

    def publish_sensor_snapshot(
        self,
        *,
        lidar_scan: Any | None = None,
        imu: Any | None = None,
    ) -> int:
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
        return {
            "service": "dds_endpoint_service",
            "endpoint_contract": self._contract.name,
            "runtime_contract": self._contract.runtime_contract,
            "transport": self._transport_strategy,
            "typed_messages": {
                binding.topic: binding.idl_type
                for binding in self._contract.bindings
            },
            "started": self._started,
            "endpoint_to_lingtu_channels": {
                binding.topic: binding.topic
                for binding in self._contract.bindings
                if binding.direction == "endpoint_to_lingtu"
            },
            "lingtu_to_endpoint_channels": {
                binding.topic: binding.topic
                for binding in self._contract.bindings
                if binding.direction == "lingtu_to_endpoint"
            },
            "publish_counts": dict(self._publish_counts),
            "receive_counts": dict(self._receive_counts),
            "decode_errors": dict(self._decode_errors),
            "last_publish_ts": self._last_publish_ts,
            "last_receive_ts": self._last_receive_ts,
        }

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()
        from runtime.transport.factory import create_transport

        return create_transport("dds")

    def _ensure_transport(self) -> Any:
        self._transport = self._transport or self._create_default_transport()
        return self._transport

    def _binding(self, topic: str) -> DDSEndpointBinding:
        return self._contract.binding_for_topic(topic)

    def _publisher(self, topic: str) -> Any:
        publisher = self._publishers.get(topic)
        if publisher is not None:
            return publisher
        transport = self._ensure_transport()
        publisher = transport.create_publisher(
            TopicConfig(name=topic, qos_depth=self._qos_depth, reliable=self._reliable)
        )
        self._publishers[topic] = publisher
        return publisher

    def _subscribe_binding(self, binding: DDSEndpointBinding) -> Any:
        transport = self._ensure_transport()
        return transport.create_subscriber(
            TopicConfig(name=binding.topic, qos_depth=self._qos_depth, reliable=self._reliable),
            lambda msg: self._on_binding_message(binding, msg),
        )

    def _on_binding_message(self, binding: DDSEndpointBinding, msg: Any) -> None:
        try:
            decoded = (
                msg
                if self._transport_strategy == "local"
                else from_dds_message(binding.topic, msg)
            )
        except Exception:
            self._decode_errors[binding.topic] += 1
            logger.exception("Failed to decode DDS endpoint payload on %s", binding.topic)
            return
        self._receive_counts[binding.topic] += 1
        self._last_receive_ts = time.time()
        if self._on_lingtu_message is not None:
            self._on_lingtu_message(
                DDSEndpointEvent(
                    topic=binding.topic,
                    channel=binding.topic,
                    schema=binding.schema,
                    message=decoded,
                    ts=self._last_receive_ts,
                )
            )
