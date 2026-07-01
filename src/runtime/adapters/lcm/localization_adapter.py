"""LCM localization endpoint adapter for Thunder field runtime."""

from __future__ import annotations

import logging
import time
from collections import Counter
from collections.abc import Callable
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.tf import FrameTree
from runtime.module import Module
from runtime.msgs.geometry import Quaternion, Transform, Vector3
from runtime.msgs.gnss import GnssOdom
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import Imu, PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out
from runtime.transport.abc import TopicConfig

from .contracts import (
    THUNDER_FIELD_LCM_CONTRACT_NAME,
    LCMEndpointBinding,
    LCMEndpointContract,
    endpoint_contract,
)
from .endpoint_codec import loads_endpoint_message

logger = logging.getLogger(__name__)


@register(
    "localization_adapter",
    "lcm_endpoint",
    description="LCM endpoint adapter for Thunder localization and map streams",
)
class LCMLocalizationAdapterModule(Module, layer=1):
    """Bridge LCM endpoint channels into LingTu localization Module ports."""

    lidar_scan: Out[PointCloud2]
    imu: Out[Imu]
    registered_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]
    saved_map: Out[PointCloud2]
    odometry: Out[Odometry]
    localization_quality: Out[float]
    alive: Out[bool]
    localization_status: Out[dict]
    gnss_fusion_health: Out[dict]
    map_odom_tf: Out[dict]
    map_frame_jump_event: Out[dict]
    scene_mode: Out[str]

    visual_odom: In[Odometry]
    gnss_odom: In[GnssOdom]

    def __init__(
        self,
        endpoint_contract: str = THUNDER_FIELD_LCM_CONTRACT_NAME,
        backend_profile: str = "bridge",
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._contract = self._resolve_contract(endpoint_contract)
        self._backend_profile = str(backend_profile or "bridge")
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._subscriptions: list[Any] = []
        self._last_message_ts = 0.0
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_visual_odom: Odometry | None = None
        self._last_gnss_odom: GnssOdom | None = None
        self._frame_tree = kw.get("frame_tree") or FrameTree.from_robot_config()
        self._backend_status = BackendStatus.configured_as("lcm_endpoint")

    def setup(self) -> None:
        self.visual_odom.subscribe(self._on_visual_odom)
        self.gnss_odom.subscribe(self._on_gnss_odom)
        self._transport = self._transport or self._create_default_transport()
        for binding in self._contract.bindings:
            if binding.direction != "endpoint_to_lingtu":
                continue
            if not self._has_output_for_topic(binding.topic):
                logger.debug(
                    "LCM localization adapter has no output port for %s",
                    binding.topic,
                )
                continue
            self._subscriptions.append(self._subscribe_binding(binding))
        logger.info(
            "LCMLocalizationAdapterModule: subscribed to %s channels for %s",
            len(self._subscriptions),
            self._contract.name,
        )

    def stop(self) -> None:
        for sub in self._subscriptions:
            close = getattr(sub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("LCM subscription close failed", exc_info=True)
        self._subscriptions.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("LCM transport close failed", exc_info=True)
        self._transport = None
        super().stop()

    def health(self) -> dict[str, Any]:
        return {
            **self._backend_status.as_health_fields(),
            "endpoint_contract": self._contract.name,
            "transport": self._contract.transport,
            "subscribed_channels": [
                binding.channel
                for binding in self._contract.bindings
                if binding.direction == "endpoint_to_lingtu"
                and self._has_output_for_topic(binding.topic)
            ],
            "message_counts": dict(self._message_counts),
            "decode_errors": dict(self._decode_errors),
            "last_message_ts": self._last_message_ts,
        }

    @staticmethod
    def _resolve_contract(contract_name: str) -> LCMEndpointContract:
        name = str(contract_name or THUNDER_FIELD_LCM_CONTRACT_NAME)
        return endpoint_contract(name)

    @staticmethod
    def _create_default_transport() -> Any:
        from runtime.transport.factory import create_transport

        return create_transport("lcm")

    def _subscribe_binding(self, binding: LCMEndpointBinding) -> Any:
        transport = self._transport
        if transport is None:
            raise RuntimeError("LCM localization adapter transport is not initialized")

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
            "LCM localization adapter transport must expose subscribe() "
            "or create_subscriber()"
        )

    def _on_binding_payload(self, binding: LCMEndpointBinding, payload: Any) -> None:
        try:
            msg = loads_endpoint_message(binding, payload)
        except Exception:
            self._decode_errors[binding.topic] += 1
            logger.exception("Failed to decode LCM endpoint payload on %s", binding.channel)
            return

        self._last_message_ts = time.time()
        self._message_counts[binding.topic] += 1
        self._publish_topic(binding.topic, msg)

    def _publish_topic(self, topic: str, msg: Any) -> None:
        if topic == TOPICS.lidar_scan and isinstance(msg, PointCloud2):
            self.lidar_scan.publish(msg)
            return
        if topic == TOPICS.imu and isinstance(msg, Imu):
            self.imu.publish(msg)
            return
        if topic == TOPICS.registered_cloud and isinstance(msg, PointCloud2):
            self.registered_cloud.publish(msg)
            return
        if topic == TOPICS.map_cloud and isinstance(msg, PointCloud2):
            self.map_cloud.publish(msg)
            self._publish_tracking_status(topic, msg.ts)
            return
        if topic == TOPICS.odometry and isinstance(msg, Odometry):
            self._frame_tree.update_odometry(msg)
            self.odometry.publish(msg)
            self._publish_tracking_status(topic, msg.ts)
            return
        if topic == TOPICS.localization_quality:
            self.localization_quality.publish(float(msg))
            return
        if topic == TOPICS.localization_health:
            self._publish_localization_health(msg)
            return
        logger.debug("LCM localization adapter ignored %s payload %s", topic, type(msg).__name__)

    def _publish_tracking_status(self, source_topic: str, ts: float) -> None:
        status = {
            "state": "TRACKING",
            "confidence": 1.0,
            "backend": "lcm_endpoint",
            "backend_profile": self._backend_profile,
            "health_source": self._contract.name,
            "endpoint_contract": self._contract.name,
            "source_topic": source_topic,
            "ts": ts or time.time(),
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": False,
            "map_save_supported": False,
            "map_save_source": "lcm_endpoint",
        }
        self.alive.publish(True)
        self.localization_status.publish(status)

    def _publish_localization_health(self, msg: Any) -> None:
        status = dict(msg) if isinstance(msg, dict) else {"state": str(msg)}
        state = str(status.get("state") or "UNKNOWN").upper()
        status.setdefault("state", state)
        status.setdefault("confidence", 1.0 if state in {"TRACKING", "LOCKED", "OK"} else 0.0)
        status.setdefault("backend", "lcm_endpoint")
        status.setdefault("backend_profile", self._backend_profile)
        status.setdefault("health_source", self._contract.name)
        status.setdefault("endpoint_contract", self._contract.name)
        status.setdefault("ts", time.time())
        status.setdefault("relocalization_supported", False)
        status.setdefault("saved_map_relocalization_supported", False)
        status.setdefault("restart_recovery_supported", False)
        status.setdefault("map_save_supported", False)
        status.setdefault("map_save_source", "lcm_endpoint")

        self.alive.publish(state not in {"LOST", "DIVERGED", "UNINIT"})
        self.localization_status.publish(status)

        quality = status.get("quality", status.get("confidence"))
        if quality is not None:
            try:
                self.localization_quality.publish(float(quality))
            except (TypeError, ValueError):
                pass
        if isinstance(status.get("gnss_fusion_health"), dict):
            self.gnss_fusion_health.publish(dict(status["gnss_fusion_health"]))
        if isinstance(status.get("map_odom_tf"), dict):
            tf = dict(status["map_odom_tf"])
            self._update_map_odom_tf(tf)
            self.map_odom_tf.publish(tf)
        scene_mode = status.get("scene_mode")
        if scene_mode:
            self.scene_mode.publish(str(scene_mode))

    def _update_map_odom_tf(self, tf: dict[str, Any]) -> None:
        if not tf or not tf.get("valid", False):
            return
        self._frame_tree.set_transform(
            Transform(
                translation=Vector3(float(tf["tx"]), float(tf["ty"]), float(tf["tz"])),
                rotation=Quaternion(
                    float(tf["qx"]),
                    float(tf["qy"]),
                    float(tf["qz"]),
                    float(tf["qw"]),
                ),
                frame_id=topic_default_frame_id(TOPICS.map_cloud),
                child_frame_id=topic_default_frame_id(TOPICS.odometry),
                ts=float(tf.get("ts") or time.time()),
            )
        )

    def _on_visual_odom(self, msg: Odometry) -> None:
        self._last_visual_odom = msg

    def _on_gnss_odom(self, msg: GnssOdom) -> None:
        self._last_gnss_odom = msg

    @staticmethod
    def _has_output_for_topic(topic: str) -> bool:
        return topic in {
            TOPICS.lidar_scan,
            TOPICS.imu,
            TOPICS.registered_cloud,
            TOPICS.map_cloud,
            TOPICS.odometry,
            TOPICS.localization_quality,
            TOPICS.localization_health,
        }
