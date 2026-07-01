"""LCM input adapter for Thunder navigation."""

from __future__ import annotations

import json
import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import Out
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
    "navigation",
    "lcm_nav_input",
    description="LCM navigation input adapter for Thunder",
)
class LCMNavInModule(Module, layer=5):
    """Subscribe to endpoint command channels and publish LingTu navigation inputs."""

    goal_pose: Out[PoseStamped]
    cancel: Out[str]
    instruction: Out[str]

    def __init__(
        self,
        endpoint_contract: str = THUNDER_FIELD_LCM_CONTRACT_NAME,
        default_frame_id: str | None = None,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._contract = self._resolve_contract(endpoint_contract)
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._subscriptions: list[Any] = []
        self._last_message_ts = 0.0
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.goal_pose)
        )
        self._backend_status = BackendStatus.configured_as(
            "lcm_nav_input"
        )

    def setup(self) -> None:
        self._transport = self._transport or self._create_default_transport()
        for binding in self._contract.bindings:
            if binding.direction != "endpoint_to_lingtu":
                continue
            if not self._has_output_for_topic(binding.topic):
                continue
            self._subscriptions.append(self._subscribe_binding(binding))
        logger.info(
            "LCMNavInModule: subscribed to %s command channels for %s",
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
                    logger.debug("LCM command subscription close failed", exc_info=True)
        self._subscriptions.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("LCM command transport close failed", exc_info=True)
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

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()

        from runtime.transport.factory import create_transport

        return create_transport("lcm")

    def _subscribe_binding(self, binding: LCMEndpointBinding) -> Any:
        transport = self._transport
        if transport is None:
            raise RuntimeError("LCM navigation input adapter transport is not initialized")

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
            "LCM navigation input adapter transport must expose subscribe() "
            "or create_subscriber()"
        )

    def _on_binding_payload(self, binding: LCMEndpointBinding, payload: Any) -> None:
        try:
            msg = loads_endpoint_message(binding, payload)
            self._publish_topic(binding.topic, msg)
        except Exception:
            self._decode_errors[binding.topic] += 1
            logger.exception("Failed to decode LCM command payload on %s", binding.channel)
            return

        self._last_message_ts = time.time()
        self._message_counts[binding.topic] += 1

    def _publish_topic(self, topic: str, msg: Any) -> None:
        if topic == TOPICS.goal_pose:
            self.goal_pose.publish(self._coerce_pose_stamped(msg))
            return
        if topic == TOPICS.cancel:
            self.cancel.publish(self._coerce_text_command(msg))
            return
        if topic == TOPICS.semantic_instruction:
            text = self._coerce_text_command(msg)
            if text:
                self.instruction.publish(text)
            return
        logger.debug(
            "LCM navigation input adapter ignored %s payload %s",
            topic,
            type(msg).__name__,
        )

    def _coerce_pose_stamped(self, value: Any) -> PoseStamped:
        if isinstance(value, PoseStamped):
            return PoseStamped(
                pose=value.pose,
                ts=float(value.ts or 0.0),
                frame_id=str(value.frame_id or self._default_frame_id),
            )
        if isinstance(value, Pose):
            return PoseStamped(pose=value, frame_id=self._default_frame_id)
        if isinstance(value, Mapping):
            return self._pose_from_mapping(value)
        if hasattr(value, "pose"):
            return PoseStamped(
                pose=self._coerce_pose(getattr(value, "pose")),
                ts=self._coerce_float(getattr(value, "ts", 0.0)),
                frame_id=str(getattr(value, "frame_id", "") or self._default_frame_id),
            )
        if hasattr(value, "x") and hasattr(value, "y"):
            return PoseStamped(
                pose=self._pose_from_xyz_orientation(
                    getattr(value, "x", 0.0),
                    getattr(value, "y", 0.0),
                    getattr(value, "z", 0.0),
                    getattr(value, "orientation", None),
                ),
                frame_id=self._default_frame_id,
            )
        try:
            seq = list(value)
        except TypeError:
            seq = []
        if len(seq) >= 2:
            return PoseStamped(
                pose=self._pose_from_xyz_orientation(
                    seq[0],
                    seq[1],
                    seq[2] if len(seq) >= 3 else 0.0,
                    seq[3:7] if len(seq) >= 7 else None,
                ),
                frame_id=self._default_frame_id,
            )
        raise TypeError(f"cannot coerce {type(value).__name__} to PoseStamped")

    def _pose_from_mapping(self, value: Mapping[str, Any]) -> PoseStamped:
        data = dict(value)
        if "pose" in data:
            data.setdefault("frame_id", self._default_frame_id)
            try:
                return PoseStamped.from_dict(data)
            except (TypeError, ValueError, KeyError):
                return PoseStamped(
                    pose=self._coerce_pose(data.get("pose")),
                    ts=self._coerce_float(data.get("ts", 0.0)),
                    frame_id=str(data.get("frame_id") or self._default_frame_id),
                )
        return PoseStamped(
            pose=self._pose_from_xyz_orientation(
                data.get("x", 0.0),
                data.get("y", 0.0),
                data.get("z", 0.0),
                data.get("orientation"),
            ),
            ts=self._coerce_float(data.get("ts", 0.0)),
            frame_id=str(data.get("frame_id") or self._default_frame_id),
        )

    def _coerce_pose(self, value: Any) -> Pose:
        if isinstance(value, Pose):
            return value
        if isinstance(value, Mapping):
            try:
                return Pose.from_dict(dict(value))
            except (TypeError, ValueError, KeyError):
                return self._pose_from_xyz_orientation(
                    value.get("x", 0.0),
                    value.get("y", 0.0),
                    value.get("z", 0.0),
                    value.get("orientation"),
                )
        if hasattr(value, "position"):
            return Pose(
                self._coerce_position(getattr(value, "position")),
                self._coerce_orientation(getattr(value, "orientation", None)),
            )
        if hasattr(value, "x") and hasattr(value, "y"):
            return self._pose_from_xyz_orientation(
                getattr(value, "x", 0.0),
                getattr(value, "y", 0.0),
                getattr(value, "z", 0.0),
                getattr(value, "orientation", None),
            )
        raise TypeError(f"cannot coerce {type(value).__name__} to Pose")

    def _pose_from_xyz_orientation(
        self,
        x: Any,
        y: Any,
        z: Any,
        orientation: Any = None,
    ) -> Pose:
        return Pose(
            Vector3(self._coerce_float(x), self._coerce_float(y), self._coerce_float(z)),
            self._coerce_orientation(orientation),
        )

    def _coerce_position(self, value: Any) -> Vector3:
        if isinstance(value, Vector3):
            return value
        if isinstance(value, Mapping):
            return Vector3(
                self._coerce_float(value.get("x", 0.0)),
                self._coerce_float(value.get("y", 0.0)),
                self._coerce_float(value.get("z", 0.0)),
            )
        if hasattr(value, "x") and hasattr(value, "y"):
            return Vector3(
                self._coerce_float(getattr(value, "x", 0.0)),
                self._coerce_float(getattr(value, "y", 0.0)),
                self._coerce_float(getattr(value, "z", 0.0)),
            )
        return Vector3(value)

    @staticmethod
    def _coerce_orientation(value: Any) -> Quaternion:
        if value is None:
            return Quaternion()
        if isinstance(value, Quaternion):
            return value
        if isinstance(value, Mapping):
            return Quaternion(
                value.get("x", 0.0),
                value.get("y", 0.0),
                value.get("z", 0.0),
                value.get("w", 1.0),
            )
        if all(hasattr(value, name) for name in ("x", "y", "z", "w")):
            return Quaternion(value.x, value.y, value.z, value.w)
        return Quaternion(value)

    @staticmethod
    def _coerce_text_command(value: Any) -> str:
        if value is None:
            return ""
        if isinstance(value, str):
            return value
        if isinstance(value, Mapping):
            for key in ("text", "instruction", "reason", "action", "command", "id"):
                item = value.get(key)
                if item is not None:
                    return str(item)
            return json.dumps(dict(value), ensure_ascii=True, sort_keys=True)
        return str(value)

    @staticmethod
    def _coerce_float(value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _has_output_for_topic(topic: str) -> bool:
        return topic in {
            TOPICS.goal_pose,
            TOPICS.cancel,
            TOPICS.semantic_instruction,
        }
