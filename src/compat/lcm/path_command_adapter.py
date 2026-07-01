"""LCM egress adapter for Thunder navigation paths and velocity commands."""

from __future__ import annotations

import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any

from core.backend_status import BackendStatus
from core.module import Module
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Path
from core.registry import register
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.stream import In
from core.transport.abc import TopicConfig

from .contracts import (
    THUNDER_FIELD_LCM_CONTRACT_NAME,
    LCMEndpointBinding,
    LCMEndpointContract,
    endpoint_contract,
)
from .endpoint_codec import dumps_endpoint_message

logger = logging.getLogger(__name__)


@register(
    "navigation",
    "lcm_path_command_bridge",
    description="LCM endpoint adapter for Thunder path and cmd_vel egress",
)
class LCMPathCommandBridgeModule(Module, layer=5):
    """Publish LingTu navigation outputs through the Thunder LCM endpoint contract."""

    global_path: In[list]
    local_path: In[Path]
    waypoint: In[PoseStamped]
    cmd_vel: In[Twist]

    def __init__(
        self,
        endpoint_contract: str = THUNDER_FIELD_LCM_CONTRACT_NAME,
        default_frame_id: str | None = None,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        qos_depth: int = 10,
        reliable: bool = False,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._contract = self._resolve_contract(endpoint_contract)
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._publishers: dict[str, Any] = {}
        self._publish_counts: Counter[str] = Counter()
        self._publish_errors: Counter[str] = Counter()
        self._last_publish_ts = 0.0
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._global_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.global_path)
        )
        self._local_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.local_path)
        )
        self._waypoint_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.nav_way_point)
        )
        self._backend_status = BackendStatus.configured_as("lcm_path_command_bridge")

    def setup(self) -> None:
        self.global_path.subscribe(self._on_global_path)
        self.local_path.subscribe(self._on_local_path)
        self.waypoint.subscribe(self._on_waypoint)
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self._transport = self._transport or self._create_default_transport()
        logger.info(
            "LCMPathCommandBridgeModule: publishing navigation egress for %s",
            self._contract.name,
        )

    def stop(self) -> None:
        for publisher in self._publishers.values():
            close = getattr(publisher, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("LCM publisher close failed", exc_info=True)
        self._publishers.clear()
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
            "published_channels": {
                topic: self._binding(topic).channel
                for topic in self._published_topics()
            },
            "publish_counts": dict(self._publish_counts),
            "publish_errors": dict(self._publish_errors),
            "last_publish_ts": self._last_publish_ts,
        }

    @staticmethod
    def _resolve_contract(contract_name: str) -> LCMEndpointContract:
        name = str(contract_name or THUNDER_FIELD_LCM_CONTRACT_NAME)
        return endpoint_contract(name)

    @staticmethod
    def _create_default_transport() -> Any:
        from core.transport.factory import create_transport

        return create_transport("lcm")

    def _on_global_path(self, path: Path | list[Any]) -> None:
        self._publish_message(
            TOPICS.global_path,
            self._normalize_path(path, default_frame_id=self._global_default_frame_id),
        )

    def _on_local_path(self, path: Path | list[Any]) -> None:
        self._publish_message(
            TOPICS.local_path,
            self._normalize_path(path, default_frame_id=self._local_default_frame_id),
        )

    def _on_waypoint(self, pose: Any) -> None:
        self._publish_message(
            TOPICS.nav_way_point,
            self._coerce_pose_stamped(pose, self._waypoint_default_frame_id),
        )

    def _on_cmd_vel(self, twist: Twist) -> None:
        if not isinstance(twist, Twist):
            self._publish_errors[TOPICS.cmd_vel] += 1
            logger.warning(
                "LCMPathCommandBridgeModule ignored non-Twist cmd_vel: %s",
                type(twist).__name__,
            )
            return
        self._publish_message(TOPICS.cmd_vel, twist)

    def _publish_message(self, topic: str, msg: Any) -> None:
        binding = self._binding(topic)
        try:
            payload = dumps_endpoint_message(binding, msg)
            self._publish_payload(binding, payload)
        except Exception:
            self._publish_errors[topic] += 1
            logger.exception("Failed to publish LCM endpoint payload for %s", topic)
            return
        self._publish_counts[topic] += 1
        self._last_publish_ts = time.time()

    def _publish_payload(self, binding: LCMEndpointBinding, payload: bytes) -> None:
        transport = self._transport
        if transport is None:
            raise RuntimeError("LCM path command bridge transport is not initialized")

        publish = getattr(transport, "publish", None)
        if callable(publish):
            publish(binding.channel, payload)
            return

        publisher = self._publishers.get(binding.topic)
        if publisher is None:
            create_publisher = getattr(transport, "create_publisher", None)
            if not callable(create_publisher):
                raise TypeError(
                    "LCM path command bridge transport must expose publish() "
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

    def _binding(self, topic: str) -> LCMEndpointBinding:
        binding = self._contract.binding_for_topic(topic)
        if binding.direction != "lingtu_to_endpoint":
            raise ValueError(f"{topic} is not a LingTu-to-endpoint LCM binding")
        return binding

    @staticmethod
    def _published_topics() -> tuple[str, ...]:
        return (
            TOPICS.global_path,
            TOPICS.local_path,
            TOPICS.nav_way_point,
            TOPICS.cmd_vel,
        )

    @classmethod
    def _normalize_path(cls, path: Path | list[Any], *, default_frame_id: str) -> Path:
        if isinstance(path, Path):
            frame_id = str(path.frame_id or default_frame_id)
            poses = [
                cls._coerce_pose_stamped(pose, frame_id)
                for pose in path.poses
            ]
            return Path(poses=poses, ts=float(path.ts or 0.0), frame_id=frame_id)

        frame_id = default_frame_id
        ts = 0.0
        if hasattr(path, "frame_id"):
            frame_id = str(getattr(path, "frame_id", "") or default_frame_id)
        if hasattr(path, "ts"):
            try:
                ts = float(getattr(path, "ts", 0.0) or 0.0)
            except (TypeError, ValueError):
                ts = 0.0

        poses_source = getattr(path, "poses", path) or []
        try:
            poses_iter = list(poses_source)
        except TypeError:
            poses_iter = []
        poses = [cls._coerce_pose_stamped(pose, frame_id) for pose in poses_iter]
        return Path(poses=poses, ts=ts, frame_id=frame_id)

    @classmethod
    def _coerce_pose_stamped(cls, value: Any, frame_id: str) -> PoseStamped:
        if isinstance(value, PoseStamped):
            return PoseStamped(
                pose=value.pose,
                ts=float(value.ts or 0.0),
                frame_id=str(value.frame_id or frame_id),
            )
        if isinstance(value, Pose):
            return PoseStamped(pose=value, frame_id=frame_id)
        if isinstance(value, Mapping):
            return cls._coerce_mapping_pose(value, frame_id)
        if hasattr(value, "pose"):
            pose = cls._coerce_pose(getattr(value, "pose"), frame_id)
            return PoseStamped(
                pose=pose,
                ts=cls._coerce_float(getattr(value, "ts", 0.0)),
                frame_id=str(getattr(value, "frame_id", "") or frame_id),
            )
        if hasattr(value, "x") and hasattr(value, "y"):
            return PoseStamped(
                pose=cls._pose_from_xyz_orientation(
                    getattr(value, "x", 0.0),
                    getattr(value, "y", 0.0),
                    getattr(value, "z", 0.0),
                    getattr(value, "orientation", None),
                ),
                frame_id=frame_id,
            )
        try:
            seq = list(value)
        except TypeError:
            seq = []
        if len(seq) >= 2:
            return PoseStamped(
                pose=cls._pose_from_xyz_orientation(
                    seq[0],
                    seq[1],
                    seq[2] if len(seq) >= 3 else 0.0,
                    seq[3:7] if len(seq) >= 7 else None,
                ),
                frame_id=frame_id,
            )
        return PoseStamped(frame_id=frame_id)

    @classmethod
    def _coerce_mapping_pose(cls, value: Mapping[str, Any], frame_id: str) -> PoseStamped:
        if "pose" in value:
            data = dict(value)
            data.setdefault("frame_id", frame_id)
            try:
                return PoseStamped.from_dict(data)
            except (TypeError, ValueError, KeyError):
                pose = cls._coerce_pose(data.get("pose"), frame_id)
                return PoseStamped(
                    pose=pose,
                    ts=cls._coerce_float(data.get("ts", 0.0)),
                    frame_id=str(data.get("frame_id") or frame_id),
                )
        return PoseStamped(
            pose=cls._pose_from_xyz_orientation(
                value.get("x", 0.0),
                value.get("y", 0.0),
                value.get("z", 0.0),
                value.get("orientation"),
            ),
            ts=cls._coerce_float(value.get("ts", 0.0)),
            frame_id=str(value.get("frame_id") or frame_id),
        )

    @classmethod
    def _coerce_pose(cls, value: Any, frame_id: str) -> Pose:
        del frame_id
        if isinstance(value, Pose):
            return value
        if isinstance(value, Mapping):
            try:
                return Pose.from_dict(dict(value))
            except (TypeError, ValueError, KeyError):
                return cls._pose_from_xyz_orientation(
                    value.get("x", 0.0),
                    value.get("y", 0.0),
                    value.get("z", 0.0),
                    value.get("orientation"),
                )
        if hasattr(value, "position"):
            return Pose(
                cls._coerce_position(getattr(value, "position")),
                cls._coerce_orientation(getattr(value, "orientation", None)),
            )
        if hasattr(value, "x") and hasattr(value, "y"):
            return cls._pose_from_xyz_orientation(
                getattr(value, "x", 0.0),
                getattr(value, "y", 0.0),
                getattr(value, "z", 0.0),
                getattr(value, "orientation", None),
            )
        try:
            seq = list(value)
        except TypeError:
            seq = []
        if len(seq) >= 2:
            return cls._pose_from_xyz_orientation(
                seq[0],
                seq[1],
                seq[2] if len(seq) >= 3 else 0.0,
                seq[3:7] if len(seq) >= 7 else None,
            )
        return Pose()

    @classmethod
    def _pose_from_xyz_orientation(
        cls,
        x: Any,
        y: Any,
        z: Any,
        orientation: Any = None,
    ) -> Pose:
        return Pose(
            Vector3(cls._coerce_float(x), cls._coerce_float(y), cls._coerce_float(z)),
            cls._coerce_orientation(orientation),
        )

    @classmethod
    def _coerce_position(cls, value: Any) -> Vector3:
        if isinstance(value, Vector3):
            return value
        if isinstance(value, Mapping):
            return Vector3(
                cls._coerce_float(value.get("x", 0.0)),
                cls._coerce_float(value.get("y", 0.0)),
                cls._coerce_float(value.get("z", 0.0)),
            )
        if hasattr(value, "x") and hasattr(value, "y"):
            return Vector3(
                cls._coerce_float(getattr(value, "x", 0.0)),
                cls._coerce_float(getattr(value, "y", 0.0)),
                cls._coerce_float(getattr(value, "z", 0.0)),
            )
        try:
            return Vector3(value)
        except (TypeError, ValueError):
            return Vector3()

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
        try:
            return Quaternion(value)
        except (TypeError, ValueError):
            return Quaternion()

    @staticmethod
    def _coerce_float(value: Any) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0
