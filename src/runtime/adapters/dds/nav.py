"""Typed DDS navigation input/output adapters."""

from __future__ import annotations

import logging
import time
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from runtime.msgs.nav import Path
from runtime.registry import register
from runtime.runtime_interface import TOPICS, body_frame_id, topic_default_frame_id
from runtime.stream import In, Out
from runtime.transport.abc import TopicConfig

logger = logging.getLogger(__name__)


@register(
    "navigation",
    "dds_nav_output",
    description="Typed DDS navigation output adapter",
)
class DDSNavOutModule(Module, layer=5):
    """Publish LingTu navigation outputs as typed DDS messages."""

    global_path: In[Path]
    local_path: In[Path]
    waypoint: In[PoseStamped]
    cmd_vel: In[Twist]

    def __init__(
        self,
        default_frame_id: str | None = None,
        cmd_frame_id: str | None = None,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        domain_id: int = 0,
        qos_depth: int = 10,
        reliable: bool = True,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._domain_id = int(domain_id)
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._publishers: dict[str, Any] = {}
        self._publish_counts: Counter[str] = Counter()
        self._publish_errors: Counter[str] = Counter()
        self._last_publish_ts = 0.0
        self._path_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.global_path)
        )
        self._local_path_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.local_path)
        )
        self._waypoint_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.nav_way_point)
        )
        self._cmd_frame_id = str(cmd_frame_id or body_frame_id())
        self._backend_status = BackendStatus.configured_as("dds_nav_output")

    def setup(self) -> None:
        self._transport = self._transport or self._create_default_transport()
        self.global_path.subscribe(
            lambda path: self._publish_path(
                TOPICS.global_path,
                path,
                self._path_frame_id,
            )
        )
        self.local_path.subscribe(
            lambda path: self._publish_path(
                TOPICS.local_path,
                path,
                self._local_path_frame_id,
            )
        )
        self.waypoint.subscribe(
            lambda pose: self._publish(
                TOPICS.nav_way_point,
                _to_dds_pose_stamped(pose, self._waypoint_frame_id),
            )
        )
        self.cmd_vel.subscribe(
            lambda twist: self._publish(
                TOPICS.cmd_vel,
                _to_dds_twist_stamped(twist, self._cmd_frame_id),
            )
        )

    def stop(self) -> None:
        for publisher in self._publishers.values():
            close = getattr(publisher, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS nav publisher close failed", exc_info=True)
        self._publishers.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS nav transport close failed", exc_info=True)
        self._transport = None
        super().stop()

    def health(self) -> dict[str, Any]:
        return {
            **self._backend_status.as_health_fields(),
            "transport": "dds",
            "published_topics": list(self._publishers),
            "publish_counts": dict(self._publish_counts),
            "publish_errors": dict(self._publish_errors),
            "last_publish_ts": self._last_publish_ts,
        }

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()
        from runtime.transport.dds import DDSTransport

        return DDSTransport(domain_id=self._domain_id)

    def _publish_path(self, topic: str, path: Path | list[Any], frame_id: str) -> None:
        self._publish(topic, _to_dds_path(path, frame_id))

    def _publish(self, topic: str, msg: Any) -> None:
        try:
            publisher = self._publisher(topic)
            publisher.publish(msg)
        except Exception:
            self._publish_errors[topic] += 1
            logger.exception("Failed to publish typed DDS nav payload for %s", topic)
            return
        self._publish_counts[topic] += 1
        self._last_publish_ts = time.time()

    def _publisher(self, topic: str) -> Any:
        publisher = self._publishers.get(topic)
        if publisher is not None:
            return publisher
        if self._transport is None:
            raise RuntimeError("DDS nav output transport is not initialized")
        publisher = self._transport.create_publisher(
            TopicConfig(
                name=topic,
                qos_depth=self._qos_depth,
                reliable=self._reliable,
            )
        )
        self._publishers[topic] = publisher
        return publisher


@register(
    "navigation",
    "dds_nav_input",
    description="Typed DDS navigation input adapter",
)
class DDSNavInModule(Module, layer=5):
    """Subscribe to typed DDS navigation commands and publish Module outputs."""

    goal_pose: Out[PoseStamped]
    cancel: Out[str]
    instruction: Out[str]

    def __init__(
        self,
        default_frame_id: str | None = None,
        transport: Any | None = None,
        transport_factory: Callable[[], Any] | None = None,
        domain_id: int = 0,
        qos_depth: int = 10,
        reliable: bool = True,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._transport = transport
        self._transport_factory = transport_factory
        self._owns_transport = transport is None
        self._domain_id = int(domain_id)
        self._qos_depth = int(qos_depth)
        self._reliable = bool(reliable)
        self._subscriptions: list[Any] = []
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_message_ts = 0.0
        self._default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.goal_pose)
        )
        self._backend_status = BackendStatus.configured_as("dds_nav_input")

    def setup(self) -> None:
        self._transport = self._transport or self._create_default_transport()
        self._subscribe(TOPICS.goal_pose, self._on_goal_pose)
        self._subscribe(TOPICS.cancel, self._on_cancel)
        self._subscribe(TOPICS.semantic_instruction, self._on_instruction)

    def stop(self) -> None:
        for sub in self._subscriptions:
            close = getattr(sub, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS nav subscription close failed", exc_info=True)
        self._subscriptions.clear()
        if self._owns_transport and self._transport is not None:
            close = getattr(self._transport, "close", None)
            if callable(close):
                try:
                    close()
                except (RuntimeError, OSError, ValueError):
                    logger.debug("DDS nav transport close failed", exc_info=True)
        self._transport = None
        super().stop()

    def health(self) -> dict[str, Any]:
        return {
            **self._backend_status.as_health_fields(),
            "transport": "dds",
            "subscribed_topics": [
                TOPICS.goal_pose,
                TOPICS.cancel,
                TOPICS.semantic_instruction,
            ],
            "message_counts": dict(self._message_counts),
            "decode_errors": dict(self._decode_errors),
            "last_message_ts": self._last_message_ts,
        }

    def _create_default_transport(self) -> Any:
        if self._transport_factory is not None:
            return self._transport_factory()
        from runtime.transport.dds import DDSTransport

        return DDSTransport(domain_id=self._domain_id)

    def _subscribe(self, topic: str, callback: Callable[[Any], None]) -> None:
        if self._transport is None:
            raise RuntimeError("DDS nav input transport is not initialized")
        sub = self._transport.create_subscriber(
            TopicConfig(
                name=topic,
                qos_depth=self._qos_depth,
                reliable=self._reliable,
            ),
            callback,
        )
        self._subscriptions.append(sub)

    def _on_goal_pose(self, msg: Any) -> None:
        try:
            self.goal_pose.publish(_from_dds_pose_stamped(msg, self._default_frame_id))
        except Exception:
            self._decode_errors[TOPICS.goal_pose] += 1
            logger.exception("Failed to decode typed DDS goal pose")
            return
        self._record_message(TOPICS.goal_pose)

    def _on_cancel(self, msg: Any) -> None:
        self.cancel.publish(_from_dds_string(msg))
        self._record_message(TOPICS.cancel)

    def _on_instruction(self, msg: Any) -> None:
        text = _from_dds_string(msg)
        if text:
            self.instruction.publish(text)
        self._record_message(TOPICS.semantic_instruction)

    def _record_message(self, topic: str) -> None:
        self._message_counts[topic] += 1
        self._last_message_ts = time.time()


def _dds() -> Any:
    from message import dds_types as dds_mod

    return dds_mod


def _to_dds_time(ts: float | int | None) -> Any:
    dds_mod = _dds()
    value = float(ts or time.time())
    sec = int(value)
    nanosec = int(max(0.0, value - sec) * 1_000_000_000)
    return dds_mod.DDS_Time(sec=sec, nanosec=nanosec)


def _from_dds_time(stamp: Any) -> float:
    sec = float(getattr(stamp, "sec", 0.0))
    nanosec = float(getattr(stamp, "nanosec", 0.0))
    return sec + nanosec / 1_000_000_000.0


def _to_dds_header(frame_id: str, ts: float | int | None) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Header(stamp=_to_dds_time(ts), frame_id=str(frame_id or ""))


def _to_dds_vector(value: Any) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Vector3(
        x=float(getattr(value, "x", 0.0)),
        y=float(getattr(value, "y", 0.0)),
        z=float(getattr(value, "z", 0.0)),
    )


def _to_dds_point(value: Any) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Point(
        x=float(getattr(value, "x", 0.0)),
        y=float(getattr(value, "y", 0.0)),
        z=float(getattr(value, "z", 0.0)),
    )


def _to_dds_quaternion(value: Any) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Quaternion(
        x=float(getattr(value, "x", 0.0)),
        y=float(getattr(value, "y", 0.0)),
        z=float(getattr(value, "z", 0.0)),
        w=float(getattr(value, "w", 1.0)),
    )


def _to_dds_pose(value: Pose) -> Any:
    dds_mod = _dds()
    return dds_mod.DDS_Pose(
        position=_to_dds_point(value.position),
        orientation=_to_dds_quaternion(value.orientation),
    )


def _coerce_pose_stamped(value: Any, frame_id: str) -> PoseStamped:
    if isinstance(value, PoseStamped):
        return PoseStamped(
            pose=value.pose,
            ts=float(value.ts or 0.0),
            frame_id=str(value.frame_id or frame_id),
        )
    if isinstance(value, Pose):
        return PoseStamped(pose=value, frame_id=frame_id)
    if isinstance(value, Mapping):
        if "pose" in value:
            data = dict(value)
            data.setdefault("frame_id", frame_id)
            return PoseStamped.from_dict(data)
        return PoseStamped(
            pose=Pose(
                float(value.get("x", 0.0)),
                float(value.get("y", 0.0)),
                float(value.get("z", 0.0)),
            ),
            frame_id=str(value.get("frame_id") or frame_id),
            ts=float(value.get("ts", 0.0) or 0.0),
        )
    if hasattr(value, "x") and hasattr(value, "y"):
        return PoseStamped(
            pose=Pose(
                float(getattr(value, "x", 0.0)),
                float(getattr(value, "y", 0.0)),
                float(getattr(value, "z", 0.0)),
            ),
            frame_id=frame_id,
        )
    seq = list(value)
    return PoseStamped(
        pose=Pose(
            float(seq[0]),
            float(seq[1]),
            float(seq[2]) if len(seq) > 2 else 0.0,
        ),
        frame_id=frame_id,
    )


def _to_dds_pose_stamped(value: Any, frame_id: str) -> Any:
    dds_mod = _dds()
    pose = _coerce_pose_stamped(value, frame_id)
    return dds_mod.DDS_PoseStamped(
        header=_to_dds_header(pose.frame_id or frame_id, pose.ts),
        pose=_to_dds_pose(pose.pose),
    )


def _to_dds_path(path: Path | list[Any], frame_id: str) -> Any:
    dds_mod = _dds()
    effective_frame = str(getattr(path, "frame_id", "") or frame_id)
    ts = float(getattr(path, "ts", 0.0) or time.time())
    poses = [
        _to_dds_pose_stamped(point, effective_frame)
        for point in (getattr(path, "poses", path) or [])
    ]
    return dds_mod.DDS_Path(
        header=_to_dds_header(effective_frame, ts),
        poses=poses,
    )


def _to_dds_twist_stamped(twist: Twist, frame_id: str) -> Any:
    dds_mod = _dds()
    if not isinstance(twist, Twist):
        raise TypeError(f"cmd_vel expects Twist, got {type(twist).__name__}")
    return dds_mod.DDS_TwistStamped(
        header=_to_dds_header(frame_id, getattr(twist, "ts", 0.0)),
        twist=dds_mod.DDS_Twist(
            linear=_to_dds_vector(twist.linear),
            angular=_to_dds_vector(twist.angular),
        ),
    )


def _from_dds_pose_stamped(msg: Any, frame_id: str) -> PoseStamped:
    header = getattr(msg, "header", None)
    pose = getattr(msg, "pose", None)
    if pose is None:
        raise TypeError("DDS PoseStamped missing pose")
    position = getattr(pose, "position")
    orientation = getattr(pose, "orientation", None)
    return PoseStamped(
        pose=Pose(
            position=Vector3(
                float(getattr(position, "x", 0.0)),
                float(getattr(position, "y", 0.0)),
                float(getattr(position, "z", 0.0)),
            ),
            orientation=Quaternion(
                float(getattr(orientation, "x", 0.0)),
                float(getattr(orientation, "y", 0.0)),
                float(getattr(orientation, "z", 0.0)),
                float(getattr(orientation, "w", 1.0)),
            ),
        ),
        ts=_from_dds_time(getattr(header, "stamp", None)),
        frame_id=str(getattr(header, "frame_id", "") or frame_id),
    )


def _from_dds_string(msg: Any) -> str:
    return str(getattr(msg, "data", msg) or "")
