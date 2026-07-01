"""Lightweight topic contracts for portable adapters.

``PortableTopic`` is the ROS-topic replacement at LingTu's bottom layer: it
defines canonical topic names and LingTu-owned payload types.  Production
delivery should go through ``runtime.portable.topic_transport`` backed by the
existing Local/SHM/LCM/DDS transports.

``LocalTopicHub`` remains only as a test/local probe helper.  Do not use it as a
runtime transport, broker, discovery service, or DDS/LCM substitute.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Generic, TypeVar

from runtime.contracts.runtime import RuntimeContractRegistry, TopicContract
from runtime.msgs.geometry import Twist
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.sensor import CameraIntrinsics, Image, Imu, PointCloud2
from runtime.runtime_interface import TOPICS

from .contracts import PortableCommandFrame, PortablePlanningFrame, PortableSensorFrame

T = TypeVar("T")
Callback = Callable[[Any], None]


@dataclass(frozen=True)
class PortableTopic(Generic[T]):
    """Canonical topic token plus the LingTu-owned payload type."""

    name: str
    payload_type: type[T]
    field: str
    direction: str
    description: str = ""

    def contract(self, *, runtime_contract: str | None = None) -> TopicContract:
        return RuntimeContractRegistry().topic_contract(self.name, runtime_contract=runtime_contract)

    def contract_dict(self, *, runtime_contract: str | None = None) -> dict[str, Any]:
        """Return a JSON-ready topic contract, falling back for new portable topics."""

        try:
            return self.contract(runtime_contract=runtime_contract).to_dict()
        except ValueError:
            return {
                "topic": self.name,
                "formats": (self.payload_type.__name__,),
                "allowed_frame_ids": (),
                "default_frame_id": None,
            }


SENSOR_TOPICS: dict[str, PortableTopic[Any]] = {
    "odometry": PortableTopic(TOPICS.odometry, Odometry, "odometry", "source", "Normalized odometry"),
    "lidar_cloud": PortableTopic(TOPICS.registered_cloud, PointCloud2, "lidar_cloud", "source", "Body-frame LiDAR cloud"),
    "map_cloud": PortableTopic(TOPICS.map_cloud, PointCloud2, "map_cloud", "source", "Map/odom-frame cloud"),
    "imu": PortableTopic(TOPICS.imu, Imu, "imu", "source", "Normalized IMU sample"),
    "camera_image": PortableTopic(TOPICS.camera_color, Image, "camera_image", "source", "RGB camera image"),
    "depth_image": PortableTopic(TOPICS.camera_depth, Image, "depth_image", "source", "Depth image"),
    "camera_info": PortableTopic(TOPICS.camera_info, CameraIntrinsics, "camera_info", "source", "Camera intrinsics"),
}

COMMAND_TOPICS: dict[str, PortableTopic[Any]] = {
    "cmd_vel": PortableTopic(TOPICS.cmd_vel, Twist, "cmd_vel", "sink", "Arbitrated velocity command"),
    "stop_signal": PortableTopic(TOPICS.stop, int, "stop_signal", "sink", "Stop/safety level"),
}

PLANNING_TOPICS: dict[str, PortableTopic[Any]] = {
    "global_path": PortableTopic(TOPICS.global_path, Path, "global_path", "planning", "Global path"),
    "local_path": PortableTopic(TOPICS.local_path, Path, "local_path", "planning", "Local path"),
    "goal_path": PortableTopic(TOPICS.far_reach_goal, Path, "goal_path", "planning", "Goal/path input"),
}


class LocalTopicHub:
    """Tiny synchronous local topic probe for tests.

    Internal LingTu modules should use ModulePort/Blueprint wiring.  Cross-
    process or production adapter communication should use
    ``PortableTopicTransport`` over Local/SHM/LCM/DDS.  This class intentionally
    exists only for no-dependency unit tests and one-process probes.
    """

    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callback]] = defaultdict(list)
        self._last: dict[str, Any] = {}

    def subscribe(self, topic: str | PortableTopic[Any], callback: Callback) -> Callable[[], None]:
        name = topic_name(topic)
        self._subscribers[name].append(callback)

        def unsubscribe() -> None:
            callbacks = self._subscribers.get(name)
            if not callbacks:
                return
            try:
                callbacks.remove(callback)
            except ValueError:
                return
            if not callbacks:
                self._subscribers.pop(name, None)

        return unsubscribe

    def publish(self, topic: str | PortableTopic[Any], payload: Any) -> int:
        name = topic_name(topic)
        self._last[name] = payload
        callbacks = list(self._subscribers.get(name, ()))
        for callback in callbacks:
            callback(payload)
        return len(callbacks)

    def last(self, topic: str | PortableTopic[Any], default: Any = None) -> Any:
        return self._last.get(topic_name(topic), default)

    def publish_sensor_frame(self, frame: PortableSensorFrame) -> dict[str, int]:
        counts: dict[str, int] = {}
        for spec in SENSOR_TOPICS.values():
            payload = getattr(frame, spec.field)
            if payload is not None:
                counts[spec.name] = self.publish(spec, payload)
        return counts

    def publish_command_frame(self, frame: PortableCommandFrame) -> dict[str, int]:
        counts: dict[str, int] = {}
        for spec in COMMAND_TOPICS.values():
            payload = getattr(frame, spec.field)
            if payload is not None:
                counts[spec.name] = self.publish(spec, payload)
        return counts

    def publish_planning_frame(self, frame: PortablePlanningFrame) -> dict[str, int]:
        counts: dict[str, int] = {}
        for spec in PLANNING_TOPICS.values():
            payload = getattr(frame, spec.field)
            if payload is not None:
                counts[spec.name] = self.publish(spec, payload)
        return counts


def topic_name(topic: str | PortableTopic[Any]) -> str:
    return topic.name if isinstance(topic, PortableTopic) else str(topic)


def portable_topic_contracts(*, runtime_contract: str | None = None) -> dict[str, dict[str, Any]]:
    """Return JSON-ready contracts for the portable bottom-layer topic subset."""

    topics = {**SENSOR_TOPICS, **COMMAND_TOPICS, **PLANNING_TOPICS}
    return {
        key: {
            **spec.contract_dict(runtime_contract=runtime_contract),
            "payload_type": spec.payload_type.__name__,
            "field": spec.field,
            "direction": spec.direction,
            "description": spec.description,
        }
        for key, spec in sorted(topics.items())
    }
