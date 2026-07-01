"""ROS 2 navigation input adapter."""

from __future__ import annotations

import logging
import time
from collections import Counter
from typing import Any

from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import Out

logger = logging.getLogger(__name__)


@register(
    "navigation",
    "ros2_nav_input",
    description="ROS 2 navigation input adapter",
)
class ROS2NavInModule(Module, layer=5):
    """Subscribe to ROS 2 navigation commands and publish LingTu inputs."""

    goal_pose: Out[PoseStamped]
    cancel: Out[str]
    instruction: Out[str]

    def __init__(
        self,
        node_name: str = "nav_input",
        goal_pose_topic: str = TOPICS.goal_pose,
        cancel_topic: str = TOPICS.cancel,
        instruction_topic: str = TOPICS.semantic_instruction,
        default_frame_id: str | None = None,
        qos_depth: int = 10,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._node_name = node_name
        self._goal_pose_topic = goal_pose_topic
        self._cancel_topic = cancel_topic
        self._instruction_topic = instruction_topic
        self._default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.goal_pose)
        )
        self._qos_depth = int(qos_depth)
        self._node = None
        self._executor = None
        self._subscriptions: list[Any] = []
        self._message_counts: Counter[str] = Counter()
        self._decode_errors: Counter[str] = Counter()
        self._last_message_ts = 0.0
        self._backend_status = BackendStatus.configured_as("ros2_nav_input")

    def setup(self) -> None:
        try:
            from geometry_msgs.msg import PoseStamped as ROSPoseStamped
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String as ROSString

            from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            qos = QoSProfile(depth=self._qos_depth, reliability=ReliabilityPolicy.RELIABLE)
            self._subscriptions = [
                node.create_subscription(
                    ROSPoseStamped,
                    self._goal_pose_topic,
                    self._on_goal_pose,
                    qos,
                ),
                node.create_subscription(
                    ROSString,
                    self._cancel_topic,
                    self._on_cancel,
                    qos,
                ),
                node.create_subscription(
                    ROSString,
                    self._instruction_topic,
                    self._on_instruction,
                    qos,
                ),
            ]
            self._node = node
            self._executor = executor
            logger.info(
                "ROS2NavInModule: subscribing to %s, %s, %s",
                self._goal_pose_topic,
                self._cancel_topic,
                self._instruction_topic,
            )
        except Exception as exc:
            self._cleanup_ros2_node()
            logger.warning("ROS2NavInModule disabled: %s", exc)

    def stop(self) -> None:
        self._cleanup_ros2_node()
        super().stop()

    def health(self) -> dict[str, Any]:
        return {
            **self._backend_status.as_health_fields(),
            "transport": "ros2",
            "subscribed_topics": [
                self._goal_pose_topic,
                self._cancel_topic,
                self._instruction_topic,
            ],
            "message_counts": dict(self._message_counts),
            "decode_errors": dict(self._decode_errors),
            "last_message_ts": self._last_message_ts,
        }

    def _cleanup_ros2_node(self) -> None:
        node = self._node
        executor = self._executor
        if node is not None:
            for sub in self._subscriptions:
                try:
                    node.destroy_subscription(sub)
                except Exception:
                    pass
            try:
                if executor is not None:
                    executor.remove_node(node)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        self._node = None
        self._executor = None
        self._subscriptions = []

    def _on_goal_pose(self, msg: Any) -> None:
        try:
            self.goal_pose.publish(self._from_ros_pose_stamped(msg))
        except Exception:
            self._decode_errors[TOPICS.goal_pose] += 1
            logger.exception("Failed to decode ROS2 goal pose")
            return
        self._record_message(TOPICS.goal_pose)

    def _on_cancel(self, msg: Any) -> None:
        self.cancel.publish(self._from_ros_string(msg))
        self._record_message(TOPICS.cancel)

    def _on_instruction(self, msg: Any) -> None:
        text = self._from_ros_string(msg)
        if text:
            self.instruction.publish(text)
        self._record_message(TOPICS.semantic_instruction)

    def _record_message(self, topic: str) -> None:
        self._message_counts[topic] += 1
        self._last_message_ts = time.time()

    def _from_ros_pose_stamped(self, msg: Any) -> PoseStamped:
        header = getattr(msg, "header", None)
        frame_id = str(getattr(header, "frame_id", "") or self._default_frame_id)
        ros_pose = getattr(msg, "pose", None)
        position = getattr(ros_pose, "position", None)
        orientation = getattr(ros_pose, "orientation", None)
        return PoseStamped(
            pose=Pose(
                Vector3(
                    float(getattr(position, "x", 0.0)),
                    float(getattr(position, "y", 0.0)),
                    float(getattr(position, "z", 0.0)),
                ),
                Quaternion(
                    float(getattr(orientation, "x", 0.0)),
                    float(getattr(orientation, "y", 0.0)),
                    float(getattr(orientation, "z", 0.0)),
                    float(getattr(orientation, "w", 1.0)),
                ),
            ),
            ts=self._from_ros_time(getattr(header, "stamp", None)),
            frame_id=frame_id,
        )

    @staticmethod
    def _from_ros_string(msg: Any) -> str:
        return str(getattr(msg, "data", msg) or "")

    @staticmethod
    def _from_ros_time(stamp: Any) -> float:
        if stamp is None:
            return 0.0
        sec = float(getattr(stamp, "sec", 0.0))
        nanosec = float(getattr(stamp, "nanosec", 0.0))
        return sec + nanosec / 1_000_000_000.0
