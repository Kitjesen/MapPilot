"""ROS 2 visualization bridge for LingTu navigation paths."""

from __future__ import annotations

import logging
from typing import Any

from core.module import Module
from core.msgs.nav import Path
from core.registry import register
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.stream import In

logger = logging.getLogger(__name__)


@register(
    "navigation",
    "ros2_path_bridge",
    description="ROS 2 nav_msgs/Path visualization bridge",
)
class ROS2PathBridgeModule(Module, layer=5):
    """Publish internal LingTu paths as ROS 2 nav_msgs/Path for RViz."""

    global_path: In[list]
    local_path: In[Path]

    def __init__(
        self,
        node_name: str = "nav_path_bridge",
        global_path_topic: str = TOPICS.global_path,
        local_path_topic: str = TOPICS.local_path,
        default_frame_id: str | None = None,
        qos_depth: int = 10,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._node_name = node_name
        self._global_path_topic = global_path_topic
        self._local_path_topic = local_path_topic
        self._global_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.global_path)
        )
        self._local_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.local_path)
        )
        self._qos_depth = int(qos_depth)
        self._node = None
        self._executor = None
        self._pub_global = None
        self._pub_local = None

    def setup(self) -> None:
        self.global_path.subscribe(self._on_global_path)
        self.local_path.subscribe(self._on_local_path)

        try:
            from nav_msgs.msg import Path as ROSPath
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            from compat.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            qos = QoSProfile(depth=self._qos_depth, reliability=ReliabilityPolicy.RELIABLE)
            self._pub_global = node.create_publisher(ROSPath, self._global_path_topic, qos)
            self._pub_local = node.create_publisher(ROSPath, self._local_path_topic, qos)
            self._node = node
            self._executor = executor
            logger.info(
                "ROS2PathBridgeModule: publishing %s and %s",
                self._global_path_topic,
                self._local_path_topic,
            )
        except Exception as exc:
            self._cleanup_ros2_node()
            logger.warning("ROS2PathBridgeModule disabled: %s", exc)

    def stop(self) -> None:
        self._cleanup_ros2_node()
        super().stop()

    def _cleanup_ros2_node(self) -> None:
        node = self._node
        executor = self._executor
        if node is not None:
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
        self._pub_global = None
        self._pub_local = None

    def _on_global_path(self, path: list) -> None:
        if self._pub_global is not None:
            self._pub_global.publish(
                self._to_ros_path(path, default_frame_id=self._global_default_frame_id)
            )

    def _on_local_path(self, path: Path) -> None:
        if self._pub_local is not None:
            self._pub_local.publish(
                self._to_ros_path(path, default_frame_id=self._local_default_frame_id)
            )

    def _to_ros_path(self, path: Path | list, *, default_frame_id: str | None = None):
        from geometry_msgs.msg import PoseStamped as ROSPoseStamped
        from nav_msgs.msg import Path as ROSPath

        msg = ROSPath()
        effective_default_frame_id = default_frame_id or self._local_default_frame_id
        frame_id = str(getattr(path, "frame_id", "") or effective_default_frame_id)
        stamp = self._node.get_clock().now().to_msg() if self._node is not None else None
        msg.header.frame_id = frame_id
        if stamp is not None:
            msg.header.stamp = stamp

        poses = getattr(path, "poses", path) or []
        for pose in poses:
            ros_pose = ROSPoseStamped()
            ros_pose.header.frame_id = str(getattr(pose, "frame_id", "") or frame_id)
            if stamp is not None:
                ros_pose.header.stamp = stamp
            x, y, z = self._coerce_xyz(pose)
            ros_pose.pose.position.x = x
            ros_pose.pose.position.y = y
            ros_pose.pose.position.z = z
            orientation = getattr(pose, "orientation", None)
            ros_pose.pose.orientation.x = float(getattr(orientation, "x", 0.0))
            ros_pose.pose.orientation.y = float(getattr(orientation, "y", 0.0))
            ros_pose.pose.orientation.z = float(getattr(orientation, "z", 0.0))
            ros_pose.pose.orientation.w = float(getattr(orientation, "w", 1.0))
            msg.poses.append(ros_pose)
        return msg

    @staticmethod
    def _coerce_xyz(pose: Any) -> tuple[float, float, float]:
        if hasattr(pose, "x") and hasattr(pose, "y"):
            return (
                float(getattr(pose, "x", 0.0)),
                float(getattr(pose, "y", 0.0)),
                float(getattr(pose, "z", 0.0)),
            )
        try:
            arr = list(pose)
        except TypeError:
            arr = []
        if len(arr) >= 2:
            return (
                float(arr[0]),
                float(arr[1]),
                float(arr[2]) if len(arr) >= 3 else 0.0,
            )
        return 0.0, 0.0, 0.0
