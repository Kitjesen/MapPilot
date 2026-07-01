"""ROS 2 waypoint bridge for LingTu navigation waypoints.

Subscribes to NavigationModule's ``waypoint`` Out port and publishes
``geometry_msgs/PointStamped`` to ``/nav/way_point`` so external ROS 2
tools can display the active navigation waypoint.
"""

from __future__ import annotations

import logging
from typing import Any

from core.module import Module
from core.msgs.geometry import PoseStamped
from core.registry import register
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.stream import In

logger = logging.getLogger(__name__)


@register(
    "navigation",
    "ros2_waypoint_bridge",
    description="ROS 2 geometry_msgs/PointStamped waypoint bridge",
)
class ROS2WaypointBridgeModule(Module, layer=5):
    """Publish the active navigation waypoint as a ROS 2 PointStamped."""

    waypoint: In[PoseStamped]

    def __init__(
        self,
        node_name: str = "nav_waypoint_bridge",
        waypoint_topic: str = TOPICS.nav_way_point,
        default_frame_id: str | None = None,
        qos_depth: int = 10,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._node_name = node_name
        self._waypoint_topic = waypoint_topic
        self._default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.nav_way_point)
        )
        self._qos_depth = int(qos_depth)
        self._node = None
        self._executor = None
        self._pub = None

    def setup(self) -> None:
        self.waypoint.subscribe(self._on_waypoint)

        try:
            from geometry_msgs.msg import PointStamped
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            from compat.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            qos = QoSProfile(
                depth=self._qos_depth, reliability=ReliabilityPolicy.RELIABLE
            )
            self._pub = node.create_publisher(PointStamped, self._waypoint_topic, qos)
            self._node = node
            self._executor = executor
            logger.info(
                "ROS2WaypointBridgeModule: publishing %s",
                self._waypoint_topic,
            )
        except Exception as exc:
            self._cleanup_ros2_node()
            logger.warning("ROS2WaypointBridgeModule disabled: %s", exc)

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
        self._pub = None

    def _on_waypoint(self, pose: PoseStamped) -> None:
        if self._pub is None:
            return
        try:
            from geometry_msgs.msg import PointStamped

            pt = PointStamped()
            frame_id = str(getattr(pose, "frame_id", "") or self._default_frame_id)
            pt.header.frame_id = frame_id
            pos = pose.pose.position
            pt.point.x = float(pos.x)
            pt.point.y = float(pos.y)
            pt.point.z = float(pos.z)
            self._pub.publish(pt)
        except Exception:
            pass
