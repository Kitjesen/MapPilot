"""ROS 2 map output adapter."""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.msgs.numpy_compat import np
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In

logger = logging.getLogger(__name__)


@register("map", "ros2_map_output", description="ROS 2 map output adapter")
class ROS2MapOutModule(Module, layer=2):
    """Publish internal LingTu exploration grids as ROS 2 OccupancyGrid."""

    exploration_grid: In[dict]

    def __init__(
        self,
        node_name: str = "map_output",
        exploration_grid_topic: str = TOPICS.exploration_grid,
        default_frame_id: str | None = None,
        qos_depth: int = 2,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._node_name = str(node_name)
        self._exploration_grid_topic = str(exploration_grid_topic)
        self._default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.exploration_grid)
        )
        self._qos_depth = int(qos_depth)
        self._node = None
        self._executor = None
        self._pub_exploration = None

    def setup(self) -> None:
        self.exploration_grid.subscribe(self._on_exploration_grid)
        self.exploration_grid.set_policy("latest")

        try:
            from nav_msgs.msg import OccupancyGrid as ROSOccupancyGrid
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            qos = QoSProfile(
                depth=self._qos_depth,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._pub_exploration = node.create_publisher(
                ROSOccupancyGrid,
                self._exploration_grid_topic,
                qos,
            )
            self._node = node
            self._executor = executor
            logger.info(
                "ROS2MapOutModule: publishing %s",
                self._exploration_grid_topic,
            )
        except Exception as exc:
            self._cleanup_ros2_node()
            logger.warning("ROS2MapOutModule disabled: %s", exc)

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
        self._pub_exploration = None

    def _on_exploration_grid(self, grid: dict[str, Any]) -> None:
        if self._pub_exploration is None:
            return
        self._pub_exploration.publish(self._to_ros_grid(grid))

    def _to_ros_grid(self, grid: dict[str, Any]):
        from nav_msgs.msg import OccupancyGrid as ROSOccupancyGrid

        msg = ROSOccupancyGrid()
        msg.header.frame_id = str(grid.get("frame_id") or self._default_frame_id)
        if self._node is not None:
            msg.header.stamp = self._node.get_clock().now().to_msg()

        values = np.asarray(grid.get("grid"), dtype=np.int16)
        if values.ndim != 2:
            values = np.zeros((0, 0), dtype=np.int16)
        height, width = values.shape
        resolution = float(grid.get("resolution") or 0.0)
        origin = grid.get("origin")
        if origin is None:
            origin_xy = (
                float(grid.get("origin_x") or 0.0),
                float(grid.get("origin_y") or 0.0),
            )
        else:
            origin_xy = (float(origin[0]), float(origin[1]))

        msg.info.resolution = resolution
        msg.info.width = int(width)
        msg.info.height = int(height)
        msg.info.origin.position.x = origin_xy[0]
        msg.info.origin.position.y = origin_xy[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = np.clip(values, -1, 100).astype(np.int8).reshape(-1).tolist()
        return msg
