"""ROS2 visualization overlay adapter for the Rerun gateway module."""

from __future__ import annotations

from typing import Any, Protocol


class RerunOverlayCallbacks(Protocol):
    """Callbacks implemented by the Rerun Module and invoked by ROS2 overlays."""

    def _on_ros2_color(self, msg: Any) -> None: ...
    def _on_ros2_depth(self, msg: Any) -> None: ...
    def _on_ros2_tf(self, msg: Any) -> None: ...
    def _on_ros2_tf_static(self, msg: Any) -> None: ...
    def _on_ros2_costmap(self, msg: Any) -> None: ...
    def _on_ros2_detections(self, msg: Any) -> None: ...
    def _on_ros2_path(self, msg: Any) -> None: ...


class RerunRos2Overlay:
    """Own the optional ROS2 subscriptions used only for Rerun overlays."""

    def __init__(self, callbacks: RerunOverlayCallbacks, topics: Any) -> None:
        self._callbacks = callbacks
        self._topics = topics
        self._node = None
        self._subscriptions: list[Any] = []

    def start(self) -> None:
        """Create ROS2 subscriptions and attach them to the shared executor."""

        from nav_msgs.msg import OccupancyGrid, Path
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from tf2_msgs.msg import TFMessage
        from visualization_msgs.msg import MarkerArray

        from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor

        ensure_rclpy()
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

        self._node = Node("rerun_bridge")
        topics = self._topics
        callbacks = self._callbacks
        self._subscriptions = [
            self._node.create_subscription(Image, topics.camera_color, callbacks._on_ros2_color, qos),
            self._node.create_subscription(Image, topics.camera_depth, callbacks._on_ros2_depth, qos),
            self._node.create_subscription(TFMessage, "/tf", callbacks._on_ros2_tf, qos_be),
            self._node.create_subscription(TFMessage, "/tf_static", callbacks._on_ros2_tf_static, qos_be),
            self._node.create_subscription(OccupancyGrid, topics.semantic_costmap, callbacks._on_ros2_costmap, qos_be),
            self._node.create_subscription(
                MarkerArray,
                topics.visualization_detections,
                callbacks._on_ros2_detections,
                qos_be,
            ),
            self._node.create_subscription(Path, topics.global_path, callbacks._on_ros2_path, qos_be),
        ]
        get_shared_executor().add_node(self._node)

    def stop(self) -> None:
        """Destroy the ROS2 overlay node and release subscriptions."""

        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._subscriptions.clear()


__all__ = ["RerunOverlayCallbacks", "RerunRos2Overlay"]
