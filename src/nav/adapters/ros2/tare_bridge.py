"""TAREROS2BridgeModule 鈥?rclpy-based subscriber for TARE exploration topics.

Extracted from ``module.py`` where ROS fallbacks polluted the main module.
This bridge handles *only* the rclpy path. The main ``TAREExplorerModule`` uses
the in-process policy by default and never imports rclpy.

Usage
-----
Add this module to the blueprint when the TARE C++ node publishes over
ROS2 topics from the TARE exploration topic contract
and cyclonedds is not available or not preferred::

    bp.add(
        TAREROS2BridgeModule,
        alias="TAREROS2BridgeModule",
        way_point_topic=TOPICS.exploration_way_point,
        ...
    )
    bp.wire("TAREROS2BridgeModule", "exploration_goal", "nav.mission", "goal_pose")
    ...

Output port contract matches ``TAREExplorerModule.exploration_goal``
so ``autoconnect`` wiring is identical.
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

_DEFAULT_GOAL_FRAME_ID = topic_default_frame_id(TOPICS.goal_pose)


@register(
    "exploration",
    "tare_ros2_bridge",
    description="ROS2 rclpy bridge for TARE exploration topics",
)
class TAREROS2BridgeModule(Module, layer=5):
    """Subscribe to TARE ROS2 topics and publish via framework ports.

    Replaces the rclpy fallback formerly embedded in ``TAREExplorerModule``.
    Publish the same output port contract so the rest of the system is
    unchanged.
    """

    exploration_goal: Out[PoseStamped]  # 鈫?Navigation.goal_pose
    exploration_path: Out[list]         # optional executable strategy path
    exploring: Out[bool]               # activity indicator
    runtime: Out[float]                # per-cycle runtime ms
    finish: Out[bool]                  # exploration done
    alive: Out[bool]                   # bridge health

    # Receive start/stop signals from blueprint (wired from TAREExplorerModule
    # or directly from user control).
    start_signal: In[bool]

    def __init__(
        self,
        way_point_topic: str = TOPICS.exploration_way_point,
        path_topic: str = TOPICS.exploration_local_path,
        runtime_topic: str = TOPICS.exploration_runtime,
        finish_topic: str = TOPICS.exploration_finish,
        start_topic: str = TOPICS.exploration_start,
        goal_frame_id: str = "",
        qos_depth: int = 10,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._way_point_topic = way_point_topic
        self._path_topic = path_topic
        self._runtime_topic = runtime_topic
        self._finish_topic = finish_topic
        self._start_topic = start_topic
        self._goal_frame_id = str(goal_frame_id or "")
        self._qos_depth = int(qos_depth)

        self._node = None
        self._executor = None
        self._subscriptions: list[Any] = []
        self._publisher = None

    # 鈹€鈹€ lifecycle 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def setup(self) -> None:
        self.start_signal.subscribe(self._on_start_signal)

        try:
            from geometry_msgs.msg import PointStamped as ROS2PointStamped
            from nav_msgs.msg import Path as ROS2Path
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Bool as ROS2Bool
            from std_msgs.msg import Float32 as ROS2Float32

            from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=self._qos_depth)
            node = Node("tare_bridge")

            self._subscriptions = [
                node.create_subscription(
                    ROS2PointStamped, self._way_point_topic,
                    self._on_waypoint, qos,
                ),
                node.create_subscription(
                    ROS2Path, self._path_topic,
                    self._on_path, qos,
                ),
                node.create_subscription(
                    ROS2Float32, self._runtime_topic,
                    self._on_runtime, qos,
                ),
                node.create_subscription(
                    ROS2Bool, self._finish_topic,
                    self._on_finish, qos,
                ),
            ]
            self._publisher = node.create_publisher(ROS2Bool, self._start_topic, qos)
            self._node = node
            self._executor = get_shared_executor()
            self._executor.add_node(node)
            logger.info("TAREROS2BridgeModule: subscribed to TARE ROS2 topics")
        except Exception as exc:
            self._cleanup_ros2_node()
            logger.warning("TAREROS2BridgeModule disabled: %s", exc)

    def start(self) -> None:
        super().start()
        self.alive.publish(self._node is not None)

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
        self._subscriptions.clear()
        self._publisher = None

    # 鈹€鈹€ ROS2 callbacks 鈫?framework ports 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_waypoint(self, msg) -> None:
        try:
            frame = msg.header.frame_id or _DEFAULT_GOAL_FRAME_ID
            output_frame = self._goal_frame_id or frame
            pose = PoseStamped(
                pose=Pose(
                    position=Vector3(
                        x=float(msg.point.x),
                        y=float(msg.point.y),
                        z=float(msg.point.z),
                    ),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                frame_id=output_frame,
            )
            self.exploration_goal.publish(pose)
        except Exception as e:
            logger.debug("TARE bridge waypoint error: %s", e)

    def _on_path(self, msg) -> None:
        try:
            frame = str(
                getattr(getattr(msg, "header", None), "frame_id", "")
                or _DEFAULT_GOAL_FRAME_ID
            )
            pts = [
                {"x": float(ps.pose.position.x),
                 "y": float(ps.pose.position.y),
                 "z": float(ps.pose.position.z),
                 "frame_id": frame}
                for ps in msg.poses
            ]
            self.exploration_path.publish(pts)
        except Exception as e:
            logger.debug("TARE bridge path error: %s", e)

    def _on_runtime(self, msg) -> None:
        try:
            self.runtime.publish(float(msg.data))
        except Exception:
            pass

    def _on_finish(self, msg) -> None:
        try:
            self.finish.publish(bool(msg.data))
        except Exception:
            pass

    def _on_start_signal(self, enable: bool) -> None:
        """Forward start/stop signal to the TARE ROS2 node."""
        if self._publisher is None:
            return
        try:
            from std_msgs.msg import Bool as ROS2Bool

            msg = ROS2Bool()
            msg.data = bool(enable)
            self._publisher.publish(msg)
        except Exception as e:
            logger.debug("TARE bridge start signal error: %s", e)
