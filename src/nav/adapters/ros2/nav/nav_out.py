"""ROS 2 navigation output adapter."""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.nav import Path
from runtime.registry import register
from runtime.runtime_interface import TOPICS, body_frame_id, topic_default_frame_id
from runtime.stream import In

logger = logging.getLogger(__name__)


@register(
    "navigation",
    "ros2_nav_output",
    description="ROS 2 navigation output adapter",
)
class ROS2NavOutModule(Module, layer=5):
    """Publish LingTu navigation outputs as ROS 2 messages."""

    global_path: In[Path]
    local_path: In[Path]
    waypoint: In[PoseStamped]
    cmd_vel: In[Twist]

    def __init__(
        self,
        node_name: str = "nav_output",
        global_path_topic: str = TOPICS.global_path,
        local_path_topic: str = TOPICS.local_path,
        waypoint_topic: str = TOPICS.nav_way_point,
        cmd_vel_topic: str = TOPICS.cmd_vel,
        default_frame_id: str | None = None,
        cmd_frame_id: str | None = None,
        qos_depth: int = 10,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._node_name = node_name
        self._global_path_topic = global_path_topic
        self._local_path_topic = local_path_topic
        self._waypoint_topic = waypoint_topic
        self._cmd_vel_topic = cmd_vel_topic
        self._global_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.global_path)
        )
        self._local_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.local_path)
        )
        self._waypoint_default_frame_id = str(
            default_frame_id or topic_default_frame_id(TOPICS.nav_way_point)
        )
        self._cmd_frame_id = str(cmd_frame_id or body_frame_id())
        self._qos_depth = int(qos_depth)
        self._node = None
        self._executor = None
        self._pub_global = None
        self._pub_local = None
        self._pub_waypoint = None
        self._pub_cmd_vel = None

    def setup(self) -> None:
        self.global_path.subscribe(self._on_global_path)
        self.local_path.subscribe(self._on_local_path)
        self.waypoint.subscribe(self._on_waypoint)
        self.cmd_vel.subscribe(self._on_cmd_vel)

        try:
            from geometry_msgs.msg import (
                PoseStamped as ROSPoseStamped,
                TwistStamped as ROSTwistStamped,
            )
            from nav_msgs.msg import Path as ROSPath
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()
            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            qos = QoSProfile(depth=self._qos_depth, reliability=ReliabilityPolicy.RELIABLE)
            self._pub_global = node.create_publisher(ROSPath, self._global_path_topic, qos)
            self._pub_local = node.create_publisher(ROSPath, self._local_path_topic, qos)
            self._pub_waypoint = node.create_publisher(
                ROSPoseStamped,
                self._waypoint_topic,
                qos,
            )
            self._pub_cmd_vel = node.create_publisher(
                ROSTwistStamped,
                self._cmd_vel_topic,
                qos,
            )
            self._node = node
            self._executor = executor
            logger.info(
                "ROS2NavOutModule: publishing %s, %s, %s, %s",
                self._global_path_topic,
                self._local_path_topic,
                self._waypoint_topic,
                self._cmd_vel_topic,
            )
        except Exception as exc:
            self._cleanup_ros2_node()
            logger.warning("ROS2NavOutModule disabled: %s", exc)

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
        self._pub_waypoint = None
        self._pub_cmd_vel = None

    def _on_global_path(self, path: Path | list) -> None:
        if self._pub_global is not None:
            self._pub_global.publish(
                self._to_ros_path(path, default_frame_id=self._global_default_frame_id)
            )

    def _on_local_path(self, path: Path) -> None:
        if self._pub_local is not None:
            self._pub_local.publish(
                self._to_ros_path(path, default_frame_id=self._local_default_frame_id)
            )

    def _on_waypoint(self, pose: PoseStamped) -> None:
        if self._pub_waypoint is not None:
            self._pub_waypoint.publish(self._to_ros_waypoint(pose))

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self._pub_cmd_vel is not None:
            self._pub_cmd_vel.publish(self._to_ros_twist(twist))

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

    def _to_ros_waypoint(self, pose: PoseStamped):
        from geometry_msgs.msg import PoseStamped as ROSPoseStamped

        msg = ROSPoseStamped()
        msg.header.frame_id = str(
            getattr(pose, "frame_id", "") or self._waypoint_default_frame_id
        )
        if self._node is not None:
            msg.header.stamp = self._node.get_clock().now().to_msg()
        pos = pose.pose.position
        msg.pose.position.x = float(pos.x)
        msg.pose.position.y = float(pos.y)
        msg.pose.position.z = float(pos.z)
        orientation = pose.pose.orientation
        msg.pose.orientation.x = float(getattr(orientation, "x", 0.0))
        msg.pose.orientation.y = float(getattr(orientation, "y", 0.0))
        msg.pose.orientation.z = float(getattr(orientation, "z", 0.0))
        msg.pose.orientation.w = float(getattr(orientation, "w", 1.0))
        return msg

    def _to_ros_twist(self, twist: Twist):
        from geometry_msgs.msg import TwistStamped

        if not isinstance(twist, Twist):
            raise TypeError(f"cmd_vel expects Twist, got {type(twist).__name__}")
        msg = TwistStamped()
        msg.header.frame_id = self._cmd_frame_id
        if self._node is not None:
            msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.twist.linear.x = float(twist.linear.x)
        msg.twist.linear.y = float(twist.linear.y)
        msg.twist.linear.z = float(twist.linear.z)
        msg.twist.angular.x = float(twist.angular.x)
        msg.twist.angular.y = float(twist.angular.y)
        msg.twist.angular.z = float(twist.angular.z)
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
