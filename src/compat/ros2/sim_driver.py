"""ROS2SimDriverModule bridges ROS 2 simulation topics to Module ports.

Subscribes to ROS2 canonical odometry, registered-cloud, and camera topics.
Publishes to the ROS2 canonical cmd_vel topic (TwistStamped).

Module ports are identical to MujocoDriverModule for drop-in replacement.

Usage::

    from compat.ros2.sim_driver import ROS2SimDriverModule
    bp.add(ROS2SimDriverModule,
           node_name="lingtu_ros2_driver")
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Any

from core.module import Module
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.numpy_compat import np
from core.msgs.sensor import CameraIntrinsics, Image, ImageFormat, PointCloud2
from core.registry import register
from core.runtime_interface import (
    FRAMES,
    TOPICS,
    runtime_fixed_path_frame_ids,
    topic_default_frame_id,
)
from core.stream import In, Out

logger = logging.getLogger("drivers.sim.ros2_sim_driver")

# PointCloud2 point step when the cloud is XYZI (x,y,z,intensity each float32).
_XYZI_POINT_STEP = 16  # 4 x float32
ROS2_SIM_ODOM_FRAME_ID = topic_default_frame_id(TOPICS.odometry)
ROS2_SIM_REGISTERED_CLOUD_FRAME_ID = topic_default_frame_id(TOPICS.registered_cloud)
# The ROS2 simulation bridge has no map->odom localizer; live map cloud remains odom-frame.
ROS2_SIM_LIVE_MAP_CLOUD_FRAME_ID = ROS2_SIM_ODOM_FRAME_ID
ROS2_SIM_CMD_VEL_FRAME_ID = topic_default_frame_id(TOPICS.cmd_vel)
ROS2_SIM_GOAL_FRAME_ID = runtime_fixed_path_frame_ids()[0]
ROS2_SIM_CAMERA_FRAME_ID = FRAMES.camera


@register("driver", "sim_ros2", description="ROS2 bridge driver for MuJoCo simulation")
@register("driver_protocol", "ros2_bridge", description="ROS2 bridge simulation driver")
class ROS2SimDriverModule(Module, layer=1):
    """Bridges ROS2 topics from mujoco_ros2_bridge to Module ports.

    Subscribes to ROS2:
      canonical odometry         - nav_msgs/Odometry @ 50 Hz
      canonical registered cloud - sensor_msgs/PointCloud2 (XYZI) @ 10 Hz
      /camera/color/image_raw    - sensor_msgs/Image (optional, BGR8)

    Publishes to ROS2:
      canonical cmd_vel - geometry_msgs/TwistStamped

    Module ports mirror MujocoDriverModule for drop-in replacement.
    """

    # -- Inputs --
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # -- Outputs --
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud2]
    map_cloud: Out[PointCloud2]   # alias for lidar_cloud; feeds OccupancyGrid/Terrain
    camera_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraIntrinsics]
    goal_pose: Out[PoseStamped]
    alive: Out[bool]
    robot_state: Out[dict]

    def __init__(
        self,
        node_name: str = "lingtu_ros2_driver",
        spin_rate: float = 100.0,
        odom_topic: str = TOPICS.odometry,
        cloud_topic: str = TOPICS.registered_cloud,
        map_cloud_topic: str = TOPICS.map_cloud,
        image_topic: str = TOPICS.camera_color,
        depth_topic: str = TOPICS.camera_depth,
        info_topic: str = TOPICS.camera_info,
        cmd_vel_topic: str = TOPICS.cmd_vel,
        qos_depth: int = 10,
        # In real-robot nav mode, CameraBridgeModule (perception stack)
        # owns the camera/depth topics. Subscribing here AGAIN duplicates
        # the high-bandwidth depth stream and starves uvicorn of GIL.
        # Default off; turn on only for true MuJoCo simulation runs.
        enable_camera: bool = False,
        latch_stop_signal: bool = True,
        **kw,
    ):
        super().__init__(**kw)
        self._node_name = node_name
        # Kept for config/status compatibility. rclpy callbacks are handled by
        # the shared executor, not a private per-module spin loop.
        self._spin_rate = spin_rate
        self._odom_topic = odom_topic
        self._cloud_topic = cloud_topic
        self._map_cloud_topic = map_cloud_topic
        self._image_topic = image_topic
        self._depth_topic = depth_topic
        self._info_topic = info_topic
        self._cmd_vel_topic = cmd_vel_topic
        self._qos_depth = qos_depth
        self._enable_camera = enable_camera
        self._latch_stop_signal = bool(latch_stop_signal)

        self._node = None
        self._executor = None
        self._pub_cmd_vel = None
        self._running = False
        self._stopped = False
        self._cloud_worker_lock = threading.Lock()
        self._cloud_worker_thread: threading.Thread | None = None
        self._cloud_worker_drops = 0

        # Latest command, written by Module cmd_vel callback.
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_wz = 0.0

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)

        node = None
        executor = None
        try:
            from geometry_msgs.msg import TwistStamped
            from nav_msgs.msg import Odometry as ROS2Odom
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import CameraInfo, PointCloud2
            from sensor_msgs.msg import Image as ROS2Image
            try:
                from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            except ImportError:
                MutuallyExclusiveCallbackGroup = None

            from compat.ros2.context import ensure_rclpy, get_shared_executor

            ensure_rclpy()

            control_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                depth=self._qos_depth,
            )
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=1,
            )

            node = Node(self._node_name)
            executor = get_shared_executor()
            executor.add_node(node)
            group = (
                MutuallyExclusiveCallbackGroup() if MutuallyExclusiveCallbackGroup
                else None
            )
            odom_group = cloud_group = camera_group = control_group = group

            # Always-on subscribers (low bandwidth)
            self._create_subscription(
                node, ROS2Odom, self._odom_topic, self._on_ros2_odom,
                control_qos, callback_group=odom_group)
            self._create_subscription(
                node, PointCloud2, self._cloud_topic, self._on_ros2_registered_cloud,
                sensor_qos, callback_group=cloud_group)
            self._create_subscription(
                node, PointCloud2, self._map_cloud_topic, self._on_ros2_map_cloud,
                sensor_qos, callback_group=cloud_group)

            # Camera + depth: high bandwidth, only when explicitly enabled
            # (real-robot nav uses CameraBridgeModule which owns these topics)
            if self._enable_camera:
                self._create_subscription(
                    node, ROS2Image, self._image_topic, self._on_ros2_image,
                    sensor_qos, callback_group=camera_group)
                self._create_subscription(
                    node, ROS2Image, self._depth_topic, self._on_ros2_depth,
                    sensor_qos, callback_group=camera_group)
                self._create_subscription(
                    node, CameraInfo, self._info_topic, self._on_ros2_info,
                    sensor_qos, callback_group=camera_group)
                logger.info(
                    "ROS2SimDriverModule: camera subscriptions enabled "
                    "(%s, %s, %s)",
                    self._image_topic, self._depth_topic, self._info_topic,
                )

            # Goal pose subscriber bridges ROS2 TOPICS.goal_pose to Module port.
            from geometry_msgs.msg import PoseStamped as ROS2PoseStamped
            self._create_subscription(
                node, ROS2PoseStamped, TOPICS.goal_pose, self._on_ros2_goal,
                control_qos, callback_group=control_group)

            # Publisher
            pub_cmd_vel = node.create_publisher(
                TwistStamped, self._cmd_vel_topic, control_qos)
            self._node = node
            self._executor = executor
            self._pub_cmd_vel = pub_cmd_vel

            logger.info(
                "ROS2SimDriverModule: node '%s' created - odom=%s cloud=%s cmd_vel=%s",
                self._node_name, self._odom_topic, self._cloud_topic, self._cmd_vel_topic,
            )

        except ImportError as e:
            self._cleanup_ros2_node(node=node, executor=executor)
            logger.error("ROS2SimDriverModule: rclpy not available: %s", e)
        except Exception as e:
            self._cleanup_ros2_node(node=node, executor=executor)
            logger.error("ROS2SimDriverModule: setup failed: %s", e)

    def start(self):
        super().start()
        if self._node is None:
            self._running = False
            self.alive.publish(False)
            return

        self._running = True
        self.alive.publish(True)
        self._publish_robot_state()
        logger.info("ROS2SimDriverModule: callbacks handled by shared ROS2 executor")

    def stop(self):
        self._running = False
        if self._cloud_worker_thread and self._cloud_worker_thread.is_alive():
            self._cloud_worker_thread.join(timeout=1.0)
        self._cloud_worker_thread = None
        self._cleanup_ros2_node()
        self.alive.publish(False)
        self._publish_robot_state()
        super().stop()

    def _cleanup_ros2_node(self, node=None, executor=None) -> None:
        node = node if node is not None else self._node
        executor = executor if executor is not None else self._executor
        if node is not None:
            try:
                if executor is not None:
                    executor.remove_node(node)
            except Exception:
                logger.debug("ros2_sim: error removing node from executor", exc_info=True)
            try:
                node.destroy_node()
            except Exception:
                logger.debug("ros2_sim: error destroying node", exc_info=True)
        if node is self._node:
            self._node = None
        if executor is self._executor:
            self._executor = None
        self._pub_cmd_vel = None

    @staticmethod
    def _create_subscription(node, msg_type, topic, callback, qos, *, callback_group=None):
        if callback_group is None:
            return node.create_subscription(msg_type, topic, callback, qos)
        try:
            return node.create_subscription(
                msg_type, topic, callback, qos, callback_group=callback_group)
        except TypeError:
            return node.create_subscription(msg_type, topic, callback, qos)

    # -- ROS2 callbacks (called from shared executor) ------------------------

    def _on_ros2_odom(self, msg) -> None:
        """Convert ROS2 nav_msgs/Odometry to Module Odometry and publish."""
        odom = Odometry(
            pose=Pose(
                position=Vector3(
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ),
                orientation=Quaternion(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ),
            ),
            twist=Twist(
                linear=Vector3(
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ),
                angular=Vector3(
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ),
            ),
            ts=time.time(),
            frame_id=msg.header.frame_id or ROS2_SIM_ODOM_FRAME_ID,
        )
        self.odometry.publish(odom)

    def _on_ros2_cloud(self, msg) -> None:
        self._on_ros2_registered_cloud(msg)

    def _on_ros2_registered_cloud(self, msg) -> None:
        self._submit_cloud_worker(lambda: self._process_registered_cloud(msg))

    def _on_ros2_map_cloud(self, msg) -> None:
        self._submit_cloud_worker(lambda: self._process_map_cloud(msg))

    def _submit_cloud_worker(self, task) -> bool:
        if not self._running:
            return False
        if not self._cloud_worker_lock.acquire(blocking=False):
            self._cloud_worker_drops += 1
            return False

        def run() -> None:
            try:
                if self._running:
                    task()
            finally:
                self._cloud_worker_lock.release()

        self._cloud_worker_thread = threading.Thread(
            target=run,
            daemon=True,
            name="ros2_sim_driver_cloud",
        )
        self._cloud_worker_thread.start()
        return True

    def _pointcloud_from_ros2(self, msg, *, default_frame: str) -> PointCloud2 | None:
        """Convert ROS2 sensor_msgs/PointCloud2 to Module PointCloud2."""
        try:
            n_pts = msg.width * msg.height
            if n_pts == 0:
                return None

            raw = bytes(msg.data)
            step = msg.point_step

            if step == _XYZI_POINT_STEP:
                # Fast path: XYZI packed as 4 x float32; extract XYZ columns only.
                arr = np.frombuffer(raw, dtype=np.float32).reshape(n_pts, 4)
                pts = np.ascontiguousarray(arr[:, :3])
            else:
                # Generic path: parse x/y/z field offsets from PointFields
                offsets = _parse_xyz_offsets(msg.fields)
                if offsets is None:
                    logger.warning("ROS2SimDriverModule: cannot parse PointCloud2 fields")
                    return None
                ox, oy, oz = offsets
                pts = np.zeros((n_pts, 3), dtype=np.float32)
                for i in range(n_pts):
                    base = i * step
                    pts[i, 0] = struct.unpack_from("<f", raw, base + ox)[0]
                    pts[i, 1] = struct.unpack_from("<f", raw, base + oy)[0]
                    pts[i, 2] = struct.unpack_from("<f", raw, base + oz)[0]

            # Filter out invalid (NaN/Inf) points
            valid = np.isfinite(pts).all(axis=1)
            pts = pts[valid]

            if pts.shape[0] == 0:
                return None

            return PointCloud2(
                points=pts,
                frame_id=msg.header.frame_id or default_frame,
                ts=time.time(),
            )
        except Exception as e:
            logger.error("ROS2SimDriverModule: cloud conversion error: %s", e)
            return None

    def _process_registered_cloud(self, msg) -> None:
        cloud = self._pointcloud_from_ros2(
            msg,
            default_frame=ROS2_SIM_REGISTERED_CLOUD_FRAME_ID,
        )
        if cloud is not None:
            self.lidar_cloud.publish(cloud)

    def _process_map_cloud(self, msg) -> None:
        cloud = self._pointcloud_from_ros2(
            msg,
            default_frame=ROS2_SIM_LIVE_MAP_CLOUD_FRAME_ID,
        )
        if cloud is not None:
            self.map_cloud.publish(cloud)

    def _on_ros2_image(self, msg) -> None:
        """Convert ROS2 sensor_msgs/Image to Module Image."""
        try:
            encoding = getattr(msg, "encoding", "bgr8").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("bgr8", "rgb8"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)
                fmt = ImageFormat.BGR if encoding == "bgr8" else ImageFormat.RGB
            elif encoding in ("mono8", "8uc1"):
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(h, w)
                fmt = ImageFormat.GRAY
            elif encoding in ("32fc1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                fmt = ImageFormat.DEPTH_F32
            else:
                # Unknown encoding: treat as raw bytes, best effort.
                arr = np.frombuffer(raw, dtype=np.uint8)
                fmt = ImageFormat.GRAY

            self.camera_image.publish(Image(
                data=arr.copy(),
                format=fmt,
                ts=time.time(),
                frame_id=msg.header.frame_id or ROS2_SIM_CAMERA_FRAME_ID,
            ))
        except Exception as e:
            logger.error("ROS2SimDriverModule: image conversion error: %s", e)

    def _on_ros2_depth(self, msg) -> None:
        """Convert ROS2 depth image to Module Image."""
        try:
            encoding = getattr(msg, "encoding", "16uc1").lower()
            h, w = msg.height, msg.width
            raw = bytes(msg.data)

            if encoding in ("16uc1",):
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16
            elif encoding in ("32fc1",):
                arr = np.frombuffer(raw, dtype=np.float32).reshape(h, w)
                fmt = ImageFormat.DEPTH_F32
            else:
                arr = np.frombuffer(raw, dtype=np.uint16).reshape(h, w)
                fmt = ImageFormat.DEPTH_U16

            self.depth_image.publish(Image(
                data=arr.copy(),
                format=fmt,
                ts=time.time(),
                frame_id=msg.header.frame_id or ROS2_SIM_CAMERA_FRAME_ID,
            ))
        except Exception as e:
            logger.error("ROS2SimDriverModule: depth conversion error: %s", e)

    def _on_ros2_info(self, msg) -> None:
        """Convert ROS2 CameraInfo to Module CameraIntrinsics."""
        try:
            k = msg.k
            self.camera_info.publish(CameraIntrinsics(
                fx=float(k[0]),
                fy=float(k[4]),
                cx=float(k[2]),
                cy=float(k[5]),
                width=int(msg.width),
                height=int(msg.height),
                depth_scale=1.0,
            ))
        except Exception as e:
            logger.error("ROS2SimDriverModule: camera info conversion error: %s", e)

    def _on_ros2_goal(self, msg) -> None:
        """Convert ROS2 geometry_msgs/PoseStamped to the goal_pose port."""
        try:
            p = msg.pose.position
            q = msg.pose.orientation
            self.goal_pose.publish(PoseStamped(
                pose=Pose(
                    position=Vector3(float(p.x), float(p.y), float(p.z)),
                    orientation=Quaternion(float(q.x), float(q.y), float(q.z), float(q.w)),
                ),
                frame_id=msg.header.frame_id or ROS2_SIM_GOAL_FRAME_ID,
                ts=time.time(),
            ))
            logger.info("ROS2SimDriverModule: goal received (%.1f, %.1f)", p.x, p.y)
        except Exception as e:
            logger.error("ROS2SimDriverModule: goal conversion error: %s", e)

    # -- Module input callbacks ----------------------------------------------

    def _on_cmd_vel(self, twist: Twist) -> None:
        if self._stopped:
            return
        self._cmd_vx = twist.linear.x if hasattr(twist.linear, "x") else 0.0
        self._cmd_vy = twist.linear.y if hasattr(twist.linear, "y") else 0.0
        self._cmd_wz = twist.angular.z if hasattr(twist.angular, "z") else 0.0
        self._publish_cmd_vel()

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._cmd_vx = 0.0
            self._cmd_vy = 0.0
            self._cmd_wz = 0.0
            self._stopped = self._latch_stop_signal
            self._publish_cmd_vel()
            return
        self._stopped = False

    # -- ROS2 publish helpers ------------------------------------------------

    def _publish_cmd_vel(self) -> None:
        """Publish current velocity command to the canonical ROS2 cmd_vel topic."""
        if self._pub_cmd_vel is None or self._node is None:
            return
        try:
            from geometry_msgs.msg import TwistStamped

            msg = TwistStamped()
            now = time.time()
            msg.header.stamp.sec = int(now)
            msg.header.stamp.nanosec = int((now % 1.0) * 1e9)
            msg.header.frame_id = ROS2_SIM_CMD_VEL_FRAME_ID
            msg.twist.linear.x = self._cmd_vx
            msg.twist.linear.y = self._cmd_vy
            msg.twist.linear.z = 0.0
            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = self._cmd_wz
            self._pub_cmd_vel.publish(msg)
        except Exception as e:
            logger.warning("ROS2SimDriverModule: cmd_vel publish error: %s", e)

    # -- Robot state ---------------------------------------------------------

    def _publish_robot_state(self) -> None:
        """Publish ROS2 sim operational state."""
        from drivers.sim import build_sim_robot_state
        self.robot_state.publish(build_sim_robot_state())

    # -- Health --------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["ros2"] = {
            "node": self._node_name,
            "running": self._running,
            "has_node": self._node is not None,
            "spin_rate": self._spin_rate,
            "odom_topic": self._odom_topic,
            "cloud_topic": self._cloud_topic,
            "cmd_vel_topic": self._cmd_vel_topic,
            "cloud_worker_drops": self._cloud_worker_drops,
        }
        return info


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_xyz_offsets(fields) -> tuple | None:
    """Extract byte offsets for x, y, z fields from a PointCloud2 field list.

    Returns (offset_x, offset_y, offset_z) or None if any field is missing.
    """
    offsets: dict[str, int] = {}
    for field in fields:
        name = field.name.lower()
        if name in ("x", "y", "z"):
            offsets[name] = field.offset
    if len(offsets) < 3:
        return None
    return offsets["x"], offsets["y"], offsets["z"]
