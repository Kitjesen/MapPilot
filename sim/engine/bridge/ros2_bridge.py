"""
Simulation ROS2 unified bridge

Converts simulation engine data into ROS2 messages so the navigation stack
is unaware it is running in simulation.

Published topics (identical to real robot):
  /camera/color/image_raw        sensor_msgs/Image
  /camera/color/camera_info      sensor_msgs/CameraInfo
  /camera/depth/image_raw        sensor_msgs/Image
  /slam/odometry                  nav_msgs/Odometry
  /slam/registered_cloud          sensor_msgs/PointCloud2
  /slam/map_cloud                 sensor_msgs/PointCloud2
  TF: map -> odom -> body -> camera_link

Subscribed topics:
  /nav/cmd_vel                   geometry_msgs/TwistStamped
"""
import threading
import time
from typing import Optional

import numpy as np

from runtime.runtime_interface import FRAMES, TOPICS


class SimROS2Bridge:
    """Bridge simulation engine data to ROS2 topics.

    engine must provide:
      engine.get_robot_state()   -> RobotState (pos, quat_wxyz, vel, omega)
      engine.get_lidar_points()  -> np.ndarray shape (N,4) xyzi, world frame
      engine.get_camera_data()   -> CameraData (rgb, depth, K)
      engine.apply_velocity(vx, vy, wz) -> None

    transport:
      When enable_shm=True, camera images are also published via SHM (~200us).
      High-speed consumers can read from SHM; ROS2 tools can read via DDS.
    """

    # Default publish rates (Hz)
    ODOM_HZ = 50
    CAMERA_HZ = 15
    CLOUD_HZ = 10

    def __init__(self, engine, node_name: str = "sim_ros2_bridge", enable_shm: bool = False):
        self._engine = engine
        self._node_name = node_name
        self._enable_shm = enable_shm
        self._lock = threading.Lock()
        self._running = False

        # cmd_vel state
        self._cmd_vx: float = 0.0
        self._cmd_vy: float = 0.0
        self._cmd_wz: float = 0.0
        self._cmd_time: float = 0.0
        self._cmd_watchdog_sec: float = 0.2

        # SHM fast channel (lazy init)
        self._shm_rgb_pub = None
        self._shm_depth_pub = None

        # ROS2 objects (lazy init)
        self._node = None
        self._odom_pub = None
        self._cloud_pub = None
        self._cloud_body_pub = None
        self._img_pub = None
        self._info_pub = None
        self._depth_pub = None
        self._tf_broadcaster = None
        self._static_tf_broadcaster = None

        # Message type references
        self._Odometry = None
        self._PointCloud2 = None
        self._PointField = None
        self._TransformStamped = None
        self._Image = None
        self._CameraInfo = None

        self._init_ros2()

    # ── Initialization ────────────────────────────────────────────────────────

    def _init_ros2(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped, TransformStamped
        from sensor_msgs.msg import PointCloud2, PointField, Image, CameraInfo
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self._node_name)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_tl = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers
        self._odom_pub = self._node.create_publisher(
            Odometry, TOPICS.odometry, qos)
        self._cloud_pub = self._node.create_publisher(
            PointCloud2, TOPICS.map_cloud, qos)
        self._cloud_body_pub = self._node.create_publisher(
            PointCloud2, TOPICS.registered_cloud, qos)
        self._img_pub = self._node.create_publisher(
            Image, TOPICS.camera_color, qos)
        self._depth_pub = self._node.create_publisher(
            Image, TOPICS.camera_depth, qos)
        self._info_pub = self._node.create_publisher(
            CameraInfo, TOPICS.camera_info, qos_tl)

        # TF broadcasters
        self._tf_broadcaster = TransformBroadcaster(self._node)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self._node)

        # Subscriber
        self._node.create_subscription(
            TwistStamped, TOPICS.cmd_vel, self._cmd_vel_cb, qos)

        # Save message type references
        self._Odometry = Odometry
        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._TransformStamped = TransformStamped
        self._Image = Image
        self._CameraInfo = CameraInfo

        # Publish static TF: map -> odom (identical in simulation)
        self._publish_static_map_odom()

        # SHM fast channel (optional, ~200us vs DDS ~1300us for 900KB image)
        if self._enable_shm:
            try:
                import sys, os
                _repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
                _src_root = os.path.join(_repo_root, 'src')
                for _p in (_repo_root, _src_root):
                    if _p not in sys.path:
                        sys.path.insert(0, _p)
                from transport.shm_transport import SHMTransport
                from transport.core import TopicConfig
                _shm = SHMTransport()
                self._shm_rgb_pub = _shm.create_publisher(
                    TopicConfig(name="/camera/color/image_raw", buffer_size=4*1024*1024))
                self._shm_depth_pub = _shm.create_publisher(
                    TopicConfig(name="/camera/depth/image_raw", buffer_size=4*1024*1024))
                self._node.get_logger().info("[SimROS2Bridge] SHM fast channel enabled (camera)")
            except Exception as e:
                self._node.get_logger().warning(f"[SimROS2Bridge] SHM init failed: {e}")
                self._enable_shm = False

        self._node.get_logger().info(
            f"[SimROS2Bridge] node '{self._node_name}' started"
            f"{' [+SHM]' if self._enable_shm else ''}")

    def _publish_static_map_odom(self):
        """Publish static TF: map -> odom (treated as same frame in simulation)."""
        tf = self._TransformStamped()
        tf.header.stamp = self._node.get_clock().now().to_msg()
        tf.header.frame_id = FRAMES.map
        tf.child_frame_id = FRAMES.odom
        tf.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform([tf])

    # ── Subscription callbacks ─────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: "TwistStamped"):
        with self._lock:
            self._cmd_vx = msg.twist.linear.x
            self._cmd_vy = msg.twist.linear.y
            self._cmd_wz = msg.twist.angular.z
            self._cmd_time = time.time()

    def _watchdog(self):
        """Zero out cmd_vel if no command received within 200ms."""
        if time.time() - self._cmd_time > self._cmd_watchdog_sec:
            with self._lock:
                self._cmd_vx = 0.0
                self._cmd_vy = 0.0
                self._cmd_wz = 0.0

    # ── Publish methods ───────────────────────────────────────────────────────

    def publish_odometry(self):
        """Publish odometry + TF: odom -> body."""
        state = self._engine.get_robot_state()
        if state is None:
            return

        now = self._node.get_clock().now().to_msg()
        pos = state.position          # (3,) xyz
        q = state.quat_wxyz           # (4,) w,x,y,z
        vel = state.linear_velocity   # (3,)
        omega = state.angular_velocity  # (3,)

        # Odometry
        odom = self._Odometry()
        odom.header.stamp = now
        odom.header.frame_id = FRAMES.odom
        odom.child_frame_id = FRAMES.body
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        # ROS quaternion: x,y,z,w
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.pose.pose.orientation.w = float(q[0])
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = float(omega[2])
        self._odom_pub.publish(odom)

        # TF: odom -> body
        tf = self._TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = FRAMES.odom
        tf.child_frame_id = FRAMES.body
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation.x = float(q[1])
        tf.transform.rotation.y = float(q[2])
        tf.transform.rotation.z = float(q[3])
        tf.transform.rotation.w = float(q[0])
        self._tf_broadcaster.sendTransform(tf)

    def publish_pointcloud(self):
        """Publish LiDAR point cloud to /slam/map_cloud and /slam/registered_cloud."""
        pts = self._engine.get_lidar_points()  # (N,4) xyzi world frame
        if pts is None or len(pts) == 0:
            return

        now = self._node.get_clock().now().to_msg()
        fields = [
            self._PointField(name="x", offset=0,  datatype=7, count=1),
            self._PointField(name="y", offset=4,  datatype=7, count=1),
            self._PointField(name="z", offset=8,  datatype=7, count=1),
            self._PointField(name="intensity", offset=12, datatype=7, count=1),
        ]
        pts_f32 = pts.astype(np.float32)

        # World frame -> /slam/map_cloud
        msg = self._PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = FRAMES.odom
        msg.height = 1
        msg.width = len(pts_f32)
        msg.is_dense = False
        msg.is_bigendian = False
        msg.fields = fields
        msg.point_step = 16
        msg.row_step = 16 * len(pts_f32)
        msg.data = pts_f32.tobytes()
        self._cloud_pub.publish(msg)

        # Body frame -> /slam/registered_cloud
        state = self._engine.get_robot_state()
        if state is not None:
            body_pos = state.position
            q = state.quat_wxyz  # w,x,y,z
            # Quaternion -> rotation matrix
            w, x, y, z = q[0], q[1], q[2], q[3]
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
                [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
            ], dtype=np.float32)
            pts_body_xyz = (pts_f32[:, :3] - body_pos.astype(np.float32)) @ R
            pts_body = np.hstack([pts_body_xyz, pts_f32[:, 3:4]])

            msg2 = self._PointCloud2()
            msg2.header.stamp = now
            msg2.header.frame_id = FRAMES.body
            msg2.height = 1
            msg2.width = len(pts_body)
            msg2.is_dense = False
            msg2.is_bigendian = False
            msg2.fields = fields
            msg2.point_step = 16
            msg2.row_step = 16 * len(pts_body)
            msg2.data = pts_body.tobytes()
            self._cloud_body_pub.publish(msg2)

    def publish_camera(self):
        """Publish camera images and camera_info."""
        cam = self._engine.get_camera_data()
        if cam is None:
            return

        now = self._node.get_clock().now().to_msg()

        # RGB image
        if cam.rgb is not None:
            h, w = cam.rgb.shape[:2]
            img_msg = self._Image()
            img_msg.header.stamp = now
            img_msg.header.frame_id = FRAMES.camera
            img_msg.height = h
            img_msg.width = w
            img_msg.encoding = "rgb8"
            img_msg.is_bigendian = False
            img_msg.step = w * 3
            rgb_bytes = cam.rgb.tobytes()
            img_msg.data = rgb_bytes
            self._img_pub.publish(img_msg)
            # SHM parallel publish (raw bytes, ~200us)
            if self._shm_rgb_pub is not None:
                self._shm_rgb_pub.publish(rgb_bytes)

        # Depth image
        if cam.depth is not None:
            h, w = cam.depth.shape[:2]
            depth_msg = self._Image()
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = FRAMES.camera
            depth_msg.height = h
            depth_msg.width = w
            depth_msg.encoding = "32FC1"
            depth_msg.is_bigendian = False
            depth_msg.step = w * 4
            depth_bytes = cam.depth.astype(np.float32).tobytes()
            depth_msg.data = depth_bytes
            self._depth_pub.publish(depth_msg)
            # SHM parallel publish
            if self._shm_depth_pub is not None:
                self._shm_depth_pub.publish(depth_bytes)

        # CameraInfo
        if cam.K is not None:
            h, w = (cam.rgb.shape[:2] if cam.rgb is not None
                    else cam.depth.shape[:2])
            info = self._CameraInfo()
            info.header.stamp = now
            info.header.frame_id = FRAMES.camera
            info.height = h
            info.width = w
            K = cam.K.flatten().tolist()
            info.k = K
            self._info_pub.publish(info)

    # ── Control loop ──────────────────────────────────────────────────────────

    def spin_once(self):
        """Process one round of ROS2 callbacks (non-blocking)."""
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def apply_control(self):
        """Send current cmd_vel to simulation engine (with watchdog)."""
        self._watchdog()
        with self._lock:
            vx, vy, wz = self._cmd_vx, self._cmd_vy, self._cmd_wz
        self._engine.apply_velocity(vx, vy, wz)

    def run(
        self,
        odom_hz: float = ODOM_HZ,
        camera_hz: float = CAMERA_HZ,
        cloud_hz: float = CLOUD_HZ,
    ):
        """Main loop: publish topics at configured rates until stop() is called."""
        self._running = True
        odom_dt = 1.0 / odom_hz
        cam_dt = 1.0 / camera_hz
        cloud_dt = 1.0 / cloud_hz

        last_odom = 0.0
        last_cam = 0.0
        last_cloud = 0.0

        while self._running:
            now = time.time()

            self.spin_once()
            self.apply_control()

            if now - last_odom >= odom_dt:
                self.publish_odometry()
                last_odom = now

            if now - last_cam >= cam_dt:
                self.publish_camera()
                last_cam = now

            if now - last_cloud >= cloud_dt:
                self.publish_pointcloud()
                last_cloud = now

            # Short sleep to avoid 100% CPU (rate-limited to fastest topic)
            time.sleep(min(odom_dt, cam_dt, cloud_dt) * 0.5)

    def stop(self):
        """Stop the main loop."""
        self._running = False

    def destroy(self):
        """Destroy the ROS2 node."""
        if self._node is not None:
            self._node.destroy_node()
