"""RerunBridgeModule 鈥?on-demand Rerun visualization as a Module.

Logs ModulePort data to the Rerun web viewer and can optionally subscribe to
ROS2 topics for visualization-only overlays. It is not the product runtime
communication boundary.

Usage in blueprint:
    bp.add(RerunBridgeModule, web_port=9090, grpc_port=9877)

Or start/stop at runtime:
    system.get_module("RerunBridgeModule").start_rerun()
    system.get_module("RerunBridgeModule").stop_rerun()
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from runtime.module import Module, skill
from runtime.msgs.nav import Odometry, PoseStamped
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS
from runtime.stream import In, Out

logger = logging.getLogger("gateway.rerun_bridge_module")

# Robot body dimensions (half-sizes in meters) 鈥?Thunder quadruped
_ROBOT_HALF = [0.35, 0.155, 0.15]
_VOXEL_SIZE = 0.08


@register("visualization", "rerun", description="On-demand Rerun 3D visualization")
class RerunBridgeModule(Module, layer=6):
    """On-demand Rerun visualization bridge.

    Subscribes to Module ports (odometry, map_cloud) and optionally to ROS2
    topics (TF, costmap, detections, camera) for visualization overlays.
    Runtime product dataflow remains Gateway + ModulePorts.

    The Rerun server is lazy 鈥?only started when start_rerun() is called.
    """

    # Module ports (auto-wired from SLAM / Driver)
    odometry:  In[Odometry]
    map_cloud: In[PointCloud2]
    goal_pose: In[PoseStamped]   # navigation goal (from GatewayModule click)

    # Output: whether rerun is active (for UI/status)
    rerun_active: Out[bool]

    def __init__(
        self,
        web_port: int = 9090,
        grpc_port: int = 9877,
        voxel_size: float = 0.08,
        max_points: int = 20000,
        cloud_throttle: int = 5,
        odom_throttle: int = 2,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._web_port = web_port
        self._grpc_port = grpc_port
        self._voxel_size = voxel_size
        self._max_points = max_points
        self._cloud_throttle = cloud_throttle
        self._odom_throttle = odom_throttle

        self._rr = None  # lazy import
        self._active = False
        self._trajectory: list = []
        self._counts = {"odom": 0, "cloud": 0}
        self._last_odom_t = 0.0

        # Optional visualization-only ROS2 node, created on start_rerun.
        self._ros2_node = None
        self._ros2_subs = []

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.subscribe(self._on_cloud)
        self.goal_pose.subscribe(self._on_goal_pose)

    def start(self) -> None:
        super().start()
        self.rerun_active.publish(False)

    def stop(self) -> None:
        self.stop_rerun()
        super().stop()

    # 鈹€鈹€ Rerun lifecycle 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    @skill
    def start_rerun(self) -> str:
        """Start Rerun web viewer. Returns URL."""
        if self._active:
            return f"already running: http://localhost:{self._web_port}"

        try:
            import rerun as rr
            self._rr = rr
            rr.init("lingtu_live")
            server_uri = rr.serve_grpc(grpc_port=self._grpc_port)
            rr.serve_web_viewer(
                open_browser=False,
                web_port=self._web_port,
                connect_to=server_uri,
            )
            self._active = True
            self.rerun_active.publish(True)

            # Start optional ROS2 subscriptions for camera/TF/costmap overlays.
            self._start_ros2_subs()

            url = f"http://localhost:{self._web_port}"
            logger.info("Rerun started: %s", url)
            return url

        except Exception as e:
            logger.error("Failed to start Rerun: %s", e)
            return f"failed: {e}"

    @skill
    def stop_rerun(self) -> str:
        """Stop Rerun visualization and release resources."""
        if not self._active:
            return "not running"

        self._stop_ros2_subs()
        self._active = False
        self._rr = None
        self._trajectory.clear()
        self.rerun_active.publish(False)
        logger.info("Rerun stopped")
        return "stopped"

    @skill
    def rerun_status(self) -> str:
        """Return Rerun status."""
        import json
        return json.dumps({
            "active": self._active,
            "url": f"http://localhost:{self._web_port}" if self._active else None,
            "counts": dict(self._counts),
        })

    # 鈹€鈹€ Module port callbacks 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_odom(self, odom: Odometry) -> None:
        self._counts["odom"] += 1
        if not self._active or self._rr is None:
            return
        if self._counts["odom"] % self._odom_throttle != 0:
            return

        rr = self._rr
        x, y, z = odom.x, odom.y, odom.z

        # Robot body 鈥?wireframe box
        try:
            rr.log("world/robot", rr.Boxes3D(
                centers=[[x, y, z + _ROBOT_HALF[2]]],
                half_sizes=[_ROBOT_HALF],
                colors=[[0, 255, 127]],
                fill_mode="MajorWireframe",
            ))

            # Heading arrow
            yaw = odom.yaw
            dx, dy = math.cos(yaw) * 0.8, math.sin(yaw) * 0.8
            rr.log("world/heading", rr.Arrows3D(
                origins=[[x, y, z + 0.3]],
                vectors=[[dx, dy, 0]],
                colors=[[255, 255, 0]],
                radii=0.05,
            ))

            # Trajectory
            self._trajectory.append([x, y, z])
            if len(self._trajectory) > 2:
                rr.log("world/trajectory", rr.LineStrips3D(
                    [self._trajectory[-1000:]],
                    colors=[[0, 100, 255]],
                ))

            # SLAM Hz
            now = time.time()
            if self._last_odom_t > 0:
                dt = now - self._last_odom_t
                if dt > 0:
                    rr.log("metrics/slam_hz", rr.Scalars(1.0 / dt))
            self._last_odom_t = now
        except Exception as e:
            logger.debug("rerun odom log failed: %s", e)

    def _on_cloud(self, cloud: PointCloud2) -> None:
        self._counts["cloud"] += 1
        if not self._active or self._rr is None:
            return
        if self._counts["cloud"] % self._cloud_throttle != 0:
            return

        rr = self._rr
        try:
            xyz = cloud.points[:, :3].astype(np.float32)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            if len(xyz) > self._max_points:
                idx = np.random.choice(len(xyz), self._max_points, replace=False)
                xyz = xyz[idx]
            if len(xyz) == 0:
                return

            # Voxelize for block rendering
            vs = self._voxel_size
            vox_idx = np.floor(xyz / vs).astype(np.int32)
            _, unique_idx = np.unique(vox_idx, axis=0, return_index=True)
            centers = (vox_idx[unique_idx].astype(np.float32) + 0.5) * vs

            # Color by height
            z = centers[:, 2]
            z_norm = np.clip((z - z.min()) / max(z.max() - z.min(), 0.01), 0, 1)
            colors = np.zeros((len(centers), 3), dtype=np.uint8)
            colors[:, 0] = (z_norm * 255).astype(np.uint8)
            colors[:, 2] = ((1 - z_norm) * 255).astype(np.uint8)
            colors[:, 1] = 80

            half = vs * 0.5
            rr.log("world/point_cloud", rr.Boxes3D(
                centers=centers,
                half_sizes=np.full((len(centers), 3), half, dtype=np.float32),
                colors=colors,
            ))
        except Exception as e:
            logger.debug("rerun point cloud log failed: %s", e)

    def _on_goal_pose(self, goal: PoseStamped) -> None:
        """Log navigation goal marker in Rerun when a click-to-navigate is sent."""
        if not self._active or self._rr is None:
            return
        rr = self._rr
        x, y, z = goal.pose.position.x, goal.pose.position.y, goal.pose.position.z
        try:
            rr.log("world/nav_goal", rr.Points3D(
                [[x, y, z + 0.1]],
                radii=[0.25],
                colors=[[0, 255, 170]],
                labels=["goal"],
            ))
            rr.log("world/nav_goal_ring", rr.Boxes3D(
                centers=[[x, y, 0.02]],
                half_sizes=[[0.3, 0.3, 0.01]],
                colors=[[0, 255, 170, 80]],
            ))
        except Exception as e:
            logger.debug("rerun goal marker log failed: %s", e)

    # 鈹€鈹€ Optional ROS2 visualization subscriptions 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _start_ros2_subs(self) -> None:
        """Subscribe to ROS2 topics for optional visualization overlays only."""
        try:
            from runtime.adapters.ros2.context import ensure_rclpy, get_shared_executor
            ensure_rclpy()
            from nav_msgs.msg import OccupancyGrid, Path
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image
            from tf2_msgs.msg import TFMessage
            from visualization_msgs.msg import MarkerArray

            qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
            qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

            self._ros2_node = Node("rerun_bridge")
            self._ros2_subs = [
                self._ros2_node.create_subscription(Image, TOPICS.camera_color, self._on_ros2_color, qos),
                self._ros2_node.create_subscription(Image, TOPICS.camera_depth, self._on_ros2_depth, qos),
                self._ros2_node.create_subscription(TFMessage, "/tf", self._on_ros2_tf, qos_be),
                self._ros2_node.create_subscription(TFMessage, "/tf_static", self._on_ros2_tf_static, qos_be),
                self._ros2_node.create_subscription(
                    OccupancyGrid, TOPICS.semantic_costmap,
                    self._on_ros2_costmap, qos_be,
                ),
                self._ros2_node.create_subscription(
                    MarkerArray, TOPICS.visualization_detections,
                    self._on_ros2_detections, qos_be,
                ),
                self._ros2_node.create_subscription(Path, TOPICS.global_path, self._on_ros2_path, qos_be),
            ]
            get_shared_executor().add_node(self._ros2_node)
            logger.info("RerunBridge: ROS2 subscriptions active")

        except ImportError:
            logger.info("RerunBridge: rclpy not available, Module-only visualization")
        except Exception as e:
            logger.warning("RerunBridge: ROS2 setup failed: %s", e)

    def _stop_ros2_subs(self) -> None:
        if self._ros2_node:
            self._ros2_node.destroy_node()
            self._ros2_node = None
        self._ros2_subs.clear()

    # 鈹€鈹€ ROS2 camera callbacks 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    _color_count = 0
    _depth_count = 0

    @staticmethod
    def _crop_square(img: np.ndarray) -> np.ndarray:
        h, w = img.shape[:2]
        if h > w:
            margin = (h - w) // 2
            return img[margin:margin + w]
        return img

    def _on_ros2_color(self, msg) -> None:
        self._color_count += 1
        if not self._active or self._rr is None:
            return
        if self._color_count % 6 != 0:
            return
        try:
            h, w = msg.height, msg.width
            enc = msg.encoding.lower()
            if enc in ("bgr8", "rgb8"):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                if enc == "bgr8":
                    img = img[:, :, ::-1]
                img = self._crop_square(np.rot90(img, k=1))
                self._rr.log("camera/color", self._rr.Image(img))
        except Exception as e:
            logger.debug("rerun color frame log failed: %s", e)

    def _on_ros2_depth(self, msg) -> None:
        self._depth_count += 1
        if not self._active or self._rr is None:
            return
        if self._depth_count % 5 != 0:
            return
        try:
            h, w = msg.height, msg.width
            enc = msg.encoding.lower()
            if enc == "16uc1":
                img = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
                img = self._crop_square(np.rot90(img, k=1))
                self._rr.log("camera/depth", self._rr.DepthImage(img, meter=1000.0))
            elif enc == "32fc1":
                img = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
                img = self._crop_square(np.rot90(img, k=1))
                self._rr.log("camera/depth", self._rr.DepthImage(img, meter=1.0))
        except Exception as e:
            logger.debug("rerun depth frame log failed: %s", e)

    # 鈹€鈹€ ROS2 TF callback 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_ros2_tf(self, msg) -> None:
        if not self._active or self._rr is None:
            return
        rr = self._rr
        try:
            for tf in msg.transforms:
                child = tf.child_frame_id.lstrip("/")
                t = tf.transform.translation
                q = tf.transform.rotation
                rr.log(f"world/tf/{child}", rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                ))
        except Exception as e:
            logger.debug("rerun TF log failed: %s", e)

    def _on_ros2_tf_static(self, msg) -> None:
        if not self._active or self._rr is None:
            return
        rr = self._rr
        try:
            for tf in msg.transforms:
                child = tf.child_frame_id.lstrip("/")
                t = tf.transform.translation
                q = tf.transform.rotation
                rr.log(f"world/tf/{child}", rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
                ), static=True)
        except Exception as e:
            logger.debug("rerun TF static log failed: %s", e)

    # 鈹€鈹€ ROS2 costmap callback 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    _costmap_count = 0

    def _on_ros2_costmap(self, msg) -> None:
        self._costmap_count += 1
        if not self._active or self._rr is None:
            return
        if self._costmap_count % 5 != 0:
            return
        try:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            grid = np.array(msg.data, dtype=np.int8).reshape(h, w)
            img = np.zeros((h, w, 3), dtype=np.uint8)
            img[grid == 0] = [40, 80, 40]
            img[grid > 50] = [200, 50, 50]
            img[grid < 0] = [60, 60, 60]
            img[(grid > 0) & (grid <= 50)] = [180, 160, 40]

            x0, y0 = ox, oy
            x1, y1 = ox + w * res, oy + h * res
            vertices = np.array([
                [x0, y0, 0.01], [x1, y0, 0.01],
                [x1, y1, 0.01], [x0, y1, 0.01],
            ], dtype=np.float32)
            self._rr.log("world/costmap", self._rr.Mesh3D(
                vertex_positions=vertices,
                triangle_indices=np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32),
                vertex_texcoords=np.array([[0, 1], [1, 1], [1, 0], [0, 0]], dtype=np.float32),
                albedo_texture=img,
            ))
        except Exception as e:
            logger.debug("rerun costmap log failed: %s", e)

    # 鈹€鈹€ ROS2 detections callback 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_ros2_detections(self, msg) -> None:
        if not self._active or self._rr is None:
            return
        try:
            positions, labels, colors = [], [], []
            for m in msg.markers:
                if m.action == 2:
                    continue
                px, py, pz = m.pose.position.x, m.pose.position.y, m.pose.position.z
                if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                    continue
                positions.append([px, py, pz])
                labels.append(m.text or f"obj_{m.id}")
                r = int(m.color.r * 255) if m.color.r <= 1.0 else int(m.color.r)
                g = int(m.color.g * 255) if m.color.g <= 1.0 else int(m.color.g)
                b = int(m.color.b * 255) if m.color.b <= 1.0 else int(m.color.b)
                colors.append([max(r, 50), max(g, 50), max(b, 50)])

            if positions:
                self._rr.log("world/detections", self._rr.Points3D(
                    positions, labels=labels, colors=colors, radii=0.12,
                ))
        except Exception as e:
            logger.debug("rerun detections log failed: %s", e)

    # 鈹€鈹€ ROS2 path callback 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

    def _on_ros2_path(self, msg) -> None:
        if not self._active or self._rr is None:
            return
        try:
            pts = [[ps.pose.position.x, ps.pose.position.y, ps.pose.position.z]
                   for ps in msg.poses]
            if len(pts) > 1:
                self._rr.log("world/nav_path", self._rr.LineStrips3D(
                    [pts], colors=[[0, 255, 100]], radii=0.04,
                ))
        except Exception as e:
            logger.debug("rerun nav path log failed: %s", e)
