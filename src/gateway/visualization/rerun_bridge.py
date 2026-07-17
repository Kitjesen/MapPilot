"""RerunBridgeModule 鈥?on-demand Rerun visualization as a Module.

Logs Module port data to the Rerun web viewer. This module is a visualization
consumer, not the product runtime communication boundary.

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
from runtime.stream import In, Out

logger = logging.getLogger("gateway.rerun_bridge_module")

# Robot body dimensions (half-sizes in meters) 鈥?Thunder quadruped
_ROBOT_HALF = [0.35, 0.155, 0.15]
_VOXEL_SIZE = 0.08


@register("visualization", "rerun", description="On-demand Rerun 3D visualization")
class RerunBridgeModule(Module, layer=6):
    """On-demand Rerun visualization bridge.

    Subscribes only to LingTu Module ports. Runtime product dataflow remains
    Gateway plus native DDS/Module boundaries.

    The Rerun server is lazy 鈥?only started when start_rerun() is called.
    """

    # Module ports (auto-wired from SLAM / Driver)
    odometry: In[Odometry]
    map_cloud: In[PointCloud2]
    goal_pose: In[PoseStamped]  # navigation goal (from GatewayModule click)

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

    def setup(self) -> None:
        """Subscribe to module streams used by the optional Rerun bridge."""
        self.odometry.subscribe(self._on_odom)
        self.map_cloud.subscribe(self._on_cloud)
        self.goal_pose.subscribe(self._on_goal_pose)

    def start(self) -> None:
        """Start the module with Rerun initially inactive."""
        super().start()
        self.rerun_active.publish(False)

    def stop(self) -> None:
        """Stop Rerun and release bridge resources."""
        self.stop_rerun()
        super().stop()

    # Rerun lifecycle

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

        return json.dumps(
            {
                "active": self._active,
                "url": f"http://localhost:{self._web_port}" if self._active else None,
                "counts": dict(self._counts),
            }
        )

    # Module port callbacks

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
            rr.log(
                "world/robot",
                rr.Boxes3D(
                    centers=[[x, y, z + _ROBOT_HALF[2]]],
                    half_sizes=[_ROBOT_HALF],
                    colors=[[0, 255, 127]],
                    fill_mode="MajorWireframe",
                ),
            )

            # Heading arrow
            yaw = odom.yaw
            dx, dy = math.cos(yaw) * 0.8, math.sin(yaw) * 0.8
            rr.log(
                "world/heading",
                rr.Arrows3D(
                    origins=[[x, y, z + 0.3]],
                    vectors=[[dx, dy, 0]],
                    colors=[[255, 255, 0]],
                    radii=0.05,
                ),
            )

            # Trajectory
            self._trajectory.append([x, y, z])
            if len(self._trajectory) > 2:
                rr.log(
                    "world/trajectory",
                    rr.LineStrips3D(
                        [self._trajectory[-1000:]],
                        colors=[[0, 100, 255]],
                    ),
                )

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

            z = xyz[:, 2]
            z_norm = np.clip((z - z.min()) / max(z.max() - z.min(), 0.01), 0.0, 1.0)
            low = z_norm < 0.5
            colors = np.zeros((len(xyz), 3), dtype=np.uint8)
            colors[low, 0] = (46 + (140 - 46) * (z_norm[low] * 2.0)).astype(np.uint8)
            colors[low, 1] = 140
            colors[low, 2] = (128 + (140 - 128) * (z_norm[low] * 2.0)).astype(np.uint8)
            hi = ~low
            hi_t = (z_norm[hi] - 0.5) * 2.0
            colors[hi, 0] = (140 + (199 - 140) * hi_t).astype(np.uint8)
            colors[hi, 1] = (140 + (153 - 140) * hi_t).astype(np.uint8)
            colors[hi, 2] = (140 + (89 - 140) * hi_t).astype(np.uint8)

            radius = max(0.01, min(float(self._voxel_size) * 0.35, 0.06))
            rr.log(
                "world/point_cloud",
                rr.Points3D(
                    xyz,
                    colors=colors,
                    radii=radius,
                ),
            )
        except Exception as e:
            logger.debug("rerun point cloud log failed: %s", e)

    def _on_goal_pose(self, goal: PoseStamped) -> None:
        """Log navigation goal marker in Rerun when a click-to-navigate is sent."""
        if not self._active or self._rr is None:
            return
        rr = self._rr
        x, y, z = goal.pose.position.x, goal.pose.position.y, goal.pose.position.z
        try:
            rr.log(
                "world/nav_goal",
                rr.Points3D(
                    [[x, y, z + 0.1]],
                    radii=[0.25],
                    colors=[[0, 255, 170]],
                    labels=["goal"],
                ),
            )
            rr.log(
                "world/nav_goal_ring",
                rr.Boxes3D(
                    centers=[[x, y, 0.02]],
                    half_sizes=[[0.3, 0.3, 0.01]],
                    colors=[[0, 255, 170, 80]],
                ),
            )
        except Exception as e:
            logger.debug("rerun goal marker log failed: %s", e)
