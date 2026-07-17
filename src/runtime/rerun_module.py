"""RerunModule — 3D visualization bridge for LingTu navigation.

Subscribes to key data streams and logs them to Rerun viewer.
Open browser at http://localhost:9090 to see real-time 3D navigation.

Usage::

    system = autoconnect(
        ...,
        RerunModule.blueprint(app_name="lingtu"),
    ).build()

Or standalone:
    python -m runtime.rerun_module  # starts viewer + waits for data
"""

from __future__ import annotations

import logging
import time
from typing import Any

import numpy as np

from runtime.module import Module
from runtime.msgs.geometry import PoseStamped, Twist
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.semantic import SafetyState, SceneGraph
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.stream import In

logger = logging.getLogger(__name__)

try:
    import rerun as rr

    _RERUN_AVAILABLE = True
except ImportError:
    _RERUN_AVAILABLE = False
    rr = None


def _height_colors(xyz: np.ndarray) -> np.ndarray:
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
    return colors


@register("visualization", "rerun", description="Rerun 3D visualization bridge")
class RerunModule(Module, layer=6):
    """Rerun 3D visualization — logs navigation data to browser viewer.

    Subscribes to odometry, path, scene graph, safety state.
    Logs to Rerun with rate limiting for heavy messages.

    Open http://localhost:9090 in browser to view.
    """

    # Subscribe to everything worth visualizing
    odometry: In[Odometry]
    global_path: In[Path]
    scene_graph: In[SceneGraph]
    safety_state: In[SafetyState]
    waypoint: In[PoseStamped]
    cmd_vel: In[Twist]
    map_cloud: In[PointCloud2]

    def __init__(
        self,
        app_name: str = "lingtu",
        web_port: int = 9090,
        max_image_fps: float = 5.0,
        max_cloud_fps: float = 2.0,
        max_cloud_points: int = 20000,
        cloud_point_radius: float = 0.035,
        **kw,
    ):
        super().__init__(**kw)
        self._app_name = app_name
        self._web_port = web_port
        self._max_image_fps = max_image_fps
        self._max_cloud_fps = max_cloud_fps
        self._max_cloud_points = max(1, int(max_cloud_points))
        self._cloud_point_radius = max(0.001, float(cloud_point_radius))
        self._trajectory: list = []
        self._max_trajectory = 5000
        self._last_odom_log = 0.0
        self._odom_min_interval = 0.05  # 20Hz max
        self._last_cloud_log = 0.0

    def setup(self) -> None:
        """Subscribe to visualization streams and start the Rerun viewer."""
        if not _RERUN_AVAILABLE:
            logger.error("Rerun not installed. Run: pip install rerun-sdk")
            return

        rr.init(self._app_name, spawn=False)
        try:
            rr.serve_web(open_browser=False, web_port=self._web_port)
            logger.info("Rerun viewer at http://localhost:%d", self._web_port)
        except Exception:
            logger.warning("Rerun serve_web failed, trying connect")
            try:
                rr.connect()
            except Exception:
                logger.error("Rerun connection failed")
                return

        # Set up coordinate system
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

        self.odometry.subscribe(self._on_odom)
        self.global_path.subscribe(self._on_path)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.safety_state.subscribe(self._on_safety)
        self.waypoint.subscribe(self._on_waypoint)
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.map_cloud.subscribe(self._on_cloud)

    # -- Odometry → robot pose + trajectory ----------------------------------

    def _on_odom(self, odom: Odometry):
        now = time.time()
        if now - self._last_odom_log < self._odom_min_interval:
            return
        self._last_odom_log = now

        pos = np.array([odom.x, odom.y, getattr(odom, "z", 0.35)])

        # Robot position
        rr.log("world/robot/position", rr.Points3D([pos], colors=[[0, 255, 0]], radii=[0.15]))

        # Trajectory trail
        self._trajectory.append(pos.tolist())
        if len(self._trajectory) > self._max_trajectory:
            self._trajectory = self._trajectory[-self._max_trajectory :]
        if len(self._trajectory) >= 2:
            rr.log("world/robot/trajectory", rr.LineStrips3D([self._trajectory], colors=[[0, 200, 0]]))

        # Velocity arrow
        vx = odom.vx if hasattr(odom, "vx") else 0.0
        vy = odom.vy if hasattr(odom, "vy") else 0.0
        speed = (vx**2 + vy**2) ** 0.5
        if speed > 0.01:
            end = pos + np.array([vx, vy, 0.0]) * 2.0
            rr.log("world/robot/velocity", rr.Arrows3D(origins=[pos], vectors=[end - pos], colors=[[255, 255, 0]]))

    # -- Global path ---------------------------------------------------------

    def _on_path(self, path: Path | list):
        if not path:
            return
        pts = []
        for p in getattr(path, "poses", path):
            if hasattr(p, "x") and hasattr(p, "y"):
                pts.append(
                    [
                        float(getattr(p, "x", 0.0)),
                        float(getattr(p, "y", 0.0)),
                        float(getattr(p, "z", 0.3)),
                    ]
                )
            elif isinstance(p, (list, tuple, np.ndarray)) and len(p) >= 2:
                pts.append([float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.3])
        if len(pts) >= 2:
            rr.log("world/navigation/path", rr.LineStrips3D([pts], colors=[[0, 150, 255]], radii=[0.05]))

    # -- Point cloud ---------------------------------------------------------

    def _on_cloud(self, cloud: PointCloud2):
        now = time.time()
        if now - self._last_cloud_log < 1.0 / max(self._max_cloud_fps, 0.1):
            return
        self._last_cloud_log = now

        xyz = np.asarray(cloud.points[:, :3], dtype=np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if len(xyz) == 0:
            return
        if len(xyz) > self._max_cloud_points:
            step = max(1, int(np.ceil(len(xyz) / self._max_cloud_points)))
            xyz = xyz[::step][: self._max_cloud_points]

        rr.log(
            "world/map/points",
            rr.Points3D(
                xyz,
                colors=_height_colors(xyz),
                radii=self._cloud_point_radius,
            ),
        )

    # -- Scene graph objects --------------------------------------------------

    def _on_scene_graph(self, sg: SceneGraph):
        sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
        try:
            import json

            data = json.loads(sg_json) if isinstance(sg_json, str) else sg_json
            objects = data.get("objects", [])
            if not objects:
                return

            positions = []
            labels = []
            colors = []
            for obj in objects:
                pos = obj.get("position")
                if pos is None:
                    continue
                if isinstance(pos, dict):
                    positions.append([pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)])
                elif isinstance(pos, (list, tuple)):
                    positions.append([float(pos[0]), float(pos[1]), float(pos[2]) if len(pos) > 2 else 0.5])
                labels.append(obj.get("label", "?"))
                conf = obj.get("confidence", 0.5)
                r = int(255 * (1 - conf))
                g = int(255 * conf)
                colors.append([r, g, 50])

            if positions:
                rr.log("world/scene_graph/objects", rr.Points3D(positions, colors=colors, radii=[0.1], labels=labels))
        except Exception as exc:
            logger.debug("Rerun scene graph log failed: %s", exc)

    # -- Safety state ---------------------------------------------------------

    def _on_safety(self, state: SafetyState):
        level = state.level if hasattr(state, "level") else 0
        label = ["SAFE", "WARN", "STOP"][min(level, 2)]
        rr.log(
            "world/safety",
            rr.TextLog(f"Safety: {label}", level=rr.TextLogLevel.WARN if level > 0 else rr.TextLogLevel.INFO),
        )

    # -- Waypoint -------------------------------------------------------------

    def _on_waypoint(self, wp: PoseStamped):
        pos = [wp.pose.position.x, wp.pose.position.y, wp.pose.position.z]
        rr.log(
            "world/navigation/waypoint", rr.Points3D([pos], colors=[[255, 100, 0]], radii=[0.2], labels=["waypoint"])
        )

    # -- Cmd vel --------------------------------------------------------------

    def _on_cmd_vel(self, twist: Twist):
        rr.log("world/robot/cmd_vel", rr.Scalar(twist.linear.x))

    # -- Health ---------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        """Return Rerun visualization health and trajectory cache size."""
        info = super().port_summary()
        info["rerun"] = {
            "available": _RERUN_AVAILABLE,
            "web_port": self._web_port,
            "trajectory_len": len(self._trajectory),
        }
        return info

    def stop(self):
        """Disconnect Rerun before stopping the module."""
        if _RERUN_AVAILABLE:
            try:
                rr.disconnect()
            except Exception as exc:
                logger.debug("Rerun disconnect failed: %s", exc)
        super().stop()
