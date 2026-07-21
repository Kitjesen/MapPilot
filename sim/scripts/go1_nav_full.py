"""Go1 autonomous navigation 閳?3D MuJoCo demo with full Blueprint pipeline.

Professional navigation demo showing:
  - Go1 RL walking in 3D MuJoCo scene
  - LiDAR point cloud as 3D spheres in the scene (red/orange)
  - Global path as 3D cyan spheres
  - Robot trail as 3D blue spheres
  - 2D minimap overlay in bottom-right corner (occupancy + LiDAR + path)
  - INFO text panel (top-left)

Module pipeline:
  Go1SimDriverModule -> LiveMapper -> Navigation (A*)
  -> LocalPlanner -> PathFollower -> Go1SimDriverModule (closed loop)

Run on Windows (GPU, MuJoCo viewer works):
  cd D:\\inovxio\\brain\\lingtu
  set PYTHONPATH=src;.
  python sim\\scripts\\go1_nav_full.py

Output: go1_nav_full.mp4 in the repo root.
"""

from __future__ import annotations

import logging
import math
import os
import sys
import threading
import time
from typing import List, Optional

import numpy as np

# ---------------------------------------------------------------------------
# Path setup 閳?Windows local, NOT S100P
# ---------------------------------------------------------------------------

# Remove any leftover EGL override 閳?on Windows we use WGL (default)
os.environ.pop("MUJOCO_GL", None)

# Repo root is two levels up from sim/scripts/
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", ".."))

sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, REPO)

OUTPUT = os.path.join(REPO, "go1_nav_full.mp4")

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(name)s %(levelname)s %(message)s",
)
logger = logging.getLogger("go1_nav_full")

# ---------------------------------------------------------------------------
# Module framework imports
# ---------------------------------------------------------------------------

from sim.engine.core.world import ObstacleConfig

from drivers.sim.go1_sim_driver import Go1SimDriverModule
from nav.local.path_follower import PathFollower
from nav.local.terrain import Terrain
from nav.navigation import MissionState, Navigation
from nav.local.local_planner import LocalPlanner
from runtime.blueprint import Blueprint
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import PointCloud
from runtime.stream import In, Out

# ---------------------------------------------------------------------------
# LiveMapper 閳?inline Module (layer 3)
# ---------------------------------------------------------------------------


class LiveMapper(Module, layer=3):
    """200x200 occupancy grid from LiDAR + odometry.

    Publishes costmap every 3 scans.
    Sends goal once after goal_delay seconds.

    In:  lidar_in (PointCloud), odom_in (Odometry)
    Out: costmap_out (dict), goal_cmd (PoseStamped)
    """

    lidar_in: In[PointCloud]
    odom_in: In[Odometry]
    costmap_out: Out[dict]
    goal_cmd: Out[PoseStamped]

    GS = 200  # grid cells per axis
    GR = 0.2  # metres per cell

    def __init__(self, goal_xy=(6.0, 1.5), goal_delay=3.0, **kw):
        super().__init__(**kw)
        self._goal_xy = goal_xy
        self._goal_delay = goal_delay

        self._lock = threading.Lock()
        self.grid = np.zeros((self.GS, self.GS), dtype=np.float32)
        self.lidar_pts: np.ndarray = np.zeros((0, 3), dtype=np.float32)
        self.trail: list[tuple] = []
        self.scans = 0
        self.rx = 0.0
        self.ry = 0.0

        self._start_time: float | None = None
        self._goal_sent = False

    def setup(self):
        self.lidar_in.subscribe(self._on_lidar)
        self.odom_in.subscribe(self._on_odom)

    def start(self):
        super().start()
        self._start_time = time.time()

    def _on_odom(self, odom: Odometry):
        x = odom.pose.position.x
        y = odom.pose.position.y
        with self._lock:
            self.rx = x
            self.ry = y
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.15:
                self.trail.append((x, y))

        if not self._goal_sent and self._start_time is not None and time.time() - self._start_time >= self._goal_delay:
            self._goal_sent = True
            gx, gy = self._goal_xy
            goal = PoseStamped(
                pose=Pose(
                    position=Vector3(float(gx), float(gy), 0.0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id="map",
                ts=time.time(),
            )
            logger.info("LiveMapper: sending goal (%.1f, %.1f)", gx, gy)
            self.goal_cmd.publish(goal)

    def _on_lidar(self, cloud: PointCloud):
        pts = np.asarray(cloud.points, dtype=np.float32)
        with self._lock:
            rx, ry = self.rx, self.ry
            self._update_grid(pts, rx, ry)
            self.lidar_pts = pts
            self.scans += 1
            count = self.scans

        if count % 3 == 0:
            with self._lock:
                grid_copy = self.grid.copy()
                rx_snap, ry_snap = self.rx, self.ry
            self.costmap_out.publish(
                {
                    "grid": grid_copy,
                    "resolution": self.GR,
                    "origin": [-self.GS * self.GR / 2.0, -self.GS * self.GR / 2.0],
                    "rx": rx_snap,
                    "ry": ry_snap,
                }
            )

    def _update_grid(self, pts: np.ndarray, rx: float, ry: float):
        h = self.GS // 2
        for p in pts:
            d = math.hypot(float(p[0]) - rx, float(p[1]) - ry)
            if 0.3 < d < 8.0:
                gx = int(p[0] / self.GR + h)
                gy = int(p[1] / self.GR + h)
                if 0 <= gx < self.GS and 0 <= gy < self.GS:
                    self.grid[gy, gx] = min(self.grid[gy, gx] + 30, 100)

    def health(self):
        info = super().port_summary()
        info["live_mapper"] = {
            "scans": self.scans,
            "trail_len": len(self.trail),
            "goal_sent": self._goal_sent,
        }
        return info


# ---------------------------------------------------------------------------
# Scene configuration
# ---------------------------------------------------------------------------

# Corridor with two obstacles to force interesting navigation behaviour.
# The robot starts at (0,0) and the goal is at (6,1.5).
OBSTACLES = [
    ObstacleConfig("wall_L", "box", [4.0, 0.1, 0.5], [3.0, 3.5, 0.25], [0.45, 0.45, 0.55, 1.0]),
    ObstacleConfig("wall_R", "box", [4.0, 0.1, 0.5], [3.0, -0.5, 0.25], [0.45, 0.45, 0.55, 1.0]),
    ObstacleConfig("block1", "box", [0.3, 0.6, 0.4], [2.5, 1.5, 0.20], [0.70, 0.30, 0.20, 1.0]),
    ObstacleConfig("block2", "box", [0.2, 0.4, 0.4], [4.0, 2.0, 0.20], [0.70, 0.30, 0.20, 1.0]),
]

GOAL_XY = (6.0, 1.5)

# ---------------------------------------------------------------------------
# Build & start the Blueprint pipeline
# ---------------------------------------------------------------------------

logger.info("=== Go1 3D Navigation Demo 閳?LingTu Blueprint ===")

bp = Blueprint()

bp.add(
    Go1SimDriverModule,
    obstacles=OBSTACLES,
    goal_marker=(GOAL_XY[0], GOAL_XY[1], 0.15),
    start_pos=(0.0, 0.0, 0.30),
    sim_rate=50.0,
)

bp.add(LiveMapper, goal_xy=GOAL_XY, goal_delay=3.0)

bp.add(Terrain, backend="simple")
bp.add(LocalPlanner, backend="cmu_py")
bp.add(PathFollower, backend="pid", max_speed=0.4, lookahead=1.5)

bp.add(Navigation, planner="astar", waypoint_threshold=2.0, downsample_dist=1.0, stuck_timeout=12.0, max_replan_count=3)

# Closed-loop wiring
bp.wire("Go1SimDriverModule", "odometry", "nav.mission", "odometry")
bp.wire("Go1SimDriverModule", "odometry", "nav.local_planner", "odometry")
bp.wire("Go1SimDriverModule", "odometry", "nav.path_follower", "odometry")
bp.wire("Go1SimDriverModule", "odometry", "LiveMapper", "odom_in")
bp.wire("Go1SimDriverModule", "lidar_cloud", "LiveMapper", "lidar_in")
bp.wire("Go1SimDriverModule", "lidar_cloud", "nav.terrain", "map_cloud")
bp.wire("Go1SimDriverModule", "odometry", "nav.terrain", "odometry")
bp.wire("nav.terrain", "terrain_map", "nav.local_planner", "terrain_map")
bp.wire("LiveMapper", "costmap_out", "nav.mission", "costmap")
bp.wire("LiveMapper", "goal_cmd", "nav.mission", "goal_pose")
bp.wire("nav.mission", "waypoint", "nav.local_planner", "waypoint")
bp.wire("nav.local_planner", "local_path", "nav.path_follower", "local_path")
bp.wire("nav.path_follower", "cmd_vel", "Go1SimDriverModule", "cmd_vel")

system = bp.build()

# Grab module references before start (refs are stable)
driver: Go1SimDriverModule = system.get_module("Go1SimDriverModule")
mapper: LiveMapper = system.get_module("LiveMapper")
nav: Navigation = system.get_module("nav.mission")

logger.info("Starting Module pipeline (sim thread launches inside driver)...")
system.start()
logger.info("Pipeline running 閳?main thread entering render loop.")

# ---------------------------------------------------------------------------
# Renderer setup (main thread only)
# ---------------------------------------------------------------------------

import cv2
import mujoco

# Wait for driver to load the MuJoCo model (happens in setup() before start())
_t_wait = 0.0
while driver._model is None and _t_wait < 15.0:
    time.sleep(0.1)
    _t_wait += 0.1
if driver._model is None:
    logger.error("Driver model not loaded after 15s 閳?aborting.")
    system.stop()
    sys.exit(1)

model = driver._model
data = driver._data

# Output resolution
W, H = 1280, 720

# Set offscreen render resolution on the model
model.vis.global_.offwidth = W
model.vis.global_.offheight = H

renderer = mujoco.Renderer(model, H, W)

# Camera: follows robot from behind and above (isometric-ish)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [3.0, 1.5, 0.3]
cam.distance = 7.0
cam.elevation = -35.0
cam.azimuth = -50.0

# Video writer
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
FPS = 15
vid = cv2.VideoWriter(OUTPUT, fourcc, FPS, (W, H))

# Recording parameters
MAX_S = 90  # max recording duration in seconds
EXTRA_S = 4  # extra seconds after goal reached

# Pre-allocate identity rotation matrix (reused every frame for mjv_initGeom)
_EYE3 = np.eye(3, dtype=np.float64).flatten()

# ---------------------------------------------------------------------------
# 3D scene overlay helpers
# ---------------------------------------------------------------------------


def _add_sphere(scn, pos: np.ndarray, radius: float, rgba):
    """Add a single sphere geom to the mjvScene for 3D overlay."""
    if scn.ngeom >= scn.maxgeom - 1:
        return
    mujoco.mjv_initGeom(
        scn.geoms[scn.ngeom],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, 0.0, 0.0], dtype=np.float64),
        pos.astype(np.float64),
        _EYE3,
        np.array(rgba, dtype=np.float32),
    )
    scn.ngeom += 1


def _inject_3d_overlays(
    scn,
    lidar_pts: np.ndarray,  # Nx3 world-frame LiDAR hit positions
    global_path: list,  # list of np.ndarray [x,y,z] waypoints
    trail: list,  # list of (x,y) tuples 閳?robot history
    robot_z: float,  # robot Z for trail elevation
    goal: tuple,  # (x, y) goal position
    goal_z: float = 0.15,
):
    """Inject 3D visualization geoms into the MuJoCo scene.

    Draws:
      - LiDAR hits: small red/orange spheres at hit positions (every 4th point)
      - Global path: cyan spheres floating above floor
      - Robot trail: small blue spheres at past positions
      - Goal marker: large green sphere
    """
    # --- LiDAR hits (red/orange, subsampled) ---
    if lidar_pts is not None and len(lidar_pts) > 0:
        for i in range(0, len(lidar_pts), 4):
            if scn.ngeom >= scn.maxgeom - 5:
                break
            p = lidar_pts[i]
            # Colour: bright orange-red
            _add_sphere(scn, np.array([p[0], p[1], float(p[2]) + 0.05]), 0.06, [1.0, 0.45, 0.1, 0.85])

    # --- Global path (cyan spheres) ---
    if global_path and len(global_path) > 0:
        for i, wp in enumerate(global_path):
            if scn.ngeom >= scn.maxgeom - 5:
                break
            # wp is np.ndarray [x, y] or [x, y, z]
            wx = float(wp[0])
            wy = float(wp[1])
            wz = float(wp[2]) if len(wp) > 2 else 0.0
            # Colour: bright cyan, slightly transparent
            _add_sphere(scn, np.array([wx, wy, wz + 0.20]), 0.10, [0.0, 0.85, 0.95, 0.90])

    # --- Robot trail (blue spheres, last 120 points, every other) ---
    if len(trail) >= 2:
        for i in range(0, min(len(trail), 120), 2):
            if scn.ngeom >= scn.maxgeom - 5:
                break
            tx, ty = trail[i]
            _add_sphere(scn, np.array([tx, ty, robot_z + 0.05]), 0.05, [0.2, 0.5, 1.0, 0.7])

    # --- Goal marker (large bright green sphere) ---
    if scn.ngeom < scn.maxgeom - 1:
        _add_sphere(scn, np.array([float(goal[0]), float(goal[1]), goal_z + 0.25]), 0.22, [0.1, 1.0, 0.2, 0.90])


# ---------------------------------------------------------------------------
# 2D minimap overlay helper (bottom-right corner)
# ---------------------------------------------------------------------------

_MAP_PX = 260  # minimap size in pixels
_MAP_HALF = _MAP_PX // 2


def _draw_minimap(
    frame: np.ndarray,
    grid: np.ndarray,
    lidar_pts: np.ndarray,
    trail: list,
    rx: float,
    ry: float,
    goal: tuple,
    nav_path: list,
    gr: float,  # metres per cell
    gs: int,  # grid cells per axis
) -> None:
    """Draw a 2D minimap in the bottom-right corner of the frame (in-place)."""
    H_f, W_f = frame.shape[:2]
    ox = W_f - _MAP_PX - 12  # panel left edge
    oy = H_f - _MAP_PX - 12  # panel top edge

    # Dark background panel (semi-transparent)
    overlay = frame.copy()
    cv2.rectangle(overlay, (ox, oy), (ox + _MAP_PX, oy + _MAP_PX), (12, 12, 18), -1)
    cv2.addWeighted(overlay, 0.80, frame, 0.20, 0, frame)

    # Border
    cv2.rectangle(frame, (ox, oy), (ox + _MAP_PX, oy + _MAP_PX), (70, 70, 90), 1)

    # Panel label
    cv2.putText(frame, "MAP", (ox + 6, oy + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 160), 1, cv2.LINE_AA)

    def w2px(wx, wy):
        """World coord -> panel-local pixel."""
        px = int(_MAP_HALF + (wx - rx) / gr)
        py = int(_MAP_HALF - (wy - ry) / gr)
        return px, py

    def to_frame(px, py):
        return ox + px, oy + py

    # Occupied grid cells (red)
    h = gs // 2
    for cj in range(0, gs, 2):
        for ci in range(0, gs, 2):
            v = grid[cj, ci]
            if v <= 10:
                continue
            wx = (ci - h) * gr
            wy = (cj - h) * gr
            ppx, ppy = w2px(wx, wy)
            if 0 <= ppx < _MAP_PX and 0 <= ppy < _MAP_PX:
                intensity = min(255, int(v * 2.5))
                color = (intensity // 6, intensity // 10, intensity)
                cv2.circle(frame, to_frame(ppx, ppy), 2, color, -1)

    # LiDAR hits (green dots, every 3rd)
    if lidar_pts is not None and len(lidar_pts) > 0:
        for i in range(0, len(lidar_pts), 3):
            ppx, ppy = w2px(lidar_pts[i, 0], lidar_pts[i, 1])
            if 0 <= ppx < _MAP_PX and 0 <= ppy < _MAP_PX:
                cv2.circle(frame, to_frame(ppx, ppy), 1, (0, 210, 0), -1)

    # Global path (cyan line)
    if nav_path and len(nav_path) > 1:
        prev = None
        for wp in nav_path:
            ppx, ppy = w2px(float(wp[0]), float(wp[1]))
            fp = to_frame(ppx, ppy)
            if prev is not None and 0 <= ppx < _MAP_PX and 0 <= ppy < _MAP_PX:
                cv2.line(frame, prev, fp, (210, 200, 0), 1)
            prev = fp if (0 <= ppx < _MAP_PX and 0 <= ppy < _MAP_PX) else None

    # Robot trail (orange)
    if len(trail) >= 2:
        pts_px = []
        for tx, ty in trail[-200:]:
            ppx, ppy = w2px(tx, ty)
            pts_px.append((ppx, ppy))
        for i in range(1, len(pts_px)):
            x0, y0 = pts_px[i - 1]
            x1, y1 = pts_px[i]
            if 0 <= x0 < _MAP_PX and 0 <= y0 < _MAP_PX and 0 <= x1 < _MAP_PX and 0 <= y1 < _MAP_PX:
                cv2.line(frame, to_frame(x0, y0), to_frame(x1, y1), (50, 130, 230), 1)

    # Goal marker (red star)
    gpx, gpy = w2px(float(goal[0]), float(goal[1]))
    if 0 <= gpx < _MAP_PX and 0 <= gpy < _MAP_PX:
        cv2.drawMarker(frame, to_frame(gpx, gpy), (0, 0, 220), cv2.MARKER_STAR, 14, 2)

    # Robot (orange filled circle, white border)
    rpx, rpy = _MAP_HALF, _MAP_HALF  # always centred
    cv2.circle(frame, to_frame(rpx, rpy), 6, (0, 150, 255), -1)
    cv2.circle(frame, to_frame(rpx, rpy), 6, (255, 255, 255), 1)

    # Scale bar (5m)
    bar_px = int(5.0 / gr)
    bx0 = ox + 10
    by0 = oy + _MAP_PX - 10
    cv2.line(frame, (bx0, by0), (bx0 + bar_px, by0), (160, 160, 160), 1)
    cv2.line(frame, (bx0, by0 - 3), (bx0, by0 + 3), (160, 160, 160), 1)
    cv2.line(frame, (bx0 + bar_px, by0 - 3), (bx0 + bar_px, by0 + 3), (160, 160, 160), 1)
    cv2.putText(frame, "5m", (bx0 + bar_px + 3, by0 + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (160, 160, 160), 1)


# ---------------------------------------------------------------------------
# Main render loop
# ---------------------------------------------------------------------------

max_frames = MAX_S * FPS
goal_reached = False
extra_frames = 0
frame_idx = 0

t_render_start = time.time()
spf = 1.0 / FPS

logger.info("Recording up to %ds @ %d fps 閳?%s", MAX_S, FPS, OUTPUT)

while frame_idx < max_frames:
    t_frame_start = time.monotonic()
    frame_idx += 1
    elapsed = frame_idx / FPS

    if data is None or model is None:
        break

    # --- Snapshot robot state (sim thread owns qpos/qvel) ---
    try:
        rx = float(data.qpos[0])
        ry = float(data.qpos[1])
        rz = float(data.qpos[2])
    except Exception:
        rx, ry, rz = 0.0, 0.0, 0.30

    gdist = math.hypot(rx - GOAL_XY[0], ry - GOAL_XY[1])
    nav_state = nav._state
    wp_index = nav._wp_index
    wp_total = len(nav._path)

    # Snapshot mapper state (lock for consistency)
    with mapper._lock:
        grid_snap = mapper.grid.copy()
        pts_snap = mapper.lidar_pts.copy() if len(mapper.lidar_pts) > 0 else np.zeros((0, 3))
        trail_snap = list(mapper.trail)
        scans_snap = mapper.scans

    # Snapshot global path from Navigation
    nav_path_snap = list(nav._path) if nav._path else []

    # --- Camera: smoothly follow robot ---
    # Keep the goal in view: look at midpoint between robot and goal
    mid_x = (rx + GOAL_XY[0]) * 0.5
    mid_y = (ry + GOAL_XY[1]) * 0.5
    cam.lookat[:] = [mid_x, mid_y, 0.4]

    # --- Render 3D MuJoCo scene ---
    try:
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, cam)

        # Inject 3D overlays into the renderer's internal scene
        scn = renderer._scene
        _inject_3d_overlays(
            scn,
            lidar_pts=pts_snap,
            global_path=nav_path_snap,
            trail=trail_snap,
            robot_z=rz,
            goal=GOAL_XY,
        )

        rgb = renderer.render()
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except Exception as e:
        logger.warning("Render error frame %d: %s", frame_idx, e)
        frame = np.zeros((H, W, 3), dtype=np.uint8)

    # --- 2D minimap overlay (bottom-right) ---
    _draw_minimap(
        frame,
        grid_snap,
        pts_snap,
        trail_snap,
        rx,
        ry,
        GOAL_XY,
        nav_path_snap,
        gr=LiveMapper.GR,
        gs=LiveMapper.GS,
    )

    # --- Info text (top-left) ---
    occ_cells = int((grid_snap > 40).sum())
    info_lines = [
        ("Go1 Autonomous Navigation  -  LingTu", (255, 255, 255), 0.65, 2),
        ("t=%.1fs  pos=(%.2f, %.2f)  dist_goal=%.2fm" % (elapsed, rx, ry, gdist), (210, 210, 210), 0.48, 1),
        (
            "STATE: %-12s  wp=%d/%d  occ=%d  scans=%d" % (nav_state, wp_index, wp_total, occ_cells, scans_snap),
            (160, 220, 160),
            0.48,
            1,
        ),
    ]
    for i, (txt, color, scale_f, thickness) in enumerate(info_lines):
        # Drop shadow
        cv2.putText(
            frame,
            txt,
            (16 + 1, 28 + i * 24 + 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale_f,
            (0, 0, 0),
            thickness + 1,
            cv2.LINE_AA,
        )
        cv2.putText(frame, txt, (16, 28 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, scale_f, color, thickness, cv2.LINE_AA)

    # Legend (bottom-left)
    legend = [
        (" Orange spheres = LiDAR hits", (30, 130, 220)),
        (" Cyan spheres   = Global path", (215, 200, 0)),
        (" Blue spheres   = Robot trail", (220, 130, 30)),
        (" Green sphere   = Goal", (60, 220, 60)),
    ]
    for i, (txt, color) in enumerate(legend):
        y = H - 15 - (len(legend) - 1 - i) * 18
        cv2.putText(frame, txt, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA)

    # --- Goal reached banner ---
    if nav_state == MissionState.SUCCESS or gdist < 0.8:
        if not goal_reached:
            logger.info("GOAL REACHED at t=%.1fs pos=(%.2f,%.2f)", elapsed, rx, ry)
        goal_reached = True
        banner = "GOAL REACHED!"
        (tw, _), _ = cv2.getTextSize(banner, cv2.FONT_HERSHEY_SIMPLEX, 1.6, 3)
        bx = (W - tw) // 2
        by = H // 2 - 20
        cv2.putText(frame, banner, (bx + 2, by + 2), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 60, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, banner, (bx, by), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 255, 80), 3, cv2.LINE_AA)

    vid.write(frame)

    # Progress log every 5 seconds
    if frame_idx % (FPS * 5) == 0:
        logger.info(
            "t=%3.0fs  pos=(%.2f, %.2f)  dist=%.2f  state=%-12s  wp=%d/%d  scans=%d",
            elapsed,
            rx,
            ry,
            gdist,
            nav_state,
            wp_index,
            wp_total,
            scans_snap,
        )

    # Stop after extra footage when goal reached
    if goal_reached:
        extra_frames += 1
        if extra_frames >= EXTRA_S * FPS:
            logger.info("Stopping after %.0fs extra footage.", EXTRA_S)
            break

    # Throttle to FPS rate
    elapsed_frame = time.monotonic() - t_frame_start
    wait = spf - elapsed_frame
    if wait > 0.0:
        time.sleep(wait)

# ---------------------------------------------------------------------------
# Shutdown
# ---------------------------------------------------------------------------

system.stop()
renderer.close()
vid.release()

size_mb = os.path.getsize(OUTPUT) / 1024 / 1024
logger.info("=== Video saved: %s (%.1f MB) ===", OUTPUT, size_mb)
print("\n=== Video: %s (%.1f MB) ===" % (OUTPUT, size_mb))
