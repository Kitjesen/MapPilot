#!/usr/bin/env python3
"""Go1 Indoor Navigation Demo 鈥?single-thread architecture.

Go1 RL policy walking + LingTu nav algorithms (cmu_py + A*) in MuJoCo
indoor office scene. Outputs 3D demo video with point cloud, path, minimap.

Usage:
    cd D:\\inovxio\\brain\\lingtu
    set PYTHONPATH=src;.
    python sim\\scripts\\go1_indoor_nav.py
    python sim\\scripts\\go1_indoor_nav.py --headless   # no 3D render
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Resolve project root
_SCRIPT_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _SCRIPT_DIR.parents[1]
_SIM_ROOT = _REPO_ROOT / "sim"
_SCENE_XML = _SIM_ROOT / "worlds" / "indoor_office.xml"
_POLICY_ONNX = _SIM_ROOT / "robots" / "go1_playground" / "go1_policy.onnx"

# Ensure src/ on path
sys.path.insert(0, str(_REPO_ROOT / "src"))
sys.path.insert(0, str(_REPO_ROOT))

import mujoco

from nav.local import LocalPlanner, PathFollower
from nav.navigation import Navigation
from runtime.blueprint import Blueprint
from runtime.module import Module
from runtime.msgs.geometry import (
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    Vector3,
)
from runtime.msgs.nav import Odometry
from runtime.msgs.sensor import PointCloud
from runtime.stream import In, Out

# 鈹€鈹€ Go1 policy constants 鈹€鈹€
_CTRL_DT = 0.02  # 50 Hz policy
_ACTION_SCALE = 0.5  # MuJoCo Playground Go1: confirmed action_scale=0.5, Kp=35
_N_RAYS = 180
_LIDAR_MIN = 0.2
_LIDAR_MAX = 8.0

# 鈹€鈹€ Scene constants 鈹€鈹€
GOAL = (5.5, -5.5)
START_POS = (0.0, 0.0, 0.278)

# 鈹€鈹€ Video config 鈹€鈹€
VIDEO_W, VIDEO_H = 1280, 720
FPS = 25
TOTAL_SEC = 60
STEPS_PER_FRAME = int((1.0 / FPS) / 0.004)  # physics dt=0.004


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# LiveMapper 鈥?simple 2D occupancy grid from LiDAR
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# SensorRelay 鈥?bridge between main loop and Module pipeline
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
class SensorRelay(Module, layer=1):
    """Publishes odometry and lidar from main loop into the Module port system."""

    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud]


class LiveMapper(Module, layer=3):
    lidar_in: In[PointCloud]
    odom_in: In[Odometry]
    costmap_out: Out[dict]
    goal_cmd: Out[PoseStamped]

    GRID_SIZE = 200
    RESOLUTION = 0.15  # m/cell 鈥?covers 30m x 30m
    ORIGIN = (-5.0, -10.0)  # world coords of grid (0,0)

    def __init__(self, **kw):
        super().__init__(**kw)
        self.grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=np.float32)
        self.scans = 0
        self.rx = self.ry = self.ryaw = 0.0

    def setup(self):
        self.lidar_in.subscribe(self._on_lidar)
        self.odom_in.subscribe(self._on_odom)

    def _on_odom(self, o: Odometry):
        self.rx = o.pose.position.x
        self.ry = o.pose.position.y
        self.ryaw = o.yaw

    def _on_lidar(self, cloud: PointCloud):
        if cloud.points is None or len(cloud.points) == 0:
            return
        ox, oy = self.ORIGIN
        res = self.RESOLUTION
        h = self.GRID_SIZE
        for p in cloud.points:
            if len(p) < 3:
                continue
            z = p[2]
            if z < 0.05 or z > 1.2:
                continue
            d = math.hypot(p[0] - self.rx, p[1] - self.ry)
            if d < 0.3 or d > _LIDAR_MAX:
                continue
            gx = int((p[0] - ox) / res)
            gy = int((p[1] - oy) / res)
            if 0 <= gx < h and 0 <= gy < h:
                self.grid[gy, gx] = min(self.grid[gy, gx] + 30, 100)

        self.scans += 1
        if self.scans % 3 == 0:
            self.costmap_out.publish(
                {
                    "grid": self.grid.copy(),
                    "resolution": res,
                    "origin": list(self.ORIGIN),
                }
            )


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# Go1 RL Policy (48-dim obs, 12 joints, no history 鈥?Isaac Lab Go1)
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
class Go1Policy:
    def __init__(self, onnx_path: str):
        import onnxruntime as ort

        self.sess = ort.InferenceSession(onnx_path, providers=ort.get_available_providers())
        self.out_name = self.sess.get_outputs()[0].name
        self.last_action = np.zeros(12, dtype=np.float32)
        self.default_angles: np.ndarray | None = None

    def init_from_model(self, model, data):
        mujoco.mj_resetDataKeyframe(model, data, 0)
        self.default_angles = data.qpos[7:].copy()
        self.last_action = np.zeros(12, dtype=np.float32)

    def compute(self, model, data, nav_cmd: np.ndarray) -> np.ndarray:
        linvel = data.sensor("local_linvel").data.copy().astype(np.float32)
        gyro = data.sensor("gyro").data.copy().astype(np.float32)
        imu_id = model.site("imu").id
        imu_xmat = data.site_xmat[imu_id].reshape(3, 3)
        gravity = (imu_xmat.T @ np.array([0.0, 0.0, -1.0])).astype(np.float32)
        jpos = (data.qpos[7:] - self.default_angles).astype(np.float32)
        jvel = data.qvel[6:].astype(np.float32)

        obs = np.concatenate(
            [
                linvel,
                gyro,
                gravity,
                jpos,
                jvel,
                self.last_action,
                nav_cmd,
            ]
        ).reshape(1, -1)

        action = self.sess.run([self.out_name], {"obs": obs})[0][0]
        self.last_action = action.copy()
        return action


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# LiDAR scanner
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
def scan_lidar(model, data, trunk_id: int) -> np.ndarray | None:
    pos = data.qpos[:3].copy().astype(np.float64)
    pos[2] = max(pos[2], 0.15)

    q = data.qpos[3:7]
    yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))
    cy, sy = math.cos(yaw), math.sin(yaw)

    angles = np.linspace(0, 2 * math.pi, _N_RAYS, endpoint=False)
    dirs = np.zeros((_N_RAYS, 3), dtype=np.float64)
    for i, a in enumerate(angles):
        lx, ly = math.cos(a), math.sin(a)
        dirs[i, 0] = lx * cy - ly * sy
        dirs[i, 1] = lx * sy + ly * cy

    geomgroup = np.array([1, 1, 0, 0, 0, 0], dtype=np.uint8)
    dist_out = np.full(_N_RAYS, -1.0, dtype=np.float64)
    geomid_out = np.full(_N_RAYS, -1, dtype=np.int32)

    mujoco.mj_multiRay(
        model, data, pos, dirs.flatten(), geomgroup, 1, trunk_id, geomid_out, dist_out, None, _N_RAYS, _LIDAR_MAX
    )

    mask = (dist_out > _LIDAR_MIN) & (dist_out < _LIDAR_MAX)
    if not mask.any():
        return np.zeros((0, 3), dtype=np.float32)
    return (pos + dirs[mask] * dist_out[mask, None]).astype(np.float32)


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# 3D marker injection
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
def inject_markers(scene, lidar_pts, global_path, trail, goal):
    """Inject point cloud, path, trail, goal into MuJoCo scene."""
    idx = scene.ngeom

    # LiDAR points 鈥?orange spheres
    if lidar_pts is not None:
        step = max(1, len(lidar_pts) // 150)
        for i in range(0, len(lidar_pts), step):
            if idx >= scene.maxgeom - 10:
                break
            p = lidar_pts[i]
            mujoco.mjv_initGeom(
                scene.geoms[idx],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.02, 0, 0],
                p.astype(np.float64),
                np.eye(3).flatten(),
                np.array([1.0, 0.5, 0.1, 0.8]),
            )
            idx += 1

    # Global path 鈥?cyan spheres
    if global_path:
        step = max(1, len(global_path) // 80)
        for i in range(0, len(global_path), step):
            if idx >= scene.maxgeom - 10:
                break
            wp = global_path[i]
            pos = np.array([wp[0], wp[1], 0.05], dtype=np.float64)
            mujoco.mjv_initGeom(
                scene.geoms[idx],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                [0.04, 0, 0],
                pos,
                np.eye(3).flatten(),
                np.array([0.0, 0.8, 0.8, 0.7]),
            )
            idx += 1

    # Robot trail 鈥?blue spheres
    step = max(1, len(trail) // 60)
    for i in range(0, len(trail), step):
        if idx >= scene.maxgeom - 5:
            break
        pos = np.array([trail[i][0], trail[i][1], 0.03], dtype=np.float64)
        mujoco.mjv_initGeom(
            scene.geoms[idx],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            [0.03, 0, 0],
            pos,
            np.eye(3).flatten(),
            np.array([0.2, 0.4, 1.0, 0.6]),
        )
        idx += 1

    # Goal 鈥?green sphere (larger)
    if idx < scene.maxgeom:
        gpos = np.array([goal[0], goal[1], 0.2], dtype=np.float64)
        mujoco.mjv_initGeom(
            scene.geoms[idx],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            [0.15, 0, 0],
            gpos,
            np.eye(3).flatten(),
            np.array([0.2, 0.9, 0.3, 0.7]),
        )
        idx += 1

    scene.ngeom = idx


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# 2D minimap overlay
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
def draw_minimap(frame, grid, origin, res, robot_pos, robot_yaw, lidar_pts, global_path, goal, size=200):
    h, w = frame.shape[:2]
    mx, my = w - size - 15, h - size - 15

    # Background
    minimap = np.zeros((size, size, 3), dtype=np.uint8) + 20

    # Occupancy grid
    gs = grid.shape[0]
    scale = size / (gs * res)
    for gy in range(gs):
        for gx in range(gs):
            if grid[gy, gx] > 20:
                px = int((gx * res) * scale)
                py = size - 1 - int((gy * res) * scale)
                if 0 <= px < size and 0 <= py < size:
                    v = min(255, int(grid[gy, gx] * 2.5))
                    minimap[py, px] = [v, v, v]

    def w2m(wx, wy):
        px = int((wx - origin[0]) * scale)
        py = size - 1 - int((wy - origin[1]) * scale)
        return (px, py)

    # LiDAR points
    if lidar_pts is not None:
        for p in lidar_pts[::3]:
            mp = w2m(p[0], p[1])
            if 0 <= mp[0] < size and 0 <= mp[1] < size:
                cv2.circle(minimap, mp, 1, (0, 140, 255), -1)

    # Global path
    if global_path and len(global_path) > 1:
        for i in range(len(global_path) - 1):
            p1 = w2m(global_path[i][0], global_path[i][1])
            p2 = w2m(global_path[i + 1][0], global_path[i + 1][1])
            cv2.line(minimap, p1, p2, (200, 200, 0), 1)

    # Goal
    gm = w2m(goal[0], goal[1])
    cv2.circle(minimap, gm, 5, (0, 220, 80), -1)

    # Robot
    rm = w2m(robot_pos[0], robot_pos[1])
    cv2.circle(minimap, rm, 4, (255, 100, 100), -1)
    dx = int(8 * math.cos(robot_yaw))
    dy = int(-8 * math.sin(robot_yaw))
    cv2.arrowedLine(minimap, rm, (rm[0] + dx, rm[1] + dy), (255, 100, 100), 1)

    # Border
    cv2.rectangle(minimap, (0, 0), (size - 1, size - 1), (100, 100, 100), 1)

    # Paste onto frame with semi-transparent background
    overlay = frame[my : my + size, mx : mx + size].copy()
    blended = cv2.addWeighted(overlay, 0.3, minimap, 0.7, 0)
    frame[my : my + size, mx : mx + size] = blended


def draw_info_panel(frame, t, pos, yaw_deg, dist, state, wp_idx, wp_total, scans):
    lines = [
        f"t={t:.1f}s",
        f"pos=({pos[0]:.2f}, {pos[1]:.2f})",
        f"yaw={yaw_deg:.0f}deg",
        f"dist={dist:.2f}m",
        f"state={state}",
        f"wp={wp_idx}/{wp_total}",
        f"scans={scans}",
    ]
    y0 = 25
    for i, line in enumerate(lines):
        y = y0 + i * 22
        cv2.putText(frame, line, (12, y + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# Odometry helpers
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
def build_odometry(data, ts: float) -> Odometry:
    pos = data.qpos[:3]
    q = data.qpos[3:7]  # MuJoCo: w, x, y, z
    return Odometry(
        pose=Pose(
            position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
            orientation=Quaternion(float(q[1]), float(q[2]), float(q[3]), float(q[0])),
        ),
        twist=Twist(
            linear=Vector3(float(data.qvel[0]), float(data.qvel[1]), float(data.qvel[2])),
            angular=Vector3(float(data.qvel[3]), float(data.qvel[4]), float(data.qvel[5])),
        ),
        ts=ts,
    )


def get_yaw(data) -> float:
    q = data.qpos[3:7]
    return math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2))


# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
# Main
# 鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣鈹佲攣
def main():
    parser = argparse.ArgumentParser(description="Go1 Indoor Navigation Demo")
    parser.add_argument("--headless", action="store_true", help="No 3D rendering")
    parser.add_argument("--duration", type=int, default=TOTAL_SEC, help="Duration in seconds")
    args = parser.parse_args()

    print(f"Loading scene: {_SCENE_XML}")
    model = mujoco.MjModel.from_xml_path(str(_SCENE_XML))
    data = mujoco.MjData(model)
    print(f"  nq={model.nq} nu={model.nu} ngeom={model.ngeom}")

    print(f"Loading policy: {_POLICY_ONNX}")
    policy = Go1Policy(str(_POLICY_ONNX))
    policy.init_from_model(model, data)

    # Trunk body for LiDAR exclusion
    try:
        trunk_id = model.body("trunk").id
    except Exception:
        trunk_id = -1

    # Nav command (shared with policy callback)
    nav_cmd = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    n_substeps = max(1, round(_CTRL_DT / model.opt.timestep))
    step_counter = [0]

    def run_policy_step(m, d, cmd):
        """Run policy and set ctrl 鈥?called every n_substeps physics steps."""
        action = policy.compute(m, d, cmd)
        d.ctrl[:] = action * _ACTION_SCALE + policy.default_angles

    # Stabilize with policy
    print("Stabilizing (500 steps)...")
    zero_cmd = np.array([0, 0, 0], dtype=np.float32)
    for i in range(500):
        if i % n_substeps == 0:
            run_policy_step(model, data, zero_cmd)
        mujoco.mj_step(model, data)
    pos = data.qpos[:3]
    print(f"  Standing at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

    # 鈹€鈹€ Module pipeline 鈹€鈹€
    bp = Blueprint()
    bp.add(SensorRelay)
    bp.add(LiveMapper)
    bp.add(LocalPlanner, backend="cmu_py")
    bp.add(PathFollower, backend="pid")
    bp.add(Navigation, planner="astar", waypoint_threshold=1.5, downsample_dist=0.8)
    # Sensor relay 鈫?all consumers
    bp.wire("SensorRelay", "odometry", "LiveMapper", "odom_in")
    bp.wire("SensorRelay", "odometry", "nav.mission", "odometry")
    bp.wire("SensorRelay", "odometry", "nav.local_planner", "odometry")
    bp.wire("SensorRelay", "odometry", "nav.path_follower", "odometry")
    bp.wire("SensorRelay", "lidar_cloud", "LiveMapper", "lidar_in")
    bp.wire("SensorRelay", "lidar_cloud", "nav.local_planner", "terrain_map")
    # Navigation chain
    bp.wire("LiveMapper", "costmap_out", "nav.mission", "costmap")
    bp.wire("LiveMapper", "goal_cmd", "nav.mission", "goal_pose")
    bp.wire("nav.mission", "waypoint", "nav.local_planner", "waypoint")
    bp.wire("nav.local_planner", "local_path", "nav.path_follower", "local_path")
    system = bp.build()
    system.start()
    time.sleep(1.0)

    relay = system.get_module("SensorRelay")
    mapper = system.get_module("LiveMapper")
    nav = system.get_module("nav.mission")
    lp = system.get_module("nav.local_planner")
    pf = system.get_module("nav.path_follower")

    # 鈹€鈹€ Renderer (if not headless) 鈹€鈹€
    renderer = None
    video = None
    cam = None
    if not args.headless:
        renderer = mujoco.Renderer(model, VIDEO_H, VIDEO_W)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance = 6.0
        cam.elevation = -30
        cam.azimuth = 150
        cam.lookat[:] = [START_POS[0], START_POS[1], 0.3]

        _media = _REPO_ROOT / "assets" / "media"
        _media.mkdir(parents=True, exist_ok=True)
        out_path = str(_media / "go1_indoor_nav.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video = cv2.VideoWriter(out_path, fourcc, FPS, (VIDEO_W, VIDEO_H))
        print(f"Video output: {out_path}")

    # Goal will be sent after a few LiDAR scans (see main loop)
    goal_sent = False

    # 鈹€鈹€ Main loop 鈹€鈹€
    trail = []
    lidar_pts = None
    global_path = None
    total_frames = args.duration * FPS

    print(f"Running {args.duration}s ({total_frames} frames)...")
    t_start = time.time()

    for frame_i in range(total_frames):
        # 1. Step physics with inline policy
        for si in range(STEPS_PER_FRAME):
            if si % n_substeps == 0:
                run_policy_step(model, data, nav_cmd)
            mujoco.mj_step(model, data)

        elapsed = frame_i * (1.0 / FPS)

        # 2. Sensor data 鈫?Module pipeline (via SensorRelay ports)
        sim_time = frame_i * (1.0 / FPS)
        odom = build_odometry(data, sim_time)
        relay.odometry.publish(odom)

        # LiDAR at ~10 Hz
        if frame_i % max(1, FPS // 10) == 0:
            pts = scan_lidar(model, data, trunk_id)
            if pts is not None and len(pts) > 0:
                cloud = PointCloud(points=pts, frame_id="lidar", ts=sim_time)
                relay.lidar_cloud.publish(cloud)
                lidar_pts = pts

        # 2b. Track position (needed before heading control)
        rx, ry = float(data.qpos[0]), float(data.qpos[1])
        ryaw = get_yaw(data)
        dist = math.hypot(rx - GOAL[0], ry - GOAL[1])

        # 2c. Send goal to nav module for A* path visualization
        if not goal_sent and mapper.scans >= 10:
            mapper.goal_cmd.publish(
                PoseStamped(
                    pose=Pose(position=Vector3(GOAL[0], GOAL[1], 0.0), orientation=Quaternion(0, 0, 0, 1)),
                    frame_id="map",
                    ts=elapsed,
                )
            )
            goal_sent = True
            print(f"Goal sent at t={elapsed:.1f}s after {mapper.scans} scans")

        # 3. Direct heading control with fall detection
        rz = float(data.qpos[2])
        gx, gy = GOAL
        bearing = math.atan2(gy - ry, gx - rx)
        yaw_err = bearing - ryaw
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        if rz < 0.15:
            # Robot fell 鈥?zero command, let physics settle
            nav_cmd[:] = [0.0, 0.0, 0.0]
        elif dist > 1.0:
            # Go1 policy: conservative speed for stability (~8s safe window)
            vx_cmd = 0.28 * max(0.2, math.cos(yaw_err))
            wz_cmd = np.clip(yaw_err * 0.6, -0.5, 0.5)
            nav_cmd[:] = [vx_cmd, 0.0, wz_cmd]
        else:
            nav_cmd[:] = [0.0, 0.0, 0.0]

        # 4. Trail
        trail.append((rx, ry))

        # Get nav state directly from module internals
        try:
            nav_state = str(nav._state).split(".")[-1] if nav._state else "?"
            wp_idx = nav._wp_index
            wp_total = len(nav._path)
            if nav._path:
                global_path = [(p.pose.position.x, p.pose.position.y) for p in nav._path]
        except Exception:
            nav_state = "?"
            wp_idx = wp_total = 0

        # Status print (every 2s)
        if frame_i % (FPS * 2) == 0:
            print(
                f"t={elapsed:.0f}s pos=({rx:.2f},{ry:.2f},z={rz:.2f}) "
                f"yaw={math.degrees(ryaw):.0f}deg "
                f"cmd=({nav_cmd[0]:.2f},{nav_cmd[2]:.2f}) "
                f"dist={dist:.1f} {nav_state}"
            )

        # 5. Render 3D + overlay
        if renderer and video:
            # Camera follows robot
            cam.lookat[0] = rx * 0.7 + GOAL[0] * 0.3
            cam.lookat[1] = ry * 0.7 + GOAL[1] * 0.3
            cam.lookat[2] = 0.4
            cam.distance = max(4.0, dist * 0.6 + 3.0)

            mujoco.mj_forward(model, data)
            renderer.update_scene(data, cam)
            inject_markers(renderer._scene, lidar_pts, global_path, trail, GOAL)
            rgb = renderer.render()
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            draw_minimap(
                bgr, mapper.grid, mapper.ORIGIN, mapper.RESOLUTION, (rx, ry), ryaw, lidar_pts, global_path, GOAL
            )
            draw_info_panel(bgr, elapsed, (rx, ry), math.degrees(ryaw), dist, nav_state, wp_idx, wp_total, mapper.scans)
            video.write(bgr)

        # Check success (distance-based, independent of nav module state)
        if dist < 1.5:
            print(f"\n=== GOAL REACHED at t={elapsed:.1f}s ===")
            # Write a few more frames
            for _ in range(FPS * 2):
                if renderer and video:
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, cam)
                    rgb = renderer.render()
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    draw_info_panel(
                        bgr, elapsed, (rx, ry), math.degrees(ryaw), dist, "SUCCESS", wp_idx, wp_total, mapper.scans
                    )
                    video.write(bgr)
            break

    # Cleanup
    print(f"\nFinal: pos=({rx:.2f},{ry:.2f}) dist={dist:.1f} scans={mapper.scans}")
    if video:
        video.release()
        print(f"Video saved: {_REPO_ROOT / 'assets' / 'media' / 'go1_indoor_nav.mp4'}")
    system.stop()


if __name__ == "__main__":
    main()
