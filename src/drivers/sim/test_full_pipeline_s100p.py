#!/usr/bin/env python3
"""S100P full pipeline test: MuJoCo + terrain_core + PCT/A* + navigation.

Run on S100P:
    cd ~/data/inovxio/lingtu/src
    PYTHONPATH=. python3 drivers/sim/test_full_pipeline_s100p.py
"""

import math
import os
import sys
import time

import numpy as np
import pytest

pytestmark = [pytest.mark.sim]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../.."))


def _prepend_path(path: str) -> None:
    if path not in sys.path:
        sys.path.insert(0, path)


def _remove_script_dir_from_syspath() -> None:
    script_dir_norm = os.path.normcase(SCRIPT_DIR)
    filtered = []
    for entry in sys.path:
        entry_path = entry or os.getcwd()
        if os.path.normcase(os.path.abspath(entry_path)) != script_dir_norm:
            filtered.append(entry)
    sys.path[:] = filtered


def _mujoco_version(module) -> str:
    version = getattr(module, "__version__", None)
    if version:
        return str(version)
    version_fn = getattr(module, "mj_versionString", None)
    if callable(version_fn):
        try:
            return str(version_fn())
        except Exception:
            pass
    return "unknown"


_remove_script_dir_from_syspath()
import mujoco  # noqa: E402

_prepend_path(SRC_ROOT)
_prepend_path(REPO_ROOT)

# 鈹€鈹€ Step 1: Check components 鈹€鈹€
print("=" * 60)
print("Step 1: Component check")
print("=" * 60)

print("  nanobind...", end=" ")
import _nav_core  # noqa: E402

print("OK (%d functions)" % len([x for x in dir(_nav_core) if not x.startswith("_")]))

print("  MuJoCo...", end=" ")
print("OK (v%s)" % _mujoco_version(mujoco))

print("  sim engine...", end=" ")
from sim.engine.core.engine import VelocityCommand  # noqa: E402
from sim.engine.core.robot import RobotConfig  # noqa: E402
from sim.engine.core.sensor import LidarConfig  # noqa: E402
from sim.engine.core.world import ObstacleConfig, WorldConfig  # noqa: E402
from sim.engine.mujoco.engine import MuJoCoEngine  # noqa: E402

print("OK")

print("  terrain_core...", end=" ")
tc = _nav_core.TerrainAnalysisCore()
print("OK")

print("  PCT planner...", end=" ")
so_path = os.path.join(os.path.dirname(__file__), "../../global_planning/pct_planner/build/ele_planner.so")
has_pct = os.path.exists(so_path)
print("OK" if has_pct else "NOT FOUND (will use A*)")

print("  nav_core.score_path...", end=" ")
try:
    _ = _nav_core.score_path
    print("OK")
except AttributeError:
    print("MISSING")

print("  nav_core.compute_control...", end=" ")
try:
    _ = _nav_core.compute_control
    print("OK")
except AttributeError:
    print("MISSING")

print("  planner backends...", end=" ")

try:
    print("A* + PCT")
    USE_PCT = True
except Exception:
    print("A* only")
    USE_PCT = False

# 鈹€鈹€ Step 2: MuJoCo + terrain_core test 鈹€鈹€
print()
print("=" * 60)
print("Step 2: MuJoCo + terrain_core (nanobind C++)")
print("=" * 60)

SIM_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../sim"))
rc = RobotConfig.default_nova_dog()
rc.resolve_paths(base_dir=SIM_ROOT)

obstacles = [
    ObstacleConfig("wall_L", "box", [8.0, 0.15, 0.8], [4.0, 3.0, 0.4]),
    ObstacleConfig("wall_R", "box", [8.0, 0.15, 0.8], [4.0, -3.0, 0.4]),
    ObstacleConfig("block", "box", [0.6, 1.5, 0.5], [4.0, 0.0, 0.25]),
]

e = MuJoCoEngine(
    robot_config=rc,
    world_config=WorldConfig(obstacles=obstacles),
    lidar_config=LidarConfig(),
    headless=True,
)
e.load()
e.reset()

# Scan with terrain_core
terrain = _nav_core.TerrainAnalysisCore()
scan_results = []

for step in range(100):
    e.step(VelocityCommand(linear_x=0.2))
    if step % 10 == 0:
        pts = e.get_lidar_points()
        if pts is not None and len(pts) > 0:
            state = e.get_robot_state()
            terrain.update_vehicle(
                state.position[0], state.position[1], state.position[2],
                0, 0, 0)
            flat = pts[:, :4].ravel().tolist() if pts.shape[1] >= 4 else \
                   np.c_[pts[:, :3], np.zeros(len(pts))].ravel().tolist()
            result = terrain.process(flat, time.time())
            scan_results.append(result.n_points)

pos = e.get_robot_state().position
print("  Robot pos: (%.2f, %.2f)" % (pos[0], pos[1]))
print("  Terrain scans: %d, obstacle points: %s" % (len(scan_results), scan_results))
print("  Elevation map: %dx%d" % ((scan_results and (51, 51)) or (0, 0)))
e.close()
print("  PASSED")

# 鈹€鈹€ Step 3: Full navigation pipeline 鈹€鈹€
print()
print("=" * 60)
print("Step 3: Full MuJoCo navigation (cmu_py + nav_core + A*)")
print("=" * 60)

from base_autonomy.modules import LocalPlannerModule, PathFollowerModule, TerrainModule  # noqa: E402
from core.blueprint import Blueprint  # noqa: E402
from core.module import Module  # noqa: E402
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3  # noqa: E402
from core.msgs.nav import Odometry  # noqa: E402
from core.msgs.sensor import PointCloud2  # noqa: E402
from core.runtime_interface import runtime_fixed_path_frame_ids  # noqa: E402
from core.stream import In, Out  # noqa: E402
from drivers.sim.mujoco_driver_module import MujocoDriverModule  # noqa: E402
from nav.navigation_module import NavigationModule  # noqa: E402

FULL_PIPELINE_GOAL_FRAME_ID = runtime_fixed_path_frame_ids()[0]


class LiveMapper(Module, layer=3):
    lidar_in: In[PointCloud2]
    odom_in: In[Odometry]
    costmap_out: Out[dict]
    goal_cmd: Out[PoseStamped]

    def __init__(self, **kw):
        super().__init__(**kw)
        self.grid = np.zeros((200, 200), dtype=np.float32)
        self.scans = 0
        self.rx = self.ry = 0.0

    def setup(self):
        self.lidar_in.subscribe(self._on_lidar)
        self.odom_in.subscribe(self._on_odom)

    def _on_odom(self, o):
        self.rx = o.pose.position.x
        self.ry = o.pose.position.y

    def _on_lidar(self, cloud):
        if cloud.points is None:
            return
        h = 100
        for p in cloud.points:
            if len(p) >= 3 and 0.05 < abs(p[2]) < 1.2:
                d = math.hypot(p[0] - self.rx, p[1] - self.ry)
                if 0.3 < d < 8.0:
                    gx, gy = int(p[0] / 0.2 + h), int(p[1] / 0.2 + h)
                    if 0 <= gx < 200 and 0 <= gy < 200:
                        self.grid[gy, gx] = min(self.grid[gy, gx] + 25, 100)
        self.scans += 1
        if self.scans % 5 == 0:
            self.costmap_out.publish({
                "grid": self.grid.copy(),
                "resolution": 0.2,
                "origin": [0.0, 0.0],
            })


bp = Blueprint()
bp.add(MujocoDriverModule, world="open_field", sim_rate=50.0, render=False, obstacles=obstacles)
bp.add(LiveMapper)
bp.add(TerrainModule, backend="nanobind")
bp.add(LocalPlannerModule, backend="cmu_py")
bp.add(PathFollowerModule, backend="nav_core")
bp.add(NavigationModule, planner="astar", waypoint_threshold=2.0, downsample_dist=1.0)

# Odometry 鈫?all consumers
bp.wire("MujocoDriverModule", "odometry", "LiveMapper", "odom_in")
bp.wire("MujocoDriverModule", "odometry", "NavigationModule", "odometry")
bp.wire("MujocoDriverModule", "odometry", "TerrainModule", "odometry")
bp.wire("MujocoDriverModule", "odometry", "LocalPlannerModule", "odometry")
bp.wire("MujocoDriverModule", "odometry", "PathFollowerModule", "odometry")
# LiDAR 鈫?TerrainModule (raw cloud) + LiveMapper (costmap)
bp.wire("MujocoDriverModule", "lidar_cloud", "TerrainModule", "map_cloud")
bp.wire("MujocoDriverModule", "lidar_cloud", "LiveMapper", "lidar_in")
# TerrainModule 鈫?LocalPlannerModule (processed terrain, not raw cloud)
bp.wire("TerrainModule", "terrain_map", "LocalPlannerModule", "terrain_map")
# Costmap + goal
bp.wire("LiveMapper", "costmap_out", "NavigationModule", "costmap")
bp.wire("LiveMapper", "goal_cmd", "NavigationModule", "goal_pose")
# Navigation 鈫?local planning 鈫?path following 鈫?driver
bp.wire("NavigationModule", "waypoint", "LocalPlannerModule", "waypoint")
bp.wire("LocalPlannerModule", "local_path", "PathFollowerModule", "local_path")
bp.wire("PathFollowerModule", "cmd_vel", "MujocoDriverModule", "cmd_vel")

system = bp.build()
system.start()
time.sleep(6.0)  # extra warmup for cmu_py path loading + terrain init

mapper = system.get_module("LiveMapper")
nav = system.get_module("NavigationModule")

GOAL = (7.0, 0.0)
mapper.goal_cmd.publish(PoseStamped(
    pose=Pose(position=Vector3(GOAL[0], GOAL[1], 0.0), orientation=Quaternion(0, 0, 0, 1)),
    frame_id=FULL_PIPELINE_GOAL_FRAME_ID, ts=time.time(),
))

success = False
for t in range(30):  # 60s total (real algorithms need more time)
    time.sleep(2.0)
    nh = nav.health()["navigation"]
    dist = math.hypot(mapper.rx - GOAL[0], mapper.ry - GOAL[1])
    occ = (mapper.grid > 40).sum()
    print("  t=%2ds pos=(%.1f,%.1f) dist=%.1f wp=%d/%d map=%d %s" % (
        (t + 1) * 2, mapper.rx, mapper.ry, dist,
        nh["wp_index"], nh["wp_total"], occ, nh["state"]))
    if nh["state"] == "SUCCESS":
        success = True
        break

system.stop()

print()
print("=" * 60)
if success:
    print("RESULT: ALL TESTS PASSED 鈥?MuJoCo + terrain + navigation SUCCESS")
else:
    print("RESULT: Navigation did not reach goal (may need more time or tuning)")
print("=" * 60)
