"""LocalPlannerModule — local path planning as a pluggable Module.

Takes terrain map + waypoint + odometry, produces obstacle-free local path.

NAV COMPUTE CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md):
  This module owns the L2 LOCAL PLANNING layer.
    - MAIN INPUT  : terrain_map (+ boundary + added_obstacles) — local geometry.
    - MAIN OUTPUT : local_path (+ control_hint).
    - ALGORITHM   : CMU / nav_core point-cloud voxel scoring (NOT costmap
                    rolling optimisation — this is not a DWA/TEB planner).
  `fused_cost`/`costmap` is a GLOBAL-gating signal and is intentionally NOT a
  primary scoring input here. The `esdf` In port is a RESERVED extension point:
  nav_core's LocalPlannerCore has no set_esdf binding yet, so _on_esdf only
  caches the field — it does NOT influence scoring. See contract §5 before
  claiming ESDF affects local planning.

Backends:
  "nanobind" — C++ LocalPlannerCore via nanobind (full CMU scoring, zero ROS2) [PREFERRED]
  "cmu"      — C++ local_planner (NativeModule subprocess, needs ROS2)
  "cmu_py"   — Pure-Python CMU scoring via numpy (slower fallback, no ROS2)
  "simple"   — straight-line path for testing

Usage::

    bp.add(LocalPlannerModule, backend="nanobind")  # C++ in-process, no ROS2
    bp.add(LocalPlannerModule, backend="cmu")        # C++ subprocess, needs ROS2
"""

from __future__ import annotations

import logging
import math
import os
import time
from pathlib import Path as FsPath
from typing import Any

import numpy as np

from base_autonomy.modules._nav_core_loader import nav_core_build_hint, try_import_nav_core
from core.backend_status import BackendStatus, require_backend
from core.module import Module
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud2
from core.registry import register
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.stream import In, Out

logger = logging.getLogger(__name__)

# Pre-computed sin/cos lookup table for 36 rotation directions (10° steps, -180..+170°)
# Matches RotLUT in local_planner_core.hpp
_ROT_SIN = np.array([math.sin((10.0 * i - 180.0) * math.pi / 180.0) for i in range(36)])
_ROT_COS = np.array([math.cos((10.0 * i - 180.0) * math.pi / 180.0) for i in range(36)])

# Default planner parameters (mirrors localPlanner.cpp defaults)
_PATH_NUM  = 343
_GROUP_NUM = 7

_OBSTACLE_HEIGHT_THRE = 0.5   # m — points above this block paths
_GROUND_HEIGHT_THRE   = 0.1   # m — points above this contribute terrain penalty
_POINT_PER_PATH_THRE  = 2     # blocked path threshold (pointPerPathThre_)
_PATH_SCALE           = 1.0   # pathScale_ default
_PATH_RANGE           = 3.5   # adjacentRange_ default — scan radius in metres
_DIR_THRE             = 90.0  # degrees — max allowed direction deviation
_DIR_WEIGHT           = 0.02  # direction penalty weight in path scoring (radians penalty factor)
_GOAL_CLEAR_RANGE     = 0.5   # m
_BOUNDARY_INTENSITY   = 100.0  # intensity value for hard-obstacle boundary points (matches C++ boundaryHandler)
_OBSTACLE_INTENSITY   = 200.0  # intensity value for added-obstacle points (matches C++ addedObstaclesHandler)
_MIN_TRACKABLE_LOCAL_PATH_XY = 0.30


def _load_paths(paths_dir: str) -> dict | None:
    """Load pre-computed paths from PLY files.

    Returns dict with keys:
      paths          — list of Nx3 float arrays, one per path_id
      group_of_path  — int array[pathNum], group_id for each path_id
      start_paths    — list of Mx3 float arrays, one per group_id (start segment)
      correspondences — list of lists, correspondences[voxel_idx] = [path_ids...]
      grid_params    — dict with voxel grid dimensions read from correspondences
    """
    pd = FsPath(paths_dir)

    # -- paths.ply: all path points with path_id and group_id --
    paths_ply = pd / "paths.ply"
    if not paths_ply.exists():
        logger.error("LocalPlannerModule [cmu_py]: paths.ply not found at %s", paths_ply)
        return None

    paths_data: list[list] = [[] for _ in range(_PATH_NUM)]
    with open(paths_ply, "rb") as f:
        raw = f.read()
    pos = raw.find(b"end_header\r\n")
    if pos == -1:
        pos = raw.find(b"end_header\n")
        if pos == -1:
            logger.error("LocalPlannerModule [cmu_py]: bad paths.ply header")
            return None
        pos += len(b"end_header\n")
    else:
        pos += len(b"end_header\r\n")
    text = raw[pos:].decode("ascii")
    for line in text.strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        path_id  = int(parts[3])
        if 0 <= path_id < _PATH_NUM:
            paths_data[path_id].append((x, y, z))
    paths = [np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)
             for pts in paths_data]

    # -- pathList.ply: group_id for each path_id --
    path_list_ply = pd / "pathList.ply"
    group_of_path = np.zeros(_PATH_NUM, dtype=np.int32)
    with open(path_list_ply, "rb") as f:
        raw2 = f.read()
    pos2 = raw2.find(b"end_header\r\n")
    if pos2 == -1:
        pos2 = raw2.find(b"end_header\n") + len(b"end_header\n")
    else:
        pos2 += len(b"end_header\r\n")
    text2 = raw2[pos2:].decode("ascii")
    for line in text2.strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        path_id  = int(parts[3])
        group_id = int(parts[4])
        if 0 <= path_id < _PATH_NUM and 0 <= group_id < _GROUP_NUM:
            group_of_path[path_id] = group_id

    # -- startPaths.ply: one group-start segment per group_id --
    start_ply = pd / "startPaths.ply"
    start_paths_data: list[list] = [[] for _ in range(_GROUP_NUM)]
    with open(start_ply, "rb") as f:
        raw3 = f.read()
    pos3 = raw3.find(b"end_header\r\n")
    if pos3 == -1:
        pos3 = raw3.find(b"end_header\n") + len(b"end_header\n")
    else:
        pos3 += len(b"end_header\r\n")
    text3 = raw3[pos3:].decode("ascii")
    for line in text3.strip().splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        x, y, z     = float(parts[0]), float(parts[1]), float(parts[2])
        group_id     = int(parts[3])
        if 0 <= group_id < _GROUP_NUM:
            start_paths_data[group_id].append((x, y, z))
    start_paths = [np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)
                   for pts in start_paths_data]

    # -- correspondences.txt: voxel_id → list of blocked path_ids --
    corr_txt = pd / "correspondences.txt"
    correspondences: dict[int, list[int]] = {}
    max_voxel_id = 0
    with open(corr_txt, encoding="utf-8") as f:
        for line in f:
            tokens = line.split()
            if not tokens:
                continue
            voxel_id = int(tokens[0])
            path_ids_for_voxel = [int(t) for t in tokens[1:] if t != "-1"]
            if path_ids_for_voxel:
                correspondences[voxel_id] = path_ids_for_voxel
            max_voxel_id = max(max_voxel_id, voxel_id)

    # Derive grid dimensions from max voxel id and known defaults
    # gridVoxelNumX=161, gridVoxelNumY=531 from local_planner_core.hpp defaults
    grid_voxel_num_x = 161
    grid_voxel_num_y = 531
    # Validate against max voxel id (voxel_id = gridVoxelNumY * indX + indY)
    expected_max = grid_voxel_num_x * grid_voxel_num_y - 1
    if max_voxel_id > expected_max:
        grid_voxel_num_y = (max_voxel_id // grid_voxel_num_x) + 2
        logger.warning("LocalPlannerModule [cmu_py]: adjusted gridVoxelNumY to %d", grid_voxel_num_y)

    logger.info(
        "LocalPlannerModule [cmu_py]: loaded %d paths, %d groups, %d non-empty correspondences",
        _PATH_NUM, _GROUP_NUM, len(correspondences),
    )
    return {
        "paths":          paths,
        "group_of_path":  group_of_path,
        "start_paths":    start_paths,
        "correspondences": correspondences,
        "grid_voxel_num_x": grid_voxel_num_x,
        "grid_voxel_num_y": grid_voxel_num_y,
    }


def _score_paths_numpy(
    obstacle_pts: np.ndarray,          # Nx3 float32, body-frame (x fwd, y left)
    rel_goal_x: float,
    rel_goal_y: float,
    rel_goal_dis: float,
    joy_dir_deg: float,                 # direction to goal in body frame (degrees)
    correspondences: dict[int, list[int]],
    group_of_path: np.ndarray,
    grid_voxel_num_x: int,
    grid_voxel_num_y: int,
    # W2-6: grid params were hardcoded; now configurable with CMU defaults.
    grid_voxel_size: float = 0.02,
    grid_voxel_offset_x: float = 3.2,
    grid_voxel_offset_y: float = 5.25,
    search_radius: float = 0.45,
) -> np.ndarray:
    """Vectorised path scoring.  Returns clearPathPerGroupScore[36 * GROUP_NUM]."""

    inv_gs = 1.0 / grid_voxel_size
    offset_x_half = grid_voxel_offset_x + grid_voxel_size * 0.5
    offset_y_half = grid_voxel_offset_y + grid_voxel_size * 0.5
    scale_y_coef_a = search_radius / grid_voxel_offset_y
    scale_y_coef_b = 1.0 / grid_voxel_offset_x
    path_range_sq = _PATH_RANGE * _PATH_RANGE

    # Scoring accumulators
    clear_path_list = np.zeros(_PATH_NUM * 36, dtype=np.int32)
    path_penalty_list = np.zeros(_PATH_NUM * 36, dtype=np.float32)
    clear_per_group_score = np.zeros(36 * _GROUP_NUM, dtype=np.float64)
    clear_per_group_num   = np.zeros(36 * _GROUP_NUM, dtype=np.int32)

    # Direction validity: skip rotations too far from goal direction
    ang_diff_list = np.array([
        abs(joy_dir_deg - (10.0 * d - 180.0)) % 360.0
        for d in range(36)
    ], dtype=np.float32)
    ang_diff_list = np.where(ang_diff_list > 180.0, 360.0 - ang_diff_list, ang_diff_list)
    rot_dir_valid = ang_diff_list <= _DIR_THRE  # shape (36,)

    if obstacle_pts.shape[0] > 0:
        xs = obstacle_pts[:, 0].astype(np.float64)
        ys = obstacle_pts[:, 1].astype(np.float64)
        # Use intensity column (index 3) if available for terrain height, else use z
        if obstacle_pts.shape[1] >= 4:
            hs = obstacle_pts[:, 3].astype(np.float64)
        else:
            hs = obstacle_pts[:, 2].astype(np.float64)

        dist_sq = xs * xs + ys * ys
        in_range = dist_sq < path_range_sq

        for rot_dir in range(36):
            if not rot_dir_valid[rot_dir]:
                continue
            rc = _ROT_COS[rot_dir]
            rs = _ROT_SIN[rot_dir]

            # Rotate obstacle points into this rotation frame
            x2 = rc * xs + rs * ys   # shape (N,)
            y2 = -rs * xs + rc * ys

            scale_y = x2 * scale_y_coef_b + scale_y_coef_a * (grid_voxel_offset_x - x2) * scale_y_coef_b
            valid_scale = np.abs(scale_y) > 1e-6

            ind_x = ((offset_x_half - x2) * inv_gs).astype(np.int32)
            # Avoid divide-by-zero in y scaling
            safe_scale_y = np.where(valid_scale, scale_y, 1.0)
            ind_y = ((offset_y_half - y2 / safe_scale_y) * inv_gs).astype(np.int32)

            grid_valid = (
                valid_scale
                & in_range
                & (ind_x >= 0) & (ind_x < grid_voxel_num_x)
                & (ind_y >= 0) & (ind_y < grid_voxel_num_y)
            )

            valid_indices = np.where(grid_valid)[0]
            for vi in valid_indices:
                voxel_id = int(ind_x[vi]) * grid_voxel_num_y + int(ind_y[vi])
                if voxel_id not in correspondences:
                    continue
                h = float(hs[vi])
                for path_id in correspondences[voxel_id]:
                    idx = _PATH_NUM * rot_dir + path_id
                    if h > _OBSTACLE_HEIGHT_THRE:
                        clear_path_list[idx] += 1
                    elif h > _GROUND_HEIGHT_THRE:
                        if path_penalty_list[idx] < h:
                            path_penalty_list[idx] = h

    # Score paths into group scores
    omni_thre    = 5.0
    slope_weight = 0.0

    for rot_dir in range(36):
        if not rot_dir_valid[rot_dir]:
            continue
        ang_diff = float(ang_diff_list[rot_dir])
        dw = abs(_DIR_WEIGHT * ang_diff)
        sqrt_sqrt_dw = math.sqrt(math.sqrt(dw))

        # computeRotDirW
        if rot_dir < 18:
            rot_dir_w = abs(abs(rot_dir - 9.0) + 1.0)
        else:
            rot_dir_w = abs(abs(rot_dir - 27.0) + 1.0)
        rot_dir_w2 = rot_dir_w * rot_dir_w
        rot_dir_w4 = rot_dir_w2 * rot_dir_w2

        base = _PATH_NUM * rot_dir
        for path_id in range(_PATH_NUM):
            if clear_path_list[base + path_id] >= _POINT_PER_PATH_THRE:
                continue  # blocked
            group_id  = int(group_of_path[path_id])
            # computeGroupDirW
            group_dir_w = 4.0 - abs(group_id - 3.0)

            terrain_penalty = float(path_penalty_list[base + path_id])
            terrain_factor = (max(0.0, 1.0 - slope_weight * terrain_penalty)
                              if slope_weight > 0.0 else 1.0)

            if rel_goal_dis < omni_thre:
                score = (1.0 - sqrt_sqrt_dw) * group_dir_w * group_dir_w * terrain_factor
            else:
                score = (1.0 - sqrt_sqrt_dw) * rot_dir_w4 * terrain_factor

            group_idx = _GROUP_NUM * rot_dir + group_id
            clear_per_group_score[group_idx] += score
            clear_per_group_num[group_idx]   += 1

    return clear_per_group_score


_AVAILABLE_LOCAL_PLANNER_BACKENDS = ("nanobind", "cmu", "cmu_py", "simple")


@register("local_planner", "simple", description="Explicit straight-line local planner for tests only")
@register("local_planner", "cmu_py", description="Pure-Python CMU local planner scorer")
@register("local_planner", "nanobind", description="C++ LocalPlannerCore via nanobind")
@register("local_planner", "cmu", description="CMU-style C++ local planner (NativeModule)")
class LocalPlannerModule(Module, layer=2):
    """Local path planning — obstacle avoidance + path scoring.

    "cmu"    backend: C++ local_planner NativeModule (requires ROS2)
    "cmu_py" backend: Pure-Python CMU scorer via _nav_core + numpy (no ROS2)
    "simple" backend: straight-line to waypoint (testing)
    """

    # -- Inputs --
    odometry:    In[Odometry]
    terrain_map: In[PointCloud2]
    waypoint:    In[PoseStamped]
    global_path: In[list]
    clear_path:  In[bool]
    map_frame_jump_event: In[dict]
    boundary:    In[PointCloud2]
    added_obstacles: In[PointCloud2]
    esdf:        In[dict]       # ESDF distance field + gradients for smoother scoring

    # -- Outputs --
    local_path: Out[Path]
    control_hint: Out[dict]
    alive:      Out[bool]

    def __init__(self, backend: str = "cmu", **kw):
        super().__init__(**kw)
        require_backend("local_planner", backend, _AVAILABLE_LOCAL_PLANNER_BACKENDS)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend = backend
        self._node = None
        self._robot_pos = np.zeros(3)
        self._robot_yaw = 0.0
        self._latest_waypoint: PoseStamped | None = None
        self._global_path_points: np.ndarray | None = None
        self._path_frame_id = topic_default_frame_id(TOPICS.local_path)
        self._corridor_lookahead_m = float(kw.get("corridor_lookahead_m", 3.0))
        self._allow_direct_track_fallback = bool(
            kw.get("allow_direct_track_fallback", False)
        )
        self._ignore_near_field_stop = bool(kw.get("ignore_near_field_stop", False))
        self._direct_track_fallback_min_distance_m = float(
            kw.get("direct_track_fallback_min_distance_m", _MIN_TRACKABLE_LOCAL_PATH_XY)
        )
        self._min_trackable_local_path_xy = float(
            kw.get("min_trackable_local_path_m", _MIN_TRACKABLE_LOCAL_PATH_XY)
        )
        self._terrain_points: np.ndarray | None = None
        self._boundary_points: np.ndarray | None = None
        self._added_obstacle_points: np.ndarray | None = None
        self._esdf_field: np.ndarray | None = None
        self._esdf_resolution: float = 0.2
        self._esdf_origin: np.ndarray | None = None
        self._last_cmu_py_time: float = 0.0

        # cmu_py state
        self._path_data: dict | None = None
        self._nav_core = None

        # nanobind state
        self._core = None  # _nav_core.LocalPlannerCore

        # One-shot warning flags
        self._warned_no_core: bool = False
        self._warned_no_path_data: bool = False
        self._last_control_hint: dict[str, Any] = {}
        self._last_local_path_points: int = 0
        self._last_local_path_span_m: float = 0.0
        self._last_direct_track_fallback_ts: float = 0.0
        self._effective_local_planner_params: dict[str, Any] = {}

        # W2-6: cmu_py grid parameters — pulled from config at setup() if the
        # `local_planner_grid` section is present, otherwise keep CMU defaults.
        # Exposing them as instance attrs makes them unit-testable and lets the
        # stack factory override per-robot without touching magic numbers.
        self._grid_voxel_size: float = 0.02
        self._grid_voxel_offset_x: float = 3.2
        self._grid_voxel_offset_y: float = 5.25
        self._grid_search_radius: float = 0.45
        try:
            from core.config import get_config
            cfg = get_config()
            lpg = cfg.raw.get("local_planner_grid", {}) if hasattr(cfg, "raw") else {}
            if lpg:
                self._grid_voxel_size = float(lpg.get("voxel_size", self._grid_voxel_size))
                self._grid_voxel_offset_x = float(lpg.get("x_offset", self._grid_voxel_offset_x))
                self._grid_voxel_offset_y = float(lpg.get("y_offset", self._grid_voxel_offset_y))
                self._grid_search_radius = float(lpg.get("search_radius", self._grid_search_radius))
                logger.info(
                    "LocalPlannerModule: loaded grid params from config "
                    "(voxel=%.3f, x_off=%.2f, y_off=%.2f, radius=%.2f)",
                    self._grid_voxel_size, self._grid_voxel_offset_x,
                    self._grid_voxel_offset_y, self._grid_search_radius,
                )
        except (ImportError, AttributeError, KeyError):
            logger.info(
                "LocalPlannerModule: using default grid params "
                "(voxel=0.02, x_off=3.2, y_off=5.25, radius=0.45) — "
                "add local_planner_grid to robot_config.yaml to override"
            )

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.terrain_map.subscribe(self._on_terrain)
        self.terrain_map.set_policy("latest")
        self.waypoint.subscribe(self._on_waypoint)
        self.global_path.subscribe(self._on_global_path)
        self.clear_path.subscribe(self._on_clear_path)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)
        self.boundary.subscribe(self._on_boundary)
        self.boundary.set_policy("latest")
        self.added_obstacles.subscribe(self._on_added_obstacles)
        self.added_obstacles.set_policy("latest")
        self.esdf.subscribe(self._on_esdf)
        self.esdf.set_policy("latest")

        if self._backend == "nanobind":
            self._setup_nanobind()
        elif self._backend == "cmu":
            self._setup_cmu()
        elif self._backend == "cmu_py":
            self._setup_cmu_py()
        else:
            logger.info("LocalPlannerModule: simple backend (straight-line)")

    # ------------------------------------------------------------------ #
    # Backend setup                                                        #
    # ------------------------------------------------------------------ #

    def _setup_nanobind(self):
        """C++ LocalPlannerCore via nanobind — full CMU scoring in-process, zero ROS2."""
        _nav_core = try_import_nav_core(("LocalPlannerParams", "LocalPlannerCore"))
        if _nav_core is None:
            logger.warning(
                "LocalPlannerModule [nanobind]: compatible _nav_core not found; "
                "falling back to cmu_py scorer. To restore the C++ backend: %s",
                nav_core_build_hint(),
            )
            self._backend_status.use("cmu_py", reason="compatible _nav_core missing")
            self._backend = "cmu_py"
            self._setup_cmu_py()
            return
        try:
            params = _nav_core.LocalPlannerParams()
            summary: dict[str, Any] = {}
            missing: list[str] = []

            def set_param(name: str, value: Any) -> None:
                if not hasattr(params, name):
                    missing.append(name)
                    return
                setattr(params, name, value)
                summary[name] = getattr(params, name)

            def current(name: str, fallback: Any) -> Any:
                return getattr(params, name, fallback)

            try:
                from core.config import get_config
                cfg = get_config()
                raw = getattr(cfg, "raw", {}) or {}
                lp = raw.get("local_planner", {}) or {}
                ta = raw.get("terrain", {}) or {}
                geom = getattr(cfg, "geometry", None)
                safety = getattr(cfg, "safety", None)
                speed = getattr(cfg, "speed", None)

                set_param("vehicle_length", getattr(geom, "vehicle_length", current("vehicle_length", 0.6)))
                set_param("vehicle_width", getattr(geom, "vehicle_width", current("vehicle_width", 0.6)))
                set_param("sensor_offset_x", getattr(geom, "sensor_offset_x", current("sensor_offset_x", 0.0)))
                set_param("sensor_offset_y", getattr(geom, "sensor_offset_y", current("sensor_offset_y", 0.0)))
                set_param(
                    "obstacle_height_thre",
                    getattr(safety, "obstacle_height_thre", current("obstacle_height_thre", 0.2)),
                )
                set_param(
                    "ground_height_thre",
                    getattr(safety, "ground_height_thre", current("ground_height_thre", 0.1)),
                )
                set_param("max_speed", getattr(speed, "max_speed", current("max_speed", 1.0)))
                set_param("autonomy_speed", getattr(speed, "autonomy_speed", current("autonomy_speed", 1.0)))

                for name, default in (
                    ("two_way_drive", True),
                    ("adjacent_range", 3.5),
                    ("check_obstacle", True),
                    ("check_rot_obstacle", False),
                    ("use_terrain_analysis", True),
                    ("point_per_path_thre", 2),
                    ("dir_weight", 0.02),
                    ("dir_thre", 90.0),
                    ("path_scale", 1.0),
                    ("min_path_scale", 0.75),
                    ("path_scale_step", 0.25),
                    ("path_scale_by_speed", True),
                    ("min_path_range", 1.0),
                    ("path_range_step", 0.5),
                    ("path_range_by_speed", True),
                    ("path_crop_by_goal", True),
                    ("slope_weight", 0.0),
                    ("goal_clear_range", 0.5),
                    ("near_field_stop_dis", 0.5),
                    ("recovery_blocked_thre", 2.0),
                    ("recovery_rotate_time", 2.5),
                    ("recovery_backup_time", 1.5),
                    ("recovery_max_cycles", 3),
                    ("cost_height_thre_1", 0.15),
                    ("cost_height_thre_2", 0.1),
                    ("dir_to_vehicle", False),
                    ("goal_behind_range", 0.8),
                    ("freeze_ang", 90.0),
                    ("freeze_time", 2.0),
                    ("omni_dir_goal_thre", 1.0),
                    ("slow_path_num_thre", 5),
                    ("slow_group_num_thre", 1),
                ):
                    set_param(name, lp.get(name, current(name, default)))

                set_param("min_rel_z", lp.get("min_rel_z", ta.get("min_rel_z", current("min_rel_z", -0.5))))
                set_param("max_rel_z", lp.get("max_rel_z", ta.get("max_rel_z", current("max_rel_z", 0.25))))
            except ImportError:
                logger.info("LocalPlannerModule [nanobind]: core config unavailable; using C++ defaults")

            self._effective_local_planner_params = summary
            if missing:
                logger.warning(
                    "LocalPlannerModule [nanobind]: _nav_core is missing LocalPlannerParams fields: %s",
                    ", ".join(sorted(set(missing))),
                )

            self._core = _nav_core.LocalPlannerCore(params)

            paths_dir = os.path.join(
                os.path.dirname(__file__), "..", "local_planner", "paths")
            paths_dir = os.path.normpath(paths_dir)
            if not self._core.load_paths(paths_dir):
                raise RuntimeError(
                    f"LocalPlannerModule [nanobind]: failed to load paths from {paths_dir}. "
                    f"Cannot start without path lookup table."
                )

            logger.info(
                "LocalPlannerModule [nanobind]: effective CMU params "
                "(adjacent_range=%.2f, point_per_path_thre=%s, dir_weight=%.3f, "
                "slope_weight=%.2f, near_field_stop_dis=%.2f)",
                float(summary.get("adjacent_range", current("adjacent_range", 3.5))),
                summary.get("point_per_path_thre", current("point_per_path_thre", 2)),
                float(summary.get("dir_weight", current("dir_weight", 0.02))),
                float(summary.get("slope_weight", current("slope_weight", 0.0))),
                float(summary.get("near_field_stop_dis", current("near_field_stop_dis", 0.5))),
            )

            logger.info("LocalPlannerModule [nanobind]: C++ LocalPlannerCore loaded (343 paths × 36 dirs)")
        except RuntimeError:
            raise
        except Exception as e:
            raise RuntimeError(
                f"LocalPlannerModule [nanobind]: _nav_core init failed: {e}. "
                f"Rebuild _nav_core or explicitly choose backend='cmu_py' / 'simple'."
            ) from e

    def _setup_cmu(self):
        try:
            from base_autonomy.native_factories import local_planner
            from core.config import get_config

            cfg = get_config()
            self._node = local_planner(cfg)
            try:
                self._node.setup()
            except (FileNotFoundError, PermissionError) as e:
                logger.warning("LocalPlannerModule: setup failed: %s", e)
                self._node = None
        except ImportError as e:
            logger.warning("LocalPlannerModule: C++ backend not available: %s", e)

    def _setup_cmu_py(self):
        # Try to use _nav_core for accelerated scoring even in cmu_py mode
        _nav_core = try_import_nav_core()
        if _nav_core is not None:
            self._nav_core = _nav_core
            logger.info("LocalPlannerModule [cmu_py]: _nav_core scoring acceleration loaded")
        else:
            logger.info("LocalPlannerModule [cmu_py]: using numpy-only scorer")

        # Load pre-computed path files
        paths_dir = os.path.join(
            os.path.dirname(__file__),
            "..", "local_planner", "paths"
        )
        paths_dir = os.path.normpath(paths_dir)
        self._path_data = _load_paths(paths_dir)
        if self._path_data is None:
            raise RuntimeError(
                f"LocalPlannerModule [cmu_py]: failed to load paths from {paths_dir}. "
                f"Cannot start without path lookup table — use backend='simple' explicitly "
                f"for passthrough testing only."
            )

    # ------------------------------------------------------------------ #
    # Module lifecycle                                                     #
    # ------------------------------------------------------------------ #

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
                logger.info("LocalPlannerModule [cmu]: C++ node started")
            except Exception as e:
                logger.error("LocalPlannerModule: start failed: %s", e)
        self.alive.publish(True)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception as e:
                logger.warning("LocalPlannerModule stop: %s", e)
            self._node = None
        self._core = None
        self.alive.publish(False)
        super().stop()

    # ------------------------------------------------------------------ #
    # Input handlers                                                       #
    # ------------------------------------------------------------------ #

    def _on_odom(self, odom: Odometry):
        self._robot_pos = np.array([odom.x, odom.y, getattr(odom, "z", 0.0)])
        self._robot_yaw = getattr(odom, "yaw", 0.0)

        if self._backend == "nanobind" and self._latest_waypoint is not None:
            if self._core is not None:
                self._run_nanobind(odom.ts if hasattr(odom, "ts") else time.time())
            else:
                if not self._warned_no_core:
                    logger.error(
                        "LocalPlannerModule: nanobind _core is None — "
                        "not publishing any path. This should never happen "
                        "if setup() succeeded.")
                    self._warned_no_core = True
                return
        elif self._backend == "cmu_py" and self._latest_waypoint is not None:
            now = time.time()
            if now - self._last_cmu_py_time >= 1.0:
                self._last_cmu_py_time = now
                self._run_cmu_py()

    def _on_esdf(self, data: dict) -> None:
        """Store ESDF distance field for proximity-aware scoring."""
        self._esdf_field = data.get("distance_field")
        self._esdf_resolution = data.get("resolution", 0.2)
        origin = data.get("origin")
        if origin is not None:
            self._esdf_origin = np.array(origin[:2], dtype=np.float64)

    def _on_terrain(self, cloud: PointCloud2):
        """Store terrain obstacle points for local planning."""
        if cloud.points is not None:
            self._terrain_points = cloud.points

    def _on_boundary(self, cloud: PointCloud2):
        """Store geofence boundary points (treated as hard obstacles)."""
        if cloud.points is not None:
            self._boundary_points = cloud.points

    def _on_added_obstacles(self, cloud: PointCloud2):
        """Store externally injected obstacle points."""
        if cloud.points is not None:
            self._added_obstacle_points = cloud.points

    def _coerce_path_point(self, point: Any) -> np.ndarray | None:
        if isinstance(point, PoseStamped):
            return np.array([point.x, point.y, point.z], dtype=float)
        pose = getattr(point, "pose", None)
        if pose is not None and hasattr(pose, "position"):
            pos = pose.position
            return np.array([
                float(getattr(pos, "x", 0.0)),
                float(getattr(pos, "y", 0.0)),
                float(getattr(pos, "z", 0.0)),
            ], dtype=float)
        if isinstance(point, dict):
            if "pose" in point and isinstance(point["pose"], dict):
                return self._coerce_path_point(point["pose"])
            if "position" in point:
                return self._coerce_path_point(point["position"])
            if "x" in point and "y" in point:
                return np.array([
                    float(point.get("x", 0.0)),
                    float(point.get("y", 0.0)),
                    float(point.get("z", 0.0)),
                ], dtype=float)
        try:
            arr = np.asarray(point, dtype=float).reshape(-1)
        except (TypeError, ValueError):
            return None
        if arr.size < 2 or not np.all(np.isfinite(arr[:2])):
            return None
        out = np.zeros(3, dtype=float)
        out[: min(arr.size, 3)] = arr[: min(arr.size, 3)]
        return out

    def _on_global_path(self, path: Any) -> None:
        frame_id = getattr(path, "frame_id", None)
        if frame_id:
            self._path_frame_id = str(frame_id)
        points_src = getattr(path, "poses", path)
        points = []
        try:
            iterator = iter(points_src)
        except TypeError:
            iterator = iter(())
        for item in iterator:
            point = self._coerce_path_point(item)
            if point is not None:
                points.append(point)
        self._global_path_points = (
            np.asarray(points, dtype=float) if points else None
        )

    def _clear_local_plan(self) -> None:
        self._latest_waypoint = None
        self._global_path_points = None
        self._publish_control_hint(reason="clear_path")
        self._publish_local_path([])

    def _on_map_frame_jump(self, event: dict) -> None:
        if isinstance(event, dict):
            self._clear_local_plan()

    def _waypoint_goal(self, wp: PoseStamped) -> np.ndarray:
        return np.array([
            wp.pose.position.x,
            wp.pose.position.y,
            wp.pose.position.z,
        ], dtype=float)

    def _effective_goal(self, wp: PoseStamped) -> np.ndarray:
        return self._select_corridor_goal(self._waypoint_goal(wp))

    def _select_corridor_goal(self, fallback_goal: np.ndarray) -> np.ndarray:
        points = self._global_path_points
        if points is None or len(points) < 2:
            return fallback_goal
        robot_xy = self._robot_pos[:2]
        if (
            points.shape[1] >= 3
            and len(self._robot_pos) >= 3
            and np.isfinite(float(self._robot_pos[2]))
            and np.all(np.isfinite(points[:, :3]))
        ):
            dists = np.linalg.norm(points[:, :3] - self._robot_pos[:3], axis=1)
        else:
            dists = np.linalg.norm(points[:, :2] - robot_xy, axis=1)
        if not np.all(np.isfinite(dists)):
            return fallback_goal
        idx = int(np.argmin(dists))
        if idx >= len(points) - 1:
            return fallback_goal

        remaining = max(0.1, self._corridor_lookahead_m)
        cursor = points[idx].astype(float, copy=True)
        for next_idx in range(idx + 1, len(points)):
            target = points[next_idx].astype(float, copy=False)
            segment = target - cursor
            seg_len = float(np.linalg.norm(segment[:2]))
            if seg_len <= 1e-6:
                cursor = target.astype(float, copy=True)
                continue
            if remaining <= seg_len:
                return cursor + segment * (remaining / seg_len)
            remaining -= seg_len
            cursor = target.astype(float, copy=True)
        return fallback_goal

    def _on_waypoint(self, wp: PoseStamped):
        """Store waypoint; simple backend generates path immediately."""
        self._latest_waypoint = wp
        if wp is not None:
            self._path_frame_id = wp.frame_id or self._path_frame_id

        if self._backend == "simple" and wp is not None:
            goal = self._effective_goal(wp)
            path_points = self._straight_line(self._robot_pos, goal, step=0.5)
            poses = [
                PoseStamped(
                    pose=Pose(
                        position=Vector3(p[0], p[1], p[2]),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id=self._path_frame_id,
                )
                for p in path_points
            ]
            self._publish_local_path(poses)
            self._publish_control_hint(reason="simple_path")
        elif self._backend == "nanobind" and wp is not None and self._core is not None:
            self._run_nanobind(float(getattr(wp, "ts", 0.0) or time.time()))

    def _on_clear_path(self, clear: bool) -> None:
        if not clear:
            return
        self._clear_local_plan()

    # ------------------------------------------------------------------ #
    # nanobind C++ scorer (full CMU algorithm, in-process)                 #
    # ------------------------------------------------------------------ #

    def _merge_obstacle_clouds(self) -> np.ndarray:
        """Merge terrain + boundary + added_obstacles into single Nx4 array.

        Boundary points get intensity=100 (hard obstacle, matches C++ boundaryHandler).
        Added obstacle points get intensity=200 (matches C++ addedObstaclesHandler).
        """
        clouds = []
        if self._terrain_points is not None and self._terrain_points.shape[0] > 0:
            pts = self._terrain_points.astype(np.float32)
            if pts.shape[1] == 3:
                buf = np.zeros((len(pts), 4), dtype=np.float32)
                buf[:, :3] = pts
                clouds.append(buf)
            elif pts.shape[1] >= 4:
                clouds.append(pts[:, :4].astype(np.float32, copy=False))

        if self._boundary_points is not None and self._boundary_points.shape[0] > 0:
            bpts = self._boundary_points.astype(np.float32)
            buf = np.zeros((len(bpts), 4), dtype=np.float32)
            buf[:, :min(3, bpts.shape[1])] = bpts[:, :min(3, bpts.shape[1])]
            buf[:, 3] = _BOUNDARY_INTENSITY
            clouds.append(buf)

        if self._added_obstacle_points is not None and self._added_obstacle_points.shape[0] > 0:
            apts = self._added_obstacle_points.astype(np.float32)
            buf = np.zeros((len(apts), 4), dtype=np.float32)
            buf[:, :min(3, apts.shape[1])] = apts[:, :min(3, apts.shape[1])]
            buf[:, 3] = _OBSTACLE_INTENSITY
            clouds.append(buf)

        if not clouds:
            return np.zeros((0, 4), dtype=np.float32)
        return np.concatenate(clouds, axis=0)

    def _run_nanobind(self, timestamp: float):
        """Run C++ LocalPlannerCore.plan() and publish result."""
        wp = self._latest_waypoint
        if wp is None:
            return
        if self._core is None:
            if not self._warned_no_core:
                logger.error("LocalPlannerModule: _core lost at runtime — not publishing path")
                self._warned_no_core = True
            return

        self._core.set_vehicle(
            self._robot_pos[0], self._robot_pos[1], self._robot_pos[2],
            self._robot_yaw)
        goal = self._effective_goal(wp)
        self._core.set_goal(float(goal[0]), float(goal[1]))

        merged = self._merge_obstacle_clouds()
        obs_flat = merged.ravel().tolist() if merged.shape[0] > 0 else []

        result = self._core.plan(obs_flat, timestamp)
        self._publish_control_hint(
            slow_down=int(getattr(result, "slow_down", 0) or 0),
            near_field_stop=bool(getattr(result, "near_field_stop", False)),
            path_found=bool(getattr(result, "path_found", True)),
            recovery_state=int(getattr(result, "recovery_state", 0) or 0),
            reason="nanobind",
        )

        if not result.path:
            if self._publish_direct_track_fallback(
                near_field_stop=bool(getattr(result, "near_field_stop", False)),
                path_found=bool(getattr(result, "path_found", False)),
                recovery_state=int(getattr(result, "recovery_state", 0) or 0),
                reason="no_local_path",
            ):
                return
            self._publish_control_hint(
                slow_down=int(getattr(result, "slow_down", 0) or 0),
                near_field_stop=bool(getattr(result, "near_field_stop", False)),
                path_found=bool(getattr(result, "path_found", False)),
                recovery_state=int(getattr(result, "recovery_state", 0) or 0),
                safety_stop=True,
                reason="no_local_path",
            )
            self._publish_local_path([])
            return

        raw_xy = np.asarray([[float(v.x), float(v.y)] for v in result.path], dtype=float)
        xy_length = (
            float(np.sum(np.linalg.norm(np.diff(raw_xy, axis=0), axis=1)))
            if len(raw_xy) > 1
            else 0.0
        )
        xy_span = float(np.linalg.norm(raw_xy[-1] - raw_xy[0])) if len(raw_xy) > 1 else 0.0
        path_found = bool(getattr(result, "path_found", True))
        recovery_state = int(getattr(result, "recovery_state", 0))
        trackable = (
            len(result.path) >= 2
            and max(xy_length, xy_span) >= self._min_trackable_local_path_xy
            and (path_found or recovery_state in (1, 2))
        )
        if not trackable:
            if self._publish_direct_track_fallback(
                near_field_stop=bool(getattr(result, "near_field_stop", False)),
                path_found=path_found,
                recovery_state=recovery_state,
                reason="untrackable_local_path",
            ):
                return
            self._publish_control_hint(
                slow_down=int(getattr(result, "slow_down", 0) or 0),
                near_field_stop=bool(getattr(result, "near_field_stop", False)),
                path_found=path_found,
                recovery_state=recovery_state,
                safety_stop=True,
                reason="untrackable_local_path",
            )
            self._publish_local_path([])
            return

        poses = []
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        for v in result.path:
            wx = v.x * cos_yaw - v.y * sin_yaw + self._robot_pos[0]
            wy = v.x * sin_yaw + v.y * cos_yaw + self._robot_pos[1]
            wz = v.z + self._robot_pos[2]
            poses.append(PoseStamped(
                pose=Pose(
                    position=Vector3(wx, wy, wz),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id=self._path_frame_id,
            ))
        self._publish_local_path(poses)

    # ------------------------------------------------------------------ #
    # CMU Python scorer                                                    #
    # ------------------------------------------------------------------ #

    def _run_cmu_py(self):
        """Run the CMU local planner scoring algorithm and publish best path."""
        wp = self._latest_waypoint
        if wp is None:
            return
        if self._path_data is None:
            if not self._warned_no_path_data:
                logger.error(
                    "LocalPlannerModule [cmu_py]: _path_data is None — "
                    "not publishing any path. This should never happen "
                    "if setup() succeeded.")
                self._warned_no_path_data = True
            return

        pd = self._path_data
        correspondences    = pd["correspondences"]
        group_of_path      = pd["group_of_path"]
        start_paths        = pd["start_paths"]
        grid_voxel_num_x   = pd["grid_voxel_num_x"]
        grid_voxel_num_y   = pd["grid_voxel_num_y"]

        # Goal in body frame
        goal = self._effective_goal(wp)
        gx = goal[0] - self._robot_pos[0]
        gy = goal[1] - self._robot_pos[1]
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        rel_goal_x = gx * cos_yaw + gy * sin_yaw
        rel_goal_y = -gx * sin_yaw + gy * cos_yaw
        rel_goal_dis = math.sqrt(rel_goal_x * rel_goal_x + rel_goal_y * rel_goal_y)
        joy_dir_deg = math.atan2(rel_goal_y, rel_goal_x) * 180.0 / math.pi
        # Clamp to avoid reverse paths
        joy_dir_deg = max(-95.0, min(95.0, joy_dir_deg))

        # Merge all obstacle sources and convert to body frame
        merged = self._merge_obstacle_clouds()
        obs_pts: np.ndarray
        if merged.shape[0] > 0:
            dx = merged[:, 0] - self._robot_pos[0]
            dy = merged[:, 1] - self._robot_pos[1]
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            bz = merged[:, 2] if merged.shape[1] >= 3 else np.zeros(len(dx), dtype=np.float32)
            intensity = merged[:, 3]
            obs_pts = np.stack([bx, by, bz, intensity], axis=1).astype(np.float32)
        else:
            obs_pts = np.zeros((0, 3), dtype=np.float32)

        # Score all paths
        group_scores = _score_paths_numpy(
            obs_pts,
            rel_goal_x, rel_goal_y, rel_goal_dis,
            joy_dir_deg,
            correspondences, group_of_path,
            grid_voxel_num_x, grid_voxel_num_y,
        )

        # Select best group (mirrors selectBestGroup in local_planner_core.hpp)
        selected_group_id = -1
        max_score = 0.0
        for i in range(36 * _GROUP_NUM):
            if group_scores[i] > max_score:
                max_score = group_scores[i]
                selected_group_id = i

        if selected_group_id < 0 or max_score <= 0.0:
            # No feasible local path: publish empty so downstream stops instead of tracking into obstacles.
            self._publish_control_hint(
                safety_stop=True,
                path_found=False,
                reason="no_feasible_local_path",
            )
            self._publish_local_path([])
            return

        rot_dir    = selected_group_id // _GROUP_NUM
        group_id   = selected_group_id  % _GROUP_NUM
        rc = _ROT_COS[rot_dir]
        rs = _ROT_SIN[rot_dir]

        seg = start_paths[group_id]
        if seg.shape[0] == 0:
            return

        # Rotate path points from start-path frame back to world frame
        # Body-frame path: rotate by -rot_dir angle, then translate by robot pos + yaw
        # The start_paths are defined in robot body frame at rot_dir rotation.
        # To get world coordinates: rotate back (inverse rotation = transpose) then
        # apply robot pose.
        poses = []
        for pt in seg:
            # Inverse rotation: x_body = rc*px - rs*py, y_body = rs*px + rc*py
            # (inverse of: px = rc*x_body + rs*y_body, py = -rs*x_body + rc*y_body)
            x_body = rc * float(pt[0]) - rs * float(pt[1])
            y_body = rs * float(pt[0]) + rc * float(pt[1])
            z_body = float(pt[2])
            # Body frame → world frame via robot yaw
            x_world = (x_body * cos_yaw - y_body * sin_yaw) + self._robot_pos[0]
            y_world = (x_body * sin_yaw + y_body * cos_yaw) + self._robot_pos[1]
            z_world = z_body + self._robot_pos[2]
            poses.append(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(x_world, y_world, z_world),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id=self._path_frame_id,
                )
            )

        self._publish_local_path(poses)
        self._publish_control_hint(reason="cmu_py_path")

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _straight_line(self, start: np.ndarray, goal: np.ndarray, step: float = 0.5):
        diff = goal - start
        dist = np.linalg.norm(diff)
        if dist < step:
            return [start, goal]
        n = max(int(dist / step), 2)
        return [start + diff * (i / n) for i in range(1, n + 1)]

    def _publish_control_hint(
        self,
        *,
        slow_down: int = 0,
        near_field_stop: bool = False,
        safety_stop: bool | None = None,
        path_found: bool | None = None,
        recovery_state: int | None = None,
        reason: str = "",
    ) -> None:
        slow_down = max(0, min(3, int(slow_down or 0)))
        near_field_stop = bool(near_field_stop)
        if safety_stop is None:
            stop = near_field_stop and not self._ignore_near_field_stop
        else:
            stop = bool(safety_stop)
        payload: dict[str, Any] = {
            "ts": time.time(),
            "source": "LocalPlannerModule",
            "slow_down": slow_down,
            "near_field_stop": near_field_stop,
            "safety_stop": stop,
            "reason": reason,
        }
        if path_found is not None:
            payload["path_found"] = bool(path_found)
        if recovery_state is not None:
            payload["recovery_state"] = int(recovery_state)
        self._last_control_hint = dict(payload)
        self.control_hint.publish(payload)

    def _publish_local_path(self, poses: list[PoseStamped]) -> None:
        self._last_local_path_points = len(poses)
        if len(poses) >= 2:
            start = poses[0].pose.position
            end = poses[-1].pose.position
            self._last_local_path_span_m = float(
                math.hypot(end.x - start.x, end.y - start.y)
            )
        else:
            self._last_local_path_span_m = 0.0
        self.local_path.publish(Path(poses=poses, frame_id=self._path_frame_id))

    def _publish_direct_track_fallback(
        self,
        *,
        near_field_stop: bool,
        path_found: bool,
        recovery_state: int,
        reason: str,
    ) -> bool:
        if not self._allow_direct_track_fallback:
            return False
        if near_field_stop:
            if not self._ignore_near_field_stop:
                return False
            if not path_found:
                return False
        wp = self._latest_waypoint
        if wp is None:
            return False
        goal = self._effective_goal(wp)
        if not np.all(np.isfinite(goal[:2])):
            return False
        span = float(np.linalg.norm(goal[:2] - self._robot_pos[:2]))
        if span < max(0.0, self._direct_track_fallback_min_distance_m):
            return False
        path_points = self._straight_line(self._robot_pos.copy(), goal, step=0.5)
        if len(path_points) < 2:
            return False
        poses = [
            PoseStamped(
                pose=Pose(
                    position=Vector3(float(p[0]), float(p[1]), float(p[2])),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id=self._path_frame_id,
            )
            for p in path_points
        ]
        self._last_direct_track_fallback_ts = time.time()
        self._publish_control_hint(
            near_field_stop=near_field_stop,
            safety_stop=False,
            path_found=path_found,
            recovery_state=recovery_state,
            reason=f"direct_track_fallback:{reason}",
        )
        self._publish_local_path(poses)
        return True

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        core_paths_loaded = False
        if self._core is not None and hasattr(self._core, "paths_loaded"):
            core_paths_loaded = bool(self._core.paths_loaded())
        cmu_py_ready = self._backend == "cmu_py" and self._path_data is not None
        native_running = (
            self._node is not None
            and getattr(self._node, "_process", None) is not None
        )
        info["local_planner"] = {
            **self._backend_status.as_health_fields(),
            "paths_loaded":  self._path_data is not None or core_paths_loaded,
            "terrain_pts":   (self._terrain_points.shape[0]
                              if self._terrain_points is not None else 0),
            "running":       (self._core is not None) or native_running or cmu_py_ready,
            "direct_track_fallback_enabled": self._allow_direct_track_fallback,
            "min_trackable_local_path_m": round(self._min_trackable_local_path_xy, 3),
            "last_direct_track_fallback_age_ms": (
                round((time.time() - self._last_direct_track_fallback_ts) * 1000)
                if self._last_direct_track_fallback_ts > 0
                else None
            ),
            "last_control_hint": dict(self._last_control_hint),
            "last_local_path_points": self._last_local_path_points,
            "last_local_path_span_m": round(self._last_local_path_span_m, 3),
            "effective_params": dict(self._effective_local_planner_params),
        }
        return info
