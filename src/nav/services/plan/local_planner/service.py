"""LocalPlanner - local path planning as a pluggable Module.

Takes terrain map + waypoint + odometry, produces obstacle-free local path.

NAV COMPUTE CONTRACT (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md):
  This module owns the L2 LOCAL PLANNING layer.
    - MAIN INPUT  : terrain_map (+ terrain_map_ext + boundary + added_obstacles)
                    + traversability risk grid.
    - MAIN OUTPUT : local_path (+ control_hint).
    - ALGORITHM   : CMU / nav_kernel point-cloud voxel scoring (NOT costmap
                    rolling optimisation - this is not a DWA/TEB planner).
  `fused_cost`/`costmap` is a GLOBAL-gating signal and is intentionally NOT a
  primary scoring input here. `traversability` is consumed by the native C++
  scorer; `esdf` is still reserved until clearance-aware local scoring lands.

Backends:
  "nanobind" - C++ nav_kernel LocalPlanner via nanobind (full CMU scoring, zero ROS2) [PREFERRED]
  "cmu_py"   - Pure-Python CMU scoring via numpy (slower fallback, no ROS2)
  "simple"   - straight-line path for testing

Usage::

    bp.add(LocalPlanner, backend="nanobind")  # C++ in-process, no ROS2
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any

from nav.services.plan.local_planner.cmu_py import (
    plan_cmu_py_local_path,
    score_cmu_py_paths as _score_paths_numpy,
)
from nav.services.plan.local_planner.geometry import (
    coerce_path_point,
    planning_origin,
    select_corridor_goal,
)
from nav.services.plan.local_planner.models import CmuPyLocalPlannerRequest
from nav.services.plan.local_planner.output import LocalPlannerOutputMixin
from nav.services.plan.local_planner.obstacles import (
    merge_obstacle_clouds,
    traversability_grid_from_payload,
)
from nav.services.frame_transforms import (
    is_map_frame_jump_event,
    transform_xyz_yaw_with_map_odom,
)
from nav.services.plan.contracts import (
    LOCAL_PLANNER_BACKENDS,
    require_local_planner_backend,
)
from nav.services.plan.local_planner.runtime import setup_local_planner_backend
from runtime.backend_status import BackendStatus
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry, Path
from runtime.msgs.numpy_compat import np
from runtime.msgs.sensor import PointCloud2
from runtime.registry import register
from runtime.runtime_interface import TOPICS, topic_default_frame_id
from runtime.runtime_interface import map_frame_id, normalize_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)
_AVAILABLE_LOCAL_PLANNER_BACKENDS = LOCAL_PLANNER_BACKENDS

_MIN_TRACKABLE_LOCAL_PATH_XY = 0.30


@register("local_planner", "simple", description="Explicit straight-line local planner for tests only")
@register("local_planner", "cmu_py", description="Pure-Python CMU local planner scorer")
@register("local_planner", "nanobind", description="C++ nav_kernel LocalPlanner via nanobind")
class LocalPlanner(Module, LocalPlannerOutputMixin, layer=2):
    """Local path planning - obstacle avoidance + path scoring.

    "cmu_py" backend: Pure-Python CMU scorer with optional native-kernel acceleration (no ROS2)
    "simple" backend: straight-line to waypoint (testing)
    """

    runtime_id = "nav.local_planner"

    # -- Inputs --
    odometry:    In[Odometry]
    terrain_map: In[PointCloud2]
    terrain_map_ext: In[PointCloud2]
    traversability: In[dict]
    waypoint:    In[PoseStamped]
    global_path: In[Path]
    clear_path:  In[bool]
    map_odom_tf: In[dict]
    map_frame_jump_event: In[dict]
    boundary:    In[PointCloud2]
    added_obstacles: In[PointCloud2]
    check_obstacle: In[bool]
    esdf:        In[dict]       # ESDF distance field + gradients for smoother scoring

    # -- Outputs --
    local_path: Out[Path]
    control_hint: Out[dict]
    alive:      Out[bool]

    def __init__(self, backend: str = "nanobind", **kw):
        super().__init__(**kw)
        require_local_planner_backend(backend)
        self._backend_status = BackendStatus.configured_as(backend)
        self._backend = backend
        self._robot_pos = [0.0, 0.0, 0.0]
        self._robot_yaw = 0.0
        self._planning_frame_id = (
            normalize_frame_id(kw.get("planning_frame_id")) or map_frame_id()
        )
        self._sensor_offset_x = 0.0
        self._sensor_offset_y = 0.0
        self._latest_waypoint: PoseStamped | None = None
        self._global_path_points: np.ndarray | None = None
        self._path_frame_id = self._planning_frame_id
        self._corridor_lookahead_m = float(kw.get("corridor_lookahead_m", 3.0))
        self._allow_direct_track_fallback = bool(
            kw.get("allow_direct_track_fallback", False)
        )
        self._ignore_near_field_stop = bool(kw.get("ignore_near_field_stop", False))
        self._direct_track_fallback_min_distance_m = float(
            kw.get("direct_track_fallback_min_distance_m", _MIN_TRACKABLE_LOCAL_PATH_XY)
        )
        self._direct_track_fallback_clearance_m = float(
            kw.get("direct_track_fallback_clearance_m", 0.35)
        )
        self._min_trackable_local_path_xy = float(
            kw.get("min_trackable_local_path_m", _MIN_TRACKABLE_LOCAL_PATH_XY)
        )
        self._terrain_points: np.ndarray | None = None
        self._terrain_ext_points: np.ndarray | None = None
        self._traversability: dict[str, Any] = {}
        self._boundary_points: np.ndarray | None = None
        self._added_obstacle_points: np.ndarray | None = None
        self._check_obstacle_enabled = True
        self._esdf_field: np.ndarray | None = None
        self._esdf_resolution: float = 0.2
        self._esdf_origin: np.ndarray | None = None
        self._last_cmu_py_time: float = 0.0

        # cmu_py state
        self._path_data: dict | None = None
        self._native_kernel = None

        # nanobind state
        self._core = None  # lingtu_nav_kernel.LocalPlanner

        # One-shot warning flags
        self._warned_no_core: bool = False
        self._warned_no_path_data: bool = False
        self._last_control_hint: dict[str, Any] = {}
        self._last_local_path_points: int = 0
        self._last_local_path_span_m: float = 0.0
        self._last_direct_track_fallback_ts: float = 0.0
        self._last_result_diagnostics: dict[str, Any] = {}
        self._last_traversability_grid_status: dict[str, Any] = {}
        self._input_frames: dict[str, str] = {}
        self._map_odom_tf: dict[str, Any] | None = None
        self._last_odometry_transform_status: dict[str, Any] = {}
        self._last_frame_status: dict[str, Any] = {
            "planning_frame": self._planning_frame_id,
            "ok": True,
            "inputs": {},
        }
        self._effective_local_planner_params: dict[str, Any] = {}

        # W2-6: cmu_py grid parameters are pulled from config at setup() if the
        # `local_planner_grid` section is present, otherwise keep CMU defaults.
        # Exposing them as instance attrs makes them unit-testable and lets the
        # stack factory override per-robot without touching magic numbers.
        self._grid_voxel_size = 0.02
        self._grid_voxel_offset_x = 3.2
        self._grid_voxel_offset_y = 5.25
        self._grid_search_radius = 0.45

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.terrain_map.subscribe(self._on_terrain)
        self.terrain_map.set_policy("latest")
        self.terrain_map_ext.subscribe(self._on_terrain_ext)
        self.terrain_map_ext.set_policy("latest")
        self.traversability.subscribe(self._on_traversability)
        self.traversability.set_policy("latest")
        self.waypoint.subscribe(self._on_waypoint)
        self.global_path.subscribe(self._on_global_path)
        self.clear_path.subscribe(self._on_clear_path)
        self.map_odom_tf.subscribe(self._on_map_odom_tf)
        self.map_frame_jump_event.subscribe(self._on_map_frame_jump)
        self.boundary.subscribe(self._on_boundary)
        self.boundary.set_policy("latest")
        self.added_obstacles.subscribe(self._on_added_obstacles)
        self.added_obstacles.set_policy("latest")
        self.check_obstacle.subscribe(self._on_check_obstacle)
        self.esdf.subscribe(self._on_esdf)
        self.esdf.set_policy("latest")

        self._setup_backend_runtime(self._backend)

    def _setup_backend_runtime(self, backend: str) -> None:
        runtime = setup_local_planner_backend(
            backend,
            status=self._backend_status,
        )
        self._backend = runtime.backend
        self._backend_status = runtime.status
        self._core = runtime.core
        self._native_kernel = runtime.nav_kernel
        self._path_data = runtime.path_data
        self._effective_local_planner_params = runtime.effective_params
        self._grid_voxel_size = runtime.grid_config.voxel_size
        self._grid_voxel_offset_x = runtime.grid_config.x_offset
        self._grid_voxel_offset_y = runtime.grid_config.y_offset
        self._grid_search_radius = runtime.grid_config.search_radius
        self._sensor_offset_x = float(
            runtime.effective_params.get("sensor_offset_x", self._sensor_offset_x)
        )
        self._sensor_offset_y = float(
            runtime.effective_params.get("sensor_offset_y", self._sensor_offset_y)
        )

    def _setup_nanobind(self) -> None:
        """Compatibility hook for tests and field diagnostics."""

        self._setup_backend_runtime("nanobind")

    # ------------------------------------------------------------------ #
    # Module lifecycle                                                     #
    # ------------------------------------------------------------------ #

    def start(self):
        super().start()
        self.alive.publish(True)

    def stop(self):
        self._core = None
        self.alive.publish(False)
        super().stop()

    def _record_input_frame(self, source: str, frame_id: Any) -> str:
        normalized = normalize_frame_id(frame_id) or self._planning_frame_id
        self._input_frames[source] = normalized
        self._refresh_frame_status()
        return normalized

    def _refresh_frame_status(self) -> dict[str, Any]:
        mismatches = {
            source: frame
            for source, frame in self._input_frames.items()
            if frame != self._planning_frame_id
        }
        status: dict[str, Any] = {
            "planning_frame": self._planning_frame_id,
            "ok": not mismatches,
            "inputs": dict(self._input_frames),
        }
        if mismatches:
            status["mismatches"] = mismatches
        self._last_frame_status = status
        return status

    def _ensure_frames_for_planning(self) -> bool:
        if self._latest_waypoint is not None:
            self._record_input_frame(
                "waypoint", getattr(self._latest_waypoint, "frame_id", None)
            )
        status = self._refresh_frame_status()
        if status["ok"]:
            return True
        self._last_result_diagnostics = {
            "backend": self._backend,
            "path_found": False,
            "reason": "frame_mismatch",
            "frame_status": dict(status),
        }
        self._publish_control_hint(
            safety_stop=True,
            path_found=False,
            reason="frame_mismatch",
        )
        self._publish_local_path([])
        return False

    def _on_map_odom_tf(self, msg: dict) -> None:
        if isinstance(msg, dict) and msg.get("valid") is not False:
            self._map_odom_tf = dict(msg)
        else:
            self._map_odom_tf = None

    # ------------------------------------------------------------------ #
    # Input handlers                                                       #
    # ------------------------------------------------------------------ #

    def _on_odom(self, odom: Odometry):
        source_frame = normalize_frame_id(getattr(odom, "frame_id", None))
        pos, yaw, output_frame, transformed, reason = transform_xyz_yaw_with_map_odom(
            [odom.x, odom.y, getattr(odom, "z", 0.0)],
            getattr(odom, "yaw", 0.0),
            source_frame=source_frame,
            target_frame=self._planning_frame_id,
            map_odom_tf=self._map_odom_tf,
        )
        self._record_input_frame("odometry", output_frame or source_frame)
        self._last_odometry_transform_status = {
            "source_frame": source_frame,
            "target_frame": self._planning_frame_id,
            "output_frame": output_frame,
            "transformed": transformed,
            "reason": reason,
        }
        self._robot_pos = pos
        self._robot_yaw = float(yaw if yaw is not None else getattr(odom, "yaw", 0.0))

        if self._backend == "nanobind" and self._latest_waypoint is not None:
            if not self._ensure_frames_for_planning():
                return
            if self._core is not None:
                self._run_nanobind(odom.ts if hasattr(odom, "ts") else time.time())
            else:
                if not self._warned_no_core:
                    logger.error(
                        "LocalPlanner: nanobind _core is None - "
                        "not publishing any path. This should never happen "
                        "if setup() succeeded.")
                    self._warned_no_core = True
                return
        elif self._backend == "cmu_py" and self._latest_waypoint is not None:
            if not self._ensure_frames_for_planning():
                return
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
        self._record_input_frame("esdf", data.get("frame_id"))

    def _on_terrain(self, cloud: PointCloud2):
        """Store terrain obstacle points for local planning."""
        self._record_input_frame("terrain_map", getattr(cloud, "frame_id", None))
        if cloud.points is not None:
            self._terrain_points = cloud.points

    def _on_terrain_ext(self, cloud: PointCloud2):
        """Store extended terrain obstacle points for local planning."""
        self._record_input_frame("terrain_map_ext", getattr(cloud, "frame_id", None))
        if cloud.points is not None:
            self._terrain_ext_points = cloud.points

    def _on_traversability(self, data: dict) -> None:
        """Store terrain traversability status for local safety diagnostics."""
        self._traversability = dict(data) if isinstance(data, dict) else {}
        if isinstance(data, dict) and data.get("frame_id"):
            self._record_input_frame("traversability", data.get("frame_id"))

    def _traversability_summary(self) -> dict[str, Any]:
        data = self._traversability
        if not data:
            return {}
        return {
            key: data[key]
            for key in (
                "traversability_class",
                "class",
                "status",
                "risk_max",
                "risk_mean",
            )
            if key in data
        }

    def _on_boundary(self, cloud: PointCloud2):
        """Store geofence boundary points (treated as hard obstacles)."""
        self._record_input_frame("boundary", getattr(cloud, "frame_id", None))
        if cloud.points is not None:
            self._boundary_points = cloud.points

    def _on_added_obstacles(self, cloud: PointCloud2):
        """Store externally injected obstacle points."""
        self._record_input_frame("added_obstacles", getattr(cloud, "frame_id", None))
        if cloud.points is not None:
            self._added_obstacle_points = cloud.points

    def _on_check_obstacle(self, enabled: bool) -> None:
        self._check_obstacle_enabled = bool(enabled)

    def _coerce_path_point(self, point: Any) -> np.ndarray | None:
        return coerce_path_point(point)

    def _on_global_path(self, path: Any) -> None:
        frame_id = self._record_input_frame("global_path", getattr(path, "frame_id", None))
        if frame_id == self._planning_frame_id:
            self._path_frame_id = frame_id
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
        if is_map_frame_jump_event(event):
            self._clear_local_plan()

    def _waypoint_goal(self, wp: PoseStamped) -> np.ndarray:
        return np.array([
            wp.pose.position.x,
            wp.pose.position.y,
            wp.pose.position.z,
        ], dtype=float)

    def _effective_goal(self, wp: PoseStamped) -> np.ndarray:
        return self._select_corridor_goal(self._waypoint_goal(wp))

    def _planning_origin(self) -> np.ndarray:
        return planning_origin(
            self._robot_pos,
            self._robot_yaw,
            self._sensor_offset_x,
            self._sensor_offset_y,
        )

    def _select_corridor_goal(self, fallback_goal: np.ndarray) -> np.ndarray:
        return select_corridor_goal(
            self._global_path_points,
            self._planning_origin(),
            fallback_goal,
            self._corridor_lookahead_m,
        )

    def _on_waypoint(self, wp: PoseStamped):
        """Store waypoint; simple backend generates path immediately."""
        self._latest_waypoint = wp
        if wp is not None:
            frame_id = self._record_input_frame("waypoint", getattr(wp, "frame_id", None))
            if frame_id == self._planning_frame_id:
                self._path_frame_id = frame_id

        if self._backend == "simple" and wp is not None:
            if not self._ensure_frames_for_planning():
                return
            goal = self._effective_goal(wp)
            path_points = self._straight_line(self._planning_origin(), goal, step=0.5)
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

    def _merge_obstacle_clouds(self, *, include_traversability: bool = True) -> np.ndarray:
        params = self._effective_local_planner_params
        traversability_range_m = float(
            params.get("path_range", params.get("adjacent_range", 3.5))
        )
        return merge_obstacle_clouds(
            terrain_points=self._terrain_points,
            terrain_ext_points=self._terrain_ext_points,
            traversability=self._traversability if include_traversability else None,
            robot_position=self._robot_pos,
            traversability_range_m=traversability_range_m,
            boundary_points=self._boundary_points,
            added_obstacle_points=self._added_obstacle_points,
            check_obstacle_enabled=self._check_obstacle_enabled,
        )

    def _traversability_grid(self) -> tuple[np.ndarray, float, np.ndarray] | None:
        return traversability_grid_from_payload(self._traversability)

    def _sync_core_traversability_grid(self) -> None:
        if self._core is None or not hasattr(self._core, "set_traversability_grid"):
            self._last_traversability_grid_status = {
                "native": False,
                "reason": "core_has_no_grid_port",
            }
            return
        payload = self._traversability_grid()
        if payload is None:
            if hasattr(self._core, "clear_traversability_grid"):
                self._core.clear_traversability_grid()
            self._last_traversability_grid_status = {
                "native": True,
                "active": False,
                "reason": "no_valid_grid",
            }
            return
        grid, resolution, origin = payload
        native_grid = np.ascontiguousarray(grid, dtype=np.float32)
        self._core.set_traversability_grid(
            native_grid,
            float(resolution),
            float(origin[0]),
            float(origin[1]),
        )
        self._last_traversability_grid_status = {
            "native": True,
            "active": True,
            "rows": int(native_grid.shape[0]),
            "cols": int(native_grid.shape[1]),
            "resolution": round(float(resolution), 4),
            "origin": [round(float(origin[0]), 4), round(float(origin[1]), 4)],
            "hard_cost": float(
                self._effective_local_planner_params.get(
                    "traversability_hard_cost", 90.0
                )
            ),
            "soft_cost": float(
                self._effective_local_planner_params.get(
                    "traversability_soft_cost", 40.0
                )
            ),
        }

    def _record_native_grid_status(
        self,
        grid: np.ndarray | None,
        resolution: float = 0.0,
        origin: np.ndarray | None = None,
    ) -> None:
        if grid is None or origin is None:
            self._last_traversability_grid_status = {
                "native": True,
                "active": False,
                "reason": "no_valid_grid",
            }
            return
        self._last_traversability_grid_status = {
            "native": True,
            "active": True,
            "rows": int(grid.shape[0]),
            "cols": int(grid.shape[1]),
            "resolution": round(float(resolution), 4),
            "origin": [round(float(origin[0]), 4), round(float(origin[1]), 4)],
            "hard_cost": float(
                self._effective_local_planner_params.get(
                    "traversability_hard_cost", 90.0
                )
            ),
            "soft_cost": float(
                self._effective_local_planner_params.get(
                    "traversability_soft_cost", 40.0
                )
            ),
        }

    def _run_native_plan_frame(
        self,
        goal: np.ndarray,
        obs_flat: np.ndarray,
        timestamp: float,
    ):
        if self._core is None or not hasattr(self._core, "plan_frame"):
            return None
        payload = self._traversability_grid()
        if payload is None:
            self._record_native_grid_status(None)
            if not hasattr(self._core, "plan_frame_without_grid"):
                return None
            return self._core.plan_frame_without_grid(
                self._robot_pos[0],
                self._robot_pos[1],
                self._robot_pos[2],
                self._robot_yaw,
                float(goal[0]),
                float(goal[1]),
                obs_flat,
                timestamp,
            )
        grid, resolution, origin = payload
        native_grid = np.ascontiguousarray(grid, dtype=np.float32)
        self._record_native_grid_status(native_grid, float(resolution), origin)
        return self._core.plan_frame(
            self._robot_pos[0],
            self._robot_pos[1],
            self._robot_pos[2],
            self._robot_yaw,
            float(goal[0]),
            float(goal[1]),
            native_grid,
            float(resolution),
            float(origin[0]),
            float(origin[1]),
            obs_flat,
            timestamp,
        )

    def _run_nanobind(self, timestamp: float):
        """Run C++ nav_kernel nav.local_planner.plan() and publish result."""
        wp = self._latest_waypoint
        if wp is None:
            return
        if not self._ensure_frames_for_planning():
            return
        if self._core is None:
            if not self._warned_no_core:
                logger.error("LocalPlanner: _core lost at runtime - not publishing path")
                self._warned_no_core = True
            return

        goal = self._effective_goal(wp)

        merged = self._merge_obstacle_clouds(
            include_traversability=not (
                hasattr(self._core, "plan_frame")
                or hasattr(self._core, "set_traversability_grid")
            )
        )
        obs_flat = np.ascontiguousarray(merged, dtype=np.float32).ravel()

        result = self._run_native_plan_frame(goal, obs_flat, timestamp)
        if result is None:
            self._core.set_vehicle(
                self._robot_pos[0], self._robot_pos[1], self._robot_pos[2],
                self._robot_yaw)
            self._core.set_goal(float(goal[0]), float(goal[1]))
            self._sync_core_traversability_grid()
            result = self._core.plan(obs_flat, timestamp)
        result_path = list(getattr(result, "path", []) or [])
        path_found = bool(getattr(result, "path_found", True))
        near_field_stop = bool(getattr(result, "near_field_stop", False))
        recovery_state = int(getattr(result, "recovery_state", 0) or 0)
        slow_down = int(getattr(result, "slow_down", 0) or 0)
        xy_length, xy_span = self._result_path_xy_metrics(result_path)
        self._last_result_diagnostics = {
            "backend": "nanobind",
            "timestamp": float(timestamp),
            "path_point_count": len(result_path),
            "path_length_m": round(float(xy_length), 3),
            "path_span_m": round(float(xy_span), 3),
            "path_found": path_found,
            "near_field_stop": near_field_stop,
            "recovery_state": recovery_state,
            "slow_down": slow_down,
            "effective_goal": [
                round(float(goal[0]), 3),
                round(float(goal[1]), 3),
                round(float(goal[2]), 3) if len(goal) >= 3 else 0.0,
            ],
            "robot_pos": [
                round(float(self._robot_pos[0]), 3),
                round(float(self._robot_pos[1]), 3),
                round(float(self._robot_pos[2]), 3)
                if len(self._robot_pos) >= 3
                else 0.0,
            ],
            "robot_yaw": round(float(self._robot_yaw), 3),
            "obstacle_point_count": int(merged.shape[0]),
            "traversability": self._traversability_summary(),
            "traversability_grid": dict(self._last_traversability_grid_status),
        }
        self._publish_control_hint(
            slow_down=slow_down,
            near_field_stop=near_field_stop,
            path_found=path_found,
            recovery_state=recovery_state,
            reason="nanobind",
        )

        if not result_path:
            if self._publish_direct_track_fallback(
                near_field_stop=near_field_stop,
                path_found=path_found,
                recovery_state=recovery_state,
                reason="no_local_path",
            ):
                return
            self._publish_control_hint(
                slow_down=slow_down,
                near_field_stop=near_field_stop,
                path_found=path_found,
                recovery_state=recovery_state,
                safety_stop=True,
                reason="no_local_path",
            )
            self._publish_local_path([])
            return

        trackable = (
            len(result_path) >= 2
            and max(xy_length, xy_span) >= self._min_trackable_local_path_xy
            and (path_found or recovery_state in (1, 2))
        )
        if not trackable:
            if self._publish_direct_track_fallback(
                near_field_stop=near_field_stop,
                path_found=path_found,
                recovery_state=recovery_state,
                reason="untrackable_local_path",
            ):
                return
            self._publish_control_hint(
                slow_down=slow_down,
                near_field_stop=near_field_stop,
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
        origin = self._planning_origin()
        for v in result_path:
            wx = v.x * cos_yaw - v.y * sin_yaw + origin[0]
            wy = v.x * sin_yaw + v.y * cos_yaw + origin[1]
            wz = v.z + origin[2]
            poses.append(PoseStamped(
                pose=Pose(
                    position=Vector3(wx, wy, wz),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id=self._path_frame_id,
            ))
        self._publish_local_path(poses)

    # ------------------------------------------------------------------ #
    # CMU Python adapter bridge                                                    #
    # ------------------------------------------------------------------ #

    def _run_cmu_py(self):
        """Run the CMU local planner scoring algorithm and publish best path."""
        wp = self._latest_waypoint
        if wp is None:
            return
        if not self._ensure_frames_for_planning():
            return
        if self._path_data is None:
            if not self._warned_no_path_data:
                logger.error(
                    "LocalPlanner [cmu_py]: _path_data is None - "
                    "not publishing any path. This should never happen "
                    "if setup() succeeded.")
                self._warned_no_path_data = True
            return

        goal = self._effective_goal(wp)
        params = self._effective_local_planner_params
        traversability_grid = self._traversability_grid()
        if traversability_grid is None:
            trav_grid = None
            trav_resolution = 0.0
            trav_origin = None
        else:
            trav_grid, trav_resolution, trav_origin = traversability_grid
        decision = plan_cmu_py_local_path(
            CmuPyLocalPlannerRequest(
                path_data=self._path_data,
                robot_pos=self._robot_pos.copy(),
                robot_yaw=self._robot_yaw,
                goal=goal,
                obstacle_points_world=self._merge_obstacle_clouds(),
                sensor_offset_x=self._sensor_offset_x,
                sensor_offset_y=self._sensor_offset_y,
                grid_voxel_size=self._grid_voxel_size,
                grid_voxel_offset_x=self._grid_voxel_offset_x,
                grid_voxel_offset_y=self._grid_voxel_offset_y,
                grid_search_radius=self._grid_search_radius,
                vehicle_length=float(params.get("vehicle_length", 0.6)),
                vehicle_width=float(params.get("vehicle_width", 0.6)),
                obstacle_height_thre=float(params.get("obstacle_height_thre", 0.2)),
                ground_height_thre=float(params.get("ground_height_thre", 0.1)),
                point_per_path_thre=int(params.get("point_per_path_thre", 2)),
                dir_weight=float(params.get("dir_weight", 0.02)),
                dir_thre=float(params.get("dir_thre", 90.0)),
                path_range=float(params.get("path_range", 3.5)),
                slope_weight=float(params.get("slope_weight", 0.0)),
                use_cost=bool(params.get("use_cost", False)),
                check_rot_obstacle=bool(params.get("check_rot_obstacle", False)),
                two_way_drive=bool(params.get("two_way_drive", True)),
                traversability_grid=trav_grid,
                traversability_resolution=float(trav_resolution),
                traversability_origin=trav_origin,
                use_traversability_cost=bool(params.get("use_traversability_cost", True)),
                traversability_hard_cost=float(params.get("traversability_hard_cost", 90.0)),
                traversability_soft_cost=float(params.get("traversability_soft_cost", 40.0)),
                traversability_weight=float(params.get("traversability_weight", 0.01)),
            )
        )
        if decision is None:
            return
        self._last_result_diagnostics = {
            "backend": "cmu_py",
            "path_point_count": len(decision.world_path),
            "path_found": decision.path_found,
            "selected_group_id": decision.selected_group_id,
            "selected_rot_dir": decision.selected_rot_dir,
            "selected_path_group": decision.selected_path_group,
            "score": round(float(decision.score), 3),
            "effective_goal": [
                round(float(goal[0]), 3),
                round(float(goal[1]), 3),
                round(float(goal[2]), 3) if len(goal) >= 3 else 0.0,
            ],
            "obstacle_point_count": decision.obstacle_point_count,
            "traversability": self._traversability_summary(),
        }
        if not decision.path_found:
            self._publish_control_hint(
                safety_stop=decision.safety_stop,
                path_found=decision.path_found,
                reason=decision.reason,
            )
            self._publish_local_path([])
            return

        self._publish_local_path(
            [
                PoseStamped(
                    pose=Pose(
                        position=Vector3(x, y, z),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id=self._path_frame_id,
                )
                for x, y, z in decision.world_path
            ]
        )
        self._publish_control_hint(reason=decision.reason)
        return






