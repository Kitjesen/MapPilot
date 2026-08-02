"""PCT global planner."""

from __future__ import annotations

import logging
import os
import traceback

from nav.services.plan.contracts import GlobalPlanRequest, GlobalPlanResult
from runtime.msgs.numpy_compat import np
from runtime.registry import register

from .runtime.api import PCT_DEFAULT_PLANNER_RUNTIME, load_pct_planner_runtime

logger = logging.getLogger(__name__)

_PCT_HEIGHT_SNAP_TOLERANCE_M = 0.25
_PCT_START_XY_SNAP_TOLERANCE_M = 0.5
_PCT_GOAL_XY_TOLERANCE_M = 0.35
_PCT_PLANNER_PATH_MODES = {
    "astar_raw_path": "astar_raw_path",
    "native_astar_raw_path": "astar_raw_path",
    "rust_astar_raw_path": "astar_raw_path",
    "optimized_trajectory": "optimized_trajectory",
    "native_optimized_trajectory": "optimized_trajectory",
    "rust_optimized_trajectory": "optimized_trajectory",
}


def _canonical_pct_planner_path_mode(value: object) -> str:
    raw_mode = str(value or "").strip()
    try:
        return _PCT_PLANNER_PATH_MODES[raw_mode]
    except KeyError as exc:
        raise ValueError(
            f"unsupported PCT planner path mode: {raw_mode!r}"
        ) from exc


# ---------------------------------------------------------------------------
# PCT planner: native terrain-aware planner
# ---------------------------------------------------------------------------

@register("planner_backend", "pct",
          description="PCT tomogram global planner")
class PCTPlanner:
    """PCT planner wrapper around the selected runtime.

    Capabilities:
      - 3D A* on multi-slice traversability grid (stairs, ramps)
      - GPMP trajectory optimisation (smooth, kinematically feasible)
      - Gradient-aware elevation planning

    The default runtime is Rust process/FFI. The legacy native .so path remains
    explicit parity infrastructure only.
    """

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9):
        self._planner = None
        self._tomogram_path = tomogram_path
        self._obstacle_thr = obstacle_thr
        self._available = False
        self._load_error: str = ""
        self._last_plan_error: str = ""
        self._last_plan_diagnostics: dict[str, object] = {}
        self._last_plan_reached_goal: bool = False
        self._last_plan_path_mode: str = ""
        self._planner_runtime_name: str = PCT_DEFAULT_PLANNER_RUNTIME

        # 2D ground-floor grid for _find_safe_goal BFS (extracted from tomogram)
        self._grid: np.ndarray | None = None
        self._trav_3d: np.ndarray | None = None
        self._elev_3d: np.ndarray | None = None
        self._grid_is_projection: bool = False
        self._resolution: float = 0.2
        self._origin = [0.0, 0.0]
        self._slice_h0: float = 0.0
        self._slice_dh: float = 0.5

        # Live costmap overlay for dynamic obstacle avoidance
        self._costmap: np.ndarray | None = None
        self._costmap_resolution: float = 0.2
        self._costmap_origin = [0.0, 0.0]

        self._try_load(tomogram_path)

    def _try_load(self, tomogram_path: str) -> None:
        """Try to import TomogramPlanner and load the tomogram.

        Sets self._available = True only when both the .so and the tomogram
        are successfully loaded.
        """
        tomogram_path = os.path.abspath(tomogram_path)

        if not os.path.exists(tomogram_path):
            self._load_error = (
                f"Tomogram file not found: '{tomogram_path}'. "
                f"Set tomogram= in your profile or copy the map to "
                f"~/data/inovxio/data/maps/active/tomogram.pickle"
            )
            logger.error("PCT planner: %s", self._load_error)
            return

        try:
            runtime = load_pct_planner_runtime(
                tomogram_path,
                obstacle_thr=self._obstacle_thr,
            )
            planner = runtime.planner
            runtime_paths = runtime.runtime_paths
            self._planner = planner
            self._planner_runtime_name = runtime.name
            self._available = True
            logger.info(
                "PCT ele_planner loaded: %s runtime=%s lib_dir=%s map_dim=%s slices=%s",
                tomogram_path,
                runtime.name,
                runtime_paths.lib_dir if runtime_paths is not None else "?",
                getattr(planner, "map_dim", "?"),
                getattr(planner, "n_slice", "?"),
            )
            self._extract_grid(tomogram_path)
        except Exception as e:
            self._load_error = f"PCT runtime/loadTomogram failed: {e}"
            logger.exception("PCT planner failed to load tomogram: %s", tomogram_path)

    @property
    def available(self) -> bool:
        return self._available

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        path = self.plan(request.start, request.goal)
        return GlobalPlanResult(
            path=path,
            reached_goal=self._last_plan_reached_goal,
            error=self._last_plan_error,
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            diagnostics=dict(self._last_plan_diagnostics),
        )

    @staticmethod
    def _env_truthy(name: str, default: bool) -> bool:
        value = os.environ.get(name)
        if value is None:
            return bool(default)
        text = value.strip().lower()
        if text == "":
            return bool(default)
        return text not in {"0", "false", "no", "off"}

    def _extract_grid(self, tomogram_path: str) -> None:
        """Extract 2D ground-floor traversability grid from the tomogram pickle.

        This grid is used by GlobalPlanner._find_safe_goal() BFS to
        verify goals are in free space before sending them to the 3D planner.
        """
        import pickle
        try:
            with open(tomogram_path, "rb") as f:
                raw = pickle.load(f)  # noqa: S301  # trusted local tomogram  # noqa: S301  # trusted local tomogram
        except Exception:
            logger.warning("PCT: could not load tomogram pickle for grid extraction", exc_info=True)
            return

        if not isinstance(raw, dict):
            return

        tomo_data = raw.get("data")
        grid_info = raw.get("grid_info") or {}
        axis_order = str(grid_info.get("axis_order") or "")
        res = float(raw.get("resolution", 0.2))
        self._slice_h0 = float(raw.get("slice_h0", 0.0))
        self._slice_dh = float(raw.get("slice_dh", 0.5))

        if tomo_data is not None and hasattr(tomo_data, "ndim") and tomo_data.ndim == 4:
            # data shape: (5, n_slices, H, W), channel 0 = traversability
            trav_3d = np.asarray(tomo_data[0], dtype=np.float32)
            transpose_xy = axis_order in {"builder_xy", "xy"} or (
                not axis_order and "slice_h0" in raw and "slice_dh" in raw
            )
            if transpose_xy:
                trav_3d = np.transpose(trav_3d, (0, 2, 1))
            self._trav_3d = trav_3d
            elev_3d = np.asarray(tomo_data[3], dtype=np.float32) if len(tomo_data) > 3 else None
            if elev_3d is not None and transpose_xy:
                elev_3d = np.transpose(elev_3d, (0, 2, 1))
            self._elev_3d = elev_3d
            self._grid = np.nanmin(self._trav_3d, axis=0).astype(np.float32)
            self._grid_is_projection = True
            if raw.get("origin") is not None:
                self._origin = np.array(raw.get("origin", [0, 0])[:2], dtype=np.float64)
            else:
                center = np.array(raw.get("center", [0, 0])[:2], dtype=np.float64)
                h, w = self._grid.shape
                self._origin = center - np.array([w * res / 2, h * res / 2])
        else:
            grid = raw.get("grid", raw.get("traversability"))
            if grid is not None:
                self._grid = np.asarray(grid, dtype=np.float32)
            self._trav_3d = None
            self._elev_3d = None
            self._grid_is_projection = False
            self._origin = np.array(raw.get("origin", [0, 0])[:2], dtype=np.float64)

        self._resolution = res
        self._static_grid: np.ndarray | None = None
        if self._grid is not None:
            self._static_grid = self._grid.copy()
            logger.info(
                "PCT: extracted 2D grid for goal safety BFS: shape=%s res=%.3f",
                self._grid.shape, res,
            )

    def update_map(self, grid: np.ndarray, resolution: float = 0.2,
                   origin: np.ndarray | None = None) -> None:
        """Accept live costmap for dynamic obstacle checking in _find_safe_goal.

        The costmap is stored separately and merged with the static tomogram
        grid when GlobalPlanner calls _find_safe_goal(). The 3D PCT
        planner itself still uses the pre-built tomogram for path planning.
        """
        self._costmap = np.asarray(grid, dtype=np.float32)
        self._costmap_resolution = resolution
        if origin is not None:
            self._costmap_origin = np.array(origin[:2], dtype=np.float64)

        # Merge costmap obstacles into _grid so _find_safe_goal sees them
        if self._grid is not None:
            self._merge_costmap()

    def _merge_costmap(self) -> None:
        """Overlay dynamic costmap obstacles onto the static tomogram grid.

        Resets _grid from _static_grid first, then overlays costmap obstacles.
        Works even when costmap and tomogram have different resolutions/origins.
        """
        if self._costmap is None or self._static_grid is None:
            return

        # Reset to clean static grid before merging
        self._grid = self._static_grid.copy()

        cm = self._costmap
        cm_res = self._costmap_resolution
        cm_ox, cm_oy = self._costmap_origin[0], self._costmap_origin[1]
        g_res = self._resolution
        g_ox, g_oy = self._origin[0], self._origin[1]
        gh, gw = self._grid.shape
        _ch, _cw = cm.shape

        # Find costmap cells that are obstacles
        obs_rows, obs_cols = np.where(cm >= self._obstacle_thr)
        if len(obs_rows) == 0:
            return

        # Convert costmap obstacle cells to world coords, then to grid coords
        world_x = cm_ox + obs_cols * cm_res
        world_y = cm_oy + obs_rows * cm_res
        grid_cols = np.round((world_x - g_ox) / g_res).astype(int)
        grid_rows = np.round((world_y - g_oy) / g_res).astype(int)

        # Filter in-bounds and mark obstacles
        mask = (grid_cols >= 0) & (grid_cols < gw) & (grid_rows >= 0) & (grid_rows < gh)
        valid_rows = grid_rows[mask]
        valid_cols = grid_cols[mask]
        if len(valid_rows) > 0:
            self._grid[valid_rows, valid_cols] = np.maximum(
                self._grid[valid_rows, valid_cols],
                cm[obs_rows[mask], obs_cols[mask]],
            )

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        """Plan a 3D path from start to goal.

        Args:
            start: world coords [x, y, z]
            goal:  world coords [x, y, z]

        Returns:
            List of (x, y, z) tuples; empty list means planning failed.
            Caller (GlobalPlanner) raises RuntimeError on empty return.
        """
        self._last_plan_error = ""
        self._last_plan_reached_goal = False
        self._last_plan_path_mode = ""
        self._last_plan_diagnostics = {
            "planner": "pct",
            "tomogram": getattr(self, "_tomogram_path", ""),
            "pct_planner_runtime": {
                "runtime": getattr(
                    self,
                    "_planner_runtime_name",
                    PCT_DEFAULT_PLANNER_RUNTIME,
                ),
                "ok": bool(
                    getattr(self, "_available", False)
                    and getattr(self, "_planner", None) is not None
                ),
            },
            "reload_on_native_exception": self._env_truthy(
                "LINGTU_PCT_RELOAD_ON_EXCEPTION",
                True,
            ),
        }
        if self._planner is None:
            logger.error(
                "PCT planner unavailable (%s); cannot plan. "
                "Is this running with native PCT .so files matching this arch/Python ABI?",
                self._load_error,
            )
            self._last_plan_error = "pct planner unavailable"
            self._last_plan_diagnostics["available"] = False
            self._last_plan_diagnostics["load_error"] = self._load_error
            return []

        start_pos = np.asarray(start[:2], dtype=np.float64)
        raw_start_pos = start_pos.copy()
        goal_pos  = np.asarray(goal[:2],  dtype=np.float64)
        start_pos, start_projection = self._project_start_to_traversable_xy(
            start_pos,
            max_snap_m=_PCT_START_XY_SNAP_TOLERANCE_M,
        )
        start_projection_fields = self._start_projection_plan_fields(start_projection)
        if start_projection.get("status") == "failed":
            self._last_plan_error = str(
                start_projection.get("reason", "start_xy_projection_failed")
            )
            self._last_plan_diagnostics.update(
                {
                    "available": True,
                    "start_xyz": np.asarray(start[:3], dtype=float).tolist(),
                    "goal_xyz": np.asarray(goal[:3], dtype=float).tolist(),
                    "start_xy_raw": raw_start_pos.tolist(),
                    "start_xy": start_pos.tolist(),
                    "start_projection": start_projection,
                    "goal_xy": goal_pos.tolist(),
                    "stage": "start_xy_projection",
                    "error_message": self._last_plan_error,
                    **start_projection_fields,
                }
            )
            logger.warning(
                "PCT rejected start before planning: start=%s goal=%s reason=%s",
                start,
                goal,
                self._last_plan_error,
            )
            return []
        # Choose heights that land on traversable tomogram slices at each XY.
        # A raw 2-D goal often arrives with z=0, while get_surface_height()
        # can snap to an upper slice whose traversability is a hard barrier.
        try:
            start_h = self._select_traversable_height(
                start_pos,
                float(start[2]) if len(start) > 2 else 0.0,
                max_snap_m=_PCT_HEIGHT_SNAP_TOLERANCE_M,
                reject_out_of_bounds=True,
            )
            raw_goal_z = float(goal[2]) if len(goal) > 2 else float("nan")
            planar_goal_height = (
                not np.isfinite(raw_goal_z)
                or (
                    abs(raw_goal_z) <= 1e-6
                    and np.isfinite(start_h)
                    and abs(float(start_h)) > 1e-6
                )
            )
            goal_height_fallback = float(start_h) if planar_goal_height else raw_goal_z
            goal_h = self._select_traversable_height(
                goal_pos,
                goal_height_fallback,
                max_snap_m=_PCT_HEIGHT_SNAP_TOLERANCE_M,
                reject_out_of_bounds=True,
            )
        except ValueError as exc:
            self._last_plan_error = str(exc)
            self._last_plan_diagnostics.update(
                {
                    "available": True,
                    "start_xyz": np.asarray(start[:3], dtype=float).tolist(),
                    "goal_xyz": np.asarray(goal[:3], dtype=float).tolist(),
                    "start_xy_raw": raw_start_pos.tolist(),
                    "start_xy": start_pos.tolist(),
                    "start_projection": start_projection,
                    **start_projection_fields,
                    "goal_requested_height": (
                        float(goal[2]) if len(goal) > 2 else None
                    ),
                    "goal_height_reference": locals().get(
                        "goal_height_fallback",
                        None,
                    ),
                    "goal_height_source": (
                        "start_height_for_planar_goal"
                        if locals().get("planar_goal_height", False)
                        else "goal_z"
                    ),
                    "error_type": exc.__class__.__name__,
                    "error_message": str(exc),
                    "stage": "height_selection",
                }
            )
            logger.warning(
                "PCT rejected start/goal before planning: start=%s goal=%s reason=%s",
                start,
                goal,
                exc,
            )
            return []
        if self._is_near_zero_route(start_pos, goal_pos):
            self._last_plan_reached_goal = True
            self._last_plan_diagnostics.update(
                {
                    "available": True,
                    "start_xy_raw": raw_start_pos.tolist(),
                    "start_xy": start_pos.tolist(),
                    "start_projection": start_projection,
                    **start_projection_fields,
                    "goal_xy": goal_pos.tolist(),
                    "goal_requested_height": (
                        float(goal[2]) if len(goal) > 2 else None
                    ),
                    "goal_height_reference": locals().get(
                        "goal_height_fallback",
                        None,
                    ),
                    "goal_height_source": (
                        "start_height_for_planar_goal"
                        if locals().get("planar_goal_height", False)
                        else "goal_z"
                    ),
                    "start_height": float(start_h),
                    "goal_height": float(goal_h),
                    "near_zero_route": True,
                }
            )
            logger.info(
                "PCT native plan bypassed for near-zero route: start=%s goal=%s",
                start,
                goal,
            )
            return [
                (float(start_pos[0]), float(start_pos[1]), float(start_h)),
                (float(goal_pos[0]), float(goal_pos[1]), float(goal_h)),
            ]

        self._last_plan_diagnostics.update(
            {
                "available": True,
                "start_xyz": np.asarray(start[:3], dtype=float).tolist(),
                "goal_xyz": np.asarray(goal[:3], dtype=float).tolist(),
                "start_xy_raw": raw_start_pos.tolist(),
                "start_xy": start_pos.tolist(),
                "start_projection": start_projection,
                **start_projection_fields,
                "goal_xy": goal_pos.tolist(),
                "goal_requested_height": (
                    float(goal[2]) if len(goal) > 2 else None
                ),
                "goal_height_reference": float(goal_height_fallback),
                "goal_height_source": (
                    "start_height_for_planar_goal"
                    if planar_goal_height
                    else "goal_z"
                ),
                "start_height": float(start_h),
                "goal_height": float(goal_h),
                "stage": "native_plan",
                "native_retry_count": 0,
            }
        )
        result = self._plan_native_with_reload_retry(
            start_pos,
            goal_pos,
            start_h,
            goal_h,
            start,
            goal,
        )
        if self._wrapper_reports_known_collision():
            self._last_plan_error = "pct planner rejected known-collision path"
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics.update(
                {
                    "stage": "native_plan_collision_rejected",
                    "error_message": self._last_plan_error,
                    "unsafe_path_discarded": True,
                    "goal_reached": False,
                }
            )
            logger.error("%s: start=%s goal=%s", self._last_plan_error, start, goal)
            return []
        if result is None and self._last_plan_error:
            return []

        if result is None or len(result) == 0:
            self._last_plan_error = "pct native plan returned no path"
            self._last_plan_diagnostics.update(
                {
                    "stage": "native_plan_empty",
                    "error_message": self._last_plan_error,
                }
            )
            logger.warning(
                "PCT plan() returned no path: start=%s goal=%s", start, goal
            )
            return []

        # result is np.ndarray (N, 3) in world coords [x, y, z]
        try:
            arr = np.asarray(result, dtype=np.float64)
            if arr.ndim == 2 and arr.shape[1] >= 3:
                path = [(float(p[0]), float(p[1]), float(p[2])) for p in arr]
                self._record_terminal_goal_status(path, goal_pos)
                self._last_plan_diagnostics.update(
                    {
                        "stage": "native_plan_success",
                        "result_shape": list(arr.shape),
                        "path_points": int(arr.shape[0]),
                    }
                )
                return path
            elif arr.ndim == 2 and arr.shape[1] == 2:
                path = [(float(p[0]), float(p[1]), goal_h) for p in arr]
                self._record_terminal_goal_status(path, goal_pos)
                self._last_plan_diagnostics.update(
                    {
                        "stage": "native_plan_success",
                        "result_shape": list(arr.shape),
                        "path_points": int(arr.shape[0]),
                    }
                )
                return path
            else:
                self._last_plan_error = f"unexpected pct result shape: {arr.shape}"
                self._last_plan_diagnostics.update(
                    {
                        "stage": "result_conversion",
                        "error_message": self._last_plan_error,
                        "result_shape": list(arr.shape),
                    }
                )
                logger.error("Unexpected PCT result shape: %s", arr.shape)
                return []
        except Exception:
            self._last_plan_error = "pct result conversion failed"
            self._last_plan_diagnostics.update(
                {
                    "stage": "result_conversion",
                    "error_message": self._last_plan_error,
                }
            )
            logger.exception("PCT result conversion failed")
            return []

    def _record_terminal_goal_status(
        self,
        path: list[tuple[float, float, float]],
        goal_pos: np.ndarray,
    ) -> None:
        tolerance = max(
            _PCT_GOAL_XY_TOLERANCE_M,
            float(getattr(self, "_resolution", 0.2) or 0.2) * 1.5,
        )
        if not path:
            self._last_plan_reached_goal = False
            self._last_plan_diagnostics.update(
                {
                    "goal_reached": False,
                    "goal_terminal_xy": [],
                    "goal_terminal_error_m": None,
                    "goal_terminal_tolerance_m": float(tolerance),
                }
            )
            return

        terminal_xy = np.asarray(path[-1][:2], dtype=float)
        goal_xy = np.asarray(goal_pos[:2], dtype=float)
        distance = float(np.linalg.norm(terminal_xy - goal_xy))
        reached = bool(distance <= tolerance)
        self._last_plan_reached_goal = reached
        self._last_plan_diagnostics.update(
            {
                "goal_reached": reached,
                "goal_terminal_xy": terminal_xy.tolist(),
                "goal_terminal_error_m": round(distance, 4),
                "goal_terminal_tolerance_m": float(tolerance),
            }
        )

    def _record_wrapper_path_diagnostics(self) -> None:
        planner = self._planner
        if planner is None:
            return

        path_mode = str(getattr(planner, "last_path_mode", "") or "")
        if path_mode:
            canonical_path_mode = _canonical_pct_planner_path_mode(path_mode)
            self._last_plan_path_mode = canonical_path_mode
            self._last_plan_diagnostics["last_path_mode"] = path_mode
            self._last_plan_diagnostics["pct_planner_path_mode"] = canonical_path_mode

        if hasattr(planner, "last_optimizer_enabled") or hasattr(planner, "optimize_trajectory"):
            self._last_plan_diagnostics["pct_optimizer_enabled"] = bool(
                getattr(
                    planner,
                    "last_optimizer_enabled",
                    getattr(planner, "optimize_trajectory", False),
                )
            )
        if hasattr(planner, "last_optimizer_attempted"):
            self._last_plan_diagnostics["pct_optimizer_attempted"] = bool(
                getattr(planner, "last_optimizer_attempted")
            )
        if hasattr(planner, "last_optimizer_accepted"):
            accepted = getattr(planner, "last_optimizer_accepted")
            self._last_plan_diagnostics["pct_optimizer_accepted"] = (
                bool(accepted) if accepted in (True, False) else None
            )
        if hasattr(planner, "last_optimizer_reject_reason"):
            self._last_plan_diagnostics["pct_optimizer_reject_reason"] = str(
                getattr(planner, "last_optimizer_reject_reason") or ""
            )
        if hasattr(planner, "last_optimizer_blocked_sample_count"):
            try:
                blocked = int(getattr(planner, "last_optimizer_blocked_sample_count") or 0)
            except (TypeError, ValueError):
                blocked = 0
            self._last_plan_diagnostics["pct_optimizer_blocked_sample_count"] = blocked
        if hasattr(planner, "last_raw_path_blocked_sample_count"):
            try:
                blocked = int(getattr(planner, "last_raw_path_blocked_sample_count") or 0)
            except (TypeError, ValueError):
                blocked = 0
            self._last_plan_diagnostics["pct_optimizer_raw_blocked_sample_count"] = blocked
        for attr, key in (
            ("last_optimizer_elapsed_ms", "pct_optimizer_elapsed_ms"),
            ("last_optimizer_initial_cost", "pct_optimizer_initial_cost"),
            ("last_optimizer_final_cost", "pct_optimizer_final_cost"),
            ("last_optimizer_iterations", "pct_optimizer_iterations"),
            ("last_optimizer_accepted_steps", "pct_optimizer_accepted_steps"),
            ("last_optimizer_linear_solver", "pct_optimizer_linear_solver"),
            ("last_optimizer_linear_solve_fallbacks", "pct_optimizer_linear_solve_fallbacks"),
            ("last_optimizer_input_states", "pct_optimizer_input_states"),
            ("last_raw_path_count", "pct_raw_path_count"),
        ):
            if hasattr(planner, attr):
                self._last_plan_diagnostics[key] = getattr(planner, attr)

    def _wrapper_reports_known_collision(self) -> bool:
        """Reject any wrapper result whose final candidate is known to be blocked."""
        diagnostics = self._last_plan_diagnostics
        optimized_blocked = int(
            diagnostics.get("pct_optimizer_blocked_sample_count", 0) or 0
        )
        raw_blocked = int(
            diagnostics.get("pct_optimizer_raw_blocked_sample_count", 0) or 0
        )
        path_mode = str(diagnostics.get("pct_planner_path_mode", "") or "")
        optimizer_accepted = diagnostics.get("pct_optimizer_accepted")

        if raw_blocked > 0:
            return True
        return bool(
            path_mode == "optimized_trajectory"
            and optimizer_accepted is False
            and optimized_blocked > 0
        )

    def _plan_native_with_reload_retry(
        self,
        start_pos: np.ndarray,
        goal_pos: np.ndarray,
        start_h: float,
        goal_h: float,
        raw_start: np.ndarray,
        raw_goal: np.ndarray,
    ) -> object | None:
        attempts = 2 if self._last_plan_diagnostics.get("reload_on_native_exception") else 1
        last_exc: Exception | None = None
        for attempt in range(attempts):
            try:
                if self._planner is None:
                    raise RuntimeError("pct planner unavailable after reload")
                result = self._planner.plan(start_pos, goal_pos, start_h, goal_h)
                self._record_wrapper_path_diagnostics()
                if attempt > 0:
                    self._last_plan_diagnostics["native_retry_count"] = attempt
                    self._last_plan_diagnostics["recovered_by_reload"] = True
                    logger.warning(
                        "PCT plan() recovered after reloading native runtime for start=%s goal=%s",
                        raw_start,
                        raw_goal,
                    )
                return result
            except Exception as exc:  # pragma: no cover - exact native failures are host-specific
                last_exc = exc
                error_record = {
                    "type": exc.__class__.__name__,
                    "message": str(exc),
                    "traceback_tail": traceback.format_exc(limit=6).splitlines()[-12:],
                }
                self._last_plan_diagnostics.setdefault("native_exceptions", [])
                exceptions = self._last_plan_diagnostics["native_exceptions"]
                if isinstance(exceptions, list):
                    exceptions.append(error_record)
                self._last_plan_diagnostics["native_retry_count"] = attempt
                logger.exception(
                    "PCT plan() raised exception for start=%s goal=%s attempt=%d/%d",
                    raw_start,
                    raw_goal,
                    attempt + 1,
                    attempts,
                )
                if attempt + 1 >= attempts:
                    break
                if not self._reload_native_planner_after_exception():
                    break

        self._last_plan_error = "pct native plan raised exception"
        if last_exc is not None:
            self._last_plan_diagnostics.update(
                {
                    "stage": "native_plan_exception",
                    "error_type": last_exc.__class__.__name__,
                    "error_message": str(last_exc),
                    "recovered_by_reload": False,
                }
            )
        return None

    def _reload_native_planner_after_exception(self) -> bool:
        tomogram_path = self._tomogram_path
        self._planner = None
        self._available = False
        self._load_error = ""
        try:
            self._try_load(tomogram_path)
        except Exception as exc:  # defensive; _try_load normally catches internally.
            self._load_error = f"PCT runtime reload failed: {exc}"
            logger.exception("PCT planner reload failed after native plan exception")
        ok = self._planner is not None and self._available
        self._last_plan_diagnostics["reload_attempted"] = True
        self._last_plan_diagnostics["reload_ok"] = bool(ok)
        if not ok:
            self._last_plan_diagnostics["reload_error"] = self._load_error
        return bool(ok)

    def _project_start_to_traversable_xy(
        self,
        pos: np.ndarray,
        *,
        max_snap_m: float,
    ) -> tuple[np.ndarray, dict[str, object]]:
        """Move a blocked start cell to the nearest traversable tomogram cell."""
        try:
            pos_xy = np.asarray(pos[:2], dtype=np.float64)
        except (TypeError, ValueError) as exc:
            return np.asarray([float("nan"), float("nan")], dtype=np.float64), {
                "projected": False,
                "attempted": False,
                "status": "invalid",
                "reason": "start_xy_not_numeric",
                "error": str(exc),
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }
        if pos_xy.shape[0] < 2 or not np.all(np.isfinite(pos_xy[:2])):
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "invalid",
                "reason": "start_xy_not_finite",
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }

        planner = self._planner
        trav = getattr(planner, "layers_t", None)
        if trav is None:
            trav = getattr(self, "_trav_3d", None)
        if planner is None or trav is None:
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "unavailable",
                "reason": "no_traversability_grid",
                "raw_xy": pos_xy.tolist(),
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }

        trav = np.asarray(trav, dtype=np.float32)
        if trav.ndim != 3 or trav.shape[0] == 0:
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "invalid",
                "reason": "invalid_traversability_grid",
                "raw_xy": pos_xy.tolist(),
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }
        rows = int(trav.shape[1])
        cols = int(trav.shape[2])
        if rows <= 0 or cols <= 0:
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "invalid",
                "reason": "empty_traversability_grid",
                "raw_xy": pos_xy.tolist(),
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }

        raw_col, raw_row = self._xy_to_tomogram_cell(pos_xy)
        if not (0 <= raw_col < cols and 0 <= raw_row < rows):
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "out_of_bounds",
                "reason": "start_xy_outside_tomogram_bounds",
                "raw_xy": pos_xy.tolist(),
                "raw_col": int(raw_col),
                "raw_row": int(raw_row),
                "grid_cols": cols,
                "grid_rows": rows,
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }

        raw_cost = self._min_traversability_cost(trav, raw_row, raw_col)
        if raw_cost is not None and raw_cost < self._obstacle_thr:
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "not_needed",
                "reason": "start_cell_traversable",
                "raw_xy": pos_xy.tolist(),
                "raw_col": int(raw_col),
                "raw_row": int(raw_row),
                "raw_min_cost": float(raw_cost),
                "max_snap_m": float(max_snap_m),
                "radius_cells": 0,
                "searched_cells_count": 1,
                "traversable_candidate_count": 1,
            }

        resolution = float(self._resolution or 0.0)
        if not np.isfinite(resolution) or resolution <= 0.0:
            return pos_xy, {
                "projected": False,
                "attempted": False,
                "status": "invalid",
                "reason": "invalid_tomogram_resolution",
                "raw_xy": pos_xy.tolist(),
                "raw_col": int(raw_col),
                "raw_row": int(raw_row),
                "raw_min_cost": None if raw_cost is None else float(raw_cost),
                "max_snap_m": float(max_snap_m),
                "radius_cells": None,
                "searched_cells_count": 0,
                "traversable_candidate_count": 0,
            }

        max_cells = int(np.ceil(float(max_snap_m) / resolution))
        best: tuple[float, int, int, float, np.ndarray] | None = None
        searched_cells_count = 0
        traversable_candidate_count = 0
        for row in range(max(0, raw_row - max_cells), min(rows, raw_row + max_cells + 1)):
            for col in range(max(0, raw_col - max_cells), min(cols, raw_col + max_cells + 1)):
                searched_cells_count += 1
                cost = self._min_traversability_cost(trav, row, col)
                if cost is None or cost >= self._obstacle_thr:
                    continue
                candidate_xy = self._tomogram_cell_to_xy(col, row)
                distance_m = float(np.linalg.norm(candidate_xy[:2] - pos_xy[:2]))
                if distance_m > float(max_snap_m) + 1e-9:
                    continue
                traversable_candidate_count += 1
                rank = (distance_m, row, col, float(cost), candidate_xy)
                if best is None or rank[:4] < best[:4]:
                    best = rank

        if best is None:
            return pos_xy, {
                "projected": False,
                "attempted": True,
                "status": "failed",
                "reason": "no_traversable_start_cell_within_tolerance",
                "raw_xy": pos_xy.tolist(),
                "raw_col": int(raw_col),
                "raw_row": int(raw_row),
                "raw_min_cost": None if raw_cost is None else float(raw_cost),
                "max_snap_m": float(max_snap_m),
                "radius_cells": int(max_cells),
                "searched_cells_count": int(searched_cells_count),
                "traversable_candidate_count": int(traversable_candidate_count),
            }

        distance_m, projected_row, projected_col, projected_cost, projected_xy = best
        return projected_xy, {
            "projected": True,
            "attempted": True,
            "status": "used",
            "raw_xy": pos_xy.tolist(),
            "projected_xy": projected_xy.tolist(),
            "distance_m": float(distance_m),
            "raw_col": int(raw_col),
            "raw_row": int(raw_row),
            "raw_min_cost": None if raw_cost is None else float(raw_cost),
            "projected_col": int(projected_col),
            "projected_row": int(projected_row),
            "projected_min_cost": float(projected_cost),
            "max_snap_m": float(max_snap_m),
            "radius_cells": int(max_cells),
            "searched_cells_count": int(searched_cells_count),
            "traversable_candidate_count": int(traversable_candidate_count),
        }

    @staticmethod
    def _start_projection_plan_fields(
        start_projection: dict[str, object],
    ) -> dict[str, object]:
        return {
            "start_xy_projection_requested": True,
            "start_xy_projection_attempted": bool(
                start_projection.get("attempted", False)
            ),
            "start_xy_projection_radius_m": start_projection.get("max_snap_m"),
            "start_xy_projection_radius_cells": start_projection.get("radius_cells"),
            "start_xy_projection_status": start_projection.get("status"),
            "start_xy_projection_from": start_projection.get("raw_xy"),
            "start_xy_projection_to": start_projection.get(
                "projected_xy",
                start_projection.get("raw_xy"),
            ),
            "start_xy_projection_delta_m": start_projection.get("distance_m", 0.0),
            "start_xy_projection_searched_cells_count": start_projection.get(
                "searched_cells_count",
                0,
            ),
            "start_xy_projection_traversable_candidate_count": (
                start_projection.get("traversable_candidate_count", 0)
            ),
        }

    def _xy_to_tomogram_cell(self, pos_xy: np.ndarray) -> tuple[int, int]:
        planner = self._planner
        try:
            if planner is None:
                raise RuntimeError("missing planner")
            idx = planner.pos2idx(pos_xy)
            return int(round(float(idx[0]))), int(round(float(idx[1])))
        except Exception:
            resolution = float(self._resolution or 0.0)
            if not np.isfinite(resolution) or resolution <= 0.0:
                return 0, 0
            return (
                int(round((float(pos_xy[0]) - self._origin[0]) / resolution)),
                int(round((float(pos_xy[1]) - self._origin[1]) / resolution)),
            )

    def _tomogram_cell_to_xy(self, col: int, row: int) -> np.ndarray:
        resolution = float(self._resolution or 0.0)
        return np.asarray(
            [
                float(self._origin[0]) + float(col) * resolution,
                float(self._origin[1]) + float(row) * resolution,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _min_traversability_cost(
        trav: np.ndarray,
        row: int,
        col: int,
    ) -> float | None:
        try:
            costs = np.asarray(trav[:, row, col], dtype=np.float32)
        except Exception:
            return None
        finite = costs[np.isfinite(costs)]
        if finite.size == 0:
            return None
        return float(np.min(finite))

    def _select_traversable_height(
        self,
        pos: np.ndarray,
        fallback_z: float,
        *,
        max_snap_m: float | None = None,
        reject_out_of_bounds: bool = False,
    ) -> float:
        """Return a height that maps to a traversable tomogram slice."""
        if not np.isfinite(fallback_z):
            if reject_out_of_bounds:
                raise ValueError("goal height must be finite")
            fallback = 0.0
        else:
            fallback = float(fallback_z)
        planner = self._planner
        trav = getattr(planner, "layers_t", None)
        if trav is None:
            trav = getattr(self, "_trav_3d", None)
        if planner is None or trav is None:
            return fallback

        trav = np.asarray(trav, dtype=np.float32)
        if trav.ndim != 3 or trav.shape[0] == 0:
            return fallback

        try:
            pos_xy = np.asarray(pos[:2], dtype=np.float64)
        except (TypeError, ValueError) as exc:
            if reject_out_of_bounds:
                raise ValueError("goal xy must be numeric") from exc
            return fallback
        if pos_xy.shape[0] < 2 or not np.all(np.isfinite(pos_xy[:2])):
            if reject_out_of_bounds:
                raise ValueError("goal xy must be finite")
            return fallback

        try:
            idx = planner.pos2idx(pos_xy)
            raw_col = int(round(float(idx[0])))
            raw_row = int(round(float(idx[1])))
        except Exception:
            if self._resolution <= 0:
                if reject_out_of_bounds:
                    raise ValueError("tomogram resolution must be positive")
                return fallback
            raw_col = int(round((float(pos_xy[0]) - self._origin[0]) / self._resolution))
            raw_row = int(round((float(pos_xy[1]) - self._origin[1]) / self._resolution))
        if not (0 <= raw_col < trav.shape[2] and 0 <= raw_row < trav.shape[1]):
            if reject_out_of_bounds:
                raise ValueError("goal xy is outside tomogram bounds")
        col = int(np.clip(raw_col, 0, trav.shape[2] - 1))
        row = int(np.clip(raw_row, 0, trav.shape[1] - 1))

        elev = getattr(planner, "layers_g", None)
        if elev is not None:
            elev = np.asarray(elev, dtype=np.float32)
            if elev.shape != trav.shape:
                elev = None

        def raw_slice_value(z: float) -> float:
            try:
                return float(planner.pos2slice(float(z)))
            except Exception:
                if self._slice_dh == 0:
                    return 0.0
                return (float(z) - self._slice_h0) / self._slice_dh

        def to_slice(z: float) -> int:
            raw_slice = round(raw_slice_value(z))
            return int(np.clip(raw_slice, 0, trav.shape[0] - 1))

        def slice_height(slice_idx: int, preferred: float | None = None) -> float:
            if (
                preferred is not None
                and np.isfinite(preferred)
                and to_slice(preferred) == slice_idx
            ):
                return float(preferred)
            if elev is not None:
                height = float(elev[slice_idx, row, col])
                if np.isfinite(height):
                    return height
            return float(self._slice_h0 + slice_idx * self._slice_dh)

        if reject_out_of_bounds and max_snap_m is not None:
            raw_fallback_slice = raw_slice_value(fallback)
            layer_tolerance = (
                float(max_snap_m) / abs(float(self._slice_dh))
                if self._slice_dh
                else 0.0
            )
            if (
                raw_fallback_slice < -layer_tolerance
                or raw_fallback_slice > (trav.shape[0] - 1) + layer_tolerance
            ):
                raise ValueError("goal height is outside tomogram layers")

        candidates: list[tuple[int, float | None]] = [(to_slice(fallback), fallback)]
        try:
            surface_h = float(
                planner.get_surface_height(np.asarray(pos[:2], dtype=np.float64))
            )
            if np.isfinite(surface_h):
                candidates.append((to_slice(surface_h), surface_h))
        except Exception as e:
            logger.warning("GlobalPlanner get_surface_height failed: %s", e)
        candidates.extend((slice_idx, None) for slice_idx in range(trav.shape[0]))

        seen: set[int] = set()
        for slice_idx, preferred_h in candidates:
            if slice_idx in seen:
                continue
            seen.add(slice_idx)
            candidate_h = slice_height(slice_idx, preferred_h)
            if (
                max_snap_m is not None
                and np.isfinite(candidate_h)
                and abs(candidate_h - fallback) > float(max_snap_m)
            ):
                continue
            cost = float(trav[slice_idx, row, col])
            if np.isfinite(cost) and cost < self._obstacle_thr:
                return candidate_h

        if reject_out_of_bounds:
            raise ValueError(
                "no traversable tomogram slice within "
                f"{float(max_snap_m):.2f}m of requested height"
                if max_snap_m is not None
                else "no traversable tomogram slice at requested xy"
            )

        return fallback

    def _is_near_zero_route(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> bool:
        """Return True when the route is too short for the native PCT solver."""
        tolerance = max(0.05, float(self._resolution or 0.0))
        if float(np.linalg.norm(goal_pos - start_pos)) <= tolerance:
            return True
        planner = self._planner
        if planner is None or not hasattr(planner, "pos2idx"):
            return False
        try:
            start_idx = np.asarray(planner.pos2idx(start_pos), dtype=float)
            goal_idx = np.asarray(planner.pos2idx(goal_pos), dtype=float)
        except Exception:
            return False
        if start_idx.shape != goal_idx.shape or start_idx.size == 0:
            return False
        return bool(
            np.array_equal(
                np.rint(start_idx).astype(int),
                np.rint(goal_idx).astype(int),
            )
        )



