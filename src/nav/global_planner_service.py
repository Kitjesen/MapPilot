"""GlobalPlannerService — pure-algorithm planning backend wrapper.

Not a Module. Used internally by NavigationModule to separate planning
logic from the mission FSM.
"""
from __future__ import annotations

import logging
import os
import pickle
import time
from pathlib import Path
from typing import Any

from core.msgs.numpy_compat import np
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.same_source_map_artifacts import (
    validate_saved_map_artifact_dir,
)
from nav.plan_safety import evaluate_backend_path_safety

logger = logging.getLogger(__name__)


class GlobalPlannerService:
    """Wraps a planner backend: create → plan → downsample.

    Not a Module — no ports, no lifecycle. Just algorithm logic.
    Swap the backend by passing a different planner_name.
    """

    def __init__(
        self,
        planner_name: str = "astar",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        downsample_dist: float = 2.0,
        plan_safety_policy: str = "observe",
        fallback_planner_name: str = "astar",
        expected_saved_map_frame_id: str | None = None,
    ) -> None:
        self._planner_name = planner_name
        self._tomogram = tomogram
        self._obstacle_thr = obstacle_thr
        self._downsample_dist = downsample_dist
        self._plan_safety_policy = plan_safety_policy
        self._fallback_planner_name = fallback_planner_name
        self._expected_saved_map_frame_id = expected_saved_map_frame_id
        self._backend = None
        self._fallback_backend = None
        self._last_plan_report: dict[str, Any] = {}
        self._map_artifact_gate: dict[str, Any] = self._default_map_artifact_gate()
        self._last_map_update: tuple[np.ndarray, float, np.ndarray | None] | None = None
        self._warned_no_grid: bool = False

    def setup(self) -> None:
        """Create the planner backend. Must be called before plan()."""
        self._map_artifact_gate = self._validate_map_artifact_gate()
        self._backend = self._create_backend()

    @property
    def is_ready(self) -> bool:
        return self._backend is not None

    @property
    def has_map(self) -> bool:
        if self._backend is None:
            return False
        grid = getattr(self._backend, "_grid", None)
        return grid is not None and hasattr(grid, "shape") and getattr(grid, "size", 0) > 0

    @property
    def map_artifact_gate(self) -> dict[str, Any]:
        return dict(self._map_artifact_gate)

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        safe_goal_tolerance: float = 4.0,
    ) -> tuple[list[np.ndarray], float]:
        """Plan a path from start to goal — multi-stage with safety fallbacks.

        Algorithm flow:
          1. Input validation     — backend readiness + map artifact gate
          2. Goal safety check    — BFS nearest free cell within safe_goal_tolerance
          3. Primary planning     — call the configured backend (PCT / A*)
          4. Empty path recovery  — if the primary returned no path, try safe replan
                                    to a nearby reachable cell
          5. Path safety eval     — if non-empty, evaluate against live obstacles;
                                    on failure either replan to a nearby goal
                                    (_try_primary_safe_replan) or take the safe
                                    prefix of the original path (_try_primary_safe_prefix)
          6. Fallback planner     — if safety still fails, try fallback backend
                                    (policy: reject / observe / fallback_astar)
          7. Downsample + return  — thin waypoints to minimum spacing, log stats

        If the goal lands on an obstacle, BFS-searches the nearest free cell
        within safe_goal_tolerance meters (reference: dimos _find_safe_goal).

        Returns:
            (path, plan_ms) where path is list of np.ndarray([x, y, z]).
        Raises:
            RuntimeError if backend not ready or planner returns empty path.
        """
        if self._backend is None:
            raise RuntimeError("GlobalPlannerService: backend not set up")
        if self._map_artifact_gate_blocks():
            reason = self._map_artifact_gate_failure_reason()
            self._last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "selected_path_safety": None,
                "rejected_plans": [
                    {
                        "planner": self._planner_name,
                        "reason": reason,
                        "artifact_gate": self.map_artifact_gate,
                    }
                ],
                "fallback_reason": reason,
                "policy": self._plan_safety_policy,
                "reached_goal": False,
            }
            raise RuntimeError(f"GlobalPlannerService: {reason}")

        # --- Phase 2: Goal safety adjustment (BFS nearest free cell) ---
        requested_goal = np.asarray(goal[:3], dtype=float).copy()

        # Goal safety: if backend exposes a costmap, verify goal is in free space
        goal_grid_checked = self._backend_has_grid(self._backend)
        safe_goal = self._find_safe_goal(
            goal,
            tolerance=safe_goal_tolerance,
            start=start,
        )
        if safe_goal is not None:
            if not np.array_equal(safe_goal[:2], goal[:2]):
                logger.info(
                    "Goal adjusted: (%.1f,%.1f) → (%.1f,%.1f) (nearest free cell)",
                    goal[0], goal[1], safe_goal[0], safe_goal[1],
                )
            goal = safe_goal
        elif goal_grid_checked:
            reason = (
                "goal has no reachable free cell within "
                f"{float(safe_goal_tolerance):.1f}m"
            )
            self._last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "selected_path_safety": None,
                "rejected_plans": [
                    {
                        "planner": self._planner_name,
                        "reason": reason,
                        "goal": requested_goal.tolist(),
                    }
                ],
                "fallback_reason": reason,
                "policy": self._plan_safety_policy,
                "reached_goal": False,
            }
            raise RuntimeError(f"GlobalPlannerService: {reason}")

        # --- Phase 3: Primary planning ---
        raw_path, plan_ms = self._plan_with_backend(self._backend, start, goal)

        selected_backend = self._backend
        selected_planner = self._planner_name
        selected_path = raw_path
        selected_plan_ms = plan_ms
        selected_reached_goal = bool(getattr(selected_backend, "_last_plan_reached_goal", True))
        selected_safety = None
        rejected_plans: list[dict[str, Any]] = []
        primary_replan: dict[str, Any] | None = None
        fallback_reason = ""
        # --- Phase 4: Empty path recovery (replan to nearby safe goal) ---
        if not raw_path:
            backend_plan_error = str(
                getattr(self._backend, "_last_plan_error", "") or ""
            ).strip()
            empty_path_reason = backend_plan_error or "primary planner returned empty path"
            planner_diagnostics = self._backend_plan_diagnostics(self._backend)
            if self._plan_safety_policy == "reject":
                self._last_plan_report = {
                    "primary_planner": self._planner_name,
                    "selected_planner": self._planner_name,
                    "selected_path_safety": None,
                    "rejected_plans": [
                        {
                            "planner": self._planner_name,
                            "reason": empty_path_reason,
                            "planner_diagnostics": planner_diagnostics,
                        }
                    ],
                    "fallback_reason": empty_path_reason,
                    "policy": self._plan_safety_policy,
                    "reached_goal": False,
                    "planner_diagnostics": planner_diagnostics,
                }
                raise RuntimeError(f"GlobalPlannerService: {empty_path_reason}")
            repaired = self._try_primary_safe_replan(
                selected_backend,
                start,
                requested_goal,
                {
                    "ok": False,
                    "reason": "empty_path",
                    "blocked_sample_count": 0,
                },
                tolerance=safe_goal_tolerance,
                reason="initial_primary_empty_path",
                alternate_goals=[goal],
                include_line_back=True,
            )
            if repaired is None:
                self._last_plan_report = {
                    "primary_planner": self._planner_name,
                    "selected_planner": self._planner_name,
                    "selected_path_safety": None,
                    "rejected_plans": [
                        {
                            "planner": self._planner_name,
                            "reason": empty_path_reason,
                            "planner_diagnostics": planner_diagnostics,
                        }
                    ],
                    "fallback_reason": empty_path_reason,
                    "policy": self._plan_safety_policy,
                    "reached_goal": False,
                    "planner_diagnostics": planner_diagnostics,
                }
                raise RuntimeError(f"GlobalPlannerService: {empty_path_reason}")
            (
                repaired_goal,
                repaired_path,
                repaired_ms,
                repaired_safety,
                repaired_reached_goal,
                repair_report,
            ) = repaired
            selected_path = repaired_path
            selected_plan_ms += repaired_ms
            selected_safety = repaired_safety
            selected_reached_goal = repaired_reached_goal
            goal = repaired_goal
            primary_replan = repair_report
            logger.warning(
                "GlobalPlannerService: %s returned empty path; replanned with "
                "%s to reachable nearby goal (%.2f, %.2f, %.2f)",
                self._planner_name,
                self._planner_name,
                repaired_goal[0],
                repaired_goal[1],
                repaired_goal[2],
            )
        # --- Phase 5: Path safety evaluation + repair ---
        else:
            selected_safety = self._evaluate_path_safety(selected_backend, selected_path)
        if selected_safety is not None and not selected_safety.get("ok", True):
            fallback_reason = (
                f"{self._planner_name} path_safety failed "
                f"({selected_safety.get('blocked_sample_count', 0)} blocked samples)"
            )
            rejected_plans.append(
                {
                    "planner": self._planner_name,
                    "path_safety": selected_safety,
                    "reason": fallback_reason,
                }
            )
            if self._plan_safety_policy != "reject":
                repaired = self._try_primary_safe_replan(
                    selected_backend,
                    start,
                    goal,
                    selected_safety,
                    tolerance=safe_goal_tolerance,
                    alternate_goals=[requested_goal],
                    include_line_back=True,
                )
                if repaired is not None:
                    (
                        repaired_goal,
                        repaired_path,
                        repaired_ms,
                        repaired_safety,
                        repaired_reached_goal,
                        repair_report,
                    ) = repaired
                    selected_path = repaired_path
                    selected_plan_ms += repaired_ms
                    selected_safety = repaired_safety
                    selected_reached_goal = repaired_reached_goal
                    goal = repaired_goal
                    primary_replan = repair_report
                    fallback_reason = ""
                    rejected_plans.clear()
                    logger.warning(
                        "GlobalPlannerService: %s path crossed live obstacles; "
                        "replanned with %s to nearby safe goal (%.2f, %.2f, %.2f)",
                        self._planner_name,
                        self._planner_name,
                        repaired_goal[0],
                        repaired_goal[1],
                        repaired_goal[2],
                    )
                else:
                    prefix_repair = self._try_primary_safe_prefix(
                        selected_backend,
                        selected_path,
                        selected_safety,
                        reason="initial_primary_path_safety_failed",
                    )
                    if prefix_repair is not None:
                        (
                            repaired_goal,
                            repaired_path,
                            repaired_safety,
                            repair_report,
                        ) = prefix_repair
                        selected_path = repaired_path
                        selected_safety = repaired_safety
                        selected_reached_goal = False
                        goal = repaired_goal
                        primary_replan = repair_report
                        fallback_reason = ""
                        rejected_plans.clear()
                        logger.warning(
                            "GlobalPlannerService: %s path crossed live obstacles; "
                            "using safe PCT prefix to (%.2f, %.2f, %.2f)",
                            self._planner_name,
                            repaired_goal[0],
                            repaired_goal[1],
                            repaired_goal[2],
                        )

        # --- Phase 6: Fallback planner decision ---
        if fallback_reason:
            if self._plan_safety_policy == "reject":
                self._last_plan_report = {
                    "primary_planner": self._planner_name,
                    "selected_planner": self._planner_name,
                    "selected_path_safety": selected_safety,
                    "rejected_plans": rejected_plans,
                    "fallback_reason": fallback_reason,
                    "policy": self._plan_safety_policy,
                }
                raise RuntimeError(f"GlobalPlannerService: {fallback_reason}")
            if self._plan_safety_policy == "fallback_astar":
                if self._planner_name.lower() == self._fallback_planner_name.lower():
                    self._last_plan_report = {
                        "primary_planner": self._planner_name,
                        "selected_planner": self._planner_name,
                        "selected_path_safety": selected_safety,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": fallback_reason,
                        "policy": self._plan_safety_policy,
                    }
                    raise RuntimeError(
                        "GlobalPlannerService: plan safety failed and no distinct fallback planner is configured"
                    )
                fb_backend = self._get_fallback_backend()
                fb_path, fb_ms = self._plan_with_backend(fb_backend, start, goal)
                fb_safety = self._evaluate_path_safety(fb_backend, fb_path)
                if fb_path and (fb_safety is None or fb_safety.get("ok", False)):
                    selected_backend = fb_backend
                    selected_planner = self._fallback_planner_name
                    selected_path = fb_path
                    selected_plan_ms += fb_ms
                    selected_safety = fb_safety
                    selected_reached_goal = bool(getattr(fb_backend, "_last_plan_reached_goal", True))
                    logger.warning(
                        "GlobalPlannerService: %s; using fallback planner '%s'",
                        fallback_reason,
                        self._fallback_planner_name,
                    )
                else:
                    rejected_plans.append(
                        {
                            "planner": self._fallback_planner_name,
                            "path_safety": fb_safety,
                            "reason": "fallback planner unsafe or empty",
                        }
                    )
                    self._last_plan_report = {
                        "primary_planner": self._planner_name,
                        "selected_planner": self._planner_name,
                        "selected_path_safety": selected_safety,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": fallback_reason,
                        "policy": self._plan_safety_policy,
                    }
                    raise RuntimeError(
                        "GlobalPlannerService: plan safety failed and fallback planner did not produce a safe path"
                    )
            elif self._plan_safety_policy == "observe":
                logger.warning("GlobalPlannerService: %s", fallback_reason)

        # --- Phase 7: Downsample and return ---
        downsample_goal = goal
        if not selected_reached_goal and selected_path:
            endpoint = np.array(selected_path[-1][:3], dtype=float)
            downsample_goal = endpoint
            logger.warning(
                "GlobalPlannerService: planner '%s' returned a safe partial path to "
                "(%.2f, %.2f, %.2f); original goal remains unreachable in current map",
                selected_planner,
                endpoint[0],
                endpoint[1],
                endpoint[2],
            )
        self._last_plan_report = {
            "primary_planner": self._planner_name,
            "selected_planner": selected_planner,
            "selected_path_safety": selected_safety,
            "fallback_reason": fallback_reason,
            "rejected_plans": rejected_plans,
            "policy": self._plan_safety_policy,
            "reached_goal": selected_reached_goal,
        }
        planner_diagnostics = self._backend_plan_diagnostics(selected_backend)
        if planner_diagnostics:
            self._last_plan_report["planner_diagnostics"] = planner_diagnostics
        if primary_replan is not None:
            self._last_plan_report["primary_replan"] = primary_replan
        path = self._downsample(selected_path, downsample_goal)
        logger.info(
            "Planned %d waypoints in %.1fms (planner=%s)",
            len(path), selected_plan_ms, selected_planner,
        )
        return path, selected_plan_ms

    def update_map(
        self,
        grid: np.ndarray,
        resolution: float = 0.2,
        origin: np.ndarray | None = None,
    ) -> None:
        """Push live costmap to the backend (if supported)."""
        grid_arr = np.asarray(grid, dtype=np.float32).copy()
        origin_arr = None if origin is None else np.asarray(origin[:2], dtype=float).copy()
        self._last_map_update = (grid_arr, float(resolution), origin_arr)
        self._push_map_update(self._backend, grid_arr, float(resolution), origin_arr)
        self._push_map_update(self._fallback_backend, grid_arr, float(resolution), origin_arr)

    # ------------------------------------------------------------------ #
    # Goal Safety (BFS nearest free cell, ref: dimos _find_safe_goal)      #
    # ------------------------------------------------------------------ #

    def _find_safe_goal(
        self,
        goal: np.ndarray,
        tolerance: float = 4.0,
        start: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """Return a free goal cell, optionally constrained to start reachability.

        Returns adjusted goal (np.ndarray) or None if no costmap available.
        If start is provided, the selected cell must be in the same traversable
        component as the robot. This keeps goal safety aligned with A* instead
        of accepting isolated free cells that the planner cannot actually reach.
        """
        if self._backend is None:
            return None

        # Backend must expose costmap grid + resolution + origin
        grid = getattr(self._backend, "_grid", None)
        resolution = getattr(self._backend, "_resolution", 0.2)
        origin = getattr(self._backend, "_origin", None)

        if grid is None or not hasattr(grid, "shape") or grid.ndim != 2:
            if not self._warned_no_grid:
                logger.warning(
                    "GlobalPlannerService: _find_safe_goal bypassed — "
                    "backend '%s' has no 2D _grid attribute. "
                    "Goal safety checking is DISABLED.",
                    self._planner_name,
                )
                self._warned_no_grid = True
            return None

        # Convert world → grid coordinates
        grid_arr = np.asarray(grid, dtype=np.float32)
        ox = origin[0] if origin is not None else 0.0
        oy = origin[1] if origin is not None else 0.0
        gx = int(round((float(goal[0]) - ox) / resolution))
        gy = int(round((float(goal[1]) - oy) / resolution))
        h, w = grid_arr.shape

        reachable: set[tuple[int, int]] | None = None
        if start is not None:
            sx = int(round((float(start[0]) - ox) / resolution))
            sy = int(round((float(start[1]) - oy) / resolution))
            start_cell = (sx, sy)
            if not self._grid_cell_free(grid_arr, sx, sy):
                start_search = max(2, int(round(1.0 / float(resolution))))
                nearest = self._nearest_free_grid_cell(
                    grid_arr,
                    sx,
                    sy,
                    start_search,
                )
                if nearest is None:
                    logger.warning(
                        "No free start cell near planner start (%.1f, %.1f)",
                        float(start[0]),
                        float(start[1]),
                    )
                    return None
                start_cell = nearest
            reachable = self._reachable_grid_component(grid_arr, start_cell)
            if not reachable:
                logger.warning(
                    "Planner start has no reachable free component (%.1f, %.1f)",
                    float(start[0]),
                    float(start[1]),
                )
                return None

        def acceptable_cell(cx: int, cy: int) -> bool:
            if not self._grid_cell_free(grid_arr, cx, cy):
                return False
            return reachable is None or (cx, cy) in reachable

        # If goal is in bounds and free, no adjustment needed
        if 0 <= gx < w and 0 <= gy < h:
            if acceptable_cell(gx, gy):
                gz = float(goal[2]) if len(goal) > 2 else 0.0
                return np.array([goal[0], goal[1], gz])

        # BFS: expand outward from goal cell, find nearest free cell
        from collections import deque
        max_cells = int(tolerance / resolution)
        visited = set()
        queue = deque()
        queue.append((gx, gy, 0))
        visited.add((gx, gy))

        while queue:
            cx, cy, dist = queue.popleft()
            if dist > max_cells:
                continue

            if 0 <= cx < w and 0 <= cy < h:
                if acceptable_cell(cx, cy):
                    # Found free cell — convert back to world
                    wx = ox + cx * resolution
                    wy = oy + cy * resolution
                    gz = float(goal[2]) if len(goal) > 2 else 0.0
                    return np.array([wx, wy, gz])

            if dist >= max_cells:
                continue
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) not in visited:
                    visited.add((nx, ny))
                    queue.append((nx, ny, dist + 1))

        logger.warning(
            "No free cell within %.1fm of goal (%.1f, %.1f)",
            tolerance, goal[0], goal[1],
        )
        return None  # No free cell found — let planner try anyway

    def _grid_cell_free(self, grid: np.ndarray, col: int, row: int) -> bool:
        h, w = grid.shape
        return bool(
            0 <= col < w
            and 0 <= row < h
            and np.isfinite(grid[row, col])
            and grid[row, col] < self._obstacle_thr
        )

    def _nearest_free_grid_cell(
        self,
        grid: np.ndarray,
        col: int,
        row: int,
        max_cells: int,
    ) -> tuple[int, int] | None:
        from collections import deque

        if self._grid_cell_free(grid, col, row):
            return (col, row)
        queue = deque([(col, row, 0)])
        visited = {(col, row)}
        while queue:
            cx, cy, dist = queue.popleft()
            if dist >= max_cells:
                continue
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in visited:
                    continue
                visited.add((nx, ny))
                if self._grid_cell_free(grid, nx, ny):
                    return (nx, ny)
                queue.append((nx, ny, dist + 1))
        return None

    def _reachable_grid_component(
        self,
        grid: np.ndarray,
        start_cell: tuple[int, int],
    ) -> set[tuple[int, int]]:
        from collections import deque

        if not self._grid_cell_free(grid, start_cell[0], start_cell[1]):
            return set()
        queue = deque([start_cell])
        visited = {start_cell}
        while queue:
            cx, cy = queue.popleft()
            for dx, dy in [
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (-1, 1),
                (1, -1),
                (1, 1),
            ]:
                nx, ny = cx + dx, cy + dy
                cell = (nx, ny)
                if cell in visited:
                    continue
                if not self._grid_cell_free(grid, nx, ny):
                    continue
                if dx and dy and (
                    not self._grid_cell_free(grid, cx + dx, cy)
                    or not self._grid_cell_free(grid, cx, cy + dy)
                ):
                    continue
                visited.add(cell)
                queue.append(cell)
        return visited

    @staticmethod
    def _backend_has_grid(backend: Any) -> bool:
        grid = getattr(backend, "_grid", None)
        return bool(
            grid is not None
            and hasattr(grid, "shape")
            and getattr(grid, "ndim", 0) == 2
        )

    # ------------------------------------------------------------------ #
    # Internals                                                            #
    # ------------------------------------------------------------------ #

    def _plan_with_backend(
        self,
        backend: Any,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> tuple[list, float]:
        t0 = time.time()
        raw_path = backend.plan(start, goal)
        return raw_path, (time.time() - t0) * 1000

    def _evaluate_path_safety(self, backend: Any, path: list) -> dict[str, Any] | None:
        if self._plan_safety_policy == "off":
            return None
        return evaluate_backend_path_safety(path, backend, obstacle_thr=self._obstacle_thr)

    @staticmethod
    def _backend_plan_diagnostics(backend: Any) -> dict[str, Any]:
        diagnostics = getattr(backend, "_last_plan_diagnostics", None)
        if not isinstance(diagnostics, dict):
            return {}
        return dict(diagnostics)

    def _try_primary_safe_replan(
        self,
        backend: Any,
        start: np.ndarray,
        goal: np.ndarray,
        initial_safety: dict[str, Any],
        *,
        tolerance: float,
        reason: str = "initial_primary_path_safety_failed",
        alternate_goals: list[np.ndarray] | None = None,
        include_line_back: bool = False,
    ) -> tuple[np.ndarray, list, float, dict[str, Any], bool, dict[str, Any]] | None:
        """Retry the primary planner to a nearby goal when live safety rejects a path.

        Algorithm flow:
          1. Grid validation       — extract costmap grid, resolution, origin
          2. Helper closures       — is_clear, world_to_cell, reachable_component BFS
          3. Seed goal preparation — deduplicate alternate goals
          4. Candidate generation  — spiral outward from seed goals + line_back
                                    interpolation + reachable component sweeping
          5. Plan + safety check   — for each candidate, call backend.plan() then
                                    evaluate_backend_path_safety; return first safe path

        This keeps PCT as the selected planner for exploration targets that are
        close to an obstacle edge. A* fallback remains the last resort when the
        primary planner cannot find any path-safe nearby goal.
        """
        # --- Phase 1: Grid validation and param extraction ---
        grid = getattr(backend, "_grid", None)
        resolution = float(getattr(backend, "_resolution", 0.2))
        origin = getattr(backend, "_origin", None)
        if grid is None or not hasattr(grid, "shape") or getattr(grid, "ndim", 0) != 2:
            return None
        if resolution <= 0 or origin is None:
            return None

        grid_arr = np.asarray(grid, dtype=np.float32)
        if grid_arr.ndim != 2 or grid_arr.size == 0:
            return None

        h, w = grid_arr.shape
        ox, oy = float(origin[0]), float(origin[1])
        max_cells = max(1, int(round(max(float(tolerance), resolution) / resolution)))
        max_candidates = 240 if include_line_back else 120
        candidates_checked = 0

        def is_clear(cx: int, cy: int) -> bool:
            if cx < 0 or cy < 0 or cx >= w or cy >= h:
                return False
            y0, y1 = max(0, cy - 1), min(h, cy + 2)
            x0, x1 = max(0, cx - 1), min(w, cx + 2)
            patch = grid_arr[y0:y1, x0:x1]
            return bool(
                np.all(np.isfinite(patch))
                and np.nanmax(patch) < float(self._obstacle_thr)
            )

        def world_goal(cx: int, cy: int) -> np.ndarray:
            return np.array(
                [
                    ox + cx * resolution,
                    oy + cy * resolution,
                    float(goal[2]) if len(goal) > 2 else 0.0,
                ],
                dtype=float,
            )

        def world_to_cell(world: np.ndarray) -> tuple[int, int]:
            return (
                int(round((float(world[0]) - ox) / resolution)),
                int(round((float(world[1]) - oy) / resolution)),
            )

        # --- Phase 2: Candidate generation (line_back + nearby_ring + reachable_component) ---
        candidate_cells: list[tuple[float, int, int, str]] = []
        seen_cells: set[tuple[int, int]] = set()

        def add_candidate(score: float, cx: int, cy: int, source: str) -> None:
            cell = (cx, cy)
            if cell in seen_cells:
                return
            seen_cells.add(cell)
            if is_clear(cx, cy):
                candidate_cells.append((score, cx, cy, source))

        def nearest_clear_cell(cx: int, cy: int, limit: int) -> tuple[int, int] | None:
            from collections import deque

            if is_clear(cx, cy):
                return (cx, cy)
            queue = deque([(cx, cy, 0)])
            visited = {(cx, cy)}
            while queue:
                cur_x, cur_y, dist = queue.popleft()
                if dist >= limit:
                    continue
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = cur_x + dx, cur_y + dy
                    if (nx, ny) in visited:
                        continue
                    visited.add((nx, ny))
                    if is_clear(nx, ny):
                        return (nx, ny)
                    queue.append((nx, ny, dist + 1))
            return None

        def reachable_component() -> set[tuple[int, int]]:
            """Return clear cells connected to the current robot cell."""
            from collections import deque

            sx, sy = world_to_cell(np.asarray(start[:3], dtype=float))
            seed = nearest_clear_cell(sx, sy, max_cells)
            if seed is None:
                return set()

            queue = deque([seed])
            visited = {seed}
            while queue:
                cx, cy = queue.popleft()
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = cx + dx, cy + dy
                    cell = (nx, ny)
                    if cell in visited or not is_clear(nx, ny):
                        continue
                    visited.add(cell)
                    queue.append(cell)
            return visited

        seed_goals: list[np.ndarray] = [np.asarray(goal[:3], dtype=float)]
        for alt in alternate_goals or []:
            alt_arr = np.asarray(alt[:3], dtype=float)
            if not np.isfinite(alt_arr).all():
                logger.warning("Non-finite alternate goal, skipping")
                continue
            if not any(np.linalg.norm(alt_arr[:2] - seed[:2]) < resolution for seed in seed_goals):
                seed_goals.append(alt_arr)

        start_xy = np.asarray(start[:2], dtype=float)
        line_step = max(resolution * 2.0, 0.25)
        component_cells = reachable_component() if include_line_back else set()

        def is_reachable_candidate(cx: int, cy: int) -> bool:
            return not component_cells or (cx, cy) in component_cells

        for seed_index, seed in enumerate(seed_goals):
            gx, gy = world_to_cell(seed)
            original_cell = (gx, gy)
            if include_line_back:
                seed_xy = np.asarray(seed[:2], dtype=float)
                distance = float(np.linalg.norm(seed_xy - start_xy))
                steps = max(2, min(max_candidates // 2, int(distance / line_step) + 1))
                for rank, frac in enumerate(np.linspace(0.0, 0.92, steps)):
                    xy = seed_xy + (start_xy - seed_xy) * float(frac)
                    cx, cy = world_to_cell(np.asarray([xy[0], xy[1], seed[2]], dtype=float))
                    if is_reachable_candidate(cx, cy):
                        add_candidate(
                            seed_index * 10000.0 + rank,
                            cx,
                            cy,
                            "line_back",
                        )

            for radius in range(1, max_cells + 1):
                before = len(candidate_cells)
                for dy in range(-radius, radius + 1):
                    for dx in range(-radius, radius + 1):
                        if max(abs(dx), abs(dy)) != radius:
                            continue
                        cx, cy = gx + dx, gy + dy
                        if (cx, cy) == original_cell:
                            continue
                        distance = (dx * dx + dy * dy) ** 0.5
                        if is_reachable_candidate(cx, cy):
                            add_candidate(
                                seed_index * 10000.0 + 5000.0 + radius + distance * 1e-3,
                                cx,
                                cy,
                                "nearby_ring",
                            )
                if len(candidate_cells) >= max_candidates or (
                    len(candidate_cells) > before and include_line_back
                ):
                    break

        if include_line_back and component_cells:
            goal_xy = np.asarray(goal[:2], dtype=float)
            goal_vec = goal_xy - start_xy
            goal_dist = float(np.linalg.norm(goal_vec))
            unit = goal_vec / goal_dist if goal_dist > 1e-6 else np.zeros(2, dtype=float)
            component_ranked: list[tuple[float, int, int]] = []
            for cx, cy in component_cells:
                candidate = world_goal(cx, cy)
                delta = candidate[:2] - start_xy
                dist_start = float(np.linalg.norm(delta))
                if dist_start < max(resolution, 0.25):
                    continue
                dist_goal = float(np.linalg.norm(candidate[:2] - goal_xy))
                if goal_dist > 1e-6:
                    progress = float(np.dot(delta, unit))
                    perp = float(np.linalg.norm(delta - progress * unit))
                    score = 20000.0 + dist_goal + 0.05 * perp - 0.01 * progress
                else:
                    score = 20000.0 + dist_start
                component_ranked.append((score, cx, cy))
            component_ranked.sort(key=lambda item: item[0])
            remaining = max(0, max_candidates - len(candidate_cells))
            for score, cx, cy in component_ranked[:remaining]:
                add_candidate(score, cx, cy, "reachable_component")

        # --- Phase 3: Candidate evaluation — plan + safety-check each ---
        candidate_cells.sort(key=lambda item: item[0])
        for _score, cx, cy, source in candidate_cells[:max_candidates]:
            candidates_checked += 1
            candidate_goal = world_goal(cx, cy)
            candidate_path, candidate_ms = self._plan_with_backend(
                backend,
                start,
                candidate_goal,
            )
            if not candidate_path:
                continue
            candidate_safety = self._evaluate_path_safety(backend, candidate_path)
            if candidate_safety is None or candidate_safety.get("ok", False):
                reached_goal = bool(getattr(backend, "_last_plan_reached_goal", True))
                report = {
                    "used": True,
                    "reason": reason,
                    "initial_path_safety": initial_safety,
                    "original_goal": np.asarray(goal[:3], dtype=float).tolist(),
                    "repaired_goal": candidate_goal[:3].tolist(),
                    "candidate_count": candidates_checked,
                    "candidate_source": source,
                    "selected_path_safety": candidate_safety,
                }
                return (
                    candidate_goal,
                    candidate_path,
                    candidate_ms,
                    candidate_safety,
                    reached_goal,
                    report,
                )
        return None

    def _try_primary_safe_prefix(
        self,
        backend: Any,
        path: list,
        initial_safety: dict[str, Any],
        *,
        reason: str,
    ) -> tuple[np.ndarray, list, dict[str, Any], dict[str, Any]] | None:
        """Use the safe prefix of a primary-planner path as a staged goal."""
        if not path or len(path) < 2:
            return None
        if self._plan_safety_policy == "off":
            return None
        resolution = float(getattr(backend, "_resolution", 0.2) or 0.2)
        min_prefix_dist = max(0.75, resolution * 3.0)
        points = [np.asarray(p[:3], dtype=float) for p in path]
        start = points[0]
        for end_index in range(len(points) - 2, 0, -1):
            prefix = points[: end_index + 1]
            endpoint = prefix[-1]
            if float(np.linalg.norm(endpoint[:2] - start[:2])) < min_prefix_dist:
                continue
            candidate_path = [p.copy() for p in prefix]
            candidate_safety = self._evaluate_path_safety(backend, candidate_path)
            if candidate_safety is not None and not candidate_safety.get("ok", False):
                continue
            report = {
                "used": True,
                "reason": reason,
                "initial_path_safety": initial_safety,
                "original_goal": points[-1][:3].tolist(),
                "repaired_goal": endpoint[:3].tolist(),
                "candidate_count": 1,
                "candidate_source": "safe_prefix",
                "selected_path_safety": candidate_safety,
            }
            return endpoint.copy(), candidate_path, candidate_safety, report
        return None

    def _get_fallback_backend(self):
        if self._fallback_backend is None:
            self._fallback_backend = self._create_backend(self._fallback_planner_name)
            if self._last_map_update is not None:
                grid, resolution, origin = self._last_map_update
                self._push_map_update(self._fallback_backend, grid, resolution, origin)
        return self._fallback_backend

    @staticmethod
    def _push_map_update(
        backend: Any,
        grid: np.ndarray,
        resolution: float,
        origin: np.ndarray | None,
    ) -> None:
        if backend is not None and hasattr(backend, "update_map"):
            backend.update_map(grid, resolution=resolution, origin=origin)

    @property
    def last_plan_report(self) -> dict[str, Any]:
        return dict(self._last_plan_report)

    def backend_status(self) -> dict[str, Any]:
        report = self.last_plan_report
        selected = str(report.get("selected_planner") or self._planner_name)
        fallback_reason = str(report.get("fallback_reason") or "")
        return {
            "configured_backend": self._planner_name,
            "backend": selected,
            "fallback_backend": self._fallback_planner_name,
            "degraded": bool(fallback_reason) or selected != self._planner_name,
            "degraded_reason": fallback_reason,
        }

    def reload_tomogram(self, tomogram: str) -> dict[str, Any]:
        """Reload the active tomogram/costmap through the planner service boundary."""
        self._tomogram = str(tomogram or "")
        self._map_artifact_gate = self._validate_map_artifact_gate()
        if self._backend is None:
            return {
                "ok": False,
                "backend": self._planner_name,
                "reason": "planner_backend_not_ready",
            }
        load_tomogram = getattr(self._backend, "_load_tomogram", None)
        if callable(load_tomogram):
            load_tomogram(self._tomogram)
            return {"ok": True, "backend": self._planner_name, "mode": "tomogram"}
        update_map = getattr(self._backend, "update_map", None)
        if callable(update_map):
            with open(self._tomogram, "rb") as f:
                data = pickle.load(f)  # noqa: S301  # trusted local tomogram
            grid = data.get("grid") if isinstance(data, dict) else None
            if grid is None:
                return {
                    "ok": False,
                    "backend": self._planner_name,
                    "reason": "tomogram_has_no_grid",
                }
            update_map(grid, resolution=data.get("resolution", 0.2))
            return {"ok": True, "backend": self._planner_name, "mode": "costmap"}
        return {
            "ok": False,
            "backend": self._planner_name,
            "reason": "planner_backend_reload_unsupported",
        }

    def _is_pct_planner(self, name: str | None = None) -> bool:
        return (name or self._planner_name).lower() == "pct"

    def _default_map_artifact_gate(self) -> dict[str, Any]:
        return {
            "required": self._is_pct_planner(),
            "ok": True,
            "reason": "not_checked",
            "blockers": [],
        }

    def _resolve_tomogram_path(self) -> str:
        tomogram_path = self._tomogram
        if not tomogram_path or not os.path.exists(tomogram_path):
            active_tomo = os.path.join(
                os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps")),
                "active", "tomogram.pickle",
            )
            if os.path.exists(active_tomo):
                tomogram_path = active_tomo
                logger.info(
                    "GlobalPlannerService: using active map: %s", tomogram_path
                )
        return tomogram_path

    def _validate_map_artifact_gate(self) -> dict[str, Any]:
        if not self._is_pct_planner():
            return {
                "required": False,
                "ok": True,
                "reason": "not_required_for_planner",
                "planner": self._planner_name,
                "blockers": [],
            }
        tomogram_path = self._resolve_tomogram_path()
        expected_frame_id = (
            self._expected_saved_map_frame_id
            or topic_default_frame_id(TOPICS.saved_map_cloud)
        )
        if not tomogram_path:
            return {
                "schema_version": "lingtu.saved_map_artifacts.gate.v1",
                "required": True,
                "ok": False,
                "reason": "tomogram_required_for_pct_planner",
                "planner": self._planner_name,
                "tomogram": "",
                "expected_frame_id": expected_frame_id,
                "blockers": ["tomogram required for pct planner"],
            }
        gate = validate_saved_map_artifact_dir(
            Path(tomogram_path).resolve().parent,
            require_tomogram=True,
            expected_frame_id=expected_frame_id,
        )
        gate["required"] = True
        gate["planner"] = self._planner_name
        gate["tomogram"] = str(tomogram_path)
        gate["expected_frame_id"] = expected_frame_id
        if gate.get("ok") is True:
            gate["reason"] = "saved_map_artifact_ok"
        else:
            gate["reason"] = "saved_map_artifact_missing_or_invalid"
        return gate

    def _map_artifact_gate_blocks(self) -> bool:
        return (
            bool(self._map_artifact_gate.get("required", False))
            and self._map_artifact_gate.get("ok") is not True
        )

    def _map_artifact_gate_failure_reason(self) -> str:
        blockers = [
            str(item)
            for item in (self._map_artifact_gate.get("blockers") or [])
            if str(item)
        ]
        detail = "; ".join(blockers) if blockers else "unknown blocker"
        return f"saved map artifact gate failed: {detail}"

    def _create_backend(self, name: str | None = None):
        from core.plugin_seed import seed_registered_plugins
        from core.registry import get
        name = (name or self._planner_name).lower()
        seed_registered_plugins(groups=("planner_backend",))
        try:
            BackendCls = get("planner_backend", name)
        except KeyError as err:
            from core.registry import list_plugins
            available = list_plugins("planner_backend")
            raise ValueError(
                f"Unknown planner: '{name}'. Available: {available}"
            ) from err

        tomogram_path = self._resolve_tomogram_path()

        return BackendCls(tomogram_path, self._obstacle_thr)

    def _downsample(self, path: list, goal: np.ndarray) -> list[np.ndarray]:
        if not path:
            return []
        result = [np.array(path[0][:3])]
        for p in path[1:]:
            pt = (np.array(p[:3]) if len(p) >= 3
                  else np.array([p[0], p[1], 0.0]))
            if np.linalg.norm(pt - result[-1]) >= self._downsample_dist:
                result.append(pt)
        goal_pt = (np.array(goal[:3]) if len(goal) >= 3
                   else np.array([goal[0], goal[1], 0.0]))
        if np.linalg.norm(goal_pt - result[-1]) > 1e-6:
            result.append(goal_pt)
        return result
