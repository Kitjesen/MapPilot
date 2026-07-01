"""Grid goal-safety and safe-replan helpers for GlobalPlanner."""

from __future__ import annotations

import logging
from collections import deque
from typing import Any

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


class GlobalPlannerGridSafetyMixin:
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
                    "GlobalPlanner: _find_safe_goal bypassed -"
                    "backend '%s' has no 2D _grid attribute. "
                    "Goal safety checking is DISABLED.",
                    self._planner_name,
                )
                self._warned_no_grid = True
            return None

        # Convert world ->grid coordinates
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
                    # Found free cell -convert back to world
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
        return None  # No free cell found -let planner try anyway

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
          1. Grid validation: extract costmap grid, resolution, origin.
          2. Helper closures: is_clear, world_to_cell, reachable_component BFS.
          3. Seed goal preparation: deduplicate alternate goals.
          4. Candidate generation: spiral outward from seed goals, optional
             line-back interpolation, and reachable component sweeping.
          5. Plan + safety check: call the selected primary backend for each
             candidate and return the first path that passes live safety.

        This keeps the configured primary backend, such as OctoPlanner3D, as
        the only planner in product profiles where fallback_planner_name is
        intentionally empty.
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

        # --- Phase 3: Candidate evaluation -plan + safety-check each ---
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
