"""EXPERIMENTAL — not used in production navigation stack.
Hybrid Planner — topology-assisted hierarchical path planning.

Core idea:
  Decompose global planning into two layers:
  1. Topology layer: Dijkstra on the topology graph → room sequence
  2. Geometry layer: local A* on the Tomogram traversability grid for each adjacent room pair

Advantages:
  - Smaller A* search space (only between room pairs; ~3-10x faster than global A*)
  - Exploits high-level connectivity from the topology graph

Failure semantics (important):
  - A* returns None → segment fails → whole path success=False.
  - Never fall back to straight-line: a straight path may cross walls or obstacles,
    making the robot believe there is a route when there is none.
  - On success=False, the caller must trigger LERa recovery or replan rather than
    executing the path blindly.

References:
  - TopoNav (2025): topology graph as spatial memory
  - Hierarchical Planning: layered planning reduces search space
  - Legacy grid A* behavior, kept here as a ROS-free implementation.
"""

import heapq
import logging
import time
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)


# ── Pure Python A* on traversability grid ───────────────────────────────────
#
# Ported from the legacy grid A* implementation with ROS2 dependencies removed. Performs
# 8-connected A* directly on a 2D traversability grid.


def _w2g(wx: float, wy: float, cx: float, cy: float, res: float, ox: int, oy: int, nx: int, ny: int) -> tuple[int, int]:
    """World coordinates (m) → grid index (clipped to grid bounds)."""
    ix = int(round((wx - cx) / res)) + ox
    iy = int(round((wy - cy) / res)) + oy
    return int(np.clip(ix, 0, nx - 1)), int(np.clip(iy, 0, ny - 1))


def _g2w(ix: int, iy: int, cx: float, cy: float, res: float, ox: int, oy: int) -> tuple[float, float]:
    """Grid index → world coordinates (m)."""
    return (ix - ox) * res + cx, (iy - oy) * res + cy


def _find_nearest_free(trav: np.ndarray, ci: int, cj: int, obs_thr: float, radius: int = 5) -> tuple[int, int]:
    """Find the nearest free cell within radius cells; return the original position if none found
    (A* will then report None itself)."""
    if trav[ci, cj] < obs_thr:
        return ci, cj
    best, best_dist = (ci, cj), float("inf")
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            ni, nj = ci + di, cj + dj
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr:
                    d = di * di + dj * dj
                    if d < best_dist:
                        best, best_dist = (ni, nj), d
    return best


def _astar_on_grid(
    trav: np.ndarray, start: tuple[int, int], goal: tuple[int, int], obs_thr: float, timeout_sec: float = 5.0
) -> list[tuple[int, int]] | None:
    """8-connected A* on a traversability grid.

    Args:
        trav:        2D float array; trav[i,j] < obs_thr means traversable
        start/goal:  (ix, iy) grid coordinates
        obs_thr:     obstacle threshold
        timeout_sec: seconds before giving up; returns None on timeout (no exception)

    Returns:
        list[(ix, iy)] path, or None if unreachable or timed out
    """
    deadline = time.monotonic() + timeout_sec

    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan heuristic

    open_q = [(h(start, goal), 0.0, start, [])]
    visited: dict = {}

    while open_q:
        if time.monotonic() > deadline:
            logger.warning("A* timeout after %.1fs (visited %d nodes)", timeout_sec, len(visited))
            return None
        _f, g, cur, path = heapq.heappop(open_q)
        if cur in visited:
            continue
        visited[cur] = True
        path = [*path, cur]
        if cur == goal:
            return path
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ni, nj = cur[0] + dx, cur[1] + dy
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr and (ni, nj) not in visited:
                    cost = g + (1.414 if dx and dy else 1.0)
                    heapq.heappush(open_q, (cost + h((ni, nj), goal), cost, (ni, nj), path))
    return None  # unreachable


def _downsample_cells(cells: list, min_dist: int = 3) -> list:
    """Uniform downsampling: consecutive kept points are >= min_dist cells apart."""
    if len(cells) <= 2:
        return cells
    kept = [cells[0]]
    for c in cells[1:]:
        dx, dy = c[0] - kept[-1][0], c[1] - kept[-1][1]
        if dx * dx + dy * dy >= min_dist * min_dist:
            kept.append(c)
    if kept[-1] != cells[-1]:
        kept.append(cells[-1])
    return kept


# ── Data classes ─────────────────────────────────────────────────────────────


@dataclass
class PathSegment:
    """One geometry segment connecting two adjacent rooms."""

    from_room_id: int
    to_room_id: int
    waypoints: list[np.ndarray]  # sequence of waypoints [[x, y, z], ...]
    cost: float  # total Euclidean length (m)
    planning_time: float  # wall-clock planning time (s)


@dataclass
class HybridPath:
    """Result of a hybrid path plan.

    success=False means at least one local A* segment could not be planned.
    The caller must not execute this path; trigger a recovery strategy instead.
    """

    success: bool
    waypoints: list[np.ndarray]
    room_sequence: list[int]
    segments: list[PathSegment]
    total_cost: float
    total_planning_time: float
    topology_planning_time: float
    geometry_planning_time: float
    num_waypoints: int
    num_rooms: int
    # Diagnostic message on failure — helps LERa decide the recovery action
    failure_reason: str = ""


# ── Main planner ─────────────────────────────────────────────────────────────


class HybridPlanner:
    """
    Hybrid planner — topology-assisted hierarchical path planning.

    Algorithm:
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Input: start, goal (world coords), topology graph TSG, traversability grid
    Output: HybridPath (execute only when success=True)

    1. Locate the rooms containing start and goal
    2. Dijkstra on topology graph → room sequence [R1, R2, ..., Rn]
    3. For each adjacent room pair (Ri, Ri+1):
    4.     8-connected A* on the traversability grid (bounded by search_bbox)
    5.     Any segment failure → whole path success=False, return immediately
    6. Return the complete path

    Straight-line fallback has been removed entirely.
    The only correct response to A* failure is to propagate it upward;
    SemanticPlannerModule LERa decides the recovery action.
    """

    def __init__(
        self,
        topology_graph,
        trav: np.ndarray | None = None,
        trav_res: float = 0.2,
        trav_cx: float = 0.0,
        trav_cy: float = 0.0,
        obstacle_thr: float = 49.9,
        astar_timeout_sec: float = 5.0,
        min_downsample_dist: int = 3,
    ):
        """
        Args:
            topology_graph:    TopologySemGraph providing Dijkstra shortest paths
            trav:              optional 2D float32 traversability grid (shape: [nx, ny])
                               trav[i,j] < obstacle_thr means traversable.
                               Source: Tomogram.layers_t[slice_idx] or _load_tomogram()[0].
                               If None, segment planning always returns None (path always fails).
            trav_res:          grid resolution (m/cell), default 0.2 m
            trav_cx/trav_cy:   world coordinates of the grid center (m)
            obstacle_thr:      obstacle threshold
            astar_timeout_sec: per-segment A* timeout (seconds)
            min_downsample_dist: minimum cell distance between consecutive kept waypoints

        Note: the planner_fn callback parameter has been removed. Pass the traversability
        grid directly; A* runs internally. Benefits:
          1. Clear failure semantics: A* None → plan_path success=False → caller LERa
          2. No implicit straight-line fallback
          3. No cross-layer semantic→nav import
        """
        self.tsg = topology_graph
        self._trav = trav  # None → no map, all segments fail
        self._trav_res = trav_res
        self._trav_cx = trav_cx
        self._trav_cy = trav_cy
        self._obs_thr = obstacle_thr
        self._astar_timeout = astar_timeout_sec
        self._min_ds = min_downsample_dist

        if trav is not None:
            self._nx, self._ny = trav.shape
            self._ox, self._oy = self._nx // 2, self._ny // 2
        else:
            self._nx = self._ny = self._ox = self._oy = 0

    def update_traversability(
        self,
        trav: np.ndarray,
        res: float,
        cx: float,
        cy: float,
    ) -> None:
        """Hot-swap the traversability grid (call when map updates; not thread-safe, caller must lock)."""
        self._trav = trav
        self._trav_res = res
        self._trav_cx = cx
        self._trav_cy = cy
        self._nx, self._ny = trav.shape
        self._ox, self._oy = self._nx // 2, self._ny // 2

    def plan_path(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        search_radius_factor: float = 1.5,
        max_planning_time: float = 5.0,
    ) -> HybridPath:
        """Plan a hybrid path from start to goal.

        Returns:
            HybridPath. On success=False waypoints is empty and failure_reason explains why.
            The caller must check the success field before executing the path.
        """
        t0 = time.time()

        # 1. Locate rooms
        start_room = self._locate_room(start[:2])
        goal_room = self._locate_room(goal[:2])

        if start_room is None or goal_room is None:
            reason = (
                f"room lookup failed: start_room={start_room}, goal_room={goal_room} "
                f"(start={start[:2]}, goal={goal[:2]})"
            )
            logger.warning(reason)
            return self._failed(reason)

        # 2. Topology layer: Dijkstra
        t_topo = time.time()
        cost, room_seq = self.tsg.shortest_path(start_room, goal_room)
        topo_time = time.time() - t_topo

        if cost == float("inf") or len(room_seq) == 0:
            reason = f"no topological path: room {start_room} → {goal_room}"
            logger.warning(reason)
            return self._failed(reason, topo_time=topo_time)

        logger.info("Topo path: %s cost=%.2f (%.1fms)", room_seq, cost, topo_time * 1000)

        # 3. Geometry layer: A*
        t_geom = time.time()
        segments: list[PathSegment] = []
        waypoints: list[np.ndarray] = [start.copy()]

        for i in range(len(room_seq) - 1):
            if time.time() - t0 > max_planning_time:
                reason = f"planning timeout at segment {i}/{len(room_seq) - 1}"
                logger.warning(reason)
                return self._failed(reason, segments, waypoints, topo_time, time.time() - t_geom)

            seg = self._plan_local_segment(
                from_room_id=room_seq[i],
                to_room_id=room_seq[i + 1],
                start_pos=waypoints[-1],
                goal_pos=goal if i == len(room_seq) - 2 else None,
                search_radius_factor=search_radius_factor,
            )

            if seg is None:
                reason = (
                    f"A* failed for segment room {room_seq[i]} → {room_seq[i + 1]} "
                    f"(start={waypoints[-1][:2]}) — "
                    f"path is BLOCKED, no straight-line fallback"
                )
                logger.error(reason)
                return self._failed(reason, segments, waypoints, topo_time, time.time() - t_geom)

            segments.append(seg)
            waypoints.extend(seg.waypoints[1:])

        geom_time = time.time() - t_geom
        total_time = time.time() - t0

        return HybridPath(
            success=True,
            waypoints=waypoints,
            room_sequence=room_seq,
            segments=segments,
            total_cost=sum(s.cost for s in segments),
            total_planning_time=total_time,
            topology_planning_time=topo_time,
            geometry_planning_time=geom_time,
            num_waypoints=len(waypoints),
            num_rooms=len(room_seq),
        )

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _plan_local_segment(
        self,
        from_room_id: int,
        to_room_id: int,
        start_pos: np.ndarray,
        goal_pos: np.ndarray | None,
        search_radius_factor: float,
    ) -> PathSegment | None:
        """Plan the local geometry segment between two adjacent rooms.

        Returns:
            PathSegment, or None if A* finds no path, times out, or there is no map.
            Never returns a straight-line path — None means None.
        """
        t0 = time.time()

        from_room = self.tsg.get_node(from_room_id)
        to_room = self.tsg.get_node(to_room_id)
        if from_room is None or to_room is None:
            logger.warning("Room node missing: %s or %s", from_room_id, to_room_id)
            return None

        if self._trav is None:
            logger.warning(
                "No traversability grid — cannot plan segment %d→%d. Call update_traversability() before plan_path().",
                from_room_id,
                to_room_id,
            )
            return None

        # Goal position: real goal for the last segment, room center for intermediate segments
        if goal_pos is None:
            goal_pos = np.array([to_room.center[0], to_room.center[1], start_pos[2]])

        # Search region bounding box
        search_bbox = self._compute_search_bbox(from_room, to_room, search_radius_factor)

        # World → grid coordinates
        si, sj = _w2g(
            start_pos[0],
            start_pos[1],
            self._trav_cx,
            self._trav_cy,
            self._trav_res,
            self._ox,
            self._oy,
            self._nx,
            self._ny,
        )
        gi, gj = _w2g(
            goal_pos[0],
            goal_pos[1],
            self._trav_cx,
            self._trav_cy,
            self._trav_res,
            self._ox,
            self._oy,
            self._nx,
            self._ny,
        )

        # Snap start/goal to nearest free cell if they land on an obstacle
        si, sj = _find_nearest_free(self._trav, si, sj, self._obs_thr)
        gi, gj = _find_nearest_free(self._trav, gi, gj, self._obs_thr)

        # Clip to search_bbox: run A* only within the bounding box sub-grid
        bi_min, bj_min = _w2g(
            search_bbox["x_min"],
            search_bbox["y_min"],
            self._trav_cx,
            self._trav_cy,
            self._trav_res,
            self._ox,
            self._oy,
            self._nx,
            self._ny,
        )
        bi_max, bj_max = _w2g(
            search_bbox["x_max"],
            search_bbox["y_max"],
            self._trav_cx,
            self._trav_cy,
            self._trav_res,
            self._ox,
            self._oy,
            self._nx,
            self._ny,
        )
        bi_min, bi_max = min(bi_min, bi_max), max(bi_min, bi_max)
        bj_min, bj_max = min(bj_min, bj_max), max(bj_min, bj_max)

        # Map global grid coords to sub-grid local coords
        si_loc = int(np.clip(si - bi_min, 0, bi_max - bi_min))
        sj_loc = int(np.clip(sj - bj_min, 0, bj_max - bj_min))
        gi_loc = int(np.clip(gi - bi_min, 0, bi_max - bi_min))
        gj_loc = int(np.clip(gj - bj_min, 0, bj_max - bj_min))

        trav_sub = self._trav[bi_min : bi_max + 1, bj_min : bj_max + 1]
        if trav_sub.size == 0:
            logger.warning("Empty trav subgrid for segment %d→%d", from_room_id, to_room_id)
            return None

        # A* in subgrid
        cells = _astar_on_grid(
            trav_sub,
            (si_loc, sj_loc),
            (gi_loc, gj_loc),
            self._obs_thr,
            self._astar_timeout,
        )

        if cells is None:
            logger.warning(
                "A* found no path: room %d → %d (start_grid=(%d,%d) goal_grid=(%d,%d) subgrid=%s)",
                from_room_id,
                to_room_id,
                si_loc,
                sj_loc,
                gi_loc,
                gj_loc,
                trav_sub.shape,
            )
            return None

        # Downsample and convert grid coords → world coords
        cells = _downsample_cells(cells, self._min_ds)
        z = float(start_pos[2])
        pts: list[np.ndarray] = []
        for ci, cj in cells:
            wx, wy = _g2w(ci + bi_min, cj + bj_min, self._trav_cx, self._trav_cy, self._trav_res, self._ox, self._oy)
            pts.append(np.array([wx, wy, z]))

        # Force exact start and goal endpoints
        pts[0] = start_pos.copy()
        pts[-1] = goal_pos.copy()

        cost = sum(float(np.linalg.norm(pts[k + 1] - pts[k])) for k in range(len(pts) - 1))

        return PathSegment(
            from_room_id=from_room_id,
            to_room_id=to_room_id,
            waypoints=pts,
            cost=cost,
            planning_time=time.time() - t0,
        )

    def _locate_room(self, position: np.ndarray) -> int | None:
        """Locate the room containing a position (three-stage lookup:
        get_room_by_position -> convex hull -> nearest centroid)."""
        room_id = self.tsg.get_room_by_position(position[0], position[1], radius=5.0)
        if room_id is not None:
            return room_id

        for room in self.tsg.rooms:
            if room.convex_hull is not None and len(room.convex_hull) >= 3:
                if self._point_in_polygon(np.array(position), room.convex_hull):
                    return room.node_id

        rooms = self.tsg.rooms
        if not rooms:
            return None
        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - position)))
        return nearest.node_id

    def _compute_search_bbox(self, from_room, to_room, radius_factor: float) -> dict:
        """Compute the merged bbox of two rooms, expanded by radius_factor."""
        if from_room.bounding_box and to_room.bounding_box:
            x_min = min(from_room.bounding_box["x_min"], to_room.bounding_box["x_min"])
            x_max = max(from_room.bounding_box["x_max"], to_room.bounding_box["x_max"])
            y_min = min(from_room.bounding_box["y_min"], to_room.bounding_box["y_min"])
            y_max = max(from_room.bounding_box["y_max"], to_room.bounding_box["y_max"])
        else:
            centers = np.array([from_room.center, to_room.center])
            x_min = centers[:, 0].min() - 5.0
            x_max = centers[:, 0].max() + 5.0
            y_min = centers[:, 1].min() - 5.0
            y_max = centers[:, 1].max() + 5.0

        margin = ((x_max - x_min) + (y_max - y_min)) / 2 * (radius_factor - 1.0)
        return {
            "x_min": x_min - margin,
            "x_max": x_max + margin,
            "y_min": y_min - margin,
            "y_max": y_max + margin,
        }

    @staticmethod
    def _failed(
        reason: str,
        segments: list[PathSegment] | None = None,
        waypoints: list[np.ndarray] | None = None,
        topo_time: float = 0.0,
        geom_time: float = 0.0,
    ) -> HybridPath:
        """Construct a failed HybridPath with a diagnostic reason string."""
        return HybridPath(
            success=False,
            waypoints=[],
            room_sequence=[],
            segments=segments or [],
            total_cost=float("inf"),
            total_planning_time=topo_time + geom_time,
            topology_planning_time=topo_time,
            geometry_planning_time=geom_time,
            num_waypoints=0,
            num_rooms=0,
            failure_reason=reason,
        )

    @staticmethod
    def _point_in_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
        """Ray casting algorithm: check whether a point lies inside a polygon."""
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    @staticmethod
    def _point_in_bbox(point: np.ndarray, bbox: dict) -> bool:
        return bbox["x_min"] <= point[0] <= bbox["x_max"] and bbox["y_min"] <= point[1] <= bbox["y_max"]


# ── Planner comparison utilities ─────────────────────────────────────────────


@dataclass
class PlannerComparison:
    """Performance comparison result between two planners."""

    hybrid_time: float
    baseline_time: float
    speedup: float
    hybrid_waypoints: int
    baseline_waypoints: int
    hybrid_cost: float
    baseline_cost: float
    hybrid_rooms: int
    hybrid_success: bool
    baseline_success: bool


def compare_planners(
    hybrid_planner: HybridPlanner,
    baseline_planner,
    test_cases: list[tuple[np.ndarray, np.ndarray]],
) -> list[PlannerComparison]:
    """Compare the HybridPlanner against a baseline planner.

    Args:
        hybrid_planner:   HybridPlanner instance
        baseline_planner: baseline planner (must implement plan_path())
        test_cases:       [(start, goal), ...]

    Returns:
        List of comparison results; failed cases are included with success=False
    """
    results = []
    for i, (start, goal) in enumerate(test_cases):
        logger.info("Test case %d/%d: %s → %s", i + 1, len(test_cases), start[:2], goal[:2])

        t0 = time.time()
        hybrid_result = hybrid_planner.plan_path(start, goal)
        hybrid_ms = (time.time() - t0) * 1000

        t1 = time.time()
        baseline_result = baseline_planner.plan_path(start, goal)
        baseline_ms = (time.time() - t1) * 1000

        results.append(
            PlannerComparison(
                hybrid_time=hybrid_ms,
                baseline_time=baseline_ms,
                speedup=baseline_ms / hybrid_ms if hybrid_ms > 0 else 0.0,
                hybrid_waypoints=hybrid_result.num_waypoints,
                baseline_waypoints=getattr(baseline_result, "num_waypoints", 0),
                hybrid_cost=hybrid_result.total_cost,
                baseline_cost=getattr(baseline_result, "total_cost", float("inf")),
                hybrid_rooms=hybrid_result.num_rooms,
                hybrid_success=hybrid_result.success,
                baseline_success=getattr(baseline_result, "success", False),
            )
        )

    return results
