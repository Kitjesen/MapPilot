"""test_hybrid_planner.py — HybridPlanner unit tests

Tests the correct failure semantics of hybrid_planner.py:
  - A* finds no path → plan_path success=False with failure_reason
  - No traversability grid → success=False (no crash)
  - Topology graph has no path → success=False
  - A* succeeds → success=True, non-empty waypoints, no fake straight-line path

No scipy, ROS2, torch, or other heavy dependencies.
Uses only numpy + inline mock objects.
"""

import math

import numpy as np
import pytest

from perception.hybrid_planner import (
    HybridPlanner,
    _astar_on_grid,
    _downsample_cells,
    _find_nearest_free,
    _g2w,
    _w2g,
)

# ── Mock topology graph ───────────────────────────────────────────────────────

class MockRoom:
    def __init__(self, node_id, cx, cy, bbox=None):
        self.node_id = node_id
        self.center = np.array([cx, cy])
        self.bounding_box = bbox  # {"x_min","x_max","y_min","y_max"} or None
        self.convex_hull = None


class MockTSG:
    """Minimal topology-graph mock — only implements the interface HybridPlanner uses."""

    def __init__(self, rooms, edges):
        """
        rooms: list of MockRoom
        edges: dict {(from_id, to_id): cost}  — bidirectional
        """
        self._rooms = {r.node_id: r for r in rooms}
        self._edges = {}
        for (a, b), c in edges.items():
            self._edges.setdefault(a, {})[b] = c
            self._edges.setdefault(b, {})[a] = c

    @property
    def rooms(self):
        return list(self._rooms.values())

    def get_node(self, node_id):
        return self._rooms.get(node_id)

    def get_room_by_position(self, x, y, radius=5.0):
        for r in self._rooms.values():
            if math.hypot(r.center[0] - x, r.center[1] - y) <= radius:
                return r.node_id
        return None

    def shortest_path(self, src, dst):
        """Dijkstra — returns (cost, [node_id, ...]) or (inf, [])."""
        import heapq
        dist = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, math.inf):
                continue
            if u == dst:
                path = []
                cur = dst
                while cur in prev:
                    path.append(cur)
                    cur = prev[cur]
                path.append(src)
                path.reverse()
                return d, path
            for v, w in self._edges.get(u, {}).items():
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return math.inf, []


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_open_trav(nx=100, ny=100, obs_thr=49.9):
    """Fully traversable grid — all cells below obs_thr."""
    return np.zeros((nx, ny), dtype=np.float32)


def _make_walled_trav(nx=50, ny=50, obs_thr=49.9):
    """Grid with a wall across the middle, leaving a 3-cell gap."""
    trav = np.zeros((nx, ny), dtype=np.float32)
    mid = nx // 2
    # Horizontal wall at row mid, with a gap in the center
    trav[mid, :] = 100.0
    trav[mid, ny // 2 - 1: ny // 2 + 2] = 0.0  # gap
    return trav


def _make_blocked_trav(nx=50, ny=50):
    """Grid with a solid wall across the middle — no gap."""
    trav = np.zeros((nx, ny), dtype=np.float32)
    trav[nx // 2, :] = 100.0  # no gap
    return trav


def _simple_planner(tsg, trav, **kw):
    """Build a minimal two-room HybridPlanner for testing."""
    return HybridPlanner(
        topology_graph=tsg,
        trav=trav,
        trav_res=0.2,
        trav_cx=0.0,
        trav_cy=0.0,
        **kw,
    )


# ── A* low-level function tests ───────────────────────────────────────────────

class TestAstarOnGrid:
    def test_direct_path_open_grid(self):
        trav = _make_open_trav(20, 20)
        path = _astar_on_grid(trav, (0, 0), (10, 10), obs_thr=49.9)
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (10, 10)

    def test_returns_none_when_blocked(self):
        """Fully blocked goal → None, not a straight line."""
        trav = _make_blocked_trav(20, 20)
        path = _astar_on_grid(trav, (0, 5), (15, 5), obs_thr=49.9)
        assert path is None, "A* should return None for blocked path, not a straight line"

    def test_timeout_returns_none(self):
        """Timeout returns None without raising an exception."""
        # Large grid + very short timeout
        trav = np.zeros((200, 200), dtype=np.float32)
        path = _astar_on_grid(trav, (0, 0), (199, 199), obs_thr=49.9, timeout_sec=0.0001)
        assert path is None

    def test_start_equals_goal(self):
        trav = _make_open_trav(10, 10)
        path = _astar_on_grid(trav, (5, 5), (5, 5), obs_thr=49.9)
        assert path is not None
        assert path == [(5, 5)]

    def test_path_avoids_obstacle(self):
        """Path must go around the obstacle, not through it."""
        trav = _make_walled_trav(30, 30)
        path = _astar_on_grid(trav, (5, 5), (20, 5), obs_thr=49.9)
        assert path is not None
        # No cell in the path should be in the wall row
        for ci, cj in path:
            assert trav[ci, cj] < 49.9, "path goes through obstacle"

    def test_no_path_through_wall_without_gap(self):
        """Solid wall with no gap → None."""
        trav = np.zeros((20, 20), dtype=np.float32)
        trav[10, :] = 100.0  # solid wall
        path = _astar_on_grid(trav, (0, 10), (15, 10), obs_thr=49.9)
        assert path is None


class TestAstarHelpers:
    def test_w2g_g2w_roundtrip(self):
        cx, cy, res, nx, ny, ox, oy = 0.0, 0.0, 0.2, 100, 100, 50, 50
        for wx, wy in [(1.0, 2.0), (-3.0, 0.5), (0.0, 0.0)]:
            ix, iy = _w2g(wx, wy, cx, cy, res, ox, oy, nx, ny)
            rx, ry = _g2w(ix, iy, cx, cy, res, ox, oy)
            # Round-trip accuracy within one cell width (0.2 m)
            assert abs(rx - wx) <= res + 1e-9
            assert abs(ry - wy) <= res + 1e-9

    def test_find_nearest_free_already_free(self):
        trav = np.zeros((10, 10), dtype=np.float32)
        ni, nj = _find_nearest_free(trav, 5, 5, obs_thr=49.9)
        assert (ni, nj) == (5, 5)

    def test_find_nearest_free_on_obstacle(self):
        trav = np.full((10, 10), 100.0, dtype=np.float32)
        trav[5, 7] = 0.0  # only free cell
        ni, nj = _find_nearest_free(trav, 5, 5, obs_thr=49.9, radius=5)
        assert (ni, nj) == (5, 7)

    def test_downsample_removes_close_points(self):
        cells = [(i, 0) for i in range(20)]
        result = _downsample_cells(cells, min_dist=3)
        # The last point is always kept regardless of spacing;
        # all other consecutive kept-points must be >= min_dist apart.
        for k in range(len(result) - 2):
            dx = result[k + 1][0] - result[k][0]
            dy = result[k + 1][1] - result[k][1]
            assert math.hypot(dx, dy) >= 3 - 1e-9
        # First and last points must always be preserved
        assert result[0] == cells[0]
        assert result[-1] == cells[-1]


# ── HybridPlanner failure semantics tests ────────────────────────────────────

class TestHybridPlannerFailureSemantic:
    """Core tests: success=False must be returned instead of a fake straight-line path."""

    def test_no_trav_grid_returns_failure(self):
        """No traversability grid → success=False, no crash, no straight-line path."""
        # Rooms are far enough apart that get_room_by_position(radius=5) cannot cross-match
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 30.0, 0.0)]
        tsg = MockTSG(rooms, {(1, 2): 30.0})
        planner = HybridPlanner(topology_graph=tsg, trav=None)

        # Start near room1, goal near room2 (each within their own radius)
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([30.0, 0.0, 0.0]))

        assert not result.success, "Must return failure when no trav grid"
        assert len(result.waypoints) == 0, "Blocked path must have no waypoints"
        assert result.failure_reason != "", "Must include failure_reason"

    def test_blocked_path_returns_failure_not_straight_line(self):
        """A* cannot cross obstacle → success=False, NOT a straight-line fallback."""
        # 100×100 grid at 0.5 m/cell → 50 m wide
        # room1 center at (5, 0), room2 at (45, 0), far enough to avoid radius overlap
        nx, ny = 100, 100
        rooms = [
            MockRoom(1,  5.0, 0.0, {"x_min":  0, "x_max": 20, "y_min": -10, "y_max": 10}),
            MockRoom(2, 45.0, 0.0, {"x_min": 30, "x_max": 50, "y_min": -10, "y_max": 10}),
        ]
        tsg = MockTSG(rooms, {(1, 2): 40.0})

        trav = np.zeros((nx, ny), dtype=np.float32)
        trav[50, :] = 100.0  # solid wall at grid row 50, no gap

        # res=0.5 m, cx=25 m, cy=0 → grid cell (50,50) = world (25 m, 0 m)
        planner = HybridPlanner(
            topology_graph=tsg,
            trav=trav,
            trav_res=0.5,
            trav_cx=25.0,
            trav_cy=0.0,
        )

        start = np.array([5.0,  0.0, 0.0])   # near room1
        goal  = np.array([45.0, 0.0, 0.0])   # near room2 (must cross the wall)
        result = planner.plan_path(start, goal)

        assert not result.success, \
            "A* cannot cross the wall — must return failure, NOT a straight-line fallback"
        assert len(result.waypoints) == 0
        assert "BLOCKED" in result.failure_reason

    def test_no_topo_path_returns_failure(self):
        """Two disconnected rooms in the topology → success=False."""
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 20.0, 0.0)]
        tsg = MockTSG(rooms, {})  # no edges — disconnected
        trav = _make_open_trav()
        planner = _simple_planner(tsg, trav)

        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([20.0, 0.0, 0.0]))
        assert not result.success
        assert len(result.waypoints) == 0

    def test_missing_room_returns_failure(self):
        """Start/goal cannot be matched to any room → success=False."""
        tsg = MockTSG([], {})  # no rooms
        trav = _make_open_trav()
        planner = _simple_planner(tsg, trav)

        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]))
        assert not result.success


# ── HybridPlanner success tests ───────────────────────────────────────────────

class TestHybridPlannerSuccess:
    def _two_room_planner(self, trav=None):
        rooms = [
            MockRoom(1, 0.0, 0.0, {"x_min": -5, "x_max": 5, "y_min": -5, "y_max": 5}),
            MockRoom(2, 8.0, 0.0, {"x_min": 3, "x_max": 13, "y_min": -5, "y_max": 5}),
        ]
        tsg = MockTSG(rooms, {(1, 2): 8.0})
        if trav is None:
            trav = _make_open_trav(200, 100)
        return HybridPlanner(
            topology_graph=tsg,
            trav=trav,
            trav_res=0.2,
            trav_cx=8.0,  # centered on the map
            trav_cy=0.0,
        )

    def test_open_corridor_success(self):
        """Two rooms, fully open grid → success=True with non-empty waypoints."""
        planner = self._two_room_planner()
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]))
        assert result.success
        assert len(result.waypoints) >= 2

    def test_waypoints_start_and_end_correct(self):
        """First and last waypoints must exactly equal start and goal."""
        planner = self._two_room_planner()
        start = np.array([0.0, 0.0, 0.0])
        goal  = np.array([8.0, 0.0, 0.0])
        result = planner.plan_path(start, goal)
        if result.success:
            np.testing.assert_allclose(result.waypoints[0], start, atol=1e-6)
            np.testing.assert_allclose(result.waypoints[-1], goal, atol=1e-6)

    def test_path_stays_in_free_space(self):
        """Every waypoint must map to a traversable grid cell."""
        planner = self._two_room_planner()
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]))
        if not result.success:
            pytest.skip("Path not found — skip obstacle check")

        trav = planner._trav
        for wp in result.waypoints:
            ix, iy = _w2g(wp[0], wp[1],
                          planner._trav_cx, planner._trav_cy,
                          planner._trav_res,
                          planner._ox, planner._oy,
                          planner._nx, planner._ny)
            assert trav[ix, iy] < planner._obs_thr, \
                f"Waypoint {wp} maps to obstacle grid ({ix},{iy})"

    def test_update_traversability(self):
        """Hot-swapping the trav grid via update_traversability() should work."""
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 8.0, 0.0)]
        tsg = MockTSG(rooms, {(1, 2): 8.0})
        # Explicitly create with trav=None
        planner = HybridPlanner(topology_graph=tsg, trav=None)
        assert planner._trav is None

        new_trav = _make_open_trav(200, 100)
        planner.update_traversability(new_trav, res=0.2, cx=8.0, cy=0.0)
        assert planner._trav is not None
        assert planner._nx == 200

    def test_same_room_single_segment(self):
        """Start and goal in the same room → room_sequence length 1, still succeeds."""
        rooms = [MockRoom(1, 0.0, 0.0, {"x_min": -5, "x_max": 5, "y_min": -5, "y_max": 5})]
        tsg = MockTSG(rooms, {})
        trav = _make_open_trav(100, 100)
        planner = HybridPlanner(
            topology_graph=tsg,
            trav=trav,
            trav_res=0.2,
            trav_cx=0.0,
            trav_cy=0.0,
        )
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        # Same room: shortest_path returns [1], no geometry segments needed.
        # Result may be success=True (no segments) or success=False.
        # Important: must never produce a straight-line path.
        if result.success:
            assert len(result.waypoints) >= 1


# ── HybridPath dataclass tests ────────────────────────────────────────────────

class TestHybridPath:
    def test_failure_has_empty_waypoints(self):
        hp = HybridPlanner._failed("test reason")
        assert not hp.success
        assert hp.waypoints == []
        assert hp.failure_reason == "test reason"
        assert hp.total_cost == float("inf")

    def test_failure_reason_propagated(self):
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 100.0, 0.0)]
        tsg = MockTSG(rooms, {})  # disconnected
        planner = HybridPlanner(topology_graph=tsg, trav=_make_open_trav())
        result = planner.plan_path(np.array([0, 0, 0]), np.array([100, 0, 0]))
        assert not result.success
        assert "no topological path" in result.failure_reason
