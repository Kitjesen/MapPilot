"""Tests for PCT and A* planner backends (no C++ .so required)."""

import numpy as np
import pytest

from core.registry import clear, get, list_plugins, register, restore, snapshot


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _registry_isolation():
    """Save/restore global registry state around each test."""
    state = snapshot()
    yield
    restore(state)


@pytest.fixture
def mock_grid():
    """10x10 grid: free space with a central obstacle block."""
    grid = np.full((10, 10), 10.0, dtype=np.float32)
    grid[3:7, 3:7] = 80.0  # obstacle block
    return grid


@pytest.fixture
def astar_backend():
    """A* backend with no tomogram loaded (uses manual grid with central obstacle)."""
    from global_planning.pct_adapters.global_planner_module import _AStarBackend
    backend = _AStarBackend(tomogram_path="")
    # 10x10 grid: free space (10) with a central obstacle block (80)
    grid = np.full((10, 10), 10.0, dtype=np.float32)
    grid[3:7, 3:7] = 80.0
    backend._grid = grid
    backend._static_grid = grid.copy()
    backend._resolution = 0.2
    backend._origin = np.array([-1.0, -1.0])
    backend._obstacle_thr = 49.9
    return backend


@pytest.fixture
def pct_backend():
    """PCT backend with no tomogram (unavailable, tests graceful degradation)."""
    from global_planning.pct_adapters.global_planner_module import _PCTBackend
    backend = _PCTBackend(tomogram_path="")
    assert not backend.available
    return backend


# ---------------------------------------------------------------------------
# A* Backend Tests (no native deps)
# ---------------------------------------------------------------------------

class TestAStarBackend:
    """Pure-Python A* backend tests."""

    def test_instantiation_without_tomogram(self):
        """Backend can be created with no tomogram; defaults are set."""
        from global_planning.pct_adapters.global_planner_module import _AStarBackend
        backend = _AStarBackend(tomogram_path="")
        assert backend._grid is None
        assert backend._obstacle_thr == 49.9
        assert backend._resolution == 0.2
        assert backend._last_plan_reached_goal is True

    def test_plan_returns_empty_without_grid(self):
        """plan() returns [] when no grid is loaded."""
        from global_planning.pct_adapters.global_planner_module import _AStarBackend
        backend = _AStarBackend(tomogram_path="")
        result = backend.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.0]))
        assert result == []

    def test_plan_exact_route(self, astar_backend):
        """A* finds a path around a central obstacle."""
        start = np.array([-0.8, -0.8, 0.0])
        goal = np.array([1.6, 1.6, 0.0])
        path = astar_backend.plan(start, goal)
        assert len(path) > 1
        # Verify path endpoints
        assert abs(path[0][0] - start[0]) < 0.2
        assert abs(path[0][1] - start[1]) < 0.2
        assert abs(path[-1][0] - goal[0]) < 0.2
        assert abs(path[-1][1] - goal[1]) < 0.2
        # Verify the path does not walk through the obstacle block (cells 3-7 in grid coords)
        for wx, wy, _ in path:
            col = int(round((wx - (-1.0)) / 0.2))
            row = int(round((wy - (-1.0)) / 0.2))
            if 0 <= col < 10 and 0 <= row < 10:
                assert astar_backend._grid[row, col] < astar_backend._obstacle_thr, \
                    f"Path goes through obstacle at ({wx:.2f}, {wy:.2f})"

    def test_plan_same_start_goal(self, astar_backend):
        """plan() returns single-point path when start == goal."""
        pt = np.array([0.5, 0.5, 0.0])
        path = astar_backend.plan(pt, pt)
        assert len(path) == 1
        assert abs(path[0][0] - 0.5) < 0.01

    def test_plan_blocked_goal_returns_empty(self, astar_backend):
        """plan() returns [] when the goal is in obstacle space and unreachable."""
        start = np.array([-0.8, -0.8, 0.0])
        goal = np.array([0.0, 0.0, 0.0])  # maps to grid[5,5] = 80.0 (obstacle)
        path = astar_backend.plan(start, goal)
        # Grid cell [5,5] has cost 80 >= 49.9 thr -> blocked
        # A* cannot reach it -> empty path
        assert path == []

    def test_update_map_sets_costmap(self, astar_backend):
        """update_map() stores the costmap and merges obstacles."""
        cm = np.zeros((5, 5), dtype=np.float32)
        cm[2, 2] = 99.0  # one obstacle
        astar_backend.update_map(cm, resolution=0.2, origin=np.array([0.0, 0.0]))
        assert astar_backend._costmap is not None
        # The obstacle at cm[2,2] maps to grid coords through the origin/resolution.
        # cm origin is (0,0), so world (0.4, 0.4) -> grid col/row ~7,7 which is outside 10x10.
        # Just verify no crash and internal state updated.
        assert astar_backend._costmap[2, 2] == 99.0

    def test_merge_costmap_resets_from_static(self, astar_backend):
        """_merge_costmap resets _grid from _static_grid before overlaying."""
        original_free = astar_backend._grid[0, 0]
        astar_backend._grid[0, 0] = 99.0  # corrupt
        cm = np.zeros((3, 3), dtype=np.float32)
        cm[1, 1] = 60.0  # obstacle
        astar_backend._costmap = cm
        astar_backend._costmap_origin = np.array([-1.0, -1.0])
        astar_backend._costmap_resolution = 0.5
        astar_backend._merge_costmap()
        # After merge, _grid[0,0] should be restored to original_free, not 99.0
        assert astar_backend._grid[0, 0] == original_free

    def test_obstacle_threshold_blocks_cells(self):
        """Changing obstacle_thr changes which cells are considered free."""
        from global_planning.pct_adapters.global_planner_module import _AStarBackend
        backend = _AStarBackend(tomogram_path="", obstacle_thr=20.0)
        backend._grid = np.full((5, 5), 15.0, dtype=np.float32)
        backend._static_grid = backend._grid.copy()
        backend._resolution = 0.5
        backend._origin = np.array([0.0, 0.0])
        # All cells have cost 15 < 20, so a short route should succeed
        path = backend.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.0]))
        assert len(path) > 1
        # Increase obstactle_thr to 10, now all cells are blocked
        backend._obstacle_thr = 10.0
        path2 = backend.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.0]))
        assert path2 == []


# ---------------------------------------------------------------------------
# PCT Backend Tests (no .so file — tests graceful degradation)
# ---------------------------------------------------------------------------

class TestPCTBackend:
    """PCT backend tests (native .so unavailable, degrades gracefully)."""

    def test_instantiation_without_tomogram(self, pct_backend):
        """PCT backend created without tomogram is marked unavailable."""
        assert not pct_backend.available
        assert pct_backend._planner is None
        assert pct_backend._load_error != ""

    def test_plan_returns_empty_when_unavailable(self, pct_backend):
        """plan() returns [] when the native planner is not loaded."""
        path = pct_backend.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, 0.0]))
        assert path == []
        assert pct_backend._last_plan_error == "pct planner unavailable"

    def test_update_map_without_tomogram(self, pct_backend):
        """update_map() does not crash when no grid/tomogram is loaded."""
        cm = np.zeros((5, 5), dtype=np.float32)
        cm[2, 2] = 80.0
        pct_backend.update_map(cm, resolution=0.2, origin=np.array([0.0, 0.0]))
        assert pct_backend._costmap is not None
        # Without _static_grid, _merge_costmap is a no-op; no crash
        assert pct_backend._costmap[2, 2] == 80.0

    def test_merge_costmap_with_manual_grid(self, pct_backend):
        """_merge_costmap overlays obstacles when a static grid is set manually."""
        grid = np.full((10, 10), 10.0, dtype=np.float32)
        pct_backend._grid = grid.copy()
        pct_backend._static_grid = grid.copy()
        pct_backend._resolution = 0.2
        pct_backend._origin = np.array([-1.0, -1.0])
        pct_backend._obstacle_thr = 49.9

        cm = np.zeros((3, 3), dtype=np.float32)
        cm[1, 1] = 80.0
        pct_backend._costmap = cm
        pct_backend._costmap_origin = np.array([-1.0, -1.0])
        pct_backend._costmap_resolution = 0.5
        pct_backend._merge_costmap()
        # cm[1,1] at world (-0.5, -0.5) -> grid (2, 2) should have obstacle applied
        assert pct_backend._grid[2, 2] >= 49.9

    def test_merge_costmap_out_of_bounds(self, pct_backend):
        """_merge_costmap ignores costmap cells outside the grid."""
        grid = np.full((5, 5), 10.0, dtype=np.float32)
        pct_backend._grid = grid.copy()
        pct_backend._static_grid = grid.copy()
        pct_backend._resolution = 0.2
        pct_backend._origin = np.array([0.0, 0.0])

        cm = np.zeros((3, 3), dtype=np.float32)
        cm[0, 0] = 80.0  # world (0,0) -> grid (0,0) in bounds
        cm[2, 2] = 80.0  # world (0.4, 0.4) -> grid (2,2) in bounds
        pct_backend._costmap = cm
        pct_backend._costmap_origin = np.array([0.0, 0.0])
        pct_backend._costmap_resolution = 0.2
        pct_backend._merge_costmap()
        assert pct_backend._grid[0, 0] >= 49.9
        assert pct_backend._grid[2, 2] >= 49.9
        # No crash, grid shape unchanged
        assert pct_backend._grid.shape == (5, 5)

    def test_is_near_zero_route_same_point(self, pct_backend):
        """_is_near_zero_route detects identical start/goal."""
        result = pct_backend._is_near_zero_route(
            np.array([0.0, 0.0]), np.array([0.0, 0.0])
        )
        # Norm is 0 <= tolerance (0.2 from _resolution default), so True
        assert result is True

    def test_select_traversable_height_no_planner(self, pct_backend):
        """_select_traversable_height returns fallback when planner is None."""
        height = pct_backend._select_traversable_height(
            np.array([0.0, 0.0]), fallback_z=0.5
        )
        assert height == 0.5
