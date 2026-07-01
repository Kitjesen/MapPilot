"""Tests for planner goal safety 閳?_find_safe_goal BFS, PCT grid extraction,
costmap overlay, and GlobalPlanner.update_map integration.

All tests are pure-Python, no ROS2 / hardware / .so files required.
"""

from __future__ import annotations

import os
import pickle
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from nav.services.plan.global_planner.service import GlobalPlanner

# ---------------------------------------------------------------------------
# Helper: create a fake tomogram pickle for testing
# ---------------------------------------------------------------------------


class _DummyTomogramPlanner:
    """Small planner stand-in for PCT height selection tests."""

    def __init__(
        self,
        traversability: np.ndarray,
        elevation: np.ndarray | None = None,
        slice_h0: float = -0.5,
        slice_dh: float = 0.5,
    ):
        self.layers_t = traversability
        self.layers_g = elevation
        self.slice_h0 = slice_h0
        self.slice_dh = slice_dh
        self.n_slice = traversability.shape[0]

    def pos2idx(self, _pos):
        return np.array([1, 1], dtype=np.float32)

    def pos2slice(self, z):
        raw_slice = round((z - self.slice_h0) / self.slice_dh)
        return float(np.clip(raw_slice, 0, self.n_slice - 1))

    def get_surface_height(self, _pos):
        return 0.0


def _make_tomogram_pickle(
    shape: tuple = (100, 100),
    resolution: float = 0.2,
    center: tuple = (0.0, 0.0),
    obstacle_cells: list | None = None,
) -> str:
    """Create a temporary tomogram pickle and return its path.

    Returns a pickle with data shape (5, 1, H, W) 閳?standard tomogram format.
    Channel 0 = traversability (0 = free, 100 = obstacle).
    """
    h, w = shape
    data = np.zeros((5, 1, h, w), dtype=np.float32)
    if obstacle_cells:
        for row, col in obstacle_cells:
            data[0, 0, row, col] = 100.0

    tomo = {
        "data": data,
        "resolution": resolution,
        "center": list(center),
    }
    fd, path = tempfile.mkstemp(suffix=".pickle")
    with open(path, "wb") as f:
        pickle.dump(tomo, f)
    os.close(fd)
    return path


class _GridPlanner:
    def __init__(self, grid, resolution=0.2, origin=None):
        self._grid = np.asarray(grid, dtype=np.float32).copy()
        self._resolution = float(resolution)
        self._origin = np.asarray(origin if origin is not None else [0.0, 0.0], dtype=float)
        self._costmap = None

    @classmethod
    def from_tomogram(cls, path: str):
        with open(path, "rb") as f:
            tomo = pickle.load(f)
        grid = np.asarray(tomo["data"][0, 0], dtype=np.float32)
        resolution = float(tomo.get("resolution", 0.2))
        center = np.asarray(tomo.get("center", [0.0, 0.0]), dtype=float)
        h, w = grid.shape
        origin = center - np.asarray([w * resolution / 2.0, h * resolution / 2.0])
        return cls(grid, resolution=resolution, origin=origin)

    def update_map(self, grid, resolution=0.2, origin=None):
        del resolution, origin
        self._costmap = np.asarray(grid, dtype=np.float32).copy()


# ---------------------------------------------------------------------------
# 1. A* backend 閳?_find_safe_goal with tomogram grid
# ---------------------------------------------------------------------------

class TestGridGoalSafety(unittest.TestCase):
    """Test _find_safe_goal BFS with a planner-loaded tomogram grid."""

    def setUp(self):
        # 50x50 grid, 0.5m resolution, centered at (0, 0)
        # Place obstacles in a block around row=25, col=25 (center of grid)
        obstacles = [(r, c) for r in range(24, 27) for c in range(24, 27)]
        self._tomo_path = _make_tomogram_pickle(
            shape=(50, 50), resolution=0.5, center=(0, 0),
            obstacle_cells=obstacles,
        )
        self._svc = GlobalPlanner(
            planner_name="pct",
            tomogram=self._tomo_path,
            obstacle_thr=49.9,
        )
        self._svc._backend = _GridPlanner.from_tomogram(self._tomo_path)

    def tearDown(self):
        os.unlink(self._tomo_path)

    def test_has_map(self):
        self.assertTrue(self._svc.has_map)

    def test_free_goal_unchanged(self):
        """Goal on free cell should pass through unchanged."""
        # Grid origin = (0,0) - (25*0.5, 25*0.5) = (-12.5, -12.5)
        # Use (-10, -10) which maps to col=5, row=5 閳?far from obstacles at center
        goal = np.array([-10.0, -10.0, 0.0])
        safe = self._svc._find_safe_goal(goal)
        self.assertIsNotNone(safe)
        self.assertAlmostEqual(safe[0], -10.0, places=1)
        self.assertAlmostEqual(safe[1], -10.0, places=1)

    def test_obstacle_goal_adjusted(self):
        """Goal on obstacle should be moved to nearest free cell."""
        # The center of grid at (0, 0) has obstacles
        goal = np.array([0.0, 0.0, 0.0])
        # Place goal at the obstacle cluster
        # Grid center = col=25, row=25. World = center - (25*0.5, 25*0.5) + (25*0.5, 25*0.5) = (0, 0)
        # Actually, origin = center - (W*res/2, H*res/2) = (0,0) - (12.5, 12.5) = (-12.5, -12.5)
        # So world (0,0) 閳?col = (0-(-12.5))/0.5 = 25, row = 25 閳?obstacle!
        safe = self._svc._find_safe_goal(goal)
        self.assertIsNotNone(safe)
        # Should be near (0,0) but not exactly on it
        self.assertLess(np.linalg.norm(safe[:2]), 3.0)

    def test_z_preserved(self):
        """Goal Z coordinate should be preserved after BFS adjustment."""
        goal = np.array([0.0, 0.0, 5.5])
        safe = self._svc._find_safe_goal(goal)
        self.assertIsNotNone(safe)
        self.assertAlmostEqual(safe[2], 5.5)

    def test_no_free_cell_returns_none(self):
        """If all cells within tolerance are obstacles, return None."""
        # Fill entire grid with obstacles
        all_obs = [(r, c) for r in range(50) for c in range(50)]
        path = _make_tomogram_pickle(
            shape=(50, 50), resolution=0.5, center=(0, 0),
            obstacle_cells=all_obs,
        )
        svc = GlobalPlanner(planner_name="pct", tomogram=path, obstacle_thr=49.9)
        svc._backend = _GridPlanner.from_tomogram(path)
        result = svc._find_safe_goal(np.array([0.0, 0.0, 0.0]), tolerance=2.0)
        self.assertIsNone(result)
        os.unlink(path)


# ---------------------------------------------------------------------------
# 2. PCT planner 閳?grid extraction from tomogram
# ---------------------------------------------------------------------------

class TestPCTGridExtraction(unittest.TestCase):
    """Test that PCT planner extracts _grid from tomogram for goal safety BFS.

    Since ele_planner.so is not available on dev machines, we test
    _extract_grid() directly by instantiating a partial PCTPlanner.
    """

    def _make_pct_backend(self):
        """Create a PCTPlanner without the C++ .so (won't be available)."""
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        # Constructor will fail to import TomogramPlanner 閳?that's OK.
        # We manually call _extract_grid() to test grid extraction.
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = None
        backend._obstacle_thr = 49.9
        backend._available = False
        backend._load_error = "test"
        backend._grid = None
        backend._trav_3d = None
        backend._resolution = 0.2
        backend._origin = np.zeros(2)
        backend._slice_h0 = 0.0
        backend._slice_dh = 0.5
        backend._costmap = None
        backend._costmap_resolution = 0.2
        backend._costmap_origin = np.zeros(2)
        return backend

    def test_extract_grid_4d_tomogram(self):
        """Standard 4D tomogram 閳?ground-floor 2D grid extracted."""
        path = _make_tomogram_pickle(shape=(80, 60), resolution=0.3, center=(5, 10))
        backend = self._make_pct_backend()
        backend._extract_grid(path)
        os.unlink(path)

        self.assertIsNotNone(backend._grid)
        self.assertEqual(backend._grid.shape, (80, 60))
        self.assertIsNotNone(backend._trav_3d)
        self.assertEqual(backend._trav_3d.shape, (1, 80, 60))
        self.assertAlmostEqual(backend._resolution, 0.3)
        # origin = center - (W*res/2, H*res/2) = (5, 10) - (9, 12) = (-4, -2)
        self.assertAlmostEqual(backend._origin[0], 5 - 60 * 0.3 / 2, places=3)
        self.assertAlmostEqual(backend._origin[1], 10 - 80 * 0.3 / 2, places=3)

    def test_extract_grid_preserves_obstacles(self):
        """Obstacle cells in tomogram should be reflected in _grid."""
        obstacles = [(10, 20), (11, 20), (10, 21)]
        path = _make_tomogram_pickle(shape=(50, 50), resolution=0.2, center=(0, 0),
                                     obstacle_cells=obstacles)
        backend = self._make_pct_backend()
        backend._extract_grid(path)
        os.unlink(path)

        self.assertGreaterEqual(backend._grid[10, 20], 100.0)
        self.assertGreaterEqual(backend._grid[11, 20], 100.0)
        self.assertGreaterEqual(backend._grid[10, 21], 100.0)
        # A free cell should be 0
        self.assertAlmostEqual(backend._grid[0, 0], 0.0)

    def test_extract_grid_bad_pickle(self):
        """Non-dict pickle should not crash."""
        fd, path = tempfile.mkstemp(suffix=".pickle")
        with open(path, "wb") as f:
            pickle.dump("not a dict", f)
        os.close(fd)

        backend = self._make_pct_backend()
        backend._extract_grid(path)  # Should not raise
        os.unlink(path)
        self.assertIsNone(backend._grid)

    def test_extract_grid_missing_file(self):
        """Missing file should not crash."""
        backend = self._make_pct_backend()
        backend._extract_grid("/nonexistent/tomogram.pickle")
        self.assertIsNone(backend._grid)

    def test_static_grid_preserved(self):
        """After _extract_grid, _static_grid should be a clean copy."""
        path = _make_tomogram_pickle(shape=(30, 30), resolution=0.2, center=(0, 0))
        backend = self._make_pct_backend()
        backend._extract_grid(path)
        os.unlink(path)

        self.assertIsNotNone(backend._static_grid)
        np.testing.assert_array_equal(backend._grid, backend._static_grid)
        # Modifying _grid should not affect _static_grid
        backend._grid[0, 0] = 999.0
        self.assertAlmostEqual(backend._static_grid[0, 0], 0.0)

    def test_select_traversable_height_skips_blocked_surface_slice(self):
        """PCT height selection should not choose a blocked upper slice."""
        backend = self._make_pct_backend()
        traversability = np.full((3, 3, 3), 50.0, dtype=np.float32)
        traversability[0, 1, 1] = 13.0
        elevation = np.full_like(traversability, np.nan)
        elevation[1, 1, 1] = 0.0

        backend._planner = _DummyTomogramPlanner(traversability, elevation)
        backend._trav_3d = traversability
        backend._slice_h0 = -0.5
        backend._slice_dh = 0.5

        height = backend._select_traversable_height(np.array([0.0, 0.0]), 0.0)

        self.assertAlmostEqual(height, -0.5)
        self.assertEqual(int(backend._planner.pos2slice(height)), 0)


# ---------------------------------------------------------------------------
# 3. PCT planner 閳?costmap overlay (update_map + _merge_costmap)
# ---------------------------------------------------------------------------

class TestPCTCostmapOverlay(unittest.TestCase):
    """Test live costmap merging into PCT's _grid."""

    def _make_backend_with_grid(self, grid_shape=(50, 50)):
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = None
        backend._obstacle_thr = 49.9
        backend._available = False
        backend._load_error = "test"
        backend._grid = np.zeros(grid_shape, dtype=np.float32)
        backend._static_grid = backend._grid.copy()
        backend._resolution = 0.2
        backend._origin = np.array([-5.0, -5.0])
        backend._costmap = None
        backend._costmap_resolution = 0.2
        backend._costmap_origin = np.zeros(2)
        return backend

    def test_update_map_stores_costmap(self):
        backend = self._make_backend_with_grid()
        costmap = np.zeros((20, 20), dtype=np.float32)
        backend.update_map(costmap, resolution=0.3, origin=np.array([1.0, 2.0]))
        self.assertIsNotNone(backend._costmap)
        self.assertAlmostEqual(backend._costmap_resolution, 0.3)
        self.assertAlmostEqual(backend._costmap_origin[0], 1.0)

    def test_costmap_obstacles_merged(self):
        """Costmap obstacles should appear in _grid after update_map."""
        backend = self._make_backend_with_grid()
        # Costmap: 10x10, same resolution/origin as grid
        costmap = np.zeros((10, 10), dtype=np.float32)
        costmap[5, 5] = 100.0  # obstacle at costmap (5, 5)
        backend.update_map(costmap, resolution=0.2, origin=np.array([-5.0, -5.0]))

        # This costmap cell maps to grid cell (5, 5)
        self.assertGreaterEqual(backend._grid[5, 5], 100.0)
        # Other cells should still be free
        self.assertAlmostEqual(backend._grid[0, 0], 0.0)

    def test_costmap_reset_on_update(self):
        """Each update_map should reset _grid from _static_grid before merging."""
        backend = self._make_backend_with_grid()

        # First costmap: obstacle at (3, 3)
        cm1 = np.zeros((10, 10), dtype=np.float32)
        cm1[3, 3] = 100.0
        backend.update_map(cm1, resolution=0.2, origin=np.array([-5.0, -5.0]))
        self.assertGreaterEqual(backend._grid[3, 3], 100.0)

        # Second costmap: no obstacle at (3, 3), obstacle at (7, 7)
        cm2 = np.zeros((10, 10), dtype=np.float32)
        cm2[7, 7] = 100.0
        backend.update_map(cm2, resolution=0.2, origin=np.array([-5.0, -5.0]))

        # (3, 3) should be clear again (reset from static)
        self.assertAlmostEqual(backend._grid[3, 3], 0.0)
        # (7, 7) should have the new obstacle
        self.assertGreaterEqual(backend._grid[7, 7], 100.0)

    def test_static_obstacles_preserved_after_costmap(self):
        """Static tomogram obstacles should not be cleared by costmap overlay."""
        backend = self._make_backend_with_grid()
        # Set static obstacle
        backend._static_grid[10, 10] = 100.0
        backend._grid[10, 10] = 100.0

        # Empty costmap
        cm = np.zeros((5, 5), dtype=np.float32)
        backend.update_map(cm, resolution=0.2, origin=np.array([-5.0, -5.0]))

        # Static obstacle should survive
        self.assertGreaterEqual(backend._grid[10, 10], 100.0)

    def test_different_resolution_costmap(self):
        """Costmap with different resolution should be projected correctly."""
        backend = self._make_backend_with_grid(grid_shape=(100, 100))
        backend._resolution = 0.1
        backend._origin = np.array([0.0, 0.0])
        backend._static_grid = backend._grid.copy()

        # Costmap: 10x10, resolution=0.5, origin=(0, 0)
        # Costmap cell (2, 3) 閳?world (1.5, 1.0) 閳?grid (15, 10)
        costmap = np.zeros((10, 10), dtype=np.float32)
        costmap[2, 3] = 100.0
        backend.update_map(costmap, resolution=0.5, origin=np.array([0.0, 0.0]))

        self.assertGreaterEqual(backend._grid[10, 15], 50.0)  # approximate mapping


# ---------------------------------------------------------------------------
# 4. GlobalPlanner 閳?update_map reaches PCT planner
# ---------------------------------------------------------------------------

class TestGlobalPlannerCostmap(unittest.TestCase):
    """Test that GlobalPlanner.update_map reaches the active planner."""

    def test_update_map_reaches_planner(self):
        """Costmap updates should reach the planner without discarding static tomogram."""
        path = _make_tomogram_pickle(shape=(30, 30), resolution=0.2, center=(0, 0))
        svc = GlobalPlanner(planner_name="pct", tomogram=path)
        svc._backend = _GridPlanner.from_tomogram(path)
        os.unlink(path)

        new_grid = np.zeros((20, 20), dtype=np.float32)
        new_grid[10, 10] = 100.0
        svc.update_map(new_grid, resolution=0.2, origin=np.array([0.0, 0.0]))

        # Grid should be updated
        backend = svc._backend
        self.assertEqual(backend._grid.shape, (30, 30))
        self.assertEqual(backend._costmap[10, 10], 100.0)

    def test_update_map_pct_has_method(self):
        """PCT planner now has update_map method."""
        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        self.assertTrue(hasattr(PCTPlanner, "update_map"))

    def test_find_safe_goal_without_grid_returns_none(self):
        """If backend has no grid, _find_safe_goal returns None (passthrough)."""
        with tempfile.TemporaryDirectory() as tmpdir:
            with mock.patch.dict(os.environ, {"NAV_MAP_DIR": tmpdir}):
                svc = GlobalPlanner(planner_name="pct", tomogram="")
                result = svc._find_safe_goal(np.array([1.0, 2.0, 0.0]))
        self.assertIsNone(result)

    def test_find_safe_goal_out_of_bounds(self):
        """Goal far outside the grid should trigger BFS search."""
        path = _make_tomogram_pickle(shape=(20, 20), resolution=0.5, center=(0, 0))
        svc = GlobalPlanner(planner_name="pct", tomogram=path)
        svc._backend = _GridPlanner.from_tomogram(path)
        os.unlink(path)

        # Goal way outside the grid 閳?BFS will start out of bounds
        result = svc._find_safe_goal(np.array([100.0, 100.0, 0.0]), tolerance=1.0)
        # Should return None since no reachable free cell within tolerance
        self.assertIsNone(result)

    def test_find_safe_goal_rejects_disconnected_free_goal(self):
        """A free cell behind a wall is not a safe goal for the current start."""
        svc = GlobalPlanner(planner_name="pct", tomogram="")

        grid = np.zeros((20, 20), dtype=np.float32)
        grid[10, :] = 100.0
        svc._backend = _GridPlanner(grid, resolution=1.0, origin=np.array([0.0, 0.0]))
        svc.update_map(grid, resolution=1.0, origin=np.array([0.0, 0.0]))

        safe = svc._find_safe_goal(
            np.array([2.0, 15.0, 0.0]),
            tolerance=2.0,
            start=np.array([2.0, 2.0, 0.0]),
        )

        self.assertIsNone(safe)


# ---------------------------------------------------------------------------
# 5. Integration: PCT grid + _find_safe_goal
# ---------------------------------------------------------------------------

class TestPCTFindSafeGoalIntegration(unittest.TestCase):
    """Integration test: PCT grid extraction 閳?_find_safe_goal BFS."""

    def test_pct_grid_enables_find_safe_goal(self):
        """After loading a tomogram, PCT _grid enables goal safety checking."""
        obstacles = [(25, 25), (25, 26), (26, 25), (26, 26)]
        path = _make_tomogram_pickle(
            shape=(50, 50), resolution=0.2, center=(0, 0),
            obstacle_cells=obstacles,
        )

        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = None
        backend._obstacle_thr = 49.9
        backend._available = False
        backend._load_error = "test"
        backend._grid = None
        backend._resolution = 0.2
        backend._origin = np.zeros(2)
        backend._costmap = None
        backend._costmap_resolution = 0.2
        backend._costmap_origin = np.zeros(2)
        backend._extract_grid(path)
        os.unlink(path)

        # Wire it into GlobalPlanner manually
        svc = GlobalPlanner.__new__(GlobalPlanner)
        svc._backend = backend
        svc._obstacle_thr = 49.9
        svc._downsample_dist = 2.0

        # Goal at (0, 0) 閳?grid center (25, 25) 閳?obstacle
        # origin = (0,0) - (50*0.2/2, 50*0.2/2) = (-5, -5)
        # world (0,0) 閳?col = (0-(-5))/0.2 = 25, row = 25 閳?obstacle!
        safe = svc._find_safe_goal(np.array([0.0, 0.0, 0.0]))
        self.assertIsNotNone(safe)
        # Should be moved away from (0, 0)
        dist = np.linalg.norm(safe[:2])
        self.assertGreater(dist, 0.1)
        self.assertLess(dist, 3.0)

    def test_pct_costmap_overlay_blocks_goal(self):
        """Dynamic costmap obstacle blocks a previously-free goal."""
        # Clean tomogram 閳?no static obstacles
        path = _make_tomogram_pickle(shape=(50, 50), resolution=0.2, center=(0, 0))

        from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
        backend = PCTPlanner.__new__(PCTPlanner)
        backend._planner = None
        backend._obstacle_thr = 49.9
        backend._available = False
        backend._load_error = "test"
        backend._grid = None
        backend._resolution = 0.2
        backend._origin = np.zeros(2)
        backend._costmap = None
        backend._costmap_resolution = 0.2
        backend._costmap_origin = np.zeros(2)
        backend._extract_grid(path)
        os.unlink(path)

        svc = GlobalPlanner.__new__(GlobalPlanner)
        svc._backend = backend
        svc._obstacle_thr = 49.9
        svc._downsample_dist = 2.0

        # Goal at (0, 0) should be free initially
        safe = svc._find_safe_goal(np.array([0.0, 0.0, 0.0]))
        self.assertIsNotNone(safe)
        self.assertAlmostEqual(safe[0], 0.0, places=1)

        # Now push a costmap with obstacle at the same location
        origin = backend._origin
        costmap = np.zeros((50, 50), dtype=np.float32)
        costmap[25, 25] = 100.0  # block center
        costmap[24, 25] = 100.0
        costmap[25, 24] = 100.0
        costmap[26, 25] = 100.0
        costmap[25, 26] = 100.0
        backend.update_map(costmap, resolution=0.2, origin=origin)

        # Now (0, 0) should be blocked and goal adjusted
        safe2 = svc._find_safe_goal(np.array([0.0, 0.0, 0.0]))
        self.assertIsNotNone(safe2)
        # Should be moved away from the blocked center
        dist = np.linalg.norm(safe2[:2])
        self.assertGreater(dist, 0.1)


if __name__ == "__main__":
    unittest.main()
