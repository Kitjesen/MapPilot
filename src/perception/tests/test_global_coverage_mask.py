"""
test_global_coverage_mask.py — GlobalCoverageMask 单元测试

覆盖:
  - 初始化状态
  - world_to_grid / grid_to_world 坐标转换
  - update_from_local_grid: 占据栅格→GCM更新
  - get_coverage_ratio: 0 → 正值
  - get_frontier_cells: 已覆盖区域的边界检测
  - get_frontier_clusters: 聚类
  - get_statistics: 统计字段
  - CoverageCell: hash/eq
"""

import unittest

import numpy as np

from perception.scene_understanding.global_coverage_mask import (
    CoverageCell,
    GlobalCoverageMask,
)


class TestGlobalCoverageMaskInit(unittest.TestCase):
    def test_default_resolution(self):
        gcm = GlobalCoverageMask()
        self.assertAlmostEqual(gcm.resolution, 0.5)

    def test_custom_resolution(self):
        gcm = GlobalCoverageMask(resolution=1.0)
        self.assertAlmostEqual(gcm.resolution, 1.0)

    def test_empty_state(self):
        gcm = GlobalCoverageMask()
        self.assertEqual(gcm.covered_cells, 0)
        self.assertEqual(gcm.total_cells, 0)
        self.assertEqual(gcm.get_coverage_ratio(), 0.0)
        self.assertEqual(gcm.get_frontier_cells(), [])


class TestCoordinateConversion(unittest.TestCase):
    def setUp(self):
        self.gcm = GlobalCoverageMask(resolution=1.0)

    def test_world_to_grid_origin(self):
        gx, gy = self.gcm.world_to_grid(np.array([0.0, 0.0]))
        self.assertEqual(gx, 0)
        self.assertEqual(gy, 0)

    def test_world_to_grid_positive(self):
        gx, gy = self.gcm.world_to_grid(np.array([3.0, 4.0]))
        self.assertEqual(gx, 3)
        self.assertEqual(gy, 4)

    def test_world_to_grid_negative(self):
        gx, gy = self.gcm.world_to_grid(np.array([-2.0, -3.0]))
        self.assertEqual(gx, -2)
        self.assertEqual(gy, -3)

    def test_grid_to_world_roundtrip(self):
        gcm = GlobalCoverageMask(resolution=0.5)
        world = np.array([1.5, 2.5])
        grid = gcm.world_to_grid(world)
        back = gcm.grid_to_world(grid)
        # Grid→world gives center of cell, so check within resolution
        self.assertAlmostEqual(back[0], world[0], delta=gcm.resolution)
        self.assertAlmostEqual(back[1], world[1], delta=gcm.resolution)


class TestUpdateFromLocalGrid(unittest.TestCase):
    def setUp(self):
        self.gcm = GlobalCoverageMask(resolution=0.5)

    def test_empty_grid_no_coverage(self):
        grid = np.ones((10, 10, 3))  # all occupied → no free cells
        self.gcm.update_from_local_grid(
            grid, grid_resolution=0.5, grid_origin=np.array([0.0, 0.0, 0.0])
        )
        self.assertEqual(self.gcm.covered_cells, 0)

    def test_free_cells_marked_covered(self):
        grid = np.zeros((5, 5, 1))  # all free (value < 0.5)
        self.gcm.update_from_local_grid(
            grid, grid_resolution=0.5, grid_origin=np.array([0.0, 0.0, 0.0])
        )
        self.assertGreater(self.gcm.covered_cells, 0)

    def test_coverage_ratio_positive_after_update(self):
        grid = np.zeros((5, 5, 1))
        self.gcm.update_from_local_grid(
            grid, grid_resolution=0.5, grid_origin=np.array([0.0, 0.0, 0.0])
        )
        self.assertGreater(self.gcm.get_coverage_ratio(), 0.0)
        self.assertLessEqual(self.gcm.get_coverage_ratio(), 1.0)

    def test_incremental_update_accumulates(self):
        """Two separate grids covering different areas → more cells after second update."""
        grid = np.zeros((3, 3, 1))
        self.gcm.update_from_local_grid(
            grid, grid_resolution=1.0, grid_origin=np.array([0.0, 0.0, 0.0])
        )
        cells_after_first = self.gcm.covered_cells

        self.gcm.update_from_local_grid(
            grid, grid_resolution=1.0, grid_origin=np.array([10.0, 10.0, 0.0])
        )
        cells_after_second = self.gcm.covered_cells
        self.assertGreaterEqual(cells_after_second, cells_after_first)

    def test_mixed_free_occupied(self):
        # 3D grid: z=0 layer free, z=1 layer occupied
        grid = np.zeros((4, 4, 2))
        grid[:, :, 1] = 1.0  # z=1 occupied
        self.gcm.update_from_local_grid(
            grid, grid_resolution=0.5, grid_origin=np.array([0.0, 0.0, 0.0])
        )
        # Only z=0 cells should be covered (4×4 = 16 free voxels)
        self.assertGreater(self.gcm.covered_cells, 0)


class TestGetFrontierCells(unittest.TestCase):
    def test_isolated_single_cell_is_frontier(self):
        """A single covered cell surrounded by uncovered → should be frontier."""
        gcm = GlobalCoverageMask(resolution=1.0)
        # Mark one cell covered
        gcm._mark_cell_covered(0, 0)
        frontiers = gcm.get_frontier_cells()
        self.assertIn((0, 0), frontiers)

    def test_large_covered_interior_not_frontier(self):
        """A cell completely surrounded by covered neighbors is not a frontier."""
        gcm = GlobalCoverageMask(resolution=1.0)
        # Mark a 3×3 block
        for x in range(-1, 2):
            for y in range(-1, 2):
                gcm._mark_cell_covered(x, y)
        frontiers = gcm.get_frontier_cells()
        # Center cell (0,0) surrounded by all 8 neighbors → not frontier
        self.assertNotIn((0, 0), frontiers)

    def test_edge_cells_are_frontiers(self):
        """Edge cells of a covered block should be frontiers."""
        gcm = GlobalCoverageMask(resolution=1.0)
        for x in range(-1, 2):
            for y in range(-1, 2):
                gcm._mark_cell_covered(x, y)
        frontiers = gcm.get_frontier_cells()
        # All 8 edge cells should be frontiers
        self.assertIn((-1, 0), frontiers)
        self.assertIn((1, 0), frontiers)


class TestGetFrontierClusters(unittest.TestCase):
    def test_returns_list_of_lists(self):
        gcm = GlobalCoverageMask(resolution=1.0)
        gcm._mark_cell_covered(0, 0)
        clusters = gcm.get_frontier_clusters(min_cluster_size=1)
        self.assertIsInstance(clusters, list)
        for cluster in clusters:
            self.assertIsInstance(cluster, list)

    def test_min_cluster_size_filters_small_clusters(self):
        gcm = GlobalCoverageMask(resolution=1.0)
        gcm._mark_cell_covered(0, 0)
        # Very high min_cluster_size should filter out any small cluster
        clusters = gcm.get_frontier_clusters(min_cluster_size=1000)
        self.assertEqual(clusters, [])

    def test_cluster_cells_are_frontiers(self):
        gcm = GlobalCoverageMask(resolution=1.0)
        # Cover a line
        for x in range(5):
            gcm._mark_cell_covered(x, 0)
        clusters = gcm.get_frontier_clusters(min_cluster_size=1)
        frontiers = set(gcm.get_frontier_cells())
        for cluster in clusters:
            for cell in cluster:
                self.assertIn(cell, frontiers)


class TestGetStatistics(unittest.TestCase):
    def test_statistics_fields(self):
        gcm = GlobalCoverageMask()
        stats = gcm.get_statistics()
        self.assertIn("covered_cells", stats)
        self.assertIn("total_cells", stats)
        self.assertIn("coverage_ratio", stats)

    def test_statistics_after_update(self):
        gcm = GlobalCoverageMask(resolution=1.0)
        grid = np.zeros((4, 4, 1))
        gcm.update_from_local_grid(
            grid, grid_resolution=1.0, grid_origin=np.array([0.0, 0.0, 0.0])
        )
        stats = gcm.get_statistics()
        self.assertGreater(stats["covered_cells"], 0)
        self.assertGreater(stats["coverage_ratio"], 0.0)


class TestCoverageCell(unittest.TestCase):
    def test_hash_based_on_xy(self):
        c1 = CoverageCell(x=1, y=2)
        c2 = CoverageCell(x=1, y=2, covered=True, visit_count=5)
        self.assertEqual(hash(c1), hash(c2))

    def test_equality(self):
        c1 = CoverageCell(x=3, y=4)
        c2 = CoverageCell(x=3, y=4)
        self.assertEqual(c1, c2)

    def test_inequality_different_coords(self):
        c1 = CoverageCell(x=1, y=2)
        c2 = CoverageCell(x=1, y=3)
        self.assertNotEqual(c1, c2)

    def test_in_set(self):
        c1 = CoverageCell(x=0, y=0)
        c2 = CoverageCell(x=0, y=0, covered=True)
        s = {c1}
        self.assertIn(c2, s)


if __name__ == "__main__":
    unittest.main()
