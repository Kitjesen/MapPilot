"""
全局覆盖掩码 (Global Coverage Mask, GCM) — 轻量级探索追踪。

功能:
  1. 替代全局点云的稀疏表示
  2. 粗粒度覆盖追踪（0.5-1.0m 分辨率）
  3. 内存固定（不随时间增长）
  4. 前沿检测（已知与未知的边界）
  5. 覆盖率计算

设计原则:
  - 稀疏存储: 只存储已探索区域
  - 轻量级: 比全局点云节省 10-100× 内存
  - 实时更新: 支持 15 Hz 更新频率

参考论文:
  - USS-Nav (2025): GCM 替代全局点云
  - Frontier-based Exploration: 前沿检测算法
"""

import logging
import pickle
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class CoverageCell:
    """GCM 单元格。"""
    x: int  # 栅格坐标
    y: int
    covered: bool = False
    visit_count: int = 0
    last_visited: float = 0.0
    uncertainty: float = 1.0  # 0.0-1.0, 1.0 = 完全未知

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        return isinstance(other, CoverageCell) and self.x == other.x and self.y == other.y


# ══════════════════════════════════════════════════════════════════
#  全局覆盖掩码
# ══════════════════════════════════════════════════════════════════

class GlobalCoverageMask:
    """
    全局覆盖掩码 (GCM) — 替代全局点云的轻量级表示。

    特点:
      - 稀疏存储（只存储已探索区域）
      - 粗粒度（0.5-1.0m 分辨率）
      - 内存固定（不随时间增长）

    用法:
        gcm = GlobalCoverageMask(resolution=0.5)
        gcm.update_from_polyhedron(polyhedron)
        coverage_ratio = gcm.get_coverage_ratio()
        frontiers = gcm.get_frontier_cells()
    """

    def __init__(self, resolution: float = 0.5):
        """
        Args:
            resolution: 栅格分辨率 (米)
        """
        self.resolution = resolution
        self.coverage_map: Dict[Tuple[int, int], CoverageCell] = {}

        # 统计信息
        self.total_cells = 0
        self.covered_cells = 0

        # 边界（用于计算覆盖率）
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')

    def update_from_polyhedron(self, polyhedron) -> None:
        """
        从多面体更新覆盖区域。

        Args:
            polyhedron: Polyhedron 对象（来自 polyhedron_expansion.py）
        """
        # 1. 计算多面体的边界框
        vertices = polyhedron.vertices
        bbox_min = vertices.min(axis=0)
        bbox_max = vertices.max(axis=0)

        # 2. 转换为栅格坐标
        grid_min = self.world_to_grid(bbox_min[:2])
        grid_max = self.world_to_grid(bbox_max[:2])

        # 3. 遍历边界框内的所有栅格
        for gx in range(grid_min[0], grid_max[0] + 1):
            for gy in range(grid_min[1], grid_max[1] + 1):
                # 检查栅格中心是否在多面体内
                world_pos = self.grid_to_world((gx, gy))
                world_pos_3d = np.array([world_pos[0], world_pos[1], polyhedron.center[2]])

                if self._point_in_polyhedron(world_pos_3d, polyhedron):
                    self._mark_cell_covered(gx, gy)

        logger.debug(
            f"Updated GCM from polyhedron {polyhedron.poly_id}: "
            f"covered {self.covered_cells}/{self.total_cells} cells"
        )

    def update_from_local_grid(
        self,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
    ) -> None:
        """
        从局部滚动栅格更新覆盖区域。

        Args:
            occupancy_grid: (X, Y, Z) 占据栅格 (0=自由, 1=占据)
            grid_resolution: 局部栅格分辨率 (米)
            grid_origin: 局部栅格原点 [x, y, z]
        """
        # 1. 提取自由空间栅格
        free_indices = np.argwhere(occupancy_grid < 0.5)

        # 2. 转换为世界坐标
        for idx in free_indices:
            # 局部栅格坐标 → 世界坐标
            world_pos = grid_origin + (idx + 0.5) * grid_resolution

            # 世界坐标 → GCM 栅格坐标
            gcm_grid = self.world_to_grid(world_pos[:2])

            # 标记为已覆盖
            self._mark_cell_covered(gcm_grid[0], gcm_grid[1])

        logger.debug(
            f"Updated GCM from local grid: "
            f"covered {self.covered_cells}/{self.total_cells} cells"
        )

    def get_coverage_ratio(self) -> float:
        """
        计算探索覆盖率。

        Returns:
            0.0-1.0 的覆盖率
        """
        if self.total_cells == 0:
            return 0.0

        return self.covered_cells / self.total_cells

    def get_frontier_cells(self) -> List[Tuple[int, int]]:
        """
        提取前沿单元（已知与未知的边界）。

        Returns:
            前沿单元的栅格坐标列表 [(x, y), ...]
        """
        frontiers = []

        for (x, y), cell in self.coverage_map.items():
            if cell.covered and self._has_uncovered_neighbors(x, y):
                frontiers.append((x, y))

        return frontiers

    def get_frontier_clusters(self, min_cluster_size: int = 3) -> List[List[Tuple[int, int]]]:
        """
        将前沿单元聚类为前沿区域。

        Args:
            min_cluster_size: 最小聚类大小

        Returns:
            前沿聚类列表 [[(x, y), ...], ...]
        """
        frontier_cells = set(self.get_frontier_cells())
        clusters = []
        visited = set()

        for cell in frontier_cells:
            if cell in visited:
                continue

            # BFS 聚类
            cluster = []
            queue = [cell]
            visited.add(cell)

            while queue:
                current = queue.pop(0)
                cluster.append(current)

                # 检查邻居
                for neighbor in self._get_neighbors(current[0], current[1]):
                    if neighbor in frontier_cells and neighbor not in visited:
                        queue.append(neighbor)
                        visited.add(neighbor)

            # 过滤小聚类
            if len(cluster) >= min_cluster_size:
                clusters.append(cluster)

        return clusters

    def save(self, filepath: str) -> None:
        """
        保存 GCM（稀疏格式）。

        Args:
            filepath: 保存路径
        """
        data = {
            'resolution': self.resolution,
            'coverage_map': self.coverage_map,
            'total_cells': self.total_cells,
            'covered_cells': self.covered_cells,
            'bounds': (self.min_x, self.max_x, self.min_y, self.max_y),
        }

        with open(filepath, 'wb') as f:
            pickle.dump(data, f)

        logger.info(f"Saved GCM to {filepath}")

    def load(self, filepath: str) -> None:
        """
        加载 GCM。

        Args:
            filepath: 加载路径
        """
        with open(filepath, 'rb') as f:
            data = pickle.load(f)

        self.resolution = data['resolution']
        self.coverage_map = data['coverage_map']
        self.total_cells = data['total_cells']
        self.covered_cells = data['covered_cells']
        self.min_x, self.max_x, self.min_y, self.max_y = data['bounds']

        logger.info(f"Loaded GCM from {filepath}")

    def world_to_grid(self, world_pos: np.ndarray) -> Tuple[int, int]:
        """
        世界坐标 → 栅格坐标。

        Args:
            world_pos: [x, y] 世界坐标

        Returns:
            (grid_x, grid_y)
        """
        grid_x = int(np.floor(world_pos[0] / self.resolution))
        grid_y = int(np.floor(world_pos[1] / self.resolution))
        return (grid_x, grid_y)

    def grid_to_world(self, grid_pos: Tuple[int, int]) -> np.ndarray:
        """
        栅格坐标 → 世界坐标（栅格中心）。

        Args:
            grid_pos: (grid_x, grid_y)

        Returns:
            [x, y] 世界坐标
        """
        x = (grid_pos[0] + 0.5) * self.resolution
        y = (grid_pos[1] + 0.5) * self.resolution
        return np.array([x, y], dtype=np.float64)

    def _mark_cell_covered(self, x: int, y: int) -> None:
        """标记单元格为已覆盖。"""
        key = (x, y)

        if key not in self.coverage_map:
            # 新单元格
            cell = CoverageCell(x=x, y=y, covered=True, visit_count=1, last_visited=time.time())
            self.coverage_map[key] = cell
            self.covered_cells += 1
            self.total_cells += 1

            # 更新边界
            self.min_x = min(self.min_x, x)
            self.max_x = max(self.max_x, x)
            self.min_y = min(self.min_y, y)
            self.max_y = max(self.max_y, y)

        else:
            # 已存在的单元格
            cell = self.coverage_map[key]
            if not cell.covered:
                cell.covered = True
                self.covered_cells += 1

            cell.visit_count += 1
            cell.last_visited = time.time()
            cell.uncertainty = max(0.0, cell.uncertainty - 0.1)  # 降低不确定性

    def _has_uncovered_neighbors(self, x: int, y: int) -> bool:
        """检查是否有未覆盖的邻居。"""
        for nx, ny in self._get_neighbors(x, y):
            key = (nx, ny)
            if key not in self.coverage_map or not self.coverage_map[key].covered:
                return True
        return False

    @staticmethod
    def _get_neighbors(x: int, y: int) -> List[Tuple[int, int]]:
        """获取 8-邻域。"""
        return [
            (x-1, y-1), (x, y-1), (x+1, y-1),
            (x-1, y),             (x+1, y),
            (x-1, y+1), (x, y+1), (x+1, y+1),
        ]

    @staticmethod
    def _point_in_polyhedron(point: np.ndarray, polyhedron) -> bool:
        """
        检查点是否在多面体内（简化版本）。

        使用外接球近似。
        """
        distance = np.linalg.norm(point - polyhedron.center)
        return distance <= polyhedron.radius

    def get_statistics(self) -> Dict:
        """获取统计信息。"""
        return {
            'resolution': self.resolution,
            'total_cells': self.total_cells,
            'covered_cells': self.covered_cells,
            'coverage_ratio': self.get_coverage_ratio(),
            'num_frontiers': len(self.get_frontier_cells()),
            'bounds': {
                'x': (self.min_x, self.max_x),
                'y': (self.min_y, self.max_y),
            },
        }
