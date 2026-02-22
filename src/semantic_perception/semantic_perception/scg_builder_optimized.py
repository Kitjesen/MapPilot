"""
SCG 构建优化版本 (Optimized SCG Builder)

优化内容:
1. 使用 KDTree 空间索引加速邻居查找
2. 并行化边构建
3. 批量射线追踪
4. 增量更新支持

性能提升: 预期 50-70%
"""

import numpy as np
from scipy.spatial import KDTree
from typing import List, Dict, Tuple
from concurrent.futures import ThreadPoolExecutor, as_completed
import logging

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  优化的 SCG 构建器
# ══════════════════════════════════════════════════════════════════

class OptimizedSCGBuilder:
    """优化的 SCG 构建器 — 使用空间索引和并行化。"""

    def __init__(self, config=None):
        self.config = config
        self.nodes = {}  # {poly_id: Polyhedron}
        self.edges = []
        self.kdtree = None
        self.centers = None
        self.poly_ids = None

    def add_polyhedron(self, polyhedron):
        """添加多面体节点。"""
        self.nodes[polyhedron.poly_id] = polyhedron
        # 标记需要重建索引
        self.kdtree = None

    def _build_spatial_index(self):
        """构建空间索引（KDTree）。"""
        if len(self.nodes) == 0:
            return

        # 提取所有中心点
        self.poly_ids = list(self.nodes.keys())
        self.centers = np.array([
            self.nodes[pid].center for pid in self.poly_ids
        ])

        # 构建 KDTree
        self.kdtree = KDTree(self.centers)

        logger.info(f"Built KDTree with {len(self.nodes)} nodes")

    def build_edges_optimized(
        self,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
        max_workers: int = 4,
    ):
        """
        优化的边构建 — 使用空间索引和并行化。

        Args:
            occupancy_grid: 占据栅格
            resolution: 分辨率
            origin: 原点
            max_workers: 最大线程数
        """
        if len(self.nodes) < 2:
            logger.warning("Need at least 2 nodes to build edges")
            return

        # 构建空间索引
        if self.kdtree is None:
            self._build_spatial_index()

        logger.info(f"Building edges with {max_workers} workers...")

        # 并行构建边
        self.edges = []

        # 为每个节点找邻居
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            futures = []

            for i, poly_id in enumerate(self.poly_ids):
                poly = self.nodes[poly_id]

                # 使用 KDTree 查找邻居（半径搜索）
                search_radius = poly.radius * 3  # 搜索半径
                neighbor_indices = self.kdtree.query_ball_point(
                    poly.center,
                    search_radius
                )

                # 过滤掉自己
                neighbor_indices = [
                    idx for idx in neighbor_indices
                    if self.poly_ids[idx] != poly_id
                ]

                # 提交任务
                for neighbor_idx in neighbor_indices:
                    neighbor_id = self.poly_ids[neighbor_idx]
                    neighbor = self.nodes[neighbor_id]

                    future = executor.submit(
                        self._check_edge,
                        poly,
                        neighbor,
                        occupancy_grid,
                        resolution,
                        origin,
                    )
                    futures.append((poly_id, neighbor_id, future))

            # 收集结果
            for poly_id, neighbor_id, future in futures:
                try:
                    edge_type = future.result()
                    if edge_type is not None:
                        self.edges.append({
                            'from_id': poly_id,
                            'to_id': neighbor_id,
                            'edge_type': edge_type,
                        })
                except Exception as e:
                    logger.warning(f"Failed to check edge {poly_id}->{neighbor_id}: {e}")

        logger.info(f"Built {len(self.edges)} edges")

    def _check_edge(
        self,
        poly1,
        poly2,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
    ) -> str:
        """
        检查两个多面体之间的边类型。

        Returns:
            'adjacency', 'connectivity', 'accessibility', or None
        """
        # 1. 检查 Adjacency（共享面）
        distance = np.linalg.norm(poly1.center - poly2.center)
        if distance < (poly1.radius + poly2.radius) * 0.5:
            return 'adjacency'

        # 2. 检查 Connectivity（视线可达）
        if self._check_line_of_sight(
            poly1.center,
            poly2.center,
            occupancy_grid,
            resolution,
            origin,
        ):
            return 'connectivity'

        # 3. 检查 Accessibility（可通行）
        # 简化版本：如果距离不太远且没有明显障碍
        if distance < 10.0:  # 10 米内
            return 'accessibility'

        return None

    def _check_line_of_sight(
        self,
        start: np.ndarray,
        end: np.ndarray,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
    ) -> bool:
        """
        检查两点之间是否有视线（批量采样）。

        Returns:
            True if 视线可达
        """
        # 在直线上采样点
        num_samples = int(np.linalg.norm(end - start) / resolution) + 1
        num_samples = min(num_samples, 100)  # 限制最大采样数

        # 批量生成采样点（向量化）
        t = np.linspace(0, 1, num_samples)
        samples = start + np.outer(t, end - start)

        # 转换到栅格坐标（向量化）
        grid_coords = ((samples - origin) / resolution).astype(int)

        # 边界检查（向量化）
        grid_shape = np.array(occupancy_grid.shape)
        valid_mask = np.all((grid_coords >= 0) & (grid_coords < grid_shape), axis=1)

        if not np.any(valid_mask):
            return False

        valid_coords = grid_coords[valid_mask]

        # 批量查询占据值（向量化）
        occupancy_values = occupancy_grid[
            valid_coords[:, 0],
            valid_coords[:, 1],
            valid_coords[:, 2]
        ]

        # 检查是否有障碍物
        return np.all(occupancy_values < 0.5)

    def query_neighbors(
        self,
        point: np.ndarray,
        radius: float,
    ) -> List[int]:
        """
        查询指定点附近的多面体（使用 KDTree）。

        Args:
            point: 查询点 [x, y, z]
            radius: 搜索半径

        Returns:
            多面体 ID 列表
        """
        if self.kdtree is None:
            self._build_spatial_index()

        indices = self.kdtree.query_ball_point(point, radius)
        return [self.poly_ids[idx] for idx in indices]

    def get_statistics(self) -> dict:
        """获取统计信息。"""
        if len(self.edges) == 0:
            return {
                'num_nodes': len(self.nodes),
                'num_edges': 0,
                'edge_types': {},
            }

        # 统计边类型
        edge_types = {}
        for edge in self.edges:
            edge_type = edge['edge_type']
            edge_types[edge_type] = edge_types.get(edge_type, 0) + 1

        return {
            'num_nodes': len(self.nodes),
            'num_edges': len(self.edges),
            'edge_types': edge_types,
            'avg_degree': len(self.edges) * 2 / len(self.nodes) if len(self.nodes) > 0 else 0,
        }


# ══════════════════════════════════════════════════════════════════
#  增量更新支持
# ══════════════════════════════════════════════════════════════════

class IncrementalSCGBuilder(OptimizedSCGBuilder):
    """增量更新的 SCG 构建器。"""

    def __init__(self, config=None):
        super().__init__(config)
        self.edge_cache = {}  # {(id1, id2): edge_type}

    def add_polyhedron_incremental(
        self,
        polyhedron,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
    ):
        """
        增量添加多面体并更新边。

        Args:
            polyhedron: 新多面体
            occupancy_grid: 占据栅格
            resolution: 分辨率
            origin: 原点
        """
        # 添加节点
        self.add_polyhedron(polyhedron)

        # 重建空间索引
        self._build_spatial_index()

        # 只检查新节点与现有节点的边
        new_id = polyhedron.poly_id

        # 查找邻居
        search_radius = polyhedron.radius * 3
        neighbor_indices = self.kdtree.query_ball_point(
            polyhedron.center,
            search_radius
        )

        # 检查边
        for neighbor_idx in neighbor_indices:
            neighbor_id = self.poly_ids[neighbor_idx]

            if neighbor_id == new_id:
                continue

            # 检查缓存
            edge_key = tuple(sorted([new_id, neighbor_id]))
            if edge_key in self.edge_cache:
                continue

            # 检查边
            neighbor = self.nodes[neighbor_id]
            edge_type = self._check_edge(
                polyhedron,
                neighbor,
                occupancy_grid,
                resolution,
                origin,
            )

            if edge_type is not None:
                self.edges.append({
                    'from_id': new_id,
                    'to_id': neighbor_id,
                    'edge_type': edge_type,
                })
                self.edge_cache[edge_key] = edge_type

        logger.info(f"Added polyhedron {new_id}, now {len(self.edges)} edges")


# ══════════════════════════════════════════════════════════════════
#  性能对比测试
# ══════════════════════════════════════════════════════════════════

def benchmark_scg_optimizations():
    """对比 SCG 构建优化前后的性能。"""
    import time
    from semantic_perception.polyhedron_expansion import Polyhedron
    from scipy.spatial import ConvexHull

    print("=" * 60)
    print("SCG 构建优化对比测试")
    print("=" * 60)

    # 创建测试数据
    occupancy_grid = np.random.rand(100, 100, 40) < 0.3
    resolution = 0.1
    origin = np.array([0, 0, 0])

    # 创建 20 个多面体
    polyhedra = []
    for i in range(20):
        vertices = np.random.rand(10, 3) * 2 + np.array([i * 2, 0, 0])
        center = vertices.mean(axis=0)
        radius = np.max(np.linalg.norm(vertices - center, axis=1))
        hull = ConvexHull(vertices)

        poly = Polyhedron(
            poly_id=i,
            vertices=vertices,
            faces=hull.simplices,
            center=center,
            radius=radius,
            volume=1.0,
            seed_point=center.copy(),
            sample_points=vertices.copy(),
        )
        polyhedra.append(poly)

    # 测试 1: 原始版本（O(N²) 遍历）
    print("\n1. 原始版本（O(N²) 遍历）")

    start_time = time.time()
    edge_count = 0
    for i, poly1 in enumerate(polyhedra):
        for j, poly2 in enumerate(polyhedra):
            if i >= j:
                continue
            # 简单距离检查
            distance = np.linalg.norm(poly1.center - poly2.center)
            if distance < (poly1.radius + poly2.radius) * 3:
                edge_count += 1
    original_time = time.time() - start_time

    print(f"  时间: {original_time*1000:.2f} ms")
    print(f"  边数: {edge_count}")

    # 测试 2: 优化版本（KDTree + 并行）
    print("\n2. 优化版本（KDTree + 并行）")

    builder = OptimizedSCGBuilder()
    for poly in polyhedra:
        builder.add_polyhedron(poly)

    start_time = time.time()
    builder.build_edges_optimized(
        occupancy_grid,
        resolution,
        origin,
        max_workers=4,
    )
    optimized_time = time.time() - start_time

    stats = builder.get_statistics()

    print(f"  时间: {optimized_time*1000:.2f} ms")
    print(f"  边数: {stats['num_edges']}")
    print(f"  加速比: {original_time / optimized_time:.2f}×")

    # 测试 3: 增量更新
    print("\n3. 增量更新")

    incremental_builder = IncrementalSCGBuilder()

    # 先添加 15 个
    for poly in polyhedra[:15]:
        incremental_builder.add_polyhedron(poly)

    incremental_builder.build_edges_optimized(
        occupancy_grid,
        resolution,
        origin,
        max_workers=4,
    )

    # 增量添加剩余 5 个
    start_time = time.time()
    for poly in polyhedra[15:]:
        incremental_builder.add_polyhedron_incremental(
            poly,
            occupancy_grid,
            resolution,
            origin,
        )
    incremental_time = time.time() - start_time

    print(f"  增量添加 5 个节点时间: {incremental_time*1000:.2f} ms")
    print(f"  总边数: {len(incremental_builder.edges)}")

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)


if __name__ == "__main__":
    benchmark_scg_optimizations()
