"""
多面体扩展 (Polyhedron Expansion) — USS-Nav 核心算法实现。

功能:
  从局部占据栅格生成多面体节点，构建空间连通图 (SCG)。

算法流程:
  1. 球面采样: 从种子点向外扩展，探索自由空间边界
  2. 凸包计算: 使用 QuickHull 生成多面体
  3. 碰撞检测: 检查多面体是否与障碍物碰撞
  4. 迭代扩展: 选择新种子点，重复上述过程
  5. SCG 构建: 建立多面体间的拓扑连接

参考:
  - USS-Nav (2025): Algorithm 1 - Polyhedron Expansion
"""

import logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.spatial import ConvexHull, KDTree

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class Polyhedron:
    """多面体节点。"""
    poly_id: int
    vertices: np.ndarray          # (N, 3) 凸包顶点
    faces: np.ndarray             # (F, 3) 三角面片索引
    center: np.ndarray            # [x, y, z] 中心点
    volume: float                 # 体积 (立方米)
    radius: float                 # 外接球半径
    seed_point: np.ndarray        # 种子点
    sample_points: np.ndarray     # 采样点集 (用于调试)


@dataclass
class PolyhedronExpansionConfig:
    """多面体扩展配置参数。"""
    # 球面采样参数
    num_sphere_samples: int = 48      # 球面采样点数
    r_min: float = 0.5                # 最小半径 (米)
    r_max: float = 3.0                # 最大半径 (米)
    r_step: float = 0.5               # 半径步长 (米)

    # 碰撞检测参数
    collision_check_samples: int = 100  # 碰撞检测采样数
    collision_threshold: float = 0.3    # 碰撞阈值 (占据概率)

    # 种子点选择参数
    min_polyhedron_volume: float = 1.0  # 最小多面体体积 (立方米)
    max_polyhedra: int = 50             # 最大多面体数量
    coverage_threshold: float = 0.8     # 覆盖率阈值 (停止条件)

    # SCG 构建参数
    adjacency_threshold: float = 0.2    # 邻接阈值 (米)


# ══════════════════════════════════════════════════════════════════
#  球面采样器
# ══════════════════════════════════════════════════════════════════

class SphereSampler:
    """球面采样器 — Fibonacci 均匀采样。"""

    @staticmethod
    def fibonacci_sphere(num_samples: int) -> np.ndarray:
        """
        Fibonacci 球面均匀采样。

        Args:
            num_samples: 采样点数

        Returns:
            (num_samples, 3) 单位方向向量
        """
        points = []
        phi = np.pi * (3.0 - np.sqrt(5.0))  # 黄金角

        for i in range(num_samples):
            y = 1 - (i / float(num_samples - 1)) * 2  # y: 1 → -1
            radius = np.sqrt(1 - y * y)

            theta = phi * i

            x = np.cos(theta) * radius
            z = np.sin(theta) * radius

            points.append([x, y, z])

        return np.array(points, dtype=np.float64)

    @staticmethod
    def sample_free_space(
        seed: np.ndarray,
        directions: np.ndarray,
        radii: np.ndarray,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
    ) -> np.ndarray:
        """
        在球面方向上采样自由空间点。

        Args:
            seed: 种子点 [x, y, z]
            directions: (N, 3) 单位方向向量
            radii: (M,) 半径数组
            occupancy_grid: (X, Y, Z) 占据栅格 (0=自由, 1=占据)
            grid_resolution: 栅格分辨率 (米)
            grid_origin: 栅格原点 [x, y, z]

        Returns:
            (K, 3) 自由空间采样点
        """
        free_points = [seed]

        for r in radii:
            for d in directions:
                p = seed + r * d

                # 检查是否在自由空间
                if SphereSampler._is_free(p, occupancy_grid, grid_resolution, grid_origin):
                    free_points.append(p)

        return np.array(free_points, dtype=np.float64)

    @staticmethod
    def _is_free(
        point: np.ndarray,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
    ) -> bool:
        """检查点是否在自由空间。"""
        # 世界坐标 → 栅格坐标
        grid_pos = ((point - origin) / resolution).astype(int)

        # 边界检查
        if np.any(grid_pos < 0) or np.any(grid_pos >= occupancy_grid.shape):
            return False

        # 占据检查 (0 = 自由, 1 = 占据)
        return occupancy_grid[tuple(grid_pos)] < 0.5


# ══════════════════════════════════════════════════════════════════
#  凸包计算器
# ══════════════════════════════════════════════════════════════════

class ConvexHullComputer:
    """凸包计算器 — 使用 scipy.spatial.ConvexHull。"""

    @staticmethod
    def compute(points: np.ndarray) -> Optional[dict]:
        """
        计算点集的凸包。

        Args:
            points: (N, 3) 点集

        Returns:
            {
                "vertices": (M, 3) 凸包顶点,
                "faces": (F, 3) 三角面片索引,
                "volume": float 体积,
                "center": [x, y, z] 中心点,
                "radius": float 外接球半径,
            }
            或 None (如果计算失败)
        """
        if len(points) < 4:
            logger.warning(f"Too few points ({len(points)}) for convex hull")
            return None

        try:
            hull = ConvexHull(points)

            vertices = points[hull.vertices]
            center = vertices.mean(axis=0)
            radius = np.max(np.linalg.norm(vertices - center, axis=1))

            return {
                "vertices": vertices,
                "faces": hull.simplices,
                "volume": hull.volume,
                "center": center,
                "radius": float(radius),
            }

        except Exception as e:
            logger.warning(f"Failed to compute convex hull: {e}")
            return None


# ══════════════════════════════════════════════════════════════════
#  碰撞检测器
# ══════════════════════════════════════════════════════════════════

class CollisionChecker:
    """碰撞检测器 — 采样点方法。"""

    @staticmethod
    def check_collision(
        polyhedron_vertices: np.ndarray,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
        num_samples: int = 100,
        threshold: float = 0.3,
    ) -> bool:
        """
        检查多面体是否与障碍物碰撞。

        方法: 在多面体内部采样点，检查是否有点在障碍物内。

        Args:
            polyhedron_vertices: (N, 3) 凸包顶点
            occupancy_grid: (X, Y, Z) 占据栅格
            grid_resolution: 栅格分辨率 (米)
            grid_origin: 栅格原点 [x, y, z]
            num_samples: 采样点数
            threshold: 占据阈值

        Returns:
            True if 碰撞
        """
        # 计算边界框
        bbox_min = polyhedron_vertices.min(axis=0)
        bbox_max = polyhedron_vertices.max(axis=0)

        # 在边界框内随机采样
        samples = np.random.uniform(bbox_min, bbox_max, (num_samples, 3))

        # 过滤出多面体内部的点
        inside_count = 0
        collision_count = 0

        for p in samples:
            if CollisionChecker._point_in_convex_hull(p, polyhedron_vertices):
                inside_count += 1

                # 检查是否在障碍物内
                if CollisionChecker._is_occupied(
                    p, occupancy_grid, grid_resolution, grid_origin, threshold
                ):
                    collision_count += 1

        if inside_count == 0:
            return False

        # 如果超过 30% 的内部点在障碍物内，认为碰撞
        collision_ratio = collision_count / inside_count
        return collision_ratio > 0.3

    @staticmethod
    def _point_in_convex_hull(point: np.ndarray, vertices: np.ndarray) -> bool:
        """
        检查点是否在凸包内 (简化版本)。

        方法: 检查点到中心的距离是否小于外接球半径。
        """
        center = vertices.mean(axis=0)
        radius = np.max(np.linalg.norm(vertices - center, axis=1))
        return np.linalg.norm(point - center) <= radius

    @staticmethod
    def _is_occupied(
        point: np.ndarray,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
        threshold: float,
    ) -> bool:
        """检查点是否在障碍物内。"""
        grid_pos = ((point - origin) / resolution).astype(int)

        if np.any(grid_pos < 0) or np.any(grid_pos >= occupancy_grid.shape):
            return True  # 边界外视为障碍物

        return occupancy_grid[tuple(grid_pos)] > threshold


# ══════════════════════════════════════════════════════════════════
#  种子点选择器
# ══════════════════════════════════════════════════════════════════

class SeedSelector:
    """种子点选择器 — 最远点优先策略。"""

    @staticmethod
    def select_next_seed(
        candidates: np.ndarray,
        existing_polyhedra: List[Polyhedron],
    ) -> Optional[np.ndarray]:
        """
        选择下一个种子点 (距离所有已有多面体最远)。

        Args:
            candidates: (N, 3) 候选点集
            existing_polyhedra: 已有多面体列表

        Returns:
            种子点 [x, y, z] 或 None
        """
        if len(candidates) == 0:
            return None

        if len(existing_polyhedra) == 0:
            # 第一个种子点: 选择候选点的中心
            return candidates.mean(axis=0)

        # 构建已有多面体中心的 KD-Tree
        centers = np.array([p.center for p in existing_polyhedra])
        tree = KDTree(centers)

        # 找到距离最近多面体最远的候选点
        max_dist = -1
        best_seed = None

        for c in candidates:
            dist, _ = tree.query(c)
            if dist > max_dist:
                max_dist = dist
                best_seed = c

        return best_seed


# ══════════════════════════════════════════════════════════════════
#  多面体扩展器 (主算法)
# ══════════════════════════════════════════════════════════════════

class PolyhedronExpander:
    """
    多面体扩展器 — USS-Nav Algorithm 1 实现。

    用法:
        expander = PolyhedronExpander(config)
        polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)
    """

    def __init__(self, config: PolyhedronExpansionConfig):
        self.config = config
        self.sphere_sampler = SphereSampler()
        self.hull_computer = ConvexHullComputer()
        self.collision_checker = CollisionChecker()
        self.seed_selector = SeedSelector()

    def expand(
        self,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
    ) -> List[Polyhedron]:
        """
        多面体扩展主算法。

        Args:
            occupancy_grid: (X, Y, Z) 占据栅格 (0=自由, 1=占据)
            grid_resolution: 栅格分辨率 (米)
            grid_origin: 栅格原点 [x, y, z]

        Returns:
            多面体列表
        """
        logger.info("Starting polyhedron expansion...")

        # 1. 提取自由空间候选点
        candidates = self._extract_free_space_candidates(
            occupancy_grid, grid_resolution, grid_origin
        )
        logger.info(f"Extracted {len(candidates)} free space candidates")

        # 2. 预计算球面采样方向
        directions = self.sphere_sampler.fibonacci_sphere(
            self.config.num_sphere_samples
        )
        radii = np.arange(
            self.config.r_min,
            self.config.r_max + self.config.r_step,
            self.config.r_step,
        )

        # 3. 迭代扩展多面体
        polyhedra = []
        covered_points = set()

        for i in range(self.config.max_polyhedra):
            # 选择种子点
            remaining_candidates = np.array([
                c for j, c in enumerate(candidates)
                if j not in covered_points
            ])

            if len(remaining_candidates) == 0:
                logger.info("No more candidates, stopping expansion")
                break

            seed = self.seed_selector.select_next_seed(
                remaining_candidates, polyhedra
            )

            if seed is None:
                break

            # 球面采样
            sample_points = self.sphere_sampler.sample_free_space(
                seed=seed,
                directions=directions,
                radii=radii,
                occupancy_grid=occupancy_grid,
                grid_resolution=grid_resolution,
                grid_origin=grid_origin,
            )

            if len(sample_points) < 4:
                logger.warning(f"Too few sample points ({len(sample_points)}), skipping")
                continue

            # 计算凸包
            hull_result = self.hull_computer.compute(sample_points)

            if hull_result is None:
                continue

            # 检查体积
            if hull_result["volume"] < self.config.min_polyhedron_volume:
                logger.debug(f"Polyhedron volume too small ({hull_result['volume']:.2f}), skipping")
                continue

            # 碰撞检测
            if self.collision_checker.check_collision(
                polyhedron_vertices=hull_result["vertices"],
                occupancy_grid=occupancy_grid,
                grid_resolution=grid_resolution,
                grid_origin=grid_origin,
                num_samples=self.config.collision_check_samples,
                threshold=self.config.collision_threshold,
            ):
                logger.debug("Polyhedron collides with obstacles, skipping")
                continue

            # 创建多面体节点
            poly = Polyhedron(
                poly_id=len(polyhedra),
                vertices=hull_result["vertices"],
                faces=hull_result["faces"],
                center=hull_result["center"],
                volume=hull_result["volume"],
                radius=hull_result["radius"],
                seed_point=seed,
                sample_points=sample_points,
            )

            polyhedra.append(poly)

            # 标记已覆盖的候选点
            for j, c in enumerate(candidates):
                if np.linalg.norm(c - poly.center) <= poly.radius:
                    covered_points.add(j)

            logger.info(
                f"Polyhedron {poly.poly_id}: volume={poly.volume:.2f}m³, "
                f"radius={poly.radius:.2f}m, covered={len(covered_points)}/{len(candidates)}"
            )

            # 检查覆盖率
            coverage = len(covered_points) / len(candidates)
            if coverage >= self.config.coverage_threshold:
                logger.info(f"Coverage threshold reached ({coverage:.2%}), stopping")
                break

        logger.info(f"Expansion complete: {len(polyhedra)} polyhedra generated")
        return polyhedra

    def _extract_free_space_candidates(
        self,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
    ) -> np.ndarray:
        """
        提取自由空间候选点。

        Args:
            occupancy_grid: (X, Y, Z) 占据栅格
            grid_resolution: 栅格分辨率
            grid_origin: 栅格原点

        Returns:
            (N, 3) 候选点集 (世界坐标)
        """
        # 找到所有自由空间栅格
        free_indices = np.argwhere(occupancy_grid < 0.5)

        # 转换为世界坐标
        candidates = grid_origin + free_indices * grid_resolution

        # 下采样 (避免候选点过多)
        if len(candidates) > 1000:
            indices = np.random.choice(len(candidates), 1000, replace=False)
            candidates = candidates[indices]

        return candidates
