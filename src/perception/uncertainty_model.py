"""
不确定性建模 (Uncertainty Model) — 为多面体和 GCM 添加不确定性度量。

功能:
  1. 计算多面体的不确定性（基于占据概率）
  2. 计算 GCM 的整体不确定性
  3. 计算信息增益（用于探索决策）
  4. 选择探索目标（最大信息增益）

设计原则:
  - 概率建模: 使用熵度量不确定性
  - 信息增益: 结合不确定性、新颖性、可达性
  - 探索策略: 基于不确定性的主动探索

参考论文:
  - USS-Nav (2025): 不确定性感知的空间表示
  - Active SLAM: 信息增益探索
  - Frontier-based Exploration: 前沿探索策略
"""

import logging

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  不确定性建模
# ══════════════════════════════════════════════════════════════════

class UncertaintyModel:
    """
    不确定性建模 — 为多面体和 GCM 添加不确定性度量。

    用途:
      - 评估多面体的不确定性
      - 指导探索决策
      - 计算信息增益

    用法:
        model = UncertaintyModel()
        uncertainty = model.compute_polyhedron_uncertainty(poly, grid)
        target = model.select_exploration_target(scg, gcm, robot_pose)
    """

    def __init__(self, entropy_threshold: float = 0.5):
        """
        Args:
            entropy_threshold: 熵阈值（用于判断高不确定性）
        """
        self.entropy_threshold = entropy_threshold

    def compute_polyhedron_uncertainty(
        self,
        polyhedron,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
        num_samples: int = 100,
    ) -> float:
        """
        计算多面体的不确定性。

        方法:
          1. 在多面体内部采样点
          2. 查询占据栅格的占据概率
          3. 计算平均熵

        Args:
            polyhedron: Polyhedron 对象
            occupancy_grid: (X, Y, Z) 占据栅格 (0-1 概率)
            grid_resolution: 栅格分辨率 (米)
            grid_origin: 栅格原点 [x, y, z]
            num_samples: 采样点数

        Returns:
            平均熵 (0.0-1.0)
        """
        # 1. 在多面体内部采样点
        samples = self._sample_points_in_polyhedron(polyhedron, num_samples)

        if len(samples) == 0:
            return 0.0

        # 2. 查询占据概率并计算熵
        entropies = []

        for point in samples:
            # 查询占据概率
            p_occupied = self._query_occupancy_probability(
                point, occupancy_grid, grid_resolution, grid_origin
            )

            # 计算熵: H = -p*log(p) - (1-p)*log(1-p)
            entropy = self._compute_entropy(p_occupied)
            entropies.append(entropy)

        # 3. 返回平均熵
        return float(np.mean(entropies))

    def compute_gcm_uncertainty(self, gcm) -> float:
        """
        计算 GCM 的整体不确定性。

        Args:
            gcm: GlobalCoverageMask 对象

        Returns:
            平均不确定性 (0.0-1.0)
        """
        if len(gcm.coverage_map) == 0:
            return 1.0  # 完全未知

        total_uncertainty = 0.0
        for cell in gcm.coverage_map.values():
            total_uncertainty += cell.uncertainty

        return total_uncertainty / len(gcm.coverage_map)

    def compute_information_gain(
        self,
        polyhedron,
        robot_pose: np.ndarray,
        visit_count: int = 0,
        max_distance: float = 10.0,
    ) -> float:
        """
        计算多面体的信息增益。

        公式:
          IG = uncertainty × novelty × reachability

        Args:
            polyhedron: Polyhedron 对象
            robot_pose: 机器人位置 [x, y, z]
            visit_count: 访问次数
            max_distance: 最大可达距离 (米)

        Returns:
            信息增益 (0.0-1.0)
        """
        # 1. 不确定性因子（假设多面体有 uncertainty 属性）
        uncertainty = getattr(polyhedron, 'uncertainty', 0.5)

        # 2. 新颖性因子（访问次数越少越新颖）
        novelty = 1.0 / (1.0 + visit_count)

        # 3. 可达性因子（距离越近越可达）
        distance = np.linalg.norm(polyhedron.center - robot_pose)
        reachability = np.exp(-distance / max_distance)

        # 4. 综合信息增益
        information_gain = uncertainty * novelty * reachability

        return float(information_gain)

    def select_exploration_target(
        self,
        scg_builder,
        gcm,
        robot_pose: np.ndarray,
        visited_polyhedra: dict[int, int] | None = None,
    ) -> tuple[int, float] | None:
        """
        选择探索目标（最大信息增益）。

        Args:
            scg_builder: SCGBuilder 对象
            gcm: GlobalCoverageMask 对象
            robot_pose: 机器人位置 [x, y, z]
            visited_polyhedra: 访问记录 {poly_id: visit_count}

        Returns:
            (poly_id, information_gain) 或 None
        """
        if visited_polyhedra is None:
            visited_polyhedra = {}

        best_target = None
        max_gain = -float('inf')

        for poly_id, poly in scg_builder.nodes.items():
            # 计算信息增益
            visit_count = visited_polyhedra.get(poly_id, 0)
            gain = self.compute_information_gain(poly, robot_pose, visit_count)

            if gain > max_gain:
                max_gain = gain
                best_target = poly_id

        if best_target is not None:
            logger.info(
                f"Selected exploration target: polyhedron {best_target} "
                f"with information gain {max_gain:.4f}"
            )
            return (best_target, max_gain)

        return None

    def select_frontier_target(
        self,
        gcm,
        robot_pose: np.ndarray,
        max_distance: float = 10.0,
    ) -> tuple[tuple[int, int], float] | None:
        """
        选择前沿探索目标。

        Args:
            gcm: GlobalCoverageMask 对象
            robot_pose: 机器人位置 [x, y, z]
            max_distance: 最大可达距离 (米)

        Returns:
            ((grid_x, grid_y), information_gain) 或 None
        """
        # 获取前沿聚类
        frontier_clusters = gcm.get_frontier_clusters(min_cluster_size=3)

        if not frontier_clusters:
            logger.info("No frontier clusters found")
            return None

        best_target = None
        max_gain = -float('inf')

        for cluster in frontier_clusters:
            # 计算聚类中心
            cluster_positions = [gcm.grid_to_world(cell) for cell in cluster]
            cluster_center = np.mean(cluster_positions, axis=0)
            cluster_center_3d = np.array([cluster_center[0], cluster_center[1], robot_pose[2]])

            # 计算信息增益
            # 前沿的不确定性固定为 1.0（完全未知）
            uncertainty = 1.0
            # 新颖性基于聚类大小
            novelty = min(1.0, len(cluster) / 10.0)
            # 可达性基于距离
            distance = np.linalg.norm(cluster_center_3d - robot_pose)
            reachability = np.exp(-distance / max_distance)

            gain = uncertainty * novelty * reachability

            if gain > max_gain:
                max_gain = gain
                # 选择聚类中心作为目标
                best_target = cluster[len(cluster) // 2]

        if best_target is not None:
            logger.info(
                f"Selected frontier target: {best_target} "
                f"with information gain {max_gain:.4f}"
            )
            return (best_target, max_gain)

        return None

    def update_polyhedron_uncertainty(
        self,
        polyhedron,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
    ) -> None:
        """
        更新多面体的不确定性属性。

        Args:
            polyhedron: Polyhedron 对象
            occupancy_grid: (X, Y, Z) 占据栅格
            grid_resolution: 栅格分辨率 (米)
            grid_origin: 栅格原点 [x, y, z]
        """
        uncertainty = self.compute_polyhedron_uncertainty(
            polyhedron, occupancy_grid, grid_resolution, grid_origin
        )

        # 设置 uncertainty 属性
        polyhedron.uncertainty = uncertainty

    def _sample_points_in_polyhedron(
        self,
        polyhedron,
        num_samples: int,
    ) -> np.ndarray:
        """
        在多面体内部采样点。

        方法: 在边界框内随机采样，过滤出多面体内部的点。

        Args:
            polyhedron: Polyhedron 对象
            num_samples: 采样点数

        Returns:
            (M, 3) 采样点 (M <= num_samples)
        """
        # 计算边界框
        vertices = polyhedron.vertices
        bbox_min = vertices.min(axis=0)
        bbox_max = vertices.max(axis=0)

        # 在边界框内随机采样
        samples = []
        attempts = 0
        max_attempts = num_samples * 10

        while len(samples) < num_samples and attempts < max_attempts:
            point = np.random.uniform(bbox_min, bbox_max)

            # 检查是否在多面体内（使用外接球近似）
            if self._point_in_polyhedron(point, polyhedron):
                samples.append(point)

            attempts += 1

        return np.array(samples) if samples else np.array([]).reshape(0, 3)

    @staticmethod
    def _point_in_polyhedron(point: np.ndarray, polyhedron) -> bool:
        """检查点是否在多面体内（外接球近似）。"""
        distance = np.linalg.norm(point - polyhedron.center)
        return distance <= polyhedron.radius

    @staticmethod
    def _query_occupancy_probability(
        point: np.ndarray,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
    ) -> float:
        """
        查询点的占据概率。

        Args:
            point: [x, y, z]
            occupancy_grid: (X, Y, Z) 占据栅格 (0-1 概率)
            resolution: 栅格分辨率
            origin: 栅格原点

        Returns:
            占据概率 (0.0-1.0)
        """
        # 世界坐标 → 栅格坐标
        grid_pos = ((point - origin) / resolution).astype(int)

        # 边界检查
        if np.any(grid_pos < 0) or np.any(grid_pos >= occupancy_grid.shape):
            return 0.5  # 未知区域，返回最大熵的概率

        # 查询占据概率
        return float(occupancy_grid[tuple(grid_pos)])

    @staticmethod
    def _compute_entropy(p: float) -> float:
        """
        计算二元熵。

        H(p) = -p*log2(p) - (1-p)*log2(1-p)

        Args:
            p: 概率 (0.0-1.0)

        Returns:
            熵 (0.0-1.0)
        """
        # 避免 log(0)
        p = np.clip(p, 1e-10, 1.0 - 1e-10)

        entropy = -p * np.log2(p) - (1 - p) * np.log2(1 - p)

        return float(entropy)


# ══════════════════════════════════════════════════════════════════
#  探索策略
# ══════════════════════════════════════════════════════════════════

class ExplorationStrategy:
    """
    探索策略 — 基于不确定性的主动探索。

    策略:
      1. 优先探索高不确定性的多面体
      2. 优先探索前沿区域
      3. 平衡探索和利用

    用法:
        strategy = ExplorationStrategy(uncertainty_model)
        target = strategy.select_next_target(scg, gcm, robot_pose)
    """

    def __init__(self, uncertainty_model: UncertaintyModel):
        """
        Args:
            uncertainty_model: UncertaintyModel 对象
        """
        self.uncertainty_model = uncertainty_model
        self.visited_polyhedra: dict[int, int] = {}  # {poly_id: visit_count}

    def select_next_target(
        self,
        scg_builder,
        gcm,
        robot_pose: np.ndarray,
        prefer_frontier: bool = True,
    ) -> np.ndarray | None:
        """
        选择下一个探索目标。

        策略:
          1. 如果 prefer_frontier=True，优先选择前沿
          2. 否则，选择信息增益最大的多面体

        Args:
            scg_builder: SCGBuilder 对象
            gcm: GlobalCoverageMask 对象
            robot_pose: 机器人位置 [x, y, z]
            prefer_frontier: 是否优先前沿

        Returns:
            目标位置 [x, y, z] 或 None
        """
        if prefer_frontier:
            # 尝试选择前沿目标
            frontier_result = self.uncertainty_model.select_frontier_target(
                gcm, robot_pose
            )

            if frontier_result is not None:
                grid_pos, _gain = frontier_result
                # 转换为世界坐标
                world_pos = gcm.grid_to_world(grid_pos)
                return np.array([world_pos[0], world_pos[1], robot_pose[2]])

        # 选择多面体目标
        poly_result = self.uncertainty_model.select_exploration_target(
            scg_builder, gcm, robot_pose, self.visited_polyhedra
        )

        if poly_result is not None:
            poly_id, _gain = poly_result
            poly = scg_builder.nodes[poly_id]
            return poly.center

        return None

    def mark_visited(self, poly_id: int) -> None:
        """标记多面体为已访问。"""
        self.visited_polyhedra[poly_id] = self.visited_polyhedra.get(poly_id, 0) + 1

    def reset(self) -> None:
        """重置访问记录。"""
        self.visited_polyhedra.clear()

    def get_statistics(self) -> dict:
        """获取统计信息。"""
        return {
            'total_visited': len(self.visited_polyhedra),
            'total_visits': sum(self.visited_polyhedra.values()),
            'most_visited': max(self.visited_polyhedra.values()) if self.visited_polyhedra else 0,
        }
