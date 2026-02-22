"""
SCG-based 路径规划器 (SCG Path Planner) — 纯基于 SCG 的路径规划。

功能:
  1. 在 SCG 上搜索多面体序列
  2. 在多面体内部和边界生成路径
  3. 不依赖全局 Tomogram
  4. 路径平滑和优化

设计原则:
  - 轻量级: 只依赖 SCG，不需要全局栅格
  - 高效: A* 搜索 + 启发式
  - 鲁棒: 处理边界情况

参考论文:
  - USS-Nav (2025): SCG-based 路径规划
  - Hybrid A*: 路径平滑算法
"""

import heapq
import logging
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class PathSegment:
    """路径段（在两个多面体之间）。"""
    from_poly_id: int
    to_poly_id: int
    waypoints: np.ndarray  # (N, 3) 路径点
    distance: float
    edge_type: str  # "adjacency" | "connectivity" | "accessibility"


@dataclass
class SCGPath:
    """SCG 路径。"""
    polyhedron_sequence: List[int]  # 多面体 ID 序列
    segments: List[PathSegment]  # 路径段
    total_distance: float
    planning_time: float
    success: bool


# ══════════════════════════════════════════════════════════════════
#  路径平滑器
# ══════════════════════════════════════════════════════════════════

class PathSmoother:
    """
    路径平滑器 — 使用梯度下降平滑路径。

    方法:
      - 迭代平滑: 每个点向邻居的平均位置移动
      - 保持端点: 起点和终点不变
      - 碰撞检测: 可选
    """

    def __init__(self, iterations: int = 10, alpha: float = 0.5):
        """
        Args:
            iterations: 平滑迭代次数
            alpha: 平滑系数 (0.0-1.0)
        """
        self.iterations = iterations
        self.alpha = alpha

    def smooth(self, path: np.ndarray) -> np.ndarray:
        """
        平滑路径。

        Args:
            path: (N, 3) 路径点

        Returns:
            (N, 3) 平滑后的路径点
        """
        if len(path) < 3:
            return path

        smoothed = path.copy()

        for _ in range(self.iterations):
            for i in range(1, len(smoothed) - 1):
                # 向邻居的平均位置移动
                neighbor_avg = 0.5 * (smoothed[i-1] + smoothed[i+1])
                smoothed[i] = (1 - self.alpha) * smoothed[i] + self.alpha * neighbor_avg

        return smoothed

    def simplify(self, path: np.ndarray, tolerance: float = 0.1) -> np.ndarray:
        """
        简化路径（Douglas-Peucker 算法）。

        Args:
            path: (N, 3) 路径点
            tolerance: 简化容差 (米)

        Returns:
            (M, 3) 简化后的路径点 (M <= N)
        """
        if len(path) < 3:
            return path

        # Douglas-Peucker 算法
        def perpendicular_distance(point, line_start, line_end):
            """计算点到线段的垂直距离。"""
            if np.allclose(line_start, line_end):
                return np.linalg.norm(point - line_start)

            line_vec = line_end - line_start
            point_vec = point - line_start
            line_len = np.linalg.norm(line_vec)
            line_unitvec = line_vec / line_len
            point_vec_scaled = point_vec / line_len
            t = np.dot(line_unitvec, point_vec_scaled)
            t = np.clip(t, 0.0, 1.0)
            nearest = line_start + t * line_vec
            return np.linalg.norm(point - nearest)

        def douglas_peucker(points, tolerance):
            """递归简化。"""
            if len(points) < 3:
                return points

            # 找到距离起点-终点连线最远的点
            max_dist = 0.0
            max_index = 0
            for i in range(1, len(points) - 1):
                dist = perpendicular_distance(points[i], points[0], points[-1])
                if dist > max_dist:
                    max_dist = dist
                    max_index = i

            # 如果最大距离小于容差，简化为起点-终点
            if max_dist < tolerance:
                return np.array([points[0], points[-1]])

            # 递归简化两段
            left = douglas_peucker(points[:max_index+1], tolerance)
            right = douglas_peucker(points[max_index:], tolerance)

            # 合并（去除重复点）
            return np.vstack([left[:-1], right])

        return douglas_peucker(path, tolerance)


# ══════════════════════════════════════════════════════════════════
#  SCG 路径规划器
# ══════════════════════════════════════════════════════════════════

class SCGPathPlanner:
    """
    SCG-based 路径规划器 — 纯基于 SCG 的路径规划。

    特点:
      - 在 SCG 上搜索多面体序列
      - 在多面体内部和边界生成路径
      - 不依赖全局 Tomogram

    用法:
        planner = SCGPathPlanner(scg_builder)
        result = planner.plan(start, goal)
    """

    def __init__(self, scg_builder):
        """
        Args:
            scg_builder: SCGBuilder 实例
        """
        self.scg = scg_builder
        self.path_smoother = PathSmoother(iterations=10, alpha=0.5)

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        smooth: bool = True,
        simplify: bool = True,
    ) -> SCGPath:
        """
        规划路径。

        Args:
            start: 起点 [x, y, z]
            goal: 终点 [x, y, z]
            smooth: 是否平滑路径
            simplify: 是否简化路径

        Returns:
            SCGPath 对象
        """
        start_time = time.time()

        # 1. 定位起点和终点所在的多面体
        start_poly_id = self._locate_polyhedron(start)
        goal_poly_id = self._locate_polyhedron(goal)

        if start_poly_id is None:
            logger.warning(f"Start point {start} not in any polyhedron")
            return SCGPath(
                polyhedron_sequence=[],
                segments=[],
                total_distance=0.0,
                planning_time=time.time() - start_time,
                success=False,
            )

        if goal_poly_id is None:
            logger.warning(f"Goal point {goal} not in any polyhedron")
            return SCGPath(
                polyhedron_sequence=[],
                segments=[],
                total_distance=0.0,
                planning_time=time.time() - start_time,
                success=False,
            )

        # 2. 在 SCG 上 A* 搜索多面体序列
        poly_sequence = self._search_polyhedron_sequence(start_poly_id, goal_poly_id)

        if poly_sequence is None:
            logger.warning(f"No path found from poly {start_poly_id} to {goal_poly_id}")
            return SCGPath(
                polyhedron_sequence=[],
                segments=[],
                total_distance=0.0,
                planning_time=time.time() - start_time,
                success=False,
            )

        # 3. 生成穿过多面体序列的路径
        segments = self._generate_path_segments(poly_sequence, start, goal)

        # 4. 合并路径段
        full_path = self._merge_segments(segments)

        # 5. 路径平滑
        if smooth and len(full_path) > 2:
            full_path = self.path_smoother.smooth(full_path)

        # 6. 路径简化
        if simplify and len(full_path) > 2:
            full_path = self.path_smoother.simplify(full_path, tolerance=0.2)

        # 7. 重新生成路径段（基于平滑后的路径）
        segments = self._split_path_into_segments(full_path, poly_sequence)

        # 8. 计算总距离
        total_distance = self._compute_path_length(full_path)

        planning_time = time.time() - start_time

        logger.info(
            f"SCG path planning: {len(poly_sequence)} polyhedra, "
            f"{len(full_path)} waypoints, {total_distance:.2f}m, {planning_time*1000:.2f}ms"
        )

        return SCGPath(
            polyhedron_sequence=poly_sequence,
            segments=segments,
            total_distance=total_distance,
            planning_time=planning_time,
            success=True,
        )

    def _locate_polyhedron(self, point: np.ndarray) -> Optional[int]:
        """
        定位点所在的多面体。

        Args:
            point: [x, y, z]

        Returns:
            多面体 ID 或 None
        """
        for poly_id, poly in self.scg.polyhedra.items():
            if self._point_in_polyhedron(point, poly):
                return poly_id
        return None

    def _search_polyhedron_sequence(
        self,
        start_id: int,
        goal_id: int,
    ) -> Optional[List[int]]:
        """
        A* 搜索多面体序列。

        Args:
            start_id: 起点多面体 ID
            goal_id: 终点多面体 ID

        Returns:
            多面体 ID 序列 或 None
        """
        if start_id == goal_id:
            return [start_id]

        # A* 搜索
        open_set = []
        heapq.heappush(open_set, (0.0, start_id))

        came_from = {}
        g_score = {start_id: 0.0}
        f_score = {start_id: self._heuristic(start_id, goal_id)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_id:
                # 重建路径
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            # 遍历邻居
            for neighbor_id, edge_cost in self._get_neighbors(current):
                tentative_g = g_score[current] + edge_cost

                if neighbor_id not in g_score or tentative_g < g_score[neighbor_id]:
                    came_from[neighbor_id] = current
                    g_score[neighbor_id] = tentative_g
                    f_score[neighbor_id] = tentative_g + self._heuristic(neighbor_id, goal_id)
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))

        return None

    def _get_neighbors(self, poly_id: int) -> List[Tuple[int, float]]:
        """
        获取多面体的邻居。

        Args:
            poly_id: 多面体 ID

        Returns:
            [(neighbor_id, edge_cost), ...]
        """
        neighbors = []

        for edge in self.scg.edges:
            if edge['from_id'] == poly_id:
                neighbors.append((edge['to_id'], edge['cost']))
            elif edge['to_id'] == poly_id:
                neighbors.append((edge['from_id'], edge['cost']))

        return neighbors

    def _heuristic(self, poly_id: int, goal_id: int) -> float:
        """
        启发式函数（欧氏距离）。

        Args:
            poly_id: 当前多面体 ID
            goal_id: 目标多面体 ID

        Returns:
            启发式代价
        """
        poly = self.scg.polyhedra[poly_id]
        goal_poly = self.scg.polyhedra[goal_id]
        return np.linalg.norm(poly.center - goal_poly.center)

    def _generate_path_segments(
        self,
        poly_sequence: List[int],
        start: np.ndarray,
        goal: np.ndarray,
    ) -> List[PathSegment]:
        """
        生成穿过多面体序列的路径段。

        Args:
            poly_sequence: 多面体 ID 序列
            start: 起点
            goal: 终点

        Returns:
            路径段列表
        """
        segments = []

        for i in range(len(poly_sequence) - 1):
            from_id = poly_sequence[i]
            to_id = poly_sequence[i + 1]

            # 找到连接点
            if i == 0:
                # 第一段：从起点到连接点
                from_point = start
            else:
                # 中间段：从上一个连接点到下一个连接点
                from_point = segments[-1].waypoints[-1]

            if i == len(poly_sequence) - 2:
                # 最后一段：从连接点到终点
                to_point = goal
            else:
                # 中间段：到下一个连接点
                to_point = self._find_connection_point(from_id, to_id)

            # 创建路径段
            waypoints = np.array([from_point, to_point])
            distance = np.linalg.norm(to_point - from_point)
            edge_type = self._get_edge_type(from_id, to_id)

            segment = PathSegment(
                from_poly_id=from_id,
                to_poly_id=to_id,
                waypoints=waypoints,
                distance=distance,
                edge_type=edge_type,
            )
            segments.append(segment)

        return segments

    def _find_connection_point(self, poly1_id: int, poly2_id: int) -> np.ndarray:
        """
        找到两个多面体的连接点。

        Args:
            poly1_id: 多面体 1 ID
            poly2_id: 多面体 2 ID

        Returns:
            连接点 [x, y, z]
        """
        poly1 = self.scg.polyhedra[poly1_id]
        poly2 = self.scg.polyhedra[poly2_id]

        # 检查是否有 Adjacency 边（共享面）
        edge = self._find_edge(poly1_id, poly2_id)

        if edge and edge['type'] == 'adjacency':
            # 如果有共享面，返回共享面的中心
            # 简化：返回两个中心的中点
            return 0.5 * (poly1.center + poly2.center)
        else:
            # 否则，返回两个中心的中点
            return 0.5 * (poly1.center + poly2.center)

    def _find_edge(self, poly1_id: int, poly2_id: int) -> Optional[Dict]:
        """查找两个多面体之间的边。"""
        for edge in self.scg.edges:
            if (edge['from_id'] == poly1_id and edge['to_id'] == poly2_id) or \
               (edge['from_id'] == poly2_id and edge['to_id'] == poly1_id):
                return edge
        return None

    def _get_edge_type(self, poly1_id: int, poly2_id: int) -> str:
        """获取边的类型。"""
        edge = self._find_edge(poly1_id, poly2_id)
        return edge['type'] if edge else 'unknown'

    def _merge_segments(self, segments: List[PathSegment]) -> np.ndarray:
        """
        合并路径段为完整路径。

        Args:
            segments: 路径段列表

        Returns:
            (N, 3) 路径点
        """
        if not segments:
            return np.array([])

        waypoints = [segments[0].waypoints[0]]

        for segment in segments:
            waypoints.append(segment.waypoints[-1])

        return np.array(waypoints)

    def _split_path_into_segments(
        self,
        path: np.ndarray,
        poly_sequence: List[int],
    ) -> List[PathSegment]:
        """
        将路径分割为路径段。

        Args:
            path: (N, 3) 路径点
            poly_sequence: 多面体 ID 序列

        Returns:
            路径段列表
        """
        if len(poly_sequence) < 2:
            return []

        segments = []
        points_per_segment = max(2, len(path) // (len(poly_sequence) - 1))

        for i in range(len(poly_sequence) - 1):
            start_idx = i * points_per_segment
            end_idx = min((i + 1) * points_per_segment + 1, len(path))

            if i == len(poly_sequence) - 2:
                end_idx = len(path)

            waypoints = path[start_idx:end_idx]
            distance = self._compute_path_length(waypoints)
            edge_type = self._get_edge_type(poly_sequence[i], poly_sequence[i+1])

            segment = PathSegment(
                from_poly_id=poly_sequence[i],
                to_poly_id=poly_sequence[i+1],
                waypoints=waypoints,
                distance=distance,
                edge_type=edge_type,
            )
            segments.append(segment)

        return segments

    @staticmethod
    def _point_in_polyhedron(point: np.ndarray, polyhedron) -> bool:
        """
        检查点是否在多面体内（简化版本）。

        使用外接球近似。
        """
        distance = np.linalg.norm(point - polyhedron.center)
        return distance <= polyhedron.radius

    @staticmethod
    def _compute_path_length(path: np.ndarray) -> float:
        """计算路径长度。"""
        if len(path) < 2:
            return 0.0

        length = 0.0
        for i in range(len(path) - 1):
            length += np.linalg.norm(path[i+1] - path[i])

        return length
