"""
基线方法包装器 (Baseline Wrappers) — 统一的基线方法接口。

支持的基线方法:
  1. PCT A* (传统 A* 路径规划)
  2. USS-Nav (我们的实现)
  3. 简化的 Voxel-based 规划器

功能:
  - 统一的规划接口
  - 性能监控
  - 与评估框架集成

设计原则:
  - 统一接口: 所有方法使用相同的 API
  - 可扩展: 易于添加新的基线方法
  - 性能监控: 自动收集性能指标
"""

import logging
import time
from abc import ABC, abstractmethod

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  基础规划器接口
# ══════════════════════════════════════════════════════════════════

class BasePlanner(ABC):
    """
    基础规划器 — 所有规划器的基类。

    子类需要实现:
      - initialize()
      - update()
      - plan()
      - get_statistics()
    """

    def __init__(self, name: str):
        """
        Args:
            name: 规划器名称
        """
        self.name = name
        self.initialized = False

    @abstractmethod
    def initialize(self, **kwargs):
        """
        初始化规划器。

        Args:
            **kwargs: 初始化参数
        """
        pass

    @abstractmethod
    def update(self, robot_pose: np.ndarray, point_cloud: np.ndarray):
        """
        更新地图。

        Args:
            robot_pose: 机器人位姿 [x, y, z, roll, pitch, yaw]
            point_cloud: (N, 3) 点云数据
        """
        pass

    @abstractmethod
    def plan(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """
        规划路径。

        Args:
            start: 起点 [x, y, z]
            goal: 终点 [x, y, z]

        Returns:
            (N, 3) 路径点 或 None
        """
        pass

    @abstractmethod
    def get_statistics(self) -> dict:
        """
        获取统计信息。

        Returns:
            统计信息字典
        """
        pass


# ══════════════════════════════════════════════════════════════════
#  PCT A* 基线
# ══════════════════════════════════════════════════════════════════

class PCTAStarPlanner(BasePlanner):
    """
    PCT A* 规划器 — 传统的 A* 路径规划。

    特点:
      - 基于占据栅格的 A*
      - 简单高效
      - 作为基线对比

    用法:
        planner = PCTAStarPlanner()
        planner.initialize(resolution=0.1, size=(100, 100, 40))
        planner.update(robot_pose, point_cloud)
        path = planner.plan(start, goal)
    """

    def __init__(self):
        super().__init__("PCT A*")
        self.resolution = 0.1
        self.occupancy_grid = None
        self.grid_origin = np.array([0, 0, 0])
        self.update_count = 0
        self.planning_count = 0
        self.total_planning_time = 0.0

    def initialize(
        self,
        resolution: float = 0.1,
        size: tuple[int, int, int] = (100, 100, 40),
        origin: np.ndarray = None,
    ):
        """初始化 PCT A* 规划器。"""
        self.resolution = resolution
        self.occupancy_grid = np.zeros(size, dtype=np.float32)
        if origin is not None:
            self.grid_origin = origin
        self.initialized = True
        logger.info(f"PCT A* initialized: resolution={resolution}, size={size}")

    def update(self, robot_pose: np.ndarray, point_cloud: np.ndarray):
        """更新占据栅格。"""
        if not self.initialized:
            raise RuntimeError("Planner not initialized")

        # 简化实现：将点云投影到栅格
        for point in point_cloud:
            grid_pos = self._world_to_grid(point)
            if grid_pos is not None:
                x, y, z = grid_pos
                self.occupancy_grid[x, y, z] = 1.0  # 占据

        self.update_count += 1

    def plan(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """使用 A* 规划路径。"""
        if not self.initialized:
            raise RuntimeError("Planner not initialized")

        start_time = time.time()

        # 转换到栅格坐标
        start_grid = self._world_to_grid(start)
        goal_grid = self._world_to_grid(goal)

        if start_grid is None or goal_grid is None:
            logger.warning("Start or goal outside grid")
            return None

        # A* 搜索（简化版本：2D）
        path_grid = self._astar_2d(start_grid[:2], goal_grid[:2])

        if path_grid is None:
            logger.warning("No path found")
            return None

        # 转换回世界坐标
        path_world = []
        for grid_pos in path_grid:
            world_pos = self._grid_to_world((grid_pos[0], grid_pos[1], start_grid[2]))
            path_world.append(world_pos)

        path = np.array(path_world)

        planning_time = time.time() - start_time
        self.total_planning_time += planning_time
        self.planning_count += 1

        logger.info(f"PCT A* planned path: {len(path)} points, {planning_time*1000:.2f}ms")
        return path

    def get_statistics(self) -> dict:
        """获取统计信息。"""
        return {
            "name": self.name,
            "update_count": self.update_count,
            "planning_count": self.planning_count,
            "avg_planning_time_ms": (
                self.total_planning_time / self.planning_count * 1000
                if self.planning_count > 0
                else 0.0
            ),
            "grid_size": self.occupancy_grid.shape if self.occupancy_grid is not None else None,
            "resolution": self.resolution,
            "memory_mb": (
                self.occupancy_grid.nbytes / 1024 / 1024
                if self.occupancy_grid is not None
                else 0.0
            ),
        }

    def _world_to_grid(self, world_pos: np.ndarray) -> tuple[int, int, int] | None:
        """世界坐标 → 栅格坐标。"""
        grid_pos = ((world_pos - self.grid_origin) / self.resolution).astype(int)

        if np.any(grid_pos < 0) or np.any(grid_pos >= self.occupancy_grid.shape):
            return None

        return tuple(grid_pos)

    def _grid_to_world(self, grid_pos: tuple[int, int, int]) -> np.ndarray:
        """栅格坐标 → 世界坐标。"""
        world_pos = (np.array(grid_pos) + 0.5) * self.resolution + self.grid_origin
        return world_pos

    def _astar_2d(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]] | None:
        """
        2D A* 搜索。

        Args:
            start: 起点栅格坐标 (x, y)
            goal: 终点栅格坐标 (x, y)

        Returns:
            路径栅格坐标列表 或 None
        """
        import heapq

        # 启发式函数（欧氏距离）
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        # 邻居（8-连通）
        def get_neighbors(pos):
            x, y = pos
            neighbors = []
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < self.occupancy_grid.shape[0] and 0 <= ny < self.occupancy_grid.shape[1]:
                        # 检查占据（简化：只检查 z=0 层）
                        if self.occupancy_grid[nx, ny, 0] < 0.5:
                            neighbors.append((nx, ny))
            return neighbors

        # A* 搜索
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # 重建路径
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in get_neighbors(current):
                tentative_g = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None


# ══════════════════════════════════════════════════════════════════
#  USS-Nav 规划器
# ══════════════════════════════════════════════════════════════════

class USSNavPlanner(BasePlanner):
    """
    USS-Nav 规划器 — 我们的实现。

    特点:
      - 基于 SCG 的路径规划
      - 不确定性感知
      - 探索策略

    用法:
        planner = USSNavPlanner()
        planner.initialize()
        planner.update(robot_pose, point_cloud)
        path = planner.plan(start, goal)
    """

    def __init__(self):
        super().__init__("USS-Nav")
        self.scg_builder = None
        self.gcm = None
        self.local_grid = None
        self.path_planner = None
        self.update_count = 0
        self.planning_count = 0
        self.total_planning_time = 0.0

    def initialize(self, **kwargs):
        """初始化 USS-Nav 规划器。"""
        # 导入 USS-Nav 组件
        try:
            from perception.scene_understanding.global_coverage_mask import (
                GlobalCoverageMask,
            )
            from perception.scene_understanding.local_rolling_grid import LocalRollingGrid
            from perception.scene_understanding.scg_builder import SCGBuilder, SCGConfig
            from perception.scene_understanding.scg_path_planner import SCGPathPlanner

            # 创建组件
            self.scg_builder = SCGBuilder(SCGConfig())
            self.gcm = GlobalCoverageMask(resolution=0.5)
            self.local_grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)
            self.path_planner = SCGPathPlanner(self.scg_builder)

            self.initialized = True
            logger.info("USS-Nav initialized")

        except ImportError as e:
            logger.error(f"Failed to import USS-Nav components: {e}")
            raise

    def update(self, robot_pose: np.ndarray, point_cloud: np.ndarray):
        """更新 USS-Nav 地图。"""
        if not self.initialized:
            raise RuntimeError("Planner not initialized")

        # 更新局部滚动栅格
        self.local_grid.update(robot_pose, point_cloud)

        # 更新 GCM
        # （简化：这里应该从 SCG 更新）

        self.update_count += 1

    def plan(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray | None:
        """使用 SCG 规划路径。"""
        if not self.initialized:
            raise RuntimeError("Planner not initialized")

        start_time = time.time()

        # 使用 SCG 路径规划器
        result = self.path_planner.plan(start, goal)

        if not result.success:
            logger.warning("USS-Nav planning failed")
            return None

        # 提取路径点
        path = []
        for segment in result.segments:
            path.extend(segment.waypoints)

        path = np.array(path) if path else None

        planning_time = time.time() - start_time
        self.total_planning_time += planning_time
        self.planning_count += 1

        if path is not None:
            logger.info(f"USS-Nav planned path: {len(path)} points, {planning_time*1000:.2f}ms")

        return path

    def get_statistics(self) -> dict:
        """获取统计信息。"""
        return {
            "name": self.name,
            "update_count": self.update_count,
            "planning_count": self.planning_count,
            "avg_planning_time_ms": (
                self.total_planning_time / self.planning_count * 1000
                if self.planning_count > 0
                else 0.0
            ),
            "num_polyhedra": len(self.scg_builder.nodes) if self.scg_builder else 0,
            "num_edges": len(self.scg_builder.edges) if self.scg_builder else 0,
        }


# ══════════════════════════════════════════════════════════════════
#  规划器工厂
# ══════════════════════════════════════════════════════════════════

def create_planner(planner_type: str, **kwargs) -> BasePlanner:
    """
    创建规划器。

    Args:
        planner_type: 规划器类型 ("pct_astar" | "uss_nav")
        **kwargs: 初始化参数

    Returns:
        规划器实例
    """
    if planner_type == "pct_astar":
        planner = PCTAStarPlanner()
    elif planner_type == "uss_nav":
        planner = USSNavPlanner()
    else:
        raise ValueError(f"Unknown planner type: {planner_type}")

    planner.initialize(**kwargs)
    return planner
