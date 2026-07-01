"""
局部滚动栅格 (Local Rolling Grid) — 固定内存的局部地图。

功能:
  1. 固定大小的局部栅格（8×8×4m）
  2. 跟随机器人移动（滚动更新）
  3. 内存不增长
  4. 贝叶斯占据更新

设计原则:
  - Fixed memory: grid size does not grow
  - 滚动更新: 跟随机器人移动
  - 概率建模: 贝叶斯占据更新

参考论文:
  - USS-Nav (2025): 局部滚动栅格
  - OctoMap: 概率占据栅格
  - Rolling Occupancy Grid: 滚动栅格算法
"""

import logging

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  局部滚动栅格
# ══════════════════════════════════════════════════════════════════

class LocalRollingGrid:
    """
    局部滚动栅格 — 固定内存的局部地图。

    特点:
      - 固定大小（8×8×4m）
      - 跟随机器人移动
      - 内存不增长

    用法:
        grid = LocalRollingGrid(size=(8.0, 8.0, 4.0), resolution=0.1)
        grid.update(robot_pose, point_cloud)
        occupancy = grid.get_occupancy_grid()
    """

    def __init__(
        self,
        size: tuple[float, float, float] = (8.0, 8.0, 4.0),
        resolution: float = 0.1,
        roll_threshold: float = 2.0,
    ):
        """
        Args:
            size: 栅格大小 (x, y, z) in meters
            resolution: 栅格分辨率 (米)
            roll_threshold: 滚动阈值（机器人移动超过此距离时滚动）
        """
        self.size = np.array(size, dtype=np.float64)
        self.resolution = resolution
        self.roll_threshold = roll_threshold

        # 栅格尺寸
        self.grid_size = tuple((self.size / resolution).astype(int))

        # 占据栅格（概率）
        self.occupancy = np.full(self.grid_size, 0.5, dtype=np.float32)

        # 当前中心位置（世界坐标）
        self.center = np.array([0.0, 0.0, 0.0], dtype=np.float64)

        # 贝叶斯更新参数
        self.p_occupied_given_hit = 0.7  # P(occupied | hit)
        self.p_occupied_given_miss = 0.4  # P(occupied | miss)

        # 统计信息
        self.update_count = 0
        self.roll_count = 0

    def update(
        self,
        robot_pose: np.ndarray,
        point_cloud: np.ndarray | None = None,
    ) -> None:
        """
        更新局部栅格。

        Args:
            robot_pose: 机器人位姿 [x, y, z] 或 [x, y, z, roll, pitch, yaw]
            point_cloud: (N, 3) 点云数据（世界坐标）
        """
        robot_pos = robot_pose[:3]

        # 1. 检查是否需要滚动
        if self._should_roll(robot_pos):
            self._roll_grid(robot_pos)

        # 2. 更新占据栅格
        if point_cloud is not None and len(point_cloud) > 0:
            self._update_occupancy(robot_pos, point_cloud)

        self.update_count += 1

    def _should_roll(self, robot_pos: np.ndarray) -> bool:
        """
        判断是否需要滚动。

        Args:
            robot_pos: 机器人位置 [x, y, z]

        Returns:
            是否需要滚动
        """
        distance = np.linalg.norm(robot_pos[:2] - self.center[:2])
        return distance > self.roll_threshold

    def _roll_grid(self, new_center: np.ndarray) -> None:
        """
        滚动栅格到新中心。

        方法:
          1. 计算偏移量（栅格单元数）
          2. 使用 numpy.roll 平移栅格内容
          3. 清空新区域（设置为未知）

        Args:
            new_center: 新中心位置 [x, y, z]
        """
        # 计算偏移量（世界坐标）
        offset = new_center - self.center

        # 转换为栅格单元数
        grid_offset = (offset / self.resolution).astype(int)

        # 滚动栅格（只滚动 x 和 y 轴）
        if grid_offset[0] != 0:
            self.occupancy = np.roll(self.occupancy, -grid_offset[0], axis=0)
            # 清空新区域
            if grid_offset[0] > 0:
                self.occupancy[:grid_offset[0], :, :] = 0.5
            else:
                self.occupancy[grid_offset[0]:, :, :] = 0.5

        if grid_offset[1] != 0:
            self.occupancy = np.roll(self.occupancy, -grid_offset[1], axis=1)
            # 清空新区域
            if grid_offset[1] > 0:
                self.occupancy[:, :grid_offset[1], :] = 0.5
            else:
                self.occupancy[:, grid_offset[1]:, :] = 0.5

        # 更新中心
        self.center = new_center
        self.roll_count += 1

        logger.debug(
            f"Rolled grid to {new_center}, offset={grid_offset}, "
            f"roll_count={self.roll_count}"
        )

    def _update_occupancy(
        self,
        robot_pos: np.ndarray,
        point_cloud: np.ndarray,
    ) -> None:
        """
        更新占据栅格（贝叶斯更新）。

        方法:
          1. 将点云转换到栅格坐标
          2. 对每个点，更新其占据概率（hit）
          3. 对射线路径上的点，更新为自由（miss）

        Args:
            robot_pos: 机器人位置 [x, y, z]
            point_cloud: (N, 3) 点云数据
        """
        # 简化版本：只更新点云位置为占据，不进行射线追踪
        for point in point_cloud:
            grid_pos = self.world_to_grid(point)

            if grid_pos is not None:
                # 贝叶斯更新（hit）
                self._bayesian_update(grid_pos, hit=True)

    def _bayesian_update(self, grid_pos: tuple[int, int, int], hit: bool) -> None:
        """
        贝叶斯占据更新。

        公式:
          P(occ|z) = P(z|occ) * P(occ) / P(z)

        Args:
            grid_pos: 栅格坐标 (x, y, z)
            hit: True=观测到占据, False=观测到自由
        """
        x, y, z = grid_pos

        # 当前占据概率
        p_occ = self.occupancy[x, y, z]

        # 观测概率
        if hit:
            p_z_given_occ = self.p_occupied_given_hit
            p_z_given_free = 1.0 - self.p_occupied_given_hit
        else:
            p_z_given_occ = self.p_occupied_given_miss
            p_z_given_free = 1.0 - self.p_occupied_given_miss

        # 贝叶斯更新
        p_z = p_z_given_occ * p_occ + p_z_given_free * (1.0 - p_occ)
        p_occ_new = (p_z_given_occ * p_occ) / p_z

        # 限制范围
        p_occ_new = np.clip(p_occ_new, 0.01, 0.99)

        # 更新
        self.occupancy[x, y, z] = p_occ_new

    def get_occupancy_grid(self) -> np.ndarray:
        """
        获取占据栅格。

        Returns:
            (X, Y, Z) 占据栅格 (0-1 概率)
        """
        return self.occupancy.copy()

    def get_binary_occupancy(self, threshold: float = 0.5) -> np.ndarray:
        """
        获取二值占据栅格。

        Args:
            threshold: 占据阈值

        Returns:
            (X, Y, Z) 二值栅格 (0=自由, 1=占据)
        """
        return (self.occupancy > threshold).astype(np.float32)

    def world_to_grid(self, world_pos: np.ndarray) -> tuple[int, int, int] | None:
        """
        世界坐标 → 栅格坐标。

        Args:
            world_pos: [x, y, z] 世界坐标

        Returns:
            (grid_x, grid_y, grid_z) 或 None（如果超出边界）
        """
        # 相对于中心的位置
        relative_pos = world_pos - self.center

        # 转换为栅格坐标（中心对应栅格中心）
        grid_pos = (relative_pos / self.resolution + np.array(self.grid_size) / 2).astype(int)

        # 边界检查
        if np.any(grid_pos < 0) or np.any(grid_pos >= self.grid_size):
            return None

        return tuple(grid_pos)

    def grid_to_world(self, grid_pos: tuple[int, int, int]) -> np.ndarray:
        """
        栅格坐标 → 世界坐标（栅格中心）。

        Args:
            grid_pos: (grid_x, grid_y, grid_z)

        Returns:
            [x, y, z] 世界坐标
        """
        # 栅格坐标 → 相对位置
        relative_pos = (np.array(grid_pos) + 0.5 - np.array(self.grid_size) / 2) * self.resolution

        # 相对位置 → 世界坐标
        world_pos = self.center + relative_pos

        return world_pos

    def reset(self) -> None:
        """重置栅格。"""
        self.occupancy.fill(0.5)
        self.center = np.array([0.0, 0.0, 0.0])
        self.update_count = 0
        self.roll_count = 0

    def get_statistics(self) -> dict:
        """获取统计信息。"""
        return {
            'grid_size': self.grid_size,
            'resolution': self.resolution,
            'center': self.center.tolist(),
            'update_count': self.update_count,
            'roll_count': self.roll_count,
            'occupied_cells': int(np.sum(self.occupancy > 0.5)),
            'free_cells': int(np.sum(self.occupancy < 0.5)),
            'unknown_cells': int(np.sum(np.abs(self.occupancy - 0.5) < 0.01)),
        }


# ══════════════════════════════════════════════════════════════════
#  辅助函数
# ══════════════════════════════════════════════════════════════════

def create_mock_point_cloud(
    robot_pos: np.ndarray,
    num_points: int = 1000,
    max_range: float = 5.0,
) -> np.ndarray:
    """
    创建模拟点云（用于测试）。

    Args:
        robot_pos: 机器人位置 [x, y, z]
        num_points: 点数
        max_range: 最大距离

    Returns:
        (N, 3) 点云
    """
    # 在机器人周围随机生成点
    points = []

    for _ in range(num_points):
        # 随机方向
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, np.pi)
        r = np.random.uniform(0.5, max_range)

        # 球坐标 → 笛卡尔坐标
        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi)

        point = robot_pos + np.array([x, y, z])
        points.append(point)

    return np.array(points)
