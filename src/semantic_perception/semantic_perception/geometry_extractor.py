"""
几何提取器 (Geometry Extractor) — 从 Tomogram 提取房间几何信息。

功能:
  1. 从 Tomogram 多层可通行性栅格中提取房间的几何形状
  2. 计算边界框 (AABB)
  3. 生成凸包 (2D 多边形)
  4. 计算可通行面积
  5. 提取高度范围

设计原则:
  - 轻量级: 只提取关键几何信息，不存储完整栅格
  - 鲁棒性: 处理边界情况和异常数据
  - 性能: 单个房间提取 < 10ms
"""

import logging
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial import ConvexHull

logger = logging.getLogger(__name__)


class GeometryExtractor:
    """从 Tomogram 提取房间几何信息的工具类。"""

    def __init__(self, tomogram):
        """
        Args:
            tomogram: PCT Tomogram 实例
        """
        self.tomogram = tomogram
        self.resolution = tomogram.resolution
        self.map_center = tomogram.center
        self.map_dim_x = tomogram.map_dim_x
        self.map_dim_y = tomogram.map_dim_y

    def extract_room_geometry(
        self,
        room_center: np.ndarray,
        search_radius: float = 5.0,
        cost_threshold: float = 0.5,
        min_cells: int = 10,
    ) -> Dict[str, Any]:
        """
        提取房间的几何信息。

        算法流程:
        1. 在 Tomogram 中定位房间中心对应的栅格坐标
        2. 在 search_radius 范围内提取可通行栅格 (trav_cost < threshold)
        3. 计算边界框 (AABB)
        4. 生成凸包 (QuickHull / scipy.spatial.ConvexHull)
        5. 计算可通行面积
        6. 提取高度范围 (从 layers_g 和 layers_c)

        Args:
            room_center: 房间中心 [x, y] (世界坐标)
            search_radius: 搜索半径 (米)
            cost_threshold: 可通行代价阈值 (0-1)
            min_cells: 最小可通行栅格数 (用于过滤噪声)

        Returns:
            {
                "bounding_box": {"x_min": ..., "x_max": ..., "y_min": ..., "y_max": ...},
                "convex_hull": np.ndarray (N, 2),
                "traversable_area": float,
                "height_range": {"floor": float, "ceiling": float},
                "confidence": float,
            }
        """
        try:
            # 1. 世界坐标 → 栅格坐标
            center_grid = self.world_to_grid(room_center)
            if center_grid is None:
                logger.warning(f"Room center {room_center} out of map bounds")
                return self._empty_geometry()

            # 2. 提取可通行栅格
            radius_cells = int(search_radius / self.resolution)
            traversable_cells = self.extract_traversable_cells(
                center_grid=center_grid,
                radius_cells=radius_cells,
                cost_threshold=cost_threshold,
            )

            if len(traversable_cells) < min_cells:
                logger.warning(
                    f"Too few traversable cells ({len(traversable_cells)}) "
                    f"for room at {room_center}"
                )
                return self._empty_geometry()

            # 3. 计算边界框
            bounding_box = self.compute_bounding_box(traversable_cells)

            # 4. 计算凸包
            convex_hull = self.compute_convex_hull(traversable_cells)

            # 5. 计算可通行面积
            traversable_area = self.compute_traversable_area(traversable_cells)

            # 6. 提取高度范围
            height_range = self.extract_height_range(center_grid, radius_cells)

            # 7. 计算置信度
            confidence = self._compute_confidence(
                len(traversable_cells), convex_hull, bounding_box
            )

            return {
                "bounding_box": bounding_box,
                "convex_hull": convex_hull,
                "traversable_area": traversable_area,
                "height_range": height_range,
                "confidence": confidence,
            }

        except Exception as e:
            logger.error(f"Failed to extract geometry for room at {room_center}: {e}")
            return self._empty_geometry()

    def world_to_grid(self, world_pos: np.ndarray) -> Optional[Tuple[int, int]]:
        """
        世界坐标 → 栅格坐标。

        Args:
            world_pos: [x, y] 世界坐标

        Returns:
            (grid_x, grid_y) 或 None (如果超出边界)
        """
        # Tomogram 中心对应栅格中心
        dx = world_pos[0] - self.map_center[0]
        dy = world_pos[1] - self.map_center[1]

        grid_x = int(self.map_dim_x / 2 + dx / self.resolution)
        grid_y = int(self.map_dim_y / 2 + dy / self.resolution)

        if 0 <= grid_x < self.map_dim_x and 0 <= grid_y < self.map_dim_y:
            return (grid_x, grid_y)
        return None

    def grid_to_world(self, grid_x: int, grid_y: int) -> np.ndarray:
        """
        栅格坐标 → 世界坐标。

        Args:
            grid_x, grid_y: 栅格坐标

        Returns:
            [x, y] 世界坐标
        """
        dx = (grid_x - self.map_dim_x / 2) * self.resolution
        dy = (grid_y - self.map_dim_y / 2) * self.resolution

        return np.array([
            self.map_center[0] + dx,
            self.map_center[1] + dy,
        ], dtype=np.float64)

    def extract_traversable_cells(
        self,
        center_grid: Tuple[int, int],
        radius_cells: int,
        cost_threshold: float,
    ) -> List[Tuple[int, int]]:
        """
        提取可通行栅格单元。

        Args:
            center_grid: 中心栅格坐标 (grid_x, grid_y)
            radius_cells: 搜索半径 (栅格数)
            cost_threshold: 可通行代价阈值 (0-1)

        Returns:
            可通行栅格坐标列表 [(grid_x, grid_y), ...]
        """
        cx, cy = center_grid
        traversable = []

        # 使用第一层 (地面层) 的可通行性
        # inflated_cost: 0 = 完全可通行, 1 = 障碍物
        cost_map = self.tomogram.inflated_cost[0]  # (map_dim_x, map_dim_y)

        x_min = max(0, cx - radius_cells)
        x_max = min(self.map_dim_x, cx + radius_cells + 1)
        y_min = max(0, cy - radius_cells)
        y_max = min(self.map_dim_y, cy + radius_cells + 1)

        for gx in range(x_min, x_max):
            for gy in range(y_min, y_max):
                # 圆形搜索区域
                dist_sq = (gx - cx) ** 2 + (gy - cy) ** 2
                if dist_sq > radius_cells ** 2:
                    continue

                # 可通行性检查
                if cost_map[gx, gy] < cost_threshold:
                    traversable.append((gx, gy))

        return traversable

    def compute_bounding_box(
        self,
        cells: List[Tuple[int, int]],
    ) -> Dict[str, float]:
        """
        计算边界框 (AABB)。

        Args:
            cells: 栅格坐标列表 [(grid_x, grid_y), ...]

        Returns:
            {"x_min": float, "x_max": float, "y_min": float, "y_max": float}
        """
        if not cells:
            return {"x_min": 0.0, "x_max": 0.0, "y_min": 0.0, "y_max": 0.0}

        grid_xs = [c[0] for c in cells]
        grid_ys = [c[1] for c in cells]

        # 转换为世界坐标
        min_world = self.grid_to_world(min(grid_xs), min(grid_ys))
        max_world = self.grid_to_world(max(grid_xs), max(grid_ys))

        return {
            "x_min": float(min_world[0]),
            "x_max": float(max_world[0]),
            "y_min": float(min_world[1]),
            "y_max": float(max_world[1]),
        }

    def compute_convex_hull(
        self,
        cells: List[Tuple[int, int]],
    ) -> np.ndarray:
        """
        计算凸包 (使用 scipy.spatial.ConvexHull)。

        Args:
            cells: 栅格坐标列表 [(grid_x, grid_y), ...]

        Returns:
            (N, 2) 数组，凸包顶点 (世界坐标)
        """
        if len(cells) < 3:
            # 不足 3 个点，无法构成凸包
            return np.array([[0.0, 0.0]], dtype=np.float64)

        # 转换为世界坐标
        world_points = np.array([
            self.grid_to_world(gx, gy) for gx, gy in cells
        ], dtype=np.float64)

        try:
            hull = ConvexHull(world_points)
            # 返回凸包顶点 (按顺序)
            hull_vertices = world_points[hull.vertices]
            return hull_vertices

        except Exception as e:
            logger.warning(f"Failed to compute convex hull: {e}")
            # 退化情况: 返回边界框的四个角点
            bbox = self.compute_bounding_box(cells)
            return np.array([
                [bbox["x_min"], bbox["y_min"]],
                [bbox["x_max"], bbox["y_min"]],
                [bbox["x_max"], bbox["y_max"]],
                [bbox["x_min"], bbox["y_max"]],
            ], dtype=np.float64)

    def compute_traversable_area(
        self,
        cells: List[Tuple[int, int]],
    ) -> float:
        """
        计算可通行面积 (平方米)。

        Args:
            cells: 栅格坐标列表 [(grid_x, grid_y), ...]

        Returns:
            面积 (平方米)
        """
        cell_area = self.resolution ** 2
        return len(cells) * cell_area

    def extract_height_range(
        self,
        center_grid: Tuple[int, int],
        radius_cells: int,
    ) -> Dict[str, float]:
        """
        提取高度范围 (从 layers_g 和 layers_c)。

        Args:
            center_grid: 中心栅格坐标 (grid_x, grid_y)
            radius_cells: 搜索半径 (栅格数)

        Returns:
            {"floor": float, "ceiling": float}
        """
        cx, cy = center_grid

        x_min = max(0, cx - radius_cells)
        x_max = min(self.map_dim_x, cx + radius_cells + 1)
        y_min = max(0, cy - radius_cells)
        y_max = min(self.map_dim_y, cy + radius_cells + 1)

        # 提取地面高程 (layers_g) 和天花板高程 (layers_c)
        # 使用第一层 (地面层)
        floor_heights = self.tomogram.layers_g[0, x_min:x_max, y_min:y_max]
        ceiling_heights = self.tomogram.layers_c[0, x_min:x_max, y_min:y_max]

        # 过滤无效值 (Tomogram 中 -1e6 表示未观测)
        valid_floor = floor_heights[floor_heights > -1e5]
        valid_ceiling = ceiling_heights[ceiling_heights < 1e5]

        if len(valid_floor) == 0 or len(valid_ceiling) == 0:
            return {"floor": 0.0, "ceiling": 2.5}

        floor = float(np.median(valid_floor))
        ceiling = float(np.median(valid_ceiling))

        return {"floor": floor, "ceiling": ceiling}

    def _compute_confidence(
        self,
        num_cells: int,
        convex_hull: np.ndarray,
        bounding_box: Dict[str, float],
    ) -> float:
        """
        计算几何信息的置信度。

        考虑因素:
        1. 可通行栅格数量 (越多越可靠)
        2. 凸包顶点数 (越多越复杂，可能越准确)
        3. 凸包面积 vs 边界框面积 (比值越高越规则)

        Returns:
            0.0-1.0 的置信度
        """
        # 1. 栅格数量因子 (10-1000 个栅格 → 0.5-1.0)
        cell_factor = min(1.0, 0.5 + num_cells / 2000.0)

        # 2. 凸包顶点数因子 (3-20 个顶点 → 0.5-1.0)
        vertex_factor = min(1.0, 0.5 + len(convex_hull) / 40.0)

        # 3. 形状规则性因子
        bbox_area = (
            (bounding_box["x_max"] - bounding_box["x_min"]) *
            (bounding_box["y_max"] - bounding_box["y_min"])
        )
        if bbox_area > 0 and len(convex_hull) >= 3:
            # 计算凸包面积 (Shoelace formula)
            hull_area = self._polygon_area(convex_hull)
            shape_factor = hull_area / bbox_area if bbox_area > 0 else 0.5
        else:
            shape_factor = 0.5

        # 综合置信度
        confidence = (cell_factor + vertex_factor + shape_factor) / 3.0
        return float(np.clip(confidence, 0.0, 1.0))

    @staticmethod
    def _polygon_area(vertices: np.ndarray) -> float:
        """
        计算多边形面积 (Shoelace formula)。

        Args:
            vertices: (N, 2) 顶点数组

        Returns:
            面积 (平方米)
        """
        if len(vertices) < 3:
            return 0.0

        x = vertices[:, 0]
        y = vertices[:, 1]

        # Shoelace formula
        area = 0.5 * np.abs(
            np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))
        )
        return float(area)

    @staticmethod
    def _empty_geometry() -> Dict[str, Any]:
        """返回空几何信息 (用于错误情况)。"""
        return {
            "bounding_box": {"x_min": 0.0, "x_max": 0.0, "y_min": 0.0, "y_max": 0.0},
            "convex_hull": np.array([[0.0, 0.0]], dtype=np.float64),
            "traversable_area": 0.0,
            "height_range": {"floor": 0.0, "ceiling": 2.5},
            "confidence": 0.0,
        }
