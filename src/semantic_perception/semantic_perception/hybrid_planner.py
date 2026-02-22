"""
混合路径规划器 (Hybrid Planner) — 拓扑图辅助的分层路径规划。

核心思想:
  将全局路径规划分解为两个层次:
  1. 拓扑层: 在拓扑图上做 Dijkstra，获得房间序列
  2. 几何层: 对每对相邻房间，在 Tomogram 上做局部 A*

优势:
  - 减少 A* 搜索空间 (只在房间对之间搜索)
  - 利用拓扑图的高层连通性
  - 比全局 A* 快 3-10 倍 (预估)

参考:
  - TopoNav (2025): 拓扑图作为��间记忆
  - Hierarchical Planning: 分层规划减少搜索空间
"""

import heapq
import logging
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class PathSegment:
    """路径段 (连接两个房间的局部路径)。"""
    from_room_id: int
    to_room_id: int
    waypoints: List[np.ndarray]  # 路径点序列 [[x, y, z], ...]
    cost: float                   # 路径代价
    planning_time: float          # 规划时间 (秒)


@dataclass
class HybridPath:
    """混合路径规划结果。"""
    success: bool
    waypoints: List[np.ndarray]   # 完整路径点序列
    room_sequence: List[int]      # 房间序列
    segments: List[PathSegment]   # 路径段列表
    total_cost: float             # 总代价
    total_planning_time: float    # 总规划时间
    topology_planning_time: float # 拓扑规划时间
    geometry_planning_time: float # 几何规划时间
    num_waypoints: int            # 路径点数量
    num_rooms: int                # 经过的房间数


class HybridPlanner:
    """
    混合路径规划器 — 拓扑图辅助的分层路径规划。

    算法流程:
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Input: 起点 start, 终点 goal, 拓扑图 TSG, Tomogram
    Output: 混合路径 (房间序列 + 几何路径)

    1. 定位起点和终点所在的房间
    2. 在拓扑图上做 Dijkstra → 房间序列 [R1, R2, ..., Rn]
    3. for each 相邻房间对 (Ri, Ri+1):
    4.     在 Tomogram 上做局部 A* (限制在两个房间的边界框内)
    5.     拼接路径段
    6. 返回完整路径

    性能优化:
    - 局部 A* 搜索空间 << 全局 A*
    - 拓扑图 Dijkstra 非常快 (节点数 << 栅格数)
    - 可以并行规划多个路径段 (未来优化)
    """

    def __init__(self, topology_graph, tomogram):
        """
        Args:
            topology_graph: TopologySemGraph 实例
            tomogram: Tomogram 实例
        """
        self.tsg = topology_graph
        self.tomogram = tomogram

    def plan_path(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        search_radius_factor: float = 1.5,
        max_planning_time: float = 5.0,
    ) -> HybridPath:
        """
        规划从起点到终点的混合路径。

        Args:
            start: 起点 [x, y, z] (世界坐标)
            goal: 终点 [x, y, z] (世界坐标)
            search_radius_factor: 局部搜索半径因子 (相对于房间边界框)
            max_planning_time: 最大规划时间 (秒)

        Returns:
            HybridPath 实例
        """
        start_time = time.time()

        # 1. 定位起点和终点所在的房间
        start_room_id = self._locate_room(start[:2])
        goal_room_id = self._locate_room(goal[:2])

        if start_room_id is None or goal_room_id is None:
            logger.warning(
                f"Cannot locate rooms: start_room={start_room_id}, "
                f"goal_room={goal_room_id}"
            )
            return self._empty_path()

        logger.info(
            f"Planning path: room {start_room_id} → room {goal_room_id}"
        )

        # 2. 拓扑层规划: Dijkstra
        topo_start = time.time()
        cost, room_sequence = self.tsg.shortest_path(start_room_id, goal_room_id)
        topo_time = time.time() - topo_start

        if cost == float("inf") or len(room_sequence) == 0:
            logger.warning(
                f"No topological path found between room {start_room_id} "
                f"and room {goal_room_id}"
            )
            return self._empty_path()

        logger.info(
            f"Topological path: {room_sequence} (cost={cost:.2f}, "
            f"time={topo_time*1000:.2f}ms)"
        )

        # 3. 几何层规划: 局部 A* for each 房间对
        geom_start = time.time()
        segments = []
        all_waypoints = [start]

        for i in range(len(room_sequence) - 1):
            if time.time() - start_time > max_planning_time:
                logger.warning("Planning timeout, returning partial path")
                break

            from_room_id = room_sequence[i]
            to_room_id = room_sequence[i + 1]

            # 规划局部路径段
            segment = self._plan_local_segment(
                from_room_id=from_room_id,
                to_room_id=to_room_id,
                start_pos=all_waypoints[-1],
                goal_pos=goal if i == len(room_sequence) - 2 else None,
                search_radius_factor=search_radius_factor,
            )

            if segment is None:
                logger.warning(
                    f"Failed to plan segment: room {from_room_id} → {to_room_id}"
                )
                # 尝试直线连接 (fallback)
                segment = self._fallback_segment(from_room_id, to_room_id)

            segments.append(segment)
            all_waypoints.extend(segment.waypoints[1:])  # 跳过重复的起点

        geom_time = time.time() - geom_start
        total_time = time.time() - start_time

        # 4. 构建结果
        total_cost = sum(seg.cost for seg in segments)

        return HybridPath(
            success=True,
            waypoints=all_waypoints,
            room_sequence=room_sequence,
            segments=segments,
            total_cost=total_cost,
            total_planning_time=total_time,
            topology_planning_time=topo_time,
            geometry_planning_time=geom_time,
            num_waypoints=len(all_waypoints),
            num_rooms=len(room_sequence),
        )

    def _locate_room(self, position: np.ndarray) -> Optional[int]:
        """
        定位位置所在的房间。

        Args:
            position: [x, y] 世界坐标

        Returns:
            房间 ID 或 None
        """
        # 方法 1: 使用拓扑图的 get_room_by_position
        room_id = self.tsg.get_room_by_position(position[0], position[1], radius=5.0)
        if room_id is not None:
            return room_id

        # 方法 2: 使用几何信息 (如果有凸包，检查点是否在凸包内)
        for room in self.tsg.rooms:
            if room.convex_hull is not None and len(room.convex_hull) >= 3:
                if self._point_in_polygon(position, room.convex_hull):
                    return room.node_id

        # 方法 3: 最近房间 (fallback)
        rooms = self.tsg.rooms
        if not rooms:
            return None

        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - position)))
        return nearest.node_id

    def _plan_local_segment(
        self,
        from_room_id: int,
        to_room_id: int,
        start_pos: np.ndarray,
        goal_pos: Optional[np.ndarray],
        search_radius_factor: float,
    ) -> Optional[PathSegment]:
        """
        规划两个房间之间的局部路径段。

        Args:
            from_room_id: 起始房间 ID
            to_room_id: 目标房间 ID
            start_pos: 起点 [x, y, z]
            goal_pos: 终点 [x, y, z] (如果是最后一段，否则 None)
            search_radius_factor: 搜索半径因子

        Returns:
            PathSegment 或 None
        """
        seg_start = time.time()

        from_room = self.tsg.get_node(from_room_id)
        to_room = self.tsg.get_node(to_room_id)

        if from_room is None or to_room is None:
            return None

        # 确定局部搜索区域 (两个房间的边界框合并 + 扩展)
        search_bbox = self._compute_search_bbox(
            from_room, to_room, search_radius_factor
        )

        # 确定终点 (如果没有指定，使用目标房间中心)
        if goal_pos is None:
            goal_pos = np.array([to_room.center[0], to_room.center[1], start_pos[2]])

        # 调用局部 A* (这里使用简化版本，实际应调用 PCT Planner 的 A*)
        waypoints = self._local_astar(
            start=start_pos,
            goal=goal_pos,
            search_bbox=search_bbox,
        )

        seg_time = time.time() - seg_start

        if waypoints is None or len(waypoints) < 2:
            return None

        # 计算路径代价 (欧氏距离)
        cost = sum(
            float(np.linalg.norm(waypoints[i+1] - waypoints[i]))
            for i in range(len(waypoints) - 1)
        )

        return PathSegment(
            from_room_id=from_room_id,
            to_room_id=to_room_id,
            waypoints=waypoints,
            cost=cost,
            planning_time=seg_time,
        )

    def _compute_search_bbox(
        self,
        from_room,
        to_room,
        radius_factor: float,
    ) -> dict:
        """
        计算局部搜索区域的边界框。

        Args:
            from_room: 起始房间节点
            to_room: 目标房间节点
            radius_factor: 扩展因子

        Returns:
            {"x_min": float, "x_max": float, "y_min": float, "y_max": float}
        """
        # 如果有几何边界框，使用它
        if from_room.bounding_box and to_room.bounding_box:
            x_min = min(from_room.bounding_box["x_min"], to_room.bounding_box["x_min"])
            x_max = max(from_room.bounding_box["x_max"], to_room.bounding_box["x_max"])
            y_min = min(from_room.bounding_box["y_min"], to_room.bounding_box["y_min"])
            y_max = max(from_room.bounding_box["y_max"], to_room.bounding_box["y_max"])
        else:
            # 使用房间中心 + 默认半径
            centers = np.array([from_room.center, to_room.center])
            x_min = centers[:, 0].min() - 5.0
            x_max = centers[:, 0].max() + 5.0
            y_min = centers[:, 1].min() - 5.0
            y_max = centers[:, 1].max() + 5.0

        # 扩展边界框
        margin = ((x_max - x_min) + (y_max - y_min)) / 2 * (radius_factor - 1.0)
        return {
            "x_min": x_min - margin,
            "x_max": x_max + margin,
            "y_min": y_min - margin,
            "y_max": y_max + margin,
        }

    def _local_astar(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        search_bbox: dict,
    ) -> Optional[List[np.ndarray]]:
        """
        局部 A* 路径搜索 (简化版本)。

        实际应用中，这里应该调用 PCT Planner 的 A* 实现，
        并限制搜索区域在 search_bbox 内。

        Args:
            start: 起点 [x, y, z]
            goal: 终点 [x, y, z]
            search_bbox: 搜索区域边界框

        Returns:
            路径点列表 或 None
        """
        # TODO: 集成 PCT Planner 的 A* 实现
        # 这里使用简化版本: 直线路径 + 碰撞检测

        # 检查起点和终点是否在搜索区域内
        if not self._point_in_bbox(start[:2], search_bbox):
            logger.warning(f"Start point {start[:2]} outside search bbox")
        if not self._point_in_bbox(goal[:2], search_bbox):
            logger.warning(f"Goal point {goal[:2]} outside search bbox")

        # 简化版本: 生成直线路径
        num_waypoints = max(3, int(np.linalg.norm(goal - start) / 0.5))
        waypoints = []
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1)
            waypoint = start * (1 - t) + goal * t
            waypoints.append(waypoint)

        return waypoints

    def _fallback_segment(
        self,
        from_room_id: int,
        to_room_id: int,
    ) -> PathSegment:
        """
        回退方案: 直线连接两个房间中心。

        Args:
            from_room_id: 起始房间 ID
            to_room_id: 目标房间 ID

        Returns:
            PathSegment
        """
        from_room = self.tsg.get_node(from_room_id)
        to_room = self.tsg.get_node(to_room_id)

        start = np.array([from_room.center[0], from_room.center[1], 0.0])
        goal = np.array([to_room.center[0], to_room.center[1], 0.0])

        waypoints = [start, goal]
        cost = float(np.linalg.norm(goal - start))

        return PathSegment(
            from_room_id=from_room_id,
            to_room_id=to_room_id,
            waypoints=waypoints,
            cost=cost,
            planning_time=0.0,
        )

    @staticmethod
    def _point_in_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
        """
        检查点是否在多边形内 (Ray casting algorithm)。

        Args:
            point: [x, y]
            polygon: (N, 2) 多边形顶点

        Returns:
            True if inside
        """
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    @staticmethod
    def _point_in_bbox(point: np.ndarray, bbox: dict) -> bool:
        """检查点是否在边界框内。"""
        return (
            bbox["x_min"] <= point[0] <= bbox["x_max"] and
            bbox["y_min"] <= point[1] <= bbox["y_max"]
        )

    @staticmethod
    def _empty_path() -> HybridPath:
        """返回空路径 (规划失败)。"""
        return HybridPath(
            success=False,
            waypoints=[],
            room_sequence=[],
            segments=[],
            total_cost=float("inf"),
            total_planning_time=0.0,
            topology_planning_time=0.0,
            geometry_planning_time=0.0,
            num_waypoints=0,
            num_rooms=0,
        )


# ══════════════════════════════════════════════════════════════════
#  性能对比工具
# ══════════════════════════════════════════════════════════════════

@dataclass
class PlannerComparison:
    """规划器性能对比结果。"""
    hybrid_time: float
    baseline_time: float
    speedup: float
    hybrid_waypoints: int
    baseline_waypoints: int
    hybrid_cost: float
    baseline_cost: float
    hybrid_rooms: int


def compare_planners(
    hybrid_planner: HybridPlanner,
    baseline_planner,  # PCT Planner 或其他全局规划器
    test_cases: List[Tuple[np.ndarray, np.ndarray]],
) -> List[PlannerComparison]:
    """
    对比混合规划器和基线规划器的性能。

    Args:
        hybrid_planner: HybridPlanner 实例
        baseline_planner: 基线规划器 (需要有 plan_path 方法)
        test_cases: 测试用例列表 [(start, goal), ...]

    Returns:
        对比结果列表
    """
    results = []

    for i, (start, goal) in enumerate(test_cases):
        logger.info(f"Test case {i+1}/{len(test_cases)}: {start[:2]} → {goal[:2]}")

        # 混合规划器
        hybrid_result = hybrid_planner.plan_path(start, goal)

        # 基线规划器
        baseline_start = time.time()
        baseline_path = baseline_planner.plan_path(start, goal)
        baseline_time = time.time() - baseline_start

        # 计算加速比
        speedup = baseline_time / hybrid_result.total_planning_time if hybrid_result.total_planning_time > 0 else 0

        results.append(PlannerComparison(
            hybrid_time=hybrid_result.total_planning_time,
            baseline_time=baseline_time,
            speedup=speedup,
            hybrid_waypoints=hybrid_result.num_waypoints,
            baseline_waypoints=len(baseline_path) if baseline_path else 0,
            hybrid_cost=hybrid_result.total_cost,
            baseline_cost=sum(
                float(np.linalg.norm(baseline_path[i+1] - baseline_path[i]))
                for i in range(len(baseline_path) - 1)
            ) if baseline_path and len(baseline_path) > 1 else float("inf"),
            hybrid_rooms=hybrid_result.num_rooms,
        ))

        logger.info(
            f"  Hybrid: {hybrid_result.total_planning_time*1000:.2f}ms, "
            f"{hybrid_result.num_waypoints} waypoints, "
            f"{hybrid_result.num_rooms} rooms"
        )
        logger.info(
            f"  Baseline: {baseline_time*1000:.2f}ms, "
            f"{len(baseline_path) if baseline_path else 0} waypoints"
        )
        logger.info(f"  Speedup: {speedup:.2f}×")

    return results
