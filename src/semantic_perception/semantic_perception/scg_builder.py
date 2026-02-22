"""
空间连通图 (Spatial Connectivity Graph, SCG) 构建器。

功能:
  将多面体节点连接成拓扑图，支持三种拓扑边:
  1. Adjacency (邻接): 共享面或边
  2. Connectivity (连通): 自由空间通道
  3. Accessibility (可达): 间接可达

特性:
  - 增量更新: 支持动态添加/删除节点
  - 回环检测: 检测并合并重复的多面体
  - 路径搜索: Dijkstra 最短路径
  - 区域分割: 支持 Leiden 社区检测

参考:
  - USS-Nav (2025): Spatial Connectivity Graph
"""

import heapq
import logging
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from scipy.spatial import KDTree

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ════════════════════════════════════���═════════════════════════════

class EdgeType(Enum):
    """拓扑边类型。"""
    ADJACENCY = "adjacency"        # 邻接: 共享面或边
    CONNECTIVITY = "connectivity"  # 连通: 自由空间通道
    ACCESSIBILITY = "accessibility"  # 可达: 间接可达


@dataclass
class SCGEdge:
    """SCG 边。"""
    from_id: int
    to_id: int
    edge_type: EdgeType
    weight: float = 1.0  # 边权重 (用于路径搜索)
    confidence: float = 1.0  # 置信度

    @property
    def pair(self) -> Tuple[int, int]:
        """返回节点对 (排序后)。"""
        return (min(self.from_id, self.to_id), max(self.from_id, self.to_id))


@dataclass
class SCGConfig:
    """SCG 构建配置。"""
    # 邻接检测参数
    adjacency_threshold: float = 0.2  # 顶点距离阈值 (米)

    # 连通性检测参数
    connectivity_samples: int = 20  # 连通性检测采样数
    connectivity_threshold: float = 0.5  # 占据阈值

    # 回环检测参数
    loop_closure_threshold: float = 0.5  # 回环检测距离阈值 (米)
    loop_closure_volume_ratio: float = 0.8  # 体积比阈值

    # 边权重参数
    adjacency_weight: float = 1.0
    connectivity_weight: float = 1.5
    accessibility_weight: float = 2.0


# ══════════════════════════════════════════════════════════════════
#  SCG 构建器
# ══════════════════════════════════════════════════════════════════

class SCGBuilder:
    """
    空间连通图 (SCG) 构建器。

    用法:
        builder = SCGBuilder(config)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges(occupancy_grid, ...)
        path = builder.shortest_path(0, 5)
    """

    def __init__(self, config: SCGConfig):
        self.config = config

        # 节点存储 (使用 polyhedron_expansion.Polyhedron)
        self.nodes: Dict[int, any] = {}  # {poly_id: Polyhedron}

        # 边存储
        self.edges: List[SCGEdge] = []
        self.adjacency_map: Dict[int, List[SCGEdge]] = defaultdict(list)

        # 空间索引 (用于快速查询)
        self._kdtree: Optional[KDTree] = None
        self._kdtree_ids: List[int] = []

    # ── 节点管理 ──────────────────────────────────────────────

    def add_polyhedron(self, polyhedron) -> None:
        """
        添加多面体节点。

        Args:
            polyhedron: Polyhedron 实例
        """
        poly_id = polyhedron.poly_id

        # 回环检测: 检查是否与已有节点重复
        duplicate_id = self._detect_loop_closure(polyhedron)
        if duplicate_id is not None:
            logger.info(f"Loop closure detected: {poly_id} → {duplicate_id}")
            # 合并节点 (这里简化处理，直接跳过)
            return

        # 添加节点
        self.nodes[poly_id] = polyhedron

        # 重建空间索引
        self._rebuild_kdtree()

        logger.debug(f"Added polyhedron {poly_id}, total nodes: {len(self.nodes)}")

    def remove_polyhedron(self, poly_id: int) -> None:
        """
        移除多面体节点。

        Args:
            poly_id: 多面体 ID
        """
        if poly_id not in self.nodes:
            return

        # 移除节点
        del self.nodes[poly_id]

        # 移除相关边
        self.edges = [e for e in self.edges if e.from_id != poly_id and e.to_id != poly_id]
        self.adjacency_map.pop(poly_id, None)
        for neighbors in self.adjacency_map.values():
            neighbors[:] = [e for e in neighbors if e.from_id != poly_id and e.to_id != poly_id]

        # 重建空间索引
        self._rebuild_kdtree()

        logger.debug(f"Removed polyhedron {poly_id}, remaining nodes: {len(self.nodes)}")

    def get_polyhedron(self, poly_id: int):
        """获取多面体节点。"""
        return self.nodes.get(poly_id)

    def get_neighbors(self, poly_id: int) -> List[int]:
        """获取邻居节点 ID 列表。"""
        return [
            e.to_id if e.from_id == poly_id else e.from_id
            for e in self.adjacency_map.get(poly_id, [])
        ]

    # ── 边构建 ──────────────────────────────────────────────

    def build_edges(
        self,
        occupancy_grid: Optional[np.ndarray] = None,
        grid_resolution: Optional[float] = None,
        grid_origin: Optional[np.ndarray] = None,
    ) -> None:
        """
        构建所有拓扑边。

        Args:
            occupancy_grid: 占据栅格 (用于连通性检测)
            grid_resolution: 栅格分辨率
            grid_origin: 栅格原点
        """
        logger.info("Building SCG edges...")

        # 清空现有边
        self.edges = []
        self.adjacency_map = defaultdict(list)

        poly_ids = list(self.nodes.keys())
        num_pairs = len(poly_ids) * (len(poly_ids) - 1) // 2

        logger.info(f"Checking {num_pairs} polyhedron pairs...")

        # 遍历所有节点对
        for i, id1 in enumerate(poly_ids):
            for id2 in poly_ids[i+1:]:
                poly1 = self.nodes[id1]
                poly2 = self.nodes[id2]

                # 1. 检查邻接
                if self._check_adjacency(poly1, poly2):
                    self._add_edge(
                        id1, id2,
                        EdgeType.ADJACENCY,
                        self.config.adjacency_weight
                    )
                    continue

                # 2. 检查连通性
                if occupancy_grid is not None:
                    if self._check_connectivity(
                        poly1, poly2,
                        occupancy_grid, grid_resolution, grid_origin
                    ):
                        self._add_edge(
                            id1, id2,
                            EdgeType.CONNECTIVITY,
                            self.config.connectivity_weight
                        )
                        continue

        # 3. 添加可达边 (通过 BFS 检测)
        self._add_accessibility_edges()

        logger.info(
            f"SCG edges built: {len(self.edges)} edges "
            f"({self._count_edges_by_type()})"
        )

    def _check_adjacency(self, poly1, poly2) -> bool:
        """
        检查两个多面体是否邻接。

        方法: 检查顶点距离，如果有顶点非常接近，则邻接。
        """
        vertices1 = poly1.vertices
        vertices2 = poly2.vertices

        # 使用 KD-Tree 加速最近邻查询
        tree = KDTree(vertices2)

        for v1 in vertices1:
            dist, _ = tree.query(v1)
            if dist < self.config.adjacency_threshold:
                return True

        return False

    def _check_connectivity(
        self,
        poly1,
        poly2,
        occupancy_grid: np.ndarray,
        grid_resolution: float,
        grid_origin: np.ndarray,
    ) -> bool:
        """
        检查两个多面体是否连通。

        方法: 在两个多面体中心之间采样点，检查是否都在自由空间。
        """
        center1 = poly1.center
        center2 = poly2.center

        # 在两个中心之间采样
        num_samples = self.config.connectivity_samples
        for i in range(num_samples):
            t = i / (num_samples - 1)
            p = center1 * (1 - t) + center2 * t

            # 检查是否在自由空间
            if self._is_occupied(p, occupancy_grid, grid_resolution, grid_origin):
                return False  # 有障碍物阻挡

        return True  # 连通

    def _add_accessibility_edges(self) -> None:
        """
        添加可达边 (间接可达)。

        方法: 对于没有直接连接的节点对，如果存在路径，则添加可达边。
        """
        poly_ids = list(self.nodes.keys())

        for id1 in poly_ids:
            # 找到所有可达但没有直接连接的节点
            reachable = self._bfs_reachable(id1)
            direct_neighbors = set(self.get_neighbors(id1))

            for id2 in reachable:
                if id2 != id1 and id2 not in direct_neighbors:
                    # 计算路径长度作为权重
                    _, path = self.shortest_path(id1, id2)
                    weight = len(path) * self.config.accessibility_weight

                    self._add_edge(
                        id1, id2,
                        EdgeType.ACCESSIBILITY,
                        weight,
                        confidence=0.5  # 可达边置信度较低
                    )

    def _add_edge(
        self,
        from_id: int,
        to_id: int,
        edge_type: EdgeType,
        weight: float,
        confidence: float = 1.0,
    ) -> None:
        """添加边到 SCG。"""
        edge = SCGEdge(
            from_id=from_id,
            to_id=to_id,
            edge_type=edge_type,
            weight=weight,
            confidence=confidence,
        )

        self.edges.append(edge)
        self.adjacency_map[from_id].append(edge)
        self.adjacency_map[to_id].append(edge)

    # ── 路径搜索 ──────────────────────────────────────────────

    def shortest_path(
        self,
        from_id: int,
        to_id: int,
    ) -> Tuple[float, List[int]]:
        """
        Dijkstra 最短路径。

        Args:
            from_id: 起始节点 ID
            to_id: 目标节点 ID

        Returns:
            (cost, path) — cost 为总代价, path 为节点 ID 序列
        """
        if from_id not in self.nodes or to_id not in self.nodes:
            return float("inf"), []

        dist_map: Dict[int, float] = {from_id: 0.0}
        prev_map: Dict[int, int] = {}
        heap = [(0.0, from_id)]
        visited_set: Set[int] = set()

        while heap:
            cost, uid = heapq.heappop(heap)
            if uid in visited_set:
                continue
            visited_set.add(uid)

            if uid == to_id:
                # 重建路径
                path = []
                cur = to_id
                while cur in prev_map:
                    path.append(cur)
                    cur = prev_map[cur]
                path.append(from_id)
                return cost, list(reversed(path))

            # 遍历邻居
            for edge in self.adjacency_map.get(uid, []):
                vid = edge.to_id if edge.from_id == uid else edge.from_id
                if vid in visited_set:
                    continue

                edge_cost = edge.weight
                new_cost = cost + edge_cost

                if new_cost < dist_map.get(vid, float("inf")):
                    dist_map[vid] = new_cost
                    prev_map[vid] = uid
                    heapq.heappush(heap, (new_cost, vid))

        return float("inf"), []

    def _bfs_reachable(self, from_id: int) -> Set[int]:
        """BFS 查找所有可达节点。"""
        reachable = set()
        queue = [from_id]
        visited = {from_id}

        while queue:
            uid = queue.pop(0)
            reachable.add(uid)

            for vid in self.get_neighbors(uid):
                if vid not in visited:
                    visited.add(vid)
                    queue.append(vid)

        return reachable

    # ── 回环检测 ──────────────────────────────────────────────

    def _detect_loop_closure(self, polyhedron) -> Optional[int]:
        """
        检测回环 (是否与已有节点重复)。

        方法: 检查中心距离和体积比。

        Returns:
            重复节点的 ID 或 None
        """
        if len(self.nodes) == 0:
            return None

        # 使用 KD-Tree 查找最近节点
        if self._kdtree is None:
            return None

        dist, idx = self._kdtree.query(polyhedron.center)

        if dist < self.config.loop_closure_threshold:
            candidate_id = self._kdtree_ids[idx]
            candidate = self.nodes[candidate_id]

            # 检查体积比
            volume_ratio = min(
                polyhedron.volume / candidate.volume,
                candidate.volume / polyhedron.volume,
            )

            if volume_ratio > self.config.loop_closure_volume_ratio:
                return candidate_id

        return None

    def _rebuild_kdtree(self) -> None:
        """重建空间索引 (KD-Tree)。"""
        if len(self.nodes) == 0:
            self._kdtree = None
            self._kdtree_ids = []
            return

        centers = []
        ids = []

        for poly_id, poly in self.nodes.items():
            centers.append(poly.center)
            ids.append(poly_id)

        self._kdtree = KDTree(np.array(centers))
        self._kdtree_ids = ids

    # ── 工具方法 ──────────────────────────────────────────────

    @staticmethod
    def _is_occupied(
        point: np.ndarray,
        occupancy_grid: np.ndarray,
        resolution: float,
        origin: np.ndarray,
    ) -> bool:
        """检查点是否在障碍物内。"""
        grid_pos = ((point - origin) / resolution).astype(int)

        if np.any(grid_pos < 0) or np.any(grid_pos >= occupancy_grid.shape):
            return True  # 边界外视为障碍物

        return occupancy_grid[tuple(grid_pos)] > 0.5

    def _count_edges_by_type(self) -> str:
        """统计各类型边的数量。"""
        counts = defaultdict(int)
        for edge in self.edges:
            counts[edge.edge_type.value] += 1

        return ", ".join(f"{k}={v}" for k, v in counts.items())

    # ── 序列化 ──────────────────────────────────────────────

    def to_dict(self) -> dict:
        """导出为字典 (用于序列化)。"""
        return {
            "nodes": {
                poly_id: {
                    "center": poly.center.tolist(),
                    "volume": poly.volume,
                    "radius": poly.radius,
                }
                for poly_id, poly in self.nodes.items()
            },
            "edges": [
                {
                    "from_id": e.from_id,
                    "to_id": e.to_id,
                    "edge_type": e.edge_type.value,
                    "weight": e.weight,
                    "confidence": e.confidence,
                }
                for e in self.edges
            ],
        }

    # ── 可视化辅助 ──────────────────────────────────────────────

    def get_statistics(self) -> dict:
        """获取 SCG 统计信息。"""
        edge_counts = defaultdict(int)
        for edge in self.edges:
            edge_counts[edge.edge_type.value] += 1

        return {
            "num_nodes": len(self.nodes),
            "num_edges": len(self.edges),
            "edge_types": dict(edge_counts),
            "avg_degree": len(self.edges) * 2 / len(self.nodes) if self.nodes else 0,
        }
