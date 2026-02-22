"""
Leiden 区域分割 — 在 SCG 上进行社区检测。

功能:
  使用 Leiden 算法对空间连通图进行社区检测，将多面体节点分组为语义区域。

优势:
  - 比 DBSCAN 更适合图结构数据
  - 考虑拓扑连通性，而非仅空间距离
  - 支持多层次分割

参考:
  - Leiden Algorithm (Traag et al., 2019)
  - USS-Nav: 使用 Leiden 进行区域分割
  - Hydra: 层次场景图的社区检测

依赖:
  - python-igraph (或 networkx + leidenalg)
"""

import logging
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class Region:
    """语义区域。"""
    region_id: int
    node_ids: List[int]  # 包含的多面体节点 ID
    center: np.ndarray   # 区域中心
    volume: float        # 总体积
    region_type: str = "unknown"  # 区域类型 (corridor, room, ...)


@dataclass
class LeidenConfig:
    """Leiden 分割配置。"""
    resolution: float = 1.0  # 分辨率参数 (越大越多社区)
    min_region_size: int = 2  # 最小区域大小 (节点数)
    use_weights: bool = True  # 是否使用边权重


# ══════════════════════════════════════════════════════════════════
#  Leiden 区域分割器
# ══════════════════════════════════════════════════════════════════

class LeidenSegmenter:
    """
    Leiden 区域分割器。

    用法:
        segmenter = LeidenSegmenter(config)
        regions = segmenter.segment(scg_builder)
    """

    def __init__(self, config: LeidenConfig):
        self.config = config

        # 检查依赖
        self._check_dependencies()

    def _check_dependencies(self) -> None:
        """检查依赖库。"""
        try:
            import igraph
            self.use_igraph = True
            logger.info("Using igraph for Leiden clustering")
        except ImportError:
            try:
                import networkx
                import leidenalg
                self.use_igraph = False
                logger.info("Using networkx + leidenalg for Leiden clustering")
            except ImportError:
                logger.warning(
                    "Neither igraph nor (networkx + leidenalg) found. "
                    "Falling back to simple connected components."
                )
                self.use_igraph = None

    def segment(self, scg_builder) -> List[Region]:
        """
        对 SCG 进行 Leiden 社区检测。

        Args:
            scg_builder: SCGBuilder 实例

        Returns:
            区域列表
        """
        logger.info("Starting Leiden segmentation...")

        if len(scg_builder.nodes) == 0:
            logger.warning("No nodes in SCG, skipping segmentation")
            return []

        # 根据可用库选择实现
        if self.use_igraph:
            communities = self._segment_igraph(scg_builder)
        elif self.use_igraph is False:
            communities = self._segment_networkx(scg_builder)
        else:
            communities = self._segment_fallback(scg_builder)

        # 过滤小区域
        communities = [
            c for c in communities
            if len(c) >= self.config.min_region_size
        ]

        # 构建 Region 对象
        regions = []
        for i, node_ids in enumerate(communities):
            region = self._create_region(i, node_ids, scg_builder)
            regions.append(region)

        logger.info(f"Segmentation complete: {len(regions)} regions")
        return regions

    def _segment_igraph(self, scg_builder) -> List[List[int]]:
        """使用 igraph 实现 Leiden 聚类。"""
        import igraph as ig

        # 构建 igraph 图
        g = ig.Graph()

        # 添加节点
        node_ids = list(scg_builder.nodes.keys())
        g.add_vertices(len(node_ids))

        # 节点 ID 映射
        id_to_idx = {nid: i for i, nid in enumerate(node_ids)}

        # 添加边
        edges = []
        weights = []

        for edge in scg_builder.edges:
            idx1 = id_to_idx[edge.from_id]
            idx2 = id_to_idx[edge.to_id]
            edges.append((idx1, idx2))

            if self.config.use_weights:
                # 权重取倒数 (距离越小，权重越大)
                weights.append(1.0 / edge.weight if edge.weight > 0 else 1.0)
            else:
                weights.append(1.0)

        g.add_edges(edges)

        # Leiden 聚类
        if self.config.use_weights:
            partition = g.community_leiden(
                weights=weights,
                resolution_parameter=self.config.resolution,
            )
        else:
            partition = g.community_leiden(
                resolution_parameter=self.config.resolution,
            )

        # 转换为节点 ID 列表
        communities = []
        for community in partition:
            node_ids_in_community = [node_ids[idx] for idx in community]
            communities.append(node_ids_in_community)

        return communities

    def _segment_networkx(self, scg_builder) -> List[List[int]]:
        """使用 networkx + leidenalg 实现 Leiden 聚类。"""
        import networkx as nx
        import leidenalg
        import igraph as ig

        # 构建 networkx 图
        G = nx.Graph()

        # 添加节点
        G.add_nodes_from(scg_builder.nodes.keys())

        # 添加边
        for edge in scg_builder.edges:
            weight = 1.0 / edge.weight if self.config.use_weights and edge.weight > 0 else 1.0
            G.add_edge(edge.from_id, edge.to_id, weight=weight)

        # 转换为 igraph (leidenalg 需要)
        g = ig.Graph.from_networkx(G)

        # Leiden 聚类
        if self.config.use_weights:
            weights = [e['weight'] for e in g.es]
            partition = leidenalg.find_partition(
                g,
                leidenalg.RBConfigurationVertexPartition,
                weights=weights,
                resolution_parameter=self.config.resolution,
            )
        else:
            partition = leidenalg.find_partition(
                g,
                leidenalg.RBConfigurationVertexPartition,
                resolution_parameter=self.config.resolution,
            )

        # 转换为节点 ID 列表
        node_ids = list(G.nodes())
        communities = []
        for community in partition:
            node_ids_in_community = [node_ids[idx] for idx in community]
            communities.append(node_ids_in_community)

        return communities

    def _segment_fallback(self, scg_builder) -> List[List[int]]:
        """
        回退方案: 简单的连通分量检测。

        当 igraph 和 leidenalg 都不可用时使用。
        """
        logger.warning("Using fallback: connected components (not Leiden)")

        visited = set()
        communities = []

        for node_id in scg_builder.nodes.keys():
            if node_id in visited:
                continue

            # BFS 查找连通分量
            component = []
            queue = [node_id]
            visited.add(node_id)

            while queue:
                uid = queue.pop(0)
                component.append(uid)

                for vid in scg_builder.get_neighbors(uid):
                    if vid not in visited:
                        visited.add(vid)
                        queue.append(vid)

            communities.append(component)

        return communities

    def _create_region(
        self,
        region_id: int,
        node_ids: List[int],
        scg_builder,
    ) -> Region:
        """
        创建 Region 对象。

        Args:
            region_id: 区域 ID
            node_ids: 包含的节点 ID 列表
            scg_builder: SCGBuilder 实例

        Returns:
            Region 实例
        """
        # 计算区域中心 (所有节点中心的平均)
        centers = []
        total_volume = 0.0

        for nid in node_ids:
            poly = scg_builder.get_polyhedron(nid)
            if poly:
                centers.append(poly.center)
                total_volume += poly.volume

        center = np.mean(centers, axis=0) if centers else np.zeros(3)

        # 推断区域类型 (基于形状和大小)
        region_type = self._infer_region_type(node_ids, scg_builder)

        return Region(
            region_id=region_id,
            node_ids=node_ids,
            center=center,
            volume=total_volume,
            region_type=region_type,
        )

    def _infer_region_type(
        self,
        node_ids: List[int],
        scg_builder,
    ) -> str:
        """
        推断区域类型。

        启发式规则:
        - 节点数少 (1-2): 可能是小房间
        - 节点数多 (>5): 可能是走廊
        - 细长形状: 走廊
        - 紧凑形状: 房间
        """
        num_nodes = len(node_ids)

        if num_nodes <= 2:
            return "small_room"
        elif num_nodes >= 5:
            # 检查形状: 计算节点中心的分布
            centers = []
            for nid in node_ids:
                poly = scg_builder.get_polyhedron(nid)
                if poly:
                    centers.append(poly.center)

            if len(centers) >= 3:
                centers = np.array(centers)
                # 计算主成分分析 (PCA)
                cov = np.cov(centers.T)
                eigenvalues = np.linalg.eigvalsh(cov)

                # 如果最大特征值远大于其他，说明是细长形状 (走廊)
                if eigenvalues[-1] > 3 * eigenvalues[-2]:
                    return "corridor"

        return "room"


# ══════════════════════════════════════════════════════════════════
#  对比工具
# ══════════════════════════════════════════════════════════════════

def compare_segmentation(
    leiden_regions: List[Region],
    dbscan_clusters: List[List[int]],
) -> dict:
    """
    对比 Leiden 和 DBSCAN 的分割结果。

    Args:
        leiden_regions: Leiden 分割结果
        dbscan_clusters: DBSCAN 聚类结果 (节点 ID 列表的列表)

    Returns:
        对比统计
    """
    leiden_clusters = [r.node_ids for r in leiden_regions]

    # 计算 Adjusted Rand Index (ARI)
    # 这里简化处理，只统计基本信息
    stats = {
        "leiden_num_regions": len(leiden_clusters),
        "dbscan_num_clusters": len(dbscan_clusters),
        "leiden_avg_size": np.mean([len(c) for c in leiden_clusters]) if leiden_clusters else 0,
        "dbscan_avg_size": np.mean([len(c) for c in dbscan_clusters]) if dbscan_clusters else 0,
    }

    return stats
