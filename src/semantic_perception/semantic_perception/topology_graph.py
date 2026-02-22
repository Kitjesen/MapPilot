"""
拓扑语义图 (Topological Semantic Graph, TSG) — 房间级空间拓扑 + 语义联想 + 信息增益探索。

参考论文:
  - Hydra (RSS 2022): 层次3D场景图, Places→Rooms 社区检测, 实时拓扑连通
  - TopoNav (2025): 拓扑图作为空间记忆, 连接性+邻接性+语义
  - L3MVN (IROS 2023): LLM-guided 拓扑探索, frontier 评分
  - SG-Nav (NeurIPS 2024): 子图推理 + LLM 常识 + frontier 插值
  - TACS-Graphs (2025): 可通行性感知场景图, 一致性房间分割
  - Concept-Guided Exploration (2025): Room+Door 自治概念, 层次约束传播

核心创新 (创新5: Topology-Aware Information Gain Exploration):
  将房间拓扑图从纯粹的"连通性描述"升级为"探索决策引擎":
  1. 语义联想边: room_type → expected_objects (带概率)
  2. 前沿节点 (Frontier): 已知空间边界处的未探索方向
  3. 穿越记忆 (Traversal Memory): 记录机器人实际路径
  4. 信息增益评分: IG(node) = semantic_prior × novelty × reachability_decay
  5. 图上最短路径: Dijkstra 选择最优探索目标
"""

import heapq
import logging
import math
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  数据结构
# ══════════════════════════════════════════════════════════════════

@dataclass
class TopoNode:
    """拓扑图节点 (房间 或 前沿)。"""
    node_id: int
    node_type: str                      # "room" | "frontier"
    name: str
    center: np.ndarray                  # [x, y]
    room_type: str = "unknown"          # corridor, office, kitchen, ...
    semantic_labels: List[str] = field(default_factory=list)

    # 探索状态
    visited: bool = False
    visit_count: int = 0
    last_visited: float = 0.0           # timestamp
    objects_found: int = 0              # 进入该房间后发现的物体数

    # 前沿特有
    frontier_direction: Optional[np.ndarray] = None  # 前沿探索方向 [dx, dy]
    frontier_size: float = 0.0                       # 前沿宽度 (米)
    predicted_room_type: str = ""                     # 语义预测: 前沿那侧可能是什么房间

    # ═══ 几何信息 (USS-Nav 融合) ═══
    # 边界框 (AABB - Axis-Aligned Bounding Box)
    bounding_box: Optional[Dict[str, float]] = None
    # 格式: {"x_min": float, "x_max": float, "y_min": float, "y_max": float}

    # 凸包 (2D 多边形顶点)
    convex_hull: Optional[np.ndarray] = None
    # 格式: (N, 2) 数组，顺时针或逆时针排列的顶点

    # 可通行区域面积 (平方米)
    traversable_area: float = 0.0

    # 高度范围 (米)
    height_range: Optional[Dict[str, float]] = None
    # 格式: {"floor": float, "ceiling": float}

    # 几何质量指标
    geometry_confidence: float = 0.0
    # 0.0-1.0，表示几何信息的可靠性

    # 最后几何更新时间
    geometry_updated: float = 0.0

    def __hash__(self):
        return self.node_id

    def __eq__(self, other):
        return isinstance(other, TopoNode) and self.node_id == other.node_id


@dataclass
class TopoEdge:
    """拓扑图边 (房间间连通)。"""
    from_id: int
    to_id: int
    edge_type: str                      # "door" | "proximity" | "passage" | "frontier_link"
    distance: float = 0.0               # 欧氏距离
    traversal_count: int = 0            # 机器人实际穿越次数
    last_traversed: float = 0.0         # 最后穿越时间
    mediator_label: str = ""            # 连通介质标签
    mediator_pos: Optional[np.ndarray] = None
    confidence: float = 0.5             # 连通置信度

    @property
    def pair(self) -> Tuple[int, int]:
        return (min(self.from_id, self.to_id), max(self.from_id, self.to_id))


@dataclass
class ExplorationTarget:
    """探索目标推荐结果。"""
    node_id: int
    node_name: str
    node_type: str
    position: np.ndarray                # [x, y] 目标位置
    score: float                        # 综合评分
    semantic_score: float               # 语义先验分
    novelty_score: float                # 新颖度分
    reachability_score: float           # 可达性分
    information_gain: float             # 信息增益
    hops: int                           # 拓扑跳数
    path: List[int]                     # 拓扑路径 (node_id 序列)
    reasoning: str


# ══════════════════════════════════════════════════════════════════
#  拓扑语义图主类
# ══════════════════════════════════════════════════════════════════

# 探索衰减常数
_REACHABILITY_LAMBDA = 0.3         # 距离衰减系数
_NOVELTY_TIME_TAU = 120.0          # 新颖度时间衰减 (秒)
_FRONTIER_BASE_PRIOR = 0.4         # 前沿基础先验
_MIN_FRONTIER_DISTANCE = 1.5       # 前沿最小距离 (米)
_SEMANTIC_BOOST_FACTOR = 1.5       # 语义匹配提升因子
_VISITED_PENALTY = 0.1             # 已访问房间惩罚因子


class TopologySemGraph:
    """
    拓扑语义图 — 结合空间拓扑、语义联想和信息增益的探索决策引擎。

    核心能力:
      1. 维护房间+前沿的拓扑图
      2. 记录机器人穿越历史
      3. 计算每个节点的信息增益
      4. 选择最优探索目标 (Dijkstra + IG)
      5. 生成 LLM 可消费的拓扑摘要

    用法:
      tsg = TopologySemGraph()
      tsg.update_from_scene_graph(scene_graph_dict)
      tsg.record_robot_position(x, y)
      target = tsg.get_best_exploration_target("找灭火器", semantic_engine)
    """

    def __init__(self):
        self._nodes: Dict[int, TopoNode] = {}
        self._edges: List[TopoEdge] = []
        self._adjacency: Dict[int, List[TopoEdge]] = defaultdict(list)
        self._next_frontier_id = 10000   # 前沿ID从10000开始, 避免与room_id冲突

        # 穿越记忆
        self._traversal_history: List[Dict] = []
        self._current_room_id: int = -1
        self._robot_position: Optional[np.ndarray] = None
        self._robot_trajectory: List[np.ndarray] = []

        # 语义联想缓存
        self._room_object_expectations: Dict[str, Dict[str, float]] = {}

        # ═══ 几何提取器 (USS-Nav 融合) ═══
        self._geometry_extractor = None

    # ── 图构建 ──────────────────────────────────────────────────

    def set_geometry_extractor(self, geometry_extractor) -> None:
        """
        设置几何提取器 (连接到 Tomogram)。

        Args:
            geometry_extractor: GeometryExtractor 实例
        """
        self._geometry_extractor = geometry_extractor
        logger.info("Geometry extractor connected to topology graph")

    def update_from_scene_graph(self, sg: Dict) -> None:
        """
        从场景图字典同步拓扑图。

        保留已有的穿越记忆和访问状态, 只更新节点属性和边。
        """
        rooms = sg.get("rooms", [])
        topology_edges = sg.get("topology_edges", [])

        existing_visits = {
            nid: (n.visited, n.visit_count, n.last_visited, n.objects_found)
            for nid, n in self._nodes.items()
            if n.node_type == "room"
        }

        new_room_ids = set()
        for room in rooms:
            rid = room.get("room_id", -1)
            if rid < 0:
                continue
            new_room_ids.add(rid)

            center = np.array([
                room.get("center", {}).get("x", 0.0),
                room.get("center", {}).get("y", 0.0),
            ])
            labels = room.get("semantic_labels", [])
            name = room.get("name", f"room_{rid}")

            node = TopoNode(
                node_id=rid,
                node_type="room",
                name=name,
                center=center,
                room_type=self._infer_room_type(name),
                semantic_labels=labels[:12],
            )

            # ═══ 几何信息提取 (USS-Nav 融合) ═══
            if self._geometry_extractor is not None:
                try:
                    geometry = self._geometry_extractor.extract_room_geometry(
                        room_center=center,
                        search_radius=5.0,
                        cost_threshold=0.5,
                    )
                    node.bounding_box = geometry["bounding_box"]
                    node.convex_hull = geometry["convex_hull"]
                    node.traversable_area = geometry["traversable_area"]
                    node.height_range = geometry["height_range"]
                    node.geometry_confidence = geometry["confidence"]
                    node.geometry_updated = time.time()

                    logger.debug(
                        f"Extracted geometry for {name}: "
                        f"area={geometry['traversable_area']:.2f}m², "
                        f"confidence={geometry['confidence']:.2f}"
                    )
                except Exception as e:
                    logger.warning(f"Failed to extract geometry for room {rid}: {e}")

            if rid in existing_visits:
                node.visited, node.visit_count, node.last_visited, node.objects_found = (
                    existing_visits[rid]
                )

            self._nodes[rid] = node

        stale_rooms = [
            nid for nid, n in self._nodes.items()
            if n.node_type == "room" and nid not in new_room_ids
        ]
        for nid in stale_rooms:
            self._remove_node(nid)

        self._rebuild_edges(topology_edges)

    def add_frontier(
        self,
        position: np.ndarray,
        direction: np.ndarray,
        nearest_room_id: int,
        frontier_size: float = 2.0,
        predicted_room_type: str = "",
    ) -> int:
        """
        添加前沿节点并连接到最近的房间。

        前沿代表已知空间边界处的未探索方向。

        Returns:
            新前沿节点的 ID
        """
        fid = self._next_frontier_id
        self._next_frontier_id += 1

        dir_norm = np.linalg.norm(direction)
        if dir_norm > 0:
            direction = direction / dir_norm

        node = TopoNode(
            node_id=fid,
            node_type="frontier",
            name=f"frontier_{fid}",
            center=np.asarray(position[:2], dtype=np.float64),
            frontier_direction=direction,
            frontier_size=frontier_size,
            predicted_room_type=predicted_room_type,
        )
        self._nodes[fid] = node

        if nearest_room_id in self._nodes:
            room_node = self._nodes[nearest_room_id]
            dist = float(np.linalg.norm(node.center - room_node.center))
            edge = TopoEdge(
                from_id=nearest_room_id,
                to_id=fid,
                edge_type="frontier_link",
                distance=dist,
                confidence=0.3,
            )
            self._edges.append(edge)
            self._adjacency[nearest_room_id].append(edge)
            self._adjacency[fid].append(edge)

        return fid

    def update_frontiers_from_costmap(
        self,
        frontier_points: List[np.ndarray],
        frontier_sizes: Optional[List[float]] = None,
    ) -> List[int]:
        """
        从 costmap 前沿点更新前沿节点。

        清除旧前沿, 添加新前沿并连接到最近房间。

        Args:
            frontier_points: 前沿中心点列表 [[x, y], ...]
            frontier_sizes: 前沿宽度列表 (可选)

        Returns:
            新前沿ID列表
        """
        old_frontier_ids = [
            nid for nid, n in self._nodes.items()
            if n.node_type == "frontier"
        ]
        for fid in old_frontier_ids:
            self._remove_node(fid)

        sizes = frontier_sizes or [2.0] * len(frontier_points)
        new_ids = []

        rooms = [n for n in self._nodes.values() if n.node_type == "room"]
        if not rooms:
            return new_ids

        for i, fp in enumerate(frontier_points):
            fp = np.asarray(fp[:2], dtype=np.float64)

            nearest_room = min(rooms, key=lambda r: float(np.linalg.norm(r.center - fp)))
            direction = fp - nearest_room.center
            dist = float(np.linalg.norm(direction))

            if dist < _MIN_FRONTIER_DISTANCE:
                continue

            fid = self.add_frontier(
                position=fp,
                direction=direction,
                nearest_room_id=nearest_room.node_id,
                frontier_size=sizes[i],
            )
            new_ids.append(fid)

        return new_ids

    # ── 穿越记忆 ──────────────────────────────────────────────

    def record_robot_position(self, x: float, y: float) -> Optional[int]:
        """
        记录机器人当前位置, 自动检测房间切换。

        Returns:
            当前房间ID (如果在已知房间内), 否则 None
        """
        pos = np.array([x, y], dtype=np.float64)
        self._robot_position = pos

        if len(self._robot_trajectory) == 0 or float(
            np.linalg.norm(pos - self._robot_trajectory[-1])
        ) > 0.3:
            self._robot_trajectory.append(pos)
            if len(self._robot_trajectory) > 500:
                self._robot_trajectory = self._robot_trajectory[-300:]

        rooms = [n for n in self._nodes.values() if n.node_type == "room"]
        if not rooms:
            return None

        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - pos)))
        dist = float(np.linalg.norm(nearest.center - pos))

        room_radius = 4.0
        if dist > room_radius:
            return None

        new_room_id = nearest.node_id
        if new_room_id != self._current_room_id:
            old_room_id = self._current_room_id
            self._current_room_id = new_room_id

            now = time.time()
            nearest.visited = True
            nearest.visit_count += 1
            nearest.last_visited = now

            if old_room_id >= 0 and old_room_id in self._nodes:
                self._record_traversal(old_room_id, new_room_id, now)

            logger.debug(
                "Room transition: %s → %s",
                self._nodes.get(old_room_id, TopoNode(-1, "", "", np.zeros(2))).name,
                nearest.name,
            )

        return new_room_id

    def _record_traversal(self, from_id: int, to_id: int, timestamp: float) -> None:
        """记录房间间穿越。"""
        self._traversal_history.append({
            "from": from_id,
            "to": to_id,
            "time": timestamp,
        })
        if len(self._traversal_history) > 200:
            self._traversal_history = self._traversal_history[-100:]

        pair = (min(from_id, to_id), max(from_id, to_id))
        for edge in self._edges:
            if edge.pair == pair:
                edge.traversal_count += 1
                edge.last_traversed = timestamp
                edge.confidence = min(1.0, edge.confidence + 0.2)
                return

        # 穿越了但没有对应边 → 新增 traversal 边
        dist = 0.0
        n1 = self._nodes.get(from_id)
        n2 = self._nodes.get(to_id)
        if n1 and n2:
            dist = float(np.linalg.norm(n1.center - n2.center))

        edge = TopoEdge(
            from_id=from_id,
            to_id=to_id,
            edge_type="traversal",
            distance=dist,
            traversal_count=1,
            last_traversed=timestamp,
            confidence=0.8,
        )
        self._edges.append(edge)
        self._adjacency[from_id].append(edge)
        self._adjacency[to_id].append(edge)

    # ── 信息增益计算 ─────────────────────────────────────────

    def compute_information_gain(
        self,
        node_id: int,
        target_instruction: str,
        semantic_engine=None,
    ) -> float:
        """
        计算单个节点的信息增益 (TopoNav + L3MVN 融合)。

        IG(node) = semantic_prior × novelty × uncertainty_reduction

        - semantic_prior: 语义先验 — 目标物体在此类房间中的概率
        - novelty: 新颖度 — 未访问或长时间未访问 = 高
        - uncertainty_reduction: 访问该节点能减少多少全局不确定性
        """
        node = self._nodes.get(node_id)
        if node is None:
            return 0.0

        # 1. 语义先验
        semantic_score = _FRONTIER_BASE_PRIOR
        if semantic_engine is not None:
            if node.node_type == "room":
                room_dict = {
                    "room_id": node.node_id,
                    "name": node.name,
                    "semantic_labels": node.semantic_labels,
                }
                priors = semantic_engine.score_rooms_for_target(
                    target_instruction, [room_dict],
                )
                if priors:
                    semantic_score = priors[0].prior_score
            elif node.node_type == "frontier":
                if node.predicted_room_type:
                    predictions = semantic_engine.predict_target_rooms(target_instruction)
                    for rt, prob, _ in predictions:
                        if rt == node.predicted_room_type:
                            semantic_score = prob * _SEMANTIC_BOOST_FACTOR
                            break

        # 2. 新颖度
        novelty = 1.0
        if node.visited:
            dt = time.time() - node.last_visited if node.last_visited > 0 else 0
            visit_decay = math.exp(-node.visit_count * 0.5)
            time_recovery = 1.0 - math.exp(-dt / _NOVELTY_TIME_TAU)
            novelty = _VISITED_PENALTY + (1.0 - _VISITED_PENALTY) * visit_decay * time_recovery

        # 3. 不确定性缩减
        uncertainty = 1.0
        if node.node_type == "frontier":
            uncertainty = 1.2 + 0.1 * min(node.frontier_size, 5.0)
        elif node.node_type == "room" and not node.visited:
            uncertainty = 1.0 + 0.05 * max(0, 5 - len(node.semantic_labels))

        return semantic_score * novelty * uncertainty

    def compute_all_information_gains(
        self,
        target_instruction: str,
        semantic_engine=None,
    ) -> Dict[int, float]:
        """计算所有节点的信息增益。"""
        return {
            nid: self.compute_information_gain(nid, target_instruction, semantic_engine)
            for nid in self._nodes
        }

    # ── 图上最短路径 ─────────────────────────────────────────

    def shortest_path(self, from_id: int, to_id: int) -> Tuple[float, List[int]]:
        """
        Dijkstra 最短路径 (距离加权)。

        Returns:
            (cost, path) — cost 为总距离, path 为 node_id 序列
        """
        if from_id not in self._nodes or to_id not in self._nodes:
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
                path = []
                cur = to_id
                while cur in prev_map:
                    path.append(cur)
                    cur = prev_map[cur]
                path.append(from_id)
                return cost, list(reversed(path))

            for edge in self._adjacency.get(uid, []):
                vid = edge.to_id if edge.from_id == uid else edge.from_id
                if vid in visited_set:
                    continue
                edge_cost = max(edge.distance, 0.1)
                # 低置信度边增加代价
                if edge.confidence < 0.5:
                    edge_cost *= 2.0
                new_cost = cost + edge_cost
                if new_cost < dist_map.get(vid, float("inf")):
                    dist_map[vid] = new_cost
                    prev_map[vid] = uid
                    heapq.heappush(heap, (new_cost, vid))

        return float("inf"), []

    def hop_distances(self, from_id: int) -> Dict[int, int]:
        """BFS 计算从 from_id 到所有节点的跳数。"""
        if from_id not in self._nodes:
            return {}

        hops: Dict[int, int] = {from_id: 0}
        queue = [from_id]
        while queue:
            uid = queue.pop(0)
            for edge in self._adjacency.get(uid, []):
                vid = edge.to_id if edge.from_id == uid else edge.from_id
                if vid not in hops:
                    hops[vid] = hops[uid] + 1
                    queue.append(vid)
        return hops

    # ── 探索目标选择 (核心算法) ───────────────────────────────

    def get_best_exploration_target(
        self,
        target_instruction: str,
        semantic_engine=None,
        top_k: int = 3,
    ) -> List[ExplorationTarget]:
        """
        选择最优探索目标 — 融合语义先验 + 拓扑可达 + 信息增益。

        算法 (Algorithm 2: Topology-Aware Information Gain Exploration):
        ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        Input: 目标指令 I, 拓扑图 TSG, 语义引擎 SE, 当前位置 p_robot
        Output: 最优探索目标列表 (按综合评分降序)

        1: for each node n in TSG do
        2:     IG(n) ← compute_information_gain(n, I, SE)
        3:     cost(n) ← shortest_path(current_room, n).cost
        4:     reachability(n) ← 1 / (1 + λ · cost)
        5:     score(n) ← IG(n) × reachability(n)
        6: end for
        7: return top_k(sort_by_score(nodes))

        相比已有工作的优势:
        - L3MVN: 只有 LLM 评分 frontiers, 无拓扑路径优化
        - TopoNav: 有拓扑但无语义先验知识库
        - SG-Nav: 子图评分插值到 frontier, 但不考虑穿越历史
        """
        if not self._nodes:
            return []

        source = self._current_room_id
        if source < 0:
            rooms = [n for n in self._nodes.values() if n.node_type == "room"]
            if rooms and self._robot_position is not None:
                source = min(
                    rooms, key=lambda r: float(np.linalg.norm(r.center - self._robot_position))
                ).node_id
            elif rooms:
                source = rooms[0].node_id
            else:
                return []

        all_ig = self.compute_all_information_gains(target_instruction, semantic_engine)
        hop_map = self.hop_distances(source)

        candidates: List[ExplorationTarget] = []

        for nid, node in self._nodes.items():
            if nid == source:
                continue

            ig = all_ig.get(nid, 0.0)
            if ig < 0.01:
                continue

            cost, path = self.shortest_path(source, nid)
            hops = hop_map.get(nid, -1)

            if cost == float("inf"):
                reachability = 0.1
            else:
                reachability = 1.0 / (1.0 + _REACHABILITY_LAMBDA * cost)

            # 语义先验分 (从 IG 中分离, 用于诊断)
            semantic_s = _FRONTIER_BASE_PRIOR
            if semantic_engine:
                if node.node_type == "room":
                    rd = {"room_id": nid, "name": node.name, "semantic_labels": node.semantic_labels}
                    ps = semantic_engine.score_rooms_for_target(target_instruction, [rd])
                    if ps:
                        semantic_s = ps[0].prior_score
                elif node.predicted_room_type:
                    preds = semantic_engine.predict_target_rooms(target_instruction)
                    for rt, prob, _ in preds:
                        if rt == node.predicted_room_type:
                            semantic_s = prob
                            break

            novelty_s = 1.0
            if node.visited:
                novelty_s = _VISITED_PENALTY

            score = ig * reachability

            reasoning_parts = [f"IG={ig:.2f}", f"reach={reachability:.2f}"]
            if node.node_type == "frontier":
                reasoning_parts.append(f"frontier→{node.predicted_room_type or '?'}")
            if node.visited:
                reasoning_parts.append(f"visited(×{node.visit_count})")
            if hops >= 0:
                reasoning_parts.append(f"{hops}hops")

            candidates.append(ExplorationTarget(
                node_id=nid,
                node_name=node.name,
                node_type=node.node_type,
                position=node.center.copy(),
                score=score,
                semantic_score=semantic_s,
                novelty_score=novelty_s,
                reachability_score=reachability,
                information_gain=ig,
                hops=hops,
                path=path,
                reasoning="; ".join(reasoning_parts),
            ))

        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates[:top_k]

    # ── LLM Prompt 生成 ──────────────────────────────────────

    def to_prompt_context(self, language: str = "zh") -> str:
        """
        生成 LLM 可消费的拓扑摘要。

        输出格式:
          rooms: R0(corridor,✓), R1(office,?)
          connections: R0 ↔ R1(door), R0 ↔ F10001(frontier→kitchen?)
          visited: R0(2次), R2(1次)
          unexplored frontiers: F10001(东侧,可能是kitchen), F10002(南侧)
        """
        rooms = sorted(
            [n for n in self._nodes.values() if n.node_type == "room"],
            key=lambda n: n.node_id,
        )
        frontiers = sorted(
            [n for n in self._nodes.values() if n.node_type == "frontier"],
            key=lambda n: n.node_id,
        )

        parts = []

        if language == "zh":
            # 房间列表
            room_strs = []
            for r in rooms:
                status = "[已探索]" if r.visited else "[未探索]"
                room_strs.append(f"R{r.node_id}({r.name},{status})")
            if room_strs:
                parts.append(f"已知房间: {', '.join(room_strs)}")

            # 连通关系
            edge_strs = []
            for e in self._edges:
                n1 = self._nodes.get(e.from_id)
                n2 = self._nodes.get(e.to_id)
                if n1 and n2:
                    traversed = f",穿越{e.traversal_count}次" if e.traversal_count > 0 else ""
                    edge_strs.append(
                        f"{n1.name} ↔ {n2.name}({e.edge_type}{traversed})"
                    )
            if edge_strs:
                parts.append(f"连通关系: {'; '.join(edge_strs[:10])}")

            # 前沿
            if frontiers:
                frontier_strs = []
                for f in frontiers:
                    pred = f"→可能是{f.predicted_room_type}" if f.predicted_room_type else ""
                    dx, dy = (0, 0)
                    if f.frontier_direction is not None:
                        dx, dy = f.frontier_direction[0], f.frontier_direction[1]
                    direction = self._direction_name_zh(dx, dy)
                    frontier_strs.append(f"{f.name}({direction}{pred})")
                parts.append(f"未探索前沿: {', '.join(frontier_strs)}")

            # 探索记忆
            visited_rooms = [r for r in rooms if r.visited]
            if visited_rooms:
                visited_strs = [f"{r.name}({r.visit_count}次)" for r in visited_rooms]
                parts.append(f"已访问: {', '.join(visited_strs)}")

        else:
            room_strs = []
            for r in rooms:
                status = "visited" if r.visited else "unvisited"
                room_strs.append(f"R{r.node_id}({r.name},{status})")
            if room_strs:
                parts.append(f"Rooms: {', '.join(room_strs)}")

            edge_strs = []
            for e in self._edges:
                n1 = self._nodes.get(e.from_id)
                n2 = self._nodes.get(e.to_id)
                if n1 and n2:
                    t = f",traversed:{e.traversal_count}" if e.traversal_count > 0 else ""
                    edge_strs.append(f"{n1.name} ↔ {n2.name}({e.edge_type}{t})")
            if edge_strs:
                parts.append(f"Connections: {'; '.join(edge_strs[:10])}")

            if frontiers:
                frontier_strs = []
                for f in frontiers:
                    pred = f"→{f.predicted_room_type}" if f.predicted_room_type else ""
                    dx, dy = (0, 0)
                    if f.frontier_direction is not None:
                        dx, dy = f.frontier_direction[0], f.frontier_direction[1]
                    d = self._direction_name_en(dx, dy)
                    frontier_strs.append(f"{f.name}({d}{pred})")
                parts.append(f"Frontiers: {', '.join(frontier_strs)}")

            visited_rooms = [r for r in rooms if r.visited]
            if visited_rooms:
                visited_strs = [f"{r.name}(×{r.visit_count})" for r in visited_rooms]
                parts.append(f"Visited: {', '.join(visited_strs)}")

        return "\n".join(parts)

    # ── 序列化 ──────────────────────────────────────────────

    def to_dict(self) -> Dict:
        """导出为可 JSON 序列化的字典。"""
        nodes_list = []
        for n in self._nodes.values():
            d = {
                "node_id": n.node_id,
                "node_type": n.node_type,
                "name": n.name,
                "center": {"x": round(float(n.center[0]), 2), "y": round(float(n.center[1]), 2)},
                "room_type": n.room_type,
                "visited": n.visited,
                "visit_count": n.visit_count,
            }
            if n.node_type == "frontier":
                d["predicted_room_type"] = n.predicted_room_type
                d["frontier_size"] = round(n.frontier_size, 2)
                if n.frontier_direction is not None:
                    d["frontier_direction"] = {
                        "dx": round(float(n.frontier_direction[0]), 2),
                        "dy": round(float(n.frontier_direction[1]), 2),
                    }

            # ═══ 几何信息序列化 ═══
            if n.bounding_box:
                d["bounding_box"] = {
                    k: round(float(v), 2) for k, v in n.bounding_box.items()
                }
            if n.convex_hull is not None:
                d["convex_hull"] = [[round(float(x), 2), round(float(y), 2)]
                                    for x, y in n.convex_hull]
            if n.traversable_area > 0:
                d["traversable_area"] = round(n.traversable_area, 2)
            if n.height_range:
                d["height_range"] = {
                    k: round(float(v), 2) for k, v in n.height_range.items()
                }
            if n.geometry_confidence > 0:
                d["geometry_confidence"] = round(n.geometry_confidence, 2)

            nodes_list.append(d)

        edges_list = []
        for e in self._edges:
            edges_list.append({
                "from_id": e.from_id,
                "to_id": e.to_id,
                "edge_type": e.edge_type,
                "distance": round(e.distance, 2),
                "traversal_count": e.traversal_count,
                "confidence": round(e.confidence, 2),
                "mediator": e.mediator_label,
            })

        return {
            "nodes": nodes_list,
            "edges": edges_list,
            "current_room_id": self._current_room_id,
            "traversal_count": len(self._traversal_history),
        }

    @classmethod
    def from_dict(cls, data: Dict) -> "TopologySemGraph":
        """从字典恢复拓扑图。"""
        tsg = cls()
        for nd in data.get("nodes", []):
            center = np.array([nd["center"]["x"], nd["center"]["y"]])
            node = TopoNode(
                node_id=nd["node_id"],
                node_type=nd["node_type"],
                name=nd["name"],
                center=center,
                room_type=nd.get("room_type", "unknown"),
                visited=nd.get("visited", False),
                visit_count=nd.get("visit_count", 0),
            )
            if nd["node_type"] == "frontier":
                node.predicted_room_type = nd.get("predicted_room_type", "")
                node.frontier_size = nd.get("frontier_size", 2.0)
                fd = nd.get("frontier_direction")
                if fd:
                    node.frontier_direction = np.array([fd["dx"], fd["dy"]])

            # ═══ 几何信息反序列化 ═══
            if "bounding_box" in nd:
                node.bounding_box = nd["bounding_box"]
            if "convex_hull" in nd:
                node.convex_hull = np.array(nd["convex_hull"], dtype=np.float64)
            if "traversable_area" in nd:
                node.traversable_area = nd["traversable_area"]
            if "height_range" in nd:
                node.height_range = nd["height_range"]
            if "geometry_confidence" in nd:
                node.geometry_confidence = nd["geometry_confidence"]

            tsg._nodes[nd["node_id"]] = node

        for ed in data.get("edges", []):
            edge = TopoEdge(
                from_id=ed["from_id"],
                to_id=ed["to_id"],
                edge_type=ed["edge_type"],
                distance=ed.get("distance", 0.0),
                traversal_count=ed.get("traversal_count", 0),
                confidence=ed.get("confidence", 0.5),
                mediator_label=ed.get("mediator", ""),
            )
            tsg._edges.append(edge)
            tsg._adjacency[edge.from_id].append(edge)
            tsg._adjacency[edge.to_id].append(edge)

        tsg._current_room_id = data.get("current_room_id", -1)
        return tsg

    # ── 查询 ──────────────────────────────────────────────────

    @property
    def rooms(self) -> List[TopoNode]:
        return [n for n in self._nodes.values() if n.node_type == "room"]

    @property
    def frontiers(self) -> List[TopoNode]:
        return [n for n in self._nodes.values() if n.node_type == "frontier"]

    @property
    def current_room_id(self) -> int:
        return self._current_room_id

    @property
    def visited_room_ids(self) -> Set[int]:
        return {n.node_id for n in self._nodes.values() if n.node_type == "room" and n.visited}

    @property
    def unvisited_room_ids(self) -> Set[int]:
        return {
            n.node_id for n in self._nodes.values()
            if n.node_type == "room" and not n.visited
        }

    def get_node(self, node_id: int) -> Optional[TopoNode]:
        return self._nodes.get(node_id)

    def get_neighbors(self, node_id: int) -> List[int]:
        """获取邻居节点ID。"""
        neighbors = []
        for edge in self._adjacency.get(node_id, []):
            vid = edge.to_id if edge.from_id == node_id else edge.from_id
            neighbors.append(vid)
        return neighbors

    def get_room_by_position(self, x: float, y: float, radius: float = 4.0) -> Optional[int]:
        """根据位置查找最近的房间节点。"""
        pos = np.array([x, y])
        rooms = self.rooms
        if not rooms:
            return None
        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - pos)))
        if float(np.linalg.norm(nearest.center - pos)) <= radius:
            return nearest.node_id
        return None

    # ── 内部方法 ──────────────────────────────────────────────

    def _remove_node(self, node_id: int) -> None:
        """移除节点及其关联边。"""
        self._nodes.pop(node_id, None)
        self._edges = [e for e in self._edges if e.from_id != node_id and e.to_id != node_id]
        self._adjacency.pop(node_id, None)
        for nid in list(self._adjacency.keys()):
            self._adjacency[nid] = [
                e for e in self._adjacency[nid]
                if e.from_id != node_id and e.to_id != node_id
            ]

    def _rebuild_edges(self, topology_edges: List[Dict]) -> None:
        """从 scene graph 的 topology_edges 重建边 (保留穿越记忆)。"""
        old_traversals: Dict[Tuple[int, int], Tuple[int, float, float]] = {}
        for e in self._edges:
            if e.traversal_count > 0:
                old_traversals[e.pair] = (e.traversal_count, e.last_traversed, e.confidence)

        self._edges = []
        self._adjacency = defaultdict(list)

        for te in topology_edges:
            from_id = te.get("from_room", -1)
            to_id = te.get("to_room", -1)
            if from_id < 0 or to_id < 0:
                continue
            if from_id not in self._nodes or to_id not in self._nodes:
                continue

            mp = te.get("mediator_pos")
            mediator_np = None
            if mp:
                mediator_np = np.array([mp.get("x", 0), mp.get("y", 0)])

            edge = TopoEdge(
                from_id=from_id,
                to_id=to_id,
                edge_type=te.get("type", "proximity"),
                distance=te.get("distance", 0.0),
                mediator_label=te.get("mediator", ""),
                mediator_pos=mediator_np,
                confidence=0.5,
            )

            pair = edge.pair
            if pair in old_traversals:
                tc, lt, conf = old_traversals[pair]
                edge.traversal_count = tc
                edge.last_traversed = lt
                edge.confidence = max(conf, edge.confidence)

            self._edges.append(edge)
            self._adjacency[from_id].append(edge)
            self._adjacency[to_id].append(edge)

        for pair, (tc, lt, conf) in old_traversals.items():
            already = any(e.pair == pair for e in self._edges)
            if not already and pair[0] in self._nodes and pair[1] in self._nodes:
                edge = TopoEdge(
                    from_id=pair[0],
                    to_id=pair[1],
                    edge_type="traversal",
                    distance=float(np.linalg.norm(
                        self._nodes[pair[0]].center - self._nodes[pair[1]].center
                    )),
                    traversal_count=tc,
                    last_traversed=lt,
                    confidence=conf,
                )
                self._edges.append(edge)
                self._adjacency[pair[0]].append(edge)
                self._adjacency[pair[1]].append(edge)

    @staticmethod
    def _infer_room_type(name: str) -> str:
        """从房间名推断标准类型。"""
        n = name.lower()
        mapping = {
            "corridor": ["corridor", "hallway", "走廊", "过道"],
            "office": ["office", "办公"],
            "kitchen": ["kitchen", "厨房", "茶水"],
            "meeting_room": ["meeting", "会议"],
            "bathroom": ["bathroom", "卫生间", "洗手间"],
            "stairwell": ["stair", "楼梯"],
            "lobby": ["lobby", "大厅", "前台"],
            "storage": ["storage", "储物", "仓库"],
            "lab": ["lab", "实验"],
            "classroom": ["classroom", "教室"],
        }
        for room_type, keywords in mapping.items():
            if any(kw in n for kw in keywords):
                return room_type
        return "unknown"

    @staticmethod
    def _direction_name_zh(dx: float, dy: float) -> str:
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return "未知方向"
        angle = math.atan2(dy, dx) * 180 / math.pi
        if -22.5 <= angle < 22.5:
            return "东侧"
        elif 22.5 <= angle < 67.5:
            return "东北"
        elif 67.5 <= angle < 112.5:
            return "北侧"
        elif 112.5 <= angle < 157.5:
            return "西北"
        elif angle >= 157.5 or angle < -157.5:
            return "西侧"
        elif -157.5 <= angle < -112.5:
            return "西南"
        elif -112.5 <= angle < -67.5:
            return "南侧"
        else:
            return "东南"

    @staticmethod
    def _direction_name_en(dx: float, dy: float) -> str:
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            return "unknown"
        angle = math.atan2(dy, dx) * 180 / math.pi
        if -22.5 <= angle < 22.5:
            return "east"
        elif 22.5 <= angle < 67.5:
            return "NE"
        elif 67.5 <= angle < 112.5:
            return "north"
        elif 112.5 <= angle < 157.5:
            return "NW"
        elif angle >= 157.5 or angle < -157.5:
            return "west"
        elif -157.5 <= angle < -112.5:
            return "SW"
        elif -112.5 <= angle < -67.5:
            return "south"
        else:
            return "SE"
