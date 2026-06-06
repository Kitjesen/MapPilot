from __future__ import annotations

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
  1. 语义联想边: room_type → expected_objects (带概率)
  2. 前沿节点 (Frontier): 已知空间边界处的未探索方向
  3. 穿越记忆 (Traversal Memory): 记录机器人实际路径
  4. 信息增益评分: IG(node) = semantic_prior × novelty × reachability_decay
  5. 图上最短路径: Dijkstra 选择最优探索目标

数据类、常量、纯函数辅助见 topo_types.py。
"""

import logging
import time
from collections import defaultdict

from core.msgs.numpy_compat import np

from memory.spatial.topology_types import (
    _FRONTIER_BASE_PRIOR,
    _MIN_FRONTIER_DISTANCE,
    # 常量
    _REACHABILITY_LAMBDA,
    _VISITED_PENALTY,
    ExplorationTarget,
    TopoEdge,
    # 数据类 (向后兼容重新导出)
    TopoNode,
    compute_node_information_gain,
    graph_hop_distances,
    graph_shortest_path,
    # 纯函数
    infer_room_type,
    tsg_nodes_edges_from_dict,
    tsg_to_dict,
    tsg_to_prompt_context,
)

logger = logging.getLogger(__name__)


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
        self._nodes: dict[int, TopoNode] = {}
        self._edges: list[TopoEdge] = []
        self._adjacency: dict[int, list[TopoEdge]] = defaultdict(list)
        self._next_frontier_id = 10000

        self._traversal_history: list[dict] = []
        self._current_room_id: int = -1
        self._robot_position: np.ndarray | None = None
        self._robot_trajectory: list[np.ndarray] = []

        self._room_object_expectations: dict[str, dict[str, float]] = {}
        self._geometry_extractor = None

    # ── 图构建 ──────────────────────────────────────────────────

    def set_geometry_extractor(self, geometry_extractor) -> None:
        """设置几何提取器 (连接到 Tomogram)。"""
        self._geometry_extractor = geometry_extractor
        logger.info("Geometry extractor connected to topology graph")

    def update_from_scene_graph(self, sg: dict) -> None:
        """从场景图字典同步拓扑图，保留穿越记忆和访问状态。"""
        rooms = sg.get("rooms", [])
        topology_edges = sg.get("topology_edges", [])

        existing_visits = {
            nid: (n.visited, n.visit_count, n.last_visited, n.objects_found)
            for nid, n in self._nodes.items()
            if n.node_type == "room"
        }

        new_room_ids: set[int] = set()
        for room in rooms:
            rid = room.get("room_id", -1)
            if rid < 0:
                continue
            new_room_ids.add(rid)
            center = np.array([
                room.get("center", {}).get("x", 0.0),
                room.get("center", {}).get("y", 0.0),
            ])
            name = room.get("name", f"room_{rid}")
            node = TopoNode(
                node_id=rid,
                node_type="room",
                name=name,
                center=center,
                room_type=infer_room_type(name),
                semantic_labels=room.get("semantic_labels", [])[:12],
            )
            if self._geometry_extractor is not None:
                try:
                    geometry = self._geometry_extractor.extract_room_geometry(
                        room_center=center, search_radius=5.0, cost_threshold=0.5,
                    )
                    node.bounding_box = geometry["bounding_box"]
                    node.convex_hull = geometry["convex_hull"]
                    node.traversable_area = geometry["traversable_area"]
                    node.height_range = geometry["height_range"]
                    node.geometry_confidence = geometry["confidence"]
                    node.geometry_updated = time.time()
                    logger.debug(
                        "Extracted geometry for %s: area=%.2fm², confidence=%.2f",
                        name, geometry["traversable_area"], geometry["confidence"],
                    )
                except Exception as e:
                    logger.warning("Failed to extract geometry for room %d: %s", rid, e)
            if rid in existing_visits:
                node.visited, node.visit_count, node.last_visited, node.objects_found = (
                    existing_visits[rid]
                )
            self._nodes[rid] = node

        for nid in [nid for nid, n in self._nodes.items()
                    if n.node_type == "room" and nid not in new_room_ids]:
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
        """添加前沿节点并连接到最近的房间。返回新前沿节点的 ID。"""
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
                from_id=nearest_room_id, to_id=fid,
                edge_type="frontier_link", distance=dist, confidence=0.3,
            )
            self._edges.append(edge)
            self._adjacency[nearest_room_id].append(edge)
            self._adjacency[fid].append(edge)

        return fid

    def update_frontiers_from_costmap(
        self,
        frontier_points: list[np.ndarray],
        frontier_sizes: list[float] | None = None,
    ) -> list[int]:
        """从 costmap 前沿点清除旧前沿并添加新前沿。"""
        for fid in [nid for nid, n in self._nodes.items() if n.node_type == "frontier"]:
            self._remove_node(fid)

        sizes = frontier_sizes or [2.0] * len(frontier_points)
        rooms = [n for n in self._nodes.values() if n.node_type == "room"]
        if not rooms:
            return []

        new_ids = []
        for i, fp in enumerate(frontier_points):
            fp = np.asarray(fp[:2], dtype=np.float64)
            nearest_room = min(rooms, key=lambda r: float(np.linalg.norm(r.center - fp)))
            direction = fp - nearest_room.center
            if float(np.linalg.norm(direction)) < _MIN_FRONTIER_DISTANCE:
                continue
            new_ids.append(self.add_frontier(
                position=fp, direction=direction,
                nearest_room_id=nearest_room.node_id, frontier_size=sizes[i],
            ))
        return new_ids

    # ── 穿越记忆 ──────────────────────────────────────────────

    def record_robot_position(self, x: float, y: float) -> int | None:
        """记录机器人当前位置, 自动检测房间切换。"""
        pos = np.array([x, y], dtype=np.float64)
        self._robot_position = pos

        if (not self._robot_trajectory
                or float(np.linalg.norm(pos - self._robot_trajectory[-1])) > 0.3):
            self._robot_trajectory.append(pos)
            if len(self._robot_trajectory) > 500:
                self._robot_trajectory = self._robot_trajectory[-300:]

        rooms = [n for n in self._nodes.values() if n.node_type == "room"]
        if not rooms:
            return None

        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - pos)))
        if float(np.linalg.norm(nearest.center - pos)) > 4.0:
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
        self._traversal_history.append({"from": from_id, "to": to_id, "time": timestamp})
        if len(self._traversal_history) > 200:
            self._traversal_history = self._traversal_history[-100:]

        pair = (min(from_id, to_id), max(from_id, to_id))
        for edge in self._edges:
            if edge.pair == pair:
                edge.traversal_count += 1
                edge.last_traversed = timestamp
                edge.confidence = min(1.0, edge.confidence + 0.2)
                return

        n1, n2 = self._nodes.get(from_id), self._nodes.get(to_id)
        dist = float(np.linalg.norm(n1.center - n2.center)) if n1 and n2 else 0.0
        edge = TopoEdge(
            from_id=from_id, to_id=to_id, edge_type="traversal",
            distance=dist, traversal_count=1, last_traversed=timestamp, confidence=0.8,
        )
        self._edges.append(edge)
        self._adjacency[from_id].append(edge)
        self._adjacency[to_id].append(edge)

    # ── 信息增益 + 路径 ──────────────────────────────────────

    def compute_information_gain(
        self, node_id: int, target_instruction: str, semantic_engine=None,
    ) -> float:
        """计算单个节点的信息增益。"""
        node = self._nodes.get(node_id)
        if node is None:
            return 0.0
        return compute_node_information_gain(node, target_instruction, semantic_engine)

    def compute_all_information_gains(
        self, target_instruction: str, semantic_engine=None,
    ) -> dict[int, float]:
        """计算所有节点的信息增益。"""
        return {
            nid: compute_node_information_gain(n, target_instruction, semantic_engine)
            for nid, n in self._nodes.items()
        }

    def shortest_path(self, from_id: int, to_id: int) -> tuple[float, list[int]]:
        """Dijkstra 最短路径 (距离加权)。"""
        return graph_shortest_path(from_id, to_id, self._nodes, self._adjacency)

    def hop_distances(self, from_id: int) -> dict[int, int]:
        """BFS 计算从 from_id 到所有节点的跳数。"""
        return graph_hop_distances(from_id, self._nodes, self._adjacency)

    # ── 探索目标选择 ─────────────────────────────────────────

    def get_best_exploration_target(
        self,
        target_instruction: str,
        semantic_engine=None,
        top_k: int = 3,
    ) -> list[ExplorationTarget]:
        """
        选择最优探索目标 — 融合语义先验 + 拓扑可达 + 信息增益。

        score(n) = IG(n) × reachability(n), reachability = 1/(1 + λ·cost)
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
        candidates: list[ExplorationTarget] = []

        for nid, node in self._nodes.items():
            if nid == source:
                continue
            ig = all_ig.get(nid, 0.0)
            if ig < 0.01:
                continue
            cost, path = self.shortest_path(source, nid)
            hops = hop_map.get(nid, -1)
            reachability = 0.1 if cost == float("inf") else 1.0 / (1.0 + _REACHABILITY_LAMBDA * cost)

            semantic_s = _FRONTIER_BASE_PRIOR
            if semantic_engine:
                if node.node_type == "room":
                    rd = {"room_id": nid, "name": node.name, "semantic_labels": node.semantic_labels}
                    ps = semantic_engine.score_rooms_for_target(target_instruction, [rd])
                    if ps:
                        semantic_s = ps[0].prior_score
                elif node.predicted_room_type:
                    for rt, prob, _ in semantic_engine.predict_target_rooms(target_instruction):
                        if rt == node.predicted_room_type:
                            semantic_s = prob
                            break

            reasoning_parts = [f"IG={ig:.2f}", f"reach={reachability:.2f}"]
            if node.node_type == "frontier":
                reasoning_parts.append(f"frontier→{node.predicted_room_type or '?'}")
            if node.visited:
                reasoning_parts.append(f"visited(×{node.visit_count})")
            if hops >= 0:
                reasoning_parts.append(f"{hops}hops")

            candidates.append(ExplorationTarget(
                node_id=nid, node_name=node.name, node_type=node.node_type,
                position=node.center.copy(), score=ig * reachability,
                semantic_score=semantic_s,
                novelty_score=_VISITED_PENALTY if node.visited else 1.0,
                reachability_score=reachability,
                information_gain=ig, hops=hops, path=path,
                reasoning="; ".join(reasoning_parts),
            ))

        candidates.sort(key=lambda c: c.score, reverse=True)
        return candidates[:top_k]

    # ── LLM Prompt + 序列化 ──────────────────────────────────

    def to_prompt_context(self, language: str = "zh") -> str:
        """生成 LLM 可消费的拓扑摘要。"""
        return tsg_to_prompt_context(self._nodes, self._edges, language)

    def to_dict(self) -> dict:
        """导出为可 JSON 序列化的字典。"""
        return tsg_to_dict(self._nodes, self._edges, self._current_room_id, self._traversal_history)

    @classmethod
    def from_dict(cls, data: dict) -> "TopologySemGraph":
        """从字典恢复拓扑图。"""
        tsg = cls()
        tsg._nodes, tsg._edges, tsg._adjacency, tsg._current_room_id = (
            tsg_nodes_edges_from_dict(data)
        )
        return tsg

    def save_to_file(self, path: str) -> bool:
        """保存拓扑图到 JSON 文件。"""
        import json as _json
        import os
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                _json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)
            logger.info("TopologySemGraph saved to %s (%d nodes, %d edges)",
                        path, len(self._nodes), len(self._edges))
            return True
        except Exception as e:
            logger.error("Failed to save TopologySemGraph to %s: %s", path, e)
            return False

    @classmethod
    def load_from_file(cls, path: str) -> "TopologySemGraph | None":
        """从 JSON 文件恢复拓扑图。"""
        import json as _json
        try:
            with open(path, encoding='utf-8') as f:
                data = _json.load(f)
            tsg = cls.from_dict(data)
            logger.info("TopologySemGraph loaded from %s (%d nodes, %d edges)",
                        path, len(tsg._nodes), len(tsg._edges))
            return tsg
        except (FileNotFoundError, _json.JSONDecodeError) as e:
            logger.warning("Failed to load TopologySemGraph from %s: %s", path, e)
            return None

    # ── 查询 ──────────────────────────────────────────────────

    @property
    def rooms(self) -> list[TopoNode]:
        return [n for n in self._nodes.values() if n.node_type == "room"]

    @property
    def frontiers(self) -> list[TopoNode]:
        return [n for n in self._nodes.values() if n.node_type == "frontier"]

    @property
    def current_room_id(self) -> int:
        return self._current_room_id

    @property
    def visited_room_ids(self) -> set[int]:
        return {n.node_id for n in self._nodes.values() if n.node_type == "room" and n.visited}

    @property
    def unvisited_room_ids(self) -> set[int]:
        return {n.node_id for n in self._nodes.values()
                if n.node_type == "room" and not n.visited}

    def get_node(self, node_id: int) -> TopoNode | None:
        return self._nodes.get(node_id)

    def get_neighbors(self, node_id: int) -> list[int]:
        """获取邻居节点ID。"""
        return [
            e.to_id if e.from_id == node_id else e.from_id
            for e in self._adjacency.get(node_id, [])
        ]

    def get_room_by_position(self, x: float, y: float, radius: float = 4.0) -> int | None:
        """根据位置查找最近的房间节点。"""
        pos = np.array([x, y])
        rooms = self.rooms
        if not rooms:
            return None
        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - pos)))
        return nearest.node_id if float(np.linalg.norm(nearest.center - pos)) <= radius else None

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

    def _rebuild_edges(self, topology_edges: list[dict]) -> None:
        """从 scene graph 的 topology_edges 重建边 (保留穿越记忆)。"""
        old_traversals: dict[tuple[int, int], tuple[int, float, float]] = {}
        for e in self._edges:
            if e.traversal_count > 0:
                old_traversals[e.pair] = (e.traversal_count, e.last_traversed, e.confidence)

        self._edges = []
        self._adjacency = defaultdict(list)

        for te in topology_edges:
            from_id, to_id = te.get("from_room", -1), te.get("to_room", -1)
            if from_id < 0 or to_id < 0:
                continue
            if from_id not in self._nodes or to_id not in self._nodes:
                continue
            mp = te.get("mediator_pos")
            mediator_np = np.array([mp.get("x", 0), mp.get("y", 0)]) if mp else None
            edge = TopoEdge(
                from_id=from_id, to_id=to_id,
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
            if not any(e.pair == pair for e in self._edges):
                if pair[0] in self._nodes and pair[1] in self._nodes:
                    edge = TopoEdge(
                        from_id=pair[0], to_id=pair[1], edge_type="traversal",
                        distance=float(np.linalg.norm(
                            self._nodes[pair[0]].center - self._nodes[pair[1]].center
                        )),
                        traversal_count=tc, last_traversed=lt, confidence=conf,
                    )
                    self._edges.append(edge)
                    self._adjacency[pair[0]].append(edge)
                    self._adjacency[pair[1]].append(edge)
