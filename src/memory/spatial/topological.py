"""
拓扑记忆 — 记录机器人探索过的位置 + 关键帧。

参考论文:
  - VLMnav (2024):     拓扑图 + 关键帧 → VLM 选择下一个节点
  - L3MVN (ICRA 2024): Language-guided 拓扑图 + frontier 探索
  - CLIP-Nav (2023):   CLIP 特征的拓扑图, 文本查询匹配节点

核心功能:
  1. 每到一个新位置, 存储 (position, keyframe_CLIP_feature, scene_graph_snapshot)
  2. 探索时: 查找最近的未探索 frontier, 避免重访
  3. 回溯时: 找到之前看到目标的位置, 快速返回
  4. LLM 查询时: "回到有红色沙发的房间" → CLIP 匹配拓扑节点
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field

from core.msgs.numpy_compat import np
from core.utils.sanitize import safe_json_dumps

logger = logging.getLogger(__name__)


@dataclass
class TopoNode:
    """拓扑图节点 = 机器人到过的一个位置。"""
    node_id: int
    position: np.ndarray                # [x, y, z]
    timestamp: float                    # 首次到达时间
    visit_count: int = 1               # 访问次数
    last_visit: float = 0.0            # 最近访问时间

    # 感知快照 (到达时的观测)
    clip_feature: np.ndarray = field(default_factory=lambda: np.array([]))
    scene_snapshot: str = ""            # 到达时的场景图 JSON 子集
    visible_labels: list[str] = field(default_factory=list)

    # 图连接
    neighbors: list[int] = field(default_factory=list)  # 相邻节点 ID
    edge_distances: dict[int, float] = field(default_factory=dict)

    # 创新4: 房间关联
    room_id: int = -1                   # 所属房间 ID
    room_name: str = ""                 # 所属房间名称

    # VLingMem 区域摘要
    region_summary: str = ""
    explored: bool = False

    def to_dict(self) -> dict:
        d = {
            "node_id": self.node_id,
            "position": {
                "x": round(float(self.position[0]), 2),
                "y": round(float(self.position[1]), 2),
            },
            "visit_count": self.visit_count,
            "visible_labels": self.visible_labels,
            "neighbors": self.neighbors,
        }
        if self.room_id >= 0:
            d["room_id"] = self.room_id
            d["room_name"] = self.room_name
        return d


class TopologicalMemory:
    """
    拓扑记忆图。

    使用策略:
      - 每次机器人移动 > new_node_distance 米, 创建新节点
      - 节点之间按导航顺序连边
      - 每个节点存储 CLIP 全景特征 (来自感知) 和可见物体列表
      - 查询时支持: 文本匹配节点, 找最近未访问区域, 回溯路径
    """

    def __init__(
        self,
        new_node_distance: float = 2.0,    # 新建节点的最小移动距离
        max_nodes: int = 500,
    ):
        self.new_node_distance = new_node_distance
        self.max_nodes = max_nodes
        self._lock = threading.Lock()

        self._nodes: dict[int, TopoNode] = {}
        self._next_id = 0
        self._current_node_id: int = -1
        self._path_history: list[int] = []  # 访问过的节点 ID 序列
        self._last_position: np.ndarray | None = None

        # 创新4: 房间级覆盖追踪
        self._visited_rooms: dict[int, dict] = {}  # room_id → {name, first_visit, visit_count, node_ids}
        self._room_transition_history: list[tuple[int, int]] = []  # (from_room, to_room) 序列

        # FSR-VLN viewpoint edges: node_id → [(neighbor_id, weight), ...]
        self._viewpoint_edges: dict[int, list[tuple[int, float]]] = defaultdict(list)

        # CLIP 编码器（可选，由外部注入）
        self._clip_encoder = None

    @property
    def nodes(self) -> dict[int, TopoNode]:
        with self._lock:
            return dict(self._nodes)

    @property
    def current_node(self) -> TopoNode | None:
        with self._lock:
            return self._nodes.get(self._current_node_id)

    @property
    def visited_positions(self) -> list[np.ndarray]:
        """所有已访问位置。"""
        with self._lock:
            return [n.position.copy() for n in self._nodes.values()]

    def set_clip_encoder(self, clip_encoder) -> None:
        """注入 CLIP 编码器，用于 query_by_text 的语义匹配。"""
        self._clip_encoder = clip_encoder

    def update_position(
        self,
        position: np.ndarray,
        visible_labels: list[str] | None = None,
        scene_snapshot: str = "",
        clip_feature: np.ndarray | None = None,
        room_id: int = -1,
        room_name: str = "",
    ) -> TopoNode | None:
        """
        更新机器人位置, 必要时创建新节点。

        Args:
            position: [x, y, z]
            visible_labels: 当前可见物体标签列表
            scene_snapshot: 当前场景图子集
            clip_feature: 当前视角的 CLIP 全局特征
            room_id: 当前所在房间 ID (创新4)
            room_name: 当前所在房间名称 (创新4)

        Returns:
            新创建的节点 (如果创建了), 否则 None
        """
        pos = np.array(position[:2], dtype=np.float64)  # 2D

        with self._lock:
            return self._update_position_locked(pos, position, visible_labels,
                                                 scene_snapshot, clip_feature,
                                                 room_id, room_name)

    def _update_position_locked(
        self,
        pos: np.ndarray,
        position: np.ndarray,
        visible_labels: list[str] | None,
        scene_snapshot: str,
        clip_feature: np.ndarray | None,
        room_id: int,
        room_name: str,
    ) -> TopoNode | None:
        """update_position 内部实现（已持有 _lock）。"""
        # 检查是否需要创建新节点
        nearest, nearest_dist = self._find_nearest_node(pos)

        if nearest is not None and nearest_dist < self.new_node_distance:
            # 更新现有节点
            nearest.visit_count += 1
            nearest.last_visit = time.time()
            if visible_labels:
                existing = set(nearest.visible_labels)
                for lbl in visible_labels:
                    if lbl not in existing:
                        nearest.visible_labels.append(lbl)
            if scene_snapshot:
                nearest.scene_snapshot = scene_snapshot
            if clip_feature is not None and clip_feature.size > 0:
                nearest.clip_feature = clip_feature
            if room_id >= 0:
                nearest.room_id = room_id
                nearest.room_name = room_name

            if nearest.node_id != self._current_node_id:
                self._add_edge(self._current_node_id, nearest.node_id, nearest_dist)
                # 检测房间转换
                prev_node = self._nodes.get(self._current_node_id)
                if prev_node and prev_node.room_id >= 0 and nearest.room_id >= 0:
                    if prev_node.room_id != nearest.room_id:
                        self._room_transition_history.append(
                            (prev_node.room_id, nearest.room_id)
                        )
                self._current_node_id = nearest.node_id
                self._path_history.append(nearest.node_id)
                if len(self._path_history) > 1000:
                    self._path_history = self._path_history[-500:]

            # 更新房间覆盖
            if room_id >= 0:
                self._update_room_coverage(room_id, room_name, nearest.node_id)

            self._last_position = pos
            return None
        else:
            new_node = TopoNode(
                node_id=self._next_id,
                position=np.array(position[:3], dtype=np.float64),
                timestamp=time.time(),
                last_visit=time.time(),
                visible_labels=visible_labels or [],
                scene_snapshot=scene_snapshot,
                clip_feature=clip_feature if clip_feature is not None else np.array([]),
                room_id=room_id,
                room_name=room_name,
            )
            self._nodes[self._next_id] = new_node

            if self._current_node_id >= 0:
                dist = float(np.linalg.norm(
                    pos - self._nodes[self._current_node_id].position[:2]
                ))
                self._add_edge(self._current_node_id, self._next_id, dist)
                # 检测房间转换
                prev_node = self._nodes.get(self._current_node_id)
                if prev_node and prev_node.room_id >= 0 and room_id >= 0:
                    if prev_node.room_id != room_id:
                        self._room_transition_history.append(
                            (prev_node.room_id, room_id)
                        )

            self._current_node_id = self._next_id
            self._path_history.append(self._next_id)
            if len(self._path_history) > 1000:
                self._path_history = self._path_history[-500:]
            self._next_id += 1
            self._last_position = pos

            if room_id >= 0:
                self._update_room_coverage(room_id, room_name, new_node.node_id)

            self._prune_if_needed()
            self._connect_viewpoint_edges(new_node.node_id)
            return new_node

    def query_by_text(self, query: str, top_k: int = 3) -> list[TopoNode]:
        """
        文本查询匹配拓扑节点（CLIP-Nav 方案）。

        匹配策略（优先级降序）:
          1. CLIP 文本-图像语义相似度（若节点有 clip_feature 且 clip_encoder 可用）
          2. 标签关键词匹配
          3. 场景快照包含匹配

        Args:
            query: 查询文本，如 "有红色沙发的房间" / "entrance area"
            top_k: 返回最多 N 个节点

        Returns:
            匹配的节点列表（按相关度降序）
        """
        query_lower = query.lower()

        # CLIP 编码在锁外进行（可能耗时）
        query_clip_feat = None
        if self._clip_encoder is not None:
            try:
                q_feats = self._clip_encoder.encode_text([query])
                if q_feats is not None and q_feats.size > 0:
                    query_clip_feat = q_feats[0]
            except (ValueError, TypeError, AttributeError) as e:
                logger.debug("CLIP text encoding for topo query failed: %s", e)

        # 持锁快照节点数据，减少锁持有时间
        with self._lock:
            nodes_snapshot = dict(self._nodes)
            vp_edges_snapshot = {k: list(v) for k, v in self._viewpoint_edges.items()}

        scored_nodes: list[tuple[TopoNode, float]] = []

        # Phase 1: 计算每个节点的原始分数
        raw_scores: dict[int, float] = {}

        for node in nodes_snapshot.values():
            score = 0.0

            # ── 源1: CLIP 文本-图像相似度（最高权重）──
            if query_clip_feat is not None and node.clip_feature.size > 0:
                try:
                    import numpy as _np
                    nf = node.clip_feature
                    nf_norm = _np.linalg.norm(nf)
                    if nf_norm > 0:
                        clip_sim = float(_np.dot(query_clip_feat, nf / nf_norm))
                        # CLIP 相似度映射到 [0, 3]，与关键词分数量纲对齐
                        score += max(0.0, clip_sim) * 3.0
                except (ValueError, TypeError) as e:
                    logger.debug("CLIP node similarity failed: %s", e)

            # ── 源2: 标签关键词匹配 ──
            for label in node.visible_labels:
                if label.lower() in query_lower or query_lower in label.lower():
                    score += 1.0

            # ── 源3: 场景快照包含匹配 ──
            if node.scene_snapshot and query_lower in node.scene_snapshot.lower():
                score += 0.5

            if score > 0:
                raw_scores[node.node_id] = score

        # Phase 2: 1-hop viewpoint neighbor boost (FSR-VLN Fast Path 图检索)
        boosted_scores: dict[int, float] = {}
        for node_id, score in raw_scores.items():
            boosted = score
            for neighbor_id, edge_w in sorted(
                vp_edges_snapshot.get(node_id, []), key=lambda x: x[1], reverse=True
            ):
                if neighbor_id not in nodes_snapshot:
                    continue  # 已被 prune 的节点，跳过
                neighbor_score = raw_scores.get(neighbor_id, 0)
                boosted = max(boosted, score + 0.3 * edge_w * neighbor_score)
            boosted_scores[node_id] = boosted

        for node_id, score in boosted_scores.items():
            scored_nodes.append((nodes_snapshot[node_id], score))

        scored_nodes.sort(key=lambda x: x[1], reverse=True)
        return [node for node, _ in scored_nodes[:top_k]]

    def get_least_visited_direction(
        self,
        current_position: np.ndarray,
        min_distance: float = 3.0,
    ) -> np.ndarray | None:
        """
        获取最少探索的方向 (L3MVN 方案)。

        用于探索策略: 优先去访问次数最少的区域。

        Args:
            current_position: [x, y]
            min_distance: 最小距离, 避免返回太近的位置

        Returns:
            方向向量 [dx, dy] 或 None
        """
        if not self._nodes:
            return None

        # 计算每个方向扇区 (8 个方向) 的访问密度
        sector_count = 8
        sector_visits = [0] * sector_count
        sector_distances = [float('inf')] * sector_count

        pos = np.array(current_position[:2])

        for node in self._nodes.values():
            delta = node.position[:2] - pos
            dist = float(np.linalg.norm(delta))
            if dist < 0.1:
                continue

            angle = math.atan2(delta[1], delta[0])  # -pi to pi
            sector = int((angle + math.pi) / (2 * math.pi) * sector_count) % sector_count
            sector_visits[sector] += node.visit_count
            sector_distances[sector] = min(sector_distances[sector], dist)

        # 找访问最少的方向
        min_visits = min(sector_visits) if sector_visits else 0
        candidates = [
            i for i, v in enumerate(sector_visits)
            if v == min_visits
        ]

        if not candidates:
            return None

        # 在候选中选最远的方向 (鼓励探索新区域)
        best = max(candidates, key=lambda i: sector_distances[i])
        angle = (best / sector_count) * 2 * math.pi - math.pi

        return np.array([math.cos(angle), math.sin(angle)])

    def get_backtrack_position(self, steps_back: int = 1) -> np.ndarray | None:
        """
        回溯到之前的位置 (LOVON 的 BACKTRACK 动作)。

        Args:
            steps_back: 回退步数

        Returns:
            之前位置的 [x, y, z] 或 None
        """
        with self._lock:
            if len(self._path_history) <= steps_back:
                return None
            target_id = self._path_history[-(steps_back + 1)]
            node = self._nodes.get(target_id)
            return node.position.copy() if node else None

    @property
    def visited_room_ids(self) -> set:
        """已访问的房间 ID 集合。"""
        return set(self._visited_rooms.keys())

    def get_room_coverage(self) -> list[dict]:
        """获取房间级探索覆盖情况。"""
        coverage = []
        for rid, info in self._visited_rooms.items():
            coverage.append({
                "room_id": rid,
                "room_name": info.get("name", f"room_{rid}"),
                "visit_count": info.get("visit_count", 0),
                "node_count": len(info.get("node_ids", [])),
                "first_visit": info.get("first_visit", 0.0),
            })
        coverage.sort(key=lambda x: x["visit_count"], reverse=True)
        return coverage

    def get_room_transitions(self) -> list[tuple[int, int]]:
        """获取房间转换历史 (用于验证拓扑边)。"""
        return list(self._room_transition_history)

    def _update_room_coverage(
        self, room_id: int, room_name: str, node_id: int,
    ) -> None:
        """更新房间级覆盖信息。"""
        if room_id not in self._visited_rooms:
            self._visited_rooms[room_id] = {
                "name": room_name,
                "first_visit": time.time(),
                "visit_count": 0,
                "node_ids": set(),
            }
        info = self._visited_rooms[room_id]
        info["visit_count"] += 1
        info["node_ids"].add(node_id)
        if room_name and room_name != info.get("name", ""):
            info["name"] = room_name

    def update_region_summary(self, node_id: int, visible_labels: list[str], room_type: str = "") -> None:
        """更新指定节点的区域摘要（VLingMem 风格）。"""
        with self._lock:
            node = self._nodes.get(node_id)
        if node is None:
            return
        label_str = "、".join(list(dict.fromkeys(visible_labels))[:8]) if visible_labels else "无"
        room_str = f"推测为{room_type}" if room_type and room_type != "未知区域" else "区域类型未知"
        status = "已充分探索" if node.explored else "部分探索"
        node.region_summary = f"包含对象：{label_str}，{room_str}，{status}"

    def get_explored_summaries(self) -> list[str]:
        """返回所有已探索区域的摘要（VLingMem 风格）。"""
        with self._lock:
            nodes_snap = list(self._nodes.items())
        summaries = []
        for nid, node in nodes_snap:
            if node.region_summary:
                pos_str = f"({node.position[0]:.1f},{node.position[1]:.1f})" if hasattr(node, 'position') else ""
                summaries.append(f"节点{nid}{pos_str}：{node.region_summary}")
        return summaries

    def get_exploration_summary(self) -> dict:
        """
        生成探索摘要 (给 LLM 消费)。

        Returns:
            {
              "total_nodes": int,
              "total_visits": int,
              "coverage_estimate": float,
              "least_explored_direction": str,
              "recently_visited": [...]
            }
        """
        with self._lock:
            if not self._nodes:
                return {"total_nodes": 0, "total_visits": 0, "coverage_estimate": 0.0}
            nodes_snap = list(self._nodes.values())
            recent_ids = self._path_history[-5:]
            recent = [
                self._nodes[nid].to_dict()
                for nid in recent_ids
                if nid in self._nodes
            ]
            room_count = len(self._visited_rooms)
            trans_count = len(self._room_transition_history)

        total_visits = sum(n.visit_count for n in nodes_snap)

        # 覆盖范围估计 (凸包面积的近似)
        positions = np.array([n.position[:2] for n in nodes_snap])
        if len(positions) >= 3:
            from scipy.spatial import ConvexHull
            try:
                hull = ConvexHull(positions)
                coverage = hull.volume  # 2D 时 volume = area
            except (ValueError, ImportError) as e:
                logger.debug("ConvexHull coverage calculation failed: %s", e)
                coverage = 0.0
        else:
            coverage = 0.0

        return {
            "total_nodes": len(nodes_snap),
            "total_visits": total_visits,
            "coverage_area_m2": round(coverage, 1),
            "recently_visited": recent,
            "rooms_visited": room_count,
            "room_transitions": trans_count,
        }

    def get_graph_json(self) -> str:
        """导出拓扑图为 JSON。"""
        with self._lock:
            nodes_snap = list(self._nodes.values())
            path_snap = list(self._path_history[-20:])
        nodes = [n.to_dict() for n in nodes_snap]
        edges = []
        seen = set()
        for node in nodes_snap:
            for neighbor_id in node.neighbors:
                key = (min(node.node_id, neighbor_id), max(node.node_id, neighbor_id))
                if key not in seen:
                    seen.add(key)
                    edges.append({
                        "from": key[0],
                        "to": key[1],
                        "distance": round(node.edge_distances.get(neighbor_id, 0), 2),
                    })

        return safe_json_dumps({
            "nodes": nodes,
            "edges": edges,
            "path_history": path_snap,
        })

    # ── 持久化 ──

    def save_to_file(self, path: str) -> bool:
        """保存拓扑记忆到 JSON 文件。"""
        import os

        from core.utils.sanitize import safe_json_dump as _safe_dump

        with self._lock:
            nodes_copy = dict(self._nodes)
            rooms_copy = dict(self._visited_rooms)
            path_hist = list(self._path_history[-100:])
            next_id = self._next_id
            cur_id = self._current_node_id
            room_trans = list(self._room_transition_history[-50:])

        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            nodes_data = {}
            for nid, node in nodes_copy.items():
                nodes_data[str(nid)] = {
                    "position": node.position.tolist(),
                    "timestamp": node.timestamp,
                    "visit_count": node.visit_count,
                    "last_visit": node.last_visit,
                    "visible_labels": node.visible_labels,
                    "neighbors": node.neighbors,
                    "edge_distances": {str(k): v for k, v in node.edge_distances.items()},
                    "room_id": node.room_id,
                    "room_name": node.room_name,
                    "region_summary": node.region_summary,
                    "explored": node.explored,
                }

            visited_rooms_data = {}
            for rid, info in rooms_copy.items():
                visited_rooms_data[str(rid)] = {
                    "name": info.get("name", ""),
                    "first_visit": info.get("first_visit", 0.0),
                    "visit_count": info.get("visit_count", 0),
                    "node_ids": list(info.get("node_ids", set())),
                }

            data = {
                "version": "1.0",
                "nodes": nodes_data,
                "next_id": next_id,
                "current_node_id": cur_id,
                "path_history": path_hist,
                "visited_rooms": visited_rooms_data,
                "room_transitions": room_trans,
            }
            _safe_dump(data, path)
            logger.info("TopologicalMemory saved to %s (%d nodes)", path, len(nodes_copy))
            return True
        except Exception as e:
            logger.error("Failed to save TopologicalMemory to %s: %s", path, e)
            return False

    def load_from_file(self, path: str) -> bool:
        """从 JSON 文件恢复拓扑记忆。"""
        import json
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logger.warning("Failed to load TopologicalMemory from %s: %s", path, e)
            return False

        # 先解析数据（无需锁），再持锁写入
        new_nodes: dict[int, TopoNode] = {}
        for nid_str, ndata in data.get("nodes", {}).items():
            try:
                nid = int(nid_str)
            except (TypeError, ValueError):
                continue
            if not isinstance(ndata, dict):
                continue
            node = TopoNode(
                node_id=nid,
                position=np.array(ndata["position"]),
                timestamp=ndata.get("timestamp", 0.0),
                visit_count=ndata.get("visit_count", 1),
                last_visit=ndata.get("last_visit", 0.0),
                visible_labels=ndata.get("visible_labels", []),
                neighbors=ndata.get("neighbors", []),
                edge_distances={int(k): v for k, v in ndata.get("edge_distances", {}).items()},
                room_id=ndata.get("room_id", -1),
                room_name=ndata.get("room_name", ""),
                region_summary=ndata.get("region_summary", ""),
                explored=ndata.get("explored", False),
            )
            new_nodes[nid] = node

        new_rooms: dict[int, dict] = {}
        for rid_str, rdata in data.get("visited_rooms", {}).items():
            rid = int(rid_str)
            new_rooms[rid] = {
                "name": rdata.get("name", ""),
                "first_visit": rdata.get("first_visit", 0.0),
                "visit_count": rdata.get("visit_count", 0),
                "node_ids": set(rdata.get("node_ids", [])),
            }

        with self._lock:
            self._nodes.clear()
            self._nodes.update(new_nodes)
            self._path_history = data.get("path_history", [])
            self._visited_rooms.clear()
            self._visited_rooms.update(new_rooms)
            self._room_transition_history = [
                tuple(t) for t in data.get("room_transitions", [])
            ]
            self._next_id = data.get("next_id", max(self._nodes.keys(), default=-1) + 1)
            self._current_node_id = data.get("current_node_id", -1)
            self._viewpoint_edges.clear()

        logger.info("TopologicalMemory loaded from %s (%d nodes)", path, len(new_nodes))
        return True

    # ── Viewpoint edges (FSR-VLN) ──

    def _connect_viewpoint_edges(self, new_node_id: int) -> None:
        """为新节点建立 viewpoint 互联边 (FSR-VLN HMSG Viewpoint 层)。"""
        new_node = self._nodes[new_node_id]
        new_labels = set(new_node.visible_labels)

        for nid, node in self._nodes.items():
            if nid == new_node_id:
                continue
            dist = float(np.linalg.norm(new_node.position[:2] - node.position[:2]))
            other_labels = set(node.visible_labels)

            # 建边条件: 距离<4m 且有共同可见标签, 或距离<2m
            shared = new_labels & other_labels
            if dist < 4.0 and len(shared) > 0:
                pass  # 建边
            elif dist < 2.0:
                pass  # 物理相邻, 建边
            else:
                continue

            # 边权重 = Jaccard similarity, 最小 0.1
            union = new_labels | other_labels
            jaccard = len(shared) / len(union) if len(union) > 0 else 0.0
            weight = max(jaccard, 0.1)

            self._viewpoint_edges[new_node_id].append((nid, weight))
            self._viewpoint_edges[nid].append((new_node_id, weight))

        # Cap viewpoint edges per node to avoid unbounded growth
        _MAX_VP_EDGES = 20
        vpe = self._viewpoint_edges[new_node_id]
        if len(vpe) > _MAX_VP_EDGES:
            vpe.sort(key=lambda x: x[1], reverse=True)
            self._viewpoint_edges[new_node_id] = vpe[:_MAX_VP_EDGES]

    def get_viewpoint_neighbors(self, node_id: int) -> list[tuple[int, float]]:
        """返回 viewpoint 邻居 [(neighbor_id, weight), ...]，按 weight 降序。"""
        neighbors = self._viewpoint_edges.get(node_id, [])
        return sorted(neighbors, key=lambda x: x[1], reverse=True)

    # ── 内部方法 ──

    def _find_nearest_node(self, pos_2d: np.ndarray) -> tuple[TopoNode | None, float]:
        """找最近节点。"""
        best_node = None
        best_dist = float('inf')

        for node in self._nodes.values():
            dist = float(np.linalg.norm(pos_2d - node.position[:2]))
            if dist < best_dist:
                best_dist = dist
                best_node = node

        return best_node, best_dist

    def _add_edge(self, from_id: int, to_id: int, distance: float):
        """在两个节点间添加边。"""
        if from_id < 0 or to_id < 0:
            return
        if from_id not in self._nodes or to_id not in self._nodes:
            return

        a = self._nodes[from_id]
        b = self._nodes[to_id]

        if to_id not in a.neighbors:
            a.neighbors.append(to_id)
            a.edge_distances[to_id] = distance

        if from_id not in b.neighbors:
            b.neighbors.append(from_id)
            b.edge_distances[from_id] = distance

    def _prune_if_needed(self):
        """如果节点超过上限, 移除最旧最少访问的。"""
        while len(self._nodes) > self.max_nodes:
            # 找访问次数最少的 (排除当前节点)
            candidates = [nid for nid in self._nodes if nid != self._current_node_id]
            if not candidates:
                break  # Only current node remains, cannot prune
            worst_id = min(
                candidates,
                key=lambda nid: (self._nodes[nid].visit_count, self._nodes[nid].last_visit),
            )
            # 移除边引用
            node = self._nodes[worst_id]
            for neighbor_id in node.neighbors:
                if neighbor_id in self._nodes:
                    nb = self._nodes[neighbor_id]
                    if worst_id in nb.neighbors:
                        nb.neighbors.remove(worst_id)
                    nb.edge_distances.pop(worst_id, None)
            # 清理 viewpoint edges 中的悬挂引用
            self._viewpoint_edges.pop(worst_id, None)
            for nb_list in self._viewpoint_edges.values():
                nb_list[:] = [(nid, w) for nid, w in nb_list if nid != worst_id]
            del self._nodes[worst_id]
