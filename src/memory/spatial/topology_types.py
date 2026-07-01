"""
拓扑图数据类型 + 常量 + 纯函数辅助 — 供 topology_graph.py 和其他模块共享。

从 topology_graph.py 中提取, 保持向后兼容。
"""

from __future__ import annotations

import heapq
import logging
import math
import time as _time_mod
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any

from runtime.msgs.numpy_compat import np
logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════
#  常量
# ══════════════════════════════════════════════════════════════════

_REACHABILITY_LAMBDA = 0.3
_NOVELTY_TIME_TAU = 120.0
_FRONTIER_BASE_PRIOR = 0.4
_MIN_FRONTIER_DISTANCE = 1.5
_SEMANTIC_BOOST_FACTOR = 1.5
_VISITED_PENALTY = 0.1


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
    room_type: str = "unknown"
    semantic_labels: list[str] = field(default_factory=list)

    visited: bool = False
    visit_count: int = 0
    last_visited: float = 0.0
    objects_found: int = 0

    frontier_direction: np.ndarray | None = None
    frontier_size: float = 0.0
    predicted_room_type: str = ""

    bounding_box: dict[str, float] | None = None
    convex_hull: np.ndarray | None = None
    traversable_area: float = 0.0
    height_range: dict[str, float] | None = None
    geometry_confidence: float = 0.0
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
    edge_type: str
    distance: float = 0.0
    traversal_count: int = 0
    last_traversed: float = 0.0
    mediator_label: str = ""
    mediator_pos: np.ndarray | None = None
    confidence: float = 0.5

    @property
    def pair(self) -> tuple[int, int]:
        return (min(self.from_id, self.to_id), max(self.from_id, self.to_id))


@dataclass
class ExplorationTarget:
    """探索目标推荐结果。"""
    node_id: int
    node_name: str
    node_type: str
    position: np.ndarray
    score: float
    semantic_score: float
    novelty_score: float
    reachability_score: float
    information_gain: float
    hops: int
    path: list[int]
    reasoning: str


# ══════════════════════════════════════════════════════════════════
#  纯函数辅助 (从 TopologySemGraph 提取, 无实例状态依赖)
# ══════════════════════════════════════════════════════════════════

def infer_room_type(name: str) -> str:
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


def direction_name_zh(dx: float, dy: float) -> str:
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


def direction_name_en(dx: float, dy: float) -> str:
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


def graph_shortest_path(
    from_id: int,
    to_id: int,
    nodes: dict[int, TopoNode],
    adjacency: dict[int, list[TopoEdge]],
) -> tuple[float, list[int]]:
    """
    Dijkstra 最短路径 (距离加权)。

    Returns:
        (cost, path) — cost 为总距离, path 为 node_id 序列
    """
    if from_id not in nodes or to_id not in nodes:
        return float("inf"), []

    dist_map: dict[int, float] = {from_id: 0.0}
    prev_map: dict[int, int] = {}
    heap = [(0.0, from_id)]
    visited_set: set[int] = set()

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

        for edge in adjacency.get(uid, []):
            vid = edge.to_id if edge.from_id == uid else edge.from_id
            if vid in visited_set:
                continue
            edge_cost = max(edge.distance, 0.1) * (2.0 if edge.confidence < 0.5 else 1.0)
            new_cost = cost + edge_cost
            if new_cost < dist_map.get(vid, float("inf")):
                dist_map[vid] = new_cost
                prev_map[vid] = uid
                heapq.heappush(heap, (new_cost, vid))

    return float("inf"), []


def graph_hop_distances(
    from_id: int,
    nodes: dict[int, TopoNode],
    adjacency: dict[int, list[TopoEdge]],
) -> dict[int, int]:
    """BFS 计算从 from_id 到所有节点的跳数。"""
    if from_id not in nodes:
        return {}
    hops: dict[int, int] = {from_id: 0}
    queue = [from_id]
    while queue:
        uid = queue.pop(0)
        for edge in adjacency.get(uid, []):
            vid = edge.to_id if edge.from_id == uid else edge.from_id
            if vid not in hops:
                hops[vid] = hops[uid] + 1
                queue.append(vid)
    return hops


def compute_node_information_gain(
    node: TopoNode,
    target_instruction: str,
    semantic_engine: Any,
) -> float:
    """
    计算单个节点的信息增益 (TopoNav + L3MVN 融合)。

    IG(node) = semantic_prior × novelty × uncertainty_reduction
    """
    semantic_score = _FRONTIER_BASE_PRIOR
    if semantic_engine is not None:
        if node.node_type == "room":
            room_dict = {
                "room_id": node.node_id,
                "name": node.name,
                "semantic_labels": node.semantic_labels,
            }
            priors = semantic_engine.score_rooms_for_target(target_instruction, [room_dict])
            if priors:
                semantic_score = priors[0].prior_score
        elif node.node_type == "frontier" and node.predicted_room_type:
            predictions = semantic_engine.predict_target_rooms(target_instruction)
            for rt, prob, _ in predictions:
                if rt == node.predicted_room_type:
                    semantic_score = prob * _SEMANTIC_BOOST_FACTOR
                    break

    novelty = 1.0
    if node.visited:
        dt = _time_mod.time() - node.last_visited if node.last_visited > 0 else 0
        visit_decay = math.exp(-node.visit_count * 0.5)
        time_recovery = 1.0 - math.exp(-dt / _NOVELTY_TIME_TAU)
        novelty = _VISITED_PENALTY + (1.0 - _VISITED_PENALTY) * visit_decay * time_recovery

    uncertainty = 1.0
    if node.node_type == "frontier":
        uncertainty = 1.2 + 0.1 * min(node.frontier_size, 5.0)
    elif node.node_type == "room" and not node.visited:
        uncertainty = 1.0 + 0.05 * max(0, 5 - len(node.semantic_labels))

    return semantic_score * novelty * uncertainty


def tsg_to_dict(
    nodes: dict[int, TopoNode],
    edges: list[TopoEdge],
    current_room_id: int,
    traversal_history: list[dict],
) -> dict:
    """将拓扑图状态导出为可 JSON 序列化的字典。"""
    nodes_list = []
    for n in nodes.values():
        d: dict = {
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
        if n.bounding_box:
            d["bounding_box"] = {k: round(float(v), 2) for k, v in n.bounding_box.items()}
        if n.convex_hull is not None:
            d["convex_hull"] = [[round(float(x), 2), round(float(y), 2)] for x, y in n.convex_hull]
        if n.traversable_area > 0:
            d["traversable_area"] = round(n.traversable_area, 2)
        if n.height_range:
            d["height_range"] = {k: round(float(v), 2) for k, v in n.height_range.items()}
        if n.geometry_confidence > 0:
            d["geometry_confidence"] = round(n.geometry_confidence, 2)
        nodes_list.append(d)

    edges_list = [{
        "from_id": e.from_id,
        "to_id": e.to_id,
        "edge_type": e.edge_type,
        "distance": round(e.distance, 2),
        "traversal_count": e.traversal_count,
        "confidence": round(e.confidence, 2),
        "mediator": e.mediator_label,
    } for e in edges]

    return {
        "nodes": nodes_list,
        "edges": edges_list,
        "current_room_id": current_room_id,
        "traversal_count": len(traversal_history),
    }


def tsg_nodes_edges_from_dict(
    data: dict,
) -> tuple[dict[int, TopoNode], list[TopoEdge], dict[int, list[TopoEdge]], int]:
    """从字典恢复节点、边、邻接表和 current_room_id。"""
    nodes: dict[int, TopoNode] = {}
    edges: list[TopoEdge] = []
    adjacency: dict[int, list[TopoEdge]] = defaultdict(list)

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
        nodes[nd["node_id"]] = node

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
        edges.append(edge)
        adjacency[edge.from_id].append(edge)
        adjacency[edge.to_id].append(edge)

    current_room_id = data.get("current_room_id", -1)
    return nodes, edges, adjacency, current_room_id


def tsg_to_prompt_context(
    nodes: dict[int, TopoNode],
    edges: list[TopoEdge],
    language: str = "zh",
) -> str:
    """生成 LLM 可消费的拓扑摘要。"""
    rooms = sorted(
        [n for n in nodes.values() if n.node_type == "room"], key=lambda n: n.node_id
    )
    frontiers = sorted(
        [n for n in nodes.values() if n.node_type == "frontier"], key=lambda n: n.node_id
    )
    parts = []

    if language == "zh":
        if rooms:
            parts.append("已知房间: " + ", ".join(
                f"R{r.node_id}({r.name},{'[已探索]' if r.visited else '[未探索]'})"
                for r in rooms
            ))
        edge_strs = []
        for e in edges:
            n1, n2 = nodes.get(e.from_id), nodes.get(e.to_id)
            if n1 and n2:
                t = f",穿越{e.traversal_count}次" if e.traversal_count > 0 else ""
                edge_strs.append(f"{n1.name} ↔ {n2.name}({e.edge_type}{t})")
        if edge_strs:
            parts.append(f"连通关系: {'; '.join(edge_strs[:10])}")
        if frontiers:
            fstrs = []
            for f in frontiers:
                pred = f"→可能是{f.predicted_room_type}" if f.predicted_room_type else ""
                dx, dy = (
                    f.frontier_direction[0], f.frontier_direction[1],
                ) if f.frontier_direction is not None else (0, 0)
                fstrs.append(f"{f.name}({direction_name_zh(dx, dy)}{pred})")
            parts.append(f"未探索前沿: {', '.join(fstrs)}")
        visited_rooms = [r for r in rooms if r.visited]
        if visited_rooms:
            parts.append("已访问: " + ", ".join(f"{r.name}({r.visit_count}次)" for r in visited_rooms))
    else:
        if rooms:
            parts.append("Rooms: " + ", ".join(
                f"R{r.node_id}({r.name},{'visited' if r.visited else 'unvisited'})"
                for r in rooms
            ))
        edge_strs = []
        for e in edges:
            n1, n2 = nodes.get(e.from_id), nodes.get(e.to_id)
            if n1 and n2:
                t = f",traversed:{e.traversal_count}" if e.traversal_count > 0 else ""
                edge_strs.append(f"{n1.name} ↔ {n2.name}({e.edge_type}{t})")
        if edge_strs:
            parts.append(f"Connections: {'; '.join(edge_strs[:10])}")
        if frontiers:
            fstrs = []
            for f in frontiers:
                pred = f"→{f.predicted_room_type}" if f.predicted_room_type else ""
                dx, dy = (
                    f.frontier_direction[0], f.frontier_direction[1],
                ) if f.frontier_direction is not None else (0, 0)
                fstrs.append(f"{f.name}({direction_name_en(dx, dy)}{pred})")
            parts.append(f"Frontiers: {', '.join(fstrs)}")
        visited_rooms = [r for r in rooms if r.visited]
        if visited_rooms:
            parts.append("Visited: " + ", ".join(f"{r.name}(×{r.visit_count})" for r in visited_rooms))

    return "\n".join(parts)
