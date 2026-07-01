"""
持久化房间-物体拓扑知识图谱。

在建图 (mapping) 阶段, 从场景图实时统计 room-object 共现关系;
保存到 JSON 文件; 导航 (planning) 阶段加载, 替代手写 ROOM_OBJECT_PRIORS。

设计:
  - 每个 (room_type, object_label) 对记录: observation_count, total_room_visits
  - 共现概率 = observation_count / total_room_visits  (贝叶斯平滑后)
  - 支持多次 mapping session 合并 (additive)
  - 房间间拓扑邻接也持久化 (door/traversal)

文件格式 (room_object_kg.json):
{
  "version": "1.0",
  "sessions": 3,
  "room_types": {
    "office": {
      "total_visits": 15,
      "objects": {
        "desk": {"count": 14, "avg_confidence": 0.87},
        "chair": {"count": 13, "avg_confidence": 0.82},
        ...
      }
    },
    ...
  },
  "room_adjacency": [
    {"from": "corridor", "to": "office", "count": 8, "mediator": "door"},
    ...
  ],
  "object_room_index": {
    "fire extinguisher": [["corridor", 0.70], ["stairwell", 0.55]],
    ...
  }
}
"""

import json
import logging
import os
import time
from collections import defaultdict

logger = logging.getLogger(__name__)

# Laplace smoothing pseudo-count
_SMOOTHING_ALPHA = 1.0
_SMOOTHING_BETA = 2.0  # equivalent to 2 "phantom" visits
_DEFAULT_COOCCUR_MIN_PROB = 0.3   # minimum co-occurrence probability for related-object retrieval
_DEFAULT_COOCCUR_TOP_K = 10        # max objects returned per co-occurrence query


class RoomObjectKG:
    """
    持久化房间-物体共现知识图谱。

    Usage (mapping):
        kg = RoomObjectKG()
        kg.load("maps/semantic/room_object_kg.json")  # merge with previous
        # ... during mapping:
        kg.observe_room("office", ["desk", "chair", "monitor"], confidences=[0.9, 0.8, 0.85])
        kg.observe_adjacency("corridor", "office", mediator="door")
        # ... save:
        kg.save("maps/semantic/room_object_kg.json")

    Usage (navigation):
        kg = RoomObjectKG()
        kg.load("maps/semantic/room_object_kg.json")
        priors = kg.to_room_object_priors()  # Dict[str, Dict[str, float]]
        engine = SemanticPriorEngine(room_priors=priors)
    """

    def __init__(self) -> None:
        # room_type → {total_visits, objects: {label → {count, conf_sum}}}
        self._rooms: dict[str, dict] = {}
        # [(from_type, to_type, count, mediator)]
        self._adjacency: dict[tuple[str, str], dict] = {}
        self._session_count: int = 0
        self._last_save_time: float = 0.0

    # ── 观测接口 (mapping 阶段) ──

    def observe_room(
        self,
        room_type: str,
        object_labels: list[str],
        confidences: list[float] | None = None,
    ) -> None:
        """
        记录一次房间观测: 在 room_type 中看到了 object_labels。

        每次机器人进入/停留在某房间时调用一次。
        """
        rt = room_type.lower().strip()
        if not rt or rt == "unknown" or rt == "unknown_area":
            return

        if rt not in self._rooms:
            self._rooms[rt] = {"total_visits": 0, "objects": {}}

        info = self._rooms[rt]
        info["total_visits"] += 1

        confs = confidences or [0.8] * len(object_labels)
        for i, label in enumerate(object_labels):
            lbl = label.lower().strip()
            if not lbl:
                continue
            conf = confs[i] if i < len(confs) else 0.8

            if lbl not in info["objects"]:
                info["objects"][lbl] = {"count": 0, "conf_sum": 0.0}
            info["objects"][lbl]["count"] += 1
            info["objects"][lbl]["conf_sum"] += conf

    def observe_adjacency(
        self,
        from_type: str,
        to_type: str,
        mediator: str = "traversal",
    ) -> None:
        """记录房间间拓扑邻接 (机器人从 from_type 移动到 to_type)。"""
        ft = from_type.lower().strip()
        tt = to_type.lower().strip()
        if not ft or not tt or ft == tt:
            return
        # Canonical order for undirected edge
        key = (min(ft, tt), max(ft, tt))
        if key not in self._adjacency:
            self._adjacency[key] = {"count": 0, "mediators": defaultdict(int)}
        self._adjacency[key]["count"] += 1
        self._adjacency[key]["mediators"][mediator] += 1

    # ── 查询接口 ──

    def to_room_object_priors(
        self,
        min_observations: int = 2,
    ) -> dict[str, dict[str, float]]:
        """
        将累积统计转换为 ROOM_OBJECT_PRIORS 格式。

        概率 = (count + alpha) / (total_visits + beta)  (Laplace 平滑)
        """
        priors: dict[str, dict[str, float]] = {}
        for rt, info in self._rooms.items():
            total = info["total_visits"]
            if total < 1:
                continue
            objects: dict[str, float] = {}
            for lbl, stats in info["objects"].items():
                if stats["count"] < min_observations:
                    continue
                prob = (stats["count"] + _SMOOTHING_ALPHA) / (total + _SMOOTHING_BETA)
                prob = min(prob, 0.98)  # cap at 0.98
                objects[lbl] = round(prob, 3)
            if objects:
                # Sort by probability descending
                priors[rt] = dict(sorted(objects.items(), key=lambda x: x[1], reverse=True))
        return priors

    def get_object_rooms(self, label: str) -> list[tuple[str, float]]:
        """反向查询: 物体最可能在哪种房间中? → [(room_type, probability)]"""
        lbl = label.lower().strip()
        results: list[tuple[str, float]] = []
        for rt, info in self._rooms.items():
            total = info["total_visits"]
            if total < 1:
                continue
            stats = info["objects"].get(lbl)
            if stats and stats["count"] > 0:
                prob = (stats["count"] + _SMOOTHING_ALPHA) / (total + _SMOOTHING_BETA)
                results.append((rt, round(min(prob, 0.98), 3)))
        results.sort(key=lambda x: x[1], reverse=True)
        return results

    def get_cooccurring_objects(
        self,
        label: str,
        min_prob: float = _DEFAULT_COOCCUR_MIN_PROB,
        top_k: int = _DEFAULT_COOCCUR_TOP_K,
    ) -> list[tuple[str, float]]:
        """
        Find objects that co-occur with `label` in the same room types.

        Uses learned P(object|room) to compute implicit co-occurrence:
        for each room where `label` appears, collect other objects with
        high probability. Returns weighted list sorted by co-occurrence
        strength.

        Inspired by ASCENT (Gong et al. 2025) knowledge graph co-occurrence.

        Args:
            label: Object label to find co-occurring objects for.
            min_prob: Minimum P(object|room) to consider (default 0.3).
            top_k: Maximum number of co-occurring objects to return.

        Returns:
            List of (co_label, weight) sorted descending by weight.
            Weight = max over rooms of P(label|room) * P(co_label|room).
        """
        lbl = label.lower().strip()
        # Find rooms where this label appears
        label_rooms = self.get_object_rooms(lbl)
        if not label_rooms:
            return []

        priors = self.to_room_object_priors(min_observations=1)
        co_scores: dict[str, float] = {}

        for room_type, label_prob in label_rooms:
            room_objects = priors.get(room_type, {})
            for obj_label, obj_prob in room_objects.items():
                if obj_label == lbl or obj_prob < min_prob:
                    continue
                # Co-occurrence strength = P(label|room) * P(co_object|room)
                strength = label_prob * obj_prob
                co_scores[obj_label] = max(
                    co_scores.get(obj_label, 0.0), strength,
                )

        results = sorted(co_scores.items(), key=lambda x: x[1], reverse=True)
        return results[:top_k]

    def get_adjacency_graph(self) -> list[dict]:
        """获取房间邻接图。"""
        result = []
        for (ft, tt), info in self._adjacency.items():
            top_mediator = max(info["mediators"], key=info["mediators"].get) if info["mediators"] else "traversal"
            result.append({
                "from": ft,
                "to": tt,
                "count": info["count"],
                "mediator": top_mediator,
            })
        result.sort(key=lambda x: x["count"], reverse=True)
        return result

    def get_stats(self) -> dict:
        """获取统计摘要。"""
        total_objects = sum(
            len(info["objects"]) for info in self._rooms.values()
        )
        total_observations = sum(
            sum(s["count"] for s in info["objects"].values())
            for info in self._rooms.values()
        )
        return {
            "sessions": self._session_count,
            "room_types": len(self._rooms),
            "unique_objects": total_objects,
            "total_observations": total_observations,
            "adjacency_edges": len(self._adjacency),
        }

    @property
    def room_types(self) -> list[str]:
        return list(self._rooms.keys())

    @property
    def is_empty(self) -> bool:
        return len(self._rooms) == 0

    # ── 持久化 ──

    def save(self, path: str) -> bool:
        """保存知识图谱到 JSON 文件。"""
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

            data = {
                "version": "1.0",
                "sessions": self._session_count,
                "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "room_types": {},
                "room_adjacency": [],
            }

            for rt, info in self._rooms.items():
                objects_data = {}
                for lbl, stats in info["objects"].items():
                    avg_conf = stats["conf_sum"] / max(stats["count"], 1)
                    objects_data[lbl] = {
                        "count": stats["count"],
                        "avg_confidence": round(avg_conf, 3),
                    }
                data["room_types"][rt] = {
                    "total_visits": info["total_visits"],
                    "objects": objects_data,
                }

            data["room_adjacency"] = self.get_adjacency_graph()

            # Build inverse index for convenience
            priors = self.to_room_object_priors(min_observations=1)
            object_room_index: dict[str, list] = {}
            for rt, objs in priors.items():
                for lbl, prob in objs.items():
                    if lbl not in object_room_index:
                        object_room_index[lbl] = []
                    object_room_index[lbl].append([rt, prob])
            for lbl in object_room_index:
                object_room_index[lbl].sort(key=lambda x: x[1], reverse=True)
            data["object_room_index"] = object_room_index

            with open(path, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            self._last_save_time = time.time()
            logger.info(
                "RoomObjectKG saved to %s (%d room types, %d object types)",
                path, len(self._rooms),
                sum(len(info["objects"]) for info in self._rooms.values()),
            )
            return True
        except Exception as e:
            logger.error("Failed to save RoomObjectKG to %s: %s", path, e)
            return False

    def load(self, path: str) -> bool:
        """
        从 JSON 文件加载知识图谱 (合并到当前数据)。

        支持多次调用 load() 合并多个 session 的数据。
        """
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logger.warning("Failed to load RoomObjectKG from %s: %s", path, e)
            return False

        version = data.get("version", "1.0")
        if version != "1.0":
            logger.warning("Unknown RoomObjectKG version: %s", version)

        self._session_count += data.get("sessions", 1)

        # Merge room data
        for rt, rdata in data.get("room_types", {}).items():
            if rt not in self._rooms:
                self._rooms[rt] = {"total_visits": 0, "objects": {}}
            info = self._rooms[rt]
            info["total_visits"] += rdata.get("total_visits", 0)

            for lbl, ostats in rdata.get("objects", {}).items():
                if lbl not in info["objects"]:
                    info["objects"][lbl] = {"count": 0, "conf_sum": 0.0}
                obj = info["objects"][lbl]
                count = ostats.get("count", 0)
                obj["count"] += count
                avg_conf = ostats.get("avg_confidence", 0.8)
                obj["conf_sum"] += avg_conf * count

        # Merge adjacency
        for adj in data.get("room_adjacency", []):
            ft = adj.get("from", "")
            tt = adj.get("to", "")
            if not ft or not tt:
                continue
            key = (min(ft, tt), max(ft, tt))
            if key not in self._adjacency:
                self._adjacency[key] = {"count": 0, "mediators": defaultdict(int)}
            self._adjacency[key]["count"] += adj.get("count", 1)
            med = adj.get("mediator", "traversal")
            self._adjacency[key]["mediators"][med] += adj.get("count", 1)

        stats = self.get_stats()
        logger.info(
            "RoomObjectKG loaded from %s (merged: %d sessions, %d room types, %d objects)",
            path, self._session_count, stats["room_types"], stats["unique_objects"],
        )
        return True

    def start_new_session(self) -> None:
        """标记新的 mapping session 开始。"""
        self._session_count += 1
        logger.info("RoomObjectKG: new session #%d started", self._session_count)


# ── 从场景图 JSON 提取 room-object 统计的辅助函数 ──

def extract_room_objects_from_scene_graph(
    scene_graph_json: str,
) -> list[tuple[str, list[str], list[float]]]:
    """
    从场景图 JSON 中提取 (room_type, [object_labels], [confidences])。

    兼容两种场景图格式:
      1. regions 格式: {"regions": [{"name": "office", "object_ids": [...]}], "objects": [...]}
      2. 扁平格式: {"objects": [{"label": "desk", "region_id": 0, ...}]}
    """
    try:
        sg = json.loads(scene_graph_json)
    except (json.JSONDecodeError, TypeError):
        return []

    objects = sg.get("objects", [])
    if not isinstance(objects, list):
        return []

    # Build object lookup
    obj_map: dict = {}  # id → {label, confidence, region_id}
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        oid = obj.get("id", obj.get("object_id", -1))
        label = str(obj.get("label", "")).lower().strip()
        if not label:
            continue
        conf = float(obj.get("confidence", obj.get("best_score", 0.8)))
        region_id = obj.get("region_id", -1)
        obj_map[oid] = {"label": label, "confidence": conf, "region_id": region_id}

    regions = sg.get("regions", [])
    results: list[tuple[str, list[str], list[float]]] = []

    if isinstance(regions, list) and regions:
        for region in regions:
            if not isinstance(region, dict):
                continue
            name = str(region.get("name", "")).lower().strip()
            if not name:
                continue
            # Normalize room type
            room_type = _normalize_room_type_simple(name)
            obj_ids = region.get("object_ids", [])
            labels = []
            confs = []
            for oid in obj_ids:
                if oid in obj_map:
                    labels.append(obj_map[oid]["label"])
                    confs.append(obj_map[oid]["confidence"])
            if labels:
                results.append((room_type, labels, confs))
    else:
        # Fallback: group by region_id
        from collections import defaultdict
        region_objs: dict[int, list[tuple[str, float]]] = defaultdict(list)
        for _oid, info in obj_map.items():
            rid = info["region_id"]
            if rid >= 0:
                region_objs[rid].append((info["label"], info["confidence"]))

        if region_objs:
            for _rid, objs in region_objs.items():
                labels = [o[0] for o in objs]
                confs = [o[1] for o in objs]
                room_type = _infer_room_type_from_labels(labels)
                if room_type:
                    results.append((room_type, labels, confs))

    return results


def _normalize_room_type_simple(name: str) -> str:
    """简化版房间类型归一化。"""
    # Import the full version if available
    try:
        from .semantic_prior import SemanticPriorEngine
        return SemanticPriorEngine._normalize_room_type(name)
    except (ImportError, AttributeError):
        pass

    # Fallback minimal mapping
    _map = {
        "corridor": "corridor", "hallway": "corridor", "passage": "corridor",
        "走廊": "corridor", "过道": "corridor",
        "office": "office", "办公室": "office",
        "kitchen": "kitchen", "厨房": "kitchen",
        "meeting": "meeting_room", "会议室": "meeting_room",
        "bathroom": "bathroom", "卫生间": "bathroom",
        "stairwell": "stairwell", "楼梯": "stairwell",
        "lobby": "lobby", "大厅": "lobby",
        "storage": "storage", "储物间": "storage",
        "lab": "lab", "实验室": "lab",
        "classroom": "classroom", "教室": "classroom",
    }
    for key, val in _map.items():
        if key in name:
            return val
    return name


def _infer_room_type_from_labels(labels: list[str]) -> str:
    """从物体标签推断房间类型 (与 instance_tracker.infer_room_type 类似)。"""
    try:
        from runtime.msgs.scene import infer_room_type
        return infer_room_type(labels)
    except ImportError:
        pass

    # Minimal fallback
    labels_lower = set(label.lower() for label in labels)
    if labels_lower & {"desk", "computer", "monitor", "keyboard"}:
        return "office"
    if labels_lower & {"refrigerator", "sink", "microwave", "oven"}:
        return "kitchen"
    if labels_lower & {"toilet", "mirror"}:
        return "bathroom"
    if labels_lower & {"stairs", "railing"}:
        return "stairwell"
    if labels_lower & {"door", "sign", "exit"}:
        return "corridor"
    return ""
