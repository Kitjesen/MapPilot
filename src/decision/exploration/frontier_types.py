"""
Frontier 数据类型 + 常量 + 纯函数辅助 — 供 frontier_scorer.py 和其他模块共享。

从 frontier_scorer.py 中提取, 保持向后兼容。
"""

from __future__ import annotations

import logging
import math
import re
from dataclasses import dataclass, field
from typing import Any

from runtime.msgs.numpy_compat import np
from runtime.utils.sanitize import sanitize_float

logger = logging.getLogger(__name__)

# ── 双语扩展 (延迟导入，首次使用时初始化) ──
try:
    from decision.goal_resolution.chinese_tokenizer import _ZH_TO_EN
    from decision.goal_resolution.chinese_tokenizer import expand_bilingual as _expand_bilingual
    _ZH_TO_EN_KEYS: frozenset = frozenset(_ZH_TO_EN.keys())
except ImportError:
    _expand_bilingual = None
    _ZH_TO_EN_KEYS = frozenset()

# ── OccupancyGrid 常量 ──
FREE_CELL = 0
OCCUPIED_CELL = 100
UNKNOWN_CELL = -1

# ── 共现先验表 (模块级常量, 避免每次 score_frontiers 重建) ──
_COOCCURRENCE: dict[str, list[str]] = {
    "fire extinguisher": ["door", "sign", "stairs", "elevator", "corridor"],
    "extinguisher": ["door", "sign", "stairs"],
    "灭火器": ["door", "sign", "stairs", "elevator"],
    "chair": ["desk", "table", "computer"],
    "椅子": ["desk", "table", "computer"],
    "desk": ["chair", "computer", "monitor", "keyboard"],
    "桌子": ["chair", "computer", "monitor"],
    "refrigerator": ["sink", "microwave", "table", "chair"],
    "冰箱": ["sink", "microwave", "table"],
    "toilet": ["sink", "mirror", "door"],
    "马桶": ["sink", "mirror", "door"],
    "elevator": ["door", "sign", "stairs", "button"],
    "电梯": ["door", "sign", "stairs"],
}
# 预计算反向索引: co_label → set(keys) — 用于 O(1) 查找
_COOCCURRENCE_REVERSE: dict[str, set[str]] = {}
for _key, _co_labels in _COOCCURRENCE.items():
    for _cl in _co_labels:
        _COOCCURRENCE_REVERSE.setdefault(_cl, set()).add(_key)


@dataclass
class Frontier:
    """一个 frontier 区域。"""
    frontier_id: int
    cells: list[tuple[int, int]] = field(default_factory=list)  # (row, col) 列表
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    center_world: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    size: int = 0                    # 包含的 cell 数量
    score: float = 0.0              # 语言相关性评分
    distance: float = 0.0           # 到机器人的距离
    direction_label: str = ""       # "north" / "east" / etc.
    nearby_labels: list[str] = field(default_factory=list)  # 附近已知物体
    description: str = ""             # 自然语言描述 (L3MVN/OmniNav 风格)

    def to_dict(self) -> dict:
        return {
            "frontier_id": self.frontier_id,
            "center_world": {
                "x": round(sanitize_float(float(self.center_world[0])), 2),
                "y": round(sanitize_float(float(self.center_world[1])), 2),
            },
            "size": self.size,
            "score": round(sanitize_float(self.score), 3),
            "distance": round(sanitize_float(self.distance), 2),
            "direction": self.direction_label,
            "nearby_objects": self.nearby_labels,
        }


# ══════════════════════════════════════════════════════════════════
#  纯函数辅助 (从 FrontierScorer 提取, 无实例状态依赖)
# ══════════════════════════════════════════════════════════════════

def angle_to_label(angle: float) -> str:
    """角度 (rad, atan2 约定) → 方向标签。

    0 = east, pi/2 = north, pi/-pi = west, -pi/2 = south
    """
    directions = [
        "east", "northeast", "north", "northwest",
        "west", "southwest", "south", "southeast",
    ]
    idx = round(angle / (2 * math.pi) * 8) % 8
    return directions[idx]


def angle_diff(a: float, b: float) -> float:
    """计算两个角度的最小差 (-pi, pi]。O(1) 闭合公式，无循环。"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def cooccurrence_score(inst_keywords: set[str], label: str) -> float:
    """
    常识共现评分 (MTU3D 空间先验)。

    使用模块级 _COOCCURRENCE_REVERSE 反向索引实现 O(1) 查找。

    Args:
        inst_keywords: 指令双语关键词集 (已通过 expand_bilingual 扩展)
        label: 场景物体标签 (小写)
    """
    keys_for_label = _COOCCURRENCE_REVERSE.get(label)
    if keys_for_label and keys_for_label & inst_keywords:
        return 0.25
    return 0.0


def extract_bilingual_keywords(inst_lower: str) -> set[str]:
    """从指令提取关键词并双语扩展, 用于跨语言 frontier 评分。"""
    stop_en = {
        "the", "a", "an", "to", "go", "find", "near", "with", "at",
        "of", "please", "where", "is", "i", "want", "need", "look",
    }
    stop_zh_prefix = ["去", "找", "到", "在", "看", "帮", "把", "让", "给"]
    en_tokens = re.findall(r"[a-z]+", inst_lower)
    keywords: list[str] = [t for t in en_tokens if t not in stop_en and len(t) > 1]
    zh_splitters = r"的|旁边|附近|里面|上面|下面|前面|后面|左边|右边|对面|中间|那里|那边|这里|这边|那个|这个"
    zh_raw = re.findall(r"[\u4e00-\u9fff]+", inst_lower)
    for zh in zh_raw:
        parts = re.split(zh_splitters, zh)
        for part in parts:
            cleaned = part
            changed = True
            while changed and len(cleaned) > 1:
                changed = False
                for pfx in stop_zh_prefix:
                    if cleaned.startswith(pfx) and len(cleaned) > len(pfx):
                        cleaned = cleaned[len(pfx):]
                        changed = True
                        break
            if len(cleaned) > 1 or (len(cleaned) == 1 and cleaned in _ZH_TO_EN_KEYS):
                keywords.append(cleaned)
    if _expand_bilingual is not None:
        keywords = _expand_bilingual(keywords)
    return set(keywords)


def estimate_information_gain(
    frontier: "Frontier",
    grid: np.ndarray | None,
    ig_radius_cells: int,
) -> float:
    """
    估算 frontier 的信息增益 (USS-Nav: IG_i)。

    统计 frontier 中心附近 ig_radius_cells 范围内的 unknown cell 数量。
    如果没有 costmap, 退化为 frontier.size 作为近似。
    """
    if grid is None:
        return max(float(frontier.size), 1.0)

    rows, cols = grid.shape
    center_r = int(round(frontier.center[0]))
    center_c = int(round(frontier.center[1]))
    r = ig_radius_cells

    r_lo = max(0, center_r - r)
    r_hi = min(rows, center_r + r + 1)
    c_lo = max(0, center_c - r)
    c_hi = min(cols, center_c + r + 1)

    patch = grid[r_lo:r_hi, c_lo:c_hi]
    unknown_count = int(np.count_nonzero(patch == UNKNOWN_CELL))
    return max(float(unknown_count), 1.0)


def tsp_sort_frontiers(
    frontiers: list["Frontier"],
    robot_position: np.ndarray,
    grid: np.ndarray | None,
    frontier_limit: int,
    ig_radius_cells: int,
) -> list["Frontier"]:
    """
    USS-Nav 贪心 nearest-neighbor TSP 重排序。

    代价矩阵: cost(i, j) = 1/IG_j + euclidean_dist(i, j)
    从 robot 出发, 每步选最低代价的未访问 frontier。
    frontier 数量 > frontier_limit 时跳过 TSP, 保持原排序。
    """
    n = len(frontiers)
    if n < 2 or n > frontier_limit:
        return frontiers

    ig = [estimate_information_gain(f, grid, ig_radius_cells) for f in frontiers]
    centers = np.array([f.center_world for f in frontiers])  # (n, 2)
    robot_xy = robot_position[:2]

    visited = [False] * n
    order: list[int] = []
    current_pos = robot_xy

    for _ in range(n):
        best_idx = -1
        best_cost = float('inf')
        for j in range(n):
            if visited[j]:
                continue
            dist = float(np.linalg.norm(current_pos - centers[j]))
            cost = 1.0 / ig[j] + dist
            if cost < best_cost:
                best_cost = cost
                best_idx = j
        if best_idx < 0:
            break
        visited[best_idx] = True
        order.append(best_idx)
        current_pos = centers[best_idx]

    result = [frontiers[i] for i in order]

    if logger.isEnabledFor(logging.DEBUG):
        ids = [f.frontier_id for f in result]
        igs_ordered = [ig[i] for i in order]
        logger.debug(
            "TSP reorder: %s (IG=%s)", ids,
            [round(g, 1) for g in igs_ordered],
        )

    return result


def compute_semantic_prior_score(
    frontier: "Frontier",
    robot_position: np.ndarray,
    scene_rooms: list[dict],
    room_priors_cache: dict[int, float],
    semantic_prior_engine: Any,
) -> float:
    """
    计算 frontier 方向对应房间的语义先验评分 (创新4)。

    思路: frontier 方向 → 最近房间 → 该房间的先验概率
    如果 frontier 指向高先验但未探索的房间类型 → 高分
    """
    if room_priors_cache and scene_rooms:
        best_score = 0.0
        for room in scene_rooms:
            room_center = np.array([
                float(room.get("center", {}).get("x", 0.0)),
                float(room.get("center", {}).get("y", 0.0)),
            ])
            frontier_vec = frontier.center_world - robot_position[:2]
            room_vec = room_center - robot_position[:2]
            fn = np.linalg.norm(frontier_vec)
            rn = np.linalg.norm(room_vec)
            if fn < 0.1 or rn < 0.1:
                continue
            cos_sim = float(np.dot(frontier_vec, room_vec) / (fn * rn))
            if cos_sim < 0.3:
                continue
            room_id = room.get("room_id", -1)
            prior = room_priors_cache.get(room_id, 0.0)
            best_score = max(best_score, cos_sim * prior)
        return min(1.0, best_score)

    if (semantic_prior_engine and frontier.nearby_labels
            and hasattr(semantic_prior_engine, 'predict_room_type_from_labels')):
        room_scores = semantic_prior_engine.predict_room_type_from_labels(
            frontier.nearby_labels,
        )
        if room_scores:
            top_score = next(iter(room_scores.values()), 0.0)
            return min(1.0, top_score)

    return 0.0


def compute_uncertainty_score(
    frontier: "Frontier",
    robot_position: np.ndarray,
    scene_rooms: list[dict],
    room_type_posteriors: dict[int, Any],
) -> float:
    """
    计算 frontier 方向附近房间的语义不确定性评分。

    高 entropy (房间类型不确定) → 高分 → 鼓励探索。
    """
    if not room_type_posteriors:
        return 0.0

    best_score = 0.0
    for room in scene_rooms:
        room_center = np.array([
            float(room.get("center", {}).get("x", 0.0)),
            float(room.get("center", {}).get("y", 0.0)),
        ])
        frontier_vec = frontier.center_world - robot_position[:2]
        room_vec = room_center - robot_position[:2]
        fn = np.linalg.norm(frontier_vec)
        rn = np.linalg.norm(room_vec)
        if fn < 0.1 or rn < 0.1:
            continue
        cos_sim = float(np.dot(frontier_vec, room_vec) / (fn * rn))
        if cos_sim < 0.3:
            continue
        room_id = room.get("room_id", -1)
        posterior = room_type_posteriors.get(room_id)
        if posterior is None:
            continue
        entropy = posterior.entropy if hasattr(posterior, 'entropy') else 0.0
        n_hyp = len(posterior.hypotheses) if hasattr(posterior, 'hypotheses') else 1
        max_entropy = math.log2(max(n_hyp, 2))
        norm_entropy = min(1.0, entropy / max_entropy) if max_entropy > 0 else 0.0
        best_score = max(best_score, cos_sim * norm_entropy)

    return min(1.0, best_score)


def compute_kg_room_score(
    inst_keywords: set[str],
    nearby_labels: list[str],
    room_object_kg: Any,
) -> float:
    """
    P2: 用 KG 推断 frontier 方向的房间类型, 查询目标在该房间的概率。

    思路 (SEEK-style):
      1. 从附近物体推断可能的房间类型 (通过 KG 反向查询)
      2. 对每个推断出的房间类型, 查询指令目标在该房间的概率
      3. 返回最高概率作为评分
    """
    if not room_object_kg:
        return 0.0

    candidate_rooms: dict[str, float] = {}
    for lbl in nearby_labels:
        rooms = room_object_kg.get_object_rooms(lbl)
        for room_type, prob in rooms:
            candidate_rooms[room_type] = max(candidate_rooms.get(room_type, 0.0), prob)

    if not candidate_rooms:
        return 0.0

    priors = room_object_kg.to_room_object_priors(min_observations=1)
    best_score = 0.0
    for room_type, room_prob in candidate_rooms.items():
        room_priors = priors.get(room_type, {})
        for kw in inst_keywords:
            kw_lower = kw.lower()
            for obj_label, obj_prob in room_priors.items():
                if kw_lower in obj_label or obj_label in kw_lower:
                    best_score = max(best_score, room_prob * obj_prob)

    return min(1.0, best_score)


def generate_frontier_description(
    frontier: "Frontier",
    semantic_prior_engine: Any,
    desc_cache: dict[str, str],
) -> str:
    """为 frontier 生成自然语言描述（L3MVN/OmniNav 风格）。

    结果按标签集合缓存：相同标签组合直接返回缓存，避免重复调用
    predict_room_type_from_labels（每次约 5-20ms）。
    """
    labels = getattr(frontier, 'nearby_labels', []) or []
    unique_labels = list(dict.fromkeys(labels))[:6]

    cache_key = ",".join(sorted(unique_labels))
    if cache_key in desc_cache:
        return desc_cache[cache_key]

    room_type = "未知区域"
    if unique_labels and semantic_prior_engine is not None:
        try:
            if hasattr(semantic_prior_engine, 'predict_room_type_from_labels'):
                room_probs = semantic_prior_engine.predict_room_type_from_labels(unique_labels)
                if room_probs:
                    best_room = max(room_probs, key=room_probs.get)
                    if room_probs[best_room] > 0.3:
                        room_type = best_room
        except (ImportError, TypeError, ValueError, AttributeError) as e:
            logger.debug("Room type prediction failed: %s", e)

    if unique_labels:
        desc = f"可见对象：{'、'.join(unique_labels)}，推测区域：{room_type}"
    else:
        desc = f"未知区域（尚无可见对象），推测：{room_type}"

    if len(desc_cache) >= 256:
        stale_keys = list(desc_cache.keys())[:128]
        for k in stale_keys:
            del desc_cache[k]
    desc_cache[cache_key] = desc
    return desc
