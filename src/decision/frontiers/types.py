"""Shared frontier data types and pure scoring helpers."""

from __future__ import annotations

import logging
import math
import re
from dataclasses import dataclass, field
from typing import Any

from runtime.msgs.numpy_compat import np
from runtime.utils.sanitize import sanitize_float

logger = logging.getLogger(__name__)


try:
    from decision.goals.tokenizer import _ZH_TO_EN
    from decision.goals.tokenizer import expand_bilingual as _expand_bilingual

    _ZH_TO_EN_KEYS: frozenset = frozenset(_ZH_TO_EN.keys())
except ImportError:
    _expand_bilingual = None
    _ZH_TO_EN_KEYS = frozenset()


FREE_CELL = 0
OCCUPIED_CELL = 100
UNKNOWN_CELL = -1


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

_COOCCURRENCE_REVERSE: dict[str, set[str]] = {}
for _key, _co_labels in _COOCCURRENCE.items():
    for _cl in _co_labels:
        _COOCCURRENCE_REVERSE.setdefault(_cl, set()).add(_key)


@dataclass
class Frontier:
    """Frontier."""

    frontier_id: int
    cells: list[tuple[int, int]] = field(default_factory=list)
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    center_world: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    size: int = 0
    score: float = 0.0
    distance: float = 0.0
    direction_label: str = ""  # "north" / "east" / etc.
    nearby_labels: list[str] = field(default_factory=list)
    description: str = ""

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


def angle_to_label(angle: float) -> str:
    """Angle to label."""
    directions = [
        "east",
        "northeast",
        "north",
        "northwest",
        "west",
        "southwest",
        "south",
        "southeast",
    ]
    idx = round(angle / (2 * math.pi) * 8) % 8
    return directions[idx]


def angle_diff(a: float, b: float) -> float:
    """Angle diff."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def cooccurrence_score(inst_keywords: set[str], label: str) -> float:
    """Cooccurrence score."""
    keys_for_label = _COOCCURRENCE_REVERSE.get(label)
    if keys_for_label and keys_for_label & inst_keywords:
        return 0.25
    return 0.0


def extract_bilingual_keywords(inst_lower: str) -> set[str]:
    """Extract bilingual keywords."""
    stop_en = {
        "the",
        "a",
        "an",
        "to",
        "go",
        "find",
        "near",
        "with",
        "at",
        "of",
        "please",
        "where",
        "is",
        "i",
        "want",
        "need",
        "look",
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
                        cleaned = cleaned[len(pfx) :]
                        changed = True
                        break
            if len(cleaned) > 1 or (len(cleaned) == 1 and cleaned in _ZH_TO_EN_KEYS):
                keywords.append(cleaned)
    if _expand_bilingual is not None:
        keywords = _expand_bilingual(keywords)
    return set(keywords)


def estimate_information_gain(
    frontier: Frontier,
    grid: np.ndarray | None,
    ig_radius_cells: int,
) -> float:
    """Estimate information gain."""
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
    frontiers: list[Frontier],
    robot_position: np.ndarray,
    grid: np.ndarray | None,
    frontier_limit: int,
    ig_radius_cells: int,
) -> list[Frontier]:
    """Tsp sort frontiers."""
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
        best_cost = float("inf")
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
            "TSP reorder: %s (IG=%s)",
            ids,
            [round(g, 1) for g in igs_ordered],
        )

    return result


def compute_semantic_prior_score(
    frontier: Frontier,
    robot_position: np.ndarray,
    scene_rooms: list[dict],
    room_priors_cache: dict[int, float],
    semantic_prior_engine: Any,
) -> float:
    """Compute semantic prior score."""
    if room_priors_cache and scene_rooms:
        best_score = 0.0
        for room in scene_rooms:
            room_center = np.array(
                [
                    float(room.get("center", {}).get("x", 0.0)),
                    float(room.get("center", {}).get("y", 0.0)),
                ]
            )
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

    if (
        semantic_prior_engine
        and frontier.nearby_labels
        and hasattr(semantic_prior_engine, "predict_room_type_from_labels")
    ):
        room_scores = semantic_prior_engine.predict_room_type_from_labels(
            frontier.nearby_labels,
        )
        if room_scores:
            top_score = next(iter(room_scores.values()), 0.0)
            return min(1.0, top_score)

    return 0.0


def compute_uncertainty_score(
    frontier: Frontier,
    robot_position: np.ndarray,
    scene_rooms: list[dict],
    room_type_posteriors: dict[int, Any],
) -> float:
    """Compute uncertainty score."""
    if not room_type_posteriors:
        return 0.0

    best_score = 0.0
    for room in scene_rooms:
        room_center = np.array(
            [
                float(room.get("center", {}).get("x", 0.0)),
                float(room.get("center", {}).get("y", 0.0)),
            ]
        )
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
        entropy = posterior.entropy if hasattr(posterior, "entropy") else 0.0
        n_hyp = len(posterior.hypotheses) if hasattr(posterior, "hypotheses") else 1
        max_entropy = math.log2(max(n_hyp, 2))
        norm_entropy = min(1.0, entropy / max_entropy) if max_entropy > 0 else 0.0
        best_score = max(best_score, cos_sim * norm_entropy)

    return min(1.0, best_score)


def compute_kg_room_score(
    inst_keywords: set[str],
    nearby_labels: list[str],
    room_object_kg: Any,
) -> float:
    """Compute kg room score."""
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
    frontier: Frontier,
    semantic_prior_engine: Any,
    desc_cache: dict[str, str],
) -> str:
    """Generate frontier description."""
    labels = getattr(frontier, "nearby_labels", []) or []
    unique_labels = list(dict.fromkeys(labels))[:6]

    cache_key = ",".join(sorted(unique_labels))
    if cache_key in desc_cache:
        return desc_cache[cache_key]

    room_type = "未知区域"
    if unique_labels and semantic_prior_engine is not None:
        try:
            if hasattr(semantic_prior_engine, "predict_room_type_from_labels"):
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
