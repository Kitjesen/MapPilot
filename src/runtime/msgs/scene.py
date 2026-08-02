"""Scene graph data types — shared between perception/decision and memory.

Pure data classes + constants. No algorithm logic, no external dependencies.
Extracted from perception/tracked_objects.py for cross-module sharing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# -- Spatial relation constants (SG-Nav style) --

RELATION_NEAR_THRESHOLD = 1.5
RELATION_ON_THRESHOLD = 0.3
REGION_CLUSTER_RADIUS = 3.0
FLOOR_HEIGHT = 3.0
FLOOR_MERGE_TOLERANCE = 0.8

# -- BA-HSG belief parameters --

BELIEF_SIGMA_BASE = 0.05
BELIEF_SIGMA_DEPTH_COEFF = 0.02
BELIEF_NEG_EVIDENCE_WEIGHT = 0.5
BELIEF_FRESHNESS_TAU = 30.0
BELIEF_REPROJ_KAPPA = 5.0
BELIEF_ROOM_BOOST = 0.3
BELIEF_LATERAL_SHARE = 0.1

# -- KG-Augmented Loopy Belief Propagation --

BP_MAX_ITERATIONS = 3
BP_CONVERGENCE_EPS = 0.005
BP_KG_PRIOR_BOOST = 1.5
BP_KG_UNEXPECTED_PENALTY = 0.3
BP_ROOM_TO_OBJ_WEIGHT = 0.6
BP_OBJ_TO_ROOM_WEIGHT = 0.8
BP_LATERAL_DECAY = 0.7
BP_PHANTOM_BASE_ALPHA = 0.8
BP_PHANTOM_MIN_ROOM_CONFIDENCE = 0.4

# -- Safety-Aware Differential Credibility --

SAFETY_THRESHOLDS_NAVIGATION = {
    "safe": 0.25, "caution": 0.15, "dangerous": 0.10, "forbidden": 0.05,
}
SAFETY_THRESHOLDS_INTERACTION = {
    "safe": 0.40, "caution": 0.60, "dangerous": 0.80, "forbidden": 0.95,
}
SAFETY_PRIOR_ALPHA_SCALE = {
    "safe": 1.0, "caution": 1.2, "dangerous": 1.5, "forbidden": 2.0,
}

# -- Room type rules --

ROOM_TYPE_RULES = {
    "corridor": {"keywords": ["door", "sign", "corridor", "hallway", "exit", "门", "走廊", "出口"], "min_match": 1, "priority": 2},
    "office": {"keywords": ["desk", "chair", "computer", "monitor", "keyboard", "mouse", "办公", "桌", "电脑"], "min_match": 2, "priority": 3},
    "kitchen": {"keywords": ["refrigerator", "sink", "microwave", "oven", "kettle", "冰箱", "厨房", "水壶"], "min_match": 1, "priority": 3},
    "meeting_room": {"keywords": ["table", "chair", "screen", "projector", "whiteboard", "会议", "投影"], "min_match": 2, "priority": 3},
    "bathroom": {"keywords": ["toilet", "sink", "mirror", "卫生间", "洗手", "镜"], "min_match": 1, "priority": 3},
    "stairwell": {"keywords": ["stairs", "staircase", "railing", "楼梯", "扶手"], "min_match": 1, "priority": 3},
    "lobby": {"keywords": ["sofa", "reception", "lobby", "大厅", "沙发", "前台"], "min_match": 1, "priority": 2},
    "storage": {"keywords": ["shelf", "cabinet", "box", "storage", "储物", "柜", "架"], "min_match": 2, "priority": 1},
}

GROUP_KEYWORDS = {
    "safety": ["fire", "extinguisher", "alarm", "灭火", "应急"],
    "furniture": ["chair", "desk", "table", "sofa", "cabinet", "shelf", "椅", "桌", "柜"],
    "structure": ["door", "window", "stairs", "elevator", "hall", "corridor", "门", "窗", "楼梯"],
    "electronics": ["monitor", "screen", "computer", "tv", "phone", "显示", "电脑", "电视"],
    "utility": ["trash", "bin", "bottle", "refrigerator", "sink", "lamp", "垃圾", "冰箱", "灯"],
}

ROOM_NAMING_STABILITY_COUNT = 3
ROOM_NAMING_STABILITY_SEC = 10.0


def infer_room_type(labels: list[str]) -> str:
    """Infer room type from object labels (rule-based fallback for LLM naming)."""
    labels_lower = [label.lower() for label in labels]
    best_type = ""
    best_priority = -1

    for room_type, rule in ROOM_TYPE_RULES.items():
        matches = sum(1 for kw in rule["keywords"] if any(kw in label for label in labels_lower))
        if matches >= rule["min_match"]:
            score = matches * 10 + rule["priority"]
            if score > best_priority:
                best_priority = score
                best_type = room_type

    if best_type:
        return best_type
    if labels:
        from collections import Counter
        common = Counter(labels).most_common(2)
        return f"area_{'_'.join(label for label, _ in common)}"
    return "unknown_area"



# -- Data classes --

@dataclass
class SpatialRelation:
    """Oriented spatial relation between two tracked objects (e.g. "on", "near")."""
    subject_id: int = -1
    relation: str = ""
    object_id: int = -1
    distance: float = 0.0


@dataclass
class Region:
    """A clustered region in space grouping nearby objects (SG-Nav clustering unit)."""
    region_id: int = -1
    center: np.ndarray | None = None
    object_ids: list = field(default_factory=list)
    name: str = ""
    node_ids: list[int] = field(default_factory=list)
    region_type: str = "unknown"
    llm_named: bool = False


@dataclass
class GroupNode:
    """A semantic group within a room — objects sharing a sub-region and label."""
    group_id: int = -1
    room_id: int = -1
    name: str = ""
    center: np.ndarray | None = None
    object_ids: list[int] = field(default_factory=list)
    semantic_labels: list[str] = field(default_factory=list)


@dataclass
class RoomNode:
    """A named room in the topology graph with associated objects and groups."""
    room_id: int = -1
    name: str = ""
    center: np.ndarray | None = None
    object_ids: list[int] = field(default_factory=list)
    group_ids: list[int] = field(default_factory=list)
    semantic_labels: list[str] = field(default_factory=list)
    floor_id: int = -1
    llm_named: bool = False
    clip_feature: np.ndarray | None = None
    feature_count: int = 0


@dataclass
class FloorNode:
    """A floor level defined by a vertical Z range containing rooms and objects."""
    floor_id: int = -1
    floor_level: int = 0
    z_range: tuple[float, float] = (0.0, 0.0)
    object_ids: list[int] = field(default_factory=list)
    room_ids: list[int] = field(default_factory=list)
    center_z: float = 0.0


@dataclass
class PhantomNode:
    """A hypothesised object from KG prior with Beta-distribution existence belief."""
    phantom_id: int = -1
    label: str = ""
    room_id: int = -1
    room_type: str = ""
    position: np.ndarray | None = None
    belief_alpha: float = 1.0
    belief_beta: float = 1.0
    kg_prior_strength: float = 0.0
    safety_level: str = "safe"
    source: str = ""

    @property
    def existence_prob(self) -> float:
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)


@dataclass
class RoomTypePosterior:
    room_id: int
    hypotheses: dict[str, float] = field(default_factory=dict)
    _log_posteriors: dict[str, float] = field(default_factory=dict)

    @property
    def best_type(self) -> str:
        if not self.hypotheses:
            return "unknown"
        return max(self.hypotheses, key=self.hypotheses.get)

    @property
    def best_confidence(self) -> float:
        if not self.hypotheses:
            return 0.0
        return max(self.hypotheses.values())

    @property
    def entropy(self) -> float:
        if not self.hypotheses:
            return 0.0
        vals = list(self.hypotheses.values())
        total = sum(vals)
        if total <= 0:
            return 0.0
        h = 0.0
        for v in vals:
            p = v / total
            if p > 1e-10:
                h -= p * math.log2(p)
        return h


@dataclass
class BeliefMessage:
    source_type: str
    source_id: int
    target_type: str
    target_id: int
    message_type: str
    delta_alpha: float = 0.0
    delta_beta: float = 0.0
    weight: float = 1.0


@dataclass
class ViewNode:
    view_id: int
    position: np.ndarray
    timestamp: float
    room_id: int = -1
    object_ids: list[int] = field(default_factory=list)
    key_labels: list[str] = field(default_factory=list)
