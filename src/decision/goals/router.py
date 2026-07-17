"""Routing policy for fast and slow goal resolution."""

import logging
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class AdaCoTDecision(Enum):
    """Ada Co T Decision."""

    FAST = "fast"
    SLOW = "slow"
    AUTO = "auto"


_SPATIAL_MODIFIERS_ZH = {
    "旁边",
    "附近",
    "左边",
    "右边",
    "前面",
    "后面",
    "上面",
    "下面",
    "里面",
    "外面",
    "之间",
    "对面",
    "远离",
    "靠近",
    "中间",
}
_SPATIAL_MODIFIERS_EN = {
    "near",
    "beside",
    "next to",
    "left of",
    "right of",
    "in front of",
    "behind",
    "above",
    "below",
    "between",
    "inside",
    "outside",
    "opposite",
    "away from",
    "close to",
}


_NEGATION_ZH = {"不是", "不要", "除了", "而不是", "非", "别"}
_NEGATION_EN = {"not", "except", "other than", "instead of", "without", "don't"}


_COMPOUND_ZH = {"然后", "接着", "之后", "并且", "先", "再", "最后"}
_COMPOUND_EN = {"then", "after that", "and then", "first", "next", "finally"}


@dataclass
class AdaCoTConfig:
    """Ada Co T Config."""

    simple_max_tokens: int = 6
    complex_min_spatial: int = 2

    few_objects_max: int = 5
    many_objects_min: int = 30

    ambiguity_same_label_min: int = 3

    fast_score_min: float = 0.6
    slow_score_min: float = 0.6


class AdaCoTRouter:
    """Ada Co T Router."""

    def __init__(self, config: AdaCoTConfig | None = None):
        self._config = config or AdaCoTConfig()
        self._stats = {"fast": 0, "slow": 0, "auto": 0}

    def decide(
        self,
        instruction: str,
        scene_graph: dict | None = None,
        keywords: list[str] | None = None,
        score_entropy: float = 0.0,
    ) -> AdaCoTDecision:
        """Decide."""

        if score_entropy > 1.5:
            self._stats["slow"] += 1
            logger.debug(
                "AdaCoT: high score_entropy=%.2f -> SLOW",
                score_entropy,
            )
            return AdaCoTDecision.SLOW

        cfg = self._config

        inst_lower = instruction.lower().strip()
        tokens = inst_lower.split()
        n_tokens = len(tokens)

        n_spatial = sum(1 for w in _SPATIAL_MODIFIERS_ZH if w in instruction)
        n_spatial += sum(1 for w in _SPATIAL_MODIFIERS_EN if w in inst_lower)

        has_negation = any(w in instruction for w in _NEGATION_ZH) or any(w in inst_lower for w in _NEGATION_EN)

        has_compound = any(w in instruction for w in _COMPOUND_ZH) or any(w in inst_lower for w in _COMPOUND_EN)

        n_objects = 0
        n_relations = 0
        max_same_label = 0

        if scene_graph:
            objects = scene_graph.get("objects", [])
            n_objects = len(objects)
            n_relations = len(scene_graph.get("relations", []))

            label_counts: dict[str, int] = {}
            for obj in objects:
                lbl = str(obj.get("label", "")).lower()
                if lbl:
                    label_counts[lbl] = label_counts.get(lbl, 0) + 1
            if label_counts:
                max_same_label = max(label_counts.values())

        fast_score = 0.0
        slow_score = 0.0

        if n_tokens <= cfg.simple_max_tokens:
            fast_score += 0.3
        elif n_tokens > cfg.simple_max_tokens * 2:
            slow_score += 0.2

        if n_spatial == 0:
            fast_score += 0.2
        elif n_spatial >= cfg.complex_min_spatial:
            slow_score += 0.3

        if has_negation:
            slow_score += 0.3

        if has_compound:
            slow_score += 0.3

        if 0 < n_objects <= cfg.few_objects_max:
            fast_score += 0.2
        elif n_objects >= cfg.many_objects_min:
            slow_score += 0.2

        if max_same_label >= cfg.ambiguity_same_label_min:
            slow_score += 0.2

        if n_relations > 0 and n_spatial > 0:
            slow_score += 0.1

        decision = AdaCoTDecision.AUTO

        if fast_score >= cfg.fast_score_min and slow_score < cfg.fast_score_min:
            decision = AdaCoTDecision.FAST
        elif slow_score >= cfg.slow_score_min and fast_score < cfg.slow_score_min:
            decision = AdaCoTDecision.SLOW

        self._stats[decision.value] += 1
        logger.debug(
            "AdaCoT: fast=%.2f slow=%.2f -> %s | tokens=%d spatial=%d neg=%s compound=%s objs=%d same_label=%d",
            fast_score,
            slow_score,
            decision.value,
            n_tokens,
            n_spatial,
            has_negation,
            has_compound,
            n_objects,
            max_same_label,
        )
        return decision

    @property
    def stats(self) -> dict[str, int]:
        """Stats."""
        return dict(self._stats)
