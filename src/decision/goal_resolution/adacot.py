"""
AdaCoT (Adaptive Chain-of-Thought) 动态快慢路径切换 (VLingNav 2026)。

根据指令复杂度和场景图状态, 预判应该走 Fast Path 还是 Slow Path,
避免: (1) 简单指令浪费 LLM 调用; (2) 复杂指令 Fast Path 错误匹配。

判断维度:
  1. 指令复杂度 — 子句数、空间修饰语数、否定词
  2. 场景图丰富度 — 物体数、关系数、同类物体数
  3. 匹配歧义度 — 候选数、top-2 分差

输出:
  - FAST:  直接走 Fast Path, 跳过 Slow Path
  - SLOW:  直接走 Slow Path, 跳过 Fast Path
  - AUTO:  走现有 Fast → Slow 流程
"""

import logging
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class AdaCoTDecision(Enum):
    """AdaCoT 路径决策。"""
    FAST = "fast"    # 简单场景, 直走 Fast Path
    SLOW = "slow"    # 复杂场景, 直走 Slow Path
    AUTO = "auto"    # 不确定, 走默认 Fast→Slow


# 空间修饰词 (中英文)
_SPATIAL_MODIFIERS_ZH = {
    "旁边", "附近", "左边", "右边", "前面", "后面", "上面", "下面",
    "里面", "外面", "之间", "对面", "远离", "靠近", "中间",
}
_SPATIAL_MODIFIERS_EN = {
    "near", "beside", "next to", "left of", "right of",
    "in front of", "behind", "above", "below", "between",
    "inside", "outside", "opposite", "away from", "close to",
}

# 否定词
_NEGATION_ZH = {"不是", "不要", "除了", "而不是", "非", "别"}
_NEGATION_EN = {"not", "except", "other than", "instead of", "without", "don't"}

# 复合指令标志
_COMPOUND_ZH = {"然后", "接着", "之后", "并且", "先", "再", "最后"}
_COMPOUND_EN = {"then", "after that", "and then", "first", "next", "finally"}


@dataclass
class AdaCoTConfig:
    """AdaCoT 可调参数。"""
    # 指令复杂度阈值
    simple_max_tokens: int = 6          # <= 此 token 数 → 偏简单
    complex_min_spatial: int = 2        # >= 此空间修饰词数 → 偏复杂
    # 场景图阈值
    few_objects_max: int = 5            # <= 此物体数 → Fast 足够
    many_objects_min: int = 30          # >= 此物体数 → 偏 Slow
    # 歧义阈值
    ambiguity_same_label_min: int = 3   # 同标签物体 >= 此数 → 歧义高
    # 综合分数阈值
    fast_score_min: float = 0.6         # >= 此分 → 推荐 FAST
    slow_score_min: float = 0.6         # >= 此分 → 推荐 SLOW


class AdaCoTRouter:
    """AdaCoT 路径路由器。

    基于指令复杂度 + 场景图状态, 快速预判走哪条路径。
    设计目标: 计算量 < 1ms, 不调用任何模型。
    """

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
        """根据指令和场景图状态决定路径。

        Args:
            instruction: 用户自然语言指令
            scene_graph: 解析后的场景图 dict (可选)
            keywords: 已提取的关键词 (可选, 避免重复计算)
            score_entropy: Fast Path 候选得分熵 (AdaNav, 0=未计算)

        Returns:
            AdaCoTDecision
        """
        # AdaNav: 高熵强制 SLOW (候选得分分布均匀 → 歧义高)
        if score_entropy > 1.5:
            self._stats["slow"] += 1
            logger.debug(
                "AdaCoT: high score_entropy=%.2f → SLOW", score_entropy,
            )
            return AdaCoTDecision.SLOW

        cfg = self._config

        # ── 1. 指令复杂度特征 ──
        inst_lower = instruction.lower().strip()
        tokens = inst_lower.split()
        n_tokens = len(tokens)

        # 空间修饰词计数
        n_spatial = sum(1 for w in _SPATIAL_MODIFIERS_ZH if w in instruction)
        n_spatial += sum(1 for w in _SPATIAL_MODIFIERS_EN if w in inst_lower)

        # 否定词
        has_negation = (
            any(w in instruction for w in _NEGATION_ZH)
            or any(w in inst_lower for w in _NEGATION_EN)
        )

        # 复合指令 (多步)
        has_compound = (
            any(w in instruction for w in _COMPOUND_ZH)
            or any(w in inst_lower for w in _COMPOUND_EN)
        )

        # ── 2. 场景图特征 ──
        n_objects = 0
        n_relations = 0
        max_same_label = 0

        if scene_graph:
            objects = scene_graph.get("objects", [])
            n_objects = len(objects)
            n_relations = len(scene_graph.get("relations", []))

            # 同标签物体计数
            label_counts: dict[str, int] = {}
            for obj in objects:
                lbl = str(obj.get("label", "")).lower()
                if lbl:
                    label_counts[lbl] = label_counts.get(lbl, 0) + 1
            if label_counts:
                max_same_label = max(label_counts.values())

        # ── 3. 综合评分 ──
        fast_score = 0.0
        slow_score = 0.0

        # 简短指令 → Fast
        if n_tokens <= cfg.simple_max_tokens:
            fast_score += 0.3
        elif n_tokens > cfg.simple_max_tokens * 2:
            slow_score += 0.2

        # 少空间修饰 → Fast; 多空间修饰 → Slow
        if n_spatial == 0:
            fast_score += 0.2
        elif n_spatial >= cfg.complex_min_spatial:
            slow_score += 0.3

        # 否定句 → Slow (需要推理排除)
        if has_negation:
            slow_score += 0.3

        # 复合指令 → Slow (需要分解)
        if has_compound:
            slow_score += 0.3

        # 物体少 → Fast; 物体多 → Slow (搜索空间大)
        if 0 < n_objects <= cfg.few_objects_max:
            fast_score += 0.2
        elif n_objects >= cfg.many_objects_min:
            slow_score += 0.2

        # 高歧义 → Slow
        if max_same_label >= cfg.ambiguity_same_label_min:
            slow_score += 0.2

        # 有关系数据 + 有空间修饰 → Slow (关系推理)
        if n_relations > 0 and n_spatial > 0:
            slow_score += 0.1

        # ── 4. 决策 ──
        decision = AdaCoTDecision.AUTO

        if fast_score >= cfg.fast_score_min and slow_score < cfg.fast_score_min:
            decision = AdaCoTDecision.FAST
        elif slow_score >= cfg.slow_score_min and fast_score < cfg.slow_score_min:
            decision = AdaCoTDecision.SLOW

        self._stats[decision.value] += 1
        logger.debug(
            "AdaCoT: fast=%.2f slow=%.2f → %s | tokens=%d spatial=%d "
            "neg=%s compound=%s objs=%d same_label=%d",
            fast_score, slow_score, decision.value,
            n_tokens, n_spatial, has_negation, has_compound,
            n_objects, max_same_label,
        )
        return decision

    @property
    def stats(self) -> dict[str, int]:
        """路由统计。"""
        return dict(self._stats)
