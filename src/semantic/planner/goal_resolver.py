"""
目标解析器 — 将自然语言指令 + 场景图 → 3D 目标坐标。

核心流程 (VLingNav 2026 双进程 + ESCA 选择性 Grounding):

  ┌─ Fast Path (System 1, 无需 LLM):
  │   场景图直接匹配 → 高置信度 → 直接输出坐标
  │   参考: VLingNav (arXiv 2601.08665) AdaCoT 机制
  │         OmniNav (ICLR 2026) Fast-Slow 系统
  │
  └─ Slow Path (System 2, 调用 LLM):
      ESCA 选择性 Grounding → 过滤场景图 → LLM 推理
      参考: ESCA/SGCLIP (NeurIPS 2025) 选择性 grounding
            AdaNav (ICLR 2026) 不确定性自适应
      可选: GPT-4o Vision 视觉确认 (VLMnav 2024)

为什么 Fast Path 重要:
  - VLingNav 发现 70%+ 的导航步骤用 System 1 即可完成
  - OmniNav 的 Fast 模块支持 5 Hz 控制频率
  - 省去 LLM API 调用 → 延迟从 ~2s 降到 ~10ms, API 费用降低 90%

模块结构:
  goal_resolver.py  — GoalResolver 主类 + 数据类 (本文件)
  fast_path.py      — FastPathMixin: Fast Path 场景图匹配方法
  slow_path.py      — SlowPathMixin: Slow Path LLM 推理方法
"""

import logging
import math
from dataclasses import dataclass


from core.runtime_interface import map_frame_id

from .adacot import AdaCoTRouter
from .fast_path import FastPathMixin
from .llm_client import LLMConfig, create_llm_client
from .slow_path import SlowPathMixin
from memory.spatial.tagged_locations import TaggedLocationStore

try:
    from memory.spatial.topology_graph import TopologySemGraph
except ImportError:
    TopologySemGraph = None

logger = logging.getLogger(__name__)
GOAL_RESOLVER_MAP_FRAME_ID = map_frame_id()


# ── 多源置信度权重 (AdaNav 不确定性融合) ──
WEIGHT_LABEL_MATCH = 0.35       # 标签文本匹配
WEIGHT_CLIP_SIM = 0.35          # CLIP 视觉-语言相似度
WEIGHT_DETECTOR_SCORE = 0.15    # 检测器置信度
WEIGHT_SPATIAL_HINT = 0.15      # 空间关系提示命中


@dataclass
class GoalResult:
    """目标解析结果。"""
    action: str                    # "navigate" | "explore"
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_label: str = ""
    confidence: float = 0.0
    reasoning: str = ""
    is_valid: bool = False
    error: str = ""
    path: str = ""                 # "fast" | "slow" — 标记走了哪条路径
    candidate_id: int = -1         # BA-HSG: 候选物体 ID
    frame_id: str = GOAL_RESOLVER_MAP_FRAME_ID
    hint_room: str = ""                            # OmniNav: 目标所在推测房间名
    hint_room_center: list[float] | None = None # OmniNav: 房间中心坐标 [x,y,z]
    score_entropy: float = 0.0                     # AdaNav: Fast Path 得分熵


@dataclass
class TargetHypothesis:
    """BA-HSG 多假设目标信念 (§3.4.3)。"""
    object_id: int
    label: str
    position: list[float]          # [x, y, z]
    score: float                   # Fast Path fused score
    credibility: float             # BA-HSG composite credibility
    room_match: float              # room-instruction compatibility
    posterior: float = 0.0         # P(this is the true target | history)
    verified: bool = False         # 是否已到达验证过
    rejected: bool = False         # 是否已被拒绝


class TargetBeliefManager:
    """BA-HSG 多假设目标信念管理器 (§3.4.3)。

    维护候选目标的后验分布，支持贝叶斯更新和期望代价选择。
    解决多实例歧义问题 (e.g., 多把椅子, 多个门)。
    """

    def __init__(self, gamma1: float = 1.0, gamma2: float = 0.5, gamma3: float = 0.3) -> None:
        self._hypotheses: list[TargetHypothesis] = []
        self._gamma1 = gamma1  # fused score weight
        self._gamma2 = gamma2  # credibility weight
        self._gamma3 = gamma3  # room match weight
        self._accept_threshold = 0.7  # 后验阈值 → 确认目标

    def init_from_candidates(
        self,
        candidates: list[dict],
        instruction: str = "",
    ) -> None:
        """从 Fast Path 候选列表初始化后验。"""
        self._hypotheses = []
        for c in candidates:
            belief = c.get("belief", {})
            h = TargetHypothesis(
                object_id=c.get("id", -1),
                label=c.get("label", ""),
                position=c.get("position", [0, 0, 0]),
                score=c.get("fused_score", 0.5),
                credibility=belief.get("credibility", 0.5) if isinstance(belief, dict) else 0.5,
                room_match=c.get("room_match", 0.5),
            )
            self._hypotheses.append(h)
        self._compute_posterior()

    def _compute_posterior(self) -> None:
        """计算归一化后验: p_i ∝ exp(γ1·score + γ2·cred + γ3·room)。"""
        if not self._hypotheses:
            return
        log_scores = []
        for h in self._hypotheses:
            if h.rejected:
                log_scores.append(-100.0)
            else:
                log_scores.append(
                    self._gamma1 * h.score
                    + self._gamma2 * h.credibility
                    + self._gamma3 * h.room_match
                )
        max_ls = max(log_scores)
        exp_scores = [math.exp(ls - max_ls) for ls in log_scores]
        total = sum(exp_scores) or 1.0
        for h, es in zip(self._hypotheses, exp_scores):
            h.posterior = es / total

    def bayesian_update(self, object_id: int, detected: bool, clip_sim: float = 0.5) -> None:
        """贝叶斯验证更新: 到达候选附近后观测结果 (§3.4.3)。"""
        for h in self._hypotheses:
            if h.object_id == object_id:
                if detected and clip_sim > 0.7:
                    likelihood = 0.9
                elif detected:
                    likelihood = 0.4 + 0.3 * clip_sim
                else:
                    likelihood = 0.1
                    h.rejected = True
                h.posterior *= likelihood
                h.verified = True
            else:
                # 其他候选: 如果目标被确认在 object_id, 其他后验降低
                if detected and clip_sim > 0.7:
                    h.posterior *= 0.3
        # 重新归一化
        total = sum(h.posterior for h in self._hypotheses) or 1.0
        for h in self._hypotheses:
            h.posterior /= total

    def select_next_target(
        self,
        robot_position: list[float] | None = None,
        beta: float = 0.5,
        rho: float = 0.2,
    ) -> TargetHypothesis | None:
        """期望代价选择: argmin(nav_cost - β·posterior + ρ·info_gain) (§3.4.3)。"""
        active = [h for h in self._hypotheses if not h.rejected and not h.verified]
        if not active:
            # 所有候选已验证或拒绝 → 返回后验最高的已验证候选
            verified = [h for h in self._hypotheses if h.verified and not h.rejected]
            return max(verified, key=lambda h: h.posterior) if verified else None

        best = None
        best_utility = -float("inf")
        for h in active:
            nav_cost = 0.0
            if robot_position:
                dx = h.position[0] - robot_position[0]
                dy = h.position[1] - robot_position[1]
                nav_cost = (dx ** 2 + dy ** 2) ** 0.5
            # 信息增益: 到达该候选后能区分其他候选的能力
            info_gain = 0.0
            for h2 in active:
                if h2.object_id != h.object_id:
                    d = sum((a - b) ** 2 for a, b in zip(h.position, h2.position)) ** 0.5
                    if d < 3.0:
                        info_gain += 0.3  # 附近有其他候选 → 一次验证可区分多个
            utility = beta * h.posterior - nav_cost / 10.0 + rho * info_gain
            if utility > best_utility:
                best_utility = utility
                best = h
        return best

    @property
    def best_hypothesis(self) -> TargetHypothesis | None:
        """后验最高的未拒绝假设。"""
        active = [h for h in self._hypotheses if not h.rejected]
        return max(active, key=lambda h: h.posterior) if active else None

    @property
    def is_converged(self) -> bool:
        """后验是否已收敛 (最高后验 > 阈值)。"""
        best = self.best_hypothesis
        return best is not None and best.posterior > self._accept_threshold

    @property
    def num_active(self) -> int:
        return sum(1 for h in self._hypotheses if not h.rejected)


class GoalResolver(FastPathMixin, SlowPathMixin):
    """
    目标解析器: VLingNav 双进程 + ESCA 选择性 Grounding + AdaNav 置信度融合。

    实现通过多继承组合:
      FastPathMixin — 场景图直接匹配 (System 1, <200ms)
      SlowPathMixin — LLM 推理 + 选择性 Grounding (System 2, ~2s)
    """

    def __init__(
        self,
        primary_config: LLMConfig,
        fallback_config: LLMConfig | None = None,
        confidence_threshold: float = 0.6,
        fast_path_threshold: float = 0.75,   # Fast 路径最低置信度
        max_replan_attempts: int = 3,
        tagged_location_store: TaggedLocationStore | None = None,
        save_dir: str = "",
    ) -> None:
        self._primary = create_llm_client(primary_config)
        self._fallback = (
            create_llm_client(fallback_config) if fallback_config else None
        )
        self._confidence_threshold = confidence_threshold
        self._fast_path_threshold = fast_path_threshold
        self._max_replan_attempts = max_replan_attempts

        # Tag 记忆层 (层 0: 用户手动标记的地点)
        self._tag_store: TaggedLocationStore | None = tagged_location_store

        # 探索状态
        self._explored_directions: list[dict[str, float]] = []
        self._explore_step_count = 0
        self._visited_room_ids: set = set()

        # BA-HSG: 多假设目标信念管理器
        self._belief_manager = TargetBeliefManager()

        # 创新4: 语义先验引擎 (拓扑感知探索)
        import os as _os
        import time as _time

        from memory.knowledge.semantic_prior import SemanticPriorEngine
        _kg_path = _os.path.join(save_dir, "room_object_kg.json") if save_dir else None
        self._semantic_prior_engine = SemanticPriorEngine(kg_path=_kg_path)
        self._kg_path = _kg_path
        self._kg_reload_interval = 120.0  # seconds between KG reloads
        self._last_kg_reload = _time.time()

        # P1: 房间-物体知识图谱 (KG-backed room prediction)
        self._room_object_kg = None

        # 创新5: 拓扑语义图 (Topology Semantic Graph)
        self._tsg: TopologySemGraph | None = None
        if TopologySemGraph is not None:
            self._tsg = TopologySemGraph()

        # AdaCoT: 动态快慢路径路由 (VLingNav 2026)
        self._adacot = AdaCoTRouter()

    # ================================================================
    #  KG hot-reload (SemanticMapperModule saves every 30s)
    # ================================================================

    def maybe_reload_kg(self) -> None:
        """Reload learned KG priors if the file has been updated recently."""
        import os
        import time
        if not self._kg_path or not os.path.exists(self._kg_path):
            return
        now = time.time()
        if now - self._last_kg_reload < self._kg_reload_interval:
            return
        self._last_kg_reload = now
        if self._semantic_prior_engine.load_learned_priors(self._kg_path):
            logger.debug("GoalResolver: KG priors hot-reloaded from %s", self._kg_path)

    # ================================================================
    #  Tag 记忆层 (层 0: 精确/模糊匹配用户标记地点)
    # ================================================================

    def _resolve_by_tag(self, instruction: str) -> GoalResult | None:
        """Tag 记忆查询：精确匹配优先，然后模糊匹配。

        Args:
            instruction: 用户自然语言指令

        Returns:
            GoalResult (path="tag", confidence=1.0) 或 None
        """
        if self._tag_store is None:
            return None

        # 精确匹配
        entry = self._tag_store.query(instruction)
        if entry is None:
            # 模糊匹配 (标签名包含于指令，或指令包含于标签名)
            entry = self._tag_store.query_fuzzy(instruction)

        if entry is None:
            return None

        pos = entry["position"]
        name = entry["name"]
        logger.info("[Tag层] 命中标签地点 '%s' at (%.2f, %.2f, %.2f)", name, pos[0], pos[1], pos[2])
        return GoalResult(
            action="navigate",
            target_x=float(pos[0]),
            target_y=float(pos[1]),
            target_z=float(pos[2]),
            target_label=name,
            confidence=1.0,
            reasoning=f"Tag memory exact/fuzzy match: '{name}'",
            is_valid=True,
            path="tag",
        )

    # ================================================================
    #  BA-HSG: 多假设验证与重选 (§3.4.3)
    # ================================================================

    def verify_and_reselect(
        self,
        object_id: int,
        detected: bool,
        clip_sim: float = 0.5,
        robot_position: list[float] | None = None,
    ) -> GoalResult | None:
        """到达候选目标后执行贝叶斯更新, 如需要则重选目标。

        Args:
            object_id: 当前验证的物体 ID
            detected: 到达后是否检测到目标
            clip_sim: CLIP 相似度
            robot_position: 当前机器人位置 [x, y]

        Returns:
            新的 GoalResult 如果需要重选, 否则 None (当前目标确认)
        """
        if not self._belief_manager._hypotheses:
            return None

        self._belief_manager.bayesian_update(object_id, detected, clip_sim)

        best = self._belief_manager.best_hypothesis
        if best is None:
            return None

        if self._belief_manager.is_converged and best.verified:
            logger.info(
                "BA-HSG belief converged: '%s' posterior=%.3f",
                best.label, best.posterior,
            )
            return None

        # 需要验证下一个候选
        next_target = self._belief_manager.select_next_target(robot_position)
        if next_target is None:
            return None

        logger.info(
            "BA-HSG reselect: switching to '%s' (posterior=%.3f, %d active)",
            next_target.label, next_target.posterior,
            self._belief_manager.num_active,
        )
        return GoalResult(
            action="navigate",
            target_x=next_target.position[0],
            target_y=next_target.position[1],
            target_z=next_target.position[2],
            target_label=next_target.label,
            confidence=next_target.posterior,
            reasoning=f"BA-HSG reselect: posterior={next_target.posterior:.3f}",
            is_valid=True,
            path="fast",
            candidate_id=next_target.object_id,
            frame_id=GOAL_RESOLVER_MAP_FRAME_ID,
        )
