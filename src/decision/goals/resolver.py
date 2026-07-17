"""Fast/slow natural-language goal resolution."""

import logging
import math
from dataclasses import dataclass
from typing import Any

from decision.llm.client import LLMConfig, create_llm_client
from memory.spatial.tagged_locations import TaggedLocationStore
from runtime.runtime_interface import map_frame_id

from .fast import FastPathMixin
from .router import AdaCoTRouter
from .scene_cache import SceneGraphCache
from .slow import SlowPathMixin

try:
    from memory.spatial.topology_graph import TopologySemGraph
except ImportError:
    TopologySemGraph = None

logger = logging.getLogger(__name__)
GOAL_RESOLVER_MAP_FRAME_ID = map_frame_id()


WEIGHT_LABEL_MATCH = 0.35
WEIGHT_CLIP_SIM = 0.35
WEIGHT_DETECTOR_SCORE = 0.15
WEIGHT_SPATIAL_HINT = 0.15


@dataclass
class GoalResult:
    """Goal Result."""

    action: str  # "navigate" | "explore"
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_label: str = ""
    confidence: float = 0.0
    reasoning: str = ""
    is_valid: bool = False
    error: str = ""
    path: str = ""
    candidate_id: int = -1
    frame_id: str = GOAL_RESOLVER_MAP_FRAME_ID
    hint_room: str = ""
    hint_room_center: list[float] | None = None
    score_entropy: float = 0.0


@dataclass
class TargetHypothesis:
    """Target Hypothesis."""

    object_id: int
    label: str
    position: list[float]  # [x, y, z]
    score: float  # Fast Path fused score
    credibility: float  # BA-HSG composite credibility
    room_match: float  # room-instruction compatibility
    posterior: float = 0.0  # P(this is the true target | history)
    verified: bool = False
    rejected: bool = False


class TargetBeliefManager:
    """Target Belief Manager."""

    def __init__(self, gamma1: float = 1.0, gamma2: float = 0.5, gamma3: float = 0.3) -> None:
        self._hypotheses: list[TargetHypothesis] = []
        self._gamma1 = gamma1  # fused score weight
        self._gamma2 = gamma2  # credibility weight
        self._gamma3 = gamma3  # room match weight
        self._accept_threshold = 0.7

    def init_from_candidates(
        self,
        candidates: list[dict],
        instruction: str = "",
    ) -> None:
        """Init from candidates."""
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
        """Compute posterior."""
        if not self._hypotheses:
            return
        log_scores = []
        for h in self._hypotheses:
            if h.rejected:
                log_scores.append(-100.0)
            else:
                log_scores.append(self._gamma1 * h.score + self._gamma2 * h.credibility + self._gamma3 * h.room_match)
        max_ls = max(log_scores)
        exp_scores = [math.exp(ls - max_ls) for ls in log_scores]
        total = sum(exp_scores) or 1.0
        for h, es in zip(self._hypotheses, exp_scores):
            h.posterior = es / total

    def bayesian_update(self, object_id: int, detected: bool, clip_sim: float = 0.5) -> None:
        """Bayesian update."""
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
                if detected and clip_sim > 0.7:
                    h.posterior *= 0.3

        total = sum(h.posterior for h in self._hypotheses) or 1.0
        for h in self._hypotheses:
            h.posterior /= total

    def select_next_target(
        self,
        robot_position: list[float] | None = None,
        beta: float = 0.5,
        rho: float = 0.2,
    ) -> TargetHypothesis | None:
        """Select next target."""
        active = [h for h in self._hypotheses if not h.rejected and not h.verified]
        if not active:
            verified = [h for h in self._hypotheses if h.verified and not h.rejected]
            return max(verified, key=lambda h: h.posterior) if verified else None

        best = None
        best_utility = -float("inf")
        for h in active:
            nav_cost = 0.0
            if robot_position:
                dx = h.position[0] - robot_position[0]
                dy = h.position[1] - robot_position[1]
                nav_cost = (dx**2 + dy**2) ** 0.5

            info_gain = 0.0
            for h2 in active:
                if h2.object_id != h.object_id:
                    d = sum((a - b) ** 2 for a, b in zip(h.position, h2.position)) ** 0.5
                    if d < 3.0:
                        info_gain += 0.3
            utility = beta * h.posterior - nav_cost / 10.0 + rho * info_gain
            if utility > best_utility:
                best_utility = utility
                best = h
        return best

    @property
    def best_hypothesis(self) -> TargetHypothesis | None:
        """Best hypothesis."""
        active = [h for h in self._hypotheses if not h.rejected]
        return max(active, key=lambda h: h.posterior) if active else None

    @property
    def is_converged(self) -> bool:
        """Is converged."""
        best = self.best_hypothesis
        return best is not None and best.posterior > self._accept_threshold

    @property
    def num_active(self) -> int:
        return sum(1 for h in self._hypotheses if not h.rejected)


class GoalResolver(FastPathMixin, SlowPathMixin):
    """Goal Resolver."""

    def __init__(
        self,
        primary_config: LLMConfig,
        fallback_config: LLMConfig | None = None,
        confidence_threshold: float = 0.6,
        fast_path_threshold: float = 0.75,
        max_replan_attempts: int = 3,
        tagged_location_store: TaggedLocationStore | None = None,
        save_dir: str = "",
    ) -> None:
        self._primary = create_llm_client(primary_config)
        self._fallback = create_llm_client(fallback_config) if fallback_config else None
        self._confidence_threshold = confidence_threshold
        self._fast_path_threshold = fast_path_threshold
        self._max_replan_attempts = max_replan_attempts

        self._tag_store: TaggedLocationStore | None = tagged_location_store

        self._explored_directions: list[dict[str, float]] = []
        self._explore_step_count = 0
        self._visited_room_ids: set = set()

        self._belief_manager = TargetBeliefManager()

        # Scene graph parse cache: avoids re-parsing identical scene graphs.
        # Only caches the instruction-independent JSON->dict step.
        self._scene_cache = SceneGraphCache(ttl_sec=2.0)

        import os as _os
        import time as _time

        from memory.knowledge.semantic_prior import SemanticPriorEngine

        _kg_path = _os.path.join(save_dir, "room_object_kg.json") if save_dir else None
        self._semantic_prior_engine = SemanticPriorEngine(kg_path=_kg_path)
        self._kg_path = _kg_path
        self._kg_reload_interval = 120.0  # seconds between KG reloads
        self._last_kg_reload = _time.time()

        self._room_object_kg = None

        self._tsg: TopologySemGraph | None = None
        if TopologySemGraph is not None:
            self._tsg = TopologySemGraph()

        self._adacot = AdaCoTRouter()

        # Decision metrics for observability (do not affect resolution logic).
        self._fast_path_attempts: int = 0
        self._fast_path_hits: int = 0
        self._slow_path_invocations: int = 0
        self._resolve_times: list[float] = []
        self._max_resolve_history: int = 100

    def get_metrics(self) -> dict[str, Any]:
        """Return decision metrics for health reporting."""
        fast_hit_rate = self._fast_path_hits / self._fast_path_attempts if self._fast_path_attempts > 0 else 0.0
        avg_resolve_ms = sum(self._resolve_times) / len(self._resolve_times) if self._resolve_times else 0.0
        return {
            "fast_path_hit_rate": round(fast_hit_rate, 3),
            "fast_path_attempts": self._fast_path_attempts,
            "slow_path_invocations": self._slow_path_invocations,
            "avg_resolve_ms": round(avg_resolve_ms, 1),
        }

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

    # ================================================================

    def _resolve_by_tag(self, instruction: str) -> GoalResult | None:
        """Resolve by tag."""
        if self._tag_store is None:
            return None

        entry = self._tag_store.query(instruction)
        if entry is None:
            entry = self._tag_store.query_fuzzy(instruction)

        if entry is None:
            return None

        pos = entry["position"]
        name = entry["name"]
        logger.info("[Tag] matched saved location '%s' at (%.2f, %.2f, %.2f)", name, pos[0], pos[1], pos[2])
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

    # ================================================================

    def verify_and_reselect(
        self,
        object_id: int,
        detected: bool,
        clip_sim: float = 0.5,
        robot_position: list[float] | None = None,
    ) -> GoalResult | None:
        """Verify and reselect."""
        if not self._belief_manager._hypotheses:
            return None

        self._belief_manager.bayesian_update(object_id, detected, clip_sim)

        best = self._belief_manager.best_hypothesis
        if best is None:
            return None

        if self._belief_manager.is_converged and best.verified:
            logger.info(
                "BA-HSG belief converged: '%s' posterior=%.3f",
                best.label,
                best.posterior,
            )
            return None

        next_target = self._belief_manager.select_next_target(robot_position)
        if next_target is None:
            return None

        logger.info(
            "BA-HSG reselect: switching to '%s' (posterior=%.3f, %d active)",
            next_target.label,
            next_target.posterior,
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
