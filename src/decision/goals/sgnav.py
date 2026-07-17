"""Scene-graph navigation scoring helpers."""

import json
import logging
import math
import re
import threading
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from decision.frontiers.types import Frontier
from decision.llm.prompts import build_sgnav_subgraph_prompt

logger = logging.getLogger(__name__)

# Loaded from config/semantic_scoring.yaml [sgnav_heuristic] on first call.
_DEFAULTS_SGNAV: dict = {
    "keyword": 0.55,
    "relation": 0.20,
    "distance": 0.15,
    "richness": 0.10,
}
_SGNAV_WEIGHTS: dict = dict(_DEFAULTS_SGNAV)
_sgnav_weights_loaded: bool = False


def _load_semantic_scoring_yaml() -> dict:
    """Load config/semantic_scoring.yaml from the repo root."""
    repo_root = Path(__file__).resolve().parents[3]
    yaml_path = repo_root / "config" / "semantic_scoring.yaml"
    try:
        with open(yaml_path, encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except (FileNotFoundError, OSError):
        return {}


def _load_sgnav_weights() -> None:
    """Load sgnav_heuristic weights from config/semantic_scoring.yaml (once)."""
    global _SGNAV_WEIGHTS, _sgnav_weights_loaded
    if _sgnav_weights_loaded:
        return
    _sgnav_weights_loaded = True
    try:
        section = _load_semantic_scoring_yaml().get("sgnav_heuristic")
        if section is None:
            logger.info(
                "sgnav_heuristic section absent — using default weights. See config/semantic_scoring.yaml to tune."
            )
            return
        _SGNAV_WEIGHTS["keyword"] = float(section.get("keyword", _DEFAULTS_SGNAV["keyword"]))
        _SGNAV_WEIGHTS["relation"] = float(section.get("relation", _DEFAULTS_SGNAV["relation"]))
        _SGNAV_WEIGHTS["distance"] = float(section.get("distance", _DEFAULTS_SGNAV["distance"]))
        _SGNAV_WEIGHTS["richness"] = float(section.get("richness", _DEFAULTS_SGNAV["richness"]))
        logger.debug(
            "sgnav_heuristic weights loaded: keyword=%.2f relation=%.2f distance=%.2f richness=%.2f",
            _SGNAV_WEIGHTS["keyword"],
            _SGNAV_WEIGHTS["relation"],
            _SGNAV_WEIGHTS["distance"],
            _SGNAV_WEIGHTS["richness"],
        )
    except Exception as exc:
        logger.warning("Failed to load sgnav_heuristic weights from config: %s", exc)


@dataclass
class SubgraphCandidate:
    """Subgraphcandidate."""

    subgraph_id: str
    level: str
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    room_id: int = -1
    group_id: int = -1
    object_ids: list[int] = field(default_factory=list)
    object_labels: list[str] = field(default_factory=list)
    relation_count: int = 0

    def to_prompt_dict(self) -> dict:
        return {
            "subgraph_id": self.subgraph_id,
            "level": self.level,
            "room_id": self.room_id,
            "group_id": self.group_id,
            "center": {
                "x": round(float(self.center[0]), 2),
                "y": round(float(self.center[1]), 2),
            },
            "object_count": len(self.object_ids),
            "object_labels": self.object_labels[:10],
            "relation_count": self.relation_count,
        }


@dataclass
class SubgraphScore:
    """Subgraphscore."""

    subgraph_id: str
    level: str
    score: float
    reason: str
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))


@dataclass
class ObservationRecord:
    """Observationrecord."""

    timestamp: float
    match_count: int
    max_score: float
    belief_credibility: float
    path_confidence: float
    confirmed_visible: bool


@dataclass
class FrontierSelection:
    """Frontierselection."""

    frontier: Frontier
    score: float
    reasoning: str
    subgraph_scores: list[SubgraphScore] = field(default_factory=list)


class SGNavReasoner:
    """Sgnavreasoner."""

    def __init__(
        self,
        max_subgraphs: int = 6,
        use_llm_reasoning: bool = True,
        heuristic_weight: float = 0.45,
        llm_weight: float = 0.55,
        frontier_base_weight: float = 0.55,
        room_gate_weight: float = 0.25,
        interp_decay_distance: float = 4.0,
        credibility_decay: float = 0.9,
        false_positive_penalty: float = 0.2,
        reject_threshold: float = 0.25,
        min_confidence_for_bypass: float = 0.85,
        reperception_n_max: int = 10,
        reperception_s_thresh: float = 0.8,
    ):
        self.max_subgraphs = max_subgraphs
        self.use_llm_reasoning = use_llm_reasoning
        self.heuristic_weight = heuristic_weight
        self.llm_weight = llm_weight
        self.frontier_base_weight = frontier_base_weight
        self.room_gate_weight = min(max(room_gate_weight, 0.0), 1.0)
        self.interp_decay_distance = max(0.5, interp_decay_distance)

        self.credibility_decay = min(max(credibility_decay, 0.0), 0.999)
        self.false_positive_penalty = min(max(false_positive_penalty, 0.0), 0.95)
        self.reject_threshold = min(max(reject_threshold, 0.01), 0.99)
        self.min_confidence_for_bypass = min(max(min_confidence_for_bypass, 0.0), 1.0)

        self.reperception_n_max = max(1, reperception_n_max)
        self.reperception_s_thresh = min(max(reperception_s_thresh, 0.0), 1.0)

        self._target_credibility: dict[str, float] = {}
        self._observation_history: dict[str, list[ObservationRecord]] = {}
        self._cred_lock = threading.Lock()

    @property
    def target_credibility(self) -> dict[str, float]:
        with self._cred_lock:
            return dict(self._target_credibility)

    def reset(self):
        """Reset."""
        with self._cred_lock:
            self._target_credibility.clear()
            self._observation_history.clear()

    async def select_frontier(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: dict[str, float],
        frontiers: list[Frontier],
        language: str = "zh",
        llm_chat: Callable[[list[dict[str, str]]], Awaitable[str | None]] | None = None,
        frontier_descriptions: list[str] | None = None,
        explored_summaries: list[str] | None = None,
    ) -> FrontierSelection | None:
        """Select frontier."""
        if not frontiers:
            return None

        subgraph_scores = await self.reason_subgraphs(
            instruction=instruction,
            scene_graph_json=scene_graph_json,
            robot_position=robot_position,
            language=language,
            llm_chat=llm_chat,
            frontier_descriptions=frontier_descriptions,
            explored_summaries=explored_summaries,
        )

        best: Frontier | None = None
        best_score = -1.0
        best_details = ""

        for frontier in frontiers:
            sg_signal, top_reason = self._interpolate_to_frontier(frontier, subgraph_scores)
            combined = self.frontier_base_weight * float(frontier.score) + (1.0 - self.frontier_base_weight) * sg_signal
            if combined > best_score:
                best_score = combined
                best = frontier
                best_details = f"base={frontier.score:.2f}, sg={sg_signal:.2f}, combined={combined:.2f}; {top_reason}"

        if best is None:
            return None

        return FrontierSelection(
            frontier=best,
            score=float(best_score),
            reasoning=best_details,
            subgraph_scores=subgraph_scores,
        )

    async def reason_subgraphs(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: dict[str, float],
        language: str = "zh",
        llm_chat: Callable[[list[dict[str, str]]], Awaitable[str | None]] | None = None,
        frontier_descriptions: list[str] | None = None,
        explored_summaries: list[str] | None = None,
    ) -> list[SubgraphScore]:
        """Reason subgraphs."""
        candidates = self._extract_subgraphs(scene_graph_json)
        if not candidates:
            return []

        heuristic_scores, heuristic_reasons = self._score_subgraphs_heuristic(
            instruction=instruction,
            candidates=candidates,
            robot_position=robot_position,
        )

        llm_scores: dict[str, float] = {}
        if self.use_llm_reasoning and llm_chat is not None:
            llm_scores = await self._score_subgraphs_llm(
                instruction=instruction,
                candidates=candidates,
                language=language,
                llm_chat=llm_chat,
                frontier_descriptions=frontier_descriptions,
                explored_summaries=explored_summaries,
            )

        outputs: list[SubgraphScore] = []
        for c in candidates:
            h = heuristic_scores.get(c.subgraph_id, 0.0)
            llm_val = llm_scores.get(c.subgraph_id)
            if llm_val is None:
                score = h
                reason = f"heuristic={h:.2f}; {heuristic_reasons.get(c.subgraph_id, '')}"
            else:
                score = self.heuristic_weight * h + self.llm_weight * llm_val
                reason = f"heuristic={h:.2f}, llm={llm_val:.2f}; {heuristic_reasons.get(c.subgraph_id, '')}"

            outputs.append(
                SubgraphScore(
                    subgraph_id=c.subgraph_id,
                    level=c.level,
                    score=float(min(max(score, 0.0), 1.0)),
                    reason=reason,
                    center=c.center,
                )
            )

        outputs.sort(key=lambda s: s.score, reverse=True)
        outputs = self._apply_room_level_gating(outputs, candidates)
        outputs.sort(key=lambda s: s.score, reverse=True)
        return outputs

    def _apply_room_level_gating(
        self,
        outputs: list[SubgraphScore],
        candidates: list[SubgraphCandidate],
    ) -> list[SubgraphScore]:
        """Apply room level gating."""
        if not outputs or self.room_gate_weight <= 0.0:
            return outputs

        room_score_by_room_id: dict[int, float] = {}
        candidate_map = {c.subgraph_id: c for c in candidates}

        for s in outputs:
            c = candidate_map.get(s.subgraph_id)
            if c is None:
                continue
            if s.level == "room" and c.room_id >= 0:
                room_score_by_room_id[c.room_id] = max(
                    room_score_by_room_id.get(c.room_id, 0.0),
                    s.score,
                )

        if not room_score_by_room_id:
            return outputs

        adjusted: list[SubgraphScore] = []
        for s in outputs:
            c = candidate_map.get(s.subgraph_id)
            if c is None or s.level == "room" or c.room_id < 0:
                adjusted.append(s)
                continue

            room_score = room_score_by_room_id.get(c.room_id)
            if room_score is None:
                adjusted.append(s)
                continue

            gated = (1.0 - self.room_gate_weight) * s.score + self.room_gate_weight * room_score
            s.score = float(min(max(gated, 0.0), 1.0))
            s.reason = f"{s.reason}; room_gate={room_score:.2f}"
            adjusted.append(s)

        return adjusted

    def evaluate_target_credibility(
        self,
        target_label: str,
        scene_graph_json: str,
        path_confidence: float,
        confirmed_visible: bool = False,
    ) -> tuple[bool, float, str]:
        """Evaluate target credibility."""
        key = (target_label or "").strip().lower()
        if not key:
            return False, 1.0, "empty target label"

        sg = self._parse_scene_graph(scene_graph_json)
        objects = sg.get("objects", []) if isinstance(sg.get("objects"), list) else []

        tokens = self._extract_keywords(key)
        if not tokens:
            tokens = [key]

        match_count = 0
        max_score = 0.0
        max_belief_credibility = 0.0
        sum_existence_prob = 0.0

        for obj in objects:
            if not isinstance(obj, dict):
                continue
            label = str(obj.get("label", "")).lower()
            if not label:
                continue
            if any(t in label or label in t for t in tokens):
                match_count += 1
                try:
                    max_score = max(max_score, float(obj.get("score", 0.0)))
                except (TypeError, ValueError):
                    pass
                belief = obj.get("belief", {})
                if isinstance(belief, dict):
                    p_exist = belief.get("P_exist", 0.5)
                    obj_cred = belief.get("credibility", 0.5)
                    sum_existence_prob += p_exist
                    max_belief_credibility = max(max_belief_credibility, obj_cred)

        obs = ObservationRecord(
            timestamp=time.monotonic(),
            match_count=match_count,
            max_score=max_score,
            belief_credibility=max_belief_credibility,
            path_confidence=path_confidence,
            confirmed_visible=confirmed_visible,
        )
        with self._cred_lock:
            history = self._observation_history.setdefault(key, [])
            history.append(obs)
            if len(history) > self.reperception_n_max:
                history[:] = history[-self.reperception_n_max :]

        n_obs = len(history)

        if max_belief_credibility > 0:
            avg_exist = sum_existence_prob / max(match_count, 1)
            evidence = (
                0.3 * min(1.0, match_count / 3.0)
                + 0.3 * max_belief_credibility
                + 0.2 * avg_exist
                + 0.2 * min(1.0, max(path_confidence, 0.0))
            )
        else:
            evidence = (
                0.4 * min(1.0, match_count / 3.0)
                + 0.4 * min(1.0, max_score)
                + 0.2 * min(1.0, max(path_confidence, 0.0))
            )

        if confirmed_visible:
            evidence = max(evidence, 0.9)

        with self._cred_lock:
            prev = self._target_credibility.get(key, 0.5)
            cred = prev * self.credibility_decay + evidence * (1.0 - self.credibility_decay)

            if match_count == 0 and not confirmed_visible:
                cred -= self.false_positive_penalty

            cred = min(max(cred, 0.0), 1.0)
            self._target_credibility[key] = cred

        positive_evidences = []
        for h in history:
            if h.match_count > 0 or h.confirmed_visible:
                if h.belief_credibility > 0:
                    avg_e = h.belief_credibility * 0.5 + min(1.0, h.max_score) * 0.3 + min(1.0, h.path_confidence) * 0.2
                else:
                    avg_e = min(1.0, h.max_score) * 0.6 + min(1.0, h.path_confidence) * 0.4
                if h.confirmed_visible:
                    avg_e = max(avg_e, 0.9)
                positive_evidences.append(avg_e)

        n_positive = len(positive_evidences)
        if n_positive > 0:
            s_accum = sum(positive_evidences) / n_positive
        else:
            s_accum = 0.0

        accum_accept = s_accum >= self.reperception_s_thresh and n_positive >= 2

        reject = (
            cred < self.reject_threshold
            and path_confidence < self.min_confidence_for_bypass
            and not confirmed_visible
            and not accum_accept
        )

        reason = (
            f"cred={cred:.2f}, matches={match_count}, max_score={max_score:.2f}, "
            f"belief_cred={max_belief_credibility:.2f}, "
            f"path_conf={path_confidence:.2f}, visible={confirmed_visible}, "
            f"n_obs={n_obs}/{self.reperception_n_max}, "
            f"n_positive={n_positive}, s_accum={s_accum:.2f}/{self.reperception_s_thresh}"
        )
        return reject, cred, reason

    def _extract_subgraphs(self, scene_graph_json: str) -> list[SubgraphCandidate]:
        sg = self._parse_scene_graph(scene_graph_json)

        raw = sg.get("subgraphs", [])
        if isinstance(raw, list) and raw:
            parsed = self._parse_raw_subgraphs(raw)
            if parsed:
                return parsed[: self.max_subgraphs]

        objects = sg.get("objects", []) if isinstance(sg.get("objects", []), list) else []
        relations = sg.get("relations", []) if isinstance(sg.get("relations", []), list) else []
        obj_by_id = {
            int(o.get("id", -1)): o for o in objects if isinstance(o, dict) and isinstance(o.get("id"), (int, float))
        }

        room_like = sg.get("rooms", [])
        if not isinstance(room_like, list) or not room_like:
            room_like = sg.get("regions", []) if isinstance(sg.get("regions", []), list) else []

        candidates: list[SubgraphCandidate] = []

        if isinstance(room_like, list) and room_like:
            for idx, r in enumerate(room_like):
                if not isinstance(r, dict):
                    continue
                object_ids = []
                for oid in r.get("object_ids", []):
                    try:
                        object_ids.append(int(oid))
                    except (TypeError, ValueError):
                        continue

                labels = []
                for oid in object_ids:
                    obj = obj_by_id.get(oid)
                    if obj:
                        labels.append(str(obj.get("label", "")))

                center_xy = self._to_center_xy(r.get("center", {}))
                room_id = self._to_int(r.get("room_id"), default=self._to_int(r.get("region_id"), idx))

                rel_count = 0
                object_id_set = set(object_ids)
                for rel in relations:
                    if not isinstance(rel, dict):
                        continue
                    sid = self._to_int(rel.get("subject_id"), default=-1)
                    oid = self._to_int(rel.get("object_id"), default=-1)
                    if sid in object_id_set and oid in object_id_set:
                        rel_count += 1

                candidates.append(
                    SubgraphCandidate(
                        subgraph_id=f"room_{room_id}",
                        level="room",
                        center=center_xy,
                        room_id=room_id,
                        object_ids=object_ids,
                        object_labels=labels,
                        relation_count=rel_count,
                    )
                )

        if not candidates:
            labels = [str(o.get("label", "")) for o in objects if isinstance(o, dict)]
            obj_ids = [int(o.get("id", i)) for i, o in enumerate(objects) if isinstance(o, dict)]
            center = self._compute_objects_center(objects)
            candidates.append(
                SubgraphCandidate(
                    subgraph_id="room_global",
                    level="room",
                    center=center,
                    room_id=0,
                    object_ids=obj_ids,
                    object_labels=labels,
                    relation_count=len(relations),
                )
            )

        candidates.sort(key=lambda c: (len(c.object_ids), c.relation_count), reverse=True)
        return candidates[: self.max_subgraphs]

    def _parse_raw_subgraphs(self, raw_subgraphs: list[dict]) -> list[SubgraphCandidate]:
        parsed: list[SubgraphCandidate] = []
        for sg in raw_subgraphs:
            if not isinstance(sg, dict):
                continue
            subgraph_id = str(sg.get("subgraph_id", ""))
            if not subgraph_id:
                continue
            labels = sg.get("object_labels", [])
            if not isinstance(labels, list):
                labels = []
            object_ids = sg.get("object_ids", [])
            if not isinstance(object_ids, list):
                object_ids = []

            parsed.append(
                SubgraphCandidate(
                    subgraph_id=subgraph_id,
                    level=str(sg.get("level", "room")),
                    center=self._to_center_xy(sg.get("center", {})),
                    room_id=self._to_int(sg.get("room_id"), default=-1),
                    group_id=self._to_int(sg.get("group_id"), default=-1),
                    object_ids=[self._to_int(x, default=-1) for x in object_ids],
                    object_labels=[str(x) for x in labels if str(x)],
                    relation_count=self._to_int(sg.get("relation_count"), default=0),
                )
            )

        return parsed

    def _score_subgraphs_heuristic(
        self,
        instruction: str,
        candidates: list[SubgraphCandidate],
        robot_position: dict[str, float],
    ) -> tuple[dict[str, float], dict[str, str]]:
        _load_sgnav_weights()  # idempotent, loads from config on first call
        w_kw = _SGNAV_WEIGHTS["keyword"]
        w_rel = _SGNAV_WEIGHTS["relation"]
        w_dist = _SGNAV_WEIGHTS["distance"]
        w_rich = _SGNAV_WEIGHTS["richness"]

        keywords = self._extract_keywords(instruction)
        relation_words = {
            "near",
            "left",
            "right",
            "front",
            "behind",
            "on",
            "above",
            "below",
            "旁",
            "附近",
            "左",
            "右",
            "前",
            "后",
            "上",
            "下",
        }

        robot_xy = np.array(
            [
                float(robot_position.get("x", 0.0)),
                float(robot_position.get("y", 0.0)),
            ],
            dtype=np.float64,
        )

        scores: dict[str, float] = {}
        reasons: dict[str, str] = {}

        for cand in candidates:
            label_text = " ".join(cand.object_labels).lower()

            hit_count = 0
            for kw in keywords:
                if kw in label_text:
                    hit_count += 1

            keyword_score = hit_count / max(len(keywords), 1)
            relation_score = min(1.0, cand.relation_count / 6.0)

            dist = float(np.linalg.norm(cand.center - robot_xy))
            distance_score = math.exp(-dist / self.interp_decay_distance)

            richness_score = min(1.0, len(cand.object_ids) / 8.0)

            relation_hint = any(w in instruction.lower() for w in relation_words)
            relation_bonus = 0.1 if relation_hint and cand.relation_count > 0 else 0.0

            score = (
                w_kw * keyword_score
                + w_rel * relation_score
                + w_dist * distance_score
                + w_rich * richness_score
                + relation_bonus
            )
            score = min(max(score, 0.0), 1.0)

            scores[cand.subgraph_id] = score
            reasons[cand.subgraph_id] = (
                f"kw={keyword_score:.2f}, rel={relation_score:.2f}, "
                f"dist={distance_score:.2f}, rich={richness_score:.2f}"
            )
            logger.debug(
                "%s",
                {
                    "module": "sgnav_heuristic",
                    "candidate_id": cand.subgraph_id,
                    "sub_scores": {
                        "keyword": round(keyword_score, 4),
                        "relation": round(relation_score, 4),
                        "distance": round(distance_score, 4),
                        "richness": round(richness_score, 4),
                    },
                    "weighted_total": round(score, 4),
                    "ts": time.time(),
                },
            )

        return scores, reasons

    async def _score_subgraphs_llm(
        self,
        instruction: str,
        candidates: list[SubgraphCandidate],
        language: str,
        llm_chat: Callable[[list[dict[str, str]]], Awaitable[str | None]],
        frontier_descriptions: list[str] | None = None,
        explored_summaries: list[str] | None = None,
    ) -> dict[str, float]:
        messages = build_sgnav_subgraph_prompt(
            instruction=instruction,
            subgraphs=[c.to_prompt_dict() for c in candidates],
            language=language,
            frontier_descriptions=frontier_descriptions,
            explored_summaries=explored_summaries,
        )

        try:
            response = await llm_chat(messages)
            if not response:
                return {}
            data = self._extract_json(response)
            raw_scores = data.get("subgraph_scores", [])
            if not isinstance(raw_scores, list):
                return {}

            out: dict[str, float] = {}
            for item in raw_scores:
                if not isinstance(item, dict):
                    continue
                sid = str(item.get("subgraph_id", "")).strip()
                if not sid:
                    continue
                try:
                    score = float(item.get("score", 0.0))
                except (TypeError, ValueError):
                    continue
                out[sid] = min(max(score, 0.0), 1.0)
            return out
        except Exception as e:
            logger.debug("SG-Nav LLM subgraph scoring failed: %s", e)
            return {}

    def _interpolate_to_frontier(
        self,
        frontier: Frontier,
        subgraph_scores: list[SubgraphScore],
    ) -> tuple[float, str]:
        if not subgraph_scores:
            return 0.0, "no subgraph score"

        weighted_sum = 0.0
        weight_sum = 0.0
        nearest: tuple[SubgraphScore, float] | None = None

        for sg in subgraph_scores:
            dist = float(np.linalg.norm(frontier.center_world - sg.center))
            w = math.exp(-dist / self.interp_decay_distance)
            weighted_sum += sg.score * w
            weight_sum += w

            if nearest is None or dist < nearest[1]:
                nearest = (sg, dist)

        signal = weighted_sum / max(weight_sum, 1e-6)
        signal = min(max(signal, 0.0), 1.0)

        if nearest is None:
            return signal, "no nearest subgraph"

        near_sg, near_dist = nearest
        detail = f"nearest={near_sg.subgraph_id}({near_sg.level}), sg_score={near_sg.score:.2f}, dist={near_dist:.2f}m"
        return signal, detail

    def _parse_scene_graph(self, scene_graph_json: str) -> dict:
        try:
            data = json.loads(scene_graph_json)
            if isinstance(data, dict):
                return data
        except (json.JSONDecodeError, TypeError):
            pass
        return {}

    @staticmethod
    def _compute_objects_center(objects: list[dict]) -> np.ndarray:
        points = []
        for obj in objects:
            if not isinstance(obj, dict):
                continue
            pos = obj.get("position", {})
            if not isinstance(pos, dict):
                continue
            try:
                x = float(pos.get("x", 0.0))
                y = float(pos.get("y", 0.0))
            except (TypeError, ValueError):
                continue
            if np.isfinite(x) and np.isfinite(y):
                points.append([x, y])

        if not points:
            return np.array([0.0, 0.0], dtype=np.float64)
        arr = np.asarray(points, dtype=np.float64)
        return arr.mean(axis=0)

    @staticmethod
    def _to_int(value, default: int = 0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _to_center_xy(center_obj) -> np.ndarray:
        if not isinstance(center_obj, dict):
            return np.array([0.0, 0.0], dtype=np.float64)
        try:
            x = float(center_obj.get("x", 0.0))
            y = float(center_obj.get("y", 0.0))
        except (TypeError, ValueError):
            return np.array([0.0, 0.0], dtype=np.float64)

        if not np.isfinite(x) or not np.isfinite(y):
            return np.array([0.0, 0.0], dtype=np.float64)
        return np.array([x, y], dtype=np.float64)

    @staticmethod
    def _extract_keywords(text: str) -> list[str]:
        if not text:
            return []

        stop_words = {
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
            "去",
            "找",
            "到",
            "在",
            "附近",
            "的",
            "一个",
            "一下",
            "please",
        }
        tokens = re.findall(r"[A-Za-z0-9_]+|[\u4e00-\u9fff]{1,4}", text.lower())
        seen: set = set()
        dedup = []
        for t in tokens:
            if t in stop_words or len(t) <= 1 or t in seen:
                continue
            seen.add(t)
            dedup.append(t)
        return dedup

    @staticmethod
    def _extract_json(text) -> dict:
        if isinstance(text, dict):
            return text
        if not text:
            return {}

        text = text.strip()
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass

        fenced = re.search(r"```(?:json)?\s*(.*?)\s*```", text, re.S | re.I)
        if fenced:
            try:
                return json.loads(fenced.group(1))
            except json.JSONDecodeError:
                pass

        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            candidate = text[start : end + 1]
            try:
                return json.loads(candidate)
            except json.JSONDecodeError:
                return {}
        return {}
