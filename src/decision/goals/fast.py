"""Fast-path goal resolution strategies."""

from __future__ import annotations

import logging
import math
import re
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

import yaml

from runtime.runtime_interface import map_frame_id

if TYPE_CHECKING:
    from .resolver import GoalResult

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)
FAST_PATH_MAP_FRAME_ID = map_frame_id()

# Loaded from config/semantic_scoring.yaml [fast_path_fusion] on first call.
# These defaults match the original hardcoded values.
_DEFAULTS_FAST_PATH: dict = {
    "label": 0.35,
    "clip": 0.35,
    "detector": 0.15,
    "spatial": 0.15,
}
WEIGHT_LABEL_MATCH: float = _DEFAULTS_FAST_PATH["label"]
WEIGHT_CLIP_SIM: float = _DEFAULTS_FAST_PATH["clip"]
WEIGHT_DETECTOR_SCORE: float = _DEFAULTS_FAST_PATH["detector"]
WEIGHT_SPATIAL_HINT: float = _DEFAULTS_FAST_PATH["spatial"]
_fast_path_weights_loaded: bool = False


def _load_semantic_scoring_yaml() -> dict:
    """Load config/semantic_scoring.yaml from the repo root."""
    repo_root = Path(__file__).resolve().parents[3]
    yaml_path = repo_root / "config" / "semantic_scoring.yaml"
    try:
        with open(yaml_path, encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    except (FileNotFoundError, OSError):
        return {}


def _load_fast_path_weights() -> None:
    """Load fast_path_fusion weights from config/semantic_scoring.yaml (once)."""
    global WEIGHT_LABEL_MATCH, WEIGHT_CLIP_SIM, WEIGHT_DETECTOR_SCORE
    global WEIGHT_SPATIAL_HINT, _fast_path_weights_loaded
    if _fast_path_weights_loaded:
        return
    _fast_path_weights_loaded = True
    try:
        section = _load_semantic_scoring_yaml().get("fast_path_fusion")
        if section is None:
            logger.info(
                "fast_path_fusion section absent — using default weights. See config/semantic_scoring.yaml to tune."
            )
            return
        WEIGHT_LABEL_MATCH = float(section.get("label", _DEFAULTS_FAST_PATH["label"]))
        WEIGHT_CLIP_SIM = float(section.get("clip", _DEFAULTS_FAST_PATH["clip"]))
        WEIGHT_DETECTOR_SCORE = float(section.get("detector", _DEFAULTS_FAST_PATH["detector"]))
        WEIGHT_SPATIAL_HINT = float(section.get("spatial", _DEFAULTS_FAST_PATH["spatial"]))
        logger.debug(
            "fast_path_fusion weights loaded: label=%.2f clip=%.2f detector=%.2f spatial=%.2f",
            WEIGHT_LABEL_MATCH,
            WEIGHT_CLIP_SIM,
            WEIGHT_DETECTOR_SCORE,
            WEIGHT_SPATIAL_HINT,
        )
    except Exception as exc:
        logger.warning("Failed to load fast_path_fusion weights from config: %s", exc)


class FastPathMixin:
    """Fastpathmixin."""

    # ================================================================
    # ================================================================

    def _compute_score_entropy(self, scores: list[float]) -> float:
        """Compute score entropy."""
        if not scores or len(scores) < 2:
            return 0.0
        arr = np.array(scores, dtype=float)
        arr = arr - arr.min()
        total = arr.sum()
        if total < 1e-9:
            return math.log(len(scores))
        probs = arr / total
        probs = np.clip(probs, 1e-9, 1.0)
        return float(-np.sum(probs * np.log(probs)))

    # ================================================================
    # ================================================================

    def fast_resolve(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: dict[str, float] | None = None,
        clip_encoder: Any | None = None,
    ) -> GoalResult | None:
        """Fast resolve with metrics wrapper.

        Records fast/slow path decisions, timing, and hit rate for observability.
        The actual resolution logic lives in ``_fast_resolve_impl``.
        """
        self._fast_path_attempts += 1
        t0 = time.monotonic()
        result, candidates_count, best_score = self._fast_resolve_impl(
            instruction, scene_graph_json, robot_position, clip_encoder
        )
        elapsed_ms = (time.monotonic() - t0) * 1000
        self._resolve_times.append(elapsed_ms)
        if len(self._resolve_times) > self._max_resolve_history:
            self._resolve_times.pop(0)

        is_fast = result is not None and getattr(result, "path", "") == "fast"
        if is_fast and result.confidence >= self._fast_path_threshold:
            self._fast_path_hits += 1

        best_confidence = result.confidence if result is not None else best_score
        logger.info(
            "[GoalResolver] resolve result: path=%s confidence=%.3f candidates=%d elapsed_ms=%.1f",
            "fast" if is_fast else "slow",
            best_confidence,
            candidates_count,
            elapsed_ms,
        )
        return result

    def _fast_resolve_impl(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: dict[str, float] | None = None,
        clip_encoder: Any | None = None,
    ) -> tuple[GoalResult | None, int, float]:
        """Fast resolve implementation."""
        _load_fast_path_weights()  # idempotent, loads from config on first call
        from runtime.utils.sanitize import safe_json_loads

        from .resolver import GoalResult

        candidates_count = 0
        best_score = 0.0

        # Pure JSON->dict parse is instruction-independent, so cache it.
        # The cache only wraps this deterministic step; all instruction-
        # dependent scoring below still runs every call.
        scene_cache = getattr(self, "_scene_cache", None)
        if scene_cache is not None:
            sg = scene_cache.get_or_parse(
                scene_graph_json,
                lambda payload: safe_json_loads(payload, default=None),
            )
        else:
            sg = safe_json_loads(scene_graph_json, default=None)
        if sg is None:
            return None, candidates_count, best_score

        objects = sg.get("objects", [])
        if not objects:
            keywords = self._extract_keywords(instruction)
            room_result = self._try_room_fallback(instruction, keywords, sg, clip_encoder)
            return room_result, candidates_count, best_score

        allowed_obj_ids = self._get_object_ids_in_top_rooms(instruction, sg, clip_encoder, top_k=2)
        if allowed_obj_ids is not None:
            objects = [o for o in objects if o.get("id") in allowed_obj_ids]
            if not objects:
                keywords = self._extract_keywords(instruction)
                room_result = self._try_room_fallback(instruction, keywords, sg, clip_encoder)
                return room_result, candidates_count, best_score

        relations = sg.get("relations", [])
        inst_lower = instruction.lower()

        keywords = self._extract_keywords(instruction)

        subject_labels, modifier_labels = self._parse_instruction_roles(
            inst_lower, keywords, [o.get("label", "").lower() for o in objects]
        )

        scored: list[tuple[dict, float, str]] = []

        # Pre-compute CLIP similarities in one batched call instead of N individual calls.
        _clip_sims: dict = {}
        if clip_encoder is not None and hasattr(clip_encoder, "batch_text_image_similarity"):
            _clip_objs = [
                (obj.get("id"), np.array(obj["clip_feature"])) for obj in objects if obj.get("clip_feature") is not None
            ]
            if _clip_objs:
                try:
                    _ids, _feats = zip(*_clip_objs)
                    _sims = clip_encoder.batch_text_image_similarity([instruction], list(_feats))
                    if _sims is not None and len(_sims) > 0:
                        for _j, _oid in enumerate(_ids):
                            _clip_sims[_oid] = float(_sims[0][_j])
                except Exception as _e:
                    logger.debug("Batch CLIP similarity failed, falling back per-object: %s", _e)

        for obj in objects:
            label = obj.get("label", "").lower()
            score = obj.get("score", 0.5)
            det_count = obj.get("detection_count", 1)

            label_score = 0.0
            is_subject = False

            for subj in subject_labels:
                if subj == label:
                    label_score = 1.0
                    is_subject = True
                    break
                if subj in label or label in subj:
                    label_score = max(label_score, 0.9)
                    is_subject = True

            if not is_subject:
                for mod in modifier_labels:
                    if mod == label or mod in label or label in mod:
                        label_score = max(label_score, 0.3)
                        break

            if label_score == 0.0:
                for kw in keywords:
                    if kw in label or label in kw:
                        label_score = max(label_score, 0.5)

            # KG co-occurrence: object shares room with instruction target
            # (ASCENT-style learned co-occurrence via P(obj|room))
            if label_score == 0.0 and getattr(self, "_room_object_kg", None):
                for kw in keywords:
                    co_objects = self._room_object_kg.get_cooccurring_objects(kw)
                    for co_label, co_weight in co_objects:
                        if co_label in label or label in co_label:
                            label_score = max(label_score, 0.35 * min(co_weight / 0.5, 1.0))
                            break
                    if label_score > 0:
                        break

            if label_score == 0.0:
                continue

            detector_score = min(score, 1.0) * min(det_count / 3, 1.0)

            clip_score = 0.0
            has_real_clip = False
            obj_id = obj.get("id")
            if obj_id in _clip_sims:
                clip_score = _clip_sims[obj_id]
                has_real_clip = True
            elif clip_encoder is not None and obj.get("clip_feature") is not None:
                # Fallback: per-object call (batch_text_image_similarity unavailable)
                try:
                    clip_feature = np.array(obj.get("clip_feature"))
                    if clip_feature.size > 0:
                        similarities = clip_encoder.text_image_similarity(instruction, [clip_feature])
                        clip_score = similarities[0] if similarities else 0.0
                        has_real_clip = True
                except Exception as e:
                    logger.warning("CLIP similarity computation failed: %s", e)

            spatial_score = 0.0
            for rel in relations:
                obj_id = obj.get("id")
                if rel.get("subject_id") == obj_id or rel.get("object_id") == obj_id:
                    is_subject = rel.get("subject_id") == obj_id

                    related_id = rel["object_id"] if is_subject else rel["subject_id"]
                    related_obj = next((o for o in objects if o.get("id") == related_id), None)
                    if related_obj:
                        related_label = related_obj.get("label", "").lower()

                        label_core = self._extract_core_noun(label)
                        related_core = self._extract_core_noun(related_label)

                        label_in_inst = label_core in inst_lower or label in inst_lower
                        related_in_inst = related_core in inst_lower or related_label in inst_lower

                        if label_in_inst and related_in_inst:
                            label_pos = inst_lower.find(label_core if label_core in inst_lower else label)
                            related_pos = inst_lower.find(related_core if related_core in inst_lower else related_label)

                            if label_pos < related_pos:
                                spatial_score = 1.0
                                break
                            else:
                                spatial_score = 0.2

                        elif rel.get("relation") == "near":
                            spatial_score = max(spatial_score, 0.3)

            if has_real_clip:
                fused_score = (
                    WEIGHT_LABEL_MATCH * label_score
                    + WEIGHT_CLIP_SIM * clip_score
                    + WEIGHT_DETECTOR_SCORE * detector_score
                    + WEIGHT_SPATIAL_HINT * spatial_score
                )
            else:
                fused_score = 0.75 * label_score + 0.15 * detector_score + 0.10 * spatial_score

            obj_source = obj.get("source", "observed")
            if obj_source == "kg_prior":
                fused_score *= 0.4
            elif obj_source == "loaded":
                fused_score *= 0.7

            clip_tag = f"clip={clip_score:.2f}" if has_real_clip else "clip=N/A"
            src_tag = f"src={obj_source}" if obj_source != "observed" else ""
            reason = (
                f"label={label_score:.1f}, {clip_tag}, "
                f"det={detector_score:.2f}, spatial={spatial_score:.1f} → fused={fused_score:.2f}"
                f" {src_tag}"
            ).strip()
            scored.append((obj, fused_score, reason))
            logger.debug(
                "%s",
                {
                    "module": "fast_path_fusion",
                    "candidate_id": obj.get("id"),
                    "sub_scores": {
                        "label": round(label_score, 4),
                        "clip": round(clip_score, 4),
                        "detector": round(detector_score, 4),
                        "spatial": round(spatial_score, 4),
                    },
                    "weighted_total": round(fused_score, 4),
                    "ts": time.time(),
                },
            )

        if not scored:
            room_result = self._try_room_fallback(instruction, keywords, sg, clip_encoder)
            return room_result, candidates_count, best_score

        scored.sort(key=lambda x: x[1], reverse=True)
        best_obj, best_score, best_reason = scored[0]
        candidates_count = len(scored)

        candidate_scores = [sc for _, sc, _ in scored]
        score_entropy = self._compute_score_entropy(candidate_scores)

        if clip_encoder is not None and len(scored) >= 2:
            top_candidates = [(obj, sc, r) for obj, sc, r in scored[:5] if sc > best_score * 0.8]
            if len(top_candidates) >= 2:
                core_labels = [self._extract_core_noun(o.get("label", "")).lower() for o, _, _ in top_candidates]
                if len(set(core_labels)) == 1:
                    clip_ranked = self._clip_attribute_disambiguate(instruction, top_candidates, clip_encoder)
                    if clip_ranked:
                        best_obj, best_score, best_reason = clip_ranked[0]
                        best_reason += " [CLIP-attr-disambig]"

        if robot_position:

            def _get_xy(obj_dict):
                """Get xy."""
                p = obj_dict.get("position", {})
                if isinstance(p, (list, tuple)):
                    return (p[0] if len(p) > 0 else 0), (p[1] if len(p) > 1 else 0)
                return p.get("x", 0), p.get("y", 0)

            bx, by = _get_xy(best_obj)
            rx, ry = robot_position.get("x", 0), robot_position.get("y", 0)
            dist = math.sqrt((bx - rx) ** 2 + (by - ry) ** 2)
            for obj2, sc2, _ in scored[1:3]:
                if sc2 > best_score * 0.9:
                    o2x, o2y = _get_xy(obj2)
                    dist2 = math.sqrt((o2x - rx) ** 2 + (o2y - ry) ** 2)
                    if dist2 < dist * 0.5:
                        best_obj, best_score, best_reason = obj2, sc2, _
                        break

        if best_score < self._fast_path_threshold:
            logger.info(
                "Fast path score %.2f < threshold %.2f, deferring to Slow path. Best: %s (%s)",
                best_score,
                self._fast_path_threshold,
                best_obj.get("label", "->"),
                best_reason,
            )

            phantom_nodes = sg.get("phantom_nodes", [])
            if phantom_nodes and keywords:
                phantom_scored = []
                for pn in phantom_nodes:
                    p_label = pn.get("label", "").lower()
                    p_exist = pn.get("P_exist", 0.0)
                    label_match = any(kw in p_label or p_label in kw for kw in keywords)
                    if label_match and p_exist > 0.3:
                        phantom_scored.append((pn, p_exist))

                if phantom_scored:
                    phantom_scored.sort(key=lambda x: x[1], reverse=True)
                    best_pn, best_p_exist = phantom_scored[0]
                    pn_pos = best_pn.get("position", {})
                    pn_x = pn_pos.get("x", 0.0)
                    pn_y = pn_pos.get("y", 0.0)
                    logger.info(
                        "Phantom node hit: '%s' P_exist=%.2f at (%.2f, %.2f), room=%s",
                        best_pn.get("label", "->"),
                        best_p_exist,
                        pn_x,
                        pn_y,
                        best_pn.get("room_type", "->"),
                    )
                    return (
                        GoalResult(
                            action="explore",
                            target_x=pn_x,
                            target_y=pn_y,
                            target_label=f"phantom:{best_pn.get('label', '')}",
                            confidence=best_p_exist * 0.7,
                            reasoning=(
                                f"Phantom node: expected {best_pn.get('label', '->')} "
                                f"in {best_pn.get('room_type', '->')} "
                                f"(P_exist={best_p_exist:.2f})"
                            ),
                            is_valid=True,
                            path="fast",
                            frame_id=sg.get("frame_id", FAST_PATH_MAP_FRAME_ID),
                        ),
                        candidates_count,
                        best_p_exist * 0.7,
                    )

            room_result = self._try_room_fallback(
                instruction,
                keywords,
                sg,
                clip_encoder,
            )
            if room_result is not None:
                return room_result, candidates_count, best_score

            return None, candidates_count, best_score

        if len(scored) >= 2:
            candidates_for_belief = []
            for obj_dict, fused_sc, _ in scored[:8]:
                pos_d = obj_dict.get("position", {})
                if isinstance(pos_d, (list, tuple)):
                    pos_xyz = [
                        pos_d[0] if len(pos_d) > 0 else 0,
                        pos_d[1] if len(pos_d) > 1 else 0,
                        pos_d[2] if len(pos_d) > 2 else 0,
                    ]
                else:
                    pos_xyz = [pos_d.get("x", 0), pos_d.get("y", 0), pos_d.get("z", 0)]
                candidates_for_belief.append(
                    {
                        "id": obj_dict.get("id", -1),
                        "label": obj_dict.get("label", ""),
                        "position": pos_xyz,
                        "fused_score": fused_sc,
                        "belief": obj_dict.get("belief", {}),
                        "room_match": 0.5,
                    }
                )
            self._belief_manager.init_from_candidates(candidates_for_belief, instruction)

            robot_pos = [robot_position.get("x", 0), robot_position.get("y", 0)] if robot_position else None
            selected = self._belief_manager.select_next_target(robot_pos)
            if selected and not self._belief_manager.is_converged:
                logger.info(
                    "BA-HSG multi-hypothesis: %d active candidates, top posterior=%.3f (%s)",
                    self._belief_manager.num_active,
                    selected.posterior,
                    selected.label,
                )

        pos = best_obj.get("position", {})
        if isinstance(pos, (list, tuple)):
            px, py, pz = (pos[0] if len(pos) > 0 else 0, pos[1] if len(pos) > 1 else 0, pos[2] if len(pos) > 2 else 0)
        else:
            px, py, pz = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)

        logger.info(
            "Fast path hit: '%s' at (%.2f, %.2f), score=%.2f [%s]",
            best_obj.get("label", "->"),
            px,
            py,
            best_score,
            best_reason,
        )

        target_frame = sg.get("frame_id", FAST_PATH_MAP_FRAME_ID)

        best_source = best_obj.get("source", "observed")
        if best_source == "kg_prior":
            logger.info(
                "Fast path KG-prior: '%s' → action=explore (搜索区域, 非精确目标)",
                best_obj.get("label", "->"),
            )
            return (
                GoalResult(
                    action="explore",
                    target_x=px,
                    target_y=py,
                    target_z=pz,
                    target_label=best_obj.get("label", ""),
                    confidence=best_score,
                    reasoning=f"Fast path KG-prior: {best_reason} (explore region)",
                    is_valid=True,
                    path="fast",
                    candidate_id=best_obj.get("id", -1),
                    frame_id=target_frame,
                    score_entropy=score_entropy,
                ),
                candidates_count,
                best_score,
            )

        return (
            GoalResult(
                action="navigate",
                target_x=px,
                target_y=py,
                target_z=pz,
                target_label=best_obj.get("label", ""),
                confidence=best_score,
                reasoning=f"Fast path: {best_reason}",
                is_valid=True,
                path="fast",
                candidate_id=best_obj.get("id", -1),
                frame_id=target_frame,
                score_entropy=score_entropy,
            ),
            candidates_count,
            best_score,
        )

    @staticmethod
    def _extract_core_noun(label: str) -> str:
        """Extract core noun."""
        colors = {
            "red",
            "blue",
            "green",
            "yellow",
            "white",
            "black",
            "gray",
            "grey",
            "orange",
            "purple",
            "pink",
            "brown",
            "cyan",
            "magenta",
            "红色",
            "蓝色",
            "绿色",
            "黄色",
            "白色",
            "黑色",
            "灰色",
            "橙色",
            "紫色",
            "粉色",
            "棕色",
            "红",
            "蓝",
            "绿",
            "黄",
            "白",
            "黑",
            "灰",
        }

        words = label.split()

        core_words = [w for w in words if w.lower() not in colors]

        if core_words:
            return " ".join(core_words)
        else:
            return label

    @staticmethod
    def _get_object_ids_in_top_rooms(
        instruction: str,
        scene_graph: dict,
        clip_encoder,
        top_k: int = 2,
    ) -> set | None:
        """Get object ids in top rooms."""
        if clip_encoder is None:
            return None

        rooms_data = scene_graph.get("rooms", [])
        rooms_with_clip = [r for r in rooms_data if r.get("clip_feature") is not None]
        if len(rooms_with_clip) >= 2:
            try:
                query_feat = clip_encoder.encode_text([instruction])
                if query_feat.size > 0:
                    q = query_feat[0]
                    scored = []
                    for r in rooms_with_clip:
                        rf = np.array(r["clip_feature"])
                        norm = np.linalg.norm(rf)
                        sim = float(np.dot(q, rf / norm)) if norm > 0 else 0.0
                        scored.append((r, sim))
                    scored.sort(key=lambda x: x[1], reverse=True)
                    allowed = set()
                    for r, _ in scored[:top_k]:
                        allowed.update(r.get("object_ids", []))
                    if allowed:
                        return allowed
            except (ValueError, TypeError, AttributeError) as e:
                logger.debug("HOV-SG room CLIP scoring failed: %s", e)

        subgraphs = scene_graph.get("subgraphs", [])
        rooms = [s for s in subgraphs if s.get("level") == "room"]
        if len(rooms) < 2:
            return None
        obj_by_id = {o.get("id"): o for o in scene_graph.get("objects", []) if o.get("id") is not None}
        room_texts = []
        for r in rooms:
            labels = r.get("object_labels", [])
            if not labels:
                oids = r.get("object_ids", [])
                labels = [obj_by_id.get(oid, {}).get("label", "") for oid in oids]
            room_texts.append(" ".join(str(label) for label in labels[:10] if label))
        if not room_texts:
            return None
        try:
            sims = clip_encoder.text_text_similarity(instruction, room_texts)
        except (ValueError, TypeError, AttributeError) as e:
            logger.debug("CLIP text_text_similarity failed: %s", e)
            return None
        if not sims or len(sims) != len(rooms):
            return None
        ranked = sorted(range(len(rooms)), key=lambda i: sims[i], reverse=True)
        allowed = set()
        for i in ranked[:top_k]:
            allowed.update(rooms[i].get("object_ids", []))
        return allowed

    @staticmethod
    def _clip_attribute_disambiguate(
        instruction: str,
        candidates: list[tuple[dict, float, str]],
        clip_encoder,
    ) -> list[tuple[dict, float, str]]:
        """Clip attribute disambiguate."""
        clip_scored = []
        for obj, fused_score, reason in candidates:
            clip_feature = obj.get("clip_feature")
            if clip_feature is not None:
                try:
                    feat = np.array(clip_feature)
                    if feat.size > 0:
                        sims = clip_encoder.text_image_similarity(instruction, [feat])
                        clip_sim = sims[0] if sims else 0.0
                        combined = 0.7 * fused_score + 0.3 * clip_sim
                        new_reason = f"{reason}, clip_attr={clip_sim:.3f}, combined={combined:.3f}"
                        clip_scored.append((obj, combined, new_reason))
                        continue
                except (ValueError, TypeError, AttributeError) as e:
                    logger.debug("CLIP attribute disambiguate failed for '%s': %s", obj.get("label", "->"), e)
            clip_scored.append((obj, fused_score, reason))

        clip_scored.sort(key=lambda x: x[1], reverse=True)
        return clip_scored

    def _try_room_fallback(
        self,
        instruction: str,
        keywords: list[str],
        scene_graph: dict,
        clip_encoder: Any | None = None,
    ) -> GoalResult | None:
        """Try room fallback."""
        from .resolver import GoalResult

        rooms = scene_graph.get("rooms", [])
        if not rooms:
            return None

        inst_lower = instruction.lower()
        best_room = None
        best_match = 0.0

        for room in rooms:
            name = room.get("name", "")
            center = room.get("center")
            if not name or not center:
                continue

            name_lower = name.lower()
            match_score = 0.0

            if clip_encoder is not None and room.get("clip_feature") is not None:
                try:
                    rf = np.array(room["clip_feature"])
                    if rf.size > 0:
                        sims = clip_encoder.text_image_similarity(instruction, [rf])
                        if sims:
                            match_score = max(match_score, sims[0])
                except (ValueError, TypeError, AttributeError):
                    pass

            for kw in keywords:
                kw_l = kw.lower()
                if kw_l in name_lower or name_lower in kw_l:
                    match_score = max(match_score, 0.8)
                    break

            if match_score < 0.5:
                if name_lower in inst_lower or inst_lower in name_lower:
                    match_score = max(match_score, 0.7)

            if match_score > best_match:
                best_match = match_score
                best_room = room

        if best_room is None or best_match < 0.5:
            return None

        center = best_room["center"]
        if isinstance(center, (list, tuple)):
            cx = center[0] if len(center) > 0 else 0.0
            cy = center[1] if len(center) > 1 else 0.0
            cz = center[2] if len(center) > 2 else 0.0
        elif isinstance(center, dict):
            cx = center.get("x", 0.0)
            cy = center.get("y", 0.0)
            cz = center.get("z", 0.0)
        else:
            return None

        confidence = best_match * 0.6
        room_name = best_room.get("name", "unknown_room")
        logger.info(
            "Fast path room fallback: '%s' match=%.2f conf=%.2f at (%.2f, %.2f)",
            room_name,
            best_match,
            confidence,
            cx,
            cy,
        )

        return GoalResult(
            action="navigate",
            target_x=cx,
            target_y=cy,
            target_z=cz,
            target_label=room_name,
            confidence=confidence,
            reasoning=f"Fast path room fallback: '{room_name}' match={best_match:.2f}",
            is_valid=True,
            path="fast",
            frame_id=scene_graph.get("frame_id", FAST_PATH_MAP_FRAME_ID),
            hint_room=room_name,
            hint_room_center=[cx, cy, cz],
        )

    @staticmethod
    def _extract_keywords(instruction: str) -> list[str]:
        """Extract keywords."""
        import re as _re

        stop_words = {
            "the",
            "a",
            "an",
            "to",
            "go",
            "find",
            "get",
            "me",
            "for",
            "and",
            "or",
            "is",
            "at",
            "in",
            "on",
            "near",
            "next",
            "by",
            "of",
            "with",
            "from",
            "去",
            "到",
            "找",
            "拿",
            "的",
            "在",
            "旁边",
            "附近",
            "那个",
            "请",
            "帮",
            "我",
            "一个",
            "把",
            "了",
            "着",
            "过",
        }

        chinese_parts = _re.findall(r"[\u4e00-\u9fff]+", instruction)
        english_parts = _re.findall(r"[a-zA-Z]+", instruction.lower())
        chinese_text = " ".join(chinese_parts)

        all_keywords: list[str] = []

        for w in english_parts:
            w_lower = w.lower()
            if w_lower not in stop_words and len(w_lower) > 1:
                all_keywords.append(w_lower)

        if chinese_text:
            try:
                from .tokenizer import extract_keywords

                zh_keywords = extract_keywords(
                    chinese_text,
                    min_length=2,
                    filter_stopwords=True,
                    keep_colors=True,
                    keep_spatial=True,
                )
                all_keywords.extend(zh_keywords)
            except ImportError:
                zh_tokens = _re.findall(r"[\u4e00-\u9fff]+", chinese_text)
                all_keywords.extend(t for t in zh_tokens if t not in stop_words and len(t) > 1)

        deduped = list(set(all_keywords))

        try:
            from .tokenizer import expand_bilingual

            deduped = expand_bilingual(deduped)
        except ImportError:
            pass

        return deduped

    def _parse_instruction_roles(
        self,
        inst_lower: str,
        keywords: list[str],
        scene_labels: list[str],
    ) -> tuple[list[str], list[str]]:
        """Parse instruction roles."""
        subjects, modifiers = self._parse_roles_regex(inst_lower, keywords, scene_labels)
        if subjects:
            return subjects, modifiers

        subjects, modifiers = self._parse_roles_scene_order(inst_lower, keywords, scene_labels)
        if subjects:
            return subjects, modifiers

        logger.debug(
            "Role parsing: Level 1+2 failed, treating all %d keywords as subjects",
            len(keywords),
        )
        return keywords[:], []

    @staticmethod
    def _parse_roles_regex(
        inst_lower: str,
        keywords: list[str],
        scene_labels: list[str],
    ) -> tuple[list[str], list[str]]:
        """Parse roles regex."""
        subjects: list[str] = []
        modifiers: list[str] = []

        en_patterns = [
            r"\b(?:find|go\s+to|navigate\s+to|locate|get)\s+([\w\s]+?)\s+(?:near|by|beside|next\s+to|behind|in\s+front\s+of|left\s+of|right\s+of|on|under|above|below)\s+(?:the\s+)?([\w\s]+)",
            r"\b([\w]+)\s+(?:near|by|beside|next\s+to)\s+(?:the\s+)?([\w]+)",
        ]

        for pat in en_patterns:
            m = re.search(pat, inst_lower)
            if m:
                subj_str = m.group(1).strip()
                mod_str = m.group(2).strip()
                for lbl in scene_labels:
                    if lbl in subj_str or subj_str in lbl:
                        subjects.append(lbl)
                    if lbl in mod_str or mod_str in lbl:
                        modifiers.append(lbl)
                if subjects:
                    return list(set(subjects)), list(set(modifiers))

        zh_patterns = [
            r"([\u4e00-\u9fff]+?)(?:旁边|附近|左边|右边|前面|后面|上面|下面|对面|里面)的([\u4e00-\u9fff]+)",
            r"(?:去|到|找|找到|寻找)([\u4e00-\u9fff]+)",
        ]

        try:
            from .tokenizer import translate_label as _tl
        except ImportError:
            _tl = None

        def _match_label(text: str, lbl: str) -> bool:
            """Match label."""
            if lbl in text or text in lbl:
                return True
            if _tl is not None:
                for alias in _tl(text):
                    if alias.lower() in lbl or lbl in alias.lower():
                        return True
            return False

        for i, pat in enumerate(zh_patterns):
            m = re.search(pat, inst_lower)
            if m:
                if i == 0:
                    mod_str = m.group(1)
                    subj_str = m.group(2)
                    for lbl in scene_labels:
                        if _match_label(subj_str, lbl):
                            subjects.append(lbl)
                        if _match_label(mod_str, lbl):
                            modifiers.append(lbl)
                    if subjects:
                        return list(set(subjects)), list(set(modifiers))
                else:
                    subj_str = m.group(1)
                    for lbl in scene_labels:
                        if _match_label(subj_str, lbl):
                            subjects.append(lbl)
                    if subjects:
                        return list(set(subjects)), []

        return [], []

    @staticmethod
    def _parse_roles_scene_order(
        inst_lower: str,
        keywords: list[str],
        scene_labels: list[str],
    ) -> tuple[list[str], list[str]]:
        """Parse roles scene order."""
        found_in_scene = []
        for kw in keywords:
            for lbl in scene_labels:
                if kw in lbl or lbl in kw:
                    found_in_scene.append(lbl)
                    break
        for lbl in scene_labels:
            if lbl in inst_lower and lbl not in found_in_scene:
                found_in_scene.append(lbl)

        if found_in_scene:
            return [found_in_scene[0]], found_in_scene[1:]
        return [], []
