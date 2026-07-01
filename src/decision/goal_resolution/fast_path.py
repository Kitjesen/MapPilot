"""
fast_path.py — Fast Path (System 1) 目标解析 Mixin。

从 goal_resolver.py 提取，GoalResolver 通过多继承使用:
    class GoalResolver(FastPathMixin, SlowPathMixin): ...

包含:
  - fast_resolve(): 场景图直接匹配，无需 LLM
  - _compute_score_entropy(): AdaNav 不确定度指标
  - _extract_keywords(): 指令关键词提取 (jieba + 双语扩展)
  - _parse_instruction_roles() 及子方法: 主语/修饰语解析
  - _try_room_fallback(): 区域级 fallback
  - _get_object_ids_in_top_rooms(): GAP Room CLIP 筛选
  - _clip_attribute_disambiguate(): B7 CLIP 属性消歧
  - _extract_core_noun(): 核心名词提取
"""
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
    from .goal_resolver import GoalResult

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)
FAST_PATH_MAP_FRAME_ID = map_frame_id()

# ── Multi-source confidence weights (AdaNav uncertainty fusion) ──
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
                "fast_path_fusion section absent — using default weights. "
                "See config/semantic_scoring.yaml to tune."
            )
            return
        WEIGHT_LABEL_MATCH = float(section.get("label", _DEFAULTS_FAST_PATH["label"]))
        WEIGHT_CLIP_SIM = float(section.get("clip", _DEFAULTS_FAST_PATH["clip"]))
        WEIGHT_DETECTOR_SCORE = float(section.get("detector", _DEFAULTS_FAST_PATH["detector"]))
        WEIGHT_SPATIAL_HINT = float(section.get("spatial", _DEFAULTS_FAST_PATH["spatial"]))
        logger.debug(
            "fast_path_fusion weights loaded: label=%.2f clip=%.2f detector=%.2f spatial=%.2f",
            WEIGHT_LABEL_MATCH, WEIGHT_CLIP_SIM, WEIGHT_DETECTOR_SCORE, WEIGHT_SPATIAL_HINT,
        )
    except Exception as exc:
        logger.warning("Failed to load fast_path_fusion weights from config: %s", exc)


class FastPathMixin:
    """
    Fast Path (System 1) 方法集合。

    依赖 self 上存在:
      - self._fast_path_threshold: float
      - self._belief_manager: TargetBeliefManager
      - self._extract_keywords (静态方法, 通过 self 访问)
    """

    # ================================================================
    #  AdaNav: 得分熵计算
    # ================================================================

    def _compute_score_entropy(self, scores: list[float]) -> float:
        """计算候选得分归一化后的香农熵（AdaNav 不确定度指标）。

        高熵表示候选得分均匀分布（歧义高），低熵表示某个候选明显胜出。
        """
        if not scores or len(scores) < 2:
            return 0.0
        arr = np.array(scores, dtype=float)
        arr = arr - arr.min()
        total = arr.sum()
        if total < 1e-9:
            return math.log(len(scores))  # 均匀分布 → 最大熵
        probs = arr / total
        probs = np.clip(probs, 1e-9, 1.0)
        return float(-np.sum(probs * np.log(probs)))

    # ================================================================
    #  Fast Path — System 1 (VLingNav / OmniNav)
    # ================================================================

    def fast_resolve(
        self,
        instruction: str,
        scene_graph_json: str,
        robot_position: dict[str, float] | None = None,
        clip_encoder: Any | None = None,
    ) -> GoalResult | None:
        """
        Fast Path: 场景图直接匹配, 无需 LLM。

        参考:
          - VLingNav (2026): AdaCoT — 简单情况用 System 1
          - OmniNav (ICLR 2026): Fast 模块 5 Hz waypoint
          - AdaNav (ICLR 2026): 高确定性 → 跳过深度推理

        匹配算法:
          1. 从场景图提取物体列表
          2. 提取指令主语 (目标物体) 与修饰语 (空间参考物)
          3. 多源评分: 主语匹配 + CLIP + 检测分 + 空间关系
          4. 最高分 > fast_path_threshold → 直接返回目标
          5. 否则 → 返回 None, 交给 Slow Path

        Args:
            instruction: 用户指令
            scene_graph_json: 场景图 JSON
            robot_position: 当前位置
            clip_encoder: CLIP 编码器 (可选)

        Returns:
            GoalResult or None (None = 需要 Slow Path)
        """
        _load_fast_path_weights()  # idempotent, loads from config on first call
        from runtime.utils.sanitize import safe_json_loads

        from .goal_resolver import GoalResult

        sg = safe_json_loads(scene_graph_json, default=None)
        if sg is None:
            return None

        objects = sg.get("objects", [])
        if not objects:
            # 无物体但可能有 room 匹配
            keywords = self._extract_keywords(instruction)
            return self._try_room_fallback(instruction, keywords, sg, clip_encoder)

        # GAP: 逐层 CLIP 筛选 (FSR-VLN 路线 A) — 先筛 Room 再筛 Object
        allowed_obj_ids = self._get_object_ids_in_top_rooms(
            instruction, sg, clip_encoder, top_k=2
        )
        if allowed_obj_ids is not None:  # None = 未启用/无 rooms
            objects = [o for o in objects if o.get("id") in allowed_obj_ids]
            if not objects:
                keywords = self._extract_keywords(instruction)
                return self._try_room_fallback(instruction, keywords, sg, clip_encoder)

        relations = sg.get("relations", [])
        inst_lower = instruction.lower()

        # ── 提取指令关键词 ──
        keywords = self._extract_keywords(instruction)

        # ── 提取主语 (目标) 与修饰语 (空间参考物) ──
        # "find chair near the door" → subject="chair", modifier="door"
        # "去门旁边的椅子"           → subject="椅子",  modifier="门"
        subject_labels, modifier_labels = self._parse_instruction_roles(
            inst_lower, keywords, [o.get("label", "").lower() for o in objects]
        )

        # ── 对每个物体打分 (AdaNav 多源置信度) ──
        scored: list[tuple[dict, float, str]] = []

        # Pre-compute CLIP similarities in one batched call instead of N individual calls.
        # batch_text_image_similarity(texts, features) → (1, K) matrix.
        _clip_sims: dict = {}  # obj_id → float similarity
        if clip_encoder is not None and hasattr(clip_encoder, "batch_text_image_similarity"):
            _clip_objs = [
                (obj.get("id"), np.array(obj["clip_feature"]))
                for obj in objects
                if obj.get("clip_feature") is not None
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

            # 源 1: 标签文本匹配 — 区分主语 vs 修饰语
            label_score = 0.0
            is_subject = False  # 是否匹配为指令的主语 (真正目标)

            # 主语匹配 (目标物体, 高分)
            for subj in subject_labels:
                if subj == label:
                    label_score = 1.0
                    is_subject = True
                    break
                if subj in label or label in subj:
                    label_score = max(label_score, 0.9)
                    is_subject = True

            # 如果不是主语, 检查是否是修饰语 (空间参考物, 低分)
            if not is_subject:
                for mod in modifier_labels:
                    if mod == label or mod in label or label in mod:
                        label_score = max(label_score, 0.3)  # 修饰语低分
                        break

            # 通用关键词匹配 (兜底)
            if label_score == 0.0:
                for kw in keywords:
                    if kw in label or label in kw:
                        label_score = max(label_score, 0.5)

            # KG co-occurrence: object shares room with instruction target
            # (ASCENT-style learned co-occurrence via P(obj|room))
            if label_score == 0.0 and getattr(self, '_room_object_kg', None):
                for kw in keywords:
                    co_objects = self._room_object_kg.get_cooccurring_objects(kw)
                    for co_label, co_weight in co_objects:
                        if co_label in label or label in co_label:
                            label_score = max(label_score, 0.35 * min(co_weight / 0.5, 1.0))
                            break
                    if label_score > 0:
                        break

            if label_score == 0.0:
                continue  # 完全不相关, 跳过

            # 源 2: 检测器置信度 (多次观测 → 更可靠)
            detector_score = min(score, 1.0) * min(det_count / 3, 1.0)

            # 源 3: CLIP 视觉-语言相似度 — use pre-computed batch result when available
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
                        similarities = clip_encoder.text_image_similarity(
                            instruction, [clip_feature]
                        )
                        clip_score = similarities[0] if similarities else 0.0
                        has_real_clip = True
                except Exception as e:
                    logger.warning("CLIP similarity computation failed: %s", e)
            # 无真实CLIP时不使用伪造近似 — 将权重重分配给其他真实信号

            # 源 4: 空间关系提示
            # 修复: 区分主体(要找的)和参考物(用于定位的)
            # "find chair near door" → chair是主体, door是参考物
            # 只有主体应该获得空间关系加分
            spatial_score = 0.0
            for rel in relations:
                obj_id = obj.get("id")
                if rel.get("subject_id") == obj_id or rel.get("object_id") == obj_id:
                    # 判断当前物体是关系的主语还是宾语
                    is_subject = (rel.get("subject_id") == obj_id)

                    related_id = (
                        rel["object_id"] if is_subject
                        else rel["subject_id"]
                    )
                    related_obj = next(
                        (o for o in objects if o.get("id") == related_id), None
                    )
                    if related_obj:
                        related_label = related_obj.get("label", "").lower()

                        # 提取核心词（去掉颜色等修饰词）
                        # "red chair" → "chair", "blue door" → "door"
                        label_core = self._extract_core_noun(label)
                        related_core = self._extract_core_noun(related_label)

                        # 关键修复: 检查核心词是否在指令中
                        # "go to chair near door" 应该匹配 "red chair" near "door"
                        label_in_inst = label_core in inst_lower or label in inst_lower
                        related_in_inst = related_core in inst_lower or related_label in inst_lower

                        if label_in_inst and related_in_inst:
                            # 检查指令中的语义: 哪个是主体，哪个是参考
                            # "find chair near door" → "chair"在"door"前面 → chair是主体
                            label_pos = inst_lower.find(label_core if label_core in inst_lower else label)
                            related_pos = inst_lower.find(related_core if related_core in inst_lower else related_label)

                            if label_pos < related_pos:
                                # 当前物体在前 → 是主体 → 给高分
                                spatial_score = 1.0
                                break
                            else:
                                # 当前物体在后 → 是参考物 → 给低分
                                spatial_score = 0.2

                        # 通用近距离关系加分(保底)
                        elif rel.get("relation") == "near":
                            spatial_score = max(spatial_score, 0.3)

            # 综合评分 (AdaNav 风格加权融合)
            if has_real_clip:
                # 4源完整融合
                fused_score = (
                    WEIGHT_LABEL_MATCH * label_score
                    + WEIGHT_CLIP_SIM * clip_score
                    + WEIGHT_DETECTOR_SCORE * detector_score
                    + WEIGHT_SPATIAL_HINT * spatial_score
                )
            else:
                # 无CLIP: 标签匹配是主信号 — 精确匹配即过 0.75 阈值
                # 标签匹配 0.75, 检测器 0.15, 空间 0.10
                fused_score = (
                    0.75 * label_score
                    + 0.15 * detector_score
                    + 0.10 * spatial_score
                )

            # ── 来源可信度折扣 ──
            # observed (真实相机检测) 全权信任
            # kg_prior (知识图谱先验) 大幅折扣 — 坐标不可靠, 只做搜索提示
            # loaded (历史加载) 中等折扣
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
            # Scoring audit log — enables offline weight learning (W3-2 Phase 2)
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
            return self._try_room_fallback(instruction, keywords, sg, clip_encoder)

        # 取最高分
        scored.sort(key=lambda x: x[1], reverse=True)
        best_obj, best_score, best_reason = scored[0]

        # AdaNav: 计算候选得分熵
        candidate_scores = [sc for _, sc, _ in scored]
        score_entropy = self._compute_score_entropy(candidate_scores)

        # ── B7: CLIP 属性消歧 (区分 "red chair" vs "blue chair") ──
        # 当存在多个同类型高分候选时, 用 CLIP 做精细排序
        if clip_encoder is not None and len(scored) >= 2:
            top_candidates = [
                (obj, sc, r) for obj, sc, r in scored[:5]
                if sc > best_score * 0.8
            ]
            if len(top_candidates) >= 2:
                # 检查是否同类型 (核心名词相同, 只是属性不同)
                core_labels = [
                    self._extract_core_noun(o.get("label", "")).lower()
                    for o, _, _ in top_candidates
                ]
                if len(set(core_labels)) == 1:
                    # 同类型多个候选: 用 CLIP 区分属性
                    clip_ranked = self._clip_attribute_disambiguate(
                        instruction, top_candidates, clip_encoder
                    )
                    if clip_ranked:
                        best_obj, best_score, best_reason = clip_ranked[0]
                        best_reason += " [CLIP-attr-disambig]"

        # ── 距离衰减 (近距离目标优先) ──
        if robot_position:
            def _get_xy(obj_dict):
                """统一获取物体 x, y 坐标（支持 dict / list 格式）。"""
                p = obj_dict.get("position", {})
                if isinstance(p, (list, tuple)):
                    return (p[0] if len(p) > 0 else 0), (p[1] if len(p) > 1 else 0)
                return p.get("x", 0), p.get("y", 0)

            bx, by = _get_xy(best_obj)
            rx, ry = robot_position.get("x", 0), robot_position.get("y", 0)
            dist = math.sqrt((bx - rx) ** 2 + (by - ry) ** 2)
            # 如果有相近分数但更近的候选, 考虑切换
            for obj2, sc2, _ in scored[1:3]:
                if sc2 > best_score * 0.9:  # 分数差距 < 10%
                    o2x, o2y = _get_xy(obj2)
                    dist2 = math.sqrt((o2x - rx) ** 2 + (o2y - ry) ** 2)
                    if dist2 < dist * 0.5:  # 近一倍以上 → 切换
                        best_obj, best_score, best_reason = obj2, sc2, _
                        break

        # ── 判断是否够格走 Fast Path ──
        if best_score < self._fast_path_threshold:
            logger.info(
                "Fast path score %.2f < threshold %.2f, deferring to Slow path. "
                "Best: %s (%s)",
                best_score, self._fast_path_threshold,
                best_obj.get("label", "?"), best_reason,
            )

            # ── Phantom 节点探索目标 (Belief Scene Graphs) ──
            # Fast Path miss 时, 检查 KG 推断的 phantom (blind) 节点。
            # 如果存在高概率 phantom 与指令匹配, 引导探索前往确认。
            phantom_nodes = sg.get("phantom_nodes", [])
            if phantom_nodes and keywords:
                phantom_scored = []
                for pn in phantom_nodes:
                    p_label = pn.get("label", "").lower()
                    p_exist = pn.get("P_exist", 0.0)
                    # 匹配指令关键词
                    label_match = any(
                        kw in p_label or p_label in kw for kw in keywords
                    )
                    if label_match and p_exist > 0.3:
                        phantom_scored.append((pn, p_exist))

                if phantom_scored:
                    phantom_scored.sort(key=lambda x: x[1], reverse=True)
                    best_pn, best_p_exist = phantom_scored[0]
                    pn_pos = best_pn.get("position", {})
                    pn_x = pn_pos.get("x", 0.0)
                    pn_y = pn_pos.get("y", 0.0)
                    logger.info(
                        "Phantom node hit: '%s' P_exist=%.2f at (%.2f, %.2f), "
                        "room=%s",
                        best_pn.get("label", "?"), best_p_exist,
                        pn_x, pn_y, best_pn.get("room_type", "?"),
                    )
                    return GoalResult(
                        action="explore",
                        target_x=pn_x,
                        target_y=pn_y,
                        target_label=f"phantom:{best_pn.get('label', '')}",
                        confidence=best_p_exist * 0.7,
                        reasoning=(
                            f"Phantom node: expected {best_pn.get('label', '?')} "
                            f"in {best_pn.get('room_type', '?')} "
                            f"(P_exist={best_p_exist:.2f})"
                        ),
                        is_valid=True,
                        path="fast",
                        frame_id=sg.get("frame_id", FAST_PATH_MAP_FRAME_ID),
                    )

            # ── Room 级 fallback: 区域指令匹配 ("去厨房"、"到走廊") ──
            room_result = self._try_room_fallback(
                instruction, keywords, sg, clip_encoder,
            )
            if room_result is not None:
                return room_result

            return None  # 不够确定, 交给 LLM

        # ── BA-HSG: 初始化多假设目标信念 ──
        if len(scored) >= 2:
            candidates_for_belief = []
            for obj_dict, fused_sc, _ in scored[:8]:
                pos_d = obj_dict.get("position", {})
                # 统一转换为 [x, y, z] 列表（支持 dict 和 list 格式）
                if isinstance(pos_d, (list, tuple)):
                    pos_xyz = [
                        pos_d[0] if len(pos_d) > 0 else 0,
                        pos_d[1] if len(pos_d) > 1 else 0,
                        pos_d[2] if len(pos_d) > 2 else 0,
                    ]
                else:
                    pos_xyz = [pos_d.get("x", 0), pos_d.get("y", 0), pos_d.get("z", 0)]
                candidates_for_belief.append({
                    "id": obj_dict.get("id", -1),
                    "label": obj_dict.get("label", ""),
                    "position": pos_xyz,
                    "fused_score": fused_sc,
                    "belief": obj_dict.get("belief", {}),
                    "room_match": 0.5,
                })
            self._belief_manager.init_from_candidates(candidates_for_belief, instruction)

            # 使用 Belief-GoalNav 多假设选择
            robot_pos = (
                [robot_position.get("x", 0), robot_position.get("y", 0)]
                if robot_position else None
            )
            selected = self._belief_manager.select_next_target(robot_pos)
            if selected and not self._belief_manager.is_converged:
                logger.info(
                    "BA-HSG multi-hypothesis: %d active candidates, "
                    "top posterior=%.3f (%s)",
                    self._belief_manager.num_active,
                    selected.posterior,
                    selected.label,
                )

        pos = best_obj.get("position", {})
        # position 可能是 dict {"x":...} 或 list [x, y, z]，统一处理
        if isinstance(pos, (list, tuple)):
            px, py, pz = (pos[0] if len(pos) > 0 else 0,
                          pos[1] if len(pos) > 1 else 0,
                          pos[2] if len(pos) > 2 else 0)
        else:
            px, py, pz = pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)

        logger.info(
            "Fast path hit: '%s' at (%.2f, %.2f), score=%.2f [%s]",
            best_obj.get("label", "?"),
            px, py,
            best_score, best_reason,
        )

        # 从场景图获取坐标系，默认为 map
        target_frame = sg.get("frame_id", FAST_PATH_MAP_FRAME_ID)

        # ── 来源降级: KG 先验只做搜索区域, 不做精确导航目标 ──
        best_source = best_obj.get("source", "observed")
        if best_source == "kg_prior":
            logger.info(
                "Fast path KG-prior: '%s' → action=explore (搜索区域, 非精确目标)",
                best_obj.get("label", "?"),
            )
            return GoalResult(
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
            )

        return GoalResult(
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
        )

    @staticmethod
    def _extract_core_noun(label: str) -> str:
        """
        提取标签的核心名词（去掉颜色等修饰词）。

        例如:
            "red chair" → "chair"
            "blue door" → "door"
            "fire extinguisher" → "fire extinguisher" (保持不变)

        Args:
            label: 物体标签

        Returns:
            核心名词
        """
        # 常见颜色词
        colors = {
            "red", "blue", "green", "yellow", "white", "black", "gray", "grey",
            "orange", "purple", "pink", "brown", "cyan", "magenta",
            "红色", "蓝色", "绿色", "黄色", "白色", "黑色", "灰色",
            "橙色", "紫色", "粉色", "棕色", "红", "蓝", "绿", "黄", "白", "黑", "灰"
        }

        # 分词
        words = label.split()

        # 去掉颜色词
        core_words = [w for w in words if w.lower() not in colors]

        if core_words:
            return " ".join(core_words)
        else:
            # 如果全是颜色词，返回原标签
            return label

    @staticmethod
    def _get_object_ids_in_top_rooms(
        instruction: str,
        scene_graph: dict,
        clip_encoder,
        top_k: int = 2,
    ) -> set | None:
        """
        GAP: Room CLIP 筛选 (FSR-VLN 路线 A + HOV-SG view embeddings)。
        优先用 rooms JSON 中的 clip_feature（HOV-SG view embeddings 均值）做
        文本-图像相似度匹配；无特征时 fallback 到 label 文本匹配。
        无 clip/rooms 时返回 None（不做筛选）。
        """
        if clip_encoder is None:
            return None

        # ── 路线1: HOV-SG — rooms JSON 中的 clip_feature ──
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

        # ── 路线2: fallback — subgraphs label 文本匹配（原有逻辑）──
        subgraphs = scene_graph.get("subgraphs", [])
        rooms = [s for s in subgraphs if s.get("level") == "room"]
        if len(rooms) < 2:
            return None
        obj_by_id = {
            o.get("id"): o for o in scene_graph.get("objects", [])
            if o.get("id") is not None
        }
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
        """
        B7: 用 CLIP 对同类型多候选做属性消歧。

        当场景中有 "red chair" 和 "blue chair" 而指令说 "找红色椅子" 时,
        CLIP 能通过视觉-语言对齐区分属性。

        Args:
            instruction: 用户指令
            candidates: (object_dict, score, reason) 列表
            clip_encoder: CLIPEncoder 实例

        Returns:
            按 CLIP 相似度重排后的候选列表
        """
        clip_scored = []
        for obj, fused_score, reason in candidates:
            clip_feature = obj.get("clip_feature")
            if clip_feature is not None:
                try:
                    feat = np.array(clip_feature)
                    if feat.size > 0:
                        sims = clip_encoder.text_image_similarity(
                            instruction, [feat]
                        )
                        clip_sim = sims[0] if sims else 0.0
                        # 融合: 70% 原始分 + 30% CLIP属性匹配
                        combined = 0.7 * fused_score + 0.3 * clip_sim
                        new_reason = (
                            f"{reason}, clip_attr={clip_sim:.3f}, "
                            f"combined={combined:.3f}"
                        )
                        clip_scored.append((obj, combined, new_reason))
                        continue
                except (ValueError, TypeError, AttributeError) as e:
                    logger.debug("CLIP attribute disambiguate failed for '%s': %s",
                                 obj.get("label", "?"), e)
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
        """Room 级 fallback: 当物体匹配失败时, 尝试匹配区域/房间。

        支持 "去厨房"、"到走廊"、"find the kitchen" 等区域级指令。
        从 scene_graph["rooms"] 中提取 room name/center/clip_feature,
        用关键词匹配或 CLIP 相似度评分, 返回 room center 作为导航目标。
        """
        from .goal_resolver import GoalResult

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

            # 方式 1: CLIP 文本-图像相似度 (room 有 clip_feature 时)
            if clip_encoder is not None and room.get("clip_feature") is not None:
                try:
                    rf = np.array(room["clip_feature"])
                    if rf.size > 0:
                        sims = clip_encoder.text_image_similarity(
                            instruction, [rf]
                        )
                        if sims:
                            match_score = max(match_score, sims[0])
                except (ValueError, TypeError, AttributeError):
                    pass

            # 方式 2: 关键词文本匹配
            for kw in keywords:
                kw_l = kw.lower()
                if kw_l in name_lower or name_lower in kw_l:
                    match_score = max(match_score, 0.8)
                    break

            # 方式 3: 直接子串匹配 (指令包含房间名 或 房间名包含指令关键部分)
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
            room_name, best_match, confidence, cx, cy,
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
        """
        从指令中提取关键词 (使用jieba精确分词)。

        升级说明 (P0任务):
        - 原实现: 简单regex分词，中文按字符组
        - 新实现: jieba精确分词，支持自定义词典
        - 回退: jieba未安装时自动回退到简单分词

        参考: SEMANTIC_NAV_REPORT.md 第11.1节
        """
        import re as _re

        stop_words = {
            "the", "a", "an", "to", "go", "find", "get", "me", "for", "and", "or",
            "is", "at", "in", "on", "near", "next", "by", "of", "with", "from",
            "去", "到", "找", "拿", "的", "在", "旁边", "附近", "那个",
            "请", "帮", "我", "一个", "把", "了", "着", "过",
        }

        # 先分离中英文 — 避免jieba在混合文本上的切分错误
        chinese_parts = _re.findall(r'[\u4e00-\u9fff]+', instruction)
        english_parts = _re.findall(r'[a-zA-Z]+', instruction.lower())
        chinese_text = " ".join(chinese_parts)

        all_keywords: list[str] = []

        # 英文: 简单分词 (停用词过滤 + 长度过滤)
        for w in english_parts:
            w_lower = w.lower()
            if w_lower not in stop_words and len(w_lower) > 1:
                all_keywords.append(w_lower)

        # 中文: 使用jieba精确分词 (如果有的话)
        if chinese_text:
            try:
                from .chinese_tokenizer import extract_keywords
                zh_keywords = extract_keywords(
                    chinese_text,
                    min_length=2,
                    filter_stopwords=True,
                    keep_colors=True,
                    keep_spatial=True,
                )
                all_keywords.extend(zh_keywords)
            except ImportError:
                # 回退: 中文按连续字符组
                zh_tokens = _re.findall(r'[\u4e00-\u9fff]+', chinese_text)
                all_keywords.extend(
                    t for t in zh_tokens if t not in stop_words and len(t) > 1
                )

        deduped = list(set(all_keywords))

        # ── 双语扩展: 中文关键词 → 补英文, 英文关键词 → 补中文 ──
        # 解决: YOLO 输出英文标签, 用户说中文 → Fast Path label 匹配断掉
        try:
            from .chinese_tokenizer import expand_bilingual
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
        """
        从指令中解析主语 (导航目标) 和修饰语 (空间参考物)。

        B6 升级: 三级解析策略
          1. 规则匹配 (regex) — 最快, 处理常见句式
          2. 场景标签顺序匹配 — 兜底
          3. LLM 回退 — 复杂指令 (如 "找到书房里靠窗的那把红色椅子")

        Args:
            inst_lower: 指令小写
            keywords: 提取的关键词
            scene_labels: 场景图中所有物体标签 (小写)

        Returns:
            (subject_labels, modifier_labels)
        """
        # 第 1 级: 规则匹配
        subjects, modifiers = self._parse_roles_regex(inst_lower, keywords, scene_labels)
        if subjects:
            return subjects, modifiers

        # 第 2 级: 场景标签顺序匹配
        subjects, modifiers = self._parse_roles_scene_order(inst_lower, keywords, scene_labels)
        if subjects:
            return subjects, modifiers

        # Level 1 + Level 2 都未匹配 — 全部当主语
        # 注意: 不在此处调用 LLM (原 Level 3)，因为:
        #   1. 此函数在 Fast Path (<200ms) 同步调用，LLM 阻塞 ~5s 违反性能目标
        #   2. 需要 LLM 解析的复杂指令应走 Slow Path (AdaNav 熵触发)
        #   3. 全部当主语是安全的 fallback — 置信度阈值会过滤误匹配
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
        """第 1 级: 正则匹配常见句式。"""
        subjects: list[str] = []
        modifiers: list[str] = []

        en_patterns = [
            r'\b(?:find|go\s+to|navigate\s+to|locate|get)\s+([\w\s]+?)\s+(?:near|by|beside|next\s+to|behind|in\s+front\s+of|left\s+of|right\s+of|on|under|above|below)\s+(?:the\s+)?([\w\s]+)',
            r'\b([\w]+)\s+(?:near|by|beside|next\s+to)\s+(?:the\s+)?([\w]+)',
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
            r'([\u4e00-\u9fff]+?)(?:旁边|附近|左边|右边|前面|后面|上面|下面|对面|里面)的([\u4e00-\u9fff]+)',
            r'(?:去|到|找|找到|寻找)([\u4e00-\u9fff]+)',
        ]

        # 双语扩展: 将中文捕获组扩展为英文, 以匹配英文场景标签
        try:
            from .chinese_tokenizer import translate_label as _tl
        except ImportError:
            _tl = None

        def _match_label(text: str, lbl: str) -> bool:
            """检查 text 和 lbl 是否匹配 (含双语扩展)。"""
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
        """第 2 级: 按指令中出现顺序, 第一个匹配场景物体的词为主语。"""
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
