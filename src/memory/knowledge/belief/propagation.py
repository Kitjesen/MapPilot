"""
belief_propagation.py — KG-Augmented Loopy Belief Propagation Mixin

包含从 InstanceTracker 提取的信念传播相关方法:
  - propagate_beliefs() — 主入口, 多轮迭代 BP
  - _bp_phase_upward()  — Phase 1: 上行传播 Object→Room
  - _bp_phase_downward() / _bp_phase_downward_gcn() / _bp_phase_downward_kg_lookup()
                         — Phase 2: 下行传播 Room→Object
  - _bp_phase_lateral() — Phase 3: 横向传播 Spatial neighbor sharing
  - _bp_phase_phantom_generation() — Phase 4: Phantom 节点生成
  - get_phantom_nodes(), promote_phantom(), get_room_type_posteriors()
  - get_exploration_targets(), get_bp_diagnostics()
"""

import logging
import math
from typing import TYPE_CHECKING

import numpy as np

from runtime.msgs.scene import (
    BELIEF_LATERAL_SHARE,
    BP_CONVERGENCE_EPS,
    BP_KG_PRIOR_BOOST,
    BP_KG_UNEXPECTED_PENALTY,
    BP_LATERAL_DECAY,
    BP_MAX_ITERATIONS,
    BP_OBJ_TO_ROOM_WEIGHT,
    BP_PHANTOM_BASE_ALPHA,
    BP_PHANTOM_MIN_ROOM_CONFIDENCE,
    BP_ROOM_TO_OBJ_WEIGHT,
    RELATION_NEAR_THRESHOLD,
    SAFETY_PRIOR_ALPHA_SCALE,
    BeliefMessage,
    PhantomNode,
    RoomTypePosterior,
    TrackedObject,
)

if TYPE_CHECKING:
    from runtime.msgs.scene import Region

logger = logging.getLogger(__name__)


class BeliefPropagationMixin:
    """KG-Augmented Loopy Belief Propagation — 信念传播 Mixin。

    依赖 self._objects, self._knowledge_graph, self._belief_model,
    self._room_type_posteriors, self._phantom_nodes, self._next_phantom_id,
    self._bp_messages_log, self._bp_iteration_count, self._bp_convergence_history,
    self.compute_regions() (由 RoomManagerMixin 提供)。
    """

    def propagate_beliefs(self, regions: list["Region"] | None = None) -> None:
        """KG-Augmented Iterative Loopy Belief Propagation (BA-HSG v2 核心).

        与 BSG (ICRA 2024) 的本质区别:
          - BSG 用 5-layer GCN 从 HM3D 训练数据学习 CECI → 我们用 KG 结构化先验 (零训练)
          - BSG 单次 GCN 前向推理 → 我们做多轮迭代 BP 直到收敛
          - BSG 无安全约束 → 我们有 safety-aware 差异化阈值
          - BSG 的 blind nodes 只有存在性概率 → 我们的 phantom nodes 有空间先验 + 安全等级

        与 Commonsense BSG (2025) 的区别:
          - 他们用 LLM 生成空间本体 → 我们直接用 KG 结构化关系 (更快, 可离线)
          - 他们学空间分布 (位置概率) → 我们做信念传播 (存在性 + 房间类型联合推理)

        迭代结构 (每轮):
          Phase 1 — 上行: Object evidence → Room type posterior
          Phase 2 — 下行: Room type posterior → Object KG prior injection
          Phase 3 — 横向: Spatial neighbor belief sharing (distance-weighted)
          Phase 4 — 生成: Room posterior → Phantom nodes for unseen expected objects
          Phase 5 — 收敛检查: max |Δ P_exist| < ε

        Args:
            regions: Optional pre-computed regions.  When None, the mixin falls
                     back to ``self.compute_regions()`` for backward compatibility.
        """
        if not self._objects:
            return

        if regions is None:
            regions = self.compute_regions()
        self._cached_regions = regions

        # 每轮 BP 从头开始计算房间后验和 phantom (避免跨调用残留)
        self._room_type_posteriors.clear()
        self._phantom_nodes.clear()

        # 记录迭代前的 P_exist 快照 (收敛判据)
        prev_beliefs: dict[int, float] = {oid: obj.existence_prob for oid, obj in self._objects.items()}
        self._bp_messages_log.clear()

        for iteration in range(BP_MAX_ITERATIONS):
            # ── Phase 1: 上行传播 — 物体证据 → 房间类型贝叶斯后验 ──
            self._bp_phase_upward(regions)

            # ── Phase 2: 下行传播 — 房间类型后验 + KG → 物体先验注入 ──
            self._bp_phase_downward(regions)

            # ── Phase 3: 横向传播 — 空间邻居信念共享 ──
            self._bp_phase_lateral()

            # ── Phase 4: Phantom 节点生成 — 基于房间后验的盲节点推理 ──
            if iteration == BP_MAX_ITERATIONS - 1:
                self._bp_phase_phantom_generation(regions)

            # ── Phase 5: 收敛检查 ──
            for obj in self._objects.values():
                obj._update_credibility()

            max_delta = 0.0
            for oid, obj in self._objects.items():
                delta = abs(obj.existence_prob - prev_beliefs.get(oid, 0.5))
                max_delta = max(max_delta, delta)
                prev_beliefs[oid] = obj.existence_prob

            self._bp_convergence_history.append(max_delta)
            if len(self._bp_convergence_history) > 100:
                del self._bp_convergence_history[:-100]
            self._bp_iteration_count += 1

            if max_delta < BP_CONVERGENCE_EPS:
                logger.debug("Loopy BP converged at iteration %d (Δ=%.6f)", iteration, max_delta)
                break

    # ──────────────────────────────────────────────────────────
    #  Phase 1: 上行传播 — Object → Room Type Posterior
    # ──────────────────────────────────────────────────────────

    def _bp_phase_upward(self, regions: list["Region"]) -> None:
        """Phase 1: 用观测到的物体标签更新房间类型贝叶斯后验。

        数学公式 (对数域):
          log P(room_type=t | O) ∝ log P(t) + Σ_i log P(o_i | t)

        其中:
          P(t) = 均匀先验 (1/|T|), T = KG 支持的房间类型集合
          P(o_i | t) = {
            0.8  if o_i ∈ KG.get_room_expected_objects(t)  (高似然)
            0.05 if o_i ∉ KG.get_room_expected_objects(t)  (低但非零: 开放世界)
          }

        这等价于 BSG 的 CECI 中 "从观测直方图推断完整直方图",
        但我们不需要 GCN 训练 — KG 直接提供似然函数。
        """
        if self._knowledge_graph is None:
            return

        room_types = [
            "office",
            "kitchen",
            "corridor",
            "meeting_room",
            "bathroom",
            "stairwell",
            "lobby",
            "storage",
            "server_room",
            "warehouse",
            "lab",
            "parking",
            "outdoor",
            "elevator_hall",
            "factory",
            "hospital",
            "entrance",
            "utility_room",
            "bedroom",
            "living_room",
            "break_room",
            "laundry",
        ]

        for region in regions:
            labels_in_room = [self._objects[oid].label.lower() for oid in region.object_ids if oid in self._objects]
            if not labels_in_room:
                continue

            log_posteriors: dict[str, float] = {}
            log_prior = -math.log(max(len(room_types), 1))

            for rtype in room_types:
                expected = set(self._knowledge_graph.get_room_expected_objects(rtype))
                if not expected:
                    continue

                log_likelihood = 0.0
                for lbl in labels_in_room:
                    if lbl in expected:
                        log_likelihood += math.log(0.8)
                    else:
                        log_likelihood += math.log(0.05)

                log_posteriors[rtype] = log_prior + log_likelihood * BP_OBJ_TO_ROOM_WEIGHT

            # Softmax 归一化
            if log_posteriors:
                max_log = max(log_posteriors.values())
                exp_vals = {t: math.exp(lp - max_log) for t, lp in log_posteriors.items()}
                total = sum(exp_vals.values())
                posteriors = {t: v / total for t, v in exp_vals.items()} if total > 0 else {}

                self._room_type_posteriors[region.region_id] = RoomTypePosterior(
                    room_id=region.region_id,
                    hypotheses=posteriors,
                    _log_posteriors=log_posteriors,
                )

    # ──────────────────────────────────────────────────────────
    #  Phase 2: 下行传播 — Room Type Posterior → Object Prior
    # ──────────────────────────────────────────────────────────

    def _bp_phase_downward(self, regions: list["Region"]) -> None:
        """Phase 2: 下行传播 — 房间后验 → 物体先验注入。

        两种模式 (自动切换):
          A. GCN 推理模式 (self._belief_model 可用时):
             用训练好的 KG-BELIEF GCN 预测完整直方图 → 设 alpha/beta
             这是论文的核心贡献: neuro-symbolic GCN with KG features
          B. KG 查表模式 (fallback):
             用 KG.get_room_expected_objects + 房间后验 → 启发式 alpha/beta

        与 BSG (ICRA 2024) 的本质区别:
          BSG: 5-layer GCN, 输入仅 1 通道 (object histogram), 在 HM3D 上训练
          我们: 4-layer GCN, 输入 5 通道 (histogram + KG prior + co-occurrence
                + safety + affordances), 用 safety-weighted loss, KG 合成训练数据
        """
        if self._knowledge_graph is None:
            return

        # ── 模式 A: GCN 推理 (当训练好的模型可用时) ──
        if self._belief_model is not None:
            self._bp_phase_downward_gcn(regions)
            return

        # ── 模式 B: KG 查表 fallback ──
        self._bp_phase_downward_kg_lookup(regions)

    def _bp_phase_downward_gcn(self, regions: list["Region"]) -> None:
        """Phase 2A: 用 KG-BELIEF GCN 推理完整直方图。

        GCN 输出: per-room predicted probability for each object class
        对于每个房间中的物体:
          - predicted_prob > 0.5 → 该物体是 "期望的" → alpha 提升
          - predicted_prob < 0.2 且 detection_count <= 2 → beta 惩罚
        """
        room_labels: list[list[str]] = []
        room_types: list[str] = []
        region_list: list = []

        for region in regions:
            labels = [self._objects[oid].label for oid in region.object_ids if oid in self._objects]
            posterior = self._room_type_posteriors.get(region.region_id)
            rtype = posterior.best_type if posterior else "unknown"
            room_labels.append(labels)
            room_types.append(rtype)
            region_list.append(region)

        if not room_labels:
            return

        try:
            prob_matrix = self._belief_model.predict(room_labels, room_types)
        except Exception as e:
            logger.warning("GCN inference failed, falling back to KG lookup: %s", e)
            self._bp_phase_downward_kg_lookup(regions)
            return

        label2idx = self._belief_model.label2idx

        for r_idx, region in enumerate(region_list):
            probs = prob_matrix[r_idx]
            best_type = room_types[r_idx]

            for oid in region.object_ids:
                obj = self._objects.get(oid)
                if obj is None:
                    continue

                obj_idx = label2idx.get(obj.label.lower(), -1)
                gcn_prob = float(probs[obj_idx]) if obj_idx >= 0 else 0.0

                if gcn_prob > 0.5:
                    delta_a = BP_KG_PRIOR_BOOST * gcn_prob * BP_ROOM_TO_OBJ_WEIGHT
                    obj.kg_prior_alpha += delta_a
                    obj.belief_alpha += delta_a
                    obj.is_kg_expected = True
                    obj.kg_prior_source = f"gcn:{best_type}(p={gcn_prob:.2f})"

                    self._bp_messages_log.append(
                        BeliefMessage(
                            source_type="room",
                            source_id=region.region_id,
                            target_type="object",
                            target_id=oid,
                            message_type="existence",
                            delta_alpha=delta_a,
                            weight=gcn_prob,
                        )
                    )

                elif gcn_prob < 0.2 and obj.detection_count <= 2:
                    delta_b = BP_KG_UNEXPECTED_PENALTY * (1.0 - gcn_prob)
                    obj.belief_beta += delta_b
                    obj.is_kg_expected = False

                    self._bp_messages_log.append(
                        BeliefMessage(
                            source_type="room",
                            source_id=region.region_id,
                            target_type="object",
                            target_id=oid,
                            message_type="existence",
                            delta_beta=delta_b,
                            weight=1.0 - gcn_prob,
                        )
                    )

    def _bp_phase_downward_kg_lookup(self, regions: list["Region"]) -> None:
        """Phase 2B: KG 查表 fallback (无训练模型时使用)。

        用 KG.get_room_expected_objects + 房间后验 → 启发式 alpha/beta.
        """
        for region in regions:
            posterior = self._room_type_posteriors.get(region.region_id)
            if posterior is None:
                continue

            best_type = posterior.best_type
            best_conf = posterior.best_confidence
            if best_type == "unknown" or best_conf < 0.1:
                continue

            for oid in region.object_ids:
                obj = self._objects.get(oid)
                if obj is None:
                    continue

                p_expected = 0.0
                for rtype, rprob in posterior.hypotheses.items():
                    rt_expected = set(self._knowledge_graph.get_room_expected_objects(rtype))
                    if obj.label.lower() in rt_expected:
                        p_expected += rprob

                if p_expected > 0.3:
                    delta_a = BP_KG_PRIOR_BOOST * p_expected * BP_ROOM_TO_OBJ_WEIGHT
                    obj.kg_prior_alpha += delta_a
                    obj.belief_alpha += delta_a
                    obj.is_kg_expected = True
                    obj.kg_prior_source = f"room_type:{best_type}(p={best_conf:.2f})"

                    self._bp_messages_log.append(
                        BeliefMessage(
                            source_type="room",
                            source_id=region.region_id,
                            target_type="object",
                            target_id=oid,
                            message_type="existence",
                            delta_alpha=delta_a,
                            weight=p_expected,
                        )
                    )

                elif p_expected < 0.1 and obj.detection_count <= 2:
                    delta_b = BP_KG_UNEXPECTED_PENALTY * (1.0 - p_expected)
                    obj.belief_beta += delta_b
                    obj.is_kg_expected = False

                    self._bp_messages_log.append(
                        BeliefMessage(
                            source_type="room",
                            source_id=region.region_id,
                            target_type="object",
                            target_id=oid,
                            message_type="existence",
                            delta_beta=delta_b,
                            weight=1.0 - p_expected,
                        )
                    )

    # ──────────────────────────────────────────────────────────
    #  Phase 3: 横向传播 — Spatial Neighbor Belief Sharing
    # ──────────────────────────────────────────────────────────

    def _bp_phase_lateral(self) -> None:
        """Phase 3: 空间邻近物体间的信念共享 (距离加权衰减)。

        公式: Δα_i = share × Σ_{j∈N(i)} exp(-d_ij/τ) × (C_j - 0.5)

        其中 N(i) = {j : dist(i,j) < RELATION_NEAR_THRESHOLD}
        τ = BP_LATERAL_DECAY (衰减系数, 越小衰减越快)

        这比 BSG 的 GCN 邻域聚合更直接, 且不需要学习注意力权重。
        物理直觉: 高可信度物体 "照亮" 附近区域, 让低置信物体更可信。

        优化: 使用 cKDTree 加速近邻查询 (O(n log n) vs O(n²))
        """
        obj_list = list(self._objects.values())
        n = len(obj_list)
        if n < 2:
            return

        positions = np.array([o.position[:3] for o in obj_list])

        if n > 30:
            try:
                from scipy.spatial import cKDTree

                tree = cKDTree(positions[:, :2])
                pairs = tree.query_pairs(r=RELATION_NEAR_THRESHOLD)
            except ImportError:
                diffs = positions[:, None, :2] - positions[None, :, :2]
                dists_2d = np.linalg.norm(diffs, axis=2)
                ii, jj = np.where((dists_2d < RELATION_NEAR_THRESHOLD) & (dists_2d > 0))
                pairs = {(min(a, b), max(a, b)) for a, b in zip(ii, jj)}
        else:
            pairs = [
                (i, j)
                for i in range(n)
                for j in range(i + 1, n)
                if np.linalg.norm(obj_list[i].position[:2] - obj_list[j].position[:2]) < RELATION_NEAR_THRESHOLD
            ]

        for i, j in pairs:
            o1, o2 = obj_list[i], obj_list[j]
            dist = float(np.linalg.norm(o1.position[:2] - o2.position[:2]))
            decay = math.exp(-dist / max(BP_LATERAL_DECAY * RELATION_NEAR_THRESHOLD, 0.1))

            if o2.credibility > o1.credibility + 0.1:
                delta = BELIEF_LATERAL_SHARE * decay * (o2.credibility - 0.5)
                if delta > 0:
                    o1.belief_alpha += delta
            elif o1.credibility > o2.credibility + 0.1:
                delta = BELIEF_LATERAL_SHARE * decay * (o1.credibility - 0.5)
                if delta > 0:
                    o2.belief_alpha += delta

    # ──────────────────────────────────────────────────────────
    #  Phase 4: Phantom Node Generation (Blind Nodes)
    # ──────────────────────────────────────────────────────────

    def _bp_phase_phantom_generation(self, regions: list["Region"]) -> None:
        """Phase 4: 基于房间后验 + KG 生成 phantom (blind) 节点。

        与 BSG (ICRA 2024) blind nodes 的本质区别:
          BSG: GCN 输出直方图 → 高概率类别 → blind node
          我们: KG.get_room_expected_objects(t) × P(t|O) → phantom node

        优势:
          1. 无需训练数据 (BSG 需要 HM3D 生成训练集)
          2. 可解释: 每个 phantom 知道 "为什么被创建" (来源: 哪个房间类型, 多大置信度)
          3. 带安全等级: 危险的 phantom (如 gas_cylinder in lab) 立即触发避障规划
          4. 可增量更新: 当物体被实际检测到, phantom 被 "实体化" (promote)

        Phantom 节点的位置: 房间中心 (未来可用 KG spatial prior 细化)
        """
        if self._knowledge_graph is None:
            return

        self._phantom_nodes.clear()
        self._next_phantom_id = 0

        for region in regions:
            posterior = self._room_type_posteriors.get(region.region_id)
            if posterior is None or posterior.best_confidence < BP_PHANTOM_MIN_ROOM_CONFIDENCE:
                continue

            observed_labels = {self._objects[oid].label.lower() for oid in region.object_ids if oid in self._objects}

            # 对每个高概率房间假设, 收集期望但未见的物体
            phantom_candidates: dict[str, float] = {}  # label → aggregated P
            phantom_safety: dict[str, str] = {}

            for rtype, rprob in posterior.hypotheses.items():
                if rprob < 0.1:
                    continue
                expected = self._knowledge_graph.get_room_expected_objects(rtype)
                for obj_label in expected:
                    if obj_label.lower() not in observed_labels:
                        phantom_candidates[obj_label] = phantom_candidates.get(obj_label, 0.0) + rprob
                        # 从 KG 获取安全等级
                        if obj_label not in phantom_safety:
                            props = self._knowledge_graph.enrich_object_properties(obj_label)
                            phantom_safety[obj_label] = props.get("safety_level", "safe")

            # 创建 phantom 节点 (只保留聚合概率 > 阈值的)
            for label, agg_prob in sorted(phantom_candidates.items(), key=lambda x: -x[1]):
                if agg_prob < BP_PHANTOM_MIN_ROOM_CONFIDENCE:
                    continue

                safety = phantom_safety.get(label, "safe")
                safety_scale = SAFETY_PRIOR_ALPHA_SCALE.get(safety, 1.0)
                alpha = BP_PHANTOM_BASE_ALPHA * agg_prob * safety_scale

                phantom = PhantomNode(
                    phantom_id=self._next_phantom_id,
                    label=label,
                    room_id=region.region_id,
                    room_type=posterior.best_type,
                    position=region.center.copy() if hasattr(region, "center") else np.zeros(3),
                    belief_alpha=alpha,
                    belief_beta=1.0,
                    kg_prior_strength=agg_prob,
                    safety_level=safety,
                    source=f"kg_phantom:{posterior.best_type}(p={agg_prob:.2f})",
                )
                self._phantom_nodes[self._next_phantom_id] = phantom
                self._next_phantom_id += 1

    # ──────────────────────────────────────────────────────────
    #  Phantom Node API
    # ──────────────────────────────────────────────────────────

    def get_phantom_nodes(self) -> list[PhantomNode]:
        """获取当前所有 phantom (blind) 节点, 按存在概率降序排列。"""
        return sorted(
            self._phantom_nodes.values(),
            key=lambda p: p.existence_prob,
            reverse=True,
        )

    def promote_phantom(self, phantom_id: int, detection) -> TrackedObject | None:
        """将 phantom 节点实体化: 当检测到与 phantom 匹配的物体时调用。

        phantom 的先验 α 被继承到新 TrackedObject 中 — 信息不丢失。
        """
        phantom = self._phantom_nodes.pop(phantom_id, None)
        if phantom is None:
            return None

        import time as _time

        obj = TrackedObject(
            object_id=self._next_id,
            label=detection.label,
            position=detection.position.copy(),
            best_score=detection.score,
            last_seen=_time.time(),
            features=detection.features.copy() if detection.features.size > 0 else np.array([]),
            belief_alpha=1.5 + phantom.belief_alpha,  # 继承 phantom 先验
            belief_beta=1.0,
            kg_prior_alpha=phantom.belief_alpha,
            kg_prior_source=phantom.source,
            is_kg_expected=True,
            safety_level=phantom.safety_level,
        )
        self._enrich_from_kg(obj)
        self._objects[self._next_id] = obj
        self._next_id += 1
        return obj

    def get_room_type_posteriors(self) -> dict[int, RoomTypePosterior]:
        """获取所有房间的类型后验分布 (供 LLM 和规划器使用)。"""
        return dict(self._room_type_posteriors)

    def get_exploration_targets(self) -> list[dict]:
        """基于信念状态推荐探索目标 (结合 phantom 和房间不确定性)。

        策略:
          1. 高不确定性房间 (高熵) → 需要更多观测
          2. 高价值 phantom → 有重要的未确认物体
          3. 安全相关 phantom → 优先确认危险物体
        """
        targets = []

        for rtp in self._room_type_posteriors.values():
            if rtp.entropy > 1.5:
                region = None
                for r in getattr(self, "_cached_regions", []):
                    if r.region_id == rtp.room_id:
                        region = r
                        break
                if region is not None:
                    targets.append(
                        {
                            "type": "explore_room",
                            "room_id": rtp.room_id,
                            "reason": f"high_uncertainty (H={rtp.entropy:.2f})",
                            "position": region.center.tolist(),
                            "priority": rtp.entropy,
                        }
                    )

        for phantom in self.get_phantom_nodes()[:10]:
            priority = phantom.existence_prob
            if phantom.safety_level in ("dangerous", "forbidden"):
                priority *= 3.0  # 危险物体优先探索确认
            targets.append(
                {
                    "type": "confirm_phantom",
                    "phantom_id": phantom.phantom_id,
                    "label": phantom.label,
                    "room_id": phantom.room_id,
                    "room_type": phantom.room_type,
                    "reason": f"expected_{phantom.label} (P={phantom.existence_prob:.2f})",
                    "position": phantom.position.tolist(),
                    "priority": priority,
                    "safety_level": phantom.safety_level,
                }
            )

        targets.sort(key=lambda t: -t["priority"])
        return targets

    def get_bp_diagnostics(self) -> dict:
        """获取信念传播诊断信息 (调试和论文实验用)。"""
        return {
            "total_iterations": self._bp_iteration_count,
            "convergence_history": self._bp_convergence_history[-20:],
            "num_room_posteriors": len(self._room_type_posteriors),
            "num_phantom_nodes": len(self._phantom_nodes),
            "num_messages_last_round": len(self._bp_messages_log),
            "room_posteriors": {
                rid: {
                    "best_type": rtp.best_type,
                    "confidence": round(rtp.best_confidence, 3),
                    "entropy": round(rtp.entropy, 3),
                    "top3": sorted(rtp.hypotheses.items(), key=lambda x: -x[1])[:3],
                }
                for rid, rtp in self._room_type_posteriors.items()
            },
            "phantom_summary": [
                {
                    "label": p.label,
                    "room_type": p.room_type,
                    "P_exist": round(p.existence_prob, 3),
                    "safety": p.safety_level,
                }
                for p in self.get_phantom_nodes()[:10]
            ],
        }
