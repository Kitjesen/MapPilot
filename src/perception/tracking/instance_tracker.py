"""
实例追踪器 — 跨帧目标匹配 + 去重 + 位置平滑 + 空间关系图。

参考论文:
  - ConceptGraphs (ICRA 2024): 增量式 3D 语义场景图, 物体间空间关系
  - SG-Nav (NeurIPS 2024): 层次场景图 (房间→物体→关系) + LLM 推理
  - LOVON (2024/2025): 物体追踪 + EMA 位置平滑 + Unitree 四足验证
  - DovSG (IEEE RA-L 2025): 动态开放词汇 3D 场景图 + 局部更新
  - OpenFunGraph (CVPR 2025 Highlight): 功能性场景图 + 可供性
  - SPADE (IROS 2025): 层次化场景图路径规划 + 四足验证
  - EmbodiedRAG (2024-10): 场景图 RAG 检索, token 减 10x

场景图结构 v2 (Floor → Room → Group → Object):
  {
    "floors": [...],
    "rooms": [...],
    "groups": [...],
    "objects": [...],
    "relations": [...],
    "topology_edges": [...],
    "kg_enrichment": {...}
  }
"""

import asyncio
import logging
import math
import threading
import time
from collections.abc import Callable

import numpy as np

from memory.spatial.room_inferencer import RoomInferencer
from runtime.config import get_config
from runtime.runtime_interface import map_frame_id

from .projection import Detection3D
from .scene_graph_builder import SceneGraphBuilder
from .tracker_knowledge import TrackerKnowledgeEnrichment
from .tracker_queries import InstanceTrackerQueries
from .tracker_serializer import InstanceTrackerSerializer
from .tracker_storage import InstanceTrackerStorage

INSTANCE_TRACKER_MAP_FRAME_ID = map_frame_id()

# ── 从子模块导入所有公开符号 (向后兼容: 外部 from .instance_tracker import X 继续有效) ──
from .tracked_objects import (
    REGION_CLUSTER_RADIUS,
    RELATION_NEAR_THRESHOLD,
    SAFETY_PRIOR_ALPHA_SCALE,
    SAFETY_THRESHOLDS_INTERACTION,
    SAFETY_THRESHOLDS_NAVIGATION,
    BeliefMessage,
    FloorNode,
    PhantomNode,
    Region,
    RoomNode,
    RoomTypePosterior,
    TrackedObject,
    ViewNode,
    infer_room_type,
)

logger = logging.getLogger(__name__)


class BeliefPropagationMixin:
    """Lazy proxy for memory.knowledge.belief.propagation.BeliefPropagationMixin.

    Uses __getattr__ to defer importing the real mixin until a method is
    first accessed.  This avoids eager cross-package (semantic -> memory)
    imports at module load time.
    """

    _real_cls = None

    @classmethod
    def _resolve(cls):
        if cls._real_cls is None:
            from memory.knowledge.belief.propagation import (
                BeliefPropagationMixin as _Real,
            )

            cls._real_cls = _Real
        return cls._real_cls

    def __getattr__(self, name):
        real = self._resolve()
        attr = getattr(real, name)
        if callable(attr):
            import functools

            return functools.partial(attr, self)
        return attr


class InstanceTracker(BeliefPropagationMixin):
    """
    维护全局物体实例表 (USS-Nav 双指标优先级融合)。

    匹配策略 (USS-Nav §IV-C):
      优先级 1 (Semantic Match):
        Ωsem(vi, vj) > sem_threshold (0.75) 且 Ωgeo(Ci, Cj) > geo_weak (0.1)
      优先级 2 (Geometric Match):
        Ωgeo(Ci, Cj) > geo_strong (0.5) 且 Ωgeo(Cj, Ci) > geo_strong (0.5)
      Fallback: 同类别 + 空间距离 < merge_distance

    Phase 3 refactor: scene-graph construction is delegated to
    SceneGraphBuilder, and room inference is delegated to RoomInferencer.

    Phase 4: all numeric thresholds are loaded from runtime config
    (perception.tracking.*) with the previous hard-coded values as defaults.
    """

    def __init__(
        self,
        merge_distance: float | None = None,
        iou_threshold: float | None = None,
        clip_threshold: float | None = None,
        max_objects: int | None = None,
        stale_timeout: float | None = None,
        max_views: int | None = None,
        knowledge_graph=None,
        use_hungarian_matching: bool | None = None,
        enable_dedup_merge: bool | None = None,
        dedup_distance: float | None = None,
        dedup_clip_threshold: float | None = None,
        dedup_time_window: float | None = None,
    ):
        # Resolve tunable defaults from robot_config.yaml; explicit kwargs win.
        cfg = get_config().perception.tracking
        self.merge_distance = merge_distance if merge_distance is not None else cfg.merge_distance
        self.iou_threshold = iou_threshold if iou_threshold is not None else cfg.iou_threshold
        self.clip_threshold = clip_threshold if clip_threshold is not None else cfg.clip_threshold
        self.max_objects = max_objects if max_objects is not None else cfg.max_objects
        self.stale_timeout = stale_timeout if stale_timeout is not None else cfg.stale_timeout
        self.max_views = max_views if max_views is not None else cfg.max_views

        # Stage 1a matching improvements (gated; default OFF -> legacy behavior).
        self.use_hungarian_matching = (
            use_hungarian_matching if use_hungarian_matching is not None else cfg.use_hungarian_matching
        )
        self.enable_dedup_merge = enable_dedup_merge if enable_dedup_merge is not None else cfg.enable_dedup_merge
        self.dedup_distance = dedup_distance if dedup_distance is not None else cfg.dedup_distance
        self.dedup_clip_threshold = (
            dedup_clip_threshold if dedup_clip_threshold is not None else cfg.dedup_clip_threshold
        )
        self.dedup_time_window = dedup_time_window if dedup_time_window is not None else cfg.dedup_time_window

        # USS-Nav fusion thresholds (§IV-C)
        self.SEM_THRESHOLD = cfg.sem_threshold
        self.GEO_WEAK_THRESHOLD = cfg.geo_weak_threshold
        self.GEO_STRONG_THRESHOLD = cfg.geo_strong_threshold
        self.GEO_POINT_DIST_TAU = cfg.geo_point_dist_tau
        self.CANDIDATE_RADIUS = cfg.candidate_radius

        # FOV check parameters (OneMap: only record negative evidence for visible objects)
        self.FOV_HALF_ANGLE = cfg.fov_half_angle
        self.FOV_MAX_RANGE = cfg.fov_max_range

        self._lock = threading.Lock()  # 线程安全: 保护 _objects 并发访问
        self._objects: dict[int, TrackedObject] = {}
        self._next_id = 0

        # 关键视角层 (view nodes)
        self._views: dict[int, ViewNode] = {}
        self._next_view_id = 0
        self._last_view_id = -1

        # Phase 3: scene-graph construction lives in SceneGraphBuilder.
        self._scene_graph_builder = SceneGraphBuilder(self._objects, self._views)

        # Phase 3: room inference lives in RoomInferencer (memory side).
        self._room_inferencer = RoomInferencer()

        # Phase 3.5: serialization and persistence extracted to dedicated modules.
        self._serializer = InstanceTrackerSerializer()
        self._storage = InstanceTrackerStorage()

        # 知识图谱 (ConceptBot / OpenFunGraph 增强)
        self._knowledge_graph = knowledge_graph

        # Phase 3.5b: query API extracted to InstanceTrackerQueries.
        self._queries = InstanceTrackerQueries(
            self._objects,
            scene_graph_builder=self._scene_graph_builder,
            knowledge_graph=self._knowledge_graph,
        )

        # Phase 3.5c: KG / belief enrichment extracted to TrackerKnowledgeEnrichment.
        self._knowledge = TrackerKnowledgeEnrichment(
            get_objects=lambda: self._objects,
            get_knowledge_graph=lambda: self._knowledge_graph,
            set_knowledge_graph=lambda kg: setattr(self, "_knowledge_graph", kg),
            get_belief_model=lambda: self._belief_model,
            set_belief_model=lambda m: setattr(self, "_belief_model", m),
            queries=self._queries,
        )

        # 楼层层 (SPADE / HOV-SG) — retained on the tracker for consumers that
        # read it directly; the authoritative source is SceneGraphBuilder.
        self._cached_floors: list[FloorNode] = []
        self._cached_regions: list = []

        # ── Loopy Belief Propagation 状态 ──
        self._room_type_posteriors: dict[int, RoomTypePosterior] = {}
        self._phantom_nodes: dict[int, PhantomNode] = {}
        self._next_phantom_id = 0
        self._bp_messages_log: list[BeliefMessage] = []  # 调试用: 最近一轮的消息
        self._bp_iteration_count = 0  # 统计: 总 BP 迭代次数
        self._bp_convergence_history: list[float] = []  # 统计: 每轮最大 Δ
        self._last_bp_time: float = 0.0  # BP 节流: 最多 1Hz

        # ── Neuro-Symbolic Belief GCN (KG-BELIEF) ──
        self._belief_model = None  # BeliefPredictor | None

        # W2-2: warn once if intrinsics_fx is 0 — avoid per-frame log spam,
        # also prevents silently using a default 600 px focal length that
        # would produce wrong 3D extents on any camera that isn't 640×480.
        self._warned_no_fx: bool = False

    @property
    def objects(self) -> dict[int, TrackedObject]:
        with self._lock:
            return dict(self._objects)  # 返回快照，防止外部迭代时并发修改

    @property
    def views(self) -> dict[int, ViewNode]:
        return self._views

    def set_room_namer(self, namer: Callable) -> None:
        """Register the async LLM room-naming callback.

        Phase 3: delegated to RoomInferencer.
        """
        self._room_inferencer.set_room_namer(namer)

    def _is_in_fov(
        self,
        obj_pos: np.ndarray,
        camera_pos: np.ndarray,
        camera_forward: np.ndarray,
    ) -> bool:
        """检查物体是否在相机视锥体内 (OneMap 理念: 只对可见物体记录负面证据)。"""
        diff = obj_pos[:3] - camera_pos[:3]
        dist = np.linalg.norm(diff)
        if dist < 0.1 or dist > self.FOV_MAX_RANGE:
            return False
        cos_angle = np.dot(diff, camera_forward[:3]) / (dist * max(np.linalg.norm(camera_forward[:3]), 1e-7))
        return cos_angle > math.cos(self.FOV_HALF_ANGLE)

    def update(
        self,
        detections: list[Detection3D],
        camera_pos: np.ndarray | None = None,
        camera_forward: np.ndarray | None = None,
        intrinsics_fx: float = 0.0,
    ) -> list[TrackedObject]:
        """
        用本帧检测结果更新全局物体表。

        Args:
            detections: 本帧 3D 检测列表
            camera_pos: 相机世界坐标 [x,y,z] (用于 FOV 检查)
            camera_forward: 相机朝向单位向量 [fx,fy,fz] (用于 FOV 检查)
            intrinsics_fx: 相机焦距 fx (用于 3D 包围盒估算, 0=使用默认 600)

        Returns:
            本帧匹配/新建的 TrackedObject 列表
        """
        # W2-2: warn once if caller forgot to provide camera focal length —
        # fx=0 would otherwise fall back to a 600 px default that mis-scales
        # 3D extents on any non-640x480 camera.
        if intrinsics_fx <= 0.0 and not self._warned_no_fx:
            logger.warning(
                "InstanceTracker.update() called with intrinsics_fx=%s — "
                "3D extent estimation will be skipped for this detection "
                "batch. Pass camera fx from intrinsics to enable it.",
                intrinsics_fx,
            )
            self._warned_no_fx = True

        with self._lock:
            matched: list[TrackedObject] = []

            # Stage 1a: optionally resolve all detection->object assignments
            # globally via the Hungarian algorithm. When the gate is OFF we
            # keep the legacy per-detection greedy path unchanged.
            hungarian_assign: dict[int, TrackedObject] | None = None
            if self.use_hungarian_matching:
                hungarian_assign = self._match_detections_hungarian(detections, list(self._objects.values()))

            for det_idx, det in enumerate(detections):
                # 传递真实 fx 给 _update_extent
                if intrinsics_fx > 0:
                    det._intrinsics_fx = intrinsics_fx
                if hungarian_assign is not None:
                    best_obj = hungarian_assign.get(det_idx)
                else:
                    best_obj = self._find_match(det)
                if best_obj is not None:
                    best_obj.update(det)
                    matched.append(best_obj)
                else:
                    # USS-Nav: 新实例含点云
                    init_points = np.empty((0, 3))
                    if hasattr(det, "points") and det.points is not None and len(det.points) > 0:
                        init_points = det.points.copy()
                    obj = TrackedObject(
                        object_id=self._next_id,
                        label=det.label,
                        position=det.position.copy(),
                        best_score=det.score,
                        last_seen=time.time(),
                        features=det.features.copy() if det.features.size > 0 else np.array([]),
                        points=init_points,
                    )
                    self._knowledge.enrich_from_kg(obj)
                    self._objects[self._next_id] = obj
                    self._next_id += 1
                    matched.append(obj)

            # BA-HSG: 负面证据 — 仅对视锥体内的已知物体记录 miss (OneMap 理念)
            detected_ids = {m.object_id for m in matched}
            has_fov = camera_pos is not None and camera_forward is not None
            for obj in self._objects.values():
                if obj.object_id not in detected_ids and obj.detection_count >= 2:
                    dt = time.time() - obj.last_seen
                    if dt > 5.0:
                        # FOV 检查: 只对确实在视野内但没检测到的物体记录 miss
                        if has_fov:
                            if self._is_in_fov(obj.position, camera_pos, camera_forward):
                                obj.record_miss()
                        else:
                            # 无 FOV 信息时保持原行为 (兼容)
                            obj.record_miss()

            # 清理过期实例
            self._prune_stale()

            # 限制最大数量
            if len(self._objects) > self.max_objects:
                sorted_objs = sorted(
                    self._objects.values(),
                    key=lambda o: (o.credibility, o.detection_count),
                    reverse=True,
                )
                self._objects = {o.object_id: o for o in sorted_objs[: self.max_objects]}

            # Stage 1a: optional post-matching duplicate merge (gated OFF by
            # default). Rewrites `matched` so callers never see a dropped id.
            if self.enable_dedup_merge:
                remap = self._dedup_merge()
                if remap:
                    deduped: list[TrackedObject] = []
                    seen: set[int] = set()
                    for m in matched:
                        target = remap.get(m.object_id, m)
                        if target.object_id not in seen:
                            seen.add(target.object_id)
                            deduped.append(target)
                    matched = deduped

        # BA-HSG: 图扩散传播 (节流: 最多 1Hz, 避免 O(n²) 每帧开销)
        now = time.time()
        if now - self._last_bp_time >= 1.0:
            regions = self._scene_graph_builder.compute_regions(self._room_inferencer.room_name_cache)
            self.propagate_beliefs(regions=regions)
            self._last_bp_time = now

        return matched

    def record_view(
        self,
        camera_position: np.ndarray,
        object_ids: list[int],
        min_distance: float = 1.0,
        min_interval: float = 1.0,
    ) -> ViewNode | None:
        """
        记录关键视角 (view 节点)。

        - 距离/时间与上一个 view 太接近时, 合并更新
        - 否则创建新 view
        """
        if object_ids is None:
            object_ids = []
        unique_ids = list(dict.fromkeys(int(oid) for oid in object_ids if oid in self._objects))
        if not unique_ids:
            return None

        pos = np.asarray(camera_position[:3], dtype=np.float64)
        now = time.time()

        # 尝试与最近 view 合并
        if self._last_view_id in self._views:
            last = self._views[self._last_view_id]
            dist = float(np.linalg.norm(last.position[:2] - pos[:2]))
            dt = now - last.timestamp
            if dist < min_distance and dt < min_interval:
                merged = set(last.object_ids)
                merged.update(unique_ids)
                last.object_ids = sorted(merged)
                last.key_labels = self._collect_labels(last.object_ids, limit=8)
                last.timestamp = now
                return last

        # 创建新 view
        view = ViewNode(
            view_id=self._next_view_id,
            position=pos,
            timestamp=now,
            object_ids=unique_ids,
            key_labels=self._collect_labels(unique_ids, limit=8),
        )
        self._views[self._next_view_id] = view
        self._last_view_id = self._next_view_id
        self._next_view_id += 1

        self._prune_views_if_needed()
        return view

    def _prune_views_if_needed(self) -> None:
        if len(self._views) <= self.max_views:
            return
        # 删除最旧 view
        oldest = sorted(self._views.values(), key=lambda v: v.timestamp)[: len(self._views) - self.max_views]
        for v in oldest:
            self._views.pop(v.view_id, None)

    def _collect_labels(self, object_ids: list[int], limit: int = 8) -> list[str]:
        labels: list[str] = []
        seen = set()
        for oid in object_ids:
            obj = self._objects.get(oid)
            if obj is None:
                continue
            lbl = obj.label
            if lbl not in seen:
                labels.append(lbl)
                seen.add(lbl)
            if len(labels) >= limit:
                break
        return labels

    def _find_match(self, det: Detection3D) -> TrackedObject | None:
        """
        USS-Nav §IV-C: 双指标优先级融合匹配。

        优先级 1 — Semantic Match:
          Ωsem > 0.75 且 Ωgeo > 0.1 (语义强 + 几何弱)
        优先级 2 — Geometric Match:
          Ωgeo(Ci→Cj) > 0.5 且 Ωgeo(Cj→Ci) > 0.5 (双向几何强)
        Fallback — Legacy:
          同类别 + 空间距离 < merge_distance (无点云时的降级路径)
        """
        det_has_points = hasattr(det, "points") and det.points is not None and len(det.points) > 0

        # 空间预过滤: 只考虑 CANDIDATE_RADIUS 内的候选 (USS-Nav 用 ikd-tree 2m 半径)
        candidates = []
        for obj in self._objects.values():
            dist = float(np.linalg.norm(obj.position - det.position))
            if dist < self.CANDIDATE_RADIUS:
                candidates.append((obj, dist))

        if not candidates:
            return None

        # ── 优先级 1: Semantic Match (语义强 + 几何弱) ──
        best_sem_match: TrackedObject | None = None
        best_sem_score = -1.0

        for obj, dist in candidates:
            # 语义相似度 Ωsem (USS-Nav Eq.2: text feature cosine similarity)
            omega_sem = 0.0
            if det.features.size > 0 and obj.features.size > 0:
                omega_sem = self._cosine_similarity(det.features, obj.features)

            if omega_sem <= self.SEM_THRESHOLD:
                continue

            # 几何相似度 Ωgeo (USS-Nav Eq.1: 点云重叠比)
            omega_geo = 0.0
            if det_has_points and len(obj.points) > 0:
                omega_geo = self._geometric_similarity(det.points, obj.points)
            else:
                omega_geo = max(0.0, 1.0 - dist / self.merge_distance)

            if omega_geo >= self.GEO_WEAK_THRESHOLD and omega_sem > best_sem_score:
                best_sem_score = omega_sem
                best_sem_match = obj

        if best_sem_match is not None:
            return best_sem_match

        # ── 优先级 2: Geometric Match (双向几何强) ──
        if det_has_points:
            best_geo_match: TrackedObject | None = None
            best_geo_score = -1.0

            for obj, _dist in candidates:
                if len(obj.points) == 0:
                    continue

                omega_geo_fwd = self._geometric_similarity(det.points, obj.points)
                omega_geo_bwd = self._geometric_similarity(obj.points, det.points)

                if omega_geo_fwd >= self.GEO_STRONG_THRESHOLD and omega_geo_bwd >= self.GEO_STRONG_THRESHOLD:
                    combined = omega_geo_fwd + omega_geo_bwd
                    if combined > best_geo_score:
                        best_geo_score = combined
                        best_geo_match = obj

            if best_geo_match is not None:
                return best_geo_match

        # ── Fallback: 同类别 + 空间距离 (无点云时的降级路径) ──
        best_fallback: TrackedObject | None = None
        best_dist = self.merge_distance

        for obj, dist in candidates:
            if obj.label.lower() != det.label.lower():
                continue
            if dist < best_dist:
                best_dist = dist
                best_fallback = obj

        return best_fallback

    # ════════════════════════════════════════════════════════════
    #  Stage 1a: Hungarian global-optimal matching (gated)
    # ════════════════════════════════════════════════════════════

    def _match_detections_hungarian(
        self,
        detections: list[Detection3D],
        candidates: list[TrackedObject],
    ) -> dict[int, TrackedObject]:
        """Globally optimal detection->object assignment (scipy Hungarian).

        Builds a rectangular cost matrix ``cost[i, j]`` for detection ``i`` and
        candidate object ``j``::

            omega_sem = cosine(det.features, obj.features)          # CLIP sim
            omega_geo = 1 - dist / merge_distance   (dist < merge_distance)
            combined  = 0.6 * omega_sem + 0.4 * omega_geo
            cost      = -combined   if combined > 0.3   else INVALID (999)

        ``scipy.optimize.linear_sum_assignment`` minimises total cost and
        natively supports rectangular matrices (detections != candidates).
        Only assignments landing on a *valid* cell (cost < INVALID) are kept;
        unassigned detections fall through to new-object creation in
        :meth:`update`.

        Returns a mapping ``{detection_index: TrackedObject}``.
        """
        result: dict[int, TrackedObject] = {}
        n_det = len(detections)
        n_cand = len(candidates)
        if n_det == 0 or n_cand == 0:
            return result

        from scipy.optimize import linear_sum_assignment

        INVALID = 999.0
        cost = np.full((n_det, n_cand), INVALID, dtype=np.float64)
        for i, det in enumerate(detections):
            det_has_feat = det.features.size > 0
            for j, obj in enumerate(candidates):
                dist = float(np.linalg.norm(obj.position - det.position))
                omega_sem = 0.0
                if det_has_feat and obj.features.size > 0:
                    omega_sem = self._cosine_similarity(det.features, obj.features)
                omega_geo = 0.0
                if dist < self.merge_distance:
                    omega_geo = 1.0 - dist / self.merge_distance
                combined = 0.6 * omega_sem + 0.4 * omega_geo
                if combined > 0.3:
                    cost[i, j] = -combined

        row_ind, col_ind = linear_sum_assignment(cost)
        for i, j in zip(row_ind, col_ind):
            # Skip filler assignments landing on an invalid (999) cell.
            if cost[i, j] < INVALID:
                result[int(i)] = candidates[int(j)]
        return result

    # ════════════════════════════════════════════════════════════
    #  Stage 1a: post-matching object dedup / merge (gated)
    # ════════════════════════════════════════════════════════════

    def _dedup_merge(self) -> dict[int, TrackedObject]:
        """Merge near-duplicate objects that survived matching.

        Two objects are merged when ALL of the following hold:
          * same label (case-insensitive)
          * position distance < ``dedup_distance``
          * CLIP cosine similarity > ``dedup_clip_threshold``
          * ``|last_seen_a - last_seen_b|`` < ``dedup_time_window``

        The object with the higher ``credibility`` survives; the other is
        fused into it via EMA on position/features and dropped from the table.

        Returns ``{dropped_object_id: surviving_object}`` so callers can
        rewrite any references they hold.
        """
        remap: dict[int, TrackedObject] = {}
        # Higher credibility first so the survivor is always the stronger one.
        objs = sorted(
            self._objects.values(),
            key=lambda o: o.credibility,
            reverse=True,
        )
        dropped: set[int] = set()
        for i in range(len(objs)):
            keep = objs[i]
            if keep.object_id in dropped:
                continue
            for j in range(i + 1, len(objs)):
                drop = objs[j]
                if drop.object_id in dropped:
                    continue
                if keep.label.lower() != drop.label.lower():
                    continue
                dist = float(np.linalg.norm(keep.position - drop.position))
                if dist >= self.dedup_distance:
                    continue
                sim = 0.0
                if keep.features.size > 0 and drop.features.size > 0:
                    sim = self._cosine_similarity(keep.features, drop.features)
                if sim <= self.dedup_clip_threshold:
                    continue
                if abs(keep.last_seen - drop.last_seen) >= self.dedup_time_window:
                    continue
                self._merge_objects(keep, drop)
                dropped.add(drop.object_id)
                remap[drop.object_id] = keep

        for oid in dropped:
            self._objects.pop(oid, None)
        return remap

    @staticmethod
    def _merge_objects(
        keep: TrackedObject,
        drop: TrackedObject,
        ema_alpha: float = 0.5,
    ) -> None:
        """Fuse *drop* into *keep* via EMA on position/features.

        ``keep`` is assumed to be the higher-credibility survivor.
        """
        keep.position = (1.0 - ema_alpha) * keep.position + ema_alpha * drop.position
        if keep.features.size > 0 and drop.features.size > 0:
            fused = (1.0 - ema_alpha) * keep.features + ema_alpha * drop.features
            norm = np.linalg.norm(fused)
            if norm > 0:
                fused = fused / norm
            keep.features = fused
        elif drop.features.size > 0:
            keep.features = drop.features.copy()
        keep.detection_count += drop.detection_count
        keep.last_seen = max(keep.last_seen, drop.last_seen)
        keep.best_score = max(keep.best_score, drop.best_score)

    @staticmethod
    def _geometric_similarity(
        cloud_a: np.ndarray,
        cloud_b: np.ndarray,
        tau: float | None = None,
    ) -> float:
        """
        USS-Nav Eq.1: 几何相似度 — 点云 A 中匹配点的比例。

        Ω(Ci, Cj) = (1/|Ci|) Σ_{p∈Ci} I(min_{q∈Cj} ||p-q|| ≤ τ)

        使用 scipy.spatial.cKDTree 加速最近邻查询。
        """
        if tau is None:
            tau = get_config().perception.tracking.geo_point_dist_tau
        if cloud_a is None or cloud_b is None:
            return 0.0
        if len(cloud_a) == 0 or len(cloud_b) == 0:
            return 0.0

        try:
            from scipy.spatial import cKDTree

            tree_b = cKDTree(cloud_b)
            dists, _ = tree_b.query(cloud_a, k=1)
            matched = np.sum(dists <= tau)
            return float(matched) / len(cloud_a)
        except ImportError:
            diffs = cloud_a[:, None, :] - cloud_b[None, :, :]
            min_dists = np.min(np.linalg.norm(diffs, axis=2), axis=1)
            matched = np.sum(min_dists <= tau)
            return float(matched) / len(cloud_a)

    def _prune_stale(self) -> None:
        """清除长时间未见的实例。"""
        now = time.time()
        stale_ids = [oid for oid, obj in self._objects.items() if now - obj.last_seen > self.stale_timeout]
        for oid in stale_ids:
            del self._objects[oid]

    @staticmethod
    def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
        """余弦相似度。"""
        norm_a = np.linalg.norm(a)
        norm_b = np.linalg.norm(b)
        if norm_a == 0 or norm_b == 0:
            return 0.0
        return float(np.dot(a, b) / (norm_a * norm_b))

    def get_scene_graph_json(self) -> str:
        """
        导出完整场景图为 JSON (SG-Nav 风格)。

        Phase 3.5: delegated to InstanceTrackerSerializer.
        """
        return self._serializer.to_json(self)

    # ════════════════════════════════════════════════════════════
    #  KG 知识增强 — delegated to TrackerKnowledgeEnrichment (Phase 3.5c)
    # ════════════════════════════════════════════════════════════

    def set_knowledge_graph(self, kg) -> None:
        """注入知识图谱 (运行时设置)。Delegated to TrackerKnowledgeEnrichment."""
        self._knowledge.set_knowledge_graph(kg)

    def load_belief_model(self, path: str) -> bool:
        """加载训练好的 KG-BELIEF GCN 模型权重。Delegated to TrackerKnowledgeEnrichment."""
        return self._knowledge.load_belief_model(path)

    def train_belief_model(
        self,
        num_scenes: int = 5000,
        epochs: int = 50,
        save_path: str | None = None,
    ) -> bool:
        """训练 KG-BELIEF GCN 模型 (从 KG 合成数据)。Delegated to TrackerKnowledgeEnrichment."""
        return self._knowledge.train_belief_model(num_scenes=num_scenes, epochs=epochs, save_path=save_path)

    # ════════════════════════════════════════════════════════════
    #  开放词汇查询 — delegated to InstanceTrackerQueries (Phase 3.5b)
    # ════════════════════════════════════════════════════════════

    def query_by_text(
        self,
        query: str,
        top_k: int = 5,
        clip_encoder=None,
    ) -> list[TrackedObject]:
        """按文本查询匹配物体。Delegated to InstanceTrackerQueries."""
        return self._queries.query_by_text(query, top_k=top_k, clip_encoder=clip_encoder)

    def query_spatial(
        self,
        target: str,
        spatial_hint: str = "",
        anchor: str = "",
        top_k: int = 5,
        clip_encoder=None,
    ) -> list[TrackedObject]:
        """空间感知查询。Delegated to InstanceTrackerQueries."""
        return self._queries.query_spatial(
            target, spatial_hint=spatial_hint, anchor=anchor, top_k=top_k, clip_encoder=clip_encoder
        )

    def query_by_affordance(
        self,
        affordance: str,
        top_k: int = 10,
    ) -> list[TrackedObject]:
        """按可供性查询。Delegated to InstanceTrackerQueries."""
        return self._queries.query_by_affordance(affordance, top_k=top_k)

    def query_by_safety(
        self,
        safety_level: str = "dangerous",
    ) -> list[TrackedObject]:
        """查询特定安全等级的物体。Delegated to InstanceTrackerQueries."""
        return self._queries.query_by_safety(safety_level)

    def query_by_floor(
        self,
        floor_level: int,
        label: str | None = None,
        top_k: int = 20,
    ) -> list[TrackedObject]:
        """按楼层查询物体。Delegated to InstanceTrackerQueries."""
        return self._queries.query_by_floor(floor_level, label=label, top_k=top_k)

    def extract_subgraph_for_task(
        self,
        target: str,
        max_nodes: int = 30,
        clip_encoder=None,
    ) -> dict:
        """任务相关子图提取。Delegated to InstanceTrackerQueries."""
        return self._queries.extract_subgraph_for_task(target, max_nodes=max_nodes, clip_encoder=clip_encoder)

    # ════════════════════════════════════════════════════════════
    #  DovSG 动态场景图更新 (IEEE RA-L 2025)
    # ════════════════════════════════════════════════════════════

    def compute_scene_diff(self, prev_snapshot: dict) -> dict:
        """
        计算场景图差异 (DovSG 局部更新核心)。

        Phase 3.5: delegated to InstanceTrackerSerializer.
        """
        return self._serializer.compute_scene_diff(self, prev_snapshot)

    def apply_local_update(
        self,
        region_id: int,
        new_detections: list[Detection3D],
    ) -> dict:
        """
        DovSG 局部更新: 只更新指定区域的物体, 其余保持不变。

        比全量 update() 更高效, 用于:
        - 机器人操作后 (PICK/PLACE) 只更新操作区域
        - 人工干预后 (物体被移动) 只更新受影响区域
        """
        region_obj_ids = set()
        for obj in self._objects.values():
            if obj.region_id == region_id:
                region_obj_ids.add(obj.object_id)

        matched_ids = set()
        new_objects = []

        for det in new_detections:
            best_obj = None
            best_dist = self.merge_distance
            for oid in region_obj_ids:
                obj = self._objects.get(oid)
                if obj is None or obj.label.lower() != det.label.lower():
                    continue
                dist = float(np.linalg.norm(obj.position - det.position))
                if dist < best_dist:
                    best_dist = dist
                    best_obj = obj

            if best_obj is not None:
                best_obj.update(det)
                matched_ids.add(best_obj.object_id)
            else:
                obj = TrackedObject(
                    object_id=self._next_id,
                    label=det.label,
                    position=det.position.copy(),
                    best_score=det.score,
                    last_seen=time.time(),
                    features=det.features.copy() if det.features.size > 0 else np.array([]),
                    region_id=region_id,
                )
                self._knowledge.enrich_from_kg(obj)
                self._objects[self._next_id] = obj
                new_objects.append(obj)
                self._next_id += 1

        unmatched = region_obj_ids - matched_ids
        for oid in unmatched:
            obj = self._objects.get(oid)
            if obj is not None:
                obj.record_miss()

        return {
            "region_id": region_id,
            "updated": len(matched_ids),
            "added": len(new_objects),
            "missed": len(unmatched),
        }

    # ════════════════════════════════════════════════════════════
    #  语义嵌入索引 — delegated to InstanceTrackerQueries (Phase 3.5b)
    # ════════════════════════════════════════════════════════════

    def build_embedding_index(self) -> bool:
        """构建 CLIP 特征索引。Delegated to InstanceTrackerQueries."""
        return self._queries.build_embedding_index()

    def query_by_embedding(
        self,
        query_embedding: np.ndarray,
        top_k: int = 5,
        min_similarity: float = 0.2,
    ) -> list[tuple[TrackedObject, float]]:
        """用 CLIP 嵌入向量查询最相似物体。Delegated to InstanceTrackerQueries."""
        return self._queries.query_by_embedding(query_embedding, top_k=top_k, min_similarity=min_similarity)

    def get_open_vocabulary_matches(
        self,
        query: str,
        clip_encoder=None,
        top_k: int = 5,
    ) -> list[dict]:
        """开放词汇查询 (完整流程)。Delegated to InstanceTrackerQueries."""
        return self._queries.get_open_vocabulary_matches(query, clip_encoder=clip_encoder, top_k=top_k)

    # ════════════════════════════════════════════════════════════
    #  场景图持久化 (长期记忆)
    # ════════════════════════════════════════════════════════════

    def save_to_file(self, path: str) -> None:
        """保存场景图到文件 (长期记忆持久化)。

        Phase 3.5: delegated to InstanceTrackerStorage.
        """
        self._storage.save(self, path)

    def load_from_file(self, path: str) -> bool:
        """从文件恢复场景图 (长期记忆加载)。

        Phase 3.5: delegated to InstanceTrackerStorage.
        """
        return self._storage.load(self, path)

    # ════════════════════════════════════════════════════════════
    #  Public incremental-update API
    # ════════════════════════════════════════════════════════════

    def update_local(
        self,
        new_detections: list["Detection3D"],
        robot_pos: "np.ndarray",
        update_radius: float = 5.0,
    ) -> dict:
        """Process only objects within *update_radius* metres of *robot_pos*.

        Objects outside the radius are left untouched but listed under
        ``decayed`` so callers know they were skipped.

        Returns
        -------
        dict with keys:
          added   — list of object_ids added in this call
          updated — list of object_ids whose position/features were refreshed
          decayed — list of object_ids outside the update radius (not touched)
        """
        in_radius_ids: set = set()
        out_radius_ids: list = []
        for oid, obj in self._objects.items():
            dist = float(np.linalg.norm(obj.position[:2] - robot_pos[:2]))
            if dist <= update_radius:
                in_radius_ids.add(oid)
            else:
                out_radius_ids.append(oid)

        # Snapshot of object-ids before update for diff
        before_ids = set(self._objects.keys())

        # Run detections through the main update pipeline
        self.update(new_detections)

        after_ids = set(self._objects.keys())
        added_ids = list(after_ids - before_ids)
        updated_ids = [oid for oid in before_ids if oid in after_ids and oid in in_radius_ids]

        return {
            "added": added_ids,
            "updated": updated_ids,
            "decayed": out_radius_ids,
        }

    def remove_stale_objects(
        self,
        max_age: float | None = None,
        stale_timeout_sec: float | None = None,
        min_confidence: float = 0.0,
    ) -> list[str]:
        """Remove objects that have not been seen recently.

        Parameters
        ----------
        max_age / stale_timeout_sec:
            Age threshold in seconds (either keyword accepted).
            Defaults to the instance's ``stale_timeout``.
        min_confidence:
            Objects with credibility >= this value are kept even if stale
            (set to 0.0 to remove all stale regardless of confidence).
        """
        now = time.time()
        timeout = stale_timeout_sec if stale_timeout_sec is not None else max_age
        if timeout is None:
            timeout = self.stale_timeout
        stale = [
            obj
            for obj in list(self._objects.values())
            if now - obj.last_seen > timeout and obj.credibility < min_confidence
        ]
        for obj in stale:
            self._objects.pop(obj.object_id, None)
        return [str(obj.object_id) for obj in stale]

    # ════════════════════════════════════════════════════════════
    #  Scene-graph / room compatibility wrappers
    # ════════════════════════════════════════════════════════════

    # These methods used to be provided by RoomManagerMixin.  Thin wrappers
    # are preserved so existing callers (tests, notebooks, downstream modules)
    # keep working while the implementation lives in SceneGraphBuilder and
    # RoomInferencer.

    def compute_regions(self) -> list[Region]:
        """Backward-compatible wrapper around SceneGraphBuilder.compute_regions."""
        return self._scene_graph_builder.compute_regions(self._room_inferencer.room_name_cache)

    def compute_spatial_relations(self) -> list:
        """Backward-compatible wrapper around SceneGraphBuilder.compute_spatial_relations."""
        return self._scene_graph_builder.compute_spatial_relations()

    def compute_groups(self, regions: list[Region]) -> list:
        """Backward-compatible wrapper around SceneGraphBuilder.compute_groups."""
        return self._scene_graph_builder.compute_groups(regions)

    def compute_rooms(
        self,
        regions: list[Region],
        groups: list,
    ) -> list[RoomNode]:
        """Backward-compatible wrapper around RoomInferencer.compute_rooms."""
        return self._room_inferencer.compute_rooms(self._objects, regions, groups)

    def query_rooms_by_embedding(
        self,
        embedding: np.ndarray,
        top_k: int = 3,
    ) -> list[tuple[RoomNode, float]]:
        """Backward-compatible wrapper around RoomInferencer.query_rooms_by_embedding."""
        return self._room_inferencer.query_rooms_by_embedding(self._room_inferencer.last_rooms, embedding, top_k)

    def compute_topology_edges(self, rooms: list[RoomNode]) -> list[dict]:
        """Backward-compatible wrapper around SceneGraphBuilder.compute_topology_edges."""
        return self._scene_graph_builder.compute_topology_edges(rooms)

    def compute_floors(self) -> list[FloorNode]:
        """Backward-compatible wrapper around SceneGraphBuilder.compute_floors."""
        return self._scene_graph_builder.compute_floors()

    def assign_rooms_to_floors(
        self,
        floors: list[FloorNode],
        rooms: list[RoomNode],
    ) -> None:
        """Backward-compatible wrapper around SceneGraphBuilder.assign_rooms_to_floors."""
        self._scene_graph_builder.assign_rooms_to_floors(floors, rooms)

    def get_scene_graph_diff_json(self, prev_snapshot: dict) -> str:
        """Return a JSON string describing changes since *prev_snapshot*.

        Phase 3.5: delegated to InstanceTrackerSerializer.

        Parameters
        ----------
        prev_snapshot:
            A dict previously returned by ``json.loads(get_scene_graph_json())``.
        """
        return self._serializer.diff_to_json(self, prev_snapshot)
