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

from core.runtime_interface import map_frame_id
from core.utils.sanitize import safe_json_dump, safe_json_dumps, sanitize_position
from .projection import Detection3D
from memory.spatial.room_manager import RoomManagerMixin

INSTANCE_TRACKER_MAP_FRAME_ID = map_frame_id()

# ── 从子模块导入所有公开符号 (向后兼容: 外部 from .instance_tracker import X 继续有效) ──
from .tracked_objects import (  # noqa: E402
    REGION_CLUSTER_RADIUS,
    RELATION_NEAR_THRESHOLD,
    SAFETY_PRIOR_ALPHA_SCALE,
    SAFETY_THRESHOLDS_INTERACTION,
    SAFETY_THRESHOLDS_NAVIGATION,
    BeliefMessage,
    FloorNode,
    PhantomNode,
    RoomNode,
    RoomTypePosterior,
    TrackedObject,
    ViewNode,
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


class InstanceTracker(BeliefPropagationMixin, RoomManagerMixin):
    """
    维护全局物体实例表 (USS-Nav 双指标优先级融合)。

    匹配策略 (USS-Nav §IV-C):
      优先级 1 (Semantic Match):
        Ωsem(vi, vj) > sem_threshold (0.75) 且 Ωgeo(Ci, Cj) > geo_weak (0.1)
      优先级 2 (Geometric Match):
        Ωgeo(Ci, Cj) > geo_strong (0.5) 且 Ωgeo(Cj, Ci) > geo_strong (0.5)
      Fallback: 同类别 + 空间距离 < merge_distance
    """

    # USS-Nav 融合阈值 (§IV-C)
    SEM_THRESHOLD = 0.75          # 语义匹配阈值
    GEO_WEAK_THRESHOLD = 0.1     # 弱几何重叠 (语义匹配时的辅助条件)
    GEO_STRONG_THRESHOLD = 0.5   # 强双向几何重叠
    GEO_POINT_DIST_TAU = 0.05    # 点云匹配距离阈值 τ (m), USS-Nav Eq.1
    CANDIDATE_RADIUS = 2.0       # ikd-tree 候选查询半径 (m)

    # FOV 检查参数 (OneMap 理念: 只对视野内物体记录负面证据)
    FOV_HALF_ANGLE = 0.52         # 相机水平半视角 (rad, ~60° FOV → 30°)
    FOV_MAX_RANGE = 5.0           # 最大检测距离 (m)

    def __init__(
        self,
        merge_distance: float = 0.5,
        iou_threshold: float = 0.3,
        clip_threshold: float = 0.75,
        max_objects: int = 200,
        stale_timeout: float = 300.0,
        max_views: int = 300,
        knowledge_graph=None,
    ):
        self.merge_distance = merge_distance
        self.iou_threshold = iou_threshold
        self.clip_threshold = clip_threshold
        self.max_objects = max_objects
        self.stale_timeout = stale_timeout
        self.max_views = max_views

        self._lock = threading.Lock()  # 线程安全: 保护 _objects 并发访问
        self._objects: dict[int, TrackedObject] = {}
        self._next_id = 0

        # 关键视角层 (view nodes)
        self._views: dict[int, ViewNode] = {}
        self._next_view_id = 0
        self._last_view_id = -1

        # Room LLM 命名状态 (创新1: 在线增量场景图补强)
        self._room_llm_namer: Callable | None = None  # async (labels) -> str
        self._room_name_cache: dict[int, str] = {}       # region_id -> LLM name
        self._last_rooms: list[RoomNode] = []              # OneMap: 最近一次 compute_rooms 缓存
        self._llm_pending_tasks: dict[int, asyncio.Task] = {}  # region_id -> pending LLM task
        self._region_stability: dict[int, tuple[frozenset, float]] = {}  # region_id -> (obj_id_set, stable_since)

        # 知识图谱 (ConceptBot / OpenFunGraph 增强)
        self._knowledge_graph = knowledge_graph

        # 楼层层 (SPADE / HOV-SG)
        self._cached_floors: list[FloorNode] = []
        self._cached_regions: list = []

        # ── Loopy Belief Propagation 状态 ──
        self._room_type_posteriors: dict[int, RoomTypePosterior] = {}
        self._phantom_nodes: dict[int, PhantomNode] = {}
        self._next_phantom_id = 0
        self._bp_messages_log: list[BeliefMessage] = []  # 调试用: 最近一轮的消息
        self._bp_iteration_count = 0                      # 统计: 总 BP 迭代次数
        self._bp_convergence_history: list[float] = []    # 统计: 每轮最大 Δ
        self._last_bp_time: float = 0.0                   # BP 节流: 最多 1Hz

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
        """注册 Room LLM 命名回调 (async def namer(labels: list[str]) -> str)。

        创新1 补强: Region 稳定后 (物体数 >= 3 且持续 10s 无变化),
        调用 LLM 将 'area_with_door_chair' 命名为 '走廊' / 'office' 等可读名称,
        提升 LLM 推理时的语义质量。
        """
        self._room_llm_namer = namer

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

            for det in detections:
                # 传递真实 fx 给 _update_extent
                if intrinsics_fx > 0:
                    det._intrinsics_fx = intrinsics_fx
                best_obj = self._find_match(det)
                if best_obj is not None:
                    best_obj.update(det)
                    matched.append(best_obj)
                else:
                    # USS-Nav: 新实例含点云
                    init_points = np.empty((0, 3))
                    if hasattr(det, 'points') and det.points is not None and len(det.points) > 0:
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
                    self._enrich_from_kg(obj)
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
                self._objects = {o.object_id: o for o in sorted_objs[:self.max_objects]}

        # BA-HSG: 图扩散传播 (节流: 最多 1Hz, 避免 O(n²) 每帧开销)
        now = time.time()
        if now - self._last_bp_time >= 1.0:
            self.propagate_beliefs()
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
        det_has_points = (
            hasattr(det, 'points') and det.points is not None and len(det.points) > 0
        )

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

                if (omega_geo_fwd >= self.GEO_STRONG_THRESHOLD
                        and omega_geo_bwd >= self.GEO_STRONG_THRESHOLD):
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

    @staticmethod
    def _geometric_similarity(
        cloud_a: np.ndarray,
        cloud_b: np.ndarray,
        tau: float = 0.05,
    ) -> float:
        """
        USS-Nav Eq.1: 几何相似度 — 点云 A 中匹配点的比例。

        Ω(Ci, Cj) = (1/|Ci|) Σ_{p∈Ci} I(min_{q∈Cj} ||p-q|| ≤ τ)

        使用 scipy.spatial.cKDTree 加速最近邻查询。
        """
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
        stale_ids = [
            oid for oid, obj in self._objects.items()
            if now - obj.last_seen > self.stale_timeout
        ]
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

        包含:
          - objects: 物体列表 (id, label, position, score)
          - relations: 空间关系 (subject→relation→object)
          - regions: 空间区域 (聚类的物体组)
          - summary: 自然语言摘要 (帮助 LLM 理解)
        """
        # 先计算 region, 以更新 object.region_id
        regions = self.compute_regions()

        # 空间关系
        relations = self.compute_spatial_relations()
        relations_list = [
            {
                "subject_id": r.subject_id,
                "relation": r.relation,
                "object_id": r.object_id,
                "distance": r.distance,
            }
            for r in relations
        ]

        # SG-Nav 层次结构: room/group
        groups = self.compute_groups(regions)
        rooms = self.compute_rooms(regions, groups)

        # View 层（关键视角）
        views = [
            ViewNode(
                view_id=v.view_id,
                position=v.position.copy(),
                timestamp=v.timestamp,
                room_id=v.room_id,
                object_ids=list(v.object_ids),
                key_labels=list(v.key_labels),
            )
            for v in self._views.values()
        ]
        views = self._assign_view_rooms(views, rooms)

        hierarchy_edges = self._build_hierarchy_edges(rooms, groups)
        hierarchy_edges.extend(self._build_view_edges(views))

        # 拓扑连通边 (创新4: 房间间连通关系)
        topology_edges = self.compute_topology_edges(rooms)

        # 估算前沿方向 (创新5: 前沿节点 — 已知空间边界的未探索方向)
        frontier_nodes = self._estimate_frontier_directions(rooms)

        # objects (此时 region_id 已更新)
        objects_list = []
        for obj in self._objects.values():
            sp = sanitize_position(obj.position)
            obj_pos = {
                "x": round(sp[0], 3),
                "y": round(sp[1], 3),
                "z": round(sp[2], 3),
            }
            entry = {
                "id": obj.object_id,
                "label": obj.label,
                "position": obj_pos,
                "score": round(obj.best_score, 3),
                "detection_count": obj.detection_count,
                "region_id": obj.region_id,
                "floor_level": obj.floor_level,
                "belief": obj.to_belief_dict(),
                "source": obj.source,
                "last_observed_time": round(obj.last_observed_time, 2),
                "is_simulated": obj.is_simulated,
            }
            if obj.kg_concept_id:
                entry["kg"] = {
                    "concept_id": obj.kg_concept_id,
                    "safety": obj.safety_level,
                    "affordances": obj.affordances,
                }
            objects_list.append(entry)

        regions_list = [
            {
                "region_id": r.region_id,
                "name": r.name,
                "center": {
                    "x": round(float(r.center[0]), 2),
                    "y": round(float(r.center[1]), 2),
                },
                "object_ids": r.object_ids,
            }
            for r in regions
        ]

        rooms_list = [
            {
                "room_id": rm.room_id,
                "name": rm.name,
                "center": {
                    "x": round(float(rm.center[0]), 2),
                    "y": round(float(rm.center[1]), 2),
                },
                "object_ids": rm.object_ids,
                "group_ids": rm.group_ids,
            }
            for rm in rooms
        ]

        groups_list = [
            {
                "group_id": g.group_id,
                "room_id": g.room_id,
                "name": g.name,
                "center": {
                    "x": round(float(g.center[0]), 2),
                    "y": round(float(g.center[1]), 2),
                },
                "object_ids": g.object_ids,
            }
            for g in groups
        ]

        views_list = [
            {
                "view_id": v.view_id,
                "room_id": v.room_id,
                "timestamp": round(float(v.timestamp), 3),
                "position": {
                    "x": round(float(v.position[0]), 2),
                    "y": round(float(v.position[1]), 2),
                    "z": round(float(v.position[2]), 2),
                },
                "object_ids": v.object_ids,
                "key_labels": v.key_labels[:10],
            }
            for v in views
        ]

        subgraphs = self._build_subgraphs(
            rooms=rooms,
            groups=groups,
            views=views,
            objects_by_id=self._objects,
            relations_list=relations_list,
        )

        # Floor 层 (SPADE / HOV-SG)
        floors = self.compute_floors()
        self.assign_rooms_to_floors(floors, rooms)
        floors_list = [
            {
                "floor_id": f.floor_id,
                "floor_level": f.floor_level,
                "z_range": [round(f.z_range[0], 2), round(f.z_range[1], 2)],
                "center_z": round(f.center_z, 2),
                "room_ids": f.room_ids,
                "object_count": len(f.object_ids),
            }
            for f in floors
        ]

        # KG 统计
        kg_stats = {"enriched": 0, "dangerous": 0, "graspable": 0}
        for obj in self._objects.values():
            if obj.kg_concept_id:
                kg_stats["enriched"] += 1
            if obj.safety_level in ("dangerous", "forbidden"):
                kg_stats["dangerous"] += 1
            if "graspable" in obj.affordances:
                kg_stats["graspable"] += 1

        # 自然语言摘要 (帮 LLM 理解, SG-Nav 的 key insight)
        summary_parts = []
        floor_desc = f"{len(floors_list)} floors, " if len(floors_list) > 1 else ""
        summary_parts.append(
            f"Scene has {len(objects_list)} objects, {floor_desc}"
            f"{len(rooms_list)} rooms, {len(groups_list)} groups, {len(views_list)} views."
        )
        if kg_stats["dangerous"] > 0:
            summary_parts.append(
                f"⚠️ {kg_stats['dangerous']} dangerous/forbidden objects detected."
            )
        for rm in rooms:
            labels = [
                self._objects[oid].label
                for oid in rm.object_ids
                if oid in self._objects
            ]
            if labels:
                summary_parts.append(f"{rm.name}: contains {', '.join(labels[:8])}")

        # 关键的近距离关系
        for rel in relations[:10]:  # 限制数量避免 token 爆炸
            subj = self._objects.get(rel.subject_id)
            obj_ = self._objects.get(rel.object_id)
            if subj and obj_:
                summary_parts.append(
                    f"{subj.label} is {rel.relation} {obj_.label} ({rel.distance}m)"
                )

        # 拓扑摘要
        if topology_edges:
            summary_parts.append(
                f"Topology: {len(topology_edges)} room connections"
            )
            for te in topology_edges[:5]:
                fr = te.get("from_room", "?")
                to = te.get("to_room", "?")
                et = te.get("type", "?")
                fr_name = next(
                    (r.name for r in rooms if r.room_id == fr), f"room_{fr}"
                )
                to_name = next(
                    (r.name for r in rooms if r.room_id == to), f"room_{to}"
                )
                summary_parts.append(f"{fr_name} ↔ {to_name} ({et})")

        # ── Belief Propagation 诊断 + Phantom Nodes ──
        phantom_list = [
            {
                "phantom_id": p.phantom_id,
                "label": p.label,
                "room_id": p.room_id,
                "room_type": p.room_type,
                "position": {
                    "x": round(float(p.position[0]), 2),
                    "y": round(float(p.position[1]), 2) if len(p.position) > 1 else 0.0,
                },
                "P_exist": round(p.existence_prob, 3),
                "kg_prior_strength": round(p.kg_prior_strength, 3),
                "safety_level": p.safety_level,
                "source": p.source,
            }
            for p in self.get_phantom_nodes()
        ]

        room_posteriors_list = {}
        for rid, rtp in self._room_type_posteriors.items():
            top3 = sorted(rtp.hypotheses.items(), key=lambda x: -x[1])[:3]
            room_posteriors_list[str(rid)] = {
                "best_type": rtp.best_type,
                "confidence": round(rtp.best_confidence, 3),
                "entropy": round(rtp.entropy, 3),
                "top3": [{t: round(p, 3)} for t, p in top3],
            }

        bp_diag = {
            "iterations": self._bp_iteration_count,
            "convergence": self._bp_convergence_history[-5:]
                if self._bp_convergence_history else [],
        }

        if phantom_list:
            summary_parts.append(
                f"Phantom (blind) nodes: {len(phantom_list)} expected but unseen objects"
            )
            dangerous_phantoms = [p for p in phantom_list if p["safety_level"] in ("dangerous", "forbidden")]
            if dangerous_phantoms:
                summary_parts.append(
                    f"⚠️ {len(dangerous_phantoms)} dangerous phantom objects predicted"
                )

        scene_graph_dict = {
            "timestamp": time.time(),
            "frame_id": INSTANCE_TRACKER_MAP_FRAME_ID,
            "graph_level": "hierarchical",
            "graph_version": "3.0",
            "object_count": len(objects_list),
            "objects": objects_list,
            "relations": relations_list,
            "regions": regions_list,
            "floors": floors_list,
            "rooms": rooms_list,
            "groups": groups_list,
            "views": views_list,
            "hierarchy_edges": hierarchy_edges,
            "topology_edges": topology_edges,
            "frontier_nodes": frontier_nodes,
            "subgraphs": subgraphs,
            "kg_stats": kg_stats,
            "phantom_nodes": phantom_list,
            "room_type_posteriors": room_posteriors_list,
            "belief_propagation": bp_diag,
            "summary": " | ".join(summary_parts),
        }
        return safe_json_dumps(scene_graph_dict)

    # ════════════════════════════════════════════════════════════
    #  KG 知识增强 (ConceptBot OPE / OpenFunGraph)
    # ════════════════════════════════════════════════════════════

    def set_knowledge_graph(self, kg) -> None:
        """注入知识图谱 (运行时设置)。"""
        self._knowledge_graph = kg
        logger.info("KG injected into InstanceTracker (%d existing objects to enrich)",
                     len(self._objects))
        for obj in self._objects.values():
            self._enrich_from_kg(obj)

    def load_belief_model(self, path: str) -> bool:
        """加载训练好的 KG-BELIEF GCN 模型权重。

        加载成功后, _bp_phase_downward 会自动使用 GCN 推理替代 KG 查表。
        """
        if self._knowledge_graph is None:
            logger.warning("Cannot load belief model without KG")
            return False
        try:
            from memory.knowledge.belief.network import BeliefPredictor
            self._belief_model = BeliefPredictor.from_kg(
                self._knowledge_graph, weights_path=path)
            logger.info("Belief GCN model loaded from %s", path)
            return True
        except Exception as e:
            logger.warning("Failed to load belief model: %s", e)
            return False

    def train_belief_model(
        self,
        num_scenes: int = 5000,
        epochs: int = 50,
        save_path: str | None = None,
    ) -> bool:
        """训练 KG-BELIEF GCN 模型 (从 KG 合成数据)。"""
        if self._knowledge_graph is None:
            logger.warning("Cannot train belief model without KG")
            return False
        try:
            import torch

            from memory.knowledge.belief.network import (
                BeliefPredictor,
                KGBeliefGCN,
                KGSceneGraphDataset,
                SafetyWeightedBCELoss,
                build_affordance_vectors,
                build_cooccurrence_matrix,
                build_object_vocabulary,
                build_room_prior_vectors,
                build_safety_loss_weights,
                build_safety_vector,
            )
            from memory.knowledge.belief.network import (
                BeliefTrainer as BTrainer,
            )

            kg = self._knowledge_graph
            label2idx, idx2label = build_object_vocabulary(kg)
            C = len(label2idx)
            cooc = build_cooccurrence_matrix(kg, label2idx)
            safety_vec = build_safety_vector(kg, label2idx)
            aff_mat = build_affordance_vectors(kg, label2idx)
            priors = build_room_prior_vectors(kg, label2idx)
            loss_weights = build_safety_loss_weights(kg, label2idx)

            model = KGBeliefGCN(num_objects=C)
            loss_fn = SafetyWeightedBCELoss(torch.tensor(loss_weights))

            n_train = int(num_scenes * 0.8)
            train_ds = KGSceneGraphDataset(
                kg, label2idx, cooc, safety_vec, aff_mat, priors,
                num_scenes=n_train, seed=42)
            val_ds = KGSceneGraphDataset(
                kg, label2idx, cooc, safety_vec, aff_mat, priors,
                num_scenes=num_scenes - n_train, seed=123)

            trainer = BTrainer(model, loss_fn)
            result = trainer.train(train_ds, val_ds, epochs=epochs)

            predictor = BeliefPredictor(
                model, label2idx, idx2label, cooc, safety_vec, aff_mat, priors)
            self._belief_model = predictor

            if save_path:
                predictor.save_weights(save_path)

            logger.info("Belief GCN trained: %d scenes, %d epochs, best_val=%.4f",
                        num_scenes, epochs, result["best_val_loss"])
            return True
        except Exception as e:
            logger.warning("Failed to train belief model: %s", e)
            return False

    def _enrich_from_kg(self, obj: TrackedObject) -> None:
        """用 KG 补充 TrackedObject 的结构化属性 (ConceptBot OPE) + 安全阈值注入。"""
        if self._knowledge_graph is None:
            return
        props = self._knowledge_graph.enrich_object_properties(obj.label)
        if props.get("kg_matched"):
            obj.kg_concept_id = props.get("concept_id", "")
            obj.safety_level = props.get("safety_level", "safe")
            obj.affordances = props.get("affordances", [])
            obj.functional_properties = props

            # Safety-Aware Differential Thresholds — 根据安全等级设置双阈值
            obj.safety_nav_threshold = SAFETY_THRESHOLDS_NAVIGATION.get(
                obj.safety_level, 0.25)
            obj.safety_interact_threshold = SAFETY_THRESHOLDS_INTERACTION.get(
                obj.safety_level, 0.40)

            # 保护性偏见: 危险物体初始 α 更高 → 更快被 "相信存在" → 更早触发避障
            safety_boost = SAFETY_PRIOR_ALPHA_SCALE.get(obj.safety_level, 1.0)
            if safety_boost > 1.0 and obj.detection_count <= 1:
                obj.belief_alpha += (safety_boost - 1.0) * 0.5

    # ════════════════════════════════════════════════════════════
    #  开放词汇查询 (EmbodiedRAG + CLIP 增强)
    # ════════════════════════════════════════════════════════════

    def query_by_text(
        self,
        query: str,
        top_k: int = 5,
        clip_encoder=None,
    ) -> list[TrackedObject]:
        """
        按文本查询匹配物体。

        匹配策略 (参考 LOVON / ConceptGraphs):
          1. 如果有 CLIP 编码器: 文本 → CLIP 特征 → 与物体 CLIP 特征余弦相似度
          2. Fallback: 字符串子串匹配 (兼容无 CLIP 场景)

        Args:
            query: 查询文本, 如 "红色灭火器" 或 "the red thing near the door"
            top_k: 最多返回数量
            clip_encoder: 可选的 CLIPEncoder 实例

        Returns:
            匹配的 TrackedObject 列表 (按相关度降序)
        """
        objects = list(self._objects.values())
        if not objects:
            return []

        # ── 方式 1: CLIP 语义匹配 (精确) ──
        if clip_encoder is not None:
            features = [obj.features for obj in objects]
            has_features = any(f.size > 0 for f in features)

            if has_features:
                similarities = clip_encoder.text_image_similarity(query, features)
                scored = list(zip(objects, similarities))
                scored.sort(key=lambda x: x[1], reverse=True)
                # 过滤低相似度 (< 0.2 基本无关)
                return [obj for obj, sim in scored[:top_k] if sim > 0.2]

        # ── 方式 2: KG 别名匹配 ──
        if self._knowledge_graph is not None:
            concept = self._knowledge_graph.lookup(query)
            if concept is not None:
                all_names = [n.lower() for n in concept.names_en + concept.names_zh]
                kg_matches = [
                    obj for obj in objects
                    if obj.label.lower() in all_names
                    or any(n in obj.label.lower() for n in all_names)
                ]
                if kg_matches:
                    kg_matches.sort(key=lambda o: o.credibility, reverse=True)
                    return kg_matches[:top_k]

        # ── 方式 3: 字符串匹配 (最终 Fallback) ──
        query_lower = query.lower()
        matches = []

        for obj in objects:
            if query_lower in obj.label.lower() or obj.label.lower() in query_lower:
                matches.append(obj)

        matches.sort(key=lambda o: o.best_score, reverse=True)
        return matches[:top_k]

    def query_spatial(
        self,
        target: str,
        spatial_hint: str = "",
        anchor: str = "",
        top_k: int = 5,
        clip_encoder=None,
    ) -> list[TrackedObject]:
        """
        空间感知查询 (EmbodiedRAG + SG-Nav 空间推理)。

        支持:
        - "门旁边的灭火器" → target=灭火器, spatial_hint=near, anchor=门
        - "桌子上的杯子" → target=杯子, spatial_hint=on, anchor=桌子
        - "3楼的灭火器" → target=灭火器, floor_hint=3

        Args:
            target: 目标物体名称
            spatial_hint: 空间关系 ("near", "on", "left_of", etc.)
            anchor: 参照物名称
            top_k: 返回数量
            clip_encoder: CLIP 编码器

        Returns:
            按相关度排序的物体列表
        """
        candidates = self.query_by_text(target, top_k=50, clip_encoder=clip_encoder)
        if not candidates:
            return []

        if not spatial_hint or not anchor:
            return candidates[:top_k]

        anchor_objs = self.query_by_text(anchor, top_k=10, clip_encoder=clip_encoder)
        if not anchor_objs:
            return candidates[:top_k]

        relations = self.compute_spatial_relations()

        scored = []
        for cand in candidates:
            spatial_score = 0.0
            for anch in anchor_objs:
                for rel in relations:
                    if rel.relation == spatial_hint:
                        if (rel.subject_id == cand.object_id and rel.object_id == anch.object_id) or \
                           (rel.object_id == cand.object_id and rel.subject_id == anch.object_id):
                            spatial_score = max(spatial_score, 1.0 - min(rel.distance / 5.0, 0.8))

                dist = float(np.linalg.norm(cand.position - anch.position))
                if dist < RELATION_NEAR_THRESHOLD * 2:
                    proximity_score = 1.0 - dist / (RELATION_NEAR_THRESHOLD * 2)
                    spatial_score = max(spatial_score, proximity_score * 0.5)

            final_score = 0.6 * cand.credibility + 0.4 * spatial_score
            scored.append((cand, final_score))

        scored.sort(key=lambda x: x[1], reverse=True)
        return [obj for obj, _ in scored[:top_k]]

    def query_by_affordance(
        self,
        affordance: str,
        top_k: int = 10,
    ) -> list[TrackedObject]:
        """
        按可供性查询 (OpenFunGraph 功能查询)。

        "可以抓的东西" → affordance="graspable"
        "能坐的" → affordance="sittable"
        """
        matches = [
            obj for obj in self._objects.values()
            if affordance in obj.affordances
        ]
        matches.sort(key=lambda o: o.credibility, reverse=True)
        return matches[:top_k]

    def query_by_safety(
        self,
        safety_level: str = "dangerous",
    ) -> list[TrackedObject]:
        """查询特定安全等级的物体。"""
        return [
            obj for obj in self._objects.values()
            if obj.safety_level == safety_level
        ]

    def query_by_floor(
        self,
        floor_level: int,
        label: str | None = None,
        top_k: int = 20,
    ) -> list[TrackedObject]:
        """按楼层查询物体 (SPADE 层次规划)。"""
        matches = [
            obj for obj in self._objects.values()
            if obj.floor_level == floor_level
        ]
        if label:
            label_lower = label.lower()
            matches = [
                obj for obj in matches
                if label_lower in obj.label.lower() or obj.label.lower() in label_lower
            ]
        matches.sort(key=lambda o: o.credibility, reverse=True)
        return matches[:top_k]

    def extract_subgraph_for_task(
        self,
        target: str,
        max_nodes: int = 30,
        clip_encoder=None,
    ) -> dict:
        """
        EmbodiedRAG 风格的任务相关子图提取。

        不传整个场景图给 LLM, 而是只提取与 target 相关的局部子图,
        将 LLM 输入 token 减少 ~10x (EmbodiedRAG 2024 核心贡献)。

        Args:
            target: 目标物体名称或描述
            max_nodes: 最大子图节点数
            clip_encoder: CLIP 编码器

        Returns:
            精简的场景图 dict (直接可作为 LLM prompt)
        """
        target_objs = self.query_by_text(target, top_k=5, clip_encoder=clip_encoder)

        relevant_ids = set()
        for obj in target_objs:
            relevant_ids.add(obj.object_id)

        relations = self.compute_spatial_relations()
        for rel in relations:
            if rel.subject_id in relevant_ids or rel.object_id in relevant_ids:
                relevant_ids.add(rel.subject_id)
                relevant_ids.add(rel.object_id)

        if len(relevant_ids) < max_nodes:
            for obj in target_objs:
                for other in self._objects.values():
                    if other.object_id in relevant_ids:
                        continue
                    dist = float(np.linalg.norm(obj.position[:2] - other.position[:2]))
                    if dist < REGION_CLUSTER_RADIUS:
                        relevant_ids.add(other.object_id)
                    if len(relevant_ids) >= max_nodes:
                        break

        sub_objects = []
        for oid in relevant_ids:
            obj = self._objects.get(oid)
            if obj is None:
                continue
            entry = {
                "id": obj.object_id,
                "label": obj.label,
                "position": {
                    "x": round(float(obj.position[0]), 2),
                    "y": round(float(obj.position[1]), 2),
                    "z": round(float(obj.position[2]), 2),
                },
                "credibility": round(obj.credibility, 2),
                "floor": obj.floor_level,
            }
            if obj.kg_concept_id:
                entry["safety"] = obj.safety_level
                entry["affordances"] = obj.affordances
            sub_objects.append(entry)

        sub_relations = [
            {
                "subject_id": r.subject_id,
                "relation": r.relation,
                "object_id": r.object_id,
                "distance": r.distance,
            }
            for r in relations
            if r.subject_id in relevant_ids and r.object_id in relevant_ids
        ]

        kg_notes = []
        if self._knowledge_graph is not None:
            for obj in target_objs:
                constraint = self._knowledge_graph.check_safety(obj.label, "approach")
                if constraint:
                    kg_notes.append(constraint.message_en)
                locations = self._knowledge_graph.get_typical_locations(obj.label)
                if locations:
                    kg_notes.append(
                        f"{obj.label} typically found in: {', '.join(locations[:3])}"
                    )

        return {
            "target": target,
            "subgraph_nodes": len(sub_objects),
            "objects": sub_objects,
            "relations": sub_relations,
            "kg_notes": kg_notes,
        }

    # ════════════════════════════════════════════════════════════
    #  DovSG 动态场景图更新 (IEEE RA-L 2025)
    # ════════════════════════════════════════════════════════════

    def compute_scene_diff(self, prev_snapshot: dict) -> dict:
        """
        计算场景图差异 (DovSG 局部更新核心)。

        对比当前场景图与上一次快照, 返回变化事件列表。
        用于: 1) 检测人工干预导致的场景变化
              2) 触发局部重规划
              3) 减少 LLM token 消耗 (只传 diff)
        """
        events = []
        prev_objects = {o["id"]: o for o in prev_snapshot.get("objects", [])}
        curr_objects = {oid: obj for oid, obj in self._objects.items()}

        # 新出现的物体
        for oid, obj in curr_objects.items():
            if oid not in prev_objects:
                events.append({
                    "type": "object_added",
                    "object_id": oid,
                    "label": obj.label,
                    "position": obj.position.tolist(),
                    "confidence": round(obj.credibility, 3),
                })

        # 消失的物体
        for pid, pdata in prev_objects.items():
            if pid not in curr_objects:
                events.append({
                    "type": "object_removed",
                    "object_id": pid,
                    "label": pdata.get("label", "unknown"),
                    "last_position": [
                        pdata["position"]["x"],
                        pdata["position"]["y"],
                        pdata["position"]["z"],
                    ] if "position" in pdata else [],
                })

        # 位置显著变化的物体 (可能被人移动)
        MOVE_THRESHOLD = 0.8  # 米
        for oid, obj in curr_objects.items():
            if oid in prev_objects:
                prev_pos = prev_objects[oid].get("position", {})
                px = prev_pos.get("x", 0)
                py = prev_pos.get("y", 0)
                pz = prev_pos.get("z", 0)
                dist = float(np.linalg.norm(
                    obj.position - np.array([px, py, pz])
                ))
                if dist > MOVE_THRESHOLD:
                    events.append({
                        "type": "object_moved",
                        "object_id": oid,
                        "label": obj.label,
                        "prev_position": [px, py, pz],
                        "curr_position": obj.position.tolist(),
                        "displacement": round(dist, 3),
                    })

        # 信念显著变化 (从高可信 → 低可信, 或反之)
        BELIEF_CHANGE_THRESHOLD = 0.3
        for oid, obj in curr_objects.items():
            if oid in prev_objects:
                prev_belief = prev_objects[oid].get("belief", {})
                prev_cred = prev_belief.get("credibility", 0.5)
                if abs(obj.credibility - prev_cred) > BELIEF_CHANGE_THRESHOLD:
                    events.append({
                        "type": "belief_changed",
                        "object_id": oid,
                        "label": obj.label,
                        "prev_credibility": round(prev_cred, 3),
                        "curr_credibility": round(obj.credibility, 3),
                    })

        return {
            "timestamp": time.time(),
            "total_events": len(events),
            "events": events,
            "summary": self._summarize_diff(events),
        }

    @staticmethod
    def _summarize_diff(events: list[dict]) -> str:
        """生成场景变化的自然语言摘要。"""
        if not events:
            return "No changes detected."
        parts = []
        added = [e for e in events if e["type"] == "object_added"]
        removed = [e for e in events if e["type"] == "object_removed"]
        moved = [e for e in events if e["type"] == "object_moved"]
        if added:
            labels = [e["label"] for e in added[:5]]
            parts.append(f"{len(added)} added: {', '.join(labels)}")
        if removed:
            labels = [e["label"] for e in removed[:5]]
            parts.append(f"{len(removed)} removed: {', '.join(labels)}")
        if moved:
            descs = [f"{e['label']}({e['displacement']:.1f}m)" for e in moved[:5]]
            parts.append(f"{len(moved)} moved: {', '.join(descs)}")
        return " | ".join(parts)

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
                self._enrich_from_kg(obj)
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
    #  语义嵌入索引 (EmbodiedRAG + CLIP 加速检索)
    # ════════════════════════════════════════════════════════════

    def build_embedding_index(self) -> bool:
        """
        构建 CLIP 特征索引 (加速 EmbodiedRAG 检索)。

        将所有有 CLIP 特征的物体组织为矩阵,
        支持批量余弦相似度查询。
        """
        objects_with_features = [
            obj for obj in self._objects.values()
            if obj.features.size > 0
        ]
        if not objects_with_features:
            self._embedding_index = None
            self._embedding_ids = []
            return False

        features = np.stack([obj.features for obj in objects_with_features])
        norms = np.linalg.norm(features, axis=1, keepdims=True)
        norms = np.where(norms > 0, norms, 1.0)
        self._embedding_index = features / norms
        self._embedding_ids = [obj.object_id for obj in objects_with_features]
        return True

    def query_by_embedding(
        self,
        query_embedding: np.ndarray,
        top_k: int = 5,
        min_similarity: float = 0.2,
    ) -> list[tuple[TrackedObject, float]]:
        """
        用 CLIP 嵌入向量查询最相似物体 (批量余弦相似度)。

        比 query_by_text 中逐个计算快 10-50x。
        """
        if not hasattr(self, '_embedding_index') or self._embedding_index is None:
            self.build_embedding_index()
        if self._embedding_index is None or len(self._embedding_ids) == 0:
            return []

        q = np.asarray(query_embedding, dtype=np.float64)
        q_norm = np.linalg.norm(q)
        if q_norm > 0:
            q = q / q_norm

        sims = self._embedding_index @ q
        top_indices = np.argsort(sims)[::-1][:top_k]

        results = []
        for idx in top_indices:
            sim = float(sims[idx])
            if sim < min_similarity:
                break
            oid = self._embedding_ids[idx]
            obj = self._objects.get(oid)
            if obj is not None:
                results.append((obj, sim))

        return results

    def get_open_vocabulary_matches(
        self,
        query: str,
        clip_encoder=None,
        top_k: int = 5,
    ) -> list[dict]:
        """
        开放词汇查询: 文本 → CLIP → 场景图物体 (完整流程)。

        融合三个信号:
          1. CLIP 语义相似度
          2. KG 知识匹配
          3. 字符串匹配

        返回带置信度评分的匹配结果。
        """
        results = []

        # Signal 1: CLIP 嵌入匹配
        clip_matches = []
        if clip_encoder is not None:
            try:
                q_embedding = clip_encoder.encode_text(query)
                clip_matches = self.query_by_embedding(
                    q_embedding, top_k=top_k * 2, min_similarity=0.15,
                )
            except Exception as e:
                logger.warning("CLIP query failed: %s", e)

        for obj, sim in clip_matches:
            results.append({
                "object_id": obj.object_id,
                "label": obj.label,
                "position": obj.position.tolist(),
                "clip_similarity": round(sim, 3),
                "kg_match": bool(obj.kg_concept_id),
                "credibility": round(obj.credibility, 3),
                "score": round(0.5 * sim + 0.3 * obj.credibility + 0.2 * (1.0 if obj.kg_concept_id else 0.0), 3),
            })

        # Signal 2: KG 别名匹配
        if self._knowledge_graph is not None:
            concept = self._knowledge_graph.lookup(query)
            if concept is not None:
                all_names = set(n.lower() for n in concept.names_en + concept.names_zh)
                for obj in self._objects.values():
                    if obj.label.lower() in all_names or any(n in obj.label.lower() for n in all_names):
                        existing = next((r for r in results if r["object_id"] == obj.object_id), None)
                        if existing is None:
                            results.append({
                                "object_id": obj.object_id,
                                "label": obj.label,
                                "position": obj.position.tolist(),
                                "clip_similarity": 0.0,
                                "kg_match": True,
                                "credibility": round(obj.credibility, 3),
                                "score": round(0.6 * obj.credibility + 0.4, 3),
                            })
                        elif existing is not None:
                            existing["kg_match"] = True
                            existing["score"] = min(1.0, existing["score"] + 0.2)

        # Signal 3: String fallback
        query_lower = query.lower()
        for obj in self._objects.values():
            if query_lower in obj.label.lower() or obj.label.lower() in query_lower:
                existing = next((r for r in results if r["object_id"] == obj.object_id), None)
                if existing is None:
                    results.append({
                        "object_id": obj.object_id,
                        "label": obj.label,
                        "position": obj.position.tolist(),
                        "clip_similarity": 0.0,
                        "kg_match": bool(obj.kg_concept_id),
                        "credibility": round(obj.credibility, 3),
                        "score": round(0.5 * obj.credibility + 0.3, 3),
                    })

        results.sort(key=lambda r: r["score"], reverse=True)
        return results[:top_k]

    # ════════════════════════════════════════════════════════════
    #  场景图持久化 (长期记忆)
    # ════════════════════════════════════════════════════════════

    def save_to_file(self, path: str) -> None:
        """保存场景图到文件 (长期记忆持久化)。"""
        data = {
            "version": "2.0",
            "objects": {},
            "views": {},
            "room_names": dict(self._room_name_cache),
            "next_id": self._next_id,
            "next_view_id": self._next_view_id,
        }
        for oid, obj in self._objects.items():
            data["objects"][str(oid)] = {
                "label": obj.label,
                "position": obj.position.tolist(),
                "best_score": obj.best_score,
                "detection_count": obj.detection_count,
                "last_seen": obj.last_seen,
                "extent": obj.extent.tolist(),
                "belief_alpha": obj.belief_alpha,
                "belief_beta": obj.belief_beta,
                "position_variance": obj.position_variance,
                "miss_streak": obj.miss_streak,
                "region_id": obj.region_id,
                "kg_concept_id": obj.kg_concept_id,
                "safety_level": obj.safety_level,
                "affordances": obj.affordances,
                "floor_level": obj.floor_level,
                "features": obj.features.tolist() if obj.features is not None and obj.features.size > 0 else [],
                "points": obj.points.tolist() if obj.points is not None and len(obj.points) > 0 else [],
            }
        for vid, view in self._views.items():
            data["views"][str(vid)] = {
                "position": view.position.tolist(),
                "timestamp": view.timestamp,
                "room_id": view.room_id,
                "object_ids": view.object_ids,
                "key_labels": view.key_labels,
            }

        safe_json_dump(data, path)
        logger.info("Scene graph saved to %s (%d objects, %d views)",
                     path, len(self._objects), len(self._views))

    def load_from_file(self, path: str) -> bool:
        """从文件恢复场景图 (长期记忆加载)。"""
        import json
        try:
            with open(path, encoding='utf-8') as f:
                data = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError) as e:
            logger.warning("Failed to load scene graph from %s: %s", path, e)
            return False

        self._objects.clear()
        self._views.clear()

        for oid_str, odata in data.get("objects", {}).items():
            oid = int(oid_str)
            obj = TrackedObject(
                object_id=oid,
                label=odata["label"],
                position=np.array(odata["position"]),
                best_score=odata["best_score"],
                detection_count=odata.get("detection_count", 1),
                last_seen=odata.get("last_seen", 0.0),
                extent=np.array(odata.get("extent", [0.2, 0.2, 0.2])),
                belief_alpha=odata.get("belief_alpha", 1.5),
                belief_beta=odata.get("belief_beta", 1.0),
                position_variance=odata.get("position_variance", 1.0),
                miss_streak=odata.get("miss_streak", 0),
                region_id=odata.get("region_id", -1),
                kg_concept_id=odata.get("kg_concept_id", ""),
                safety_level=odata.get("safety_level", "safe"),
                affordances=odata.get("affordances", []),
                floor_level=odata.get("floor_level", 0),
                features=np.array(odata.get("features", [])),
                points=np.array(odata.get("points", [])).reshape(-1, 3) if odata.get("points") else np.empty((0, 3)),
            )
            obj._update_credibility()
            self._enrich_from_kg(obj)
            obj.source = "loaded"
            self._objects[oid] = obj

        for vid_str, vdata in data.get("views", {}).items():
            vid = int(vid_str)
            self._views[vid] = ViewNode(
                view_id=vid,
                position=np.array(vdata["position"]),
                timestamp=vdata["timestamp"],
                room_id=vdata.get("room_id", -1),
                object_ids=vdata.get("object_ids", []),
                key_labels=vdata.get("key_labels", []),
            )

        self._room_name_cache = data.get("room_names", {})
        # Convert string keys back to int
        self._room_name_cache = {int(k): v for k, v in self._room_name_cache.items()}
        self._next_id = data.get("next_id", max(self._objects.keys(), default=-1) + 1)
        self._next_view_id = data.get("next_view_id", max(self._views.keys(), default=-1) + 1)

        logger.info("Scene graph loaded from %s (%d objects, %d views)",
                     path, len(self._objects), len(self._views))
        return True

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
        added_ids   = list(after_ids - before_ids)
        updated_ids = [oid for oid in before_ids if oid in after_ids and oid in in_radius_ids]

        return {
            "added":   added_ids,
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
            obj for obj in list(self._objects.values())
            if now - obj.last_seen > timeout and obj.credibility < min_confidence
        ]
        for obj in stale:
            self._objects.pop(obj.object_id, None)
        return [str(obj.object_id) for obj in stale]

    def get_scene_graph_diff_json(self, prev_snapshot: dict) -> str:
        """Return a JSON string describing changes since *prev_snapshot*.

        The output has keys: ``added``, ``updated``, ``removed``, ``summary``, ``timestamp``.

        Parameters
        ----------
        prev_snapshot:
            A dict previously returned by ``json.loads(get_scene_graph_json())``.
        """
        import json as _json
        raw = self.compute_scene_diff(prev_snapshot)

        # Reshape 'events' list into keyed buckets that tests expect
        added, updated, removed = [], [], []
        for evt in raw.get("events", []):
            t = evt.get("type", "")
            if t == "object_added":
                pos = evt.get("position", [0, 0, 0])
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    pos_dict = {"x": pos[0], "y": pos[1], "z": pos[2]}
                else:
                    pos_dict = {"x": 0.0, "y": 0.0, "z": 0.0}
                obj = self._objects.get(evt["object_id"])
                added.append({"id": evt["object_id"], "label": evt.get("label", ""),
                               "position": pos_dict,
                               "credibility": obj.credibility if obj else evt.get("confidence", 0.0)})
            elif t == "object_removed":
                removed.append({"id": evt["object_id"], "label": evt.get("label", "")})
            elif t == "object_moved":
                updated.append({"id": evt["object_id"], "label": evt.get("label", ""),
                                 "old_position": evt.get("old_position", []),
                                 "new_position": evt.get("new_position", [])})

        result = {
            "added":     added,
            "updated":   updated,
            "removed":   removed,
            "summary":   raw.get("summary", ""),
            "timestamp": raw.get("timestamp", 0.0),
        }
        return _json.dumps(result, ensure_ascii=False)
