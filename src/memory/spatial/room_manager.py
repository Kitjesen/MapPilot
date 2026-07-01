"""
room_manager.py — Room/Floor/Region/Group 管理 Mixin (从 instance_tracker.py 提取)

包含:
  - compute_regions()       — DBSCAN 空间聚类
  - compute_groups()        — Group 层构建
  - compute_rooms()         — Region → RoomNode (含 LLM 命名)
  - compute_floors()        — 楼层推断
  - assign_rooms_to_floors() — Room→Floor 分配
  - query_rooms_by_embedding() — OneMap CLIP 室查询
  - compute_spatial_relations() — 空间关系计算
  - compute_topology_edges()    — 拓扑连通边
  - _estimate_frontier_directions() — 前沿方向估算
  - _build_hierarchy_edges(), _build_view_edges()
  - _assign_view_rooms(), _build_subgraphs()
  - _trigger_room_llm_naming(), _infer_group_name()
"""

import asyncio
import logging
import math
import time
from typing import TYPE_CHECKING

import numpy as np

from runtime.msgs.scene import (
    FLOOR_HEIGHT,
    FLOOR_MERGE_TOLERANCE,
    GROUP_KEYWORDS,
    REGION_CLUSTER_RADIUS,
    RELATION_NEAR_THRESHOLD,
    RELATION_ON_THRESHOLD,
    ROOM_NAMING_STABILITY_COUNT,
    ROOM_NAMING_STABILITY_SEC,
    FloorNode,
    GroupNode,
    Region,
    RoomNode,
    SpatialRelation,
    TrackedObject,
    ViewNode,
    infer_room_type,
)

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class RoomManagerMixin:
    """Room/Floor/Region/Group 管理 Mixin。

    依赖 self._objects, self._room_llm_namer, self._room_name_cache,
    self._last_rooms, self._llm_pending_tasks, self._region_stability,
    self._cached_regions, self._cached_floors。
    """

    def compute_spatial_relations(self) -> list[SpatialRelation]:
        """
        计算所有物体对之间的空间关系 (ConceptGraphs + SG-Nav)。

        D2 升级: 使用包围盒感知的距离和重叠判断, 而非纯质心距离。

        关系判断策略 (参考 ConceptGraphs ICRA 2024):
          - near:        包围盒间隙 < RELATION_NEAR_THRESHOLD (扣除物体尺寸)
          - on:           水平投影重叠 + z 轴上方紧邻 (A底面 ≈ B顶面)
          - above/below:  z 轴分离, 水平投影有重叠
          - left_of/right_of/in_front_of/behind: 方向超过包围盒尺寸才判定

        相比旧版 (纯距离):
          - "杯子在桌子上" 不再需要两个质心近 0.5m, 而是检查包围盒垂直接触
          - "椅子在桌子旁边" 会扣除桌椅各自的宽度, 避免大物体一直被判 near

        Returns:
            SpatialRelation 列表
        """
        relations: list[SpatialRelation] = []
        objs = list(self._objects.values())
        n = len(objs)

        # 大场景加速: 预计算距离矩阵, 仅处理近邻对
        max_relation_dist = RELATION_NEAR_THRESHOLD * 3
        if n > 50:
            positions = np.array([o.position[:3] for o in objs])
            try:
                from scipy.spatial import cKDTree
                tree = cKDTree(positions)
                candidate_pairs = tree.query_pairs(r=max_relation_dist)
            except ImportError:
                diffs = positions[:, None, :] - positions[None, :, :]
                dists = np.linalg.norm(diffs, axis=2)
                ii, jj = np.where((dists < max_relation_dist) & (dists > 0))
                candidate_pairs = {(min(a, b), max(a, b)) for a, b in zip(ii, jj)}
        else:
            candidate_pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]

        for i, j in candidate_pairs:
            a, b = objs[i], objs[j]

            dx = float(a.position[0] - b.position[0])
            dy = float(a.position[1] - b.position[1])
            dz = float(a.position[2] - b.position[2])
            math.sqrt(dx * dx + dy * dy)
            dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

            # 包围盒感知的间隙距离 (扣除两物体各自半径)
            gap_x = max(0.0, abs(dx) - a.extent[0] - b.extent[0])
            gap_y = max(0.0, abs(dy) - a.extent[1] - b.extent[1])
            gap_z = max(0.0, abs(dz) - a.extent[2] - b.extent[2])
            bbox_gap_3d = math.sqrt(gap_x**2 + gap_y**2 + gap_z**2)
            bbox_gap_2d = math.sqrt(gap_x**2 + gap_y**2)

            # 水平投影是否重叠 (x 和 y 轴包围盒都有交集)
            overlap_x = (abs(dx) < a.extent[0] + b.extent[0])
            overlap_y = (abs(dy) < a.extent[1] + b.extent[1])
            horizontal_overlap = overlap_x and overlap_y

            # near: 包围盒间隙 < 阈值
            if bbox_gap_3d < RELATION_NEAR_THRESHOLD:
                relations.append(SpatialRelation(
                    subject_id=a.object_id,
                    relation="near",
                    object_id=b.object_id,
                    distance=round(dist_3d, 2),
                ))

            # on: A 在 B 上面
            a_bottom = a.position[2] - a.extent[2]
            a_top = a.position[2] + a.extent[2]
            b_bottom = b.position[2] - b.extent[2]
            b_top = b.position[2] + b.extent[2]

            if horizontal_overlap:
                on_tolerance = RELATION_ON_THRESHOLD + 0.1 * (a.extent[2] + b.extent[2])
                if abs(a_bottom - b_top) < on_tolerance and dz > 0:
                    relations.append(SpatialRelation(
                        subject_id=a.object_id,
                        relation="on",
                        object_id=b.object_id,
                        distance=round(abs(a_bottom - b_top), 2),
                    ))
                elif abs(b_bottom - a_top) < on_tolerance and dz < 0:
                    relations.append(SpatialRelation(
                        subject_id=b.object_id,
                        relation="on",
                        object_id=a.object_id,
                        distance=round(abs(b_bottom - a_top), 2),
                    ))

                # above/below (不接触但垂直对齐)
                if gap_z > on_tolerance:
                    if dz > 0:
                        relations.append(SpatialRelation(
                            subject_id=a.object_id,
                            relation="above",
                            object_id=b.object_id,
                            distance=round(gap_z, 2),
                        ))
                    else:
                        relations.append(SpatialRelation(
                            subject_id=a.object_id,
                            relation="below",
                            object_id=b.object_id,
                            distance=round(gap_z, 2),
                        ))

            # 方向关系: 仅在一定距离内, 且方向分量超过包围盒尺寸
            if bbox_gap_2d < RELATION_NEAR_THRESHOLD * 2:
                dir_threshold_y = a.extent[1] + b.extent[1] + 0.3
                dir_threshold_x = a.extent[0] + b.extent[0] + 0.3

                if dy > dir_threshold_y:
                    relations.append(SpatialRelation(
                        subject_id=a.object_id,
                        relation="left_of",
                        object_id=b.object_id,
                    ))
                elif dy < -dir_threshold_y:
                    relations.append(SpatialRelation(
                        subject_id=a.object_id,
                        relation="right_of",
                        object_id=b.object_id,
                    ))

                if dx > dir_threshold_x:
                    relations.append(SpatialRelation(
                        subject_id=a.object_id,
                        relation="in_front_of",
                        object_id=b.object_id,
                    ))
                elif dx < -dir_threshold_x:
                    relations.append(SpatialRelation(
                        subject_id=a.object_id,
                        relation="behind",
                        object_id=b.object_id,
                    ))

        return relations

    def compute_regions(self) -> list[Region]:
        """
        将物体按空间位置聚类为区域 (SG-Nav 的"房间级"层次)。

        使用 DBSCAN 聚类 (参考 SG-Nav NeurIPS 2024):
        - eps = REGION_CLUSTER_RADIUS: 同一区域内物体的最大距离
        - min_samples = 1: 允许单物体区域 (孤立物体仍需归属)
        - 噪声点 (label=-1) 各自成独立区域

        相比旧版距离遍历:
        - 聚类结果不依赖物体遍历顺序
        - 同一簇中任意两点间都可达 → 真正的密度连通
        - 非凸形状的区域也能正确聚合

        Returns:
            Region 列表
        """

        objs = list(self._objects.values())
        if not objs:
            return []

        positions_2d = np.array([obj.position[:2] for obj in objs], dtype=np.float64)

        try:
            from sklearn.cluster import DBSCAN
            clustering = DBSCAN(
                eps=REGION_CLUSTER_RADIUS,
                min_samples=1,
                metric="euclidean",
            ).fit(positions_2d)
            cluster_labels = clustering.labels_
        except ImportError:
            # sklearn 不可用时回退到 scipy
            try:
                from scipy.cluster.hierarchy import fcluster, linkage
                if len(positions_2d) >= 2:
                    Z = linkage(positions_2d, method="single", metric="euclidean")
                    cluster_labels = fcluster(Z, t=REGION_CLUSTER_RADIUS, criterion="distance") - 1
                else:
                    cluster_labels = np.array([0])
            except ImportError:
                # 最终回退: 每个物体独立区域
                cluster_labels = np.arange(len(objs))

        # 按簇ID分组
        cluster_to_objs: dict[int, list[int]] = {}
        for idx, cl in enumerate(cluster_labels):
            cl_int = int(cl)
            if cl_int == -1:
                # DBSCAN 噪声点, 分配唯一簇ID
                noise_id = max(cluster_to_objs.keys(), default=-1) + 1
                cluster_to_objs[noise_id] = [idx]
            else:
                cluster_to_objs.setdefault(cl_int, []).append(idx)

        regions: list[Region] = []
        for region_id, obj_indices in enumerate(cluster_to_objs.values()):
            obj_ids = [objs[i].object_id for i in obj_indices]
            center = positions_2d[obj_indices].mean(axis=0)

            for i in obj_indices:
                objs[i].region_id = region_id

            region = Region(
                region_id=region_id,
                center=center,
                object_ids=obj_ids,
                name=f"region_{region_id}",
            )
            regions.append(region)

        # 语义命名 (SG-Nav) — 创新1: 使用增强规则命名替代 area_with_ 拼接
        for region in regions:
            labels = [
                self._objects[oid].label
                for oid in region.object_ids
                if oid in self._objects
            ]
            if labels:
                # 已有 LLM 缓存则跳过
                if region.region_id in self._room_name_cache:
                    region.name = self._room_name_cache[region.region_id]
                    region.llm_named = True
                else:
                    region.name = infer_room_type(labels)

        # BA-HSG: 缓存 regions 供信念传播使用
        self._cached_regions = regions

        return regions

    def compute_groups(self, regions: list[Region]) -> list[GroupNode]:
        """
        根据 room/region 内物体语义构建 group 层。

        SG-Nav 的 group 层用于把同一区域中的同类物体聚为中间节点，
        降低 LLM 直接处理 object 级图的复杂度。
        """
        groups: list[GroupNode] = []
        group_index: dict[tuple[int, str], int] = {}

        for room in regions:
            for oid in room.object_ids:
                obj = self._objects.get(oid)
                if obj is None:
                    continue

                group_name = self._infer_group_name(obj.label)
                key = (room.region_id, group_name)

                if key not in group_index:
                    gid = len(groups)
                    groups.append(
                        GroupNode(
                            group_id=gid,
                            room_id=room.region_id,
                            name=group_name,
                            center=obj.position[:2].copy(),
                            object_ids=[oid],
                            semantic_labels=[obj.label],
                        )
                    )
                    group_index[key] = gid
                else:
                    gid = group_index[key]
                    g = groups[gid]
                    g.object_ids.append(oid)
                    n = len(g.object_ids)
                    g.center = (g.center * (n - 1) + obj.position[:2]) / n
                    if obj.label not in g.semantic_labels:
                        g.semantic_labels.append(obj.label)

        return groups

    def compute_rooms(self, regions: list[Region], groups: list[GroupNode]) -> list[RoomNode]:
        """将 region 标准化为 room 节点。

        创新1 补强: 当 Region 满足稳定条件时 (物体数 >= ROOM_NAMING_STABILITY_COUNT
        且持续 ROOM_NAMING_STABILITY_SEC 无变化), 异步调用 LLM 为 Room 赋予
        可读的语义名称 (e.g. '走廊', 'office'), 替换掉 'area_with_door_chair' 式的
        拼接命名。LLM 命名结果缓存在 _room_name_cache 中, 避免重复调用。
        """
        room_to_groups: dict[int, list[int]] = {}
        for g in groups:
            room_to_groups.setdefault(g.room_id, []).append(g.group_id)

        now = time.time()
        rooms: list[RoomNode] = []
        for r in regions:
            labels = [
                self._objects[oid].label
                for oid in r.object_ids
                if oid in self._objects
            ]

            # 决定 room name: 优先 LLM 缓存, 其次 region 原名, 最后 fallback
            room_name = r.name if r.name else f"room_{r.region_id}"
            is_llm_named = r.llm_named

            # 检查 LLM 缓存
            if r.region_id in self._room_name_cache:
                room_name = self._room_name_cache[r.region_id]
                is_llm_named = True
            elif self._room_llm_namer and not is_llm_named:
                # 检查稳定性: 物体集合是否已稳定
                current_set = frozenset(r.object_ids)
                prev = self._region_stability.get(r.region_id)
                if prev is not None and prev[0] == current_set:
                    stable_duration = now - prev[1]
                else:
                    self._region_stability[r.region_id] = (current_set, now)
                    stable_duration = 0.0

                if (len(r.object_ids) >= ROOM_NAMING_STABILITY_COUNT
                        and stable_duration >= ROOM_NAMING_STABILITY_SEC):
                    # 触发异步 LLM 命名 (fire-and-forget, 结果下次 compute 时可用)
                    self._trigger_room_llm_naming(r.region_id, labels)

            # OneMap: EMA 聚合 room 内物体 CLIP 特征
            room_clip = None
            feat_count = 0
            alpha = 0.3  # EMA 平滑系数
            for oid in r.object_ids:
                obj = self._objects.get(oid)
                if obj is None or obj.features.size == 0:
                    continue
                f = np.asarray(obj.features, dtype=np.float64)
                f_norm = np.linalg.norm(f)
                if f_norm == 0:
                    continue
                f = f / f_norm
                if room_clip is None:
                    room_clip = f.copy()
                else:
                    room_clip = alpha * f + (1.0 - alpha) * room_clip
                feat_count += 1
            # L2 归一化最终聚合特征
            if room_clip is not None:
                norm = np.linalg.norm(room_clip)
                if norm > 0:
                    room_clip = room_clip / norm

            rooms.append(
                RoomNode(
                    room_id=r.region_id,
                    name=room_name,
                    center=r.center.copy(),
                    object_ids=list(r.object_ids),
                    group_ids=room_to_groups.get(r.region_id, []),
                    semantic_labels=labels,
                    llm_named=is_llm_named,
                    clip_feature=room_clip,
                    feature_count=feat_count,
                )
            )
        self._last_rooms = rooms
        return rooms

    def query_rooms_by_embedding(
        self, embedding: np.ndarray, top_k: int = 3
    ) -> list[tuple[RoomNode, float]]:
        """OneMap: 用 CLIP embedding 查询最相似的 room。

        Args:
            embedding: 查询向量 (e.g. 512-dim CLIP feature)
            top_k: 返回 top-k 个匹配结果

        Returns:
            [(RoomNode, cosine_score), ...] 按相似度降序
        """
        with self._lock:
            rooms = list(self._last_rooms)
        if not rooms:
            return []

        q = np.asarray(embedding, dtype=np.float64).ravel()
        q_norm = np.linalg.norm(q)
        if q_norm == 0:
            return []
        q = q / q_norm

        scored: list[tuple[RoomNode, float]] = []
        for room in rooms:
            if room.clip_feature is None:
                continue
            score = float(np.dot(q, room.clip_feature))
            scored.append((room, score))

        scored.sort(key=lambda x: x[1], reverse=True)
        return scored[:top_k]

    _LLM_NAMING_MAX_CONCURRENT = 3
    _LLM_NAMING_TIMEOUT_S = 15.0

    def _trigger_room_llm_naming(self, region_id: int, labels: list[str]) -> None:
        """异步调用 LLM 为 Room 命名 (限并发 + 超时 + task 追踪)。"""
        if region_id in self._room_name_cache:
            return
        if len(self._llm_pending_tasks) >= self._LLM_NAMING_MAX_CONCURRENT:
            return
        self._room_name_cache[region_id] = f"naming_room_{region_id}"

        async def _do_name():
            try:
                name = await asyncio.wait_for(
                    self._room_llm_namer(labels),
                    timeout=self._LLM_NAMING_TIMEOUT_S,
                )
                if name and isinstance(name, str) and len(name.strip()) > 0:
                    self._room_name_cache[region_id] = name.strip()
                    logger.info("LLM named room %d -> '%s' (labels: %s)",
                                region_id, name.strip(), labels[:6])
                else:
                    del self._room_name_cache[region_id]
            except asyncio.TimeoutError:
                logger.warning("LLM room naming timed out for region %d", region_id)
                if region_id in self._room_name_cache:
                    del self._room_name_cache[region_id]
            except Exception as e:
                logger.warning("LLM room naming failed for region %d: %s", region_id, e)
                if region_id in self._room_name_cache:
                    del self._room_name_cache[region_id]
            finally:
                self._llm_pending_tasks.pop(region_id, None)

        try:
            loop = asyncio.get_running_loop()
            task = loop.create_task(_do_name())
            self._llm_pending_tasks[region_id] = task
        except RuntimeError:
            if region_id in self._room_name_cache:
                del self._room_name_cache[region_id]

    @staticmethod
    def _infer_group_name(label: str) -> str:
        lower = label.lower()
        for group_name, words in GROUP_KEYWORDS.items():
            if any(w in lower for w in words):
                return group_name
        return "others"

    # ── 拓扑层: 房间间连通关系 (创新4: Hydra/Concept-Guided 风格) ──

    DOOR_KEYWORDS = {"door", "gate", "entrance", "exit", "opening",
                     "门", "出口", "入口", "通道"}
    PASSAGE_KEYWORDS = {"corridor", "hallway", "passage", "通道", "走廊", "过道"}

    def compute_topology_edges(
        self,
        rooms: list[RoomNode],
    ) -> list[dict]:
        """
        计算房间间拓扑连通边 (Hydra + Concept-Guided Exploration 融合)。

        三种检测策略:
        1. Door-mediated: "door"物体处于两个房间边界 → 连通边
        2. Proximity: 两个房间中心距 < 阈值且有物体靠近 → 隐含连通
        3. Passage-mediated: "corridor"类房间自动与相邻房间连通
        """
        if len(rooms) < 2:
            return []

        edges: list[dict] = []
        seen_pairs: set = set()

        {r.room_id: r for r in rooms}

        # Strategy 1: Door-mediated connectivity
        door_objects = [
            obj for obj in self._objects.values()
            if any(kw in obj.label.lower() for kw in self.DOOR_KEYWORDS)
        ]

        for door in door_objects:
            door_pos = door.position[:2]
            distances = []
            for room in rooms:
                dist = float(np.linalg.norm(door_pos - room.center))
                distances.append((room.room_id, dist))
            distances.sort(key=lambda x: x[1])

            if len(distances) >= 2:
                r1, d1 = distances[0]
                r2, d2 = distances[1]
                if r1 != r2 and d2 < REGION_CLUSTER_RADIUS * 2.0:
                    pair = (min(r1, r2), max(r1, r2))
                    if pair not in seen_pairs:
                        seen_pairs.add(pair)
                        edges.append({
                            "from_room": pair[0],
                            "to_room": pair[1],
                            "type": "door",
                            "mediator": door.label,
                            "mediator_pos": {
                                "x": round(float(door.position[0]), 2),
                                "y": round(float(door.position[1]), 2),
                            },
                            "distance": round(d1 + d2, 2),
                        })

        # Strategy 2: Proximity connectivity
        PROXIMITY_THRESHOLD = REGION_CLUSTER_RADIUS * 1.8
        for i, r1 in enumerate(rooms):
            for r2 in rooms[i + 1:]:
                pair = (min(r1.room_id, r2.room_id), max(r1.room_id, r2.room_id))
                if pair in seen_pairs:
                    continue
                dist = float(np.linalg.norm(r1.center - r2.center))
                if dist < PROXIMITY_THRESHOLD:
                    # Check if there are objects near the boundary
                    boundary_objs = 0
                    midpoint = (r1.center + r2.center) / 2.0
                    for oid in list(r1.object_ids) + list(r2.object_ids):
                        obj = self._objects.get(oid)
                        if obj is None:
                            continue
                        d_to_mid = float(np.linalg.norm(obj.position[:2] - midpoint))
                        if d_to_mid < dist * 0.6:
                            boundary_objs += 1
                    if boundary_objs > 0:
                        seen_pairs.add(pair)
                        edges.append({
                            "from_room": pair[0],
                            "to_room": pair[1],
                            "type": "proximity",
                            "distance": round(dist, 2),
                        })

        # Strategy 3: Passage-mediated (corridor rooms connect to all neighbors)
        for room in rooms:
            is_passage = any(
                kw in room.name.lower()
                for kw in self.PASSAGE_KEYWORDS
            )
            if not is_passage:
                continue
            for other in rooms:
                if other.room_id == room.room_id:
                    continue
                pair = (min(room.room_id, other.room_id),
                        max(room.room_id, other.room_id))
                if pair in seen_pairs:
                    continue
                dist = float(np.linalg.norm(room.center - other.center))
                if dist < REGION_CLUSTER_RADIUS * 2.5:
                    seen_pairs.add(pair)
                    edges.append({
                        "from_room": pair[0],
                        "to_room": pair[1],
                        "type": "passage",
                        "mediator": room.name,
                        "distance": round(dist, 2),
                    })

        return edges

    def _estimate_frontier_directions(self, rooms: list[RoomNode]) -> list[dict]:
        """
        估算前沿方向 (创新5: 拓扑语义图前沿节点)。

        策略:
        1. 对每个房间, 检查物体分布的"稀疏方向" — 物体只在一侧说明另一侧未探索
        2. 检测房间间的"开放边界" — 两个不相邻房间之间的空白可能是前沿
        3. 出口介质 (door, corridor) 指向但无对应房间 → 前沿

        参考: TACS-Graphs (2025) 可通行性感知 + Concept-Guided Exploration (2025) 门概念
        """
        if not rooms or not self._objects:
            return []

        frontiers: list[dict] = []
        all_positions = np.array([obj.position[:2] for obj in self._objects.values()])
        all_positions.mean(axis=0)

        door_kw = {"door", "gate", "entrance", "exit", "门", "出口", "入口"}
        door_objects = [
            obj for obj in self._objects.values()
            if any(kw in obj.label.lower() for kw in door_kw)
        ]

        room_centers = {r.room_id: r.center for r in rooms}
        {r.room_id: set(r.object_ids) for r in rooms}

        for door in door_objects:
            door_pos = door.position[:2]
            near_rooms = []
            for room in rooms:
                dist = float(np.linalg.norm(door_pos - room.center))
                near_rooms.append((room.room_id, dist))
            near_rooms.sort(key=lambda x: x[1])

            if len(near_rooms) >= 1:
                closest_rid, closest_dist = near_rooms[0]
                has_second = (
                    len(near_rooms) >= 2
                    and near_rooms[1][1] < REGION_CLUSTER_RADIUS * 2.0
                )
                if not has_second and closest_dist < REGION_CLUSTER_RADIUS * 1.5:
                    direction = door_pos - room_centers[closest_rid]
                    dnorm = float(np.linalg.norm(direction))
                    if dnorm > 0.5:
                        direction = direction / dnorm
                        frontier_pos = door_pos + direction * 2.0
                        frontiers.append({
                            "position": {
                                "x": round(float(frontier_pos[0]), 2),
                                "y": round(float(frontier_pos[1]), 2),
                            },
                            "direction": {
                                "dx": round(float(direction[0]), 2),
                                "dy": round(float(direction[1]), 2),
                            },
                            "nearest_room_id": closest_rid,
                            "mediator": door.label,
                            "frontier_size": 2.0,
                            "source": "door_outward",
                        })

        for room in rooms:
            if len(room.object_ids) < 3:
                continue
            obj_positions = []
            for oid in room.object_ids:
                obj = self._objects.get(oid)
                if obj is not None:
                    obj_positions.append(obj.position[:2])
            if len(obj_positions) < 3:
                continue

            positions = np.array(obj_positions)
            centroid = positions.mean(axis=0)

            n_sectors = 4
            sector_counts = [0] * n_sectors
            for p in positions:
                angle = math.atan2(p[1] - centroid[1], p[0] - centroid[0])
                sector = int((angle + math.pi) / (2 * math.pi) * n_sectors) % n_sectors
                sector_counts[sector] += 1

            total = sum(sector_counts)
            for s_idx, cnt in enumerate(sector_counts):
                if cnt == 0 and total >= 3:
                    angle = (s_idx + 0.5) * (2 * math.pi / n_sectors) - math.pi
                    direction = np.array([math.cos(angle), math.sin(angle)])
                    frontier_pos = centroid + direction * REGION_CLUSTER_RADIUS

                    too_close = any(
                        float(np.linalg.norm(frontier_pos - r.center)) < REGION_CLUSTER_RADIUS * 0.8
                        for r in rooms if r.room_id != room.room_id
                    )
                    if too_close:
                        continue

                    frontiers.append({
                        "position": {
                            "x": round(float(frontier_pos[0]), 2),
                            "y": round(float(frontier_pos[1]), 2),
                        },
                        "direction": {
                            "dx": round(float(direction[0]), 2),
                            "dy": round(float(direction[1]), 2),
                        },
                        "nearest_room_id": room.room_id,
                        "mediator": "",
                        "frontier_size": 1.5,
                        "source": "sparse_sector",
                    })

        return frontiers[:10]

    @staticmethod
    def _build_hierarchy_edges(rooms: list[RoomNode], groups: list[GroupNode]) -> list[dict]:
        """构建 room→group→object 的层次边。"""
        edges: list[dict] = []

        for room in rooms:
            for gid in room.group_ids:
                edges.append({
                    "parent_type": "room",
                    "parent_id": room.room_id,
                    "child_type": "group",
                    "child_id": gid,
                    "relation": "contains",
                })

        for g in groups:
            for oid in g.object_ids:
                edges.append({
                    "parent_type": "group",
                    "parent_id": g.group_id,
                    "child_type": "object",
                    "child_id": oid,
                    "relation": "contains",
                })

        return edges

    @staticmethod
    def _build_view_edges(views: list[ViewNode]) -> list[dict]:
        """构建 room→view→object 边。"""
        edges: list[dict] = []
        for v in views:
            if v.room_id >= 0:
                edges.append({
                    "parent_type": "room",
                    "parent_id": v.room_id,
                    "child_type": "view",
                    "child_id": v.view_id,
                    "relation": "contains_view",
                })
            for oid in v.object_ids:
                edges.append({
                    "parent_type": "view",
                    "parent_id": v.view_id,
                    "child_type": "object",
                    "child_id": oid,
                    "relation": "observes",
                })
        return edges

    @staticmethod
    def _assign_view_rooms(views: list[ViewNode], rooms: list[RoomNode]) -> list[ViewNode]:
        """为 view 节点分配 room_id（按最近 room center）。"""
        if not views:
            return []
        if not rooms:
            return views

        room_centers = [r.center for r in rooms]
        room_ids = [r.room_id for r in rooms]
        for v in views:
            if v.room_id >= 0:
                continue
            pos2d = v.position[:2]
            dists = [float(np.linalg.norm(pos2d - c)) for c in room_centers]
            best_idx = int(np.argmin(dists))
            v.room_id = room_ids[best_idx]
        return views

    @staticmethod
    def _build_subgraphs(
        rooms: list[RoomNode],
        groups: list[GroupNode],
        views: list[ViewNode],
        objects_by_id: dict[int, TrackedObject],
        relations_list: list[dict],
    ) -> list[dict]:
        """构建 SG-Nav 风格子图摘要，供 planner 侧按子图推理。"""
        subgraphs: list[dict] = []

        # room 级子图
        for room in rooms:
            labels = [
                objects_by_id[oid].label
                for oid in room.object_ids
                if oid in objects_by_id
            ]
            object_set = set(room.object_ids)
            relation_count = 0
            for rel in relations_list:
                sid = rel.get("subject_id")
                oid = rel.get("object_id")
                if sid in object_set and oid in object_set:
                    relation_count += 1

            subgraphs.append({
                "subgraph_id": f"room_{room.room_id}",
                "level": "room",
                "room_id": room.room_id,
                "center": {
                    "x": round(float(room.center[0]), 2),
                    "y": round(float(room.center[1]), 2),
                },
                "object_ids": room.object_ids,
                "object_labels": labels[:12],
                "relation_count": relation_count,
            })

        # group 级子图
        for g in groups:
            labels = [
                objects_by_id[oid].label
                for oid in g.object_ids
                if oid in objects_by_id
            ]
            subgraphs.append({
                "subgraph_id": f"group_{g.group_id}",
                "level": "group",
                "room_id": g.room_id,
                "group_id": g.group_id,
                "name": g.name,
                "center": {
                    "x": round(float(g.center[0]), 2),
                    "y": round(float(g.center[1]), 2),
                },
                "object_ids": g.object_ids,
                "object_labels": labels[:12],
                "relation_count": 0,
            })

        # view 级子图
        for v in views:
            subgraphs.append({
                "subgraph_id": f"view_{v.view_id}",
                "level": "view",
                "room_id": v.room_id,
                "center": {
                    "x": round(float(v.position[0]), 2),
                    "y": round(float(v.position[1]), 2),
                },
                "object_ids": v.object_ids,
                "object_labels": v.key_labels[:12],
                "relation_count": 0,
            })

        return subgraphs

    def compute_floors(self) -> list[FloorNode]:
        """
        通过物体 z 坐标聚类推断楼层 (HOV-SG 层次场景图的 Floor 层)。

        策略:
        - 按 z 坐标排序, 以 FLOOR_HEIGHT 为间距聚类
        - 第一个聚类为 level=0 (地面层), 向上递增
        - 每个 FloorNode 记录 z 范围和包含的 room_ids
        """
        objs = list(self._objects.values())
        if not objs:
            self._cached_floors = []
            return []

        z_values = np.array([float(obj.position[2]) for obj in objs])
        z_sorted_indices = np.argsort(z_values)
        sorted_list = z_sorted_indices.tolist()

        floors: list[FloorNode] = []
        current_floor_objs: list[int] = []
        current_z_min = z_values[sorted_list[0]]
        current_z_sum = 0.0

        for rank, idx in enumerate(sorted_list):
            z = z_values[idx]
            obj = objs[idx]

            if z - current_z_min > FLOOR_HEIGHT - FLOOR_MERGE_TOLERANCE and current_floor_objs:
                center_z = current_z_sum / len(current_floor_objs)
                # rank >= 1 is guaranteed here since current_floor_objs is non-empty
                prev_idx = sorted_list[rank - 1]
                z_max = z_values[prev_idx]
                floors.append(FloorNode(
                    floor_id=len(floors),
                    floor_level=len(floors),
                    z_range=(current_z_min, z_max),
                    object_ids=list(current_floor_objs),
                    center_z=center_z,
                ))
                current_floor_objs = []
                current_z_min = z
                current_z_sum = 0.0

            current_floor_objs.append(obj.object_id)
            current_z_sum += z
            obj.floor_level = len(floors)

        if current_floor_objs:
            center_z = current_z_sum / len(current_floor_objs)
            floors.append(FloorNode(
                floor_id=len(floors),
                floor_level=len(floors),
                z_range=(current_z_min, z_values[z_sorted_indices[-1]]),
                object_ids=list(current_floor_objs),
                center_z=center_z,
            ))

        self._cached_floors = floors
        return floors

    def assign_rooms_to_floors(
        self, floors: list[FloorNode], rooms: list[RoomNode]
    ) -> None:
        """将 rooms 分配到 floors (通过其物体的 floor_level 投票)。"""
        for floor in floors:
            floor.room_ids.clear()
        floor_obj_sets = {f.floor_id: set(f.object_ids) for f in floors}

        for room in rooms:
            room_obj_set = set(room.object_ids)
            best_floor = -1
            best_overlap = 0
            for f in floors:
                overlap = len(room_obj_set & floor_obj_sets[f.floor_id])
                if overlap > best_overlap:
                    best_overlap = overlap
                    best_floor = f.floor_id
            if best_floor >= 0:
                floors[best_floor].room_ids.append(room.room_id)
