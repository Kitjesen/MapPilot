"""
Frontier 评分探索 — 统一目标 Grounding 与 Frontier 选择。

参考论文:
  - MTU3D (ICCV 2025): "Move to Understand"
    核心: 把 frontier 也视为一种"查询", 与目标 grounding 统一优化
    → 比 VLFM 提升 14-23%

  - OpenFrontier (2025): FrontierNet + VLM set-of-mark
  - VLFM (2023): Visual-Language Frontier Map (基线方法)
  - L3MVN (ICRA 2024): 拓扑节点 + frontier 结合

  创新4 扩展 (Topology-Aware Semantic Exploration):
  - Hydra (RSS 2022): 层次3D场景图, 房间拓扑连通
  - Concept-Guided Exploration (2025): Room + Door 自治概念
  新增: frontier 评分 += f(semantic_prior, topology_reachability)
        semantic_prior = "该方向房间包含目标的先验概率"
        topology = "通过拓扑图可达的未探索房间"

  USS-Nav 扩展 (arXiv 2602.00708):
  - TSP 优化 frontier 访问顺序
  - 信息增益 IG = frontier 周围未知格子数量
  - 代价矩阵: cost(i,j) = 1/IG_j + path_dist(i→j)
  - 贪心 nearest-neighbor TSP → 取序列第一个 frontier
"""

import math
import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import numpy as np

# 双语扩展: 用于 frontier 语言评分中的跨语言关键词匹配
try:
    from .chinese_tokenizer import expand_bilingual as _expand_bilingual, _ZH_TO_EN
    _ZH_TO_EN_KEYS = frozenset(_ZH_TO_EN.keys())
except ImportError:
    _expand_bilingual = None
    _ZH_TO_EN_KEYS = frozenset()

logger = logging.getLogger(__name__)

# ── OccupancyGrid 常量 ──
FREE_CELL = 0
OCCUPIED_CELL = 100
UNKNOWN_CELL = -1


@dataclass
class Frontier:
    """一个 frontier 区域。"""
    frontier_id: int
    cells: List[Tuple[int, int]] = field(default_factory=list)  # (row, col) 列表
    center: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    center_world: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    size: int = 0                    # 包含的 cell 数量
    score: float = 0.0              # 语言相关性评分
    distance: float = 0.0           # 到机器人的距离
    direction_label: str = ""       # "north" / "east" / etc.
    nearby_labels: List[str] = field(default_factory=list)  # 附近已知物体
    description: str = ""             # 自然语言描述 (L3MVN/OmniNav 风格)

    def to_dict(self) -> Dict:
        return {
            "frontier_id": self.frontier_id,
            "center_world": {
                "x": round(float(self.center_world[0]), 2),
                "y": round(float(self.center_world[1]), 2),
            },
            "size": self.size,
            "score": round(self.score, 3),
            "distance": round(self.distance, 2),
            "direction": self.direction_label,
            "nearby_objects": self.nearby_labels,
        }


class FrontierScorer:
    """
    Frontier 提取 + 评分。

    用法:
      1. update_costmap(grid, info) — 更新 costmap
      2. extract_frontiers(robot_pos) — 提取 frontier
      3. score_frontiers(instruction, ...) — 评分
      4. get_best_frontier() — 获取最佳探索目标
    
    创新3 补强: 增加 CLIP 视觉评分通道 (VLFM 核心思路),
    对每个 frontier 方向的最近观测帧做 CLIP(image, instruction) 相似度计算。
    """

    def __init__(
        self,
        min_frontier_size: int = 5,      # 最小 frontier 大小 (cells)
        max_frontiers: int = 10,          # 最多保留的 frontier 数
        cluster_radius_cells: int = 3,    # 聚类半径
        distance_weight: float = 0.2,     # 距离权重 (越近越好)
        novelty_weight: float = 0.3,      # 新颖度权重 (未去过更好)
        language_weight: float = 0.2,     # 语言相关性权重
        grounding_weight: float = 0.3,    # MTU3D: grounding 势权重
        # 创新3: 视觉评分权重 (VLFM 核心)
        vision_weight: float = 0.0,       # 默认 0 = 不启用; 启用时建议 0.15
        # 创新4: 语义先验权重 (Topology-Aware Semantic Exploration)
        semantic_prior_weight: float = 0.0,  # 默认 0 = 不启用; 启用时建议 0.2
        # USS-Nav TSP: 贪心 nearest-neighbor 重排序
        tsp_reorder: bool = True,         # 是否启用 TSP 后处理重排序
        tsp_frontier_limit: int = 20,     # frontier 数量超过此值时跳过 TSP
        tsp_ig_radius_cells: int = 10,    # 信息增益估算半径 (cells)
        # C6: 以下阈值全部参数化 (ablation 实验依赖)
        novelty_distance: float = 5.0,    # 新颖度归一化距离 (m)
        nearby_object_radius: float = 3.0,  # 附近物体检索半径 (m)
        grounding_angle_threshold: float = 0.7854,  # 空间梯度角度阈值 (rad, 默认 pi/4)
        cooccurrence_bonus: float = 0.25,  # 共现先验加分
        grounding_spatial_bonus: float = 0.1,  # 方向有物体的 grounding 加分
        grounding_keyword_bonus: float = 0.4,  # 方向有指令相关物体的加分
        grounding_relation_bonus: float = 0.15,  # 关系链延伸加分
    ):
        self.min_frontier_size = min_frontier_size
        self.max_frontiers = max_frontiers
        self.cluster_radius_cells = cluster_radius_cells
        self.distance_weight = distance_weight
        self.novelty_weight = novelty_weight
        self.language_weight = language_weight
        self.grounding_weight = grounding_weight
        self.novelty_distance = max(novelty_distance, 0.1)
        self.nearby_object_radius = max(nearby_object_radius, 0.1)
        self.grounding_angle_threshold = max(grounding_angle_threshold, 0.01)
        self.cooccurrence_bonus = cooccurrence_bonus
        self.grounding_spatial_bonus = grounding_spatial_bonus
        self.grounding_keyword_bonus = grounding_keyword_bonus
        self.grounding_relation_bonus = grounding_relation_bonus

        self.vision_weight = max(vision_weight, 0.0)
        self.semantic_prior_weight = max(semantic_prior_weight, 0.0)

        self.tsp_reorder = tsp_reorder
        self.tsp_frontier_limit = max(tsp_frontier_limit, 2)
        self.tsp_ig_radius_cells = max(tsp_ig_radius_cells, 1)

        self._grid: Optional[np.ndarray] = None
        self._resolution: float = 0.05  # m/cell
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._frontiers: List[Frontier] = []

        # P0: Frontier 失败记忆 (避免重复探索失败位置)
        # 参考 SEEK (RSS 2024): 探索失败 → 降低该方向优先级
        self._failed_positions: List[np.ndarray] = []
        self._failure_penalty_radius: float = 3.0  # 惩罚半径 (m)
        self._failure_penalty_decay: float = 0.7   # 惩罚衰减系数

        # 创新3: 方向观测缓存
        self._directional_features: Dict[int, np.ndarray] = {}
        self._clip_encoder = None

        # 创新4: 语义先验引擎 (拓扑感知探索)
        self._semantic_prior_engine = None
        self._room_priors_cache: Dict[int, float] = {}  # room_id → prior_score

        # P2: 房间-物体知识图谱 (SEEK-style P(target|room))
        self._room_object_kg = None

        # L3MVN: frontier 描述缓存 (避免每轮重复调用 predict_room_type_from_labels)
        # key = 排序后标签逗号拼接, value = 已生成描述; 上限 256 条
        self._frontier_desc_cache: Dict[str, str] = {}

    def update_costmap(
        self,
        grid_data: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
    ):
        """
        更新 costmap 数据。

        Args:
            grid_data: 2D array, 值为 0 (free) / 100 (occupied) / -1 (unknown)
            resolution: m/cell
            origin_x, origin_y: 地图原点在世界坐标的位置
        """
        self._grid = grid_data
        self._resolution = resolution
        self._origin_x = origin_x
        self._origin_y = origin_y

    def extract_frontiers(
        self,
        robot_position: np.ndarray,
    ) -> List[Frontier]:
        """
        从 costmap 提取 frontier cells 并聚类。

        Frontier 定义: 与 unknown cell 相邻的 free cell。

        Args:
            robot_position: [x, y] 世界坐标

        Returns:
            Frontier 列表 (按大小降序)
        """
        if self._grid is None:
            return []

        rows, cols = self._grid.shape
        frontier_mask = np.zeros((rows, cols), dtype=bool)

        # 找 frontier cells
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if self._grid[r, c] != FREE_CELL:
                    continue
                # 检查 4-邻域是否有 unknown
                for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    if self._grid[r + dr, c + dc] == UNKNOWN_CELL:
                        frontier_mask[r, c] = True
                        break

        # 聚类 frontier cells (简单连通分量)
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters: List[List[Tuple[int, int]]] = []

        for r in range(rows):
            for c in range(cols):
                if frontier_mask[r, c] and not visited[r, c]:
                    # BFS 聚类
                    cluster = []
                    queue = deque([(r, c)])
                    visited[r, c] = True

                    while queue:
                        cr, cc = queue.popleft()
                        cluster.append((cr, cc))

                        for dr in range(-self.cluster_radius_cells, self.cluster_radius_cells + 1):
                            for dc in range(-self.cluster_radius_cells, self.cluster_radius_cells + 1):
                                nr, nc = cr + dr, cc + dc
                                if (0 <= nr < rows and 0 <= nc < cols
                                        and frontier_mask[nr, nc]
                                        and not visited[nr, nc]):
                                    visited[nr, nc] = True
                                    queue.append((nr, nc))

                    if len(cluster) >= self.min_frontier_size:
                        clusters.append(cluster)

        # 转换为 Frontier 对象
        self._frontiers = []
        for i, cluster in enumerate(clusters):
            cells = np.array(cluster, dtype=np.float64)
            center_cell = cells.mean(axis=0)

            # Cell → world
            center_world = np.array([
                self._origin_x + center_cell[1] * self._resolution,
                self._origin_y + center_cell[0] * self._resolution,
            ])

            dist = float(np.linalg.norm(center_world - robot_position[:2]))

            # 方向标签
            dx = center_world[0] - robot_position[0]
            dy = center_world[1] - robot_position[1]
            angle = math.atan2(dy, dx)
            direction = self._angle_to_label(angle)

            frontier = Frontier(
                frontier_id=i,
                cells=cluster,
                center=center_cell,
                center_world=center_world,
                size=len(cluster),
                distance=dist,
                direction_label=direction,
            )
            self._frontiers.append(frontier)

        # 按大小降序
        self._frontiers.sort(key=lambda f: f.size, reverse=True)
        self._frontiers = self._frontiers[:self.max_frontiers]

        return self._frontiers

    def set_clip_encoder(self, clip_encoder) -> None:
        """注入 CLIP 编码器, 启用 frontier 视觉评分 (创新3: VLFM 核心)。"""
        self._clip_encoder = clip_encoder
        if self.vision_weight <= 0.0:
            self.vision_weight = 0.15
            logger.info("Frontier vision scoring enabled (weight=0.15)")

    def set_semantic_prior_engine(self, engine) -> None:
        """注入语义先验引擎 (创新4: Topology-Aware Semantic Exploration)。"""
        self._semantic_prior_engine = engine
        if self.semantic_prior_weight <= 0.0:
            self.semantic_prior_weight = 0.2
            logger.info("Frontier semantic prior scoring enabled (weight=0.2)")

    def set_room_object_kg(self, kg) -> None:
        """注入房间-物体知识图谱 (P2: SEEK-style P(target|room))。"""
        self._room_object_kg = kg

    def record_frontier_failure(self, position: np.ndarray) -> None:
        """
        记录探索失败的 frontier 位置 (P0: 失败记忆)。

        当机器人导航到某个 frontier 后未发现有价值信息时调用。
        后续评分将惩罚该位置附近的 frontier, 避免重复探索死角。

        参考 SEEK (RSS 2024): Dynamic Scene Graph + 失败记忆
        """
        pos = np.asarray(position[:2], dtype=np.float64)
        self._failed_positions.append(pos)
        logger.info(
            "Frontier failure recorded at (%.2f, %.2f), total failures: %d",
            pos[0], pos[1], len(self._failed_positions),
        )

    def _compute_failure_penalty(self, frontier_center: np.ndarray) -> float:
        """
        计算 frontier 与已失败位置的惩罚分 (P0)。

        Returns:
            [0, 1] — 0=无惩罚, 1=最大惩罚 (完全重叠)
        """
        if not self._failed_positions:
            return 0.0

        max_penalty = 0.0
        for fp in self._failed_positions:
            dist = float(np.linalg.norm(frontier_center - fp))
            if dist < self._failure_penalty_radius:
                # 越近惩罚越大, 线性衰减
                penalty = self._failure_penalty_decay * (
                    1.0 - dist / self._failure_penalty_radius
                )
                max_penalty = max(max_penalty, penalty)
        return min(1.0, max_penalty)

    def clear_failure_memory(self) -> None:
        """清空失败记忆 (新任务时调用)。"""
        self._failed_positions.clear()

    def update_room_priors(
        self,
        instruction: str,
        rooms: List[Dict],
        visited_room_ids: Optional[set] = None,
    ) -> None:
        """
        更新房间语义先验缓存 (每次场景图更新后调用)。

        将每个房间对目标指令的先验概率缓存起来,
        在 score_frontiers 时用于评估 frontier 方向对应房间的先验。
        """
        if not self._semantic_prior_engine:
            return
        priors = self._semantic_prior_engine.score_rooms_for_target(
            instruction, rooms, visited_room_ids,
        )
        self._room_priors_cache.clear()
        for rp in priors:
            self._room_priors_cache[rp.room_id] = rp.prior_score

    def update_directional_observation(
        self,
        robot_position: np.ndarray,
        camera_yaw: float,
        image_features: np.ndarray,
    ) -> None:
        """
        缓存某方向的 CLIP 视觉特征 (创新3: VLFM 方向观测缓存)。
        
        每帧由感知节点调用: 提取当前相机朝向的 CLIP 图像特征,
        存入以 angle_bin 为 key 的缓存。当 frontier 评分时,
        取 frontier 方向最近的 angle_bin 的图像特征做
        CLIP(image, instruction) 评分。
        
        Args:
            robot_position: [x, y] 世界坐标 (用于判断缓存有效性)
            camera_yaw: 相机朝向 (弧度, atan2 约定)
            image_features: CLIP 图像特征向量 (L2 归一化)
        """
        if image_features.size == 0:
            return
        # 离散化为 8 个方向 bin (每 45°)
        angle_bin = round(camera_yaw / (2 * math.pi) * 8) % 8
        feat = np.asarray(image_features, dtype=np.float64)
        feat_norm = np.linalg.norm(feat)
        if feat_norm > 0:
            feat = feat / feat_norm
        self._directional_features[angle_bin] = feat

    def _compute_vision_score(
        self, frontier_angle: float, instruction: str
    ) -> float:
        """
        计算 frontier 方向的 CLIP 视觉评分 (VLFM 核心改进)。
        
        取 frontier 方向最近的 angle_bin 的缓存图像特征,
        与 instruction 的 CLIP 文本特征做余弦相似度。
        """
        if not self._clip_encoder or not self._directional_features:
            return 0.0

        # 找最近方向 bin
        target_bin = round(frontier_angle / (2 * math.pi) * 8) % 8

        # 检查 ±1 bin
        for offset in [0, 1, -1]:
            candidate_bin = (target_bin + offset) % 8
            if candidate_bin in self._directional_features:
                img_feat = self._directional_features[candidate_bin]
                try:
                    sim = self._clip_encoder.text_image_similarity(
                        instruction, [img_feat]
                    )
                    if sim and len(sim) > 0:
                        return max(0.0, min(1.0, float(sim[0])))
                except (ValueError, TypeError, AttributeError) as e:
                    logger.debug("CLIP frontier vision scoring failed: %s", e)
        return 0.0

    def score_frontiers(
        self,
        instruction: str,
        robot_position: np.ndarray,
        visited_positions: Optional[List[np.ndarray]] = None,
        scene_objects: Optional[List[Dict]] = None,
        scene_relations: Optional[List[Dict]] = None,
        scene_rooms: Optional[List[Dict]] = None,
    ) -> List[Frontier]:
        """
        对 frontier 评分 (MTU3D 统一 Grounding + VLFM 融合)。

        MTU3D (ICCV 2025) 改进:
          评分 = distance + novelty + language + grounding_potential

          grounding_potential 计算方法:
            1. 场景图物体分布的"空间梯度": 物体密度从已知区域到
               frontier 方向递增 → 目标可能在更远处
            2. 关系链延伸: 如果指令目标的关联物体在 frontier 方向
               → 目标更可能在该方向
            3. 物体类别共现: 目标通常与哪些物体共现 (常识),
               frontier 附近有这些物体 → 加分

        Args:
            instruction: 用户指令 ("找灭火器")
            robot_position: [x, y]
            visited_positions: 已访问位置列表 (拓扑记忆)
            scene_objects: 场景图物体列表
            scene_relations: 场景图空间关系列表

        Returns:
            评分后的 Frontier 列表 (score 降序)
        """
        if not self._frontiers:
            return []

        max_dist = max(f.distance for f in self._frontiers) or 1.0
        inst_lower = instruction.lower()

        # ── 双语关键词集 (跨语言匹配) ──
        inst_keywords = self._extract_bilingual_keywords(inst_lower)

        # ── 预计算: 物体空间分布方向 ──
        obj_directions: List[Tuple[float, str]] = []  # (angle, label)
        if scene_objects:
            for obj in scene_objects:
                obj_pos = np.array([obj["position"]["x"], obj["position"]["y"]])
                delta = obj_pos - robot_position[:2]
                if np.linalg.norm(delta) > 0.5:
                    angle = math.atan2(delta[1], delta[0])
                    obj_directions.append((angle, obj["label"].lower()))

        for frontier in self._frontiers:
            # ── 1. 距离分 (归一化) ──
            dist_score = 1.0 - (frontier.distance / max_dist)

            # ── 2. 新颖度分 (L3MVN 拓扑记忆) ──
            novelty_score = 1.0
            if visited_positions:
                min_visited_dist = min(
                    float(np.linalg.norm(frontier.center_world - vp[:2]))
                    for vp in visited_positions
                ) if visited_positions else float('inf')
                novelty_score = min(1.0, min_visited_dist / self.novelty_distance)

            # ── 3. 语言相关性分 (VLFM) ──
            language_score = 0.0
            nearby_labels = []
            if scene_objects:
                for obj in scene_objects:
                    obj_pos = np.array([obj["position"]["x"], obj["position"]["y"]])
                    dist_to_frontier = float(np.linalg.norm(
                        frontier.center_world - obj_pos
                    ))
                    if dist_to_frontier < self.nearby_object_radius:
                        nearby_labels.append(obj["label"])
                        # 跨语言匹配: "灭火器" 的关键词集含 "fire extinguisher"
                        lbl_lower = obj["label"].lower()
                        if any(kw in lbl_lower or lbl_lower in kw
                               for kw in inst_keywords):
                            language_score += 0.5
                        language_score += self._cooccurrence_score(
                            inst_keywords, lbl_lower
                        )
                frontier.nearby_labels = nearby_labels
                language_score = min(1.0, language_score)

            # ── 4. Grounding Potential (MTU3D 核心改进) ──
            grounding_score = 0.0

            # 4a. 空间梯度: 该方向有相关物体 → 目标可能在更远处
            frontier_angle = math.atan2(
                frontier.center_world[1] - robot_position[1],
                frontier.center_world[0] - robot_position[0],
            )
            for obj_angle, obj_label in obj_directions:
                angle_diff = abs(self._angle_diff(frontier_angle, obj_angle))
                if angle_diff < self.grounding_angle_threshold:
                    grounding_score += self.grounding_spatial_bonus
                    if any(kw in obj_label or obj_label in kw
                           for kw in inst_keywords):
                        grounding_score += self.grounding_keyword_bonus

            # 4b. 关系链延伸 (SG-Nav): 指令目标的关联物体在该方向
            if scene_relations and scene_objects:
                for rel in scene_relations:
                    # 找附近物体是否是"关系链中指向目标"的
                    for nearby_lbl in nearby_labels:
                        related_obj = next(
                            (o for o in scene_objects
                             if o.get("id") == rel.get("object_id")
                             and o.get("label", "").lower() == nearby_lbl.lower()),
                            None,
                        )
                        if related_obj:
                            grounding_score += self.grounding_relation_bonus

            grounding_score = min(1.0, grounding_score)

            # ── 5. 视觉评分 (创新3: VLFM 核心 — CLIP(image, instruction)) ──
            vision_score = 0.0
            if self.vision_weight > 0.0 and self._clip_encoder:
                vision_score = self._compute_vision_score(frontier_angle, instruction)

            # ── 6. 语义先验评分 (创新4: Topology-Aware Semantic Exploration) ──
            semantic_score = 0.0
            if self.semantic_prior_weight > 0.0 and scene_rooms:
                semantic_score = self._compute_semantic_prior_score(
                    frontier, robot_position, scene_rooms,
                )

            # ── 7. KG 目标-房间概率评分 (P2: SEEK-style) ──
            # 如果附近物体暗示某种房间类型, 用 KG 查询目标在该房间的概率
            kg_room_score = 0.0
            if self._room_object_kg and nearby_labels:
                kg_room_score = self._compute_kg_room_score(
                    inst_keywords, nearby_labels,
                )

            # ── 综合评分 ──
            total_w = (self.distance_weight + self.novelty_weight
                       + self.language_weight + self.grounding_weight
                       + self.vision_weight + self.semantic_prior_weight)
            if total_w > 0:
                frontier.score = (
                    self.distance_weight / total_w * dist_score
                    + self.novelty_weight / total_w * novelty_score
                    + self.language_weight / total_w * language_score
                    + self.grounding_weight / total_w * grounding_score
                    + self.vision_weight / total_w * vision_score
                    + self.semantic_prior_weight / total_w * semantic_score
                )
            else:
                frontier.score = 0.0

            # P2: KG room score 作为 additive bonus (不改变权重归一化)
            frontier.score += 0.1 * kg_room_score

            # P0: 失败记忆惩罚 (乘性, 在最终分上折扣)
            failure_penalty = self._compute_failure_penalty(frontier.center_world)
            if failure_penalty > 0:
                frontier.score *= (1.0 - failure_penalty)

        self._frontiers.sort(key=lambda f: f.score, reverse=True)

        # USS-Nav TSP 后处理: 用 IG + 路径距离重排序访问顺序
        if self.tsp_reorder and len(self._frontiers) >= 2:
            self._frontiers = self._tsp_sort_frontiers(
                self._frontiers, robot_position,
            )

        # 为每个 frontier 生成自然语言描述 (L3MVN/OmniNav 风格)
        for frontier in self._frontiers:
            frontier.description = self._generate_frontier_description(frontier)

        return self._frontiers

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """计算两个角度的最小差 (-pi, pi]。"""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    @staticmethod
    def _cooccurrence_score(inst_keywords: Set[str], label: str) -> float:
        """
        常识共现评分 (MTU3D 空间先验)。

        某些物体经常一起出现:
          fire extinguisher ↔ door, sign, stairs, corridor
          kitchen ↔ chair, table, trash can, refrigerator
          bathroom ↔ door, sign, mirror
          office ↔ desk, chair, computer

        Args:
            inst_keywords: 指令双语关键词集 (已通过 expand_bilingual 扩展)
            label: 场景物体标签 (小写)
        """
        COOCCURRENCE = {
            "fire extinguisher": ["door", "sign", "stairs", "elevator", "corridor"],
            "extinguisher": ["door", "sign", "stairs"],
            "灭火器": ["door", "sign", "stairs", "elevator"],
            "chair": ["desk", "table", "computer"],
            "椅子": ["desk", "table", "computer"],
            "desk": ["chair", "computer", "monitor", "keyboard"],
            "桌子": ["chair", "computer", "monitor"],
            "refrigerator": ["sink", "microwave", "table", "chair"],
            "冰箱": ["sink", "microwave", "table"],
            "toilet": ["sink", "mirror", "door"],
            "马桶": ["sink", "mirror", "door"],
            "elevator": ["door", "sign", "stairs", "button"],
            "电梯": ["door", "sign", "stairs"],
        }
        for key, co_labels in COOCCURRENCE.items():
            if key in inst_keywords and label in co_labels:
                return 0.25
        return 0.0

    def _compute_semantic_prior_score(
        self,
        frontier: Frontier,
        robot_position: np.ndarray,
        scene_rooms: List[Dict],
    ) -> float:
        """
        计算 frontier 方向对应房间的语义先验评分 (创新4)。

        思路: frontier 方向 → 最近房间 → 该房间的先验概率
        如果 frontier 指向高先验但未探索的房间类型 → 高分

        当 scene_rooms 为空时 (早期探索), 退化为基于 frontier 附近物体的
        CLIP 房间类型预测: nearby_labels → predict_room_type → 先验匹配。
        """
        # 正常路径: 有房间信息时基于 room_priors_cache
        if self._room_priors_cache and scene_rooms:
            best_score = 0.0
            for room in scene_rooms:
                room_center = np.array([
                    float(room.get("center", {}).get("x", 0.0)),
                    float(room.get("center", {}).get("y", 0.0)),
                ])

                frontier_vec = frontier.center_world - robot_position[:2]
                room_vec = room_center - robot_position[:2]
                fn = np.linalg.norm(frontier_vec)
                rn = np.linalg.norm(room_vec)
                if fn < 0.1 or rn < 0.1:
                    continue

                cos_sim = float(np.dot(frontier_vec, room_vec) / (fn * rn))
                if cos_sim < 0.3:
                    continue

                room_id = room.get("room_id", -1)
                prior = self._room_priors_cache.get(room_id, 0.0)

                score = cos_sim * prior
                best_score = max(best_score, score)

            return min(1.0, best_score)

        # 退化路径: 无房间信息时, 用 frontier 附近物体预测房间类型
        if (self._semantic_prior_engine and frontier.nearby_labels
                and hasattr(self._semantic_prior_engine, 'predict_room_type_from_labels')):
            room_scores = self._semantic_prior_engine.predict_room_type_from_labels(
                frontier.nearby_labels,
            )
            if room_scores:
                # 取最高匹配房间类型的分数 (已排序)
                top_score = next(iter(room_scores.values()), 0.0)
                return min(1.0, top_score)

        return 0.0

    def _compute_kg_room_score(
        self,
        inst_keywords: Set[str],
        nearby_labels: List[str],
    ) -> float:
        """
        P2: 用 KG 推断 frontier 方向的房间类型, 查询目标在该房间的概率。

        思路 (SEEK-style):
          1. 从附近物体推断可能的房间类型 (通过 KG 反向查询)
          2. 对每个推断出的房间类型, 查询指令目标在该房间的概率
          3. 返回最高概率作为评分

        这比 hand-coded 共现表更准确, 因为 KG 是从真实环境学来的。
        """
        if not self._room_object_kg:
            return 0.0

        # 1. 从附近物体推断房间类型
        candidate_rooms: Dict[str, float] = {}  # room_type → max_probability
        for lbl in nearby_labels:
            rooms = self._room_object_kg.get_object_rooms(lbl)
            for room_type, prob in rooms:
                candidate_rooms[room_type] = max(
                    candidate_rooms.get(room_type, 0.0), prob
                )

        if not candidate_rooms:
            return 0.0

        # 2. 对每个候选房间, 查询目标关键词在该房间的概率
        priors = self._room_object_kg.to_room_object_priors(min_observations=1)
        best_score = 0.0
        for room_type, room_prob in candidate_rooms.items():
            room_priors = priors.get(room_type, {})
            for kw in inst_keywords:
                kw_lower = kw.lower()
                for obj_label, obj_prob in room_priors.items():
                    if kw_lower in obj_label or obj_label in kw_lower:
                        # P(target in room) = P(room|nearby_objects) × P(target|room)
                        score = room_prob * obj_prob
                        best_score = max(best_score, score)

        return min(1.0, best_score)

    # ── USS-Nav TSP 方法 ──────────────────────────────────────────

    def _estimate_information_gain(self, frontier: Frontier) -> float:
        """
        估算 frontier 的信息增益 (USS-Nav: IG_i)。

        统计 frontier 中心附近 tsp_ig_radius_cells 范围内的 unknown cell 数量。
        如果没有 costmap, 退化为 frontier.size 作为近似。
        """
        if self._grid is None:
            return max(float(frontier.size), 1.0)

        rows, cols = self._grid.shape
        center_r = int(round(frontier.center[0]))
        center_c = int(round(frontier.center[1]))
        r = self.tsp_ig_radius_cells

        r_lo = max(0, center_r - r)
        r_hi = min(rows, center_r + r + 1)
        c_lo = max(0, center_c - r)
        c_hi = min(cols, center_c + r + 1)

        patch = self._grid[r_lo:r_hi, c_lo:c_hi]
        unknown_count = int(np.count_nonzero(patch == UNKNOWN_CELL))
        return max(float(unknown_count), 1.0)

    def _tsp_sort_frontiers(
        self,
        frontiers: List[Frontier],
        robot_position: np.ndarray,
    ) -> List[Frontier]:
        """
        USS-Nav 贪心 nearest-neighbor TSP 重排序。

        代价矩阵: cost(i, j) = 1/IG_j + euclidean_dist(i, j)
        从 robot 出发, 每步选最低代价的未访问 frontier。
        frontier 数量 > tsp_frontier_limit 时跳过 TSP, 保持原排序。

        Args:
            frontiers: SEEK 评分后的 frontier 列表
            robot_position: [x, y] 世界坐标

        Returns:
            TSP 重排序后的 frontier 列表
        """
        n = len(frontiers)
        if n < 2 or n > self.tsp_frontier_limit:
            return frontiers

        # 预计算每个 frontier 的信息增益
        ig = [self._estimate_information_gain(f) for f in frontiers]

        # 预计算 frontier 中心坐标矩阵
        centers = np.array([f.center_world for f in frontiers])  # (n, 2)
        robot_xy = robot_position[:2]

        # 贪心 nearest-neighbor TSP
        visited = [False] * n
        order: List[int] = []
        current_pos = robot_xy

        for _ in range(n):
            best_idx = -1
            best_cost = float('inf')
            for j in range(n):
                if visited[j]:
                    continue
                dist = float(np.linalg.norm(current_pos - centers[j]))
                cost = 1.0 / ig[j] + dist
                if cost < best_cost:
                    best_cost = cost
                    best_idx = j
            if best_idx < 0:
                break
            visited[best_idx] = True
            order.append(best_idx)
            current_pos = centers[best_idx]

        result = [frontiers[i] for i in order]

        if logger.isEnabledFor(logging.DEBUG):
            ids = [f.frontier_id for f in result]
            igs = [ig[i] for i in order]
            logger.debug(
                "TSP reorder: %s (IG=%s)", ids,
                [round(g, 1) for g in igs],
            )

        return result

    def get_best_frontier(self) -> Optional[Frontier]:
        """获取评分最高的 frontier (TSP 重排序后为访问序列第一个)。"""
        if self._frontiers:
            return self._frontiers[0]
        return None

    @staticmethod
    def _extract_bilingual_keywords(inst_lower: str) -> Set[str]:
        """从指令提取关键词并双语扩展, 用于跨语言 frontier 评分。"""
        import re
        stop_en = {
            "the", "a", "an", "to", "go", "find", "near", "with", "at",
            "of", "please", "where", "is", "i", "want", "need", "look",
        }
        # 中文停用前缀/后缀 — 用于从连续中文串中剥离动词前缀
        stop_zh_prefix = ["去", "找", "到", "在", "看", "帮", "把", "让", "给"]
        # 提取英文 token
        en_tokens = re.findall(r"[a-z]+", inst_lower)
        keywords = [t for t in en_tokens if t not in stop_en and len(t) > 1]
        # 提取中文 token: 先整段提取, 再切分并清理
        # 用常见分隔词 (的/旁边/附近/里面...) 切割长中文串, 得到名词片段
        zh_splitters = r"的|旁边|附近|里面|上面|下面|前面|后面|左边|右边|对面|中间|那里|那边|这里|这边|那个|这个"
        zh_raw = re.findall(r"[\u4e00-\u9fff]+", inst_lower)
        for zh in zh_raw:
            # 按空间/结构词切分: "门旁边的椅子" → ["门", "椅子"]
            parts = re.split(zh_splitters, zh)
            for part in parts:
                # 剥离停用前缀: "去找灭火器" → "灭火器"
                cleaned = part
                changed = True
                while changed and len(cleaned) > 1:
                    changed = False
                    for pfx in stop_zh_prefix:
                        if cleaned.startswith(pfx) and len(cleaned) > len(pfx):
                            cleaned = cleaned[len(pfx):]
                            changed = True
                            break
                # 单字中文: 只保留在双语字典中有映射的 (门→door, 窗→window 等)
                if len(cleaned) > 1 or (len(cleaned) == 1 and cleaned in _ZH_TO_EN_KEYS):
                    keywords.append(cleaned)
        # 双语扩展
        if _expand_bilingual is not None:
            keywords = _expand_bilingual(keywords)
        return set(keywords)

    def _generate_frontier_description(self, frontier: Frontier) -> str:
        """为 frontier 生成自然语言描述（L3MVN/OmniNav 风格）。

        结果按标签集合缓存：相同标签组合直接返回缓存，避免重复调用
        predict_room_type_from_labels（每次约 5-20ms）。
        """
        labels = getattr(frontier, 'nearby_labels', []) or []
        unique_labels = list(dict.fromkeys(labels))[:6]  # 最多6个去重标签

        # 缓存 key: 排序后标签集合（描述只依赖标签内容，不依赖原始顺序）
        cache_key = ",".join(sorted(unique_labels))
        if cache_key in self._frontier_desc_cache:
            return self._frontier_desc_cache[cache_key]

        room_type = "未知区域"
        if unique_labels and self._semantic_prior_engine is not None:
            try:
                if hasattr(self._semantic_prior_engine, 'predict_room_type_from_labels'):
                    room_probs = self._semantic_prior_engine.predict_room_type_from_labels(
                        unique_labels,
                    )
                    if room_probs:
                        best_room = max(room_probs, key=room_probs.get)
                        confidence = room_probs[best_room]
                        if confidence > 0.3:
                            room_type = best_room
            except (ImportError, TypeError, ValueError, AttributeError) as e:
                logger.debug("Room type prediction failed: %s", e)

        if unique_labels:
            desc = f"可见对象：{'、'.join(unique_labels)}，推测区域：{room_type}"
        else:
            desc = f"未知区域（尚无可见对象），推测：{room_type}"

        # 写入缓存；超过 256 条时清除最旧的一半（防内存无限增长）
        if len(self._frontier_desc_cache) >= 256:
            stale_keys = list(self._frontier_desc_cache.keys())[:128]
            for k in stale_keys:
                del self._frontier_desc_cache[k]
        self._frontier_desc_cache[cache_key] = desc
        return desc

    def get_frontiers_summary(self) -> str:
        """导出 frontier 摘要 (给 LLM 消费)。"""
        import json
        return json.dumps({
            "frontier_count": len(self._frontiers),
            "frontiers": [f.to_dict() for f in self._frontiers[:5]],
        }, ensure_ascii=False)

    @staticmethod
    def _angle_to_label(angle: float) -> str:
        """角度 (rad, atan2 约定) → 方向标签。

        0 = east, pi/2 = north, pi/-pi = west, -pi/2 = south
        """
        # 8 方向
        directions = [
            "east", "northeast", "north", "northwest",
            "west", "southwest", "south", "southeast",
        ]
        idx = round(angle / (2 * math.pi) * 8) % 8
        return directions[idx]
