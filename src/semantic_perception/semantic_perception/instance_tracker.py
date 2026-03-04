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
import time
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

from .projection import Detection3D

logger = logging.getLogger(__name__)


# ── 空间关系常量 (参考 SG-Nav 的关系类型) ──

RELATION_NEAR_THRESHOLD = 1.5      # 米, 小于此距离视为 "near"
RELATION_ON_THRESHOLD = 0.3        # 米, z 轴差异小于此视为 "on" (物体在另一个上面)
REGION_CLUSTER_RADIUS = 3.0        # 米, 同一区域的聚类半径
FLOOR_HEIGHT = 3.0                 # 米, 楼层高度估算 (用于 z 坐标→楼层映射)
FLOOR_MERGE_TOLERANCE = 0.8        # 米, 楼层 z 坐标合并容差

# ── BA-HSG 信念参数 (Belief-Aware Hierarchical Scene Graph) ──
BELIEF_SIGMA_BASE = 0.05           # 位置基准噪声 (m)
BELIEF_SIGMA_DEPTH_COEFF = 0.02    # 深度比例噪声系数
BELIEF_NEG_EVIDENCE_WEIGHT = 0.5   # 负面证据权重 (未检测到时)
BELIEF_FRESHNESS_TAU = 30.0        # 时间衰减常数 (s)
BELIEF_REPROJ_KAPPA = 5.0          # 重投影误差衰减常数 (px)
BELIEF_ROOM_BOOST = 0.3            # 房间信念对新物体的提升幅度
BELIEF_LATERAL_SHARE = 0.1         # 空间邻居信念分享系数

# ── KG-Augmented Loopy Belief Propagation (论文级升级) ──
# 参考: Belief Scene Graphs (ICRA 2024), Commonsense BSG (2025)
# 创新: 用 KG 结构化先验替代 GCN 训练, 多轮迭代替代单次推理
BP_MAX_ITERATIONS = 3              # Loopy BP 最大迭代轮数
BP_CONVERGENCE_EPS = 0.005         # 收敛判据: max |Δ P_exist| < ε
BP_KG_PRIOR_BOOST = 1.5           # KG 先验: 期望物体在该房间时 α 提升量
BP_KG_UNEXPECTED_PENALTY = 0.3    # KG 约束: 非期望物体 β 增加量 (温和怀疑)
BP_ROOM_TO_OBJ_WEIGHT = 0.6       # 下行传播强度: 房间后验→物体先验
BP_OBJ_TO_ROOM_WEIGHT = 0.8       # 上行传播强度: 物体证据→房间类型后验
BP_LATERAL_DECAY = 0.7            # 横向传播衰减: 距离越远信息越弱
BP_PHANTOM_BASE_ALPHA = 0.8       # Phantom (blind) 节点初始 α
BP_PHANTOM_MIN_ROOM_CONFIDENCE = 0.4  # 房间类型置信度 > 此值才生成 phantom

# ── Safety-Aware Differential Credibility (安全感知差异化阈值) ──
# 创新: 根据物体危险等级动态调整所需可信度阈值
# 双阈值策略: 导航避障用低阈值 (宁可信其有), 交互操作用高阈值 (严格确认)
SAFETY_THRESHOLDS_NAVIGATION = {
    "safe": 0.25,
    "caution": 0.15,       # 更低! 检测到可能危险物 → 尽早避障
    "dangerous": 0.10,     # 极低: 一点迹象就触发避障
    "forbidden": 0.05,     # 几乎零容忍: 微弱信号即报警
}
SAFETY_THRESHOLDS_INTERACTION = {
    "safe": 0.40,
    "caution": 0.60,       # 需要更多确认
    "dangerous": 0.80,     # 严格确认才允许靠近
    "forbidden": 0.95,     # 几乎不可能允许交互
}
# Dangerous 物体 Alpha 初始保守系数 (贝叶斯先验偏保守)
SAFETY_PRIOR_ALPHA_SCALE = {
    "safe": 1.0,
    "caution": 1.2,        # 略微提升 → 更快确认 (宁可误报)
    "dangerous": 1.5,      # 明显提升 → 危险物体更快被 "相信存在"
    "forbidden": 2.0,      # 最高 → 极少量证据即确认 (保护性偏见)
}

# ── 规则 Room 命名映射 (创新1: 增强启发式, 替代 area_with_ 拼接) ──
ROOM_TYPE_RULES = {
    "corridor": {
        "keywords": ["door", "sign", "corridor", "hallway", "exit", "门", "走廊", "出口"],
        "min_match": 1,
        "priority": 2,
    },
    "office": {
        "keywords": ["desk", "chair", "computer", "monitor", "keyboard", "mouse", "办公", "桌", "电脑"],
        "min_match": 2,
        "priority": 3,
    },
    "kitchen": {
        "keywords": ["refrigerator", "sink", "microwave", "oven", "kettle", "冰箱", "厨房", "水壶"],
        "min_match": 1,
        "priority": 3,
    },
    "meeting_room": {
        "keywords": ["table", "chair", "screen", "projector", "whiteboard", "会议", "投影"],
        "min_match": 2,
        "priority": 3,
    },
    "bathroom": {
        "keywords": ["toilet", "sink", "mirror", "卫生间", "洗手", "镜"],
        "min_match": 1,
        "priority": 3,
    },
    "stairwell": {
        "keywords": ["stairs", "staircase", "railing", "楼梯", "扶手"],
        "min_match": 1,
        "priority": 3,
    },
    "lobby": {
        "keywords": ["sofa", "reception", "lobby", "大厅", "沙发", "前台"],
        "min_match": 1,
        "priority": 2,
    },
    "storage": {
        "keywords": ["shelf", "cabinet", "box", "storage", "储物", "柜", "架"],
        "min_match": 2,
        "priority": 1,
    },
}


def infer_room_type(labels: List[str]) -> str:
    """基于物体标签推断 Room 类型 (增强规则命名)。
    
    比 'area_with_door_chair' 更可读, 且不需要 LLM 调用。
    当 LLM 命名不可用时作为 fallback。
    """
    labels_lower = [l.lower() for l in labels]
    best_type = ""
    best_priority = -1
    best_matches = 0

    for room_type, rule in ROOM_TYPE_RULES.items():
        matches = sum(
            1 for kw in rule["keywords"]
            if any(kw in l for l in labels_lower)
        )
        if matches >= rule["min_match"]:
            score = matches * 10 + rule["priority"]
            if score > best_priority:
                best_priority = score
                best_type = room_type
                best_matches = matches

    if best_type:
        return best_type

    # Fallback: 用最常见物体标签
    if labels:
        from collections import Counter
        common = Counter(labels).most_common(2)
        return f"area_{'_'.join(l for l, _ in common)}"
    return "unknown_area"


GROUP_KEYWORDS = {
    "safety": ["fire", "extinguisher", "alarm", "灭火", "应急"],
    "furniture": ["chair", "desk", "table", "sofa", "cabinet", "shelf", "椅", "桌", "柜"],
    "structure": ["door", "window", "stairs", "elevator", "hall", "corridor", "门", "窗", "楼梯"],
    "electronics": ["monitor", "screen", "computer", "tv", "phone", "显示", "电脑", "电视"],
    "utility": ["trash", "bin", "bottle", "refrigerator", "sink", "lamp", "垃圾", "冰箱", "灯"],
}


@dataclass
class SpatialRelation:
    """物体间空间关系 (SG-Nav 风格)。"""
    subject_id: int
    relation: str                  # "near" | "left_of" | "right_of" | "in_front_of" | "behind" | "on" | "above" | "below"
    object_id: int
    distance: float = 0.0


ROOM_NAMING_STABILITY_COUNT = 3       # Region 内物体数 >= 此值时可触发 LLM 命名
ROOM_NAMING_STABILITY_SEC = 10.0      # Region 内物体集合稳定持续 N 秒后触发 LLM 命名


@dataclass
class Region:
    """空间区域 (SG-Nav 层次场景图的房间级)。"""
    region_id: int
    center: np.ndarray             # [x, y] 区域中心
    object_ids: List[int] = field(default_factory=list)
    name: str = ""                 # 自动命名或 LLM 命名
    llm_named: bool = False        # 是否已经被 LLM 命名


@dataclass
class GroupNode:
    """层次场景图的 group 层节点。"""

    group_id: int
    room_id: int
    name: str
    center: np.ndarray
    object_ids: List[int] = field(default_factory=list)
    semantic_labels: List[str] = field(default_factory=list)


@dataclass
class RoomNode:
    """层次场景图的 room 层节点。"""

    room_id: int
    name: str
    center: np.ndarray
    object_ids: List[int] = field(default_factory=list)
    group_ids: List[int] = field(default_factory=list)
    semantic_labels: List[str] = field(default_factory=list)
    llm_named: bool = False        # 是否由 LLM 命名


@dataclass
class FloorNode:
    """层次场景图的 floor 层节点 (SPADE IROS 2025 + HOV-SG 层次)。

    场景图层次: Floor → Room → Group → Object
    楼层通过物体 z 坐标聚类推断, 每 FLOOR_HEIGHT 为一层。
    """

    floor_id: int
    floor_level: int               # 楼层号: 0=地面层, 1=二楼, -1=地下
    z_range: Tuple[float, float]   # (z_min, z_max) 该楼层物体 z 坐标范围
    room_ids: List[int] = field(default_factory=list)
    object_ids: List[int] = field(default_factory=list)
    center_z: float = 0.0         # 楼层 z 中心


@dataclass
class PhantomNode:
    """盲节点 (Blind Node): 由 KG 先验 + 房间类型后验推断的未见但期望存在的物体。

    参考 Belief Scene Graphs (ICRA 2024) 的 blind node 概念,
    但用 KG 结构化先验替代 GCN 训练 — 无需训练数据, 可解释, 可扩展。

    生成条件:
      1. 房间类型后验置信度 > BP_PHANTOM_MIN_ROOM_CONFIDENCE
      2. KG.get_room_expected_objects(room_type) 包含此物体
      3. 该物体在该房间内尚未被检测到
    """
    phantom_id: int
    label: str                         # 期望物体类别
    room_id: int                       # 所属房间
    room_type: str                     # 推断的房间类型
    position: np.ndarray               # 预测位置 (房间中心 + KG 偏移)
    belief_alpha: float                # 存在性先验 α
    belief_beta: float = 1.0           # 存在性先验 β
    kg_prior_strength: float = 0.0     # KG 先验强度 P(object | room_type)
    safety_level: str = "safe"
    source: str = "kg_room_prior"      # 来源标记

    @property
    def existence_prob(self) -> float:
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)


@dataclass
class RoomTypePosterior:
    """房间类型贝叶斯后验 — 多假设追踪。

    不同于 BSG 直接用标签, 也不同于 HOV-SG 用 CLIP 投票,
    我们维护 P(room_type | observed_objects) 的完整后验分布。

    更新公式 (对数域避免下溢):
      log P(t | O) ∝ log P(t) + Σ_i log P(o_i | t)
    其中 P(o_i | t) 来自 KG.get_room_expected_objects(t) 的匹配情况。
    """
    room_id: int
    hypotheses: Dict[str, float] = field(default_factory=dict)
    _log_posteriors: Dict[str, float] = field(default_factory=dict)

    @property
    def best_type(self) -> str:
        if not self.hypotheses:
            return "unknown"
        return max(self.hypotheses, key=self.hypotheses.get)

    @property
    def best_confidence(self) -> float:
        if not self.hypotheses:
            return 0.0
        return max(self.hypotheses.values())

    @property
    def entropy(self) -> float:
        """信息熵 — 衡量房间类型不确定性, 高熵 = 需要更多探索。"""
        if not self.hypotheses:
            return 0.0
        vals = list(self.hypotheses.values())
        total = sum(vals)
        if total <= 0:
            return 0.0
        h = 0.0
        for v in vals:
            p = v / total
            if p > 1e-10:
                h -= p * math.log2(p)
        return h


@dataclass
class BeliefMessage:
    """信念传播消息 (loopy BP 的 message 抽象)。

    在层次场景图中, 消息沿以下方向流动:
      Object → Room (上行): 物体存在性证据聚合为房间类型后验
      Room → Object (下行): 房间类型后验调整物体先验
      Object ↔ Object (横向): 空间邻居信念共享
      Room → Phantom (生成): 房间期望但未见物体 → 产生 phantom node
    """
    source_type: str      # "object" | "room" | "phantom"
    source_id: int
    target_type: str      # "object" | "room" | "phantom"
    target_id: int
    message_type: str     # "existence" | "room_type" | "spatial_context"
    delta_alpha: float = 0.0
    delta_beta: float = 0.0
    weight: float = 1.0


@dataclass
class ViewNode:
    """关键视角节点 (FSR-VLN 风格的 view 层近似实现)。"""

    view_id: int
    position: np.ndarray
    timestamp: float
    room_id: int = -1
    object_ids: List[int] = field(default_factory=list)
    key_labels: List[str] = field(default_factory=list)


@dataclass
class TrackedObject:
    """全局跟踪的单个物体实例 (BA-HSG + USS-Nav 点云融合)。"""
    object_id: int
    label: str
    position: np.ndarray          # [x, y, z] world frame (点云质心)
    best_score: float
    detection_count: int = 1
    last_seen: float = 0.0        # timestamp
    features: np.ndarray = field(default_factory=lambda: np.array([]))
    region_id: int = -1           # 所属区域 (SG-Nav 层次)
    extent: np.ndarray = field(default_factory=lambda: np.array([0.2, 0.2, 0.2]))
    # USS-Nav: 物体累积点云 (N, 3) world frame
    points: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))

    # ── BA-HSG 信念状态 ──
    # Beta(α, β) 存在性分布: P(exists) ~ Beta(α, β)
    belief_alpha: float = 1.5     # 初始偏乐观
    belief_beta: float = 1.0
    # 位置不确定性 (各向同性 Gaussian σ²)
    position_variance: float = 1.0  # 初始高不确定性 (m²)
    # 负面证据: 在预期视域内连续未检测到的次数
    miss_streak: int = 0
    # 复合可信度 (缓存值, 由 compute_credibility() 更新)
    credibility: float = 0.5

    # ── KG 知识增强 (ConceptBot / OpenFunGraph) ──
    kg_concept_id: str = ""        # 匹配的 KG 概念 ID
    safety_level: str = "safe"     # safe / caution / dangerous / forbidden
    affordances: List[str] = field(default_factory=list)  # graspable, openable, ...
    functional_properties: Dict = field(default_factory=dict)  # KG 补充属性
    floor_level: int = 0           # 所在楼层

    # ── KG-Augmented Prior (论文核心创新) ──
    # 与 BSG (ICRA 2024) 的 GCN 学习不同, 我们用 KG 结构化先验直接注入 Beta 参数
    kg_prior_alpha: float = 0.0    # KG 注入的先验 α (累计, 可分解来源)
    kg_prior_source: str = ""      # 先验来源追踪 ("room_type:office", "co_occurrence:desk")
    is_kg_expected: bool = False   # 此物体是否为当前房间类型的 KG 期望物体
    safety_nav_threshold: float = 0.25   # 导航避障用的安全阈值 (动态设置)
    safety_interact_threshold: float = 0.40  # 交互操作用的安全阈值

    def update(self, det: Detection3D, alpha: float = 0.3) -> None:
        """用新检测更新位置、置信度、点云和信念状态 (USS-Nav + BA-HSG)。"""
        # USS-Nav: 融合点云 → 更新质心位置
        if hasattr(det, 'points') and det.points is not None and len(det.points) > 0:
            self._fuse_pointcloud(det.points)
            self.position = np.mean(self.points, axis=0) if len(self.points) > 0 else det.position.copy()
            self.position_variance = max(0.01, self.position_variance * 0.8)
        else:
            depth = float(np.linalg.norm(det.position[:2]))
            obs_var = (BELIEF_SIGMA_BASE + BELIEF_SIGMA_DEPTH_COEFF * depth) ** 2
            new_var = 1.0 / (1.0 / max(self.position_variance, 1e-6) + 1.0 / max(obs_var, 1e-6))
            self.position = new_var * (
                self.position / max(self.position_variance, 1e-6)
                + det.position / max(obs_var, 1e-6)
            )
            self.position_variance = new_var

        self.best_score = max(self.best_score, det.score)
        self.detection_count += 1
        self.last_seen = time.time()
        self.miss_streak = 0

        safety_scale = SAFETY_PRIOR_ALPHA_SCALE.get(self.safety_level, 1.0)
        self.belief_alpha += 1.0 * safety_scale

        if det.features.size > 0:
            self.features = self._fuse_feature(det.features)
        self._update_extent(det)
        self._update_credibility()

    def _fuse_pointcloud(
        self, new_points: np.ndarray, max_total: int = 1024,
    ) -> None:
        """
        USS-Nav: 增量融合新观测点云到物体累积点云。

        策略: 合并 → 体素降采样 → 限制最大点数。
        """
        if new_points is None or len(new_points) == 0:
            return
        if self.points is None or len(self.points) == 0:
            self.points = new_points.copy()
        else:
            self.points = np.vstack([self.points, new_points])

        from .projection import _voxel_downsample, POINTCLOUD_VOXEL_SIZE
        self.points = _voxel_downsample(self.points, POINTCLOUD_VOXEL_SIZE, max_total)

    def _fuse_feature(self, new_feature: np.ndarray) -> np.ndarray:
        """多视角 CLIP 特征融合 (ConceptGraphs 风格 EMA + L2 normalize)。
        
        创新1 补强: 每次新观测以 α=0.3 权重融入历史特征 (0.7),
        并做 L2 归一化保证特征在单位球面上。当检测质量 (best_score) 较高时
        给予新观测更大权重, 质量低时减小权重, 对齐 ConceptGraphs 论文的
        multi-view feature aggregation 思路。
        
        质量感知 α 计算:
          base_alpha = 0.3
          quality_factor = clamp(best_score / 0.8, 0.5, 1.5)
          alpha = min(0.5, base_alpha * quality_factor)
        """
        nf = np.asarray(new_feature, dtype=np.float64)
        if nf.size == 0:
            return self.features

        nf_norm = np.linalg.norm(nf)
        if nf_norm > 0:
            nf = nf / nf_norm

        if self.features.size == 0:
            return nf

        of = np.asarray(self.features, dtype=np.float64)
        of_norm = np.linalg.norm(of)
        if of_norm > 0:
            of = of / of_norm

        # ConceptGraphs 风格: 固定 base_alpha = 0.3, 质量感知调节
        base_alpha = 0.3
        quality_factor = max(0.5, min(1.5, self.best_score / 0.8))
        alpha = min(0.5, base_alpha * quality_factor)

        fused = (1.0 - alpha) * of + alpha * nf
        fused_norm = np.linalg.norm(fused)
        if fused_norm > 0:
            fused = fused / fused_norm
        return fused

    def _update_extent(self, det: Detection3D) -> None:
        """从 2D bbox 和深度估算 3D 包围盒半径。"""
        if det.depth <= 0 or det.bbox_2d.size < 4:
            return
        x1, y1, x2, y2 = det.bbox_2d
        bbox_w = max(float(x2 - x1), 1.0)
        bbox_h = max(float(y2 - y1), 1.0)
        # 粗略估算: 假设 fx ~ 600 (典型 RGB-D 相机)
        fx_approx = 600.0
        extent_x = (bbox_w / fx_approx) * det.depth * 0.5
        extent_y = extent_x  # 对称近似
        extent_z = (bbox_h / fx_approx) * det.depth * 0.5
        new_ext = np.array([
            max(extent_x, 0.05),
            max(extent_y, 0.05),
            max(extent_z, 0.05),
        ])
        # EMA 平滑尺寸
        self.extent = 0.3 * new_ext + 0.7 * self.extent

    # ── BA-HSG 信念方法 ──

    @property
    def existence_prob(self) -> float:
        """Beta 后验均值: P(exists) = α / (α + β)。"""
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)

    @property
    def existence_uncertainty(self) -> float:
        """Beta 后验方差 (不确定性指标)。"""
        a, b = self.belief_alpha, self.belief_beta
        return (a * b) / ((a + b) ** 2 * (a + b + 1))

    def record_miss(self) -> None:
        """在预期视域内未检测到该物体 → 负面证据。"""
        self.miss_streak += 1
        self.belief_beta += BELIEF_NEG_EVIDENCE_WEIGHT
        self._update_credibility()

    def _update_credibility(self) -> None:
        """计算复合可信度 C — 安全感知版 (BA-HSG v2)。

        与 BSG (ICRA 2024) 的区别: 我们的可信度计算融入了
        KG 先验强度和安全等级, 而不仅仅是检测统计量。

        公式:
          C = w_exist × P_exist + w_view × view_div + w_fresh × freshness
              + w_det × det_score + w_kg × kg_factor
        其中 kg_factor = min(kg_prior_alpha / 3, 0.3) 反映 KG 对该物体的信心。
        """
        dt = time.time() - self.last_seen if self.last_seen > 0 else 999.0
        view_diversity = 1.0 - 1.0 / max(self.detection_count, 1)
        freshness = math.exp(-dt / BELIEF_FRESHNESS_TAU)
        kg_factor = min(self.kg_prior_alpha / 3.0, 0.3) if self.kg_prior_alpha > 0 else 0.0

        self.credibility = (
            0.35 * self.existence_prob
            + 0.15 * view_diversity
            + 0.15 * freshness
            + 0.15 * min(self.best_score, 1.0)
            + 0.20 * kg_factor
        )

        # 动态设置安全阈值
        self.safety_nav_threshold = SAFETY_THRESHOLDS_NAVIGATION.get(
            self.safety_level, 0.25)
        self.safety_interact_threshold = SAFETY_THRESHOLDS_INTERACTION.get(
            self.safety_level, 0.40)

    @property
    def is_confirmed_for_navigation(self) -> bool:
        """导航层是否应将此物体视为障碍/标志 (低阈值, 宁可信其有)。"""
        return self.credibility >= self.safety_nav_threshold

    @property
    def is_confirmed_for_interaction(self) -> bool:
        """交互层是否允许与此物体进行操作 (高阈值, 严格确认)。"""
        return self.credibility >= self.safety_interact_threshold

    def to_belief_dict(self) -> dict:
        """导出信念状态 (供 LLM prompt 和日志使用)。"""
        d = {
            "P_exist": round(self.existence_prob, 3),
            "sigma_pos": round(math.sqrt(self.position_variance), 3),
            "credibility": round(self.credibility, 3),
            "detections": self.detection_count,
            "miss_streak": self.miss_streak,
            "confirmed_nav": self.is_confirmed_for_navigation,
            "confirmed_interact": self.is_confirmed_for_interaction,
        }
        if self.kg_concept_id:
            d["kg_concept"] = self.kg_concept_id
            d["safety"] = self.safety_level
            d["affordances"] = self.affordances
        if self.kg_prior_alpha > 0:
            d["kg_prior_alpha"] = round(self.kg_prior_alpha, 3)
            d["kg_prior_source"] = self.kg_prior_source
            d["is_kg_expected"] = self.is_kg_expected
        if self.floor_level != 0:
            d["floor"] = self.floor_level
        return d


class InstanceTracker:
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

        self._objects: Dict[int, TrackedObject] = {}
        self._next_id = 0

        # 关键视角层 (view nodes)
        self._views: Dict[int, ViewNode] = {}
        self._next_view_id = 0
        self._last_view_id = -1

        # Room LLM 命名状态 (创新1: 在线增量场景图补强)
        self._room_llm_namer: Optional[Callable] = None  # async (labels) -> str
        self._room_name_cache: Dict[int, str] = {}       # region_id -> LLM name
        self._region_stability: Dict[int, Tuple[frozenset, float]] = {}  # region_id -> (obj_id_set, stable_since)

        # 知识图谱 (ConceptBot / OpenFunGraph 增强)
        self._knowledge_graph = knowledge_graph

        # 楼层层 (SPADE / HOV-SG)
        self._cached_floors: List[FloorNode] = []

        # ── Loopy Belief Propagation 状态 ──
        self._room_type_posteriors: Dict[int, RoomTypePosterior] = {}
        self._phantom_nodes: Dict[int, PhantomNode] = {}
        self._next_phantom_id = 0
        self._bp_messages_log: List[BeliefMessage] = []  # 调试用: 最近一轮的消息
        self._bp_iteration_count = 0                      # 统计: 总 BP 迭代次数
        self._bp_convergence_history: List[float] = []    # 统计: 每轮最大 Δ

        # ── Neuro-Symbolic Belief GCN (KG-BELIEF) ──
        self._belief_model = None  # Optional[BeliefPredictor]

        # ── Neuro-Symbolic Belief GCN (论文核心: 训练式信念推理) ──
        self._belief_model = None  # Optional[BeliefPredictor]

        # ── Neuro-Symbolic Belief GCN (KG-BELIEF) ──
        self._belief_model = None  # Optional[BeliefPredictor]

    @property
    def objects(self) -> Dict[int, TrackedObject]:
        return self._objects

    @property
    def views(self) -> Dict[int, ViewNode]:
        return self._views

    def set_room_namer(self, namer: Callable) -> None:
        """注册 Room LLM 命名回调 (async def namer(labels: List[str]) -> str)。
        
        创新1 补强: Region 稳定后 (物体数 >= 3 且持续 10s 无变化),
        调用 LLM 将 'area_with_door_chair' 命名为 '走廊' / 'office' 等可读名称,
        提升 LLM 推理时的语义质量。
        """
        self._room_llm_namer = namer

    def update(self, detections: List[Detection3D]) -> List[TrackedObject]:
        """
        用本帧检测结果更新全局物体表。

        Args:
            detections: 本帧 3D 检测列表

        Returns:
            本帧匹配/新建的 TrackedObject 列表
        """
        matched: List[TrackedObject] = []

        for det in detections:
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

        # BA-HSG: 负面证据 — 在视域内的已知物体未被检测到 → miss
        detected_ids = {m.object_id for m in matched}
        for obj in self._objects.values():
            if obj.object_id not in detected_ids and obj.detection_count >= 2:
                dt = time.time() - obj.last_seen
                if dt > 5.0:
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

        # BA-HSG: 图扩散传播 (每次更新后执行)
        self.propagate_beliefs()

        return matched

    def record_view(
        self,
        camera_position: np.ndarray,
        object_ids: List[int],
        min_distance: float = 1.0,
        min_interval: float = 1.0,
    ) -> Optional[ViewNode]:
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

    def _collect_labels(self, object_ids: List[int], limit: int = 8) -> List[str]:
        labels: List[str] = []
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

    def _find_match(self, det: Detection3D) -> Optional[TrackedObject]:
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
        best_sem_match: Optional[TrackedObject] = None
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
            best_geo_match: Optional[TrackedObject] = None
            best_geo_score = -1.0

            for obj, dist in candidates:
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
        best_fallback: Optional[TrackedObject] = None
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

    # ================================================================
    #  空间关系 (ConceptGraphs / SG-Nav)
    # ================================================================

    def compute_spatial_relations(self) -> List[SpatialRelation]:
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
        relations: List[SpatialRelation] = []
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
            dist_2d = math.sqrt(dx * dx + dy * dy)
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

    def compute_regions(self) -> List[Region]:
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
        from collections import Counter

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
        cluster_to_objs: Dict[int, List[int]] = {}
        for idx, cl in enumerate(cluster_labels):
            cl_int = int(cl)
            if cl_int == -1:
                # DBSCAN 噪声点, 分配唯一簇ID
                noise_id = max(cluster_to_objs.keys(), default=-1) + 1
                cluster_to_objs[noise_id] = [idx]
            else:
                cluster_to_objs.setdefault(cl_int, []).append(idx)

        regions: List[Region] = []
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

    def compute_groups(self, regions: List[Region]) -> List[GroupNode]:
        """
        根据 room/region 内物体语义构建 group 层。

        SG-Nav 的 group 层用于把同一区域中的同类物体聚为中间节点，
        降低 LLM 直接处理 object 级图的复杂度。
        """
        groups: List[GroupNode] = []
        group_index: Dict[Tuple[int, str], int] = {}

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

    def compute_rooms(self, regions: List[Region], groups: List[GroupNode]) -> List[RoomNode]:
        """将 region 标准化为 room 节点。
        
        创新1 补强: 当 Region 满足稳定条件时 (物体数 >= ROOM_NAMING_STABILITY_COUNT
        且持续 ROOM_NAMING_STABILITY_SEC 无变化), 异步调用 LLM 为 Room 赋予
        可读的语义名称 (e.g. '走廊', 'office'), 替换掉 'area_with_door_chair' 式的
        拼接命名。LLM 命名结果缓存在 _room_name_cache 中, 避免重复调用。
        """
        room_to_groups: Dict[int, List[int]] = {}
        for g in groups:
            room_to_groups.setdefault(g.room_id, []).append(g.group_id)

        now = time.time()
        rooms: List[RoomNode] = []
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

            rooms.append(
                RoomNode(
                    room_id=r.region_id,
                    name=room_name,
                    center=r.center.copy(),
                    object_ids=list(r.object_ids),
                    group_ids=room_to_groups.get(r.region_id, []),
                    semantic_labels=labels,
                    llm_named=is_llm_named,
                )
            )
        return rooms

    _LLM_NAMING_MAX_CONCURRENT = 3
    _LLM_NAMING_TIMEOUT_S = 15.0
    _llm_pending_tasks: Dict[int, asyncio.Task] = {}

    def _trigger_room_llm_naming(self, region_id: int, labels: List[str]) -> None:
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
        rooms: List[RoomNode],
    ) -> List[Dict]:
        """
        计算房间间拓扑连通边 (Hydra + Concept-Guided Exploration 融合)。

        三种检测策略:
        1. Door-mediated: "door"物体处于两个房间边界 → 连通边
        2. Proximity: 两个房间中心距 < 阈值且有物体靠近 → 隐含连通
        3. Passage-mediated: "corridor"类房间自动与相邻房间连通
        """
        if len(rooms) < 2:
            return []

        edges: List[Dict] = []
        seen_pairs: set = set()

        room_by_id = {r.room_id: r for r in rooms}

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

    def _estimate_frontier_directions(self, rooms: List[RoomNode]) -> List[Dict]:
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

        frontiers: List[Dict] = []
        all_positions = np.array([obj.position[:2] for obj in self._objects.values()])
        scene_center = all_positions.mean(axis=0)

        door_kw = {"door", "gate", "entrance", "exit", "门", "出口", "入口"}
        door_objects = [
            obj for obj in self._objects.values()
            if any(kw in obj.label.lower() for kw in door_kw)
        ]

        room_centers = {r.room_id: r.center for r in rooms}
        room_obj_ids = {r.room_id: set(r.object_ids) for r in rooms}

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
    def _build_hierarchy_edges(rooms: List[RoomNode], groups: List[GroupNode]) -> List[Dict]:
        """构建 room→group→object 的层次边。"""
        edges: List[Dict] = []

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
    def _build_view_edges(views: List[ViewNode]) -> List[Dict]:
        """构建 room→view→object 边。"""
        edges: List[Dict] = []
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
    def _assign_view_rooms(views: List[ViewNode], rooms: List[RoomNode]) -> List[ViewNode]:
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
        rooms: List[RoomNode],
        groups: List[GroupNode],
        views: List[ViewNode],
        objects_by_id: Dict[int, TrackedObject],
        relations_list: List[Dict],
    ) -> List[Dict]:
        """构建 SG-Nav 风格子图摘要，供 planner 侧按子图推理。"""
        subgraphs: List[Dict] = []

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

    def propagate_beliefs(self) -> None:
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
        """
        if not self._objects:
            return

        regions = self.compute_regions()
        self._cached_regions = regions

        # 每轮 BP 从头开始计算房间后验和 phantom (避免跨调用残留)
        self._room_type_posteriors.clear()
        self._phantom_nodes.clear()

        # 记录迭代前的 P_exist 快照 (收敛判据)
        prev_beliefs: Dict[int, float] = {
            oid: obj.existence_prob for oid, obj in self._objects.items()
        }
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
            self._bp_iteration_count += 1

            if max_delta < BP_CONVERGENCE_EPS:
                logger.debug("Loopy BP converged at iteration %d (Δ=%.6f)", iteration, max_delta)
                break

    # ──────────────────────────────────────────────────────────
    #  Phase 1: 上行传播 — Object → Room Type Posterior
    # ──────────────────────────────────────────────────────────

    def _bp_phase_upward(self, regions: List['Region']) -> None:
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
            "office", "kitchen", "corridor", "meeting_room", "bathroom",
            "stairwell", "lobby", "storage", "server_room", "warehouse",
            "lab", "parking", "outdoor", "elevator_hall", "factory",
            "hospital", "entrance", "utility_room", "bedroom", "living_room",
            "break_room", "laundry",
        ]

        for region in regions:
            labels_in_room = [
                self._objects[oid].label.lower()
                for oid in region.object_ids
                if oid in self._objects
            ]
            if not labels_in_room:
                continue

            log_posteriors: Dict[str, float] = {}
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

    def _bp_phase_downward(self, regions: List['Region']) -> None:
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

    def _bp_phase_downward_gcn(self, regions: List['Region']) -> None:
        """Phase 2A: 用 KG-BELIEF GCN 推理完整直方图。

        GCN 输出: per-room predicted probability for each object class
        对于每个房间中的物体:
          - predicted_prob > 0.5 → 该物体是 "期望的" → alpha 提升
          - predicted_prob < 0.2 且 detection_count <= 2 → beta 惩罚
        """
        room_labels: List[List[str]] = []
        room_types: List[str] = []
        region_list: List = []

        for region in regions:
            labels = [
                self._objects[oid].label
                for oid in region.object_ids
                if oid in self._objects
            ]
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

                    self._bp_messages_log.append(BeliefMessage(
                        source_type="room", source_id=region.region_id,
                        target_type="object", target_id=oid,
                        message_type="existence",
                        delta_alpha=delta_a, weight=gcn_prob,
                    ))

                elif gcn_prob < 0.2 and obj.detection_count <= 2:
                    delta_b = BP_KG_UNEXPECTED_PENALTY * (1.0 - gcn_prob)
                    obj.belief_beta += delta_b
                    obj.is_kg_expected = False

                    self._bp_messages_log.append(BeliefMessage(
                        source_type="room", source_id=region.region_id,
                        target_type="object", target_id=oid,
                        message_type="existence",
                        delta_beta=delta_b, weight=1.0 - gcn_prob,
                    ))

    def _bp_phase_downward_kg_lookup(self, regions: List['Region']) -> None:
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
                    rt_expected = set(
                        self._knowledge_graph.get_room_expected_objects(rtype))
                    if obj.label.lower() in rt_expected:
                        p_expected += rprob

                if p_expected > 0.3:
                    delta_a = BP_KG_PRIOR_BOOST * p_expected * BP_ROOM_TO_OBJ_WEIGHT
                    obj.kg_prior_alpha += delta_a
                    obj.belief_alpha += delta_a
                    obj.is_kg_expected = True
                    obj.kg_prior_source = f"room_type:{best_type}(p={best_conf:.2f})"

                    self._bp_messages_log.append(BeliefMessage(
                        source_type="room", source_id=region.region_id,
                        target_type="object", target_id=oid,
                        message_type="existence",
                        delta_alpha=delta_a, weight=p_expected,
                    ))

                elif p_expected < 0.1 and obj.detection_count <= 2:
                    delta_b = BP_KG_UNEXPECTED_PENALTY * (1.0 - p_expected)
                    obj.belief_beta += delta_b
                    obj.is_kg_expected = False

                    self._bp_messages_log.append(BeliefMessage(
                        source_type="room", source_id=region.region_id,
                        target_type="object", target_id=oid,
                        message_type="existence",
                        delta_beta=delta_b, weight=1.0 - p_expected,
                    ))

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
            pairs = [(i, j) for i in range(n) for j in range(i + 1, n)
                     if np.linalg.norm(obj_list[i].position[:2] - obj_list[j].position[:2])
                     < RELATION_NEAR_THRESHOLD]

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

    def _bp_phase_phantom_generation(self, regions: List['Region']) -> None:
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

            observed_labels = {
                self._objects[oid].label.lower()
                for oid in region.object_ids
                if oid in self._objects
            }

            # 对每个高概率房间假设, 收集期望但未见的物体
            phantom_candidates: Dict[str, float] = {}  # label → aggregated P
            phantom_safety: Dict[str, str] = {}

            for rtype, rprob in posterior.hypotheses.items():
                if rprob < 0.1:
                    continue
                expected = self._knowledge_graph.get_room_expected_objects(rtype)
                for obj_label in expected:
                    if obj_label.lower() not in observed_labels:
                        phantom_candidates[obj_label] = (
                            phantom_candidates.get(obj_label, 0.0) + rprob
                        )
                        # 从 KG 获取安全等级
                        if obj_label not in phantom_safety:
                            props = self._knowledge_graph.enrich_object_properties(obj_label)
                            phantom_safety[obj_label] = props.get("safety_level", "safe")

            # 创建 phantom 节点 (只保留聚合概率 > 阈值的)
            for label, agg_prob in sorted(
                phantom_candidates.items(), key=lambda x: -x[1]
            ):
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
                    position=region.center.copy() if hasattr(region, 'center') else np.zeros(3),
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

    def get_phantom_nodes(self) -> List[PhantomNode]:
        """获取当前所有 phantom (blind) 节点, 按存在概率降序排列。"""
        return sorted(
            self._phantom_nodes.values(),
            key=lambda p: p.existence_prob,
            reverse=True,
        )

    def promote_phantom(self, phantom_id: int, detection: 'Detection3D') -> Optional[TrackedObject]:
        """将 phantom 节点实体化: 当检测到与 phantom 匹配的物体时调用。

        phantom 的先验 α 被继承到新 TrackedObject 中 — 信息不丢失。
        """
        phantom = self._phantom_nodes.pop(phantom_id, None)
        if phantom is None:
            return None

        obj = TrackedObject(
            object_id=self._next_id,
            label=detection.label,
            position=detection.position.copy(),
            best_score=detection.score,
            last_seen=time.time(),
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

    def get_room_type_posteriors(self) -> Dict[int, RoomTypePosterior]:
        """获取所有房间的类型后验分布 (供 LLM 和规划器使用)。"""
        return dict(self._room_type_posteriors)

    def get_exploration_targets(self) -> List[Dict]:
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
                for r in getattr(self, '_cached_regions', []):
                    if r.region_id == rtp.room_id:
                        region = r
                        break
                if region is not None:
                    targets.append({
                        "type": "explore_room",
                        "room_id": rtp.room_id,
                        "reason": f"high_uncertainty (H={rtp.entropy:.2f})",
                        "position": region.center.tolist(),
                        "priority": rtp.entropy,
                    })

        for phantom in self.get_phantom_nodes()[:10]:
            priority = phantom.existence_prob
            if phantom.safety_level in ("dangerous", "forbidden"):
                priority *= 3.0  # 危险物体优先探索确认
            targets.append({
                "type": "confirm_phantom",
                "phantom_id": phantom.phantom_id,
                "label": phantom.label,
                "room_id": phantom.room_id,
                "room_type": phantom.room_type,
                "reason": f"expected_{phantom.label} (P={phantom.existence_prob:.2f})",
                "position": phantom.position.tolist(),
                "priority": priority,
                "safety_level": phantom.safety_level,
            })

        targets.sort(key=lambda t: -t["priority"])
        return targets

    def get_bp_diagnostics(self) -> Dict:
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

    def get_scene_graph_json(self) -> str:
        """
        导出完整场景图为 JSON (SG-Nav 风格)。

        包含:
          - objects: 物体列表 (id, label, position, score)
          - relations: 空间关系 (subject→relation→object)
          - regions: 空间区域 (聚类的物体组)
          - summary: 自然语言摘要 (帮助 LLM 理解)
        """
        import json

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
            entry = {
                "id": obj.object_id,
                "label": obj.label,
                "position": {
                    "x": round(float(obj.position[0]), 3),
                    "y": round(float(obj.position[1]), 3),
                    "z": round(float(obj.position[2]), 3),
                },
                "score": round(obj.best_score, 3),
                "detection_count": obj.detection_count,
                "region_id": obj.region_id,
                "floor_level": obj.floor_level,
                "belief": obj.to_belief_dict(),
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

        return json.dumps(
            {
                "timestamp": time.time(),
                "frame_id": "map",
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
            },
            ensure_ascii=False,
        )

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
            from semantic_perception.belief_network import BeliefPredictor
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
        save_path: Optional[str] = None,
    ) -> bool:
        """训练 KG-BELIEF GCN 模型 (从 KG 合成数据)。"""
        if self._knowledge_graph is None:
            logger.warning("Cannot train belief model without KG")
            return False
        try:
            from semantic_perception.belief_network import (
                BeliefPredictor, BeliefTrainer as BTrainer,
                KGBeliefGCN, KGSceneGraphDataset, SafetyWeightedBCELoss,
                build_object_vocabulary, build_cooccurrence_matrix,
                build_safety_vector, build_affordance_vectors,
                build_room_prior_vectors, build_safety_loss_weights,
            )
            import torch

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
    #  楼层层 (Floor Level — SPADE IROS 2025 / HOV-SG)
    # ════════════════════════════════════════════════════════════

    def compute_floors(self) -> List[FloorNode]:
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

        floors: List[FloorNode] = []
        current_floor_objs: List[int] = []
        current_z_min = z_values[z_sorted_indices[0]]
        current_z_sum = 0.0

        for idx in z_sorted_indices:
            z = z_values[idx]
            obj = objs[idx]

            if z - current_z_min > FLOOR_HEIGHT - FLOOR_MERGE_TOLERANCE and current_floor_objs:
                center_z = current_z_sum / len(current_floor_objs)
                floors.append(FloorNode(
                    floor_id=len(floors),
                    floor_level=len(floors),
                    z_range=(current_z_min, z_values[z_sorted_indices[
                        z_sorted_indices.tolist().index(idx) - 1
                    ]] if idx > 0 else current_z_min),
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
        self, floors: List[FloorNode], rooms: List[RoomNode]
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

    # ════════════════════════════════════════════════════════════
    #  开放词汇查询 (EmbodiedRAG + CLIP 增强)
    # ════════════════════════════════════════════════════════════

    def query_by_text(
        self,
        query: str,
        top_k: int = 5,
        clip_encoder=None,
    ) -> List[TrackedObject]:
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
    ) -> List[TrackedObject]:
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
    ) -> List[TrackedObject]:
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
    ) -> List[TrackedObject]:
        """查询特定安全等级的物体。"""
        return [
            obj for obj in self._objects.values()
            if obj.safety_level == safety_level
        ]

    def query_by_floor(
        self,
        floor_level: int,
        label: Optional[str] = None,
        top_k: int = 20,
    ) -> List[TrackedObject]:
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
    ) -> Dict:
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

    def compute_scene_diff(self, prev_snapshot: Dict) -> Dict:
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
    def _summarize_diff(events: List[Dict]) -> str:
        """生成场景变化的自然语言摘要。"""
        if not events:
            return "No changes detected."
        parts = []
        added = [e for e in events if e["type"] == "object_added"]
        removed = [e for e in events if e["type"] == "object_removed"]
        moved = [e for e in events if e["type"] == "object_moved"]
        if added:
            labels = [e["label"] for e in added[:5]]
            parts.append(f"{len(added)} new: {', '.join(labels)}")
        if removed:
            labels = [e["label"] for e in removed[:5]]
            parts.append(f"{len(removed)} gone: {', '.join(labels)}")
        if moved:
            descs = [f"{e['label']}({e['displacement']:.1f}m)" for e in moved[:5]]
            parts.append(f"{len(moved)} moved: {', '.join(descs)}")
        return " | ".join(parts)

    def apply_local_update(
        self,
        region_id: int,
        new_detections: List[Detection3D],
    ) -> Dict:
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
    ) -> List[Tuple[TrackedObject, float]]:
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
    ) -> List[Dict]:
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
            }
        for vid, view in self._views.items():
            data["views"][str(vid)] = {
                "position": view.position.tolist(),
                "timestamp": view.timestamp,
                "room_id": view.room_id,
                "object_ids": view.object_ids,
                "key_labels": view.key_labels,
            }

        import json
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        logger.info("Scene graph saved to %s (%d objects, %d views)",
                     path, len(self._objects), len(self._views))

    def load_from_file(self, path: str) -> bool:
        """从文件恢复场景图 (长期记忆加载)。"""
        import json
        try:
            with open(path, 'r', encoding='utf-8') as f:
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
            )
            obj._update_credibility()
            self._enrich_from_kg(obj)
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
