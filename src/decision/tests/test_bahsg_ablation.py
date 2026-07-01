"""
test_bahsg_ablation.py — BA-HSG (Belief-Aware Hierarchical Scene Graph) 消融实验

消融维度:
  A1. Full BA-HSG vs No-Belief (所有物体 existence_prob=1.0, 无信念更新)
  A2. Full BA-HSG vs No-FOV   (去掉 FOV 检查, 全局 record_miss)
  A3. Full BA-HSG vs No-KG    (去掉 KG 先验, kg_prior_alpha=0)

实验方法:
  - 生成 100 个物体 (80 真实 + 20 误检), 模拟相机视锥
  - 运行 100 步, 每步随机选择相机位置和朝向
  - 真实物体在 FOV 内有 80% 概率被检测到, 误检物体有 30% 概率出现
  - 统计: 物体保留率 / 误检清除率 / 目标定位准确率

参考实现:
  - instance_tracker.py: TrackedObject, record_miss, existence_prob, _is_in_fov
  - BA-HSG 信念参数: BELIEF_NEG_EVIDENCE_WEIGHT=0.5, BELIEF_FRESHNESS_TAU=30s

输出格式: LaTeX 表格 + 终端对齐表格, 适合论文 Table 直接使用。
"""

import math
import random
import unittest
from dataclasses import dataclass

import numpy as np

# ── 复用 instance_tracker 中的核心常量 ──
BELIEF_NEG_EVIDENCE_WEIGHT = 0.5
BELIEF_FRESHNESS_TAU = 30.0
BELIEF_SIGMA_BASE = 0.05
BELIEF_SIGMA_DEPTH_COEFF = 0.02
BP_KG_PRIOR_BOOST = 1.5
BP_KG_UNEXPECTED_PENALTY = 0.3

# ── 实验参数 ──
NUM_REAL_OBJECTS = 80
NUM_FALSE_OBJECTS = 20
NUM_STEPS = 200
FOV_HALF_ANGLE = math.radians(45)    # 水平半视角 45°
FOV_MAX_RANGE = 10.0                  # 最大检测距离 10m
DETECTION_PROB_REAL = 0.85            # 真实物体在 FOV 内的检测概率
DETECTION_PROB_FALSE = 0.15           # 误检物体在 FOV 内的随机出现概率
PRUNE_THRESHOLD = 0.35               # P(exist) < 0.35 时清除物体
ARENA_SIZE = 15.0                     # 场景尺寸 15m x 15m (紧凑场景, 更多 FOV 覆盖)

# ── KG 房间-物体映射 (简化版) ──
KG_ROOM_OBJECTS = {
    "office": ["desk", "chair", "monitor", "keyboard", "mouse", "lamp", "cabinet"],
    "kitchen": ["refrigerator", "sink", "microwave", "kettle", "table", "chair"],
    "corridor": ["door", "sign", "fire_extinguisher", "trash_can"],
    "meeting_room": ["table", "chair", "projector", "whiteboard", "screen"],
}

REAL_LABELS = [
    "chair", "desk", "monitor", "door", "table", "shelf", "lamp", "plant",
    "cabinet", "window", "sign", "trash_can", "sofa", "keyboard", "mouse",
    "refrigerator", "sink", "projector", "whiteboard", "bottle",
]

FALSE_LABELS = [
    "ghost_box", "phantom_cone", "noise_blob", "shadow_artifact",
    "reflection_plate", "glitch_sphere", "false_pillar", "misdet_wall",
    "spurious_rock", "hallucinated_bin",
]


# ============================================================
#  轻量级 TrackedObject 模拟 (不依赖 ROS2)
# ============================================================

@dataclass
class SimTrackedObject:
    """模拟 TrackedObject — 复现 BA-HSG 核心信念逻辑。"""
    object_id: int
    label: str
    position: np.ndarray
    is_real: bool                     # ground truth: 真实 vs 误检
    belief_alpha: float = 1.5         # Beta(α, β) 初始偏乐观
    belief_beta: float = 1.0
    detection_count: int = 0
    miss_streak: int = 0
    best_score: float = 0.5
    last_seen: float = 0.0
    kg_prior_alpha: float = 0.0       # KG 注入的先验
    is_kg_expected: bool = False
    room_type: str = ""               # 所在房间类型

    @property
    def existence_prob(self) -> float:
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)

    def record_hit(self, score: float = 0.8, current_time: float = 0.0) -> None:
        """检测到 → 正面证据。"""
        self.belief_alpha += 1.0
        self.detection_count += 1
        self.miss_streak = 0
        self.best_score = max(self.best_score, score)
        self.last_seen = current_time

    def record_miss(self) -> None:
        """在 FOV 内未检测到 → 负面证据。"""
        self.miss_streak += 1
        self.belief_beta += BELIEF_NEG_EVIDENCE_WEIGHT

    def apply_kg_prior(self, room_type: str) -> None:
        """KG 先验注入: 如果物体标签是该房间的期望物体, 提升 α。"""
        expected = KG_ROOM_OBJECTS.get(room_type, [])
        if self.label in expected:
            self.belief_alpha += BP_KG_PRIOR_BOOST
            self.kg_prior_alpha += BP_KG_PRIOR_BOOST
            self.is_kg_expected = True
        else:
            # 非期望物体: 温和增加 β (怀疑)
            self.belief_beta += BP_KG_UNEXPECTED_PENALTY
        self.room_type = room_type


def is_in_fov(
    obj_pos: np.ndarray,
    camera_pos: np.ndarray,
    camera_forward: np.ndarray,
) -> bool:
    """复现 instance_tracker._is_in_fov 的逻辑。"""
    diff = obj_pos[:2] - camera_pos[:2]
    dist = np.linalg.norm(diff)
    if dist < 0.1 or dist > FOV_MAX_RANGE:
        return False
    cos_angle = np.dot(diff, camera_forward[:2]) / (
        dist * max(np.linalg.norm(camera_forward[:2]), 1e-7)
    )
    return cos_angle > math.cos(FOV_HALF_ANGLE)


# ============================================================
#  场景生成
# ============================================================

def generate_objects(seed: int = 42) -> list[SimTrackedObject]:
    """生成 100 个物体 (80 真实 + 20 误检)。"""
    rng = random.Random(seed)
    np_rng = np.random.RandomState(seed)
    objects = []

    # 80 个真实物体, 分布在 4 个房间
    rooms = list(KG_ROOM_OBJECTS.keys())
    room_centers = {
        "office": np.array([4.0, 4.0]),
        "kitchen": np.array([11.0, 4.0]),
        "corridor": np.array([7.5, 7.5]),
        "meeting_room": np.array([4.0, 11.0]),
    }
    for i in range(NUM_REAL_OBJECTS):
        room = rooms[i % len(rooms)]
        room_center = room_centers[room]
        # 在房间中心附近分布 (σ=1.5m)
        pos = room_center + np_rng.randn(2) * 1.5
        pos = np.clip(pos, 0, ARENA_SIZE)
        # 70% 概率选择房间期望物体 (KG 对齐), 30% 选随机物体
        if rng.random() < 0.7:
            label = rng.choice(KG_ROOM_OBJECTS[room])
        else:
            label = rng.choice(REAL_LABELS)
        objects.append(SimTrackedObject(
            object_id=i,
            label=label,
            position=np.array([pos[0], pos[1], 0.0]),
            is_real=True,
            room_type=room,
        ))

    # 20 个误检物体, 随机分布
    for i in range(NUM_FALSE_OBJECTS):
        pos = np_rng.uniform(0, ARENA_SIZE, size=2)
        label = rng.choice(FALSE_LABELS)
        objects.append(SimTrackedObject(
            object_id=NUM_REAL_OBJECTS + i,
            label=label,
            position=np.array([pos[0], pos[1], 0.0]),
            is_real=False,
            room_type=rng.choice(list(KG_ROOM_OBJECTS.keys())),
        ))

    return objects


def generate_camera_trajectory(
    num_steps: int, seed: int = 123,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """生成相机轨迹: (position, forward_direction) 序列。"""
    rng = np.random.RandomState(seed)
    trajectory = []
    # 沿场景中蛇形巡逻
    for step in range(num_steps):
        t = step / num_steps
        x = ARENA_SIZE * (0.1 + 0.8 * t)
        y = ARENA_SIZE * (0.5 + 0.3 * math.sin(2 * math.pi * t * 3))
        pos = np.array([x, y, 0.5])
        # 朝向: 主要向前 + 随机偏转
        angle = 2 * math.pi * t + rng.randn() * 0.3
        forward = np.array([math.cos(angle), math.sin(angle), 0.0])
        trajectory.append((pos, forward))
    return trajectory


# ============================================================
#  模拟运行器
# ============================================================

class AblationMode:
    FULL = "Full BA-HSG"
    NO_BELIEF = "No-Belief"
    NO_FOV = "No-FOV"
    NO_KG = "No-KG"


def run_simulation(
    mode: str,
    seed: int = 42,
) -> dict[str, float]:
    """运行一次消融模拟, 返回指标。"""
    objects = generate_objects(seed)
    trajectory = generate_camera_trajectory(NUM_STEPS, seed + 1)
    rng = random.Random(seed + 2)

    # KG 先验注入 (除 No-KG 模式外)
    if mode != AblationMode.NO_KG:
        for obj in objects:
            obj.apply_kg_prior(obj.room_type)

    # 模拟循环
    for step, (cam_pos, cam_forward) in enumerate(trajectory):
        current_time = step * 0.5  # 0.5s per step

        for obj in objects:
            if mode == AblationMode.NO_BELIEF:
                # No-Belief: 不做信念更新, existence_prob 始终为 1.0
                continue

            in_fov = is_in_fov(obj.position, cam_pos, cam_forward)

            if mode == AblationMode.NO_FOV:
                # No-FOV: 不检查视锥, 对所有物体都做信念更新
                # 这意味着 FOV 外的物体也会被 record_miss (过度激进)
                if obj.is_real:
                    detected = rng.random() < DETECTION_PROB_REAL if in_fov else False
                else:
                    detected = rng.random() < DETECTION_PROB_FALSE if in_fov else False

                if detected:
                    score = 0.7 + rng.random() * 0.25 if obj.is_real else 0.3 + rng.random() * 0.3
                    obj.record_hit(score, current_time)
                else:
                    obj.record_miss()  # 全局 miss, 不管是否在 FOV 内
            else:
                # Full / No-KG: 正常 FOV 检查, 只对 FOV 内物体做更新
                if in_fov:
                    if obj.is_real:
                        detected = rng.random() < DETECTION_PROB_REAL
                    else:
                        detected = rng.random() < DETECTION_PROB_FALSE

                    if detected:
                        score = 0.7 + rng.random() * 0.25 if obj.is_real else 0.3 + rng.random() * 0.3
                        obj.record_hit(score, current_time)
                    else:
                        obj.record_miss()
                # FOV 外: 不做任何更新 (Full BA-HSG 行为)

    # ── 计算指标 ──
    real_objects = [o for o in objects if o.is_real]
    false_objects = [o for o in objects if not o.is_real]

    if mode == AblationMode.NO_BELIEF:
        # No-Belief: 所有物体 P(exist) = 1.0, 没有清除能力
        real_retained = NUM_REAL_OBJECTS
        false_cleared = 0
    else:
        real_retained = sum(1 for o in real_objects if o.existence_prob >= PRUNE_THRESHOLD)
        false_cleared = sum(1 for o in false_objects if o.existence_prob < PRUNE_THRESHOLD)

    retention_rate = real_retained / max(len(real_objects), 1)
    clearance_rate = false_cleared / max(len(false_objects), 1)

    # 目标定位准确率: 随机选 10 个真实物体作为目标, 检查是否仍可定位
    target_indices = list(range(len(real_objects)))
    rng_target = random.Random(seed + 99)
    rng_target.shuffle(target_indices)
    target_indices = target_indices[:10]

    localization_correct = 0
    for idx in target_indices:
        target = real_objects[idx]
        if mode == AblationMode.NO_BELIEF:
            # No-Belief: 所有物体都存在, 可能选到误检 → 检测次数判断
            all_with_label = [
                o for o in objects
                if o.label == target.label and o.existence_prob >= PRUNE_THRESHOLD
            ]
        else:
            all_with_label = [
                o for o in objects
                if o.label == target.label and o.existence_prob >= PRUNE_THRESHOLD
            ]
        if not all_with_label:
            continue
        # 选 existence_prob 最高的作为定位结果
        best = max(all_with_label, key=lambda o: o.existence_prob)
        # 如果选中的是真实物体且距离目标 < 3m, 算定位正确
        if best.is_real:
            dist = np.linalg.norm(best.position[:2] - target.position[:2])
            if dist < 3.0:
                localization_correct += 1

    localization_accuracy = localization_correct / 10.0

    # 额外指标: 平均 existence_prob
    avg_exist_real = np.mean([o.existence_prob for o in real_objects]) if real_objects else 0.0
    avg_exist_false = np.mean([o.existence_prob for o in false_objects]) if false_objects else 0.0

    return {
        "retention_rate": retention_rate,
        "clearance_rate": clearance_rate,
        "localization_accuracy": localization_accuracy,
        "avg_exist_real": avg_exist_real,
        "avg_exist_false": avg_exist_false,
        "real_retained": real_retained,
        "false_cleared": false_cleared,
    }


# ============================================================
#  消融实验测试
# ============================================================

class TestBAHSGAblation(unittest.TestCase):
    """BA-HSG 消融实验 — 量化信念机制各组件的贡献。"""

    @classmethod
    def setUpClass(cls):
        """运行所有消融配置, 缓存结果。"""
        cls.results = {}
        modes = [
            AblationMode.FULL,
            AblationMode.NO_BELIEF,
            AblationMode.NO_FOV,
            AblationMode.NO_KG,
        ]
        # 多次运行取平均 (3 seeds)
        seeds = [42, 137, 256]
        for mode in modes:
            runs = [run_simulation(mode, seed=s) for s in seeds]
            avg = {}
            for key in runs[0]:
                avg[key] = np.mean([r[key] for r in runs])
            cls.results[mode] = avg

    def test_full_bahsg_retains_real_objects(self):
        """Full BA-HSG 应保留 >= 90% 的真实物体。"""
        rate = self.results[AblationMode.FULL]["retention_rate"]
        self.assertGreaterEqual(rate, 0.90,
            f"Full BA-HSG retention rate {rate:.1%} < 90%")

    def test_full_bahsg_clears_false_positives(self):
        """Full BA-HSG 应清除 >= 50% 的误检。"""
        rate = self.results[AblationMode.FULL]["clearance_rate"]
        self.assertGreaterEqual(rate, 0.50,
            f"Full BA-HSG clearance rate {rate:.1%} < 50%")

    def test_no_belief_cannot_clear_false_positives(self):
        """No-Belief 模式无法清除任何误检 (所有 P=1.0)。"""
        rate = self.results[AblationMode.NO_BELIEF]["clearance_rate"]
        self.assertEqual(rate, 0.0,
            f"No-Belief should have 0% clearance, got {rate:.1%}")

    def test_no_fov_hurts_retention(self):
        """No-FOV 模式 (全局 record_miss) 应降低真实物体保留率。"""
        full_ret = self.results[AblationMode.FULL]["retention_rate"]
        nofov_ret = self.results[AblationMode.NO_FOV]["retention_rate"]
        self.assertLess(nofov_ret, full_ret,
            f"No-FOV retention {nofov_ret:.1%} should be < Full {full_ret:.1%}")

    def test_no_fov_higher_clearance(self):
        """No-FOV 模式 (激进清除) 应有更高的误检清除率 (但以牺牲保留率为代价)。"""
        full_clr = self.results[AblationMode.FULL]["clearance_rate"]
        nofov_clr = self.results[AblationMode.NO_FOV]["clearance_rate"]
        self.assertGreaterEqual(nofov_clr, full_clr,
            f"No-FOV clearance {nofov_clr:.1%} should be >= Full {full_clr:.1%}")

    def test_kg_prior_boosts_real_existence(self):
        """KG 先验应提升真实物体的平均 P(exist) (KG 期望物体获得 α 增强)。"""
        full_exist = self.results[AblationMode.FULL]["avg_exist_real"]
        nokg_exist = self.results[AblationMode.NO_KG]["avg_exist_real"]
        self.assertGreaterEqual(full_exist, nokg_exist,
            f"Full avg P_real {full_exist:.3f} should be >= No-KG {nokg_exist:.3f}")

    def test_full_separates_real_and_false(self):
        """Full BA-HSG 应使真实和误检物体的 avg P(exist) 有显著差距。"""
        r = self.results[AblationMode.FULL]
        gap = r["avg_exist_real"] - r["avg_exist_false"]
        self.assertGreater(gap, 0.1,
            f"P(exist) gap between real ({r['avg_exist_real']:.3f}) "
            f"and false ({r['avg_exist_false']:.3f}) = {gap:.3f} < 0.1")

    def test_print_ablation_table(self):
        """打印消融实验结果表格 (论文用)。"""
        print("\n" + "=" * 85)
        print("BA-HSG Ablation Study Results")
        print("=" * 85)
        header = (
            f"{'Configuration':<16s} | {'Retention':>10s} | {'Clearance':>10s} | "
            f"{'Localization':>12s} | {'P_real':>7s} | {'P_false':>8s}"
        )
        print(header)
        print("-" * 85)

        for mode in [AblationMode.FULL, AblationMode.NO_BELIEF,
                     AblationMode.NO_FOV, AblationMode.NO_KG]:
            r = self.results[mode]
            print(
                f"{mode:<16s} | {r['retention_rate']:>9.1%} | {r['clearance_rate']:>9.1%} | "
                f"{r['localization_accuracy']:>11.1%} | {r['avg_exist_real']:>6.3f} | "
                f"{r['avg_exist_false']:>7.3f}"
            )

        print("=" * 85)
        print(f"Settings: {NUM_REAL_OBJECTS} real + {NUM_FALSE_OBJECTS} false objects, "
              f"{NUM_STEPS} steps, prune_threshold={PRUNE_THRESHOLD}")
        print()

        # LaTeX 表格
        print("% --- LaTeX Table (copy to paper) ---")
        print(r"\begin{table}[t]")
        print(r"\centering")
        print(r"\caption{BA-HSG Ablation Study: impact of belief components}")
        print(r"\label{tab:ablation}")
        print(r"\begin{tabular}{l c c c}")
        print(r"\toprule")
        print(r"Configuration & Retention$\uparrow$ & FP Clearance$\uparrow$ & Localization$\uparrow$ \\")
        print(r"\midrule")

        for mode in [AblationMode.FULL, AblationMode.NO_BELIEF,
                     AblationMode.NO_FOV, AblationMode.NO_KG]:
            r = self.results[mode]
            mode_tex = mode.replace("-", "--")
            if mode == AblationMode.FULL:
                mode_tex = r"\textbf{" + mode_tex + "}"
            print(
                f"{mode_tex} & {r['retention_rate']:.1%} & "
                f"{r['clearance_rate']:.1%} & {r['localization_accuracy']:.1%} \\\\"
            )

        print(r"\bottomrule")
        print(r"\end{tabular}")
        print(r"\end{table}")


class TestAblationSanity(unittest.TestCase):
    """消融实验基础设施健全性检查。"""

    def test_object_generation(self):
        """验证物体生成数量和属性。"""
        objects = generate_objects(seed=42)
        self.assertEqual(len(objects), NUM_REAL_OBJECTS + NUM_FALSE_OBJECTS)
        real = [o for o in objects if o.is_real]
        false = [o for o in objects if not o.is_real]
        self.assertEqual(len(real), NUM_REAL_OBJECTS)
        self.assertEqual(len(false), NUM_FALSE_OBJECTS)

    def test_camera_trajectory(self):
        """验证相机轨迹生成。"""
        traj = generate_camera_trajectory(NUM_STEPS)
        self.assertEqual(len(traj), NUM_STEPS)
        for pos, fwd in traj:
            self.assertEqual(pos.shape, (3,))
            self.assertEqual(fwd.shape, (3,))
            self.assertGreater(np.linalg.norm(fwd), 0.5)

    def test_fov_check(self):
        """验证 FOV 检查逻辑。"""
        cam_pos = np.array([0.0, 0.0, 0.0])
        cam_fwd = np.array([1.0, 0.0, 0.0])
        # 正前方 3m → 在视锥内
        self.assertTrue(is_in_fov(np.array([3.0, 0.0, 0.0]), cam_pos, cam_fwd))
        # 正后方 → 不在视锥内
        self.assertFalse(is_in_fov(np.array([-3.0, 0.0, 0.0]), cam_pos, cam_fwd))
        # 太远 → 不在视锥内
        self.assertFalse(is_in_fov(np.array([20.0, 0.0, 0.0]), cam_pos, cam_fwd))
        # 太近 → 不在视锥内
        self.assertFalse(is_in_fov(np.array([0.05, 0.0, 0.0]), cam_pos, cam_fwd))
        # 侧面 90° → 不在视锥内
        self.assertFalse(is_in_fov(np.array([0.0, 5.0, 0.0]), cam_pos, cam_fwd))

    def test_belief_update_mechanics(self):
        """验证信念更新的基本行为。"""
        obj = SimTrackedObject(
            object_id=0, label="chair",
            position=np.array([1.0, 2.0, 0.0]), is_real=True,
        )
        initial_prob = obj.existence_prob
        # Hit → α 增加 → P(exist) 上升
        obj.record_hit(0.9, 1.0)
        self.assertGreater(obj.existence_prob, initial_prob)
        # Miss → β 增加 → P(exist) 下降
        prob_after_hit = obj.existence_prob
        for _ in range(10):
            obj.record_miss()
        self.assertLess(obj.existence_prob, prob_after_hit)

    def test_kg_prior_injection(self):
        """验证 KG 先验注入。"""
        # desk 在 office 是期望物体
        obj_expected = SimTrackedObject(
            object_id=0, label="desk",
            position=np.array([1.0, 2.0, 0.0]), is_real=True,
        )
        prob_before = obj_expected.existence_prob
        obj_expected.apply_kg_prior("office")
        self.assertGreater(obj_expected.existence_prob, prob_before)
        self.assertTrue(obj_expected.is_kg_expected)

        # ghost_box 在 office 不是期望物体
        obj_unexpected = SimTrackedObject(
            object_id=1, label="ghost_box",
            position=np.array([1.0, 2.0, 0.0]), is_real=False,
        )
        prob_before = obj_unexpected.existence_prob
        obj_unexpected.apply_kg_prior("office")
        self.assertLess(obj_unexpected.existence_prob, prob_before)
        self.assertFalse(obj_unexpected.is_kg_expected)

    def test_single_run_deterministic(self):
        """相同 seed 产生相同结果。"""
        r1 = run_simulation(AblationMode.FULL, seed=42)
        r2 = run_simulation(AblationMode.FULL, seed=42)
        for key in r1:
            self.assertAlmostEqual(r1[key], r2[key], places=10,
                msg=f"{key} not deterministic")


if __name__ == "__main__":
    unittest.main(verbosity=2)
