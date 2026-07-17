"""Decision module."""

import math
import random
import unittest
from dataclasses import dataclass

import numpy as np

BELIEF_NEG_EVIDENCE_WEIGHT = 0.5
BELIEF_FRESHNESS_TAU = 30.0
BELIEF_SIGMA_BASE = 0.05
BELIEF_SIGMA_DEPTH_COEFF = 0.02
BP_KG_PRIOR_BOOST = 1.5
BP_KG_UNEXPECTED_PENALTY = 0.3


NUM_REAL_OBJECTS = 80
NUM_FALSE_OBJECTS = 20
NUM_STEPS = 200
FOV_HALF_ANGLE = math.radians(45)
FOV_MAX_RANGE = 10.0
DETECTION_PROB_REAL = 0.85
DETECTION_PROB_FALSE = 0.15
PRUNE_THRESHOLD = 0.35
ARENA_SIZE = 15.0


KG_ROOM_OBJECTS = {
    "office": ["desk", "chair", "monitor", "keyboard", "mouse", "lamp", "cabinet"],
    "kitchen": ["refrigerator", "sink", "microwave", "kettle", "table", "chair"],
    "corridor": ["door", "sign", "fire_extinguisher", "trash_can"],
    "meeting_room": ["table", "chair", "projector", "whiteboard", "screen"],
}

REAL_LABELS = [
    "chair",
    "desk",
    "monitor",
    "door",
    "table",
    "shelf",
    "lamp",
    "plant",
    "cabinet",
    "window",
    "sign",
    "trash_can",
    "sofa",
    "keyboard",
    "mouse",
    "refrigerator",
    "sink",
    "projector",
    "whiteboard",
    "bottle",
]

FALSE_LABELS = [
    "ghost_box",
    "phantom_cone",
    "noise_blob",
    "shadow_artifact",
    "reflection_plate",
    "glitch_sphere",
    "false_pillar",
    "misdet_wall",
    "spurious_rock",
    "hallucinated_bin",
]


# ============================================================

# ============================================================


@dataclass
class SimTrackedObject:
    """Sim Tracked Object."""

    object_id: int
    label: str
    position: np.ndarray
    is_real: bool
    belief_alpha: float = 1.5
    belief_beta: float = 1.0
    detection_count: int = 0
    miss_streak: int = 0
    best_score: float = 0.5
    last_seen: float = 0.0
    kg_prior_alpha: float = 0.0
    is_kg_expected: bool = False
    room_type: str = ""

    @property
    def existence_prob(self) -> float:
        return self.belief_alpha / (self.belief_alpha + self.belief_beta)

    def record_hit(self, score: float = 0.8, current_time: float = 0.0) -> None:
        """Record hit."""
        self.belief_alpha += 1.0
        self.detection_count += 1
        self.miss_streak = 0
        self.best_score = max(self.best_score, score)
        self.last_seen = current_time

    def record_miss(self) -> None:
        """Record miss."""
        self.miss_streak += 1
        self.belief_beta += BELIEF_NEG_EVIDENCE_WEIGHT

    def apply_kg_prior(self, room_type: str) -> None:
        """Apply kg prior."""
        expected = KG_ROOM_OBJECTS.get(room_type, [])
        if self.label in expected:
            self.belief_alpha += BP_KG_PRIOR_BOOST
            self.kg_prior_alpha += BP_KG_PRIOR_BOOST
            self.is_kg_expected = True
        else:
            self.belief_beta += BP_KG_UNEXPECTED_PENALTY
        self.room_type = room_type


def is_in_fov(
    obj_pos: np.ndarray,
    camera_pos: np.ndarray,
    camera_forward: np.ndarray,
) -> bool:
    """Is in fov."""
    diff = obj_pos[:2] - camera_pos[:2]
    dist = np.linalg.norm(diff)
    if dist < 0.1 or dist > FOV_MAX_RANGE:
        return False
    cos_angle = np.dot(diff, camera_forward[:2]) / (dist * max(np.linalg.norm(camera_forward[:2]), 1e-7))
    return cos_angle > math.cos(FOV_HALF_ANGLE)


# ============================================================

# ============================================================


def generate_objects(seed: int = 42) -> list[SimTrackedObject]:
    """Generate objects."""
    rng = random.Random(seed)
    np_rng = np.random.RandomState(seed)
    objects = []

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

        pos = room_center + np_rng.randn(2) * 1.5
        pos = np.clip(pos, 0, ARENA_SIZE)

        if rng.random() < 0.7:
            label = rng.choice(KG_ROOM_OBJECTS[room])
        else:
            label = rng.choice(REAL_LABELS)
        objects.append(
            SimTrackedObject(
                object_id=i,
                label=label,
                position=np.array([pos[0], pos[1], 0.0]),
                is_real=True,
                room_type=room,
            )
        )

    for i in range(NUM_FALSE_OBJECTS):
        pos = np_rng.uniform(0, ARENA_SIZE, size=2)
        label = rng.choice(FALSE_LABELS)
        objects.append(
            SimTrackedObject(
                object_id=NUM_REAL_OBJECTS + i,
                label=label,
                position=np.array([pos[0], pos[1], 0.0]),
                is_real=False,
                room_type=rng.choice(list(KG_ROOM_OBJECTS.keys())),
            )
        )

    return objects


def generate_camera_trajectory(
    num_steps: int,
    seed: int = 123,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Generate camera trajectory."""
    rng = np.random.RandomState(seed)
    trajectory = []

    for step in range(num_steps):
        t = step / num_steps
        x = ARENA_SIZE * (0.1 + 0.8 * t)
        y = ARENA_SIZE * (0.5 + 0.3 * math.sin(2 * math.pi * t * 3))
        pos = np.array([x, y, 0.5])

        angle = 2 * math.pi * t + rng.randn() * 0.3
        forward = np.array([math.cos(angle), math.sin(angle), 0.0])
        trajectory.append((pos, forward))
    return trajectory


# ============================================================

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
    """Run simulation."""
    objects = generate_objects(seed)
    trajectory = generate_camera_trajectory(NUM_STEPS, seed + 1)
    rng = random.Random(seed + 2)

    if mode != AblationMode.NO_KG:
        for obj in objects:
            obj.apply_kg_prior(obj.room_type)

    for step, (cam_pos, cam_forward) in enumerate(trajectory):
        current_time = step * 0.5  # 0.5s per step

        for obj in objects:
            if mode == AblationMode.NO_BELIEF:
                continue

            in_fov = is_in_fov(obj.position, cam_pos, cam_forward)

            if mode == AblationMode.NO_FOV:
                if obj.is_real:
                    detected = rng.random() < DETECTION_PROB_REAL if in_fov else False
                else:
                    detected = rng.random() < DETECTION_PROB_FALSE if in_fov else False

                if detected:
                    score = 0.7 + rng.random() * 0.25 if obj.is_real else 0.3 + rng.random() * 0.3
                    obj.record_hit(score, current_time)
                else:
                    obj.record_miss()
            else:
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

    real_objects = [o for o in objects if o.is_real]
    false_objects = [o for o in objects if not o.is_real]

    if mode == AblationMode.NO_BELIEF:
        real_retained = NUM_REAL_OBJECTS
        false_cleared = 0
    else:
        real_retained = sum(1 for o in real_objects if o.existence_prob >= PRUNE_THRESHOLD)
        false_cleared = sum(1 for o in false_objects if o.existence_prob < PRUNE_THRESHOLD)

    retention_rate = real_retained / max(len(real_objects), 1)
    clearance_rate = false_cleared / max(len(false_objects), 1)

    target_indices = list(range(len(real_objects)))
    rng_target = random.Random(seed + 99)
    rng_target.shuffle(target_indices)
    target_indices = target_indices[:10]

    localization_correct = 0
    for idx in target_indices:
        target = real_objects[idx]
        if mode == AblationMode.NO_BELIEF:
            all_with_label = [o for o in objects if o.label == target.label and o.existence_prob >= PRUNE_THRESHOLD]
        else:
            all_with_label = [o for o in objects if o.label == target.label and o.existence_prob >= PRUNE_THRESHOLD]
        if not all_with_label:
            continue

        best = max(all_with_label, key=lambda o: o.existence_prob)

        if best.is_real:
            dist = np.linalg.norm(best.position[:2] - target.position[:2])
            if dist < 3.0:
                localization_correct += 1

    localization_accuracy = localization_correct / 10.0

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

# ============================================================


class TestBAHSGAblation(unittest.TestCase):
    """Test B A H S G Ablation."""

    @classmethod
    def setUpClass(cls):
        """Set Up Class."""
        cls.results = {}
        modes = [
            AblationMode.FULL,
            AblationMode.NO_BELIEF,
            AblationMode.NO_FOV,
            AblationMode.NO_KG,
        ]

        seeds = [42, 137, 256]
        for mode in modes:
            runs = [run_simulation(mode, seed=s) for s in seeds]
            avg = {}
            for key in runs[0]:
                avg[key] = np.mean([r[key] for r in runs])
            cls.results[mode] = avg

    def test_full_bahsg_retains_real_objects(self):
        """Test full bahsg retains real objects."""
        rate = self.results[AblationMode.FULL]["retention_rate"]
        self.assertGreaterEqual(rate, 0.90, f"Full BA-HSG retention rate {rate:.1%} < 90%")

    def test_full_bahsg_clears_false_positives(self):
        """Test full bahsg clears false positives."""
        rate = self.results[AblationMode.FULL]["clearance_rate"]
        self.assertGreaterEqual(rate, 0.50, f"Full BA-HSG clearance rate {rate:.1%} < 50%")

    def test_no_belief_cannot_clear_false_positives(self):
        """Test no belief cannot clear false positives."""
        rate = self.results[AblationMode.NO_BELIEF]["clearance_rate"]
        self.assertEqual(rate, 0.0, f"No-Belief should have 0% clearance, got {rate:.1%}")

    def test_no_fov_hurts_retention(self):
        """Test no fov hurts retention."""
        full_ret = self.results[AblationMode.FULL]["retention_rate"]
        nofov_ret = self.results[AblationMode.NO_FOV]["retention_rate"]
        self.assertLess(nofov_ret, full_ret, f"No-FOV retention {nofov_ret:.1%} should be < Full {full_ret:.1%}")

    def test_no_fov_higher_clearance(self):
        """Test no fov higher clearance."""
        full_clr = self.results[AblationMode.FULL]["clearance_rate"]
        nofov_clr = self.results[AblationMode.NO_FOV]["clearance_rate"]
        self.assertGreaterEqual(
            nofov_clr, full_clr, f"No-FOV clearance {nofov_clr:.1%} should be >= Full {full_clr:.1%}"
        )

    def test_kg_prior_boosts_real_existence(self):
        """Test kg prior boosts real existence."""
        full_exist = self.results[AblationMode.FULL]["avg_exist_real"]
        nokg_exist = self.results[AblationMode.NO_KG]["avg_exist_real"]
        self.assertGreaterEqual(
            full_exist, nokg_exist, f"Full avg P_real {full_exist:.3f} should be >= No-KG {nokg_exist:.3f}"
        )

    def test_full_separates_real_and_false(self):
        """Test full separates real and false."""
        r = self.results[AblationMode.FULL]
        gap = r["avg_exist_real"] - r["avg_exist_false"]
        self.assertGreater(
            gap,
            0.1,
            f"P(exist) gap between real ({r['avg_exist_real']:.3f}) "
            f"and false ({r['avg_exist_false']:.3f}) = {gap:.3f} < 0.1",
        )

    def test_print_ablation_table(self):
        """Test print ablation table."""
        print("\n" + "=" * 85)
        print("BA-HSG Ablation Study Results")
        print("=" * 85)
        header = (
            f"{'Configuration':<16s} | {'Retention':>10s} | {'Clearance':>10s} | "
            f"{'Localization':>12s} | {'P_real':>7s} | {'P_false':>8s}"
        )
        print(header)
        print("-" * 85)

        for mode in [AblationMode.FULL, AblationMode.NO_BELIEF, AblationMode.NO_FOV, AblationMode.NO_KG]:
            r = self.results[mode]
            print(
                f"{mode:<16s} | {r['retention_rate']:>9.1%} | {r['clearance_rate']:>9.1%} | "
                f"{r['localization_accuracy']:>11.1%} | {r['avg_exist_real']:>6.3f} | "
                f"{r['avg_exist_false']:>7.3f}"
            )

        print("=" * 85)
        print(
            f"Settings: {NUM_REAL_OBJECTS} real + {NUM_FALSE_OBJECTS} false objects, "
            f"{NUM_STEPS} steps, prune_threshold={PRUNE_THRESHOLD}"
        )
        print()

        print("% --- LaTeX Table (copy to paper) ---")
        print(r"\begin{table}[t]")
        print(r"\centering")
        print(r"\caption{BA-HSG Ablation Study: impact of belief components}")
        print(r"\label{tab:ablation}")
        print(r"\begin{tabular}{l c c c}")
        print(r"\toprule")
        print(r"Configuration & Retention$\uparrow$ & FP Clearance$\uparrow$ & Localization$\uparrow$ \\")
        print(r"\midrule")

        for mode in [AblationMode.FULL, AblationMode.NO_BELIEF, AblationMode.NO_FOV, AblationMode.NO_KG]:
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
    """Test Ablation Sanity."""

    def test_object_generation(self):
        """Test object generation."""
        objects = generate_objects(seed=42)
        self.assertEqual(len(objects), NUM_REAL_OBJECTS + NUM_FALSE_OBJECTS)
        real = [o for o in objects if o.is_real]
        false = [o for o in objects if not o.is_real]
        self.assertEqual(len(real), NUM_REAL_OBJECTS)
        self.assertEqual(len(false), NUM_FALSE_OBJECTS)

    def test_camera_trajectory(self):
        """Test camera trajectory."""
        traj = generate_camera_trajectory(NUM_STEPS)
        self.assertEqual(len(traj), NUM_STEPS)
        for pos, fwd in traj:
            self.assertEqual(pos.shape, (3,))
            self.assertEqual(fwd.shape, (3,))
            self.assertGreater(np.linalg.norm(fwd), 0.5)

    def test_fov_check(self):
        """Test fov check."""
        cam_pos = np.array([0.0, 0.0, 0.0])
        cam_fwd = np.array([1.0, 0.0, 0.0])

        self.assertTrue(is_in_fov(np.array([3.0, 0.0, 0.0]), cam_pos, cam_fwd))

        self.assertFalse(is_in_fov(np.array([-3.0, 0.0, 0.0]), cam_pos, cam_fwd))

        self.assertFalse(is_in_fov(np.array([20.0, 0.0, 0.0]), cam_pos, cam_fwd))

        self.assertFalse(is_in_fov(np.array([0.05, 0.0, 0.0]), cam_pos, cam_fwd))

        self.assertFalse(is_in_fov(np.array([0.0, 5.0, 0.0]), cam_pos, cam_fwd))

    def test_belief_update_mechanics(self):
        """Test belief update mechanics."""
        obj = SimTrackedObject(
            object_id=0,
            label="chair",
            position=np.array([1.0, 2.0, 0.0]),
            is_real=True,
        )
        initial_prob = obj.existence_prob

        obj.record_hit(0.9, 1.0)
        self.assertGreater(obj.existence_prob, initial_prob)

        prob_after_hit = obj.existence_prob
        for _ in range(10):
            obj.record_miss()
        self.assertLess(obj.existence_prob, prob_after_hit)

    def test_kg_prior_injection(self):
        """Test kg prior injection."""

        obj_expected = SimTrackedObject(
            object_id=0,
            label="desk",
            position=np.array([1.0, 2.0, 0.0]),
            is_real=True,
        )
        prob_before = obj_expected.existence_prob
        obj_expected.apply_kg_prior("office")
        self.assertGreater(obj_expected.existence_prob, prob_before)
        self.assertTrue(obj_expected.is_kg_expected)

        obj_unexpected = SimTrackedObject(
            object_id=1,
            label="ghost_box",
            position=np.array([1.0, 2.0, 0.0]),
            is_real=False,
        )
        prob_before = obj_unexpected.existence_prob
        obj_unexpected.apply_kg_prior("office")
        self.assertLess(obj_unexpected.existence_prob, prob_before)
        self.assertFalse(obj_unexpected.is_kg_expected)

    def test_single_run_deterministic(self):
        """Test single run deterministic."""
        r1 = run_simulation(AblationMode.FULL, seed=42)
        r2 = run_simulation(AblationMode.FULL, seed=42)
        for key in r1:
            self.assertAlmostEqual(r1[key], r2[key], places=10, msg=f"{key} not deterministic")


if __name__ == "__main__":
    unittest.main(verbosity=2)
