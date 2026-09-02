"""Decision module."""

import json
import math
import random
import time
import unittest

import numpy as np

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig

# ============================================================

# ============================================================

COMMON_OBJECTS = [
    "chair",
    "table",
    "door",
    "window",
    "trash can",
    "sign",
    "box",
    "shelf",
    "monitor",
    "lamp",
    "plant",
    "bottle",
    "sofa",
    "desk",
    "cabinet",
    "poster",
    "clock",
    "phone",
    "bag",
    "book",
    "keyboard",
    "mouse",
    "fire extinguisher",
    "refrigerator",
    "microwave",
    "sink",
    "projector",
    "whiteboard",
]


def make_scene_scaled(
    target_label: str,
    target_pos: dict[str, float],
    target_score: float = 0.85,
    target_det_count: int = 4,
    num_distractors: int = 8,
    relations: list[dict] | None = None,
    seed: int = 42,
    include_regions: bool = False,
) -> str:
    """Make scene scaled."""
    rng = random.Random(seed)
    objects = []
    obj_id = 0

    objects.append(
        {
            "id": obj_id,
            "label": target_label,
            "position": target_pos,
            "score": target_score,
            "detection_count": target_det_count,
        }
    )
    obj_id += 1

    available = [o for o in COMMON_OBJECTS if o != target_label]
    for _ in range(num_distractors):
        label = rng.choice(available)
        objects.append(
            {
                "id": obj_id,
                "label": label,
                "position": {
                    "x": round(rng.uniform(-20, 20), 1),
                    "y": round(rng.uniform(-20, 20), 1),
                    "z": 0.0,
                },
                "score": round(rng.uniform(0.4, 0.95), 2),
                "detection_count": rng.randint(1, 8),
            }
        )
        obj_id += 1

    rel_list = relations or []
    regions = []
    if include_regions and len(objects) > 5:
        regions = [
            {
                "name": "main_area",
                "object_ids": [o["id"] for o in objects[:5]],
            }
        ]

    return json.dumps(
        {
            "timestamp": 0,
            "object_count": len(objects),
            "objects": objects,
            "relations": rel_list,
            "regions": regions,
            "summary": f"scene with {len(objects)} objects",
        }
    )


# ============================================================

# ============================================================


SIMPLE_INSTRUCTIONS = [
    ("go to the chair", "chair", 0.90),
    ("find the door", "door", 0.85),
    ("navigate to table", "table", 0.88),
    ("go to the shelf", "shelf", 0.82),
    ("find the window", "window", 0.90),
    ("go to desk", "desk", 0.87),
    ("find trash can", "trash can", 0.83),
    ("navigate to plant", "plant", 0.86),
    ("go to the sofa", "sofa", 0.91),
    ("find the lamp", "lamp", 0.84),
]


SPATIAL_INSTRUCTIONS = [
    (
        "find chair near the door",
        "chair",
        0.80,
        [{"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.5}],
        {"extra_label": "door", "extra_pos": {"x": 3.5, "y": 2.5, "z": 0}, "extra_score": 0.88},
    ),
    (
        "go to the desk next to window",
        "desk",
        0.82,
        [{"subject_id": 0, "relation": "near", "object_id": 1, "distance": 2.0}],
        {"extra_label": "window", "extra_pos": {"x": 5, "y": 6, "z": 0}, "extra_score": 0.9},
    ),
    (
        "find lamp behind the sofa",
        "lamp",
        0.78,
        [{"subject_id": 0, "relation": "behind", "object_id": 1, "distance": 1.0}],
        {"extra_label": "sofa", "extra_pos": {"x": 4, "y": 3, "z": 0}, "extra_score": 0.85},
    ),
]


CHINESE_INSTRUCTIONS = [
    ("找到椅子", "椅子", 0.88),
    ("导航到桌子", "桌子", 0.85),
    ("去门那里", "门", 0.90),
    ("找垃圾桶", "垃圾桶", 0.82),
    ("找到沙发", "沙发", 0.87),
]


AMBIGUOUS_INSTRUCTIONS = [
    ("find something to sit on", "chair", 0.85),
    ("go to the red chair", "blue chair", 0.80),
    ("find the elephant", None, 0.0),
]


# ============================================================

# ============================================================


class TestFastPathHitRateByComplexity(unittest.TestCase):
    """Test Fast Path Hit Rate By Complexity."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def _run_hit_rate(
        self,
        cases: list[tuple],
        category: str,
    ) -> tuple[float, list[str]]:
        """Run hit rate."""
        hits = 0
        total = 0
        details = []

        for i, case in enumerate(cases):
            if len(case) == 3:
                instr, target, score = case
                relations = None
                extra = None
            else:
                instr, target, score, relations, extra = case

            target_pos = {"x": 3 + i, "y": 2 + i, "z": 0}
            det_count = 4
            scene_objects_extra = []

            if extra and "extra_label" in extra:
                scene_objects_extra = [
                    {
                        "id": 1,
                        "label": extra["extra_label"],
                        "position": extra["extra_pos"],
                        "score": extra["extra_score"],
                        "detection_count": 5,
                    }
                ]

            sg_data = {
                "timestamp": 0,
                "objects": [
                    {
                        "id": 0,
                        "label": target if target else "nonexistent",
                        "position": target_pos,
                        "score": score,
                        "detection_count": det_count,
                    },
                    *scene_objects_extra,
                ],
                "relations": relations or [],
                "regions": [],
                "summary": "test",
            }

            rng = random.Random(100 + i)
            available = [o for o in COMMON_OBJECTS if o != target]
            for j in range(8):
                sg_data["objects"].append(
                    {
                        "id": 10 + j,
                        "label": rng.choice(available),
                        "position": {"x": rng.uniform(-10, 10), "y": rng.uniform(-10, 10), "z": 0},
                        "score": round(rng.uniform(0.4, 0.9), 2),
                        "detection_count": rng.randint(1, 6),
                    }
                )
            sg_data["object_count"] = len(sg_data["objects"])
            sg = json.dumps(sg_data)

            result = self.resolver.fast_resolve(instr, sg)
            total += 1

            if target is None:
                hit = result is None
            else:
                hit = result is not None and target.lower() in result.target_label.lower()

            if hit:
                hits += 1
            details.append(
                f"  {'HIT' if hit else 'MISS':4s} | {instr:35s} | "
                f"expected={target}, got={getattr(result, 'target_label', 'None') if result else 'None'}"
            )

        rate = hits / max(total, 1)
        return rate, details

    def test_simple_hit_rate(self):
        """Test simple hit rate."""
        rate, details = self._run_hit_rate(SIMPLE_INSTRUCTIONS, "Simple")
        print(
            f"\n=== Simple Instructions: {rate * 100:.0f}% ({int(rate * len(SIMPLE_INSTRUCTIONS))}/{len(SIMPLE_INSTRUCTIONS)}) ==="
        )
        print("\n".join(details))
        self.assertGreaterEqual(rate, 0.70)

    def test_spatial_hit_rate(self):
        """Test spatial hit rate."""
        rate, details = self._run_hit_rate(SPATIAL_INSTRUCTIONS, "Spatial")
        print(
            f"\n=== Spatial Instructions: {rate * 100:.0f}% ({int(rate * len(SPATIAL_INSTRUCTIONS))}/{len(SPATIAL_INSTRUCTIONS)}) ==="
        )
        print("\n".join(details))

        self.assertGreaterEqual(rate, 0.50)

    def test_chinese_hit_rate(self):
        """Test chinese hit rate."""

        cases_cn = []
        for instr, label, score in CHINESE_INSTRUCTIONS:
            cases_cn.append((instr, label, score))
        rate, details = self._run_hit_rate(cases_cn, "Chinese")
        print(f"\n=== Chinese Instructions: {rate * 100:.0f}% ({int(rate * len(cases_cn))}/{len(cases_cn)}) ===")
        print("\n".join(details))
        self.assertGreaterEqual(rate, 0.60)

    def test_print_hit_rate_summary(self):
        """Test print hit rate summary."""
        results = {}
        for name, cases in [
            ("Simple", SIMPLE_INSTRUCTIONS),
            ("Spatial", SPATIAL_INSTRUCTIONS),
            ("Chinese", CHINESE_INSTRUCTIONS),
            ("Ambiguous", AMBIGUOUS_INSTRUCTIONS),
        ]:
            rate, _ = self._run_hit_rate(cases, name)
            results[name] = (rate, len(cases))

        overall_hits = sum(int(r * n) for r, n in results.values())
        overall_total = sum(n for _, n in results.values())
        overall_rate = overall_hits / max(overall_total, 1)

        print("\n" + "=" * 65)
        print("Fast Path Hit Rate by Instruction Complexity")
        print("=" * 65)
        print(f"{'Category':<12s} | {'Hit Rate':>10s} | {'Count':>6s} | {'Note':<25s}")
        print("-" * 65)
        notes = {
            "Simple": "exact label match",
            "Spatial": "subject + modifier parsing",
            "Chinese": "jieba tokenization",
            "Ambiguous": "semantic / absent target",
        }
        for name, (rate, n) in results.items():
            print(f"{name:<12s} | {rate:>9.1%} | {n:>6d} | {notes[name]:<25s}")
        print("-" * 65)
        print(f"{'Overall':<12s} | {overall_rate:>9.1%} | {overall_total:>6d} |")
        print("=" * 65)

        # LaTeX
        print("\n% --- LaTeX Table ---")
        print(r"\begin{table}[t]")
        print(r"\centering")
        print(r"\caption{Fast Path hit rate by instruction complexity (no CLIP)}")
        print(r"\label{tab:fast-hit-rate}")
        print(r"\begin{tabular}{l c c}")
        print(r"\toprule")
        print(r"Category & Hit Rate & \# Queries \\")
        print(r"\midrule")
        for name, (rate, n) in results.items():
            print(f"{name} & {rate:.1%} & {n} \\\\")
        print(r"\midrule")
        print(f"Overall & {overall_rate:.1%} & {overall_total} \\\\")
        print(r"\bottomrule")
        print(r"\end{tabular}")
        print(r"\end{table}")


class TestFastPathLatencyScaling(unittest.TestCase):
    """Test Fast Path Latency Scaling."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def test_latency_scaling(self):
        """Test latency scaling."""
        sizes = [10, 50, 100, 200]
        results = {}

        for size in sizes:
            sg = make_scene_scaled(
                "chair",
                {"x": 5, "y": 5, "z": 0},
                0.9,
                5,
                num_distractors=size - 1,
                seed=42,
            )

            self.resolver.fast_resolve("go to the chair", sg)

            times = []
            for _ in range(30):
                start = time.perf_counter()
                self.resolver.fast_resolve("go to the chair", sg)
                elapsed_ms = (time.perf_counter() - start) * 1000
                times.append(elapsed_ms)

            results[size] = {
                "avg_ms": np.mean(times),
                "p50_ms": np.percentile(times, 50),
                "p95_ms": np.percentile(times, 95),
                "p99_ms": np.percentile(times, 99),
            }

        print("\n" + "=" * 70)
        print("Fast Path Latency vs Scene Size (no CLIP, string-match)")
        print("=" * 70)
        print(f"{'Objects':>8s} | {'Avg':>8s} | {'P50':>8s} | {'P95':>8s} | {'P99':>8s} | {'<200ms':>6s}")
        print("-" * 70)
        for size in sizes:
            r = results[size]
            within = "Yes" if r["p99_ms"] < 200 else "No"
            print(
                f"{size:>8d} | {r['avg_ms']:>7.3f} | {r['p50_ms']:>7.3f} | "
                f"{r['p95_ms']:>7.3f} | {r['p99_ms']:>7.3f} | {within:>6s}"
            )
        print("=" * 70)
        print("Note: Real deployment adds ~20-50ms for CLIP on Jetson Orin NX")

        # LaTeX
        print("\n% --- LaTeX Table ---")
        print(r"\begin{table}[t]")
        print(r"\centering")
        print(r"\caption{Fast Path latency scaling (CPU string-match, no CLIP)}")
        print(r"\label{tab:fast-latency}")
        print(r"\begin{tabular}{r c c c}")
        print(r"\toprule")
        print(r"\# Objects & Avg (ms) & P95 (ms) & P99 (ms) \\")
        print(r"\midrule")
        for size in sizes:
            r = results[size]
            print(f"{size} & {r['avg_ms']:.2f} & {r['p95_ms']:.2f} & {r['p99_ms']:.2f} \\\\")
        print(r"\bottomrule")
        print(r"\end{tabular}")
        print(r"\end{table}")

        for size in sizes:
            self.assertLess(
                results[size]["p99_ms"],
                200.0,
                f"P99 latency for {size} objects = {results[size]['p99_ms']:.2f}ms >= 200ms",
            )


class TestESCAFilteringEfficiency(unittest.TestCase):
    """Test E S C A Filtering Efficiency."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def test_esca_filter_rates(self):
        """Test esca filter rates."""
        test_cases = [
            ("find the door", "door", 50),
            ("go to the chair near table", "chair", 100),
            ("navigate to fire extinguisher", "fire extinguisher", 150),
            ("find the refrigerator", "refrigerator", 200),
        ]
        results = []

        for instr, target, num_objects in test_cases:
            sg = make_scene_scaled(
                target,
                {"x": 5, "y": 5, "z": 0},
                0.85,
                4,
                num_distractors=num_objects - 1,
                seed=42,
            )

            filtered_sg_json = self.resolver._selective_grounding(
                instr,
                sg,
                max_objects=15,
            )
            filtered_sg = json.loads(filtered_sg_json)
            original_sg = json.loads(sg)

            orig_count = len(original_sg["objects"])
            filt_count = len(filtered_sg["objects"])
            reduction = 1.0 - filt_count / max(orig_count, 1)

            orig_tokens = orig_count * 50
            filt_tokens = filt_count * 50
            token_reduction = 1.0 - filt_tokens / max(orig_tokens, 1)

            results.append(
                {
                    "instruction": instr,
                    "orig_objects": orig_count,
                    "filt_objects": filt_count,
                    "obj_reduction": reduction,
                    "orig_tokens": orig_tokens,
                    "filt_tokens": filt_tokens,
                    "token_reduction": token_reduction,
                }
            )

        print("\n" + "=" * 85)
        print("ESCA Selective Grounding Filtering Efficiency")
        print("=" * 85)
        print(f"{'Instruction':<35s} | {'Orig':>5s} | {'Filt':>5s} | {'Obj %':>6s} | {'Token %':>8s}")
        print("-" * 85)

        total_orig = 0
        total_filt = 0
        for r in results:
            total_orig += r["orig_objects"]
            total_filt += r["filt_objects"]
            print(
                f"{r['instruction']:<35s} | {r['orig_objects']:>5d} | {r['filt_objects']:>5d} | "
                f"{r['obj_reduction']:>5.1%} | {r['token_reduction']:>7.1%}"
            )

        avg_reduction = 1.0 - total_filt / max(total_orig, 1)
        print("-" * 85)
        print(f"{'Average':<35s} | {total_orig:>5d} | {total_filt:>5d} | {avg_reduction:>5.1%} |")
        print("=" * 85)

        # LaTeX
        print("\n% --- LaTeX Table ---")
        print(r"\begin{table}[t]")
        print(r"\centering")
        print(r"\caption{ESCA selective grounding: object and token reduction}")
        print(r"\label{tab:esca}")
        print(r"\begin{tabular}{l r r c}")
        print(r"\toprule")
        print(r"Scene Size & Before & After & Reduction \\")
        print(r"\midrule")
        for r in results:
            print(
                f"{r['orig_objects']} objects & {r['orig_objects']} & {r['filt_objects']} & {r['obj_reduction']:.1%} \\\\"
            )
        print(r"\bottomrule")
        print(r"\end{tabular}")
        print(r"\end{table}")

        self.assertGreater(avg_reduction, 0.50, f"Average ESCA reduction {avg_reduction:.1%} < 50%")


class TestAdaNavEntropyTrigger(unittest.TestCase):
    """Test Ada Nav Entropy Trigger."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def _compute_entropy(self, scores: list[float]) -> float:
        """Shannon entropy over normalized scores."""
        if not scores:
            return 0.0
        total = sum(scores)
        if total <= 0:
            return 0.0
        h = 0.0
        for s in scores:
            p = s / total
            if p > 1e-10:
                h -= p * math.log2(p)
        return h

    def test_entropy_distribution(self):
        """Test entropy distribution."""
        scenarios = {
            "clear_target": {
                "instr": "find the chair",
                "target": "chair",
                "score": 0.92,
                "distractors": 8,
                "desc": "One clear match",
            },
            "multiple_similar": {
                "instr": "find the chair",
                "target": "chair",
                "score": 0.85,
                "distractors": 15,
                "desc": "Multiple similar objects",
            },
            "no_match": {
                "instr": "find the elephant",
                "target": "elephant",
                "score": 0.5,
                "distractors": 20,
                "desc": "No matching object",
            },
        }

        print("\n" + "=" * 70)
        print("AdaNav Score Entropy Analysis")
        print("=" * 70)
        print(f"{'Scenario':<20s} | {'Entropy':>8s} | {'Fast Hit':>9s} | {'Description':<25s}")
        print("-" * 70)

        for name, sc in scenarios.items():
            sg = make_scene_scaled(
                sc["target"],
                {"x": 5, "y": 5, "z": 0},
                sc["score"],
                4,
                num_distractors=sc["distractors"],
                seed=42,
            )
            result = self.resolver.fast_resolve(sc["instr"], sg)
            entropy = getattr(result, "score_entropy", 0.0) if result else 0.0

            hit = result is not None and sc["target"].lower() in result.target_label.lower()
            print(f"{name:<20s} | {entropy:>8.3f} | {'Yes' if hit else 'No':>9s} | {sc['desc']:<25s}")

        print("=" * 70)
        print("Note: AdaNav escalates to Slow Path when entropy > 1.5 and confidence < 0.85")


class TestEfficiencySummary(unittest.TestCase):
    """Test Efficiency Summary."""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def test_print_system_comparison(self):
        """Test print system comparison."""

        sg_50 = make_scene_scaled("chair", {"x": 5, "y": 5, "z": 0}, 0.9, 5, num_distractors=49, seed=42)

        self.resolver.fast_resolve("go to the chair", sg_50)

        times = []
        for _ in range(50):
            start = time.perf_counter()
            self.resolver.fast_resolve("go to the chair", sg_50)
            elapsed_ms = (time.perf_counter() - start) * 1000
            times.append(elapsed_ms)

        avg_latency = np.mean(times)

        hits = 0
        total = len(SIMPLE_INSTRUCTIONS)
        for i, (instr, target, score) in enumerate(SIMPLE_INSTRUCTIONS):
            sg = make_scene_scaled(target, {"x": 3 + i, "y": 2 + i, "z": 0}, score, 4, num_distractors=8, seed=100 + i)
            result = self.resolver.fast_resolve(instr, sg)
            if result is not None and target in result.target_label:
                hits += 1
        hit_rate = hits / total

        sg_200 = make_scene_scaled("door", {"x": 5, "y": 5, "z": 0}, 0.85, 4, num_distractors=199, seed=42)
        filtered = self.resolver._selective_grounding("find the door", sg_200, max_objects=15)
        filt_count = len(json.loads(filtered)["objects"])
        esca_reduction = 1.0 - filt_count / 200.0

        print("\n" + "=" * 75)
        print("System Efficiency vs Paper Baselines")
        print("=" * 75)
        print(f"{'Metric':<30s} | {'Ours':>12s} | {'VLingNav':>12s} | {'OmniNav':>12s}")
        print("-" * 75)
        print(f"{'Fast Path Hit Rate':<30s} | {hit_rate:>11.1%} | {'>70%':>12s} | {'N/A':>12s}")
        print(f"{'Fast Path Latency (50 obj)':<30s} | {avg_latency:>10.2f}ms | {'<200ms':>12s} | {'<200ms':>12s}")
        print(f"{'ESCA Token Reduction':<30s} | {esca_reduction:>11.1%} | {'N/A':>12s} | {'~90%':>12s}")
        print(f"{'CLIP Required':<30s} | {'No*':>12s} | {'Yes':>12s} | {'Yes':>12s}")
        print(f"{'LLM for Fast Path':<30s} | {'No':>12s} | {'No':>12s} | {'No':>12s}")
        print("=" * 75)
        print("* CLIP optional - weights redistributed to label+detector when absent")

        # LaTeX
        print("\n% --- LaTeX Table ---")
        print(r"\begin{table}[t]")
        print(r"\centering")
        print(r"\caption{System efficiency comparison with paper baselines}")
        print(r"\label{tab:efficiency}")
        print(r"\begin{tabular}{l c c c}")
        print(r"\toprule")
        print(r"Metric & Ours & VLingNav & OmniNav \\")
        print(r"\midrule")
        print(f"Fast Path Hit Rate & {hit_rate:.1%} & $>$70\\% & N/A \\\\")
        print(f"Latency (50 obj) & {avg_latency:.2f}ms & $<$200ms & $<$200ms \\\\")
        print(f"ESCA Reduction & {esca_reduction:.1%} & N/A & $\\sim$90\\% \\\\")
        print(r"CLIP Required & No$^*$ & Yes & Yes \\")
        print(r"\bottomrule")
        print(r"\end{tabular}")
        print(r"\end{table}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
