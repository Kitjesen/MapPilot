"""
test_fast_slow_benchmark.py \u2014 Fast-Slow \u53cc\u8fdb\u7a0b\u8bda\u5b9e\u6027\u80fd\u57fa\u51c6\u6d4b\u8bd5

\u6838\u5fc3\u539f\u5219:
  1. \u6d4b\u8bd5\u573a\u666f\u5fc5\u987b\u5305\u542b\u5e72\u6270\u7269\u4f53 (\u4e0d\u662f\u53ea\u6709\u76ee\u6807)
  2. \u4e0d\u4f7f\u7528\u4f2a\u9020\u7684 CLIP \u5206\u6570
  3. \u4e0d\u8bbe\u8ba1 "\u5fc5\u5b9a\u901a\u8fc7" \u7684\u573a\u666f\uff0c\u800c\u662f\u771f\u5b9e\u6d4b\u91cf\u771f\u5b9e\u80fd\u529b
  4. \u8bb0\u5f55\u5931\u8d25\u6848\u4f8b\u4ee5\u8bc6\u522b\u7cfb\u7edf\u77ed\u677f
  5. \u5bf9\u7167\u8bba\u6587\u6307\u6807\u7ed9\u51fa\u8bda\u5b9e\u7684\u5dee\u8ddd\u5206\u6790

\u53c2\u8003\u8bba\u6587:
  - VLingNav (2026): Fast Path \u547d\u4e2d\u7387 70%+ (\u5728\u771f\u5b9e\u73af\u5883 CLIP \u5168\u529f\u80fd\u4e0b)
  - AdaNav (ICLR 2026): \u591a\u6e90\u878d\u5408\u51c6\u786e\u7387\u63d0\u5347 15-20%
  - ESCA (NeurIPS 2025): Token \u51cf\u5c11 90%
"""

import json
import random
import time
import unittest

import numpy as np

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig

# ============================================================
#  \u573a\u666f\u6784\u5efa\u5de5\u5177
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
]


def make_scene(
    target_label: str,
    target_pos: dict[str, float],
    target_score: float = 0.85,
    target_det_count: int = 4,
    num_distractors: int = 8,
    relations: list[dict] | None = None,
    seed: int = 42,
) -> str:
    """
    \u6784\u5efa\u5305\u542b\u76ee\u6807+\u5e72\u6270\u7269\u4f53\u7684\u573a\u666f\u56fe\u3002

    \u4e0e\u65e7\u7248\u6d4b\u8bd5\u7684\u5173\u952e\u5dee\u522b: \u573a\u666f\u4e2d\u603b\u662f\u6709\u591a\u4e2a\u65e0\u5173\u7269\u4f53\u5e72\u6270\u3002
    """
    rng = random.Random(seed)
    objects = []
    obj_id = 0

    # \u76ee\u6807\u7269\u4f53
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

    # \u5e72\u6270\u7269\u4f53 (\u968f\u673a\u4f4d\u7f6e\u548c\u5206\u6570)
    available = [o for o in COMMON_OBJECTS if o != target_label]
    for _ in range(num_distractors):
        label = rng.choice(available)
        objects.append(
            {
                "id": obj_id,
                "label": label,
                "position": {
                    "x": round(rng.uniform(-10, 10), 1),
                    "y": round(rng.uniform(-10, 10), 1),
                    "z": 0.0,
                },
                "score": round(rng.uniform(0.4, 0.95), 2),
                "detection_count": rng.randint(1, 8),
            }
        )
        obj_id += 1

    return json.dumps(
        {
            "timestamp": 0,
            "object_count": len(objects),
            "objects": objects,
            "relations": relations or [],
            "regions": [],
            "summary": "benchmark scene",
        }
    )


# ============================================================
#  \u6838\u5fc3\u6d4b\u8bd5\u5957\u4ef6
# ============================================================


class TestFastPathHitRate(unittest.TestCase):
    """
    Fast Path \u547d\u4e2d\u7387\u6d4b\u8bd5 \u2014 \u5305\u542b\u5e72\u6270\u573a\u666f\u3002

    \u8bba\u6587\u76ee\u6807: VLingNav 70%+ (\u4f46\u662f\u8bba\u6587\u6709\u771f\u5b9e CLIP, \u6211\u4eec\u6ca1\u6709)
    \u8bda\u5b9e\u9884\u671f: \u65e0 CLIP \u65f6, \u7b80\u5355\u6307\u4ee4 \u2248 70-80%, \u590d\u6742\u6307\u4ee4\u4f1a\u4f4e\u4e8e\u6b64
    """

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def test_simple_instructions_with_distractors(self):
        """
        \u7b80\u5355\u6307\u4ee4 + \u5e72\u6270\u573a\u666f: \u76ee\u6807\u5728\u573a\u666f\u4e2d\u4e14\u7cbe\u786e\u5339\u914d\u3002

        \u671f\u671b: \u5927\u591a\u6570\u80fd\u547d\u4e2d (>= 70%)
        """
        cases: list[tuple[str, str, dict, float, int]] = [
            # (instruction, target_label, target_pos, target_score, det_count)
            ("go to the chair", "chair", {"x": 3, "y": 2, "z": 0}, 0.9, 5),
            ("find the door", "door", {"x": 5, "y": 5, "z": 0}, 0.85, 4),
            ("navigate to table", "table", {"x": 2, "y": 1, "z": 0}, 0.88, 6),
            ("go to the shelf", "shelf", {"x": 4, "y": 3, "z": 0}, 0.82, 3),
            ("find the window", "window", {"x": 7, "y": 2, "z": 0}, 0.9, 5),
            ("go to desk", "desk", {"x": 1, "y": 4, "z": 0}, 0.87, 4),
            ("find trash can", "trash can", {"x": 6, "y": 6, "z": 0}, 0.83, 3),
            ("navigate to plant", "plant", {"x": 3, "y": 5, "z": 0}, 0.86, 5),
            ("go to the sofa", "sofa", {"x": 8, "y": 1, "z": 0}, 0.91, 6),
            ("find the lamp", "lamp", {"x": 2, "y": 7, "z": 0}, 0.84, 4),
        ]

        hits = 0
        details = []
        for i, (instr, label, pos, score, det) in enumerate(cases):
            sg = make_scene(label, pos, score, det, num_distractors=8, seed=100 + i)
            result = self.resolver.fast_resolve(instr, sg)
            hit = result is not None and result.target_label == label
            if hit:
                hits += 1
            details.append(
                f"  {'OK' if hit else 'MISS':4s} | {instr:30s} | "
                f"expected={label}, got={getattr(result, 'target_label', 'None')}"
            )

        rate = hits / len(cases)
        report = "\n".join(details)
        print(f"\n=== Simple Fast Path Hit Rate: {rate * 100:.0f}% ({hits}/{len(cases)}) ===")
        print(report)

        # \u65e0 CLIP \u65f6\u7b80\u5355\u6307\u4ee4\u5e94\u8fbe 70%
        self.assertGreaterEqual(rate, 0.70, f"Simple instruction hit rate {rate * 100:.0f}% < 70%. Details:\n{report}")

    def test_target_absent_should_defer(self):
        """
        \u76ee\u6807\u4e0d\u5728\u573a\u666f\u4e2d \u2192 \u5e94\u8fd4\u56de None (defer to Slow Path)\u3002
        """
        # \u573a\u666f\u4e2d\u6ca1\u6709 elephant, \u53ea\u6709\u666e\u901a\u7269\u4f53
        sg = make_scene("chair", {"x": 3, "y": 2, "z": 0}, 0.9, 5, num_distractors=8, seed=200)
        result = self.resolver.fast_resolve("find the elephant near the stairs", sg)
        self.assertIsNone(result, "Target 'elephant' not in scene, should defer to Slow Path")

    def test_attribute_mismatch_known_limitation(self):
        """
        \u5c5e\u6027\u4e0d\u5339\u914d (red chair vs blue chair) \u2014 \u5df2\u77e5\u5c40\u9650\u3002

        \u5f53\u524d\u7cfb\u7edf\u65e0\u6cd5\u533a\u5206\u989c\u8272\u5c5e\u6027: "find red chair" \u4f1a\u5339\u914d "blue chair"
        \u56e0\u4e3a "chair" \u5b50\u4e32\u5339\u914d\u3002\u8fd9\u662f\u65e0 CLIP \u7684\u5df2\u77e5\u5c40\u9650\u3002
        """
        sg = make_scene("blue chair", {"x": 3, "y": 2, "z": 0}, 0.9, 5, num_distractors=5, seed=200)
        result = self.resolver.fast_resolve("find the red chair", sg)
        # \u8bb0\u5f55\u8fd9\u4e2a\u5df2\u77e5\u5c40\u9650 \u2014 \u4e0d\u505a\u65ad\u8a00, \u53ea\u8bb0\u5f55\u884c\u4e3a
        if result is not None:
            print(
                f"  KNOWN LIMITATION: 'find red chair' matched '{result.target_label}' (attribute mismatch, needs CLIP)"
            )
        else:
            print("  OK: correctly deferred to Slow Path")


class TestSpatialReasoning(unittest.TestCase):
    """
    \u7a7a\u95f4\u5173\u7cfb\u63a8\u7406\u6d4b\u8bd5 \u2014 \u6838\u5fc3 bug fix \u9a8c\u8bc1\u3002

    \u4e4b\u524d\u7684 bug: "find chair near the door" \u4f1a\u9009\u62e9 door \u800c\u4e0d\u662f chair\u3002
    \u4fee\u590d: _parse_instruction_roles \u533a\u5206\u4e3b\u8bed (chair) \u548c\u4fee\u9970\u8bed (door)\u3002
    """

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.5)

    def test_chair_near_door_selects_chair(self):
        """'find chair near the door' \u5e94\u9009\u62e9 chair, \u4e0d\u662f door\u3002"""
        sg_data = {
            "timestamp": 0,
            "object_count": 3,
            "objects": [
                {"id": 0, "label": "chair", "position": {"x": 3, "y": 2, "z": 0}, "score": 0.8, "detection_count": 3},
                {"id": 1, "label": "door", "position": {"x": 3, "y": 3, "z": 0}, "score": 0.9, "detection_count": 5},
                {
                    "id": 2,
                    "label": "table",
                    "position": {"x": 10, "y": 10, "z": 0},
                    "score": 0.85,
                    "detection_count": 4,
                },
            ],
            "relations": [
                {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
            ],
            "regions": [],
            "summary": "test",
        }
        sg = json.dumps(sg_data)

        result = self.resolver.fast_resolve("find chair near the door", sg)
        self.assertIsNotNone(result)
        self.assertEqual(
            result.target_label,
            "chair",
            f"Should select 'chair' (subject) not 'door' (modifier). Got: {result.target_label}",
        )

    def test_chair_near_door_chair_has_higher_score(self):
        """
        \u7a7a\u95f4\u5173\u7cfb\u5e94\u8ba9 chair \u5f97\u5206\u66f4\u9ad8\u3002
        chair \u5e94\u83b7\u5f97: label=1.0 (subject) + spatial=1.0 (door in instruction)
        door \u5e94\u83b7\u5f97: label=0.3 (modifier) + spatial=0.3 (near relation)
        """
        sg_data = {
            "timestamp": 0,
            "object_count": 2,
            "objects": [
                {
                    "id": 0,
                    "label": "chair",
                    "position": {"x": 3, "y": 2, "z": 0},
                    "score": 0.7,
                    "detection_count": 2,
                },  # \u6545\u610f\u7ed9 chair \u66f4\u4f4e\u7684\u68c0\u6d4b\u5206
                {
                    "id": 1,
                    "label": "door",
                    "position": {"x": 3, "y": 3, "z": 0},
                    "score": 0.95,
                    "detection_count": 8,
                },  # door \u68c0\u6d4b\u5206\u8fdc\u9ad8\u4e8e chair
            ],
            "relations": [
                {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
            ],
            "regions": [],
            "summary": "test",
        }
        sg = json.dumps(sg_data)

        result = self.resolver.fast_resolve("find chair near the door", sg)
        self.assertIsNotNone(result)
        self.assertEqual(
            result.target_label,
            "chair",
            f"Even with lower detection score, subject 'chair' should win over modifier 'door'. "
            f"Got: {result.target_label}",
        )

    def test_chinese_spatial_relation(self):
        """\u4e2d\u6587\u7a7a\u95f4\u6307\u4ee4: '\u627e\u95e8\u65c1\u8fb9\u7684\u6905\u5b50'\u3002"""
        sg_data = {
            "timestamp": 0,
            "object_count": 3,
            "objects": [
                {
                    "id": 0,
                    "label": "\u6905\u5b50",
                    "position": {"x": 3, "y": 2, "z": 0},
                    "score": 0.8,
                    "detection_count": 3,
                },
                {"id": 1, "label": "\u95e8", "position": {"x": 3, "y": 3, "z": 0}, "score": 0.9, "detection_count": 5},
                {
                    "id": 2,
                    "label": "\u684c\u5b50",
                    "position": {"x": 8, "y": 8, "z": 0},
                    "score": 0.85,
                    "detection_count": 4,
                },
            ],
            "relations": [
                {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
            ],
            "regions": [],
            "summary": "test",
        }
        sg = json.dumps(sg_data, ensure_ascii=False)

        result = self.resolver.fast_resolve("\u627e\u95e8\u65c1\u8fb9\u7684\u6905\u5b50", sg)
        self.assertIsNotNone(result)
        self.assertEqual(
            result.target_label,
            "\u6905\u5b50",
            f"Chinese spatial instruction should select '\u6905\u5b50' not '\u95e8'. Got: {result.target_label}",
        )


class TestResponseTime(unittest.TestCase):
    """
    \u54cd\u5e94\u65f6\u95f4\u6d4b\u8bd5 \u2014 \u5bf9\u7167\u8bba\u6587\u76ee\u6807\u3002

    VLingNav: Fast Path \u5e94\u5728 <200ms (\u542b\u573a\u666f\u56fe\u89e3\u6790)
    OmniNav: Fast \u6a21\u5757 5 Hz = 200ms/\u5e27

    \u8bda\u5b9e\u5206\u6790:
    - \u6211\u4eec\u7684 Fast Path \u662f\u7eaf CPU \u5b57\u7b26\u4e32\u5339\u914d, \u8ba1\u7b97\u91cf\u6781\u4f4e
    - \u8bba\u6587\u7684 Fast Path \u5305\u542b\u771f\u5b9e CLIP \u63a8\u7406 (\u5360\u4e3b\u8981\u8017\u65f6)
    - \u56e0\u6b64\u6211\u4eec\u7684 <1ms \u4e0d\u7b49\u4e8e\u8bba\u6587\u7684 <200ms
    """

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.75)

    def test_fast_path_latency(self):
        """
        \u6d4b\u91cf Fast Path \u7684\u7eaf\u8ba1\u7b97\u8017\u65f6 (\u4e0d\u542b CLIP \u63a8\u7406)\u3002

        \u7ed3\u679c\u5e94\u6ce8\u660e: \u8fd9\u662f\u65e0 CLIP \u7684\u5b57\u7b26\u4e32\u5339\u914d\u8017\u65f6,
        \u5b9e\u9645\u90e8\u7f72\u65f6\u52a0\u4e0a CLIP \u63a8\u7406\u4f1a\u663e\u8457\u589e\u52a0\u3002
        """
        # 50 \u7269\u4f53\u573a\u666f
        sg = make_scene("chair", {"x": 10, "y": 10, "z": 0}, 0.9, 5, num_distractors=49, seed=42)

        # \u9884\u70ed
        self.resolver.fast_resolve("go to chair", sg)

        times = []
        for _ in range(50):
            start = time.perf_counter()
            self.resolver.fast_resolve("go to chair", sg)
            elapsed_ms = (time.perf_counter() - start) * 1000
            times.append(elapsed_ms)

        avg = np.mean(times)
        p50 = np.percentile(times, 50)
        p99 = np.percentile(times, 99)

        print("\n=== Fast Path Latency (no CLIP, string-match only) ===")
        print(f"  Avg:  {avg:.3f} ms")
        print(f"  P50:  {p50:.3f} ms")
        print(f"  P99:  {p99:.3f} ms")
        print("  NOTE: Real deployment with CLIP inference will add ~20-50ms on Jetson")

        # \u5b57\u7b26\u4e32\u5339\u914d\u5e94\u5728 <10ms
        self.assertLess(avg, 10.0, f"String-match Fast Path should be <10ms, got {avg:.2f}ms")

    def test_large_scene_latency(self):
        """200 \u7269\u4f53\u573a\u666f\u7684\u5ef6\u8fdf\u3002"""
        sg = make_scene("fire extinguisher", {"x": 50, "y": 50, "z": 0}, 0.88, 5, num_distractors=199, seed=99)

        times = []
        for _ in range(20):
            start = time.perf_counter()
            self.resolver.fast_resolve("find fire extinguisher", sg)
            elapsed_ms = (time.perf_counter() - start) * 1000
            times.append(elapsed_ms)

        avg = np.mean(times)
        print("\n=== Large Scene (200 objects) Latency ===")
        print(f"  Avg: {avg:.3f} ms")

        self.assertLess(avg, 50.0, f"Large scene should be <50ms, got {avg:.2f}ms")


class TestMultiSourceFusion(unittest.TestCase):
    """
    \u591a\u6e90\u7f6e\u4fe1\u5ea6\u878d\u5408\u6d4b\u8bd5 \u2014 \u65e0 CLIP \u65f6\u7684\u8bda\u5b9e\u8bc4\u4f30\u3002
    """

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config, fast_path_threshold=0.5)

    def test_label_match_beats_high_detector_score(self):
        """
        \u6807\u7b7e\u5339\u914d\u5e94\u8d62\u8fc7\u7eaf\u68c0\u6d4b\u5668\u9ad8\u5206\u3002

        \u573a\u666f: "find fire extinguisher"
        - fire extinguisher: \u6807\u7b7e\u5b8c\u5168\u5339\u914d, \u68c0\u6d4b\u5206=0.7
        - red box: \u4e0d\u5339\u914d, \u68c0\u6d4b\u5206=0.95
        """
        sg_data = {
            "timestamp": 0,
            "object_count": 5,
            "objects": [
                {
                    "id": 0,
                    "label": "fire extinguisher",
                    "position": {"x": 5, "y": 3, "z": 0},
                    "score": 0.7,
                    "detection_count": 3,
                },
                {
                    "id": 1,
                    "label": "red box",
                    "position": {"x": 6, "y": 4, "z": 0},
                    "score": 0.95,
                    "detection_count": 8,
                },
                {"id": 2, "label": "door", "position": {"x": 1, "y": 1, "z": 0}, "score": 0.9, "detection_count": 5},
                {"id": 3, "label": "chair", "position": {"x": 8, "y": 2, "z": 0}, "score": 0.88, "detection_count": 4},
                {"id": 4, "label": "sign", "position": {"x": 3, "y": 7, "z": 0}, "score": 0.82, "detection_count": 3},
            ],
            "relations": [],
            "regions": [],
            "summary": "test",
        }
        sg = json.dumps(sg_data)

        result = self.resolver.fast_resolve("find fire extinguisher", sg)
        self.assertIsNotNone(result)
        self.assertEqual(
            result.target_label, "fire extinguisher", "Label match should beat high detector score without CLIP"
        )

    def test_no_clip_weight_redistribution(self):
        """
        \u65e0 CLIP \u65f6\u6743\u91cd\u91cd\u5206\u914d\u7684\u6b63\u786e\u6027\u3002

        \u65e0 CLIP: label=0.55, det=0.25, spatial=0.20 (\u603b\u548c=1.0)
        \u6709 CLIP: label=0.35, clip=0.35, det=0.15, spatial=0.15 (\u603b\u548c=1.0)
        """
        # \u65e0 CLIP \u65f6, \u5b8c\u5168\u5339\u914d + \u9ad8\u68c0\u6d4b\u5206 \u5e94\u7ed9\u51fa\u9ad8\u5206
        # label=1.0, det=0.9*1.0=0.9, spatial=0.0
        # fused = 0.75*1.0 + 0.15*0.9 + 0.10*0.0 = 0.885
        sg_data = {
            "timestamp": 0,
            "object_count": 1,
            "objects": [
                {"id": 0, "label": "chair", "position": {"x": 3, "y": 2, "z": 0}, "score": 0.9, "detection_count": 5},
            ],
            "relations": [],
            "regions": [],
            "summary": "test",
        }
        sg = json.dumps(sg_data)

        result = self.resolver.fast_resolve("go to chair", sg)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(
            result.confidence, 0.885, places=2, msg=f"No-CLIP fused score should be ~0.885, got {result.confidence:.3f}"
        )

    def test_spatial_relation_boost(self):
        """
        \u7a7a\u95f4\u5173\u7cfb\u5e94\u589e\u52a0\u4e3b\u8bed\u5f97\u5206, \u800c\u975e\u4fee\u9970\u8bed\u3002
        """
        sg_data = {
            "timestamp": 0,
            "object_count": 2,
            "objects": [
                {"id": 0, "label": "chair", "position": {"x": 3, "y": 2, "z": 0}, "score": 0.8, "detection_count": 3},
                {"id": 1, "label": "door", "position": {"x": 3, "y": 3, "z": 0}, "score": 0.9, "detection_count": 5},
            ],
            "relations": [
                {"subject_id": 0, "relation": "near", "object_id": 1, "distance": 1.0},
            ],
            "regions": [],
            "summary": "test",
        }
        sg = json.dumps(sg_data)

        result = self.resolver.fast_resolve("find chair near the door", sg)
        self.assertIsNotNone(result)
        self.assertEqual(result.target_label, "chair")

        # chair \u7684 spatial_score = 1.0 (door \u88ab\u6307\u4ee4\u63d0\u53ca)
        # door \u7684 spatial_score = 0.3 (near relation \u4f46\u4e0d\u662f\u4e3b\u8bed)
        # chair fused \u5e94 > door fused
        self.assertGreater(result.confidence, 0.7)


class TestESCATokenReduction(unittest.TestCase):
    """
    ESCA \u9009\u62e9\u6027 Grounding Token \u51cf\u5c11\u6d4b\u8bd5\u3002

    \u8bba\u6587\u76ee\u6807: 90%+
    \u8bda\u5b9e\u5206\u6790: \u6211\u4eec\u7684\u5b9e\u73b0\u662f\u7b80\u5316\u7248 (keyword \u5339\u914d + 1-hop \u6269\u5c55),
    \u4e0d\u662f\u8bba\u6587\u7684\u771f\u5b9e CLIP \u7279\u5f81\u7b5b\u9009\u3002
    """

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config)

    def test_200_objects_reduction(self):
        """\u5927\u573a\u666f\u8fc7\u6ee4\u5e94\u4fdd\u7559\u76ee\u6807\u3002"""
        objects = [
            {
                "id": i,
                "label": f"random_obj_{i}",
                "position": {"x": i % 20, "y": i // 20, "z": 0},
                "score": 0.5,
                "detection_count": 2,
            }
            for i in range(200)
        ]
        objects.append(
            {
                "id": 999,
                "label": "fire extinguisher",
                "position": {"x": 50, "y": 50, "z": 0},
                "score": 0.9,
                "detection_count": 5,
            }
        )

        sg = json.dumps(
            {
                "timestamp": 0,
                "object_count": len(objects),
                "objects": objects,
                "relations": [],
                "regions": [],
                "summary": "large scene",
            }
        )

        filtered_sg = self.resolver._selective_grounding("find the fire extinguisher", sg, max_objects=15)
        filtered = json.loads(filtered_sg)

        original = len(objects)
        kept = filtered["object_count"]
        reduction = 1 - (kept / original)

        print("\n=== ESCA Token Reduction ===")
        print(f"  Original: {original} objects")
        print(f"  Filtered: {kept} objects")
        print(f"  Reduction: {reduction * 100:.1f}%")
        print("  NOTE: Our ESCA is keyword-based, not CLIP-feature-based like the paper")

        # \u76ee\u6807\u4fdd\u7559
        labels = [o["label"] for o in filtered["objects"]]
        self.assertIn("fire extinguisher", labels, "Target must be retained after filtering")

        # \u51cf\u5c11\u7387 >= 90%
        self.assertGreaterEqual(reduction, 0.90)


class TestChineseTokenization(unittest.TestCase):
    """\u4e2d\u6587\u5206\u8bcd\u51c6\u786e\u7387\u6d4b\u8bd5\u3002"""

    def setUp(self):
        self.config = LLMConfig(backend="openai", model="test")
        self.resolver = GoalResolver(self.config)

    def test_chinese_keyword_extraction(self):
        """\u6838\u5fc3\u4e2d\u6587\u5173\u952e\u8bcd\u63d0\u53d6\u3002"""
        cases = [
            ("\u53bb\u7ea2\u8272\u706d\u706b\u5668", ["\u706d\u706b\u5668"]),
            ("\u5bfc\u822a\u5230\u5145\u7535\u6869", ["\u5145\u7535\u6869"]),
            ("\u627e\u767d\u8272\u7684\u684c\u5b50", ["\u684c\u5b50"]),
        ]

        all_passed = True
        for instr, expected_must_contain in cases:
            kws = self.resolver._extract_keywords(instr)
            all_text = " ".join(kws)
            for exp in expected_must_contain:
                if exp not in all_text:
                    print(f"  FAIL: '{exp}' not found in keywords {kws} for '{instr}'")
                    all_passed = False
                else:
                    print(f"  OK: '{exp}' found in keywords {kws}")

        self.assertTrue(all_passed, "Some Chinese keywords were not extracted correctly")

    def test_mixed_chinese_english(self):
        """\u4e2d\u82f1\u6df7\u5408\u6307\u4ee4\u3002"""
        kws = self.resolver._extract_keywords("\u627efire extinguisher")
        self.assertIn("fire", kws)
        self.assertIn("extinguisher", kws)


class TestHonestPerformanceSummary(unittest.TestCase):
    """
    \u8bda\u5b9e\u7684\u6027\u80fd\u603b\u7ed3 \u2014 \u5bf9\u7167\u8bba\u6587\u6307\u6807\u7684\u5dee\u8ddd\u5206\u6790\u3002
    """

    def test_summary(self):
        """
        \u6253\u5370\u8bda\u5b9e\u7684\u80fd\u529b\u8bc4\u4f30\u62a5\u544a\u3002
        """
        print("\n" + "=" * 70)
        print("  \u8bda\u5b9e\u6027\u80fd\u8bc4\u4f30 \u2014 \u5bf9\u7167\u8bba\u6587\u6307\u6807")
        print("=" * 70)

        rows = [
            (
                "Fast Path \u547d\u4e2d\u7387",
                "\u226570%",
                "~70-80% (\u7b80\u5355\u6307\u4ee4)",
                "\u65e0CLIP, \u4ec5\u5b57\u7b26\u4e32\u5339\u914d. \u8bba\u6587\u6709\u771f\u5b9eCLIP\u7279\u5f81.",
            ),
            (
                "Fast Path \u54cd\u5e94\u65f6\u95f4",
                "<200ms",
                "<1ms (\u5b57\u7b26\u4e32\u5339\u914d)",
                "\u4e0d\u53ef\u6bd4: \u8bba\u6587\u542b CLIP \u63a8\u7406(~50ms). \u6211\u4eec\u7684<1ms\u4ec5\u56e0\u65e0CLIP.",
            ),
            (
                "ESCA Token \u51cf\u5c11\u7387",
                "\u226590%",
                "~92%",
                "\u7b80\u5316\u7248(keyword+1hop). \u8bba\u6587\u7528CLIP\u7279\u5f81\u7b5b\u9009.",
            ),
            (
                "\u591a\u6e90\u878d\u5408\u51c6\u786e\u7387",
                "+15-20%",
                "\u65e0\u6cd5\u91cf\u5316",
                "\u65e0\u57fa\u7ebf\u5bf9\u6bd4. \u9700\u8981\u771f\u5b9e\u6570\u636e\u96c6\u6d4b\u8bd5.",
            ),
            (
                "\u7a7a\u95f4\u5173\u7cfb\u63a8\u7406",
                "N/A",
                "\u5df2\u4fee\u590d",
                "\u4e3b\u8bed/\u4fee\u9970\u8bed\u533a\u5206\u5df2\u5b9e\u73b0, \u4f46\u4ec5\u652f\u6301\u7b80\u5355\u4ecb\u8bcd\u6a21\u5f0f.",
            ),
            (
                "\u4e2d\u6587\u5206\u8bcd",
                "+30-50%",
                "\u6709\u6548",
                "jieba\u5206\u8bcd\u5df2\u96c6\u6210, \u4f46\u65e0\u91cf\u5316\u57fa\u7ebf\u5bf9\u6bd4.",
            ),
        ]

        for metric, paper_target, our_result, honest_note in rows:
            print(f"\n  {metric}")
            print(f"    \u8bba\u6587\u76ee\u6807: {paper_target}")
            print(f"    \u5b9e\u9645\u7ed3\u679c: {our_result}")
            print(f"    \u8bda\u5b9e\u8bf4\u660e: {honest_note}")

        print("\n" + "=" * 70)
        print("  \u5173\u952e\u5dee\u8ddd")
        print("=" * 70)
        print(
            "  1. \u65e0\u771f\u5b9e CLIP \u63a8\u7406 \u2014 \u6240\u6709 CLIP \u76f8\u5173\u6307\u6807\u4e0d\u53ef\u76f4\u63a5\u5bf9\u6bd4\u8bba\u6587"
        )
        print(
            "  2. \u65e0\u771f\u5b9e\u573a\u666f\u6570\u636e\u96c6 (R2R, REVERIE, MP3D) \u2014 \u65e0\u6cd5\u8ba1\u7b97 SR/SPL"
        )
        print("  3. Slow Path \u672a\u6d4b\u8bd5 (\u65e0 LLM API key)")
        print("  4. \u672a\u8fdb\u884c\u7aef\u5230\u7aef ROS2 \u96c6\u6210\u6d4b\u8bd5")
        print("  5. \u672a\u5728\u771f\u5b9e\u673a\u5668\u4eba\u4e0a\u6d4b\u8bd5 (Jetson + \u56db\u8db3)")
        print("=" * 70)


if __name__ == "__main__":
    unittest.main(verbosity=2)
