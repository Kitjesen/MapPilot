"""
Slow Path (System 2) end-to-end validation script.

Validates using the Moonshot / Kimi API:
  1. Hierarchical Chain-of-Thought reasoning (Room -> Group -> Object)
  2. Cross-lingual understanding (Chinese instruction + English scene graph)
  3. Exploration direction suggestion (target not in scene graph)

API key is read from environment variable 鈥?never hardcoded:
  export MOONSHOT_API_KEY=sk-xxx
"""

import asyncio
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from decision.llm_client import LLMConfig, create_llm_client, LLMError
from decision.goal_resolver import GoalResolver, GoalResult
from decision.prompt_templates import build_goal_resolution_prompt


SCENE_GRAPH = {
    "summary": "Indoor corridor with offices. 12 objects across 2 rooms.",
    "rooms": [
        {
            "id": "room_0",
            "name": "corridor",
            "object_labels": ["door", "fire extinguisher", "sign", "trash can"],
            "object_ids": [1, 2, 3, 4],
        },
        {
            "id": "room_1",
            "name": "office",
            "object_labels": ["chair", "desk", "monitor", "keyboard", "cup", "bookshelf", "window", "plant"],
            "object_ids": [5, 6, 7, 8, 9, 10, 11, 12],
        },
    ],
    "groups": [
        {"id": "group_0", "room_id": "room_0", "label": "safety", "object_ids": [2, 3]},
        {"id": "group_1", "room_id": "room_1", "label": "workstation", "object_ids": [5, 6, 7, 8]},
        {"id": "group_2", "room_id": "room_1", "label": "decoration", "object_ids": [10, 12]},
    ],
    "objects": [
        {"id": 1, "label": "door", "position": {"x": 0.0, "y": 0.0, "z": 0.0}, "score": 0.95, "detection_count": 5},
        {"id": 2, "label": "fire extinguisher", "position": {"x": 1.2, "y": 0.5, "z": 0.8}, "score": 0.92, "detection_count": 3},
        {"id": 3, "label": "exit sign", "position": {"x": 2.0, "y": 0.0, "z": 2.5}, "score": 0.88, "detection_count": 2},
        {"id": 4, "label": "trash can", "position": {"x": 0.5, "y": 1.0, "z": 0.0}, "score": 0.85, "detection_count": 4},
        {"id": 5, "label": "red chair", "position": {"x": 5.0, "y": 3.0, "z": 0.0}, "score": 0.93, "detection_count": 6},
        {"id": 6, "label": "desk", "position": {"x": 5.5, "y": 3.5, "z": 0.0}, "score": 0.97, "detection_count": 8},
        {"id": 7, "label": "monitor", "position": {"x": 5.5, "y": 3.5, "z": 0.7}, "score": 0.91, "detection_count": 5},
        {"id": 8, "label": "keyboard", "position": {"x": 5.5, "y": 3.3, "z": 0.6}, "score": 0.89, "detection_count": 4},
        {"id": 9, "label": "cup", "position": {"x": 6.0, "y": 3.0, "z": 0.6}, "score": 0.78, "detection_count": 2},
        {"id": 10, "label": "bookshelf", "position": {"x": 7.0, "y": 2.0, "z": 0.0}, "score": 0.90, "detection_count": 3},
        {"id": 11, "label": "window", "position": {"x": 5.0, "y": 5.0, "z": 1.5}, "score": 0.95, "detection_count": 7},
        {"id": 12, "label": "plant", "position": {"x": 7.5, "y": 2.5, "z": 0.0}, "score": 0.82, "detection_count": 2},
    ],
    "relations": [
        {"subject_id": 5, "object_id": 6, "relation": "near"},
        {"subject_id": 7, "object_id": 6, "relation": "on"},
        {"subject_id": 8, "object_id": 6, "relation": "on"},
        {"subject_id": 9, "object_id": 6, "relation": "near"},
        {"subject_id": 2, "object_id": 1, "relation": "near"},
        {"subject_id": 5, "object_id": 11, "relation": "near"},
        {"subject_id": 10, "object_id": 12, "relation": "near"},
    ],
}

SCENE_GRAPH_JSON = json.dumps(SCENE_GRAPH, ensure_ascii=False)

ROBOT_POS = {"x": 3.0, "y": 1.5, "z": 0.0}


@dataclass
class TestCase:
    name: str
    instruction: str
    language: str
    expected_action: str
    expected_labels: List[str]  # any of these counts as a match
    difficulty: str  # L1/L2/L3


TEST_CASES = [
    # --- EN Fast Path (no LLM needed) ---
    TestCase("EN_L1_find_chair", "find the chair", "en",
             "navigate", ["red chair", "chair"], "L1"),
    TestCase("EN_L1_find_door", "go to the door", "en",
             "navigate", ["door"], "L1"),
    TestCase("EN_L2_chair_near_desk", "find the chair near the desk", "en",
             "navigate", ["red chair", "chair"], "L2"),
    TestCase("EN_L2_cup_on_desk", "find the cup on the desk", "en",
             "navigate", ["cup"], "L2"),
    TestCase("EN_L3_office_bookshelf", "go to the bookshelf in the office", "en",
             "navigate", ["bookshelf"], "L3"),
    TestCase("EN_L3_plant_near_bookshelf", "find the plant near the bookshelf in the office", "en",
             "navigate", ["plant"], "L3"),

    # --- ZH Slow Path (needs LLM for cross-lingual) ---
    TestCase("ZH_L1_find_chair", "\u627e\u5230\u6905\u5b50", "zh",
             "navigate", ["red chair", "chair"], "L1"),
    TestCase("ZH_L1_find_door", "\u53bb\u95e8\u90a3\u91cc", "zh",
             "navigate", ["door"], "L1"),
    TestCase("ZH_L2_fire_ext_near_door", "\u627e\u5230\u95e8\u65c1\u8fb9\u7684\u706d\u706b\u5668", "zh",
             "navigate", ["fire extinguisher"], "L2"),
    TestCase("ZH_L3_corridor_sign", "\u8d70\u5eca\u91cc\u7684\u51fa\u53e3\u6807\u5fd7\u5728\u54ea\u91cc", "zh",
             "navigate", ["exit sign", "sign"], "L3"),

    # --- Explore: target not in scene (needs LLM) ---
    TestCase("EN_explore_fridge", "find the refrigerator", "en",
             "explore", [], "L3"),
    TestCase("ZH_explore_sofa", "\u627e\u5230\u6c99\u53d1", "zh",
             "explore", [], "L3"),
]


@dataclass
class TestResult:
    name: str
    difficulty: str
    passed: bool
    action_correct: bool
    label_correct: bool
    confidence: float
    reasoning: str
    latency_ms: float
    error: str = ""


def make_moonshot_config() -> LLMConfig:
    return LLMConfig(
        backend="openai",
        model="kimi-k2.5",
        api_key_env="MOONSHOT_API_KEY",
        timeout_sec=120.0,
        max_retries=2,
        temperature=1.0,  # kimi-k2.5 only accepts temperature=1
        base_url="https://api.moonshot.cn/v1",
    )


async def run_single_test(resolver: GoalResolver, tc: TestCase) -> TestResult:
    t0 = time.perf_counter()
    try:
        result: GoalResult = await resolver.resolve(
            instruction=tc.instruction,
            scene_graph_json=SCENE_GRAPH_JSON,
            robot_position=ROBOT_POS,
            language=tc.language,
            explore_if_unknown=True,
        )
        latency = (time.perf_counter() - t0) * 1000

        action_ok = result.action == tc.expected_action

        label_ok = False
        if tc.expected_action == "explore":
            label_ok = True
        else:
            got = result.target_label.lower()
            for exp in tc.expected_labels:
                if exp.lower() in got or got in exp.lower():
                    label_ok = True
                    break

        passed = action_ok and label_ok

        return TestResult(
            name=tc.name,
            difficulty=tc.difficulty,
            passed=passed,
            action_correct=action_ok,
            label_correct=label_ok,
            confidence=result.confidence,
            reasoning=result.reasoning[:200],
            latency_ms=latency,
        )
    except Exception as e:
        latency = (time.perf_counter() - t0) * 1000
        return TestResult(
            name=tc.name,
            difficulty=tc.difficulty,
            passed=False,
            action_correct=False,
            label_correct=False,
            confidence=0.0,
            reasoning="",
            latency_ms=latency,
            error=str(e)[:200],
        )


async def run_all_tests() -> List[TestResult]:
    api_key = os.environ.get("MOONSHOT_API_KEY", "")
    if not api_key:
        print("[ERROR] MOONSHOT_API_KEY not set. Run:")
        print('  export MOONSHOT_API_KEY="sk-xxx"')
        sys.exit(1)

    print(f"API key found: {api_key[:8]}...{api_key[-4:]}")
    print(f"Model: kimi-k2.5, Base URL: https://api.moonshot.cn/v1")
    print(f"Running {len(TEST_CASES)} test cases...\n")

    config = make_moonshot_config()
    resolver = GoalResolver(primary_config=config)

    results: List[TestResult] = []
    for i, tc in enumerate(TEST_CASES):
        print(f"  [{i+1}/{len(TEST_CASES)}] {tc.name} ({tc.difficulty}, {tc.language})...", end=" ", flush=True)

        # Rate-limit between LLM-requiring tests to avoid API throttling
        needs_llm = not _is_fast_path_case(tc)
        if needs_llm and i > 0:
            prev_needed_llm = not _is_fast_path_case(TEST_CASES[i - 1])
            if prev_needed_llm:
                print("[wait 10s]...", end=" ", flush=True)
                await asyncio.sleep(10)

        r = await run_single_test(resolver, tc)
        results.append(r)
        status = "PASS" if r.passed else "FAIL"
        print(f"{status}  ({r.latency_ms:.0f}ms, conf={r.confidence:.2f})")
        if r.error:
            print(f"         ERROR: {r.error}")
        elif not r.passed:
            print(f"         action_ok={r.action_correct}, label_ok={r.label_correct}")
            print(f"         reasoning: {r.reasoning[:120]}")

    return results


def _is_fast_path_case(tc: TestCase) -> bool:
    """Heuristic: English instructions with targets in scene resolve via Fast Path."""
    return tc.language == "en" and tc.expected_action == "navigate"


def generate_report(results: List[TestResult]) -> str:
    lines = ["# Slow Path (Kimi-k2.5) Validation Report\n"]
    total = len(results)
    passed = sum(1 for r in results if r.passed)
    lines.append(f"**Overall: {passed}/{total} passed ({100*passed/total:.0f}%)**\n")

    avg_lat = sum(r.latency_ms for r in results) / total if total else 0
    avg_conf = sum(r.confidence for r in results if r.passed) / max(passed, 1)
    lines.append(f"- Avg latency: {avg_lat:.0f} ms")
    lines.append(f"- Avg confidence (passed): {avg_conf:.2f}\n")

    for diff in ["L1", "L2", "L3"]:
        subset = [r for r in results if r.difficulty == diff]
        if not subset:
            continue
        p = sum(1 for r in subset if r.passed)
        lines.append(f"### {diff}: {p}/{len(subset)} passed")

        for lang in ["en", "zh"]:
            lang_sub = [r for r in subset if lang in r.name.lower()[:3]]
            if lang_sub:
                lp = sum(1 for r in lang_sub if r.passed)
                lines.append(f"  - {lang.upper()}: {lp}/{len(lang_sub)}")

        lines.append("")

    lines.append("### Detailed Results\n")
    lines.append("| # | Test | Diff | Pass | Action | Label | Conf | Latency | Error |")
    lines.append("|---|------|------|------|--------|-------|------|---------|-------|")
    for i, r in enumerate(results):
        mark = "Y" if r.passed else "N"
        act = "Y" if r.action_correct else "N"
        lbl = "Y" if r.label_correct else "N"
        err = r.error[:40] if r.error else "-"
        lines.append(
            f"| {i+1} | {r.name} | {r.difficulty} | {mark} | "
            f"{act} | {lbl} | {r.confidence:.2f} | {r.latency_ms:.0f}ms | {err} |"
        )

    lines.append("\n### Hierarchical CoT Examples\n")
    for r in results:
        if r.passed and r.reasoning:
            lines.append(f"**{r.name}**:")
            lines.append(f"> {r.reasoning[:300]}\n")

    return "\n".join(lines)


async def main():
    print("=" * 60)
    print("  Slow Path Validation -- Moonshot Kimi-k2.5")
    print("=" * 60)
    print()

    results = await run_all_tests()

    print("\n" + "=" * 60)
    print("  SUMMARY")
    print("=" * 60)
    total = len(results)
    passed = sum(1 for r in results if r.passed)
    print(f"  Total:   {passed}/{total} ({100*passed/total:.0f}%)")

    for diff in ["L1", "L2", "L3"]:
        subset = [r for r in results if r.difficulty == diff]
        p = sum(1 for r in subset if r.passed)
        print(f"  {diff}:      {p}/{len(subset)}")

    zh_cases = [r for r in results if "zh" in r.name.lower()[:3]]
    zh_pass = sum(1 for r in zh_cases if r.passed)
    print(f"  Chinese: {zh_pass}/{len(zh_cases)}")

    report = generate_report(results)
    report_path = Path(__file__).parent / "slow_path_report.md"
    report_path.write_text(report, encoding="utf-8")
    print(f"\n  Report saved to: {report_path}")


if __name__ == "__main__":
    asyncio.run(main())
