"""
Kimi-k2.5 detailed reasoning demo.
Shows complete Fast Path and Slow Path output including raw LLM reasoning.
"""
import asyncio
import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from decision.llm_client import LLMConfig, create_llm_client
from decision.goal_resolver import GoalResolver
from decision.prompt_templates import build_goal_resolution_prompt

SCENE_GRAPH = {
    "summary": "Two-floor office building. Floor 1: corridor + main office. 15 objects total.",
    "rooms": [
        {"id": "room_0", "name": "corridor", "object_labels": ["door", "fire extinguisher", "exit sign", "trash can", "umbrella stand"], "object_ids": [1, 2, 3, 4, 15]},
        {"id": "room_1", "name": "office", "object_labels": ["red chair", "blue chair", "desk", "monitor", "keyboard", "cup", "bookshelf", "window", "plant", "whiteboard"], "object_ids": [5, 13, 6, 7, 8, 9, 10, 11, 12, 14]},
    ],
    "groups": [
        {"id": "group_0", "room_id": "room_0", "label": "safety", "object_ids": [2, 3]},
        {"id": "group_1", "room_id": "room_0", "label": "entrance", "object_ids": [1, 4, 15]},
        {"id": "group_2", "room_id": "room_1", "label": "workstation", "object_ids": [5, 6, 7, 8, 9]},
        {"id": "group_3", "room_id": "room_1", "label": "decoration", "object_ids": [10, 12, 14]},
    ],
    "objects": [
        {"id": 1,  "label": "door",              "position": {"x": 0.0, "y": 0.0, "z": 0.0}, "score": 0.95, "detection_count": 8},
        {"id": 2,  "label": "fire extinguisher",  "position": {"x": 1.2, "y": 0.5, "z": 0.8}, "score": 0.92, "detection_count": 4},
        {"id": 3,  "label": "exit sign",          "position": {"x": 2.0, "y": 0.0, "z": 2.5}, "score": 0.88, "detection_count": 3},
        {"id": 4,  "label": "trash can",          "position": {"x": 0.5, "y": 1.0, "z": 0.0}, "score": 0.85, "detection_count": 5},
        {"id": 5,  "label": "red chair",          "position": {"x": 5.0, "y": 3.0, "z": 0.0}, "score": 0.93, "detection_count": 7},
        {"id": 6,  "label": "desk",               "position": {"x": 5.5, "y": 3.5, "z": 0.0}, "score": 0.97, "detection_count": 10},
        {"id": 7,  "label": "monitor",            "position": {"x": 5.5, "y": 3.5, "z": 0.7}, "score": 0.91, "detection_count": 6},
        {"id": 8,  "label": "keyboard",           "position": {"x": 5.5, "y": 3.3, "z": 0.6}, "score": 0.89, "detection_count": 5},
        {"id": 9,  "label": "cup",                "position": {"x": 6.0, "y": 3.0, "z": 0.6}, "score": 0.78, "detection_count": 2},
        {"id": 10, "label": "bookshelf",          "position": {"x": 7.0, "y": 2.0, "z": 0.0}, "score": 0.90, "detection_count": 4},
        {"id": 11, "label": "window",             "position": {"x": 5.0, "y": 5.0, "z": 1.5}, "score": 0.95, "detection_count": 8},
        {"id": 12, "label": "plant",              "position": {"x": 7.5, "y": 2.5, "z": 0.0}, "score": 0.82, "detection_count": 3},
        {"id": 13, "label": "blue chair",         "position": {"x": 6.5, "y": 4.0, "z": 0.0}, "score": 0.90, "detection_count": 5},
        {"id": 14, "label": "whiteboard",         "position": {"x": 8.0, "y": 3.0, "z": 1.2}, "score": 0.87, "detection_count": 3},
        {"id": 15, "label": "umbrella stand",     "position": {"x": 0.2, "y": 0.3, "z": 0.0}, "score": 0.75, "detection_count": 2},
    ],
    "relations": [
        {"subject_id": 5, "object_id": 6, "relation": "near", "distance": 0.7},
        {"subject_id": 13, "object_id": 6, "relation": "near", "distance": 1.2},
        {"subject_id": 7, "object_id": 6, "relation": "on"},
        {"subject_id": 8, "object_id": 6, "relation": "on"},
        {"subject_id": 9, "object_id": 6, "relation": "near", "distance": 0.8},
        {"subject_id": 2, "object_id": 1, "relation": "near", "distance": 1.5},
        {"subject_id": 5, "object_id": 11, "relation": "near", "distance": 2.0},
        {"subject_id": 10, "object_id": 12, "relation": "near", "distance": 0.7},
        {"subject_id": 14, "object_id": 10, "relation": "near", "distance": 1.2},
        {"subject_id": 15, "object_id": 1, "relation": "near", "distance": 0.3},
    ],
}

SG_JSON = json.dumps(SCENE_GRAPH, ensure_ascii=False)
ROBOT_POS = {"x": 3.0, "y": 1.5, "z": 0.0}

# (name, instruction, language, description)
TESTS = [
    ("EN-L1-simple",     "find the chair",                           "en", "L1: EN simple object"),
    ("ZH-L1-simple",     "\u627e\u5230\u6905\u5b50",                 "zh", "L1: ZH simple object"),
    ("ZH-L1-door",       "\u53bb\u95e8\u90a3\u91cc",                 "zh", "L1: ZH go to door"),
    ("EN-L2-spatial",    "find the chair near the desk",             "en", "L2: EN spatial relation"),
    ("ZH-L2-spatial",    "\u627e\u5230\u95e8\u65c1\u8fb9\u7684\u706d\u706b\u5668", "zh", "L2: ZH spatial relation"),
    ("ZH-L2-color",      "\u627e\u5230\u84dd\u8272\u7684\u6905\u5b50",             "zh", "L2: ZH color attribute"),
    ("EN-L2-on",         "find the cup on the desk",                 "en", "L2: EN 'on' relation"),
    ("EN-L3-room",       "go to the bookshelf in the office",        "en", "L3: EN room-level reasoning"),
    ("ZH-L3-room",       "\u8d70\u5eca\u91cc\u7684\u51fa\u53e3\u6807\u5fd7\u5728\u54ea\u91cc", "zh", "L3: ZH room-level reasoning"),
    ("ZH-L3-complex",    "\u529e\u516c\u5ba4\u91cc\u9760\u7a97\u7684\u90a3\u628a\u7ea2\u8272\u6905\u5b50", "zh", "L3: ZH complex room+attr+spatial"),
    ("EN-L3-multi",      "find the whiteboard near the bookshelf",   "en", "L3: EN multi-object association"),
    ("ZH-explore",       "\u627e\u5230\u51b0\u7bb1",                 "zh", "Explore: target not in scene"),
    ("EN-explore",       "find the coffee machine",                  "en", "Explore: target not in scene"),
    ("ZH-L2-umbrella",   "\u95e8\u53e3\u7684\u96e8\u4f1e\u67b6\u5728\u54ea",      "zh", "L2: uncommon object"),
    ("ZH-L3-meeting",    "\u767d\u677f\u65c1\u8fb9\u6709\u4ec0\u4e48\u4e1c\u897f", "zh", "L3: open-ended question"),
]


def make_config():
    return LLMConfig(
        backend="openai", model="kimi-k2.5",
        api_key_env="MOONSHOT_API_KEY", timeout_sec=120.0,
        max_retries=2, temperature=1.0,
        base_url="https://api.moonshot.cn/v1",
    )


def print_separator(char="=", width=72):
    print(char * width)


def print_fast_result(name, desc, result, elapsed):
    print_separator()
    print(f"  [{name}] {desc}")
    print(f"  Path: FAST PATH (no LLM)")
    print(f"  Latency: {elapsed:.1f} ms")
    print_separator("-")
    print(f"  Action:     {result.action}")
    print(f"  Target:     {result.target_label}  ({result.target_x:.1f}, {result.target_y:.1f}, {result.target_z:.1f})")
    print(f"  Confidence: {result.confidence:.2f}")
    print(f"  Scoring:    {result.reasoning}")
    print()


def print_slow_result(name, desc, result, elapsed, raw_llm=""):
    print_separator()
    print(f"  [{name}] {desc}")
    print(f"  Path: SLOW PATH (Kimi-k2.5 LLM)")
    print(f"  Latency: {elapsed:.0f} ms")
    print_separator("-")
    print(f"  Action:     {result.action}")
    if result.action == "navigate":
        print(f"  Target:     {result.target_label}  ({result.target_x:.1f}, {result.target_y:.1f}, {result.target_z:.1f})")
    elif result.action == "explore":
        print(f"  Explore to: ({result.target_x:.1f}, {result.target_y:.1f})")
    print(f"  Confidence: {result.confidence:.2f}")
    print(f"  Reasoning:  {result.reasoning[:500]}")
    if raw_llm:
        print_separator(".")
        print(f"  [RAW LLM Output]")
        for line in raw_llm.strip().split("\n"):
            print(f"    {line}")
    print()


async def run_demo():
    key = os.environ.get("MOONSHOT_API_KEY", "")
    if not key:
        print("ERROR: MOONSHOT_API_KEY not set")
        sys.exit(1)

    config = make_config()
    resolver = GoalResolver(primary_config=config)
    client = create_llm_client(config)

    print()
    print_separator("=")
    print("  Kimi-k2.5 Navigation Reasoning Demo")
    print(f"  Scene: 2 rooms, 15 objects, 10 relations")
    print(f"  Robot position: (3.0, 1.5, 0.0)")
    print_separator("=")
    print()

    all_output = []

    for i, (name, instruction, lang, desc) in enumerate(TESTS):
        # Fast Path
        t0 = time.perf_counter()
        fast = resolver.fast_resolve(instruction, SG_JSON, ROBOT_POS)
        fast_ms = (time.perf_counter() - t0) * 1000

        if fast is not None:
            print_fast_result(name, desc, fast, fast_ms)
            all_output.append({
                "name": name, "desc": desc, "instruction": instruction,
                "path": "fast", "latency_ms": round(fast_ms, 1),
                "action": fast.action, "target_label": fast.target_label,
                "confidence": fast.confidence, "reasoning": fast.reasoning,
            })
            continue

        # Slow Path 鈥?call LLM directly to capture raw output
        print(f"  [{i+1}/{len(TESTS)}] {name}: calling Kimi-k2.5...", end="", flush=True)

        if i > 0 and TESTS[i-1][2] in ("zh",) or "explore" in TESTS[i-1][0].lower():
            await asyncio.sleep(8)

        messages = build_goal_resolution_prompt(instruction, SG_JSON, ROBOT_POS, lang)
        t0 = time.perf_counter()
        try:
            raw_response = await client.chat(messages)
        except Exception as e:
            raw_response = f"ERROR: {e}"
        slow_ms = (time.perf_counter() - t0) * 1000
        print(f" done ({slow_ms:.0f}ms)")

        # Also get the parsed result
        t1 = time.perf_counter()
        result = await resolver.resolve(instruction, SG_JSON, ROBOT_POS, lang)
        resolve_ms = (time.perf_counter() - t1) * 1000

        print_slow_result(name, desc, result, slow_ms, raw_response)
        all_output.append({
            "name": name, "desc": desc, "instruction": instruction,
            "path": "slow", "latency_ms": round(slow_ms),
            "action": result.action, "target_label": result.target_label,
            "confidence": result.confidence, "reasoning": result.reasoning[:300],
            "raw_llm": raw_response[:500],
        })

        await asyncio.sleep(5)

    # Summary
    print_separator("=")
    print("  SUMMARY")
    print_separator("=")
    fast_count = sum(1 for o in all_output if o["path"] == "fast")
    slow_count = sum(1 for o in all_output if o["path"] == "slow")
    print(f"  Total tests:        {len(all_output)}")
    print(f"  Fast Path (no LLM): {fast_count}")
    print(f"  Slow Path (Kimi):   {slow_count}")
    fast_avg = sum(o["latency_ms"] for o in all_output if o["path"] == "fast") / max(fast_count, 1)
    slow_avg = sum(o["latency_ms"] for o in all_output if o["path"] == "slow") / max(slow_count, 1)
    print(f"  Fast avg latency:   {fast_avg:.1f} ms")
    print(f"  Slow avg latency:   {slow_avg:.0f} ms")
    print()

    # Save detailed JSON
    out_path = Path(__file__).parent / "kimi_demo_results.json"
    out_path.write_text(json.dumps(all_output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"  Full results saved to: {out_path}")


if __name__ == "__main__":
    asyncio.run(run_demo())
