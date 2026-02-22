#!/usr/bin/env python3
"""Test Goal Resolver Slow Path with LLM reasoning."""
import sys
import os
import json
import time
import asyncio

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")

from semantic_planner.goal_resolver import GoalResolver
from semantic_planner.llm_client import LLMConfig

# API configuration - set via environment variable
os.environ["KIMI_API_KEY"] = "sk-tpUJJs4F9dlNsPadiWXraVnN1x1PQhHnkbbqo9uMwvLkUtb8"
BASE_URL = "https://api.xiaocaseai.com"
MODEL = "kimi-k2.5"

print(f"Using API: {BASE_URL}")
print(f"Model: {MODEL}")

# Create LLM config
config = LLMConfig(
    backend="openai",
    model=MODEL,
    api_key_env="KIMI_API_KEY",
    base_url=BASE_URL,
    timeout_sec=30.0,
    temperature=0.2,
)

# Mock scene graph with rich semantic info
scene_graph = {
    "objects": [
        {"id": 1, "label": "dining table", "position": [2.0, 0.0, 3.0], "confidence": 0.92},
        {"id": 2, "label": "chair", "position": [2.5, 0.0, 3.0], "confidence": 0.88},
        {"id": 3, "label": "chair", "position": [1.5, 0.0, 3.0], "confidence": 0.87},
        {"id": 4, "label": "refrigerator", "position": [4.0, 0.0, 2.0], "confidence": 0.95},
        {"id": 5, "label": "sofa", "position": [-2.0, 0.0, 1.0], "confidence": 0.90},
        {"id": 6, "label": "tv", "position": [-3.0, 0.0, 0.0], "confidence": 0.85},
        {"id": 7, "label": "bed", "position": [0.0, 0.0, -5.0], "confidence": 0.93},
        {"id": 8, "label": "nightstand", "position": [1.0, 0.0, -5.0], "confidence": 0.80},
        {"id": 9, "label": "toilet", "position": [5.0, 0.0, -2.0], "confidence": 0.91},
        {"id": 10, "label": "sink", "position": [5.5, 0.0, -1.5], "confidence": 0.88},
    ],
    "relations": [
        {"subject_id": 2, "predicate": "near", "object_id": 1},
        {"subject_id": 3, "predicate": "near", "object_id": 1},
        {"subject_id": 4, "predicate": "in_same_room", "object_id": 1},
        {"subject_id": 5, "predicate": "facing", "object_id": 6},
        {"subject_id": 8, "predicate": "near", "object_id": 7},
        {"subject_id": 10, "predicate": "near", "object_id": 9},
    ],
    "regions": [
        {"name": "kitchen", "object_ids": [1, 2, 3, 4]},
        {"name": "living_room", "object_ids": [5, 6]},
        {"name": "bedroom", "object_ids": [7, 8]},
        {"name": "bathroom", "object_ids": [9, 10]},
    ]
}

scene_graph_json = json.dumps(scene_graph)

# Create resolver
resolver = GoalResolver(primary_config=config)

# Complex instructions that require LLM reasoning (Slow Path)
complex_instructions = [
    "Go to the room where people usually eat",
    "Find a place to watch TV",
    "Navigate to where I can wash my hands",
    "Take me to where I can sleep",
    "Go to the area with the refrigerator",
]

async def test_resolve():
    print("="*70)
    print("  Goal Resolver - Slow Path (LLM Reasoning) Test")
    print("="*70)
    print(f"\nScene has {len(scene_graph['objects'])} objects in {len(scene_graph['regions'])} regions")
    print("\nTesting complex instructions that require semantic reasoning...\n")

    for instr in complex_instructions:
        print(f'Instruction: "{instr}"')

        # First try Fast Path
        t0 = time.time()
        fast_result = resolver.fast_resolve(instr, scene_graph_json)
        fast_time = (time.time() - t0) * 1000

        if fast_result and fast_result.is_valid:
            pos = f"{fast_result.target_x:.1f}, {fast_result.target_y:.1f}, {fast_result.target_z:.1f}"
            print(f"  Fast Path [{fast_time:.2f}ms]: {fast_result.target_label} @ {pos}")
            print(f"  (Fast Path succeeded, Slow Path not needed)\n")
        else:
            print(f"  Fast Path [{fast_time:.2f}ms]: No direct match")

            # Try Full resolve (includes Slow Path with LLM)
            t0 = time.time()
            try:
                result = await resolver.resolve(instr, scene_graph_json)
                resolve_time = (time.time() - t0) * 1000

                if result and result.is_valid:
                    pos = f"{result.target_x:.1f}, {result.target_y:.1f}, {result.target_z:.1f}"
                    print(f"  Slow Path [{resolve_time:.2f}ms]: {result.target_label} @ {pos}")
                    path = getattr(result, 'path', 'N/A')
                    print(f"  Path: {path}\n")
                else:
                    error = result.error if result else "No result"
                    print(f"  Slow Path [{resolve_time:.2f}ms]: Failed - {error}\n")
            except Exception as e:
                print(f"  Slow Path: Error - {e}\n")
                import traceback
                traceback.print_exc()

    print("="*70)

if __name__ == "__main__":
    asyncio.run(test_resolve())
