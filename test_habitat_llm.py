#!/usr/bin/env python3
"""Test Goal Resolver Slow Path in Habitat simulation."""
import sys
import os
import json
import time
import asyncio
import numpy as np

sys.path.insert(0, "/home/bsrl/hongsenpang/habitat/MapPilot/src/semantic_planner")

# API configuration
os.environ["KIMI_API_KEY"] = "sk-tpUJJs4F9dlNsPadiWXraVnN1x1PQhHnkbbqo9uMwvLkUtb8"

from semantic_planner.goal_resolver import GoalResolver
from semantic_planner.llm_client import LLMConfig

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg

SCENE_PATH = "/home/bsrl/hongsenpang/habitat/data/hm3d_hf/minival/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"

def create_simulator():
    """Create Habitat simulator."""
    settings = default_sim_settings.copy()
    settings["width"] = 640
    settings["height"] = 480
    settings["scene"] = SCENE_PATH
    settings["sensor_height"] = 0.88
    settings["color_sensor"] = True
    settings["depth_sensor"] = True
    settings["seed"] = 42
    settings["enable_physics"] = True
    cfg = make_cfg(settings)
    return habitat_sim.Simulator(cfg)

def setup_navmesh(sim):
    """Generate navigation mesh."""
    navmesh_settings = habitat_sim.NavMeshSettings()
    navmesh_settings.set_defaults()
    navmesh_settings.agent_radius = 0.25
    navmesh_settings.agent_height = 0.88
    return sim.recompute_navmesh(sim.pathfinder, navmesh_settings)

def create_mock_scene_graph(sim):
    """Create a mock scene graph based on random positions in the scene."""
    objects = []
    labels = ["chair", "table", "sofa", "bed", "refrigerator", "tv", "sink", "toilet", "lamp", "door"]

    for i, label in enumerate(labels):
        pos = sim.pathfinder.get_random_navigable_point()
        objects.append({
            "id": i + 1,
            "label": label,
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "confidence": 0.85 + np.random.random() * 0.1
        })

    # Create some relations
    relations = [
        {"subject_id": 1, "predicate": "near", "object_id": 2},  # chair near table
        {"subject_id": 3, "predicate": "facing", "object_id": 6},  # sofa facing tv
        {"subject_id": 7, "predicate": "near", "object_id": 8},  # sink near toilet
    ]

    # Create regions
    regions = [
        {"name": "living_room", "object_ids": [1, 2, 3, 6, 9]},
        {"name": "bedroom", "object_ids": [4]},
        {"name": "kitchen", "object_ids": [5]},
        {"name": "bathroom", "object_ids": [7, 8]},
    ]

    return {
        "objects": objects,
        "relations": relations,
        "regions": regions,
        "frame_id": "map"
    }

async def test_navigation_with_llm(sim, resolver, scene_graph):
    """Test navigation using LLM goal resolution."""
    agent = sim.get_agent(0)

    # Test instructions
    instructions = [
        "Go to the chair",
        "Find a place to sit and watch TV",
        "Navigate to where I can wash my hands",
        "Take me to the bedroom",
    ]

    print("\n" + "="*70)
    print("  Habitat + Goal Resolver Integration Test")
    print("="*70)

    scene_graph_json = json.dumps(scene_graph)
    results = []

    for instr in instructions:
        print(f'\nInstruction: "{instr}"')

        # Get robot position
        robot_pos = agent.get_state().position
        print(f"  Robot at: [{robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}]")

        # Try Fast Path first
        t0 = time.time()
        fast_result = resolver.fast_resolve(instr, scene_graph_json)
        fast_time = (time.time() - t0) * 1000

        if fast_result and fast_result.is_valid:
            print(f"  Fast Path [{fast_time:.2f}ms]: {fast_result.target_label}")
            goal = np.array([fast_result.target_x, fast_result.target_y, fast_result.target_z])
            path_type = "fast"
        else:
            print(f"  Fast Path [{fast_time:.2f}ms]: No match, trying Slow Path...")

            # Try Slow Path with LLM
            t0 = time.time()
            try:
                result = await resolver.resolve(instr, scene_graph_json)
                slow_time = (time.time() - t0) * 1000

                if result and result.is_valid:
                    print(f"  Slow Path [{slow_time:.2f}ms]: {result.target_label}")
                    print(f"  Reasoning: {result.reasoning[:100]}..." if len(result.reasoning) > 100 else f"  Reasoning: {result.reasoning}")
                    goal = np.array([result.target_x, result.target_y, result.target_z])
                    path_type = "slow"
                else:
                    print(f"  Slow Path [{slow_time:.2f}ms]: Failed - {result.error if result else 'No result'}")
                    continue
            except Exception as e:
                print(f"  Slow Path: Error - {e}")
                continue

        # Navigate to goal
        path = habitat_sim.ShortestPath()
        path.requested_start = robot_pos
        path.requested_end = goal

        if sim.pathfinder.find_path(path):
            print(f"  Path found: {len(path.points)} waypoints, {path.geodesic_distance:.2f}m")

            # Simulate navigation
            for i, waypoint in enumerate(path.points[:20]):
                agent_state = agent.get_state()
                agent_state.position = waypoint
                agent.set_state(agent_state)

            final_dist = np.linalg.norm(agent.get_state().position - goal)
            success = final_dist < 1.0
            print(f"  Navigation: {'SUCCESS' if success else 'FAILED'} (final dist: {final_dist:.2f}m)")
            results.append({"instruction": instr, "path": path_type, "success": success})
        else:
            print(f"  No path found to goal")
            results.append({"instruction": instr, "path": path_type, "success": False})

    return results

async def main():
    print("="*70)
    print("  MapPilot Habitat + LLM Integration Test")
    print("="*70)

    # Create LLM config
    config = LLMConfig(
        backend="openai",
        model="kimi-k2.5",
        api_key_env="KIMI_API_KEY",
        base_url="https://api.xiaocaseai.com/v1",
        timeout_sec=60.0,
        temperature=0.2,
    )

    print(f"\nLLM: {config.model} @ {config.base_url}")

    # Create simulator
    print("\n[1] Creating Habitat simulator...")
    sim = create_simulator()
    print(f"    Scene loaded: {SCENE_PATH.split('/')[-1]}")

    # Setup navmesh
    print("\n[2] Generating NavMesh...")
    setup_navmesh(sim)
    print(f"    Navigable area: {sim.pathfinder.navigable_area:.2f} m2")

    # Create scene graph
    print("\n[3] Creating scene graph...")
    scene_graph = create_mock_scene_graph(sim)
    print(f"    Objects: {len(scene_graph['objects'])}")
    for obj in scene_graph["objects"]:
        pos = obj["position"]
        print(f"      - {obj['label']}: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

    # Create resolver
    print("\n[4] Initializing Goal Resolver...")
    resolver = GoalResolver(primary_config=config)
    print("    Fast-Slow dual process ready")

    # Run tests
    results = await test_navigation_with_llm(sim, resolver, scene_graph)

    # Summary
    print("\n" + "="*70)
    print("  SUMMARY")
    print("="*70)

    fast_count = sum(1 for r in results if r["path"] == "fast")
    slow_count = sum(1 for r in results if r["path"] == "slow")
    success_count = sum(1 for r in results if r["success"])

    print(f"  Total tests: {len(results)}")
    print(f"  Fast Path: {fast_count}, Slow Path: {slow_count}")
    print(f"  Success rate: {success_count}/{len(results)} ({100*success_count/len(results):.0f}%)")

    sim.close()
    print("\n  Simulator closed.")
    print("="*70)

if __name__ == "__main__":
    asyncio.run(main())
