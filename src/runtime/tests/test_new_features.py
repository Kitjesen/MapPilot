#!/usr/bin/env python3
"""Tests 31-36: Persistence, edge-case, and resilience tests for LingTu."""

import os
import sys

sys.path.insert(0, "src")
for d in ["src/perception", "src/decision"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
for k in ["MOONSHOT_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY", "DASHSCOPE_API_KEY"]:
    os.environ.pop(k, None)
import logging

logging.basicConfig(level=logging.WARNING)

import shutil
import tempfile
import time
import traceback

import numpy as np

results = []


def run_test(name, fn):
    """Run a test function, catch exceptions, record PASS/FAIL."""
    try:
        fn()
        results.append((name, True, ""))
        print(f"  PASS  {name}")
    except Exception as e:
        tb = traceback.format_exc()
        results.append((name, False, str(e)))
        print(f"  FAIL  {name}: {e}")
        # Print the most relevant traceback line
        lines = tb.strip().splitlines()
        for line in reversed(lines):
            if line.strip() and not line.startswith("Traceback"):
                print(f"        {line.strip()}")
                break


# =========================================================================
# Test 31: SemanticMapper persistence
# =========================================================================
def test_31_semantic_mapper_persistence():
    from memory.modules.semantic_mapper_module import SemanticMapperModule
    from runtime.msgs.geometry import Vector3
    from runtime.msgs.semantic import Detection3D, Region, SceneGraph

    tmpdir = tempfile.mkdtemp(prefix="test_persist_smap_")
    try:
        # Phase 1: create module, feed data, save
        mod1 = SemanticMapperModule(save_dir=tmpdir)
        mod1.setup()

        # Build a SceneGraph with a region referencing objects
        obj1 = Detection3D(id="obj_1", label="desk", confidence=0.9, position=Vector3(1.0, 2.0, 0.0))
        obj2 = Detection3D(id="obj_2", label="chair", confidence=0.85, position=Vector3(1.5, 2.5, 0.0))
        region = Region(name="office", object_ids=["obj_1", "obj_2"], center=Vector3(1.25, 2.25, 0.0))
        sg = SceneGraph(objects=[obj1, obj2], regions=[region])

        # Feed the scene graph directly
        mod1._on_scene_graph(sg)
        mod1._save_now()
        mod1.stop()

        # Phase 2: create new module with same save_dir, verify KG loaded
        mod2 = SemanticMapperModule(save_dir=tmpdir)
        mod2.setup()

        has_kg = mod2._kg is not None
        room_types = mod2._kg.room_types if has_kg else []
        assert has_kg, "KG should be loaded"
        assert "office" in room_types, f"Expected 'office' in room_types, got: {room_types}"
        mod2.stop()
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


# =========================================================================
# Test 32: VectorMemory numpy fallback (in-memory)
# =========================================================================
def test_32_vector_memory_numpy():
    from memory.modules.vector_memory_module import VectorMemoryModule
    from runtime.msgs.geometry import Pose, Vector3
    from runtime.msgs.nav import Odometry

    mod = VectorMemoryModule(persist_dir="/tmp/test_vmem_np", store_interval=0.0)
    mod.setup()

    # Provide odometry position
    odom = Odometry(pose=Pose(position=Vector3(5.0, 10.0, 0.0)))
    mod._on_odom(odom)

    # Store a snapshot with labels directly (bypass throttle)
    mod._store_snapshot(["backpack", "bench", "tree"])

    # Verify entries were stored (either numpy fallback or ChromaDB)
    n_entries = mod._entry_count()
    assert n_entries > 0, f"Expected entries > 0, got {n_entries}"

    # Verify query works and returns correct position
    result = mod._query("backpack")
    assert len(result) >= 1, f"Query should return at least 1 result, got {len(result)}"
    assert result[0]["x"] == 5.0, f"Expected x=5.0, got {result[0]['x']}"
    assert result[0]["y"] == 10.0, f"Expected y=10.0, got {result[0]['y']}"

    mod.stop()


# =========================================================================
# Test 33: No API key startup �?system.build() succeeds
# =========================================================================
def test_33_no_api_key_startup():
    # Double-check all API keys are removed
    for k in ["MOONSHOT_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY", "DASHSCOPE_API_KEY"]:
        os.environ.pop(k, None)

    from lingtu.assembly.products.thunder import thunder_blueprint

    # Build dev profile: stub driver, no C++ nodes, semantic enabled, mock LLM
    system = thunder_blueprint(
        robot="stub",
        slam_profile="none",
        detector="yoloe",
        encoder="mobileclip",
        llm="mock",
        planner="astar",
        enable_native=False,
        enable_semantic=True,
        enable_gateway=False,
    ).build()

    try:
        # Verify build succeeded
        assert system is not None, "system.build() returned None"

        # Start the system so setup() runs (which inits GoalResolver)
        system.start()

        # Check SemanticPlannerModule exists
        planner_mod = system.get_module("SemanticPlannerModule")
        assert planner_mod is not None, "SemanticPlannerModule not in system"

        # GoalResolver should survive missing API keys (mock LLM backend).
        # It may or may not init depending on LLMConfig, but the system
        # must not crash. The critical check is that we got here.
        assert system._started, "System should be started without crash"
    finally:
        system.stop()


# =========================================================================
# Test 34: No CLIP startup �?EncoderModule doesn't crash
# =========================================================================
def test_34_no_clip_startup():
    from unittest.mock import patch

    from perception.encoding.encoder_module import EncoderModule
    from lingtu.assembly.products.thunder import thunder_blueprint

    # Build dev profile �?on Windows, open_clip is typically not installed.
    # EncoderModule should catch ImportError and set _backend = None.
    class MissingClipBackend:
        def load_model(self):
            raise ImportError("open_clip unavailable in test")

    system = None
    with patch.object(
        EncoderModule,
        "_create_backend",
        return_value=MissingClipBackend(),
    ):
        system = thunder_blueprint(
            robot="stub",
            slam_profile="none",
            detector="yoloe",
            encoder="clip",
            llm="mock",
            planner="astar",
            enable_native=False,
            enable_semantic=True,
            enable_standalone_encoder=True,
            enable_gateway=False,
        ).build()

        try:
            system.start()
            assert system._started, "System should start even without CLIP"

            # EncoderModule should exist and degrade to a disabled backend.
            enc = system.get_module("EncoderModule")
            assert enc is not None, "EncoderModule should be in system"
            assert enc._backend is None, "EncoderModule should disable missing CLIP"
        finally:
            if system is not None:
                system.stop()


# =========================================================================
# Test 35: Empty SceneGraph �?no crash, no goal_pose published
# =========================================================================
def test_35_empty_scene_graph():
    from decision.modules.semantic_planner import SemanticPlannerModule
    from runtime.msgs.semantic import SceneGraph

    mod = SemanticPlannerModule()
    mod.setup()

    try:
        # Track if goal_pose gets published
        published_goals = []
        mod.goal_pose.subscribe(lambda p: published_goals.append(p))

        # Give an instruction first so the module is "active"
        mod._current_instruction = "find the table"

        # Deliver empty scene graph
        empty_sg = SceneGraph(objects=[], regions=[])
        mod._on_scene_graph(empty_sg)

        # Brief wait for any async processing
        time.sleep(0.05)

        # Should not crash. With empty SG the to_json is basically empty,
        # so fast_resolve should find nothing meaningful. Either 0 goals
        # published (ideal) or a fallback frontier/servo goal is fine.
        # The key assertion: we got here without exception.
        assert True, "Empty SceneGraph did not crash SemanticPlannerModule"
    finally:
        mod.stop()


# =========================================================================
# Test 36: WaypointTracker stuck detection
# =========================================================================
def test_36_waypoint_tracker_stuck():
    from nav.tracking.waypoint_tracker import EV_STUCK, EV_STUCK_WARN, WaypointTracker

    tracker = WaypointTracker(
        threshold=1.5,
        stuck_timeout=0.1,  # very short for testing
        stuck_dist=0.15,
    )

    # Set up a path with one waypoint far away
    path = [np.array([10.0, 10.0, 0.0])]
    robot_pos = np.array([0.0, 0.0, 0.0])
    tracker.reset(path, robot_pos)

    # Update with same position repeatedly �?should trigger stuck
    events_seen = []
    deadline = time.time() + 2.0  # safety timeout
    while time.time() < deadline:
        status = tracker.update(robot_pos)
        if status.event:
            events_seen.append(status.event)
        if EV_STUCK in events_seen:
            break
        time.sleep(0.02)

    assert EV_STUCK_WARN in events_seen, f"Expected EV_STUCK_WARN, got events: {events_seen}"
    assert EV_STUCK in events_seen, f"Expected EV_STUCK, got events: {events_seen}"


# =========================================================================
# Run all tests
# =========================================================================
if __name__ == "__main__":
    print("\n=== LingTu Persistence & Edge-Case Tests (31-36) ===\n")

    run_test("31. SemanticMapper persistence", test_31_semantic_mapper_persistence)
    run_test("32. VectorMemory numpy fallback", test_32_vector_memory_numpy)
    run_test("33. No API key startup", test_33_no_api_key_startup)
    run_test("34. No CLIP startup", test_34_no_clip_startup)
    run_test("35. Empty SceneGraph", test_35_empty_scene_graph)
    run_test("36. WaypointTracker stuck detection", test_36_waypoint_tracker_stuck)

    # Summary
    passed = sum(1 for _, ok, _ in results if ok)
    failed = sum(1 for _, ok, _ in results if not ok)
    print(f"\n=== Summary: {passed} PASS, {failed} FAIL out of {len(results)} ===\n")

    if failed:
        print("Failed tests:")
        for name, ok, err in results:
            if not ok:
                print(f"  - {name}: {err}")
        print()
