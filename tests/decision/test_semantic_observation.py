"""Object navigation consumes one source-bound snapshot and its invalidations."""

import threading
from dataclasses import replace

from runtime.msgs.nav import NavigationState
from tests.decision.test_semantic_reobservation import alternate_resolver, observe
from tests.decision.test_semantic_target_verification import harness


def test_source_restart_discards_pending_match_and_reports_owned_failure(harness):
    h = harness
    h.module._instruction_owner = "agent-a"
    h.reach()
    h.observe(100.1)
    h.await_request()
    h.reply()
    assert h.module._verification.confirmations == 1
    h.observe(100.7)
    h.await_request()
    previous = h.module._verification
    h.observe(100.8, generation=11, status="unavailable")
    h.reply()
    assert previous.state == "cancelled"
    assert "COMPLETED" not in h.statuses
    assert h.module.owned_instruction_status("agent-a") == {
        "owned": True, "terminal": True, "success": False, "state": "perception_context_changed",
    }
    h.observe(100.9, generation=11)
    assert len(h.goals) == 1
    assert len(h.requests) == 2


def test_source_restart_cancels_approach_before_old_preview_returns(harness):
    h = harness
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    original = h.modules["nav.commands"].preview_plan

    def preview(x, y, z):
        entered.set()
        try:
            assert release.wait(2.0)
            return original(x, y, z)
        finally:
            finished.set()

    h.modules["nav.commands"].preview_plan = preview
    task = h.module._active_goal
    try:
        h.observe(100.1, object_x=3.0)
        assert entered.wait(1.0)
        h.observe(100.2, generation=11, status="unavailable")
        assert task.cancel_request_id
        assert h.module._active_goal is None
    finally:
        release.set()
    assert finished.wait(1.0)
    assert len(h.goals) == 1


def test_unavailable_frame_resets_visual_evidence_without_completing_task(harness):
    h = harness
    h.reach()
    deadline = h.module._verification.deadline
    h.observe(100.1)
    h.await_request()
    h.reply()
    h.observe(100.7)
    h.await_request()
    h.observe(100.8, status="unavailable")
    h.reply()
    assert h.module._verification.confirmations == 0
    assert h.module._current_scene_graph is None
    h.observe(101.3)
    h.await_request()
    h.reply()
    assert "COMPLETED" not in h.statuses
    assert h.module._verification.attempts == 3
    assert h.module._verification.deadline == deadline


def test_old_snapshot_and_old_source_time_do_not_replace_current_scene(harness):
    h = harness
    old = h.observe(100.1)
    current = h.observe(100.3, object_x=2.02)
    h.module.observation._deliver(old)
    h.observe(99.0, object_x=9.0)
    assert h.module._current_scene_graph is current.scene_graph
    assert h.module._observation_image is current.image
    assert h.module._current_robot_pose is current.robot_pose


def test_other_product_session_cannot_consume_current_sequence(harness):
    h = harness
    observation = h.observe(100.1)
    sequence = h.module._latest_observation_sequence
    h.module.observation._deliver(replace(observation, product_session_id="other-run", sequence=1000))
    assert h.module._latest_observation_sequence == sequence
    current = h.observe(100.2)
    assert h.module._current_scene_graph is current.scene_graph


def test_new_map_accepts_fresh_task_and_ignores_old_map(harness):
    h = harness
    old = h.observe(100.1)
    h.module.navigation_state._deliver(NavigationState(boot_id="test-nav", sequence=2, map_id="map-b"))
    current = h.observe(100.2, map_id="map-b")
    h.module.instruction._deliver("find the red chair")
    h.module.observation._deliver(replace(old, sequence=3, source_ts=100.3))
    assert h.module._current_scene_graph is current.scene_graph
    assert h.module._current_instruction == "find the red chair"


def test_semantic_planner_has_only_one_live_perception_input(harness):
    ports = harness.module.ports_in
    assert "observation" in ports
    assert not {"scene_graph", "robot_pose", "observation_image", "detections"} & ports.keys()


def test_camera_restart_discards_alternative_waiting_for_native_planner(harness):
    h = harness
    alternate_resolver(h)
    finished = threading.Event()
    select = h.module._select_observation_goal
    preview = h.modules["nav.commands"].preview_plan

    def selecting(*args):
        try:
            select(*args)
        finally:
            finished.set()

    h.module._select_observation_goal = selecting
    h.modules["nav.commands"].preview_plan = lambda *args: {"reason": "navigation_busy"}
    h.reach()
    observe(h, 100.1, candidates=2)
    h.await_request()
    h.reply("mismatch")
    assert finished.wait(1.0)
    assert h.module._object_candidates["chair-2"] == "waiting_path"
    h.observe(100.2, generation=11, status="unavailable")
    assert h.module._verification is None
    assert h.module._object_candidates == {}
    h.modules["nav.commands"].preview_plan = preview
    h.observe(100.9, generation=11)
    assert len(h.goals) == 1
