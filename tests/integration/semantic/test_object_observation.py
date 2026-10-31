"""Controlled RGB-D reaches object navigation through the production snapshot."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from tests.integration.semantic.test_follow_observation import chain, wait_for

from decision.goals.resolver import GoalResolver
from decision.llm.client import LLMConfig
from decision.modules.semantic_planner import SemanticPlannerModule
from nav.commands.module import Commands
from runtime.msgs.nav import NavigationState


@pytest.fixture
def object_chain(chain, monkeypatch, tmp_path):
    original = chain.detector.detect

    def detect(image, prompt):
        detections = original(image, prompt)
        detections[0].label = "water bottle"
        return detections

    chain.detector.detect = detect
    monkeypatch.setattr(SemanticPlannerModule, "_init_backends", lambda self: None)
    preview = Mock(side_effect=lambda x, y, z, **kwargs: {
        "feasible": True, "start_valid": True, "frame_id": "map",
        "path": [{"x": 0, "y": 0, "z": 0}, {"x": x, "y": y, "z": z}],
    })
    monkeypatch.setattr("nav.commands.module.get_native_navigation_client",
                        lambda **kwargs: SimpleNamespace(preview_plan=preview))
    planner = SemanticPlannerModule(product_session_id="run-a", llm_backend="mock", map_query=SimpleNamespace())
    planner._goal_resolver = GoalResolver(primary_config=LLMConfig(backend="mock"), save_dir=str(tmp_path))
    planner.setup()
    planner.on_system_modules({"nav.commands": Commands()})
    planner.navigation_state._deliver(NavigationState(
        boot_id="nav", sequence=1, map_id="map-a", map_content_epoch=1,
    ))
    chain.perception.observation.subscribe(planner.observation._deliver)
    events = []
    planner.nav_command.subscribe(lambda raw: events.append(json.loads(raw)))
    try:
        # A cold pipeline can expire its first frame; a live camera continues.
        stamp = chain.feed()
        wait_for(lambda: chain.positive(stamp))
        stamp = chain.feed(sequence=2)
        wait_for(lambda: planner._current_scene_graph is not None and planner._current_scene_graph.ts == stamp)
        planner.instruction._deliver("帮我找一瓶水")
        wait_for(lambda: any(event["action"] == "goto" for event in events))
        yield SimpleNamespace(chain=chain, planner=planner, events=events, preview=preview, stamp=stamp)
    finally:
        planner.stop()
        assert planner.nav_command._publish_errors == 0


def test_chinese_object_request_uses_projected_object_and_checked_standoff(object_chain):
    h = object_chain
    target = h.planner._active_goal.target
    observation = h.chain.positive(h.stamp)
    assert target.object_id == observation.scene_graph.objects[0].id
    assert target.label == "water bottle"
    assert target.position[0] == pytest.approx(3.0)
    goal, = [event for event in h.events if event["action"] == "goto"]
    assert goal["x"] == pytest.approx(2.5)
    assert goal["source"] == "semantic"
    assert goal["task_id"] == h.planner._active_goal.task_id
    assert h.preview.call_count == 1
    stamp = h.chain.feed(sequence=3)
    wait_for(lambda: h.planner._current_scene_graph.ts == stamp)
    assert h.planner._observation_image.ts == h.planner._current_robot_pose.ts == stamp
    assert len([event for event in h.events if event["action"] == "goto"]) == 1


def test_camera_restart_invalidates_object_task_before_old_inference_finishes(object_chain):
    h = object_chain
    h.chain.detector.block = True
    try:
        h.chain.feed(sequence=3)
        assert h.chain.detector.entered.wait(2.0)
        h.chain.feed(generation=200, sequence=1, x=10)
        assert any(event["action"] == "cancel" for event in h.events)
        assert h.planner._current_scene_graph is None
        h.chain.detector.block = False
        h.chain.detector.release.set()
        wait_for(lambda: h.planner._current_scene_graph is not None)
        assert h.planner._observation_boundary[0] == 200
        assert len([event for event in h.events if event["action"] == "goto"]) == 1
    finally:
        h.chain.detector.release.set()
