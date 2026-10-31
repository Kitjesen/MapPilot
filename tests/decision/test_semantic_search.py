"""Search uses controlled native proposals; actual robot geometry is not simulated here."""

import json
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from decision.modules.llm import LLMResponse
from decision.modules.semantic_planner import SemanticPlannerModule
from nav.services.goals import GoalService
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import NavigationGoalState, NavigationGoalStatus, NavigationState
from runtime.msgs.semantic import Detection3D, SceneGraph, SemanticObservation
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from tests.decision.test_semantic_target_verification import _Commands


@pytest.fixture
def search(monkeypatch):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: clock.now)
    monkeypatch.setattr(SemanticPlannerModule, "_init_backends", lambda self: None)
    module = SemanticPlannerModule(map_query=SimpleNamespace())
    module._goal_resolver = SimpleNamespace(
        maybe_reload_kg=lambda: None,
        fast_resolve=lambda _text, scene, **kw: (
            SimpleNamespace(confidence=1.0, candidate_id="bottle-1", action="navigate")
            if any(obj["id"] == "bottle-1" for obj in json.loads(scene)["objects"]) else None
        ),
    )
    commands = _Commands()
    queries = []

    def query(**kw):
        queries.append(kw)
        return {
            "request_id": "view-query", "available": True, "geometry_exhausted": False, "reason": "",
            "boot_id": kw["boot_id"], "map_id": kw["map_id"], "map_content_epoch": kw["map_content_epoch"],
            "frame_id": kw["frame_id"], "frame_epoch": 9, "reference_z": 0.3,
            "candidates": [{"x": 1.5, "y": 0.0, "z": 0.3, "yaw": 0.0}],
        }

    commands.query_semantic_views = query
    goals = GoalService(command_module="nav.commands")
    goals.on_system_modules({"nav.commands": commands})
    goals.setup()
    module.setup()
    module.on_system_modules({"nav.commands": commands, "nav.goals": goals,
                              "LLMModule": SimpleNamespace(client=SimpleNamespace(supports_vision=True))})
    module.nav_command.subscribe(goals.goal_command._deliver)
    goals.goal_status.subscribe(module.goal_status._deliver)
    goals.task_status.subscribe(module.navigation_goal_status._deliver)
    module.navigation_state._deliver(NavigationState(boot_id="test-nav", sequence=1, map_id="map-a", map_content_epoch=1))
    statuses, requests, threads = [], [], []
    module.planner_status.subscribe(statuses.append)
    module.llm_request.subscribe(requests.append)
    thread_class = threading.Thread

    def thread(**kwargs):
        worker = thread_class(**kwargs)
        threads.append(worker)
        return worker

    monkeypatch.setattr("decision.modules.semantic_planner.threading", SimpleNamespace(
        Thread=thread, Timer=threading.Timer, Event=threading.Event,
    ))
    sequence = 0

    def observe(stamp, *, bottle=False, x=0.0, generation=7, other=False):
        nonlocal sequence
        sequence += 1
        clock.now = stamp
        objects = [Detection3D(id="bottle-1", label="bottle", confidence=0.95, position=Vector3(2.0, 0, 1.0),
                               ts=stamp, bbox_2d=[4, 4, 28, 28])] if bottle else []
        if other:
            objects.append(Detection3D(id="chair-1", label="chair", position=Vector3(2.0, 1.0, 0.5), ts=stamp))
        observation = SemanticObservation(
            sequence=sequence, source_ts=stamp, completed_ts=stamp, source_generation=generation,
            status="positive" if bottle else "negative", map_id="map-a", content_epoch=1,
            robot_pose=PoseStamped(Pose(Vector3(x, 0, 0.3)), frame_id="map", ts=stamp),
            image=Image(np.zeros((32, 32, 3), dtype=np.uint8), format=ImageFormat.BGR, ts=stamp),
            depth=Image(np.full((32, 32), 2.0, dtype=np.float32), format=ImageFormat.DEPTH_F32, ts=stamp),
            camera_info=CameraIntrinsics(fx=32, fy=32, cx=16, cy=16, width=32, height=32),
            map_from_camera=np.array([[0, 0, 1, x], [-1, 0, 0, 0], [0, -1, 0, 1], [0, 0, 0, 1]], dtype=float),
            scene_graph=SceneGraph(objects=objects, ts=stamp, frame_id="map", map_id="map-a", content_epoch=1),
            detections=tuple(objects),
        )
        module.observation._deliver(observation)
        return observation

    def join():
        for worker in tuple(threads):
            worker.join(3)
            assert not worker.is_alive()

    def native(state=NavigationGoalState.REACHED):
        task = module._active_goal
        goals.navigation_goal_status._deliver(NavigationGoalStatus(
            ts=clock.now, frame_id="map", boot_id="test-nav", sequence=sequence + 10, goal_epoch=sequence + 10,
            task_id=task.task_id, request_id=task.request_id, state=state,
        ))

    yield SimpleNamespace(module=module, commands=commands, goals=goals, queries=queries, statuses=statuses,
                          requests=requests, clock=clock, observe=observe, join=join, native=native)
    module.stop()
    join()
    goals.stop()


def start(search):
    search.module.submit_owned_instruction("find a bottle of water", "agent-water")
    search.observe(100.1)
    search.join()


def test_not_visible_search_moves_then_requires_fresh_bottle_and_visual_confirmation(search):
    h = search
    start(h)
    assert len(h.queries) == 1
    assert h.queries[0]["frame_epoch"] == 0 and h.queries[0]["views"] == []
    assert h.module._active_goal.purpose == "search"
    assert h.module._active_goal.pose.z == pytest.approx(0.3)
    assert h.module.owned_instruction_status("agent-water")["success"] is False
    h.clock.now = 100.2
    h.native()
    assert h.module._current_instruction
    assert "COMPLETED" not in h.statuses
    h.observe(100.2, bottle=True, x=1.5)
    h.join()
    assert h.module._active_goal.purpose == "search"
    h.observe(100.3, bottle=True, x=1.5)
    h.join()
    assert h.module._active_goal.purpose == "object"
    assert h.module.owned_instruction_status("agent-water")["state"] != "SEARCHING"
    h.clock.now = 100.4
    h.native()
    assert not h.module.owned_instruction_status("agent-water")["success"]
    for stamp in (100.5, 101.1):
        h.observe(stamp, bottle=True, x=1.5)
        h.join()
        request = h.requests[-1]
        h.module.llm_response._deliver(LLMResponse(request_id=request.request_id, model="controlled-vision",
            text=json.dumps({"target_id": "bottle-1", "verdict": "match", "reason": "controlled image evidence"})))
    assert h.module.owned_instruction_status("agent-water")["success"] is True


def test_native_coverage_exhaustion_is_failure_not_found_water(search):
    h = search
    query = h.commands.query_semantic_views
    h.commands.query_semantic_views = lambda **kw: {**query(**kw), "geometry_exhausted": True, "candidates": []}
    start(h)
    assert not h.commands.goals
    state = h.module.owned_instruction_status("agent-water")
    assert state["terminal"] and not state["success"]
    assert state["state"] == "SEARCH_COVERAGE_EXHAUSTED"


@pytest.mark.parametrize("change", ["cancel", "map", "camera"])
def test_old_native_query_cannot_dispatch_after_context_changes(search, change):
    h = search
    entered, release = threading.Event(), threading.Event()
    query = h.commands.query_semantic_views

    def blocked(**kw):
        entered.set()
        assert release.wait(2)
        return query(**kw)

    h.commands.query_semantic_views = blocked
    h.module.submit_owned_instruction("find a bottle of water", "agent-water")
    h.observe(100.1)
    assert entered.wait(1)
    try:
        if change == "cancel":
            h.module.cancel_owned_instruction("agent-water")
        elif change == "map":
            h.module.navigation_state._deliver(NavigationState(boot_id="test-nav", sequence=2,
                                                               map_id="map-b", map_content_epoch=1))
        else:
            h.observe(100.2, generation=8)
    finally:
        release.set()
    h.join()
    assert not h.commands.goals


def test_bottle_seen_during_search_cancels_only_search_before_approach(search):
    h = search
    start(h)
    previous = h.module._active_goal
    h.observe(100.2, bottle=True)
    h.join()
    assert previous.cancel_request_id
    assert h.module._active_goal is previous
    h.clock.now = 100.3
    h.native(NavigationGoalState.CANCELLED)
    h.observe(100.4, bottle=True)
    h.join()
    assert h.module._active_goal.purpose == "object"


def test_mismatch_resumes_native_search_when_remaining_objects_do_not_match(search):
    h = search
    query = h.commands.query_semantic_views
    h.commands.query_semantic_views = lambda **kw: {
        **query(**kw), "candidates": [{"x": 3.0, "y": 0.0, "z": 0.3, "yaw": 0.0}],
    }
    h.module.submit_owned_instruction("find a bottle of water", "agent-water")
    h.observe(100.1, bottle=True, other=True)
    h.join()
    assert h.module._active_goal.purpose == "object"
    h.clock.now = 100.2
    h.native()
    h.observe(100.3, bottle=True, other=True, x=1.5)
    h.join()
    request = h.requests[-1]
    h.module.llm_response._deliver(LLMResponse(
        request_id=request.request_id, model="controlled-vision",
        text=json.dumps({"target_id": "bottle-1", "verdict": "mismatch", "reason": "not water"}),
    ))
    h.join()
    assert h.module._object_candidates["bottle-1"] == "mismatch"
    assert h.module._active_goal.purpose == "search"
    assert h.module.owned_instruction_status("agent-water")["state"] == "SEARCHING"
    assert not h.module.owned_instruction_status("agent-water")["success"]
