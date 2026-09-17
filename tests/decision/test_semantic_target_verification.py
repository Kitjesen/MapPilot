"""Reached native tasks require current, correlated visual evidence to complete."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from decision.modules.llm import LLMModule, LLMResponse
from decision.modules.semantic_planner import SemanticPlannerModule
from nav.services.goals import GoalService
from runtime.msgs.geometry import Pose, PoseStamped, Vector3
from runtime.msgs.nav import (
    NavigationCommandKind,
    NavigationCommandReceipt,
    NavigationGoalState,
    NavigationGoalStatus,
    NavigationState,
)
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.msgs.sensor import Image, ImageFormat


class _Commands:
    def __init__(self):
        self.goals = []

    def preview_plan(self, x, y, z):
        return {"feasible": True, "start_valid": True, "frame_id": "map", "path": [{"x": x, "y": y, "z": z}]}

    def send_goal(self, x, y, z, yaw, *, task_id, request_id, acceptance_radius_m=None):
        self.goals.append({"task_id": task_id, "acceptance_radius_m": acceptance_radius_m})
        return NavigationCommandReceipt(accepted=True, kind=NavigationCommandKind.GOAL,
                                        task_id=task_id, request_id=request_id, reason="", endpoint_timestamp_s=100.0)

    def cancel_task(self, task_id, reason, *, request_id):
        return NavigationCommandReceipt(accepted=True, kind=NavigationCommandKind.TASK_CANCEL,
                                        task_id=task_id, request_id=request_id, reason="", endpoint_timestamp_s=100.0)


@pytest.fixture
def harness(monkeypatch):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr("decision.modules.semantic_planner.time.time", lambda: clock.now)
    monkeypatch.setattr(SemanticPlannerModule, "_init_backends", lambda self: None)
    module = SemanticPlannerModule(llm_backend="mock", map_query=SimpleNamespace())
    module._goal_resolver = SimpleNamespace(
        maybe_reload_kg=lambda: None,
        fast_resolve=Mock(return_value=SimpleNamespace(confidence=1.0, candidate_id="chair-1", action="navigate")),
    )
    module._task_decomposer = module._frontier_scorer = module._action_executor = None
    commands = _Commands()
    service = GoalService(command_module="nav.commands")
    service.on_system_modules({"nav.commands": commands})
    service.setup()
    module.setup()
    modules = {"nav.commands": commands, "nav.goals": service,
               "LLMModule": SimpleNamespace(client=SimpleNamespace(supports_vision=True))}
    module.on_system_modules(modules)
    module.nav_command.subscribe(service.goal_command._deliver)
    service.goal_status.subscribe(module.goal_status._deliver)
    service.task_status.subscribe(module.navigation_goal_status._deliver)
    goals, requests, statuses = [], [], []
    goal_sent, request_sent, completed = threading.Event(), threading.Event(), threading.Event()

    def goal(raw):
        payload = json.loads(raw)
        if payload["action"] == "goto":
            goals.append(payload)
            goal_sent.set()

    def request(req):
        requests.append(req)
        request_sent.set()

    def status(value):
        statuses.append(value)
        if value in {"COMPLETED", "TARGET_VERIFICATION_TIMEOUT", "TARGET_MISMATCH", "TARGET_UNCONFIRMED"}:
            completed.set()

    module.nav_command.subscribe(goal)
    module.llm_request.subscribe(request)
    module.planner_status.subscribe(status)
    module.navigation_state._deliver(NavigationState(boot_id="test-nav", sequence=1, map_id="map-a"))

    def observe(stamp, *, image_stamp=None, pose_stamp=None, object_stamp=None, object_x=2.0, bbox=None, label="chair",
                robot_position=(1.5, 0, 0.3), object_id="chair-1", extra_objects=()):
        clock.now = stamp
        bgr = np.zeros((32, 32, 3), dtype=np.uint8)
        bgr[:, :, 2] = 255
        module.observation_image._deliver(Image(data=bgr, format=ImageFormat.BGR, ts=stamp if image_stamp is None else image_stamp))
        module.robot_pose._deliver(PoseStamped(Pose(Vector3(*robot_position)), frame_id="map", ts=stamp if pose_stamp is None else pose_stamp))
        module.scene_graph._deliver(SceneGraph(frame_id="map", ts=stamp, objects=[Detection3D(
            id=object_id, label=label, confidence=0.95, position=Vector3(object_x, 0, 1.0),
            bbox_2d=[4, 4, 28, 28] if bbox is None else bbox, ts=stamp if object_stamp is None else object_stamp,
        ), *extra_objects]))

    observe(100.0)
    module.instruction._deliver("find the red chair")
    assert goal_sent.wait(1.0)
    assert module._active_goal.state == "accepted"

    native_sequence = 0

    def reach(state=NavigationGoalState.REACHED):
        nonlocal native_sequence
        native_sequence += 1
        task = module._active_goal
        service.navigation_goal_status._deliver(NavigationGoalStatus(
            ts=clock.now, frame_id="map", boot_id="test-nav", sequence=native_sequence, goal_epoch=native_sequence,
            task_id=task.task_id, request_id=task.request_id, state=state,
        ))

    def await_request():
        assert request_sent.wait(1.0), "fresh evidence did not reach the LLM request port"
        request_sent.clear()
        return requests[-1]

    def reply(verdict="match", *, request_id=None, error=""):
        req = requests[-1]
        module.llm_response._deliver(LLMResponse(
            request_id=request_id or req.request_id, model="test-vision",
            text=json.dumps({"target_id": json.loads(req.messages[1]["content"][0]["text"])["target_id"],
                             "verdict": verdict, "reason": "test visual evidence"}), error=error,
        ))

    yield SimpleNamespace(module=module, modules=modules, clock=clock, observe=observe, reach=reach,
                          reply=reply, await_request=await_request, goals=goals, requests=requests,
                          statuses=statuses, completed=completed, goal_sent=goal_sent)
    module.stop()
    service.stop()
    assert all(port._publish_errors == 0 for owner in (module, service) for port in owner.ports_out.values())


def test_native_arrival_is_followed_by_two_distinct_visual_confirmations(harness):
    h = harness
    h.reach()
    assert h.statuses[-1] == "VERIFYING_TARGET"
    assert json.loads(h.module.get_planner_status())["state"] == "VERIFYING_TARGET"
    assert h.module._active_goal.state == "reached"
    assert h.requests == []
    h.observe(100.1)
    first = h.await_request()
    h.reply()
    assert "COMPLETED" not in h.statuses
    h.observe(100.1)
    h.observe(100.2)
    assert len(h.requests) == 1
    h.observe(100.7)
    second = h.await_request()
    assert second.request_id != first.request_id
    h.reply(request_id=first.request_id)
    assert h.module._verification.confirmations == 1
    h.reply()
    assert h.statuses[-1] == "COMPLETED"
    result = json.loads(h.module.get_planner_status())["navigation_goal"]["verification"]
    assert result["state"] == "confirmed"
    assert [item["timestamp"] for item in result["evidence"]] == [100.1, 100.7]
    assert len(h.goals) == 1


@pytest.mark.parametrize("bad", [
    {"image_stamp": 99.0}, {"image_stamp": 100.15}, {"pose_stamp": 100.15},
    {"object_stamp": 100.0}, {"object_x": 3.0}, {"bbox": []},
    {"bbox": [4, 4, 4, 4]}, {"label": "table"},
])
def test_only_synchronized_current_candidate_images_are_sent(harness, bad):
    h = harness
    h.reach()
    h.observe(100.1, **bad)
    assert h.requests == []
    assert h.module._verification.attempts == 0
    assert len(h.goals) == 1


def test_uncertain_is_not_negative_evidence_and_attempts_are_bounded(harness):
    h = harness
    h.modules["nav.commands"].preview_plan = lambda *args: {"feasible": False}
    h.reach()
    for stamp in [100.1, 100.7, 101.3]:
        h.observe(stamp)
        h.await_request()
        h.reply("uncertain")
    assert h.completed.wait(1.0)
    assert h.statuses[-1] == "TARGET_UNCONFIRMED"
    assert h.module._verification.confirmations == 0
    assert h.module._verification.state == "uncertain"
    h.observe(102.0)
    assert len(h.requests) == 3
    assert len(h.goals) == 1


def test_new_frame_received_during_model_call_is_used_after_reply(harness):
    h = harness
    h.reach()
    h.observe(100.1)
    h.await_request()
    h.observe(100.7)
    assert len(h.requests) == 1
    h.reply()
    h.await_request()
    assert len(h.requests) == 2
    h.reply()
    assert h.statuses[-1] == "COMPLETED"


def test_visible_mismatch_finishes_without_reporting_success(harness):
    h = harness
    h.reach()
    h.observe(100.1)
    h.await_request()
    h.reply("mismatch")
    h.observe(101.0)
    assert h.statuses[-1] == "TARGET_MISMATCH"
    assert len(h.requests) == len(h.goals) == 1


@pytest.mark.parametrize("change", ["stop", "instruction", "map", "restart"])
def test_late_model_result_cannot_complete_superseded_task(harness, change):
    h = harness
    h.reach()
    h.observe(100.1)
    h.await_request()
    verification = h.module._verification
    if change == "stop":
        h.module.stop()
    elif change == "instruction":
        h.module.instruction._deliver("follow person")
    else:
        h.module.navigation_state._deliver(NavigationState(
            boot_id="other-nav" if change == "restart" else "test-nav", sequence=2,
            map_id="map-b" if change == "map" else "map-a",
        ))
    before = list(h.statuses)
    h.reply()
    assert h.statuses == before
    assert verification.state == "cancelled"


@pytest.mark.parametrize("change", ["moved", "expired", "missing"])
def test_positive_reply_requires_the_target_to_still_be_observed(harness, change):
    h = harness
    h.reach()
    h.observe(100.1)
    h.await_request()
    if change == "moved":
        h.observe(100.2, object_x=3.0)
    elif change == "expired":
        h.clock.now = 102.0
    else:
        h.module.scene_graph._deliver(SceneGraph(ts=100.2, frame_id="map"))
    h.reply()
    assert h.module._verification.confirmations == 0
    assert h.module._verification.evidence[-1]["verdict"] == "uncertain"
    assert "COMPLETED" not in h.statuses


def test_model_failure_is_explicit_and_does_not_fall_back_to_detection_score(harness):
    h = harness
    h.reach()
    h.observe(100.1)
    h.await_request()
    h.reply(error="service unavailable")
    assert h.statuses[-1] == "TARGET_VERIFICATION_UNAVAILABLE"
    assert len(h.goals) == 1


def test_nonvision_backend_cannot_verify_objects(harness):
    h = harness
    h.modules["LLMModule"].client.supports_vision = False
    h.reach()
    assert h.statuses[-1] == "TARGET_VERIFICATION_UNAVAILABLE"
    assert h.module._verification.reason == "vision_model_unavailable"


def test_camera_disconnect_times_out_without_waiting_for_another_frame(harness):
    h = harness
    h.module._verification_timeout_s = 0.03
    h.reach()
    assert h.completed.wait(1.0)
    assert h.statuses[-1] == "TARGET_VERIFICATION_TIMEOUT"
    assert len(h.goals) == 1


def test_cancel_during_image_preparation_does_not_send_an_old_request(harness, monkeypatch):
    h = harness
    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    original = h.module._publish_verification_request
    def delayed(*args):
        entered.set()
        try:
            assert release.wait(1.0)
            original(*args)
        finally:
            finished.set()
    monkeypatch.setattr(h.module, "_publish_verification_request", delayed)
    h.reach()
    try:
        h.observe(100.1)
        assert entered.wait(1.0)
        h.module.instruction._deliver("follow person")
    finally:
        release.set()
        assert finished.wait(1.0)
    assert h.requests == []


def test_reply_after_deadline_cannot_restore_confirmation(harness):
    h = harness
    h.module._verification_timeout_s = 0.1
    h.reach()
    h.observe(100.1)
    h.await_request()
    assert h.completed.wait(1.0)
    h.reply()
    assert h.statuses[-1] == "TARGET_VERIFICATION_TIMEOUT"
    assert h.module._verification.confirmations == 0


def test_real_llm_module_routes_image_request_and_response(harness):
    h = harness
    seen = []
    class VisionClient:
        supports_vision = True
        async def chat(self, messages, **kwargs):
            seen.append(messages)
            return '{"target_id":"chair-1","verdict":"match","reason":"candidate matches"}'
        async def close(self):
            pass

    llm = LLMModule(backend="mock")
    llm.setup()
    llm._client = VisionClient()
    h.modules["LLMModule"] = llm
    h.module.on_system_modules(h.modules)
    h.module.llm_request.subscribe(llm.request._deliver)
    llm.response.subscribe(h.module.llm_response._deliver)
    replied = threading.Event()
    llm.response.subscribe(lambda msg: replied.set())
    try:
        h.reach()
        h.observe(100.1)
        assert replied.wait(1.0)
        replied.clear()
        assert h.module._verification.confirmations == 1
        h.observe(100.7)
        assert h.completed.wait(1.0)
        assert h.statuses[-1] == "COMPLETED"
        assert len(seen) == 2
        assert len(seen[0][1]["content"]) == 3
    finally:
        llm.stop()
