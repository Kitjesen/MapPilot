"""Operator selection must precede native navigation, including Full access chat."""

import asyncio
import json
import threading
import time
from types import SimpleNamespace

import pytest

from decision.modules.visual_servo import VisualServoModule
from decision.vision.candidates import parse_person_review
from runtime.msgs.geometry import PoseStamped, Vector3
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import Detection3D, SemanticObservation
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


def observe(servo, sequence=1, ids=("person", "poster"), generation=1, depth=None, depth_scale=None):
    now = time.time()
    detections = []
    for index, identity in enumerate(ids):
        slot = {"person": 0, "poster": 1}.get(identity, index)
        detections.append(Detection3D(id=identity, label="person", position=Vector3(4, slot, 0),
            bbox_2d=[4 + slot * 36, 4, 30 + slot * 36, 60]))
    servo._on_observation(SemanticObservation(
        product_session_id="session", sequence=sequence, source_ts=now, completed_ts=now,
        source_generation=generation, map_id="map", content_epoch=1, status="positive",
        image=Image(np.zeros((64, 80, 3), dtype=np.uint8), ImageFormat.BGR, ts=now),
        depth=Image(depth, ImageFormat.DEPTH_U16, ts=now) if depth is not None else None,
        depth_scale_m=depth_scale, depth_range_m=(.3, 6.) if depth_scale is not None else None,
        camera_info=CameraIntrinsics(fx=80, fy=80, cx=40, cy=32, width=80, height=64),
        map_from_camera=np.eye(4),
        robot_pose=PoseStamped(frame_id="map"),
        detections=tuple(detections),
    ))


@pytest.fixture
def selection():
    loop = asyncio.new_event_loop()
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    calls = []

    async def chat(messages, **kwargs):
        calls.append(messages)
        return json.dumps([{"target_id": "person", "kind": "person", "reason": "physical person"},
                           {"target_id": "poster", "kind": "non_person", "reason": "printed portrait"}])

    servo = VisualServoModule(product_session_id="session")
    servo.setup()
    servo._vision_client = SimpleNamespace(chat=chat, supports_vision=True, is_available=lambda: True)
    servo._llm_module = SimpleNamespace(_loop=loop, _timeout_sec=2, client=servo._vision_client)
    goals = []
    servo.goal_pose.subscribe(goals.append)
    receipt = servo.start_follow(target_id="person", require_confirmation=True)
    ref = {key: receipt[key] for key in ("follow_task_id", "product_session_id")}
    observe(servo)
    preview = servo.get_follow_selection(**ref)
    deadline = time.monotonic() + 2
    while preview.get("review_state") == "reviewing" and time.monotonic() < deadline:
        time.sleep(.01)
        preview = servo.get_follow_selection(**ref)
    assert preview["review_state"] == "ready"
    yield servo, ref, preview, goals, calls
    servo.stop()
    loop.call_soon_threadsafe(loop.stop)
    thread.join(timeout=2)
    loop.close()


def test_boxes_and_review_stay_bound_to_one_frame_and_do_not_dispatch(selection):
    servo, ref, preview, goals, calls = selection
    assert servo.get_follow_task(**ref)["phase"] == "awaiting_confirmation"
    assert goals == []
    assert not servo._publish_goal_from_3d(np.array([3., 0., 0.]))
    observe(servo, 2)
    assert servo.get_follow_selection(**ref)["observation_sequence"] == preview["observation_sequence"] == 1
    assert len(calls) == 1
    assert calls[0][0]["content"][1]["image_url"]["url"] == preview["image"]
    assert [item["kind"] for item in preview["candidates"]] == ["person", "non_person"]


def wait_review(servo, ref, *, refresh=False):
    preview = servo.get_follow_selection(**ref, refresh=refresh)
    deadline = time.monotonic() + 2
    while preview.get("review_state") == "reviewing" and time.monotonic() < deadline:
        time.sleep(.01)
        preview = servo.get_follow_selection(**ref)
    assert preview["review_state"] == "ready"
    return preview


def test_new_review_receives_depth_evidence_and_skips_cached_non_person(selection):
    servo, ref, _, _, calls = selection
    observe(servo, 2, depth=np.full((64, 80), 4000, np.uint16), depth_scale=.001)
    preview = wait_review(servo, ref, refresh=True)
    sent = json.loads(calls[-1][0]["content"][0]["text"].split("Candidates:\n")[1])
    assert [item["target_id"] for item in sent] == ["person"]
    assert sent[0]["geometry"]["position_m"] == [4, 0, 0]
    assert sent[0]["geometry"]["depth"]["median_m"] == 4
    assert sent[0]["geometry"]["depth"]["valid_fraction"] == 1
    assert preview["candidates"][1]["review_source"] == "track_cache"
    count = len(calls)
    observe(servo, 3, ids=("poster",), depth=np.full((64, 80), 4000, np.uint16), depth_scale=.001)
    preview = wait_review(servo, ref, refresh=True)
    assert len(calls) == count and preview["candidates"][0]["kind"] == "non_person"
    assert not servo._person_reviews.filter(servo._latest_detections)
    assert len(servo._latest_detections) == 1


def test_local_follow_receives_filtered_detections_while_raw_observation_is_retained(selection, monkeypatch):
    servo, ref, preview, _, _ = selection
    seen = []
    prepare = servo._person_tracker.prepare_observation
    def capture(objects, *args, **kwargs):
        seen.append([obj["id"] for obj in objects])
        return prepare(objects, *args, **kwargs)
    monkeypatch.setattr(servo._person_tracker, "prepare_observation", capture)
    assert servo.confirm_follow_target(**ref, target_id="person", observation_sequence=preview["observation_sequence"])["accepted"]
    observe(servo, 2)
    assert seen and all(ids == ["person"] for ids in seen)
    assert {item.id for item in servo._latest_detections} == {"person", "poster"}


@pytest.mark.parametrize("change", ["camera", "map", "unavailable"])
def test_observation_boundary_clears_rejected_tracks(selection, change):
    from dataclasses import replace

    servo, _, _, _, _ = selection
    assert servo._person_reviews.verdict("poster")
    previous = servo._latest_observation
    updates = {"source_generation": 2} if change == "camera" else {"map_id": "another-map"} if change == "map" else {"status": "unavailable"}
    servo._on_observation(replace(previous, sequence=2, source_ts=time.time(), **updates))
    assert servo._person_reviews.verdict("poster") is None


def test_only_reviewed_person_can_be_confirmed_and_then_dispatch(selection):
    servo, ref, preview, goals, _ = selection
    seq = preview["observation_sequence"]
    assert servo.confirm_follow_target(**ref, target_id="poster", observation_sequence=seq)["reason"] == "person_not_verified"
    assert goals == []
    result = servo.confirm_follow_target(**ref, target_id="person", observation_sequence=seq)
    assert result["accepted"] and result["confirmation_required"] is False
    assert len(goals) == 1
    assert servo.get_follow_task(**ref)["confirmed_target_id"] == "person"
    retry = servo.confirm_follow_target(**ref, target_id="person", observation_sequence=seq)
    assert retry["reason"] == "already_confirmed" and len(goals) == 1
    assert servo.confirm_follow_target(**ref, target_id="poster", observation_sequence=seq)["accepted"] is False


@pytest.mark.parametrize("failure", ["old_frame", "session", "gone", "camera_stale", "cancelled", "unknown"])
def test_invalid_confirmation_cannot_dispatch(selection, failure):
    servo, ref, preview, goals, _ = selection
    args = dict(ref, target_id="person", observation_sequence=preview["observation_sequence"])
    if failure == "old_frame":
        args["observation_sequence"] += 1
    elif failure == "session":
        args["product_session_id"] = "old-session"
    elif failure == "gone":
        observe(servo, 2, ids=("poster",))
    elif failure == "camera_stale":
        servo._last_perception_time = 0
    elif failure == "cancelled":
        servo.cancel_follow_task(**ref)
    else:
        preview["candidates"][0]["kind"] = "unknown"
    assert servo.confirm_follow_target(**args)["accepted"] is False
    assert goals == []


@pytest.mark.parametrize("response", ["yes", "[]", '{}', '[{"target_id":"invented","kind":"person"}]',
    '[{"target_id":"person","kind":"person"},{"target_id":"person","kind":"non_person"}]'])
def test_incomplete_or_conflicting_model_answers_cannot_approve_person(response):
    assert parse_person_review(response, ["person"])["person"]["kind"] == "unknown"


def test_invalid_model_label_shape_remains_unknown():
    result = parse_person_review('[{"target_id":"person","kind":[]}]', ["person"])
    assert result["person"]["kind"] == "unknown"


def test_http_confirmation_preserves_lease_and_exact_observation(selection):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes.commands import register_command_routes
    from gateway.services.commands import CommandJournal, ControlLease, run_control_command

    servo, ref, preview, goals, _ = selection
    lease = ControlLease()
    lease.acquire("other-controller", 30)
    gw = SimpleNamespace(_all_modules={"VisualServoModule": servo}, _navigation_state=None,
        _state_lock=threading.RLock(), _lease=lease, _command_journal=CommandJournal(),
        _publish_command_ack=lambda *args, **kwargs: None, push_event=lambda event: None)
    gw._run_control_command = lambda command, body, action, **kwargs: run_control_command(gw, command, body, action, **kwargs)
    app = FastAPI()
    register_command_routes(app, gw)
    route = f"/api/v1/follow/{ref['follow_task_id']}"
    body = {**ref, "target_id": "person", "observation_sequence": preview["observation_sequence"],
            "client_id": "web-dashboard", "request_id": "confirm-a"}
    with TestClient(app) as client:
        observe(servo, 2)
        selection_view = client.get(route + '/selection', params={"product_session_id": ref['product_session_id']}).json()
        assert selection_view["image"] == preview["image"]
        assert client.post(route + '/confirm', json=body).status_code == 403
        assert goals == []
        lease.release("other-controller")
        body["request_id"] = "confirm-b"
        accepted = client.post(route + '/confirm', json=body).json()
        assert accepted["accepted"] and len(goals) == 1
        assert client.post(route + '/confirm', json=body).json()["accepted"] and len(goals) == 1
        assert client.post(route + '/confirm', json={**body, "target_id": "poster"}).status_code == 409
        assert client.post('/api/v1/follow/another/confirm', json=body).status_code == 422
    assert "confirm_follow_target" not in {info.func_name for info in servo.get_skill_infos()}
