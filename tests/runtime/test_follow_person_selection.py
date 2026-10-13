"""Tests for description-based person following.

Covers the "person in red" selection path added across:
  - CLIPEncoder.encode_image (interface-gap fix)
  - PersonTracker.select_by_clip / select_target_with_vlm (CLIP + multimodal VLM)
  - VisualServoModule vision-client discovery + follow target selection
  - SemanticPlannerModule follow-intent routing
"""

import asyncio
import threading
import time

import numpy as np

from decision.modules.semantic_planner import SemanticPlannerModule
from decision.modules.visual_servo import VisualServoModule
from decision.vision.person import PersonTracker, verify_candidate
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D
from runtime.msgs.sensor import Image, ImageFormat

# ── helpers ──────────────────────────────────────────────────────────────────


def _marker_crop(marker: int, size: int = 16) -> np.ndarray:
    """A crop whose every pixel equals `marker`, so a fake encoder can identify it."""
    return np.full((size, size, 3), marker, dtype=np.uint8)


class _FakeClip:
    """Fake CLIP encoder: text vector fixed, image vector keyed by crop marker."""

    def __init__(self, text_vec, marker_to_vec):
        self._t = np.asarray(text_vec, dtype=float)
        self._m = {float(k): np.asarray(v, dtype=float) for k, v in marker_to_vec.items()}

    def encode_text(self, texts):
        return np.array([self._t])

    def encode_image(self, crop):
        return self._m[float(np.asarray(crop).reshape(-1)[0])]


class _TextOnlyClip:
    """mobileclip-like encoder: text only, no image encoding."""

    def encode_text(self, texts):
        return np.array([[1.0, 0.0]])


class _ImageOnlyClip:
    def encode_image(self, crop):
        return np.array([1.0, 0.0])


class _FakeDino:
    def encode(self, crop):
        return np.array([1.0, 0.0])


def _vision_follow(
    response: str | Exception,
    release: threading.Event | None = None,
    *,
    timeout_sec: float = 1.0,
):
    started = threading.Event()

    class Client:
        supports_vision = True
        calls = 0

        @staticmethod
        def is_available():
            return True

        async def chat(self, messages, temperature=None):
            self.calls += 1
            started.set()
            while release is not None and not release.is_set():
                await asyncio.sleep(0.001)
            if isinstance(response, Exception):
                raise response
            return response

    loop = asyncio.new_event_loop()
    loop_thread = threading.Thread(target=loop.run_forever)
    loop_thread.start()

    class LLM:
        client = Client()
        _loop = loop
        _timeout_sec = timeout_sec

    vs = VisualServoModule()
    vs.on_system_modules({"LLMModule": LLM()})
    vs._mode = "follow"
    vs._target_label = "person in red"
    vs._follow_select_method = "clip"
    vs._person_tracker.set_dino_encoder(_FakeDino())
    vs._latest_bgr = np.ones((32, 32, 3), dtype=np.uint8)
    vs._person_tracker.lock_target(
        _person("original", "person", 4.0, [0, 0, 16, 16]) | {"ts": 1.0},
        vs._latest_bgr[:16, :16],
    )
    return vs, LLM.client, started, loop, loop_thread


def _offer_returning_identity(
    vs: VisualServoModule,
    candidate_id: str = "returned",
    timestamps=(2.0, 3.0, 4.0),
) -> None:
    for timestamp in timestamps:
        vs._latest_bgr_ts = timestamp
        vs._latest_detection_ts = timestamp
        vs._latest_detections = [
            Detection3D(
                id=candidate_id,
                label="person",
                confidence=0.9,
                position=Vector3(4.1, 0.0, 0.0),
                bbox_2d=[0, 0, 16, 16],
                ts=timestamp,
            )
        ]
        vs._latest_bgr_ts = 5.0
        vs._latest_detection_ts = 5.0
        vs._tick_follow()


def _close_vision_loop(loop, loop_thread) -> None:
    loop.call_soon_threadsafe(loop.stop)
    loop_thread.join(timeout=1.0)
    loop.close()


# ── CLIPEncoder.encode_image (interface-gap fix) ──────────────────────────────


def test_clip_encoder_has_encode_image():
    from perception.encoding.clip_encoder import CLIPEncoder

    enc = CLIPEncoder()  # model not loaded
    assert hasattr(enc, "encode_image")
    out = enc.encode_image(np.zeros((12, 12, 3), dtype=np.uint8))
    # Model unavailable → empty feature, but no AttributeError (the bug we fixed).
    assert hasattr(out, "size")
    assert out.size == 0


def test_person_tracker_rejects_an_image_only_encoder():
    tracker = PersonTracker()
    tracker.set_clip_encoder(_ImageOnlyClip())
    assert tracker.has_image_selector is False


# ── PersonTracker.select_by_clip ──────────────────────────────────────────────


def test_select_by_clip_picks_matching_person():
    tracker = PersonTracker()
    tracker.set_clip_encoder(_FakeClip([1.0, 0.0], {1: [1.0, 0.0], 2: [0.0, 1.0]}))
    crops = [_marker_crop(1), _marker_crop(2)]
    objs = [
        {"id": "red", "position": [1, 0, 0], "bbox": [0, 0, 10, 10]},
        {"id": "blue", "position": [2, 0, 0], "bbox": [0, 0, 10, 10]},
    ]

    idx = tracker.select_by_clip("person in red", crops, objs)
    assert idx == 0
    assert tracker._person is not None
    assert tracker._person.obj_id == "red"


def test_select_by_clip_picks_second_when_description_matches_it():
    tracker = PersonTracker()
    # text aligns with the second crop's vector
    tracker.set_clip_encoder(_FakeClip([0.0, 1.0], {1: [1.0, 0.0], 2: [0.0, 1.0]}))
    crops = [_marker_crop(1), _marker_crop(2)]
    objs = [{"id": "red", "position": [1, 0, 0]}, {"id": "blue", "position": [2, 0, 0]}]

    assert tracker.select_by_clip("person in blue", crops, objs) == 1
    assert tracker._person.obj_id == "blue"


def test_select_by_clip_no_encoder_returns_minus1():
    tracker = PersonTracker()  # no clip encoder
    assert tracker.select_by_clip("anyone", [_marker_crop(1)], [{"id": "a"}]) == -1


def test_select_by_clip_textonly_encoder_returns_minus1():
    """mobileclip (no encode_image) → cannot score crops → -1 (caller falls to VLM)."""
    tracker = PersonTracker()
    tracker.set_clip_encoder(_TextOnlyClip())
    idx = tracker.select_by_clip("person in red", [_marker_crop(1)], [{"id": "a"}])
    assert idx == -1


# ── PersonTracker.select_target_with_vlm (multimodal) ─────────────────────────


def test_select_target_with_vlm_picks_response():
    tracker = PersonTracker()
    crops = [_marker_crop(1), _marker_crop(2)]
    objs = [
        {"id": "p1", "position": [1, 0, 0], "bbox": [0, 0, 10, 10]},
        {"id": "p2", "position": [2, 0, 0], "bbox": [0, 0, 10, 10]},
    ]
    captured = {}

    async def mock_vlm(messages):
        captured["messages"] = messages
        return "选 2 号"

    idx = asyncio.run(tracker.select_target_with_vlm("person in red", crops, objs, mock_vlm))
    assert idx == 1
    assert tracker._person is not None
    assert tracker._person.obj_id == "p2"

    # The VLM was given OpenAI-style multimodal messages (text first).
    msgs = captured["messages"]
    assert isinstance(msgs, list) and msgs[0]["role"] == "user"
    content = msgs[0]["content"]
    assert isinstance(content, list)
    assert content[0]["type"] == "text"


def test_build_vlm_select_messages_structure():
    msgs = PersonTracker._build_vlm_select_messages("pick one", [_marker_crop(1), _marker_crop(2)])
    assert len(msgs) == 1 and msgs[0]["role"] == "user"
    content = msgs[0]["content"]
    assert content[0] == {"type": "text", "text": "pick one"}
    # If image encoding is available, each crop adds an image_url entry.
    image_entries = [c for c in content if c.get("type") == "image_url"]
    assert len(image_entries) in (0, 2)  # 0 when cv2/PIL absent, else one per crop


def test_identity_verifier_orders_original_trusted_and_temporal_evidence(monkeypatch):
    from decision.vision import vlm_scene

    monkeypatch.setattr(
        vlm_scene,
        "encode_image_b64",
        lambda crop: f"marker-{int(np.asarray(crop).reshape(-1)[0])}",
    )
    captured = {}

    async def mock_vlm(messages):
        captured["messages"] = messages
        return "MATCH"

    verdict, candidate_id = asyncio.run(
        verify_candidate(
            "person in red",
            _marker_crop(60),
            7,
            mock_vlm,
            reference_crop_bgr=_marker_crop(10),
            reference_crops_bgr=[_marker_crop(20), _marker_crop(30)],
            candidate_crops_bgr=[
                _marker_crop(40),
                _marker_crop(50),
                _marker_crop(60),
            ],
        )
    )

    assert (verdict, candidate_id) == ("MATCH", 7)
    content = captured["messages"][0]["content"]
    prompt = content[0]["text"]
    assert "original target crop is the identity anchor" in prompt.lower()
    assert "latest candidate frame" in prompt.lower()
    assert "clothing or carried-item evidence conflicts" in prompt.lower()
    assert "reply exactly unknown" in prompt.lower()
    labels = [entry["text"] for entry in content if entry["type"] == "text"][1:]
    assert labels == [
        "Original target crop (identity anchor):",
        "Additional trusted target view 1:",
        "Additional trusted target view 2:",
        "Candidate frame 1 of 3 (earliest):",
        "Candidate frame 2 of 3:",
        "Candidate frame 3 of 3 (latest; judge this person):",
    ]
    urls = [
        entry["image_url"]["url"]
        for entry in content
        if entry["type"] == "image_url"
    ]
    assert urls == [
        "data:image/jpeg;base64,marker-10",
        "data:image/jpeg;base64,marker-20",
        "data:image/jpeg;base64,marker-30",
        "data:image/jpeg;base64,marker-40",
        "data:image/jpeg;base64,marker-50",
        "data:image/jpeg;base64,marker-60",
    ]


# ── VisualServoModule discovery + selection ───────────────────────────────────


def test_vs_discovers_vision_client_after_module_setup():
    vs = VisualServoModule()

    class OpenAIClient:
        supports_vision = True

        @staticmethod
        def is_available():
            return True

        async def chat(self, messages, temperature=None):
            return "1"

    class _LLM:
        client = None

    llm = _LLM()
    vs.on_system_modules({"LLMModule": llm})
    assert vs.can_select_follow_target() is False

    llm.client = OpenAIClient()
    assert vs.can_select_follow_target() is True
    assert vs._vision_client is not None
    statuses = []
    vs.servo_status._add_callback(statuses.append)
    vs.start()
    try:
        assert statuses[-1]["follow_available"] is True
    finally:
        vs.stop()


def test_vs_rejects_text_only_llm():
    vs = VisualServoModule()

    class MoonshotClient:
        supports_vision = False

        async def chat(self, messages, temperature=None):
            return "1"

    class _LLM:
        client = MoonshotClient()

    vs.on_system_modules({"LLMModule": _LLM()})
    assert vs._vision_client is None


def _person(obj_id, label, x, bbox):
    return {"id": obj_id, "label": label, "position": [x, 0, 0], "bbox": bbox, "confidence": 0.9}


def test_vs_try_select_follow_locks_via_clip():
    vs = VisualServoModule()
    vs._person_tracker.set_clip_encoder(
        _FakeClip([1.0, 0.0], {1: [1.0, 0.0], 2: [0.0, 1.0]})
    )
    vs._target_label = "person in red"
    vs._follow_select_pending = True

    bgr = np.zeros((40, 40, 3), dtype=np.uint8)
    bgr[0:20, 0:20] = 1  # "red" person region (marker 1)
    bgr[0:20, 20:40] = 2  # "blue" person region (marker 2)
    vs._latest_bgr = bgr
    vs._latest_bgr_ts = 1.0
    vs._latest_detection_ts = 1.0

    scene_objects = [
        _person("red", "person", 1.0, [0, 0, 20, 20]),
        _person("blue", "person", 2.0, [20, 0, 40, 20]),
    ]
    vs._try_select_follow_target(scene_objects)

    assert vs._follow_select_pending is False
    assert vs._follow_select_method == "clip"
    assert vs._person_tracker._person.obj_id == "red"
    np.testing.assert_array_equal(vs._follow_reference_crop, bgr[0:20, 0:20])


def test_vs_does_not_replace_reference_without_vision_confirmation():
    vs = VisualServoModule()
    reference = np.full((8, 8, 3), 77, dtype=np.uint8)
    vs._follow_reference_crop = reference.copy()
    vs._target_label = "person in red"
    vs._follow_select_pending = True
    vs._latest_bgr = np.full((40, 40, 3), 123, dtype=np.uint8)
    vs._latest_bgr_ts = 1.0
    vs._latest_detection_ts = 1.0

    def unexpected_clip_selection(*_args):
        raise AssertionError("CLIP must not replace an established identity reference")

    vs._person_tracker.select_by_clip = unexpected_clip_selection
    vs._try_select_follow_target(
        [_person("other", "person", 1.0, [0, 0, 20, 20])],
        single_person_confirmed=True,
    )

    assert not vs._person_tracker.target_selected
    assert vs._follow_select_pending is False
    assert vs._follow_select_method == "unavailable"
    np.testing.assert_array_equal(vs._follow_reference_crop, reference)


def test_vs_rejects_descriptive_follow_without_clip_or_vlm():
    vs = VisualServoModule()  # no encoder, no vision client
    vs._target_label = "person in red"
    vs._follow_select_pending = True
    bgr = np.zeros((40, 40, 3), dtype=np.uint8)
    bgr[0:20, 0:20] = 1
    vs._latest_bgr = bgr
    vs._latest_bgr_ts = 1.0
    vs._latest_detection_ts = 1.0

    vs._try_select_follow_target(
        [
            _person("a", "person", 1.0, [0, 0, 20, 20]),
            _person("b", "person", 2.0, [20, 0, 40, 20]),
        ]
    )

    assert vs._follow_select_pending is False
    assert vs._follow_select_method == "unavailable"
    assert vs._person_tracker.target_selected is False


def test_vs_follow_command_arms_selection():
    vs = VisualServoModule()
    vs._follow_reference_crop = np.ones((4, 4, 3), dtype=np.uint8)
    receipt = vs.start_follow(description="person in red")
    assert receipt["accepted"] is True
    assert vs._mode == "follow"
    assert vs._target_label == "person in red"
    assert vs._follow_select_pending is True
    assert vs._follow_reference_crop is None

    vs._follow_reference_crop = np.ones((4, 4, 3), dtype=np.uint8)
    cancel = vs.cancel_follow_task(
        receipt["follow_task_id"], receipt["product_session_id"]
    )
    assert cancel["accepted"] is True
    assert vs._follow_reference_crop is None


def test_stop_and_changed_target_clear_the_identity_reference():
    vs = VisualServoModule()
    vs._person_tracker.set_dino_encoder(_FakeDino())
    person = _person("target", "person", 1.0, [0, 0, 16, 16])
    crop = _marker_crop(1)

    first = vs.start_follow(description="person in red")
    vs._person_tracker.lock_target(person, crop)
    assert vs._person_tracker.has_identity_reference is True

    vs.cancel_follow_task(first["follow_task_id"], first["product_session_id"])
    second = vs.start_follow(description="person in blue")
    assert second["accepted"] is True
    assert vs._person_tracker.has_identity_reference is False

    vs._person_tracker.lock_target(person, crop)
    assert vs._person_tracker.has_identity_reference is True
    vs.cancel_follow_task(second["follow_task_id"], second["product_session_id"])
    assert vs._person_tracker.has_identity_reference is False


def test_vlm_match_authorizes_return_then_next_frame_resumes_native_goal():
    vs, _client, _started, loop, loop_thread = _vision_follow("MATCH")
    resolved = threading.Event()
    original_resolve = vs._person_tracker.resolve_identity

    def resolve(candidate_id, source_ts, verdict):
        result = original_resolve(candidate_id, source_ts, verdict)
        resolved.set()
        return result

    vs._person_tracker.resolve_identity = resolve
    goals = []
    vs.goal_pose._add_callback(goals.append)
    try:
        _offer_returning_identity(vs)
        assert resolved.wait(timeout=1.0)
        assert goals == []
        assert vs._person_tracker.status()["dino_event"] == "identity_verified"

        vs._latest_detections = [
            Detection3D(
                id="returned",
                label="person",
                confidence=0.9,
                position=Vector3(4.2, 0.0, 0.0),
                bbox_2d=[0, 0, 16, 16],
                ts=5.0,
            )
        ]
        vs._latest_bgr_ts = 5.0
        vs._latest_detection_ts = 5.0
        vs._tick_follow()

        assert len(goals) == 1
        assert vs._target_visible is True
        assert vs._person_tracker.status()["id"] == "returned"
    finally:
        _close_vision_loop(loop, loop_thread)


def test_host_identity_verification_passes_multiview_temporal_evidence(monkeypatch):
    captured = {}
    completed = threading.Event()

    async def fake_verify(
        description,
        candidate_crop_bgr,
        candidate_id,
        _chat_fn,
        *,
        reference_crop_bgr=None,
        reference_crops_bgr=None,
        candidate_crops_bgr=None,
    ):
        captured.update(
            description=description,
            candidate_id=candidate_id,
            candidate_crop=np.asarray(candidate_crop_bgr).copy(),
            reference_crop=np.asarray(reference_crop_bgr).copy(),
            reference_crops=[np.asarray(crop).copy() for crop in reference_crops_bgr],
            candidate_crops=[np.asarray(crop).copy() for crop in candidate_crops_bgr],
        )
        return "UNKNOWN", None

    monkeypatch.setattr("decision.modules.visual_servo.verify_candidate", fake_verify)
    vs, _client, _started, loop, loop_thread = _vision_follow("UNKNOWN")
    original_worker = vs._run_identity_verification_sync

    def worker(*args):
        try:
            original_worker(*args)
        finally:
            completed.set()

    vs._run_identity_verification_sync = worker
    candidate = {
        "object": {"id": "returned", "ts": 4.0},
        "crop": _marker_crop(60),
        "reference_crop": _marker_crop(10),
        "reference_crops": [_marker_crop(20), _marker_crop(30)],
        "crops": [_marker_crop(40), _marker_crop(50), _marker_crop(60)],
        "crop_timestamps": [3.8, 3.9, 4.0],
    }
    try:
        vs._start_identity_verification(candidate)
        assert completed.wait(timeout=1.0)

        assert captured["description"] == "person in red"
        assert captured["candidate_id"] == "returned"
        assert int(captured["candidate_crop"].reshape(-1)[0]) == 60
        assert int(captured["reference_crop"].reshape(-1)[0]) == 10
        assert [int(crop.reshape(-1)[0]) for crop in captured["reference_crops"]] == [20, 30]
        assert [int(crop.reshape(-1)[0]) for crop in captured["candidate_crops"]] == [40, 50, 60]
        assert vs._person_tracker.status()["id"] == "original"
    finally:
        _close_vision_loop(loop, loop_thread)


def test_vlm_other_keeps_wrong_return_candidate_blocked():
    vs, client, _started, loop, loop_thread = _vision_follow("OTHER")
    resolved = threading.Event()
    original_resolve = vs._person_tracker.resolve_identity

    def resolve(candidate_id, source_ts, verdict):
        result = original_resolve(candidate_id, source_ts, verdict)
        resolved.set()
        return result

    vs._person_tracker.resolve_identity = resolve
    goals = []
    vs.goal_pose._add_callback(goals.append)
    try:
        _offer_returning_identity(vs, "wrong-person")
        assert resolved.wait(timeout=1.0)
        assert vs._person_tracker.status()["dino_event"] == "identity_rejected"

        vs._latest_detections = [
            Detection3D(
                id="wrong-person",
                label="person",
                confidence=0.9,
                position=Vector3(4.2, 0.0, 0.0),
                bbox_2d=[0, 0, 16, 16],
                ts=5.0,
            )
        ]
        vs._tick_follow()

        assert goals == []
        assert vs._target_visible is False
        assert vs._person_tracker.status()["id"] == "original"
        assert client.calls == 1
    finally:
        _close_vision_loop(loop, loop_thread)


def _assert_identity_verification_failure_becomes_unknown(
    response: Exception,
    *,
    release: threading.Event | None = None,
    timeout_sec: float = 1.0,
) -> None:
    vs, _client, _started, loop, loop_thread = _vision_follow(
        response,
        release,
        timeout_sec=timeout_sec,
    )
    completed = threading.Event()
    original_worker = vs._run_identity_verification_sync
    original_status = vs._person_tracker.status()

    def worker(*args):
        try:
            original_worker(*args)
        finally:
            completed.set()

    vs._run_identity_verification_sync = worker
    try:
        _offer_returning_identity(vs)
        assert completed.wait(timeout=1.0)

        status = vs._person_tracker.status()
        assert vs._person_tracker.identity_candidate() is None
        assert status["dino_event"] == "identity_unknown"
        assert status["id"] == original_status["id"]
        assert status["position"] == original_status["position"]

        # Isolate the tracker's source-time retry contract from another Host call.
        vs._vision_client = None
        _offer_returning_identity(vs, timestamps=(4.1, 4.15, 4.2))
        assert vs._person_tracker.identity_candidate() is None
        assert vs._person_tracker.status()["dino_event"] == "verification_backoff"

        _offer_returning_identity(vs, timestamps=(4.35, 4.4, 4.45))
        pending = vs._person_tracker.identity_candidate()
        assert pending is not None
        assert pending["object"]["id"] == "returned"
        assert pending["object"]["ts"] == 4.45
        status = vs._person_tracker.status()
        assert status["id"] == original_status["id"]
        assert status["position"] == original_status["position"]
    finally:
        if release is not None:
            release.set()
        _close_vision_loop(loop, loop_thread)


def test_identity_verification_exception_becomes_unknown_with_backoff():
    _assert_identity_verification_failure_becomes_unknown(RuntimeError("vision failed"))


def test_identity_verification_timeout_becomes_unknown_with_backoff():
    _assert_identity_verification_failure_becomes_unknown(
        TimeoutError("vision timed out"),
        release=threading.Event(),
        timeout_sec=0.02,
    )


def test_late_identity_match_after_stop_cannot_authorize_motion():
    release = threading.Event()
    vs, _client, started, loop, loop_thread = _vision_follow("MATCH", release)
    resolve_calls = []
    completed = threading.Event()
    original_worker = vs._run_identity_verification_sync
    vs._person_tracker.resolve_identity = lambda *args: resolve_calls.append(args)

    def worker(*args):
        try:
            original_worker(*args)
        finally:
            completed.set()

    vs._run_identity_verification_sync = worker
    goals = []
    vs.goal_pose._add_callback(goals.append)
    try:
        _offer_returning_identity(vs)
        assert started.wait(timeout=1.0)

        vs._on_servo_target("stop")
        release.set()
        assert completed.wait(timeout=1.0)

        assert resolve_calls == []
        assert goals == []
        assert vs._mode == "idle"
        assert vs._person_tracker.target_selected is False
    finally:
        release.set()
        _close_vision_loop(loop, loop_thread)


def test_late_vlm_verification_cannot_lock_a_previous_follow_intent():
    verification_started = threading.Event()
    release = threading.Event()

    class OpenAIClient:
        supports_vision = True

        def __init__(self):
            self.calls = 0

        @staticmethod
        def is_available():
            return True

        async def chat(self, messages, temperature=None):
            self.calls += 1
            if self.calls == 1:
                return "1"
            verification_started.set()
            while not release.is_set():
                await asyncio.sleep(0.001)
            return "MATCH"

    loop = asyncio.new_event_loop()
    loop_thread = threading.Thread(target=loop.run_forever)
    loop_thread.start()

    class _LLM:
        client = OpenAIClient()
        _loop = loop
        _timeout_sec = 1.0

    vs = VisualServoModule()
    try:
        vs.on_system_modules({"LLMModule": _LLM()})
        first = vs.start_follow(description="person in red")
        generation = vs._intent_generation
        persons = [_person("red", "person", 2.0, [0, 0, 30, 30])]
        crops = [np.ones((30, 30, 3), dtype=np.uint8)]
        vs._follow_select_running = True
        worker = threading.Thread(
            target=vs._run_vlm_select_sync,
            args=("person in red", crops, persons, generation),
        )
        worker.start()
        assert verification_started.wait(timeout=1.0)

        cancel = vs.cancel_follow_task(
            first["follow_task_id"], first["product_session_id"]
        )
        assert cancel["accepted"] is True
        second = vs.start_follow(description="person in blue")
        assert second["accepted"] is True
        release.set()
        worker.join(timeout=1.0)

        assert not worker.is_alive()
        assert vs._target_label == "person in blue"
        assert vs._person_tracker.target_selected is False
        assert vs._follow_select_pending is True
    finally:
        loop.call_soon_threadsafe(loop.stop)
        loop_thread.join(timeout=1.0)
        loop.close()


def test_vs_client_supports_vision():
    assert VisualServoModule._client_supports_vision(None) is False

    class Moonshot:
        supports_vision = False

        def chat(self): ...

    class OpenAI:
        supports_vision = True

        def chat(self): ...

    assert VisualServoModule._client_supports_vision(Moonshot()) is False
    assert VisualServoModule._client_supports_vision(OpenAI()) is True


# ── SemanticPlanner follow-intent routing ─────────────────────────────────────


def test_detect_follow_intent_positive():
    m = SemanticPlannerModule()
    assert m._detect_follow_intent("跟着那个穿红衣服的人") == "那个穿红衣服的人"
    assert m._detect_follow_intent("跟随前面的人") == "前面的人"
    assert m._detect_follow_intent("follow the person in red") == "the person in red"


def test_detect_follow_intent_negative():
    m = SemanticPlannerModule()
    assert m._detect_follow_intent("去找红色椅子") is None
    assert m._detect_follow_intent("跟我说说红椅子在哪") is None
    assert m._detect_follow_intent("") is None
    assert m._detect_follow_intent("导航到厨房") is None


# ── Follow re-selection after target lost ─────────────────────────────────────


def _follow_detections():
    return [
        Detection3D(
            id="p",
            label="person",
            confidence=0.9,
            position=Vector3(1.0, 0.0, 0.0),
            bbox_2d=[0, 0, 10, 10],
            ts=1.0,
        )
    ]


def test_vs_reselects_after_target_lost():
    """A previously locked target lost past Re-ID timeout re-arms selection."""
    vs = VisualServoModule()
    vs._target_label = "person in red"
    vs._follow_select_method = "clip"  # already locked once
    vs._follow_select_pending = False
    vs._person_tracker.needs_vlm_reselect = lambda: True  # simulate lost-too-long

    calls = []
    vs._try_select_follow_target = lambda objs: calls.append(objs)
    vs._person_tracker.update = lambda o, f: None
    vs._person_tracker.get_follow_waypoint = lambda robot_pos: None

    vs._latest_bgr = np.zeros((20, 20, 3), dtype=np.uint8)
    vs._latest_bgr_ts = 1.0
    vs._latest_detections = _follow_detections()
    vs._latest_detection_ts = 1.0

    vs._tick_follow()

    assert vs._follow_select_pending is True  # re-armed
    assert len(calls) == 1  # selection retried this frame


def test_vs_no_reselect_before_first_lock():
    """Without an initial lock (_follow_select_method empty), no re-selection path."""
    vs = VisualServoModule()
    vs._target_label = "person in red"
    vs._follow_select_method = ""
    vs._follow_select_pending = False
    vs._person_tracker.needs_vlm_reselect = lambda: True

    reset_calls = []
    vs._person_tracker.reset = lambda: reset_calls.append(True)
    vs._try_select_follow_target = lambda objs: None
    vs._person_tracker.update = lambda o, f: None
    vs._person_tracker.get_follow_waypoint = lambda robot_pos: None
    vs._latest_bgr = np.zeros((20, 20, 3), dtype=np.uint8)
    vs._latest_bgr_ts = 1.0
    vs._latest_detections = _follow_detections()
    vs._latest_detection_ts = 1.0

    vs._tick_follow()

    assert reset_calls == []  # re-selection branch not taken


def test_tracking_does_not_pair_a_stale_rgb_frame_with_new_detections():
    vs = VisualServoModule()
    vs._mode = "follow"
    vs._follow_select_method = "clip"
    vs._person_tracker.lock_target(_person("target", "person", 1.0, [0, 0, 10, 10]))
    vs._latest_bgr = _marker_crop(99)
    vs._latest_bgr_ts = 9.0
    vs._latest_detections = [
        Detection3D(
            id="target",
            label="person",
            confidence=0.9,
            position=Vector3(1.1, 0.0, 0.0),
            bbox_2d=[0, 0, 10, 10],
            ts=10.0,
        )
    ]
    vs._latest_detection_ts = 10.0
    frames = []
    vs._person_tracker.update = lambda _objects, frame: frames.append(frame) or False

    vs._tick_follow()

    assert frames == [None]


def test_matching_rgb_arrival_resumes_unconfirmed_follow_once():
    vs = VisualServoModule()
    vs._mode = "follow"
    vs._follow_select_method = "clip"
    vs._target_visible = False
    vs._person_tracker.lock_target(_person("target", "person", 1.0, [0, 0, 10, 10]))
    vs._latest_detection_ts = 10.0
    calls = []

    def tick():
        calls.append(True)
        vs._target_visible = True

    vs._tick_follow = tick
    image = Image(
        data=np.zeros((16, 16, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        ts=10.0,
    )

    vs._on_color(image)
    vs._on_color(image)

    assert calls == [True]


def test_explicit_id_with_dino_waits_for_matching_rgb_reference():
    vs = VisualServoModule()
    vs._mode = "follow"
    vs._target_id = "target"
    vs._person_tracker.set_dino_encoder(_FakeDino())
    vs._last_perception_time = time.monotonic()
    vs._latest_detections = [
        Detection3D(
            id="target",
            label="person",
            confidence=0.9,
            position=Vector3(1.0, 0.0, 0.0),
            bbox_2d=[0, 0, 10, 10],
            ts=10.0,
        )
    ]
    vs._latest_detection_ts = 10.0
    vs._latest_bgr = _marker_crop(9)
    vs._latest_bgr_ts = 9.0

    vs._tick_follow()
    assert vs._person_tracker.target_selected is False
    assert vs._person_tracker.has_identity_reference is False

    vs._on_color(
        Image(
            data=np.ones((16, 16, 3), dtype=np.uint8),
            format=ImageFormat.BGR,
            ts=10.0,
        )
    )

    assert vs._person_tracker.target_selected is True
    assert vs._person_tracker.has_identity_reference is True


def test_new_identity_candidate_supersedes_held_verification(monkeypatch):
    first_started = threading.Event()
    first_finished = threading.Event()
    second_started = threading.Event()
    second_resolved = threading.Event()
    release_first = threading.Event()
    resolved = []

    async def fake_verify(
        _description,
        _candidate_crop,
        candidate_id,
        _chat_fn,
        **_kwargs,
    ):
        if candidate_id == "first":
            first_started.set()
            try:
                while not release_first.is_set():
                    await asyncio.sleep(0.001)
            finally:
                first_finished.set()
        else:
            second_started.set()
        return "MATCH", candidate_id

    monkeypatch.setattr("decision.modules.visual_servo.verify_candidate", fake_verify)
    vs, _client, _started, loop, loop_thread = _vision_follow("MATCH")

    def resolve(candidate_id, source_ts, verdict):
        resolved.append((candidate_id, source_ts, verdict))
        if candidate_id == "second":
            second_resolved.set()
        return True

    vs._person_tracker.resolve_identity = resolve

    def candidate(identifier, timestamp, marker):
        crop = _marker_crop(marker)
        return {
            "object": {"id": identifier, "ts": timestamp},
            "crop": crop,
            "reference_crop": _marker_crop(1),
            "reference_crops": [],
            "crops": [crop],
            "crop_timestamps": [timestamp],
        }

    try:
        vs._start_identity_verification(candidate("first", 4.0, 4))
        assert first_started.wait(timeout=1.0)

        vs._start_identity_verification(candidate("second", 5.0, 5))
        assert second_started.wait(timeout=1.0)
        assert second_resolved.wait(timeout=1.0)
        assert first_finished.wait(timeout=1.0)

        release_first.set()
        assert resolved == [("second", 5.0, "MATCH")]
        assert vs._follow_verify_key is None
        assert vs._follow_verify_running is False
    finally:
        release_first.set()
        _close_vision_loop(loop, loop_thread)
