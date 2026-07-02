"""Tests for description-based person following.

Covers the "person in red" selection path added across:
  - CLIPEncoder.encode_image (interface-gap fix)
  - PersonTracker.select_by_clip / select_target_with_vlm (CLIP + multimodal VLM)
  - VisualServoModule.on_system_modules injection + follow target selection
  - SemanticPlannerModule follow-intent routing
"""

import asyncio

import numpy as np

from decision.vision.person_tracker import PersonTracker
from decision.modules.semantic_planner_module import SemanticPlannerModule
from decision.modules.visual_servo_module import VisualServoModule
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D, SceneGraph


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


# ── CLIPEncoder.encode_image (interface-gap fix) ──────────────────────────────

def test_clip_encoder_has_encode_image():
    from perception.encoding.clip_encoder import CLIPEncoder

    enc = CLIPEncoder()  # model not loaded
    assert hasattr(enc, "encode_image")
    out = enc.encode_image(np.zeros((12, 12, 3), dtype=np.uint8))
    # Model unavailable → empty feature, but no AttributeError (the bug we fixed).
    assert hasattr(out, "size")
    assert out.size == 0


# ── PersonTracker.select_by_clip ──────────────────────────────────────────────

def test_select_by_clip_picks_matching_person():
    tracker = PersonTracker()
    tracker.set_clip_encoder(_FakeClip([1.0, 0.0], {1: [1.0, 0.0], 2: [0.0, 1.0]}))
    crops = [_marker_crop(1), _marker_crop(2)]
    objs = [{"id": "red", "position": [1, 0, 0], "bbox": [0, 0, 10, 10]},
            {"id": "blue", "position": [2, 0, 0], "bbox": [0, 0, 10, 10]}]

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
    objs = [{"id": "p1", "position": [1, 0, 0], "bbox": [0, 0, 10, 10]},
            {"id": "p2", "position": [2, 0, 0], "bbox": [0, 0, 10, 10]}]
    captured = {}

    async def mock_vlm(messages):
        captured["messages"] = messages
        return "选 2 号"

    idx = asyncio.run(
        tracker.select_target_with_vlm("person in red", crops, objs, mock_vlm)
    )
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
    msgs = PersonTracker._build_vlm_select_messages(
        "pick one", [_marker_crop(1), _marker_crop(2)]
    )
    assert len(msgs) == 1 and msgs[0]["role"] == "user"
    content = msgs[0]["content"]
    assert content[0] == {"type": "text", "text": "pick one"}
    # If image encoding is available, each crop adds an image_url entry.
    image_entries = [c for c in content if c.get("type") == "image_url"]
    assert len(image_entries) in (0, 2)  # 0 when cv2/PIL absent, else one per crop


# ── VisualServoModule injection + selection ───────────────────────────────────

def test_vs_injects_image_capable_encoder():
    vs = VisualServoModule()

    class _Enc:
        _backend = _FakeClip([1.0, 0.0], {1: [1.0, 0.0]})

    vs.on_system_modules({"EncoderModule": _Enc()})
    assert vs._person_tracker._clip_encoder is not None


def test_vs_skips_text_only_encoder():
    vs = VisualServoModule()

    class _Enc:
        _backend = _TextOnlyClip()  # no encode_image

    vs.on_system_modules({"EncoderModule": _Enc()})
    assert vs._person_tracker._clip_encoder is None


def test_vs_attaches_vision_client():
    vs = VisualServoModule()

    class OpenAIClient:
        async def chat(self, messages, temperature=None):
            return "1"

    class _LLM:
        _client = OpenAIClient()

    vs.on_system_modules({"LLMModule": _LLM()})
    assert vs._vision_client is not None


def test_vs_rejects_text_only_llm():
    vs = VisualServoModule()

    class MoonshotClient:
        async def chat(self, messages, temperature=None):
            return "1"

    class _LLM:
        _client = MoonshotClient()

    vs.on_system_modules({"LLMModule": _LLM()})
    assert vs._vision_client is None


def _person(obj_id, label, x, bbox):
    return {"id": obj_id, "label": label, "position": [x, 0, 0],
            "bbox": bbox, "confidence": 0.9}


def test_vs_try_select_follow_locks_via_clip():
    vs = VisualServoModule()

    class _Enc:
        _backend = _FakeClip([1.0, 0.0], {1: [1.0, 0.0], 2: [0.0, 1.0]})

    vs.on_system_modules({"EncoderModule": _Enc()})
    vs._target_label = "person in red"
    vs._follow_select_pending = True

    bgr = np.zeros((40, 40, 3), dtype=np.uint8)
    bgr[0:20, 0:20] = 1   # "red" person region (marker 1)
    bgr[0:20, 20:40] = 2  # "blue" person region (marker 2)
    vs._latest_bgr = bgr

    scene_objects = [
        _person("red", "person", 1.0, [0, 0, 20, 20]),
        _person("blue", "person", 2.0, [20, 0, 40, 20]),
    ]
    vs._try_select_follow_target(scene_objects)

    assert vs._follow_select_pending is False
    assert vs._follow_select_method == "clip"
    assert vs._person_tracker._person.obj_id == "red"


def test_vs_try_select_follow_degraded_without_clip_or_vlm():
    vs = VisualServoModule()  # no encoder, no vision client
    vs._target_label = "person in red"
    vs._follow_select_pending = True
    bgr = np.zeros((40, 40, 3), dtype=np.uint8)
    bgr[0:20, 0:20] = 1
    vs._latest_bgr = bgr

    vs._try_select_follow_target([_person("a", "person", 1.0, [0, 0, 20, 20])])

    assert vs._follow_select_pending is False
    assert vs._follow_select_method == "degraded"


def test_vs_follow_command_arms_selection():
    vs = VisualServoModule()
    vs._on_servo_target("follow:person in red")
    assert vs._mode == "follow"
    assert vs._target_label == "person in red"
    assert vs._follow_select_pending is True


def test_vs_client_supports_vision():
    assert VisualServoModule._client_supports_vision(None) is False

    class Moonshot:
        def chat(self):
            ...

    class OpenAI:
        def chat(self):
            ...

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

def _follow_sg():
    return SceneGraph(objects=[
        Detection3D(id="p", label="person", confidence=0.9,
                    position=Vector3(1.0, 0.0, 0.0), bbox_2d=[0, 0, 10, 10]),
    ])


def test_vs_reselects_after_target_lost():
    """A previously locked target lost past Re-ID timeout re-arms selection."""
    vs = VisualServoModule()
    vs._target_label = "person in red"
    vs._follow_select_method = "clip"      # already locked once
    vs._follow_select_pending = False
    vs._person_tracker.needs_vlm_reselect = lambda: True  # simulate lost-too-long

    calls = []
    vs._try_select_follow_target = lambda objs: calls.append(objs)
    vs._person_tracker.update = lambda o, f: None
    vs._person_tracker.get_follow_waypoint = lambda robot_pos: None

    vs._latest_bgr = np.zeros((20, 20, 3), dtype=np.uint8)
    vs._latest_sg = _follow_sg()

    vs._tick_follow()

    assert vs._follow_select_pending is True   # re-armed
    assert len(calls) == 1                       # selection retried this frame


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
    vs._latest_sg = _follow_sg()

    vs._tick_follow()

    assert reset_calls == []  # re-selection branch not taken
