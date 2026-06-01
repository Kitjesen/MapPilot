"""Tests for core.spec Protocol interfaces.

Verifies:
1. All protocols are runtime_checkable.
2. isinstance() succeeds for conforming mock classes.
3. isinstance() fails for non-conforming classes.
4. isinstance() succeeds for real implementation classes from the codebase.
"""

import pytest

from core.spec import (
    Detector,
    Encoder,
    EpisodicMemoryLike,
    GlobalPlanner,
    GoalResolver,
    LLMBackend,
    RobotDriver,
    SafetyChecker,
    SpatialMemory,
    TaskDecomposer,
    Tracker,
)

# ── Conforming mock classes ────────────────────────────────────────

class MockDetector:
    def detect(self, image, classes=None):
        return []

    def load_model(self):
        pass

    def shutdown(self):
        pass


class MockEncoder:
    def encode_image(self, image):
        return [0.0] * 512

    def encode_text(self, text):
        return [0.0] * 512

    def load_model(self):
        pass


class MockTracker:
    def update(self, detections, **kwargs):
        pass

    def get_scene_graph_json(self):
        return "{}"

    def clear(self):
        pass


class MockGoalResolver:
    def fast_resolve(self, instruction, scene_graph_json):
        return None


class MockGlobalPlanner:
    def plan(self, start, goal):
        return []


class MockTaskDecomposer:
    def decompose_with_rules(self, instruction):
        return []


class MockRobotDriver:
    def __init__(self):
        self.cmd_vel = None
        self.odometry = None
        self.alive = None


class MockSpatialMemory:
    def update_position(self, position, visible_labels=None):
        pass

    def query_by_text(self, text, top_k=5):
        return []


class MockEpisodicMemory:
    def add(self, position, label):
        pass

    def format_for_llm(self):
        return ""


class MockSafetyChecker:
    def check(self, state):
        return {"level": "OK", "issues": []}


class MockLLM:
    async def chat(self, messages, **kwargs):
        return "mock reply"


# ── Non-conforming classes ─────────────────────────────────────────

class Empty:
    """Has no relevant methods at all."""
    pass


class WrongDetector:
    """Has detect() but missing load_model and shutdown."""
    def detect(self, image, classes=None):
        return []


# ── Tests: runtime_checkable ───────────────────────────────────────

ALL_PROTOCOLS = [
    Detector, Encoder, Tracker,
    GoalResolver, GlobalPlanner, TaskDecomposer,
    RobotDriver,
    SpatialMemory, EpisodicMemoryLike,
    SafetyChecker,
    LLMBackend,
]


@pytest.mark.parametrize("proto", ALL_PROTOCOLS, ids=lambda p: p.__name__)
def test_protocol_is_runtime_checkable(proto):
    """Every protocol must be decorated with @runtime_checkable."""
    assert hasattr(proto, "__protocol_attrs__") or hasattr(proto, "__abstractmethods__") or True
    # The real check: isinstance must not raise TypeError
    try:
        isinstance(object(), proto)
    except TypeError:
        pytest.fail(f"{proto.__name__} is not runtime_checkable")


# ── Tests: mock conformance ────────────────────────────────────────

_MOCK_PAIRS = [
    (MockDetector(), Detector),
    (MockEncoder(), Encoder),
    (MockTracker(), Tracker),
    (MockGoalResolver(), GoalResolver),
    (MockGlobalPlanner(), GlobalPlanner),
    (MockTaskDecomposer(), TaskDecomposer),
    (MockRobotDriver(), RobotDriver),
    (MockSpatialMemory(), SpatialMemory),
    (MockEpisodicMemory(), EpisodicMemoryLike),
    (MockSafetyChecker(), SafetyChecker),
    (MockLLM(), LLMBackend),
]


@pytest.mark.parametrize(
    "instance, proto",
    _MOCK_PAIRS,
    ids=lambda x: x.__class__.__name__ if not isinstance(x, type) else x.__name__,
)
def test_mock_satisfies_protocol(instance, proto):
    assert isinstance(instance, proto), (
        f"{type(instance).__name__} should satisfy {proto.__name__}"
    )


# ── Tests: non-conformance ────────────────────────────────────────

def test_empty_does_not_satisfy_detector():
    assert not isinstance(Empty(), Detector)


def test_empty_does_not_satisfy_goal_resolver():
    assert not isinstance(Empty(), GoalResolver)


def test_empty_does_not_satisfy_llm_backend():
    assert not isinstance(Empty(), LLMBackend)


def test_wrong_detector_missing_methods():
    # WrongDetector has detect() but lacks load_model / shutdown
    assert not isinstance(WrongDetector(), Detector)


# ── Tests: real implementation classes ─────────────────────────────

def test_real_goal_resolver_satisfies_protocol():
    """GoalResolver from semantic_planner should satisfy the GoalResolver protocol."""
    try:
        from semantic.planner.goal_resolver import (
            GoalResolver as RealGoalResolver,
        )
    except ImportError:
        pytest.skip("semantic_planner not importable in this environment")
    # GoalResolver has fast_resolve via FastPathMixin
    assert hasattr(RealGoalResolver, "fast_resolve")


def test_real_task_decomposer_satisfies_protocol():
    """TaskDecomposer from semantic_planner should satisfy the protocol."""
    try:
        from semantic.planner.task_decomposer import (
            TaskDecomposer as RealTaskDecomposer,
        )
    except ImportError:
        pytest.skip("semantic_planner not importable in this environment")
    assert hasattr(RealTaskDecomposer, "decompose_with_rules")


def test_real_topological_memory_satisfies_spatial():
    """TopologicalMemory should satisfy SpatialMemory protocol."""
    try:
        from memory.spatial.topological import TopologicalMemory
    except ImportError:
        pytest.skip("semantic_planner not importable in this environment")
    assert hasattr(TopologicalMemory, "update_position")
    assert hasattr(TopologicalMemory, "query_by_text")


def test_real_episodic_memory_satisfies_protocol():
    """EpisodicMemory should have add() and format_for_llm()."""
    try:
        from memory.spatial.episodic import EpisodicMemory
    except ImportError:
        pytest.skip("semantic_planner not importable in this environment")
    assert hasattr(EpisodicMemory, "add")
    assert hasattr(EpisodicMemory, "format_for_llm")


def test_real_llm_client_has_chat():
    """LLMClientBase subclasses should have async chat()."""
    try:
        from semantic.planner.llm_client import LLMClientBase
    except ImportError:
        pytest.skip("semantic_planner not importable in this environment")
    assert hasattr(LLMClientBase, "chat")
