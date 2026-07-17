"""Unit tests for the VLA navigation module and backends."""

from __future__ import annotations

import asyncio
import base64
import json
import threading
import time
from unittest import mock

import numpy as np
import pytest

from decision.modules.vla import VLAModule
from decision.modules.vla_backends import (
    MockVLABackend,
    VLABackend,
    create_vla_backend,
)
from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.msgs.sensor import CameraIntrinsics, Image, ImageFormat


@pytest.fixture
def vla_module():
    """Create a VLAModule with mock backend for testing."""
    module = VLAModule(backend="mock")
    return module


@pytest.fixture
def small_bgr_image():
    """Create a small BGR image for encoding tests."""
    arr = np.zeros((10, 10, 3), dtype=np.uint8)
    arr[:, :, 0] = 120  # B
    arr[:, :, 1] = 200  # G
    arr[:, :, 2] = 80  # R
    return Image.from_numpy(arr, fmt=ImageFormat.BGR)


@pytest.fixture
def odometry_message():
    """Create an odometry message at origin."""
    return Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0)),
        ts=time.time(),
    )


def _collect_outputs(module):
    """Attach collectors to all output ports."""
    outputs = {}
    lock = threading.Lock()

    def make_collector(name):
        def collector(msg):
            with lock:
                outputs.setdefault(name, []).append(msg)

        return collector

    for name, port in module.ports_out.items():
        port.subscribe(make_collector(name))
    return outputs, lock


# ---------------------------------------------------------------------------
# Module init
# ---------------------------------------------------------------------------


def test_module_init(vla_module):
    """VLAModule exposes the expected input and output ports."""
    module = vla_module
    assert set(module.ports_in.keys()) == {
        "color_image",
        "depth_image",
        "odometry",
        "scene_graph",
        "instruction",
        "camera_info",
        "mission_status",
    }
    assert set(module.ports_out.keys()) == {
        "goal_pose",
        "cmd_vel",
        "vla_status",
        "servo_target",
        "vla_action",
        "planner_status",
    }
    assert module._layer == 4
    assert module._run_in_worker is True
    assert module._worker_group == "semantic"
    assert "LLMModule" in module.SOFT_DEPENDS
    assert "PerceptionModule" in module.SOFT_DEPENDS


def test_preflight_mock():
    """Preflight passes for mock backend without API keys."""
    module = VLAModule(backend="mock")
    assert module.preflight() is None


def test_preflight_openai_missing_key():
    """Preflight reports missing API key for openai backend."""
    module = VLAModule(backend="openai", api_key="")
    with mock.patch.dict("os.environ", {}, clear=True):
        result = module.preflight()
    assert result is not None
    assert "OPENAI_API_KEY" in result


# ---------------------------------------------------------------------------
# Image encoding
# ---------------------------------------------------------------------------


def test_image_encoding(vla_module, small_bgr_image):
    """BGR image is converted to RGB, resized, and encoded to base64 JPEG."""
    b64 = vla_module._encode_image_to_b64(small_bgr_image)
    assert isinstance(b64, str)
    assert len(b64) > 0
    raw = base64.b64decode(b64)
    # JPEG magic bytes
    assert raw[:2] == b"\xff\xd8"
    assert raw[-2:] == b"\xff\xd9"


def test_image_resize(vla_module):
    """Large images are resized to fit the max size while keeping aspect ratio."""
    arr = np.zeros((1000, 500, 3), dtype=np.uint8)
    img = Image.from_numpy(arr, fmt=ImageFormat.BGR)
    module = VLAModule(backend="mock", image_max_size=256)
    b64 = module._encode_image_to_b64(img)
    assert isinstance(b64, str)
    assert len(b64) > 0


# ---------------------------------------------------------------------------
# Prompt building
# ---------------------------------------------------------------------------


def test_prompt_building(vla_module):
    """System prompt contains the expected action schema guidance."""
    prompt = vla_module._system_prompt
    assert "JSON" in prompt
    assert "action" in prompt
    assert "target_x" in prompt
    assert "target_y" in prompt
    assert "confidence" in prompt


# ---------------------------------------------------------------------------
# Action parsing
# ---------------------------------------------------------------------------


def test_action_parsing_markdown_json():
    """Backends can extract JSON wrapped in markdown fences."""
    text = (
        '```json\n{"action": "navigate", "target_x": 1.0, '
        '"target_y": 2.0, "target_z": 0.0, "confidence": 0.9, '
        '"reason": "go there"}\n```'
    )
    parsed = VLABackend._extract_json(text)
    normalized = VLABackend._normalize_action(parsed)
    assert normalized["action"] == "navigate"
    assert normalized["target_x"] == 1.0
    assert normalized["target_y"] == 2.0
    assert normalized["confidence"] == 0.9


def test_action_parsing_invalid_action():
    """Unknown actions are normalized to stop."""
    normalized = VLABackend._normalize_action({"action": "dance", "target_x": 0, "target_y": 0, "confidence": 0.5})
    assert normalized["action"] == "stop"


def test_action_parsing_missing_fields():
    """Missing fields get safe defaults."""
    normalized = VLABackend._normalize_action({})
    assert normalized["action"] == "stop"
    assert normalized["target_x"] == 0.0
    assert normalized["target_y"] == 0.0
    assert normalized["confidence"] == 0.0


# ---------------------------------------------------------------------------
# Distance routing
# ---------------------------------------------------------------------------


def test_distance_routing_far_goal(vla_module):
    """Targets beyond the far threshold publish a PoseStamped goal."""
    module = vla_module
    module._robot_pose = [0.0, 0.0, 0.0]
    module._robot_yaw = 0.0

    goals = []
    cmd_vels = []
    module.goal_pose.subscribe(goals.append)
    module.cmd_vel.subscribe(cmd_vels.append)

    module._route_action(
        {
            "action": "navigate",
            "target_x": 10.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "confidence": 0.9,
        }
    )

    assert len(goals) == 1
    assert isinstance(goals[0], PoseStamped)
    assert goals[0].x == pytest.approx(10.0)
    assert len(cmd_vels) == 0


def test_distance_routing_near_cmd_vel(vla_module):
    """Targets below the near threshold publish a Twist command."""
    module = vla_module
    module._robot_pose = [0.0, 0.0, 0.0]
    module._robot_yaw = 0.0

    goals = []
    cmd_vels = []
    module.goal_pose.subscribe(goals.append)
    module.cmd_vel.subscribe(cmd_vels.append)

    module._route_action(
        {
            "action": "approach",
            "target_x": 1.0,
            "target_y": 0.0,
            "target_z": 0.0,
            "confidence": 0.9,
        }
    )

    assert len(cmd_vels) == 1
    assert isinstance(cmd_vels[0], Twist)
    assert cmd_vels[0].linear.x > 0
    assert len(goals) == 0


def test_distance_routing_stop(vla_module):
    """Stop action publishes zero velocity."""
    module = vla_module
    cmd_vels = []
    module.cmd_vel.subscribe(cmd_vels.append)

    module._route_action({"action": "stop", "confidence": 1.0})

    assert len(cmd_vels) == 1
    assert cmd_vels[0].linear.x == pytest.approx(0.0)


def test_distance_routing_hysteresis(vla_module):
    """Hysteresis prevents flapping around the threshold."""
    module = vla_module
    module._robot_pose = [0.0, 0.0, 0.0]
    module._robot_yaw = 0.0

    goals = []
    cmd_vels = []
    module.goal_pose.subscribe(goals.append)
    module.cmd_vel.subscribe(cmd_vels.append)

    # First call enters near mode (below near threshold)
    module._route_action(
        {
            "action": "approach",
            "target_x": 2.5,
            "target_y": 0.0,
            "target_z": 0.0,
            "confidence": 0.9,
        }
    )
    assert len(cmd_vels) == 1

    # Target moves slightly beyond near threshold but below far threshold
    module._route_action(
        {
            "action": "approach",
            "target_x": 3.4,
            "target_y": 0.0,
            "target_z": 0.0,
            "confidence": 0.9,
        }
    )
    # Should still be cmd_vel due to hysteresis
    assert len(cmd_vels) == 2
    assert len(goals) == 0


# ---------------------------------------------------------------------------
# Circuit breaker
# ---------------------------------------------------------------------------


def test_circuit_breaker_opens_after_failures(vla_module):
    """Circuit breaker opens after threshold consecutive failures."""
    module = vla_module
    module._cb_threshold = 3

    for _ in range(3):
        module._handle_error(RuntimeError("timeout"), "go", 100.0)

    assert module._consecutive_failures >= module._cb_threshold
    assert module._circuit_open_until > time.time()
    health = module.health()
    assert health["vla"]["circuit_breaker"] == "open"


def test_circuit_breaker_half_open(vla_module):
    """Circuit breaker transitions to half-open after cooldown."""
    module = vla_module
    module._cb_threshold = 3
    module._cb_cooldown = 0.05

    for _ in range(3):
        module._handle_error(RuntimeError("timeout"), "go", 100.0)

    assert module._circuit_open_until > time.time()
    time.sleep(0.06)
    health = module.health()
    assert health["vla"]["circuit_breaker"] == "half-open"


def test_circuit_breaker_recover(vla_module):
    """Successful call resets consecutive failures."""
    module = vla_module
    module._cb_threshold = 3

    module._handle_error(RuntimeError("timeout"), "go", 100.0)
    module._handle_error(RuntimeError("timeout"), "go", 100.0)
    assert module._consecutive_failures == 2

    action = {
        "action": "navigate",
        "target_x": 5.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "confidence": 0.9,
        "reason": "ok",
    }
    module._handle_success(action, "go", 100.0)
    assert module._consecutive_failures == 0
    assert module._circuit_open_until == 0.0


# ---------------------------------------------------------------------------
# Fallback logic
# ---------------------------------------------------------------------------


def test_fallback_on_api_error(vla_module):
    """API errors publish a fallback status for the semantic planner."""
    module = vla_module
    statuses = []
    module.planner_status.subscribe(statuses.append)

    module._handle_error(RuntimeError("API down"), "find the chair", 100.0)

    assert len(statuses) == 1
    assert statuses[0].startswith("FALLBACK_TO_SEMANTIC_PLANNER")
    assert "find the chair" in statuses[0]


def test_fallback_on_low_confidence(vla_module):
    """Low-confidence actions are forwarded to VisualServo."""
    module = vla_module
    servo_targets = []
    planner_statuses = []
    module.servo_target.subscribe(servo_targets.append)
    module.planner_status.subscribe(planner_statuses.append)

    action = {
        "action": "approach",
        "target_x": 1.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "confidence": 0.2,
        "reason": "unsure",
    }
    module._handle_success(action, "find the chair", 100.0)

    assert len(servo_targets) == 1
    assert servo_targets[0] == "find the chair"
    assert len(planner_statuses) == 1
    assert planner_statuses[0].startswith("LOW_CONFIDENCE_SERVO")


# ---------------------------------------------------------------------------
# Mock backend
# ---------------------------------------------------------------------------


def test_mock_backend_factory():
    """create_vla_backend returns a MockVLABackend for backend='mock'."""
    backend = create_vla_backend("mock")
    assert isinstance(backend, MockVLABackend)


@pytest.mark.asyncio
async def test_mock_backend_navigate():
    """Mock backend returns a navigate action for generic instructions."""
    backend = MockVLABackend()
    action = await backend.infer("dummy", "go to the door", {"x": 1.0, "y": 2.0})
    assert action["action"] == "navigate"
    assert action["target_x"] == pytest.approx(6.0)
    assert action["target_y"] == pytest.approx(2.0)
    assert action["confidence"] > 0


@pytest.mark.asyncio
async def test_mock_backend_stop():
    """Mock backend returns stop for stop instructions."""
    backend = MockVLABackend()
    action = await backend.infer("dummy", "stop now", {"x": 0.0, "y": 0.0})
    assert action["action"] == "stop"


@pytest.mark.asyncio
async def test_mock_backend_fixed_action():
    """Mock backend respects a fixed action override."""
    backend = MockVLABackend(
        fixed_action={
            "action": "explore",
            "target_x": 7.0,
            "target_y": 8.0,
            "target_z": 0.0,
            "confidence": 0.6,
            "reason": "fixed",
        }
    )
    action = await backend.infer("dummy", "anything", {})
    assert action["action"] == "explore"
    assert action["target_x"] == pytest.approx(7.0)


# ---------------------------------------------------------------------------
# End-to-end mock inference
# ---------------------------------------------------------------------------


def test_mock_backend_e2e(vla_module, small_bgr_image, odometry_message):
    """VLAModule runs mock backend inference end-to-end via ThreadPoolExecutor."""
    module = vla_module
    module.setup()
    try:
        module._on_color_image(small_bgr_image)
        module._on_odometry(odometry_message)

        goals = []
        cmd_vels = []
        vla_actions = []
        statuses = []
        module.goal_pose.subscribe(goals.append)
        module.cmd_vel.subscribe(cmd_vels.append)
        module.vla_action.subscribe(vla_actions.append)
        module.vla_status.subscribe(statuses.append)

        module._on_instruction("go forward")

        # Wait for the worker thread to finish
        deadline = time.time() + 2.0
        while time.time() < deadline and not vla_actions:
            time.sleep(0.01)

        assert len(vla_actions) == 1
        assert vla_actions[0]["action"] == "navigate"
        assert len(goals) == 1
        assert len(statuses) >= 1
    finally:
        module.stop()


def test_mock_backend_e2e_no_image(vla_module):
    """VLAModule skips inference when no image is available."""
    module = vla_module
    module.setup()
    try:
        statuses = []
        module.vla_status.subscribe(statuses.append)
        module._on_instruction("go forward")
        # Should publish a no_image status synchronously
        assert len(statuses) >= 1
        assert statuses[-1]["state"] == "no_image"
    finally:
        module.stop()


# ---------------------------------------------------------------------------
# Health metrics
# ---------------------------------------------------------------------------


def test_health_metrics(vla_module):
    """Health report contains backend, call counts, latency, and circuit state."""
    module = vla_module
    module._call_count = 10
    module._success_count = 8
    module._error_count = 2
    module._total_latency_ms = 800.0

    health = module.health()
    assert health["module"] == "VLAModule"
    assert health["vla"]["backend"] == "mock"
    assert health["vla"]["call_count"] == 10
    assert health["vla"]["success_count"] == 8
    assert health["vla"]["error_count"] == 2
    assert health["vla"]["success_rate"] == pytest.approx(0.8)
    assert health["vla"]["avg_latency_ms"] == pytest.approx(100.0)
    assert health["vla"]["circuit_breaker"] == "closed"


def test_skills_exposed(vla_module):
    """VLAModule exposes vla_navigate and vla_status as skills."""
    module = vla_module
    skills = module.skills
    assert "vla_navigate" in skills
    assert "vla_status" in skills


def test_scene_graph_summary(vla_module):
    """Scene graph summary is built from the latest scene graph."""
    module = vla_module
    sg = SceneGraph(
        objects=[
            Detection3D(
                id="chair_1",
                label="chair",
                position=Vector3(1.5, 2.5, 0.3),
                confidence=0.9,
            ),
            Detection3D(
                id="desk_1",
                label="desk",
                position=Vector3(3.0, 1.0, 0.6),
                confidence=0.85,
            ),
        ]
    )
    module._on_scene_graph(sg)
    summary = module._scene_graph_summary()
    assert "chair" in summary
    assert "desk" in summary
