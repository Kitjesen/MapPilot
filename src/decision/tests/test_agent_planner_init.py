"""Tests for AgentPlannerModule - init, ports, health, and state updates."""

import builtins
import os
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [_repo, _src]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from decision.modules.agent_planner import AgentPlannerModule
from runtime.module import Module, skill
from runtime.msgs.geometry import Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.stream import In, Out

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_module(**kw) -> AgentPlannerModule:
    mod = AgentPlannerModule(**kw)
    mod.setup()
    return mod


def _make_odom(x=0.0, y=0.0):
    from runtime.msgs.geometry import Pose, Vector3

    od = Odometry()
    od.pose = Pose(position=Vector3(x, y, 0.0))
    return od


def _make_scene_graph(labels=("chair", "door")):
    objs = [Detection3D(id=str(i), label=lbl, position=Vector3(float(i), 0.0, 0.0)) for i, lbl in enumerate(labels)]
    return SceneGraph(objects=objs, regions=[])


# ---------------------------------------------------------------------------
# 1. Init / port declarations
# ---------------------------------------------------------------------------


class TestAgentPlannerInit:
    def test_layer(self):
        assert AgentPlannerModule._layer == 4

    def test_in_ports(self):
        mod = AgentPlannerModule()
        assert isinstance(mod.agent_instruction, In)
        assert isinstance(mod.scene_graph, In)
        assert isinstance(mod.odometry, In)
        assert isinstance(mod.mission_status, In)

    def test_out_ports(self):
        mod = AgentPlannerModule()
        assert isinstance(mod.goal_pose, Out)
        assert isinstance(mod.servo_target, Out)
        assert isinstance(mod.agent_message, Out)
        assert isinstance(mod.planner_status, Out)
        assert isinstance(mod.cancel, Out)

    def test_default_params(self):
        mod = AgentPlannerModule()
        assert mod._llm_backend == "kimi"
        assert mod._max_steps == 10
        assert mod._timeout == 120.0

    def test_custom_params(self):
        mod = AgentPlannerModule(
            llm_backend="mock",
            max_steps=5,
            timeout=60.0,
        )
        assert mod._llm_backend == "mock"
        assert mod._max_steps == 5
        assert mod._timeout == 60.0

    def test_setup_does_not_crash(self):
        mod = _make_module(llm_backend="mock")
        mod.stop()

    def test_llm_init_attempted_on_setup(self):
        mod = _make_module(llm_backend="mock")
        assert mod._llm_init_attempted is True
        mod.stop()


# ---------------------------------------------------------------------------
# 2. State updates
# ---------------------------------------------------------------------------


class TestAgentPlannerStateUpdate:
    def setup_method(self):
        self.mod = _make_module(llm_backend="mock")

    def teardown_method(self):
        self.mod.stop()

    def test_odometry_cached(self):
        self.mod._on_odom(_make_odom(3.0, 4.0))
        pos = self.mod._robot_pos
        assert abs(float(pos[0]) - 3.0) < 1e-6
        assert abs(float(pos[1]) - 4.0) < 1e-6

    def test_scene_graph_cached(self):
        sg = _make_scene_graph()
        self.mod._on_scene_graph(sg)
        assert self.mod._latest_sg is not None
        assert self.mod._current_scene_graph is not None

    def test_mission_status_caches_nav_state(self):
        self.mod._on_mission_status({"state": "NAVIGATING"})
        assert self.mod._last_nav_state == "NAVIGATING"

    def test_mission_status_ignores_recovering(self):
        """RECOVERING/STUCK/FAILED should not update _last_nav_state."""
        self.mod._on_mission_status({"state": "NAVIGATING"})
        self.mod._on_mission_status({"state": "RECOVERING"})
        # RECOVERING is a terminal state, should not update _last_nav_state
        assert self.mod._last_nav_state == "NAVIGATING"


# ---------------------------------------------------------------------------
# 3. Health
# ---------------------------------------------------------------------------


class TestAgentPlannerHealth:
    def test_health_contains_agent_planner_key(self):
        mod = _make_module(llm_backend="mock")
        try:
            h = mod.health()
            assert isinstance(h, dict)
            assert "agent_planner" in h
            info = h["agent_planner"]
            assert "llm_backend" in info
            assert "llm_ready" in info
            assert "tools" in info
            assert "agent_runs" in info
        finally:
            mod.stop()

    def test_health_reports_llm_backend(self):
        mod = _make_module(llm_backend="mock")
        try:
            h = mod.health()
            assert h["agent_planner"]["llm_backend"] == "mock"
        finally:
            mod.stop()

    def test_health_reports_llm_init_error_when_unavailable(self, monkeypatch):
        real_import = builtins.__import__

        def fail_llm_import(name, globals=None, locals=None, fromlist=(), level=0):
            if name == "decision.llm.client":
                raise ImportError("LLM client not available")
            return real_import(name, globals, locals, fromlist, level)

        monkeypatch.setattr(builtins, "__import__", fail_llm_import)

        mod = _make_module(llm_backend="kimi")
        try:
            h = mod.health()
            assert h["agent_planner"]["llm_ready"] is False
            assert h["agent_planner"]["llm_init_attempted"] is True
        finally:
            mod.stop()


# ---------------------------------------------------------------------------
# 4. Agent tool discovery
# ---------------------------------------------------------------------------


class _FakeSkillModule(Module):
    @skill
    def hello(self, name: str = "robot") -> str:
        """Return a greeting string."""
        return f"hello {name}"


class _UnsafeSkillModule(Module):
    @skill
    def emergency_stop(self) -> str:
        """Trigger an emergency stop."""
        return "stopped"


class TestAgentPlannerToolDiscovery:
    def test_on_system_modules_discovers_skills(self):
        mod = AgentPlannerModule(llm_backend="mock")
        tool_mod = _FakeSkillModule()
        unsafe_mod = _UnsafeSkillModule()
        mod.on_system_modules(
            {
                "AgentPlannerModule": mod,
                "FakeSkillModule": tool_mod,
                "UnsafeSkillModule": unsafe_mod,
            }
        )

        assert "hello" in mod._agent_tool_registry
        assert any(t["name"] == "hello" for t in mod._agent_tool_list)
        # emergency_stop is in the blocklist
        assert "emergency_stop" not in mod._agent_tool_registry

    def test_tool_list_schema(self):
        mod = AgentPlannerModule(llm_backend="mock")
        mod.on_system_modules(
            {
                "AgentPlannerModule": mod,
                "FakeSkillModule": _FakeSkillModule(),
            }
        )

        tool = next(t for t in mod._agent_tool_list if t["name"] == "hello")
        assert "description" in tool
        assert "inputSchema" in tool
        assert tool["inputSchema"]["type"] == "object"

    def test_empty_modules_no_crash(self):
        mod = AgentPlannerModule(llm_backend="mock")
        mod.on_system_modules({})
        assert mod._agent_tool_registry == {}
        assert mod._agent_tool_list == []
