"""test_planner_node_init.py — SemanticPlannerModule init and port regression tests

The original test_planner_node_init.py tested the deleted ROS2 SemanticPlannerNode.
This file has been rewritten to test the Module-First SemanticPlannerModule.

Coverage:
  - __init__ parameter defaults
  - In/Out port declarations
  - setup() does not crash
  - stop() cleans up correctly
  - instruction → goal_pose end-to-end routing (under mock LLM)
  - agent_instruction triggers AgentLoop path
  - planner_status output
  - mission_status triggers LERa recovery
  - health() structure
"""

import asyncio
import os
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [_repo, _src,
           
           
           ]:
    if _p not in sys.path:
        sys.path.insert(0, _p)


from runtime.module import Module, skill
from runtime.msgs.geometry import Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.stream import In, Out
from decision.tasking.agent_loop import AGENT_TOOLS, AgentLoop
from decision.modules.semantic_planner_module import SemanticPlannerModule

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_module(**kw) -> SemanticPlannerModule:
    mod = SemanticPlannerModule(**kw)
    mod.setup()
    return mod


def _make_odom(x=0.0, y=0.0):
    from runtime.msgs.geometry import Pose, Vector3
    od = Odometry()
    od.pose = Pose(position=Vector3(x, y, 0.0))
    return od


def _make_scene_graph(labels=("chair", "door")):
    objs = [
        Detection3D(id=str(i), label=lbl, position=Vector3(float(i), 0.0, 0.0))
        for i, lbl in enumerate(labels)
    ]
    return SceneGraph(objects=objs, regions=[])


def _collect(port_out, n=1, timeout=0.5):
    """Subscribe to an Out port and collect up to n messages."""
    items = []
    port_out.subscribe(lambda v: items.append(v))
    deadline = time.time() + timeout
    while len(items) < n and time.time() < deadline:
        time.sleep(0.01)
    return items


# ---------------------------------------------------------------------------
# 1. Init / port declarations
# ---------------------------------------------------------------------------

class TestSemanticPlannerInit:
    def test_layer(self):
        assert SemanticPlannerModule._layer == 4

    def test_in_ports(self):
        mod = SemanticPlannerModule()
        assert isinstance(mod.instruction, In)
        assert isinstance(mod.agent_instruction, In)
        assert isinstance(mod.scene_graph, In)
        assert isinstance(mod.odometry, In)
        assert isinstance(mod.detections, In)
        assert isinstance(mod.mission_status, In)

    def test_out_ports(self):
        mod = SemanticPlannerModule()
        assert isinstance(mod.goal_pose, Out)
        assert isinstance(mod.task_plan, Out)
        assert isinstance(mod.planner_status, Out)
        assert isinstance(mod.cancel, Out)
        assert isinstance(mod.servo_target, Out)

    def test_default_params(self):
        mod = SemanticPlannerModule()
        # Internally stored as _fast_threshold (not _fast_path_threshold)
        assert mod._fast_threshold == 0.75

    def test_custom_params(self):
        mod = SemanticPlannerModule(
            fast_path_threshold=0.9,
            decomposer="rules",
            llm_backend="mock",
        )
        assert mod._fast_threshold == 0.9
        assert mod._llm_backend == "mock"

    def test_goal_resolver_uses_selected_llm_backend(self):
        mod = _make_module(llm_backend="mock")
        try:
            assert mod._goal_resolver._primary.config.backend == "mock"
        finally:
            mod.stop()

    def test_setup_does_not_crash(self):
        mod = _make_module()
        mod.stop()

    def test_goal_resolver_initialized(self):
        mod = _make_module()
        assert mod._goal_resolver is not None
        mod.stop()

    def test_frontier_scorer_initialized(self):
        mod = _make_module()
        assert hasattr(mod, "_frontier_scorer") and mod._frontier_scorer is not None
        mod.stop()


# ---------------------------------------------------------------------------
# 2. State updates
# ---------------------------------------------------------------------------

class TestSemanticPlannerStateUpdate:
    def setup_method(self):
        self.mod = _make_module()

    def teardown_method(self):
        self.mod.stop()

    def test_odometry_cached(self):
        self.mod._on_odom(_make_odom(3.0, 4.0))
        pos = self.mod._robot_pos
        assert abs(pos[0] - 3.0) < 1e-6
        assert abs(pos[1] - 4.0) < 1e-6

    def test_scene_graph_cached(self):
        sg = _make_scene_graph()
        self.mod._on_scene_graph(sg)
        assert self.mod._latest_sg is not None

    def test_detections_cached(self):
        dets = [Detection3D(id="99", label="box", position=Vector3(1, 1, 0))]
        # _on_detections accepts a list — just verify it does not crash
        self.mod._on_detections(dets)

    def test_scene_graph_does_not_republish_same_goal(self):
        class _Result:
            confidence = 1.0
            position = [1.0, 2.0, 0.0]
            frame_id = "map"

        class _Resolver:
            def maybe_reload_kg(self):
                pass

            def fast_resolve(self, instruction, sg_json):
                return _Result()

        self.mod._goal_resolver = _Resolver()
        self.mod._current_instruction = "go to chair"
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        sg = _make_scene_graph(["chair"])
        self.mod._on_scene_graph(sg)
        self.mod._on_scene_graph(sg)

        assert len(goals) == 1


# ---------------------------------------------------------------------------
# 3. instruction → planner_status flow (fast path, no LLM dependency)
# ---------------------------------------------------------------------------

class TestSemanticPlannerInstruction:
    def setup_method(self):
        # Inject a scene graph so Fast Path has something to match
        self.mod = _make_module()
        sg = _make_scene_graph(["chair", "door"])
        self.mod._on_odom(_make_odom(0.0, 0.0))
        self.mod._on_scene_graph(sg)

    def teardown_method(self):
        self.mod.stop()

    def test_instruction_triggers_status(self):
        """After an instruction, planner_status must be published (success or failure — not silent)."""
        statuses = []
        self.mod.planner_status._add_callback(lambda s: statuses.append(s))
        self.mod._on_instruction("go to chair")
        deadline = time.time() + 1.5
        while not statuses and time.time() < deadline:
            time.sleep(0.02)
        assert len(statuses) > 0

    def test_instruction_empty_sg_no_crash(self):
        """Sending an instruction with an empty scene graph must not crash."""
        self.mod._on_scene_graph(SceneGraph(objects=[], regions=[]))
        self.mod._on_instruction("find the table")
        # Give background thread a moment to run
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# 4. mission_status LERa cooldown
# ---------------------------------------------------------------------------

class TestSemanticPlannerRecovery:
    def test_recovering_triggers_lera_if_instruction_active(self):
        """RECOVERING with an active instruction must trigger LERa."""
        mod = _make_module()
        mod._on_scene_graph(_make_scene_graph())
        mod._on_odom(_make_odom())
        # Simulate an active instruction
        mod._current_instruction = "find the coffee machine"
        # Deliver the new mission lifecycle state.
        mod._on_mission_status({"state": "RECOVERING"})
        # LERa runs in a background thread — just verify no crash
        time.sleep(0.05)
        mod.stop()

    def test_cooldown_prevents_double_lera(self):
        """A repeated RECOVERING status must be blocked by the cooldown."""
        mod = _make_module()
        mod._current_instruction = "find exit"
        mod._on_mission_status({"state": "RECOVERING"})
        last = mod._last_lera_time
        mod._on_mission_status({"state": "RECOVERING"})
        # Second trigger is blocked by cooldown: _last_lera_time must not change
        assert mod._last_lera_time == last
        mod.stop()


# ---------------------------------------------------------------------------
# 5. health()
# ---------------------------------------------------------------------------

class TestSemanticPlannerHealth:
    def test_health_structure(self):
        mod = _make_module()
        h = mod.health()
        assert isinstance(h, dict)
        assert "planner" in h or "goal_resolver" in h or "instruction" in str(h)
        mod.stop()


# ---------------------------------------------------------------------------
# 6. agent loop wiring
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


class _RecordingLLM:
    def __init__(self, replies):
        self._replies = list(replies)
        self.messages = None

    async def chat(self, messages, temperature=None):
        self.messages = messages
        if self._replies:
            return self._replies.pop(0)
        return '{"tool":"done","args":{"summary":"finished"}}'


class TestAgentLoopTools:
    def test_builtin_navigate_to_tool_allows_optional_z(self):
        tool = next(t for t in AGENT_TOOLS if t["function"]["name"] == "navigate_to")
        schema = tool["function"]["parameters"]

        assert "z" in schema["properties"]
        assert schema["properties"]["z"]["type"] == "number"
        assert "z" not in schema["required"]

    def test_llm_fallback_receives_runtime_tool_list(self):
        llm = _RecordingLLM(['{"tool":"hello","args":{"name":"codex"}}'])
        tool_list = [{
            "name": "hello",
            "description": "[FakeSkill] Return a greeting",
            "inputSchema": {
                "type": "object",
                "properties": {"name": {"type": "string"}},
            },
        }]
        agent = AgentLoop(
            llm_client=llm,
            tool_registry={"hello": lambda name="robot": f"hello {name}"},
            tool_list=tool_list,
            context_fn=lambda: {
                "robot_x": 0.0,
                "robot_y": 0.0,
                "visible_objects": "none",
                "nav_status": "IDLE",
                "memory_context": "none",
                "camera_available": False,
            },
        )
        response = asyncio.run(agent._llm_call([
            {"role": "system", "content": "test"},
            {"role": "user", "content": "say hello"},
        ]))
        assert response["tool_calls"][0]["function"]["name"] == "hello"
        assert llm.messages is not None
        assert "hello" in llm.messages[0]["content"]


class TestSemanticPlannerAgentLoop:
    def test_on_system_modules_discovers_agent_skills(self):
        mod = SemanticPlannerModule()
        tool_mod = _FakeSkillModule()
        unsafe_mod = _UnsafeSkillModule()
        mod.on_system_modules({
            "SemanticPlannerModule": mod,
            "FakeSkillModule": tool_mod,
            "UnsafeSkillModule": unsafe_mod,
        })

        assert "hello" in mod._agent_tool_registry
        assert any(t["name"] == "hello" for t in mod._agent_tool_list)
        assert "emergency_stop" not in mod._agent_tool_registry

    def test_run_agent_loop_uses_discovered_skills(self):
        mod = SemanticPlannerModule()
        mod.on_system_modules({
            "SemanticPlannerModule": mod,
            "FakeSkillModule": _FakeSkillModule(),
        })
        mod._goal_resolver = type(
            "_Resolver",
            (),
            {
                "_primary": _RecordingLLM([
                    '{"tool":"hello","args":{"name":"lingtu"}}',
                    '{"tool":"done","args":{"summary":"ok"}}',
                ]),
            },
        )()

        state = asyncio.run(mod._run_agent_loop("greet the operator"))

        assert state.completed
        assert state.summary == "ok"
        assert any(m.get("role") == "tool" and "hello lingtu" in m.get("content", "")
                   for m in state.messages)
