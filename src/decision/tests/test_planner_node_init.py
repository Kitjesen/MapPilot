"""Decision module."""

import asyncio
import math
import os
import sys
import time

_here = os.path.dirname(os.path.abspath(__file__))
_repo = os.path.abspath(os.path.join(_here, "..", "..", "..", ".."))
_src = os.path.join(_repo, "src")
for _p in [
    _repo,
    _src,
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)


from decision.modules.agent_planner import AgentPlannerModule
from decision.modules.semantic_planner import SemanticPlannerModule
from decision.tasks.agent import AGENT_TOOLS, AgentLoop
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import NavigationLifecycle, NavigationState, Odometry
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.stream import In, Out

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
    objs = [Detection3D(id=str(i), label=lbl, position=Vector3(float(i), 0.0, 0.0)) for i, lbl in enumerate(labels)]
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
        assert isinstance(mod.scene_graph, In)
        assert isinstance(mod.odometry, In)
        assert isinstance(mod.detections, In)
        assert isinstance(mod.navigation_state, In)

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

    def test_scene_graph_micro_jitter_does_not_republish_goal(self):
        class _Resolver:
            def __init__(self):
                self.calls = 0

            def maybe_reload_kg(self):
                pass

            def fast_resolve(self, instruction, sg_json):
                self.calls += 1

                class _Result:
                    confidence = 1.0
                    frame_id = "map"

                    def __init__(self, x):
                        self.position = [x, 2.0, 0.0]

                return _Result(1.0 + 0.01 * self.calls)

        self.mod._goal_resolver = _Resolver()
        self.mod._current_instruction = "go to chair"
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        self.mod._on_scene_graph(_make_scene_graph(["chair"]))
        self.mod._on_scene_graph(_make_scene_graph(["chair"]))

        assert len(goals) == 1

    def test_goal_signature_includes_yaw(self):
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        first = PoseStamped(
            Pose(Vector3(1.0, 2.0, 0.0), Quaternion.from_yaw(0.0)),
            frame_id="map",
        )
        second = PoseStamped(
            Pose(Vector3(1.0, 2.0, 0.0), Quaternion.from_yaw(math.pi / 2.0)),
            frame_id="map",
        )

        assert self.mod._publish_goal_pose_once("face the chair", first)
        assert self.mod._publish_goal_pose_once("face the chair", second)
        assert len(goals) == 2

    def test_goal_hysteresis_suppresses_bucket_boundary_jitter(self):
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        first = PoseStamped(Pose(Vector3(1.024, 2.0, 0.0)), frame_id="map")
        jitter = PoseStamped(Pose(Vector3(1.026, 2.0, 0.0)), frame_id="map")
        moved = PoseStamped(Pose(Vector3(1.08, 2.0, 0.0)), frame_id="map")

        assert self.mod._publish_goal_pose_once("go to chair", first)
        assert not self.mod._publish_goal_pose_once("go to chair", jitter)
        assert self.mod._publish_goal_pose_once("go to chair", moved)
        assert len(goals) == 2

    def test_goal_hysteresis_suppresses_small_yaw_noise(self):
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        first = PoseStamped(
            Pose(Vector3(1.0, 2.0, 0.0), Quaternion.from_yaw(0.0)),
            frame_id="map",
        )
        yaw_noise = PoseStamped(
            Pose(Vector3(1.0, 2.0, 0.0), Quaternion.from_yaw(math.radians(2.0))),
            frame_id="map",
        )

        assert self.mod._publish_goal_pose_once("face the chair", first)
        assert not self.mod._publish_goal_pose_once("face the chair", yaw_noise)
        assert len(goals) == 1

    def test_stale_scene_graph_does_not_publish_motion_goal(self):
        class _Resolver:
            def fast_resolve(self, instruction, sg_json):
                raise AssertionError("stale scene graph must not be resolved")

        self.mod._goal_resolver = _Resolver()
        self.mod._current_instruction = "go to chair"
        goals = []
        statuses = []
        self.mod.goal_pose._add_callback(goals.append)
        self.mod.planner_status._add_callback(statuses.append)

        self.mod._current_scene_graph = _make_scene_graph(["old chair"])
        stale = _make_scene_graph(["chair"])
        stale.ts = time.time() - 10.0
        self.mod._on_scene_graph(stale)

        assert goals == []
        assert statuses == ["WAITING_FOR_FRESH_SCENE_GRAPH"]
        assert self.mod._latest_sg is None
        assert self.mod._current_scene_graph is None

    def test_future_scene_graph_does_not_publish_motion_goal(self):
        class _Resolver:
            def fast_resolve(self, instruction, sg_json):
                raise AssertionError("future scene graph must not be resolved")

        self.mod._goal_resolver = _Resolver()
        self.mod._current_instruction = "go to chair"
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        future = _make_scene_graph(["chair"])
        future.ts = time.time() + 10.0
        self.mod._on_scene_graph(future)

        assert goals == []

    def test_nonfinite_scene_graph_timestamp_does_not_publish_motion_goal(self):
        class _Resolver:
            def fast_resolve(self, instruction, sg_json):
                raise AssertionError("nonfinite scene graph must not be resolved")

        self.mod._goal_resolver = _Resolver()
        self.mod._current_instruction = "go to chair"
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        invalid = _make_scene_graph(["chair"])
        invalid.ts = math.inf
        self.mod._on_scene_graph(invalid)

        assert goals == []
        assert self.mod._current_scene_graph is None

    def test_disabled_age_limit_still_rejects_invalid_source_timestamp(self):
        mod = _make_module(scene_graph_max_age_s=0.0)

        invalid = _make_scene_graph(["chair"])
        invalid.ts = math.nan

        assert mod._scene_graph_is_stale(invalid)
        mod.stop()

    def test_non_map_scene_graph_does_not_publish_motion_goal(self):
        class _Resolver:
            def fast_resolve(self, instruction, sg_json):
                raise AssertionError("non-map scene graph must not be resolved")

        self.mod._goal_resolver = _Resolver()
        self.mod._current_instruction = "go to chair"
        goals = []
        self.mod.goal_pose._add_callback(goals.append)

        odom_scene = _make_scene_graph(["chair"])
        odom_scene.frame_id = "odom"
        self.mod._on_scene_graph(odom_scene)

        assert goals == []
        assert self.mod._current_scene_graph is None


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
        """Test instruction triggers status."""
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
# 4. native navigation-state LERa cooldown
# ---------------------------------------------------------------------------


class TestSemanticPlannerRecovery:
    def test_recovering_triggers_lera_if_instruction_active(self):
        """RECOVERING with an active instruction must trigger LERa."""
        mod = _make_module()
        mod._on_scene_graph(_make_scene_graph())
        mod._on_odom(_make_odom())
        # Simulate an active instruction
        mod._current_instruction = "find the coffee machine"
        mod._on_navigation_state(
            NavigationState(boot_id="navd-test", sequence=1, lifecycle_state=NavigationLifecycle.RECOVERING)
        )

        time.sleep(0.05)
        mod.stop()

    def test_cooldown_prevents_double_lera(self):
        """A repeated RECOVERING status must be blocked by the cooldown."""
        mod = _make_module()
        mod._current_instruction = "find exit"
        state = NavigationState(boot_id="navd-test", sequence=1, lifecycle_state=NavigationLifecycle.RECOVERING)
        mod._on_navigation_state(state)
        last = mod._last_lera_time
        mod._on_navigation_state(state)
        # Second trigger is blocked by cooldown: _last_lera_time must not change
        assert mod._last_lera_time == last
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
        tool_list = [
            {
                "name": "hello",
                "description": "[FakeSkill] Return a greeting",
                "inputSchema": {
                    "type": "object",
                    "properties": {"name": {"type": "string"}},
                },
            }
        ]
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
        response = asyncio.run(
            agent._llm_call(
                [
                    {"role": "system", "content": "test"},
                    {"role": "user", "content": "say hello"},
                ]
            )
        )
        assert response["tool_calls"][0]["function"]["name"] == "hello"
        assert llm.messages is not None
        assert "hello" in llm.messages[0]["content"]


class TestSemanticPlannerAgentLoop:
    def test_on_system_modules_discovers_agent_skills(self):
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
        assert "emergency_stop" not in mod._agent_tool_registry

    def test_run_agent_loop_uses_discovered_skills(self):
        mod = AgentPlannerModule(llm_backend="mock")
        mod.on_system_modules(
            {
                "AgentPlannerModule": mod,
                "FakeSkillModule": _FakeSkillModule(),
            }
        )
        mod._llm_client = _RecordingLLM(
            [
                '{"tool":"hello","args":{"name":"lingtu"}}',
                '{"tool":"done","args":{"summary":"ok"}}',
            ]
        )

        state = asyncio.run(mod._run_agent_loop("greet the operator"))

        assert state.completed
        assert state.summary == "ok"
        assert any(m.get("role") == "tool" and "hello lingtu" in m.get("content", "") for m in state.messages)
