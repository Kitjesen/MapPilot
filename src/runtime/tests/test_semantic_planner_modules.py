"""Tests for semantic planner Module conversions.

Covers:
- GoalResolverModule: instruction + scene graph -> GoalResult (offline mode)
- TaskDecomposerModule: instruction -> task_plan with subgoals
- ActionExecutorModule: GoalResult -> PoseStamped
- autoconnect: Perception.scene_graph -> GoalResolver.scene_graph wiring
- All tests work WITHOUT LLM API keys (offline/mock mode)
"""

import json
import math


from runtime import Module, Out, autoconnect
from runtime.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.semantic import Detection3D, SceneGraph
from runtime.msgs.semantic import GoalResult as MsgGoalResult
from decision.modules.action_executor_module import ActionExecutorModule
from decision.modules.goal_resolver_module import GoalResolverModule
from decision.modules.task_decomposer_module import TaskDecomposerModule

# ============================================================================
# Helpers
# ============================================================================

def _make_scene_graph(objects_data):
    """Create SceneGraph from list of (label, x, y, z, confidence) tuples."""
    objects = []
    for i, (label, x, y, z, conf) in enumerate(objects_data):
        objects.append(Detection3D(
            id=f"obj_{i}", label=label, confidence=conf,
            position=Vector3(x, y, z)))
    return SceneGraph(objects=objects)


def _make_odom(x, y, z=0.0):
    return Odometry(pose=Pose(position=Vector3(x, y, z)))


def _collect(module, port_name):
    events = []
    port = getattr(module, port_name)
    port._add_callback(events.append)
    return events


# ============================================================================
# Stub modules for autoconnect tests
# ============================================================================

class StubPerceptionModule(Module, layer=3):
    scene_graph: Out[SceneGraph]


class StubDriverModule(Module, layer=1):
    odometry: Out[Odometry]


class StubInstructionSource(Module, layer=6):
    instruction: Out[str]


# ============================================================================
# TestGoalResolverModule -- Port scanning
# ============================================================================

class TestGoalResolverModulePorts:

    def test_ports_detected(self):
        mod = GoalResolverModule()
        assert "scene_graph" in mod.ports_in
        assert "instruction" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "resolved_goal" in mod.ports_out
        assert "planner_status" in mod.ports_out

    def test_port_types(self):
        mod = GoalResolverModule()
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_in["instruction"].msg_type is str
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_out["resolved_goal"].msg_type is MsgGoalResult
        assert mod.ports_out["planner_status"].msg_type is str

    def test_layer(self):
        assert GoalResolverModule._layer == 4
        mod = GoalResolverModule()
        assert mod.layer == 4


# ============================================================================
# TestGoalResolverModule -- Offline Fast Path
# ============================================================================

class TestGoalResolverOfflineFastPath:

    def test_chair_instruction_matches(self):
        mod = GoalResolverModule(use_slow_path=False)
        mod.setup()
        goals = _collect(mod, "resolved_goal")
        sg = _make_scene_graph([
            ("chair", 5.0, 3.0, 0.0, 0.85),
            ("table", 2.0, 1.0, 0.0, 0.90),
        ])
        mod.scene_graph._deliver(sg)
        mod.odometry._deliver(_make_odom(0, 0))
        mod.instruction._deliver("chair")
        assert len(goals) == 1
        result = goals[0]
        assert result.is_valid is True
        assert result.target_label == "chair"
        assert result.target_x == 5.0
        assert result.target_y == 3.0
        assert result.confidence > 0.5
        assert result.path == "fast"

    def test_chinese_instruction_matches(self):
        mod = GoalResolverModule(use_slow_path=False)
        mod.setup()
        goals = _collect(mod, "resolved_goal")
        sg = _make_scene_graph([
            ("bed", 10.0, 5.0, 0.35, 0.92),
        ])
        mod.scene_graph._deliver(sg)
        mod.instruction._deliver("bed")
        assert len(goals) == 1
        assert goals[0].target_label == "bed"
        assert goals[0].confidence > 0.5

    def test_no_match_publishes_failed(self):
        mod = GoalResolverModule(use_slow_path=False)
        mod.setup()
        goals = _collect(mod, "resolved_goal")
        statuses = _collect(mod, "planner_status")
        sg = _make_scene_graph([
            ("table", 2.0, 1.0, 0.0, 0.90),
        ])
        mod.scene_graph._deliver(sg)
        mod.instruction._deliver("elephant")
        assert len(goals) == 0
        last_status = json.loads(statuses[-1])
        assert last_status["state"] == "failed"

    def test_empty_scene_graph_fails(self):
        mod = GoalResolverModule(use_slow_path=False)
        mod.setup()
        goals = _collect(mod, "resolved_goal")
        mod.instruction._deliver("chair")
        assert len(goals) == 0

    def test_empty_instruction_ignored(self):
        mod = GoalResolverModule()
        mod.setup()
        statuses = _collect(mod, "planner_status")
        mod.instruction._deliver("")
        mod.instruction._deliver("   ")
        assert len(statuses) == 0

    def test_best_confidence_selected(self):
        mod = GoalResolverModule(use_slow_path=False)
        mod.setup()
        goals = _collect(mod, "resolved_goal")
        sg = _make_scene_graph([
            ("chair", 1.0, 1.0, 0.0, 0.60),
            ("chair", 5.0, 5.0, 0.0, 0.95),
            ("chair", 3.0, 3.0, 0.0, 0.70),
        ])
        mod.scene_graph._deliver(sg)
        mod.instruction._deliver("chair")
        assert len(goals) == 1
        assert goals[0].target_label == "chair"
        assert goals[0].is_valid is True
        assert goals[0].confidence > 0.5

    def test_status_resolving_then_resolved(self):
        mod = GoalResolverModule()
        mod.setup()
        statuses = _collect(mod, "planner_status")
        sg = _make_scene_graph([("door", 3.0, 4.0, 0.0, 0.88)])
        mod.scene_graph._deliver(sg)
        mod.instruction._deliver("door")
        parsed = [json.loads(s) for s in statuses]
        states = [p["state"] for p in parsed]
        assert "resolving" in states
        assert "resolved" in states
        assert states.index("resolving") < states.index("resolved")

    def test_odometry_used_in_status(self):
        mod = GoalResolverModule()
        mod.setup()
        mod.odometry._deliver(_make_odom(1.0, 2.0, 0.5))
        assert mod._latest_odom is not None
        assert mod._latest_odom.x == 1.0


# ============================================================================
# TestTaskDecomposerModule
# ============================================================================

class TestTaskDecomposerModulePorts:

    def test_ports_detected(self):
        mod = TaskDecomposerModule()
        assert "instruction" in mod.ports_in
        assert "task_plan" in mod.ports_out

    def test_port_types(self):
        mod = TaskDecomposerModule()
        assert mod.ports_in["instruction"].msg_type is str
        assert mod.ports_out["task_plan"].msg_type is dict

    def test_layer(self):
        assert TaskDecomposerModule._layer == 4


class TestTaskDecomposerOffline:

    def test_basic_decomposition(self):
        mod = TaskDecomposerModule()
        mod.setup()
        plans = _collect(mod, "task_plan")
        mod.instruction._deliver("go to the kitchen")
        assert len(plans) == 1
        plan = plans[0]
        assert "instruction" in plan
        assert "subgoals" in plan
        assert len(plan["subgoals"]) >= 1

    def test_plan_has_navigate_subgoal(self):
        mod = TaskDecomposerModule()
        mod.setup()
        plans = _collect(mod, "task_plan")
        mod.instruction._deliver("find the red cup")
        plan = plans[0]
        actions = [sg["action"] for sg in plan["subgoals"]]
        assert "navigate" in actions

    def test_empty_instruction_ignored(self):
        mod = TaskDecomposerModule()
        mod.setup()
        plans = _collect(mod, "task_plan")
        mod.instruction._deliver("")
        assert len(plans) == 0

    def test_plan_dict_structure(self):
        mod = TaskDecomposerModule()
        mod.setup()
        plans = _collect(mod, "task_plan")
        mod.instruction._deliver("patrol the corridor")
        plan = plans[0]
        assert "instruction" in plan
        assert "total_steps" in plan
        assert "current_step" in plan
        assert "is_complete" in plan
        assert "subgoals" in plan
        for sg in plan["subgoals"]:
            assert "step_id" in sg
            assert "action" in sg
            assert "target" in sg


# ============================================================================
# TestActionExecutorModule
# ============================================================================

class TestActionExecutorModulePorts:

    def test_ports_detected(self):
        mod = ActionExecutorModule()
        assert "resolved_goal" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "scene_graph" in mod.ports_in
        assert "goal_pose" in mod.ports_out
        assert "cmd_vel" in mod.ports_out
        assert "execution_status" in mod.ports_out

    def test_port_types(self):
        mod = ActionExecutorModule()
        assert mod.ports_in["resolved_goal"].msg_type is MsgGoalResult
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_in["scene_graph"].msg_type is SceneGraph
        assert mod.ports_out["goal_pose"].msg_type is PoseStamped
        assert mod.ports_out["cmd_vel"].msg_type is Twist
        assert mod.ports_out["execution_status"].msg_type is str

    def test_layer(self):
        assert ActionExecutorModule._layer == 4


class TestActionExecutorExecution:

    def test_valid_goal_produces_pose(self):
        mod = ActionExecutorModule()
        mod.setup()
        poses = _collect(mod, "goal_pose")
        statuses = _collect(mod, "execution_status")
        goal = MsgGoalResult(
            action="navigate", target_label="chair",
            target_x=5.0, target_y=3.0, target_z=0.0,
            confidence=0.9, is_valid=True, path="fast")
        mod.resolved_goal._deliver(goal)
        assert len(poses) == 1
        assert poses[0].x == 5.0
        assert poses[0].y == 3.0
        assert poses[0].frame_id == "map"
        status = json.loads(statuses[0])
        assert status["state"] == "executing"
        assert status["target_label"] == "chair"

    def test_invalid_goal_rejected(self):
        mod = ActionExecutorModule()
        mod.setup()
        poses = _collect(mod, "goal_pose")
        statuses = _collect(mod, "execution_status")
        goal = MsgGoalResult(action="navigate", is_valid=False)
        mod.resolved_goal._deliver(goal)
        assert len(poses) == 0
        status = json.loads(statuses[0])
        assert status["state"] == "rejected"

    def test_yaw_toward_target(self):
        mod = ActionExecutorModule()
        mod.setup()
        poses = _collect(mod, "goal_pose")
        mod.odometry._deliver(_make_odom(0, 0))
        goal = MsgGoalResult(
            action="navigate", target_x=1.0, target_y=1.0,
            is_valid=True, path="fast")
        mod.resolved_goal._deliver(goal)
        pose = poses[0]
        expected_yaw = math.atan2(1.0, 1.0)
        qz = pose.pose.orientation.z
        qw = pose.pose.orientation.w
        actual_yaw = 2.0 * math.atan2(qz, qw)
        assert abs(actual_yaw - expected_yaw) < 0.01

    def test_lera_recovery_escalation(self):
        mod = ActionExecutorModule(max_lera_retries=3)
        mod.setup()
        r1 = mod.lera_recover("navigate", "chair")
        assert r1 == "retry_different_path"
        r2 = mod.lera_recover("navigate", "chair")
        assert r2 == "expand_search"
        r3 = mod.lera_recover("navigate", "chair")
        assert r3 == "abort"

    def test_reset_failure_count(self):
        mod = ActionExecutorModule(max_lera_retries=3)
        mod.setup()
        mod.lera_recover("navigate", "chair")
        mod.lera_recover("navigate", "chair")
        mod.reset_failure_count()
        r = mod.lera_recover("navigate", "chair")
        assert r == "retry_different_path"


# ============================================================================
# Test autoconnect wiring
# ============================================================================

class TestAutoconnectWiring:

    def test_perception_to_goal_resolver_wiring(self):
        system = autoconnect(
            StubPerceptionModule.blueprint(),
            StubDriverModule.blueprint(),
            GoalResolverModule.blueprint(),
        ).build()
        connections = system.connections
        sg_wires = [(o, op, i, ip) for o, op, i, ip in connections
                    if ip == "scene_graph" and i == "GoalResolverModule"]
        assert len(sg_wires) == 1
        assert sg_wires[0][0] == "StubPerceptionModule"

    def test_odometry_wiring(self):
        system = autoconnect(
            StubDriverModule.blueprint(),
            GoalResolverModule.blueprint(),
        ).build()
        connections = system.connections
        odom_wires = [(o, op, i, ip) for o, op, i, ip in connections
                      if ip == "odometry" and i == "GoalResolverModule"]
        assert len(odom_wires) == 1

    def test_goal_resolver_to_action_executor_wiring(self):
        system = autoconnect(
            GoalResolverModule.blueprint(),
            ActionExecutorModule.blueprint(),
        ).build()
        connections = system.connections
        goal_wires = [(o, op, i, ip) for o, op, i, ip in connections
                      if ip == "resolved_goal" and i == "ActionExecutorModule"]
        assert len(goal_wires) == 1
        assert goal_wires[0][0] == "GoalResolverModule"

    def test_full_pipeline_wiring(self):
        system = autoconnect(
            StubPerceptionModule.blueprint(),
            StubDriverModule.blueprint(),
            StubInstructionSource.blueprint(),
            GoalResolverModule.blueprint(),
            TaskDecomposerModule.blueprint(),
            ActionExecutorModule.blueprint(),
        ).build()
        assert system.started is False
        system.start()
        assert system.started is True
        health = system.health()
        assert health["module_count"] == 6
        assert health["connection_count"] >= 3
        system.stop()
        assert system.started is False

    def test_end_to_end_data_flow(self):
        system = autoconnect(
            StubPerceptionModule.blueprint(),
            StubDriverModule.blueprint(),
            GoalResolverModule.blueprint(),
            ActionExecutorModule.blueprint(),
        ).build()
        system.start()
        perception = system.get_module("StubPerceptionModule")
        resolver = system.get_module("GoalResolverModule")
        sg = _make_scene_graph([("chair", 5.0, 3.0, 0.0, 0.85)])
        perception.scene_graph.publish(sg)
        assert resolver._latest_sg is not None
        assert len(resolver._latest_sg.objects) == 1
        assert resolver._latest_sg.objects[0].label == "chair"
        system.stop()
