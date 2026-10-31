"""Deterministic MuJoCo-truth semantic-grounding contract gate.

The fixture reads fixed object positions from a small MuJoCo model, then runs
the production TaskDecomposer, SemanticPlannerModule.resolve_task_step, and
GoalResolver against a hierarchical SceneGraph assembled by this fixture.
The model response, perception labels, room hierarchy, and relations are fixed
test inputs.  This gate proves symbolic decomposition and scene grounding only;
it does not run perception, advance the task plan, or execute robot motion.
"""

from __future__ import annotations

import argparse
import json
import time
from types import SimpleNamespace
from typing import Any

from decision.goals.resolver import GoalResolver, TargetBeliefManager
from decision.modules.semantic_planner import SemanticPlannerModule
from decision.tasks.decomposition import TaskDecomposer
from runtime.msgs.geometry import Vector3
from runtime.msgs.semantic import Detection3D, Relation, SceneGraph

INSTRUCTION = "先去会议室，然后找到桌子旁边的箱子"

# This is the only simulated LLM artifact.  It is deliberately symbolic:
# there are no coordinates, map frames, or executable navigation commands.
LLM_RESPONSE: dict[str, Any] = {
    "subgoals": [
        {"action": "navigate", "target": "会议室", "parameters": {}},
        {
            "action": "find",
            "target": "箱子",
            "parameters": {"relation": "near", "reference": "桌子"},
        },
    ]
}


def mujoco_snapshot() -> dict[str, Any]:
    """Read fixed object poses from a MuJoCo model as fixture truth."""
    import mujoco

    xml = """
    <mujoco model="lingtu-semantic-task-flow">
      <worldbody>
        <geom name="floor" type="plane" size="10 10 0.1"/>
        <body name="desk" pos="2 0 0.7"><geom name="desk_geom" type="box" size="0.7 0.45 0.7"/></body>
        <body name="box" pos="2.4 0.1 0.4"><geom name="box_geom" type="box" size="0.18 0.18 0.4"/></body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    desk_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "desk")
    box_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box")
    return {
        "backend": "mujoco",
        "source": "model_truth",
        "objects": {
            "desk": [float(value) for value in data.xpos[desk_id]],
            "box": [float(value) for value in data.xpos[box_id]],
        },
    }


def scene_snapshot(ts: float | None = None, physics: dict[str, Any] | None = None) -> SceneGraph:
    """Build the small building fixture used by the semantic simulation."""
    stamp = float(ts if ts is not None else time.time())
    physics_objects = physics.get("objects", {}) if physics else {}
    desk = physics_objects.get("desk", [2.0, 0.0, 0.7])
    box = physics_objects.get("box", [2.4, 0.1, 0.4])
    return SceneGraph(
        ts=stamp,
        frame_id="map",
        graph_level="hierarchical",
        graph_version="3.0",
        map_id="building",
        content_epoch=7,
        objects=[
            # Detector labels are normally canonical English concepts; the
            # symbolic query remains Chinese to exercise bilingual expansion.
            Detection3D(id="desk-1", label="desk", position=Vector3(*desk), ts=stamp),
            Detection3D(id="box-1", label="box", position=Vector3(*box), ts=stamp),
        ],
        relations=[Relation(subject_id="box-1", predicate="near", object_id="desk-1", confidence=0.95)],
        floors=[{"floor_id": 0, "room_ids": [1], "object_ids": ["desk-1", "box-1"]}],
        rooms=[
            {
                "room_id": 1,
                "name": "meeting_room",
                "center": {"x": float(desk[0]), "y": float(desk[1]), "z": 0.0},
                "object_ids": ["desk-1", "box-1"],
                "group_ids": [1],
            }
        ],
        groups=[{"group_id": 1, "room_id": 1, "name": "furniture", "object_ids": ["desk-1", "box-1"]}],
        hierarchy_edges=[
            {"parent_type": "floor", "parent_id": 0, "child_type": "room", "child_id": 1},
            {"parent_type": "room", "parent_id": 1, "child_type": "group", "child_id": 1},
            {"parent_type": "group", "parent_id": 1, "child_type": "object", "child_id": "desk-1"},
            {"parent_type": "group", "parent_id": 1, "child_type": "object", "child_id": "box-1"},
        ],
    )


def _offline_goal_resolver() -> GoalResolver:
    """Create the production resolver without constructing an LLM client.

    This simulation exercises the deterministic fast path.  Constructing the
    normal resolver would only add an unused LLM client and require an API-key
    warning, while no slow-path call is part of this evidence.
    """
    resolver = GoalResolver.__new__(GoalResolver)
    resolver._fast_path_attempts = 0
    resolver._fast_path_hits = 0
    resolver._resolve_times = []
    resolver._max_resolve_history = 100
    resolver._fast_path_threshold = 0.75
    resolver._scene_cache = None
    resolver._belief_manager = TargetBeliefManager()
    resolver._room_object_kg = None
    resolver._kg_path = None
    return resolver


def run() -> dict[str, Any]:
    """Run the symbolic LLM-to-fixture grounding contract gate."""
    physics = mujoco_snapshot()
    stamp = time.time()
    scene = scene_snapshot(stamp, physics)
    decomposer = TaskDecomposer()
    plan = decomposer.parse_decomposition_response(INSTRUCTION, LLM_RESPONSE)

    planner = SemanticPlannerModule(llm_backend="mock", map_query=SimpleNamespace())
    planner._task_decomposer = decomposer
    planner._goal_resolver = _offline_goal_resolver()
    planner._current_scene_graph = scene
    # The fixture is generated immediately before the query, so freshness is
    # deterministic even when this script is run under a slow debugger.
    planner._map_sample_is_stale = lambda _: False
    planner._timestamp_is_stale = lambda _: False

    motion_commands: list[Any] = []
    planner.nav_command.subscribe(motion_commands.append)

    groundings: list[dict[str, Any]] = []
    plan_errors = plan.validate()
    for step_index in range(len(plan.subgoals)):
        preview = plan.to_dict()
        preview["current_step"] = step_index
        for index, subgoal in enumerate(preview["subgoals"]):
            subgoal["status"] = "skipped" if index < step_index else "pending"
        result = json.loads(planner.resolve_task_step(json.dumps(preview, ensure_ascii=False)))
        result["grounded_step"] = step_index
        groundings.append(result)
        if result.get("status") not in {"candidate", "resolved"}:
            break

    return {
        "passed": (
            physics["backend"] == "mujoco"
            and physics["source"] == "model_truth"
            and not plan_errors
            and not plan.is_complete
            and plan.current_step == 0
            and len(groundings) == 2
            and all(step.get("status") in {"candidate", "resolved"} for step in groundings)
            and groundings[0].get("target_label") == "meeting_room"
            and groundings[1].get("candidate_id") == "box-1"
            and groundings[1].get("query") == "箱子 near 桌子"
            and all(step.get("map_id") == "building" and step.get("content_epoch") == 7 for step in groundings)
            and not motion_commands
        ),
        "instruction": INSTRUCTION,
        "physics": physics,
        "plan": plan.to_dict(),
        "groundings": groundings,
        "motion_commands": motion_commands,
        "scene": {
            "map_id": scene.map_id,
            "content_epoch": scene.content_epoch,
            "frame_id": scene.frame_id,
            "hierarchy_errors": list(scene.validate_hierarchy()),
        },
    }


def main() -> int:
    """Run the command-line simulation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="emit compact JSON")
    args = parser.parse_args()
    report = run()
    print(json.dumps(report, ensure_ascii=False, indent=None if args.json else 2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
