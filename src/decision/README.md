# Decision

`src/decision/` is the semantic decision layer. It turns high-level user
instructions and perception outputs into navigation goals, visual-servo targets,
cancel requests, task plans, and agent status messages.

Semantic planning is pure Python Module orchestration.

It is not the robot driver, SLAM, map builder, global planner, local planner, or
velocity arbiter. Those live in `src/drivers/`, `src/localization/`, and
`src/nav/`.

The local autonomy uses in-process C++/Python kernels in the navigation layer;
decision only chooses semantic intent and target outputs. Compute-heavy SLAM,
map building, global planning, local planning, and velocity arbitration stay
outside this package.

The old `src/decision/legacy/` Module wrappers have been removed. Runtime code
should use `modules/semantic_planner.py`, `modules/visual_servo.py`, and
`modules/llm.py`; tests for goal resolution, task decomposition, frontier
scoring, and action shaping target the strategy packages directly.

## Main Entry Point

The main program in this package is:

| Runtime object | File | Role |
| --- | --- | --- |
| `SemanticPlannerModule` | `modules/semantic_planner.py` | Primary semantic planner. Receives instructions, scene graph, odometry, topology summary, and mission status. Publishes `goal_pose`, `cancel`, `servo_target`, `task_plan`, and `agent_message`. |
| `VisualServoModule` | `modules/visual_servo.py` | Visual target selection and tracking. Receives the current map-frame detections, synchronized robot pose, selection image, and `servo_target`; publishes bounded map-frame goals and visual-task cancellation through native navigation. |
| `LLMModule` | `modules/llm.py` | Optional LLM backend module. The planner can also build its own LLM client directly. |

There is no standalone `main.py` under `src/decision/`. Decision modules are
started by the runtime blueprint system.

## How It Starts

The startup chain is:

```text
Product + env
  -> resolve one RunPlan
  -> build the selected Blueprint
  -> lingtu.assembly.stacks.planner()
  -> add SemanticPlannerModule, LLMModule, VisualServoModule
  -> lingtu.assembly.wires.semantic connects ports
```

The stack factory that creates decision modules is:

```text
src/lingtu/assembly/stacks/planner.py
```

The wires that connect decision modules to Gateway, MCP, Perception, Memory, and
Navigation are:

```text
src/lingtu/assembly/wires/semantic.py
```

Products such as `nav` and `explore` include semantic planning only when their
resolved Product/env RunPlan selects the semantic stack. Smaller development
Blueprints omit it explicitly.

## Runtime Data Flow

Typical instruction flow:

```text
GatewayModule.instruction
MCPServerModule.instruction
runtime nav input topic
        |
        v
SemanticPlannerModule.instruction
        |
        +-- goals/resolver.py          fast/slow target grounding
        +-- tasks/decomposition.py     task splitting
        +-- tasks/actions.py           action command shaping
        +-- frontiers/scorer.py        fallback frontier target
        |
        +--> goal_pose      -> nav.goals.goal_request
        +--> nav_command    -> nav.goals.goal_command
        +--> servo_target   -> VisualServoModule.servo_target
        +--> task_plan      -> Gateway/status consumers
        +--> agent_message  -> Gateway chat/status stream
```

Typical visual-servo flow:

```text
CameraModule.color_image
PerceptionModule.detections_3d + PerceptionModule.robot_pose
SemanticPlannerModule.servo_target or GatewayModule.servo_target
        |
        v
VisualServoModule
        |
        +--> goal_pose    -> nav.goals.visual_goal_request
        +--> goal_cancel  -> nav.goals.visual_cancel_request
        +<-- goal_status  <- nav.goals.goal_status
        +--> servo_status -> GatewayModule.visual_servo_status
```

The current Product that loads this stack is `tracking`. Within that
running Host, `find:<target>`, `follow:<target>`, and `stop` are hot task
switches; they do not reload modules or restart the Product. Follow goals are
updated at a bounded 2.5 Hz baseline and ignore sub-deadband target movement.
The status reports target visibility and the matching native navigation task;
stop remains `stopping` until native navigation publishes a terminal event.
If no image-capable selector is configured, find remains available while
description-based follow is unavailable instead of choosing an arbitrary
person; selecting a visible person by track ID remains available.

## Directory Layout

| Path | Runtime status | Purpose |
| --- | --- | --- |
| `modules/` | Runtime entrypoints | Runtime `Module` classes registered into the blueprint system. |
| `goals/` | Decision strategy | Fast/slow goal grounding, route selection, SG-Nav helper, tokenizer. |
| `tasks/` | Decision strategy | Task decomposition, action shaping, agent loop, service wrappers. |
| `frontiers/` | Decision strategy | Frontier extraction, scoring, and exploration fallback. |
| `llm/` | Decision utility | LLM clients and prompt builders. |
| `vision/` | Decision utility | BBox navigation, person tracking, Re-ID, VLM helpers. |
| `tests/` | Tests | Decision-layer regression tests. |

## Strategy Entrypoints

| Need | Use |
| --- | --- |
| Runtime semantic planning | `modules/semantic_planner.py::SemanticPlannerModule` |
| Near-target visual control | `modules/visual_servo.py::VisualServoModule` |
| Optional LLM module boundary | `modules/llm.py::LLMModule` |
| Goal grounding without Module wiring | `goals/resolver.py::GoalResolver` |
| Fast deterministic goal match | `goals/fast.py` |
| Slow LLM-backed goal match | `goals/slow.py` |
| Task decomposition | `tasks/decomposition.py::TaskDecomposer` |
| Action command shaping | `tasks/actions.py::ActionExecutor` |
| Frontier fallback scoring | `frontiers/scorer.py::FrontierScorer` |

Removed compatibility wrappers:

- `GoalResolverModule`
- `TaskDecomposerModule`
- `ActionExecutorModule`
- `FrontierModule`

Those names were test-only adapters around the strategy classes. They are not a
product startup surface anymore.

## Product Boundary

Allowed dependencies:

- `runtime`: Module base class, typed messages, registry, stream ports.
- `memory`: semantic/tagged/topological memory helpers.
- `decision.*`: local strategy packages.

Forbidden direct dependencies:

- `drivers`
- `nav`
- `gateway`
- `localization`

Decision talks to those layers through Module ports and blueprint wires, not by
importing their implementations.

## Functional Data

Some Chinese text remains intentionally in functional data:

- tokenizer vocabulary and bilingual label maps,
- rule-based Chinese command phrases,
- Chinese LLM/VLM prompts,
- tests that verify Chinese instruction handling.

Comments and docstrings should be English-only. Do not add explanatory Chinese
comments back into source code.
