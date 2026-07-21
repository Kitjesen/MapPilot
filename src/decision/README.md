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
| `VisualServoModule` | `modules/visual_servo.py` | Near-target visual control. Receives camera/depth/intrinsics/scene graph plus `servo_target`. Publishes far `goal_pose`, close-range `cmd_vel`, and `nav_stop`. |
| `LLMModule` | `modules/llm.py` | Optional LLM backend module. The planner can also build its own LLM client directly. |

There is no standalone `main.py` under `src/decision/`. Decision modules are
started by the runtime blueprint system.

## How It Starts

The startup chain is:

```text
lingtu.py <profile>
  -> cli/profile resolution
  -> lingtu.assembly.full_stack / stack factories
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

Profiles with semantic planning enabled include `dev`, `sim`, `nav`,
`explore`, and `tare_explore`. `stub` and `sim_nav` are smaller profiles and may
not include the semantic stack.

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
        +--> goal_pose      -> nav.mission.goal_pose
        +--> cancel         -> nav.mission.cancel
        +--> servo_target   -> VisualServoModule.servo_target
        +--> task_plan      -> Gateway/status consumers
        +--> agent_message  -> Gateway chat/status stream
```

Typical visual-servo flow:

```text
Camera/color/depth/intrinsics
PerceptionModule.scene_graph
SemanticPlannerModule.servo_target or GatewayModule.servo_target
        |
        v
VisualServoModule
        |
        +--> goal_pose   -> nav.mission.goal_pose      (far target)
        +--> cmd_vel     -> nav.velocity_mux           (close target)
        +--> nav_stop    -> nav.mission.stop_signal    (pause planner while servoing)
```

## Directory Layout

| Path | Runtime status | Purpose |
| --- | --- | --- |
| `modules/` | Product entrypoints | Runtime `Module` classes registered into the blueprint system. |
| `goals/` | Product strategy | Fast/slow goal grounding, route selection, SG-Nav helper, tokenizer. |
| `tasks/` | Product strategy | Task decomposition, action shaping, agent loop, service wrappers. |
| `frontiers/` | Product strategy | Frontier extraction, scoring, and exploration fallback. |
| `llm/` | Product utility | LLM clients and prompt builders. |
| `vision/` | Product utility | BBox navigation, person tracking, Re-ID, VLM helpers. |
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
