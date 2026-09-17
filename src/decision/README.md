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
| `SemanticPlannerModule` | `modules/semantic_planner.py` | Primary semantic planner. Receives instructions, scene graph, synchronized map-frame robot pose, topology summary, native admission receipts and task lifecycle. Publishes task-scoped `nav_command`, `servo_target`, `task_plan`, and `agent_message`. |
| `AgentPlannerModule` | `modules/agent_planner.py` | Bounded multi-turn tasks. Delegates motion to the semantic owner, correlates native/visual completion, and scopes cancellation to its own run. |
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
  -> add SemanticPlannerModule, AgentPlannerModule, LLMModule, VisualServoModule
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

The current `inspection` Product declares `semantic_planning` and loads the Agent
through the same planner stack. Standard `nav`
does not; `tracking` selects perception and visual following but explicitly
forbids `SemanticPlannerModule`. Check the resolved RunPlan before attributing
a robot failure to this module or claiming an available semantic capability.

Agent location tagging uses the registered `tag_location` skill, which the
MCP/Gateway path supplies with pose and map-binding checks. Without that skill,
the agent does not advertise location tagging. There is no raw `tag:` port
fallback that reports success without a persistence acknowledgement.

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
        +--> nav_command    -> nav.goals.goal_command
        +<-- goal_status    <- nav.goals.goal_status (admission receipts)
        +<-- navigation_goal_status <- nav.goals.task_status (native lifecycle)
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

The current Product that loads the visual-following stack is `tracking`. Within that
running Host, `find:<target>`, `follow:<target>`, and `stop` are hot task
switches; they do not reload modules or restart the Product. Follow goals are
updated at a bounded 2.5 Hz baseline and ignore sub-deadband target movement.
The status reports target visibility and the matching native navigation task;
stop remains `stopping` until native navigation publishes a terminal event.
If no image-capable selector is configured, find remains available while
description-based follow is unavailable instead of choosing an arbitrary
person; selecting a visible person by track ID remains available.

## Semantic Task Boundaries And Remaining Gaps

A new instruction discards the previous scene goal. Only instructions routed
into scene resolution remain eligible for scene-driven updates. A named-place
command, a rejected symbolic command, or a follow handoff cannot be reinterpreted
by a later scene callback. Cached scene data is checked again when consumed.
The robot pose comes from `PerceptionModule.robot_pose`, in the same map frame
as the scene. Missing, expired or non-map poses defer scene-goal selection;
raw odometry coordinates cannot be used to rank map-frame object distances.
Current-object grounding excludes expired tracks without deleting scene memory.
An object centroid is not a robot pose: the planner proposes up to eight nearby
positions at the robot's current navigation height, facing the object, then uses
the registered `nav.commands.preview_plan` RPC off the perception callback.
Only a native-feasible map-frame path permits dispatch. This is a same-floor
proposal mechanism, not a visibility test or a multi-floor observation planner.
Unavailable or blocked previews never fall back to the object centroid.
Replacement, cancellation, map changes, expired poses and moved/disappeared
targets invalidate pending previews. Persistent scene IDs and capture times
come from the tracker; refreshing the graph cannot refresh an unseen object.
Goals and cancellations carry separate task and request identities. GoalService
validates native lifecycle evidence before forwarding it. A cancel receipt means
admitted, not stopped; only a native terminal event ends that geometric task.
Late receipts and old tasks cannot overwrite the current task's outcome.

Semantic recovery starts only on its own native FAILED terminal. Scene updates
do not replace a paused or recovering native goal. Background results match both
the instruction epoch and goal; a replaced instruction or stopped Module cannot
dispatch a delayed retry. Cooldown delays a new failure instead of dropping it.
Map identity changes and native restarts invalidate the cached semantic goal.
Visual handoffs stop on replacement and are not restarted on every scene frame.
An admitted object approach retains its tracked target when later recognition
is missing or lower confidence; native navigation still owns path failure and
obstacle handling. Goal deduplication includes the purpose and object identity,
so two nearby candidates do not suppress each other's task lifecycle.

`send_instruction` invokes the same instruction handler used by the input port.
Agent object requests delegate to this semantic owner, including observation
preview and target verification. Agent coordinate requests accept explicit `z`;
omitting it requires a fresh map-frame pose from perception, never raw odometry
or an assumed zero floor. A map/native context change invalidates that pose.
Agent loop errors, empty replies, timeout and exhausted steps report failure.
The model must explicitly choose `done(success=true|false, summary=...)`; an
incomplete outcome reports failure. A success claim cannot override unsuccessful
motion. Coordinate tools wait for their correlated native terminal result; object
tools wait for visual verification, including bounded candidate reselection.
`AGENT_DONE` on a task without motion still reports conversational completion,
not independently verified task success.

`run_agent_task` returns a run ID. `cancel_agent_task(run_id)` affects only that
run, and `get_agent_status` separates loop state from motion results. One managed
event loop owns the model client across tasks. Replacement, cancellation, map
changes and native restarts invalidate late replies. The semantic instruction
revision also prevents the first pending model reply from replacing a newer
manual semantic instruction. A newly accepted operator/API goal received through
GoalService (`goal` or `goal_pose`) also invalidates pending semantic instructions,
model replies and observation previews. Cancellation stays scoped to previously
owned tasks; the operator's new task is not cancelled. Rejected goals, replayed
receipts and delegated visual-servo goals do not trigger this takeover. This has
local integration coverage, not field evidence or coverage of every teleop input.
Cancellation acknowledgement does not prove the robot has stopped.

Place lookup refusal, missing place clarification and symbolic interpretation
failure are terminal failures in the owned-instruction query. A new instruction
clears that failure. Delayed place-query failures carry the instruction revision
so that an old result cannot overwrite the replacement task's status.

Agent context excludes stale scene objects and uses camera frames only when the
processed image, scene graph and fresh map pose have matching timestamps. Motion
skills without Agent ownership (`vla_navigate`, `go_to_tag`) are excluded from
its automatic tool discovery. The VLA status query is `get_vla_status`; the
`vla_status` name belongs to its telemetry output port.

Known-place REACHED reports COMPLETED. A reached object goal with a tracked
target starts bounded visual verification and stops scene-driven goal updates.
The exact processed color frame arrives through `PerceptionModule.observation_image`;
its timestamp must match the scene, target observation and map-frame robot pose.
Only post-arrival observations of the same nearby target are submitted to the
registered image-capable LLM. It receives the full scene, candidate crop and the
original instruction, including attribute and relationship constraints.

Two positive observations, at least 0.5 seconds apart, report COMPLETED; repeated
timestamps cannot count twice. These are temporal observations, not guaranteed
independent viewpoints. Each view allows up to three calls. If inconclusive,
the planner previews a different observation position through native navigation.
At most three positions are tried for one object within the original 20-second
verification deadline; arrival at another position does not reset this deadline.
No images are verified while that native motion task is active. New observation
goals exclude positions within 0.30 m of tried/current positions and request a
0.15 m arrival radius. These are provisional engineering settings, not calibrated
guarantees of useful visibility or independent evidence.

A visible contradiction records TARGET_MISMATCH for that candidate. A bounded
search can then select another freshly observed object using the full instruction.
It can also switch candidates after exhausting inconclusive/reachable views.
At most three object candidates are attempted per instruction; blocked candidates
count toward that limit. Candidates already tried are excluded only for that
instruction's candidate scoring; they remain in scene context as possible
landmarks for relational instructions. Changing the instruction, map or native boot clears the search.
Uncertainty is retained as uncertainty, not relabeled as a visual mismatch.
Timeout, missing vision capability, encoding failure and model error are explicit
non-success outcomes. The declared client capability is checked: the current
MoonshotClient adapter enables this path for `kimi-k2.6`; the mock backend and
other unverified Moonshot model configurations do not claim image support.
This describes our adapters, not every model supported by those vendors.

The `kimi` and `moonshot` aliases share the Open Platform defaults:
`kimi-k2.6`, `MOONSHOT_API_KEY`, and `https://api.moonshot.cn/v1`. K2.6 requests
disable thinking and omit temperature, following its fixed sampling contract.
The old K2.5 default was retired by the provider on 2026-08-31; an explicitly
configured model is not silently upgraded. Kimi Code is a separate service and
is not the default endpoint. See the [current model list](https://platform.kimi.com/docs/models)
and [K2.6 request contract](https://platform.kimi.com/docs/guide/kimi-k2-6-quickstart).
Install the existing `llm` extra in an environment that will call real models.
Local transport tests use the real SDK with in-memory HTTP responses, not a
paid API or field images; provider availability, recognition accuracy and field
latency still require separate evidence.

Image and text calls share request handling and preserve the selected model.
Only final content is consumed as a verdict. Request latency includes dispatch
and inference waiting, including failures; empty timeout exceptions remain
explicit errors. Changing providers resets provider-specific defaults unless
overridden in that request. Stopping cancels pending calls and closes the client;
reconfiguration closes the retired client.

Native geometric state remains separate from `navigation_goal.verification` in
`get_planner_status`; Gateway receives verification progress through the existing
agent-message wire. `object_candidates` and `object_verifications` preserve the
current instruction's bounded candidate history. A verification timeout during
re-observation requests scoped native cancellation; its ACK does not prove stop.
Late REACHED cannot resurrect an expired verification. Paused/recovering native
tasks retain motion ownership. A new view's admission failure does not trigger
unbounded LERa recovery.

Memory/frontier goals without a tracked instance still report
TARGET_VERIFICATION_REQUIRED. Re-observation and alternate observed candidates
have local contract coverage; recorded-image accuracy, native simulation and field
motion remain unverified. The candidate fallback is not whole-building search:
unseen candidates still require the missing frontier/map connection described below.

`FrontierScorer` is an algorithm, not an active map subscriber. The planner's
private scorer currently has no map-input/update/extraction wiring. Reading
`center_world` correctly and passing service tests does not make field semantic
exploration available. `SGNavReasoner` and `verify_and_reselect` likewise have no
runtime planner callers. Their unit tests prove helpers, not the full search and
re-observation loop.

The strategy service regressions use concrete resolver, frontier scorer, and
action executor instances. Only model responses are replaced for offline tests;
permissive mocks cannot establish API compatibility or semantic model quality.
See [the research and integration review](../../research/semantic/navigation_review.md)
for source-backed method choices and remaining field gates.

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
