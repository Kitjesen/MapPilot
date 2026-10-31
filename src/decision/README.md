# Decision layer

`src/decision/` is the Python semantic decision layer. It converts user intent
and synchronized perception into typed task decisions, semantic targets,
visual-servo requests, and status. It does not own sensors, SLAM, maps, global
planning, local planning, velocity arbitration, or the robot driver.

Native navigation remains the authority for geometry and motion. The decision
layer submits goals, waits for native lifecycle evidence, and reports results.
A command acknowledgement is not proof that the robot moved or stopped. This
document describes current code, not a future architecture.

## Runtime entry points

The Host assembly creates these Modules when the selected Product enables them:

| Module | Source | Responsibility / owned state |
| --- | --- | --- |
| `SemanticPlannerModule` | [semantic_planner.py](modules/semantic_planner.py) | Current instruction, observed target, pending preview/model requests, verification, recovery and owned navigation goals. |
| `AgentPlannerModule` | [agent_planner.py](modules/agent_planner.py) | Current `AgentRun`, cancellation and owned skill results; [AgentLoop](tasks/agent.py) holds model messages and tool progression. |
| `VisualServoModule` | [visual_servo.py](modules/visual_servo.py) | Person reference, visual phase, Follow task records and native child goals; also retains the simpler visual Find mode. |
| `LLMModule` | [llm.py](modules/llm.py) | Model request/response handling and client lifetime; does not own a robot task. |

Assembly and port ownership are defined in:

- [semantic task stack](../lingtu/assembly/host/stacks/semantic_tasks.py)
- [semantic wires](../lingtu/assembly/host/wiring/semantic.py)

`nav/semantic` in `env=real` loads RGB-D perception, semantic planning, the
Agent stack, Follow, and native navigation. Its model declaration and process
graph are product wiring, not proof that weights are installed or the target
board is qualified. `tracking` loads perception and visual following but does
not load `SemanticPlannerModule`.
Inspect the resolved RunPlan before claiming a capability is active.

## Current call graph

```text
Gateway /api/v1/instruction and MCP send_instruction ---> AgentPlannerModule
    returns run_id; AgentLoop selects bounded domain tools
    owned motion handlers ---> SemanticPlannerModule ---> nav.goals
    tool results <--- owner status <--- native admission / lifecycle

Host without Agent ---> semantic instruction stream fallback
Explicit decompose_task ---> rules or needs_llm prompt; read-only parsing API

nav.goals <----> native global/local navigation and execution

PerceptionModule.observation ---> SemanticPlanner + VisualServo
Scene graph / pose / image ---> Agent and memory through separate inputs
```

The common `SemanticObservation` input is used by `SemanticPlannerModule` and
`VisualServoModule`. Agent context and memory still consume older separate
scene, pose, and image inputs; they have not migrated to that contract.

## Two parallel task mechanisms

`tasks/decomposition.py` contains symbolic `TaskPlan`, `SubGoal`, action
vocabulary, validation, and rule/LLM prompt parsing. It is a plan data model and
parser. Scene grounding belongs to `SemanticPlannerModule.resolve_task_step()`;
the decomposition helper does not itself drive navigation.

`tasks/agent.py` and `modules/agent_planner.py` contain the separate AgentLoop:
the model selects bounded tools, receives results, and eventually calls
`done(success=...)`. `run_agent_task()` returns an Agent `run_id`; status and
cancellation are scoped to that run. `get_agent_status()` reports the current
run; it is not an arbitrary historical run lookup. The AgentLoop and `TaskPlan`
are related but parallel mechanisms today. There is no single persistent
Mission object owning both.

The regular HTTP and MCP instruction adapters prefer `AgentPlannerModule` and
return its `run_id`. A Host that did not load Agent retains the existing
semantic instruction-stream fallback. `SemanticPlannerModule.send_instruction`
remains a Python domain method for owned execution, but it no longer exposes a
second generic `@skill` owner. Explicit `decompose_task()` remains a read-only
parse operation; a `needs_llm` result is a prompt, not an execution record.

## Intent, task, and model boundaries

`LLMModule` is the shared request boundary wired by Host. The Agent also owns a
model client/event loop for its tool loop. Host assembly now passes the same
resolved backend, model, base URL, API-key environment-variable name and request
timeout into both. AgentLoop applies that request timeout as a total bound for
each model call, separately from the whole task deadline. The two callers still
own separate client lifetimes. Matching configuration does not prove model
quality or physical task completion.

OpenAI-compatible clients, including Kimi, now send native `tools` and receive
`tool_calls`. The streaming client assembles argument fragments by call index
and retains the assistant's content, tool-call identity and `reasoning_content`
for the next request. Reasoning is protocol history, not operator-facing text.
The existing text-JSON fallback remains for clients without `chat_with_tools`.
Ordinary `chat()` and vision callers still receive text.

The Agent chooses one tool per turn. The request sets
`parallel_tool_calls=false`, and the client also rejects a multi-call reply
before any handler executes. Truncated or incomplete replies are not executed;
arguments must decode to an object and pass the existing tool validation.
Cancellation and timeout close the response stream. Owned motion and completion
evidence remain the responsibility of the existing domain handlers.

Calls are event-driven, not a fixed-rate control loop: instruction -> model
decision -> await tool result -> next decision or `done`. A query can return
quickly; a find or navigation tool waits for its owned domain result. Follow
waits for its domain task's termination or cancellation and does not repeatedly
ask the model to generate velocity commands. The model-request deadline is
separate from this wait and the Agent task budget. See
[native transport tests](../../tests/decision/test_llm_native_tools.py) for the
two-round message, cancellation and malformed-reply contracts.

`SemanticPlannerModule.decompose_task()` uses `TaskDecomposer`. Rule parsing
can return `status=parsed`; complex input returns `status=needs_llm` and a
decomposition prompt. `resolve_task_step()` grounds a validated plan against
the current scene but deliberately does not publish a navigation goal. Ordinary
instructions now use AgentLoop for model-selected tool execution; `TaskPlan`
still does not drive that loop or provide one persistent multi-step executor.

Execution-oriented Agent runs require domain evidence. A model calling
`done(success=true)` without an attempted and successful owned motion is
rejected. Explicit pure status plans are informational and do not require motion.
Runs containing FIND also require object-confirmation evidence from the owned
semantic object path. Coordinate arrival cannot complete a find request. For
the default single-target find workflow, that path receives the user's original
instruction, so a model-supplied label cannot weaken the requested target.

## Existing callable skills and APIs

Methods marked `@skill` are discovered by the MCP server at runtime and exposed
with generated JSON schemas. HTTP navigation routes are separate Gateway APIs.
The declaration and Schema belong to [runtime.module](../runtime/module.py);
[MCP](../gateway/mcp_server.py) adapts external calls, while
[Agent tools](tasks/agent.py) adapt model calls. These are distinct responsibilities.
Both use [shared discovery](../runtime/skills.py): aliases of one instance are
collapsed and different owners with the same skill name are rejected. MCP
explicitly keeps its built-in fallback policy; Agent retains its blocklist and
task-owned motion handlers. The configured `@skill(...)` form adds parameter
descriptions, optional object output schemas and a read-only hint. MCP exposes
structured object results; Agent receives the output/behavior contract in the
tool description and JSON-encoded object results. See the
[skill contract](../runtime/README.md#skill-contract) for the declaration boundary.

Follow's external `start_follow` returns an admission receipt and requires a
task/session query. Agent's internal `follow_person` waits for its owned domain
result. Shared discovery does not make those two operations equivalent or expose
the blocked raw motion tools to the Agent.

| Capability | Current entry point | Current boundary |
| --- | --- | --- |
| Intent input | `POST /api/v1/instruction`, `MCPServerModule.send_instruction` | Prefers Agent and returns `run_id`; falls back to the semantic stream only when Agent is absent. Admission is not completion. |
| Agent task | `AgentPlannerModule.run_agent_task` | Returns `run_id`; model/tool loop owns progression. |
| Agent status/cancel | `get_agent_status`, `cancel_agent_task`; HTTP `/api/v1/agent/runs/{run_id}` and `/cancel` | Current-Host run only. Cancellation is scoped to the referenced run; query domain/native state for physical stop evidence. |
| Scene read | `get_scene_graph`, `get_scene_objects` | Reads the latest cached observation. |
| Object query | `detect_objects(query)` | Case-insensitive label matching in cached scene JSON; not a fresh detector call. |
| Memory query | `query_memory(query)` | Searches available episodic/spatial/vector memory. |
| Semantic find | `navigate_to_object(instruction)` | Enters the semantic planner and its object grounding/verification path. |
| Agent object find | Agent `_tool_find_object` in `modules/agent_planner.py` | Sends an owned semantic instruction through the Agent's task path; it is not the MCP visual trigger. |
| MCP visual find | `VisualServoModule.find_object(target)` | Calls the visual-mode handler with `find:<target>`; it has no find task lifecycle. |
| Coordinate navigation | `nav.skills.navigate_to`, `POST /api/v1/goal` | Submits a map-frame goal to native navigation. |
| Plan preview | `POST /api/v1/navigation/plan` | Previews feasibility without publishing a goal. |
| Native status/control | `get_navigation_status`, task status/cancel/pause/resume routes | Queries or controls native task lifecycle. |

HTTP routes are in [navigation routes](../gateway/navigation/routes.py). MCP
discovery and built-in read tools are in [MCP server](../gateway/mcp_server.py).

## Find and Follow status

Follow has a real task lifecycle:

```text
start_follow -> follow_task_id + product_session_id
             -> get_follow_task
             -> cancel_follow_task
             -> cancel request -> native child terminal -> cancellation result
```

The task is retained for the Host's last 64 executions and does not recover
after a Host restart. Camera restart or map change invalidates stale visual
decisions and cancels owned navigation goals. Reacquisition requires existing
identity checks, multiple views, and three consecutive observations. A native
child arrival is not automatically the domain-level completion of Follow; the
Follow task has its own state and status. See [person-follow replay notes](../../docs/research/person-follow.md)
for evidence and limits.

For the recent-paper comparison, current failure cases and proposed changes,
use the [person tracking development reference](../../docs/research/person-tracking-development-reference.md).
Its paper mechanisms and external timings do not establish implemented or
field-qualified behaviour.

Follow prepares appearance features outside its lifecycle and tracking state
locks, then checks the intent generation, observation sequence and freshness
before committing. Encoding failure does not trigger a second inference under
the commit lock. The generic single-person shortcut cannot bypass an appearance
description. These are locally tested contracts; target-platform timing remains
unqualified.

When Agent starts Follow, the sustained domain wait is not charged against the
finite Agent reasoning/action deadline. Model calls still keep their per-call
timeout, and ordinary navigation/find actions still keep the Agent task
deadline. Agent retains the exact `follow_task_id` and reads that same task's
terminal domain status; it does not infer Follow success or failure from an
expired reasoning budget. Explicit cancellation remains immediate and scoped.

`VisualServoModule.find_object()` is much thinner:

```text
find_object(target) -> _on_servo_target("find:<target>") -> visual mode
```

It returns a message string and has no dedicated find task ID, task query, or
task cancellation lifecycle. `get_servo_status()` reports the broader servo
state. Do not describe this as the same complete lifecycle as Follow.

## Perception and semantic grounding

`PerceptionModule.observation` carries synchronized image, detections, scene,
pose, map identity, and source context. Semantic planning checks publication
order, session, source time, and map binding. An object centroid is not a
motion goal: the planner proposes nearby map-frame observation positions and
uses native plan preview before dispatch. Expired or unavailable observations
do not authorize motion.

Object verification is bounded by instruction, candidate history, native task
ownership, and a verification deadline. Late native receipts and old model
replies cannot overwrite a newer instruction. The local record is
`artifacts/semantic-observation-20261016/README.md`; it uses controlled
detector/native doubles and is not field or learned-model evidence.

When an object query has no current candidate, SemanticPlanner can request
observation viewpoints from the native `/nav/semantic/views/request` service.
The request carries the current map identity, frame epoch, camera range/FoV and
actual observation history. Every returned candidate still requires native 3D
plan preview before dispatch. Reaching a search viewpoint only triggers a fresh
observation; it is never reported as finding the object. The bridge passed
local Python ABI/lifetime checks, a WSL native build, real CycloneDDS correlated
request/result and bounded-timeout tests, the existing candidate algorithm,
and the combined controlled lifecycle regressions. These results do not use a
real saved production map, verified camera calibration, simulation, or a field robot.

## Strategy packages

| Package | Role | Runtime status |
| --- | --- | --- |
| `goals/` | Fast/slow goal grounding and target resolution | Used by semantic planning. |
| `semantic_navigation/` | Intent/query/observation/execution/verification helpers | Used by semantic navigation; not a separate top-level Module. |
| `tasks/` | TaskPlan, AgentLoop, and action shaping | Used, but TaskPlan and AgentLoop remain parallel. |
| `frontiers/` | Frontier extraction/scoring and VL-Nav helpers | Algorithms and experiments; the product search path uses native semantic view queries rather than allowing this Python projection to authorize motion. |
| `vision/` | Person tracking, VLM helpers, bbox and appearance logic | Used by visual tasks and replay tools. |
| `llm/` | Model clients and prompt builders | Shared boundary and helper clients; Product wiring depends on RunPlan. |

`FrontierScorer`, `SGNavReasoner`, and `verify_and_reselect` are not proof of a
live exploration planner merely because unit tests pass. The native semantic
view bridge is a LingTu product adaptation, not a complete VL-Nav reproduction. See
the [VL-Nav notes](../../docs/research/vlnav.md) and current [research index](../../docs/research/tasks.md).

## Memory boundary

The system has scene/object memory, vector or episodic lookup, and per-run
Agent message context. Agent runs are process-local and are not a durable task
memory record. Search history, excluded candidates, failure reasons, and
resumable plan steps are not yet unified into persistent task memory that
automatically feeds replanning.

Keep scene memory separate from task memory:

- Scene memory answers what was observed and where.
- Task memory answers what this task tried, completed, rejected, or should try next.

Current memory boundaries are in [memory README](../memory/README.md). Research
and code-audit ordering is tracked in [research tasks](../../docs/research/tasks.md);
autonomy design notes are in [autonomy design](../../docs/design/autonomy.md).

## Read the code in this order

1. [Semantic planner](modules/semantic_planner.py) -- instruction, grounding, verification, and native feedback.
2. [Agent Module](modules/agent_planner.py) and [AgentLoop](tasks/agent.py) -- the separate model/tool loop.
3. [Decomposition](tasks/decomposition.py) -- TaskPlan and symbolic subgoals. [ActionExecutor](tasks/actions.py) shapes commands; it does not execute hardware motion.
4. [Visual servo](modules/visual_servo.py) -- Follow and the thinner Find trigger.
5. [semantic task stack](../lingtu/assembly/host/stacks/semantic_tasks.py) and [semantic wires](../lingtu/assembly/host/wiring/semantic.py) -- what is loaded and connected.
6. [MCP server](../gateway/mcp_server.py) and [navigation routes](../gateway/navigation/routes.py) -- external skills and HTTP boundaries.
7. [goals](goals/), [semantic_navigation](semantic_navigation/), [frontiers](frontiers/), and [vision](vision/) -- algorithms used by runtime or documented experiments.

## Product boundary

Decision may depend on `runtime`, `memory`, and local `decision.*` strategies.
It must not import `drivers`, `nav`, `gateway`, or `localization`
implementations. Cross-layer communication uses typed Module ports and Host
wires. Native C++ navigation owns the final geometric and motion decision.

Functional Chinese strings remain in tokenizer data, bilingual labels, command
rules, prompts, and tests. New comments and docstrings should be English.
