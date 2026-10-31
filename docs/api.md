# API

**Status:** Current generated interface reference
**Audience:** SDK, Web, MCP, Gateway, and integration developers
**Runs on:** One LingTu Host in `env=real` or `env=sim`

This file combines the maintained integration contract with inventories
generated from source. Do not hand-edit the method or route tables; run
`python tools/docs/extract_api_docs.py`.

## Integration boundaries

| Surface | Endpoint | Owner |
| --- | --- | --- |
| REST and SSE | `http://<host>:5050` | Gateway routes and projections |
| MCP | `http://<host>:8090/mcp` | `@skill` methods discovered in the Host |
| Python SDK/CLI | `lingtu-sdk` and `lingtu.sdk` | Typed client facade |
| Camera Web media | Scene and Console: WHEP, with JPEG WebSocket fallback | Native media endpoint or optional go2rtc; Gateway signalling |

`LingTuClient` and `AsyncLingTuClient` group operations into domain clients:
`navigation`, `map`, `tasks`, `observations`, and `system`. The historical
`localization` entry remains available for compatibility. Domain clients share
one Gateway address, credentials and request timeout. Await asynchronous
methods. Existing flat methods remain supported delegates. See the
[Python SDK guide](../src/lingtu/README.md#remote-python-sdk) for classes
and examples. Navigation cancellation and software emergency stop retain
their separate `navigation.cancel()` and `system.emergency_stop()` meanings.

Gateway submits typed intent and projects runtime facts. It does not
own Product lifecycle, maps, planning, final motion, or hardware.

Camera signalling uses `GET /api/v1/webrtc/status`, `POST /api/v1/webrtc/whep`
and DELETE on the returned session `Location`. Status includes `backend`,
`ice_servers` and `rotation_degrees` from RobotConfig. Set `LINGTU_WEBRTC_URL`
to the full native WHEP URL (for example `http://127.0.0.1:9090/whep/endpoint`);
otherwise Gateway uses `LINGTU_GO2RTC_URL` (default `http://127.0.0.1:1984`).
`LINGTU_WEBRTC_ICE_SERVERS` is a JSON list in the browser RTCIceServer format.
It configures browser traversal; the selected media server also needs its own
reachable ICE candidates and TURN configuration. A signalling HTTP URL alone
does not make the video transport reachable. Gateway does not start the media
process, and never forwards encoded video. See [video implementation](research/video.md).

Product lifecycle uses `lingtu` / `python -m lingtu.control`, not REST
service orchestration assembled by a client.
Web mode switches use the separate loopback ProductControl transport:
`GET /api/v1/product-control` returns current session/operation;
`POST /api/v1/product-control/switch` accepts `request_id`, `product`
(`map` or `nav`), `map_name` (required for nav), and
`expected_product_session_id` (empty only when no Product is running).
`GET /api/v1/product-control/operations/{request_id}` retrieves the receipt.
Gateway only forwards these requests. A 202 response is admission, not
completion. Query until `succeeded`, `failed`, or `interrupted`; reconnects
must query the original ID, not create another switch. A successful switch
does not authorize robot motion. See [Operations](./operations.md#web-mapping-and-navigation-switches).

External map integrations operate on the canonical map identity
`map_id + content_epoch` and maintained artifact contracts. They must
not invent version directories or bypass ProductControl activation.

Live-mapping saves are durable operations. A successful admission is not a completed
save: poll `GET /api/v1/maps/operations/{operation_id}` and inspect
`operation.state`. A client wait timeout or a lost status response leaves the
outcome unconfirmed; continue querying that operation instead of submitting
another save. A `SUCCEEDED` detail includes the available
`operation.processing.optimization` and `operation.processing.cleanup`
summaries (`performed`, `success`, `reason_code`). Optimization can successfully
skip when measurement constraints are insufficient, so save success does not
imply completed optimization or navigation readiness. Lightweight operation
lists omit these summaries; fetch the individual detail. A same-name save may
replace an inactive map; active-map changes are rejected. Saves do not activate
a map or switch Product.

`GET /map/viewer?map=MAP_ID` returns a 307 redirect to `/?tab=map&map=MAP_ID`;
without `map` it opens the product map library. The redirect does not activate
a map or invoke ProductControl. The previous independent Gateway HTML viewer
is removed.

The library previews any saved map through `GET /api/v1/maps/{name}/pcd`,
which streams the original native artifact and returns `Cache-Control: no-store`.
`X-Lingtu-Map-Id`, `X-Lingtu-Content-Epoch`, and `X-Lingtu-Frame-Id` come from
the native artifact metadata. The viewer rejects missing or mismatched identity
and reloads after content-version, Product-session, or explicit refresh changes.
Preview neither changes the saved data nor activates the selected map.
The separate `/points` endpoint remains bound to the active live scene.

`GET /api/v1/maps/{name}/octomap` streams the saved full-probability
`octomap.ot` with the same artifact identity headers. Its occupied-voxel
display does not classify undrawn space as free or unknown.

## Saved point-cloud subset editing

`POST /api/v1/maps/{name}/save_as` forwards to native mapd's
`save_edited_source`. This synchronous source-edit operation is separate
from the live-mapping save-operation API above. Its JSON body contains:

| Field | Contract |
| --- | --- |
| `name` | New target map name, different from the source. |
| `expected_source_content_epoch` | Positive integer from the source artifact. |
| `operations` | Ordered list of subset edits, or empty to copy the source. |
| `operations[].operation` | `delete_inside` or `keep_inside`. |
| `operations[].bounds` | Finite map-frame `min_x`, `min_y`, `min_z`, `max_x`, `max_y`, `max_z`; each minimum must not exceed its maximum. Units: metres. |

All box faces are inclusive. Native mapd applies each operation to the
remaining complete source XYZ cloud; Web display sampling does not limit
the saved edit. It preserves retained coordinates without downsampling,
rejects an empty result, and leaves the source unchanged. A changed source
returns `source_epoch_mismatch`; reload before editing again. An existing
unrelated target returns `target_exists`.

Success returns the target `map_id`, new `content_epoch`, `source_map_id`,
`source_content_epoch`, `frame_id`, `point_count`, `removed_points`,
`saved_rays_preserved`, `build_options`, and `activation_ready=false`.
A complete measured-ray bundle (poses, patches, manifest, scan origin) is
retained unchanged; incomplete evidence and old derived artifacts are not
copied. Source save success does not mean the navigation artifacts exist.

If the save response is lost, keep the target name, source identity/epoch,
and exact ordered operations. Resubmit that identical request to confirm
the result. mapd returns the existing result with `replayed=true` only
while the target remains invalidated with reason `edited_source` and its
stored edit matches. It does not re-edit or overwrite that target. This
confirmation is checked before rereading the source, so later source
changes do not prevent confirmation of the saved snapshot. After rebuild
replaces the invalidated metadata, this request no longer confirms it;
inspect the target through the existing map queries instead.

Then call `POST /api/v1/maps/{target}/build_octomap` with the returned
`build_options` as its JSON body. Accepted option keys are `resolution`,
`frame_id`, `source_profile`, `data_source`, `slam_source`,
`localization_source`, and `mapping_source`. With no body, the builder
inherits the saved target metadata. A failed build leaves the source
available; retry the build rather than saving another copy. A successful
build commits a new content epoch, so reload the target artifact identity.
Neither operation activates the map; activation remains ProductControl's
responsibility. Deleting points does not establish observed free space,
ground support, route feasibility, or permission to move.

For a new navigation resolution, copy the source with `operations: []`
and build the target with the returned options and requested `resolution`.
The Web offers 0.05, 0.06, 0.10 and 0.20 m. The build uses the copied
source and available measured rays to rebuild the navigation OctoMap.
The response includes `resolution` and `frame_id`; metadata retains the
source map identity. Activate the target through ProductControl to use it.
The separate live local collision layer retains its sensor-profile resolution.

## Natural-language Agent task lifecycle

`POST /api/v1/instruction` and MCP `send_instruction` prefer the loaded
`AgentPlannerModule`. Admission returns an Agent `run_id`, `state` and
`executor=agent`; `execution_confirmed=false` and `motor_confirmed=false`
make explicit that admission is not task or motion success. A Host without
Agent retains the existing semantic instruction-stream fallback.

The instruction body accepts `text`, optional `run_id`, `control_only`
(default `false`), `request_id`, and `client_id`. MCP `send_instruction`
accepts the same text, optional run identity, and `control_only` constraint.
The Agent owner recognizes complete control utterances before starting a new
model task. Responses identify `operation=start/query/keep/cancel/pause/resume`.
Queries and keep requests return the original instruction and a `task_status`
snapshot, including `found`, without replacing the current run. An idle query
returns `state=idle` and no run; it is not task admission.

Use `control_only=true` when the caller may query or end an existing task but
must not start a new one. The Web client uses it while its motion status is
missing/stale or it still holds an active task. A non-control instruction then
returns `accepted=false`, `reason=motion_start_blocked`, and does not start or
publish a task. This request constraint grants no motion permission; ordinary
starts retain the server's existing motion checks. Unsupported pause/resume
returns `unsupported_control`. A new task while the current task is active is
rejected with `task_active`; cancellation/failure without stop confirmation
returns `stop_unconfirmed`. End the old task, wait for confirmation, and submit
the new task explicitly. A rejected receipt may reference the current run;
the presence of `run_id` must not override `accepted=false`.
Rejected command receipts use HTTP 409; Web clients read `reason` from the
error body instead of replacing it with a generic transport error.

Read the current Host run with `GET /api/v1/agent/runs/{run_id}`. The
endpoint intentionally retains only the current run: a different or expired
identity returns `found=false` and `reason=not_current`. Cancel with
`POST /api/v1/agent/runs/{run_id}/cancel`, retaining the original ID.
Cancellation is scoped to that run, so retrying an old cancellation cannot
cancel a newer Agent task. Admission of cancellation still does not confirm
that native child motion has stopped. Current-run status retains the owned
domain reference and reports `stop_confirmed` separately from Agent state.
After cancellation, keep reading while `stop_confirmed=false`; a confirmed
native/domain terminal event, or no submitted motion, can confirm task stop.
This does not measure the robot's physical velocity or certify physical rest.
Cancelling an already completed Agent run is rejected as `task_not_running`
and preserves its completed result.
Status also includes the original `instruction`, `failure_reason`, model
`summary` and `navigation` details. The summary is diagnostic model text,
not execution evidence. While waiting for a model before domain submission,
`navigation` is null. Agent chat events carry `run_id`; clients must keep
messages and query responses bound to that identity when tasks change.

The Agent rejects `done(success=true)` for execution-oriented instructions
unless an owned execution tool was attempted and its domain result succeeded.
Explicit pure status queries are informational and do not require motion.
A sustained owned Follow wait is outside the finite Agent reasoning/action
budget. Model calls retain their per-call timeout, while ordinary navigation
and find actions retain the Agent task deadline. Agent reads the terminal
status of the same `follow_task_id`; cancellation remains explicit and scoped.
`TaskPlan` remains a separate typed decomposition model; the Agent run is not
yet a durable, restart-recoverable Mission executor.

## Follow task lifecycle

Follow is an owned execution rather than a stream of anonymous visual-servo
commands. Start it with `POST /api/v1/visual_servo` using `mode=follow` and a
`target` description or explicit `target_id`. A successful receipt returns both
`follow_task_id` and `product_session_id`; retain both values for every later
operation. The same owner methods are available to MCP as `start_follow`,
`get_follow_task`, and `cancel_follow_task`.

Query one execution with
`GET /api/v1/follow/{follow_task_id}?product_session_id=...`. The returned
`state` and `terminal` describe the Follow execution; `details` contains the
current visual and native-navigation projection. A missing task or a task from
another Product run returns `found=false` with `follow_task_not_found` or
`product_session_mismatch`. Clients must not substitute the current Follow ID
when retrying an operation for an older task.

Cancel with `POST /api/v1/follow/{follow_task_id}/cancel`, passing the original
`product_session_id` plus optional `reason`, `request_id`, and `client_id`.
Admission acknowledges a cancellation request; query the task until
`terminal=true` for the outcome backed by its native child tasks. Cancelling an
old or already terminal Follow never cancels navigation owned by a newer one.
The legacy `mode=stop` command remains available for non-Follow visual search.
When Follow is active it requires the same explicit task and Product-session
reference and never falls back to a global visual-servo stop.

The synchronous `LingTuClient` and `AsyncLingTuClient` expose the same
`follow.start`, `follow.task`, and `follow.cancel` operations. Await methods
on the asynchronous client. The start
receipt exposes `follow_task_id` and `product_session_id` directly:

```python
run = robot.tasks.start_follow(target_id="track_7")
# Use an ID from current perception, after the Product and calibration are ready.
if run.accepted:
    view = robot.tasks.get_follow_task(run.follow_task_id, run.product_session_id)
    receipt = robot.tasks.cancel_follow_task(run.follow_task_id, run.product_session_id)
```

Existing flat SDK methods remain supported. `follow_person(target_id)`
delegates to `follow.start`. `stop_following`
requires both original IDs and delegates to scoped cancellation; it no longer
sends an anonymous stop. Cancellation admission still does not prove native
children have stopped; query the returned task until its actual terminal state.

For a real Follow Product with operator video, select `tracking --variant camera`.
The default `tracking/standard` keeps the existing perception/Follow graph and
does not start the preview encoder. Native simulation supports `standard` only.

## Generation

```bash
python tools/docs/extract_api_docs.py
python tools/docs/extract_api_docs.py --check
```

## Navigation status

`GET /api/v1/navigation/status` and SSE `navigation_status` expose one root-level
v3 contract. It answers four independent questions:
Implementation lives in `src/gateway/navigation/`: `routes.py` registers the
HTTP surface, `status.py` evaluates admission and publishes SSE, `projection.py`
projects the public axes, and `tasks.py` handles exact task/request queries.

| Axis | Meaning |
| --- | --- |
| `task` | Current task phase: `IDLE`, planning, execution, recovery, explicit pause, or a terminal result |
| `goal_admission` | Whether a new or replacement goal can be accepted |
| `control` | Whether autonomy, the operator, or nobody owns control |
| `motion` | Permission, observed motion, and stop-confirmation evidence |

The task lifecycle is `IDLE -> PLANNING -> EXECUTING <-> RECOVERING`, with
explicit `PAUSED` and terminal `SUCCESS`, `FAILED`, or `CANCELLED` branches.
E-stop and InputGate holds change control or motion; they do not fake a task
pause. An operator source Claim only reserves that source. Deliberate operator
motion requests a native pause that retains the ordinary navigation destination;
the task becomes PAUSED only after its stop confirmation. Release keeps it
paused. Its TaskResume request releases the takeover hold and continues the same
task; generic motion resume releases only the hold. HTTP pause/resume receipts
acknowledge the request, while native task events confirm execution.
Closing a Claim that never admitted operator input releases only that source's
permission. It does not insert a stop into an autonomous route. Such Hold/Release
receipts have `source_accepted=true` and `final_output_sequence=0`; they do not
confirm a velocity publication or a physical stop. After actual takeover, the
normal zero-output barrier still applies before the source is released.
`QUIET` is only a fresh odometry observation and is not equivalent
to `CONFIRMED`. Missing or stale evidence projects to `UNKNOWN`.

There is no `operator_state` wrapper or second rich navigation status. Full
blockers live at `/api/v1/readiness`, paths at `/api/v1/path`, native evidence at
`/api/v1/navigation/dds_snapshot`, and exact terminal evidence at
`/api/v1/navigation/tasks/{task_id}`.

## Single targets and waypoint routes

Scene exposes two target-entry modes: **单点** and **连续点**. The saved-map
viewer keeps the existing single-target behavior. In **单点**, clicking A, B,
then C before confirmation leaves C selected; it does not enqueue A or B.
Scene automatically requests a candidate/path preview after target selection or
an edit to its planning parameters. Candidate/path previews do not command
motion. Clicking **开始导航** submits one
external goal. A later accepted external goal replaces the current single
goal; while planning is busy, the native pending slot keeps only the latest
accepted goal. `planning_queued` is this single pending slot, not a FIFO.
An active inspection task owns its route and rejects unrelated external goals.

In **连续点**, Scene keeps an ordered route draft in browser memory, bound
to the active `map_id + content_epoch`. Each map click appends one numbered
`x/y/z` point. The operator can move points up or down, delete a point, or
clear the draft. Continuous mode requires at least two points; use **单点** for
one destination. A map identity change clears the draft. The numbered connector is
an editing aid, not a planned path. Scene does not preview every leg before
start; the native executor plans each leg when that point becomes current.
Scene has no separate save action. The first
start after creating or editing a draft calls the existing route save API once,
then immediately submits that saved revision once through
`POST /api/v1/inspection/tasks`. There is no new queue endpoint and no Web or
Python waypoint executor. The persisted route remains in the route list, and an
unchanged draft may reuse that route revision after the task terminates. The
running route snapshot is not editable. The single-target speed setting does not
configure this route; execution uses the active runtime path-following limit.

After an ordinary Scene remount or full-page reload, re-entering **连续点** queries
the active map's task projection and restores controls only when exactly one nonterminal task
exists. It does not guess among multiple tasks. A disconnect preserves an already
known `task_id`, disables its controls, and resumes polling it after reconnect.
Gateway or native endpoint restart recovery remains bounded by the task projection:
clients neither synthesize task state nor restart a route automatically.

Routes may also be edited and saved in the inspection workbench. The native
inspection executor advances points after arrival and configured point
actions. One `task_id` identifies the whole task; `request_id` identifies
each start/pause/resume/cancel command. Clients display status and submit
commands; they do not advance points. Pause/resume preserves the current
point, and cancellation ends the task through the existing stop handshake.
A successful start/control response acknowledges submission, not execution
or completion. Query the identified task for the native result.

Routes support finite `loop_count` rounds and `loop_interval_s` between
rounds. Native `WAITING_NEXT_LOOP` emits no goal during the interval.
After an endpoint restart, unfinished work returns paused and requires
explicit resume. The saved map and route revision must still match.
An optional final point without an action is the standby destination;
reaching it does not prove charging. Pause the task for manual charging
and resume when ready. There is no automatic charging interface yet.

Evidence completeness and equipment condition are separate report fields.
`acceptance` describes capture/history integrity; `inspection_result`
describes normal, anomaly, inconclusive, pending review or not inspected.
Ordinary photo capture never proves equipment normal. Visual actions
`capture:indicator_on/off/green/red/yellow` and `capture:door_open/closed`
compare a visible observation against the selected expectation. The action
uses one exact suffix, for example `capture:indicator_green`.
The existing `vision_service_url` selects the shared image-capable service;
an absent service or ambiguous image leaves the finding inconclusive and
does not discard the photo. Configure a viewpoint with one identifiable
target and validate recognition on recorded equipment images.
`POST /api/v1/inspection/evidence/{evidence_id}/review` records a human
`normal`, `anomaly` or `inconclusive` finding with an optional note. It
retains the original capture/model result and never changes task motion.

The Python SDK's `TaskClient` groups task operations under `robot.tasks`.
Inspection uses `inspection_routes`, `start_inspection`, `inspection_task`,
`pause_inspection`, `resume_inspection`, `cancel_inspection`, and
`inspection_report`; AsyncLingTuClient delegates
the same operations. The CLI exposes the corresponding `inspection-*`
commands. The former `batch_go` Python waypoint loop has been removed.
Use a stored native route instead; `navigation.go` and
`navigation.wait_until_arrived` remain
available for single-goal navigation.

```python
from lingtu.sdk import LingTuClient

robot = LingTuClient("robot-host")
routes = robot.tasks.inspection_routes()
# Select an existing route from routes before starting it.
result = robot.tasks.start_inspection("saved-route-id")
if result.ok and result.task_id:
    status = robot.tasks.inspection_task(result.task_id)
```

## MCP tools

This inventory is generated from `@skill` decorators. The Host exposes
the selected Modules' methods through MCP JSON-RPC on port 8090.
Agent discovery uses the same descriptors, then applies its own tool filtering
and task-owned motion handlers; the static inventory is not an availability guarantee.
Parameterized declarations add parameter descriptions, optional object output schemas,
and `readOnlyHint`. Input types, required arguments and defaults come from Python signatures.
MCP object results include `structuredContent` and JSON text. Agent function tools receive
output/behavior descriptions and JSON-encoded object results. These hints do not grant
motion permission or turn an admission receipt into a completed task.
See [the runtime contract](../src/runtime/README.md#skill-contract) and the
[MCP tool specification](https://modelcontextprotocol.io/specification/2025-11-25/server/tools).

## src/decision/modules/agent_planner.py
_AgentPlannerModule - multi-turn agent loop for complex instructions._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `run_agent_task` | `AgentPlannerModule` | `instruction: str` | `str` | Admit one task only after the previous task has relinquished motion. |
| `cancel_agent_task` | `AgentPlannerModule` | `run_id: str` | `str` | Cancel only the specified Agent task; the result does not prove physical stopping. |
| `get_agent_status` | `AgentPlannerModule` | — | `str` | Report loop state separately from verified motion and its native task. |

## src/decision/modules/semantic_planner.py
_SemanticPlannerModule - unified semantic planning in one Module._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `get_planner_status` | `SemanticPlannerModule` | — | `str` | Return current semantic planner state and counters. |
| `get_scene_objects` | `SemanticPlannerModule` | — | `str` | Read objects from the latest scene graph as a JSON array of label, confidence and position. This is cached perception, not proof of a freshly confirmed target or an executable goal. |
| `decompose_task` | `SemanticPlannerModule` | `instruction: str` | `str` | Parse an instruction into a symbolic task plan without grounding or motion. status=parsed returns task_plan.v1; status=needs_llm returns a prompt, not a completed model request. A parsed plan still needs scene grounding with resolve_task_step and explicit execution. |
| `resolve_task_step` | `SemanticPlannerModule` | `task_plan: str` | `str` | Ground the active symbolic task step against the current scene and map identity. Pass JSON from decompose_task or a validated task_plan.v1. This returns a candidate only; it neither checks an executable route nor publishes motion. Waiting, invalid or needs_llm results are not successful execution. |

## src/decision/modules/visual_servo.py
_Visual servo Module._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `start_follow` | `VisualServoModule` | `description: str`, `target_id: str`, `request_id: str`, `client_id: str` | `dict` | Start following one visually described person or an existing tracker identity. Provide description or target_id. accepted=true is only admission, not successful following. Unbound references such as me or him require a visual description or a selected target_id. Retain follow_task_id and product_session_id from the receipt and query get_follow_task; use cancel_follow_task with the same pair to stop this task. |
| `get_follow_task` | `VisualServoModule` | `follow_task_id: str`, `product_session_id: str` | `dict` | Query one Follow execution using the exact follow_task_id and product_session_id from start_follow. When found=true, inspect phase for tracking progress and terminal/result/reason for completion. found=false is unknown, not success. Only the last 64 Host executions are retained; no restart recovery. |
| `cancel_follow_task` | `VisualServoModule` | `follow_task_id: str`, `product_session_id: str`, `reason: str`, `request_id: str`, `client_id: str` | `dict` | Request cancellation of exactly the referenced Follow execution. accepted=true acknowledges the request; it does not prove motion has stopped. Query get_follow_task with the same IDs until terminal=true, after owned native children terminate. |
| `find_object` | `VisualServoModule` | `target: str` | `str` | Trigger visual find mode for a target object. |
| `follow_person` | `VisualServoModule` | `description: str`, `request_id: str`, `client_id: str` | `dict` | Trigger person following mode. |
| `stop_servo` | `VisualServoModule` | `follow_task_id: str`, `product_session_id: str` | `dict` | Cancel Follow by reference, or stop the current object approach. |
| `get_servo_status` | `VisualServoModule` | — | `dict` | Return current servo state. |

## src/decision/modules/vla.py
_VLA navigation module._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `vla_navigate` | `VLAModule` | `instruction: str` | `dict` | Send a natural-language navigation instruction to the VLA module. |
| `get_vla_status` | `VLAModule` | — | `dict` | Return current VLA health and metrics. |

## src/explore/tare/module.py
_TAREExplorerModule - connects TARE exploration to LingTu navigation._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `start_tare_exploration` | `TAREExplorerModule` | — | `str` | Send the start signal to the TARE planner. Use this to resume |
| `stop_tare_exploration` | `TAREExplorerModule` | — | `str` | Pause TARE exploration. The planner keeps running but stops |
| `get_tare_status` | `TAREExplorerModule` | — | `str` | Return TARE exploration state: waypoint count, last waypoint age, |

## src/explore/tare/supervisor.py
_ExplorationSupervisorModule — cross-process watchdog for TARE exploration._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `get_exploration_supervisor` | `ExplorationSupervisorModule` | — | `str` | Return the latest exploration supervisor state as JSON. |
| `clear_exploration_fallback` | `ExplorationSupervisorModule` | — | `str` | Reset the fallback_requested flag (use after operator handled). |

## src/gateway/mcp_server.py
_LingTu MCP Server -Model Context Protocol for AI agent control._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `get_health` | `MCPServerModule` | — | `str` | Return full system health: modules, connections, message counts. |
| `list_modules` | `MCPServerModule` | — | `str` | List all deployed modules with layer, port counts, and running state. |
| `get_config` | `MCPServerModule` | — | `str` | Return robot configuration: speed limits, geometry, safety thresholds. |
| `get_robot_position` | `MCPServerModule` | — | `str` | Return the robot's current position (x, y, z) and yaw in map frame. |
| `get_scene_graph` | `MCPServerModule` | — | `str` | Return the current flat or hierarchical scene graph snapshot. |
| `detect_objects` | `MCPServerModule` | `query: str` | `str` | Search the current scene for objects matching *query* (case-insensitive). |
| `query_memory` | `MCPServerModule` | `query: str` | `str` | Search episodic, spatial, and vector memory for past observations. |
| `list_tagged_locations` | `MCPServerModule` | — | `str` | List all named locations tagged by the robot or user. |
| `tag_location` | `MCPServerModule` | `name: str` | `str` | Save the robot's current position under *name* for future navigation. |
| `navigate_to_object` | `MCPServerModule` | `instruction: str` | `str` | Navigate to a described object or place using semantic understanding. |
| `send_instruction` | `MCPServerModule` | `text: str`, `run_id: str`, `control_only: bool` | `str` | Submit a natural-language task to the Agent when it is loaded. Retain run_id and query get_agent_status; cancel_agent_task(run_id) cancels that run only. Admission is not execution success. A Host without Agent uses its semantic instruction stream. |
| `emergency_stop` | `MCPServerModule` | — | `str` | Emergency stop -immediately halts all robot motion. |
| `set_mode` | `MCPServerModule` | `mode: str` | `str` | Set robot operating mode: manual \| autonomous \| estop. |

## src/memory/modules/episodic.py
_EpisodicMemoryModule — 时空情节记忆模块 (Module 模式封装)。_

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `get_recent_observations` | `EpisodicMemoryModule` | `count: int` | `str` | Return the most recent episodic observations (labels + position). |

## src/memory/modules/locations.py
_TaggedLocationsModule — 标签地点记忆模块 (Module 模式封装)。_

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `list_tags` | `TaggedLocationsModule` | — | `str` | List all saved location tags with positions. |
| `go_to_tag` | `TaggedLocationsModule` | `name: str` | `str` | Navigate to a tagged location by name (publishes saved_location if found). |

## src/memory/modules/semantic.py
_SemanticMapperModule -drives RoomObjectKG + TopologySemGraph from live SceneGraph._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `get_room_summary` | `SemanticMapperModule` | — | `str` | Return a text summary of all known rooms and their objects. |
| `query_room_for_object` | `SemanticMapperModule` | `label: str` | `dict` | Return which room types are most likely to contain the given object label. |
| `get_exploration_target` | `SemanticMapperModule` | `instruction: str` | `dict` | Return the best exploration target for a given instruction. |
| `get_semantic_status` | `SemanticMapperModule` | — | `dict` | Return KG + TSG statistics. |

## src/memory/modules/temporal.py
_TemporalMemoryModule — Time-indexed scene memory for temporal queries._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `query_temporal` | `TemporalMemoryModule` | `question: str` | `str` | Answer a natural language question about temporal scene memory. |
| `get_entity_history` | `TemporalMemoryModule` | `label: str` | `str` | Return a formatted summary of all sightings of a specific object label. |

## src/memory/modules/vector.py
_VectorMemoryModule — CLIP embedding + ChromaDB vector search for fuzzy spatial queries._

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `query_location` | `VectorMemoryModule` | `text: str` | `str` | Fuzzy search for a location by natural language description. |
| `get_memory_stats` | `VectorMemoryModule` | — | `str` | Return vector memory statistics. |

## src/nav/skills/skills_module.py

| Tool | Module | Parameters | Returns | Description |
| --- | --- | --- | --- | --- |
| `navigate_to` | `NavSkills` | `x: float`, `y: float`, `yaw: float`, `z: float \| None` | `str` | Submit a navigation goal in the configured planning map frame. This can move the robot. The receipt is admission only: retain request_id and query get_navigation_result for native feedback. A missing or pending result does not establish success. Cancel using the task_id in native status. |
| `stop_navigation` | `NavSkills` | `task_id: str` | `str` | Request a stop of the named navigation task without hardware E-stop. Use task_id from native navigation status, not a request_id. The receipt acknowledges submission; check native status for termination. |
| `cancel_mission` | `NavSkills` | `task_id: str`, `reason: str` | `str` | Request cancellation of a specific native navigation mission. Use task_id from native status, not the submission request_id. An accepted receipt does not establish termination; query native task status afterward. |
| `get_navigation_status` | `NavSkills` | — | `str` | Read canonical native navigation mission status. UNKNOWN means feedback is absent or stale, not that the robot is idle or the task succeeded. Use get_navigation_result(request_id) to associate feedback with this adapter's specific submission. |
| `get_navigation_result` | `NavSkills` | `request_id: str` | `str` | Read native lifecycle feedback for a request submitted by this navigation adapter. Pass request_id from the submission receipt. If found=false, do not infer completion. If found=true, inspect status for the actual native task and outcome. |
| `start_inspection` | `NavSkills` | `route_id: str`, `revision: int` | `str` | Start a stored native inspection route. |
| `navigate_to_deg` | `NavSkills` | `x: float`, `y: float`, `yaw_deg: float`, `z: float \| None` | `str` | Submit a map-frame navigation goal with heading in degrees. |
| `is_navigating` | `NavSkills` | — | `str` | Return whether a navigation mission is currently active. |
| `get_navigation_progress` | `NavSkills` | — | `str` | Return a concise progress summary derived from mission status. |


## Gateway REST API

The Gateway serves these generated route registrations on port 5050.
FastAPI's live OpenAPI UI remains available at `/docs`.

Each source file has one complete table; the rows are the route contract.

### src/gateway/maps/locations.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/locations` | List tagged navigation locations | `LocationsResponse` | `get_locations` |
| `POST /api/v1/locations` | Create or update a tagged navigation location | `LocationOperationResponse` | `post_location` |
| `DELETE /api/v1/locations/{name}` | Delete a tagged navigation location | `LocationOperationResponse` | `delete_location` |
| `PUT /api/v1/locations/{name}` | Update a tagged navigation location | `LocationOperationResponse` | `put_location` |

### src/gateway/maps/places.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/places` | List canonical semantic places | `PlaceListResponse` | `get_places` |
| `POST /api/v1/places` | Create or update a canonical semantic place on a native map | `PlaceUpsertResponse` | `post_place` |
| `GET /api/v1/places/resolve` | Resolve a canonical semantic place by name or alias | `PlaceResolveResponse` | `resolve_place` |

### src/gateway/maps/routes.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/map/global/points` | Native online whole-map preview | `—` | `get_global_map_points` |
| `GET /api/v1/map/points` | Map point cloud as JSON (from ikd-tree snapshot) | `MapPointsResponse` | `get_map_points` |
| `POST /api/v1/map/rename` | Rename a saved map | `MapLifecycleResponse` | `rename_map` |
| `POST /api/v1/map/save` | Save current SLAM map | `MapSaveOperationResponse` | `save_map_now` |
| `POST /api/v1/map_cloud/reset` | Clear accumulated map cloud (viz only, SLAM ikd-tree untouched) | `MapLifecycleResponse` | `reset_map_cloud` |
| `GET /api/v1/maps/environment/layers` | Operator-facing environment map layer state | `—` | `get_environment_map_layers` |
| `POST /api/v1/maps/import_pcd` | Import a PCD file into a LingTu map package | `MapLifecycleResponse` | `import_pcd_map` |
| `GET /api/v1/maps/operations` | List durable map-save operations | `MapSaveOperationResponse` | `list_map_operations` |
| `GET /api/v1/maps/operations/{operation_id}` | Get a durable map-save operation | `MapSaveOperationResponse` | `get_map_operation` |
| `POST /api/v1/maps/operations/{operation_id}/cancel` | Cancel a durable map-save operation | `MapSaveOperationResponse` | `cancel_map_operation` |
| `POST /api/v1/maps/operations/{operation_id}/retry` | Retry a failed durable map-save operation | `MapSaveOperationResponse` | `retry_map_operation` |
| `DELETE /api/v1/maps/{name}` | Delete a saved map | `MapLifecycleResponse` | `delete_saved_map` |
| `POST /api/v1/maps/{name}/build_occupancy` | Build a 2D occupancy artifact from a saved map | `MapLifecycleResponse` | `build_saved_map_occupancy` |
| `POST /api/v1/maps/{name}/build_octomap` | Build OctoPlanner3D octomap.ot from saved map.pcd | `MapLifecycleResponse` | `build_saved_map_octomap` |
| `POST /api/v1/maps/{name}/crop` | Crop a saved map point cloud and invalidate derived artifacts | `MapLifecycleResponse` | `crop_saved_map` |
| `POST /api/v1/maps/{name}/mark_zone` | Mark occupied/free/preblocked/traversable zones in the saved OctoMap | `MapLifecycleResponse` | `mark_saved_map_zone` |
| `GET /api/v1/maps/{name}/octomap` | Serve the native navigation OctoMap for inline preview | `—` | `get_map_octomap` |
| `GET /api/v1/maps/{name}/pcd` | Serve raw PCD file for inline preview | `—` | `get_map_pcd` |
| `GET /api/v1/maps/{name}/points` | Saved map point cloud as JSON | `MapPointsResponse` | `get_saved_map_points` |
| `POST /api/v1/maps/{name}/save_as` | Copy a saved source, optionally applying point-cloud subset edits | `—` | `save_edited_map_as` |
| `POST /api/v1/maps/{name}/validate_plan` | Preview a route on the active saved map without publishing a goal | `—` | `validate_saved_map_plan` |
| `POST /api/v1/maps/{name}/voxels/edit` | Edit saved OctoMap voxels for OctoPlanner3D | `MapLifecycleResponse` | `edit_saved_map_voxels` |
| `GET /api/v1/maps/{name}/voxels/edits` | Saved OctoMap voxel edit overlay | `—` | `get_saved_map_voxel_edits` |
| `GET /api/v1/slam/maps` | List maps through native mapd | `MapListResponse` | `slam_maps` |
| `GET /map/viewer` | Open the product map page | `—` | `map_viewer` |

### src/gateway/maps/stairs.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/maps/{name}/stairs` | — | `StairListResponse` | `list_stairs` |
| `DELETE /api/v1/maps/{name}/stairs/{stair_id}` | — | `StairListResponse` | `delete_stair` |
| `PUT /api/v1/maps/{name}/stairs/{stair_id}` | — | `StairListResponse` | `put_stair` |

### src/gateway/mcp_server.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /capabilities` | — | `—` | `capabilities` |
| `GET /health` | — | `—` | `health` |
| `POST /mcp` | — | `—` | `mcp_endpoint` |

### src/gateway/navigation/diagnostics.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/navigation/dds_snapshot` | Latest navigation data for the native DDS endpoint | `NavigationDdsSnapshotResponse` | `get_navigation_dds_snapshot` |
| `GET /api/v1/path` | Latest planned path | `PathResponse` | `get_path` |

### src/gateway/navigation/routes.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `POST /api/v1/goal` | Send navigation goal | `ControlCommandResponse` | `post_goal` |
| `POST /api/v1/navigate/click` | Navigate to map-viewer click point | `ControlCommandResponse` | `post_navigate_click` |
| `POST /api/v1/navigation/cancel` | Compatibility cancel for the current navigation mission | `ControlCommandResponse` | `post_navigation_cancel` |
| `POST /api/v1/navigation/goal_candidate` | Construct and optionally preview a navigation goal without publishing it | `GoalCandidateResponse` | `post_navigation_goal_candidate` |
| `GET /api/v1/navigation/goals/{request_id}` | Request-correlated native navigation lifecycle status | `NavigationGoalStatusQueryResponse` | `get_navigation_goal_status` |
| `POST /api/v1/navigation/plan` | Preview navigation plan without publishing a goal | `PlanPreviewResponse` | `post_navigation_plan` |
| `POST /api/v1/navigation/resume` | Release a native motion hold without replaying previous motion | `ControlCommandResponse` | `post_navigation_resume` |
| `GET /api/v1/navigation/status` | Public navigation status | `NavigationStatusResponse` | `get_navigation_status` |
| `GET /api/v1/navigation/tasks/{task_id}` | Stable native navigation task lifecycle status | `NavigationTaskStatusQueryResponse` | `get_navigation_task_status` |
| `POST /api/v1/navigation/tasks/{task_id}/cancel` | Request cancellation of one navigation task | `ControlCommandResponse` | `post_navigation_task_cancel` |
| `POST /api/v1/navigation/tasks/{task_id}/pause` | Request a stop-confirmed pause of one navigation task | `ControlCommandResponse` | `post_navigation_task_pause` |
| `POST /api/v1/navigation/tasks/{task_id}/resume` | Request continuation of the same paused navigation task | `ControlCommandResponse` | `post_navigation_task_resume` |

### src/gateway/routes/app.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/app/bootstrap` | App/Web bootstrap snapshot | `AppBootstrapResponse` | `app_bootstrap` |
| `GET /api/v1/app/capabilities` | App/Web API capability manifest | `AppCapabilitiesResponse` | `app_capabilities` |
| `GET /api/v1/app/traffic` | App/Web realtime traffic and client polling policy | `AppTrafficResponse` | `app_traffic` |

### src/gateway/routes/assets.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /robot/meshes/{filename}` | Serve robot STL mesh files | `—` | `serve_robot_mesh` |

### src/gateway/routes/camera.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/camera/snapshot` | Camera JPEG snapshot | `—` | `camera_snapshot` |
| `GET /api/v1/camera/status` | Camera frame availability and rates | `—` | `camera_status` |

### src/gateway/routes/commands.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/agent/runs/{run_id}` | Read the current Agent run by identity | `—` | `get_agent_run` |
| `POST /api/v1/agent/runs/{run_id}/cancel` | Cancel only the referenced Agent run | `ControlCommandResponse` | `cancel_agent_run` |
| `POST /api/v1/estop/reset` | Explicitly release the native software emergency-stop latch | `ControlCommandResponse` | `post_estop_reset` |
| `GET /api/v1/follow/{follow_task_id}` | Read one Follow execution | `—` | `get_follow_task` |
| `POST /api/v1/follow/{follow_task_id}/cancel` | Cancel one Follow execution | `ControlCommandResponse` | `cancel_follow_task` |
| `POST /api/v1/instruction` | Natural language navigation instruction | `ControlCommandResponse` | `post_instruction` |
| `POST /api/v1/lease` | Acquire/release/renew control lease | `LeaseResponse` | `post_lease` |
| `POST /api/v1/mode` | Switch operating mode | `ControlCommandResponse` | `post_mode` |
| `POST /api/v1/stop` | Emergency stop | `ControlCommandResponse` | `post_stop` |
| `POST /api/v1/visual_servo` | Set visual servo target | `ControlCommandResponse` | `post_visual_servo` |

### src/gateway/routes/diagnostics.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/diagnostic_pack` | Export diagnostic tarball | `—` | `diagnostic_pack` |
| `POST /api/v1/diagnostics/field-check` | Run read-only product field readiness check | `ProductFieldCheckResponse` | `product_field_check` |
| `GET /api/v1/diagnostics/plugins` | Read registered plugin categories and providers | `—` | `plugin_catalog` |
| `GET /api/v1/diagnostics/real-runtime-evidence/latest` | Read latest Thunder field runtime evidence gate summary | `RealRuntimeEvidenceLatestResponse` | `real_runtime_evidence_latest` |
| `GET /api/v1/diagnostics/routecheck/latest` | Read latest non-motion routecheck summary | `RoutecheckLatestResponse` | `routecheck_latest` |
| `GET /api/v1/diagnostics/runtime-contract` | Read canonical runtime interface contract | `RuntimeContractResponse` | `runtime_contract` |
| `POST /api/v1/inspection/acceptance` | Run read-only inspection acceptance without publishing motion commands | `InspectionAcceptanceResponse` | `inspection_acceptance` |
| `GET /api/v1/runtime/dataflow` | Read-only Product motion and Gateway observability | `RuntimeDataflowResponse` | `get_runtime_dataflow` |
| `POST /api/v1/runtime/dataflow/subscribe` | Create a read-only Gateway SSE subscription plan | `RuntimeDataflowSubscribeResponse` | `post_runtime_dataflow_subscribe` |
| `GET /api/v1/runtime/dataflow/topic` | Inspect one Gateway-observable Product topic | `RuntimeDataflowTopicDetailResponse` | `get_runtime_dataflow_topic` |

### src/gateway/routes/health.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/health` | System health overview | `HealthResponse` | `get_health` |
| `GET /api/v1/metrics` | Operator-facing runtime metrics snapshot | `—` | `get_metrics` |
| `GET /api/v1/readiness` | Client readiness snapshot | `ReadinessResponse` | `api_readiness` |
| `GET /health` | Liveness probe | `LivenessResponse` | `liveness_health` |
| `GET /ready` | Readiness probe | `ReadinessResponse` | `readiness_ready` |

### src/gateway/routes/inspection.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/inspection/evidence` | List recent verified inspection evidence | `—` | `list_inspection_evidence` |
| `GET /api/v1/inspection/evidence/{evidence_id}` | Read one verified inspection evidence manifest | `—` | `get_inspection_evidence` |
| `GET /api/v1/inspection/evidence/{evidence_id}/artifacts/{kind}` | Read one verified inspection evidence artifact | `—` | `get_inspection_evidence_artifact` |
| `GET /api/v1/inspection/evidence/{evidence_id}/measurements/{sensor_id}` | Download one verified inspection measurement by sensor identity | `—` | `get_inspection_evidence_measurement` |
| `POST /api/v1/inspection/evidence/{evidence_id}/review` | Record a human inspection finding without changing task execution | `—` | `review_inspection_evidence` |
| `GET /api/v1/inspection/routes` | List native inspection routes for a map | `InspectionRouteListResponse` | `list_inspection_routes` |
| `POST /api/v1/inspection/routes` | Create or update a native inspection route | `InspectionRouteResponse` | `put_inspection_route` |
| `DELETE /api/v1/inspection/routes/{route_id}` | Delete one native inspection route | `InspectionCommandResponse` | `delete_inspection_route` |
| `GET /api/v1/inspection/routes/{route_id}` | Read one native inspection route | `InspectionRouteResponse` | `get_inspection_route` |
| `GET /api/v1/inspection/status` | Read native inspection store/status snapshot | `InspectionStatusResponse` | `inspection_status` |
| `GET /api/v1/inspection/tasks` | List retained inspection task projections | `InspectionTaskListResponse` | `list_inspection_tasks` |
| `POST /api/v1/inspection/tasks` | Submit a task-addressed native inspection route | `InspectionTaskCommandResponse` | `start_inspection_task` |
| `GET /api/v1/inspection/tasks/{task_id}` | Read the fact-backed state of one inspection task | `InspectionTaskStatusResponse` | `get_inspection_task` |
| `POST /api/v1/inspection/tasks/{task_id}/cancel` | Request cancellation for one native inspection task | `InspectionTaskCommandResponse` | `cancel_inspection_task` |
| `GET /api/v1/inspection/tasks/{task_id}/guidance` | Read presentation guidance for one inspection task | `InspectionGuidanceResponse` | `get_inspection_task_guidance` |
| `POST /api/v1/inspection/tasks/{task_id}/pause` | Request pause for one native inspection task | `InspectionTaskCommandResponse` | `pause_inspection_task` |
| `GET /api/v1/inspection/tasks/{task_id}/report` | Read the business result of one inspection task | `InspectionTaskReportResponse` | `get_inspection_task_report` |
| `POST /api/v1/inspection/tasks/{task_id}/resume` | Request resume for one native inspection task | `InspectionTaskCommandResponse` | `resume_inspection_task` |

### src/gateway/routes/measurements.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/measurements` | — | `—` | `list_measurements` |
| `GET /api/v1/measurements/{sensor_id}` | — | `—` | `get_measurement` |
| `GET /api/v1/measurements/{sensor_id}/preview` | — | `—` | `get_measurement_preview` |
| `GET /api/v1/measurements/{sensor_id}/raw` | — | `—` | `get_raw_measurement` |

### src/gateway/routes/operations.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `POST /api/v1/explore/directed` | Set an explicit native TARE exploration direction intent | `DirectedExplorationResponse` | `explore_directed` |
| `POST /api/v1/explore/directed/clear` | Clear the explicit native TARE exploration direction intent | `DirectedExplorationResponse` | `clear_explore_directed` |
| `GET /api/v1/explore/runs` | List recent durable native Explore executions | `ExplorationRunListResponse` | `explore_runs_list` |
| `GET /api/v1/explore/runs/{exploration_run_id}` | Query one durable native Explore execution | `ExplorationRunResponse` | `explore_run_get` |
| `POST /api/v1/explore/start` | Start autonomous frontier exploration | `ExplorationCommandResponse` | `explore_start` |
| `GET /api/v1/explore/status` | Exploration status | `ExplorationStatusResponse` | `explore_status` |
| `POST /api/v1/explore/stop` | Stop autonomous frontier exploration | `ExplorationCommandResponse` | `explore_stop` |
| `POST /api/v1/localization/map-tracking` | Start continuous tracking against the running Product's active map | `LocalizationOperationResponse` | `localization_map_tracking` |
| `POST /api/v1/localization/relocalizations` | Relocalize against the running Product's active map | `LocalizationOperationResponse` | `localization_relocalize` |
| `GET /api/v1/memory/temporal` | Query temporal entity observations | `TemporalMemoryResponse` | `get_temporal_memory` |
| `POST /api/v1/memory/temporal/semantic` | Semantic similarity search over temporal observations | `TemporalMemoryResponse` | `post_temporal_semantic` |
| `POST /api/v1/recordings/start` | Start native MCAP recording | `RecordingOperationResponse` | `recording_start` |
| `GET /api/v1/recordings/status` | Native MCAP recording status | `RecordingStatusResponse` | `recording_status` |
| `POST /api/v1/recordings/stop` | Stop native MCAP recording | `RecordingOperationResponse` | `recording_stop` |
| `GET /api/v1/services/status` | Product service status | `ServiceStatusResponse` | `service_status` |
| `GET /api/v1/slam/status` | SLAM service status | `SlamStatusResponse` | `slam_status` |

### src/gateway/routes/product_control.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/product-control` | — | `—` | `control_status` |
| `GET /api/v1/product-control/operations/{request_id}` | — | `—` | `control_operation` |
| `POST /api/v1/product-control/switch` | — | `—` | `control_switch` |

### src/gateway/routes/realtime.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/events` | SSE event stream | `—` | `sse_events` |

### src/gateway/routes/recordings.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/recordings` | — | `—` | `recording_list` |
| `GET /api/v1/recordings/catalog` | — | `—` | `recording_topic_catalog` |
| `GET /api/v1/recordings/streams` | — | `—` | `recording_streams` |
| `DELETE /api/v1/recordings/{session_id}` | — | `—` | `recording_remove` |
| `GET /api/v1/recordings/{session_id}` | — | `—` | `recording_detail` |
| `GET /api/v1/recordings/{session_id}/archive` | — | `—` | `recording_archive` |
| `GET /api/v1/recordings/{session_id}/camera-frame` | — | `—` | `camera_frame` |
| `GET /api/v1/recordings/{session_id}/files/{artifact_path:path}` | — | `—` | `recording_artifact` |
| `GET /api/v1/recordings/{session_id}/scene` | — | `—` | `scene_window` |
| `GET /api/v1/recordings/{session_id}/scene-info` | — | `—` | `scene_info` |
| `GET /api/v1/recordings/{session_id}/series` | — | `—` | `recording_series` |
| `GET /api/v1/recordings/{session_id}/summary` | — | `—` | `recording_summary` |
| `POST /api/v1/recordings/{session_id}/validate` | — | `—` | `recording_validate` |

### src/gateway/routes/safety.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/safety/modes/estop` | Read emergency-stop state | `SafetyEstopResponse` | `get_safety_estop` |
| `POST /api/v1/safety/modes/estop` | Activate emergency stop | `SafetyEstopResponse` | `post_safety_estop` |

### src/gateway/routes/sensor_control.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `POST /api/v1/sensors/{sensor_id}/control` | — | `—` | `control_sensor` |

### src/gateway/routes/session.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/session` | Current Product session state and capabilities | `SessionResponse` | `session_get` |

### src/gateway/routes/status.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/localization/status` | Localization status for app and web clients | `LocalizationStatusResponse` | `get_localization_status` |
| `GET /api/v1/scene_graph` | Current scene graph | `SceneGraphResponse` | `get_scene_graph` |
| `GET /api/v1/state` | Full robot state snapshot | `StateResponse` | `get_state` |

### src/gateway/routes/video.py

| Route | Summary | Response model | Handler |
| --- | --- | --- | --- |
| `GET /api/v1/webrtc/go2rtc/status` | Probe the optional go2rtc endpoint | `Go2RTCStatusResponse` | `go2rtc_status` |
| `DELETE /api/v1/webrtc/sessions/{resource_path:path}` | Release a video receiver | `—` | `delete_session` |
| `GET /api/v1/webrtc/status` | Probe the selected media endpoint and camera display configuration | `WebRTCStatusResponse` | `media_status` |
| `POST /api/v1/webrtc/whep` | Negotiate a video receiver with the media endpoint | `—` | `offer` |
