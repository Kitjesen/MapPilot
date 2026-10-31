# Persistent semantic mapping

**Status:** Design and implementation audit, 2026-10-02  
**Scope:** The shared mapping -> save/reload -> localization -> autonomous
navigation product path, with semantic queries and language task planning as
optional capabilities.

This document separates three things that are easy to mix together:

1. the native geometric map that can authorize motion;
2. a persistent semantic index that explains what the geometry may represent;
3. an LLM-facing representation that can answer a task without becoming a
   second planner or map owner.

The papers in [the local index](papers/README.md) motivate the design. They do
not define LingTu's wire contracts and do not prove field readiness.

## Stair recognition and traversal — requirements, 2026-10-11

**Status (2026-10-12):** Manual authoring, native recognition of a selected straight
flight, persisted observed geometry, shared navigation segment handling, and the
driver's controller-profile handoff/restore have implementations. Native MuJoCo
tests now exercise the actual navigation and ONNX handoff, but complete ascent
and descent have not passed. Contract-test success is not stair qualification.
No real robot has passed stair acceptance through this implementation.
The complete requested feature includes stair recognition, durable map
annotations, navigation through stair segments, and the selected robot's
motion-control profile handoff before entering and after leaving stairs. The operator
selected manual authoring for the first delivery: select a staircase region in
the 3D map, then select its lower and upper entrances. This input method does
not remove automatic recognition or profile switching from the full requirements.
The operator clarified on 2026-10-12 that this means switching the robot's
motion-control policy/model and associated controller parameters, not merely
selecting a gait or changing a navigation preference.
Terminology is recorded in [CONTEXT.md](../../CONTEXT.md).

### Complete behavior required

| Requirement | Observable behavior |
| --- | --- |
| Recognize stairs | Find candidate flights and landings in saved or live 3D observations; estimate entrance/exit poses, vertical extent and ascending direction. Distinguish stairs from ramps, isolated obstacles and different surfaces at the same XY. An image/category label can assist recognition but is not a complete 3D result. |
| Author and correct | Support the selected manual workflow and later review/correction of automatically proposed geometry. Saving a proposal is explicit; a passing live detection must not silently replace durable annotations. |
| Preserve the map relationship | Save the feature against the existing map/frame/content identity, with entrances and a connection. Reload/edit/delete work without changing source point clouds or synthesizing missing ground. |
| Make stairs inspectable | Keep the rising connection and landings visible while inspecting multiple levels. Show entrances, direction and selected flight; display filtering must not change planning geometry. |
| Associate the actual path | Resolve stair requirements for the actual 3D path, in the shared single-goal and inspection navigation flow. Only relevant crossings acquire the requirements; traveling beside or below a staircase does not. |
| Prepare before entry | Approach a usable landing, apply entry speed/orientation constraints, and satisfy the platform's profile-switch prerequisites before crossing onto the flight. If switching requires Grounded, prepare that state on the landing through the supported body transition. Use robot extent and actual position rather than waiting until its center has entered. |
| Complete the profile handoff | The driver requests the concrete motion-control profile bound for this robot, then reads back the actual active profile. Executor permits entry only after it matches the required profile and motion readiness is restored; a sent command, gait number or arbitrary delay is not profile confirmation. |
| Traverse in either direction | Apply direction-appropriate speed/orientation constraints to local tracking while retaining live 3D collision checks. Validate ascent and descent separately; do not assume a reverse-descent policy shared by all robots. |
| Handle landings and exit | Treat turning staircases as connected flights with landings. After clearing the flight, restore the ordinary motion-control profile and confirm its readback, observing the same platform transition prerequisites, unless the next connected flight needs the current profile. Resume the same navigation task. |
| Remain controllable | Preserve pause/cancel/operator-stop behavior during entry and traversal. Temporary observation gaps or recoverable profile-command failures keep a visible waiting/retry state; they do not silently drop the goal or immediately exhaust an unrelated replan budget. |

The current recognizer uses the saved PCD, selected 3D bounds and both entrance
points. It extracts observed horizontal patches and a rising connection, keeping
distinct surfaces at the same XY. Tread/platform extents and heights are consumed
by native path association. It needs neither an LLM nor a new semantic service.
Whole-map proposals and live stair recognition remain separate, unimplemented
work; synthetic recognition tests do not replace recorded-stair validation.

### Profile handoff and execution ownership

The implemented handoff phases, pending end-to-end qualification, are:

```text
follow ordinary path
  -> approach the entrance landing and satisfy entry pose/speed
  -> satisfy profile-switch prerequisites, including Grounded if required
  -> request the robot's configured stair motion-control profile
  -> read back the active profile and confirm motion readiness
  -> track the stair segment with its direction-dependent constraints
  -> clear the exit onto the landing
  -> restore and confirm the ordinary profile when appropriate
  -> continue the same goal or inspection route
```

These are phases within the existing native navigation execution, not a second
task scheduler. Perception supplies observations; mapd stores annotations;
global planning searches geometry; Executor coordinates segment entry/exit;
the local planner tracks under the current constraints; the driver owns
platform commands and feedback. ProductControl is not used to switch profiles or
restart navigation during this sequence.

Map annotations express direction-dependent traversal requirements, not a
vendor SDK integer, model filename or concrete profile name. RobotConfig holds
static platform capability declarations; the existing adapter binds supported
requirements to concrete installed profiles, transition prerequisites and
readback semantics. Active profile selection remains runtime state. Go2 and
Thunder/DOSO must be checked separately against their actual SDK/firmware.
An automatically terrain-adaptive controller may satisfy a documented traversal
capability, but it does not demonstrate this requested model/profile switch.

Confirm the actual active profile through the platform's documented feedback.
If that readback is unavailable, profile switching remains an integration gap;
do not substitute a transport acknowledgment, a gait-state number, or an
elapsed delay. A profile-switch API also does not establish that an installed
profile can climb stairs. Qualify the selected model separately for the target
robot and stair geometry. Capability limitations apply to the affected stair
traversal, not unrelated ordinary navigation.

For a platform that requires Grounded, the entrance landing must support the
stop and body transition needed to reach Grounded, the profile change and
readback, and the return to motion readiness. Standing still does not satisfy
that prerequisite. If these preparation steps cannot be completed there,
Executor must wait before entry; it must not move onto the flight to switch.

A replan, pause, cancellation or temporary loss of stair observations must not
blindly switch back to ordinary walking while the robot is still on the flight.
Retain terrain-appropriate behavior and use the existing stop/hold and operator
control path. Recovery actions must respect the stair segment's maneuvering
constraints rather than reuse unconstrained flat-ground rotations or lateral
translations. Resume only from actual robot progress and active profile; do not
replay an old segment transition after a new path replaces it.

The UI should show useful progress such as approaching stairs, switching motion profile,
ascending/descending, waiting with a reason, and continuing ordinary navigation.
Show the confirmed active profile separately from the requested profile. Detailed SDK
state belongs in diagnostics, not the default task card.

Platform interface audit, updated 2026-10-12 without a field connection:

- Unitree's [SDK2 1.0.0 SportClient](https://github.com/unitreerobotics/unitree_sdk2/blob/1.0.0/include/unitree/robot/go2/sport/sport_client.hpp)
  declares `WalkStair`, `SwitchGait` and `GetState`; its current
  [main interface](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp)
  does not expose those same calls. The [release descriptions](https://github.com/unitreerobotics/unitree_sdk2/releases)
  distinguish non-fused and fused controllers. Bind implementation to the
  installed SDK/firmware rather than copying a gait number. SportModeState
  fields alone do not establish active-model/profile feedback in LingTu.
- The local DOSO Brainstem protocol exposes `SwitchProfile` and `GetProfile`,
  with current/available profile information. Its documented switching
  precondition is `Grounded`; this is not permission to change profiles while
  walking or merely standing still. The C++ client now exposes these calls in
  DOSO SDK 0.6; LingTu's existing driver binds `readProfile`/`switchProfile` and
  uses `GetCmsState` for the transition prerequisite. Entry preparation, actual
  profile readback, standing/motion readiness and restoration are implemented
  in [the driver](../../src/drivers/real/motion/README.md#stair-controller-profiles).
  No installed Thunder stair profile has been qualified. Empty stair-profile
  bindings remain unsupported for stairs without disabling ordinary navigation.

These are integration findings, not evidence that either platform has passed
stairs. Confirm the installed capability and transition behavior before
enabling autonomous entry for that platform.

### First delivery: manual authoring workflow

1. Open a saved map and enter annotation mode. Camera movement, point selection
   for annotations, and navigation goal selection must be distinct interactions.
2. Choose stairs and select a 3D region. A screen rectangle or XY polygon alone
   is insufficient on overlapping floors: show and edit the vertical extent too.
3. Pick lower and upper entrance points on the observed landings and inspect
   their XYZ values. This annotation stores **surface coordinates**, not body
   navigation poses. Native path association adds the configured body support
   height; it never submits the stored surface Z as a body goal Z.
   Surface-pick misses do not fall back to the robot's current plane.
4. Show the region, entrances, and lower-to-upper direction together. Save or
   cancel explicitly; editing or saving an annotation never submits motion.
5. Reload the map and recover the same annotation. The region describes the
   feature extent; it does not classify every point inside the box as stairs,
   invent missing treads, or alter PCD/OctoMap occupancy.

### Minimum information and ownership

- Map association uses the existing map ID, content epoch and map frame.
- A staircase has one map-bound annotation ID and two three-dimensional surface
  entrance points. Bounds, entrances and native recognition results are kept
  together in one native POI record for an atomic save; ordinary POI queries
  exclude this dedicated record type. A future graph projection may reference
  these entrance positions, but must not maintain a second editable copy or
  introduce task/per-attempt IDs for annotations.
- Store a label/type and an editable 3D extent. Derive the nominal ascending
  direction from the selected entrances for a single flight; do not apply that
  straight-line model to an entire turning staircase. Turning stairs can use
  separate flights joined at existing landing places.
- Upward and downward traversal requirements are distinct from the feature's
  geometry. Add typed speed/orientation/locomotion-capability fields only with a concrete
  native consumer; leave unavailable rules unspecified. Do not insert Spot's
  reverse-descent preference as a Go2/Thunder default.
- Tread/platform dimensions are measured from observations for recognition and
  path association, not entered by the operator or inferred from a semantic label.
- Native mapd owns durable annotations. Gateway transports authoring/query
  requests; the browser edits/displays them. RobotConfig and the motion adapter
  own platform capability and calibration rather than copying them into maps.

### Existing implementation and specific gaps

Native POIs persist in `<map>/pois.tsv`; the new stair APIs store one
`lingtu.stair.v1` tagged record there per staircase. `list_stairs`, `set_stair`
and `delete_stair` go through the existing mapd service; PUT/DELETE compare the
displayed content epoch with the native map under its mutation lock. Editing
an annotation does not rewrite source geometry or advance its epoch. Gateway
exposes GET `/api/v1/maps/{name}/stairs`, PUT and DELETE on the corresponding
`/{id}` resource. The editor can draw a screen rectangle, inspect/correct the
resulting 3D bounds, pick surface entrances, and save/reload/edit/delete.
The rectangle includes points along its viewing depth within the selected
height slice; it is not semantic segmentation or nearest-surface selection.
Old-epoch annotations remain inspectable as stale entries and require picking
new geometry before saving against the current map.

On save, mapd recomputes `geometry` from the selected region of the saved PCD;
browser-supplied recognition claims are ignored. Recognized geometry contains
the flight's local axes and ordered observed surface patches, including both
landings. The result is stored in the same POI record and read by nav without
linking the map service lifecycle. An unrecognized annotation can still be saved
and corrected; it does not become a recognized navigation flight. Old records
without geometry require saving again to run recognition. No source point cloud,
OctoMap occupancy or geometric `content_epoch` is changed by this operation.

The recognizer distinguishes the tested ramps, walls, isolated ledges and
disconnected patches from a stair chain. It preserves stacked surfaces and does
not fill missing treads. Its observed geometry is usable by navigation; this
does not prove free space or that a selected robot/controller can climb it.
The independent implementation follows horizontal-surface dimensions and relative
positions, motivated by the original authors' [stair detection and modelling
work](https://github.com/aperezyus/stairs_detection). It does not copy that
project's GPL implementation or add its ROS/PCL dependencies.

The native `MapGraph` still owns general persisted topology, while this feature
uses the single annotation's measured connection. It does not require a duplicate
editable graph edge, a parallel semantic database or a second stair task identity.

The existing `semantic_map.bin` stores voxel labels and confidence, not a
staircase object with entrances and traversal requirements. A saved stairs label
alone remains insufficient; the native ROI recognizer is separate from taxonomy.

Single goals and inspection goals share path activation and Executor. Native
activation reads annotations with the matching map epoch, resolves stair segments
against the actual XYZ path and measured tread heights, and supplies them to
Executor. Merely passing beside or beneath the selected box must not acquire a
stair requirement. Segments reuse the route and task identity. Global geometric
search and local live collision checking retain their existing owners;
inspection's post-arrival point actions do not become a second stair executor.

The existing driver's shared Body interface now exposes profile requests and
actual profile feedback. Its handoff retains the original observed profile,
stops on the landing, performs required body transitions, requests the configured
ascent/descent profile and independently confirms the active name and readiness.
Restoration uses that original name after exit, not a guessed normal-mode number.
Pause/cancel/observation gaps must retain appropriate behavior on the flight.
These are implemented native contracts, not evidence of Go2 support or a qualified
Thunder model; Go2 still lacks a verified actual-model switch/readback binding.

Editing annotations must not advance geometric `content_epoch`. If the saved
geometry is replaced, surface the annotation's old binding for realignment;
do not silently attach its coordinates to the new content or block unrelated
ordinary navigation merely because an optional annotation is stale.

### Current evidence and remaining qualification

- MuJoCo, 2026-10-12: the experimental Thunder session uses the existing
  twelve-step showcase and actual 4998/1119 ONNX artifacts, at 200 Hz physics and
  50 Hz policy control. ProductControl owns the native navigation, DDS bridge
  and physics feeder. Test inputs use truth localization and a 5 cm OctoMap
  built from ideal virtual LiDAR rays; this does not test SLAM or real scans.
  Native recognition reads thirteen observed surfaces, including the landings;
  the noisy-map counterpart still fails recognition and is retained as a
  negative result. No missing ground is synthesized.
- Eleven separate physical locomotion probes include one successful
  fixed-1119 ascent and one successful independent exit restoration to 4998.
  Entry-switch ascent and descent probes encounter non-foot contacts. Native
  navigation trials confirm the real 4998-to-1119 model change in both
  directions, but the final separate 120 s ascent v14 and descent v6 complete
  neither route nor exit restoration. Both also fail LiDAR scheduling and
  continuous-input checks; the scheduling variability is not fully attributed.
  Earlier 90 s descent v5 was still progressing near its deadline, and remains
  separate evidence. Zero-command acknowledgment and process shutdown do not
  prove physical settling. These results must not be combined into an end-to-end
  pass. Evidence and `tested-source-v14` are under
  `build/stairs-mujoco-20261012/`; reproduction is described in
  [Testing](../testing.md#mujoco-stair-regression).
- Simulation exposed and drove fixes for surface-chain merging, mismatched
  path-association tolerances, use of a collision-inflated footprint for landing
  fit, diagonal approach targets, and permanently latched stair progress stops.
  A stair progress stop now discards the local trajectory and retries with the
  same stair/model constraints and live collision map, rather than authorizing
  flat-ground recovery maneuvers. Model readiness must include observed settling
  after loading the actual policy, not merely successful file loading.
- The physical run also exposed a DDS acknowledgment ordering deadlock: a later
  profile request in the same dispatch batch could suppress an already earned
  command READY reply. The bridge now publishes that reply before processing the
  later request; model readiness still requires separate actual-policy feedback.
  During stair traversal, the follower now receives the flight's direction in
  the planning frame, rather than rotating toward a temporary spline tangent.
  Regression tests cover ascent/descent in both map and odom frames.
  Candidate 3D collision checks and following now share the stair body heading,
  including the actual-to-commanded yaw sweep. Rejected routes are discarded
  before retry; final contract subsets pass (33 stair, 32 grid/prediction, 5 local
  task and 5 Executor tests). This has measurable asynchronous search cost and
  does not yet establish a successful physical stair traversal.
- MuJoCo clearance measurement previously skipped a non-colliding visual stair
  edge by advancing the ray origin through the solid tread below it. That could
  report a 1.088 m base clearance instead of the actual 0.368 m. The corrected
  query preserves the original ray and excludes visual/robot geometry, retaining
  separate physical tread heights and native primitive/mesh/heightfield queries.
  This repairs simulation evidence; it does not change navigation geometry or
  relax the physical stability limits.
- A fixed-entry regression reproduced extremely slow initial motion on a long
  low-speed local route: at 0.15 m/s, a 3.48 m single-polynomial seed requested
  only about 8 mm in its first two seconds. Route tracking now reuses the local
  planner's existing bounded acceleration/cruise/deceleration seed. The same
  test requests about 0.256 m in two seconds while retaining the measured 3D
  initial velocity, optimizer, collision checks and final command limit. This
  addresses trajectory startup, not the candidate ONNX policy's stair capability.
  The progress monitor still compares actual progress with requested movement;
  its timeout and thresholds are unchanged.
- Native map tests: **2/2 passed** (`lingtu_maps_stair_geometry_test` and
  `lingtu_maps_mapd_service_dispatch_test`). Coverage includes 5 cm voxel sampling,
  rotated/elevated noisy observations, overlapping floors, ramp/wall/ledge and
  missing-surface negatives, binary-PCD save/reload, ignoring caller geometry,
  and preservation of ordinary POIs and the geometric epoch.
- Driver tests: **7/7 passed**; DOSO SDK tests: **2/2 passed**. These use local
  contracts/mock endpoints, not an installed controller or physical stairs.
- Native navigation: **31/31 stair tests passed**, covering ordinary side/below
  approaches, pending-request/first-feedback ordering, pause/cancel hold, and
  complete exit/restoration before an outside-flight goal finishes, and measured
  stop before the first restoration request. The initial
  Executor regression passed **102/102**; after the exit-arrival fix, the directly
  affected goal/arrival subset passed **14/14**. Full Linux `navd` builds, and
  `test_input_projector` passes, including preservation of profile source/receipt
  times across unrelated sensor batches. These are code/contract tests, not
  simulated or physical stair traversal. The later stair-retry and progress
  subset passes **14/14**, including both directions, retained collision stops,
  profile readiness loss, pause/cancel and unchanged ordinary recovery behavior.
- Gateway and selected-robot configuration tests: **15 passed**. The native
  recognition response is read-only in HTTP writes; Go2 and Thunder obtain
  their configured profile names from the same RobotConfig resolution path.
- Remaining delivery: automatic whole-map/live recognition; recorded real-stair
  accuracy and partial-view evaluation; a verified Go2 model API; installed,
  qualified Thunder profiles; full native MuJoCo traversal; and separate
  supervised field acceptance for ascent, descent and connected turning flights.

### Delivery slices and acceptance

| Slice | Deliverable | Evidence required |
| --- | --- | --- |
| Author and store | Manual 3D extent, lower/upper entrances, save/reload, edit/delete | Same XY on two floors stays distinct; coordinate/pose convention is correct; misses and camera drags create no false selections; no motion request; source geometry unchanged |
| Show and inspect | Highlight region and entrances, show ascending direction, restore manual height view | Rising stair connection remains inspectable; a selected region is not claimed to be per-point ground classification |
| Recognize | Saved/live 3D stair proposals with entrances and flight direction, manually correctable | Recorded real stairs, descending approaches, turning landings and partial views; ramp/obstacle negatives; reported geometry error and missed/false detections. Simulation object names or supplied labels are not perception evidence |
| Apply to navigation | Shared single-goal/inspection path segment handling, direction-dependent requirements | Only actual 3D traversal matches; ascent/descent differ correctly; canceled/replanned paths do not retain old segment state; platform-unavailable requirements are explicit |
| Integrate profile handoff | Platform entry preparation, concrete profile request/readback, on-flight tracking, exit restoration through the existing driver | Delayed/rejected requests do not enter the flight; actual profile readback and restored motion readiness precede entry; Grounded-only transitions occur on suitable landings; pause/cancel/replan while on stairs retains appropriate behavior; ordinary profile is restored only after exit. Contract tests do not prove the selected model can physically climb stairs |
| Qualify motion | Recognize and traverse stairs with the selected robot's real motion capabilities | Native simulation and supervised field evidence recorded separately for each robot/platform, SDK/firmware, stair geometry and ascent/descent. Test manual and automatic inputs, turning landings, interruptions and continuation to the final goal |

These slices do not require a new planner, semantic service, task state machine,
or a complete building/floor hierarchy. Display height controls and ordinary
navigation continue to work without staircase annotations.

Reference: Spot places staircase geometry and traversal preferences on
[map edges](https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/graph_nav/map.proto)
and represents stairs and their landings in
[stairs.proto](https://github.com/boston-dynamics/spot-sdk/blob/master/protos/bosdyn/api/stairs.proto).
Those schemas motivate the information boundary; they do not establish that
LingTu already implements Spot's recognition or locomotion behavior.

### Site Map Editor reference boundary

Boston Dynamics' [Site Map Editor Guide](https://support.bostondynamics.com/s/article/site-map-editor-guide)
distinguishes editing navigation topology from editing missions/actions. Its
visibility and height filters only change the view; hiding a recording does
not disable its navigation edges. Drawing alignment likewise does not change
robot navigation. These distinctions support keeping LingTu's display controls
separate from persistent traversal semantics.

Orbit also offers recording merges, suggested loop closures, user-created
edges, area-callback editing and continuous mission validation. These are
reference behaviors, not delivered LingTu capabilities or requirements for
this stair slice. The guide warns that manually created edges may be
unnavigable; creating an edge does not establish motion feasibility. It does
not describe switching an arbitrary robot's motion-control model/profile or
prove that LingTu has automatic stair recognition, profile handoff or a
qualified stair model. Retain the limited authoring workflow above rather
than importing Orbit's full editor scope.

## The six terms are not six semantic relations

The following names come from the LingTu map and navigation contracts. They are
different products with different owners and lifetimes, rather than six fields
that should all be copied into `object_memory`.

| Term | Meaning in LingTu | Lifetime | Can it authorize motion? |
| --- | --- | --- | --- |
| `live point cloud` | Current or rolling registered sensor observations used to update the live map and show recent geometry | milliseconds to minutes | No; it is observation evidence |
| `occupancy` | Voxel or grid belief that space is occupied, free, or unknown | live and/or saved derived artifact | Only through the native navigation contract; a display projection is not enough |
| `collision geometry` | 3D body-clearance representation, including the inflated collision envelope queried by SCAN/local safety | live, generation-stamped | Yes, as an input to native collision and final safety checks |
| `elevation` | Estimated surface height and local support shape | live or saved derived artifact | No by itself; it cannot establish clearance or free space |
| `local traversability` | Rolling control-risk/support data for the current robot and Product | short-lived, Product-dependent | Only through the declared native traversability owner; it is not semantic memory |
| `saved map` | The complete versioned map package: source geometry, metadata, navigation artifacts, and optional products | persistent, bound to `map_id` and `content_epoch` | Only after activation and all native readiness gates pass |

The source of these exact boundaries is the repository's native map contract,
especially [`src/maps/README.md`](../../src/maps/README.md),
[`docs/runtime.md`](../runtime.md), and the mapd/navigation code. No single
paper defines this six-item list. The research papers mainly inform the
semantic layer described below.

The important non-duplication rule is:

```text
object_memory.geometry_ref -> map_id + content_epoch + frame_id + spatial bounds
```

`object_memory` points at native geometry. It does not contain a private copy
of occupancy, collision geometry, elevation, or traversability.

## Proposed semantic layers

The useful product structure is deliberately small:

```text
native geometric map
  ├─ live point cloud
  ├─ occupancy / free / unknown evidence
  ├─ collision geometry
  ├─ elevation and support estimates
  └─ Product-specific local traversability

persistent semantic index
  ├─ object_memory: stable objects and their geometry references
  ├─ place/region nodes: rooms, floors, doors, stairs, named areas
  ├─ relations: near, inside, connected_to, above, blocks, leads_to
  └─ observation history and confidence

LLM representation
  └─ compact, read-only text/JSON projection of the semantic index
```

The first semantic release should keep only stable, useful nodes. It should
not persist every per-point embedding or every transient person detection.
This follows the task-driven compression idea in [Clio](https://arxiv.org/abs/2404.13696),
the object-centric representation in [FindAnything](https://arxiv.org/abs/2504.08603),
and the compact language-facing representation in [Tag Map](https://proceedings.mlr.press/v270/zhang25e.html).

## What `object_memory` means

`object_memory` is a map-bound object index, not another point-cloud map and
not the current frame's `SceneGraph`. One record should carry enough evidence
to find a candidate and enough identity to avoid silently attaching an old
object to a new map:

```text
object_id                 stable within one map identity
map_id
content_epoch
frame_id                  normally map
kind                      structural, furniture, device, person, etc.
label                     canonical label
aliases                   operator and open-vocabulary names
centroid / extent         3D summary geometry
geometry_ref              native voxel/submap/bounds reference
static_dynamic_class      structural, semi-static, or dynamic
existence_confidence
label_confidence
first_seen / last_seen
observation_count
state                     stable, moved, removed, uncertain, or stale
source                    sensor/model/review provenance
```

Relations are separate typed edges so that a weak label does not become a
false geometric fact:

```text
(object_42, inside, room_3)
(object_42, near, desk_7)
(room_3, connected_to, corridor_1)
(door_2, leads_to, room_3)
```

Every edge should carry the same map identity plus its own confidence and
last-observed time. `near` and `inside` are semantic hypotheses; native
collision and route queries remain authoritative for movement.

The hierarchy pattern is supported by [HOV-SG](https://arxiv.org/abs/2403.17846),
[Open3DSG](https://arxiv.org/abs/2312.02343), and [OrionNav](https://arxiv.org/abs/2410.06239).
The long-lived association and stale/moved handling are closer to
[SuperMap](https://arxiv.org/abs/2608.22896) and [DualMap](https://arxiv.org/abs/2506.01950).
These are design references, not classes already present in LingTu.

## How a semantic relation is added

The relation must be produced from a typed observation transaction:

```text
RGB-D / LiDAR observation
  -> map-frame geometry and detections
  -> deterministic object association
  -> object and relation update
  -> semantic artifact snapshot
  -> query/resolver
  -> native preview plan
  -> normal goal admission and motion gates
```

The association code, rather than an LLM, decides whether a new observation
matches an existing object. It uses the available 3D centroid/extent, label
compatibility, feature similarity, time continuity, and observation quality.
One missed frame lowers confidence; it does not immediately delete a stable
object. A moved object changes state and geometry history. A dynamic person or
temporary obstacle is retained as short-lived context and is not silently
written into static navigation geometry.

For LLM use, expose a small read-only projection such as:

```json
{
  "map_id": "building_a",
  "content_epoch": 42,
  "frame_id": "map",
  "objects": [
    {"object_id": "object_42", "label": "chair", "room": "office",
     "state": "stable", "confidence": 0.91}
  ]
}
```

The LLM may return a semantic candidate or a task decomposition. A
deterministic resolver then rechecks the active map identity, object freshness,
geometry reference, approach pose, and native `preview_plan`. Only the native
planner can turn that candidate into an executable route. This preserves the
boundary described by [DualMap](https://arxiv.org/abs/2506.01950) and the
language-facing map idea in [Tag Map](https://proceedings.mlr.press/v270/zhang25e.html)
without giving an LLM authority over coordinates, collision, or velocity.

## Task decomposition and robot execution

The robot-facing loop should be split into four contracts:

```text
user instruction
  -> LLM TaskPlan / TaskStep
  -> semantic query against the current hierarchical graph
  -> typed GoalCandidate
  -> native preview_plan
  -> native goal admission and execution
```

For example, “去会议室，找到靠近桌子的箱子，然后停在它前面” becomes
symbolic steps, rather than a list of model-invented coordinates:

```json
{
  "task_id": "task-001",
  "steps": [
    {"id": "room", "action": "resolve_room", "label": "meeting_room"},
    {"id": "box", "action": "resolve_object", "label": "box",
     "relation": {"predicate": "near", "label": "table"}},
    {"id": "go", "action": "navigate_to_candidate", "target_ref": "box",
     "stand_off_m": 1.0}
  ]
}
```

The current repository already has the transport and lifecycle surfaces:

| Stage | Existing surface |
| --- | --- |
| Submit natural language | MCP `send_instruction`, Gateway `/api/v1/instruction` |
| Ask for an LLM task loop | MCP `run_agent_task` when the semantic planner Product is loaded |
| Read the scene | MCP `get_scene_graph`; the returned snapshot now includes hierarchy fields when available |
| Rule/LLM decomposition | `SemanticPlannerModule.decompose_task` and `AgentPlannerModule` |
| Ground the active step | MCP `resolve_task_step`; queries the current `SceneGraph` and returns a candidate with `map_id`, `content_epoch`, and `frame_id` without dispatching motion |
| Semantic motion request | `navigate_to_object` / planner-owned instruction submission |
| Coordinate navigation adapter | `NavSkills.navigate_to`, which still goes through `nav.goals` |
| Native execution status | `get_navigation_status`, `get_navigation_result`, and native task status |

The first-stage `decompose_task` contract stops here. A rule-parsed command
returns `status: "parsed"` with a `task_plan.v1` symbolic plan. A complex command
returns `status: "needs_llm"` together with the decomposition prompt; it does
not invent a navigation subgoal. The later `parse_decomposition_response`
step accepts the LLM JSON and still strips coordinate, pose, and map-frame
fields before a semantic resolver is allowed to inspect the scene.

The next boundary is deliberately read-only: `resolve_task_step` takes that
symbolic plan, selects its pending step, and asks the existing fast resolver
to match a room or object in the current graph. For example, a `navigate`
step targeting `office` can return the room-center candidate, while a `find`
step with `relation=near` and `reference=desk` queries `box near desk`. The
result carries the graph's map identity and content epoch. It is a candidate,
not a motion admission; native preview and the navigation task lifecycle still
come later.

The normal instruction path now stops compound sentences at
`TASK_DECOMPOSITION_REQUIRED` instead of sending the whole sentence through
the single-goal navigation shortcut. Once the LLM returns a `task_plan.v1`
payload, the active step can be grounded explicitly and checked against the
current navigation map identity.

The actual authority chain is:

```text
MCP / Gateway
  -> AgentPlanner or SemanticPlanner
  -> semantic resolver
  -> nav.goals
  -> native global planner
  -> native local planner and collision gates
  -> lingtu-driver
  -> robot
```

MCP does not need a direct velocity or Brainstem tool. The existing `@skill`
discovery provides a control surface while keeping the native planner and
driver as the motion owners. `emergency_stop` remains a separate operator or
system safety action, not a normal LLM task step.

There is one current limitation to resolve before claiming a fully symbolic
LLM product: the internal AgentLoop still includes a coordinate-oriented
`navigate_to` handler for compatibility and tests. The semantic product should
eventually expose `resolve_target -> preview_plan -> execute_goal(plan_id)` to
the model and keep raw coordinate submission as an operator or planner-owned
adapter. The native gate already exists; this is a tool-catalog and task-plan
contract gap, not a reason to give the model direct driver access.

## How to verify the hierarchical scene graph

Verification is staged. A scene graph JSON containing rooms is not enough.

### L0: runtime message contract

Use a fixed tracker snapshot containing a floor, two rooms, groups, a door,
objects, and a `near` relation. Verify that:

1. the adapter preserves floor, room, group, object, and relation IDs;
2. room centers are retained;
3. `graph_level` is `hierarchical`;
4. `validate_hierarchy()` returns no errors;
5. serialization and deserialization preserve all hierarchy edges;
6. an unknown child or a hierarchy cycle is rejected by the structural check.

This is now covered by
[`tests/perception/test_scene_hierarchy.py`](../../tests/perception/test_scene_hierarchy.py).

### L1: semantic query and plan preview

Feed the validated graph to the semantic resolver and resolve:

```text
room = meeting_room
object = box
relation = near(table)
```

The result must include `object_id`, `map_id`, `content_epoch`, `frame_id`,
and semantic evidence. The resolver then calls native `preview_plan`; a
non-feasible preview must not call `send_goal`. A feasible preview produces a
native plan token or equivalent goal-admission receipt.

### L2: task lifecycle

Drive the same task through success, native rejection, localization
`DEGRADED`, timeout, cancellation, and map-identity change. The current step
must be the only step that can advance; a late recovery result from an old task
must not issue a new goal. The task result must distinguish LLM completion from
verified native motion completion.

### L3/L4: simulation and supervised field

Run the no-motion Product and native preview tests first. Simulation can prove
the task and navigation contracts, but it does not prove real locomotion.
Supervised field motion is the final gate after the graph, preview, localization,
collision, and stop paths are all green.

## What is implemented today

The answer to “这些层都已经梳理并把代码写好了吗” is **还没有全部完成**.
There is useful infrastructure, but not an end-to-end persistent semantic-map
product:

| Area | Current state | Evidence |
| --- | --- | --- |
| Native geometric layers | Implemented and owned by `mapd`/native navigation | [`src/maps/README.md`](../../src/maps/README.md), [`docs/runtime.md`](../runtime.md) |
| Runtime `SceneGraph` | Carries the flat compatibility fields plus tracker hierarchy records, map identity, content epoch, room centers, and structural validation | [`src/runtime/msgs/semantic.py`](../../src/runtime/msgs/semantic.py), [`src/perception/pipeline.py`](../../src/perception/pipeline.py), [`tests/perception/test_scene_hierarchy.py`](../../tests/perception/test_scene_hierarchy.py) |
| Semantic point labels | Per-frame projection exists; the current reconstruction path still needs one consistent frame contract before native ingestion | [`src/perception/reconstruction/reconstruction_module.py`](../../src/perception/reconstruction/reconstruction_module.py) |
| Semantic voxel core | Implemented as a native query/update/store foundation | [`src/maps/include/lingtu/maps/semantic/occupancy.hpp`](../../src/maps/include/lingtu/maps/semantic/occupancy.hpp), [`src/maps/cpp/semantic/occupancy.cpp`](../../src/maps/cpp/semantic/occupancy.cpp) |
| `semantic_map.bin` | Save/build support exists, but it is explicitly query-only and is not an activation-ready planning artifact | [`src/maps/cpp/build/pipeline.cpp`](../../src/maps/cpp/build/pipeline.cpp) |
| Live semantic ingestion in `mapd` | Not complete; there is no finished semantic DDS observation contract wired into `LiveMapEngine` | [`src/maps/cpp/mapd/`](../../src/maps/cpp/mapd/), [`src/message/idl/maps.idl`](../../src/message/idl/maps.idl) |
| Persistent `object_memory` | Not implemented as a map-bound native artifact with stable identity, history, and moved/stale state | [`src/memory/README.md`](../../src/memory/README.md) describes memory ownership |
| LLM semantic resolver to native preview | Offline contract now covers symbolic decomposition, hierarchy grounding, preview gating, and rejection; plan-token execution and persistent identity remain unfinished | [`tests/integration/semantic/test_task_flow.py`](../../tests/integration/semantic/test_task_flow.py), [`src/memory/README.md`](../../src/memory/README.md) |

The existing Python memory modules are useful semantic context, room/object
knowledge, temporal sightings, and vector retrieval. They are intentionally
not the owner of LiDAR accumulation, occupancy, save/reload, localization, or
motion. A new Python database for `object_memory` would create the second map
identity problem we are trying to avoid.

## Implementation order

The missing work should be done in this order, with one owner per boundary:

1. Make semantic labels and their points use the same declared frame.
2. Carry `map_id`, `content_epoch`, `reset_epoch`, and observation sequence
   through the semantic observation contract.
3. Add a typed semantic-label DDS input to `mapd`, reusing the existing map
   observation identity rather than sending the point cloud a second time.
4. Attach the existing semantic voxel core to live `mapd` state and reset it
   on map activation/reset.
5. Save semantic voxels and object records in the same native `MapStore`
   staging/activation transaction; do not add a second active-map store.
6. Add deterministic object association and relation confidence/history.
7. Expose a read-only semantic query and a native approach-pose/preview-plan
   resolver.
8. Connect an LLM only to the compact query and task-decomposition surface.

The first release is complete only when an LLM can be disabled and the native
mapping, save/reload, localization, global planning, local safety, and stop
path still work unchanged.
