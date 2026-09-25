# Roadmap

**Status:** Active product roadmap
**Audience:** Product, runtime, release, simulation, and field maintainers
**Runs on:** Planning across local, simulation, release, and S100P targets

This page contains only remaining cross-domain work. Shipped behavior belongs
in the owning code, configuration, and current documentation.

## Fixed boundaries

- Public runtime environments are exactly `real` and `sim`.
- Product declarations are env-independent.
- ProductControl resolves once and owns the full lifecycle.
- `real` uses systemd; `sim` owns direct child processes.
- Native sensor, SLAM, maps, traversability, navigation, and driver processes
  own the field hot path through typed DDS.
- `navd` owns final navigation arbitration; only `lingtu-driver` forwards the
  checked command to the selected robot adapter.
- Blueprint owns only the in-process Python Host Module graph.
- Windows x64, Linux/WSL x86_64, and S100P aarch64 require separate evidence.

## Active work

| Priority | Work | Completion gate |
| --- | --- | --- |
| P0 | Native MuJoCo Product parity | Every Product passes lifecycle, scenario, terminal-zero, cleanup, rollback, and repeatability on Windows and Linux/WSL independently |
| P0 | Canonical native release | `install/.../{bin,lib,etc,share}` passes package, install, activation, and rollback without production references to `build/` |
| P0 | Stable mapping/no-map `teleop_avoid` | Native sensor-to-command flow survives representative free-space, obstacle, stop, cleanup, and repeatability scenarios |
| P0 | S100P physical command safety | Fresh provenance, no-motion, fault injection, bounded supervised motion, terminal zero, and driver ACK pass on target |
| P1 | Dynamic-obstacle clearing and resource bounds | Labelled replay and long MID-360 runs meet residual, thin-obstacle, reset, CPU, memory, and DDS-volume limits |
| P1 | Motion and collision-aware path smoothing | Ramp, reversal, rotation, emergency zero, and safety ordering pass with an explicit path-smoother failure policy |
| P1 | Typed native map control/query | Persistent field operations no longer require a competing Python map-management path |
| P1 | Online global mapping backend | Resolve rejected-keyframe coverage, verify corrected save/reload, measure paced ARM performance, then complete supervised mapping/navigation; follow [the mapping execution plan](../src/localization/opt/NEXT_STEPS.md) |
| P2 | Route, following, and docking Products | Each has an explicit Product, typed lifecycle, safety boundary, simulation evidence, and physical evidence where applicable |
| P2 | Cross-language schema stability | DDS and shared-memory payloads carry explicit version, frame, timestamp, bounds, and compatibility tests |

## Execution order

1. Build each selected native artifact from the current source and IDL.
2. Resolve the exact Product and reject stale, missing, mixed-platform, or
   undeclared artifacts.
3. Run ProductControl from a fresh state root.
4. Attach the named simulation scenario to the same Product session.
5. Prove terminal zero, cleanup, and rollback.
6. Repeat independently on Windows and Linux/WSL.
7. Assemble and validate the canonical Linux field release.
8. Run S100P no-motion and fault injection before supervised motion.

## Map and place ownership review — 2026-09-17

The reviewed path is Web/API -> place lookup -> mapd -> navigation dispatch,
including the actual `real` Product compilation. This is a source and local
contract review, not validation of the installed robot release.

The field geometry pipeline is already native: sensors, Fast-LIO2, mapd,
traversability, navd, and driver. Python legitimately owns Host wiring, HTTP,
semantic names, and task intent. The remaining problem is competing place
ownership and interface drift, not the use of Python for those functions.

| Finding | Evidence and effect | Required change |
| --- | --- | --- |
| Two place stores | Web `/locations` and location-name goals use `TaggedLocationStore`; `/places` and semantic place resolution use the POI-backed `PlaceCatalog`. Creating an entry in one does not populate the other. | Make mapd the one map-bound place owner; retain semantic alias/floor interpretation as a Python projection. |
| Basic locations depend on optional semantic memory | Compiling standard `map` and `nav` for `real` includes Gateway but no `TaggedLocationsModule`. `memory()` is gated by `enable_semantic_planning`, false in both Products. The location API then reports `location_store_unavailable`. | Make basic map-bound locations available through the declared maps endpoint, without enabling the entire semantic stack. |
| Names collide across maps | The JSON store is keyed only by name. Saving `dock` with map A metadata, then `dock` with map B metadata leaves only B. | Use map-scoped identity, preserve stable place IDs, and explicitly resolve existing JSON records during consolidation. Do not silently assign unbound records to the active map. |
| Lookup repeats management requests | `PlaceCatalog.load()` makes `1 + 2N` synchronous management requests across N maps: list maps, then record and POIs per map. A local fake endpoint confirmed seven calls for three empty maps. | Consolidate the existing mapd query contract and duplicated Gateway/semantic adapters; measure latency before adding caches or another database. |
| Import checks have limited coverage | The current architecture checker reports an existing missing ownership entry for `src/lingtu/operator_keyboard.py`. Import rules cannot detect duplicate place stores or absent Product capabilities. | Declare that existing file's actual ownership and add acceptance coverage at the Product/API seam. A clean import check alone is insufficient. |

Sources:

- [Web location API](../src/gateway/maps/locations.py),
  [location-name goal resolution](../src/gateway/navigation/goals.py),
  [JSON storage](../src/memory/spatial/tagged_locations.py).
- [Place API](../src/gateway/maps/places.py),
  [semantic place catalog](../src/memory/spatial/places.py),
  [semantic planner](../src/decision/modules/semantic_planner.py),
  [native POI storage](../src/maps/cpp/service_places.cpp).
- [Host stack composition](../src/lingtu/assembly/stacks/composition.py),
  [map Product](../config/runtime_graph/products/map.yaml),
  [nav Product](../config/runtime_graph/products/nav.yaml).

Consolidation should proceed as one vertical slice: create a map-bound place
from the Web, list it, resolve the same identity through semantic input, then
submit it through the existing checked navigation command path. Prove this
with `semantic_planning` disabled for basic location operations, same-name
places on two maps, restart/reload, active-map changes, and map content updates.
Unbound or stale entries must remain non-executable until deliberately rebound.
Next migrate existing location records and their callers; only then remove the
JSON owner and redundant adapters. Moving files or adding another `places/`
wrapper before that would preserve the underlying split.

Acceptance must include actual HTTP request models and I/O failure behavior,
not just schema presence or manually injected Modules. Simulation and field
acceptance remain separate, with no-motion checks before supervised motion.

Follow-up local review reproduced and fixed four additional defects:

- A deleted saved goal could fuzzy-match another name and dispatch different
  coordinates. Structured location targets now require an exact name.
- Saving the current location accepted a cached pose after sensor loss, a
  localization failure, or a frame mismatch. It now requires a fresh, valid
  map-frame snapshot and preserves the previous location on rejection.
- SDK PCD downloads treated an early HTTP EOF as success and replaced the
  existing map with partial or empty data. Replacement now requires the
  advertised byte count; failure removes the temporary file only.
- Cancelling an artifact iterator before its first read orphaned the native
  file descriptor. Ownership now stays with the artifact until reading starts.

Regression evidence uses actual Gateway request models, the standard-library
HTTP response reader, and real temporary file descriptors. These fixes do not
resolve the two-store/Product availability findings above and are not evidence
of installation or motion acceptance on the robot.

A subsequent entry-point review reproduced four more contract failures:

- SDK `delete_location` sent POST to a DELETE-only route. It now sends DELETE
  and encodes the location name; the regression invokes the real HTTP route.
- MCP `tag_location` bypassed Gateway pose checks and erased existing map
  binding/yaw by writing directly to the JSON store. HTTP and MCP now share
  the same save operation, including failure reporting.
- The agent's fallback sent unsupported `tag:` commands and always claimed
  success. That fallback is removed; the registered tagging skill supplies
  the capability when available.
- Updating a bound location while mapd binding was unavailable silently
  replaced it with an unbound record. The operation now fails without changing
  the existing location.

Ten new interface regressions and the adjacent SDK, MCP, agent, and location
checks passed (190 tests, plus 65 subtests). This remains local contract
evidence, not a robot deployment or navigation acceptance result.

## Semantic navigation integration

The current `inspection` Product selects semantic planning; standard `nav`
does not, and `tracking` excludes `SemanticPlannerModule`. Do not infer a
capability from helper classes or a directory name.

Native request/task correlation and terminal handling now have local contract
coverage, including cancellation, replacement, native recovery and map changes.
Semantic grounding uses the synchronized map pose from perception, not raw odom.
Same-floor observation proposals now use the registered native read-only path
preview before dispatch, with local tests for stale tracks and delayed results.
Reached tracked objects now have a bounded visual-verification path using the
processed image, synchronized pose and existing vision-capable LLM interface.
The remaining work is recorded-image/model acceptance, visibility-aware viewpoint
changes, owner-supplied frontier input and native/field runtime acceptance.
SG-Nav reasoning and belief-verification helpers currently
have no runtime planner callers. Local service contracts do not establish that
the robot searches or confirms targets correctly.

The [literature and implementation review](../research/semantic/navigation_review.md)
compares primary sources and gives the staged replay, simulation, no-motion,
and supervised field gates. Task handoff, cached-scene freshness, delayed
recovery, robot-position propagation and concrete strategy API regressions are
covered in `tests/decision/test_semantic_planner_task_lifecycle.py` and
`tests/decision/test_strategy_service_contracts.py`.

### Inspection task and motion planning

The [inspection execution plan](../research/semantic/inspection_motion_plan.md)
centres route planning, native motion and recovery, observation viewpoints,
and explicitly selected person following. It separates existing source support
from missing end-to-end evidence and proposes platform-specific acceptance
targets. Start with three-point inspection and terminal motion evidence;
shared detection and asynchronous review remain supporting workstreams.
The proposal does not change the existing release or field safety gates.

### Edge following, speech, and patrol integration

The [edge integration flow](../research/semantic/edge_follow_patrol_integration.md)
maps the reviewed RDK patrol implementation to existing LingTu owners. Open work
includes real detector/ReID assembly, time-aligned target coordinates across
localization corrections, shared 2-D observations for patrol rules, and offline
speech. The current MuJoCo shirt-color fixture and local following tests do not
establish learned person detection or field identity retention. Integrate and
validate these capabilities without adding a second camera, lifecycle, or motion
owner.

## Terrain research threshold

Do not add PCA ground segmentation merely because another project uses it.

Only clean-room implement the necessary portion if representative real MID-360
data shows that the current terrain core repeatedly misclassifies slopes,
stairs, or mixed cells and the failure changes motion safety.

The acceptance set must compare the current terrain owner and the candidate on
the same labelled data, including thin obstacles, reset behavior, CPU, memory,
and latency on the S100P target. Simulation alone cannot trigger adoption.

## Release and license gates

FAST_LIO, ikd-Tree, and IKFoM provenance and license obligations require an
explicit release decision. Replacing one container does not establish a
closed-source distribution boundary.

Before commercial distribution, choose one complete path:

- authorization covering the relevant copyright chain;
- a reviewed GPL-compliant distribution boundary; or
- a clean-room replacement of the affected SLAM core.

This is an engineering release gate, not legal advice.

## Non-goals

- Do not make ROS 2 the Product API.
- Do not expose planner backend internals to Gateway or Web.
- Do not add a second Product lifecycle, map hot path, field planner, or command
  writer.
- Do not duplicate algorithms under root `real/` and `sim/` trees.
- Do not add a custom transport framework before the typed schema boundary is
  stable.

## Claim discipline

Unit tests do not prove a Product. WSL does not prove Windows-native behavior.
Simulation does not prove S100P motion safety. A roadmap item closes only when
its named evidence gate passes and the owning contract is updated.
