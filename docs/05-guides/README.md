# Task Guides

These guides organize LingTu around outcomes: prove a local integration,
produce a map, validate a route, operate an approved navigation session, use
semantic goals, or integrate an external client. They complement the detailed
contracts; they do not replace a site safety procedure or a field-readiness
gate.

> **Status:** Current navigation entry point<br>
> **Audience:** Developers, integrators, and approved robot operators<br>
> **Runs on:** Local development hosts, simulation, and field robots as stated by each workflow

## Read this before selecting a command

The labels in this page describe two separate properties. A command can be
non-motion while still changing runtime state or persistent map data.

| Label | Meaning | Examples |
| --- | --- | --- |
| **Read-only** | Does not intentionally change robot, session, map, or runtime state. | Status, health, log, dataflow, and readiness queries. |
| **No physical motion** | Does not publish a goal or velocity command to move the robot. It may still start a session, relocalize, switch a backend, or write validation artifacts. | Offline saved-map validation, Gateway route preview, and the default system-acceptance gate. |
| **State-changing** | Changes a session, SLAM mode, active map, map artifact, service, or control lease. Run only while the robot is stationary and the impact is understood. | Mapping start/save, map activation, relocalization, and service restart. |
| **Can move hardware** | Can result in a robot velocity or an autonomous mission. Use only after the required field gate and local authorization. | Teleop, `nav goal`, semantic instructions, exploration start, and visual-servo find/follow. |

On a field robot, run `scripts/lingtu` from the authorized robot-side shell or
through the deployment method approved for that robot. Reusable documentation
uses placeholders such as `<robot>`, `<map-name>`, and `<x>` rather than
selecting a field target for you.

## Choose a workflow

| I need to… | Start here | Physical-motion boundary |
| --- | --- | --- |
| Verify code, ports, or a Blueprint | [Prove it without a robot](#1-prove-an-integration-without-a-robot) | `stub`, `dev`, and `sim` do not command a physical robot. |
| Build and preserve a field map | [Create a navigation map](#2-create-and-validate-a-navigation-map) | Manual collection can move hardware; saving and artifact checks do not. |
| Check whether a saved map can route to a target | [Validate a saved map and route](#3-validate-a-saved-map-and-route-without-motion) | Use no-motion previews first. |
| Navigate to coordinates | [Run an approved navigation session](#4-run-an-approved-navigation-session) | A submitted goal can move the robot. |
| Navigate from language or vision | [Use semantic navigation or visual servo](#5-use-semantic-navigation-or-visual-servo) | A semantic or visual request can become a navigation or velocity request. |
| Explore an unknown area | [Run exploration](#6-run-exploration) | Exploration creates autonomous motion. |
| Build a web, service, or agent integration | [Integrate an external client](#7-integrate-an-external-client) | Clients submit intent and observe state; they never own the final control path. |

The [Quick Start](../QUICKSTART.md) remains the Product/env and CLI source of
truth. The [field Product guide](../architecture/FIELD_PRODUCTS.md)
defines which chain owns each field Product, and the [testing index](../07-testing/README.md)
defines the validation evidence needed for a claim.

## 1. Prove an integration without a robot

Start with the smallest profile that covers the behavior under change. This
keeps software and wiring work separate from a field-motion claim.

| Goal | Recommended starting point | What it proves |
| --- | --- | --- |
| Framework, Module lifecycle, and graph wiring | `stub` | The framework can assemble without robot I/O. |
| Semantic pipeline with deterministic behavior | `dev` | Semantic modules can be exercised with mock backends. |
| Simulated navigation behavior | `sim` | Navigation logic can run without physical hardware. |
| Full simulated stack | `sim` | MuJoCo and the selected simulated integration path can run. |

**No physical motion:**

```bash
uv sync --locked
uv run --locked python -m lingtu.control --help
uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run
```

Use the locked `uv` invocation for development-host work. Follow the
[Get Started path](../01-getting-started/README.md) for Product/env selection and
the [development guide](../03-development/README.md) for the owning package
and focused test. A successful simulation run is evidence for the simulated
path only; it does not authorize deployment or physical motion.

## 2. Create and validate a navigation map

A LingTu map is a versioned package, not a standalone PCD file. Mapping owns
map collection and artifact production; navigation consumes the selected,
validated package. Do not manually create an `active` link, patch a map file
in place, or make a planner artifact look current after changing its source.
The [map service contract](../architecture/MAP_SERVICE_CONTRACT.md) defines the
transaction and ownership rules.

### Preconditions

Before beginning a field collection session:

1. Put the robot in the approved mapping area and establish the local safety
   and supervision arrangement.
2. Confirm no teleop, visual-servo, recovery, or navigation command source is
   active.
3. Use observation and no-motion readiness checks. A useful baseline is:

   ```bash
   bash scripts/lingtu status
   PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
   PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict
   ```

   These checks do not command a route, but they must pass before data is
   trusted for map production.

### Collection and save workflow

1. **State-changing, no autonomous motion:** enter the mapping product mode.

   ```bash
   bash scripts/lingtu switch map
   ```

   This switches the native SLAM path to mapping mode. It does not make the
   robot drive itself.

2. **Can move hardware:** collect coverage using the approved supervised
   teleoperation procedure. Keep this operator action separate from map
   saving; pause and make the robot stationary before the next step.

3. **State-changing, no physical motion:** commit the map under a meaningful,
   unique name.

   Submit the authenticated Gateway map-save request documented in the
   [Gateway map reference](../api/gateway_rest.md). Save produces a durable map
   package and may run optimization, cleanup, and
   artifact builds. If a save is asynchronous, keep its returned
   `operation_id` and follow the map-save operation through
   `GET /api/v1/maps/operations/{operation_id}` rather than issuing a
   duplicate request. The [Gateway map reference](../api/gateway_rest.md)
   describes the durable map operation endpoint.

4. **Read-only:** inspect the result before treating it as a navigation map.

   ```bash
   curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/maps"
   bash scripts/lingtu status
   ```

   Check the map name, optimization result, source/frame metadata, and the
   artifact capabilities selected for the active planner. A navigation-ready
   package normally includes the source map plus metadata and the appropriate
   occupancy/planner artifact; the precise capability is selected by the
   active planner rather than by a hand-maintained filename list.

5. **No physical motion:** validate the completed saved-map package, then run
   the integrated route gate instead of editing files or calling private map
   endpoints:

   ```bash
   PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-id> --require-occupancy
   PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" \
     --map <map-name> --goal <x> <y> <yaw>
   ```

   The first command is offline and read-only. `system-acceptance` is stateful
   but sends no motion unless `--allow-motion` is explicitly provided.

## 3. Validate a saved map and route without motion

Route validation has three useful levels. Choose the least invasive one that
answers the question.

| Question | Command or API | Effect |
| --- | --- | --- |
| Is the selected saved-map package complete for planning? | `PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-id> --require-occupancy` | Native mapd validation; no goal or velocity output. |
| Is the currently running system ready and is this route safe? | Gateway `POST /api/v1/navigation/plan` | No published goal; evaluates the running navigation path. |
| Can the native field path, map bundle, localization, and a requested route pass together? | `PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT"` | No physical motion by default; it is stateful because it samples runtime readiness and may start/end a session or relocalize. |

**Native saved-map artifact validation:**

```bash
PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-id> --require-occupancy
```

Use the saved map ID known by mapd. The command validates artifact provenance
and the required occupancy product without calling Gateway or publishing a goal.
It does not evaluate a route or prove that live localization is ready.

**Field-ready route evidence, no physical motion by default:**

```bash
PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" \
  --map <map-name> \
  --goal <x> <y> <yaw> \
  --with-relocalization
```

This gate starts with Gateway readiness, then records sensor/SLAM soak, saved-map
artifact evidence, relocalization, and a requested route preview. It omits a
motion smoke test unless `--allow-motion` is explicitly supplied. Treat
`--allow-motion` as a separate field-test authorization, never as the normal
next step after a preview.

The operations CLI does not expose a localization-backend A/B switch. Resolve
one Product inside one env and use `diagnostics.field.system_acceptance` for integrated evidence;
ProductControl owns every Product transition and rollback.

A successful preview means the requested path was feasible under the reported
map, frame, planner, and safety policy. It does not guarantee a future motion
result: live obstacles, localization health, safety state, and operator
takeover can still invalidate a mission. The [global planning contract](../architecture/GLOBAL_PLANNING_CONTRACT.md)
defines that distinction.

## 4. Run an approved navigation session

This section starts only after the site procedure and the appropriate no-motion
gate have passed. It is deliberately explicit about the point where an action
can move the robot.

1. **Read-only:** verify service, localization, safety, and map readiness.

   ```bash
   systemctl --no-pager --full status 'lt-*.service'
   curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
   bash scripts/lingtu status
   ```

   For saved-map navigation, do not proceed merely because a process is
   running. Confirm the chosen map is active, localization is healthy, and the
   `map_odom_tf` validity reported by the runtime is true. If relocalization was
   requested, require a completed relocalization result.

2. **State-changing, no goal yet:** load the map into a navigation session.

   ```bash
   bash scripts/lingtu switch nav --map <map-name> --initial-pose <x> <y> <yaw>
   ```

   Omit `--initial-pose` only when the approved global/reusable localization
   workflow applies. Starting navigation changes SLAM/session state but does
   not itself submit a navigation goal.

3. **No physical motion:** run the route-preview gate for the exact proposed
   target. Inspect planner selection, feasibility, path-safety result, frame,
   and any adjusted goal or fallback diagnostics before continuing.

4. **Can move hardware:** only an authorized operator may submit the mission
   goal.

   Submit the authenticated navigation goal through the Gateway API documented
   in [gateway_rest.md](../api/gateway_rest.md). The goal is in the
   map/planning frame and yaw is in radians. It enters the
   mission/planner path; it is not a direct motor command. In the field
   endpoint, the native navigation service owns the final `/nav/cmd_vel`
   writer, and the unique `lingtu-driver` service forwards checked commands to
   the remote Brainstem controller.
   The [field Product guide](../architecture/FIELD_PRODUCTS.md)
   and [navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md)
   describe the safety and arbitration boundary.

5. **Read-only while supervised:** observe mission state, selected command
   source, localization, and path. Do not repeatedly submit a goal to clear a
   blocker; inspect the reported blocker first.

6. **State-changing stop/recovery:** use a graceful mission cancel or session
   end for a controlled stop. For an immediate hazard, follow the local
   emergency procedure and use the documented emergency-stop surface; a
   normal `nav stop`/session end is not a substitute for site emergency
   response. See [operations](../06-operations/README.md) and the
   [Gateway command reference](../api/gateway_rest.md).

## 5. Use semantic navigation or visual servo

Semantic and visual features are request interfaces above the same planning and
safety boundary. They are not permission to bypass the route or velocity path.

### Natural-language and agent requests

In a Product that includes the semantic stack, an instruction request may
resolve tags, scene objects,
vector memory, topology/frontiers, and visual servo before it submits a
navigation goal. Therefore it is **can move hardware** once it is accepted by a
live field Product. Use it only after the navigation gate above and keep a
supervisor able to stop the mission.

For client integrations, `POST /api/v1/instruction` and MCP
`send_instruction` are the corresponding request surfaces. Check the active
Product capability and the generated [MCP inventory](../api/mcp_tools.md)
before relying on a method.

### Visual servo

Visual-servo `find` and `follow` requests can generate motion and are rejected
when Safety is stopped. Changing the target first cancels the previous visual
intent, then waits for a current target lock before submitting another goal.
`stop` requests cancellation of every native navigation task still owned by
VisualServo; native navigation status remains the source of truth for
terminal/parking confirmation. `follow` also requires an image-capable target
selector, and both modes require a ready detector. The feature is available only
when the resolved Product already loads `VisualServoModule`; ProductControl does
not create that module on demand.
Use the [visual-servo API entry](../api/gateway_rest.md)
for current request schemas and the [field Product guide](../architecture/FIELD_PRODUCTS.md)
for its control boundary.

## 6. Run exploration

Exploration is autonomous motion and needs an open, supervised area plus the
same readiness discipline as navigation. There is one operator Product:
`explore`.

```bash
bash scripts/lingtu switch explore             # live mapping
bash scripts/lingtu switch explore --map yard  # saved-map localization
```

The route is selected by the presence of `--map`; TARE is an internal policy
name, not another Product. Start the Product only after a no-motion readiness
check and an approved field plan. Start the motion-capable exploration task
through authenticated `POST /api/v1/explore/start`. Then observe status and stop exploration
promptly if localization, safety, or area conditions change. The REST
endpoints `POST /api/v1/explore/start` and
`POST /api/v1/explore/stop` are motion-capable control operations, while
`GET /api/v1/explore/status` is observation. The exact Product/Host wiring is in
the [field Product guide](../architecture/FIELD_PRODUCTS.md).

## 7. Integrate an external client

Use the Gateway and MCP as request/observation boundaries, not as a parallel
robot controller.

1. Read the active runtime capabilities before enabling a client action.
2. Use read-only state, health, readiness, localization, map, and navigation
   APIs to build the client view.
3. Preview a route before presenting a motion action to an operator.
4. Send goals, language instructions, teleop, or exploration commands only
   through the documented Gateway/MCP interfaces and only with an explicit
   authorization boundary in the client.
5. Surface stop/cancel, current command source, localization loss, and
   readiness blockers prominently in the client UI or automation log.

External clients must not choose a path directly, bypass the native command boundary,
or assume that a successful HTTP response proves a safe completed mission. See
[Reference](../08-reference/README.md) for transport, generated schema, and
tool-discovery guidance.

## When to stop and investigate

Do not advance to motion when any of these is true:

- the active map or its selected planner artifact is missing, stale, or from a
  different frame/source than the current session;
- localization is degraded/lost, `map_odom_tf` is invalid, or the robot pose is
  not credible in the active map;
- an unexpected command source, control lease, safety stop, or teleop client
  is active;
- a preview is infeasible, path safety is not OK, or readiness reports a
  blocker;
- native and compatibility sensor/SLAM services are both active without an
  explicit compatibility test plan.

Use the [operations decision path](../06-operations/README.md) to isolate the
condition, then repeat the smallest relevant no-motion gate. Field evidence
belongs in the appropriate [testing and validation](../07-testing/README.md)
record rather than being inferred from a successful command exit code.

## Up next

For ongoing monitoring, service recovery, and evidence collection, continue
with [Operations](../06-operations/README.md). For REST, MCP, CLI, profile,
and contract lookup, use [Reference](../08-reference/README.md).
