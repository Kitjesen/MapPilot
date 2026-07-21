# Reference

Use this section to find the exact command, endpoint, configuration source, or
architecture contract after choosing an outcome-oriented workflow. It is a
lookup layer: generated inventories describe current API shape, architecture
contracts define ownership, and task/operations pages define the safe sequence
for using a state-changing or motion-capable action.

> **Status:** Current navigation entry point<br>
> **Audience:** Developers, integrators, automation authors, and operators<br>
> **Runs on:** All supported environments, subject to the active profile and runtime capability set

## Choose the authoritative source

| Need | Use this first | Why |
| --- | --- | --- |
| Choose a profile or run a local command | [Quick Start](../QUICKSTART.md) | It defines the current profile matrix, lifecycle commands, and common overrides. |
| Follow a map, route, semantic, or exploration procedure | [Task Guides](../05-guides/README.md) | It separates no-motion validation from actions that can move hardware. |
| Monitor, diagnose, or restart a field system | [Operations](../06-operations/README.md) | It defines the observe → isolate → recover → revalidate sequence. |
| Find REST request/response models and routes | [Gateway REST API](../api/gateway_rest.md) | It is generated from current route registrations. |
| Find MCP methods and parameters | [MCP tools](../api/mcp_tools.md) | It is generated from `@skill` methods. |
| Understand Module, Blueprint, transport, map, planning, or frame ownership | [Architecture index](../architecture/README.md) | It links current architecture contracts rather than plans or historical evidence. |
| Know whether a change is sufficiently verified | [Testing and validation](../07-testing/README.md) | It distinguishes local, simulation, and hardware evidence. |

Do not use a dated field run, an archive page, or a future plan as an API or
runtime contract. [`CURRENT.md`](../CURRENT.md) identifies the source of truth
when documents overlap.

## Interface map

| Surface | Primary purpose | Normal user | Authoritative details |
| --- | --- | --- | --- |
| `lingtu.py` / `lingtu` console command | Local profile selection, development lifecycle, REPL, and simulation. | Developer or local integrator. | [Quick Start](../QUICKSTART.md) |
| `scripts/lingtu` | Robot-side service, map, navigation, diagnostics, and evidence operations. | Field operator or maintainer. | [Operations CLI](../04-deployment/lingtu_cli.md) |
| Gateway REST | Typed HTTP requests for status, maps, sessions, navigation, diagnostics, and media. | Dashboard, service, or integration client. | [Gateway REST API](../api/gateway_rest.md) |
| SSE | Event stream from `GET /api/v1/events`. | Observability client. | [Gateway REST API](../api/gateway_rest.md) |
| WebSocket | Teleoperation and JPEG camera fallback at the Gateway. | Approved interactive client. | [Quick Start](../QUICKSTART.md) and [Gateway REST API](../api/gateway_rest.md) |
| WHEP proxy | Low-latency camera signaling via the configured media sidecar. | Video client. | [Gateway REST API](../api/gateway_rest.md) |
| MCP | JSON-RPC tools auto-discovered from loaded Module `@skill` methods. | Agent or MCP-compatible client. | [MCP tools](../api/mcp_tools.md) |

The Gateway is an application boundary, not a second navigation planner. CLI,
web, REST, and MCP clients submit intent and display state. They must not choose
a path themselves or bypass the Module/native control and safety path.

## Command-line entry points

### Local development and simulation

Use the locked environment from the repository root:

```bash
uv sync --locked
uv run --locked python lingtu.py --list
uv run --locked python lingtu.py stub
```

After installation, the `lingtu` console command is equivalent to the locked
Python invocation. Use the full `uv run --locked` form in reproducible scripts
so dependency-lock drift fails early. The CLI profile catalog comes from
`src/runtime/profiles/catalog/`; `cli/profiles_data.py` is a compatibility
export, not a second source of product defaults.

### Field operations

Run the robot operations CLI from an authorized robot-side shell:

```bash
bash scripts/lingtu status
bash scripts/lingtu doctor --non-motion --json --strict
bash scripts/lingtu health
```

Use the deployment procedure to choose and authenticate to the intended robot.
Shared documentation intentionally uses no field endpoint or address. The
operations CLI contains both read-only diagnostics and state-changing/motion
commands, so read the command effect in [Operations](../06-operations/README.md)
before using a subcommand in automation.

## Runtime vocabulary

Do not collapse these terms into a single "mode." They answer different
questions and are independently visible in runtime status.

| Term | Example | Meaning |
| --- | --- | --- |
| **Profile** | `stub`, `sim`, `map`, `nav`, `tare_explore` | Named product configuration and selected module/backend graph. |
| **Endpoint** | `thunder_field`, `mujoco_live`, `replay` | Where data enters and which process boundary owns I/O. |
| **Session mode** | `mapping`, `navigating`, `exploring` | Coarse Gateway resource session controlling shared behavior. |
| **Product session** | `teleop`, `mapping`, `navigation`, `inspection`, `exploration` | Operator-facing activity. |
| **SLAM mode** | `mapping`, `localization` | Whether the native SLAM path is building or aligning to a map. |
| **Active map** | `<map-name>` | The map package/version selected by MapsService for a consumer. |

The [product mode runtime contract](../architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md)
defines the valid chains and switching policy. A process/endpoint that reports
healthy is still not necessarily ready to accept a navigation goal.

## Gateway REST

The Gateway serves REST and related interfaces on the configured robot target.
Use a placeholder base URL in portable clients:

```bash
export LINGTU_GATEWAY='http://<robot>:5050'

# Read-only discovery and health checks.
curl -fsS "$LINGTU_GATEWAY/api/v1/auth/check"
curl -fsS "$LINGTU_GATEWAY/ready"
curl -fsS "$LINGTU_GATEWAY/api/v1/health"
curl -N "$LINGTU_GATEWAY/api/v1/events"
```

Check `auth/check` before assuming whether a client needs credentials. REST
uses JSON with Pydantic request/response models; a validation failure normally
returns HTTP 422. Treat the generated [Gateway REST API](../api/gateway_rest.md)
as the source for field names, models, and response semantics—do not copy a
payload shape from an old dashboard or script.

### Endpoint effect categories

The HTTP method alone does not tell you whether an endpoint is safe to call
during a field run. Use this table before writing an integration.

| Category | Representative routes | Effect and client rule |
| --- | --- | --- |
| Observation | `GET /health`, `GET /ready`, `GET /api/v1/health`, `GET /api/v1/session`, `GET /api/v1/navigation/status`, `GET /api/v1/path`, `GET /api/v1/readiness` | Read state only. Poll at a sensible rate and surface stale/unready state to users. |
| No-motion planning | `POST /api/v1/navigation/plan`, `POST /api/v1/navigation/goal_candidate`, `POST /api/v1/maps/{name}/validate_plan` | Constructs or previews a plan without publishing a navigation goal. Inspect feasibility, frame, and path safety before presenting a motion action. |
| Stateful but not a motion request by itself | Map save/activate/rollback/artifact routes, session routes, relocalization, runtime switch planning/execution, and configuration/backend routes | Can alter persistent data or runtime ownership. Require idle state, explicit operator intent, and a post-change readiness check. |
| Can move hardware | `POST /api/v1/cmd_vel`, `POST /api/v1/goal`, `POST /api/v1/instruction`, `POST /api/v1/navigate/click`, `POST /api/v1/explore/start`, and visual-servo find/follow | May generate motion. Gate behind explicit operator authorization and continuously show stop/cancel state. |
| Stop/cancel | `POST /api/v1/stop`, `POST /api/v1/navigation/cancel`, session end, exploration stop, and visual-servo stop | Changes control state. Use the local emergency procedure for hazards; do not infer that a stop proves the fault has cleared. |

The generated API page is an inventory, so its exact routes can evolve with
source. A client must read active capabilities and health/readiness at runtime,
not assume every documented route is available in every profile.

### Media and event streams

- Use `GET /api/v1/events` for server-sent status events rather than polling
  every detailed endpoint at high frequency.
- The teleop WebSocket is `ws://<robot>:5050/ws/teleop`. Teleop is a
  motion-capable client; it must honor its control lease/deadman behavior and
  the current safety state.
- Camera use has a WHEP proxy fast path and a Gateway camera fallback. Treat
  video availability as observability, not proof that navigation inputs are
  healthy.

## MCP

`MCPServerModule` exposes discovered Module skills through JSON-RPC at:

```text
http://<robot>:8090/mcp
```

For a Codex client, the documented registration form is:

```bash
codex mcp add --transport http lingtu http://<robot>:8090/mcp
```

Tool availability depends on the Module graph loaded by the active profile.
The generated [MCP tools inventory](../api/mcp_tools.md) tells you the method
names, modules, descriptions, and parameters, but the live server/capability
result is authoritative for a specific session.

Use tools by effect, not merely by their name:

| Tool family | Typical use | Safety rule |
| --- | --- | --- |
| Health, module, configuration, scene, map, and memory queries | Build context and monitor the system. | Read-only, but verify freshness and profile capability. |
| Map save/use and tagged-location mutations | Maintain persistent robot context. | Stateful; require a stable session and record the resulting map/version. |
| Navigation, semantic instruction, patrol, visual follow, and exploration tools | Request mission or tracking behavior. | Motion-capable; apply the same field gate as an interactive goal. |
| Stop/cancel and emergency-stop tools | Halt/release a mission or safety state according to their documented semantics. | Never use as an automatic substitute for physical incident response. |

An agent can select a tool but cannot bypass the planner, `CmdVelMux`, safety
gate, or the native field command owner. That boundary is intentional.

## Maps, frames, and planning data

Use MapsService rather than filesystem conventions as the map API. A saved map
is an immutable/versioned package with metadata, selected artifacts, checksums,
and lifecycle state; it is not just `map.pcd`. Import, crop, save, restore,
activate, artifact-build, and rollback operations must go through the map
service so derived data is invalidated and published atomically.

| Question | Contract |
| --- | --- |
| Who owns map lifecycle, artifacts, versions, and active slots? | [Map service contract](../architecture/MAP_SERVICE_CONTRACT.md) |
| What does a global plan request/result contain? | [Global planning contract](../architecture/GLOBAL_PLANNING_CONTRACT.md) |
| Who owns mission, local planning, path following, safety, and velocity priority? | [Navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md) |
| What are the allowed runtime port/channel/transport boundaries? | [Runtime bus contract](../architecture/LINGTU_RUNTIME_BUS_DECISION.md) |
| Which frame names and ROS compatibility constraints apply? | [Frame contract](../architecture/ros_frame_contract.md) |

All navigation coordinates must be interpreted in the selected planning frame.
The planner blocks a frame mismatch rather than guessing a transform. A map
artifact and a live localization pose from incompatible frames or sources are a
failed precondition, not a reason to transform values in a client.

## Configuration and extension points

| Surface | Source of truth | Use it for |
| --- | --- | --- |
| Product profiles and endpoint catalog | `src/runtime/profiles/catalog/` | Profile defaults, endpoint selection, and runtime identity. |
| Robot/device configuration | `config/` | Physical geometry, calibration, device registration, and runtime settings. |
| Module/back-end registration | `src/runtime/registry.py` and stack factories | Pluggable implementations without direct cross-layer imports. |
| Full product wiring | `src/lingtu/assembly/` | Explicit critical wires and graph assembly. |
| Public API inventories | `docs/api/` | Generated REST/MCP schema lookup. |

When changing one of these surfaces, use the owning package and architecture
contract before adding an abstraction or dependency. The [development guide](../03-development/README.md)
points to the correct package-level entry point.

## Regenerate and verify reference inventories

Regenerate API inventories whenever route registrations or `@skill` methods
change:

```bash
python scripts/docs/extract_api_docs.py
```

Review the generated diff alongside the source change. Then verify the curated
entry-point links:

```bash
python -m pytest tests/docs/test_documentation_navigation.py -q
```

Generated docs do not replace a task guide, a safety boundary, or a field
acceptance record. If a code change changes an operator-visible behavior, update
the relevant task/operations page and add the narrowest appropriate test or
field evidence as well.

## Up next

Return to [Task Guides](../05-guides/README.md) for a procedure, or use
[Operations](../06-operations/README.md) when a running robot needs
observation, recovery, or an evidence-backed handoff.
