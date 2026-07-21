# LingTu Quick Start

This is the executable reference for starting a local profile, selecting a
simulation, or preparing a field session. It keeps local, simulated, and
physical command boundaries separate so a convenient test command cannot be
mistaken for authorization to move a robot.

> **Status:** Current<br>
> **Audience:** Developers, integrators, and supervised robot operators<br>
> **Runs on:** Local development hosts, simulation, and supported field robots

## Read this before running a profile

LingTu's product model is:

```text
Module ports -> explicit wires -> selected transport -> safety/mux -> command sink
```

A profile says what LingTu should do; an endpoint says where its sensor inputs
and command output are connected. The native field path uses typed DDS and
native C++ services at its process boundaries. ROS 2 is available only as an
explicit compatibility or evaluation surface.

For the current physical Thunder field endpoint, `thunder_field` resolves to an
endpoint-owned command boundary: `lingtu-nav-dds` publishes `/nav/cmd_vel`, the
single `lingtu-driver` service consumes it, and that driver sends checked gRPC
commands to a remote Brainstem controller from `brainstem.env`.

The following statement is always true:

- A local test does not prove a simulation.
- A simulation result does not prove a field robot.
- A running process does not prove localization, map provenance, route safety,
  or readiness for motion.
- A navigation goal must still traverse planning, safety, and velocity
  arbitration before it can reach the driver.

Read [Get Started](./01-getting-started/README.md) for the staged onboarding
path and [Core concepts](./02-concepts/README.md) for the architectural model.

## Command convention

Run local LingTu CLI commands from the repository root with this exact form:

```bash
uv run --locked python lingtu.py <command-or-profile> [options]
```

This documentation intentionally does not use an unqualified `lingtu` console
script for local runs. The locked form uses the checked-in entry point and
fails if dependency metadata and `uv.lock` are inconsistent.

Robot-side service operations are different: on the deployed Linux target, use
the checked-in shell CLI:

```bash
bash scripts/lingtu <subcommand>
```

Do not substitute ad-hoc HTTP calls, `systemctl` sequences, or a sourced ROS 2
overlay for the robot-side operations CLI.

## Prepare and inspect the checkout

Install the development toolchain and inspect configuration before starting any
runtime:

```bash
uv sync --locked --extra dev
uv run --locked python lingtu.py --list --all
uv run --locked python lingtu.py show-config stub --json
uv run --locked python lingtu.py runtime-contract
uv run --locked python lingtu.py runtime-audit
```

| Check | Expected result | Motion boundary |
| --- | --- | --- |
| `uv sync --locked --extra dev` | The locked environment resolves without rewriting `uv.lock`. | None |
| `--list --all` | Current product, simulation, development, and compatibility profiles are listed. | None |
| `show-config ... --json` | A profile resolves without starting its modules. | None |
| `runtime-contract` | The canonical frame, stream, and command-boundary summary is printed. | None |
| `runtime-audit` | Repository contracts are checked for drift. | None |

If a locked command reports that `uv.lock` needs updating, do not rerun it
without `--locked`. Restore the intended checkout or make an explicit,
reviewed dependency change first. A lock mismatch is an environment-integrity
failure, not a profile failure.

## Select a profile deliberately

Use `--list` for the normal product list and `--list --all` for the full
catalog. The table below is a decision aid, not a replacement for the runtime
catalog.

| Profile or family | Primary use | Command sink | Hardware motion |
| --- | --- | --- | --- |
| `stub` | Framework and Blueprint/port/wire checks | Stub driver | No |
| `dev` | Semantic and planning development with a mock LLM | Local/stub graph | No |
| `sim_nav` | No-ROS navigation simulation | Simulation graph | No physical motion |
| `sim` | In-process MuJoCo Module stack | MuJoCo driver | Simulated motion only |
| `portable_mujoco` | Portable no-ROS MuJoCo planning/sensor path | MuJoCo driver | Simulated motion only |
| `teleop`, `teleop_avoid`, `lite` | Field product modes with different operator/control scope | Selected field endpoint | Potentially |
| `map` | Build a saved map | Selected field endpoint | Mapping session may involve supervised manual motion |
| `tracking`, `nav`, `inspection` | Saved-map tracking, navigation, or inspection | Selected field endpoint | Potentially |
| `explore`, `tare_explore` | Frontier exploration variants | Selected field endpoint | Potentially |
| `sim_gazebo`, `sim_industrial`, `sim_mujoco_live`, `sim_mujoco_octo_live`, `sim_cmu_tare` | Advanced simulation/validation profiles | Simulator or replay endpoint | Simulated motion only |
| `super_lio`, `super_lio_relocation` | Explicit Super-LIO evaluation profiles | Selected evaluation endpoint | Potentially, only after field gates |

The `map`, `nav`, `explore`, and `tare_explore` names describe product
tasks, not safe local demos. Do not launch them just to test a workstation.
First inspect the selected endpoint and then follow the appropriate simulation
or field gate.

## Local: prove the framework path

### 1. Start the no-hardware profile

```bash
uv run --locked python lingtu.py stub --no-gateway
```

When started in a TTY, the process opens the interactive REPL. Use inspection
commands only:

```text
health
connections
config
quit
```

Expected result: a Module graph starts with the stub driver, reports local
health and wiring, and exits cleanly. No robot connection, native field service,
or hardware velocity writer is used.

`--no-gateway` makes this first exercise independent of the HTTP/Gateway
optional dependency and local port availability. Omit it only when you
intentionally want to test the local Gateway surface.

### 2. Start semantic development with deterministic behavior

```bash
uv run --locked python lingtu.py dev --llm mock --no-gateway
```

The `dev` profile enables the semantic path while retaining a mock LLM. It is
appropriate for Module and message-flow development. It is not evidence that a
selected detector, a cloud model, a camera, or physical navigation is working.

If an intentionally selected backend reports a missing optional package, install
only its required extra. The available groups are described in the
[Build Guide](./01-getting-started/BUILD_GUIDE.md#select-the-python-environment).

### 3. Local lifecycle and diagnostics

These commands inspect or manage a locally started daemon. They do not create a
field session by themselves:

```bash
uv run --locked python lingtu.py status
uv run --locked python lingtu.py health
uv run --locked python lingtu.py log -f
uv run --locked python lingtu.py stop
```

`--daemon` is a Unix background-process option. Do not use it as a substitute
for robot service management; deployed field services are managed through
`scripts/lingtu` and the deployment runbook.

## Simulation: prove a named simulation boundary

### 1. Install the simulation dependency group

```bash
uv sync --locked --extra dev --extra sim-mujoco
uv run --locked python lingtu.py runtime-spec sim --json
```

The preflight output should identify a simulation data source/command sink. If
it instead identifies a physical endpoint, stop and resolve the configuration
before launch.

### 2. Run a simulation

```bash
uv run --locked python lingtu.py sim
```

For the no-ROS navigation-oriented profile:

```bash
uv run --locked python lingtu.py sim_nav
```

A simulation may produce `cmd_vel`, but it remains inside the selected
simulated driver or adapter. Never append a physical endpoint to either command
as a shortcut to field testing.

For scene selection, MuJoCo drive modes, saved-map quality gates, and the
meaning of individual simulation claims, read
[`sim/README.md`](../sim/README.md) and the
[simulation integration contract](./architecture/SIMULATION_INTEGRATION_CONTRACT.md).

## Field: prepare, validate, then supervise

Field work is a separate, operator-controlled workflow. Run its shell commands
on the target controller or an approved remote session after completing
[deployment](./04-deployment/README.md). A successful local or simulation
command does not remove any of these requirements.

### Field preconditions

Before changing product mode, mapping, or navigation state:

1. Confirm that the native DDS field services and the LingTu application were
   installed according to the deployment guide.
2. Confirm the responsible operator, clear operating area, stop mechanism,
   recovery plan, and the absence of a competing command source.
3. Verify that the selected map and localization source are correct for the
   robot and test area.
4. Run no-motion checks before any route or motion procedure.

Start with the robot-side status and diagnostics:

```bash
bash scripts/lingtu status
bash scripts/lingtu doctor
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
```

Healthy status is necessary but insufficient. For saved-map work, confirm map
artifacts and route safety without sending a goal:

```bash
bash scripts/lingtu saved-map-artifact-gate <map-directory> --require-occupancy
bash scripts/lingtu routecheck --map <map-name> --goal <x> <y> <yaw>
```

`routecheck` is explicitly non-motion and refuses to run with an active
command source. It records the route-preview and safety result; a feasible
preview is a prerequisite, not an authorization, for motion.

### Mapping workflow

The following commands change robot/session state and must be performed by an
authorized operator. They do not request autonomous navigation by themselves:

```bash
bash scripts/lingtu map start
bash scripts/lingtu map save <map-name>
bash scripts/lingtu map check <map-name> --goal <x> <y> <yaw>
bash scripts/lingtu map end
```

A saved map is a package. The map workflow produces and validates artifacts
such as the optimized `map.pcd`, metadata, and planner artifacts. Use the
[map service contract](./architecture/MAP_SERVICE_CONTRACT.md) for provenance
and artifact rules; do not copy one file out of a package and call it a
navigation map.

### Navigation session preflight

Prepare a localization/navigation session and then repeat the no-motion route
preview:

```bash
bash scripts/lingtu nav start <map-name> --initial-pose <x> <y> <yaw>
bash scripts/lingtu routecheck --map <map-name> --goal <x> <y> <yaw>
bash scripts/lingtu field-check <map-directory> --acceptance-mode field
```

`nav start` is a state-changing operation: it loads the map and initializes or
checks relocalization, but it does not submit a navigation goal itself. Treat
failed relocalization, invalid `map->odom`, missing map artifacts, a safety
blocker, or a failed route preview as a stop condition.

Actual robot motion is deliberately not a Quick Start step. The robot-side
motion-smoke workflow requires an explicit `--allow-motion` flag and must be
performed only under the controlled conditions in the
[robot operations CLI](./04-deployment/lingtu_cli.md) and the applicable
[testing/field gate](./07-testing/README.md).

## Common safe overrides

Use overrides to make an experiment explicit. Re-run `show-config` or
`runtime-spec` after changing a boundary-affecting option.

| Option | Use | Boundary |
| --- | --- | --- |
| `--llm mock` | Remove cloud-LLM dependency for local/simulation work. | Does not validate a production LLM. |
| `--no-gateway` | Omit the HTTP/Gateway stack in a local framework check. | No REST, MCP, teleop, or web status surface. |
| `--no-semantic` | Run a geometric-only graph where the profile supports it. | Does not validate semantic navigation. |
| `--planner pct` | Select the explicit legacy/manual planner experiment. | Not the default product planner. |
| `--endpoint <name>` | Choose a runtime connection layer for a product task. | Must be reviewed with `runtime-spec` or `switch-plan`; it can change the command sink. |
| `--rerun` | Enable optional visualization. | Adds visualization only; it is not a safety or readiness check. |

The product default global planner is OctoPlanner3D. PCT remains an explicit
compatibility/experiment selection; do not silently treat a fallback as product
planner validation.

`direct` is a lightweight/direct compatibility planner used by profiles such
as `lite`, `teleop`, and `map` where map-backed global planning is not the
primary task. It is not evidence that the OctoPlanner3D saved-map navigation
path is healthy.

## Optional interfaces

When the active profile includes Gateway, the default ports are:

| Port | Surface | Use |
| --- | --- | --- |
| 5050 | Gateway REST, SSE, and WebSocket | Status, maps, goals, teleop, media |
| 8090 | MCP JSON-RPC | Auto-discovered Module `@skill` tools |
| 9090 | Rerun web UI | Only when started with `--rerun` |

A reachable port is not a readiness result. Check the active profile,
localization, map, route preview, and safety state before using an interface
that can change robot state. See [Reference](./08-reference/README.md) for REST,
MCP, and CLI details.

## Troubleshooting first responses

| Symptom | First response | Do not do |
| --- | --- | --- |
| `uv --locked` reports lock drift | Restore or explicitly reconcile the intended dependency change. | Run the project unlocked. |
| A local profile cannot bind Gateway port | Re-run the framework path with `--no-gateway`, then identify the local port owner. | Kill robot or field services from a workstation tutorial. |
| A selected backend is unavailable | Install only the documented extra or native prerequisite for that backend. | Add an undeclared dependency ad hoc. |
| `No active map` or map artifact validation fails | Stop navigation preparation and repair/validate the saved map package. | Send a goal or substitute a raw point cloud. |
| Localization is degraded or lost | Treat it as a field stop condition and use the deployment recovery runbook. | Continue a navigation session because a process is still alive. |
| A ROS command is missing | Confirm whether the task is an explicit compatibility check. | Install or source ROS 2 as a normal native-product requirement. |

## Next steps

- [Get Started](./01-getting-started/README.md) — staged onboarding and clear
  local/simulation/field boundaries.
- [Build Guide](./01-getting-started/BUILD_GUIDE.md) — optional Python extras,
  native artifacts, and verification.
- [Task guides](./05-guides/README.md) — outcome-oriented mapping, navigation,
  semantic, and exploration references.
- [Operations](./06-operations/README.md) — deployed-system observation and
  recovery.
- [Reference](./08-reference/README.md) — CLI, REST, MCP, configuration, and
  architecture references.
