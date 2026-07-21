# Get Started with LingTu

Use the smallest environment that can prove the result you need. Start with a
local Module graph, move to a simulation only when the simulation boundary is
part of the claim, and use a field robot only after the required deployment and
readiness gates pass.

> **Status:** Current<br>
> **Audience:** New developers, integrators, and supervised field operators<br>
> **Runs on:** Local development hosts, simulation, and supported field robots

## Safety and runtime boundary

LingTu is **Module-First**: a `Module` is the runtime unit and a `Blueprint`
is the orchestration unit. Typed ports and explicit wires carry product data;
native DDS is used at deliberate process boundaries. ROS 2 is a compatibility
surface, not the normal product runtime or operator entry point.

On the current Thunder field endpoint, the final hardware command path is
native-owned: `lingtu-nav-dds` publishes `/nav/cmd_vel`, the single
`lingtu-driver` consumes it, and `lingtu-driver` forwards checked commands to a
remote Brainstem controller configured by `/opt/lingtu/config/brainstem.env`.
Do not add another Python or ROS velocity writer to that field path.

Treat these environments as different evidence levels:

| Environment | Safe first use | What a successful run demonstrates | What it does not demonstrate |
| --- | --- | --- | --- |
| Local | `stub` and static CLI inspection | Python environment, profile resolution, Module graph behavior | Simulation, native services, or robot motion |
| Simulation | `sim` or `sim_nav` | The selected simulated driver, dataflow, and configured scenario | Real LiDAR/IMU timing, calibration, gait, or field safety |
| Field robot | `scripts/lingtu` preflight and supervised sessions | Only the evidence collected on that robot and map | Future runs, other robots, or unattended motion |

Do not add a field endpoint to a local or simulation tutorial command. A
product task plus an endpoint changes the source and sink of sensor and command
data; inspect that boundary before launch with `runtime-spec` or
`switch-plan`. A goal is not a motor command, but it can ultimately produce
motion through planning, safety, velocity arbitration, and the driver boundary.

## Choose a starting path

| I need to... | Start with | Hardware motion | Continue with |
| --- | --- | --- | --- |
| Verify the checkout and profile catalog | [Locked environment check](#prepare-the-locked-environment) | No | [First local Module graph](#run-the-first-local-module-graph) |
| Work on framework or Module wiring | `stub` | No | [Core concepts](../02-concepts/README.md) |
| Work on semantic logic with a deterministic LLM | `dev --llm mock` | No | [Development guide](../03-development/README.md) |
| Exercise navigation logic without a physical robot | `sim_nav` | No physical motion | [Simulation path](#move-to-simulation) |
| Run an in-process MuJoCo stack | `sim` | Simulated motion only | [`sim/README.md`](../../sim/README.md) |
| Build or use a saved map on a robot | Robot-side operations CLI | Potentially, under supervision | [Field path](#enter-the-field-path-only-after-preflight) |

The complete profile catalog is code-owned in
[`src/runtime/profiles/catalog/`](../../src/runtime/profiles/catalog/). The CLI
can list the current checkout's product profiles with `--list`, and the
development, simulation, and compatibility profiles with `--list --all`.

## Prepare the locked environment

Run these commands from the repository root. The examples deliberately use the
locked environment so an accidental dependency drift fails before LingTu starts.

```bash
uv sync --locked --extra dev
uv run --locked python lingtu.py --list --all
uv run --locked python lingtu.py show-config stub --json
uv run --locked python lingtu.py runtime-audit
```

Expected checks:

- `uv sync --locked` completes without rewriting `uv.lock`.
- `--list --all` prints both product and advanced/development profiles.
- `show-config stub --json` resolves a configuration without starting modules.
- `runtime-audit` checks the repository's runtime manifest, YAML, profile, and
  collector contracts without authorizing a motion session.

`uv run --locked python lingtu.py ...` is the canonical form for every local
LingTu CLI invocation in this documentation. It chooses the repository's
locked environment and executes the checked-in entry point, rather than a
globally installed console script.

If any `--locked` command says that the lockfile needs an update, stop there.
That means the checked-out dependency metadata and lockfile do not describe the
same environment. Restore the intended revision or make and review an explicit
dependency/lockfile change; do not work around the problem by running LingTu
unlocked.

## Run the first local Module graph

### Preconditions

- You are in a terminal at the repository root.
- The locked environment check above passed.
- No field endpoint, robot host override, or active robot service is being
  attached to this local exercise.

Start the framework-only profile with Gateway disabled so this first check does
not require an HTTP server or consume a local port:

```bash
uv run --locked python lingtu.py stub --no-gateway
```

When standard input is a TTY, LingTu opens its interactive REPL. Use only
inspection commands for this first run:

```text
health
connections
quit
```

Expected checks:

- The process resolves the `stub` profile and starts a Module graph using the
  stub driver; it does not connect to real hardware.
- `health` and `connections` return local runtime information.
- `quit` stops the local process cleanly.

If there is no REPL prompt, verify that the command was started from an
interactive terminal and that `--no-repl` was not supplied. A successful
`stub` run proves only the local framework path. It is not a simulation or
field readiness result.

### Optional: semantic development without a real LLM

After the stub path is understood, use the development profile to bring up the
semantic graph with the deterministic mock LLM:

```bash
uv run --locked python lingtu.py dev --llm mock --no-gateway
```

`dev` is for integration and development. It must not be used as evidence that
a hardware detector, a cloud LLM, or a field navigation chain is operational.
Install only the optional extras required by the backend you intentionally
select; see the [Build Guide](./BUILD_GUIDE.md#select-the-python-environment)
for the supported extra groups.

## Move to simulation

Simulation is still a safety boundary. Its command sink is the simulated
driver, not the physical robot, but a simulated navigation run can create
simulated motion and should be interpreted as simulation evidence only.

### Preflight

Install the MuJoCo extra alongside development tooling, then inspect the
resolved profile before running it:

```bash
uv sync --locked --extra dev --extra sim-mujoco
uv run --locked python lingtu.py runtime-spec sim --json
```

The `sim` profile is the in-process MuJoCo Module runtime. `sim_nav` is a
no-ROS navigation simulation profile that uses the local simulation graph.
Neither is a shortcut for a field profile.

### Run a simulation profile

```bash
uv run --locked python lingtu.py sim
```

For a lighter navigation-oriented check, use:

```bash
uv run --locked python lingtu.py sim_nav
```

Expected checks:

- The resolved runtime identifies a simulation driver or simulation data
  source, not a physical field command sink.
- Any `cmd_vel` produced by these profiles stays inside the selected simulated
  driver or adapter.
- A passed run supports only the named simulation scenario and gate.

Do not treat a green simulation as permission to run `map`, `nav`, `explore`,
or `tare_explore` against hardware. Read the
[simulation integration contract](../architecture/SIMULATION_INTEGRATION_CONTRACT.md)
and the [simulation guide](../../sim/README.md) before using explicit endpoints
or claiming planner, SLAM, or closed-loop behavior.

## Enter the field path only after preflight

Field operation belongs on the target Linux controller or an approved remote
operator session. Use the robot-side shell CLI, `scripts/lingtu`; it manages
the native DDS services around the Python application. Do not replace it with
ad-hoc `curl`, `systemctl`, or a sourced ROS 2 overlay.

### Required conditions before a field session

Before changing the robot's mode or allowing any movement, confirm all of the
following:

1. The target is deployed using the [Thunder deployment guide](../04-deployment/README.md).
2. A trained operator has authority, a clear test area, an accessible stop
   path, and a recovery plan.
3. There is no conflicting teleoperation, navigation, or legacy ROS 2 service
   owning the same sensor or command boundary.
4. The saved map, localization, route safety, and field-readiness evidence are
   valid for the current robot and session.

Start with read-only or no-motion checks on the robot:

```bash
bash scripts/lingtu status
bash scripts/lingtu doctor
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/cmd_vel
bash scripts/lingtu routecheck --map <map-name> --goal <x> <y> <yaw>
```

`routecheck` is a non-motion route preflight: it requires no active command
source and records planner and safety evidence. It must pass before a
supervised motion procedure is considered. Use
[`lingtu_cli.md`](../04-deployment/lingtu_cli.md) for the current field
command contract and [Testing and validation](../07-testing/README.md) for the
applicable evidence gate.

### Mapping and saved-map preflight

These commands change the robot's product session and map state. They do not
authorize autonomous navigation; any manual robot movement during mapping still
needs the approved operator controls and safety boundary.

```bash
bash scripts/lingtu map start
bash scripts/lingtu map save <map-name>
bash scripts/lingtu map check <map-name> --goal <x> <y> <yaw>
bash scripts/lingtu map end
```

A navigation-ready map is a package, not just a point cloud. At minimum,
verify the current map contract's required artifacts and provenance; for
OctoPlanner3D navigation this normally includes the optimized `map.pcd`, map
metadata, and the required OctoMap artifact. See the
[map service contract](../architecture/MAP_SERVICE_CONTRACT.md).

### Navigation session preflight

After map validation, a navigation session can be prepared without sending a
goal:

```bash
bash scripts/lingtu nav start <map-name> --initial-pose <x> <y> <yaw>
bash scripts/lingtu routecheck --map <map-name> --goal <x> <y> <yaw>
```

`nav start` changes SLAM/session state and performs or requests saved-map
relocalization; it is therefore not a read-only command. It does not itself
submit a navigation goal. Confirm successful relocalization, a valid
`map->odom` relationship, valid map artifacts, and a feasible no-motion route
preview before following the explicitly supervised motion procedure in the
[robot operations CLI](../04-deployment/lingtu_cli.md). The motion-smoke
workflow requires an explicit `--allow-motion` flag by design; it is
intentionally outside this getting-started path.

## Where to go next

- Learn the boundaries behind these commands in [Core concepts](../02-concepts/README.md).
- Set up optional dependencies or native artifacts in the [Build Guide](./BUILD_GUIDE.md).
- Use the full [Quick Start](../QUICKSTART.md) for profile, lifecycle, and
  troubleshooting details.
- Follow [Task guides](../05-guides/README.md) for mapping, navigation,
  semantic goals, and exploration.
