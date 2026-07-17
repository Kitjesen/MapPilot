# Troubleshooting LingTu

Use this guide to diagnose a development host, simulation, or deployed field
runtime without confusing the native product path with ROS 2 compatibility
tools. Start with observation, isolate the failed boundary, then take the
narrowest recovery action that is authorized for the robot state.

> **Status:** Current native-first diagnostic guide<br>
> **Audience:** Developers, field integrators, and on-call maintainers<br>
> **Runs on:** Local development hosts, simulations, and deployed robots where
> the referenced service/CLI is installed

## Safety and scope

This page distinguishes three kinds of actions:

| Label | Meaning | Examples |
| --- | --- | --- |
| **Read-only** | Observes state without changing the session, goal, or command output. | `status`, `health`, `doctor --non-motion`, `dataflow`, logs, artifact gates, plan preview. |
| **Recovery** | Changes process/session state but is not itself a motion command. | Restarting the native localization service, relocalizing a saved map, starting a navigation session. |
| **Motion-capable** | May send a goal or allow a velocity-producing chain. | Submitting a navigation goal, teleoperation, explicit motion smoke tests. |

Run read-only checks first. Do not restart a field service, start a session,
send a goal, or publish a command merely to make a dashboard look healthy.
Those actions need the operating authority and safety conditions appropriate to
the environment.

Reusable documentation must use a configured target alias or a placeholder;
it must not embed a robot address. The commands below are intended to run from
the LingTu checkout on the relevant machine. On a deployed robot, begin with:

```bash
bash scripts/lingtu status
```

## Know which runtime you are diagnosing

The normal field product runtime is native DDS plus the Module graph:

```text
native LiDAR/IMU service
  -> native SLAM/localization service
  -> maps, mission, Gateway, MCP, and status Modules
  -> native navigation/traversability endpoint
  -> single field command owner
```

The usual native service set, subject to the selected profile, is:

```text
lingtu-livox-dds
lingtu-slam-dds
lingtu-traversability-dds
lingtu-nav-dds
lingtu
```

`lingtu` hosts the Python application/Gateway/API surface; it is not a
replacement for the native sensor, SLAM, or navigation processes. The selected
field endpoint owns final navigation command publication. Do not add a Python,
ROS, or ad-hoc DDS command writer while investigating a field fault.

ROS 2 services and topic tools are a different, explicit compatibility path.
They are covered in [ROS 2 compatibility](#ros-2-compatibility-only) below.

For service ownership and exact deployment layout, read
[Deployment](../04-deployment/README.md) and the
[native runtime contract](../architecture/NATIVE_RUNTIME.md).

## Diagnostic ladder

Use this order before changing state. It makes the result useful to the next
person who reads the incident.

```text
1. Identify the expected profile/endpoint contract.
2. Observe session, health, native services, and data flow.
3. Isolate the first missing or stale boundary.
4. Validate map/localization/planner readiness without motion.
5. Apply the narrowest authorized recovery.
6. Re-run the same observation and record the before/after evidence.
```

### First five commands (read-only)

```bash
# One-screen session, localization, mission, control, map, and recent-log view.
bash scripts/lingtu status

# Gateway's detailed health summary.
bash scripts/lingtu health

# Native services plus Gateway/dataflow checks; no ROS graph required.
bash scripts/lingtu doctor --non-motion --json --strict

# Sample readiness without commanding motion.
bash scripts/lingtu soak --duration 120 --interval 2 --json --strict

# Review recent errors and then collect a bounded log window if needed.
bash scripts/lingtu log error
bash scripts/lingtu log all
```

`doctor --non-motion` checks the native/Gateway runtime without using ROS topic
inspection. `soak` adds a time window for localization freshness, map-cloud
stability, command-source idleness, and stationary odometry displacement. A
passing `soak` is readiness evidence; it is not a route-safety proof or motion
authorization.

### Confirm the expected contract before changing anything

If the observed system does not match the intended product/profile/endpoint,
diagnose configuration first. Do not fix a profile mismatch by starting
unrelated services.

```bash
# Replace nav and thunder_field with the profile/endpoint under investigation.
bash scripts/lingtu runtime-spec nav --endpoint thunder_field
bash scripts/lingtu runtime-contract --json
bash scripts/lingtu runtime-audit --json
```

The runtime specification answers which data source, frame contract, endpoint,
and ownership model the profile resolved to. The audit checks that the source
and documentation contracts agree. For a local developer catalog, use:

```bash
uv run --locked python lingtu.py --list
uv run --locked python lingtu.py --list --all
```

## Symptom: native runtime is not ready

### Signs

- `lingtu status` reports no session, missing localization, failed module, or
  unexpected product mode.
- `doctor --non-motion --strict` returns blockers.
- `lingtu health` cannot return a healthy Gateway summary.
- A native service is inactive, repeatedly restarting, or missing its binary.

### Diagnose

```bash
bash scripts/lingtu svc status
bash scripts/lingtu doctor --non-motion --json --strict
bash scripts/lingtu health
journalctl -u lingtu -n 100 --no-pager
journalctl -u lingtu-slam-dds -n 100 --no-pager
journalctl -u lingtu-nav-dds -n 100 --no-pager
```

Read the first failing boundary rather than the last visible symptom. Examples:

| First failure | Likely owner | Next check |
| --- | --- | --- |
| Gateway unavailable | `lingtu` process or its configuration | `journalctl -u lingtu`, then `lingtu health`. |
| SLAM status missing/stale | Native LiDAR/SLAM chain | `lingtu-livox-dds`, `lingtu-slam-dds`, and localization dataflow. |
| Navigation endpoint unavailable | Native nav/traversability process or selected map/profile | Nav service logs, runtime spec, map artifact gate. |
| Module failed preflight | Missing declared artifact/configuration | Read the named Module failure reason; repair that prerequisite, not a random dependency. |

### Recover deliberately

If service restart is authorized and the fault is confined to localization,
prefer the narrow recovery:

```bash
bash scripts/lingtu svc restart localization
```

Use `svc restart all` only after evidence shows that the broader native chain
needs a cold restart. Re-run `status`, `doctor --non-motion`, and a bounded
`soak` afterward. A restarted process is not automatically navigation-ready.

## Symptom: LiDAR, IMU, or map-cloud data is missing

### Diagnose the native data path first

```bash
bash scripts/lingtu dataflow /nav/lidar_scan
bash scripts/lingtu dataflow /nav/imu
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
journalctl -u lingtu-livox-dds -n 100 --no-pager
journalctl -u lingtu-slam-dds -n 100 --no-pager
```

Interpret the first absent stage:

| Observation | Meaning | Next action |
| --- | --- | --- |
| LiDAR/IMU boundary has no samples | Sensor service, device ownership, NIC/clock, or native source issue. | Inspect `lingtu-livox-dds` logs and the deployment/device configuration. |
| Raw sensors arrive but odometry is absent | SLAM ingest, calibration, or native SLAM process issue. | Inspect `lingtu-slam-dds` logs; do not start ROS Fast-LIO as a generic fix. |
| Odometry arrives but map cloud is stale | Mapping/session mode or map-cloud producer issue. | Check session mode, `dataflow /nav/map_cloud`, and map-related blockers. |
| Gateway status is stale while native data is live | Adapter/Gateway/module boundary issue. | Inspect `lingtu` logs and the relevant ModulePort/dataflow report. |

Avoid treating a raw process as ready just because it is running. The useful
claim is that the expected typed data is fresh and has the expected frame and
source ownership.

## Symptom: localization is lost, jumps, or is not navigation-ready

### Read-only checks

```bash
bash scripts/lingtu status
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/map_cloud
bash scripts/lingtu soak --duration 120 --interval 2 --json --strict
bash scripts/lingtu log drift
```

For saved-map navigation, process liveness or a `TRACKING` label is not enough.
Confirm that localization is fresh, the active map is correct, and the
`map -> odom` relationship is valid for the active map frame. A stale or
identity-like transform that does not establish the selected map frame is a
localization blocker even if SLAM is still publishing a heartbeat.

### Separate restart from relocalization

| Operation | What it proves | When to use it |
| --- | --- | --- |
| `svc restart localization` | The native SLAM/localization service can restart and publish status. | The process/dataflow is stale or failed. |
| `nav relocalize <map> X Y YAW` | A saved map accepts an operator-provided initial pose. | A known approximate pose is available. |
| `nav global-relocalize <map>` | The backend can seek a saved-map alignment without a provided seed. | The relevant saved-map workflow authorizes it. |
| `slamcompare --map <map>` | Stationary behavior meets the configured comparison/drift gate. | A no-motion localization acceptance check is required. |

Relocalization and session start change runtime state. Use them only under the
correct field procedure; after either one, re-check status and readiness before
sending a goal.

```bash
# Recovery actions; do not run these merely to inspect state.
bash scripts/lingtu svc restart localization
bash scripts/lingtu nav relocalize <map> <x> <y> <yaw>
bash scripts/lingtu nav global-relocalize <map>
```

If stationary drift repeats after a clean restart, preserve the log/dataflow
evidence and inspect calibration, timestamp alignment, sensor visibility, map
quality, and saved-map optimization before relaxing any safety threshold.

## Symptom: map is missing, invalid, or unsuitable for planning

A saved map is a package, not just `map.pcd`. The exact artifact set depends on
the selected planner, but map provenance and matching planner artifacts are
part of the contract.

### Read-only artifact and plan checks

```bash
# Use the full saved-map directory, not only the point-cloud file.
bash scripts/lingtu saved-map-artifact-gate <map-dir> --require-occupancy --json

# No goal, stop, or velocity command is published by this preview.
bash scripts/lingtu plan-preview --internal-only --strict

# No-motion route preflight against a selected map and candidate goal.
bash scripts/lingtu routecheck --map <map> --goal <x> <y> <yaw>
```

For OctoPlanner3D navigation, expect a valid map package with `map.pcd`,
`metadata.json`, and `octomap.ot`; `occupancy.npz` is required by workflows
that explicitly request occupancy validation. `tomogram.pickle` is a legacy/PCT
artifact unless a profile explicitly selects PCT.

| Failure | What to inspect | Do not do |
| --- | --- | --- |
| No active map | Session/product configuration and map list. | Invent a file path or point the planner at an arbitrary PCD. |
| Artifact gate fails | `metadata.json`, source/profile/frame provenance, checksums, map build output. | Ignore mismatched source/frame metadata. |
| Plan preview returns no route | Map bounds, goal frame, `octomap.ot`, planner diagnostics, safety-policy rejection. | Lower obstacle/safety thresholds before artifacts and localization are healthy. |
| Map is smeared or sparse | Raw map, patches, poses, map optimization report, calibration. | Treat a rewritten/rebuilt map as proof of field route safety without a new preview. |

Follow the [map service contract](../architecture/MAP_SERVICE_CONTRACT.md) for
artifact ownership and [Parameter tuning](./PARAMETER_TUNING.md) only after the
map and localization gates are sound.

## Symptom: a navigation goal is rejected or no path is produced

Do not debug this by publishing a raw command. A rejected goal is useful
evidence that one of the required readiness gates is working.

```bash
bash scripts/lingtu status
bash scripts/lingtu health
bash scripts/lingtu plan-preview --internal-only --strict
bash scripts/lingtu routecheck --map <map> --goal <x> <y> <yaw>
bash scripts/lingtu log error
```

Work through this order:

1. Confirm the intended profile/endpoint and active map.
2. Confirm fresh localization and a valid map-frame relationship.
3. Pass the saved-map artifact gate.
4. Pass a no-motion plan preview/route preflight.
5. Read the returned blockers, rejected-plan diagnostics, and selected planner.
6. Only then use the approved goal/session workflow.

Common causes include an inactive map, goal outside map bounds, failed map
provenance gate, stale cost/traversability input, localization loss, or a
safety-policy rejection. The [global planning contract](../architecture/GLOBAL_PLANNING_CONTRACT.md)
defines what a planner request/result may claim; a global planner never owns
the final velocity command.

## Symptom: robot does not move, moves unexpectedly, or teleop is ineffective

This is a command-ownership and safety problem until proven otherwise. Do not
try to repair it by manually publishing `cmd_vel`, adding a second writer, or
starting legacy command services next to the native endpoint.

### Observe first

```bash
bash scripts/lingtu status
bash scripts/lingtu health
bash scripts/lingtu dataflow /nav/odometry
bash scripts/lingtu dataflow /nav/traversability
journalctl -u lingtu-nav-dds -n 100 --no-pager
```

Check these conditions:

| Question | Expected reasoning |
| --- | --- |
| Is the product/session mode correct? | Teleop, assisted teleop, navigation, and exploration have different allowable command paths. |
| Is safety blocking motion? | A safety stop, stale localization/map, or obstacle gate should result in zero command. Diagnose the blocker before changing policy. |
| Is there one final field writer? | The selected native endpoint owns final field command output. A duplicate writer is a fault, not redundancy. |
| Is the active source valid? | Teleop, visual servo, recovery, and path following use a priority/timeout arbitration path in Module-owned profiles. |
| Is operator takeover configured? | Assisted teleop and autonomy handoff are profile/endpoint features; they are not a general bypass of safety. |

`teleop` is not the same as `teleop_avoid`. The assisted mode has its own
native local-planning/safety branch, and a blocked scene should still command
zero. Releasing a deadman/teleop lease should not be "fixed" by reusing an old
goal or command.

For command ownership and priority details, read the
[product mode runtime contract](../architecture/PRODUCT_MODE_RUNTIME_CONTRACT.md)
and [navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md).

## Symptom: Gateway, REST, WebSocket, or MCP is unreachable

The Gateway runs inside `lingtu`; there is no separate product Gateway service
to restart. Start with the application and local listener state:

```bash
bash scripts/lingtu health
bash scripts/lingtu svc status
journalctl -u lingtu -n 100 --no-pager
ss -tnlp | grep -E '5050|8090'
```

| Observation | Likely boundary | Next step |
| --- | --- | --- |
| No listener for the expected API/MCP port | `lingtu` failed to start or port ownership conflict. | Read `lingtu` logs and resolve the first startup error. |
| Listener exists but health fails | Gateway/module readiness or configuration. | Use `lingtu health`, `doctor --non-motion`, and the failed Module reason. |
| Gateway works but one MCP tool is absent | Active profile did not load its owning Module or its method is not an intended `@skill`. | Check profile/module health and the MCP tool contract; do not expose internal unsafe methods as a quick fix. |
| API accepts a request but behavior is blocked | Downstream goal/map/localization/safety gate. | Treat the response as a diagnostic, then follow the relevant symptom section. |

See [API reference](../08-reference/README.md) for interface inventory. A
network listener being alive does not imply that navigation is ready.

## Symptom: profile, endpoint, frame, or transport mismatch

Symptoms include a correct-looking process graph paired with the wrong map
frame, an incompatible runtime endpoint, unexpected local-vs-DDS behavior, or
a simulation adapter active in a field profile.

```bash
bash scripts/lingtu runtime-spec <profile> --endpoint <endpoint>
bash scripts/lingtu runtime-contract --json
bash scripts/lingtu runtime-audit --json
bash scripts/lingtu dataflow <topic-or-channel>
```

Fix the source of truth in this order:

1. canonical product profile;
2. robot preset and runtime defaults;
3. selected endpoint and its explicit adapter configuration;
4. explicit user override;
5. product Blueprint/wire or native endpoint contract.

Do not work around a mismatch by putting a transport-specific conditional into
navigation, perception, decision, or gateway business logic. Route contracts
describe external topic/schema ownership; normal Module wires stay local unless
`routed_delivery(...)` was deliberately selected.

## Symptom: build, import, or native-extension failure

### Start with the locked local environment

```bash
uv sync --locked
uv run --locked python lingtu.py --list
uv run --locked python -m pytest src/runtime/tests/test_runtime.py src/runtime/tests/test_registry.py -q
```

If a profile reports that the native navigation kernel or OctoPlanner3D runtime
is required, treat that as a failed prerequisite, not an invitation to silently
use a slower or incompatible Python fallback. Use the build guide and the exact
build hint emitted by the profile preflight:

```bash
bash scripts/build/build_nav_kernel.sh
bash scripts/build/build_nav_endpoint.sh
```

Then run the focused test or CMake target for the changed component. Do not
solve a native product build failure by installing a broad ROS desktop stack or
random global Python packages. ROS and `cyclonedds-python` are optional
compatibility/diagnostic surfaces in parts of the repository, not a general
main-path fix.

See [Build guide](../01-getting-started/BUILD_GUIDE.md) for supported host and
native build prerequisites.

## ROS 2 compatibility only

Use this section only when the task explicitly targets a legacy ROS service,
adapter, replay/simulator bridge, or comparison gate. It is not the first
diagnostic branch for the normal native field product runtime.

```bash
# Explicit compatibility inspection only.
bash scripts/lingtu doctor --ros2
bash scripts/lingtu svc status-legacy
```

Rules:

- A missing `ros2` command is normal for a native-only product installation.
- Do not source a ROS overlay or enable legacy services as a general recovery
  step.
- Do not run legacy LiDAR/SLAM/command owners beside the native DDS chain
  unless the test plan explicitly requires the comparison.
- Keep ROS-facing code in explicit `*/adapters/ros2/`, simulator bridges, or
  legacy deployment paths; it must not become an import requirement of product
  Modules.

The supported install and service boundaries are documented under
[Legacy ROS2 compatibility](../04-deployment/README.md#legacy-ros2-compatibility).

## Collect a useful incident evidence pack

When the issue cannot be resolved immediately, record enough information to
reproduce the failed contract without exposing credentials or a deployment
address.

```bash
# Read-only runtime summary and dataflow evidence.
bash scripts/lingtu status
bash scripts/lingtu doctor --non-motion --json --strict
bash scripts/lingtu runtime-contract --json
bash scripts/lingtu runtime-audit --json
bash scripts/lingtu evidence --duration 20 --json-out runtime-evidence.json
bash scripts/lingtu log all
```

Include in an incident report:

- selected profile, endpoint, product session, and active map name;
- exact command and timestamp of the first failure;
- the first blocker/error, not only the last cascading error;
- native service state and relevant bounded journal excerpt;
- dataflow/health/doctor output, frame or artifact gate result;
- whether any recovery action was taken and its before/after evidence;
- whether the observation is local, simulation, native endpoint, or real
  hardware evidence.

Redact tokens, credentials, private network details, and other sensitive
configuration before sharing logs.

## Escalation and related documents

| Need | Read |
| --- | --- |
| Architecture/ownership vocabulary | [Core concepts](../02-concepts/README.md) |
| Development boundaries and verification | [Develop LingTu](./README.md) |
| Native services and field operations | [Deployment](../04-deployment/README.md) |
| Robot-side command reference | [Operations CLI](../04-deployment/lingtu_cli.md) |
| Map package/artifact behavior | [Map service contract](../architecture/MAP_SERVICE_CONTRACT.md) |
| Planner/safety behavior | [Navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md) |
| Validation evidence and field gates | [Testing and validation](../07-testing/README.md) |
