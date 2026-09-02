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
| **Recovery** | Changes process/session state but is not itself a motion command. | Restarting one RunPlan logical process, relocalizing a saved map, starting a navigation session. |
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
lt-lidar
lt-slam
lt-terrain
lt-nav
lingtu
```

`lingtu` hosts the Python application/Gateway/API surface; it is not a
replacement for the native sensor, SLAM, or navigation processes. The selected
field endpoint owns final navigation command publication. Do not add a Python,
ROS, or ad-hoc DDS command writer while investigating a field fault.

ROS 2 services and topic tools are a different, explicit compatibility path.
Use the [ROS role replacement map](../architecture/ROS_ROLE_REPLACEMENT_MAP.md)
to identify its supported boundary.

For service ownership and exact deployment layout, read
[Deployment](../04-deployment/README.md) and the
[native runtime contract](../architecture/NATIVE_RUNTIME.md).

## Diagnostic ladder

Use this order before changing state. It makes the result useful to the next
person who reads the incident.

```text
1. Identify the expected Product/env contract.
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
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"

# Native services plus Gateway/dataflow checks; no ROS graph required.
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict

# Sample readiness without commanding motion.
PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict

# Review recent errors and then collect a bounded log window if needed.
journalctl -u 'lt-*' -p err -n 100 --no-pager
journalctl -u 'lt-*' -n 200 --no-pager
```

`doctor --non-motion` checks the native/Gateway runtime without using ROS topic
inspection. `soak` adds a time window for localization freshness, map-cloud
stability, command-source idleness, and stationary odometry displacement. A
passing `soak` is readiness evidence; it is not a route-safety proof or motion
authorization.

### Confirm the expected contract before changing anything

If the observed system does not match the intended Product/env,
diagnose configuration first. Do not fix a Product mismatch by starting
unrelated services.

```bash
# Replace MAP_NAME with the saved map under investigation.
python -m lingtu.control switch nav --robot unitree/go2 --env real --map MAP_NAME --dry-run --json
python -m pytest tests/runtime/test_runtime_graph_contract.py -q
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
```

The ProductControl dry run answers which RunPlan, env, process roles,
map staging, and ownership model would be applied. The validators check the
runtime graph, package boundaries, and topic declarations. For a local developer catalog, use:

```bash
uv run --locked python -m lingtu.control --help
uv run --locked python -m lingtu.control --help
```

## Symptom: native runtime is not ready

### Signs

- `lingtu status` reports no session, missing localization, failed module, or
  unexpected product mode.
- `doctor --non-motion --strict` returns blockers.
- `curl -fsS http://localhost:5050/api/v1/health` cannot return a healthy
  Gateway summary.
- A native service is inactive, repeatedly restarting, or missing its binary.

### Diagnose

```bash
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
journalctl -u lingtu -n 100 --no-pager
journalctl -u lt-slam -n 100 --no-pager
journalctl -u lt-nav -n 100 --no-pager
```

Read the first failing boundary rather than the last visible symptom. Examples:

| First failure | Likely owner | Next check |
| --- | --- | --- |
| Gateway unavailable | `lt-host` process or its configuration | `journalctl -u lt-host.service`, then `curl -fsS http://localhost:5050/api/v1/health`. |
| SLAM status missing/stale | Native LiDAR/SLAM chain | `lt-lidar`, `lt-slam`, and localization dataflow. |
| Navigation endpoint unavailable | Native nav/traversability process or selected map/profile | Nav service logs, runtime spec, map artifact gate. |
| Module failed preflight | Missing declared artifact/configuration | Read the named Module failure reason; repair that prerequisite, not a random dependency. |

### Recover deliberately

After preserving status and logs, switch the active Product again through
ProductControl:

```bash
bash scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]
```

Re-run `status`, `doctor --non-motion`, and a bounded `soak` afterward. A
successful switch is not automatically navigation-ready.

## Symptom: LiDAR, IMU, or map-cloud data is missing

### Diagnose the native data path first

```bash
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/lidar_scan"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/imu"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
journalctl -u lt-lidar -n 100 --no-pager
journalctl -u lt-slam -n 100 --no-pager
```

Interpret the first absent stage:

| Observation | Meaning | Next action |
| --- | --- | --- |
| LiDAR/IMU boundary has no samples | Sensor service, device ownership, NIC/clock, or native source issue. | Inspect `lt-lidar` logs and the deployment/device configuration. |
| Raw sensors arrive but odometry is absent | SLAM ingest, calibration, or native SLAM process issue. | Inspect `lt-slam` logs; do not start ROS Fast-LIO as a generic fix. |
| Odometry arrives but map cloud is stale | Mapping/session mode or map-cloud producer issue. | Check session mode, `dataflow /nav/map_cloud`, and map-related blockers. |
| Gateway status is stale while native data is live | Adapter/Gateway/module boundary issue. | Inspect `lingtu` logs and the relevant ModulePort/dataflow report. |

Avoid treating a raw process as ready just because it is running. The useful
claim is that the expected typed data is fresh and has the expected frame and
source ownership.

## Symptom: localization is lost, jumps, or is not navigation-ready

### Read-only checks

```bash
bash scripts/lingtu status
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/map_cloud"
PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict
journalctl -u 'lt-*' -n 200 --no-pager | grep -i drift
```

For saved-map navigation, process liveness or a `TRACKING` label is not enough.
Confirm that localization is fresh, the active map is correct, and the
`map -> odom` relationship is valid for the active map frame. A stale or
identity-like transform that does not establish the selected map frame is a
localization blocker even if SLAM is still publishing a heartbeat.

### Distinguish a plain Product switch from a nav switch with relocalization

| Operation | What it proves | When to use it |
| --- | --- | --- |
| `switch <product> [--map <name>]` | The Product can start and pass declared readiness. | The active Product must be recovered after preserving evidence. |
| `nav relocalize <map> X Y YAW` | A full field nav Product cold switch succeeds and the loaded map accepts the seed. | A known approximate pose is available. |
| `nav global-relocalize <map>` | A full field nav Product cold switch succeeds and BBS3D + MapIcp aligns the loaded map. | No useful seed is available. |
| `doctor --non-motion --strict` | The active RunPlan and declared services pass stationary readiness checks. | A no-motion runtime acceptance check is required. |

These CLI relocalization commands change Product/session state. Gateway
relocalization endpoints instead retry against the already loaded Product map;
they do not start `slamd` or select another map. After either path, re-check
status and readiness before sending a goal.

```bash
# Recovery actions; do not run these merely to inspect state.
bash scripts/lingtu --robot <vendor/model> --env real switch <product> [--map <name>]
bash scripts/lingtu switch nav --map <map> --initial-pose <x> <y> <yaw>
bash scripts/lingtu switch nav --map <map> --relocalize
```

If stationary drift repeats after a clean Product switch, preserve the log/dataflow
evidence and inspect calibration, timestamp alignment, sensor visibility, map
quality, and saved-map optimization before relaxing any safety threshold.

## Symptom: map is missing, invalid, or unsuitable for planning

A saved map is a package, not just `map.pcd`. The exact artifact set depends on
the selected planner, but map provenance and matching planner artifacts are
part of the contract.

### Artifact and integrated field checks

```bash
# Use the saved map ID known by mapd.
PYTHONPATH=src python -m diagnostics.field.map_artifacts <map-id> --require-occupancy --json

# Integrated field evidence; no motion unless --allow-motion is supplied.
PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" --map <map> --goal <x> <y> <yaw>
```

For OctoPlanner3D navigation, expect a valid map package with `map.pcd`,
`metadata.json`, and `octomap.ot`; `occupancy.npz` is required by FAR and
workflows that explicitly request occupancy validation.

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
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --strict
PYTHONPATH=src python -m diagnostics.field.system_acceptance --maps-root "$LINGTU_MAPS_ROOT" --map <map> --goal <x> <y> <yaw>
journalctl -u 'lt-*' -p err -n 100 --no-pager
```

Work through this order:

1. Confirm the intended Product/env and active map.
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
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/odometry"
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=/nav/traversability"
journalctl -u lt-nav -n 100 --no-pager
```

Check these conditions:

| Question | Expected reasoning |
| --- | --- |
| Is the product/session mode correct? | Teleop, assisted teleop, navigation, and exploration have different allowable command paths. |
| Is safety blocking motion? | A safety stop, stale localization/map, or obstacle gate should result in zero command. Diagnose the blocker before changing policy. |
| Is there one final field writer? | The selected native endpoint owns final field command output. A duplicate writer is a fault, not redundancy. |
| Is the active source valid? | Teleop, visual servo, recovery, and path following use a priority/timeout arbitration path in Module-owned profiles. |
| Is operator takeover configured? | Assisted teleop and autonomy handoff are Product features; they are not a general bypass of safety. |

`teleop` is not the same as `teleop_avoid`. The assisted mode has its own
native local-planning/safety branch, and a blocked scene should still command
zero. Releasing a deadman/teleop lease should not be "fixed" by reusing an old
goal or command.

For command ownership and priority details, read the
[field Product guide](../architecture/FIELD_PRODUCTS.md)
and [navigation compute contract](../architecture/NAVIGATION_COMPUTE_CONTRACT.md).

## Symptom: Gateway, REST, WebSocket, or MCP is unreachable

The Gateway runs inside `lingtu`; there is no separate product Gateway service
to restart. Start with the application and local listener state:

```bash
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/health"
bash scripts/lingtu status
journalctl -u lingtu -n 100 --no-pager
ss -tnlp | grep -E '5050|8090'
```

| Observation | Likely boundary | Next step |
| --- | --- | --- |
| No listener for the expected API/MCP port | `lingtu` failed to start or port ownership conflict. | Read `lingtu` logs and resolve the first startup error. |
| Listener exists but health fails | Gateway/module readiness or configuration. | Use the Gateway health route, `PYTHONPATH=src python -m diagnostics.field.doctor --non-motion`, and the failed Module reason. |
| Gateway works but one MCP tool is absent | The active Product did not load its owning Module or its method is not an intended `@skill`. | Check Product/module health and the MCP tool contract; do not expose internal unsafe methods as a quick fix. |
| API accepts a request but behavior is blocked | Downstream goal/map/localization/safety gate. | Treat the response as a diagnostic, then follow the relevant symptom section. |

See [API reference](../08-reference/README.md) for interface inventory. A
network listener being alive does not imply that navigation is ready.

## Symptom: Product/env, frame, or transport mismatch

Symptoms include a correct-looking process graph paired with the wrong map
frame, an incompatible RunPlan, unexpected local-vs-DDS behavior, or
a simulation adapter active in `env=real`.

```bash
python -m lingtu.control switch <product> --robot unitree/go2 --env real --dry-run --json
python -m lingtu.control switch <product> --robot doso/thunder_v4 --env sim --dry-run --json
python -m pytest tests/runtime/test_runtime_graph_contract.py -q
python tools/validate/validate_architecture_boundaries.py
python tools/validate/validate_topics.py
curl -fsS "${LINGTU_GATEWAY_URL:?set LINGTU_GATEWAY_URL}/api/v1/runtime/dataflow/topic?topic=<topic-or-channel>"
```

Fix the source of truth in this order:

1. Product declaration in `config/runtime_graph/products/`;
2. env implementation in `config/runtime_graph/envs/`;
3. `RobotConfig`, only as internal static `env=real` robot data;
4. explicit user override;
5. Product Blueprint/wire or native endpoint contract.

Do not work around a mismatch by putting a transport-specific conditional into
navigation, perception, decision, or gateway business logic. Route contracts
describe external topic/schema ownership; normal Module wires stay local.

## Symptom: build, import, or native-extension failure

### Start with the locked local environment

```bash
uv sync --locked
uv run --locked python -m lingtu.control --help
uv run --locked python -m pytest tests/runtime/test_core.py tests/runtime/test_registry.py -q
```

If the native navigation endpoint or OctoPlanner3D runtime is missing, treat
that as a failed prerequisite. Use the build guide and the exact build hint:

```bash
bash scripts/build/build_nav_endpoint.sh
```

Then run the focused test or CMake target for the changed component. Do not
solve a native product build failure by installing a broad ROS desktop stack or
random global Python packages. The supported DDS runtime is native CycloneDDS;
there is no `cyclonedds-python` fallback to install.

See [Build guide](../01-getting-started/BUILD_GUIDE.md) for supported host and
native build prerequisites.

## Collect a useful incident evidence pack

When the issue cannot be resolved immediately, record enough information to
reproduce the failed contract without exposing credentials or a deployment
address.

```bash
# Read-only runtime summary and dataflow evidence.
bash scripts/lingtu status
PYTHONPATH=src python -m diagnostics.field.doctor --non-motion --json --strict
PYTHONPATH=src python -m diagnostics.field.soak --duration 120 --interval 2 --json --strict
PYTHONPATH=src python -m diagnostics.field.runtime_evidence --duration-sec 20 --json-out runtime-evidence.json
journalctl -u 'lt-*' -n 200 --no-pager
```

Include in an incident report:

- selected Product/env, product session, and active map name;
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
