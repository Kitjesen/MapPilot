# Simulation Integration Contract

Status: simulation/validation contract; simulation evidence is not field readiness
Audience: simulation, acceptance, adapter, Product, and RunPlan maintainers
Replaced by: not replaced

LingTu is the product system. Simulation is an `env=sim` implementation input
and validation target. It must not become a Product, public deployment
identity, safety bypass, or alternate navigation architecture.

## Runtime Model

The canonical runtime identity is:

```text
env + Product
  -> Assembly
  -> resolved RunPlan
  -> ProductControl
  -> ProductControl's internal systemd runner or declared simulation runner
  -> native processes + Host
```

Public `env` values are exactly `real` and `sim`. A supported simulator such as
MuJoCo or Gazebo is an internal backend of `env=sim`; it is not a public `env`,
Product, or endpoint selector. `ProductControl` is constructed for one fixed
env and switches one env-independent Product inside that env.

CMU Unity is not a runtime backend. Its launcher, ROS 2 topic adapter, runtime
manifest, and evidence path have been removed. Unity input is supported only by
the offline `lingtu-maps-import-unity` semantic-map import tool.

Products remain the operator modes:

- `teleop`
- `teleop_avoid`
- `map`
- `explore`
- `nav`
- `tracking`
- `inspection`

Component scripts are development entry points. They do not replace Products
and do not publish a RunPlan.

## Env Configuration

Runtime Graph source of truth:

| Source | Owns |
| --- | --- |
| `config/runtime_graph/products/*.yaml` | Env-independent Product declarations: Host graph, logical native roles, topics, and capabilities. |
| `config/runtime_graph/envs/real.yaml` | Physical robot implementation mapping, process list, and the `field_dds_v1` DDS contract. |
| `config/runtime_graph/envs/sim.yaml` | Simulation implementation backends and their acceptance runners. |
| `config/robots/<vendor>/<model>/robot.yaml` | Static physical robot/device/calibration data selected by the real robot model. |

The current `sim` env backends are internal configuration keys:

| Backend | Products | Evidence class |
| --- | --- | --- |
| `mujoco` | `teleop`, `teleop_avoid`, `map`, `explore`, `nav`, `tracking`, `inspection` | Typed-DDS MuJoCo component targets using field process boundaries. Native hot-path services are C++; the MuJoCo feeder and Product Host remain the RunPlan-declared Python processes. `explore` has separate exact `live` and `map` targets. |
| `gazebo` | `explore` | Gazebo industrial smoke and demo integration. |

Backend names may appear in env configuration, acceptance artifacts, and
operator diagnostics. They must not be described as public endpoints or
deployment identities.

The canonical backend has an acceptance target for every Product it lists and
for every declared Product variant. All current MuJoCo manifests declare
`acceptance_scope.coverage=component`; none is Product acceptance. A runner,
manifest, successful compile, or verified exact RunPlan does not upgrade that
coverage label.

## RunPlan Wiring Versus Product Acceptance

The MuJoCo catalog declares all seven Products. On a host with a complete
platform-specific native catalog, Assembly and ProductControl can resolve each
one under `env=sim, backend=mujoco` into one exact resolved RunPlan. The
`explore` variant selects its exact `live` or `map` runner/manifest pair. A host
missing a required platform implementation is rejected instead of receiving a
partial RunPlan. This is the runtime wiring boundary.

Product acceptance is a separate evidence boundary. It additionally requires
one report to prove the exact ProductControl switch, process readiness,
startup-zero acknowledgement, current-plan commit, rollback behavior, terminal
stop, and managed cleanup, plus the Product-specific lifecycle. Until a
manifest declares `coverage=product` and the report satisfies that contract,
the result remains component evidence.

The declared topology runs separate `lidar_publisher`, `imu_publisher`, and
`camera_publisher` processes from one shared publisher binary. Each process has
its own authenticated local endpoint and publishes only its assigned raw sensor
stream; SLAM remains an independent native Fast-LIO `slamd` process.
Direct-process definitions may declare private `windows` and `linux`
implementations, but RunPlan compilation selects exactly one host platform.
Only the selected artifact and command enter the RunPlan; an
unselected PE or ELF artifact is never parsed as part of the executable plan.

Windows-native parity for all MuJoCo Products is a fixed P0 target, not an
optional Linux follow-up. The current Windows catalog already declares PE
implementations for the native DDS publisher/driver bridge and the downstream
map/navigation/exploration/Host processes, but it has no built, tested, and
registered Fast-LIO `slamd.exe`. Therefore every SLAM-dependent Product is
currently rejected while compiling its Windows RunPlan, before reconciliation,
map staging, journal publication, quiesce, or child startup. Pure `teleop` does
not select `slamd` and is the only currently compilable Windows exact-chain
candidate; that is a temporary implementation state, not the accepted Windows
scope.

Linux/WSL selects the ELF catalog, but catalog compilation still requires the
generated `build/mujoco_native_dds/lingtu_mujoco_sensor_publisher` binary used
by all three sensor roles and the `lingtu_mujoco_driver_bridge` artifact; their
absence fails closed. A complete
catalog is not evidence that an exact ProductControl run completed.
Planning and control qualification uses MuJoCo truth as the simulation
localization authority. The feeder publishes that pose only as the typed SLAM
odometry prior; the selected native `slamd` remains the sole owner of canonical
SLAM odometry, registered cloud, health, and TF outputs. This isolates planning,
tracking, and locomotion results from estimator error without creating a second
output writer. It is simulation evidence, never Fast-LIO accuracy evidence or a
field-runtime fallback.

Cross-platform parity preserves one architecture:

- Windows and Linux use the same Product declarations, process roles, DDS
  topics, typed readiness, identity, lifecycle, and acceptance manifest;
- platform-private command/artifact paths may differ, but selected native
  artifacts in Windows RunPlans are PE/DLL-only and selected native artifacts
  in Linux RunPlans are ELF/SO-only; explicitly declared Python feeder/Host
  processes remain interpreter-owned;
- a Windows Product must not hide WSL/Linux subprocesses behind its RunPlan;
- neither platform may add a Python SLAM implementation or a second direct
  simulator-truth writer; estimator qualification uses its dedicated no-prior
  profile instead of the Product planning/control profile; and
- Product evidence is platform-specific. A Linux/WSL PASS cannot promote
  Windows, and a Windows PASS cannot promote Linux/WSL.

The active portability and build sequence belongs to
`docs/plans/current-roadmap.md`; this contract fixes the invariants rather than
duplicating implementation steps.

## Command Shape

Use ProductControl for Product operations:

```bash
python -m lingtu.control switch nav --robot unitree/go2 --env real --map MAP_NAME --dry-run --json
python -m lingtu.control switch nav --robot doso/thunder_v4 --env sim --map MAP_NAME --dry-run --json
bash scripts/lingtu --robot unitree/go2 --env real switch nav --map MAP_NAME
```

Use explicit component scripts for local simulation work:

```bash
uv run --locked python -m lingtu.control switch teleop --robot doso/thunder_v4 --env sim --dry-run --json
uv run --locked python sim/scripts/mujoco/native_navigation_acceptance.py --help
uv run --locked python sim/scripts/mujoco/native_control_mode_acceptance.py --help
```

Do not use endpoint flags, combined Product/env names, or current/target
endpoint switches for Product selection. Inspect Product switches through
ProductControl dry runs, RunPlan output, readiness reports, and the
appropriate acceptance artifact.

## How Simulation Enters LingTu

Simulation may enter only through declared `env=sim` backend boundaries:

1. The backend provides sensor, odometry, map, terrain, waypoint, command, or
   acceptance-runner processes declared by `config/runtime_graph/envs/sim.yaml`.
2. Assembly resolves the requested Product inside `env=sim` into one
   resolved RunPlan.
3. ProductControl applies that RunPlan or delegates to the declared
   simulation acceptance runner.
4. Host and native services consume the exact resolved contract; they must not
   independently re-resolve the Product.

Simulation command sinks must report simulation-only evidence and must never
write to physical hardware. A simulation adapter may consume `/nav/cmd_vel`
inside the simulator; it must not forward it to the field driver.

## Endpoint Meaning

Use `endpoint` only for concrete communication access points or contracts:

- HTTP: `http://<robot>:5050`, `/api/v1/runtime/dataflow`
- DDS: `field_dds_v1`, `/nav/cmd_vel`, `/slam/odometry`
- native service boundary: `lt-nav`, `mapd`, `lingtu-driver`

The field DDS contract name `field_dds_v1` is valid because it names a
typed communication contract. Do not use the former unversioned field label as
a deployment identity, public env, Product, RobotConfig selector, or CLI
endpoint selector.

## Adapter Acceptance Rules

Every simulation backend must declare:

- source ownership for world, sensors, localization, map, exploration,
  planning, path following, and command relay;
- frame mapping to `map`, `odom`, `body`, and `lidar_link`;
- topic mapping to Product topic contracts;
- command boundary and proof that `cmd_vel_sent_to_hardware=false`;
- selected saved-map artifacts and acceptance reports;
- forbidden fallbacks, especially Python or ROS fallbacks that would weaken a
  native-DDS equivalence claim.

Simulation must not:

- add simulator-specific Product architectures;
- disable native stop, stale-input, final safety, or command-authority gates;
- enable direct-goal or direct-track bypasses in Product graphs;
- relay `/nav/cmd_vel` to physical hardware;
- use cumulative debug clouds as proof of SLAM quality;
- claim field readiness from simulation evidence.

## Data Flow And Frames

The env backend boundary is also the data-flow boundary:

```text
sensor / log / simulator source
  -> env backend adapter or native service
  -> Product topic contract
  -> SLAM or relayed localization/map
  -> map layers and exploration
  -> global planner
  -> local planner and path follower
  -> final command owner
  -> simulation sink or field driver
```

The coordinate contract is:

```text
map -> odom -> body -> lidar_link
```

Reports must preserve evidence for those links when a gate requires frame
validation. `map->odom` and `body->lidar` may be static links; `odom->body`
must come from live odometry or an equivalent relayed state stream. Runtime
topic `frame_id` validation is topic-specific, not a single global assertion.

## Backend Responsibility Matrix

| Backend or env | World/sensors | SLAM/localization | Planning/control | Command sink | Allowed claim |
| --- | --- | --- | --- | --- | --- |
| `env=real` | Robot sensors through native services | Native C++ SLAM/localizer over `field_dds_v1` | Product-native `navd`/`mapd`/traversability where declared | `lingtu-driver` through the selected robot backend | Field evidence only after robot-side readiness gates. |
| `mujoco` | MuJoCo MID-360/IMU records through typed DDS | Native C++ SLAM/localization and mapd | Product-selected typed-DDS chain: native C++ hot-path services plus the declared Python feeder/Host | MuJoCo DDS command tap | Field-architecture simulation evidence; not robot readiness. |
| `gazebo` | Gazebo world, physics, rendered sensors | Gazebo-derived state/map into LingTu contracts | LingTu simulation navigation/exploration chain | Isolated Gazebo command relay | ROS/GZ integration and closed-loop smoke evidence. |

Forbidden claims are part of the contract:

- Retired simulator bridges must not be reported as ProductControl/RunPlan
  execution or Product acceptance evidence. Offline Unity semantic import does
  not execute a simulator or prove runtime behavior.
- Local MuJoCo component evidence must not be reported as native-DDS
  raw-sensor equivalence.
- `mujoco` evidence must not be reported as field readiness without a
  separate robot-side gate.
- Global planning and TARE exploration are separate capabilities. A gate may
  prove that `navd` can plan on a same-source map, but that is separate from
  proving TARE exploration quality.

## Runtime Evidence Required

A simulation run can support a Product claim only when the report proves:

- the selected manifest declares `acceptance_scope.coverage=product`;
- `env=sim`;
- the selected Product, Product session ID, and RunPlan path;
- the internal sim backend and acceptance artifact path;
- `simulation_only=true`;
- `real_robot_motion=false`;
- `cmd_vel_sent_to_hardware=false`;
- observed odometry, map, terrain, global path, local path, command, and status
  topics required by that Product;
- publisher identity and ownership match the expected backend boundary;
- map or explored area grows from the declared source when the claim requires
  mapping or exploration.

If topic samples exist but publisher identity, frame evidence, RunPlan
identity, or command-sink evidence is missing, the run is not valid LingTu
closure.
