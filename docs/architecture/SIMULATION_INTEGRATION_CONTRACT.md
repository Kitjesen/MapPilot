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
  -> fingerprinted RunPlan
  -> ProductControl
  -> ProductControl's internal systemd runner or declared simulation runner
  -> native processes + Host
```

Public `env` values are exactly `real` and `sim`. A supported simulator such as
MuJoCo or Gazebo is an internal backend of `env=sim`; it is not a public `env`,
Product, or endpoint selector. `ProductControl` is constructed for one fixed
env and switches one env-independent Product inside that env.

CMU Unity is not a current `env=sim` backend. The retained upstream baseline
launcher and pure topic adapter are external experiments; ProductControl does
not resolve them and they neither consume nor publish a RunPlan.

Products remain the operator modes:

- `teleop`
- `teleop_avoid`
- `map`
- `explore`
- `nav`
- `tracking`
- `inspection`

Local Profiles such as `stub`, `dev`, `sim`, and `sim_nav` are development
Host entry points. They do not replace field Products and do not publish a
RunPlan for field startup.

## Env Configuration

Runtime Graph source of truth:

| Source | Owns |
| --- | --- |
| `config/runtime_graph/products/*.yaml` | Env-independent Product declarations: Host graph, logical native roles, topics, and capabilities. |
| `config/runtime_graph/envs/real.yaml` | Physical Thunder implementation mapping, process list, and the `thunder_dds_v1` DDS contract. |
| `config/runtime_graph/envs/sim.yaml` | Simulation implementation backends and their acceptance runners. |
| `config/robot_config.yaml` | Static physical robot/device/calibration data referenced internally by the real env. |

The current `sim` env backends are internal configuration keys:

| Backend | Products | Evidence class |
| --- | --- | --- |
| `mujoco_native` | `nav`, `teleop_avoid` | Native-DDS-like MuJoCo acceptance using the same typed DDS contract shape as the field path. |
| `mujoco_host` | `map`, `explore` | Host simulation wiring and downstream map/navigation checks; `explore` covers live and saved-map variants. |
| `gazebo` | `explore` | Gazebo industrial smoke and demo integration. |

Backend names may appear in env configuration, acceptance artifacts, and
operator diagnostics. They must not be described as public endpoints or
deployment identities.

## Command Shape

Use ProductControl for Product operations:

```bash
python -m lingtu.control switch nav --env real --map MAP_NAME --dry-run --json
python -m lingtu.control switch nav --env sim --backend mujoco_native --map MAP_NAME --dry-run --json
bash scripts/lingtu --env real mode switch nav --map MAP_NAME
```

Use local Profiles only for local development and simulation Host runs:

```bash
uv run --locked python lingtu.py runtime-spec sim --json
uv run --locked python lingtu.py sim
uv run --locked python lingtu.py sim_nav
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
   fingerprinted RunPlan.
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
- DDS: `thunder_dds_v1`, `/nav/cmd_vel`, `/slam/odometry`
- native service boundary: `lingtu-nav-dds`, `mapd`, `lingtu-driver`

The field DDS contract name `thunder_dds_v1` is valid because it names a
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
- artifact provenance for saved maps, occupancy, tomograms, and reports;
- forbidden fallbacks, especially Python or ROS fallbacks that would weaken a
  native-DDS equivalence claim.

Simulation must not:

- add simulator-specific Product architectures;
- disable SafetyRing, stop, stale-input, or command-owner gates;
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
| `env=real` | Thunder MID-360/IMU through native services | Native C++ SLAM/localizer over `thunder_dds_v1` | Product-native `navd`/`mapd`/traversability where declared | `lingtu-driver` to remote Brainstem | Field evidence only after robot-side readiness gates. |
| `mujoco_native` | MuJoCo MID-360/IMU records through native DDS shape | Native-DDS-like SLAM/status contract | Native navigation endpoint acceptance | MuJoCo DDS command tap | Native-DDS simulation evidence; not robot readiness. |
| `mujoco_host` | MuJoCo Host simulation adapters | Host simulation odometry/map-cloud source | Python Host/downstream simulation wiring | MuJoCo adapter or gate motion | Host simulation and downstream wiring evidence. |
| `gazebo` | Gazebo world, physics, rendered sensors | Gazebo-derived state/map into LingTu contracts | LingTu simulation navigation/exploration chain | Isolated Gazebo command relay | ROS/GZ integration and closed-loop smoke evidence. |

Forbidden claims are part of the contract:

- CMU Unity baseline or adapter experiments must not be reported as a LingTu
  Product, ProductControl/RunPlan execution, LingTu-built SLAM, or Product
  acceptance evidence.
- `mujoco_host` evidence must not be reported as native-DDS raw-sensor
  equivalence.
- `mujoco_native` evidence must not be reported as field readiness without a
  separate robot-side gate.
- PCT is a planner backend. TARE is an exploration strategy. A gate may prove
  that a planner can plan on a same-source map, but that is separate from
  proving TARE exploration quality.

## Runtime Evidence Required

A simulation run can support a Product claim only when the report proves:

- `env=sim`;
- the selected Product and RunPlan fingerprint;
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
