# Simulation Integration Contract

LingTu is the product system. Simulation environments are external inputs and
validation targets. They must not become product profiles, disable safety, or
replace the navigation chain with demo-specific shortcuts.

## Product Boundary

Product profiles are the field-facing LingTu stacks:

- `map`
- `nav`
- `explore`
- `tare_explore` as the LingTu-native traversable frontier exploration profile

Experimental SLAM backends such as `super_lio` and `super_lio_relocation` are
advanced runtime choices, not default field-facing product profiles. They must
prove the same Gateway, map, and real-runtime-evidence gates before any field
readiness claim.

Simulation and development profiles are separate:

- `stub`
- `dev`
- `sim`
- `sim_mujoco_live`
- `sim_mujoco_octo_live`
- `sim_gazebo`
- `sim_industrial`
- `sim_cmu_tare`
- `sim_nav`

`sim_industrial` is the first customer-visible Gazebo delivery scene profile.
It uses the same Module graph boundary as `sim_gazebo`, but pins the industrial
yard world, filtered RViz view, frontier/map growth checks, and simulation-only
command relay contract.

CMU Unity is not a LingTu product profile. It is split into two explicit
contracts:

- `cmu_unity_baseline`: CMU's original Unity + TARE/FAR + terrain/local planner
  + path follower loop. This is reference evidence only.
- `cmu_unity_external`: LingTu's adapter/execution gate. CMU Unity owns the
  world and sensors, external TARE owns exploration waypoints, and LingTu owns
  the `/nav/*` execution chain and simulation-only command relay.

## How Simulation Enters LingTu

Simulation may enter only through explicit boundary adapters:

1. The simulator publishes sensor, odometry, map, terrain, or waypoint native streams or topics.
2. A simulation adapter converts those streams into LingTu's canonical runtime stream tokens:
   `/nav/*` for navigation data and `/exploration/*` for exploration data.
3. LingTu modules run the same global planning, local planning, path following,
   safety, and gateway contracts that product profiles use.
4. Any command relay back to a simulator must be isolated to a non-default ROS
   domain and marked `simulation_only=true`.

## Dimos-Style Operator Model

Use product task plus endpoint as the primary operator model. The product task
is what LingTu should do; the endpoint is where sensors, state, and command
output are connected.

Target operator examples:

```bash
python lingtu.py explore --endpoint mujoco_live --record
python lingtu.py nav --endpoint replay status
python lingtu.py tare_explore --endpoint cmu_unity --record
python lingtu.py runtime-contract --json
python lingtu.py runtime-spec explore --endpoint mujoco_live
python lingtu.py nav --robot thunder --dog-host 192.168.66.190
```

The algorithm graph remains the same across these forms. The endpoint supplies
the data source and owns the command sink. Simulation and replay endpoints must
report `cmd_vel_sent_to_hardware=false`; real targets must route commands only
through `CmdVelMux` and the hardware driver. Endpoint-specific evidence still
controls what claims a run can support.

## Unified Runtime Interface

LingTu core must not care whether the endpoint is a real S100P, MuJoCo, Gazebo,
CMU Unity, or a future simulator. Each endpoint must satisfy the same runtime
port contract. The only endpoint-specific code is the adapter that converts
native streams or topics, frames, and commands into this contract.

The canonical Python interface is defined in `src/runtime/runtime_interface.py`.
`config/topic_contract.yaml` mirrors it for operators and diagnostics. New
simulators or robot variants must add/choose a data-source contract there
instead of creating new `/nav/*` meanings.
Use `python lingtu.py runtime-contract` for the operator summary of frames,
topic frame_id rules, resolved real runtime data-flow, stage-interface
bindings, and algorithm interfaces. Add `--json` to export the full canonical
manifest that defines topics, data sources, endpoint command sinks, saved
artifact formats, and the same frame/data-flow/interface contracts.
The same manifest exports required TF links (`map->odom`, `odom->body`, and
`body->lidar`) plus the LiDAR mounting profiles, so endpoint adapters can be
checked against one frame contract instead of duplicating coordinate rules.
Algorithm interfaces also expose `map_dependency`, which is the boundary that
separates live SLAM/map use from saved artifacts such as a PCT tomogram.

`src/runtime/blueprints/runtime_endpoint.py` defines the Dimos-style split between
task and connection layer. The product task remains `map`, `nav`, `explore`, or
`tare_explore`; `--endpoint thunder-field|mujoco_live|gazebo|cmu_unity|replay`
selects the runtime source/sink. Legacy board aliases such as `real_s100p`
still resolve to `thunder_field` for old scripts, but they are not product
contract names. Compatibility profiles such as
`sim_mujoco_live`, `sim_mujoco_octo_live`, `sim_industrial`, and `sim_cmu_tare`
are launcher aliases for gates and demos, not independent product
architectures.

Examples:

```bash
python lingtu.py explore --endpoint mujoco_live --record
python lingtu.py nav --endpoint replay status
python lingtu.py tare_explore --endpoint cmu_unity --record
python lingtu.py nav --robot thunder --dog-host 192.168.66.190
```

External runtime launchers and `lingtu status` print the resolved boundary:
`Runtime`, `SLAM`, `Frames`, `Topic frames`, and `Flow`. This is the quick
operator check that the selected profile is using the expected data source,
SLAM/localization/map owner, TF links, topic frame_id contract, and command
sink. Use `switch-plan` when the full sim/replay/real diff is needed before
switching targets.
Use `runtime-contract` when the canonical interface summary is needed, and
`runtime-contract --json` when the full machine-checkable manifest is needed.
Use `runtime-spec` when a single profile or endpoint needs its resolved
data-flow, topic frame rules, frame links, validation blockers, and runtime
environment inspected without launching modules; add `--json` for the
machine-checkable payload.
During a live Gateway run, `/api/v1/navigation/status` exposes the same current
process boundary under `runtime`: `profile`, `endpoint`, `data_source`,
`runtime_contract`, `command_sink`, `frame_links`, `topic_allowed_frame_ids`,
`required_topic_frame_ids`, `runtime_data_flow_topics`, and
`resolved_runtime_data_flow`. `/api/v1/ready` mirrors the same boundary summary
under `runtime.boundary`, so readiness failures can be traced back to the exact
data source, topic frame contract, required frame-id evidence, data-flow topic
set, and command sink. Treat mismatches in `runtime.blockers` or
`runtime.boundary.blockers` as a run configuration error before accepting
navigation evidence.
`/api/v1/runtime/dataflow` is the live Module-first discovery surface for
Gateway clients. It lists canonical runtime stream tokens, matching ModulePorts,
Gateway payload channels, and whitelisted command interfaces while keeping
`ros2_topic_required=false`. For one stream, use
`/api/v1/runtime/dataflow/topic?topic=odometry` or the canonical token such as
`/nav/odometry`; this reports fresh ModulePort samples, payload interfaces, and
whether communication is allowed through Gateway commands. For UI clients that
need a live view, `POST /api/v1/runtime/dataflow/subscribe` returns a read-only
Gateway SSE subscription plan, typically `/api/v1/events?topic=...`, for a
whitelisted stream. It is an observation/whitelist contract, not arbitrary
publish access into ModulePorts, and not a ROS2 topic browser.

### Canonical Port Groups

| Group | Canonical stream tokens/artifacts | Producer | Consumer | Required for |
| --- | --- | --- | --- | --- |
| Canonical SLAM input | `/nav/lidar_scan`, `/nav/imu` | Real LiDAR/IMU adapter | Fast-LIO or equivalent SLAM backend | Production mapping/localization |
| Raw SLAM validation input | `/points_raw`, `/imu_raw` | Raw-sensor simulator adapter | Fast-LIO or equivalent SLAM backend | LingTu-owned mapping/localization validation |
| Localization/map output | `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud`, `/nav/localization_health` | LingTu SLAM or external-source adapter | Maps, exploration, navigation, gateway, safety | Live exploration and navigation |
| Exploration | `/exploration/start`, `/exploration/way_point`, `/nav/exploration_grid`, `/exploration/status` | TARE, frontier, or external exploration adapter | Exploration bridge, Navigation, Gateway | No-map exploration |
| Planning/execution | `/nav/goal_pose`, `/nav/patrol_goals`, `/nav/global_path`, `/nav/local_path`, `/nav/mission_status` | LingTu navigation stack | Path follower, safety, gateway, recorder | Goal navigation and route execution |
| Command | `/nav/cmd_vel` | LingTu `CmdVelMux` | Endpoint command adapter only | Simulated or real actuation |
| Artifacts | `map.pcd`, `tomogram.pickle`, `occupancy.npz`, `metadata.json` | Map manager/save pipeline | Relocalization, PCT, diagnostics | Saved-map navigation |

### Endpoint Adapter Matrix

| Endpoint | Native input | Adapter obligation | Canonical LingTu entry | Command sink | Product claim allowed |
| --- | --- | --- | --- | --- | --- |
| Real S100P | MID-360 LiDAR, IMU, robot driver | Publish canonical sensor topics, run Fast-LIO/localizer, relay only muxed safe commands | `/nav/lidar_scan + /nav/imu -> Fast-LIO -> /nav/*` | Real driver through safety/mux | Field mapping, relocalization, navigation after hardware validation |
| MuJoCo raw MID-360 | Simulated MID-360 pattern cloud and IMU | Generate credible raw sensor topics and feed Fast-LIO | `/points_raw + /imu_raw -> Fast-LIO -> /nav/*` | MuJoCo command adapter | LingTu SLAM/map/exploration smoke when raw-source evidence is present |
| Gazebo industrial | Gazebo world, physics, rendered LiDAR, sim odom | Convert Gazebo sensor/odom to `/nav/*`, isolate command relay | `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud`, `/nav/exploration_grid` | `/lingtu/gazebo/cmd_vel` | ROS/GZ integration, obstacle/map-growth, navigation smoke; not Fast-LIO quality |
| CMU Unity external | CMU `/state_estimation`, `/registered_scan`, `/terrain_map_ext`, `/way_point` | Remap external state/map/TARE waypoints into LingTu execution contract | `/nav/odometry`, `/nav/registered_cloud`, `/nav/terrain_map_ext`, `/exploration/way_point` | `/cmd_vel` in isolated simulation domain | LingTu can consume external CMU/TARE waypoints and execute; not LingTu SLAM |
| Future Unity raw-sensor mode | Unity raw LiDAR/IMU export | Publish raw LiDAR/IMU to Fast-LIO before `/nav/*` | `/points_raw + /imu_raw -> Fast-LIO -> /nav/*` | Unity command adapter | LingTu SLAM claim only after raw-source evidence passes |

### Adapter Acceptance Rules

Every endpoint adapter must declare:

- source ownership: who owns world, sensors, localization, map, exploration,
  planning, path following, and command relay;
- frame mapping: native frames to `map`, `odom`, and `body`;
- topic mapping: native topics to the canonical contract;
- command boundary: `/nav/cmd_vel` must never reach hardware from a simulation
  adapter;
- artifact provenance: saved `map.pcd`, tomogram, and occupancy products must
  record the source run and checksums;
- forbidden fallbacks: if PCT falls back to A*, or Fast-LIO is bypassed, the
  report must say so and strict gates must fail.

Simulation must not:

- add new simulator-specific product architectures when a runtime endpoint or
  compatibility gate alias is sufficient;
- disable `SafetyRingModule` stop wiring;
- enable direct-goal or direct-track planning bypasses in product profiles;
- relay `/nav/cmd_vel` to hardware or a default ROS domain;
- use cumulative debug clouds as proof of product SLAM quality.

## Fast Switching Between Sim, Replay, And Real

Fast switching means the product task graph stays stable while the endpoint
changes. Before moving between simulation, replay, and real targets, operators
should use a switch plan to inspect the boundary change:

```bash
python lingtu.py switch-plan sim_mujoco_live explore
python lingtu.py switch-plan explore explore --current-endpoint mujoco_live --endpoint thunder-field
python lingtu.py switch-plan nav nav --endpoint replay
python lingtu.py switch-plan explore nav --endpoint thunder-field
```

The default switch-plan output is an operator summary. Use `--json` or
`--json-out` for the full machine-readable diff. Both forms should make these
fields explicit:

- `endpoint`
- `robot_preset`
- `data_source`
- `runtime_contract`
- `slam_source`
- `localization_source`
- `mapping_source`
- `lidar_extrinsic_profile`
- `command_sink`
- `frame_links`
- `topic_allowed_frame_ids`
- `required_topic_frame_ids`
- `runtime_data_flow_topics`
- `resolved_runtime_data_flow`
- `simulation_only`
- `launcher` and `launcher_args`
- `current_validation`, `target_validation`, and top-level `ok`

A valid diff changes the endpoint boundary without changing the product task
semantics. Simulation and replay targets must never use
`hardware_driver_after_cmd_vel_mux`. Real targets must use
`hardware_driver_after_cmd_vel_mux` only through `CmdVelMux`; direct hardware
actuation paths are outside the contract.

Compatibility profiles are resolved back to their runtime endpoint in the
switch plan. For example, `sim_mujoco_live -> explore` should report
`mujoco_live -> thunder_field`, not an anonymous in-process source. Real targets
must report `runtime_contract=thunder_field` and must not carry launcher
arguments; simulation and replay targets may carry the gate or recording action
used by their external launcher.
Both sides of the switch plan must validate; a clean target does not hide a
broken current runtime boundary.

`switch-plan` is a dry-run/preflight contract, not an in-process hot-swap
operation. Switching endpoints requires a fresh launcher boundary or an explicit
stop/start lifecycle so stale simulation state cannot be mistaken for real
field evidence.

## Data Flow And Frames

The endpoint boundary is also the data-flow boundary:

```text
sensor/log/simulator source
  -> endpoint adapter
  -> canonical LingTu topics
  -> SLAM or relayed localization/map
  -> map layers and exploration
  -> global planner
  -> local planner and path follower
  -> CmdVelMux
  -> endpoint command sink
```

The machine-checkable template is `runtime_data_flow` in
`runtime_contract_manifest()` and `config/topic_contract.yaml`. Each stage names
its inputs, outputs, owner, frame role, and map dependency. The current stages
are `endpoint_adapter`, `slam_or_relayed_localization_map`,
`map_layers_and_exploration`, `global_planning`,
`local_planning_and_following`, and `command_boundary`.

For an actual endpoint, use `resolved_runtime_data_flow.<data_source>` or
`resolved_runtime_data_flow(data_source)`. That expanded contract replaces
template placeholders with concrete source topics and command sinks. Examples:
`thunder_field` resolves to `/nav/lidar_scan + /nav/imu -> /nav/odometry +
/nav/registered_cloud + /nav/map_cloud -> hardware_driver_after_cmd_vel_mux`;
`mujoco_fastlio2_live` resolves to `/points_raw + /imu_raw -> Fast-LIO ->
/nav/* -> mujoco_velocity_adapter`; `gazebo_industrial` resolves native Gazebo
topics into `/nav/*` and ends at `/lingtu/gazebo/cmd_vel`.

The coordinate contract is:

```text
map -> odom -> body -> lidar_link
```

Endpoint reports must preserve evidence for these links when a gate requires
frame validation. `map->odom` and `body->lidar` may be static links, while
`odom->body` must be observed from live odometry or an equivalent relayed state
stream. Registered clouds are body-frame local planning inputs; map clouds and
saved artifacts are map-frame products unless the artifact metadata says
otherwise.

Runtime topic `frame_id` validation is topic-specific, not a single global
frame assertion. `/nav/odometry` is valid when it reports either the active
planning frame (`map` by default) or the canonical odometry frame (`odom`);
Gateway reports any other odometry frame, such as `camera_link`, as a
`frame_mismatch_odometry` blocker. Costmap and goal evidence stay stricter:
their reported frames must match the active planning frame.

## SLAM Source Boundary

Every simulation contract must declare where localization and map data come
from. A run may not claim LingTu SLAM, localization, mapping, or exploration
closure unless the declared SLAM source is present in the runtime evidence.

Current boundaries:

- `gazebo_industrial` uses Gazebo odometry and a Gazebo-LiDAR-derived
  occupancy grid. It can validate command plumbing, obstacle checks, frontier
  regressions, and map growth, but it is not a Fast-LIO mapping validation.
- `cmu_unity_baseline` uses CMU Unity `/state_estimation`,
  `/registered_scan`, and `/terrain_map_ext`. It is reference behavior only.
- `cmu_unity_external` relays CMU Unity state, scan, and terrain topics into
  `/nav/*`. It validates LingTu execution against external TARE waypoints; it
  does not validate LingTu Fast-LIO.
- `mujoco_fastlio2_live` is the official MuJoCo raw-sensor Fast-LIO profile
  behind `python lingtu.py sim_mujoco_live ...`. Its native source path is
  `/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_registered +
  /cloud_map`, and the gate must relay those outputs into `/nav/odometry`,
  `/nav/registered_cloud`, and `/nav/map_cloud` before LingTu modules consume
  them.

If a report uses simulator-provided `/state_estimation`, `/registered_scan`, or
`/terrain_map_ext`, it must be described as an external simulation data source,
not as LingTu-built SLAM output.

## Gazebo Role

Gazebo is a ROS-native smoke, regression, and delivery-demo gate. It is useful
for:

- ROS/GZ bridge health;
- frame and timestamp contracts;
- LiDAR-derived occupancy growth;
- wavefront frontier regression checks;
- closed-loop command plumbing.

The supported customer-visible Gazebo entry is `sim_industrial`, launched by
`sim/scripts/launch_lingtu_gazebo_industrial_demo.sh`. Gazebo still owns only
world geometry, physics, sensor rendering, and simulated actuation; LingTu owns
occupancy mapping, exploration goal selection, global planning, local planning,
path following, `CmdVelMux`, and safety.

## CMU Unity Role

CMU Unity is a benchmark adapter for large simulation scenes and TARE/FAR-style
exploration evidence. LingTu adopts the topic contract, not the upstream stack
as a product dependency.

The adapter boundary is:

- CMU -> LingTu: `/state_estimation`, `/registered_scan`, `/terrain_map`,
  `/terrain_map_ext`, `/way_point`;
- LingTu -> CMU: `/exploration/start`;
- simulation-only command relay: `/nav/cmd_vel -> /cmd_vel`, disabled by
  default and allowed only in an isolated ROS domain.

## Responsibility Matrix

| Contract | World/Sensors | Exploration | Global/Strategy | Local Planning | Path Following | Command Owner | Product Claim |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `gazebo_industrial` | Gazebo | LingTu frontier/TARE contract | LingTu navigation | LingTu navigation | LingTu navigation | LingTu `CmdVelMux` to Gazebo adapter | LingTu closed-loop smoke/demo evidence |
| `cmu_unity_baseline` | CMU Unity | CMU TARE/FAR | CMU exploration stack | CMU `localPlanner` | CMU `pathFollower` | CMU path follower to Unity simulator | Reference effect only; not LingTu validation |
| `cmu_unity_external` | CMU Unity | External CMU TARE waypoint source | LingTu navigation, optionally PCT when the gate requires it | LingTu navigation | LingTu path follower | LingTu adapter relay to CMU simulator | LingTu can ingest CMU/TARE waypoints and execute in simulation |
| `mujoco_fastlio2_live` | MuJoCo raw LiDAR/IMU | LingTu frontier when `explore/video` is used | LingTu navigation when `explore/video` is used | LingTu navigation when `explore/video` is used | LingTu path follower when `explore/video` is used | MuJoCo velocity adapter or fixed gate motion | Fast-LIO raw sensor to canonical `/nav/*`; optional live exploration/navigation gate |
| `thunder_field` | Thunder MID-360/IMU | LingTu frontier or TARE profile | LingTu navigation/PCT | LingTu navigation | LingTu path follower | LingTu `CmdVelMux` to hardware driver | Thunder field runtime boundary; field readiness still requires robot-side evidence |

Forbidden claims are part of the runtime contract:

- `cmu_unity_baseline` must not be reported as LingTu planning, LingTu command
  ownership, a LingTu product profile, or real-robot readiness.
- `cmu_unity_external` must not be reported as CMU-baseline equivalence, pure
  LingTu exploration, "PCT executes TARE", LingTu Fast-LIO mapping, or
  real-robot readiness.
- PCT is a LingTu planner backend. TARE is an exploration strategy. A gate may
  prove that LingTu/PCT can plan on a same-source CMU map, but that is a
  separate planning claim from TARE exploration quality.

## Runtime Evidence Required

A simulation run can support a product claim only when the report proves:

- `simulation_only=true`;
- `real_robot_motion=false`;
- `cmd_vel_sent_to_hardware=false`;
- odometry, map, terrain, global path, local path, and command topics are seen;
- publisher identity matches the expected LingTu/simulation boundary nodes;
- map or explored area grows from live sensor input;
- Gateway planner diagnostics are available for strict CMU Unity runtime gates.

If topic samples exist but publisher identity is missing or fake, the run is not
a valid LingTu closure.
