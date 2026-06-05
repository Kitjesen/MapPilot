# LingTu Simulation Closure Plan

## Target

Build one product-grade simulation chain for LingTu before claiming exploration,
mapping, relocalization, or PCT navigation as delivered.

The chain must keep data provenance explicit:

```text
raw LiDAR + IMU
  -> Fast-LIO live SLAM
  -> /nav/odometry + /nav/map_cloud + /nav/registered_cloud
  -> LingTu exploration/navigation modules
  -> simulation-only /cmd_vel
```

No report may claim LingTu mapping or localization unless it proves this source
path or clearly declares an external simulator source.

## Interface-First Rule

All endpoints must connect through one LingTu runtime interface. Real hardware,
MuJoCo, Gazebo, CMU Unity, and future simulators are not separate product
architectures. They are adapters into the same contract:

```text
endpoint native topics
  -> endpoint adapter
  -> config/topic_contract.yaml canonical topics
  -> LingTu modules
  -> endpoint command adapter
```

The interface source of truth is `src/core/runtime_interface.py`.
`config/topic_contract.yaml` is the human-readable mirror checked against the
Python manifest. The ownership and simulator boundaries are documented in
`docs/architecture/SIMULATION_INTEGRATION_CONTRACT.md`.

## Current Gate Boundary Correction

The server MuJoCo + MID-360 + Fast-LIO gate does not validate every module under
`src`. Its current proven scope is deliberately narrower:

```text
MuJoCo raw MID-360 pattern LiDAR + IMU
  -> Fast-LIO live odometry/map output
  -> SlamBridge normalized /nav/odometry, /nav/registered_cloud, /nav/map_cloud
  -> same-source map.pcd + metadata + tomogram artifact generation
```

That gate is a validation harness, not the product runtime. It may launch and
observe MuJoCo, ROS 2, and Fast-LIO processes, but reusable runtime behavior
must move behind Module-First boundaries: Modules, NativeModules, bridge
services, or MapManager services. A passing fixed-drive gate must not be
reported as proof of TARE, PCT, saved-map relocalization, semantic navigation,
Gateway, all `src` modules, or real-robot readiness.

The next refactor target is:

- keep `sim/scripts/mujoco_fastlio2_live_gate.py` as a thin launcher/evidence
  collector/assertion layer;
- keep same-source map and tomogram writing in
  `src/nav/services/same_source_map_artifacts.py` so the gate only
  calls a reusable service;
- keep raw MuJoCo LiDAR/IMU message conversion in
  `src/drivers/sim/mujoco_sensor_bridge.py` so the gate only calls a reusable
  sensor bridge;
- keep MuJoCo live world resolution and engine construction in
  `src/drivers/sim/mujoco_live_runtime.py` so the gate does not own simulator
  runtime setup;
- keep Fast-LIO live config writing and process lifecycle in
  `src/slam/fastlio2_live_bridge.py` so the gate does not own the SLAM launch
  behavior;
- keep Fast-LIO `/Odometry`, `/cloud_registered`, and `/cloud_map`
  normalization into LingTu `/nav/*` topics in
  `src/slam/fastlio2_nav_bridge.py` so the gate does not own SLAM-to-nav
  runtime topic bridging;
- make `python lingtu.py sim_mujoco_live ...` and
  `python lingtu.py explore --endpoint mujoco_live` enter the same Module graph
  instead of depending on scattered gate logic.

Each endpoint contract now separates:

- `source_outputs`: what the endpoint or simulator natively provides;
- `normalized_outputs`: what the adapter publishes at the LingTu boundary;
- `algorithm_entry_outputs`: the core topics every LingTu algorithm chain must
  be able to consume;
- `algorithm_context_outputs`: extra live context topics for a specific
  algorithm path, such as an exploration grid or TARE waypoint.

Topic payload formats are also part of the contract. `TOPIC_FORMATS` in
`src/core/runtime_interface.py` maps source, normalized, and algorithm topics to
their message formats; `config/topic_contract.yaml` mirrors that map as
`topic_formats`.

For navigation/exploration, every product data source must reach the same
algorithm entry trio: `/nav/odometry`, `/nav/registered_cloud`, and
`/nav/map_cloud`. Extra outputs, such as `/nav/exploration_grid` for Gazebo or
`/exploration/way_point` for CMU TARE, are additive and do not replace that
base contract.

The key product decision is:

- Real S100P and MuJoCo raw MID-360 are true LingTu SLAM paths when they prove
  `/points_raw + /imu_raw -> Fast-LIO -> /nav/*`.
- Gazebo industrial is a ROS/GZ integration and delivery-demo path unless it
  also provides raw sensor topics into Fast-LIO.
- CMU Unity external is a benchmark/adapter path unless it exports raw
  LiDAR/IMU into Fast-LIO.
- PCT is tested only after a same-source `map.pcd` and tomogram exist.

## Dimos Reference Pattern To Adopt

Dimos is useful here because it keeps the robot application graph stable and
switches only the connection layer:

```text
same robot modules
  -> real connection OR simulator connection
  -> same LiDAR / odom / camera / cmd_vel contract
  -> same mapper / planner / explorer
```

LingTu should follow that product shape:

- `sim` remains the existing in-process MuJoCo module-graph smoke profile.
- `sim_mujoco_live` becomes the official raw-sensor simulation profile:
  MuJoCo publishes MID-360-pattern LiDAR and IMU into Fast-LIO, then LingTu
  consumes the canonical `/nav/*` outputs.
- `sim_industrial` remains the Gazebo delivery-demo path for ROS/GZ scenes.
- `sim_cmu_tare` remains an external benchmark/TARE adapter path until it
  provides raw LiDAR/IMU into LingTu Fast-LIO.

The first code change must therefore create a first-class LingTu entrypoint:

```bash
python lingtu.py sim_mujoco_live gate
python lingtu.py sim_mujoco_live explore
python lingtu.py sim_mujoco_live video
```

Those commands are allowed to call a launcher script internally, but the user
should not have to remember scattered gate commands.

Current implementation slice:

- create `sim_mujoco_live` as a first-class LingTu profile;
- bind it to the `mujoco_fastlio2_live` runtime/data-source contract;
- route `python lingtu.py sim_mujoco_live <gate|explore|video|status|stop>` to
  one launcher;
- add a Dimos-style runtime endpoint layer so product tasks can run unchanged
  against simulation endpoints:
  - `python lingtu.py explore --endpoint mujoco_live`;
  - `python lingtu.py explore --endpoint gazebo`;
  - `python lingtu.py tare_explore --endpoint mujoco_live`;
  - `python lingtu.py tare_explore --endpoint cmu_unity`;
- keep `sim_mujoco_live`, `sim_industrial`, and `sim_cmu_tare` as compatibility
  aliases for demos/gates, not as separate product architectures;
- keep raw-sensor Fast-LIO proof separate from saved-map relocalization and
  PCT proof.

Current status:

- `sim_mujoco_live` is registered in the LingTu CLI/profile layer.
- `mujoco_live` is registered as a runtime endpoint for product task entry.
- The endpoint launcher is `sim/scripts/launch_mujoco_fastlio2_live.sh`.
- Local and server CLI contract tests pass for:
  - `python lingtu.py sim_mujoco_live status`;
  - `python lingtu.py explore --endpoint mujoco_live`;
  - `python lingtu.py explore --endpoint mujoco_live --record`;
  - `python lingtu.py tare_explore --endpoint mujoco_live`;
  - visible `demo` action wiring.
- Server product entrypoint proof:
  - `python3 lingtu.py sim_mujoco_live gate`
    produced
    `artifacts/server_sim_closure/cli_sim_mujoco_live_gate8/gate-20260519_175436/report.json`;
  - `python3 lingtu.py explore --endpoint mujoco_live`
    produced
    `artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live20/explore-20260519_175530/report.json`.

## Dependency Split

### No Saved Map Required

These can run from a cold start:

- Fast-LIO live SLAM from raw LiDAR and IMU.
- Live map growth from `/cloud_map`.
- Live obstacle/local terrain view from registered cloud or terrain map.
- Exploration target generation from the live map.
- Local obstacle avoidance and short-horizon path following.

Required evidence:

- raw sensor samples exist;
- Fast-LIO publishes odometry and map cloud;
- odometry moves;
- live map area grows;
- commands stay simulation-only;
- planner/explorer consumes the same live source.

### Saved Map Required

These must not be claimed until a map is built and saved:

- saved-map navigation to a known target;
- relocalization against `map.pcd`;
- PCT global planning over a tomogram;
- semantic/location navigation to prior tags or rooms.

Required evidence:

- saved `map.pcd` exists and has points;
- tomogram exists when PCT is used;
- localizer loads the saved map;
- `/nav/relocalize` or `/nav/global_relocalize` succeeds;
- localization health reaches `LOCKED` or equivalent;
- `map->odom` or equivalent global correction is valid;
- navigation consumes the saved-map frame, not a simulator shortcut.

## Phased Delivery

### Phase 0 - Truth Boundary

Status: passed in code contract tests; latest static scan has
`unknown_nav_literals=0`.

Goal: stop false positives.

Deliverables:

- simulation contracts declare SLAM, localization, and mapping source;
- endpoint contracts declare `source_outputs`, `normalized_outputs`, and
  `algorithm_entry_outputs`;
- endpoint-specific extras are separated into `algorithm_context_outputs`;
- topic payload formats are declared in `TOPIC_FORMATS` / `topic_formats`;
- CMU/Gazebo external data cannot be reported as LingTu Fast-LIO;
- saved-map relocalization contract is labeled as contract-only until runtime
  relocalization is executed.

Exit criteria:

- server closure fails if a report claims Fast-LIO without raw LiDAR+IMU and
  Fast-LIO outputs;
- server closure fails if a saved-map runtime claim lacks a saved map and
  localizer evidence.

### Phase 1 - MuJoCo Live SLAM Base

Status: current server runtime passed the stricter canonical `/nav/*` closure
for the MuJoCo/Fast-LIO source path.

Goal: make MuJoCo the first official LingTu simulation source because it already
provides raw MID-360 pattern LiDAR and IMU.

Deliverables:

- raw MID-360 pattern cloud and IMU publish into Fast-LIO;
- Fast-LIO output is remapped to `/nav/odometry`, `/nav/map_cloud`, and
  `/nav/registered_cloud`;
- LingTu `SlamBridgeModule` reads those `/nav/*` topics.

Exit criteria:

- `/points_raw`, `/imu_raw`, `/Odometry`, `/cloud_map`,
  `/cloud_registered` are present;
- `/nav/odometry`, `/nav/map_cloud`, `/nav/registered_cloud` are present;
- map grows and odometry moves in the same run.

Current evidence:

- Unit and contract tests cover MID-360 pattern loading, MuJoCo point cloud
  field generation, IMU specific force, and source-to-canonical topic
  definitions.
- Server run
  `artifacts/server_sim_closure/mujoco_fastlio2_live/demo-20260519_050248/report.json`
  passed with `ok=true`, `canonical_nav_outputs_verified=true`,
  `/nav/odometry=1070`, `/nav/registered_cloud=1070`, `/nav/map_cloud=1070`,
  Fast-LIO movement `9.797 m`, and MuJoCo movement `9.937 m`.
- Server run
  `artifacts/server_sim_closure/mujoco_fastlio2_live_strict_fixed/explore-20260519_062600/report.json`
  passed with `ok=true`, `imu_acc_mode=gravity_only`,
  `canonical_nav_outputs_verified=true`, `/nav/odometry=245`,
  `/nav/registered_cloud=245`, `/nav/map_cloud=245`, Fast-LIO movement
  `4.620 m`, MuJoCo movement `4.657 m`, and Fast-LIO Z drift error
  `0.038 m`. This short proof is now superseded by the finite-difference IMU
  runs below because longer gravity-only live exploration exposed Fast-LIO Z
  drift.
- Server run
  `artifacts/server_sim_closure/mujoco_live_navcmd_default_fixedimu/explore-20260519_130108/report.json`
  passed with `ok=true`, `imu_acc_mode=finite_difference`,
  `canonical_nav_outputs_verified=true`, Fast-LIO movement `3.591 m`, MuJoCo
  movement `3.582 m`, Fast-LIO Z drift error `0.0218 m`, and
  `sim_realtime_factor=0.1926`.
- Server run after moving MuJoCo sensor message conversion into
  `src/drivers/sim/mujoco_sensor_bridge.py`:
  `artifacts/server_sim_closure/mujoco_live_sensor_bridge_refactor/explore-20260519_132526/report.json`
  passed with `ok=true`, `remaining_gaps=[]`, `imu_acc_mode=finite_difference`,
  Fast-LIO movement `2.710 m`, MuJoCo movement `2.703 m`, Fast-LIO Z drift
  error `0.0272 m`, yaw delta error `0.0078 rad`, `cloud_published=340`,
  `imu_published=1696`, and `sim_realtime_factor=0.1882`.
- Server smoke after moving Fast-LIO topic normalization into
  `src/slam/fastlio2_nav_bridge.py`:
  `artifacts/server_sim_closure/mujoco_nav_bridge_refactor_gate8/gate-20260519_163730/report.json`
  passed with `ok=true`, `canonical_nav_outputs_verified=true`,
  `/nav/odometry=11`, `/nav/registered_cloud=11`, `/nav/map_cloud=11`,
  `map_artifacts.ok=true`, and `remaining_gaps=[]`.
- Product CLI gate through `python3 lingtu.py sim_mujoco_live gate`:
  `artifacts/server_sim_closure/cli_sim_mujoco_live_gate8/gate-20260519_175436/report.json`
  passed with `ok=true`, `remaining_gaps=[]`,
  `canonical_nav_outputs_verified=true`, raw LiDAR/IMU verified,
  `/nav/odometry=12`, `/nav/registered_cloud=12`, `/nav/map_cloud=12`,
  Fast-LIO state `TRACKING`, `map.pcd` containing `13,809` points, tomogram
  shape `[5, 2, 154, 104]`, and no hardware output.

Required next proof:

- Keep this proof fresh whenever the MuJoCo, Fast-LIO, topic contract, or
  bridge code changes.

### Phase 2 - No-Map Exploration

Status: current MuJoCo/Fast-LIO exploration runtime passed for both
frontier-based no-map exploration and native TARE no-map exploration on the
same LingTu runtime contract.

Goal: explore without a prior map.

Deliverables:

- TARE or frontier consumes the live Fast-LIO map source;
- multiple exploration goals are produced over time;
- TARE goals are counted only after LingTu Navigation reports a matching
  terminal result;
- local planner avoids obstacles from live cloud/map data;
- robot moves enough to grow the map.

Exit criteria:

- at least 3 distinct exploration goals;
- nonzero `/nav/cmd_vel`;
- odometry delta exceeds the gate threshold;
- live map area grows;
- strict TARE/CMU gates require at least one matching Navigation `SUCCESS` and
  reject Navigation failures for TARE goals;
- no hardware command publisher;
- paths do not use saved-map-only inputs.

Known gaps:

- The gate must require multiple exploration goals, live map area growth, and a
  Navigation terminal result for each accepted exploration goal.
- CMU Unity strict runtime remains an external-source benchmark until raw
  LiDAR+IMU drives Fast-LIO, but it must still prove the TARE goal -> LingTu
  Navigation terminal feedback loop through Gateway `/api/v1/explore/status`.

Latest accepted evidence:

- Server run
  `artifacts/server_sim_closure/mujoco_live_sensor_bridge_refactor/explore-20260519_132526/report.json`
  passed with `ok=true`.
- Frontier goals: `4`; successful Navigation terminal events: `3`; failed
  terminal events: `0`.
- `/nav/cmd_vel` was nonzero and drove MuJoCo through the simulation-only
  command adapter.
- `/nav/global_path` and `/nav/local_path` were both published, with local
  path updates from `LocalPlannerModule` and `PathFollowerModule` outputting
  commands through `CmdVelMux`.
- Map growth: exploration known-area growth `48.08 m2`; coverage growth
  `0.0835`; `/nav/map_cloud` area growth was enforced by the gate.
- Planner fallback, repair, and direct-goal fallback were not used.
- Runtime note: this gate advanced only `33.92 s` of MuJoCo simulation during
  `189.40 s` wall time (`sim_realtime_factor=0.1882`). Acceptance is based on
  simulated sensor/SLAM progress, not wall-clock assumptions.
- Visible RViz/MuJoCo demo on
  `artifacts/server_sim_closure/mujoco_fastlio2_live_visible_fixed2/demo-20260519_063103/report.json`
  showed live windows, `/nav/*` topics, nonzero motion, stable Fast-LIO Z, and
  map artifacts, but did not satisfy the strict frontier terminal-count gate
  under GUI load. Treat visible demo as operator review evidence, not the
  acceptance gate.
- Server run after moving the LingTu frontier stack configuration into
  `src/drivers/sim/mujoco_lingtu_stack.py`:
  `artifacts/server_sim_closure/mujoco_lingtu_stack_refactor_explore35/explore-20260519_160505/report.json`
  passed with `ok=true`, `remaining_gaps=[]`, `frontier_goal_count=2`,
  `successful_navigation_goal_count=1`, `failed_navigation_goal_count=0`,
  `/nav/global_path` and `/nav/local_path` present, and no planner fallback.
- The same run explains the small net displacement: MuJoCo net displacement was
  `1.335 m`, but simulated path length was `3.855 m`, Fast-LIO path length was
  `5.470 m`, linear command integral was `4.727 m`, and absolute yaw command
  integral was `11.292 rad`. The robot was moving and turning; net displacement
  alone under-reports progress.
- Product task entry through `python3 lingtu.py explore --endpoint mujoco_live`:
  `artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live20/explore-20260519_175530/report.json`
  passed with `ok=true`, `remaining_gaps=[]`, raw MID-360/IMU, Fast-LIO
  odometry/map, canonical `/nav/*`, nonzero `/nav/cmd_vel`, same-source map
  artifact, and frontier exploration checks all true.
- In that product-entry run, MuJoCo simulated time was `20.02 s` over
  `114.449 s` wall time (`sim_realtime_factor=0.1899`), Fast-LIO movement was
  `2.687 m`, MuJoCo movement was `2.698 m`, Fast-LIO path length was
  `2.870 m`, MuJoCo path length was `2.801 m`, `/nav/cmd_vel` had `387`
  nonzero samples, frontier goal count was `2`, Navigation success count was
  `1`, failures were `0`, and `/nav/map_cloud` XY area growth was
  `120.228 m2`.
- Larger product scene:
  `sim/worlds/industrial_park_scene.xml` is now the default
  `sim_mujoco_live` world. It is an ASCII single-floor industrial park scene
  with perimeter walls, road/cross aisles, racks, containers, columns,
  equipment, and a roof deck so MID-360 vertical returns keep Fast-LIO
  constrained.
- Server run through the product entrypoint
  `python3 lingtu.py explore --endpoint mujoco_live` on the industrial park:
  `artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live_industrial_park25_gatefix/explore-20260519_185942/report.json`
  passed with `ok=true` and `remaining_gaps=[]`.
- Industrial-park metrics: simulated time `25.02 s`, wall time `138.696 s`
  (`sim_realtime_factor=0.1933`), Fast-LIO movement `2.777 m`, MuJoCo movement
  `2.796 m`, Fast-LIO path length `3.790 m`, MuJoCo path length `3.765 m`,
  Z drift error `0.0176 m`, yaw delta error `0.0018 rad`, `/nav/cmd_vel`
  nonzero samples `481`, frontier goals `3`, Navigation successes `2`,
  failures `0`, global path max `10` points, local path max `101` points, and
  `/nav/map_cloud` XY area growth `357.638 m2`.
- Native TARE on the same MuJoCo/Fast-LIO runtime contract:
  `artifacts/server_sim_closure/mujoco_tare_exploration_holdgoal_180s/tare-20260520_002231/report.json`
  passed with `ok=true` and `remaining_gaps=[]`.
- TARE runtime metrics: raw MID-360/IMU verified, Fast-LIO `/nav/*` verified,
  TARE goal count `3`, matching Navigation success count `1`, Navigation
  failures `0`, `/nav/global_path` count `24`, `/nav/local_path` count `324`,
  `/nav/cmd_vel` nonzero linear samples `1334`, MuJoCo movement `3.692 m`,
  Fast-LIO movement `3.662 m`, `/nav/map_cloud` XY area growth `407.614 m2`,
  exploration coverage growth ratio `0.0889`, and simulation-only hardware
  output checks passed.
- TARE bridge correction: CMU TARE still publishes waypoints in its native
  `map` frame, but `TAREExplorerModule` now supports an explicit
  `goal_frame_id` override. The MuJoCo/Fast-LIO TARE stack sets this to
  `odom` and holds each active goal until Navigation reports a terminal result,
  preventing rapid TARE waypoint churn from interrupting local execution.
- Gate correction: `OccupancyGridModule` now marks `/nav/exploration_grid` as
  `accumulation=rolling_local_window`. The product acceptance metric for
  cumulative map growth is `/nav/map_cloud`, while `/nav/exploration_grid`
  proves frontier input health (`free`, `occupied`, and `unknown` cells plus
  non-empty frontier batches).
- The older `factory` MuJoCo world remains a legacy stress scene, not the
  product default. It failed this acceptance path because multi-floor geometry
  caused Fast-LIO/MuJoCo motion and Z consistency divergence.

### Phase 3 - Map Save Productization

Status: same-source map and tomogram production is implemented and verified on
the strict MuJoCo/Fast-LIO exploration run.

Goal: turn the live map into product artifacts.

Deliverables:

- save `map.pcd`;
- build occupancy/tomogram products from the same source;
- record metadata: source, frame, timestamp, point count, checksum.

Exit criteria:

- saved map exists and point count is nonzero;
- tomogram exists when requested;
- source metadata links back to the live Fast-LIO run.

Latest accepted evidence:

- Code now saves same-source map artifacts by default in
  `src/nav/services/same_source_map_artifacts.py`, with the gate
  acting as the caller/evidence collector.
- Raw MuJoCo LiDAR/IMU ROS message conversion now lives in
  `src/drivers/sim/mujoco_sensor_bridge.py`; the gate imports that bridge
  instead of defining sensor message builders itself.
- MuJoCo live world resolution, MID-360 pattern resolution, start-pose parsing,
  memory patching, and engine construction now live in
  `src/drivers/sim/mujoco_live_runtime.py`; the gate imports that runtime
  service instead of defining simulator setup itself.
- Fast-LIO live config and process lifecycle now live in
  `src/slam/fastlio2_live_bridge.py`; the gate imports `FastLio2Process`
  instead of managing the Fast-LIO subprocess directly.
- Fast-LIO-to-LingTu nav topic normalization now lives in
  `src/slam/fastlio2_nav_bridge.py`; the gate imports
  `FastLio2NavBridgeRuntime` instead of defining the `/Odometry` and cloud
  callback bridge itself.
- LingTu MuJoCo frontier/navigation stack construction now lives in
  `src/drivers/sim/mujoco_lingtu_stack.py`; the gate calls that builder instead
  of embedding `full_stack_blueprint(...)` product configuration.
- Server strict exploration run
  `artifacts/server_sim_closure/mujoco_live_sensor_bridge_refactor/explore-20260519_132526/report.json`
  passed with `ok=true`, `canonical_nav_outputs_verified=true`, and
  `map_artifacts.ok=true`.
- Saved map:
  `same_source_map/map.pcd`, frame `odom`, source
  `/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_map -> /nav/map_cloud`.
- Metadata:
  `same_source_map/metadata.json`, includes source topics, frame, world,
  point count, and PCD checksum.
- Same-source map artifacts from the latest server run contain `56,113` PCD
  points in frame `odom`; tomogram shape is `[5, 2, 107, 103]` and records the
  same input PCD checksum.
- Latest same-source map from
  `artifacts/server_sim_closure/mujoco_lingtu_stack_refactor_explore35/explore-20260519_160505/report.json`
  contains `57,043` PCD points in frame `odom`; tomogram shape is
  `[5, 2, 148, 128]`.
- Latest industrial-park same-source map from
  `artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live_industrial_park25_gatefix/explore-20260519_185942/report.json`
  contains `58,709` PCD points in frame `odom`; tomogram shape is
  `[5, 3, 172, 179]`.
  `[5, 4, 156, 105]`; `/nav/map_cloud` XY area growth was `209.129 m2`.
- Product endpoint same-source map from
  `artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live20/explore-20260519_175530/report.json`
  contains `52,246` PCD points; tomogram shape is `[5, 2, 134, 105]`; the map
  was produced from the same raw MuJoCo MID-360 + IMU -> Fast-LIO -> `/nav/*`
  path used by the exploration run.

### Phase 4 - Saved-Map Relocalization

Status: server runtime passed on the industrial-park same-source map.

Goal: prove navigation can restart from a saved map.

Deliverables:

- localizer loads saved `map.pcd`;
- `/nav/relocalize` or `/nav/global_relocalize` succeeds;
- localization health reaches `LOCKED`;
- map-frame pose is stable enough for navigation.

Exit criteria:

- runtime relocalization executed, not just contract inspection;
- saved-map localization status is healthy;
- `/nav/odometry` and map-frame correction are usable.

Latest accepted evidence:

- Runtime gate:
  `artifacts/server_sim_closure/saved_map_relocalize_industrial_park20_gatefix/report.json`.
- Closure summary:
  `artifacts/server_sim_closure/summary_saved_map_relocalize_industrial_park20_gatefix.json`.
- Source map:
  `artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live_industrial_park25_gatefix/explore-20260519_185942/same_source_map/map.pcd`.
- Result: `ok=true`, `/nav/relocalize` succeeded, localizer state `LOCKED`.
- Localizer evidence: saved map cloud latest count `58,691`, tracking health
  samples `33`, `map->odom` XY correction `0.0098 m`, Z correction `0.0064 m`.
- Live feed evidence: Fast-LIO moved `2.288 m`, MuJoCo moved `2.283 m`,
  `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud`, and nonzero
  `/nav/cmd_vel` were all observed.

Boundary:

- This proves saved-map ICP relocalization and stable `map->odom` correction
  for the same-source MuJoCo/Fast-LIO map.
- Kidnapped-robot global relocalization is proven separately in Phase 4A and
  must not be inferred from the plain `/nav/relocalize` ICP gate.

### Phase 4A - BBS3D Kidnapped Global Relocalization

Status: server runtime passed after enabling the BBS3D CPU backend in the
localizer build.

Goal: prove the robot can start from a wrong global pose, call BBS3D global
relocalization, recover against a saved same-source map, and return to healthy
local ICP tracking.

Deliverables:

- BBS3D dependency installed under `third_party/3d_bbs/install`;
- localizer build links `libcpu_bbs3d.so`;
- localizer loads the saved `map.pcd` without relying on `last_pose.txt`;
- kidnapped initial pose is injected;
- `/nav/global_relocalize` is called explicitly;
- localization health transitions through `LOST` and then `RECOVERED` or
  `LOCKED`;
- live Fast-LIO feed continues while the relocalizer runs.

Exit criteria:

- BBS3D is enabled, not disabled by the build;
- BBS3D success is observed in the runtime report;
- `/nav/global_relocalize` service succeeds;
- localizer health reaches a tracking state after the kidnapped pose;
- live odometry/cloud samples continue and remain simulation-only.

Latest accepted evidence:

- Gate report:
  `artifacts/server_sim_closure/bbs3d_kidnapped_relocalize/report.json`.
- Closure summary:
  `artifacts/server_sim_closure/summary_policy_contact_bbs3d.json`.
- BBS3D source checkout:
  `third_party/3d_bbs` at upstream commit
  `41529a34a2fb9618b5ff560fb3c2363f1615666d`.
- Build evidence: localizer CMake reported `BBS3D enabled`, and `ldd` showed
  `libcpu_bbs3d.so` loaded from `third_party/3d_bbs/install/lib`.
- Result: `ok=true`, `global_relocalization_validated=true`,
  `/nav/global_relocalize` service success, `bbs3d_success_observed=true`,
  `bbs3d_disabled_observed=false`.
- Kidnapped pose: `x=3.0`, `y=2.0`, `yaw=1.2`.
- Health evidence: `LOCKED`, `LOST`, and `RECOVERED` were all observed;
  `lost_health_samples=258`, `tracking_health_samples=116`, latest health was
  `LOCKED`.
- Live source evidence: `/cloud_registered=346`, `/cloud_map=346`,
  `/Odometry=346`, nonzero `/nav/cmd_vel=717`, Fast-LIO movement `2.5069 m`,
  MuJoCo movement `2.5276 m`, saved map cloud `89,592` points.

Boundary:

- This proves kidnapped global relocalization in the server MuJoCo/Fast-LIO
  saved-map loop. It does not replace real robot field validation.

### Phase 5 - PCT Saved-Map Navigation

Status: server runtime passed on the industrial-park same-source map after
Phase 4 relocalization.

Goal: use the saved tomogram/map for global planning after mapping and
relocalization are proven.

Deliverables:

- PCT consumes the saved tomogram from Phase 3;
- global path starts near current localized pose;
- local planner/path follower tracks the route;
- fallback is reported explicitly and fails strict PCT gates.

Exit criteria:

- selected planner is PCT;
- no fallback in strict gate;
- route is followed with nonzero motion;
- obstacle clearance and path safety pass.

Latest accepted evidence:

- Gate report:
  `artifacts/server_sim_closure/pct_saved_map_navigation_industrial_park_gatefix/report.json`.
- Server closure:
  `artifacts/server_sim_closure/summary_pct_saved_map_navigation_industrial_park_gatefix.json`.
- Preconditions were enforced from Phase 3/4 evidence: same-source tomogram,
  saved-map relocalization report, localizer `LOCKED`, `/nav/relocalize`
  success, saved map cloud latest `58691` points, and `map->odom` xy correction
  `0.0098 m`.
- PCT preview selected `internal_free_route` with selected planner `pct`, PCT
  runtime libraries present, `path_count=7`, and no fallback reason.
- Native MuJoCo gate selected planner `pct`, used the native PCT backend, used
  no fallback, reached the goal, moved `12.0072 m`, ended `0.4973 m` from the
  goal, published `530` path points, sampled `60` local paths, and kept all
  navigation frames on `map` except `/nav/cmd_vel` in `base_link`.
- Obstacle metadata came from the MuJoCo industrial scene: `44` obstacle boxes,
  clearance checked, no collision, minimum clearance `0.3803 m`, robot radius
  margin `0.1003 m`.
- Trajectory quality passed: final progress ratio `0.9712`, p95 lateral error
  `1.0788 m`, zero progress regressions.
- Combined product closure:
  `artifacts/server_sim_closure/summary_mujoco_fastlio_relocalize_pct_nav_industrial_park.json`
  passed with required gates `fastlio2_live`, `saved_map_relocalize`, and
  `pct_saved_map_navigation` all verified and `remaining_gaps=[]`.

Boundary:

- This proves saved-map PCT navigation through the current MuJoCo kinematic
  closed-loop harness after relocalization. Physical gait/contact stability is
  proven separately in Phase 5B and must not be inferred from the PCT gate.
- It does not prove robot hardware readiness.
- PCT remains a saved-map navigation proof. Native TARE is proven separately in
  Phase 2 as no-map exploration target generation plus LingTu navigation
  execution; it is not a replacement for PCT saved-map planning.

### Phase 5C - Large Local Planning And Obstacle Avoidance

Status: server runtime passed for large static terrain, dynamic local obstacle
replanning, and a MuJoCo moving-obstacle close-pass route.

Goal: prove the global route can hand off to the production ROS2
`localPlanner` and `pathFollower`, and that the local planner avoids obstacles
instead of blindly following a straight line.

Deliverables:

- dynamic obstacle local-planner gate;
- large-terrain PCT/A* route generation;
- MuJoCo closed-loop `terrain_long` route with native PCT, native ROS2
  `localPlanner`, native ROS2 `pathFollower`, obstacle metadata, local-path
  samples, trajectory quality, and collision/clearance checks.
- Moving-obstacle MuJoCo close-pass on the same `terrain_long` route after the
  `large_terrain` tomogram exists.

Exit criteria:

- dynamic replanning changes path side when obstacle side changes;
- large terrain route is path-safe and selected by native PCT without fallback;
- MuJoCo closed-loop route reaches the goal;
- local path samples are present and obstacle-aware;
- trajectory clearance is checked and collision is false;
- moving obstacles are published, checked against the robot trail, and report
  collision false;
- hardware command output remains disabled.

Latest accepted evidence:

- Dynamic local obstacle report:
  `artifacts/server_sim_closure/dynamic_obstacle_local_planner/report.large_local.json`.
- Large terrain report:
  `artifacts/server_sim_closure/large_terrain/report.large_local.json`.
- MuJoCo native local-planner closed-loop report:
  `artifacts/server_sim_closure/native_pct_mujoco/report.large_local.json`.
- Moving-obstacle close-pass report:
  `artifacts/server_sim_closure/native_pct_mujoco/report.moving_obstacle_close_pass.json`.
- Closure summary:
  `artifacts/server_sim_closure/summary_large_local_obstacle.json`.
- Moving-obstacle closure summary:
  `artifacts/server_sim_closure/summary_moving_obstacle_close_pass.json`.
- Reproducible Dimos-style entrypoint:
  `python lingtu.py sim_mujoco_live pct-moving-obstacle`, which runs
  `sim/scripts/launch_mujoco_fastlio2_live.sh pct-moving-obstacle` against
  the existing `large_terrain` map/tomogram report and writes a closure-ready
  `report.json`.
- Server entrypoint run:
  `artifacts/server_sim_closure/mujoco_fastlio2_live/pct-moving-obstacle-20260520_162804/report.json`;
  closure summary:
  `artifacts/server_sim_closure/mujoco_fastlio2_live/pct-moving-obstacle-20260520_162804/closure_summary.json`.
- Result: closure `ok=true`; required gates `dynamic_obstacle_local_planner`,
  `large_terrain`, and `native_pct_mujoco` all verified;
  `remaining_gaps=[]`.
- Dynamic obstacle gate: `ok=true`, backend `nanobind`, five phases verified
  with avoidance sequence `straight -> right -> left -> right -> straight`,
  minimum clearance `0.4223 m`, and clear-path recovery true.
- Large terrain gate: `ok=true`; routes `terrain_short`, `terrain_long`,
  `terrain_narrow_gap`, and `terrain_slope_bypass` validated; native PCT
  backend was available and path safety passed.
- MuJoCo `terrain_long` closed loop: `ok=true`, selected planner `pct`, PCT
  native backend used, fallback not used, reached goal, moved `21.3682 m`,
  final distance `0.4988 m`, `/cmd_vel` nonzero count `6184`, path updates
  `2069`, and local path samples `60`.
- Obstacle evidence: 18 obstacle boxes, 2702 metadata obstacle points, global
  trajectory minimum clearance `0.6086 m`, clearance after robot radius
  `0.3286 m`, collision false, local path collision sample count `0`, points
  into obstacle sample count `0`, and local-path worst-sample clearance after
  robot radius `1.4898 m`.
- Trajectory quality: reference length `21.765 m`, final progress ratio
  `0.975`, mean lateral error `0.0086 m`, p95 lateral error `0.021 m`, and
  progress regressions `0`.
- Moving-obstacle close-pass: `ok=true`, selected planner `pct`, native PCT
  backend used, fallback not used, reached goal, moved `21.3704 m`, final
  distance `0.4991 m`, `/cmd_vel` nonzero count `6193`, path updates `2095`,
  moving obstacle updates `1616`, moving obstacle points max `20`, stop/clear
  sequence observed as `[2, 0, 0, 0]`, moving obstacle trail collision
  `false`, and moving obstacle clearance after robot radius `0.0816 m`.
- Latest Dimos-style entrypoint rerun: `ok=true`, selected planner `pct`,
  fallback not used, reached goal, moved `21.3707 m`, final distance
  `0.4978 m`, `/cmd_vel` nonzero count `6185`, path updates `2092`, moving
  obstacle updates `1613`, trail collision `false`, and moving obstacle
  clearance after robot radius `0.1 m`.
- Visual moving-obstacle gate rerun:
  `python lingtu.py sim_mujoco_live pct-moving-obstacle-video`, report
  `artifacts/server_sim_closure/mujoco_fastlio2_live/pct-moving-obstacle-20260520_172520/report.json`,
  closure summary
  `artifacts/server_sim_closure/mujoco_fastlio2_live/pct-moving-obstacle-20260520_172520/closure_summary.json`,
  and MP4 evidence
  `artifacts/server_sim_closure/mujoco_fastlio2_live/pct-moving-obstacle-20260520_172520/pct_moving_obstacle.mp4`.
  Result: closure `ok=true`, `remaining_gaps=[]`, selected planner `pct`,
  fallback not used, reached goal, moved `21.3682 m`, final distance
  `0.5 m`, `/cmd_vel` nonzero count `6166`, path updates `2062`, two moving
  route-crossing obstacles at `10 s` period, moving obstacle updates `1263`,
  moving obstacle points max `40`, trail collision `false`, clearance after
  robot radius `1.1744 m`, video layout `evidence`, `1379` frames at `4 FPS`,
  `1280x720`.

Boundary:

- This proves large-scale local planning and obstacle avoidance in the
  MuJoCo kinematic closed-loop plus native ROS2 localPlanner/pathFollower. It
  does not prove policy gait/contact by itself, and it does not prove real
  robot field behavior.
- The moving obstacle result proves the runtime stop-clear contract and a
  bounded simulation close-pass. It does not prove arbitrary real-world
  pedestrian/forklift behavior or policy-level dynamic avoidance on hardware.
- Closure now treats enabled `moving_obstacles` evidence as a contract:
  published updates, nonzero obstacle points, trail-clearance check, and
  collision false must all pass.
- Closure now also treats declared native PCT video evidence as a contract:
  when the report includes a video path, `video.exists=true` and
  `video.frames > 0` are required.
- 2026-05-20 follow-up: the Fast-LIO live TARE/frontier gate now has its own
  dynamic-obstacle injection path instead of relying only on the saved-map PCT
  gate. `sim/scripts/mujoco_fastlio2_live_gate.py` can inject
  `robot_crossing` moving boxes into the same raw MID-360 cloud before
  Fast-LIO consumes `/points_raw`, records obstacle density/speed parameters,
  published update counts, point-count maxima, trail clearance, and marks the
  obstacle returns red in MP4 evidence. `server_sim_closure.py` now enforces
  enabled `moving_obstacles` evidence for Fast-LIO live reports and also
  rejects declared Fast-LIO live video paths without nonzero frame/sample
  counts. New launcher actions:
  `tare-video`, `tare-moving-obstacle`, and
  `tare-moving-obstacle-video`. Local verification so far:
  `python -m pytest src/core/tests/test_native_pct_mujoco_gate.py src/core/tests/test_server_sim_closure.py src/core/tests/test_cli_no_repl.py src/core/tests/test_profile_graph_snapshots.py -q`
  passed `119`, and the live-gate helper density/speed unit test passed. Full
  server runtime recording for `tare-moving-obstacle-video` is the next
  acceptance run; do not claim it green until that artifact exists.
- Server visual smoke for the new Fast-LIO live dynamic-obstacle contract:
  `artifacts/server_sim_closure/mujoco_fastlio2_live/live-moving-obstacle-video-20260520_201337/report.json`,
  closure
  `artifacts/server_sim_closure/mujoco_fastlio2_live/live-moving-obstacle-video-20260520_201337/closure_summary.json`,
  and MP4
  `artifacts/server_sim_closure/mujoco_fastlio2_live/live-moving-obstacle-video-20260520_201337/mujoco_fastlio2_live_moving_obstacle.mp4`.
  Result: `fastlio2_live` closure `ok=true`, `remaining_gaps=[]`,
  `real_robot_motion=false`, `cmd_vel_sent_to_hardware=false`, Fast-LIO reached
  `TRACKING`, `/nav/odometry`, `/nav/registered_cloud`, and `/nav/map_cloud`
  each had `55` samples, `/nav/map_cloud` area growth was `416.1187 m2`, and
  a same-source `map.pcd` was written. Dynamic obstacle settings/evidence:
  `robot_crossing`, `count=3`, `period_s=6.0`, peak planar speed bound
  `1.0631 m/s`, `published_update_count=40`, `published_point_count_max=48`,
  trail clearance checked, collision `false`, clearance after robot radius
  `0.9529 m`, video `71` frames / `71` samples. The server had no OpenGL
  context, so the MP4 scene-render panel is blank/unavailable; the evidence
  panels still render raw/current scan, Fast-LIO map, trajectory, and red
  dynamic-obstacle returns. Local MP4 sanity check read `71` frames at
  `1280x720`; sampled frames had nonblack content and red obstacle pixels
  (`258`, `359`, `495`). This validates live SLAM + moving-obstacle video
  evidence, not TARE dynamic avoidance or full navigation around dynamic
  obstacles.

### Phase 5A - TARE Same-Source Full Chain

Status: server runtime passed end to end on the MuJoCo industrial-park endpoint.

Goal: prove the product chain that starts with LingTu `tare_explore` can build
the map artifacts later used by saved-map relocalization and PCT navigation.

Latest accepted evidence:

- Closure summary:
  `artifacts/server_sim_closure/summary_tare_mujoco_fullchain_cleanstats.json`.
- TARE/MuJoCo/Fast-LIO report:
  `artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_fullchain_cleanstats/tare-20260520_031528/report.json`.
- Same-source saved map:
  `artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_fullchain_cleanstats/tare-20260520_031528/same_source_map/map.pcd`.
- Same-source tomogram:
  `artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_fullchain_cleanstats/tare-20260520_031528/same_source_map/tomogram.pickle`.
- Saved-map relocalization report:
  `artifacts/server_sim_closure/saved_map_relocalize_tare_fullchain_cleanstats/report.json`.
- PCT saved-map navigation report:
  `artifacts/server_sim_closure/pct_saved_map_navigation_tare_fullchain_cleanstats/report.json`.

Results:

- Closure: `ok=true`, required gates
  `mujoco_tare_exploration`, `saved_map_relocalize`, and
  `pct_saved_map_navigation` all passed, `remaining_gaps=[]`.
- Simulation boundary: `cmd_vel_sent_to_hardware=false`,
  `real_robot_motion=false`.
- TARE exploration/navigation: TARE enabled, started after SLAM ready, emitted
  goals, `successful_navigation_goal_count=1`,
  `failed_navigation_goal_count=0`, internal
  `navigation_failure_count=0`, `global_path_count=24`,
  `local_path_count=336`, `/nav/cmd_vel` nonzero samples `613`.
- Fast-LIO/mapping: official MID-360 pattern SHA
  `448821576a658673e8f7929992c8c0d687eb052657d7b584d038729a83da1bfb`,
  Fast-LIO motion `4.700 m`, MuJoCo motion `4.645 m`, `map.pcd` has
  `89,615` points, tomogram shape is `[5, 3, 178, 176]`.
- Map growth: `/nav/map_cloud` XY area growth `408.681 m2`; exploration known
  area growth `117.080 m2`.
- Relocalization: `/nav/relocalize` succeeded, localizer state `LOCKED`,
  saved map cloud `89,592` points, `map->odom` XY correction `0.0037 m`.
- PCT navigation: selected planner `pct`, native backend used, fallback not
  used, reached goal, moved `5.2975 m`, final distance `0.4986 m`,
  minimum obstacle clearance `1.5771 m`, trajectory progress ratio `0.9135`.

Boundary:

- This proves the simulation product chain from TARE exploration through
  same-source map artifact generation, saved-map relocalization, and PCT
  navigation in MuJoCo.
- It does not prove real robot hardware output.
- Physical gait/contact stability and kidnapped-robot global relocalization are
  now proven by separate gates, not by this TARE full-chain gate.

### Phase 5B - Policy Gait And Contact Stability

Status: server runtime passed in MuJoCo using the policy navigation gate.

Goal: prove the simulation is not only moving a kinematic marker. The policy
robot must load the ONNX policy, maintain finite body state, keep stable
height/attitude, produce foot-ground contacts, avoid non-foot ground contacts,
and still complete the LingTu navigation command path.

Deliverables:

- contact sampling from MuJoCo `data.contact`;
- direct policy drive stability check;
- full-stack LingTu policy navigation stability check;
- policy gate included in `server_sim_closure.py`.

Exit criteria:

- ONNX policy is loaded;
- direct policy and full-stack policy navigation both pass;
- body `z`, roll, and pitch stay within thresholds;
- at least two feet are in support during contact windows;
- contact normal force is nonzero;
- non-foot ground contacts remain zero;
- `/nav/cmd_vel` stays simulation-only and no hardware command is published.

Latest accepted evidence:

- Gate report:
  `artifacts/server_sim_closure/policy_nav/report.json`.
- Closure summary:
  `artifacts/server_sim_closure/summary_policy_contact_bbs3d.json`.
- Direct policy: passed, moved `0.21797 m`, body `z=0.402..0.437`, max roll
  `0.049 rad`, max pitch `0.031 rad`, `200` foot contact samples, all four
  feet observed, average support count `3.695`, max normal force `298.36 N`,
  non-foot ground contacts `0`.
- Full-stack policy navigation: passed, `nav_state=SUCCESS`, moved
  `0.55825 m`, global/local/path-follower command path active, `110` foot
  contact samples, all four feet observed, average support count `3.509`, max
  normal force `309.10 N`, non-foot ground contacts `0`.
- Simulation boundary: `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`.

Boundary:

- This proves server-side MuJoCo policy/contact stability for the current
  model, policy, and short navigation task. It does not prove physical S100P
  hardware stability.

### Phase 6 - CMU Unity Decision

Status: external baseline only.

CMU Unity is useful as a visual/reference benchmark. It is not a LingTu
Fast-LIO closure unless one of these becomes true:

- CMU Unity exports raw LiDAR + IMU into Fast-LIO; or
- a bridge generates physically credible raw sensor topics from Unity before
  Fast-LIO.

Until then, CMU Unity reports must say external state/map source, not LingTu
SLAM.

## Immediate Work Order

0. Freeze the runtime interface layers: `source_outputs`, normalized sensor
   outputs, core algorithm entry outputs, and algorithm-specific context
   outputs must have separate meanings.
1. Freeze the runtime interface: every adapter must declare native topics,
   canonical topics, ownership, frames, command sink, artifact provenance, and
   forbidden claims.
2. Add contract checks that reject runs missing the required canonical topics
   for their endpoint type.
3. Add runtime topic shape checks for the critical trio:
   `/nav/odometry`, `/nav/registered_cloud`, `/nav/map_cloud`.
4. Re-run MuJoCo Fast-LIO live SLAM on the server and require canonical
   `/nav/*` output counts in the report.
5. Add explicit map-area growth and coverage metrics to the Phase 2 gate.
6. Run the same live Fast-LIO exploration in a larger industrial scene.
7. Save map artifacts from that same run.
8. Build tomogram from the saved map.
9. Add runtime relocalization gate.
10. Run strict PCT saved-map navigation after relocalization is proven.
11. Enable and prove BBS3D kidnapped-robot global relocalization.
12. Prove MuJoCo policy gait/contact stability instead of relying only on
    kinematic navigation gates.
13. Prove large-scale local planning and obstacle avoidance through native
    ROS2 localPlanner/pathFollower in MuJoCo.

Current status:

- Items 0-4 are implemented and server-verified for the MuJoCo raw-sensor
  endpoint.
- Item 5 is implemented for the current MuJoCo/Fast-LIO frontier gate.
- Items 6-8 are implemented and server-verified for the new industrial park
  scene through `python3 lingtu.py explore --endpoint mujoco_live`.
- Item 9 is implemented and server-verified through
  `saved_map_relocalize_industrial_park20_gatefix`.
- Item 10 is implemented and server-verified through
  `pct_saved_map_navigation_industrial_park_gatefix`.
- Item 11 is implemented and server-verified through
  `bbs3d_kidnapped_relocalize`.
- Item 12 is implemented and server-verified through `policy_nav`.
- Item 13 is implemented and server-verified through
  `summary_large_local_obstacle.json`.
- The combined MuJoCo industrial-park chain is server-verified through
  `summary_mujoco_fastlio_relocalize_pct_nav_industrial_park.json`.
- The latest focused closure for physical contact and kidnapped relocalization
  is server-verified through `summary_policy_contact_bbs3d.json`.
- Native TARE runtime on this same contract is implemented and server-verified
  through `summary_tare_mujoco_fullchain_cleanstats.json`.
- 2026-05-20 live-map recheck tightened `mujoco_tare_exploration` closure:
  normal navigation gates with TARE/frontier now must show `/nav/map_cloud` as
  the accepted cumulative map source, a positive map-area growth threshold, and
  nonzero `/nav/map_cloud` area samples. TARE now also requires real
  `/nav/exploration_grid` evidence from `raycast_lidar_exploration_grid`,
  nonzero free/unknown cells, exploration area samples, known-area growth, and
  coverage growth. The historical cleanstats TARE report still passes this
  stricter exploration contract:
  `artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_fullchain_cleanstats/tare-20260520_031528/closure_summary_live_map_contract.json`.
  Local recheck with the added exploration gate is stored at
  `artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live_fullchain_cleanstats/tare-20260520_031528/closure_summary_exploration_contract.local.json`.
- Fresh 2-goal TARE attempts did prove live Fast-LIO mapping and local
  planning inputs, but did not pass the full 2-goal robustness gate yet:
  `mujoco_tare_exploration_live_map` failed Fast-LIO yaw consistency under
  sim-clock timing, while `mujoco_tare_exploration_live_map_wall240` passed yaw,
  map growth, local path, and cmd_vel checks but completed only 1/2 required
  TARE navigation goals. Do not report these fresh 2-goal runs as green.
- `tare_explore --endpoint mujoco_live` is now the product entrypoint for the
  MuJoCo raw-sensor TARE chain. Its default launcher action is `tare`, its
  `--record` launcher action is `tare-video`, and the server closure TARE gate
  now builds a same-source tomogram by default so the next relocalization and
  PCT gates can consume the map it just produced.

## P0 Contract Checklist

These are the non-negotiable product checks before claiming any simulator or
robot profile is usable:

- Localization: `/nav/odometry` exists, frame is `odom -> body`, timestamps are
  fresh relative to clouds, and motion is nonzero when commanded.
- Mapping: `/nav/map_cloud` exists in `map` or `odom`, point count is nonzero,
  area grows during motion, and saved `map.pcd` records source metadata.
- Registered scan: `/nav/registered_cloud` exists in `body`, is not reused as a
  fake accumulated map, and has expected PointCloud2 fields.
- Exploration: TARE or frontier produces repeated goals from the live map
  source, and LingTu Navigation reports terminal status for those goals.
- Global planning: A* may use live occupancy; PCT may only use same-source
  saved tomogram after map save and localization are proven.
- Local planning/following: `/nav/global_path -> /nav/local_path ->
  /nav/cmd_vel` must be visible with no direct goal fallback in strict gates.
- Relocalization: saved-map mode must load `map.pcd`, call relocalization, and
  report `LOCKED` or equivalent before navigation claims are accepted.
- Global relocalization: kidnapped-robot claims must call
  `/nav/global_relocalize`, observe BBS3D success, and show health recovery
  after `LOST`.
- Physical policy simulation: gait/contact claims must show policy loading,
  stable body state, foot-ground contacts, zero non-foot ground contacts, and
  simulation-only command output.
- Simulation adapters: MuJoCo, Gazebo, and CMU may differ only at the
  `source_outputs`/adapter boundary; LingTu modules must consume the same
  canonical topics.
