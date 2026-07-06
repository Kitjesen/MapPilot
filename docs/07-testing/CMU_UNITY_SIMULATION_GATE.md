# CMU Unity Simulation Gate

This gate tracks the product decision for the CMU
`autonomy_stack_mecanum_wheel_platform` simulation line.

## Decision

Do not vendor or present the CMU repository as LingTu's product simulator.
Use it in two separate roles: first as a CMU baseline reference, then as a
LingTu adapter/execution gate. Do not mix the two roles in one product claim.
The controlling architecture contract is
[`../architecture/SIMULATION_INTEGRATION_CONTRACT.md`](../architecture/SIMULATION_INTEGRATION_CONTRACT.md).

Adopt the contract, not the whole stack:

- ROS 2 Humble branch only.
- External workspace outside the LingTu tree.
- CMU topic contract: `/state_estimation`, `/registered_scan`,
  `/terrain_map`, `/terrain_map_ext`, `/way_point`, `/path`, `/cmd_vel`.
- LingTu remap contract: CMU scan/state/terrain topics must map into
  `/nav/*`; TARE waypoints must map into `/exploration/*`.
- LingTu adapter contract: `sim.engine.bridge.cmu_unity_lingtu_adapter`
  must relay the external CMU ROS graph into LingTu topics and only relay
  `/nav/cmd_vel` back to CMU `/cmd_vel` when explicitly enabled for an
  isolated simulation domain.
- SLAM source contract: this CMU line does not start LingTu Fast-LIO. CMU
  `/state_estimation`, `/registered_scan`, and `/terrain_map_ext` are external
  simulation data sources relayed into `/nav/*`; they must not be reported as
  LingTu-built SLAM output.

Responsibility split:

- `cmu_unity_baseline`: CMU Unity + CMU TARE/FAR + CMU terrain/local planner +
  CMU path follower. This is the reference effect to compare against; it is not
  a LingTu capability claim.
- `cmu_unity_external`: CMU Unity provides world/sensors and external TARE
  waypoints; LingTu owns `/nav/map_cloud`, `/nav/global_path`,
  `/nav/local_path`, `/nav/cmd_vel`, safety, and the simulation-only command
  relay.
- PCT is only a LingTu global-planner backend. It is not "the executor for
  TARE". When a PCT gate is required, it must be reported as a same-source
  map planning gate, separate from the TARE exploration claim.
- Fast-LIO is a separate source-of-truth gate unless the runtime is changed to
  publish raw LiDAR and IMU into Fast-LIO and consume `/Odometry` plus
  `/cloud_map` from Fast-LIO.

Do not adopt yet:

- Unity binary environment as a product asset. It is external, not versioned in
  GitHub, and needs a separate license/checksum chain.
- Whole-repo vendoring, especially SLAM drivers, built OR-Tools binaries, and
  route planner source with unclear top-level license text.
- Jazzy branch for LingTu Humble validation.

## Preflight Gate

Run this before any runtime or video claim:

```bash
export LINGTU_CMU_AUTONOMY_WS=/path/to/autonomy_stack_mecanum_wheel_platform
PYTHONPATH=src:. python3 sim/scripts/cmu_unity_sim_gate.py \
  --json-out artifacts/server_sim_closure/cmu_unity_sim/report.json \
  --strict
```

The report schema is `lingtu.cmu_unity_sim_gate.v1`.

Pass requires:

- ROS Humble setup, functional `ros2 --help`, and functional `colcon --help`
  are present on the server;
- external CMU workspace exists;
- workspace is a Git checkout on the `humble` branch;
- CMU simulation, exploration, local planner, path follower, and TARE launch
  files exist;
- Unity environment assets are present under
  `src/base_autonomy/vehicle_simulator/mesh/unity/environment`;
- colcon build output exists at `install/setup.bash`;
- expected CMU topics are present in source/config/launch files;
- CMU Unity endpoint overrides LingTu `tare_explore` to disable wavefront and
  use the external TARE bridge;
- LingTu TARE native remaps align scan, odometry, map, waypoint, runtime, and
  start/finish topics;
- LingTu has a checked-in CMU Unity adapter with a matching relay contract;
- `simulation_only=true`, `real_robot_motion=false`, and
  `cmd_vel_sent_to_hardware=false`.

The preflight does not launch Unity. It is a gate against false confidence, not
the final runtime proof.

## LingTu Adapter

Start the adapter in the same isolated ROS domain as the CMU Unity simulation:

```bash
export ROS_DOMAIN_ID=73
PYTHONPATH=src:. python3 -m sim.engine.bridge.cmu_unity_lingtu_adapter \
  --relay-cmd-vel-to-sim
```

Required relays:

- CMU -> LingTu:
  `/state_estimation -> /nav/odometry`,
  `/state_estimation_at_scan -> /nav/state_estimation_at_scan`,
  `/registered_scan -> /nav/registered_cloud`,
  `/registered_scan -> /nav/map_cloud`,
  `/terrain_map -> /nav/terrain_map`,
  `/terrain_map_ext -> /nav/terrain_map_ext`,
  `/way_point -> /exploration/way_point`.
- LingTu -> CMU:
  `/exploration/start -> /start_exploration`.
- Simulation command relay, explicitly gated:
  `/nav/cmd_vel -> /cmd_vel` as `geometry_msgs/msg/TwistStamped`.

The command relay is disabled by default. If enabled while `ROS_DOMAIN_ID` is
unset or `0`, the adapter refuses to start unless
`--allow-default-ros-domain` is passed for a controlled local test.

## Minimal Server Startup

Use this when you want to watch the CMU Unity scene on the server screen and
verify the LingTu closed loop in the same run. Keep the ROS domain under 100;
high Fast DDS domain IDs can fail before ROS nodes start.

```bash
cd /home/bsrl/hongsenpang/lingtu_closure_22f127b2
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash || true

export LINGTU_CMU_AUTONOMY_WS=/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform
export ROS_DOMAIN_ID=75
export DISPLAY=:1
export XAUTHORITY=/home/bsrl/.Xauthority
export PYTHONPATH=src:.:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH

bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh start --gate --rviz
```

The wrapper starts CMU ROS first, waits for the Unity TCP endpoint on port
`10000`, then starts the Unity binary, the LingTu adapter, and the LingTu
simulation stack. With `--rviz`, it also opens the CMU operator layout from
`sim/planning/cmu_unity_lingtu_runtime.rviz`. This LingTu-owned view opens
the CMU-derived global map cloud on `/nav/map_cloud`, the robot-local obstacle
cloud on `/nav/registered_cloud`, CMU terrain evidence on
`/nav/terrain_map_ext`, the TARE strategy path on `/exploration/local_path`,
and LingTu's `/nav/global_path` plus `/nav/local_path`. Set
`LINGTU_CMU_RVIZ_CONFIG` to the upstream
`src/base_autonomy/vehicle_simulator/rviz/vehicle_simulator.rviz` only when the
goal is to compare against the original CMU operator layout. By default
the wrapper also stops CMU's native `localPlanner` and `pathFollower` processes
after the CMU graph comes up, so `/cmd_vel` is owned by the LingTu adapter
relay instead of competing with CMU's original controller. This order is
intentional. The upstream CMU helper
`system_simulation_with_exploration_planner.sh` starts Unity before the ROS TCP
endpoint and also starts RViz in the same helper; on the server this produced
missing `/registered_scan` data and display-related RViz failures.

Useful controls:

```bash
bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh status
bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh rviz
bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh gate
bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh stop
```

To validate PCT against a CMU same-source map, first let the robot move and
capture the filtered terrain map into an official PCT tomogram. Subscribe to
the LingTu-relayed `/nav/map_cloud` and `/nav/terrain_map_ext` topics so the
captured map is the same source the LingTu stack consumes:

```bash
python3 sim/scripts/cmu_unity_tomogram_capture.py \
  --topic /nav/map_cloud \
  --topic /nav/terrain_map_ext \
  --duration-sec 120 \
  --min-points 1000 \
  --voxel-size 0.08 \
  --z-min -0.2 \
  --z-max 2.5 \
  --pcd-out artifacts/server_sim_closure/cmu_unity_map_first/terrain_ext.pcd \
  --tomogram-out artifacts/server_sim_closure/cmu_unity_map_first/terrain_ext_official_tomogram.pickle \
  --build-tomogram \
  --tomogram-mode official \
  --resolution 0.2 \
  --slice-dh 0.3 \
  --ground-h 0.0 \
  --json-out artifacts/server_sim_closure/cmu_unity_map_first/tomogram_report.json
```

Then restart the wrapper with that same-source tomogram and require no planner
fallback:

```bash
export LINGTU_CMU_TOMOGRAM=artifacts/server_sim_closure/cmu_unity_map_first/terrain_ext_official_tomogram.pickle
export LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK=0
bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh start --gate --rviz
python3 sim/scripts/cmu_unity_runtime_gate.py \
  --publish-start \
  --timeout-sec 120 \
  --require-no-planner-fallback \
  --json-out artifacts/server_sim_closure/cmu_unity_same_source_flat_pct/runtime_report.json \
  --strict
```

`sim/scripts/cmu_unity_lingtu_stack.py` is simulation-only. It uses the ROS2
simulated driver preset through `ROS2SimDriverModule`, receives odometry/clouds
from the CMU adapter on `/nav/*`, uses `slam_profile="none"` to avoid adding
`SlamBridgeModule`, runs the external TARE bridge mode, Gateway, map modules,
and PCT with path-safety fallback. It also sets `latch_stop_signal=False` so a
simulation SafetyRing warning cannot permanently freeze `/nav/cmd_vel`.

This means the visible CMU map is not a LingTu Fast-LIO map. It is a
CMU/Unity-provided registered scan and terrain stream relayed into LingTu. A
valid Fast-LIO claim must instead prove the raw path
`/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_map`, currently
covered by `mujoco_fastlio2_live`.

Keep this as a simulation script instead of a `lingtu.py` profile: CMU Unity is
an external benchmark line, not a first-class LingTu product profile. Do not
use the real `tare_explore` profile for this server demo unless the target is a
real S100P run.

## Runtime Gates

There are two runtime gates. Both are simulation-only and neither replaces
S100P field validation.

### CMU Baseline Reference

The baseline launches the original CMU Unity environment plus CMU TARE/FAR,
terrain analysis, local planner, path follower, and vehicle simulator. It must
show the reference effect the user expects: live registered scan, dense map,
TARE exploration waypoints, path, local trajectory, and `/cmd_vel` from the CMU
path follower.

This run validates the benchmark environment and sets a visual/performance
target. It must not be reported as LingTu planning or LingTu command ownership.

### LingTu Adapter Execution

The LingTu adapter gate launches CMU Unity in an isolated ROS domain and proves
that LingTu can consume CMU/TARE evidence through its own `/nav/*` contract:

- Unity publishes scan/state inputs used by mapping and planning.
- `/registered_scan`, `/nav/registered_cloud`, `/terrain_map`,
  `/terrain_map_ext`, and `/state_estimation` are synchronized enough for
  planning.
- TARE starts through `/start_exploration`.
- TARE emits at least one `/way_point` from a real map frontier or exploration
  boundary, not from a scripted fake grid.
- LingTu produces `/nav/global_path`, `/nav/local_path`, and simulated
  `/nav/cmd_vel`.
- Simulated odometry moves non-zero distance and remains inside the environment
  bounds.
- Mapped or traversable area grows over time.
- Strict runs emit `frontier_no_gain_stall` late-window evidence, proving
  odometry, non-zero command, path publication, and map/no-gain handling stayed
  live after the initial exploration pass condition.
- Strict runs emit `tare_strategy_quality`, proving Gateway/TARE stats show
  accepted waypoints, accepted paths, at least one strategy path, bounded
  suppressed-waypoint ratio, visible rejection reasons, and successful
  navigation without recorded TARE navigation failures.
- Gateway planner diagnostics are available, so the report can prove the
  LingTu planning chain participated instead of accepting topic-only motion.
- No driver, Thunder bridge, robot Gateway motion command, or hardware
  `cmd_vel` output is used.

### PCT Same-Source Planning Gate

PCT must be tested after enough map evidence exists. The gate captures
`/nav/map_cloud` plus `/nav/terrain_map_ext` into a same-source tomogram, then
runs LingTu planning with fallback disabled. Passing this gate means PCT can
plan safely on the CMU-derived map used by LingTu. It does not mean TARE is
implemented by PCT, and it does not prove full exploration coverage by itself.

### Historical Runtime Evidence

Latest accepted server no-fallback run for the map-first CMU Unity chain:

- Runtime report:
  `artifacts/server_sim_closure/cmu_unity_pct_strict_1778790415/report.json`
- Same-source official tomogram:
  `artifacts/server_sim_closure/cmu_unity_map_first_1778789145/terrain_ext_official_tomogram.pickle`
- Input PCD:
  `artifacts/server_sim_closure/cmu_unity_map_first_1778789145/terrain_ext.pcd`
- Tomogram report:
  `artifacts/server_sim_closure/cmu_unity_map_first_1778789145/tomogram_official_report.json`
- Gate command included `--require-no-planner-fallback`, with
  `LINGTU_CMU_ALLOW_DIRECT_GOAL_FALLBACK=0`.
- `ok=true`, `blockers=[]`.
- `primary_planner=pct`, `selected_planner=pct`, `fallback_used=false`,
  `direct_goal_fallback.used=false`, `rejected_plan_count=0`.
- Same-source map capture: `2821` PCD points from `/nav/terrain_map_ext`,
  frame `map`, bounds `x=[-11.37,26.25]`, `y=[-9.33,8.98]`,
  official tomogram shape `[5,2,193,96]`.
- PCT first detected a live-costmap collision on the original TARE goal path,
  then stayed in PCT and replanned to a nearby safe goal:
  `primary_replan.used=true`, `candidate_count=1`, repaired goal
  `(3.1387, 0.6229, 0.75)`, `selected_path_safety.ok=true`.
- `/nav/cmd_vel`: `882` non-zero samples out of `884`, `max_norm=1.3838`.
- `/nav/odometry` and `/state_estimation`: `2.8085 m` displacement.
- Map growth:
  `/registered_scan=146.8125 m2`, `/nav/registered_cloud=146.8125 m2`,
  `/nav/terrain_map=26.3125 m2`, `/nav/terrain_map_ext=71.6875 m2`.
- `/nav/global_path` and `/nav/local_path` both published non-empty paths.
- Command safety:
  `/cmd_vel` publisher is only `/lingtu_cmu_unity_adapter`;
  `/cmd_vel` subscriber is only CMU `vehicleSimulator`;
  `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`.

Older accepted server no-fallback run for the flat map-first CMU Unity chain:

- Runtime report:
  `artifacts/server_sim_closure/cmu_unity_same_source_flat_pct_replan/runtime_report.json`
- Same-source tomogram:
  `artifacts/server_sim_closure/cmu_unity_map_first_terrain_tomogram/terrain_ext_flat_trav_tomogram_dh1.pickle`
- Input PCD:
  `artifacts/server_sim_closure/cmu_unity_map_first_terrain_tomogram/terrain_ext_map_first.pcd`
- Gate command included `--require-no-planner-fallback`.
- `ok=true`, `blockers=[]`.
- `primary_planner=pct`, `selected_planner=pct`, `fallback_used=false`,
  `rejected_plan_count=0`.
- PCT first detected a live-costmap collision on the original TARE goal path,
  then stayed in PCT and replanned to a nearby safe goal:
  `primary_replan.used=true`, `candidate_count=31`, repaired goal
  `(2.7832, -0.6485, 0.75)`, `selected_path_safety.ok=true`.
- `/nav/cmd_vel`: `224` non-zero samples out of `229`, `max_norm=0.8984`.
- `/nav/odometry` and `/state_estimation`: `1.3855 m` displacement.
- Map growth:
  `/registered_scan=98.1875 m2`, `/nav/registered_cloud=98.1875 m2`,
  `/nav/terrain_map=13.3125 m2`, `/nav/terrain_map_ext=53.25 m2`.
- Command safety:
  `/cmd_vel` publisher is only `/lingtu_cmu_unity_adapter`;
  `/cmd_vel` subscriber is only CMU `vehicleSimulator`;
  `simulation_only=true`, `real_robot_motion=false`,
  `cmd_vel_sent_to_hardware=false`.

Latest accepted server run after the startup-order, scan-topic gate, stale
process cleanup, and LingTu path-safety fixes. This is simulation evidence
only; it does not make CMU Unity a LingTu profile and does not prove S100P field
readiness:

- Runtime report:
  `artifacts/server_sim_closure/cmu_unity_runtime_wrapper_clean_after_safety_fix/report.json`
- Wrapper launch command:
  `ROS_DOMAIN_ID=75 DISPLAY=:1 bash sim/scripts/launch_cmu_unity_lingtu_runtime.sh start --gate`
- Video runtime report:
  `artifacts/server_sim_closure/cmu_unity_runtime_video_after_pct_fix/runtime_report.json`
- Video evidence:
  `artifacts/server_sim_closure/cmu_unity_runtime_video_after_pct_fix/cmu_unity_tare_pct_runtime_after_fix.mp4`
- Video manifest:
  `artifacts/server_sim_closure/cmu_unity_runtime_video_after_pct_fix/video_manifest.json`
- Local mirrored frames:
  `artifacts/server_sim_closure/cmu_unity_runtime_video_after_pct_fix/early_scene_frame_60.png`,
  `mid_motion_frame_145.png`, and `late_map_frame_240.png`

Pass metrics from the runtime report:

- `ok=true`, `blockers=[]`.
- TARE waypoint observed on `/way_point` and `/exploration/way_point`:
  first `(3.9142, -0.8240, 0.75)` and last `(4.5302, -0.9714, 0.75)` in
  `map`.
- Required scan topics are live:
  `/registered_scan=30` samples and `/nav/registered_cloud=30` samples.
- `/nav/cmd_vel`: `471` non-zero samples out of `496`,
  `max_norm=1.0390`.
- `/nav/odometry`: `1.1481 m` displacement.
- `/state_estimation`: `1.1481 m` displacement.
- `/registered_scan`: `143.4375 m2` area growth.
- `/nav/registered_cloud`: `143.4375 m2` area growth.
- `/nav/terrain_map`: `27.4375 m2` area growth.
- `/nav/terrain_map_ext`: `60.4375 m2` area growth.
- `simulation_only=true`, `real_robot_motion=false`, and
  `cmd_vel_sent_to_hardware=false`.
- Hardware safety report has no blocked hardware nodes; `/cmd_vel` is only
  consumed by CMU `vehicleSimulator`.
- Legacy planner diagnostics from that older run showed PCT as the primary
  planner, an A* fallback as the selected planner, and a fallback reason of
  `pct path_safety failed (9 blocked samples)`.

Earlier product claim boundary:

- This is a CMU Unity + LingTu adapter + TARE + navigation closed-loop pass.
- PCT was integrated as the primary backend, but this older wrapper run did not
  validate a pure PCT route. The PCT path was rejected by 3D path safety, and
  the then-active A* fallback produced the accepted path and motion.
- Current product profiles use OctoPlanner3D with `plan_safety_policy=reject`
  and no planner fallback. Separate native MuJoCo/PCT gates remain useful for
  isolated PCT regression coverage.

## Visual Evidence

RViz/video evidence must show:

- Unity scene view or simulator view;
- robot model and odometry path;
- live scan or registered cloud in the correct frame;
- terrain/map cloud used by the planner;
- TARE/FAR waypoint and planned path;
- local planner path and simulated cmd_vel state;
- exploration/map area growth over time.

Do not use the cumulative debug cloud as the default visual proof. It can be
shown as a diagnostic layer only after the live registered scan and planner map
are clear.

## Gazebo Split

Gazebo remains a small ROS-native smoke gate for frames, bridge behavior,
navigation plumbing, and wavefront regression checks. It is not the product
demo baseline for TARE or large-scene exploration.

The CMU/Unity line becomes the benchmark candidate for product-facing
exploration demos once both the preflight gate and runtime gate pass.
