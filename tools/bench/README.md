# Offline benchmarks

The documented multi-shape representation is the **组合碰撞模型（Compound Collision
Model）**: several collision primitives linked by body-relative transforms.
URDF is a geometry source, `Body` the C++ container, and `BODY2` a legacy internal
serialization magic. The existing format, commands and identifiers are unchanged.
Formal Go2 real and Thunder real/sim navigation select their platform compound
models. The `parts` selection below is a component benchmark with measured joint
updates; formal navd currently uses nominal joints plus measured body attitude. See the
[maintained model design](../../docs/design/navigation-body-model-and-motion-posture.md).

## Native body-and-heading route demos

`body_export.py` freezes a URDF joint posture into conservative per-part OBBs.
It uses the existing MuJoCo dependency for offline FK; the native runtime has
no MuJoCo/Python dependency. `octoplanner3d_body_demo` builds four explicit
synthetic 3D OctoMaps and runs the shared global core with a historical 0.44 m
XYZ circle, then the same request with the body model and XYZ+yaw search.

```bash
python tools/bench/body_export.py --output build/body-routes/model.txt
cmake -S src/nav/cpp/planning/global/octoplanner -B build/body-routes-native \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/body-routes-native --target octoplanner3d_body_demo -j3
build/body-routes-native/octoplanner3d_body_demo \
  build/body-routes/model.txt build/body-routes/runs
python tools/bench/pose_view.py --out build/body-routes
```

Keep failures as well as successes. `scene.json`, `map.ot`, `before.json` and
`after.json` retain maps, endpoints, real returned points and measured runtime.
`body_sweep_valid` is a post-search recheck with the same native geometric
query, not an independent collision algorithm. PNG/PDF use Arial 10 pt. The
GIF interpolates returned geometry, not physical execution time or gait.
The [2026-10-22 run](../../build/native-body-routes-20261022/index.html)
includes the original failed spherical-box export, its correction and final
results. Native model loading and Body/heading interfaces now exist. A real
platform selects a saved Body through `geometry.navigation_body_file`; an empty
value uses OBB. The frozen export does not prove live joint/FK integration.

## Thunder policy physics and navigation models

`body_physics.py` resolves one `nav/sim` RunPlan for Thunder and SCAN without
starting a Product. Both `--navigation-body obb` and `--navigation-body parts`
consume its native environment, global/local parameters, follower and braking
settings. The same plan selects the robot MJCF and ONNX policy. Only the native
collision geometry and its measured updates differ: one configured OBB, or
a 组合碰撞模型 using per-part primitive snapshots from MuJoCo's actual collision
geoms and joints.

The sim package supplies the torso-origin vertical range and support height.
Do not substitute `nav/real` geometry or copy its numbers into this benchmark.
`navigation-config.json` retains the resolved plan and native environment;
`native/resolved-control.json` and `native/navigation-body.json` retain what the
native probe used. The world, spawn, map and obstacle observations remain
explicit component fixtures; this does not start the plan's full session or
qualify Go2 hardware. MuJoCo always solves contacts with its physical robot
collision geoms, regardless of the selected navigation approximation.
Native tuning overrides not owned by the compiled plan are retained and listed
in both configuration and invocation evidence. An ambient `LINGTU_NAV_BODY_FILE`
is discarded so it cannot replace the explicitly selected fixture geometry.
The parts model now uses the configured world-XY clearance with zero isotropic
margin; the OBB bakes in its configured horizontal clearance once. The explicit
OBB benchmark removes the plan's BODY_FILE so it remains an OBB ablation after
the formal package switched to compound geometry. Historical runs with a 0.02 m
isotropic parts margin remain historical evidence, not equal-clearance comparisons.

`body_execution.py` instead repeats historical frozen-geometry fixtures without
resolving a robot Product. Its `legacy` and `body` branches use historical versus
native-default control settings, respectively. They are not a model-only A/B.
Older physics records also retain their original settings: the 2026-10-26
1.16×0.76×0.60 m OBB and 38-part runs used different controls and are not a fair
model comparison. See the [actual runs and configuration correction](../../docs/operations/mujoco-navigation-20261026.md).

## Camera and Follow parallel load

`follow_camera.py` compares the same 640×480 synthetic RGB-D input at a requested
30 FPS with and without native H.264 encoding. Both runs use the real camera
SHM reader, synchronization, projection, object tracking and Follow consumer.
The detector and appearance features are controlled, with an explicit default
40 ms delay. This measures component scheduling and resource competition; it
does not measure YOLO/DINO performance, recognition, calibration, WebRTC network
latency or robot motion. No robot service or Product is started.

Use an existing Linux environment with the repository's locked core/vision
dependencies and the prepared video runtime. In WSL keep that Python environment
on the Linux filesystem; importing SciPy from a Windows-mounted dependency tree
can dominate cold startup. Dependency warmup is recorded outside the load window.

```bash
python tools/bench/follow_camera.py \
  --output build/follow-camera-load \
  --native-bin build/camera_video/lingtu_camera_webrtc \
  --runtime-env build/deps/video/prefix/env.sh \
  --duration-s 30
```

Use a new output directory for each run. `summary.json` records actual source
rate, observation-age percentiles, pending-frame coalescing, CPU and resident
memory. It also records capture-to-observation elapsed time with the monotonic
clock, alongside the original wall-clock frame age, so host clock steps are
visible rather than reported as processing stalls. CPU percentages use one
core as 100%. Each phase preserves observation
JSONL and diagnostics; the encoding phase additionally saves native metrics,
logs and the actual Annex-B H.264 stream. All three dedicated SHM rings and the
native child process are closed at the end. Synthetic scene bitrate is not a
real-camera bandwidth estimate. Follow goal requests have no motion executor
attached and cannot establish arrival or physical stop.

## Official point-cloud global planning

This is the maintained entry for the official PCD regression. Dated field
scripts such as `build/go2-map-save-v6-20260930/run_plans.py` and
`run_diagnostics.py` retain their original NX paths and historical options;
they are execution records, not current local test commands. Keep those
scripts with their evidence. Replaying a saved-ray field map also requires
the original `.ot` file; the modeled PCD cases below cannot substitute for it.

`official_global_planner.py` runs LingTu's native global planner on the upstream
[OctoPlanner3D sample clouds](https://github.com/JackJu-HIT/OctoPlanner3D/tree/9a9cc431ea905a5878975cc6fbbce6c9618b31a4/octomap/pcd_files).
The [manifest](../../config/acceptance/official_global_planner.json) fixes the
upstream commit, conversion parameters, robot geometry, endpoints and expected
outcomes. The two building routes use the author's `main.cpp` endpoints;
the plaza route is a LingTu regression scenario on the author's map.

Run on Linux/WSL with the existing C++ compiler, CMake, OctoMap, PCL and Python 3:

```bash
# From the repository root. A Linux-only PATH avoids WSL searching Windows SDKs.
export PATH=/usr/local/bin:/usr/bin:/bin
export LINGTU_OCTOPLANNER3D_BUILD_DIR=build/official-global-planner
export LINGTU_BUILD_JOBS=4
bash scripts/build/build_octoplanner3d.sh --require-pcl
cmake --build "$LINGTU_OCTOPLANNER3D_BUILD_DIR" --target \
  octoplanner3d_verify_path octoplanner3d_neighborhood_smoke \
  octoplanner3d_no_air_climb_smoke octoplanner3d_make_test_octomap -j4
ctest --test-dir "$LINGTU_OCTOPLANNER3D_BUILD_DIR" \
  -R 'octoplanner3d_(verify_path_self_test|neighborhood_smoke|no_air_climb_smoke|edit_octomap_binary_ot_smoke)' \
  --output-on-failure

python3 tools/bench/official_global_planner.py \
  --build-dir "$LINGTU_OCTOPLANNER3D_BUILD_DIR" \
  --upstream-dir third_party/OctoPlanner3D \
  --output-dir artifacts/official-global-planner-local
```

If the upstream checkout is absent, replace `--upstream-dir ...` with
`--download`. Both methods retrieve files at the pinned commit, including the
MIT license (copyright 2026 Chunyu Ju), into
`build/official-global-planner-data/`. Subsequent runs can use that cache without
either option. The runner uses Python's standard library; it does not install
packages or access a robot.

### What the cases establish

| Case | Required behavior |
| --- | --- |
| Building cross-building route | Reach the other building; equal endpoint heights do not imply a level route. |
| Building stairs | Reach the upper floor through supported, collision-free 26-neighbor vertices. |
| Plaza route | Reach the target across the central plaza. |
| Building geometry only | Reject the start when the PCD supplies no known-free body cells. |
| Plaza occupied goal | Reject a goal enclosed by an injected occupied box. |
| Plaza missing support | Reject a goal whose surrounding floor has been replaced by explicit free space. |

The source clouds have 853,227 and 1,675,994 points. They contain sampled
surfaces, not a time series of laser rays and sensor origins. Conversion uses
0.2 m voxels and **no support dilation**. Positive cases explicitly model five
free layers above exposed, locally connected surface cells using the existing
offline converter; original occupied cells are retained. This inferred free
space is a benchmark assumption, not measured free-space evidence. The
geometry-only negative case disables it. Field saved-scan ray replay remains
the source of real navigation maps.

The fixed profile uses a 0.155 m route radius, body clearance of 0.10 m above
and below its center, support height 0.35 +/- 0.05 m, neighboring support radius
one cell, maximum step 0.45 m and maximum slope 0.57735. Terminal 3D/XY/Z error
is limited to 0.20 m, and the test independently limits start snapping to
0.45 m. Endpoints describe the body center, not floor contact points.

Success requires a normal process exit, `ok`, `reached_goal`, a nonempty finite
path, recomputed endpoint errors, and a separate path-verifier invocation.
The verifier checks the returned vertices with the planner's 3D map predicate,
verifies 26-neighbor connectivity, and requires the full route's continuous 3D
sweep and declared metric motion constraints to pass. Supported body-height
search uses a finer Z lattice when required by the slope bound; the occupancy
resolution is unchanged. The verifier shares the planner predicate and is not
an independent proof of its implementation. Its synthetic tests cover supported
rising/falling routes, insufficient step height, unknown gaps and body collision.
Deliberately colliding vertices and non-adjacent shortcuts are rejected.

Negative cases require the exact rejection reason and no returned route.
A crash, timeout or malformed JSON is a test failure. The command exits
nonzero for any mismatch. Each run retains requests, results, native logs,
converted maps, path XYZ files and `summary.json` under the output directory.
The summary labels its verdict as `validation_scope=offline_global_route_geometry`
and `execution_validated=false`; `ok` reports fixture expectations, including
expected rejections, and never reports robot-motion readiness.
Use a fresh output directory when retaining evidence from different revisions.

This is offline algorithm evidence. It does not validate SLAM scan coverage,
saved-map ray construction, localization, SCAN trajectory execution or robot
locomotion. The raw-to-snapped start connection is outside the returned global
path, but current endpoint resolution checks it under the configured geometry
and motion limits. The benchmark also bounds the snap distance. A standalone
verifier needs the original start as its planning anchor to check that entry.
No physical motion is authorized by these tests.

## Raw-dataset native mapping and route inspection

The native `ltlio_replay` tool can export the existing SLAM save snapshot after
replay. It uses `ISlamBackend::saveMap`, including PCD, keyframe patches, scan
origins, trajectories and the completeness manifest. It does not accumulate a
second map. Existing positional invocations and optional recorded events remain
supported:

```powershell
build/slam-core-windows-x64/Release/ltlio_replay.exe INPUT.ltu CONFIG.yaml OUTPUT_PREFIX --save-map NEW_MAP_DIRECTORY
build/maps-windows/Release/lingtu-mapctl.exe build CASE --map-root MAP_ROOT --resolution 0.10 --support-dilation-cells 0 --free-layers-above 0 --free-dilation-cells 0 --frame odom --slam-source ltlio --mapping-source dataset_replay
```

The export directory must not exist. Configure the existing native SDK runtime
PATH before launch. Save failure returns nonzero; successful export does not
replace the final SLAM tracking verdict. Preserve all recorded failure results.
Dataset bag conversion is offline input preparation, not a ROS runtime or a
product Velodyne driver. Use dataset-specific time semantics and calibration.

`render_mapping.py` reads the actual saved PCD and estimated IMU TUM trajectory.
It writes separate three-dimensional, top-down and side-view PNG/PDF figures in
Arial 10 pt, plus source counts/bounds and rendering metadata. Large binary PCDs
are read with a memory map and displayed using a bounded sample. Optional
`--focus-trajectory` adds independent viewports at the trajectory XYZ bounds plus
5 m; it does not change the PCD, exaggerate Z, or overwrite the full figures.

```powershell
.venv/Scripts/python.exe tools/bench/render_mapping.py --map-dir build/dataset-mapping-20261021/maps/geode --trajectory build/dataset-mapping-20261021/runs/geode/replay.map.tum --output-dir NEW_FIGURE_DIRECTORY --title "GEODE Stairs Alpha - native LTLIO" --focus-trajectory
```

`--planning-result RESULT.json` draws only a real successful/reached-goal path.
Drawing it does not qualify the path for motion. Keep the native route, sweep and
motion-diagnostic results with the figure. The 2026-10-21 experiment used complete
Corridor02 and GEODE LiDAR/IMU inputs, native saved-ray OctoMaps and fixed native
planning requests. Three routes returned `empty_path`; one short GEODE route
passed route/sweep checks but failed three slope diagnostics. Maps had no loop
graph correction. Source snapshots, commands, logs, conditional reference
comparisons and interactive inspection are recorded in
[the local experiment](../../build/dataset-mapping-20261021/README.md).
This is offline component evidence, not Product simulation or field execution.

## Saved-map global-to-local component experiment

`saved_route.py` freezes existing saved-ray `.ot` maps, fixed requests and the
final native environment resolved from the real Go2 `nav` RunPlan. It does not
start ProductControl. The opt-in C++ `benchmark_saved_route` target uses the
shared `PlannerSession`, SCAN, Follower and Executor. Feedback starts at the
original requested pose, never at a snapped waypoint. It integrates only body
XY/yaw velocities; the requested initial Z stays constant. This is not physics,
sensor feedback, the final motion arbiter or a driver test.

```powershell
python tools/bench/saved_route.py prepare --out NEW_EVIDENCE_DIRECTORY
# After building the frozen native target, run under Linux with a new label:
python tools/bench/saved_route.py run --out NEW_EVIDENCE_DIRECTORY --case geode --variant product --seconds 20 --label NEW_LABEL
python tools/bench/saved_route.py summarize --out NEW_EVIDENCE_DIRECTORY
```

Build `benchmark_saved_route` with both `LINGTU_NAV_CPP_BUILD_BENCHMARKS=ON` and
`LINGTU_NAV_CPP_BUILD_SAVED_ROUTE_BENCHMARK=ON`; the latter keeps the OctoMap
dependency opt-in. The wrapper's exit status is not the result: read each
`summary.json`. Native exit 0 only says the first local plan was ready. Report
goal state, collisions, active tracking, cancellation and request-to-route offset
separately. Inactive default tracking errors must remain unmeasured, not zero.

The [2026-10-21 experiment](../../build/nav-dataset-execution-20261021/README.md)
retains maps, requests, dirty-source snapshot, binary, build logs and five runs.
GEODE's standard-fixture runs reach the ideal planar acceptance condition;
Garage's Product run remains locally blocked. The diagnostic Garage run can
track in XY but starts at a snapped Z and does not reach the goal in 20 seconds.
The former overinflated GEODE fixture is retained as a failed experiment, not a
production SCAN defect. The native static fixture preserves occupied/free/unknown
evidence but does not reproduce live mapd integration.

The [2026-10-22 envelope regression](../../build/nav-envelope-20261022/README.md)
retains these maps and requested endpoints. The resolved SCAN combination now
requires a global radius of at least local cylinder radius plus offset, with
vertical clearance rounded up to the local collision grid. Global endpoint
resolution checks both the actual start and its connection to the first route
point. The feedback record includes `feedback_initial_position`,
`feedback_initialized_at_request_start` and `route_start_offset_m`; a globally
rejected request has no local-plan or tracking result. The older five runs above
remain historical evidence of the previous configuration and snapped-start
fixture, not current acceptance results.

## Body envelope and heading comparison

`body_envelope.py` uses the existing MuJoCo optional environment to compare
the configured all-heading circle, the heading-aligned twin-cylinder shape,
and the installed-mount Go2 URDF's 34 collision primitives. It preserves the
original collision transforms and nominal joint assignments. The experimental
search is XY + yaw at a fixed body height with full 3D obstacle queries, not
the production OctoPlanner or SCAN search and not locomotion simulation.

```powershell
.venv/Scripts/python.exe tools/bench/body_envelope.py --out NEW_OUTPUT_DIRECTORY
.venv/Scripts/python.exe -m pytest tests/tools/test_body_model.py tests/tools/test_body_envelope.py -q
```

Outputs include a review page, exact requests/poses/results, Arial 10 pt PNG/PDF
figures and a turning-sweep GIF. Positive separations are conservative bounds
from per-primitive AABBs when disjoint; overlapping bounds use MuJoCo narrow
phase. Returned paths include intermediate translation/rotation checks with a
bounded sampling margin. Visual mesh is not used as collision geometry.

The [2026-10-22 comparison](../../build/body-envelope-20261022/README.md) records
narrow-door positives, too-narrow negatives, a colliding rotation between free
endpoint headings, an articulated posture probe, and the original Garage
occupied-leaf witness. Nominal joints, missing external D435i geometry, and
experimental margins preclude treating these as field acceptance. See the
[production integration steps](../../docs/design/navigation-body-model-and-motion-posture.md).

## Camera-LiDAR optimizer

`camera_lidar_optimizer_abi_smoke.py` checks the optional Rust optimizer C ABI.
It is separate from the navigation benchmark.

## Geometry-semantic PCD view

Build a reproducible geometry-semantic scene from a binary PCD. The command
detects dominant horizontal surface levels and writes a compact scene graph;
it does not infer object names without RGB/RGB-D evidence:

```powershell
.venv/Scripts/python.exe tools/bench/semantic_pcd_scene.py `
  sim/evaluation/data/octoplanner3d/building2_9.pcd `
  --output-dir build/semantic-building2-9
```

The output is an inspection and planning-support layer. Native occupancy,
collision geometry, saved-map activation, and motion remain owned by the
native map/navigation contracts.

Global route validation follows the current 26-neighbor planning lattice: each
vertex must pass the actual-world planner query and each edge must connect adjacent
planning cells. In supported body mode the Z search step can be finer than the
OctoMap resolution. `route_valid`, `sweep_valid`, and `motion_diagnostic_valid`
are all required for a positive benchmark verdict. The verifier checks every
segment and retains separate metric motion
diagnostics in `motion_diagnostic_valid` and `motion_failures`; the aggregate
`valid` stays false when these diagnostics fail. `sweep_failures` records actual
rejected samples, while metric-only failures have no sample (`at: null`).
These results remain in `verifier.stdout.json` and must not be reported as SCAN
or field acceptance.
A planner that snaps its start must pass the original request pose to
`octoplanner3d_verify_path --planning-anchor x y z`; deriving the lattice
offset from the first snapped waypoint can change a boundary voxel query.
Search and verifier use the same metric slope/step limits and world-coordinate
sweep. The Z refinement does not change the unknown-space policy; negative
obstacle/support cases remain required. A passing route still does not qualify
SCAN execution or field locomotion.

## Text query slice: “帮我找一瓶水”

`water_bottle_slice.py` is the first paper-aligned vertical slice for an
open-vocabulary object request. It follows the OVI-MAP ownership boundary:
the text becomes a canonical label, the label must match a persisted semantic
instance, that instance must have map-frame geometry, and only then are
standoff poses sent to the native planner in read-only preview mode.

```powershell
$env:PYTHONPATH = "src"
python tools/bench/water_bottle_slice.py `
  build/datasets/replicacad/lingtu_replicacad.json `
  --output build/datasets/replicacad/water_bottle_slice.json
```

The current official ReplicaCAD `apt_0` graph contains `cup`, `box`, `table`
and other labels, but no `water_bottle`, `water bottle`, `bottle of water`, or
`水` instance. The command therefore returns `target_unavailable` and emits
no approach pose. This is a data result, not a planner failure. A small
contract fixture with explicit `semantic_evidence=["water bottle"]` produces
eight geometry-backed standoff hypotheses; when those hypotheses are passed
through the current floor-sampled map, the native planner admits one discrete
route but the independent swept-route gate returns `verifier_blocked`. The
retained fixture is labelled synthetic and is not evidence that a physical
water bottle was seen.

Render the visual trace, including the map, target instance, candidate poses,
planner preview cursor and the final verifier gate:

```powershell
python tools/bench/render_water_bottle_video.py `
  --map-image build/datasets/replicacad/lingtu_replicacad_topdown.png `
  --slice-report build/datasets/replicacad/water_bottle_contract_slice_start.json `
  --output artifacts/water-bottle-search-demo-20261005/water_bottle_search_diagnostic.mp4 `
  --report artifacts/water-bottle-search-demo-20261005/report.json `
  --cover artifacts/water-bottle-search-demo-20261005/cover.png
```

This is a diagnostic simulation visualization. The dashed route is the native
planner's returned preview; it is not an executed robot trajectory. The video
report records `robot_motion_executed=false` and `motion_authorization=false`.

## Saved-cloud WebGL drawing

`node tools/bench/cloud-render-web.mjs` serves
`http://127.0.0.1:5190/tests/cloud-render-bench.html` using the installed web
dependencies. It proxies the local preview service on port 4326 and reads its
`test6_cleaned` PCD without editing map data. Add `?power=high-performance` to
request that WebGL context preference; inspect the returned GPU name because
the browser may still choose integrated graphics.

The page compares the former fragment height-color shader, the current shader,
and an opaque diagnostic on the same full cloud and twelve interleaved views.
It checks submitted point counts, GL errors, nonempty pixels, and old/new pixel
differences. GPU timer queries and CPU plus pixel-readback timings are reported
separately. The fixed 800x600/DPR-1 PCD-layer benchmark is **not** production
interaction FPS. Results overwrite
`artifacts/web/cloud-performance-20261110/result.json`; retain each run before
changing the browser or GPU. No sampling mode is enabled in production.
