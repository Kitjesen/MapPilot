# OctoPlanner3D Multifloor Occupancy Notes

Date: 2026-07-04

## Current Product Verdict (2026-07-16)

Multifloor navigation is **not yet a closed-loop product capability**. The
saved-map/OctoPlanner3D side can produce XYZ routes over supported geometry,
but the downstream execution chain still contains planar assumptions:

- native local target selection and goal arrival primarily compare XY;
- the sampled-path local planner receives a planar target and a 2 cm XY
  correspondence map;
- PathFollower tracks XY lookahead and heading;
- occupancy, elevation, and traversability local products collapse evidence
  into robot-centric 2D/2.5D grids;
- the Web path renderer currently presents global/local paths at display
  heights rather than using their full Z geometry.

Consequently, two floors sharing the same XY can alias, a same-XY goal on the
wrong floor can be declared reached, and a vertical connector can be skipped
by planar lookahead. Existing global-route and kinematic diagnostics do not
prove ONNX locomotion, local obstacle avoidance, or arrival on another floor.

The required product model is:

```text
per-floor metric map + support-surface/local cost layer
  <-> typed connector portal (stairs/ramp/elevator, entry and exit XYZ/yaw)
  <-> building topology graph
```

Mission execution must select a floor/room first, navigate to the connector,
execute the connector transition, verify floor/map localization, and only then
dispatch the target-floor metric goal. A single XY-flattened building map is
not an acceptable substitute.

## Corrected Finding

The original OctoPlanner3D/jie_3d_nav planner is not a free-space "fly in Z"
planner. It expands 26-neighbor grid moves, but every neighbor must first pass
`isCellTraversable(...)`, including ground-support checks. The That-nav
`go2w_nav_ws` launch/map-package runtime uses these practical planner
parameters:

- `robot_radius = 0.25`
- `require_ground_support = true`
- `strict_direct_ground_support = false`
- `ground_support_xy_radius_cells = 1`
- `ground_support_depth_cells = 1`

The earlier diagnosis that OctoPlanner3D itself was freely climbing through air
was too broad. The more likely issue was stale or mixed visualization artifacts,
or a route feasibility plot not produced from the same OctoPlanner3D map/options
used by the real headless planner.

The old `building2_9` route image mixed two different map products:

- PCT `tomogram.pickle`: thin tomography slices for legacy PCT planning.
- OctoPlanner3D `building2_9.pcd` / OctoMap: full 3D occupancy input.

Using the thin tomogram as a visual explanation for a full 3D route hides the
real planner state. Do not treat that figure as proof that the native
OctoPlanner3D planner is flying through open air.

## 2026-07-07 Multifloor Stair Finding

The current MuJoCo three-floor stair test exposed a different problem:
increasing point-cloud density alone does not fix stair routing quality.

DimOS' Go2 navigation notes are a useful comparison point. That stack does not
use OctoMap as the primary planning substrate. It keeps a live 5 cm sparse voxel
hash map, clears stale observations by column carving, derives a terrain/cost
map from height change, and runs replanning A* on the cost layer. The important
lesson for LingTu is the map semantics, not the exact dependency:

```text
LiDAR frame
  -> live voxel map with stale-column clearing
  -> support / height / traversability cost layer
  -> replanning global/local planner
```

Our current saved-map gate still does this:

```text
synthetic/static scene
  -> map.pcd
  -> octomap.ot occupied voxels
  -> OctoPlanner3D 3D A*
```

That is acceptable for static obstacle occupancy, but it is not sufficient for
legged multifloor stair navigation by itself. A raw occupied tree does not know
which occupied voxels are stair treads, risers, rails, floor slabs, side walls,
or safe landing support. When those semantics are conflated, a valid-looking 3D
A* path can cut around a stair edge or appear to climb through a side face.

Latest MuJoCo evidence:

| Resolution | PCD points | OctoMap occupied voxels | Result |
| --- | ---: | ---: | --- |
| 0.09 m | 73,968 | 15,662 | Passes current OctoPlanner3D gate after support/landing fixes |
| 0.075 m | 105,904 | 23,669 | Headless OctoPlanner3D timed out after 90 s |
| 0.06 m | 197,377 | 36,313 | Headless OctoPlanner3D timed out after 90 s |

This means the short-term fix is not "make the point cloud denser." Denser
OctoMap input improves visual fidelity but explodes the 3D A* search space in
the current implementation.

Required product direction:

- Keep `octomap.ot` as the saved static 3D occupancy artifact for collision and
  map-package compatibility.
- Add a derived `support_surface` or `traversability_surface` artifact that
  classifies floor, tread, ramp, landing, obstacle, riser, wall, and unknown.
- Let OctoPlanner3D consume the occupancy tree plus that support/cost layer,
  instead of searching all free 3D cells equally.
- Use live registered cloud/traversability as an overlay for local replanning
  and dynamic obstacle avoidance; do not bake moving obstacles into the saved
  OctoMap.

For stair scenes, the validation artifact must show both:

- body/global path at robot reference height;
- nearest support-surface projection, so a body-height path is not mistaken for
  foot contact or "flying."

## Rule Going Forward

For robot-executable multifloor planning, the search graph must remain a
traversable support graph, not the whole free-space volume.

Required constraints:

- Target cells need ground support, preferably directly below the body center.
- A* expansion must call `isCellTraversable(...)` before accepting a neighbor.
- Segment step height and optional slope should be bounded by runtime options.
- Cross-floor routes must pass through ramps, stairs, or other connected support
  geometry; they must not connect floors through open air.

## Code State

The current native/headless adapter is aligned with the That-nav launch/map
package support model:

- `robot_radius = 0.25`
- `strict_direct_ground_support = false`
- `ground_support_xy_radius_cells = 1`
- `ground_support_depth_cells = 1`
- `max_step_height = 0.45`
- `max_slope = 0.0`

The C++ planner still uses the original shape:

- `make26Directions()` generates 26-neighbor moves.
- `startPlan()` rejects each neighbor unless `isCellTraversable(...)` passes.
- `isMotionAllowed(...)` applies only additional step/slope filtering.

## Flow Status

What is proven now:

- `building2_9.pcd` on the S100P board is accepted as planner input.
- The headless runtime converts `.pcd` to an internal OctoMap when built with
  PCL.
- OctoPlanner3D produces same-floor and cross-floor `path` arrays from that map.
- The route artifact records `map_source`, `options`, `start`, `goal`, and
  `path`, so the figure can be traced back to one planner run.

What this does not prove by itself:

- Localization is aligned to the same saved map.
- Local planner and PathFollower can track the global path from the live pose.
- CmdVelMux and the driver have accepted and executed motion.

So the current algorithm can do the global 3D routing we need. Full robot use
still requires the runtime chain:

```text
saved map.pcd -> pcd_to_octomap -> OctoPlanner3D path
  -> local planner -> PathFollower -> CmdVelMux -> Driver
```

The active product no-motion gate is stricter than the direct global-planner
smoke: `/api/v1/maps/{name}/validate_plan` uses the current live robot pose as
the planning start. If live pose is not in the same saved-map frame, the map
artifact can pass and the planner can still correctly reject the request.

Current diagnosis contract:

- `map_plan_ok=true`: saved map artifacts and the live start/goal produced a
  path.
- `start_occupied_or_out_of_map`: OctoPlanner3D rejected the current start.
- `pose_map_mismatch`: the rejected start/goal coordinates are far apart enough
  to indicate the live pose or requested goal is not in the active map frame.
- `saved_map_relocalization_missing`: native DDS localization has loaded a
  saved map but has not completed a saved-map relocalization, so navigation must
  not accept a goal yet.

That means "global algorithm works" and "field closed loop is ready" are not the
same claim. The former is proven by the PCD/OctoMap route smoke; the latter
requires `validate_plan`, local planning, PathFollower, mux, and driver evidence.

## PCD, OctoMap, And JSON

`.pcd` is the saved point-cloud map. OctoMap does not make the PCD disappear;
the native headless runtime converts the PCD into an internal `.ot/.bt` OctoMap
before planning. Passing a `.pcd` directly to LingTu is supported only when the
headless binary was built with PCL.

JSON is only the process protocol and artifact format:

- Request JSON: map path, start, goal, and planner options.
- Response JSON: `ok`, `path`, `reached_goal`, and diagnostics.
- Route artifact JSON: saved response plus provenance for tests and figures.

The actual trajectory for downstream navigation is the `path` field:

```json
[[x0, y0, z0], [x1, y1, z1], ...]
```

## Plotting Standard

Do not use PCT tomograms for full 3D route figures.

Use:

- Map background: real PCD or OctoMap occupancy.
- Color: height Z or occupancy state.
- Route: OctoPlanner3D route JSON.
- Diagnostics: `z_range`, `max_segment_dz`, `max_segment_slope`.

Current figure command:

```powershell
python sim\planning\octoplanner3d_route_viz.py --route-names cross_floor_up,mid_to_top --max-points 220000
```

Current output:

```text
artifacts/generated_route_cloud/building2_9_pcd_octoplanner3d_routes.png
```

## Validation Commands

Treat the S100P board as the acceptance build host. Local Windows/WSL builds are
useful only for syntax checks and quick iteration; they do not prove the real
PCL/OctoMap artifact path.

```powershell
$env:PYTHONPATH='D:\inovxio\brain\lingtu\src;D:\inovxio\brain\lingtu'
python -m pytest sim\tests\test_octoplanner3d_route_viz.py src\runtime\tests\test_planner_runtime_profile.py src\runtime\tests\test_thunder_product_blueprints.py src\nav\tests\planning_backends\test_octoplanner3d_backend.py src\nav\tests\test_path_feasibility.py -q -k "route_viz or thunder_field_navigation_resolves_octoplanner3d_without_fallback_profile or thunder_product_configs_lock_core_runtime_modes or plan_uses_headless_executable_json_protocol or configurable_in_payload or z_excursion or headless_wrapper_is_a_non_ros2_octoplanner3d_core_adapter or path_feasibility"
```

```powershell
bash scripts/build/build_octoplanner3d.sh
bash -lc 'cmake --build build/octoplanner3d_headless --target octoplanner3d_no_air_climb_smoke -j 2 && ctest --test-dir build/octoplanner3d_headless --output-on-failure'
```

For real PCD map artifacts, the converter must be built with PCL:

```powershell
bash scripts/build/build_octoplanner3d.sh --require-pcl
```

If this fails with `PCL common/io/octree was not found`, the host can still build
the `.bt` headless planner but cannot validate the full `.pcd -> OctoMap`
artifact path.

## Board Result 2026-07-04

Historical host: `sunrise@<historical-lab-host>` (the live endpoint is not
committed to public documentation).

- `bash scripts/build/build_octoplanner3d.sh --require-pcl`: passed.
- Built `octoplanner3d_headless`, `octoplanner3d_edit_octomap`, and
  `octoplanner3d_pcd_to_octomap`.
- CTest after building both smoke targets: `2/2` passed.
- `octoplanner3d_no_air_climb_smoke`: same-floor supported route passed;
  unsupported air goal was rejected.
- Direct dense PCD smoke through the real converter and headless planner:
  `pcd_conversion=true`, `path_points=16`, `goal_error_m=0.1`.
- Board pytest/pluggy mismatch was fixed by disabling the unused anyio pytest
  plugin in pytest config and making `pytest_ignore_collect` choose the hook
  signature supported by the installed pytest version.
- Focused board pytest:
  `11 passed, 41 deselected`.
- Replanned `building2_9.pcd` with That-nav launch/map-package parameters:
  `ground_short` and `top_level` passed with `z_range=0`; `cross_floor_up` and
  `cross_floor_down` passed with 220 points, `z_range=13.4m`,
  `max_segment_dz=0.2m`, and `max_segment_slope=1.0`.
- Active saved map `accept_ready_20260702_162847` passed artifact gate and
  localization was reporting ready, but product `validate_plan` failed because
  the live start was `[219143.115717, 421310.983082, 2678.665038]` for a
  near-origin goal. That is a frame/pose alignment failure, not proof that
  `.pcd -> OctoMap -> path` is broken.
- Raw SLAM status at that moment had `mode=localization`, `map_loaded=true`,
  `relocalization_state=idle`, `relocalization_quality=-1`, and identity
  `map_odom_tf`. Navigation readiness now reports
  `saved_map_relocalization_missing` for this state.
- After `lingtu nav global-relocalize accept_ready_20260702_162847`,
  relocalization returned `native_global_relocalized` with quality `0.015041`.
  The live preview start moved from UTM-like coordinates to map-frame origin
  scale (`x≈0.004`, `y≈-0.105`), and
  `/api/v1/maps/accept_ready_20260702_162847/validate_plan` passed for goals
  `[0.5, 0.3, 0.3]`, `[0.5, 0.3, 0.0]`, and `[0.8, 0.3, 0.3]` with
  `map_plan_ok=true`.

Next real validation is to rebuild the C++ headless planner, regenerate the
`building2_9` route JSON from the same OctoPlanner3D map/options, and redraw the
figure. Old PCT/tomogram route figures should be treated as stale unless their
map source, route JSON, and planner options are all recorded together.
