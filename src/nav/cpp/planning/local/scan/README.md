# Full SCAN local planner

This directory contains LingTu's ROS-free C++17 port of the complete local
planning pipeline in
[wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner), inspected at
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` (Apache-2.0). It follows the
official initialization, projected-search, rebound, time-reallocation, refine,
and dynamic-feasibility flow. “Complete” here refers to those local-planning
stages; it does not claim to copy the upstream ROS FSM, map process, or
robot-specific controller. Those responsibilities stay behind LingTu's runtime
boundaries.

No ROS, PCL, OpenCV, catkin, runtime manager, controller, or map owner is
copied here. Field `mapd` stays authoritative and publishes the complete
bounded collision snapshot on `/maps/local_collision`; SCAN builds only a
synchronous query grid for the current `Planner::plan(LocalPlanRequest)` call.
When obstacle checking is enabled, a missing, incomplete, stale, or uncovered
Mapd collision snapshot fails the plan closed; SCAN never falls back to the raw
registered-cloud point list.

- `grid.*`: robot-centred 3D occupancy, source-voxel rasterization and
  inflation, preservation of global-route 3D bends, and yaw-aware
  twin-cylinder collision queries.
- `search.*`: per-colliding-segment projected A*. Search expands only XY; Z is
  interpolated from the segment endpoints so the robot cannot escape by
  climbing over an obstacle. A one-voxel virtual boundary is used only to find
  a real in-map fallback endpoint.
- `seed.*`: previous-trajectory reuse, a bounded route-shape `Guide` seed,
  official one-segment quintic initialization, deterministic random
  intermediate-point minimum-snap retry, endpoint derivatives, and a
  route-following Z reference.
- `anchors.*`: official closest-two-thirds collision segmentation, one
  projected A* detour per collision segment, segment bounds, tangent-plane
  intersections, obstacle base/direction anchors, short-segment handling, and
  anchor propagation.
- `../../../trajectory/spline.*`: the shared upstream cubic uniform B-spline
  parameterization, exact de Boor evaluation, derivatives, and component-wise
  feasibility ratio without the upstream Eigen dependency. Planner and tracker
  use this single implementation.
- `optimizer.*`: upstream rebound objective (jerk, anchor distance, and
  feasibility), separate refine objective (jerk, trajectory fitness, and
  feasibility), fixed endpoint controls, full Z-gradient suppression, and
  in-optimizer collision rebound plus post-check restarts with retained anchors
  and doubled collision weight. The rebound objective contains no non-upstream
  route-attraction term.
- `spline.*`: complete orchestration of initialization, rebound, up to three
  restarts, feasibility-driven knot-time reallocation, B-spline
  reparameterization, refine, continuous collision post-checks, vector-norm
  dynamic rejection, and timed position/velocity/acceleration samples.
- `backend.*`: continuity reuse and conversion to the public body-frame result.

The common `tracking/follower.*` and `navigation/executor.*` implement the
paper controller's exact B-spline position/velocity feedback and heading gate:
trajectory time and translation freeze while the robot rotates into the
planned heading. Sampled trajectory points are telemetry only. CMU geometry
paths continue through the same public follower interface.

`native_nav.scan_follower` exposes the upstream controller's eight parameters
(`time_forward_s`, heading threshold, position/yaw gains, three velocity limits,
and finish distance) through RunPlan into `navd`. The defaults match upstream
`advanced_param.xml`. The controller runs at the configured `navd` tick rate;
the current Product default is 20 Hz, while upstream launches its standalone
ROS controller at 100 Hz. Raising the field rate requires an S100P timing gate,
not a silent Planner or Follower constant change.

The external seam remains `planning/local/planner.hpp`; these files are
implementation details of the `scan` backend.

## Selection

`CMU` remains the default backend of the single `nav` Product. SCAN is selected
as a local-planner implementation, not as another Product:

```text
python -m lingtu.control switch nav --local-planner scan ...
```

The equivalent compiler API is `compile_run_plan("nav", ..., local_planner="scan")`.
Product compilation emits `LINGTU_NAV_LOCAL_PLANNER_BACKEND=scan`; `navd`
parses the same value, records it in its status snapshot, and requires the
readiness value to match the RunPlan. The MuJoCo catalog keeps CMU and SCAN
targets under `acceptance.products.nav.local_planners`. SCAN configuration and recovery do not
load or call the CMU path library: `Executor` owns one backend-neutral recovery
state machine in `navigation/recovery.*` after the selected Planner returns.
Assisted teleoperation uses the same `Planner::plan(LocalPlanRequest)` entry as
route navigation, with `LocalObjective` carrying a `MotionIntent` instead of a
`RouteTarget`. SCAN applies the requested speed and terminal direction limit,
produces an exact executable B-spline plus timed telemetry samples, and never
invokes CMU.
The endpoint supplies the same collision snapshot, input identity, measured
body motion, and traversability contract used by autonomous planning.

Selecting SCAN also configures `mapd` with the official rolling-map profile:
`0.05 m` voxels, `200 x 200 x 100` cells (`10 x 10 x 5 m`), a `0.2 m`
recenter threshold, a `5.0 m` ray limit, and occupancy probabilities
`p_hit=0.85`, `p_miss=0.30`, `p_min=0.12`, `p_max=0.98`, and `p_occ=0.80`.
These values are emitted in the immutable RunPlan, so map construction and the
planner use the same resolution. Mapd's status snapshot reports the effective
resolution, XYZ dimensions, ray limit, and collision-point cap. The CMU
selection keeps its existing map profile.

The one-cell virtual layer is a paper-specified hypothesis mechanism rather
than code copied from the public upstream repository. It may help search reach
the real rolling-map edge, but no returned path or spline sample may lie in that
virtual layer.

`complete=true` proves that no occupied voxel inside the advertised AABB was
omitted; the sparse wire does not distinguish observed-free from unknown.
Unknown-space admission therefore remains a separate traversability/global-map
and final-safety policy and must not be inferred from this collision topic.

## LingTu integration boundaries

The following are intentional adaptations, not missing SCAN stages:

- `mapd` replaces the upstream ROS/PCL grid-map owner and supplies complete
  occupied voxel centres over DDS;
- OctoPlanner/FAR supplies the global route; SCAN receives the route shape, not
  only its final point;
- a bounded `Guide` seed preserves different global-route shapes that share the
  same local endpoint, while the official rebound objective remains unchanged;
- the twin-cylinder body envelope comes from RobotConfig (or a geometry-derived
  simulation default) and is shared by RunPlan, endpoint, and Planner;
- endpoint lifecycle, readiness, command arbitration, and final robot velocity
  safety remain owned by LingTu;
- no virtual boundary point can enter the executable path or trajectory.

## Performance gate

`benchmark_scan_planner` submits an 8,135-voxel complete mapd-style collision
snapshot with a vertical obstacle band that blocks the supplied route. Every
measured frame therefore runs projected A*, rebound L-BFGS, and continuous
spline collision checks; it does not use the legacy obstacle-cloud adapter.

```powershell
cmake -S src/nav/cpp -B build/nav-scan-bench-msvc `
  -G "Visual Studio 17 2022" -A x64 `
  -DLINGTU_NAV_CPP_BUILD_TESTS=OFF `
  -DLINGTU_NAV_CPP_BUILD_BENCHMARKS=ON
cmake --build build/nav-scan-bench-msvc --config Release `
  --target benchmark_scan_planner
build/nav-scan-bench-msvc/Release/benchmark_scan_planner.exe 200 8000
```

Local workstation output is diagnostic only and is not field evidence. Before
selecting `scan` in a field Product, run the same binary for 200 frames on the
S100P and record the result under `docs/07-testing/`; the acceptance gates are
p95 <= 50 ms, max <= 100 ms, and 100% planning success for each fixed scenario.

The 2026-08-25 Windows x64 Release diagnostic was repeated twice after the
row-interval inflation optimization. Both runs completed 200/200 plans and
200/200 projected-A* executions. The two `(p50, p95, max)` results were
`(27.988, 37.280, 43.361) ms` and `(28.467, 37.723, 89.038) ms`.
