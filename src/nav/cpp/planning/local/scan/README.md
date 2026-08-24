# SCAN backend

This directory contains LingTu's ROS-free C++17 port of the core algorithm in
[wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner), inspected at
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` (Apache-2.0). It follows the
paper's planning flow rather than copying its ROS process shell.

No ROS, PCL, OpenCV, catkin, runtime manager, controller, or map owner is
copied here. Field `mapd` stays authoritative and publishes the complete
bounded collision snapshot on `/maps/local_collision`; SCAN builds only a
synchronous query grid for the current `Planner::plan()` or `planIntent()` call.

- `grid.*`: robot-centred 3D occupancy, source-voxel rasterization and
  inflation, preservation of global-route 3D bends, and yaw-aware
  twin-cylinder collision queries.
- `search.*`: per-colliding-segment projected A*. Search expands only XY; Z is
  interpolated from the segment endpoints so the robot cannot escape by
  climbing over an obstacle. A one-voxel virtual boundary is used only to find
  a real in-map fallback endpoint.
- `uniform.*`: upstream cubic uniform B-spline parameterization, de Boor
  evaluation, derivatives, and component-wise feasibility ratio without the
  upstream Eigen dependency.
- `optimizer.*`: upstream jerk, rebound-distance, and feasibility objectives,
  fixed endpoint controls, XY-only L-BFGS variables, and collision-checked
  restart. A lightweight guide-point term preserves the shape of LingTu's
  supplied global route.
- `spline.*`: upstream interval initialization, boundary derivatives,
  feasibility time scaling, acceleration-bounded chord subdivision for
  collision post-checks, and timed position/velocity/acceleration samples.
- `backend.*`: continuity reuse and conversion to the public body-frame result.

The common `tracking/follower.*` and `navigation/navigator.*` implement the
paper controller's heading gate: trajectory time and translation freeze while
the robot rotates into the planned heading. CMU geometry paths continue through
the same public follower interface.

The external seam remains `planning/local/planner.hpp`; these files are
implementation details of the `scan` backend.

## Selection

`CMU` remains the field default. Select SCAN in the Product source of
truth with:

```yaml
native_nav:
  local_planner: scan
```

Product compilation emits `LINGTU_NAV_LOCAL_PLANNER_BACKEND=scan`; `navd`
parses the same value, records it in its status snapshot, and requires the
readiness value to match the RunPlan. SCAN configuration and recovery do not
load or call the CMU path library: `Navigator` owns one backend-neutral recovery
state machine in `navigation/recovery.*` after the selected Planner returns.
SCAN implements assisted-teleop `planIntent()` from the same straight intent
route used by the common Planner contract. It applies the requested speed and
terminal direction limit, produces a timed trajectory, and never invokes CMU.
The endpoint supplies the same collision snapshot, input identity, measured
body motion, and traversability contract used by autonomous planning.

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
- the route guide term prevents two different global routes with the same local
  endpoint from collapsing into one plan;
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
