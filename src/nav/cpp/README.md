# LingTu Native Navigation

This directory is the C++ source and CMake entry point for LingTu navigation.
It contains portable planning, tracking, and route-execution code together with
the native CycloneDDS endpoint that runs those algorithms in a Product.

## Ownership

This tree owns:

- global-route planner adapters;
- local planning with the CMU and SCAN backends;
- path and exact B-spline tracking;
- route execution and local recovery;
- native navigation clients, bindings, and the `navd` endpoint.

It does not own SLAM, live map state, traversability generation, Product
lifecycle, or robot actuation. Those remain with their native processes and
connect to `navd` through typed DDS. Inside `navd`, planner and controller calls
are direct C++ calls rather than additional processes or DDS nodes.

## Runtime flow

```text
route generation (event-driven)
  goal + localization + validated saved map
                  |
                  v
          Global Planner
                  |
          verified map-frame route

route execution (continuous)
  admitted route -----------+
  assisted-teleop intent ---+--> Executor
  odometry -----------------+        |
                                      v
                            Local Planner (CMU/SCAN)
                         + collision/traversability evidence
                                      |
                            path or exact B-spline
                                      |
                                      v
                                  Follower
                                      |
                            pre-safety body Twist
                                      |
                                      v
                     endpoint safety + motion authority
                                      |
                                      v
                                /nav/cmd_vel
                                      |
                                      v
                                 driver -> robot
```

`Executor` does not compute a global route and does not publish a robot
command. It executes an admitted route or assisted-teleop intent through local
planning and tracking. `Follower` produces body-frame velocity intent
(`vx`, `vy`, `wz`), not joint positions, wheel speeds, or motor torques.

Global and local planning are separate contracts with different rates and map
ownership. Global planning runs when a goal needs a route through the saved
map. Local planning runs continuously while Executor executes that route, or
directly from an assisted-teleop intent that requires no global route. They may
run in the same native process and use direct C++ calls; separation does not
require an extra DDS boundary.

## Source layout

| Directory | Responsibility |
| --- | --- |
| `planning/global/` | Global planner contract and OctoPlanner3D/FAR adapters |
| `planning/local/` | Backend-neutral local planner plus CMU, SCAN, and candidate recovery search |
| `planning/rolling/` | Portable observed-free rolling-map prefix planning and revalidation |
| `trajectory/` | Shared uniform B-spline representation and exact evaluation |
| `tracking/` | Geometric-path and exact B-spline tracking; bounded velocity shaping |
| `navigation/` | Route slicing, frame conversion, recovery lifecycle, Planner-to-Follower sequencing |
| `endpoint/` | DDS, readiness, goal lifecycle, command boundary, authority, and process lifecycle; `RollingSegmentEffectCoordinator` applies transport-free rolling effects |
| `client/` | Native client and C ABI for endpoint commands/status |
| `bindings/` | Nanobind access to portable native kernels used by focused development tools and tests |
| `platform/` | Small platform-specific runtime utilities |
| `tests/` | Portable algorithm, endpoint-contract, and performance checks |

The dependency direction is:

```text
endpoint -> navigation -> planning + tracking -> trajectory + portable nav types
client   -> endpoint message contracts
bindings -> portable planning/tracking APIs
```

Planning code must not depend on DDS or Product lifecycle. Tracking code must
not select planners. Endpoint code orchestrates transport and safety but must
not contain planner or follower algorithms.

## Core interfaces

### Global planning

[`planning/global/contract.hpp`](planning/global/contract.hpp) defines the
backend-neutral request and result contract. The normal field backend is
OctoPlanner3D; FAR is optional. Map artifact validation and asynchronous goal
identity checks belong to `endpoint/nav/input/active/` and
`endpoint/nav/runtime/goal/`, not to the planner interface.

### Local planning

[`planning/local/planner.hpp`](planning/local/planner.hpp) exposes one
`local::Planner` configured with one backend.

Both backends consume the same `LocalPlanRequest`, including the objective,
robot state, local environment, identity, and clock. Their executable targets
differ:

| Backend | Main output | Execution model |
| --- | --- | --- |
| CMU | Body-relative `PathTarget` | Follower selects a lookahead point and computes velocity online |
| SCAN | Exact body-relative `SplineTarget` | Follower evaluates the B-spline at the execution clock |

SCAN preview geometry is sampled from the spline on demand; it is not a second
executable result. Assisted teleop uses the selected backend as well: CMU
returns a geometric path and SCAN returns an exact B-spline, without switching
between them implicitly. See
[`planning/local/scan/README.md`](planning/local/scan/README.md).

### Tracking

[`tracking/follower.hpp`](tracking/follower.hpp) exposes the `Follower` control
boundary. Callers submit either a geometric path or an exact B-spline through
the same `follow` concept. Algorithm state remains private to `Follower`.

The functional result is a pre-safety body-frame `Twist`. Heading error,
distance-to-end, turn scaling, and acceleration decisions are controller
diagnostics; they are not alternative robot command outputs.

### Navigation execution

[`navigation/executor.hpp`](navigation/executor.hpp) exposes:

- `setRoute()` and `clear()` for an admitted global route;
- one `tick(const ExecutionInput&)` entry for route execution or
  local-planner-assisted teleoperation;
- one `ExecutionOutput` containing local telemetry and pre-safety velocity.

Normal autonomy always uses local planning before following. Goal completion,
terminal yaw alignment, verified recovery rotation, and fail-closed stopping are
the intentional exceptions. See
[`navigation/README.md`](navigation/README.md).

## Product and Host boundary

Both `real` and `sim` Products select the native navigation endpoint as the
autonomy owner. The Product chain is C++
`navd -> Executor -> Planner/Follower -> command boundary -> DDS command output`.

The Python Host submits typed commands through the native client and presents
skills, status, and inspection capabilities. It does not
run a second planner, follower, safety mux, or motion writer.

## Build and test

### Windows x64

Windows navigation uses the MSVC ABI exclusively. MinGW, MSYS2 `clang64`, and
ambient Conda compilers are unsupported because they can select incompatible
runtime and GoogleTest libraries. Use the Visual Studio preset even when those
tools are present in `PATH`:

```powershell
cmake --preset windows-x64-nav-portable --fresh
cmake --build --preset windows-x64-nav-portable
ctest --preset windows-x64-nav-portable
```

### Linux x64

```bash
cmake --preset linux-x64-nav-portable
cmake --build --preset linux-x64-nav-portable
ctest --preset linux-x64-nav-portable
```

### S100P / Linux aarch64

The aarch64 preset is native-on-target; it is not a cross-compilation preset.

```bash
cmake --preset linux-aarch64-nav-portable
cmake --build --preset linux-aarch64-nav-portable
ctest --preset linux-aarch64-nav-portable
```

Endpoint dependencies, CycloneDDS SDK setup, deployment, and field validation
belong in the build and endpoint guides rather than this component overview.

## Detailed references

- [Local planning and tracking contract](../../../docs/architecture/LOCAL_PLANNING_AND_TRACKING_CONTRACT.md)
- [Architecture index](../../../docs/architecture/README.md)
- [Native build guide](../../../docs/01-getting-started/BUILD_GUIDE.md)
- [Navigation execution](navigation/README.md)
- [Native endpoint](endpoint/README.md)
- [SCAN backend](planning/local/scan/README.md)
- [OctoPlanner3D backend](planning/global/octoplanner/README.md)
