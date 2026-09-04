# SCAN-Planner upstream

- Repository: https://github.com/wuyi2121/SCAN-Planner
- Commit: `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`
- License: Apache-2.0
- License copy: [`LICENSE`](LICENSE)

The supported runtime path has a strict parity contract: with the same valid
sensor/odometry/reference events, effective parameters and callback order, it
must produce the same inflated occupancy queries, DynAStar path, optimized
control points, B-spline, FSM transitions and closed-loop velocity command as
the pinned commit. A local branch that changes one of those results is not an
allowed port change.

The corresponding upstream algorithm and its L-BFGS dependency are retained
together under this directory for:

- projected `DynAStar`;
- polynomial initialization and rebound/L-BFGS optimization;
- uniform B-spline evaluation and time reallocation;
- `PlannerManager` and the six-state replanning FSM;
- the 100 Hz FSM/controller and 20 Hz future-collision callback semantics;
- the official closed-loop controller equations.

LingTu changes at the boundary are limited to replacing ROS values and
callbacks with ordinary C++ values, injecting the same clock, adding the
LingTu namespace, and C++17/MSVC portability. Invalid transport input is
rejected before it reaches this core; parity is defined for inputs accepted by
the official SCAN callback contracts.

`plan_env/grid_map.*` is the explicit map query seam. Mapd owns the official
raycast, log-odds and incremental-inflation semantics, then exposes the same
logical inflated grid through `GridMap::getInflateOccupancy()`. Four storage
optimizations are permitted because they do not change a logical cell result:

- a ring index replaces whole-grid memory moves when the window slides;
- one-bit occupancy storage replaces one byte per inflated cell on DDS;
- wider per-frame vote counters avoid signed overflow;
- per-frame bit sets replace the upstream `char` ray tags and their wraparound.

The SCAN Mapd profile disables generic time decay, so occupancy changes only
from ray evidence or rolling-window eviction. AStar node-pool reuse is likewise
an allocation optimization; node generation tags preserve the same search
state and ordering.

Thunder's measured cylinder radius/offset are explicit robot inputs. The
algorithm defaults remain the official Go2 values. RViz visualization, the Go2
gait publisher, the kinematic simulator, the open-loop controller and the
unused gradient-descent optimizer are not part of the official closed-loop
planning path and are intentionally not built into `navd`.
