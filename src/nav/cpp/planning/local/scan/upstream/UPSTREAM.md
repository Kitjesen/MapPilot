# SCAN-Planner upstream

- Repository: https://github.com/wuyi2121/SCAN-Planner
- Commit: `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`
- License: Apache-2.0
- License copy: [`LICENSE`](LICENSE)

Process and storage adaptations have a parity contract: with the same valid
sensor/odometry/reference events, effective parameters and callback order,
they preserve inflated occupancy queries, DynAStar path, optimized control
points, B-spline, FSM transitions and closed-loop velocity command. The
inherited correctness repairs listed below are explicit exceptions. Changes
to accepted collision candidates must not be described as unconditional
output equivalence to the pinned commit.

The deployed `teleop_avoid` path also includes intentional LingTu behavior:
operator direction becomes a reference corridor, side-target selection respects
the operator direction bound, assisted translation retains body heading, and
native final arbitration checks braking motion against the live grid. These
are not upstream navigation modes or an unconditional parity claim. Hardware
clearance and command caps are resolved from the active RunPlan/RobotConfig;
matching the upstream algorithm does not imply matching its effective parameters.

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

## Inherited undefined-state repair

The pinned optimizer shares an intersection-success index across successive
control points in both `initControlPoints` and
`check_collision_and_rebound`. A point without an intersection can therefore
read an uninitialized intersection left outside its valid branch. Marking a
point before a constraint is actually appended also allows propagation to
read an empty constraint vector.

The local repair separates the current point's intersection result from the
segment's propagation seed, and marks a point only after its base point and
direction are stored. This is a documented correction to inherited undefined
behavior, for which there is no defined upstream output to preserve. It does
not change intersection geometry, search cost, optimization terms, or the
controller equations on the successful path.

## Collision sampling repair

The pinned rebound/refine acceptance checks choose their time step from the
trajectory's endpoint chord. A slow detour can therefore have widely spaced
samples and miss occupied cells between them, even in the checked first two
thirds. A 10 ms cap alone also missed a shorter grazing interval in the
translated-wall regression. The local checks retain finer steps and cap the
interval at the FSM's existing 10 ms, then traverse the grid cells crossed by
each front/rear cylinder center between samples. Cylinder headings use the
spline's velocity tangent. Rebound, refine and FSM future-collision checks
share this segment query over Mapd's existing bitmap. A detected collision
uses the existing rejection/reoptimization path; it does not introduce a
second map owner or a separate command-safety planner.

This changes acceptance of candidates missed by the upstream sampler. The
sampled cylinder-center curves are approximated by line segments; this is not
a proof of continuous collision freedom for the physical robot.

## Blocked local target repair

An occupied lookahead target previously fell back to any free reference point,
including points behind the robot or within the manager's 20 cm rejection
distance. Every FSM callback then retried the same rejected target and the
adapter reported `Pending` indefinitely. Fallback now searches forward from the
odometry projection and requires at least 20 cm of progress, except at the actual
final goal. When none exists, the FSM reports a blocked target, the adapter
invalidates its active trajectory, and retries resume from observed motion when
the map or reference permits progress. The manager allows short final approaches
and rejects only coincident endpoints. These are intentional target-selection
and failure-reporting corrections; they do not change the optimizer objective.
Reference-mode target callbacks also retain the original endpoint and polynomial
duration: temporary occupancy is handled by local target selection. Truncating
the reference to the fallback point made a later map update unable to recover
the requested goal.

When initial trajectory generation has actually failed, the adapter reports
`Blocked / scan_initialization_failed` until a valid trajectory or replacement
target exists. Timer callbacks without a new attempt retain that failure.
`Pending` is reserved for work that has not produced a result. Existing safe
trajectories remain available during ordinary replanning failures.

## Reference endpoint completion repair

In reference-path mode, expiration of a local spline previously cleared the
entire target even when a short horizon or collision fallback ended far before
the reference endpoint. The adapter then kept returning that finished spline
while the navigation task stayed active with zero output. At expiration, the
FSM now compares odometry with the retained reference endpoint, using the
existing no-replan distance. An unreached endpoint starts another trajectory
from observed motion; an actually reached endpoint still enters WAIT_TARGET.
This intentionally changes reference-mode completion, without altering preset
waypoint sequencing, collision acceptance, or the optimizer objective.
