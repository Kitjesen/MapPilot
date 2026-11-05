# SCAN-Planner upstream

## Autonomous retry restoration, 2026-11-05

Reference: `wuyi2121/SCAN-Planner@348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
`GEN_NEW_TRAJ` / `REPLAN_TRAJ` count failed planning rounds, including a blocked
local target; successful rounds reset the count. At 1000 consecutive failures
the FSM enters emergency stop, waits for measured 3-D speed below 0.1 m/s,
then clears the autonomous target and waits for a new request. Multiple solver
initializations within one replan round do not count as multiple failures.

Executor's autonomous SCAN temporary Recovery goals, direct recovery rotations
and independent stall/retry loop have been removed. `NoPath` uses the existing
native terminal-stop transaction instead of triggering a global replan.
CMU and held teleop-intent recovery are unchanged. The November 4 Recovery
translation integration below is historical for autonomous SCAN.

FSM and collision periods remain 0.01 s and 0.05 s; the follower runs at 100 Hz.
The MuJoCo launcher now uses the resolved native frequency instead of silently
overriding it with 20 Hz. Native asynchronous handoff, reference continuation
and final measured-body/braking checks remain explicit local adaptations.
Final-control trajectory resets are not solver failures; the counter does
not bound every possible hold or guarantee a ten-second task timeout.
Matched evidence: `build/scan-retry-20261105/report.md`.

## Planning collision restoration, 2026-11-05

Reference: `wuyi2121/SCAN-Planner@348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
Candidate queries again use front/rear centers in the inflated 3-D map;
control-point yaw and Projected DynAStar yaw come from adjacent positions.
Rebound/refinement use the upstream leading-two-thirds sampler and chord-based
step. FSM lookahead uses upstream position/chord queries. No candidate query
uses multipart turn sweeps, controller preview yaw or predicted obstacle volumes.

The removed preview/sweep changes are recorded below as history, not current
behavior. The shared `headingDirection()` helper remains only in the controller.
The native connector and asynchronous handoff still use cylinder checks; native
command/braking checks keep measured Body geometry and predictions. Ordinary
Body/support metadata no longer enables exact-boundary fitting. Explicit heading
and operator intent contracts retain their local behavior.

Platform dimensions, map inflation, controller preview/gains, native transport,
reference initialization and the existing solver-result repairs are preserved.
This is not a byte-identical replacement of the ROS planner or a claim that
upstream's collision sampling guarantees continuous physical clearance.
Matched evidence: `build/scan-native-restore-20261104/report.md`.

## Rebound guidance and exhausted continuations, 2026-11-04

The preview-heading A* change in the next paragraph was reverted on 2026-11-05.
The reference-joining and exhausted-tail repairs remain.

Rebound A* now receives the candidate's preview heading for each collision
segment. Previously its eight XY neighbor directions selected a different
footprint: a captured guide contained five occupied points out of nine when
queried with the candidate heading. The search still has XYZ nodes, without a
yaw search dimension, and the complete optimized curve retains its changing-yaw
swept collision validation. The optional search argument leaves other A* callers
on their existing tangent-heading behavior. The pinned upstream uses its
twin-cylinder model; this aligns LingTu's compound-body guide and validation.

The reference initializer joins the measured start and selected target without
spreading their offsets across the entire remaining reference. Spreading a
10 cm start-height difference lowered later stair points as well. This changes
LingTu's reference-seed extension, not the upstream optimizer's Z constraint.

The native Backend may retain a clear executing tail while continuation planning
fails. It now drops that tail once its execution time is exhausted and reports
Blocked, allowing SCAN to retry. Previously it could return
Ready indefinitely while the follower reported finished far from the task goal.
The regression also checks obstacle removal resumes planning and that an actual
completed reference does not spuriously start a new trajectory. These changes
do not establish successful physical stair navigation.

## Recovery result ownership, 2026-11-04

LingTu's asynchronous Task previously invalidated cached failures for body and
heading changes but not a new reference generation or intent. Recovery could
therefore count the previous target's failure against a new candidate before
its SCAN result arrived. The Task now uses its existing guide-identity comparison
for that cache. Executing trajectory handoff semantics are unchanged.

Recovery translation submits a finite `RouteTarget`, retaining the selected body
heading and speed cap. The previous `MotionIntentTarget` also requested rolling
operator-motion endpoint semantics. A regression separates a blocked original
goal from a reachable temporary goal and checks that the latter waits for its
own result, reaches its endpoint and has zero terminal velocity. Executor tests
cover temporary-route continuity, cancellation and return to the original route.
Both Recovery and this asynchronous cache are LingTu extensions to the pinned
SCAN implementation below; no upstream collision or braking threshold changes.

## Candidate heading matches execution preview, 2026-11-04 (reverted)

This records the superseded experiment; candidate preview propagation and
its cache field were removed on 2026-11-05.

The pinned controller's `estimateDesiredYaw()` uses the position at
`min(duration, time + time_forward)` minus the current spline position,
falling back to velocity below 1 cm of XY displacement, then measured yaw
below 0.01 m/s XY speed. Its launch parameter is `time_forward=0.8`.
LingTu's complete-trajectory collision validation instead used instantaneous
velocity heading. A captured stair approach with measured velocity
`[-0.0738,-0.0196,0.0382]` and yaw `0.0405` therefore checked a 167-degree turn
while the same curve's 0.8 s controller preview requested only a 7-degree
correction. The capture is a failure near a side obstacle before the first
stair, not proof that the stair is traversable.

`UniformBspline::headingDirection()` now supplies the same preview/fallback
calculation to the controller and complete candidate, refinement, FSM lookahead,
handoff and deceleration collision checks. The request carries the actual
Follower preview time; explicit tangent pose stages retain zero preview.
Changing that value invalidates asynchronous results checked under the old
heading policy. Rebound control-point queries now also estimate heading from
that preview. The knot interval is supplied before building rebound guides;
adjacent control-point headings are interpolated during segment sampling.
In a captured stair approach, tangent-only seeding found no obstacle guide,
while the execution preview swept `FR_thigh_11` into the first railing post.
Using the same preview produced a guide and a collision-free optimized
candidate for that capture. This does not establish physical stair traversal.
Measured-to-required turning sweeps, Body dimensions, occupied evidence,
timed predictions and final braking checks remain in their existing owners.

The native regressions cover a brief reverse entrance with a forward preview,
a genuinely occupied turning sweep, preview clamping and fallback at the
spline endpoint, and asynchronous invalidation after a preview-time change.
This is an intentional collision-candidate change in the LingTu adapter,
not an output-parity claim for the pinned upstream or physical stair validation.

- Repository: https://github.com/wuyi2121/SCAN-Planner
- Commit: `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`
- License: Apache-2.0
- License copy: [`LICENSE`](LICENSE)

The local `plan_manage/` directory retains the upstream planning-management
module layout. It contains adapted algorithm sources, not an untouched copy.
On 2026-10-27, the port's `scan_replan_fsm.{h,cpp}` / `SCANReplanFSM` became
[`planner_fsm.{h,cpp}`](plan_manage/planner_fsm.h) / `PlannerFsm` to describe its
full trajectory lifecycle. The corresponding pinned upstream implementation
remains `src/planner/plan_manage/src/scan_replan_fsm.cpp`. Only the local file
and class names changed; the six states, timer periods and behavior did not.

Process and storage adaptations have a parity contract: with the same valid
sensor/odometry/reference events, effective parameters and callback order,
they preserve inflated occupancy queries, DynAStar path, optimized control
points, B-spline, FSM transitions and closed-loop velocity command. The
correctness repairs and intentional planning, collision and execution extensions
listed below are explicit exceptions. Changes
to accepted collision candidates must not be described as unconditional
output equivalence to the pinned commit.

The deployed `teleop_avoid` path also includes intentional LingTu behavior:
operator direction becomes a reference corridor, side-target selection respects
the operator direction bound, assisted translation retains body heading, and
native final arbitration checks braking motion against the live grid. These
are not upstream navigation modes or an unconditional parity claim. Hardware
clearance and command caps are resolved from the active RunPlan/RobotConfig;
matching the upstream algorithm does not imply matching its effective parameters.

LingTu's terminal and experimental stair turns are separate from the pinned upstream
trajectory controller. The local turn query checks the body sweep, while final
control checks actual and candidate braking with the configured deceleration.
The former extra turn-braking query used fixed `0.1 / 0.5 / 1.0` values; it
could reject a turn already permitted by final control. The matched native
`testTurnUsesFinalConfiguredBraking` fixture reproduces that rejection before
the change and confirms final braking still limits the same moving obstacle
when configured to require it afterward. This is not a physical braking test.

On 2026-11-02 navd stopped attaching recognized stair segments and publishing
stair-profile requests. The retained prototype is not an upstream SCAN feature
or a validated deployed stair workflow. The native endpoint boundary test checks
that route activation and profile publication stay disconnected; ordinary goal
and local-planner tests exercise the remaining path. This is not a physical test.

LingTu recovery rotation and CMU recovery-translation follower parameters are
owned by Follower. Autonomous SCAN translation recovery uses the ordinary spline
follower as described in the 2026-11-03 entry below. These recovery controls are
LingTu extensions to the pinned upstream controller.

## Upstream terrain capability and LingTu extensions

The [official paper, page 7](https://arxiv.org/pdf/2606.19555v1#page=7)
reports stair-obstacle simulation and a Go2 field route across three floors
(149 m, 251 s). The pinned implementation plans three-dimensional position
B-splines, supplies a height reference, checks yaw-dependent twin-cylinder
occupancy, and outputs body-frame `vx`, `vy` and `wz`. It does not plan individual
footholds or require all four frozen feet to satisfy exact contact at every
candidate. A velocity-command interface therefore does not itself exclude the
upstream terrain-following approach.

LingTu adds fixed body-heading stages, multipart collision queries and final
braking prediction. Without original terrain data the braking model holds current
Z. With a complete original-hit snapshot it can predict base height relative to
an observed local slope, starting at the measured pose; each predicted body and
contact still passes the 3-D map query. This does not predict joint motion or
prove gait capability. The formal live grid does not yet supply that raw snapshot.
`compilePoseRoute()` retains separate translation/yaw as its fallback. Continuous
level turns can now use the actual multipart body and checked tangent sweeps,
including short legs and several opposite-curvature corners. Height-changing
legs and declared stairs keep separate execution. These are local extensions,
not pinned SCAN behavior. The recorded fixed-body 4.57-degree ramp failure
occurred in global planning before SCAN ran.

The former LingTu whole-body floor-column gate was removed on 2026-10-26.
Pinned upstream SCAN checks the front/rear centers against inflated occupancy;
it does not require occupied floor plus immediately free space below every body
part projection. Local Grid no longer derives such admission from route
`supportHeightM` metadata. The later per-foot observed-ground admission and
`support_required` role were also removed; `support_contact` retains only a
geometric bottom-contact rule. Global 3-D terrain checks remain in their owner;
no new gate or feature flag replaces these local deletions.
This removal does not fix the independent nominal-foot/ground-voxel ambiguity
or qualify the formal product's physical execution.

## Bounded velocity initialization extension

`planner_manager.cpp::boundedVelocitySeed` joins acceleration, cruise and
braking polynomials for both RouteTarget and MotionIntent. Its three-dimensional
boundary velocities and configured velocity, acceleration and jerk limits set
the ramps. This avoids repeatedly restarting a slow single polynomial over an
entire long, low-speed route. Short maneuvers, nonzero initial acceleration, or
boundary velocities above the current limit retain the original initializer.

This is a deliberate LingTu trajectory change, not an output-equivalent port
optimization. The existing optimization objective, Z-reference processing,
replanning FSM and three-dimensional collision queries remain in use.
Ordinary RouteTarget without a fixed body model still uses its original
least-squares boundary fit and feasibility tolerances. Explicit body pose
segments use the exact-boundary extension described below. Neither mode gains a
strict final command limit. FinalControl and the driver retain final speed
limits. `ScanSplineVelocityBounds.LowSpeedLongRouteAcceleratesBeforeStallWindow`
compares low-speed entrances in both directions, including moving boundaries;
the existing SCAN collision and Executor regressions check the affected chain.
These are native code tests, not proof of physical stair traversal.

Stair traversal also supplies a body heading independent of the route tangent
through LingTu's `Grid` query seam and closed-loop follower. Candidate footprint
queries retain the measured-to-commanded turn; the final braking check uses
actual yaw and twist. This is an intentional LingTu footprint extension.
Ordinary RouteTarget queries and assisted MotionIntent retain their prior
heading semantics. The upstream geometry-search and optimizer algorithms are
unchanged by this adapter input.

## Fixed-joint body and exact pose-segment boundaries

LingTu's optional `LocalPlanRequest::body` changes the Grid query contract from
the upstream twin-cylinder lookup in an inflated bitmap to per-part primitive geometry checks
against the snapshot's original measured occupied and known-free cells. It
shares `collision/body` geometry with global pose planning, including margin,
contact and yaw-sweep semantics. No second occupancy map is built, and the
upstream cylinder inflation is not applied to the complete body again.
Within grid coverage, unknown air alone is not a body obstacle; measured occupied
cells still undergo geometry checks. Missing required raw storage and body
coverage outside the grid are rejected. `support_contact` allows material wholly
below the primitive's unpadded bottom; it does not infer a foothold or require
observed ground under each part. These changed
acceptance rules are a deliberate extension, not parity with the pinned query.

Formal models retain nominal joints; the endpoint applies measured body
roll/pitch and the collision Pose applies yaw once. They do not represent future
gait articulation or exact collision meshes. Dynamic
prediction cylinders use conservative obstacle AABBs, time-aligned relative
Body sweeps and prediction-horizon handling. Body dimensions are not inflated
again by the legacy cylinder radius. The no-body offline prediction path is
retained. The model cannot use the legacy padding-only departure exception to
leave a measured part collision. Braking checks retain actual pose and twist.

For requests carrying both `body` and a fixed `bodyHeading`, Backend derives
`PlanParameters::preserve_boundary_`. The manager preserves the requested
start/target P/V/A using the existing exact-boundary control-point restoration
and retiming also used by MotionIntent. This does not set `motion_intent_` and
does not change the reference FSM into the assisted-motion FSM.

The pinned reference initializer samples before the polynomial end and fits
those samples in least squares. Its fixed last three spline controls can
therefore retain a centimetre-scale endpoint error. That error is material for
a pose-route turn: the controller can finish at its spline endpoint while the
robot has not reached the collision-checked turn centre. Exact boundary
restoration fixes the requested geometry before rebound optimization; the
exact-boundary retiming path preserves it and rechecks the complete resulting
trajectory for collision. Requested zero terminal velocity remains zero.
Ordinary no-body requests retain their least-squares endpoint-boundary semantics;
their reference geometry follows the ordered-reference change below.

The specific regression is
`ScanBackend.BodyPoseSegmentsPreserveExactTurnCentreAndStoppedEndpoint`, which
includes the corner demo's first moving segment and its former short residual.
Raw-query and model-swap regressions live in `test_scan_body` and
`LocalPlanTask.BodyModelReplacementDiscardsCachedSplineWithoutNewRouteOrFrame`.
They establish native component behavior, not field deployment or gait safety.

## Global route and local collision ownership

The pinned upstream `grid_map.h::getInflateOccupancy` queries the two rotated
cylinder centres in the inflated 3D occupancy buffer. LingTu's `scan/Grid`
keeps that contract for no-body requests, continuous segment/braking checks and moving-obstacle
predictions. It does not require seven ground probes or an independently
ray-cleared support patch before the FSM can plan.

OctoPlanner owns the autonomous route's observed-ground, unknown-space,
step-height and slope checks. On 2026-11-01, the local prepared-support callback
gate was removed from SCAN trajectory, braking and boundary-departure queries.
LocalPlanRequest no longer transports that callback, and Grid no longer copies
the reference or searches its nearest height segment for support admission.
This gate was a LingTu extension absent from the pinned upstream implementation.
Route height metadata still controls the existing trajectory initialization;
measured 3D occupancy, body sweeps and prediction queries remain active.
A clear local collision query therefore does not establish ground bearing or
drop-off safety away from the accepted global route. Missing-ground fixtures
record this deliberate behavior difference; they are not physical cliff tests.

Mapd's measured occupancy and known-free evidence remain in the typed snapshot.
Removing the extra local gate does not convert unknown cells into measured
free space, change ray integration or change the DDS contract.

The corresponding upstream algorithm and its L-BFGS dependency are retained
together under this directory for:

- projected `DynAStar`;
- polynomial initialization and rebound/L-BFGS optimization;
- uniform B-spline evaluation and time reallocation;
- `PlannerManager` and the six-state replanning FSM;
- the 100 Hz FSM/controller and 20 Hz future-collision callback semantics;
- the official closed-loop controller equations.

The output-equivalent process/portability adaptations replace ROS values and
callbacks with ordinary C++ values, inject the same clock, add the LingTu
namespace, and support C++17/MSVC. They are separate from the explicit behavior
extensions and correctness repairs described in this file. Invalid transport input is
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

LingTu's moving-cluster extension is separate from that measured bitmap.
Untimed search queries conservatively use its swept region. Published spline
validation, refinement, and future-collision callbacks use
`getTrajectoryOccupancySegment()` to compare robot and obstacle at matching
times, including observation age. Prediction uses constant velocity for its
declared horizon and retains the terminal position afterward; it is not a
full space-time optimizer. Final braking checks use the same timed geometry
with reaction and deceleration time. Quiet trajectory derivatives use measured
body heading instead of amplifying nearly-zero velocity noise. Failure captures
retain prediction timing and volumes as well as the measured bitmap.

The SCAN Mapd profile disables generic time decay, so occupancy changes only
from ray evidence or rolling-window eviction. AStar node-pool reuse is likewise
an allocation optimization; node generation tags preserve the same search
state and ordering.

The local rolling grid starts unknown and consumes live `MapObservation`
scans, as the upstream cloud/pose callbacks do. Saved patches and `map.pcd`
are not replayed or forced into this grid at navigation startup. They belong
to saved-map localization and global planning. Map identity still binds live
observations to the active frame/epoch; relocalization clears the local grid.
This removes LingTu's historical seeding extension, which kept old returns in
the sensor blind region and could make every SCAN departure fail despite a
valid global route. Unknown cells retain their evidence state; collision and
braking queries are unchanged.

Thunder's measured cylinder radius/offset are explicit robot inputs. Geometry
defaults retain the port's Go2 values. LingTu's ordinary speed defaults are now
1 m/s for the planner norm and each controller axis; Product profiles and
explicit overrides remain the runtime authority. This intentionally replaces
the old 0.75/0.35 m/s tuning without changing control equations. RViz visualization, the Go2
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

## Collision sampling repair (reverted on 2026-11-05)

The following is historical. Candidate sampling has returned to upstream.

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
including points behind the robot or within the manager's former 20 cm rejection
distance. Every FSM callback then retried the same rejected target and the
adapter reported `Pending` indefinitely. Fallback searches forward from the
odometry projection. Since 2026-10-27 ordinary reference routes require only
1 mm of noncoincident progress, matching the manager instead of discarding a
short clear prefix. MotionIntent retains the 20 cm detour-candidate threshold
and actual-final-goal exception. Target occupancy is still checked. When no
usable forward target exists, the FSM reports a blocked target, the adapter
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

## Native asynchronous handoff boundary

LingTu reference-path replanning can run while the robot follows a previous
spline. When tracking remains near its reference, a replacement uses the old
spline's future P/V/A boundary,
retains the existing constrained optimization and geometry validation, and
accounts for that future time in predicted-obstacle queries. A single pending
offer is delivered before the boundary and committed only after the control
thread accepts it within a bounded deadline. Withdrawal and receipt are mutually
exclusive. Initial and paused trajectory epochs follow actual execution receipt,
not time spent waiting for the optimizer.

This is a local correction to asynchronous execution semantics. It does not
change the upstream objective into a different planner, add a 2D navigation
authority, or modify a validated spline's control points after planning. See the
parent SCAN README and the handoff regressions in `test_scan_planner.cpp`,
`test_local_plan_task.cpp` and `test_path_follower_core.cpp`.

The 2026-10-27 continuity change prepares ordinary temporary local segments
before their stopped endpoint: remaining time at most `2 * handoff lead + 3 FSM
ticks` requests replanning, unless a safe executing tail was already retained.
The old final-10-ms continuation remains a fallback. The lead comes from the
existing solve/transfer estimate; this does not add another Product knob.

If measured position and the current spline position differ by more than
`max(ctrl_pt_dist, grid resolution)`, reference-mode replanning instead starts
from measured P/V with zero acceleration through the existing branch. It does
not impose an old nominal future P/V/A boundary on a lagging robot. The
separation condition requests computation; it does not veto motion or replace
the optimized-spline and final-control checks.

Backend and follower also retain a published spline on a lower speed request
when its derivative-control-hull upper bound fits the new cap and any valid
measured XY velocity fits. Base heave alone is not horizontal overspeed.
Geometry, knots, ID and timing remain unchanged, and
the metadata mismatch alone does not repeatedly replan. Bounds are cached per
accepted trajectory. A hull bound is sufficient and may exceed the spline's
actual maximum; exceeding the cap preserves the existing hold/replan behavior.
Measured XY overspeed also retains its hold. This is not general online retiming
or continuous braking of an already overspeed spline. These are intentional
LingTu adapter/execution changes, not upstream SCAN output equivalence.

The corresponding native evidence and limitations live in
`build/replan-continuity-20261027/README.md`. Clock-delay fixtures do not prove
contact physics, a complete Product run or field response times.

## Reference geometry and executable velocity (2026-10-23)

Reference-path fitting retains supplied XYZ bends and fills long straight edges
at the existing 0.5 m reference spacing. The former minimum-spacing filter left
sparse knots whose global polynomial anticipated distant bends across a straight
approach. The polynomial objective is unchanged: block-diagonal endpoint solves
and one reduced-system factorization replace repeated full-matrix inverses.
`test_scan_polynomial` compares P/V/A with the frozen original formula.

A moving exact-boundary replan estimates its seed duration from the incoming and
outgoing velocity instead of assuming both endpoints are stopped. Actual boundary
acceleration remains in the polynomial, and existing retiming and collision checks
still validate the result. This avoids manufacturing a cruise-speed dip solely
because the new spline has a nonzero starting acceleration.

The native adapter passes the follower's existing forward/lateral speed capability
to SCAN. When a body heading is declared, local target velocities and exact-boundary
retiming respect those axes in the same frame as execution, as well as total speed.
MotionIntent without a fixed heading uses the observed body yaw for this velocity
calculation only; it does not fix the collision heading or predict a future
operator yaw-rate trajectory. No additional product limit is configured. Requests
without declared body-frame capability retain the vector-speed contract. Final follower saturation uses one
common scale so a body-axis cap cannot rotate the feedback-corrected velocity.
The scheduled handoff and execution clock are unchanged by these corrections.

## Reference-height and numerical result repairs (2026-10-25)

The reference FSM now passes its selected 3-D projection/target interval to local
initialization. Nonlinear intermediate heights retain the reference geometry in
3-D progress order instead of being overwritten by an endpoint-only linear Z.
The prior seed supplies relative acceleration/cruise progress and total duration;
retiming, physical endpoint derivatives and complete final checks remain.
This is an intentional initialization extension, not arbitrary XYZ optimization.

Refine now synchronizes the variables actually returned by L-BFGS and rejects
nonaccepted solver errors before collision acceptance. Its previously empty
error branch could leave the last failed callback trial in the output cache.
The projected A* now keeps immutable queue scores with superseded-entry handling,
and clears the previous path at the start of a search. These repair result/state
handling and can change path ordering compared with the pinned implementation.

Unused polynomial statistics were corrected: the missing mean-speed return,
per-sample time in velocity/acceleration statistics, and full polynomial squared
acceleration integration. B-spline length includes the final partial interval.
These corrections do not change the polynomial trajectory-generation equations.
Discrete statistics retain their documented sample-time convention and are not
continuous-time maxima. Nine undefined, unused legacy optimizer declarations were
removed rather than implementing a second unused API surface.

## Signed search-coordinate repair (2026-10-26)

Projected A* now rounds signed offsets to the nearest search cell in both endpoint
conversion and interpolated Z. Adding 0.5 and truncating toward zero shifted the
negative half of the search volume by one cell; a requested (2,2) start could
become (3,3), and ascending/descending search planes could disconnect at their
negative-height endpoint. `ScanAStarCoordinates` compares physical route points
and both height signs with the requested plane; queue regressions retain an
independent shortest-path bound from the actual requested start. This is an
intentional correction to inherited search discretization, not a change to the
projected-XY search model or free-Z terrain planning.

## Ordered reference and continuous-turn execution (2026-10-26/27)

Ordinary non-MotionIntent reference initialization follows the supplied ordered
3-D polyline instead of replacing its bends with the global minimum-snap curve.
The original 2026-10-26 condition required preserved boundaries and no fixed body
yaw. The 2026-10-27 change removes those two conditions, so ordinary flat S routes
and fixed-heading routes retain the same complete XYZ reference. This also
includes eligible scheduled handoffs with preserved future boundaries.
The global polynomial could introduce a reversing prefix in short S turns.
Reference progress is monotonic and commits only after planning succeeds;
failed attempts cannot consume it. MotionIntent retains its guide/detour
semantics and can use the existing linear-height fallback.

The zero-preview tangent follower and Grid use the same 0.03 m/s quiet-speed
heading fallback. During moving tangent tracking the follower adds spline
curvature yaw-rate feedforward to the existing heading feedback and angular
limit. Quiet speed, explicit heading/rate, ordinary preview and paused heading
alignment do not gain this feedforward.

Executor may restore the original stop-turn stages only while measured motion
is stationary near the blend entrance. A mid-curve failure, manual takeover or
resume retains the current curve and goal, discards the old follower trajectory
and restarts local timing from observed motion. Restoring the original turn
centre after the robot had already moved could force an occupied heading.
The 2026-10-27 cleanup retains the analytic fillet as a curvature-conditioned
pose-route reference. It still changes the original corner geometry; this is
separate from retaining ordinary XYZ references inside the manager. Raw-only
initialization failed the wall S channel, including a variant with the physical
speed cap, and was not adopted.

Executor removes whole-candidate Grid preflight and repeated prefix shortening
before local planning. SCAN receives the reference and checks its actual
optimized spline; final measured-pose and braking queries remain. Curvature
speed uses radius times the follower's moving physical yaw-rate capability,
without additionally inheriting a literal `.6` or terminal `goal_yaw_max_rate`.
Terminal alignment retains its own limit. The redundant reference-heading
array is removed; Grid and the zero-preview follower use the spline tangent.
Height metadata, forward-yaw compatibility, explicit stairs and stationary
entrance fallback remain. The inherited optimizer's no-progress restart behavior
is not fixed or counted as a deleted LingTu restriction in this change.

SCAN endpoint readiness also stops requiring an unused registered point cloud
when fresh mapd collision is its authoritative obstacle input. CMU retains its
consumed cloud requirement. This belongs to the endpoint, not the upstream
optimizer.

These are intentional LingTu execution changes. Native regressions and matched
software-loop records, including rejected experiments, live in
`build/motion-cleanup-20261027/README.md`. Ideal speed integration does not
establish contact physics, gait capability or field behavior; XYZ derivative
feasibility alone does not establish joint yaw-rate timing feasibility.

## Startup and recovery adaptations, 2026-10-27

Pinned SCAN already freezes translation while its heading error exceeds the
controller threshold, and its emergency recovery tests `odom_vel_.norm() < 0.1`.
The latter is intentionally adapted to finite XY speed for LingTu's planar
velocity-command interface: vertical balancing/stair motion alone does not mean
horizontal motion is still active. Three-dimensional trajectory velocity and
acceleration checks remain unchanged.

The local target occupancy query now uses the target's reference tangent, or a
nearby reference difference at zero tangent, rather than the chord from odometry
to that point. This changes the body orientation used for target selection;
the optimizer's final candidate sweep remains in place.

Candidate-validation and FSM lookahead passes keep the last reliable reference
tangent through consecutive quiet-speed tail samples. They do not invent a
return to the request's initial body yaw during deceleration. Without any prior
reliable tangent, quiet motion retains the measured heading. These vectors
select collision orientation only; trajectory derivatives and feasibility
limits are unchanged, and Grid has no mutable cross-query heading history.

LingTu's fixed-heading adapter checks initial alignment at the measured robot
position, then queries future geometry at the requested heading. It does not
require that initial turn again inside every future segment, or at a later timed
visit to the same position. Static and predicted occupancy, multipart and
twin-cylinder geometry share this rule. Actual-pose braking remains independent.

Executor pose-stage transitions similarly use finite XY speed for standstill.
The already aligned initial pose may hand off to translation while moving;
intermediate turn centers and actual height/yaw requirements are retained.
These are deliberate port/adapter repairs, not claims that upstream lacks any
stop policy or that the resulting motion has been qualified on hardware.
See `build/startup-cleanup-20261027/README.md` for executed regressions and limits.

## Continuous corners and speed transitions, 2026-10-27

LingTu's pose-route compiler now accepts sampled forward XYZ+yaw edges whose
headings agree with travel within the route tolerance. Compatible unblended
corners share their original XYZ reference instead of being local stopping
endpoints. This uses the existing SCAN tangent follower and complete spline
checks. Genuine in-place turns remain separate. Arbitrary independent heading
profiles and mixed explicit stair/pose routes are not added by this change.

Runtime speed reductions have a separate, cheap cubic transition before normal
SCAN optimization. Its boundary comes from observed P/XY velocity and previous
reference derivatives; acceleration/jerk bounds and complete timed collision
queries apply. The finite prefix can exceed the new cruise cap while reaching
it. Initial positive acceleration can briefly increase speed; it is not an
instant cap or a claim of monotonically decreasing speed at every instant.
Normal SCAN feasibility is unchanged and resumes after the transition boundary.
The prefix has an ordinary physical timestamp/knots, with no warped clock.

Task prioritizes the latest lower cap and withdraws stale unreceived results.
An optimizer already running is not preempted inside its iteration. The follower
freezes an invalid old reference while decelerating from the applied command;
late or failed replacement can still end at a stop. Explicit stops and final
body/braking arbitration remain authoritative. See
`build/continuous-motion-20261027/README.md` for actual verification and limits.

## Dense reference boundary timing, 2026-10-31

Pinned upstream `plan_manage/src/planner_manager.cpp`,
`SCANPlannerManager::planGlobalTrajWaypoints()` assigns each interval as
distance / max_vel and doubles the first and last intervals. LingTu previously
retained that rule after adapting dense OctoPlanner routes. A measured start to
route projection can introduce a legitimate 0.164 mm connector before the
normal 10 cm edges. With a resting boundary and a 1 m/s cap, its 0.327 ms
interval forced the minimum-snap reference far off the guide: the recorded
detour stayed at y <= 0.0026 m but the fitted reference reached y = 1.5643 m.

Waypoint timing now uses forward acceleration and backward braking speed
envelopes along 3-D arc length, followed by triangular/trapezoidal speed
integration for each interval. Existing start/end derivatives and every bend
are retained. The original upstream interval remains the lower bound, so
ordinary reference timing is unchanged when it already permits acceleration
and braking. There is no minimum waypoint spacing or fixed per-point delay.
This timing is a LingTu adaptation. It does not itself constrain every
instantaneous derivative of the fitted polynomial; SCAN's optimized spline
still undergoes its existing dynamic and collision checks.

The recorded route is retained by
`ScanReplanFsm.SubmillimeterStartConnectorKeepsTheMeasuredDetourForward`.
Before/after evidence and the subsequent physical execution remain under
`build/goal-acceptance-20261031/` and `build/navigation-repair-20261031/`.

## Terminal heading during position correction, 2026-10-31

The independent terminal-yaw branch belongs to LingTu's Executor; it is not a
condition required by the pinned SCAN FSM. Physical floor-dropout execution
revealed a conflict at the 15 cm arrival boundary: yaw alignment drifted just
outside that radius, restarting SCAN with the cruise heading, which turned the
base back against the requested final yaw.

Executor now remembers entry into terminal orientation/settling and retains
that heading while SCAN corrects the same goal's position. Position, height,
yaw, quiet-odometry and collision requirements are unchanged; drifting outside
the arrival region still prevents success. Existing stair-flight heading
priority is retained. The phase resets with the existing autonomy execution
reset, including new routes, clear and explicit resume/reconnect, while a
temporary motion pause retains it. The Map/Odom regression checks actual yaw
error and command direction, including initially aligned arrival and explicit
new-route/resume cleanup.

## Clearance-only body departure, 2026-10-31

This section describes the implementation at that date. The local shared-support
callback mentioned below was removed on 2026-11-01; the obstacle and prediction
checks described here are separate.

The pinned upstream has no LingTu multipart Body or final-command braking
adapter. A captured physical detour stopped with its source primitives clear
but its 8 cm horizontal clearance overlapping a measured box hit. The local
Body departure implementation required the same padded sweep as normal motion,
so even a clear reverse exit was rejected at its first pose. Removing a fixed
heading did not resolve this captured failure.

The existing bounded, fixed-yaw recovery departure now distinguishes the
source primitives from clearance. It requires their full 3-D continuous sweep
to be clear, the endpoint to restore normal clearance, and the padded sweep
to encounter no occupied cells beyond those overlapping the initial buffer.
Prediction and support checks remain active. Only the temporary candidate view
excludes the initial cells; normal collision/braking queries and the original
observed bitmap are unchanged. The existing final-command MotionFree check
validates a complete 35 cm exit and its reaction/braking distance each cycle,
allowing a checked prefix without requiring an instantaneous jump out of the
buffer. This is a LingTu recovery policy, not an upstream SCAN default.

Review then identified that the existing quiet-yaw allowance still permits
nonzero measured yaw. A straight exit cannot authorize that measured braking
arc. MotionFree now checks command and measured braking against both source
primitives with original evidence and full clearance with only initial buffer
cells excluded. New obstacles and timed predictions retain full clearance.
The exit-distance bound uses the same coupled linear/angular stopping time as
the braking model, with the final-command caller's actual yaw deceleration.
Braking segments also query an existing support callback when supplied; absent
support callbacks retain their previous behavior. The quiet-yaw allowance and
normal robot dimensions/margins are unchanged.

The initial buffer-cell set now uses the same maximum half-spacing sampling
coverage as continuous sweeps, avoiding a new cell caused solely by the first
midpoint's sampling allowance. The ordinary start-state query is unchanged;
a clear start cannot use this exclusion. Source-body sweeps, normal-clearance
endpoints, new cells, predictions and support remain checked. This cell-based
departure policy does not guarantee strictly monotonic obstacle distance.

## Ordinary XYZ heading follows the local spline, 2026-10-31

The pinned upstream controller's `estimateDesiredYaw()` uses a future spline
point, falling back to the spline velocity, then measured yaw when stationary.
Its `advanced_param.xml` sets `closed_loop_controller/time_forward` to 0.8 s.
LingTu had additionally overridden ordinary XYZ requests with the heading of
the nearest global polyline segment whenever a Body was configured. On a
voxel-grid zigzag in XY, that heading alternated between 45 and 0 degrees
while the local spline was smooth. Changing this request field invalidated
the active task/backend trajectory and could repeatedly freeze translation
for heading alignment. The earlier projection-connector fix still retained
this global-segment override and therefore did not solve that behavior.

Ordinary XYZ requests now leave `bodyHeading` unset and use the existing
spline-heading controller. Explicit pose stages, stair-flight headings and
the latched terminal-yaw correction retain their respective overrides. No
heading thresholds, controller gains, terminal minimum yaw rate, body margins
or recovery limits change. Grid still checks measured start yaw and subsequent
trajectory-tangent sweeps; measured final-command braking remains in place.

The matched grid-corner regression compares identical guides with and without
the Body extension: the old implementation replaced a smooth spline heading
and discarded its trajectory at the next horizontal segment. The revised test
requires the original trajectory to remain active across these segment changes.
The measured micro-connector, explicit terminal/stair cases and real occupied
corner checks remain separate regressions. An occupied in-place turn is still
rejected; a request may succeed only if its returned moving curve is clear.

## Single alignment threshold and declared-pause resume, 2026-10-31

The pinned upstream controller resumes translation whenever its yaw error is
within `dir_diff_thre`, configured as 0.8 rad. LingTu had added a separate
0.35-rad release threshold from the geometric Path follower, and the nav Product
overrode entry to 0.4 rad for an earlier fused-turn experiment. After ordinary
XYZ routes stopped overriding the spline heading, R7 still recorded unnecessary
stop/align episodes. SCAN now uses the upstream single threshold; the nav Product
uses the existing 0.8-rad default. The geometric CMU follower retains its own
hysteresis. Explicit pose, stair and terminal headings are unchanged.

LingTu's command-rate limiter also gave zero acceleration budget on the first
tick after a declared long pause. R7 recorded this even with fresh inputs, an
active spline and no alignment freeze. Such a pause now resumes with one
`nominalDt` command budget, while the full pause is still removed from reference
time. Unannounced gaps and backward time retain zero budget. This removes an
extra zero-command tick without spending the paused duration on acceleration.

Regressions exercise 0.81-to-0.79 rad release, independence from geometric Path
settings, paused replacement reference time, bounded first-frame linear/angular
commands, and unchanged unknown-gap/CMU behavior. Runtime input freshness and
actual-body braking checks remain separate requirements. New physical results
and the remaining map-publication delay are recorded under
`build/navigation-stop-review-20261031/`.

## Executor progress during an empty planning wait, 2026-10-31

The pinned upstream FSM retries failed replans and regenerates after an
emergency stop; it does not contain LingTu's `execution_stalled` recovery
monitor. R9 recorded this local monitor interrupting an accepted, moving
spline after a previous 1.74-second `Pending` wait without an active curve.
It had accumulated the last nonzero command and elapsed time during the wait.

An empty `Pending` wait now pauses the monitor's observation time without
resetting its earlier requested/observed progress. SCAN already returns
`Ready` when an active safe curve remains, including during replanning; such
execution continues to be monitored. `Blocked`, near-field holds and actual
motion without measured progress retain their existing recovery behavior.
The regression first establishes cruise, waits without a local trajectory,
then advances on a new spline and finally verifies that a genuine stall still
stops. Existing intermittent near-field and input-pause tests remain separate.

## Terminal arrival without radius truncation, 2026-10-31

The pinned official `closed_loop_controller.cpp` (lines 204-223) continues its
reference until execution reaches the spline duration and the position error is
within `finish_dist`; it does not contain LingTu's task arrival-radius branch.
LingTu's Executor had reset a moving follower as soon as the body entered the
task radius. R10 reached that radius with about 0.85 m/s applied command, before
the terminal spline's deceleration had completed, and then issued zero.

Executor now lets an active SCAN spline continue while its applied-command
baseline exceeds the existing 0.03 m/s arrival quiet bound. Task settling and
separate terminal yaw still use the unchanged goal tolerances after that
deceleration. Initially stationary goals, CMU behavior and urgent safety stops
are unchanged. The odom-frame reference uses the real final route segment when
its terminal connector would otherwise collapse to one point. This is a local
task-layer repair, not a change to the upstream optimizer or finish distance.

The real SCAN command-integration regression covers ordinary and explicit-yaw
routes plus a separate final pose rotation, with 0.15 m and 0.6 m task radii.
It requires bounded deceleration after radius entry and eventual goal/yaw
completion. The original R10 records and before/after software logs remain in
`build/navigation-stop-review-20261031/`; physical validation is separate.


## Autonomous translation recovery through the normal task, 2026-11-03

The pinned SCAN revision remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
Its FSM regenerates trajectories after planning failure or emergency stop.
LingTu's bounded recovery session, temporary goals and retry order are local
product policy.

Previously, autonomous recovery built another Grid, searched a geometric path
and sent it directly to a separate path follower. SCAN now receives each
translation candidate as a temporary MotionIntent guide and complete reference,
with a stable generation and fixed starting body heading. The existing Task,
FSM and spline follower own planning, live collision updates and execution.
Selecting a temporary goal does not authorize motion. Only a ready SCAN result
marks the recovery output verified. Measured arrival resumes the retained Route;
a rejected plan or the existing no-progress timeout advances recovery policy.

Rotation retains the existing sweep checks and turning command. CMU retains
its geometric path recovery. Cancellation, replacement and manual takeover use
the existing task cleanup. No upstream thresholds or dependency changes are
introduced by this wiring. Matching Executor regressions use the same free
collision bitmap to compare ordinary planning with an induced final-control
block, verify trajectory reuse across ticks, measured completion, and pending
candidate cancellation. Native test and physical-trial results are recorded
separately from these implementation changes.


## Task speed updates through the normal command API, 2026-11-03

Pinned SCAN revision `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` has no
LingTu task-scoped HTTP/DDS command interface. `TaskSpeedLimit = 10` is a local
product command. Previously the normal autonomous command path could set a
limit only by submitting a goal, which starts a new global planning task.

The new command changes the retained active task limit and Executor's limit
without replacing its route or restarting SCAN. The existing Backend speed
transition remains responsible for a checked deceleration and spline handover.
An already-running global replan takes the latest retained task limit when its
result activates. Task/request correlation, terminal barriers and configured
maximum speed apply at the native command boundary.

GoalPlan, ingress and terminal regressions cover task identity, no route reset,
invalid limits, retry semantics and an update during an in-flight global replan.
Client tests cover the C++ and C DDS payload/ACK roundtrip. These establish the
software command chain. Physical speed-transition evidence is recorded separately.

## Final command deadband and the active speed budget, 2026-11-04

Pinned SCAN revision `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` has no
LingTu typed-DDS final-command safety chain. Its minimum-motion deadband is
local product policy. A positive deadband above the current single-axis budget
previously erased every legal axis command, including a lower active-task cap.

Final command safety now applies the configured deadband only while it fits
inside both the current path-norm and body-axis budgets. The configuration is
retained, including an explicitly higher deadband. Only its application in the
current control cycle is skipped when it exceeds that budget. Existing maximum
limits and stale-command handling continue to apply.

The matching `test_teleop_safety` and `test_autonomy_tick_controller` regressions
exercise low body-axis budgets, low path-norm budgets, an active-task speed cap,
and a smaller configured deadband. Both added cases failed against the prior
implementation. Physical motion validation is recorded separately.
