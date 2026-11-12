# SCAN-Planner upstream

## 2026-11-11: remove performance-based motion holds

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
The `LoopGuard` integration described on 2026-11-09 below has been removed,
including its admission fields and recovery wait. SCAN's 100 Hz target cadence,
planner/controller behavior and collision checks are unchanged. Loop timing is
diagnostic; it no longer independently locks motion or delays initial readiness
until 100 timing samples exist. Missing timing metrics no longer veto Gateway
readiness. The existing JSON `control_loop_hold` field is emitted as false.

In a native component replay, 200 fresh commands spaced 20 ms apart caused 101
hold decisions in the previous guard, despite zero driver watchdog stops.
The replacement test preserves the performance warning and verifies that the
driver accepts that command stream, still stops at a real 200 ms command gap,
and rejects an expired command afterward. Input freshness, final collision and
braking, operator/estop ownership, fault holds and confirmed task termination
retain their existing owners. This removes a LingTu restriction; it is not a
field qualification or a claim that upstream provides our hardware watchdog.
Evidence is retained in `artifacts/field/go2-mapping-20261111/loop-cleanup.md`.

## 2026-11-09: restore upstream inflated-voxel cylinder queries

At the user's request, the twin-cylinder hit-bound experiment was removed.
Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` queries the inflated
3-D occupancy buffer. Grid again queries that buffer at the front/rear cylinder
centers, even when mapd also supplies measured hit bounds. No switch between
the two representations remains in the twin-cylinder path.

The existing explicit multipart Body query and its measurement evidence are
separate and unchanged. The requested lateral target minimum, final command
checks and collision-to-replanning feedback also remain. This restores the
cylinder map-query representation, not a byte-identical ROS runtime.
Historical experiment results remain in `artifacts/nav/cylinder-hits-20261109/`;
restoration verification is in `artifacts/nav/cylinder-restore-20261109/`.

## 2026-11-09: configured minimum nonzero lateral target

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` clips body-axis
velocities at their maxima; it has no minimum lateral speed. At the user's
request, LingTu's Follower adapter now applies `scan_follower.min_vy_mps`
before its existing acceleration limiter and final collision check. The `nav`
Product selects 0.35 m/s from Thunder CTS39 response trials; the shared default
is zero and other Products retain it. This is local tuning, not an upstream
default or a hardware qualification.

Zero, completion and alignment targets stay zero. The floor retains direction,
respects the current lateral/total speed caps and the endpoint braking budget
`sqrt(2 * acceleration * remaining distance outside the finish tolerance)`,
and cannot override an external hold or final safety stop. Acceleration, reversal and safety limiting may still
produce commands below the target floor. It is not a minimum applied actuator
velocity. Matched tests and simulation evidence live in
`artifacts/nav/lateral-min-20261109/`; the preceding physical response study is
`artifacts/nav/lateral-response-20261109/`.

## 2026-11-09: point-goal arrival precedes the spline clock

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`. Its controller
requires both elapsed spline time and endpoint XY error for completion. LingTu's
ordinary point-goal Executor now hands off to its existing stop transaction as
soon as the requested position is inside the task's XY and height tolerances.
It no longer waits for the spline clock and then generates a short return curve
whose tangent can command a large final turn. Explicit pose stages retain their
existing timing and stair-arrival rules. The 15 cm nav position tolerance and
the controller itself are unchanged; this is not a new terminal-yaw controller.
The 2026-11-08 ordinary spline-time completion description below is superseded.
Matched native regressions, physical runs and fixed Nav2 goal-checker references
are in `artifacts/nav/arrival-fix-20261109/`.

## 2026-11-08: navigation speed and tracking

Pinned upstream remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
The `nav` Product now requests forward speed 1.2 m/s, acceleration 0.75 m/s²
and position gain 1.5, compared with the prior local 1.0 / 0.5 / 0.8.
Upstream examples use 0.75 m/s, 0.5 m/s² and gain 0.8; these are local tuning,
not upstream defaults. The 0.8 s heading preview and 0.8 rad alignment gate remain.
Real RobotConfig still bounds forward speed; the configured Thunder real limit is
unchanged at 1.0 m/s. Planner and command acceleration share the Product budget.
Final control uses SCAN's X/Y budgets for autonomous motion, retaining operator
budgets during takeover. No minimum lateral command or policy remapping is added.
Matched MuJoCo trials, the slower-turning speed-only counterexample, policy
response curves and replay videos: `artifacts/nav/speed-tracking-20261108/`.
Each candidate has one run; the evidence is component simulation with truth
localization, not NX motion qualification.

## 2026-11-08: remove local terminal-heading control (current)

Pinned upstream remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
Ordinary navigation again follows spline heading and completes from spline time
and position, without explicit final yaw, terminal minimum rate, heading
hysteresis, time/distance-based goal-yaw takeover or a second terminal controller.
`FollowerGoal`, execution `Route.finalYaw` and the `goal_follower` settings and
native bindings are deleted. Trailing yaw-only pose stages are discarded.
Intermediate explicit pose turns retain geometry checks and reuse the SCAN
proportional yaw gain and rate cap, without a separate minimum turn rate.

The native integration still applies configured acceleration limits during
normal heading alignment. The reference freezes, but translation decelerates;
external collision/emergency stops retain immediate zero. This platform command
limit is an explicit remaining adaptation, not a claim of byte-identical ROS
code. Task position/height validation and final stop acknowledgement also remain.
Matched tests and scenes: `build/scan-upstream-goal-20261108/`.

The terminal-heading entries below are historical and are superseded by this
removal; their physical results do not describe the current controller.

## Final-heading approach, 2026-11-08

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` follows spline
heading with a 0.8-second preview and has no explicit final-yaw contract.
LingTu keeps that preview for ordinary tracking. Its Follower now retains an
explicit goal yaw when the preview reaches the final spline endpoint, not only
after the robot crosses the accepted XY radius. Recorded S-turn navigation
missed that radius by 0.7 mm and then turned from about 19 to 118 degrees toward
a return spline before turning back to zero. Intermediate endpoints retain
normal tracking heading. Goal tolerances and SCAN geometry checks are unchanged.

The active final curve converges using its existing finish distance, or yields
to a measured stop inside the goal region, rather than being cut off on first
crossing the goal radius. A robot already stopped there need not be pulled back
to an older reference. A follow-up heading
trial exposed 5 mm of post-arrival position drift beyond the accepted radius
when this convergence was cut short; the trial remains in the evidence.
The same terminal update decelerates translation and controls yaw.
The terminal hold remains active inside the accepted position region; crossing
0.03 m/s no longer resumes tracking there. This replaces the separate `braking_` latch and
does not add a timer or parameter. Two new regressions fail before this repair;
the intermediate-endpoint control passes both versions. Matched evidence lives
under `build/follower-approach-20261108/`.

A repeated reference hold could also clear the spline's command history after
FinalControl had confirmed a nonzero command. The wrapper now feeds that same
confirmed value back before spline tracking; it does not introduce another
limiter. The recorded caller-pattern regression previously resumed at 0.005 m/s
from an applied 0.4 m/s despite a 0.005 m/s per-tick limit.

## Speed-transition ownership, 2026-11-08

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`; the checked
speed-transition prefix is a LingTu adaptation. Its acceptance/withdrawal is
now one `PlannerFsm::startSpeedTransition()` operation. Manager still computes
and collision-checks the curve. Backend converts the returned trajectory and
existing status instead of separately calling Manager, adopting/abandoning the
FSM trajectory, and copying the core's control points and epoch itself.
The existing bridge-rejection regression now calls this operation and verifies
that an unreceived faster offer is withdrawn without caller-side cleanup.

Backend's `speedReductionPending_` was removed: setting it true also cleared
`active_`, while both assignments to `active_` cleared the flag. The speed
reduction predicate requires that same active curve, so its extra flag guard
could not change the decision. Numeric limits, bridge duration, sampling and
stop reasons are retained. This is the first bounded ownership cleanup, not a
claim that Backend's remaining lifecycle decisions have all been removed.
See `docs/research/scan-ownership-cleanup-20261108.md` for the remaining scope.

## Manager responsibility split, 2026-11-07

`planner_manager.cpp` now coordinates an attempt and owns its state. Stateless
reference/velocity-ramp seed construction lives in `seed.cpp`; `retiming.cpp`
implements the manager's private timing, boundary and feasibility operations.
The full retiming branch has one entry, `retime()`, instead of being embedded in
`reboundReplan()`. Its failure diagnostics and collision checks are retained;
only the caller increments the existing consecutive-failure counter.

The unused `reparamBspline` duration-increase output and its two discarded local
variables were removed. No state owner, public planning API, numeric threshold,
search budget or solver objective changed. The main source went from 1,021 to
525 lines; this is responsibility separation, not a claim of halving total code.
The pinned official manager has 555 lines, but local physical-boundary and
reference adaptations still exist in their named files. All 160 SCAN tests pass
after the split. See `docs/research/scan-manager-responsibilities-20261107.md`.

## Constrained moving-boundary refinement, 2026-11-07

Boundary follow-up: the existing fixed-endpoint precondition now uses
`PlanParameters::speedRatio()` for both endpoint velocities, matching the
physical validator's norm and body-axis limits. A fixed 0.4 m/s lateral start
under a 0.35 m/s body cap previously exhausted 16 time candidates despite a
legal 1 m/s norm limit. It now rejects without stretching that immutable
boundary. This completes LingTu's body-axis adaptation; it adds no upstream
parameter or separate validation layer. Four start/end and heading cases fail
before the change and pass after; see `artifacts/nav/tora-boundaries-20261107/`.

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` refits the whole
curve when reallocating time, then applies soft smoothness, fitness and
feasibility costs. LingTu's previous exact-boundary loop instead retained the
last interior controls while increasing the interval and resetting endpoint
P/V/A. Captured Tora garage candidates first self-intersected inside that loop.

Moving-boundary time allocation now always starts from the original rebound
curve, reuses `reparamBspline`, restores the original physical P/V/A, and builds
the fitness reference from the newly fitted curve. A failed candidate contributes
only its time deficit to the next attempt. `lengthenTime` stretches only the
middle `T - 4*h`, so its ratio is compensated to allocate the requested total
time. The existing bounded time attempts remain; no warped interior is reused.
Fixed vertical derivative violations are handled by time allocation because
the optimizer retains Z and cannot repair them by moving XY.

The new `refineConstrained` entry reuses the existing costs and L-BFGS with
local augmented-Lagrangian multipliers for the actual speed norm, observed
body-axis capacities and acceleration norm. It retains fixed endpoint controls
and Z. A `1e-6` internal squared-constraint margin applies only to adjustable
XY capacity; final acceptance still uses the original physical time ratio
`<= 1 + 1e-9` and existing 3-D collision checks. Negative solver errors remain
failures. Its inner solver uses the existing `past=3`, `delta=1e-8` stopping
criterion so accepted feasible iterates are not polished into floating-point
line-search exhaustion. The ordinary upstream-style refine entry is unchanged.

SCAN's `CUBIC_MINIMIZER2_LBFGS` also contains an inherited direction error:
the fallback uses `a < 0`, although `a` is a normalized interpolation quantity.
The original liblbfgs at `5ad02fbefefdeff339ab03635e673571055a0644` uses
`d > 0`, the actual trial direction. The port now matches that expression.
A captured bracket regression fails before this one-line fix and passes after;
real line-search failures still preserve rollback and reject the candidate.

Verification: all 154 `test_scan_planner` cases, 3 refine-solver cases, 10
elevation cases and 4 spline-velocity cases passed. The new recorded manager
regression detects two self-intersections and three backward excursions before
the repair and passes afterward. Sources, fixed inputs and matched garage
records are indexed in `docs/research/tora-retiming-fix-20261107.md`.
This is native component evidence, not field qualification. The two nested
bounded loops can exceed the old worst-case solve work; field timing remains
unqualified and the existing handoff deadline still governs acceptance.

## Execution feedback clock boundaries, 2026-11-07

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` shifts the planner
epoch from a received frozen flag. LingTu also acknowledges reception of the
first spline and schedules future spline handoffs. The first acknowledgement
previously discarded the pending frozen interval between reception and that
acknowledgement; it now retains the later of reception and the last settled
clock boundary. Already settled moving time is not frozen a second time.

The backend now pairs the latest execution ID/start/frozen feedback with that
same feedback's `nowS`, instead of the older planning request timestamp.
Task already supplies all four values under its existing mutex and clock-domain
contract; no new clock, threshold or execution protocol is introduced.

Three deterministic regressions fail before these changes and pass after.
The affected full suites pass 158 SCAN and 28 Task cases; three follower and
two Executor timing regressions also pass. See
`artifacts/nav/tora-boundaries-20261107/clock/` for red/green and full-suite logs.
These fixes alone did not resolve the captured garage handoff phase discrepancy:
the follow-up reached with zero self-intersections, but its maximum tracking
error was 21.046 mm. A bounded trace then measured the old planner epoch 10 ms
ahead of the follower before the first handoff; the candidate's boundary P/V/A
and the single reception-age calculation matched. See the `handoff/` evidence.

The native execution path now supplies the follower's existing elapsed phase,
its sample time and trajectory ID with each planning request. Task copies that
sample through its existing locked feedback. Backend extrapolates only while
unfrozen and only for the acknowledged execution ID; FSM also requires its own
current trajectory ID before aligning the epoch. A matched sample settles the
freeze boundary, so the same callback cannot add frozen time again. Consumers
without a phase sample retain the existing boolean/first-ack behavior. This
completes a local asynchronous handoff contract; upstream supplies only a
frozen boolean. No execution history queue, new tuning parameter or trajectory
shape change is needed.

Final verification passes 160 SCAN, 29 Task and 66 follower tests. The reverse
garage reaches with 45 complete curves, no strict XY self-intersections and
maximum tracking error 2.593 mm; first-handoff error stays near 2.12 mm instead
of the previous 11.7/21.0 mm jump. Executor passes 104 of 129 cases. All 25
failures also fail at exactly the same assertion-line sets in an independently
built pre-follow-up variant, so this is not a claim that the whole navigation
suite is green. The full results and baseline construction are recorded in
`artifacts/nav/tora-boundaries-20261107/verification.json` and `executor-baseline/`.

## Execution clock cleanup, 2026-11-06

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` sets controller
`dt` to zero after a gap above 0.2 s, then continues computing the command
from the retained spline. It does not stop or discard that spline for this
condition. LingTu's Executor had added a separate `suspendAutonomy()` and
zero-output branch using the same threshold. That branch, its timestamp state,
and its controller dependency have been removed.

The native controller now advances by the full nonnegative execution-time
delta. This differs from the pinned controller: the native planner derives
progress from the same execution clock and does not receive the follower's
discarded time, so retaining the upstream gap clamp would desynchronize their
reference phases. Explicit holds and heading alignment still freeze execution.
Follower command changes retain the existing `maxDt` acceleration budget;
resuming an explicit pause uses the existing nominal command step. The checked
deceleration prefix now uses reference elapsed time for its speed allowance,
independently of that bounded command step.

Real execution uses steady time. The simulation clock boundary already rejects
backward physics time; removing Executor's duplicate check does not bypass it.
No collision, input-age, driver watchdog, or runtime-health threshold changed.
Matched 0.3/0.5 s delay regressions and the pinned upstream source copies are in
`artifacts/nav/clock-gap-20261106/`. These software inputs isolate the timing
change; they do not identify the cause of every pause in the physical L trials.
Verification: four new behavior tests failed against the preceding sources;
the final affected suites passed 88 tests, the input-projector executable passed,
and the native Windows navd build linked successfully. No new physical run or
field deployment was performed for this cleanup.

## Quiet trajectory heading, 2026-11-06

The physical 90 cm L-corridor fixture exposed a local start check deriving
74.48 degrees from 6 micrometres of spline displacement while the measured
body heading was 13.71 degrees and free. Pinned SCAN
`348e8a590a50a5a6bbab8d8c6dcfd171f009be26` uses finite displacement here and
returns zero yaw below squared displacement `1e-8`; the prior LingTu version
removed that zero-heading fallback to avoid turning northbound paths east.
Neither rule represents a known quiet body heading.

Optimizer rebound/refine/retiming, active and pending trajectory checks, retained
tails and speed-transition validation now share the native velocity-based segment
query. At the physical start it uses measured yaw; moving future boundaries use
their own direction. A per-validation optional heading preserves the preceding
future direction through a quiet tail. This state is local to one traversal.
The 0.03 m/s quiet rule is LingTu behavior, not an upstream parameter claim.
Candidate/FSM map checks still exclude forecasts; final-control prediction checks
remain enabled. Moving turns and their swept collision are still checked.

The earlier duplicate rebound validation loop was removed. Native SCAN regression:
149 tests passed, including tiny/0.02 m/s displacement, future moving boundaries,
quiet northbound tails, real turning collisions and prediction separation.
This does not unify the global XYZ route with local yaw, or the follower's
configured 0.8 s heading preview with every collision sample. Physical narrow
passage tests remain unsuccessful; see
`docs/research/narrow-passages-physical-20261106.md` and
`artifacts/nav/narrow-passages-20261106/`.

## Formal envelope unification, 2026-11-05

Against pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`, the native
endpoint now uses the twin-cylinder geometry for both planning and final braking.
BODY2 loading and the dimension-derived OBB fallback were removed from the
endpoint. Mapd inflates once. Thunder uses radius 0.32 m, offset 0.23 m and
body-relative vertical bounds [-0.05,+0.35] m, fitted to the nominal MJCF body
height band with approximately 5 cm XY allowance. These are platform values,
not upstream Go2's 0.25/0.18 m example. Joint motion, lower-leg clearance and
field performance are not qualified by this fit. Native braking and prediction
remain local extensions; their geometry no longer switches to multipart Body.
Matched motion evidence belongs to `build/thunder-twin-20261105/`.

## Short-approach and corner repairs, 2026-11-05

Reference: `wuyi2121/SCAN-Planner@348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
The manager reuses exact-boundary retiming for approaches shorter than its
existing control-point spacing. A captured moving terminal approach failed with
the ordinary least-squares fit and succeeds with the exact physical boundary.
Longer moving routes retain their previous fitting mode unless the measured
start speed exceeds the new cap. Rest-to-rest segments also use exact boundary
controls: a captured clear stair-post candidate reached 5.314 m/s², exceeding
the final 5.2 m/s² acceptance limit after ordinary refinement. Time dilation
with zero endpoint derivatives keeps the clear curve and meets the configured
4 m/s² acceleration limit. This uses the existing retiming implementation,
without relaxing its limits or adding another retry policy. The regression
uses the recorded 5 cm bitmap in `fixtures/scan_rest_corner.bits` (origin
`(0.5,2.75,0.25)`, `100 x 50 x 30`, X-fast/LSB-first), its saved route, and a
settled start from the same stopped episode. A blocked speed bridge exposed a replacement
whose approximate fit shifted the start by 5 cm and retained 0.19 m/s at its
stop endpoint; the same exact-boundary mode now preserves that moving boundary. No body yaw or new
configuration threshold is introduced. Pinned upstream rejects targets closer
than 0.2 m; the native clicked-goal contract already permits shorter continuations.

Upstream getLocalTarget also uses planning_horizon / 20 / max_vel while searching
around a blocked endpoint. The port caps both reference projection and target search at resolution / max_vel
to avoid projecting beyond or skipping the last clear approach cells. In the recorded flat case the
selected endpoint moves from 18.1 cm to 12.6 cm short of the clicked goal, within
the unchanged 15 cm task tolerance. The blocked endpoint itself remains blocked.

Upstream DynAStar marks neighbor->rounds before checking occupancy. The port now
marks only admitted nodes, together with their new cost and parent. This is a
search-state correction, not an extra collision gate or yaw search dimension.

The initial collision weight remains upstream's 1. The bounded optimization
loop permits four passes instead of three, reaching the next existing doubled
collision weight only after collision failure. This resolves the saved post
request. Raising the initial weight to 4 also solved that snapshot but regressed
the matched box run, so that Product override was discarded. Inflation,
clearance distance, body dimensions, arrival tolerances and final safety remain.
Evidence: `build/scan-continuation-20261105/` (native replay and matched MuJoCo).

Two further integration corrections use the same collision geometry:

- Test the original guide's curvature before joining the measured start. A
  lateral offset after avoiding a new box must not create a mandatory bend
  back into the old, straight global route. Collinear densification samples are
  removed before the measured endpoints join the guide, so a curved route does
  not force a return to the first sample on its straight approach. Real 3-D
  bends remain in the seed. A same-map stopped-post replay failed before this
  connector correction and succeeds after it without increasing the four-pass
  optimization limit.
- A collision in the first spline interval now uses the existing bounded
  collision-cost retries. Fixed boundary controls do not fix the finite-segment
  cylinder heading: later editable controls still affect it. Candidate checks
  also cap their time step at the execution check's 10 ms. The previous mean
  distance step accepted a recorded curve that the execution check rejected
  20 ms into the curve. Occupied candidates still fail after bounded retries.
- Upstream's `estimateSegmentYaw` substitutes east (0 rad) when squared sample
  displacement is below `1e-8`. With 10 ms sampling, a valid rest-to-rest northbound
  curve crosses that cutoff; slowing the same geometry changes its apparent
  collision. The port uses `atan2` for nonzero displacements regardless of their
  length. The same 23 cm cylinder offsets reject east and admit north in the
  regression; two time scales retain the same collision result.

Autonomous GEN_NEW_TRAJ also uses the existing upstream emergency-stop settling
criterion (measured 3-D speed below 0.1 m/s) for a rest boundary. The recorded
stopped body swayed at roughly 1 cm/s; using that transient direction as the
initial cylinder tangent prevented restart. This does not alter moving spline
handoffs or MotionIntent boundaries. The moving short-approach regression still
asserts its measured initial velocity exactly. Applying this settling criterion
to new autonomous trajectories is a local difference, not an upstream default.

These are local repairs against the pinned upstream, not its default behavior.
The cropped MID360 bitmap in `tests/nav/cpp/fixtures/scan_stair_post.bits` retains
its 5 cm occupancy at origin `(0.5, 2.75, 0.25)`, dimensions `80 x 50 x 30`,
X-fast, LSB-first bits. It comes from the same Thunder stair-post failure and
contains no raw sensor identifiers. It exercises candidate geometry, not robot
arrival or physical contact.

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

`seed.cpp::boundedVelocitySeed` joins acceleration, cruise and
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


### 2026-11-06: shared nominal translation acceleration

The shared `limits.yaml` translation acceleration changes from 4 to 0.5 m/s².
SCAN, Follower and the X/Y velocity smoother still derive this value from one
source. The pinned upstream commit above uses 0.5 m/s² in
`src/planner/plan_manage/launch/advanced_param.xml`; its 0.75 m/s speed default
is not adopted here. LingTu retains its axis speed budgets, 4 m/s² braking,
yaw limits, cylinder dimensions and additive optimizer feasibility tolerance.
This is a nominal planning/command limit, not a measured physical acceleration.

Frozen Thunder v39 MuJoCo / MID360 flat-box comparisons are recorded in
`build/nav-accel-20261106/report.md`. Under the same experiment process priority,
4 m/s² reached 1/2 goals, 1 m/s² reached 2/3, and 0.5 m/s² reached 3/3;
the latter includes one uninstrumented run. All had zero box contacts.
The first final-control veto could have a clear proposed-command braking arc
while the measured-velocity braking arc collided. Later vetoes included a
front-cylinder overlap with noisy box-corner occupancy. Lower acceleration
reduces this mismatch; it does not remove final braking or prove field readiness.


### 2026-11-06: admit reachable A* guide endpoints

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` admits an A* endpoint
using the start/end chord yaw, then checks neighbors at eight discrete incoming
yaws. A recorded Thunder box-corner endpoint was free at -9.55 degrees but
occupied at every one of those eight yaws. Search exhausted 3,921 nodes on each
retry, returning no rebound guide and eventually causing an emergency stop.
The existing endpoint adjustment now also requires one admissible incoming
heading. This changes only the optimizer's internal guide endpoint; final
spline collision checks, robot dimensions and the navigation goal are retained.

The 750-byte `tests/nav/cpp/fixtures/scan_guide_endpoint.bits` fixture is a crop
of the captured 5 cm inflated map: origin (2,-1,0.4), size (60,50,2), X-fast,
LSB-first. The regression proves the old admission/search disagreement and
requires a guide whose actual incoming samples pass the same occupancy query.

Evidence: `build/nav-twist-20261106/`. Two frozen baseline MuJoCo runs stopped
locally and restarted; two otherwise matched candidate runs reached without that
emergency-stop transition (10.705/10.905 s versus 8.905/8.505 s; zero box contacts).
Nominal translation acceleration stayed 0.5 m/s². On the original video's two
reconstructed curves, 54 identical-state comparisons of the pinned official
heading formula and the native core differed by at most 3.4e-16 rad/s. This is
an A* endpoint consistency repair, not a changed tracking law or field claim.


### 2026-11-06: preserve the terminal spline while settling

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` retains its received
spline in the closed-loop controller and applies position feedback at its end.
LingTu adds measured-rest confirmation for ordinary XYZ arrival. That confirmation
used to reset the planner and follower before rest was established. In a recorded
Thunder run, a drift from inside 15 cm to 15.6–16.2 cm started a new return curve,
triggered the normal heading-alignment branch and turned from -25 to -62.6 degrees.

The ordinary-goal settling branch now reuses `pauseLinearMotion()` when a SCAN
spline is active. Existing explicit pose-stage transitions and all arrival,
collision and velocity limits retain their behavior. No new terminal heading
policy or threshold is introduced. The regression replays a 14-to-16 cm drift in
both Map and Odom frames: the old implementation requests a 180-degree turn;
the repaired implementation corrects position and returns inside the same radius.
Evidence and matched physical runs are in `build/nav-terminal-20261106/`.

A second captured interaction kept ordinary arrival waiting on nonzero feedback
commands after measured motion was already quiet, allowing a new short spline
to request another heading. The arrival branch now permits an ordinary goal on
its existing measured-rest predicate even with residual feedback commands. The
same position/height radius, 0.03 m/s 3-D speed and 0.08 rad/s yaw-rate checks
remain. The regression distinguishes measured motion at 0.06 m/s from rest at
the same in-radius pose; explicit pose-stage scheduling is not changed.

### 2026-11-06: honor an explicitly requested goal heading

The pinned upstream `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`
`scan_replan_fsm.cpp` takes XYZ goals; `closed_loop_controller.cpp` derives yaw
from the spline and stops on terminal position error. LingTu's goal API also
accepts an optional heading. Route activation had discarded that value, so a
zero-degree goal could report arrival at -35.1 degrees.

`Executor::setRoute` now forwards `finalYaw` and `yawToleranceRad` to the existing
arrival controller. A missing heading remains position-only; zero is a specified
heading. Follower computes the turn command, local turn queries and endpoint
FinalControl retain their collision responsibilities. The existing heading latch
keeps position correction aimed at the requested heading after turn-induced drift.
No gains, tolerances, stopping bounds or new planning stages were introduced.
Regressions cover zero yaw, angle wrapping, requested tolerance, measured rest,
Map/Odom correction and route replacement/resume. Matched physical evidence is
recorded in `build/nav-goal-yaw-20261106/`.

Physical acceptance remains incomplete: two repaired runs report native arrival
but end at -5.415 and -5.525 degrees after stopping, just outside the unchanged
5-degree tolerance. The CTS v39 policy responds very slowly to .2 rad/s pure
turn commands and recoils after zero. One run also exceeds the existing reverse
command fraction gate. The bounded alternative turn/settle probe is recorded in
the report, but its policy-specific tuning has not been merged or validated as
full navigation. Do not equate passing the forwarding regression with completed
physical pose-arrival acceptance.

### 2026-11-07: remove duplicate follower dispatch

Pinned SCAN `348e8a590a50a5a6bbab8d8c6dcfd171f009be26` publishes a B-spline
from its replanning FSM. Its closed-loop controller directly consumes that spline
and odometry, computes a command, and zeros the command after trajectory time and
position tolerance are satisfied. It does not implement LingTu's optional final
heading or post-stop pose confirmation contract.

LingTu retains both CMU path and SCAN spline tracking. `LocalPlan::target()` already
identifies them, so Follower now calls the matching implementation directly.
The second reference-wrapper target variant, input conversion, algorithm base,
factory registry and instance map were removed. Typed targets are borrowed from
the existing plan payload. Switching target kinds resets private execution state;
pause, reset, applied-command feedback and CMU profile selection retain their
semantics. Controller equations, acceleration budgets and tolerances are unchanged.

This removes dispatch scaffolding only. The Executor-owned terminal turn/arrival
branch remains a known separate concern; no claim of fixing CTS stop recoil or
fully unifying terminal control follows from this cleanup. Focused follower and
execution evidence is recorded in `build/follower-cleanup-20261106/` (work started
before midnight).

### 2026-11-07: explicit goal completion belongs to Follower

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`. Its ordinary
spline controller and replanning FSM are unchanged by this work. Requested final
heading and post-zero pose confirmation are LingTu contracts, not upstream defaults.

Follower now owns the terminal turn, heading latch, measured-motion settling and
applied-zero evidence. Executor retains trajectory selection and local 3D turn
collision checks. A turn already admitted at the goal continues through small
position drift; SCAN reconnects position after heading alignment, and completion
still requires the original position/height/yaw tolerances. Repeated odometry
samples cannot advance the settling interval. Turn commands obey angular
acceleration and platform rate limits; measured turn speed informs braking.

Defaults live once in `config/system/settings/follower.yaml` under
`parameters.goal_follower`, including the 0.4 rad/s minimum turn command tested
with Thunder CTS v39. Native defaults are generated from that file. This is not
hardware calibration evidence for every RobotConfig. Matched same-box runs and
all failed trials are recorded in `build/nav-converge-20261107/`; task arrival
and the broader motion-quality gates are reported separately.

### 2026-11-07: simplify goal admission and terminal correction

The same pinned upstream controller still has no requested final-yaw or post-zero
completion contract. Reviewing the recorded same-box run exposed a local
integration problem: a short position correction rotated the body to about
-152 degrees before terminal control returned it to the requested zero heading.

Follower now admits the goal heading on first entering the accepted position
range, brakes translation with the existing acceleration limit, then finishes
heading or requests SCAN position correction under that heading. Changing the
planner's heading contract while still tracking would invalidate the current
spline, so the brake precedes that change. Final command collision arbitration
remains in place. Goal yaw uses the accepted tolerance; the extra 0.5-degree
threshold was removed. Applied zero and the configured 0.2-second stopped
observation remain: removing the interval produced post-arrival yaw beyond
5 degrees in both matched trials. These are LingTu terminal semantics,
not changes to the upstream spline solver. Matched evidence is in
`build/wrapper-cleanup-20261107/`.

Position correction outside the accepted radius keeps running under that same
heading; the terminal brake is not re-entered on each correction tick. Also,
crossing the yaw tolerance decelerates the applied angular command using the
existing acceleration limit. Abruptly replacing it with zero caused repeated
stop/recoil/restart motion at the tolerance edge in the same-box experiment.

### 2026-11-08: one follower update owns goal completion

Pinned upstream remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`. Its controller
stops on spline time and position; explicit final yaw remains a LingTu task
requirement. The six-scene MuJoCo suite exposed alternating `follow`/`finish`
branches and repeated resetting of the 0.2 s observation at the 15 cm boundary.

`Follower::follow` now receives the accepted goal and owns the complete update.
The public `finish` entry and settling-time parameter are removed. Current pose
and measured rest decide completion; applied-command feedback remains for rate
limits and the endpoint still confirms task stop. Tracking converges inside the
task radius using existing quiet-speed/position-gain settings. The solver and
final collision arbitration are unchanged by this task. Verification and physical
results are recorded in `build/follower-unified-20261108/`.

The first no-timer physical trials reported arrival but ended at 13.07 and
12.02 degrees after observing until 45 s, outside the accepted 10-degree band.
Follower therefore keeps an active turn until the inner half of its accepted
yaw band, then decelerates. An initially stationary, already-accepted heading
still completes immediately. This small control hysteresis is a local response
to measured recoil, not an upstream SCAN default or a tighter task tolerance.

### 2026-11-08: continuous terminal-heading control

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`; its ordinary
controller derives heading from the spline with a 0.8 s preview. Explicit final
yaw is a LingTu contract. The local integration used that time window to admit
final yaw even when a retimed short curve and heading freeze prevented its clock
from reaching the window. It also switched from spline yaw gain to terminal
minimum-rate/hysteresis control only after entering terminal hold.

Follower now admits the final heading when either the existing time preview or
its speed-times-preview distance covers the remaining 3D curve. Intermediate
endpoints and longer remaining bends retain spline heading. Approach and hold
reuse the existing terminal turn calculation and applied-command feedback.
Normal heading alignment decelerates translation with the existing limit while
freezing reference progress; external safety stops retain immediate zero output.
The lower terminal yaw-rate target no longer bypasses angular acceleration.
There are no new configuration values, public control stages or solver changes.
Matched regressions and physical trial results are retained in
`build/turn-continuity-20261108/`.

### 2026-11-09: distinguish final braking refusals

The pinned upstream remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
LingTu's final measured-motion and candidate-command braking checks are local
integration behavior. Candidate scale-zero refusals now report
`scan_command_motion_blocked`; measured-motion refusals retain
`scan_actual_motion_blocked`. Autonomy handles both with the same existing
pause and blocked feedback. Geometry, reaction time, limits and output commands
are unchanged. Viewer and saved-route diagnostic callers use the same names.
Partial candidate scaling reports `scan_command_motion_limited`; the former
`scan_actual_motion_limited` name incorrectly described which input was scaled.
The single-use measured-query lambda and its duplicate pose check were removed
without changing short-circuit behavior. Follow-up review evidence is in
`artifacts/nav/door-review-20261109/`.

The door investigation captured an unsafe measured braking arc despite a clear
candidate arc, followed by a separate static inflated-cell refusal. Native replay
also reproduces that static refusal with zero reaction time. This diagnostic
change does not claim to fix door traversal. Evidence and the already completed
ordinary point-goal clock-wait fix are separated in
`docs/research/door-navigation-20261109.md`.

### 2026-11-09: discard a curve refused by final collision control

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`. Its future
collision callback replans before emergency stopping. LingTu's final-command
arbiter is an additional integration boundary. Its collision refusal previously
paused the follower while retaining the accepted curve, allowing the next
control update to try that curve again.

Executor now uses its existing local reset when autonomous SCAN reports a final
motion block. This cancels queued results and clears the accepted planner curve
and follower target together. The retained global route is submitted again with
the current measured state. Ordinary pauses keep their existing reference-time
semantics. The saved-route component benchmark forwards the same block feedback.

`Executor.FinalBlockReplansFromCurrentPose` fails with the old implementation
returning the old curve after a measured-position change. The corrected build
passes this regression and four related execution tests, plus the native
autonomy and teleoperation controller tests. Evidence is in
`artifacts/nav/stop-cycle-20261109/`. The physical door reruns did not pass idle
control-loop startup readiness and issued no navigation goal, so this change
has no new successful MuJoCo traversal claim.

### 2026-11-09: retain A* storage across local resets

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
LingTu's Backend reset previously rebuilt Manager and its 100 x 100 x 100
individually allocated A* nodes. Repeated final-motion refusals therefore
incurred node-pool destruction and reconstruction before the next solve.

Backend now retains Manager. Its existing initialization entry clears trajectory,
reference, progress, failure and time-source state, replaces the small Optimizer
and FSM objects, and transfers the existing AStar into the new Optimizer.
AStar's existing same-size initialization reuses its node pool. Search rounds
continue increasing so old node marks remain distinct from the next search.
Task cancellation and old-result invalidation keep their existing ownership.

The direct native reset microbenchmark records 1,010,105 normal new/new[] calls
and 56,083,632 requested bytes per reset before, versus 2 calls and 1,440 bytes
after. Five-run median reset time is 398.0651 ms before and 0.0018 ms after.
This scope excludes Task cancellation, the next solve and physical motion.
Eleven planner tests, four Task tests and three Executor tests pass, including
an updated-map reset regression and final-refused curve invalidation.
Evidence and the exact incremental patch are retained in
`artifacts/nav/reset-reuse-20261109/`.

### 2026-11-09 — reduce repeated work during sampling

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
The de Boor evaluator previously assembled a vector of separately allocated
dynamic control vectors on every call. It now uses one owning temporary Matrix
with the same descending recurrence, knot selection and clamping. Supported
dimensions and degrees remain generic. The regression compares the frozen old
recurrence for 1, 3 and 5 dimensions, degrees 0 through 5, nonuniform knots and
samples inside and outside the time span.

LingTu's independent no-go policy is a local extension. Its trajectory and
segment query entries previously checked the same no-go sweep again in their
shared measured-segment query. The shared query now owns that check, including
when ordinary occupancy checking is disabled. Prediction and measured-geometry
queries retain their existing acceptance results.

A standalone native Release benchmark evaluates cubic position and its velocity
and acceleration 600,000 times per round. Five-round median time is 579.0454 ms
before and 274.8653 ms after, with identical checksums. This measures spline
evaluation only. It does not measure full-navigation speed or Eigen allocation
counts. Source, build logs, targeted tests and results are retained in
`artifacts/nav/runtime-waste-20261109/`.

### 2026-11-09 — shorten control reasons and consolidate loop recovery

Pinned SCAN remains `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.
Final braking and control-loop holding are LingTu integration responsibilities.
Their current reason names are `brake_blocked` for measured-motion braking,
`cmd_blocked` for a refused command and `cmd_limited` for partial command
scaling. Autonomy, replay diagnostics, simulation acceptance and Web presentation
consume those names. Historical reason strings remain in recorded evidence
and display-only readers.

Goal admission and background completion now carry manual takeover and loop
holding separately. They report `takeover_active` and `loop_hold` respectively,
while preserving the previous combined blocking decision. Generic control
interruptions keep their existing ownership. LoopGuard consolidates duplicated
recovery counting and exposes one snapshot query. A frozen old/new state-machine
comparison and focused regressions are retained in
`artifacts/nav/control-cleanup-20261109/`.

The DDS navigation-state projection now includes that same loop-hold fact in
its existing `hold_reason` field. DDS and detailed JSON status use one boolean
query per publication. LoopGuard's `held()` query derives from its existing
state without constructing a full decision. Message and state-projection
regressions are retained in `artifacts/message/contracts-20261109/`.
