# SCAN backend

This directory implements the SCAN algorithm behind `nav_kernel::LocalPlanner`.
It contains LingTu's ROS-free port of
[wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) pinned to
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.

## Planning lifecycle and source layout

`upstream/` identifies the algorithm's origin, not an unmodified vendor copy.
`plan_manage/` retains the upstream module layout for comparing the port with
the pinned source. Its components have separate responsibilities:

| Component | Responsibility |
| --- | --- |
| [`PlannerFsm`](upstream/plan_manage/planner_fsm.h) | Accept targets and decide when to plan, replan, execute, hand off, finish or stop. |
| [`SCANPlannerManager`](upstream/plan_manage/planner_manager.h) | Run one planning attempt: initialize, optimize, check feasibility and adjust trajectory timing. |
| [`plan_container.hpp`](upstream/plan_manage/plan_container.hpp) | Store reference and local trajectory data. |
| [`ClosedLoopController`](upstream/plan_manage/closed_loop_controller.h) | Compute velocity commands from the trajectory reference and measured state. |

`planner_fsm.{h,cpp}` and `PlannerFsm` were renamed from the port's
`scan_replan_fsm.{h,cpp}` and `SCANReplanFSM` on 2026-10-27. The name covers the
whole local-trajectory lifecycle, not only replanning. This rename changes no
states, thresholds, scheduling or external messages. The original filename and
commit remain in the header and [upstream notes](upstream/UPSTREAM.md).

## Port contract

The algorithm sources live under [`upstream/`](upstream/UPSTREAM.md). For the
same valid inputs, parameters and callback order, process and storage adapters
must preserve the pinned algorithm's map queries, Projected DynAStar path,
rebound control points, B-spline, six-state FSM transitions and controller
command. The inherited correctness repairs and intentional trajectory
initialization extension documented below are explicit exceptions; this port
does not claim unconditional output equivalence.

Ordinary speed defaults are 1 m/s for the planner's velocity norm and each
controller body axis. Python parameter resolution, native library structs and
the bare endpoint use their language's shared speed constant. Product profiles
still declare their own limits: `nav` and `teleop_avoid` allow 1 m/s per axis
with a sqrt(2) norm budget. Explicit Env/session/task limits remain effective.
The old 0.75/0.35 m/s controller defaults are not an admission or hidden cap.
This is a deliberate LingTu tuning choice, not pinned upstream defaults.

This is the port's equivalence contract, not a claim that a passing unit suite
proves every upstream execution or robot behavior. Upstream differential
results, MuJoCo runs, and field measurements are separate evidence.

The inherited optimizer's intersection-state and collision-sampling repairs
are documented in [`upstream/UPSTREAM.md`](upstream/UPSTREAM.md). Successful
intersection and optimization equations remain unchanged, but a candidate
that fails the corrected collision check is rejected or reoptimized.

RouteTarget and MotionIntent share LingTu's acceleration/cruise/braking seed
for sufficiently long maneuvers with zero initial acceleration. This avoids
restarting a slow full-duration polynomial ramp on low-speed route replans.
Short maneuvers, nonzero initial acceleration, or boundary speeds above the
limit retain the original initializer. This changes the resulting trajectory,
not just its storage or transport; details and limits are recorded in
[`upstream/UPSTREAM.md`](upstream/UPSTREAM.md#bounded-velocity-initialization-extension).

LingTu's process and map adaptations live at the boundary:

- With no request body model, `grid.*` exposes Mapd's inflated bitmap through the official
  `GridMap::getInflateOccupancy()` query. It does not build, inflate, score, or
  smooth another map. The segment query also reads this bitmap, traversing
  cells between successive front/rear cylinder-center samples without
  allocating a temporary map or container. Global step-height and slope search
  settings are not reimplemented in this adapter. Routes can carry the shared
  support query described below. The seven-point support gate and
  its per-probe free-ray requirement have been removed; sparse ground returns
  cannot invalidate an otherwise geometry-valid local collision bitmap. Mapd still
  retains the measured occupied/free evidence. OctoPlanner selects the terrain
  layer; SCAN checks the optimized curve against the shared support query when
  supplied. A zero collision result means no
  inflated obstacle was found at the queried cells; it does not assert that
  those cells have measured free-space evidence. In this twin-cylinder mode,
  `knownFreeStorage` remains evidence, not an additional local collision gate.
  The explicit body mode below instead requires raw occupied
  evidence and does not query the inflated bitmap.
  Point and DDA segment indexing share `GridAxis` with the snapshot view. The
  origin is split into an integer base and fractional phase: only the phase is
  subtracted before floor, then the base is removed. Aligned windows retain
  Mapd's published cell-face and DDA-tie arithmetic; valid in-process snapshots
  may also have non-aligned origins. Only ULP-scale alignment error is snapped.
- `backend.*` converts `LocalPlanRequest` odometry and Route/MotionIntent into
  official FSM inputs, then converts the emitted B-spline message into
  `SplineTarget`. An emergency-stop state is exposed as `NearFieldStop`,
  retaining the upstream stationary spline only inside the FSM. It must not
  be cached as a ready navigation trajectory. If retry exhaustion clears the
  target, the result is `NoPath`; a successful replan restores `Ready`.
- `task.*` owns one serialized worker with the upstream timer
  semantics: the FSM callback targets 100 Hz and the future-collision callback
  is scheduled separately at 20 Hz. The last published trajectory remains active
  until the FSM publishes a replacement or an emergency stop, matching the
  official controller/topic behavior.
- `tracking/follower.*` adapts `SplineTarget` to the official closed-loop
  controller. CMU `PathTarget` continues through its separate path follower.

The SCAN core contains no DDS, Product, freshness, final-control, Route cost,
partial-path, virtual-boundary, custom slope, or fallback logic. Endpoint input
readiness owns `collision_stale`; an expired collision layer is not
reported as an official `no_path` result.

Mapd uses the upstream map profile: `0.05 m`, `200 x 200 x 100`, `5 m` rays,
`p_hit=0.85`, `p_miss=0.30`, `p_min=0.12`, `p_max=0.98`, and `p_occ=0.80`.
Inflation is maintained incrementally when occupancy changes. The SCAN profile
disables time-based occupancy decay: cells clear only through ray misses or a
rolling-window eviction, as in the pinned upstream GridMap. Requests without a
selected multipart Body use the platform's configured twin-cylinder radius and
offset. The ring buffer and packed collision bitmap only reduce memory moves
and DDS payload; they preserve the logical GridMap result. Product speed limits
may be lower than the upstream launch defaults; they are explicit runtime
inputs. The native follower adds command-continuity bounds; heading alignment
uses SCAN's single threshold and the planner objectives remain.

`nav` and `teleop_avoid` select `scan` by default and target 100 Hz for the SCAN
FSM and controller, with the upstream 20 Hz future-collision schedule.
`cmu` remains an explicit backend option and is not deleted or used as a silent
fallback.

## Terrain and foot placement scope

SCAN optimizes a body-position B-spline and checks its collision geometry,
smoothness, velocity and acceleration. It does not optimize joint angles,
foot placements, contact forces or a gait schedule. These reference-trajectory
limits do not establish contact stability or controller-specific terrain
capability.

Ordinary autonomous routes retain OctoPlanner's support, step and slope checks.
When the activated route carries its immutable reconstructed support query,
Executor forwards it and the reference support-height metadata through the
asynchronous SCAN request. The existing final spline-segment query samples
support at half-voxel spacing. The nearest 3-D reference segment selects a
ground-height band, including either tread of a height-changing edge; the
query uses map coordinates even when SCAN plans in odometry coordinates.
This is a LingTu extension to the pinned SCAN map adapter. It prevents an
optimized curve from crossing missing terrain while using the same inferred
surface as global planning. It does not demand a measured floor in every body
column or a fixed base-to-ground distance at every sample. Base heave does not
change the selected reference layer. Static body and braking collision checks
retain their measured-terrain handling. Individual foot support is not a Body
collision prerequisite, and these queries do not plan footsteps.

Global planning returns XYZ routes with 3-D support queries; it no longer
searches intermediate body yaw or checks multipart turn sweeps. SCAN uses the
selected Body with the local request's heading or yaw envelope. Explicit pose
routes and declared stairs retain their execution contracts. This geometric
capability is not a qualification of a locomotion controller or gait.

Collision/sweep margins enlarge collision geometry, not load-bearing area.
The former whole-body and per-foot ground proofs have been removed.

See the [traversability and foot-placement research note](../../../../../../docs/research/traversability-and-footholds-20261025.md)
for the distinction between body collision, terrain assessment and locomotion.

## Reference geometry and solver results

Reference-path requests pass the selected three-dimensional projection-to-target
time interval from the FSM to the manager. Ordinary routes retain the full
ordered XYZ reference in 3-D arc-length order, including flat S bends and routes
with a fixed body heading. This no longer depends on `preserve_boundary_` or the
absence of `body_yaw_`. The seed retains its relative acceleration/cruise
progress and duration. Boundary P/V/A restoration, retiming and final
collision/derivative checks still apply. Assisted MotionIntent retains its own
guide and detour initialization. This does not add free-Z optimization or
foot-placement control.

An occupied lookahead target may use a clear forward reference prefix. Ordinary
routes reject only a prefix shorter than 1 mm, matching the manager's
noncoincident-endpoint requirement; they no longer require 20 cm of progress.
MotionIntent retains its 20 cm detour admission and final-approach exception.
Both modes retain projection order and actual target occupancy checks.

Refinement uses the variables actually returned by L-BFGS, including its rollback
on line-search failure. Only the existing accepted termination codes continue
to final trajectory validation; other solver errors report `optimizer_error`.
Projected DynAStar uses immutable queue priorities and discards superseded
entries when a better route reaches an already open node.

Failure JSON now retains the Body models, original occupied/free evidence,
optional support height/tolerance and prediction expiry needed to reconstruct
the same Grid queries. This is geometry/support replay, not bit-exact replay of
the whole FSM and optimization history.

## Input lifetime and scheduling

`Task::update()` borrows caller views only during the call. Task takes one
immutable copy when the reference generation changes; input and timer work
share that copy. The endpoint's immutable collision storage is shared when
available; borrowed bitmaps are copied once per map epoch/generation. Neither
100 Hz FSM nor 20 Hz collision callbacks copy the full reference route.

Mapd rebuilds the exported collision bitmap only when its collision generation
changes (including a sliding-window change). Repeated observations still
advance timestamp/sequence, so content reuse does not conceal a dead sensor.
Resetting the map epoch clears the snapshot before it can be reused. Returning
Mapd snapshots and DDS serialization still copy bytes; this is not an end-to-end
zero-copy claim.

Configuration of the CMU path library stops at the Planner facade. SCAN Task
has no path-library argument and obtains its parameters at construction.

Product tuning is resolved by `lingtu.assembly.config.native_parameters` under
`scan_planner.*` and `scan_follower.*`, then recorded in the RunPlan and passed
through Endpoint configuration to the planner and controller. C++ struct
initializers are standalone library defaults, not Product configuration.
Ordinary translation acceleration now defaults to 2.0 m/s² for planning,
tracking and direct-command smoothing. Product resolution supplies the same
value; explicit overrides still apply. Final braking queries use the resolved
deceleration fields independently of the planner's acceleration, so changing
the commanded response does not implicitly claim stronger physical brakes.
Retiming a moving-boundary spline preserves its physical endpoint derivatives
and reoptimizes the free interior controls at the new interval. Moving only
the boundary controls can introduce a backwards section. A stationary-boundary
spline instead keeps its geometry and scales time directly.
Clear rolling operator guides carry the effective travel velocity at their
local endpoint; they do not imply arrival and a stop. Actual route goals and
selected blocked-guide detours retain their stopping boundary. The seed uses
the actual boundary-to-cruise velocity changes and samples its complete time
interval, including the endpoint, so an equal-speed replan can remain a cruise.
The follower starts a stamped replacement at its already elapsed phase, with
the timestamp converted into its paused execution clock. Receiving a new ID
does not rewind a moving reference to a past planning position. Explicit holds
still freeze execution instead of consuming the replacement's age. The first
trajectory starts at execution time zero: waiting for initial planning is not
executed motion. Only replacement of an already active trajectory consumes its
elapsed physical phase. `FollowerSpline.InitialPlanningWaitDoesNotConsumeExecutionTime`
covers a long initial wait before either algorithm activation or repeated stops.
Collision input freshness belongs to `local_collision.max_age_s` and is
shared by the Endpoint input gate and recovery; it is not an optimizer knob.
With obstacle checking enabled, the SCAN endpoint requires fresh mapd collision
input, not a second registered point cloud that SCAN does not consume. CMU
continues requiring its consumed point cloud. Odometry, required transforms,
autonomous localization health and driver-control readiness retain their owners.

## Execution feedback and runtime speed

Reference and odometry updates are input events; they do not replace the FSM
or future-collision timers. Accepting a reference and advancing the FSM are
separate callback actions. A reference or speed change must not keep injecting
the same event while a replan is waiting for its timer callback. The serialized
worker's 100 Hz and 20 Hz values are scheduling targets, not hard real-time
guarantees: planning work and operating-system load can delay both callbacks.

Executor owns the effective speed request. SCAN receives that limit as a
planning parameter and replans with it; the published spline carries the
corresponding speed metadata. A speed reduction retains the old trajectory when
its speed bound and any valid measured horizontal velocity fit the lower limit. Backend and follower cache
the derivative-control-hull speed bound for each accepted spline. If that bound
fits the new cap, and any valid measured XY velocity also fits, a lower metadata
limit keeps the geometry, knots, trajectory ID and execution clock. Repeated
metadata differences do not keep requesting the same replan. The controller
still applies the effective command cap.

The hull bound is a sufficient upper bound over the whole spline, not its exact
maximum speed. When the old curve exceeds a lower limit, a moving robot receives
an explicit cubic deceleration prefix from measured position/XY velocity and the
previous reference's vertical derivatives. It preserves boundary P/V/A, bounds
acceleration/jerk, and receives complete timed 3D collision queries before
publication. `SplineTarget.speedTransitionEndS` marks this finite exception to
the new cruise cap; it does not relax ordinary optimizer feasibility. The normal
FSM plans from the prefix's future boundary after deceleration. Base heave alone
is not horizontal overspeed. No independent speed-scaled clock is introduced.

While waiting for that checked replacement, the follower freezes the invalid
old reference and decelerates from its last applied command instead of issuing
an immediate zero. A late/failed replacement can therefore still lead to a stop.
An invalidated old ID cannot revive when the operator raises the limit. Real
obstacle stops, explicit holds and cancellation retain precedence. Task treats a
lower cap as a latest-only priority input, discards old unreceived offers and
preserves the executing publication. This prevents stale results from taking
over; it does not interrupt an optimizer iteration already in progress.
The manager, optimizer, AStar pool and trajectory ID sequence are retained.
Speed increases and ordinary replans may continue the current valid trajectory
until a replacement is available.
Backend owns this retention decision; Task propagates an explicit pending or
stopped completion instead of resurrecting its cached result.
The controller uses the same effective speed limit. This does not introduce a
separate speed-scaled trajectory clock.

Spline tracking bounds ordinary XY command changes in world coordinates using
`path_follower_max_accel_mps2`, and yaw changes using
`path_follower_max_yaw_accel_rad_s2`. The physical input interval is independent
of the spline execution clock. New trajectory IDs retain the previous command;
repeated or invalid time intervals do not accrue acceleration budget. Initial
valid measured body velocity may seed the command once; later measurements do
not repeatedly reset the ramp. Heading alignment enters at SCAN's existing
threshold and exits at the configured `path_follower_heading_align_exit_rad`.
An operator angular-velocity request is passed as an explicit yaw-rate target,
not converted to a continuously moving heading target. It leaves fixed-heading
alignment and can turn while translating; existing yaw acceleration, speed and
final collision limits still apply. Releasing the turn input uses the existing
reference re-anchor at the current heading.

Final control feeds its accepted command back into the follower, including
nonzero braking reductions, so releasing a slowdown cannot restore an old
high-speed candidate. This is software command feedback, not drive-response
confirmation. Holds, cancellation, heading freezes and
collision stops take precedence over smooth deceleration. Ordinary feedback
changes are bounded; these hard-stop cases deliberately remain immediate.
Merely lowering metadata on an already slow-enough spline does not create a
new hold. There is no new hard jerk guarantee or joint XYZ/yaw trajectory optimizer.

Pure assisted SCAN teleop rotation has no translation spline. When the generic
velocity smoother is disabled, FinalControl applies the same configured yaw
acceleration before the existing 3D command safety check. It records the accepted
yaw output, keeps repeated-time calls from ramping further, and clears motion
on release, invalid output, expiry or a safety stop. Spline/path commands are not
smoothed a second time by this pure-rotation branch.

Endpoint final control reports execution holds through
`Executor::pauseLinearMotion()`. Executor pauses both the planner's trajectory
time and Follower's retained progress; an allowed rotation remains available.
Ordinary input holds retain the trajectory. A requested replan invalidates the
old planner result and follower target together. Resetting only Follower would
replay a retained spline from its beginning.

The stair-width check also discards a rejected local trajectory instead of
freezing it indefinitely. The next local attempt uses measured motion while
retaining the active flight, confirmed motion profile, global route and width
constraint. Rejection still produces zero output; regenerated candidates must
pass the existing three-dimensional collision checks.

During stair traversal, `LocalPlanRequest::bodyHeading` supplies the same
planning-frame heading to candidate collision queries and the follower.
Sideways translation no longer rotates the collision footprint to the path
tangent. Alignment queries cover the shortest turn from measured yaw only at
the measured robot position. Future points and segments use the requested
heading without replaying that turn inside every future corridor. Timed queries
also distinguish a later visit to the same position from the current alignment.
This applies to measured occupancy, predictions, multipart bodies and the twin
cylinders. Dynamic queries retain obstacle height and arrival-time checks.
Stair width uses the exact maximum rectangular projection
over that turn. Final braking checks still use actual pose and twist. Changing
or clearing the requested heading invalidates retained asynchronous candidates.
Body position/pose stages can also supply this fixed heading; joined continuous
references clear it and use the spline tangent. Assisted motion keeps its
existing measured-heading behavior. These are LingTu adaptations, not upstream
SCAN equivalence.

The FSM's emergency-stop recovery uses finite measured XY speed below its
existing 0.1 m/s threshold. Finite vertical balancing or stair motion alone no
longer prevents recovery. Pose-stage transitions use the same planar meaning
of standstill with their existing 0.03 m/s threshold and yaw-rate requirement.
An already aligned initial pose followed by translation does not require an
extra stop before accepting a moving handoff. Actual intermediate turn stages
retain their position, height, heading and planar-stop requirements. These stop
decisions do not remove the planner's three-dimensional derivative limits,
measured body collision queries or braking checks.

Without a fixed body heading, local target admission uses the reference tangent
at that target, with a nearby-reference fallback at zero terminal velocity.
It no longer places the body along the chord from the current position to a
target after a bend. The optimized trajectory still undergoes its own sweep
checks. During each ordered candidate-validation or collision-lookahead pass,
quiet-speed samples retain the last reliable reference tangent. A slowing tail
therefore does not rotate back to the request's initial yaw at every sample.
An entirely quiet-speed curve with no reliable reference tangent still uses the
measured heading. This direction is used only for collision queries; physical
velocity/acceleration samples remain unchanged. No query-order state is cached
inside Grid. Regression evidence is recorded in
`build/startup-cleanup-20261027/README.md`.

## Primitive body geometry, support contact and pose-route segments

`LocalPlanRequest::body` optionally shares the immutable model selected by
RobotConfig and forwarded with the route. Global XYZ search does not use this
multipart geometry for admission. Parts retain a box, sphere, cylinder or capsule shape
and dimensions. Sphere/AABB collision is exact; the other shapes use the shared
conservative OBB/AABB SAT. The cylinder box orientation preserves the
primitive's actual vertical extent before adding the collision margin. This
avoids the fictitious below-ground corners of an arbitrarily rotated wheel box;
it is not arbitrary mesh collision.

`observedBody` is an optional immutable snapshot of current articulated part
transforms. Grid uses it in preference to the nominal `body`, then transforms
the parts from the planning frame into the collision snapshot's grid frame.
`observedBody` requires a stable `body`; otherwise Grid reports
`body_model_missing`. Task owns each snapshot across asynchronous callbacks.
Replacing the stable model invalidates cached trajectories; a fresh equivalent
articulation snapshot does not replace the guide identity or reset the FSM.
Collision callbacks query the latest snapshot, so a changed part pose can
invalidate an existing trajectory. This asynchronous update is not an immediate
motion-stop guarantee; the caller's final command checks remain necessary.

This mode reads `measuredOccupiedStorage` from the
`LocalCollisionMapView` snapshot. It checks actual occupied cells without
applying the existing twin-cylinder inflation again. Missing raw evidence is
`body_raw_evidence_missing`. Within the snapshot bounds, local occupancy and
braking queries check measured obstacles; unobserved air does not itself block
the body. The known-free and unknown evidence remains unchanged in the snapshot;
the unused free bitmap is no longer a prerequisite for Body collision queries.
The offline BodyQuery diagnostic uses the same occupied-only policy. This does
not prove every swept voxel was observed free; global terrain-support queries
retain their observed-surface requirements. Incomplete body coverage is
still rejected. An explicit body model keeps collision checking active even if
`checkObstacle` is false.

Each part pose prepares its world axes and bounds once, then reuses the same
collision kernel for occupied cells. The offline BodyQuery uses the same value
snapshot. There is no cross-pose cache or change to model dimensions, margin,
contact tolerance, or unknown-space policy.

The per-foot ground-observation proof and `support_required` role were removed
on 2026-10-27. Body collision does not require occupied/free columns beneath
individual feet, including raised feet. Global route support, step and slope
checks remain with the global planner; Body models do not bypass those checks.

The explicit `support_contact` label retains only geometric bottom contact:
occupied material entirely below the source primitive's unpadded lowest point
may touch that part. Global BodyQuery and SCAN share this rule in
[`collision/body.cpp`](../../../collision/body.cpp). Walls, risers and material
extending into the primitive remain collisions. There is no 5 mm penetration /
30 mm ground-gap gate or required free cell above each foot. Available original
terrain hits can refine a coarse cell through `collision/terrain.hpp`; this is
collision geometry, not per-foot load-bearing admission. BODY2's final per-part
bit is reserved and ignored by the reader; current exporters write zero.

For live queries, mapd publishes conservative per-cell bounds of retained
sensor hits alongside the same collision bitmap. SCAN can exclude the voxel's
empty vertical extent when all of those hits are below the part's bottom;
missing hit bounds retain the voxel collision. Bounds include sensor noise and
retain extrema until that cell is cleared, so this does not itself separate
noisy floor returns from obstacles. Failure snapshots include these bounds.
The mapd and navd builds must use the same `MapCollisionLayer` DDS definition.

Point, segment and spline queries use the same selected part snapshot. A fixed
`bodyHeading` covers the measured-to-requested turn before translating at the
requested yaw.
Final braking checks use actual pose and twist. Configured-margin endpoints are
checked directly; each segment's midpoint plus half its point-travel bound covers
the continuous sweep. Braking uses the exact arc midpoint, rather than the chord
midpoint. This avoids an artificial sampling extension behind a departing body
or beyond an arrival. Sweep spacing is at most
`min(0.01 m, 0.25 * resolution)` and the extra sampling margin is at most
5 mm. An already-buffered departure collects its initial overlap cells with
that same maximum sampling coverage; a normally clear start gains no exception.
The source-body sweep and normal-clearance endpoint remain required. This
cell-based exit policy does not guarantee strictly increasing clearance.
Zero-length sweeps and stationary braking use one pose query.

Body mode uses original map evidence and time-aligned dynamic predictions. Part articulation is
held fixed across each future sweep; the query does not predict swing, landing,
joint motion or future roll/pitch. Updating `observedBody` rechecks the next
snapshot but does not establish a future articulated swept volume. Predicted
obstacle cylinders are conservatively enclosed by AABBs and checked against
the same Body in relative motion, splitting at the prediction horizon and
covering motion between samples. No twin-cylinder radius is added to Body
queries. The no-body offline prediction path retains its prior behavior.

Pose-route moving segments (`body` plus `bodyHeading`) must end at the exact
turn centre, with their requested terminal velocity. Backend derives the
internal `preserve_boundary_` condition and the manager reuses its exact P/V/A
boundary restoration and physical-limit retiming. This prevents the ordinary
least-squares spline fit from ending centimetres before a collision-checked
turn centre. Any retimed result still passes full-duration collision validation;
MotionIntent's FSM and target-selection semantics are unchanged. Task and
Backend include immutable body identity in cached-result invalidation, so a
model replacement cannot reuse a spline approved for a previous model.

`test_scan_body`, `test_body_support_map`,
`LocalPlanTask.BodyModelReplacementDiscardsCachedSplineWithoutNewRouteOrFrame`,
`LocalPlanTask.ObservedBodySnapshotsUpdateCollisionWithoutReplacingGuideIdentity`
and `ScanBackend.BodyPoseSegmentsPreserveExactTurnCentreAndStoppedEndpoint`
cover raw geometry, local support, snapshot ownership, model replacement and
endpoint contracts. These native library interfaces do not activate a body
model in a field Product or prove
physical execution; rollout and simulation evidence are recorded separately.

Already-planned autonomous and assisted-teleop commands bypass the teleop
minimum-motion deadband so valid startup and arrival speeds are not erased.
Final maximum speed/yaw limits, input readiness, and driver command ownership
remain in force. Tracking status distinguishes retained frozen progress from
terminated or invalidated trajectories; these diagnostics describe execution
state, not proof that the robot followed the commanded motion.

An assisted-teleop tick waiting for its first asynchronous trajectory pauses
execution without cancelling that computation. Repeated idle resets do not
rebuild the planner again until a new input has arrived. Disabling collision
queries still preserves the configured positive grid resolution needed by
the upstream optimizer's sampling loops.

## Dynamic prediction query semantics

MotionLayer's dynamic confirmation counts a voxel's free-to-hit transition
only in the frame where it occurs. An old free observation is not another
independent motion observation on every later frame. The wall-visibility
regression in `test_motion_layer.cpp` reproduces that specific failure while
retaining the measured wall. This is a LingTu bug fix, not a port of Dynablox.
[Dynablox's evidence lifecycle](https://github.com/ethz-asl/dynablox/blob/main/dynablox/src/processing/ever_free_integrator.cpp)
is the reference for maintaining and retiring historical free-space evidence;
its TSDF counters and classification rules differ from ours.

The scan's occupied voxel and its immediate 3D neighbours are excluded from
same-scan ray clearing. At grazing incidence, noisy returns on a fixed wall
otherwise alternate between `Free` and `Occupied` adjacent voxels as the
sensor moves, falsely confirming motion from the changing visible centroid.
This does not remove hits or change dynamic confirmation thresholds; it keeps
ray-free evidence outside the current surface's voxel quantization band.
`testMovingSensorDoesNotTurnNoisyStaticWallIntoDynamicObject` reproduces that
case; moving, sub-voxel-speed and briefly occluded object tests remain active.

Registered LiDAR clouds do not carry a terrain-height field. Their measured
obstacle `h` retains the existing body-relative value, including for CMU.
SCAN's dynamic classification uses a separate height channel: missing source
heights may be estimated above the observed local ground selected by the
existing support-height calibration. Explicit source heights are preserved.
The worker holds the observation's immutable terrain snapshot; unavailable
ground leaves the old body-relative fallback and all measured hits intact.
The estimated slope near the actual body position only selects a layer for another observed
local fit at the target XY; it does not generate ground across a gap.

`Terrain::classificationSurface()` estimates height from measured hit-bound
midpoints, permits 5 cm fitting noise, and reuses a fit within each observed XY
column. This estimate is only for dynamic-object classification. The stricter
full-bound `Terrain::surface()`, collision exemption and ground-support queries
are unchanged. R12's raw replay demonstrates why the channels differ: the
body-relative cutoff discarded most of the pedestrian's observed lower body;
the collision-fit residual was also too strict for noisy ground-height
classification. These are LingTu sensor/integration repairs, not upstream
SCAN ground-estimation defaults.

For an already confirmed moving track, an implausible vertical centroid speed
with otherwise valid measured planar motion is treated as a missing velocity
fit. The existing unmatched-track TTL predicts from the last accepted state
with decaying confidence; the rejected fit does not refresh that state or its
expiry. Stopped, reversed, excessive-speed and unconfirmed motion retain their
existing rejection rules, while current returns remain measured obstacles.

SCAN's untimed point/segment geometry queries use the predicted object's
position at the request time, accounting for observation age. They do not
collapse the remaining future sweep into a currently occupied region.
The optimizer's candidate validation, FSM collision checks and braking queries
continue comparing robot and obstacle at corresponding times. The explicit
boundary-departure exception still checks the full remaining sweep because
that geometric escape has no arrival-time schedule.

References for time-indexed collision queries are
[TEB's dynamic obstacle constraint](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/include/teb_local_planner/g2o_types/edge_dynamic_obstacle.h)
(`estimateSpatioTemporalDistance` at the pose's arrival time) and
[PredOcc's map queries](https://github.com/siyuanwu99/pred-occ-planner/blob/main/plan_env/src/map.cpp)
(current occupancy versus occupancy at a requested time). TEB's cited dynamic
module is marked experimental and is two-dimensional; we reuse neither its
geometry nor its dependency stack. This query separation is a LingTu adapter
change, not an upstream SCAN feature.

`ScanPredictions` covers current occupancy, future crossing, time-separated
passage, observation age, real measured obstacles, expiration and braking.
Untimed search is only candidate generation: timed validation may reject a
candidate without finding a temporally feasible alternative. This is not a
complete space-time optimizer. The existing cluster cylinder, prediction
horizon and robot dimensions are unchanged. Shape-preserving voxel prediction
from [PredOcc](https://github.com/siyuanwu99/pred-occ-planner) is a future
comparison, not an implemented multi-cylinder replacement.

## Algorithm regression evidence

`tests/nav/cpp/test_scan_planner.cpp` includes `ScanTrajectoryValidation`:
five fixed 0.05 m collision-map scenes sample the emitted spline at intervals
no greater than 2 ms, checking finite values, endpoints, twin-cylinder
occupancy, and velocity/acceleration against the configured limits **including
their explicit tolerances**. A second scenario inserts a full-height wall
after planning and requires a new stationary emergency spline.

The translated-wall regression checks the rolling-horizon contract: the
published first two thirds are collision-free under 2 ms sampling, and an
unsafe tail must be replaced or stopped by the real backend collision/FSM
callbacks before execution reaches it. It does not incorrectly require every
published tail to have been checked before its future-collision callback.

These tests complement `test_local_plan_task`, the SCAN cases in
`test_executor`, `FollowerSpline` in `test_path_follower_core`, and
`test_recovery_sequence`. They are deterministic-input regression evidence;
finite sampling is not a continuous collision proof, a comparison against the
original upstream executable, or a field timing qualification. The planner's
nominal velocity/acceleration values are not strict output-command limits.

## Asynchronous trajectory handoff

The first accepted spline starts at its actual reception time. Waiting for
initial optimization is not executed motion, and the worker uses the same
accepted epoch as the follower. A held trajectory likewise does not consume
physical execution time while it is paused.

During ordinary reference following with tracking near the current reference,
a replacement is planned from the old trajectory's position, velocity and
acceleration at a future handoff time.
Optimization and collision validation use that boundary, including the future
arrival offset for dynamic-obstacle queries. Control points are not translated
or reshaped after validation to hide a tracking error.

Short ordinary local segments whose endpoint is still short of the full goal
prepare continuation before reaching their stopped endpoint. When remaining time is at
most twice the existing handoff lead plus three FSM ticks, the FSM requests
`REPLAN_TRAJ` instead of waiting for the ordinary distance trigger or the last
10 ms. This reuses the existing measured planning/transfer lead rather than
adding a Product parameter. A previously retained safe tail suppresses repeated
equivalent optimization; the final 10 ms continuation fallback remains.

When measured position differs from the current spline reference by more than
`max(ctrl_pt_dist, grid_resolution)`, the FSM also requests a new solve and
bypasses the old spline's nominal future P/V/A handoff. It reuses the existing
branch initialized from measured position and velocity, with zero boundary
acceleration. This separation test selects how to replan; it is not an
additional motion rejection or an extra collision gate. Accepted replacements
still pass the normal optimization, derivative and geometry checks.

The candidate is offered before its handoff time. The Task retains the old
trajectory until the control tick reaches that time and accepts the candidate
within its delivery deadline (20 ms after the planned boundary). Worker and
control thread share the single offer's received/withdrawn state: a withdrawn
candidate cannot subsequently be installed from an old completion. The worker
commits the new active trajectory only after actual acceptance. Late candidates
are discarded and replanned while a valid old trajectory remains available;
existing emergency stopping still applies if that trajectory becomes unsafe.

Ordinary reference replans also compare a successful replacement with the
executing tail. The old tail is eligible only after its complete remaining
interval and the connection from measured pose pass the current 3-D collision
queries, including timed predictions. For the same endpoint and terminal
velocity, a candidate must improve the remaining duration by more than 10% to
replace it. Retention preserves the old controls, start time and trajectory ID;
execution continues at its current time, rather than replaying the prefix.
A failed candidate may likewise yield to that validated tail. New references,
frozen execution and collision-triggered replans bypass this preference; speed
reductions that need a new trajectory retain the adapter's existing reset.
Extending the planning horizon
is not compared against a shorter old tail. After retention, the existing
replan-distance threshold measures progress from the retention point, avoiding
another identical optimization on the next timer tick.

Candidate collision callbacks check the remaining interval after the handoff
time, never a negative prediction time. Pause, replacement, map/guide invalidation
and cancellation invalidate the corresponding offer. These are LingTu worker
integration semantics, not claims that the unmodified upstream implements this
handoff protocol. Contract regressions and actual Thunder MuJoCo measurements
are retained in `build/navigation-closeout-20261023/`; they do not qualify field
timing or terrain outside the tested scenes.

The 2026-10-27 continuation, feasible-lower-limit and lagging-reference changes
are recorded separately in `build/replan-continuity-20261027/README.md`.
Their native clock-delay tests do not establish physical tracking or hard
real-time deadlines.

## OBB tracking continuity

The default OBB route declares body heading independently of travel direction.
Planning therefore receives the same existing X/Y speed capabilities as the
follower; retiming cannot assume that a scalar-speed-feasible diagonal is also
executable on each body axis. These values come from the native configuration
builder, not a second set of product parameters. Failure snapshots retain them.
The follower preserves the velocity direction when applying those existing caps.

Reference fitting also preserves short three-dimensional bends and samples long
straight intervals before polynomial fitting, preventing future corners from
pulling an otherwise straight approach sideways. Its block solve is numerically
compared with the original objective. Moving-boundary seed duration includes the
actual incoming/outgoing speed rather than restarting a stop-to-stop profile.

Native before/after data, including rejected intermediate candidates, and actual
Thunder MuJoCo contacts/velocity evidence are retained in
`build/obb-tracking-20261023/`. The original OBB dimensions and motion limits stay
fixed. A physical component test is separate from Go2, DDS/driver lifecycle and
field qualification; the report records those boundaries explicitly.

## Continuous reference and observed ground support

Executor joins compatible forward, nearly straight, level pose-route legs when
explicit global height metadata is present and there are no stair segments.
Each leg must agree with its travel heading within the route's yaw tolerance
and grid discretization allowance. Multiple alternating corners are supported,
each at most 90 degrees; there is no 1.5 m minimum leg length. An analytic fillet
reference still trims at most half the shorter adjacent leg and uses a radius
of at most 1 m. This curvature conditioning changes the pose-route reference
geometry. It is distinct from the manager's ordered-XYZ repair for ordinary
references; that repair does not mean the pose-route arc preserves every
original corner.

The complete reference is a guide for SCAN, not an executable trajectory.
Executor no longer requires a whole-reference Grid preflight before SCAN can
plan; a distant obstacle or the live window's extent cannot reject the reference
at this seam. SCAN checks its actual optimized spline with the multipart Body.
The follower uses the same zero-preview tangent as Grid, its normal heading
gate and physical body-axis/yaw limits. Curvature speed is limited by radius
times the moving follower's physical yaw-rate capability. The extra `.6` and
terminal-alignment yaw clamps on moving splines are gone. The redundant
per-reference heading array is also removed; actual spline tangents own heading.
`goal_yaw_max_rate` still limits actual terminal alignment. Final control keeps
the selected body at measured pose/twist and checks braking motion.

`supportHeightM` and `supportHeightToleranceM` come from the accepted global
request through Route and LocalPlanRequest. They remain route/global height
metadata; Grid no longer turns them into a requirement for observed floor under
every part's XY projection. That whole-body column gate was a LingTu extension,
not upstream SCAN behavior, and was removed on 2026-10-26. It has no replacement
flag or hidden fallback. Global 3-D support, step and slope checks remain in
their existing owner. Local measured-obstacle collision, swept motion and final
braking checks remain enabled; per-foot ground proofs were subsequently removed.
Only changes in height-metadata availability participate in asynchronous
initialization. Changing its numeric values alone does not reset an executing
spline or reinitialize the FSM; the original values remain in request and
failure evidence. This does not reintroduce the removed local admission condition.

The formal nominal-body MuJoCo failure also contains a separate contact-part
collision with ground voxels. Removing the column gate does not resolve that
subvoxel evidence gap or establish motion acceptance. The removal's checks and
replay evidence are retained in `build/nav-support-gate-removal-20261026/`.

After a failed SCAN attempt, only a stationary robot near the original entrance
can restore the stopped-turn stages. A mid-route failure, manual takeover or
resume retains the joined reference and goal
and rebuilds local timing from observed motion; it must not impose an old turn
centre at a new position. The historical native producer emits separate
translation and rotation edges. The current compiler also accepts forward
tangent XYZ+yaw samples, with an explicit check of both endpoint headings.
Arbitrary independent heading profiles remain outside that interface. Height
metadata remains necessary for the analytic fillet, while original-polyline
continuation does not need it; explicit stairs keep their own path.
The historical `--keep-turn-stages` ablation relied on omitting height metadata.
It is retired because that omission no longer forces ordinary corners to stop.
Reproduce the old ablation with its retained historical binary and records.

Current source cleanup and software-loop evidence, including rejected variants,
are recorded in `build/motion-cleanup-20261027/README.md`. The raw-reference-only
experiment failed the wall S channel even with the physical speed cap, so the
fillet conditioning remains. Optimizer no-progress restarts are inherited
upstream behavior and remain unresolved; they are not counted as a removed
LingTu admission gate. XYZ optimization alone still does not establish a joint
yaw-rate timing guarantee or qualify field motion.

The final matched Executor/SCAN/follower/braking software loops reach all four
fixed cases. They use a 33-part Go2 nominal model, 0.05 m horizontal clearance
and ideal SE2 integration, without contact physics or Product/field execution.
Obstacle cases can wait longer for the first actual optimization before falling
back to the original stages. That earlier cleanup run retained one failing
sustained diagonal-intent speed-continuity assertion; its report preserves
that result separately from the later continuation work in
`build/replan-continuity-20261027/README.md`.

### Historical fillet and physics experiments

Earlier 2026-10-23 evidence is retained in `build/navigation-team-20261023/`. The first
fixed-heading blend took 79.22 s on the supplied 22 m physical route versus
55.26 s for its stopped-turn baseline; that regression is preserved. The revised
tangent/fillet V3 activates the merge and reaches the goal, but takes 68.68 s
versus 53.98 s with the same binary retaining the original turn stages. Both
record zero illegal contacts; the 27.2% time regression fails performance
acceptance. V4 keeps the admitted terminal heading during a small arrival
correction, avoiding a 180-degree turn toward a short return tangent. It also
omits a pose-stage projection connector already within the existing arrival
tolerance, retaining the measured start and full candidate collision checks.
All 29 pose-route and 45 SCAN backend/FSM regressions pass. V4's same-binary
physical pair reaches the goal in both modes: original stages 53.32 s, merged
52.78 s. Stages drop from six to four, alignment from 6.18 to 3.40 s, and measured
concurrent translation/turning lasts 6.0 s rather than zero. Spline RMSE/peak
falls from 6.016/22.006 to 5.813/16.767 mm; goal XY error rises from 3.202 to
7.180 mm. Both runs record zero illegal contacts over 10,664/10,556 substeps.
This verifies open-space continuous turning, while the single pair's 0.54 s
(1.01%) saving does not establish a substantial speedup. The whole merged stage
still receives a conservative 0.488 m/s cap from existing lateral capacity and
heading-alignment threshold, reducing straight-line median speed from 0.561 to 0.442 m/s.
In that earlier experiment, the `nav` Product selected the existing SCAN parameter
`scan_follower.heading_error_rad=0.4` (teleop and other Products retain their
own defaults). On the same binary, route, map and analytic-plane world, this
reduced blended execution from 52.78 s to 47.00 s (10.95%), preserved 6.02 s
of simultaneous translation/turning, zero illegal contacts over 9,400
substeps, and kept spline RMSE/peak at 6.994/20.966 mm. This is one matched
component pair, so it is a candidate performance improvement rather than a
field or repeated-trial qualification. On 2026-10-31 the Product override was
removed: ordinary XYZ routes now use the local spline heading and the existing
upstream 0.8-rad default. SCAN no longer borrows the geometric Path follower's
0.35-rad exit threshold. The earlier fused-turn timing result does not establish
natural motion for this revised execution path.

The 2026-10-26 multipart short-S software closed loops and intermediate failures are
retained in `build/scan-adaptation-20261026/turn-demo/`. They use a fixed-joint
model, zero extra margin and ideal speed integration, not contact physics. No
matched plain-reference-plus-SCAN competitor existed in that historical set.
The 2026-10-27 comparison above rejects blanket removal and retains the
conditioning reference. The upstream-difference review is in
`build/scan-upstream-review-20261027/README.md`.

The final pair uses the existing analytic plane only within the common finite
mapped floor extent. Finite-heightfield contact failures remain unresolved;
subdivision and precision trials did not qualify a fix. Native CCD, contact
classification and 3D navigation checks remain active. The report preserves
those failures separately from the final pair. The final Windows Release
navd builds with these repairs and the global task's 10 s default cooperative
planning deadline; neither result qualifies Product startup or field motion.
The 0.6 m negative corridor probe returns `global_planning_timeout` after
10.007 s of search (10.261 s to first response), sends no motion command and
records no illegal contact in 164 physical substeps. This bounds response time;
it does not prove that search exhausted all routes. Support-query caching alone
did not establish a speed improvement: the retained unbounded run took 125.14 s.
The supplied route tests execution, not
automatic global route generation. Corridor and dynamic fixtures separately
exercise global search. Crossing passed without replacing the global route;
persistent blockage exhausted local recovery and remains unresolved. These
changes are LingTu integration behavior, not upstream SCAN equivalence or
field qualification.
