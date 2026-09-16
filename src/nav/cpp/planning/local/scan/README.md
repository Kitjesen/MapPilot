# SCAN local planner

This directory contains LingTu's ROS-free port of
[wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) pinned to
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.

The algorithm sources live under [`upstream/`](upstream/UPSTREAM.md). For the
same valid inputs, parameters and callback order, process and storage adapters
must preserve the pinned algorithm's map queries, Projected DynAStar path,
rebound control points, B-spline, six-state FSM transitions and controller
command. The inherited correctness repairs documented below are explicit
exceptions; this port does not claim unconditional output equivalence.

This is the port's equivalence contract, not a claim that a passing unit suite
proves every upstream execution or robot behavior. Upstream differential
results, MuJoCo runs, and field measurements are separate evidence.

The inherited optimizer's intersection-state and collision-sampling repairs
are documented in [`upstream/UPSTREAM.md`](upstream/UPSTREAM.md). Successful
intersection and optimization equations remain unchanged, but a candidate
that fails the corrected collision check is rejected or reoptimized.

LingTu-specific code is only the boundary:

- `grid.*` exposes Mapd's complete inflated bitmap through the official
  `GridMap::getInflateOccupancy()` query. It does not build, inflate, score, or
  smooth another map. The segment query also reads this bitmap, traversing
  cells between successive front/rear cylinder-center samples without
  allocating a temporary map or container.
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
readiness owns `local_collision_stale`; an expired collision layer is not
reported as an official `no_path` result.

Mapd uses the upstream map profile: `0.05 m`, `200 x 200 x 100`, `5 m` rays,
`p_hit=0.85`, `p_miss=0.30`, `p_min=0.12`, `p_max=0.98`, and `p_occ=0.80`.
Inflation is maintained incrementally when occupancy changes. The SCAN profile
disables time-based occupancy decay: cells clear only through ray misses or a
rolling-window eviction, as in the pinned upstream GridMap. Thunder replaces
the upstream robot geometry with its configured twin-cylinder radius and
offset. The ring buffer and packed collision bitmap only reduce memory moves
and DDS payload; they preserve the logical GridMap result. Product speed limits
may be lower than the upstream launch defaults; they are explicit runtime
inputs and do not change the SCAN equations.

`nav` and `teleop_avoid` select `scan` by default and target 100 Hz for the SCAN
FSM and controller, with the upstream 20 Hz future-collision schedule.
`cmu` remains an explicit backend option and is not deleted or used as a silent
fallback.

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

Product tuning is resolved by `lingtu.assembly.parameters` under
`scan_planner.*` and `scan_follower.*`, then recorded in the RunPlan and passed
through Endpoint configuration to the planner and controller. C++ struct
initializers are standalone library defaults, not Product configuration.
Collision input freshness belongs to `local_collision.max_age_s` and is
shared by the Endpoint input gate and recovery; it is not an optimizer knob.

## Execution feedback and runtime speed

Reference and odometry updates are input events; they do not replace the FSM
or future-collision timers. Accepting a reference and advancing the FSM are
separate callback actions. A reference or speed change must not keep injecting
the same event while a replan is waiting for its timer callback. The serialized
worker's 100 Hz and 20 Hz values are scheduling targets, not hard real-time
guarantees: planning work and operating-system load can delay both callbacks.

Executor owns the effective speed request. SCAN receives that limit as a
planning parameter and replans with it; the published spline carries the
corresponding speed metadata. A speed reduction invalidates the old trajectory
until a new publication arrives, even if the operator restores the old speed
while planning. The replacement starts from observed motion rather than the
invalidated spline's derivatives. The manager, optimizer, AStar pool and
trajectory ID sequence are retained. Speed increases and ordinary replans may
continue the current valid trajectory until a replacement is available.
Backend owns this retention decision; Task propagates an explicit pending or
stopped completion instead of resurrecting its cached result.
The controller uses the same effective limit. This changes explicit runtime
inputs without changing the upstream optimization or control equations, and
does not introduce a separate speed-scaled trajectory clock.

Endpoint final control reports execution holds through
`Executor::pauseLinearMotion()`. Executor pauses both the planner's trajectory
time and Follower's retained progress; an allowed rotation remains available.
Ordinary input holds retain the trajectory. A requested replan invalidates the
old planner result and follower target together. Resetting only Follower would
replay a retained spline from its beginning.

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
