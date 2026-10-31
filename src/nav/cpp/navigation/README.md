# Navigation execution

This directory owns execution of an already admitted route or assisted-motion
intent. It does not own global search, DDS, backend selection, or final robot
velocity.

## Data flow

Explicit `Route.posePoints` plus `Route.body` supports fixed-heading movement,
forward tangent motion and genuine in-place turns. `pose_route.*` preserves
same-position turns. Sampled edges may translate and change yaw when both
headings match forward travel within the route's yaw tolerance. Arbitrary
independent yaw/crabbing on one moving edge is not represented by this tangent
interface and is rejected explicitly. Mixed explicit stair/pose routes retain
their existing boundary. Ordinary `Route.points` retains its existing behavior.

Compatible intermediate forward corners share one SCAN reference. Existing
fillet conditioning remains available; when it does not apply, the original
ordered XYZ polyline can be passed directly to SCAN. Its corner is no longer a
local stopping endpoint. SCAN validates its resulting spline, and the follower
uses that spline's tangent while retaining the requested final yaw.

Separate turn stages validate the actual centre, measured yaw, turn sweep and
reconnection to the next segment. Intermediate arrival uses the existing goal,
spline-finish and support-height tolerances after that geometric check; a
centimetre-perfect base position or a separate observed-body snapshot is not an
additional prerequisite. Movement into a separate turn still settles first;
an aligned turn hands measured motion to the next translation without waiting
for another zero-velocity sample. The reference generation changes on stage
transition, not each control tick. The terminal heading is required only when
the accepted goal explicitly requests one; path headings still guide movement
and collision checks. Cancellation, replacement and frame changes discard pose
state and prior model candidates. Endpoint FinalControl remains required.

The component probe in `tests/nav/cpp/benchmark_body_execution.cpp` runs
the real global planner, SCAN and Follower on fixed map fixtures. Its internal
feedback is kinematic command integration; the separate Thunder MuJoCo bridge
uses policy physics, contacts and ray-cast sensors. Historical sole-contact
experiments are not current per-foot admission requirements. Select the body
through the platform configuration and record the actual part set used by a
trial. See `docs/design/navigation-body-model-and-motion-posture.md` and
`docs/operations/error_record.md` for current failures and validation boundaries.

Articulated arrivals measure a half-second stable 3D pose/yaw window so short
balancing velocity spikes cannot prevent completion forever. Every sample
must still be in the task's goal region, timestamps must advance, and drift
restarts the window. Yaw acquisition first enters half the task tolerance and
then retains the full tolerance while dwelling, avoiding boundary chatter.
Ordinary routes without observed body geometry keep the existing rate check.

Entering the goal region does not interrupt a moving SCAN terminal spline.
While its applied-command baseline exceeds the existing 0.03 m/s quiet bound,
the spline and normal collision checks continue so its zero-terminal-velocity
reference can decelerate. Only then may arrival settling or a separate terminal
yaw stage reset the follower. An initially stationary robot already at its goal
retains direct settling; CMU arrival and urgent safety stops retain their own
behavior. Odom-frame references retain the real final route segment if the
measured terminal connector collapses to one point during deceleration.

```text
GlobalPlanner -> Route -> Executor -> LocalPlanner -> LocalPlan -> Follower
                              |                         |
                              +------ Recovery ---------+
                                                        |
                                                 MotionCandidate
                                                        |
                                           endpoint FinalControl
```

- `route.*` defines the activated global `Route`, prepares the complete
  reference, and computes a bounded segment where the execution mode needs it.
- `executor.*` calls one already configured `LocalPlanner`, then one
  `Follower`, and returns execution state plus a pre-safety command.
- `recovery.*` is invoked only when the typed planning state or measured
  progress requires recovery.
- `state.cpp` owns odometry-derived kinematics and measured progress.
- `stairs.*` resolves measured stair surfaces against the actual 3D route and
  owns Executor's entry/profile/exit phases. It does not recognize point clouds,
  run global search, choose concrete platform models, or publish commands.

Recovery is an execution branch, not a second global route. Its verified
`path_body` is sent directly to the recovery follower while the normal route
follower is held. Only an exhausted recovery is reported to the endpoint as a
typed `local_recovery_exhausted` event; the goal owner may then request one
fresh global route. Inspection navigation uses the same retained-goal retry
path as an ordinary goal: a temporary local failure or replan with no path
does not end the waypoint. Explicit task cancellation and inspection action
failures retain their existing business policy.

The endpoint's goal controller also retains an accepted initial goal when an
ordinary search finds no path or times out. It uses the existing pending request
and retry delay, takes a fresh start pose, and keeps the original task identity.
Until a route is activated the state remains Planning. Invalid requests,
algorithm exceptions, map replacement and explicit cancellation retain their
terminal handling; preview requests never start this motion-task retry loop.

Recovery searches against the current geometry on each attempt. Finishing a
recovery action or rejecting a candidate does not require another map sequence,
epoch or generation before trying again. The endpoint owns input freshness;
Recovery owns candidate selection and its remaining-path collision checks.

An explicitly supplied collision Grid remains authoritative in recovery even
when `checkObstacle` is false, matching normal SCAN Body queries. The recovery
search's boolean controls point-cloud fallback, not whether a supplied Grid is
queried. An unchanged but current map can therefore produce another checked
candidate, or reject every candidate when the measured obstacle is still there.

The five responsibilities are:

| Owner | Produces | Does not do |
| --- | --- | --- |
| Global planner | A route through the saved 3D map | Local control or recovery |
| LocalPlanner | An executable local path or spline using live collision data | Global search, task progress, recovery state, or final command publication |
| Recovery | Recovery session state and its bounded translation/rotation search | Replace the global route or publish velocity |
| Executor | Route/recovery progress and a candidate command through the follower | Final authority arbitration |
| FinalControl | The checked final command | Search routes or choose recovery candidates |

`Recovery` is Executor's recovery state, not a separate global planner. It
tracks the selected candidate's progress and retries. Its candidate search is
an internal implementation detail of the same component; no other module
owns, schedules, or calls a recovery planner.

`LocalPlanner` is the module name. SCAN is the default algorithm behind that
interface; CMU remains an explicitly selected alternative. Executor calls the
same module interface for either algorithm.

The local boundary uses `local_no_path`, `local_blocked`, and
`local_near_field_stop` for its stable status vocabulary. `execution_stalled`
belongs to Executor: it compares requested motion with measured progress.
SCAN's `searchReason` (also surfaced as the execution reason when relevant)
retains the specific search diagnostic. A diagnostic is not another owner of
the motion state machine.

Planner capabilities are derived at the local-planner boundary. Executor and
recovery consume these capabilities; they do not branch on Scan/Cmu. SCAN
does not consume the optional 2D traversability grid, and its request identity
ignores that grid's generation. The grid remains available to the CMU backend
used by other Products.

## Local planning boundary

`Executor` submits one `LocalPlanRequest`:

```cpp
struct LocalPlanRequest {
  RobotState robot;
  LocalObjective objective;      // RouteTarget or MotionIntentTarget
  LocalRouteView reference;      // Complete planning-frame reference
  EnvironmentView environment;  // obstacles, collision, traversability
  PlanIdentity identity;
  PlanClock clock;
};
```

`LocalPlan` carries one executable `FollowTarget`:

- CMU returns `PathTarget`.
- SCAN returns `SplineTarget`.

The Follower dispatches that variant internally. Executor converts preview
geometry and robot state between the target's frames, but does not select
CMU/SCAN, inspect backend strings, or manage SCAN task reuse and continuity.
The endpoint composition root chooses and configures the Planner before
constructing Executor.

Composition selects `PlanningFrame::Map` for SCAN and `PlanningFrame::Odom`
for CMU. SCAN's complete reference, robot kinematics, collision queries and
spline tracking share map coordinates; the Follower still returns body-frame
velocity. Small map-to-odom corrections update the measured robot pose, not
the reference generation or spline execution clock.

The complete map reference is prepared once after route activation and height
calibration. The map-frame SCAN path reuses that reference; the odom-frame path
continues to update its bounded local segment. Suspension or a frame-epoch
reset discards the reference and pending local work; the next valid input
rebuilds it. A new goal and cancellation use `setRoute()` and `clear()`
respectively, which reset the planner and follower.

The global route is one ordered polyline, not a stream of new goals for its
individual points. LocalPlanner owns local target selection and replanning along
that polyline. Execution feedback goes to the endpoint's goal lifecycle; Executor
and LocalPlanner never invoke global search themselves.

## Executor ownership

Executor may:

- activate and clear a Route;
- project progress and choose the local route segment;
- request a local plan;
- invoke recovery;
- follow the returned target;
- report target, preview path, control hints, and a pre-safety velocity.

Executor may not:

- run global search;
- select/configure a local backend;
- decode or publish DDS;
- perform final authority or safety arbitration;
- duplicate CMU/SCAN continuity state;
- branch on free-form planner failure strings.

The endpoint `FinalControl` remains the sole owner of shaping, authority,
hard-stop policy, and the final velocity publication.

## Stair segments

The endpoint reads recognized, map-bound stair geometry and attaches matching
`StairSegment` values to `Route`. Matching samples the actual 3D route against
the measured tread elevations and flight width; an XY/AABB overlap does not
attach a stair requirement. Both directions, partial flight routes and multiple
flights use the same resolver.

Before entry, Executor shortens the local reference to a landing position that
keeps the full robot extent before the first riser. After alignment and measured
stop it requests `stairs_up` or `stairs_down`. The driver maps that requirement
to the installed model/profile, owns any required Grounded transition, and
reports readiness only after actual profile readback and motion readiness.
Missing, stale, unsupported, or mismatching feedback holds the same route.

Local collision planning remains active while traversing. Executor limits speed,
rejects a local lateral detour outside the observed flight, and suppresses flat
ground recovery rotations/translations there. Existing measured-stall stopping
still holds zero; it does not complete or terminate the navigation task.

The driver receives `restore` only after the complete footprint clears onto an
observed exit landing and measured motion is quiet. Until then Executor holds
zero output and retains the stair profile. Once restoration has been requested,
temporary odometry loss during Grounded/Stand does not interrupt its renewal.
Connected flights needing the same profile can keep it
while approaching the next entrance. A goal outside the flight cannot complete
until clearance and restoration are confirmed. If that goal lies close to the
last riser, the local reference may extend onto the observed landing within the
goal's existing acceptance radius; the requested goal is unchanged. When no
whole-body exit fits that radius, execution waits with `stair_exit_target`.
A deliberate goal on a tread can complete while retaining the stair profile.
`clear()` and replanning retain a profile
already in use on a flight; a changed crossing direction holds on the existing
model until a landing permits switching. The commandless
`tickStairTransition()` allows the endpoint to finish a requested handoff or
landing restoration while ordinary motion is gated. These native contracts do
not establish that an installed robot model can climb the recognized stairs.

## MuJoCo single-floor goal check

Run from the repository root with its Python development environment and freshly
built native binaries. Prepare a saved `map.pcd` / `octomap.ot` bundle from the
same industrial-park geometry as the MuJoCo scene, then run:

```sh
python sim/scripts/mujoco/native_navigation_acceptance.py \
  --manifest config/acceptance/mujoco/scan_goal.json \
  --product-map <same-source-map-directory> \
  --out-dir artifacts/scan-goal
```

This entry starts and stops the formal `nav + scan` Product through
ProductControl. Robot, policy, spawn and native parameters come from RunPlan.
MuJoCo navigation uses truth localization by default; explicit
`env_config={"backend": "mujoco", "localization": "ltlio"}` selects the
separate SLAM path. Mapping Products retain their estimator-backed defaults.
Truth replaces localization only: registered clouds contain actual simulated
LiDAR returns, without synthetic ground patches.
Truth poses are published independently of raycasting; delayed scans retain
their capture pose without rewinding live odometry. Viewer refreshes use owned
snapshots off the physics loop. Endpoint input holds freeze both SCAN planning
time and Follower execution time without replacing the active reference.

SCAN's final command check sweeps the actual body's braking path through the
same LiDAR collision bitmap used by local planning. If the requested speed
cannot stop in the available clearance, it scales the complete twist down
along the same arc and retains the active trajectory. Occupied current space,
unsafe measured motion, or an unavailable safe prefix still stops execution.
The sweep keeps measured body height: an ascending global route cannot be
used as evidence that the physical robot has already climbed the stairs.

Disabling automatic recovery does not disable stall stopping. If a commanded
robot makes no measured progress for the existing recovery observation interval,
the executor holds zero output with `execution_stalled`. A new route or
operator intent clears this hold; periodic local replanning does not. This keeps
an unqualified locomotion policy from repeatedly pushing against the same step.

Stall progress follows the previous requested motion in the planning frame.
Translation accumulates signed horizontal displacement along the commanded
path-following direction; heading alignment accumulates rotation in the
requested direction. Reverse drift, cross-track slip, body heave and incidental
yaw do not refresh a translation timer. Instantaneous velocity alone does not
prove progress. The existing distance/angle threshold is capped at half the
requested travel accumulated in that observation window, so a slow command is
not required to cover an impossible fixed distance. This is a stall detector,
not a locomotion tracking-accuracy qualification.

When local planning returns `Pending`, no executable trajectory is available:
the executor sends zero and pauses this observation window without discarding
earlier motion evidence. A safe active SCAN trajectory is returned as `Ready`
even while its replacement is being prepared, so it keeps normal progress
monitoring. `Blocked` and near-field holds still count toward failed motion.

A translating recovery also rejects excursions beyond its existing cross-track
limit. Completion requires both along-path progress and actual XY proximity to
the exit; merely projecting onto the endpoint cannot finish recovery. A rejected
action advances the existing checked candidate sequence. These checks do not
raise recovery speed, change braking limits or establish that policy 4998 can
execute low-speed lateral commands.

The goal is `(56, 32, 0.3)`. A successful global plan or accepted command is not
arrival: the report requires physical travel, native `REACHED`, final position,
collision evidence and terminal zero acknowledgement. The Viewer shows the
global Route, local spline preview and actual traveled path. Keep failures in
the report; this entry is not a claim that long-range navigation already passes.
Use a fresh output directory per run. Contact results are finalized separately
from best-effort live display updates and must match the Product session.
