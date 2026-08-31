# Local Planning and Tracking Contract

Status: current algorithm/control contract
Audience: native endpoint and navigation test maintainers
Replaced by: not replaced

This document is the canonical human-readable contract for LingTu local path
planning algorithms, scoring, path tracking, and their parameter surfaces. It
complements `local_planner_io_contract.md`, which summarizes the native planner
I/O boundary. If the two repeat algorithm,
tracking, or parameter details, this document wins for those details.

## 1. Functional boundary

The navigation control boundary is contract-first and uses
`Port -> Wire -> Transport` terminology:

| Producer port / role | Wire / payload | Transport / consumer |
| --- | --- | --- |
| navigation goal input | global-plan request and global path | direct C++ call to the global planner inside the native endpoint |
| global path / local route segment | atomic local-plan invocation and body-frame path/trajectory | direct C++ call to `LocalPlanner` inside `Executor` |
| local path or exact B-spline | tracking invocation and body-frame `vx/vy/wz` | direct C++ call to `PathFollower` inside `Executor` |
| pre-safety velocity | final command after safety and control-authority arbitration | native typed DDS `rt/nav/cmd_vel` to ThunderV4 |

`LocalPlanner` and `PathFollower` are different functions:

- `LocalPlanner` chooses a collision- and terrain-aware local path.
- `PathFollower` tracks that already-selected path and produces velocity. It
  does not search obstacles, score candidate paths, or replace the local
  planner.

The native endpoint control modes `autonomy`, `teleop`, and `teleop_avoid` are
mutually exclusive. The Host must not add a competing command writer.

The executable implementation paths are:

- route navigator: `src/nav/cpp/navigation/executor.hpp/.cpp`, with private
  route geometry in `route.cpp` and execution state in `state.cpp`
- local planner: `src/nav/cpp/planning/local/planner.hpp/.cpp`
- CMU backend: `src/nav/cpp/planning/local/cmu/backend.hpp/.cpp`
- SCAN backend: `src/nav/cpp/planning/local/scan/`
- candidate scoring: private implementation inside `planning/local/cmu/backend.cpp`
- recovery candidate search: `src/nav/cpp/planning/local/recovery.hpp/.cpp`
- recovery lifecycle and retry ownership: `src/nav/cpp/navigation/recovery.hpp/.cpp`
- path follower: `src/nav/cpp/tracking/follower.hpp/.cpp`
- native endpoint assembly: `src/nav/cpp/endpoint/nav/main.cpp`

The source tree follows the data transformation, not an upstream project name:
global and local path generation live under `planning/`, path tracking lives
under `tracking/`, and `Executor` owns route/intent execution plus the
backend-neutral recovery lifecycle. CMU and SCAN are two independent adapters
behind the single Local Planner seam; neither backend calls the other.

### 1.1 Native Local Planner request

The C++ public entry is
`nav_kernel::local::Planner::plan(const LocalPlanRequest&)`. One request is one
atomic planning frame:

| Field | Current meaning |
| --- | --- |
| `robot` | body pose plus planning-frame linear velocity, acceleration, and yaw rate |
| `objective` | either a `RouteTarget` carrying the local route segment or a `MotionIntentTarget` carrying assisted-teleop intent and its straight guide |
| `environment` | non-owning obstacle, complete collision-snapshot, and traversability views in the planning frame |
| `identity` | frame, obstacle, and traversability generations for retained-state invalidation |
| `clock` | monotonic planning time plus the optional execution clock used while spline tracking pauses for heading alignment |

`environment.obstacles` is the non-owning `N x 4` view
`[x, y, z, height]`. `environment.collision` carries the optional complete
local collision snapshot, while `environment.traversability` carries the
optional 0-100 risk grid. The constructor supplies robot and path-library
parameters. Recovery is not a Planner entry point: Executor always calls the
selected backend's single `plan()` entry first and passes the same complete
frame to its own recovery controller only after no-path or execution-stall
policy triggers. The planner exposes no pose, goal, or grid setters, and an
omitted grid in one request cannot reuse a previous request's grid.

Both backends consume `LocalObjective`; assisted teleoperation selects its
`MotionIntentTarget` alternative instead of calling a second Planner method.
CMU returns a geometric `PathTarget`. SCAN plans from the intent guide, applies
the requested speed and terminal direction limit, and returns an exact
`SplineTarget`. The endpoint supplies the same collision snapshot, identity,
measured body motion, and traversability fields used by autonomous planning.
Neither backend loads or calls the other.

The upstream CMU smart-joystick contract is differential: its lateral stick is
steering, while direct body-frame lateral velocity exists only in manual mode.
Thunder is holonomic, so LingTu adds one explicit, bounded exception for a pure
lateral assisted intent. CMU must produce an emitted path whose endpoint and
fixed `0.5 m` lookahead direction remain within `10 deg` of that intent; the
Follower then translates along the path while holding the body yaw captured at
intent admission. If no such path is collision-free, assisted teleop stops with
`teleop_assist_lateral_blocked`. It must not silently turn the request into
reverse travel or automatic rotation. This exception supports free lateral
translation and bounded yaw correction; it does not claim that the upstream
CMU path bank can perform arbitrary holonomic lateral detours around a blocking
obstacle.

`Executor` still owns route progress. It projects the body onto the nearest
forward 3-D route segment, retains every bend, and clips the final segment at
the exact configured arc-length horizon before calling Local Planner. It does
not drive back to a sparse segment's preceding waypoint and does not collapse
the segment to one target. Two routes with the same endpoint therefore remain
distinguishable. All pointers in the input are read-only and valid only for the
synchronous call.

SCAN scores and smooths against the complete route segment. CMU remains a 2-D
path-bank backend, but no longer blindly collapses every slice to its terminal
point: a straight slice keeps the terminal target, while a slice with a
significant interior bend first uses the nearest salient bend as its guide.

### 1.2 Continuous-trajectory contract

The selected backend is explicit: `cmu` keeps fixed-library geometric planning;
`scan` builds a bounded 3D query grid, replaces each colliding route segment
with projected A* that expands only XY while interpolating Z from the segment
endpoints, then applies XY-only rebound L-BFGS and produces a continuously
collision-checked cubic B-spline with time, position, velocity, acceleration,
yaw, and yaw rate. SCAN retains a safe prefix only while frame and route
generations match. Map generations trigger fresh collision evaluation but do
not create a second authoritative map; field `mapd` remains the owner.

The field collision wire is `/maps/local_collision`. `complete=true` means all
occupied voxels inside the published AABB are present; it does not claim that
unknown cells are observed free. Incomplete, non-live, stale, malformed, or
ROI-undercovering snapshots fail closed. Source stamp orders map samples;
steady receiver stamp determines freshness inside `navd`.

`LocalPlan` carries one typed status, one `FollowTarget` variant, and explicit
`ControlHints`. A ready CMU plan owns a `PathTarget`; a ready SCAN plan owns a
`SplineTarget`. SCAN preview geometry is sampled on demand rather than stored as
a second authoritative path. Backend identity and detailed failures live in the
separate debug/status snapshot, never in control branching. Recovery state is
reported by `ExecutionOutput`, not selected by a Planner backend. `Follower`
dispatches the target variant internally: it tracks the exact B-spline for SCAN
and retains the existing geometric lookahead branch for CMU.

## 2. CMU path-library model

The planner loads four offline-generated assets from
`src/nav/cpp/planning/local/cmu/paths/`:

| Asset | Runtime meaning |
| --- | --- |
| `pathList.ply` | 343 primitive trajectories and their group IDs |
| `startPaths.ply` | 7 canonical output path-group centerlines |
| `correspondences.txt` | precomputed 2 cm XY voxel-to-primitive correspondence table |
| `search_radius.txt` | radius used to generate and interpret that profile's correspondence table |

The Go2 profile uses `0.55 m`; Thunder uses `0.60 m`. Startup fails if the
radius metadata is absent or malformed. Runtime uses the loaded radius in the
same warped-grid equation as the offline generator, so a correspondence table
cannot silently be interpreted with another robot's radius.

The fixed search cardinalities are:

- 343 primitive trajectories;
- 7 path groups;
- 36 online rotations, at 10 degree intervals;
- at most `343 * 36 = 12,348` rotated primitive hypotheses per planning
  attempt;
- `36 * 7 = 252` rotation/group score bins.

The rotation angle for index `d` is `10*d - 180` degrees. Before obstacle
scoring, the target-direction gate removes disallowed rotations, so a normal
cycle may evaluate fewer than 12,348 hypotheses. The number 12,348 is the
maximum search space, not a promise that every hypothesis runs every tick.

### 2.1 Upstream compatibility boundary

The production name is **LingTu CMU backend**. It is an enhanced port of the
CMU candidate-path core, not an "official Go2 fully equivalent" runtime.

| Area | Current LingTu behavior | Compatibility status |
| --- | --- | --- |
| Candidate bank and selector | 343 primitives, 7 groups, 36 rotations, CMU direction/collision voting, and progressive range/scale reduction | Core behavior retained |
| Robot collision table | Go2 `0.55 m` and Thunder `0.60 m` libraries; runtime reads each bank's radius metadata | Closed between generation and runtime |
| Obstacle density | CMU Products voxelize registered obstacle returns at `0.05 m`; LingTu still adds stateful fusion, decay, dynamic prediction, and bounded output | Same nominal voxel size, different preprocessing |
| Terrain analysis | CMU Products currently consume obstacle evidence only; the upstream terrain-cloud height/cost path is not wired as an equivalent input | Not equivalent |
| Route intent | A straight local slice uses its terminal target; a bent slice guides first toward its nearest salient bend | LingTu extension |
| Tracking | Ordinary CMU paths use the fixed Go2 profile; pure Thunder lateral intent uses the bounded yaw-hold exception above | Upstream-compatible ordinary branch plus explicit LingTu holonomic exception |
| Recovery and command boundary | Executor recovery and planner footprint review remain active; the endpoint applies mode limits, direct-command safety, smoother commit, and authority | LingTu extension |

`point_per_path_thre=2` is retained as the upstream starting value because CMU
Products restore the upstream `0.05 m` nominal obstacle voxel size. That removes
the known `0.08 m` density mismatch, but it does not prove field equivalence:
LingTu fusion and point caps can still change hit counts. Recorded Go2 and
Thunder clouds must be replayed before this threshold is called calibrated.

The 2 cm correspondence is an XY lookup optimized for the CMU path library,
not a general 3D occupancy voxel map. Each accepted obstacle point is rotated
into a candidate orientation, mapped into the warped 2 cm lookup grid, and
then attributed to every primitive listed for that voxel. The point's fourth
field is planner height/cost evidence; it is not LiDAR reflectivity.

Before near-field stop, planner-cloud construction, rotation scoring, and
recovery checks, the native core admits only finite body-relative height
evidence at or below `obstacleHeightMax` (default `1.20 m`). The endpoint
exposes the same value as `--local-planner-obstacle-height-max-m` and
`LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MAX_M`, clamped to at least `0.2 m`.
This body-envelope ceiling prevents roof/overhead returns from blocking ground
motion. It is an admission filter, distinct from
`obstacle_height_thre=0.2 m`, which decides whether an admitted point counts as
a collision hit.

## 3. CMU gates and score

### 3.1 Gate order

For a rotated primitive `(d, p)` in path group `g`, the native core applies
these gates before it contributes a score:

1. **Rotation direction gate.** With the default `dir_to_vehicle=false`, the
   rotation direction must lie within `dir_thre` of the local target direction.
2. **Obstacle collision gate.** Correspondence hits are counted per rotated
   primitive. With the production threshold `point_per_path_thre=2`,
   `collision_hits >= 2` is a hard rejection.
3. **Assisted-teleop direction gate.** A `MotionIntentTarget` additionally requires
   both the primitive end direction and the canonical output-path end
   direction to remain within the requested maximum deviation. The emitted
   canonical path's `0.5 m` lookahead direction is checked as well, because that
   is the direction the CMU Follower actually executes. A `RouteTarget`
   uses the rotation prefilter but does not enable these extra hard direction
   gates.
4. **Traversability hard gate.** When traversability scoring is enabled, the
   maximum raw cell risk intersecting the rectangular footprint swept along
   the rotated/scaled canonical group path is
   compared with `traversability_hard_cost`; `risk >= hard_cost` is a hard
   rejection.
5. **Positive-score gate.** A candidate whose direction/height score is zero
   or negative does not contribute to its group bin.

The optional rotation-sweep obstacle gate (`check_rot_obstacle`) is applied
when selecting the final rotation/group bin. It is separate from the per-path
collision count.

### 3.2 Native scoring formula

For each surviving primitive, define:

```text
delta = angular distance(
          local target direction,
          primitive end direction + rotation angle)

q = min(abs(dir_weight * delta), 3.60)
direction_factor = max(0, 1 - q^(1/4))

group_weight(g) = 4 - abs(g - 3)
rotation_weight(d) =
  abs(d - 9) + 1,   d < 18
  abs(d - 27) + 1,  d >= 18

preference_factor =
  group_weight(g)^2,     relative_goal_distance < omni_dir_goal_thre
  rotation_weight(d)^4,  otherwise

height_factor =
  max(0, 1 - slope_weight * height_penalty),
    when use_cost=true and slope_weight>0
  1,
    otherwise

traversability_factor =
  0,                                                   risk >= hard_cost
  clamp(1 - traversability_weight*(risk-soft_cost),
        0, 1),                                        risk > soft_cost
  1,                                                   otherwise

primitive_score = direction_factor
                * preference_factor
                * height_factor
                * traversability_factor
```

The production implementation uses a lookup table for the fourth root; the
`3.60` clamp above records that implementation detail. The score of a
rotation/group bin is the sum of all surviving primitive scores assigned to
that bin. The planner selects the allowed bin with the largest positive sum.

`height_penalty` is the maximum path-correspondence height between
`ground_height_thre` and `obstacle_height_thre`. It is recorded only when
`use_cost=true`. Points above `obstacle_height_thre` count as collision hits
instead. With the current defaults:

```text
use_cost = false
slope_weight = 0.0
```

the height factor is always `1`. This does **not** disable obstacle collision
checking, and it does not by itself disable the separate traversability grid.
Today, active terrain influence comes primarily from traversability hard/soft
cost, not from `slope_weight`.

The traversability factor uses the maximum raw cell risk intersecting the
candidate's padded rectangular footprint sweep. The producer does not inflate
cells by robot radius, so body clearance is applied exactly once here. The hard
comparison is inclusive (`>=`); the soft comparison
is strict (`>`). When a traversability grid is present, out-of-grid,
non-finite, or invalid cells are fail-closed and contribute hard risk. The
endpoint input gate enforces freshness, and each motion consumer independently
verifies that the current padded footprint is fully covered before accepting a
candidate, so stale silence or a first sample just inside the grid cannot look
like free space.

The grid is a single risk channel, so blocked terrain and unobserved cells both
use hard cost. Motion consumers therefore gate the incrementally swept area:
cells fully contained by the robot's initial padded footprint are not treated
as newly entered space, while cells touching or extending beyond that initial
footprint remain fail-closed. This prevents LiDAR self-occlusion under the body
from deadlocking every translation or rotation without clearing future unknown
space in the producer.

Candidate paths are swept continuously at a step no larger than 5 cm or half a
grid cell. If a sparse path segment crosses the active planning horizon, the
in-horizon prefix is clipped to the horizon circle and checked before the
remainder is discarded; an out-of-range endpoint must never hide hard terrain
inside the commanded horizon.

### 3.3 Output path is not one of the 343 primitives

The 343 primitives are voting templates. After selecting bin
`selected = rotation_index * 7 + group_id`, the planner:

1. reads the canonical `startPaths[group_id]` centerline;
2. crops it by the current planning range and relative goal distance;
3. rotates it by the selected rotation;
4. scales it by the current path scale;
5. emits that transformed canonical centerline as the local path.

Therefore, it is incorrect to describe the result as "selecting and directly
publishing one of 343 paths." The primitives determine group evidence; the
published path is a rotated/scaled canonical `startPath` for the winning group.

The C++ core emits this path in the body frame. `Executor` transforms it to the
configured endpoint publication frame.

### 3.4 Local recovery contract

Native autonomous recovery is an execution policy owned by `Executor`, not a
third Planner backend and not a separate Product mode. Every autonomy frame
first calls exactly the configured Planner. `Executor` may then enter recovery
when that planner cannot produce a safe path or odometry shows insufficient
progress for the configured blocked interval. Plain `teleop` and `teleop_avoid`
never trigger autonomous recovery; assisted teleop must stop or publish only a
verified operator-intent path. `teleop_avoid` additionally accepts an explicit,
momentary `manual_mode` sample for operator escape. It bypasses local planning
and map-based collision checks without bypassing authority, deadman/freshness,
command limits, control-loop guard, E-stop, or driver readiness.

A blocked pure-lateral CMU intent is not handed to rotation recovery. The
ordinary CMU recovery rotation changes the meaning of a body-frame lateral
request and previously caused forward/reverse and yaw-state churn. The endpoint
publishes zero with `teleop_assist_lateral_blocked`; the operator can choose a
new direction, command yaw explicitly, or use the momentary manual escape.

Recovery is candidate-based:

- Translation candidates search an 8-neighbor body-frame lattice. They may
  include side steps and guarded reverse motion, but every segment must pass
  the same full-footprint obstacle and traversability checks before it can be
  tracked.
- Candidate validation samples the padded robot footprint, not only the path
  centerline. Obstacle overlap, missing traversability coverage, out-of-grid
  cells, non-finite cells, and cells at or above hard cost all reject the
  candidate.
- `Executor` tracks a verified recovery translation with a dedicated slow
  follower state. Recovery progress is measured from odometry pose/yaw change,
  not from command publication or elapsed time alone.
- Recovery actions have one explicit Product-compiled order. The default is
  `translate,rotate`, preserving the quadruped's useful side-step/reverse
  capability; deployments may select `rotate,translate` without changing the
  planner implementation.
- Rotation is not a fixed-angle fallback. It evaluates both signs from
  `min_rotation_rad` through `max_rotation_rad`, validates the complete padded
  footprint sweep, estimates forward corridor reach after each candidate, and
  selects the best corridor with goal alignment and smaller-angle tie breaks.
- Failed translation or rotation directions are not retried blindly in the same
  recovery episode. A failed action advances to the next configured action. A
  completed action also advances the round-robin cursor before observation and
  replanning, so an ineffective success cannot restart the first action or reset
  the attempt budget. Recovery stops after the configured total action-attempt
  limit and reports exhaustion instead of degrading into an unchecked backup.
- A completed action does not clear Mapd-owned live maps. `Executor` publishes
  zero and waits for both cloud and traversability generations/stamps to advance,
  then returns to the ordinary planner. This is LingTu's bounded
  `act -> observe -> replan` sequence, not a second behavior-tree runtime.

While rotation is active, navd status exposes the signed selected angle as
`recovery_rotation_target_rad` together with action, attempt, candidate count,
progress, and reason. Field evidence can therefore distinguish corridor-aware
selection from a fixed-angle fallback.

Recovery output remains below the command boundary. A verified recovery path
or direct rotation is still bounded by endpoint motion limits and
control-authority arbitration, which retain the highest authority over
`rt/nav/cmd_vel`.

## 4. PathFollower control contract

The production C++ follower has one public entry point,
`Follower::follow(FollowerInput)`. `FollowerInput` contains either a geometric
path or an exact B-spline; the registered internal algorithm then runs the
lookahead tracker used by CMU/recovery or the B-spline tracker used by SCAN.
Planner and Follower remain separate modules in both cases.

It is Pure-Pursuit-like only in the broad sense that it chooses a point ahead
on the path. It is not the classic Pure Pursuit controller: it does not compute
curvature as `2*sin(alpha)/lookahead` and then derive yaw rate from
`linear_speed * curvature`.

### 4.1 Lookahead and target point

The adaptive lookahead is:

```text
lookahead = clamp(
  base_look_ahead + look_ahead_ratio * abs(state.vehicle_speed),
  min_look_ahead,
  max_look_ahead)
```

`state.vehicle_speed` is the follower's persistent, acceleration-limited
**commanded scalar speed**. It is not measured odometry speed. Starting near
the previous path index, the tracker selects the first path point at least one
lookahead distance from the current vehicle-relative position, or the final
path point if none is farther away.

For the `nav` Product defaults:

```text
base_look_ahead = 0.35 m
look_ahead_ratio = 0.5 s
min_look_ahead = 0.2 m
max_look_ahead = 2.0 m
max_speed = 0.5 m/s
```

the reachable lookahead interval is `0.35-0.60 m`, not the full configured
`0.2-2.0 m` clamp interval.

### 4.2 Velocity and yaw control

The selected point defines a path direction and heading error. The controller
then applies:

```text
wz = clamp(-yaw_gain * heading_error,
           -max_yaw_rate,
           +max_yaw_rate)

vx =  cos(heading_error) * commanded_scalar_speed
vy = -sin(heading_error) * commanded_scalar_speed
```

For the pure-lateral Thunder exception, path-yaw alignment and two-way switching
are disabled. Translation still follows the accepted CMU lookahead vector, but
`wz` is a bounded closed-loop correction to the body yaw captured when the
lateral intent began. It reuses the configured goal-yaw gain and is capped by
`goal_yaw_max_rate` (`0.6 rad/s` by default); setting path `wz=0` alone is not a
yaw hold because the locomotion policy can drift during sustained lateral
motion.

The default moving and near-stop yaw gains are both `7.5`; the native
`maxYawRateRadS` default is `0.8 rad/s`. Public rotation values are always
`rad`, `rad/s`, or `rad/s^2`; the follower no longer stores a degree-per-second
limit. Heading alignment uses a stateful Schmitt trigger: translation freezes
above `headingAlignEnterRad` (default `pi/4`) and resumes only below
`headingAlignExitRad` (default `0.35`). The band prevents rotate/translate
chatter near one threshold. The same state applies to geometric paths and SCAN
timed trajectories.

In the geometric branch, `max_accel` limits only the signed scalar
`state.vehicle_speed` by `max_accel * dt`. It is not a component-wise `vx/vy`
acceleration limit, does not limit yaw acceleration, and is not a jerk limiter.
The shared final command smoother owns optional per-axis acceleration limits;
motion safety still bypasses smoothing to publish an immediate zero.

The follower's `stop_dis_thre` is `0.2 m` by default. It is not the same
quantity as either the local planner's `near_field_stop_dis=0.5 m` or the
native mission's `goal_reached_m=0.35 m`.

### 4.3 Timed trajectory tracking

The SCAN branch evaluates the planned sample at the current execution time and
combines that sample's feed-forward body velocity with body-relative position
feedback. A separate `trajectory_look_ahead_s` sample defines desired heading
from `p(t + forward) - p(t)`; current planned yaw is the zero-motion fallback and
current yaw rate remains feed-forward. When heading error exceeds the configured
threshold, translation and trajectory time freeze while yaw alignment continues.
Measured body twist seeds the first acceleration-limited command. The follower
consumes trajectory samples only; obstacle search and spline generation stay
inside Local Planner.

## 5. Native runtime configuration

Endpoint CLI/environment values are the runtime parameter authority. The
selected local backend and its parameters are resolved before motion starts;
the Host does not apply a second set of planner or follower defaults.

The Product compiler emits heading and recovery values into the immutable
RunPlan (`LINGTU_NAV_PATH_FOLLOWER_*` and `LINGTU_NAV_RECOVERY_*`). Navd parses
and validates the same values before constructing `Executor`; changing the
action order or rotation range therefore changes the resolved process
environment and takes effect only through the normal Product lifecycle.

Because `path_range_by_speed` and `path_scale_by_speed` are enabled by default,
the endpoint's normalized planning speed also affects its initial range and
scale. CMU normalizes `autonomySpeed` against the same configured follower
maximum and receives the Product's `3.0 m` local corridor range. A `0.5 m/s`
command therefore starts from the full `3.0 m` range instead of accidentally
being treated as a fraction of an unrelated `1.0 m/s` limit.

The endpoint obstacle merge budgets registered cloud, `terrain_map`, and
`terrain_map_ext` separately. `terrain_map_ext` participates in active-share
budgeting, voxel deduplication, and planner fusion only when its configured
share is greater than zero and the message is fresh and non-empty. The default
share of zero is an intentional no-effect setting.

Sensor-origin offsets are part of evidence geometry, not score tuning. The
same body-frame LiDAR offset must be supplied to navigation ray clearing and
traversability. MuJoCo acceptance resolves the compiled `lidar_site` pose
relative to `base_link` and passes that resolved XYZ through the endpoint
`--sensor-offset-{x,y,z}-m` arguments; a site-local XML coordinate must not be
assumed to already be in the body frame.

That LiDAR extrinsic must **not** also shift Local Planner state when
`Executor` already supplies a body pose. `LocalPlanRequest.robot.pose` is the
body origin and Local Planner intentionally has no sensor-offset parameter; the
LiDAR offset is used only to compute the ray-clearing sensor origin. Applying it
twice shifts the local target bearing. In the 60 m MuJoCo case it
changed the initial bearing from `85.60 deg` to `99.02 deg`, crossed the
forward/reverse direction gate, and made the planner select the reverse path
family.

### 5.1 Odometry motion sanity

The native endpoint treats reported DDS twist and pose-derived consistency as
different evidence:

- non-finite values, duplicate/reversed timestamps, and reported 3-D twist
  above the configured limit fail immediately;
- pose-derived fallback speed is planar XY because quadruped body heave is not
  navigation speed;
- the fallback uses a five-sample component median and a minimum 50 ms
  filtered-pose chord, rather than a 5-10 ms adjacent-pose derivative;
- a persistent quiet-twist XY teleport or sustained planar overspeed still
  closes the gate on the bounded window.

This distinction is relevant in both domains. MuJoCo contact solves amplify
short body-pose oscillations; real SLAM can emit isolated pose outliers. The
filter removes neither a reported velocity fault nor persistent localization
divergence.

## 6. Verification and observability contract

A local-planning acceptance artifact must distinguish these layers:

- raw/registered LiDAR points;
- the obstacle points actually admitted to the local planner;
- feasible and rejected rotation/group candidates;
- the selected canonical output path;
- traversability/occupancy cost cells;
- the PathFollower command before the command boundary;
- the final command after command-boundary and control-authority arbitration.

Candidate visualization must not imply that the 36 displayed representative
paths are all 343 primitives. One representative canonical group path per
rotation is a debug summary; the underlying score is still accumulated from
up to 343 primitives per rotation.

The native endpoint status snapshot can expose these debug-only layers through
`local_candidates` and `local_map`. They are disabled by default to keep the
control endpoint lean. Field visualization may opt in with:

```text
LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES=18   # maximum 36
LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS=640          # maximum 4096
```

When truncation is required, the status writer retains obstacles and
traversability-risk cells nearest the latest sensor origin instead of taking a
uniform sample across the whole map. The snapshot records its sampling policy,
origin, total count, returned count, freshness, and completeness. A truncated
debug map is never evidence that unreported cells are free.

The Web Scene View has an optional `local planner` layer. In `debug_nav` mode it
polls `/api/v1/navigation/dds_snapshot` at 2.5 Hz and displays planner
obstacles, traversability risk, representative candidates, and the selected
path. It hides snapshots older than 2.5 seconds, non-`map` frames, and
`fresh=false`/`valid=false` data. This bridge is best-effort observability; it
does not publish commands, enter the endpoint tick, or alter the mutually
exclusive `autonomy`, `teleop`, and `teleop_avoid` ownership contract.

At minimum, record the following with each run:

```text
control mode
path-library hash or directory
valid rotation count
selected rotation/group
collision-free primitive count
aggregate group score
terrain risk and hard/soft thresholds
effective local-planner parameters
effective PathFollower parameters
input freshness/gate reason
pre-safety and post-safety cmd_vel
```
