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
| global path / local route segment | atomic local-plan invocation and body-frame path/trajectory | direct C++ call to `LocalPlanner` inside `Navigator` |
| local path or timed trajectory | tracking invocation and body-frame `vx/vy/wz` | direct C++ call to `PathFollower` inside `Navigator` |
| pre-safety velocity | final command after safety and control-authority arbitration | native typed DDS `rt/nav/cmd_vel` to ThunderV4 |

`LocalPlanner` and `PathFollower` are different functions:

- `LocalPlanner` chooses a collision- and terrain-aware local path.
- `PathFollower` tracks that already-selected path and produces velocity. It
  does not search obstacles, score candidate paths, or replace the local
  planner.

The native endpoint control modes `autonomy`, `teleop`, and `teleop_avoid` are
mutually exclusive. The Host must not add a competing command writer.

The executable implementation paths are:

- route navigator: `src/nav/cpp/navigation/navigator.hpp/.cpp`, with private
  route geometry in `route.cpp` and execution state in `state.cpp`
- local planner: `src/nav/cpp/planning/local/planner.hpp/.cpp`
- CMU backend: `src/nav/cpp/planning/local/cmu/backend.hpp/.cpp`
- SCAN backend: `src/nav/cpp/planning/local/scan/`
- candidate scoring: private implementation inside `planning/local/cmu/backend.cpp`
- recovery candidate search: `src/nav/cpp/planning/local/recovery.hpp/.cpp`
- recovery lifecycle and retry ownership: `src/nav/cpp/navigation/recovery.hpp/.cpp`
- path follower: `src/nav/cpp/tracking/follower.hpp/.cpp`
- native endpoint assembly: `src/nav/cpp/endpoint/main.cpp`

The source tree follows the data transformation, not an upstream project name:
global and local path generation live under `planning/`, path tracking lives
under `tracking/`, and `Navigator` owns route/intent execution plus the
backend-neutral recovery lifecycle. CMU and SCAN are two independent adapters
behind the single Local Planner seam; neither backend calls the other.

### 1.1 Native Local Planner request

The C++ public entry is `nav_kernel::local::Planner::plan(const LocalPlanInput&)`.
One `LocalPlanInput` is one atomic planning frame:

| Field | Current meaning |
| --- | --- |
| `vehicle` | body pose in the selected planning frame |
| `route` | non-owning route segment from current body position through every global-route bend in the local horizon; includes generation and terminal flag |
| `kinematics` | planning-frame linear velocity/acceleration and yaw rate used to initialize a continuous trajectory |
| `identity` | frame, obstacle, and traversability generations for retained-state invalidation |
| `obstacles` | non-owning `N x 4` view `[x, y, z, height]` in that same frame |
| `collision` | optional complete local collision snapshot: occupied centers, resolution, AABB, map identity/source stamp, receiver freshness stamp, `complete`, and `live` |
| `traversability` | optional non-owning 0-100 risk-grid view plus rows, columns, resolution, and origin |
| `timestampS` | monotonic planning-cycle time; compared only with receiver-clock freshness, never directly with a DDS source/wall stamp |

The constructor supplies robot/path-library parameters. Assisted teleoperation
adds a `LocalMotionIntent`. Recovery is not a Planner entry point: Navigator
always calls the selected backend's `plan()` first and passes the same complete
frame to its own recovery controller only after no-path or execution-stall
policy triggers. The planner exposes no pose, goal, or grid setters, and an
omitted grid in one request cannot reuse a previous request's grid.

`planIntent()` is implemented by both backends and is never a backend fallback.
CMU returns a geometric path. SCAN plans from the same straight intent route,
applies the requested speed and terminal direction limit, and returns a timed
trajectory. The endpoint supplies the same collision snapshot, identity,
measured body motion, and traversability fields used by autonomous planning.
Neither backend loads or calls the other.

`Navigator` still owns route progress. It projects the body onto the nearest
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

`LocalPlanResult` reports backend, status, reason, geometric path, executable
trajectory, and bounded diagnostics. Recovery state is reported by
`NavigatorOutput`, not selected by a Planner backend. `PathFollower` tracks the
timed trajectory for SCAN and does not run another search. CMU continues through
its existing geometric lookahead branch, so backend selection does not silently
change CMU control behavior.

## 2. CMU path-library model

The planner loads three offline-generated assets from
`src/nav/cpp/planning/local/cmu/paths/`:

| Asset | Runtime meaning |
| --- | --- |
| `pathList.ply` | 343 primitive trajectories and their group IDs |
| `startPaths.ply` | 7 canonical output path-group centerlines |
| `correspondences.txt` | precomputed 2 cm XY voxel-to-primitive correspondence table |

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
3. **Assisted-teleop direction gate.** `planIntent()` additionally requires
   both the primitive end direction and the canonical output-path end
   direction to remain within the requested maximum deviation. Autonomous
   `plan()` uses the rotation prefilter but does not enable this extra hard
   primitive-direction gate.
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

The C++ core emits this path in the body frame. `Navigator` transforms it to the
configured endpoint publication frame.

### 3.4 Local recovery contract

Native autonomous recovery is an execution policy owned by `Navigator`, not a
third Planner backend and not a separate Product mode. Every autonomy frame
first calls exactly the configured Planner. `Navigator` may then enter recovery
when that planner cannot produce a safe path or odometry shows insufficient
progress for the configured blocked interval. Plain `teleop` and `teleop_avoid`
never trigger autonomous recovery; assisted teleop must stop or publish only a
verified operator-intent path.

Recovery is candidate-based:

- Translation candidates search an 8-neighbor body-frame lattice. They may
  include side steps and guarded reverse motion, but every segment must pass
  the same full-footprint obstacle and traversability checks before it can be
  tracked.
- Candidate validation samples the padded robot footprint, not only the path
  centerline. Obstacle overlap, missing traversability coverage, out-of-grid
  cells, non-finite cells, and cells at or above hard cost all reject the
  candidate.
- `Navigator` tracks a verified recovery translation with a dedicated slow
  follower state. Recovery progress is measured from odometry pose/yaw change,
  not from command publication or elapsed time alone.
- If no safe translation exists, recovery search may produce a verified direct
  rotation command. Rotation must sample the swept padded footprint in both
  candidate directions before selecting a direction.
- Failed translation or rotation directions are not retried blindly in the same
  recovery episode. Recovery stops after the configured attempt limit and
  reports exhaustion instead of degrading into an unchecked backup.

Recovery output remains below the final safety gate. A verified recovery path
or direct rotation is only pre-safety intent; the endpoint safety evaluator and
control-authority arbitration still have the highest authority over
`rt/nav/cmd_vel`.

## 4. PathFollower control contract

The production C++ follower has one public entry point,
`Follower::follow(FollowerInput)`. `FollowerInput` contains either a geometric
path or a timed trajectory; the registered internal algorithm then runs the
lookahead tracker used by CMU/recovery or the timed tracker used by SCAN.
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

For the native endpoint defaults:

```text
base_look_ahead = 0.3 m
look_ahead_ratio = 0.5 s
min_look_ahead = 0.2 m
max_look_ahead = 2.0 m
max_speed = 0.4 m/s
```

the reachable lookahead interval is `0.30-0.50 m`, not the full configured
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

The default moving and near-stop yaw gains are both `7.5`; the native yaw-rate
limit is `45 deg/s` (`0.785 rad/s`). The tracker supports holonomic `vx/vy`,
forward/reverse switching, hysteresis, end-of-path slowing, planner slow
factors, and safety-stop levels.

In the geometric branch, `max_accel` limits only the signed scalar `state.vehicle_speed` by
`max_accel * dt`. It is not a component-wise `vx/vy` acceleration limit, does
not limit yaw acceleration, and is not a jerk limiter. A downstream driver or
safety layer must enforce stricter chassis dynamics when required.

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

Because `path_range_by_speed` and `path_scale_by_speed` are enabled by default,
the endpoint's normalized planning speed also affects its initial range and
scale.

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
`Navigator` already supplies a body pose. `LocalPlanInput.vehicle` is the
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
- the PathFollower command before final safety;
- the final command after safety and control-authority arbitration.

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
