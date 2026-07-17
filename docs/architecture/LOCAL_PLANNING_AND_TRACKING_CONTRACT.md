# Local Planning and Tracking Contract

This document is the canonical human-readable contract for LingTu local path
planning algorithms, scoring, path tracking, and their parameter surfaces. It
complements `local_planner_io_contract.md`, which remains authoritative for
the external local-planner message I/O boundary. If the two repeat algorithm,
tracking, or parameter details, this document wins for those details.

## 1. Functional boundary

The navigation control boundary is contract-first and uses
`Port -> Wire -> Transport` terminology:

| Producer port / role | Wire / payload | Transport / consumer |
| --- | --- | --- |
| navigation goal input | global-plan request and global path | direct C++ call to the global planner inside the native endpoint |
| global path / local target | local-plan invocation and canonical local path | direct C++ call to `LocalPlanner` inside the endpoint |
| canonical local path | tracking invocation and body-frame `vx/vy/wz` | direct C++ call to `PathFollower` inside the endpoint |
| pre-safety velocity | final command after safety and control-authority arbitration | native typed DDS `rt/nav/cmd_vel` to ThunderV4 |

`LocalPlanner` and `PathFollower` are different functions:

- `LocalPlanner` chooses a collision- and terrain-aware local path.
- `PathFollower` tracks that already-selected path and produces velocity. It
  does not search obstacles, score candidate paths, or replace the local
  planner.

The native endpoint control modes `autonomy`, `teleop`, and `teleop_avoid` are
mutually exclusive. The native endpoint must not be run together with the
Python `CmdVelMux` control lane.

The executable implementation paths are:

- local planner: `src/nav/services/plan/local_planner/cpp/local_planner.hpp`
- scoring helpers: `src/nav/services/plan/local_planner/cpp/local_planner_scoring.hpp`
- path follower: `src/nav/kernel/src/path_follower_core.cpp`
- native endpoint assembly: `src/nav/services/endpoint/cpp/nav_native_endpoint.cpp`
- Python adapters: `src/nav/services/plan/local_planner/parameters.py` and
  `src/nav/local/path_follower_backend.py`

## 2. LocalPlanner path-library model

The planner loads three offline-generated assets from
`src/nav/services/plan/local_planner/paths/`:

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

## 3. LocalPlanner gates and score

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
   maximum risk sampled along the rotated/scaled canonical group path is
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

The traversability factor uses the maximum risk sampled along the candidate
group centerline. The hard comparison is inclusive (`>=`); the soft comparison
is strict (`>`). Out-of-grid or non-finite cells currently contribute zero
risk, so freshness and coverage must be enforced at the endpoint input gate.

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

The C++ core emits this path in the body frame. `NavLoop` transforms it to the
map frame for endpoint publication. The Python `LocalPlanner` Module publishes
in its configured planning frame.

## 4. PathFollower control contract

The production C++ `PathFollower` is best described as a **lookahead-point
holonomic geometric tracker**.

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

`max_accel` limits only the signed scalar `state.vehicle_speed` by
`max_accel * dt`. It is not a component-wise `vx/vy` acceleration limit, does
not limit yaw acceleration, and is not a jerk limiter. A downstream driver or
safety layer must enforce stricter chassis dynamics when required.

The follower's `stop_dis_thre` is `0.2 m` by default. It is not the same
quantity as either the local planner's `near_field_stop_dis=0.5 m` or the
native mission's `goal_reached_m=0.35 m`.

## 5. Native endpoint versus Python Module

These are alternative runtime lanes, not parameters that should be merged at
runtime.

### 5.1 Local planner

| Contract | Native endpoint default | Python `LocalPlanner` with `nanobind` |
| --- | --- | --- |
| Parameter authority | endpoint CLI/environment | `config/robot_config.yaml` plus Module kwargs |
| Core implementation | `LocalPlannerCore` | the same `LocalPlannerCore` through nanobind |
| Planner `maxSpeed` / `autonomySpeed` | `1.0 / 0.4`, so normalized planning speed is `0.4` | current robot config `0.875 / 0.875`, so normalized planning speed is `1.0` |
| Traversability enabled | `false` unless `--use-traversability-cost true` | `true` by default |
| Traversability hard / soft / weight | `80 / 40 / 0.01` | `90 / 40 / 0.01` |
| Height-derived cost | `useCost=false`, `slopeWeight=0` | current config `use_cost=false`, `slope_weight=0` |
| Maximum admitted relative obstacle height | 1.20 m | native core default 1.20 m |
| `terrain_map_ext` obstacle share | `0.0`; excluded unless configured above zero and fresh/non-empty | merged by the Module obstacle-cloud path |
| Collision threshold | 2 hits | 2 hits |

Because `path_range_by_speed` and `path_scale_by_speed` are enabled by default,
the endpoint's normalized planning speed also affects its initial range/scale.
Do not assume the endpoint and Python Module search identical horizons merely
because they use the same path-library files.

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

That LiDAR extrinsic must **not** also be copied into
`LocalPlannerCore.sensorOffsetX/Y` when `NavLoop` already receives a body pose.
The planner's vehicle pose is the body origin, so its planner offset remains
zero; the LiDAR offset is used only to compute the ray-clearing sensor origin.
Applying it twice shifts the local target bearing. In the 60 m MuJoCo case it
changed the initial bearing from `85.60 deg` to `99.02 deg`, crossed the
forward/reverse direction gate, and made the planner select the reverse path
family.

The `cmu_py` backend is a debug fallback, not the production scoring oracle. It
keeps the same 343/7/36 library shape but currently differs in details,
including a 5 m near-goal preference threshold (native: 1 m), Python grid
defaults, direction handling, recovery, and scale/range degradation. Product
acceptance must use the native C++ core.

### 5.2 Path follower

| Parameter | Native endpoint | Generic Python Module | `thunder_nav` Python product override |
| --- | ---: | ---: | ---: |
| `max_speed` | 0.4 m/s | 0.4 m/s | 0.20 m/s |
| `max_accel` | 1.0 m/s^2 | 1.0 m/s^2 | 10.0 m/s^2 |
| Module `lookahead` input | direct core defaults | 1.5 m | 0.35 m |
| resulting base / min / max lookahead | 0.30 / 0.20 / 2.00 m | 0.30 / 0.25 / 1.50 m | 0.20 / 0.20 / 0.35 m |
| `stop_dis_thre` | 0.20 m | 0.20 m | 0.05 m |
| yaw gain / max yaw rate | 7.5 / 45 deg/s | 7.5 / 45 deg/s unless overridden | inherited unless overridden |

The Python Module's `lookahead` constructor argument is an adapter-level
maximum, not a direct assignment to `baseLookAheadDis`. The adapter derives
base/min/max values and always sets `lookAheadRatio=0.5`.

### 5.3 Odometry motion sanity

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
