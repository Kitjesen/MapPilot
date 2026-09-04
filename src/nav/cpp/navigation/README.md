# Navigation execution

This directory owns execution of an already admitted route or assisted-motion
intent. It does not own global search, DDS, backend selection, or final robot
velocity.

## Data flow

```text
GlobalPlanner -> Route -> Executor -> LocalPlanner -> LocalPlan -> Follower
                              |                         |
                              +------ Recovery ---------+
                                                        |
                                                 MotionCandidate
                                                        |
                                           endpoint FinalControl
```

- `route.*` defines the activated global `Route` and computes progress plus
  the bounded local route segment.
- `executor.*` calls one already configured `local::Planner`, then one
  `Follower`, and returns execution state plus a pre-safety command.
- `recovery.*` is invoked only when the typed planning state or measured
  progress requires recovery.
- `state.cpp` owns odometry-derived kinematics, progress, and observation
  barriers.

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
calibration. Route progress updates only the local segment. Suspension or a
frame-epoch reset discards the reference and pending local work; the next
valid input rebuilds it. A new goal and cancellation use `setRoute()` and
`clear()` respectively, which reset the planner and follower.

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

## MuJoCo single-floor goal check

Run from the repository root with its Python development environment and freshly
built native binaries. Prepare a saved `map.pcd` / `octomap.ot` bundle from the
same industrial-park geometry as the MuJoCo scene, then run:

```sh
python sim/scripts/mujoco/native_navigation_acceptance.py \
  --manifest config/runtime_graph/acceptance/mujoco_scan_goal.json \
  --product-map <same-source-map-directory> \
  --out-dir artifacts/scan-goal
```

This entry starts and stops the formal `nav + scan` Product through
ProductControl. Robot, policy, spawn and native parameters come from RunPlan.
MuJoCo navigation uses truth localization by default; explicit
`env_config={"backend": "mujoco", "localization": "fastlio2"}` selects the
separate SLAM path. Mapping Products retain their estimator-backed defaults.
Truth replaces localization only: registered clouds contain actual simulated
LiDAR returns, without synthetic ground patches.
Truth poses are published independently of raycasting; delayed scans retain
their capture pose without rewinding live odometry. Viewer refreshes use owned
snapshots off the physics loop. Endpoint input holds freeze both SCAN planning
time and Follower execution time without replacing the active reference.

The goal is `(56, 32, 0.3)`. A successful global plan or accepted command is not
arrival: the report requires physical travel, native `REACHED`, final position,
collision evidence and terminal zero acknowledgement. The Viewer shows the
global Route, local spline preview and actual traveled path. Keep failures in
the report; this entry is not a claim that long-range navigation already passes.
Use a fresh output directory per run. Contact results are finalized separately
from best-effort live display updates and must match the Product session.
