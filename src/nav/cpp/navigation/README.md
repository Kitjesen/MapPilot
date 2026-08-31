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
  EnvironmentView environment;  // obstacles, collision, traversability
  PlanIdentity identity;
  PlanClock clock;
};
```

`LocalPlan` carries one executable `FollowTarget`:

- CMU returns `PathTarget`.
- SCAN returns `SplineTarget`.

The Follower dispatches that variant internally. Executor does not branch on
CMU/SCAN, inspect backend strings, or manage SCAN task reuse and continuity.
The endpoint composition root chooses and configures the Planner before
constructing Executor.

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
