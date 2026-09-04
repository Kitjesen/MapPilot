# SCAN local planner

This directory contains LingTu's ROS-free port of
[wuyi2121/SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) pinned to
commit `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`.

The algorithm sources live under [`upstream/`](upstream/UPSTREAM.md) and keep
the official Projected DynAStar, polynomial initialization, rebound optimizer,
uniform B-spline, time reallocation, PlannerManager, six-state replanning FSM,
future-collision check, and closed-loop controller. Port changes are limited to
removing ROS process APIs, supplying values explicitly, using the LingTu
namespace, and C++17/MSVC compatibility.

LingTu-specific code is only the boundary:

- `grid.*` exposes Mapd's complete inflated bitmap through the official
  `GridMap::getInflateOccupancy()` query. It does not build, inflate, score, or
  smooth another map.
- `backend.*` converts `LocalPlanRequest` odometry and Route/MotionIntent into
  official FSM inputs, then converts the emitted B-spline message into
  `SplineTarget`.
- `planning/local/task.*` owns one serialized worker with the upstream timer
  semantics: the FSM callback runs at 100 Hz and the future-collision callback
  runs independently at 20 Hz. The last published trajectory remains active
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
offset. Product speed limits may be lower than the upstream launch defaults;
they are explicit robot/runtime tuning and do not change the SCAN equations.

`nav` and `teleop_avoid` select `scan` by default and run the SCAN FSM and
controller at 100 Hz; future-collision checks run at the upstream 20 Hz.
`cmu` remains an explicit backend option and is not deleted or used as a silent
fallback.
