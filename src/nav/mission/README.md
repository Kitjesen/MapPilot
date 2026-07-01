# nav.mission

`nav.mission` owns the navigation task lifecycle. It is not a planner backend
and it is not a service API layer.

## Boundary

```text
nav.services.goals / nav.services.patrol / gateway
  -> nav.mission.Navigation
  -> nav.services.plan.GlobalPlanner
  -> nav.local / nav.services.plan.local_planner
  -> nav.local.path_follower
  -> nav.services.safety.VelocityMux
```

Mission accepts typed goals and route commands, turns them into lifecycle
events, calls the global planner, publishes waypoints, and handles pause,
cancel, timeout, stuck, and recovery.

## Layout

| Path | Purpose |
| --- | --- |
| `navigation.py` | Module shell: ports, config, setup, lifecycle, health |
| `model/` | Mission states, events, status payloads, frame checks, and small policies |
| `runtime/` | Event application, planning, execution, recovery, patrol, localization, and control handlers |
| `tracking/` | Waypoint arrival and stuck detection |
| `api/` | MCP/AI-callable navigation helpers |

## Runtime Files

| File | Purpose |
| --- | --- |
| `runtime/control.py` | Goal, stop, cancel, teleop, costmap, and frame-jump control handlers |
| `runtime/fsm.py` | Mission state/event application and status publishing |
| `runtime/planning.py` | Global planner call, path validation, path publication |
| `runtime/execution.py` | Odometry updates, waypoint progress, completion, timeout |
| `runtime/recovery.py` | Runtime recovery motion after stuck events |
| `runtime/patrol.py` | Patrol route execution and waypoint advancement |
| `runtime/localization.py` | Localization pause/resume and motion-hold handling |

## Rules

- Services parse commands and own assets; mission owns execution state.
- Mission calls `GlobalPlanner`; services should not push waypoint execution.
- LocalPlanner and PathFollower are downstream runtime modules, not mission
  subroutines.
- New mission behavior goes into `runtime/` only when it changes task execution.
  Data definitions stay in `model/`, tracking stays in `tracking/`, and external
  callable helpers stay in `api/`.
