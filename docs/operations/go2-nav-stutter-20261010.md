# Go2 navigation stutter and stop — 2026-10-10

## Field evidence

Read-only evidence was collected from NX `192.168.66.65` running
`v2.3.0-go2.20261010.93` (source `05f11ee735b0f529f9ba47ab6d1addd70f6fdf97`),
Product session `product-8cd5f86ca91144c4ac95977fe39d7fad`, map
`903room_v5_raised`, content epoch `1790681532521`. The operator confirmed
obstacles were nearby. No goal, teleoperation, stop, restart or switch was sent
during this investigation.

- Actual planner/follower session speed limit: **0.2 m/s**; acceleration limit:
  **1.0 m/s²**. A target requesting 0.35 m/s does not override the session limit.
- The task ended with `replan_budget_consumed`. This was the installed allowance
  of one automatic global replan per external goal, not a CPU-time budget.
- The retained SCAN failure is `collision_no_free_exit`, failure sequence 15,
  attempt 414. The candidate's first six control points are free and its last
  four occupied. The actual robot footprint and the requested spline start are
  free. Dynamic predictions are absent; the collision snapshot is live and
  approximately 123 ms old.
- The target is free at the actual robot-to-target heading (−19.517°) but
  occupied at the candidate's terminal control-point heading (139.803°).
  Nearby free positions exist. This establishes an unsuccessful local
  candidate, not impossibility of every route or a false LiDAR obstacle.
- The spline request starts **0.744 m ahead** of measured position. Its nominal
  initial planar speed is approximately **0.260 m/s**, versus measured
  **0.0305 m/s**. The intervening segment is free in the captured bitmap.
- The last contiguous nonzero driver run lasted 12.981 s. Command-speed
  integration is 0.792 m, chassis feedback-speed integration 0.205 m, and
  chassis position displacement 0.117 m. These are different measurements of
  one run, not the entire navigation task. No driver rejection, watchdog stop
  or backend error is recorded.
- Initial nav control-loop p95 work is 1.97 ms inside a 10 ms period; SLAM is
  TRACKING. Those samples do not support attributing this stop to general CPU
  starvation or localization loss.

The first recovery trigger cannot be reconstructed from the retained journal:
terminal status cleared recovery fields and there are no detailed recovery
transition events. The terminal budget reason must not replace the separate
recorded local collision failure when explaining the task.

## Local code correction

`PlannerFsm::planFromCurrentTraj()` now rebuilds autonomous reference-route
trajectories from measured odometry position and velocity, with zero initial
acceleration, through the existing polynomial initializer. It does not warm
start from the old timed spline, because that manager branch resamples old
positions and derivatives even if the caller supplies a different start.

The original nominal-start behavior also exists in
[the pinned SCAN upstream FSM](https://github.com/wuyi2121/SCAN-Planner/blob/348e8a590a50a5a6bbab8d8c6dcfd171f009be26/src/planner/plan_manage/src/scan_replan_fsm.cpp).
This correction addresses ground-robot tracking lag; it is not evidence that
this branch was a LingTu-only addition. On 2026-10-22 the same correction was
extended to continuous operator motion-intent replanning: both reference-path
branches now use measured position and velocity, with the existing polynomial
initializer. A held-input reproduction exposed a new spline beginning about
0.303 m ahead of the robot with a 0.147 m/s initial velocity mismatch. This
trajectory-boundary correction leaves global routes, collision geometry and
safety checks unchanged. The October 22 change has not been deployed to the
offline NX and does not establish physical turning or stopping performance.

The regression checks the resulting spline's actual position and velocity at
`t=0`, not only debug request fields. The old code generated a boundary about
0.361 m ahead of a stationary robot in the reproduction. Reinitialization
brings position residual below 0.5 mm and velocity residual below 0.00011 m/s
in the two tested lagging states. A 1 mm / 0.001 m/s assertion accommodates the
existing least-squares fit; no production tolerance was changed.

## Goal retention correction

The subsequent operator request changes ordinary obstruction policy. The goal
retry controller no longer consumes a lifetime allowance after one global
replacement route. Failed replacement searches retain the active target and
task identity, hold zero output, and retry after the existing 0.5 s backoff.
Successful replacement resets retry timing so a later obstruction can recover
again. Stop-confirmation timeout waits for confirmation rather than terminating
the task. One search at a time remains the execution rule.

Goal-plan completion publishes `PathActive / replan_waiting` for an ordinary
failed replacement search, without creating a terminal intent. Exact search
failure remains in planner diagnostics. The runtime retains the recovery
trigger until success or interruption. Waiting ticks skip normal autonomous
execution so an old path cannot overwrite their zero command. The webpage
labels the retained task “等待通路，自动继续” and keeps pause/cancel available.
Historical budget failures remain failures; they are not displayed as tasks
that can automatically resume.

Pause/resume also covers waiting without an installed old path, and cancelling
an in-flight search while paused. The runtime drains the cancelled result without
activating it, and resuming the same target allows another search. The resume
context carries a `replan_pending` fact derived from the existing runtime trigger;
it is not a user setting or another stored retry state. A transient driver
heartbeat gap preserves an already-active goal's retry. The existing transaction
for a new goal waiting to replace the old goal retains its own interruption rules.

Explicit cancel, stop, takeover, map/session identity changes, planner exceptions,
and actual zero-command delivery failures retain their existing handling. This
policy does not relax collision, ground support, unknown-space, driver or
authority checks, and it does not turn an invalid initial goal into movement.

## Delivery status and remaining investigation

Local focused test results are in `build/go2-stutter-20261010/`:
`test-red.log`, `test-route-fix.log`, `test-follower.log`, and build logs.
The final route/planner selection passes all 42 tests; all 11 spline-follower
tests pass. These 53 focused checks include measured-state replanning,
reference replacement, motion-intent continuation, stop/freeze, and tracking.
The native evidence and exact grid-query script are in `native/README.md`.
Existing MSVC warnings from `plan_container.hpp` and `lbfgs.hpp` remain; this
investigation did not alter those files.

The goal-retention correction passes four focused C++ suites:
`test_bounded_goal_replan_controller`, `test_goal_plan_completion`,
`test_goal_replan_runtime_coordinator`, and `test_active_path_replan_cycle`.
The consolidated run is `retry-tests-all.log` (4/4 passed). Coverage includes
ten repeated successful replans of one goal, no-path followed by success,
stop-confirmation delay, pause/resume with no retained path, cancelled-worker
drain, driver heartbeat recovery, goal replacement, and cancellation. The local
Windows `navd` target including production `loop.cpp` builds successfully;
see `retry-endpoint-build.log`. Web task-status/lifecycle/card checks pass 25/25;
web type/build and touched-file lint also pass (`retry-web-build.log` and
`retry-web-lint.log`). These are local checks, not a MuJoCo or field acceptance.

This validation uses the current uncommitted working tree based on
`0abfef052a7ff882963813602d12a6acc2a57b20`, not the installed `.93` source
snapshot. Unrelated worktree changes are retained.

This patch is local and has not been compiled for ARM or installed. NX HTTP
and SSH later became unreachable; the earlier task failure was observed while
both were reachable. A subsequent disconnection must not be presented as the
cause of that earlier local-planner failure.
The operator subsequently confirmed the robot was powered off or disconnected;
ARM build, installation and supervised field retry validation remain pending.

The candidate terminal-heading collision and weak chassis response still
require controlled replay/field validation. Local unit tests do not establish
successful obstacle avoidance on Go2. Increasing speed or retry allowance
alone does not repair an incorrect trajectory boundary.
