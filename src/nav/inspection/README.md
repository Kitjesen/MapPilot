# Inspection

`src/nav/inspection` is the ROS-free C++ owner of inspection routes and route
execution state. It does not publish velocity commands and it does not contain a
second planner. The native navigation endpoint feeds each pending route point to
the existing global planner, local planner, path follower, and safety chain.

Routes are pinned to `map_id + content_epoch`, persisted below
`<inspection-dir>/routes/<map-id>`, and published with an atomic rename. Runtime
status and the restart checkpoint live directly below `<inspection-dir>`; they
are not map artifacts. Starting a route against a different active map content
epoch is rejected.

The runtime contract is:

```text
Gateway -> typed inspection command -> native navigation endpoint
                                         -> Inspection Executor
                                         -> existing single-goal navigation loop
native navigation endpoint -> inspection status -> Gateway/Web
```

Python may wrap the C ABI for HTTP integration. Route validation, persistence,
state transitions, retries, looping, and point advancement remain in C++.

## Ownership

The native executor is the authority for facts that affect motion. It owns
route validation, the task state machine, retries, point advancement, map
pinning, and the hand-off to the existing global/local navigation loop. A
status event from this executor is the only source Gateway may use to describe
execution progress.

Inspection uses the same navigation chain as the ordinary `nav` Product:
OctoPlanner3D supplies the global route, and LocalPlanner uses its SCAN backend
to supply the heading-aware local trajectory and live 3D collision check.
Inspection does not start CMU or the standalone traversability process.

The surrounding layers have deliberately smaller roles:

- **DDS/C ABI adapters** translate fields and transport commands. They do not
  infer state, retry commands, or make a route executable.
- **Gateway/Host** owns the HTTP contract, command idempotency reservation,
  event continuity journal, evidence-record binding, and the public projection
  of native status. These records explain delivery and recovery; they cannot
  advance a point, approve motion, or replace a native state transition.
- **Web** renders the Gateway projection and sends operator intent. It does not
  maintain a second task state machine.

Gateway does not require a fresh display MapScene, a traversability layer or a
separate dynamic-residual check record to submit a route. Those are not the
inspection Product's motion authority. The route/map identity, task journal and
capture/recording contracts remain at their existing boundaries; native navd
decides execution and retains its live 3-D collision and final-command checks.

## Transport patterns

The field boundary has one transport: typed CycloneDDS. The inspection topics
use three simple message patterns rather than a second ROS-style runtime:

- `task/request` and `task/ack` submit a command and acknowledge admission;
- `task/event` is the ordered execution-fact stream, while `status` is a live
  diagnostic snapshot for observers;
- `evidence/request` and `evidence/result` pair one point action with its
  persisted result.

HTTP remains the operator-facing Gateway boundary. There is no separate ROS
topic, service, or action implementation hidden behind inspection.

If a new field is needed only to explain native execution, add it to the native
status/event contract first. Do not recreate the decision in Python or the
web page. If a field is only about HTTP delivery or persistence, keep it in
Gateway and do not make it part of the motion authority.

## Three-part mental model

There are only three things to keep in mind:

1. **Task** — `task_id` names one operator-visible inspection task. A
   `request_id` is only the retryable command that changes that task (start,
   pause, resume, or cancel); it never becomes a second task identity.
2. **Route progress** — `route_id + route_revision` pin one immutable route,
   and `point_index` is the executor's cursor in that route. `point_id` is
   merely the label stored with a route point; reports derive it from the
   pinned route and evidence no longer copies it into every request.
3. **Point operation** — when a point has an optional capture/action, the
   temporary `action_request_id` matches that asynchronous operation. A
   successful operation reuses that same value as `evidence_id`, the persisted
   artifact handle. No second ID is generated or passed by the worker; an
   unsuccessful operation has no artifact handle.

No other identity is needed to run the route. The last group is absent for
ordinary points, so most tasks only use the first two groups.

Each waypoint and retry reuses the parent `task_id` and latest accepted command
`request_id`. The planner's existing `goal_epoch` distinguishes asynchronous
planning attempts inside navd; it is not another operator task. Leg completion
stays in process and advances the Inspection Executor. It does not publish a
single-goal success event for the whole route. Only native inspection events
project the route lifecycle into `NavigationState`; Gateway displays that state.
An input or motion hold does not imply that the task has committed a pause.

Inspection waits for the planner and navigation executor to report an outcome;
it does not impose a second planning deadline or measure progress by distance
to the final waypoint. Search budgets and cancellation belong to the global
planner. Motion progress, obstruction, and recovery belong to the navigation
executor. Once the native terminal stop is confirmed, its committed result
advances the route exactly once. A cancelled leg does not count as arrival or
as a retryable navigation failure. Inspection retains the post-arrival settling
and evidence-action deadlines, and applies the route's retry/skip/stop policy
to reported failures.

The native checkpoint retains the route revision, loop, and unfinished point
across an endpoint restart. An active task returns as `paused` with
`restart_resume_required`; it sends neither a navigation goal nor an evidence
request until the operator resumes it. Resume uses the existing navigation
chain to plan the current unfinished point from the newly observed pose. A
pending cancellation remains cancelled, and already-terminal tasks are replayed
unchanged. The saved route and map content must still match the checkpoint.

Point outcomes are explicit native milestones: `point_completed` records the
current loop and point before advancing; `point_skipped:<reason>` records a skip
without claiming completion. If a restart occurs after an evidence result or
point-outcome milestone was saved but before the next-point event was saved,
recovery advances that point once. Unfinished evidence is retried only after
resume with a new action request; delayed pre-restart results cannot complete it.

`loop_count` remains a finite number of rounds. A route may set
`loop_interval_s` (default `0`) to wait between completed rounds. During
`waiting_next_loop` the task is still running, but no goal or evidence request
is emitted. The next round begins when the interval ends; the final round ends
the task without an extra wait. Pausing freezes the remaining interval in the
same process and resume continues that remainder. After an endpoint restart,
the task first waits for explicit resume and then waits one complete configured
interval; old process-clock deadlines are not reused.

`run_id` is not an inspection identity. New checkpoints and DDS messages do
not contain it; the store only reads the old field when recovering a legacy
checkpoint. `exploration_run_id` belongs to the separate exploration feature
and must not be reused for inspection tasks.

## Three-point capture regression

`tests/nav/cpp/inspection/test_three_point_capture.cpp` exercises this real C++
Executor over three observation points and three clean restarts. It checks that
arrival alone cannot capture, operation admission cannot advance a point, delayed
results cannot advance another operation, and a middle-point capture failure
leaves the third point unvisited. Navigation arrivals, stable samples and evidence
results are controlled test inputs, not a physics trajectory or persisted images.

The target is `lingtu_inspection_three_point_capture_test`; Release tests explicitly
undefine NDEBUG so their assertions remain active. The 2026-10-21 focused native
run passed all four inspection test targets. Current Python/evidence/sensor results
and the unimplemented sensor types are recorded in the
[sensor status README](../../../sim/runtime/sensors/README.md).
