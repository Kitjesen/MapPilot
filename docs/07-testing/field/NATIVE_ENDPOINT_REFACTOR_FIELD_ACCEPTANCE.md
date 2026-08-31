# Native Endpoint Refactor Field Acceptance

Status: acceptance checklist, not field evidence.

This checklist closes the gap between local endpoint-refactor verification and
product claims on the S100P. A run counts only when it uses the official profile
entry and records dated evidence under `docs/07-testing/field-runs/`.

## Local release gate

- Build the Linux endpoint with `LINGTU_NAV_ENDPOINT_RUN_TESTS=1`.
- Confirm the required CTest catalog contains the input projector, rolling
  effect coordinator, inspection command coordinator, and control-loop health
  tests.
- Run the native Python contracts and preserve the result in the release log.
- Confirm `main.cpp` owns process composition and lifecycle, while
  `runtime/loop.cpp` owns ordered DDS drains, the 20 Hz motion/safety schedule,
  effect adapters, and shutdown-zero confirmation.
- Do not raise the 20 Hz loop rate without ready field measurements showing
  both a need and sufficient utilization/overrun headroom.

## Physical safety preflight

- Use a clear, bounded test area and the normal field deployment.
- Verify the physical E-stop and a second operator before enabling motion.
- Apply the approved low-speed field limits; do not bypass `navd` final safety,
  traversability, near-field stop, control authority, or driver ownership.
- Confirm localization, active saved-map identity, LiDAR, odometry, TF,
  traversability, and driver-control readiness before sending a command.
- Start every case stationary and confirm a fresh accepted zero command before
  moving to the next case.

## Control-loop evidence gate

- Keep the default 20 Hz (50 ms period) loop unless a separately reviewed field
  dataset justifies a change; the 10 Hz LiDAR cadence is not such evidence.
- Wait for `control_loop_health.ready=true` and
  `control_loop_health.healthy=true` before accepting any motion case. The
  default rolling window is 600 samples with a 100-valid-sample warm-up; a
  `warming_up` snapshot is startup telemetry, not acceptance evidence.
- Treat a missing, empty, or malformed `control_loop_health` object as a failed
  admission gate; only explicit `warming_up` remains non-blocking at startup.
- Preserve the health reason, period, window/total sample counts, deadline miss
  and streak counters, and the configured threshold context. Defaults are a 5%
  deadline-miss-ratio limit, 90% p95-utilization limit, and three consecutive
  deadline misses.
- Record `loop_ms` mean/p50/p95/p99/max to verify actual 20 Hz cadence and
  jitter. Treat it as diagnostic evidence; health blocking remains based on
  work utilization and overruns, not ordinary scheduling jitter.
- Also record at minimum p95 utilization, deadline miss ratio, work-time
  p95/p99, and overrun p95/p99 before and after every physical motion case. A
  run that becomes `healthy=false` does not pass even if motion completes.
- Deliberately induce bounded CPU load at low speed and verify the runtime guard
  path: two consecutive mature unhealthy samples latch the guard
  (`consecutive_deadline_misses` may latch immediately), the first latch clears
  motion once, later ticks hold fresh zero motion, and all command admissions are
  blocked behind the resume requirement.
- After the induced load is removed, wait for the stable healthy confirmation
  window derived from `tick_hz` (about one second at the default 20 Hz). Confirm
  the endpoint does not auto-resume, then use the official Resume flow and require
  the existing zero-confirmation barrier before autonomy is released. A failed
  resume attempt must keep the guard latched.
- Record that this runtime hold is not the persisted software E-stop: do not use
  ClearEstop as the recovery action, and do not report a runtime guard recovery
  as E-stop persistence behavior.

## Official `inspection` profile

Record request IDs, ACKs, inspection status, navigation status, `/nav/cmd_vel`,
driver accepted-output sequence, and endpoint logs for every case.

1. Start a stored route with the expected active map and route revision.
   Require one accepted ACK, one run ID, and exactly one inspection run.
2. Replay the same request ID and kind. Require the cached ACK and no second
   Executor transition or duplicate navigation goal.
3. Reuse the request ID with another kind. Require
   `duplicate_request_id_kind_mismatch` and no state change.
4. Pause while moving. Require `inspection_paused`, a zero command, driver
   acceptance of that zero, and no later non-zero output while paused.
5. Attempt Resume while operator takeover is latched. Require
   `inspection_resume_requires_autonomy` and no resumed path.
6. Release takeover through the official ownership flow and Resume. Require an
   accepted ACK and normal status progression.
7. Cancel while moving. Require `inspection_cancelled`, a confirmed zero,
   released path authority, and no later non-zero output for that run.
8. Switch or invalidate the active map during a run. Require stop-authority
   before clear-motion, a terminal or paused inspection status, and zero hold.
9. Interrupt the evidence worker. Require
   `evidence_request_publish_failed`, failure-policy handling, and zero motion
   when the resulting state requests it.

## Saved-map `explore` route

1. Verify a frontier inside the saved-map artifact follows normal global
   planning and local safety.
2. Verify a target outside the saved map is not accepted as a raw click goal.
3. For a correlated TARE goal, require the exact
   `goal_outside_static_map` handoff before a rolling segment is considered.
4. Require matching session, reset epoch, minimum generation, live grid, and
   observed-free plus terrain-safe prefix before authority activation.
5. During rolling execution, inject or observe stale grid, unsafe cell, epoch
   change, generic navigation takeover, and cancellation separately. Each case
   must stop motion and publish the expected terminal segment status.
6. Confirm the default 20 Hz safety loop remains independent from the 10 Hz
   LiDAR update rate and preserve the control-loop evidence required above.

## Pass rule

Local tests close the software refactor only. Product completion requires dated
field evidence for both `inspection` and `explore start --map MAP`. Direct
DDS injection, commissioning smoke, and code inspection are diagnostic evidence
but do not complete a profile. Passing local ControlLoopHealth tests or observing
`healthy=true` on a development host is not S100P field evidence and must not be
used to justify a blind frequency increase.
