# MATRiX v1.1.2 runtime reverse analysis — 2026-10-13

This pass investigates the runtime mechanisms relevant to LingTu's failed
interactive camera session. It uses the retained official Linux executable and
matching debug symbols. It does not establish MATRiX's runtime frame rate.

## New evidence

Raw evidence lives under `third_party/matrix/analysis/v1.1.2/`:

- `worker-control-20261013/`: 10 full-function machine-code records.
- `sensor-drop-20261013/`: 4 full-function machine-code records.
- `worker-control-20261013/projection.elf`: 71,448-byte analysis-only ELF with
  14 selected functions and 53 initialized ranges compared byte-for-byte with
  the original executable. It preserves the original virtual addresses.
- `rea-worker-control-20261013/02-batch_decompile.json`: REA/Ghidra returned
  all 10 requested functions with `status=ok`.
- `worker-control-20261013/pseudocode/`: readable exports of those exact responses.
- REA evidence ID: `ev_76d7aa353a41f4c2ad281f866ad493ae27aa309cbe926b05b5125889abcbf3b4`.

Original source SHA256:
`ed98e648c3298d0efec44170f48b0ad528c9d3010ad743a0f0dacf20d52f7bc1`.
Projection SHA256:
`cdbdee97a10edeff2e69c2568de9f8fb57c09142922299ff6d52316103ca19b9`.

Function boundaries and names originate from the official matching debug sidecar.
Ghidra's pointer types and signatures of unselected callees remain approximate;
the machine-code call records are the authority for those boundaries.

## Physics, presentation and publishing

`FMujocoSimulationWorker::Run` (`0xddd8a90`, prior evidence) steps MuJoCo and
collects state under its physics mutex. After `PublishGeomPoseSnapshot`, it
unlocks that mutex before the robot-state publisher enqueue and
`PublishVirtualSensorSamples`. See prior pseudocode lines 606–617. This narrows
the physics critical section; it does not prove all publish operations are bounded.

The newly inspected `PublishGeomPoseSnapshot` (`0xdddc4f0`) owns a separate
snapshot mutex at worker `+0x3a0`. It keeps float arrays sized `3*N` and `9*N`,
representing geom positions and rotation matrices, and a sequence at `+0x438`.
The selected-geom branch preserves the same indexed storage. This is a geom
snapshot, not a fixed 21-body robot layout.

`CopyLatestGeomPoseSnapshot` (`0xddddb20`) locks the same mutex and compares
the consumer's sequence with the latest nonzero sequence. It copies the two
arrays only when a new sequence exists, then updates the consumer's sequence.
That establishes latest-value presentation rather than a requirement to display
every intermediate physics step.

`AMujocoSceneActor::Tick` (`0xdd5d3c0`) has both a worker path and a
`StepMujocoOnGameThread` fallback (`0xdd5d6a0` call site). Do not claim the
entire product always uses a worker. The covered worker path starts and stops
the worker and calls `UpdateComponentsFromMujoco` for presentation.

For LingTu, the useful migration is a native physics/control hot loop and a
bounded latest-pose bridge. Keep our stable IDs, model/reset generations and
typed DDS contracts; MATRiX's float matrix layout is not a reason to replace them.

## Command freshness and safe hold

`ApplyBufferedRobotCommand(double)` (`0xdddae90`) calls the command buffer's
`GetLatest` at `0xdddaefd`. A fresh native command is copied, while a payload
command uses `DecodeRobotCommandPayload` at `0xdddafd1`.

At `0xdddaff2`–`0xdddb011`, it requires a command to exist, be decoded, and
satisfy `now - command_time <= configured_timeout` (worker `+0xb0`). It then
calls `ApplyRobotCommandNative` at `0xdddb030`. The exact clock provenance of
`command_time` still requires the buffer producer/getter to be inspected.

After previously applying a valid command, a missing/expired/failed command
enters `CaptureSafeHoldTargets` (`0xdddb07f`) and `ApplySafeHoldControl`
(`0xdddb09c`). Before the first successful command, its fallback clears the
actuator buffer. Those cases must not be described as one unconditional hold policy.

`ApplySafeHoldControl` (`0xdddf010`) refreshes held targets when their layout
is unavailable or changes, then calls `ComputeSafeMotorControl` for eligible
actuators. Raw constants passed to that helper include doubles `8.0` and `0.8`.
The helper's full actuator/gear/saturation semantics are not covered here, so
these are not transferable Thunder gains. Safe hold also differs from a normal
operator-requested zero velocity; it must not replace locomotion stopping logic.

## Camera and synchronization

Existing camera machine-code evidence establishes:

| Function | Concrete behavior |
| --- | --- |
| `Tick` `0x11a57110` | Processes GPU readbacks at `0x11a57170`, polls capture scheduling at `0x11a57456`, then conditionally captures at `0x11a57483`. |
| `SubmitPerspectiveGpuReadback` `0x11a5c790` | Calls `CaptureScene` at `0x11a5c7e1`, then enqueues texture readback at `0x11a5cd06`. |
| `ProcessGpuReadbacks` `0x11a57590` | Polls/copies completed results and queues them for publication. |

The new `HandleRobotSensorSynchronizedTrigger` (`0x11a5b860`) copies the
trigger into actor storage, enables a synchronized-trigger flag (`+0x4c4`),
calls `CaptureAndPublish` at `0x11a5b8ac`, then clears the flag. Thus an external
group-trigger path exists beside ordinary camera Tick scheduling.

`NotifyProducerUnavailable` (`0x11a59500`) uses `SecondsToNanoseconds(1/frequency)`
at `0x11a59579`, records scheduling time/generation at `+0x18`/`+0x20`, and updates
the `+0x24` scheduling flag. It handles unavailable sampling through scheduler
state. This function contains no session-wide stop or physics-worker termination.
That limited observation does not prove every camera error is nonfatal.

`URobotSensorSyncWorldSubsystem::DispatchGroup` (`0x11a683a0`) sorts registered
members, runs their prepare interface, collects ready members and has a separate
commit interface pass. `Tick` (`0x11a69160`) advances overdue trigger deadlines
in a loop, carries the count of skipped deadlines, then dispatches one selected
trigger. This is evidence of grouped prepare/commit and late-trigger coalescing;
the concrete interface slots and enum labels need further resolution before
publishing an exact drop-policy implementation.

The distributed default `config.json` explicitly has `sensor_sync_mode=true`.
Therefore ordinary Tick scheduling alone does not establish the default complete
capture path. We have not proven that every GPU request has a fixed timeout or
which oldest/newest/whole-group policy is selected at runtime.

## Our current failure and correction

Run `build/ue-motion-runs-20261013/manual-20261013-163005-02ada9cb` records:

- Camera becomes ACTIVE at 08:30:51 UTC.
- At 08:31:20.594, `front_rgb` reports `DeferredCapture age=1.261`, with a
  1-second deadline. The episode retains that as its first failure.
- The required camera stream failure then marks the coordinator FAILED and
  stops the owner. The later `operation requires RUNNING` is a secondary error.
- Frames 4→271 span 29.512 seconds, around 9.05 FPS; this does not prove 30 Hz.

The directly supported code defect is in `EvaluateDeferredCapturePromotion`:
it rejected wall-clock age before checking whether the new game frame and free
readback slot already allowed progression. The repair gives eligible progression
priority after validity/identity checks. A same-frame or full-ring blocked request
still observes the bounded timeout. GPU/readback/decode checks remain separate.

Controlled build: `build/unreal-tests/ubt-camera-promotion-20261013.log`,
`Result: Succeeded` (330.16 seconds). Camera Automation under NullRHI:
`build/unreal-tests/camera-promotion-20261013/report/index.json`, 14 Success,
0 failed, 0 not-run, exit code 0. `DeferredPromotion` explicitly covers the
observed 1.261-second age and retains timeout failure for a full readback ring.
The regression also checks invalid clock/identity and reset cases.

The local manual launcher now includes the episode's first failure reason in
its terminal message alongside the owner exception. Its syntax check passed;
the previous failed run artifacts remain unchanged.

The test initially used the Editor's default Zen cache and logged storage-507
responses. C: has about 1.88 GiB free. Future runs of the local test runner use
the already-writable D: file cache, matching the manual launcher. The actual
camera test results above are from the original completed run; no cache cleanup
or retest result is invented. The manual game already used its D: cache, so
the test-cache issue is not evidence for the game's earlier deadline failure.

No interactive scene was relaunched in this pass. The repair does not itself
establish the new live session frame rate or resolve post-contact stopping drift.
The owned Editor/UBT and REA/Ghidra analysis processes exited after verification.

## Next migration work

1. Close the camera promotion regression and preserve the first error in the
   operator-facing launcher output.
2. Measure separate physics, game-thread, render-thread, GPU readback and publish
   costs in a bounded live run; optimize the observed dominant cost.
3. Resolve the safe-motor helper and command-buffer timestamp semantics before
   independently implementing per-actuator timeout hold in our native runtime.
4. Resolve synchronized sensor prepare/commit interface slots and concrete
   drop-policy enums, then implement the appropriate policy in our own scheduler.

Use the ordinary LingTu runtime/package seams for these implementations. The
reverse-analysis directory remains evidence and tooling, not another simulator.
