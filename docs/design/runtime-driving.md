# Runtime driving delivery

This slice uses the existing ThunderV4 1.0.3 model and FactoryPark 1.2.1
compiled session. MuJoCo owns motion and contacts; RobotSimUE follows its
snapshots. This is an operator-driving diagnostic, not a qualified product or
an autonomous-navigation claim.

## Execution order

1. Keep the owner loop alive through normal atomic camera-readiness updates.
2. Verify UE input -> Control admission -> native MuJoCo displacement, upright
   posture, wheel/ground contacts, and zero/release-stop behavior in one run.
3. Inspect that run's real video and camera view, not a static model showroom.
4. Verify camera cycling, HUD data, and runtime menu against the same session.
5. Connect task UI only to the existing verified task projection; missing tasks
   remain explicitly unbound. Automatic inspection needs navigation-stack
   integration and is not manufactured by the HUD.
6. Improve lighting/materials and measure smoothness in that running scene.

## Work completed on 2026-10-14

- ExternalEvidenceWatcher no longer pins the old file inode across sharing
  retries. A normal atomic producer replacement can be consumed next tick.
  Per-read identity/version validation and finite retry bounds remain.
  Regression: RED reproduced the observed exception; GREEN 35 tests passed.
- Drive HUD shows the actual camera mode. The runtime menu's camera action
  restores the previous mode before dispatching the existing camera handler.
- Requested input, admitted motion, observed motion, sensor and recording
  readouts now explain unavailable, stale, and wrong-identity states in Chinese.
- Controlled UI build and 58/58 Automation passed; locomotion diagnostic
  contracts passed 11/11. This is not a live movement acceptance verdict.
- The asynchronous HUD fix now keeps identity checks in the reader and exact
  sample alignment in the capture predicate. A real UWorld/Visual/SessionService
  regression publishes correlated status sequence 600 against applied Visual
  sequence 601 and verifies that recording remains available without falsely
  qualifying a same-frame capture. Updated UI Automation passed 60/60 with no
  failed, skipped, or warning results; the focused independent review approved.

## Interrupted actual-scene run

`build/ue-driving-live-20261014/manual-20261014-143023-e4d510e1` contains actual
FactoryPark screenshots, 70 native truth frames, admitted UE forward inputs,
and wheel/road contact forces. The base moved horizontally about 0.0355 m in
the captured interval. Its final sampled pose had 12 road contacts.

The execution stopped before probe completion and natural terminal evidence;
there was no completed locomotion verdict, driven image, or stop acceptance.
The MP4 lacks its final moov index and is not a usable delivery video. Runtime
and health manifests remained stale RUNNING after owned processes disappeared.
The exact run was explicitly recovered through `manual_run_recovery` after
resource-dead proof, resulting in FAILED without fabricated shutdown evidence.
Do not present it as a successful run or overwrite its artifacts.

The run also exposed an asynchronous HUD bug: current-generation Control and
Visual sample sequences differ normally, but the status reader treated their
temporal mismatch as identity mismatch. Normal HUD identity must remain based
on run/session/boot/source/model/reset. Exact sample sequence/time alignment
belongs to strict screenshot capture qualification.

## Still to prove

- One uninterrupted native/UE movement-and-release run with readable footage.
- Stable turn/strafe and obstacle traversal/contact behavior, not merely ground
  contact or a renderer-only collision response.
- Physical keyboard foreground operation, sustained real-time factor and
  render-sensor frequency under the actual available memory/GPU conditions.
- In-scene camera/menu interaction and real task integration.

The previous run used targeted messages to its owned window. This proves a
diagnostic UE-intent path only, not physical keyboard foreground qualification.

## Next acceptance steps

| Order | Deliverable | Observable acceptance |
| --- | --- | --- |
| 1 | One complete FactoryPark driving run | Native displacement, upright pose, release-stop, valid video, natural terminal cleanup |
| 2 | Driving UI in that real scene | Actual follow/inspection/free camera switch; requested/admitted/observed values remain distinguishable |
| 3 | Obstacle interaction | Named MuJoCo wheel/obstacle contacts and stable recovery, not a UE-only collision effect |
| 4 | Task and runtime menu | Existing task state projected into UI; pause/record/exit use their existing owners and acknowledged outcomes |
| 5 | Usable presentation | Lighting/material fixes and measured frame/sensor timing in the same scene |

Unproven steps stay unproven. A local Automation pass does not substitute for
the actual coupled-run evidence or physical keyboard operation.

## Complete actual-scene diagnostics

`build/ue-driving-complete-20261014/manual-20261014-182141-7d198300` completed
the owned profile naturally and produced a valid 22-second 1920x1080 H.264 MP4.
Its probe result was lost when Python's default GBK text decoding encountered
the UTF-8 Chinese window title. The diagnostic is FAILED, not retroactively
reinterpreted as a pass. The consumer now specifies strict UTF-8; the PowerShell
producer does the same. The probe source is retained under
`sim/tools/diagnostics/probe_drive_input.ps1`, not just an ignored build folder.
The local manual launcher now selects that controlled source. The consumer and
real Windows Chinese JSON round-trip regression passed 12/12 (one existing
pytest cache option warning remains).

The corrected follow-up run is
`build/ue-driving-utf8-20261014/manual-20261014-182827-29f3d2a9`:

- Native accepted-hold displacement: 0.329159535 m, 138 truth samples.
- Upright hold/release: passed; minimum body-up dot 0.999917, minimum base height
  0.449293 m, maximum roll 0.012370 rad, maximum pitch 0.006674 rad.
- Wheel/obstacle interaction is present in native MuJoCo contacts: 122 journal
  samples reference a speed bump; maximum contact count is 14. This does not
  prove successful obstacle traversal or recovery.
- Release request, released authority response, and accepted zero audit were
  observed. Final release-window planar speed is 0.011311 m/s, but maximum
  post-release drift is 0.081692 m, above the unchanged 0.05 m gate: FAILED.
- Press/hold origin correlation also FAILED. The persisted first accepted
  datagram does have matching origin bytes. The same-frame control claim is
  sequence 4, following first accepted motion sequence 3; the probe currently
  expects a prior claim. Resolve production ordering and evidence consumption
  with a regression, not by ignoring correlation.
- MP4: valid H.264, 1920x1080, 220 frames, 22.000 seconds; midpoint and driven
  screenshots were visually inspected. The scene is still too dark; a +1-stop
  diagnostic exposure does not solve environment fill or material quality.
- `latest.json` is CLOSED; `session.runtime.json` is STOPPED. The episode's
  SUCCEEDED means the runtime lifecycle ended normally, not that the failed
  diagnostic qualified. No UE/MuJoCo/UBT/UAT/dotnet/cl/link/MSBuild/ffmpeg process
  remained in postflight.

No formal product, physical foreground-keyboard, camera/menu live-interaction,
or autonomous-navigation qualification is claimed. Preserve both failed runs.

Immediate next implementation is the release braking behavior and
claim-before-motion/evidence consumption regression. `ControllerRuntime.hold`
and the Thunder adapter's existing wheel-braking `safe_stop` are the existing
seams to investigate; the observed drift alone does not prove their cause.
Do not teleport the body, relax the stop threshold, or give Chaos motion
authority to make the visual result appear to pass.

## Wheel-leg driving range improvement

The operator selected wheel-leg driving, not a new legged gait. The old
playable input mapping capped every translation at 0.10 m/s. It now accepts
explicit positive finite `drive_speed_limits` using the existing BaseTwist
value. Unconfigured callers retain 0.10/0.10 m/s and 0.35 rad/s. Controller
submission and admitted HUD status use the same configured scaling, with the
existing input normalization and stale-input zero unchanged. The local manual
launcher selects 0.60 m/s forward/reverse, 0.15 m/s strafe, and 0.60 rad/s yaw.
This does not alter field RobotConfig, native navigation, package policy gains,
mesh scale, or visualization authority.

The authored session `sim/sessions/examples/thunderv4_factory_park_drive/session.yaml`
starts the same robot at [0, -80, 0], facing the open entrance road. Catalog
resolution produces `build/simstudio/thunder-drive-20261014`; no runtime edits
are made directly to the previous compiled plans.

Actual coupled evidence on 2026-10-14:

| Run | Input | Native movement | Stop/upright | Overall diagnostic |
| --- | --- | --- | --- | --- |
| `build/ue-drive-range-20261014/manual-20261014-212737-635f42c4` | W for 14 wall seconds, admitted 0.60 m/s | 5.854330 m over accepted-hold truth; maximum observed speed 0.594763 m/s | Upright passed; drift 0.049734 m and final planar speed 0.000529 m/s passed the unchanged planar-stop gates | FAILED because the old PowerShell origin counter miscounted singleton records; preserve the original result |
| `build/ue-drive-turn-20261014/manual-20261014-214203-b45d3ed3` | Q for 4 wall seconds, admitted 0.60 rad/s | Positive signed yaw 1.341280 rad (76.85 degrees); maximum observed yaw speed 0.571860 rad/s | Upright and planar-stop passed; planar drift 0.004586 m. A native sample at 13.600 s also shows angular Z velocity -0.000037665 rad/s; angular settling is an additional observation, not a new formal gate | PASSED, manual targeted-message diagnostic only |

The W origin-count failure was reproduced in real Windows PowerShell 5.1:
one PSCustomObject has scalar `.Count == null`, unlike PowerShell 7. The probe
now uses `@($origin).Count`; a real correlation regression locks the fix. No
old verdict was upgraded. Q/E motion checks use signed yaw and observed
angular speed, not translation, roll, or pitch. Five real PowerShell data
cases reject insufficient yaw and wrong direction even with 1 m of translation.

Regression: 114 tests passed, Ruff and focused diff checks passed; one existing
pytest cache option warning remains. Independent narrow review approved.
The straight/turn recordings are valid 1920x1080 H.264 videos of 21/11 seconds
respectively, recorded at 10 wall-clock fps without time compression. Both runs
closed naturally; postflight had no owned UE/MuJoCo/build/ffmpeg processes.

This proves a substantially larger operator-driving demonstration, not
physical keyboard, long-duration performance, terrain traversal, task UI,
lighting quality, or field qualification. The dark scene still needs lighting
work. High-speed lateral motion remains deliberately limited because the
current flat policy has documented lateral contact-quality failures.
