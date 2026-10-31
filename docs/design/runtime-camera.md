# Runtime player camera

## Mouse controls — 2026-10-15

The runtime driving view supports these local player-camera actions:

| Input | Intent |
| --- | --- |
| Mouse wheel | Zoom the follow view by changing orbit distance; dolly forward/backward in Free |
| Hold right mouse and drag | Orbit in Follow; turn the viewing direction in Free |
| Middle mouse click | Cycle the existing runtime camera modes, like C |
| C | Existing Follow / Inspection / Free mode switch |
| Release right mouse, leave Drive, lose focus/capture | End mouse look; returning requires a new press |

WASD/QE remain robot movement, without Shift. Free camera does not secretly take
those keys away from the robot. Left mouse remains available to UI/other gameplay.

## Ownership

The project currently uses its existing Slate input preprocessor, not a newly
introduced Enhanced Input mapping stack. Mouse deltas have a separate named
`FRuntimeCameraInput` callback. They do not become robot axes, deadman claims,
MuJoCo commands, DDS sensor changes or additional protocol fields.

The camera adapter requires Drive mode, active owned viewport focus, no front-end
selector/text editing, and actual mouse ownership. A captured pointer's Slate
path must contain the configured game viewport. Without capture, a wheel/click
must hit that viewport; the first uncaptured right click is allowed through to
SViewport's normal capture path. Mouse look itself requires owned capture.
Losing capture clears its latch, including Editor/PIE release via Shift+F1.

The UI world subsystem forwards degrees and fractional wheel steps to
`ULingTuSimVisualWorldSubsystem::AdjustRuntimeCamera`. It updates/echoes camera
mode only after the backend confirms a real view target. Existing gamepad free
look remains separate and unchanged.

## Backend behavior

- Follow preserves the exact existing default local camera offset
  `(-300, 300, 180)` cm and target `(100, 0, 20)` cm. Orbit/zoom changes persist
  through subsequent truth ticks. Distance is clamped to 100–2000 cm and
  elevation to -80–80 degrees. Rebind/destruction resets these offsets.
- Free changes only its camera Actor: bounded pitch, degree yaw, and local-forward
  dolly at 50 cm per wheel step. Wheel does not alter FOV.
- Inspection remains a fixed source camera. A nonzero adjustment clones its
  current view into Free before editing; the original inspection/sensor camera
  transform and FOV are untouched. The actual mode then becomes `free`.
- Invalid values or an unconfirmed nonzero view adjustment are rejected. Zero
  input does not create cameras or fake bindings.

No robot transforms, physics authority, scene collisions or sensor calibration
are changed. This pass does not add terrain/wall occlusion avoidance for the
player camera, camera rebinding UI or per-user sensitivity settings.

## Menu boundary

Camera actions belong in driving input, not the player menu. The accepted
Continue / Restart / Settings / Main menu / Quit design and remaining lifecycle
work are tracked in [runtime-hud.md](runtime-hud.md). This camera implementation
does not imply that restart, settings or return-to-main-menu are complete.

## Verification — 2026-10-15

- Final Editor build succeeded:
  `build/unreal-tests/ubt-mouse-camera-20261015-r4.log`.
- All 64 UI/input cases passed without warnings in
  `build/unreal-tests/mouse-camera-20261015-r2/report/index.json`, including
  `MouseCamera.Actions` and `MouseCamera.ContextGuards`.
- Both affected camera cases passed in the final focused rerun, with zero
  warnings, failures or skipped cases:
  `build/unreal-tests/mouse-camera-context-20261015/report/index.json`.
  These are `CameraAdjustment` and `CameraModesUseConfirmedViewTargets`.
  This is separate evidence, not a claim of a new single 66-case clean report.
- The first input run exposed a float-tolerance assertion issue; its report is
  retained. The combined rerun exposed missing test-world contexts during camera
  cleanup. Actual world contexts were added to those two fixtures; no warnings
  were suppressed. The final focused rerun above is the corrected evidence.
- Final process inspection found no UnrealEditor, UnrealBuildTool,
  AutomationTool, MuJoCo headless, dotnet, cl, link or MSBuild processes.

No physical-mouse gameplay walkthrough, new scene capture, or MuJoCo driving
qualification was performed in this pass. These automated results do not claim
camera collision avoidance or completed player-menu lifecycle actions.
