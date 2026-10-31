# Factory front end and original music

2026-10-20 continuation of `product-shell-and-audio.md`.

## Scope

Use the user's approved, already licensed factory collection in the actual
welcome screen. Keep the robot→map flow, settings transaction and ProductControl
ownership. Add real background music rather than describing UI clicks as music.
No MuJoCo/runtime control changes, whole-factory map migration, or package build.

## Implementation

- A clean external UE render is staged as `Resources/FrontEnd/factory-corridor.png`.
  The welcome page uses it with a legibility veil, not the old operations-base
  render. A missing local background retains the existing fallback; the native
  asset contract rejects a delivery missing this required resource.
- Robot selection remains the interactive 3D presentation; a flat menu background
  does not claim that the factory is navigable or physically interactive.
- Two original 32-second stereo compositions: 基地序曲 (`base-arrival`) and 启程
  (`departure`). They are quiet electronic composition drafts, not commercial
  game recordings or final listening-approved production music.
- Menu-owned normal UE looping SoundWaves, one current component, with short
  fades at startup/track change and exit to simulation. Pages share playback;
  a new component is not created on every menu switch or loop boundary.
- Separate music volume/track, existing master/mute/UI gain, existing INI save,
  undo, defaults and discard. The selected UE device is used; OS volume and
  acoustic-sensor configuration are untouched.
- Audio assets have an explicit cook directory; the raster has an explicit
  runtime dependency. This is packaging configuration, not a completed cook.

## Evidence and limitations

- Python loop authoring was tested first: missing module RED→three passing
  contracts. WAV duration/channel/sample format, distinct arrangements, peak
  headroom and stereo seam continuity are checked from the actual PCM.
- Early capture attempts used an inappropriate cache and hit time limits.
  A later attempt exposed a reentrant screenshot request in the authored tool.
  Its new test failed on duplicate requests, then passed after setting the
  request latch before calling UE. All prior failure directories are retained.
- Corrected external capture exited 0, with `error=null`, 1920×1080 image and
  no source map save. The image was visually inspected before staging.
- The two normal game-project SoundWaves imported successfully (commandlet exit0).
- Native build/UI regression and real welcome/audio render results are recorded
  below after verification; source edits alone are not completion evidence.

## Verified delivery and corrected default entry

- `build-r2.log`: RobotSimUEEditor build succeeded. The first build's single
  C1060 compiler-heap failure is preserved in `build.log`; no unrelated source
  was reverted to hide it.
- `automation/report/index.json`: 49 native menu/settings/audio tests performed,
  47 Success plus 2 SuccessWithWarnings, failed=0/notRun=0. The two warnings are
  slow DDC reads of default OpenWorld HLOD assets, not audio/UI assertion failures.
- An initial render used `LingTuDevRuntimeUIRender`, which deliberately bypasses
  the product front end. Its misleading `welcome.png` is retained as a failed
  visual selection, not evidence of the new home. The user correctly rejected
  the visible legacy black robot exhibit.
- Added the dedicated empty `LingTuFrontEnd.umap` and set `GameDefaultMap` in
  DefaultEngine.ini. It contains no old robot-exhibit meshes. Existing robot
  model assets and the interactive robot-selection viewport are preserved.
- Removed the obsolete welcome-robot gesture step from the front-end screenshot
  driver; robot-selection inspection remains. `build-entry.log` succeeded.
- `home-render/launch.json` contains neither the old RuntimePreview map nor the
  DevRuntimeUIRender flag. The launch uses the project's actual GameDefaultMap.
  `home-render/Unreal.log` confirms `/Game/RobotSim/Maps/LingTuFrontEnd` loaded and
  `LINGTU_MENU_MUSIC_STARTED track=0 duration=32.0` occurred once.
- `home-render/home.png` was visually inspected: factory background, brand and
  Start/Settings/Quit; no model-preview label, black pedestal or drive-speed HUD.
  `result.json`: exit0, error=null, capture after 36.219 seconds, natural cleanup.
- This proves the actual Editor-backed default home and music-start path. It is
  not speaker/loopback listening evidence, a packaged build, or a factory-driving
  physics result. The two songs remain editable composition drafts.
- After the entry/driver change, `automation-entry/report/index.json` passed
  6/6 focused native entry/welcome/music-asset contracts, with no warnings,
  failures or not-run tests. The default-map config contract separately passed
  RED→GREEN. The four prior Python authoring/capture contracts passed; Ruff and
  the scoped diff check passed. Pytest retains the repository's existing
  `cache_dir` warning when its cache provider is explicitly disabled.

## Next product work

1. Verify menu→selection→loading→running→return with ProductControl ownership.
2. Migrate a selected factory bay and dependencies as a real visual-world facet;
   derive/align MuJoCo collision instead of treating a screenshot as a world.
3. Verify Thunder motion, contacts, release-stop/reset in that selected bay.
4. Bind inspection guidance/results and add appropriately licensed mechanical
   ambience to the running world, separate from menu music and sensor data.
