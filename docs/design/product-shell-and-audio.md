# Product shell and audio completion

Updated 2026-10-20. Prioritize complete user flows over additional empty screens.

Latest continuation: `factory-front-end-and-music.md`. The factory background is
now rendered in the actual default front end, with two original menu-music
drafts. The earlier batch record below is retained as historical evidence.

## This delivery

1. Finish real audio preferences: master volume, UI-cue volume and mute.
2. Keep preview, save, undo and discard in the existing settings transaction.
3. Preserve keyboard/gamepad category navigation, focus restoration and graphics
   confirmation. Entering a menu must not start physics or release motion.
4. Verify real UE widgets and persistence, then capture the implemented page.

Hogwarts package references and their limits are documented in
`research/game-ui/HOGWARTS_AUDIO_20261020.md`. Original LingTu PCM cues are used;
commercial game audio is not a distributable product dependency.

## Implemented and verified in this batch

- Added the audio category to the existing Slate settings: master volume,
  independent UI-cue volume, mute, preview and page-local defaults.
- Saved preferences use `LingTuSim.Player` in the existing user-settings INI.
  Config reload, undo and unsaved-discard preserve the selected levels and mute.
- Master playback uses the selected UE audio device's transient primary volume;
  UI gain is passed once to LingTu cue components. Windows volume is untouched.
- Keyboard/gamepad category cycling includes audio. Existing unsaved-decision,
  focus and graphics-confirmation behavior is preserved.
- A launch with disabled audio explains that condition and disables preview.
  The ordinary selection launcher does not force `-NoSound`; legacy diagnostic
  coordinator profiles still do and were deliberately not rewritten here.
- Final RobotSimUEEditor build succeeded. The focused menu/input/settings/audio
  Automation suite passed **48/48**, with zero failed, not-run or warning results:
  `build/unreal-tests/audio-settings-20261020/flow-green-r2/report/index.json`.
- Test-first evidence is preserved: missing gain API compilation failure, one
  passing gain test, a missing-audio-category flow failure, and a subsequent
  46-pass/1-fail run whose fixture read a Slate attribute before prepass. The
  fixture now performs the UI update before checking the rendered enabled state.
  A shadowed local-name build failure is also preserved rather than overwritten.
- Real RHI capture succeeded at 1600×900 for audio, controls and interface:
  `build/unreal-tests/audio-settings-20261020/render/`. Those three images were
  visually inspected with no text clipping observed. This preview omitted
  `-NoSound`; the log records successful WASAPI device initialization. It is not
  a loopback recording or a subjective speaker/listening qualification.
- Three WAV examples were generated from the actual original cue function by
  the Automation suite under
  `sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/AudioSamples/`.
- Render exited naturally; the final process scan found no UE, shader worker,
  UBT/UAT, dotnet, cl/link/MSBuild or MuJoCo process. User applications were not closed.

This verifies the Editor-backed UI, preferences and cue-generation path. The
standalone packaged game was not recooked, MuJoCo was not launched, and the
approved factory background has not yet replaced the current menu stage.

## Ordered product work

| Priority | Deliverable | Completion evidence |
| --- | --- | --- |
| P0 | Reliable shell: start, settings, back, quit; audio/display/control preferences | Native UI automation, saved preferences reload, rendered screens; no hidden runtime starts |
| P0 | Approved Factory Environment background | Clean UE capture and actual main-menu render; no editor overlays; licensed source retained externally |
| P0 | Robot → map selection and honest availability | Interactive robot inspection; selections preserved on Back; only supported combinations launch |
| P0 | Selection → loading → running → return | One ProductControl-owned lifecycle, useful loading/failure feedback, owned cleanup; no second UI resolver |
| P0 | Thunder motion/contact in a selected factory bay | Same-run UE/MuJoCo truth, collision alignment, release-stop and Reset; not inferred from a menu preview |
| P1 | Inspection task and takeover | Current target, route, progress and manual takeover bound to native task/control state |
| P1 | Task results and evidence | A complete inspection with recorded observations and truthful completion/failure status |
| P1 | Performance and distribution | Measured frame/stream rates on the target machine, packaged Windows smoke test, bounded loading and exit |
| P2 | Additional robot controllers and interactive machinery | Per-package controller/physics qualification; visual model availability alone is insufficient |

## Boundaries

- UE owns presentation and input intent; MuJoCo owns simulation time, motion and contacts.
- UI/master sound preferences affect UE playback, not sensor measurements or DDS data.
- A settings test is not a motion, rendering-performance or packaged-product test.
- This document orders the work; it does not mark unfinished rows complete.
