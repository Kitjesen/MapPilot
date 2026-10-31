# Front-end flow

Updated: 2026-10-17. Scope: the existing Slate front end, not a new launcher or
physics owner. Keep the light robot-base art direction and restrained main menu.

## Current implementation slice

Welcome → 01 Robot → 02 Map / task → existing launcher handoff.
The user explicitly rejected a separate third confirmation page. Select the map
and task, read any blocker inline, and enter directly from step 02.

- Map rows describe compatibility with the **currently selected robot**. A map
  runnable with Thunder must not appear runnable for an imported G1 body.
- Keep unsupported maps browseable. Distinguish runnable configuration count,
  preview-only, no matching configuration, quarantine, and unavailable content.
- The map page shows the selected robot, map, task and catalog description.
  Back/step-01 returns to the selected robot without changing the exact task.
- Read-only, preview-only, unsupported and committed selections cannot launch.
  The existing compiled-option confirmation and ProductControl handoff remain
  authoritative; fewer UI steps do not bypass those gates.
- Screenshot QA stops after the map page; it must not activate the final entry
  action while trying to capture a removed review page.

Verification must exercise actual Slate buttons and focus, plus a rendered
map screen. A catalog's runnable flag is not live runtime readiness or
proof of robot motion.

### Two-step verification — 2026-10-17

- Editor build succeeded: `build/unreal-tests/ubt-two-step-selection-20261017.log`.
- 13 focused Automation tests passed, including `TwoStepLaunch`, exact task
  preservation, preview/read-only launch rejection, welcome and screenshot
  contracts: `build/unreal-tests/two-step-selection-20261017/report/index.json`.
- Actual 1920x1080 map-page capture inspected:
  `build/unreal-tests/robot-map-20261017-154039/result.json`, passed. Only 01/02
  remain; the final entry button is on the map page. No launch intent or MuJoCo
  process was created, and the owned UE/build processes exited.

### Previous three-step baseline — 2026-10-17 (superseded)

- Editor build succeeded: `build/unreal-tests/ubt-selection-flow-20261017.log`.
- Welcome / RobotMapPicker Automation: 11 Success, no warnings/failures/not-run;
  `build/unreal-tests/selection-flow-20261017/report/index.json`. This includes
  `CurrentRobotCompatibility` and `ReviewQuickEdit`, with 1280x720 focus coverage.
- Actual 1920x1080 Editor Game capture passed:
  `build/unreal-tests/robot-map-20261017-144149/result.json`. Map and review images
  were visually checked for wrapping, action visibility and preview overlap.
- No MuJoCo process or selection intent; no gameplay, packaged-build or live
  handoff qualification is claimed. Owned UE/build processes were gone afterward.

## Next slices, in order

1. **Launch continuity:** inspect the existing selector-to-runtime handoff; show
   actual loading stages and actionable failure/recovery. Do not invent progress
   percentages or claim ready before the runtime reports it. Verify one actual
   successful transition and one controlled failure.
2. **First-use guidance:** contextual, dismissible hints sourced from current
   input bindings; separate camera inspection from driving. Do not create a
   second shortcut table that becomes stale after rebinding.
3. **Settings/return loop:** audit unsaved changes, display-mode confirmation,
   default restore and focus return. Reuse existing settings behavior instead of
   duplicating it; validate by changing a real setting and returning to the scene.
4. **Task/result loop:** project native task admission, progress, evidence and
   acceptance separately. Cancellation, takeover and return to menu must use
   their existing owners; a menu animation never completes a task.
5. **Art pass:** improve original modular architecture, PBR surface detail,
   composition, exposure and audio feedback. Repair robot source geometry rather
   than disguising broken surfaces with lighting. Profile an actual playable
   scene as well as the presentation-only showroom.

These are follow-up slices, not claims that they are implemented or verified.

## What to learn from AAA games

Use readable hierarchy, reversible choices, consistent back behavior, diegetic
background composition, short transitions and immediate feedback. Rebuild those
ideas with our own code/art; do not copy branded menus or proprietary game maps.

Owning a commercial game is not permission to redistribute its models, textures,
audio or cooked levels in LingTu. Inspect the applicable license before any
asset-level reuse; do not bypass DRM or encryption to obtain assets. Licensed
modular environment kits or authored/scanned assets provide an appropriate
production path. Fab assets must follow their individual license and cannot be
redistributed as a standalone asset collection.

References: [Steam Subscriber Agreement](https://store.steampowered.com/subscriber_agreement/),
[Fab Standard License](https://www.fab.com/eula).
