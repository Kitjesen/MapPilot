# Front-end menu study — 2026-10-17

## Decision

The user rejected the light welcome/login form. Do not treat that version as the
approved art direction. Preserve the separate robot/map selection flow, but make
the entry screen a composed scene with a short main menu. Operator identity is
secondary; entering the application is not authenticating an online account or
starting physics.

This is a research result and original design proposal, not a new rendered UI.
No production UE source or content was changed in this research pass.

## REA update and actual capability

- Official upstream: <https://github.com/morluto/rea>.
- npm's `latest` and the official release page both identified **6.5.0**:
  <https://github.com/morluto/rea/releases/tag/rea-agents-6.5.0>.
- `D:/inovxio/tools/rea` was clean and fast-forwarded from `c22111eca` to
  release commit `3b79af28e`, not to unreleased main.
- Existing CLI prefix `D:/inovxio/tools/rea-cli` was updated from 6.4.0 to
  exact `rea-agents@6.5.0`. The existing `rea.ps1 --version` returned `6.5.0`.
- No changes were made to the separate Linux CLI installation.
- Codex doctor reported registration drift/missing skill identity. No Codex
  configuration or approval policy was changed. This pass used the working CLI;
  it does not establish an active REA MCP connection in this chat.

The new CLI successfully inspected Palworld's actual shipping EXE's PE resources:
`build/game-ui-research-20261017/frontend/rea-palworld-pe-resources.json`.
It reported PE32+, 19 examined resource entries, 7 resources, complete coverage
of that resource table. Evidence ID:
`ev_75c04d3e557e4aec325bf8884632367782e40326b06db078a98c8ace14a9f514`.
Resources include icon, dialog, group-icon, version and manifest types; opaque
payloads do not establish their displayed behavior or the game menu layout.

REA still rejected the selected cooked Hogwarts `.uasset` as
`unsupported binary format`. A directory inventory attempt also failed because
that route requires a file or app bundle. Both original failures are retained in
`rea-package.log` and `rea-inventory.log`. Neither is reported as a recovered
Blueprint graph.

## New target-specific evidence

The installed games supplied by the user were read without launching, patching,
or modifying them. No encryption key or protection bypass was used. Existing UE
5.8 UnrealPak extracted exactly **15 files / 924,231 bytes** into:

`build/game-ui-research-20261017/frontend/`

- Hogwarts: 7 narrowly selected cooked packages.
- Palworld: 4 selected widget packages, each with `.uasset` and `.uexp`.
- `commands.json` retains exact filters and exit codes; all 11 extraction calls
  returned 0.
- `package-strings.json` records printable-byte observations and file offsets.
  These are transparent fallback observations, not REA decompilation.
- `research/game-ui/inspect_frontend_packages.py` is the bounded acquisition script.
  It requires a fresh output directory; do not delete prior evidence to rerun it.

### Hogwarts: scene and menu are separate assets

| File | Observed references/names | Supported conclusion |
| --- | --- | --- |
| `Levels/UI/Frontend.umap` | `CIV_Frontend_Ambient`, books/creature sequences; scene props; `SUB_Global_Light`, `SUB_Study_Hub_Light`; `CharacterRig`, `CameraLookAtTarget`, `PostProcessVolume` | A dedicated front-end scene references environmental content, lighting, character and camera-related objects. Not merely a flat UI background asset. |
| `Levels/UI/BP_FrontEndCamera.uasset` | `/Script/CinematicCamera`, `CineCameraComponent`, filmback/lens/focus names | There is a dedicated cinematic-camera package. Exact focal lengths and active settings were not decoded. |
| `Cinematics/Misc/FrontEnd/CIV_Frontend_Ambient.uasset` | `/Script/CameraStack`, `DA_CameraMod_AmbientCamAnimIdle`, transition-in/out names, `Frontend:PersistentLevel.CharacterRig` | An ambient sequence carries camera-related references and a front-end character-rig path. Exact motion and duration remain unknown. |
| `UI/Menus/UI_BP_FrontEnd_Menu.uasset` | Character-select button, menu-text button, start page, load menu, system menu and details-panel paths; four `characterSlot` names; hover/unhover/click delegates | Front-end responsibilities are distributed over reusable UI packages. Names do not recover the widget hierarchy or actual callback behavior. |
| `UI/Menus/UI_BP_FrontEndDetails.uasset` | `SetButtonDetails`, `SetCharacterDetails`, `CharacterName`, `characterTimeData` | Details presentation has its own package; a useful design reference for separating selected-item information from the menu. |

Useful exact offsets in `package-strings.json`:

- Frontend map: `CharacterRig` 24819; `Default__PostProcessVolume` 26530.
- Camera package: `CineCameraComponent` 413.
- Ambient package: idle camera modification reference 237; character-rig path 8754.
- Menu package: character-select reference 1209; details reference 1327;
  menu-text button reference 1396; `characterSlot1` 13053.

### Palworld: title actions and input feedback are components

| File | Observed references/names | Supported conclusion |
| --- | --- | --- |
| `Title/WBP_TItle.uasset` | `BP_GetDesiredFocusTarget`, `GetTopButton`, `SoftLastClickedButton`; options-settings and local-world-select paths; quit-dialog names | Title screen contains explicit focus-related names and references to separate options/world-selection screens. Runtime focus restoration was not tested. |
| `UserInterface/Title/WBP_TitleMenu.uasset` | Distinct local-start, options and exit button names/click delegates; menu-button package reference | Main actions are represented by separate controls rather than an operator-name form. This is structural evidence, not an exact screen reconstruction. |
| `UserInterface/Title/WBP_Title_MenuButton.uasset` | `Anm_Normal`, `Anm_Focus`, `Anm_Push`, `Anm_Disable`; `CommonButtonBase`; hover/unhover names; color tracks | Button package explicitly distinguishes normal/focus/press/disabled treatments. Their durations, curves and final appearance remain unknown. |
| `UserInterface/Title/WBP_Title_MenuBG.uasset` | `BackgroundBlur`, `Default__BackgroundBlur` | A separate background widget references blur. Exact blur strength/placement was not decoded. |

Button animation names appear at byte offsets 280/300/318/337. Title focus
target appears at 492; `SoftLastClickedButton` at 6298. Main-menu local-start
button name appears at 4541. The background blur name appears at 605.

## Original LingTu adaptation

The design judgment below is ours, not a claim about recovered game source.

1. **Entry screen is a place.** Compose a robot operations base with foreground,
   robot staging area, and distant exterior view. Use directional light, soft
   fill, believable contact shadows and restrained materials. Do not simply
   reuse the selector's blank gallery and add a text form on top.
2. **Short main menu.** Begin simulation, robot library, settings, exit. Only
   bind features that exist. A continue-session action stays absent until there
   is an actual resumable session contract.
3. **Operator name is secondary.** A small editable local identity entry can
   appear in preparation/profile settings. No repeated large login form, fake
   online account, catalog counts or engineering readiness text dominating the
   entry screen.
4. **Give the scene a directed camera.** A slow, optional camera drift and
   restrained environmental movement may provide life. Their speed/easing must
   be independently designed and measured, not guessed from asset names.
   Menu presentation must not start MuJoCo or claim live robot operation.
5. **Use button state feedback.** Original normal, focus/hover, pressed and
   unavailable treatments; keyboard/gamepad navigation, visible focus and
   restoration on return. No large web-form CTA as the entire main menu.
6. **Keep the established preparation flow.** Main menu → robot → map → task
   review → explicit launch. Robot selection remains directly inspectable in 3D.
7. **Budget presentation work.** One active scene/camera for entry, no hidden
   duplicate captures each frame, cap rendering when idle, and provide reduced
   motion. Real implementation must verify these claims.

Recommended next delivery: one original main-menu composition and its actual
UE render before extending the rest of the UI. Retain existing scene-selection,
settings and process-ownership contracts rather than replacing the whole UI stack.

## What is still unknown

- Exact commercial-game UMG tree, anchors, margins, animation curves and timing.
- Their live transitions, audio cues and controller focus behavior.
- Whether every referenced/debug asset is active in the installed game build.
- The final scene direction preferred by the user (asked separately).

No claim of full reverse engineering, exact reconstruction, new playable
qualification or improved robot rendering is made. Commercial assets remain in
local research output; none were copied into RobotSimUE Content.
