# Runtime interface: field operation

## Stepped deployment with previews — 2026-10-16

The robot/map workspace is now a three-page flow: **robot → map and task →
review**. Each page has one subject, a visible step rail and a single primary
action. Back returns one step and keeps the selected exact catalog option;
the first two Next actions never publish a launch intent. Only the Review
page can confirm through the existing handoff. Preview-only and unsupported
configurations retain their existing launch restrictions.

The visual treatment uses a large preview area, a narrow selection column,
neutral charcoal/white styling and a prominent bottom-right action. Reference
study included the [Forza car-selection screenshot](https://global.discourse-cdn.com/forza/optimized/4X/8/f/b/8fb7cd6636f87a00fea38e2d1d9e2fc0340a4deb_2_876x488.png)
and [HITMAN destination selection](https://images.pushsquare.com/4bb53eb12bf7c/hitman-3-how-to-import-all-levels-and-locations-from-hitman-1-and-hitman-2-on-ps5-ps4-7.large.jpg).
These inform layout and hierarchy only. The Forza forum link now redirects to
its closure notice; the indexed screenshot is the visual reference, not current
game documentation. All runtime pictures are our own existing model/world renders.

All two robot and four map entries have static previews under the plugin's
`Resources/Selection/`; `previews.json` records their source paths and renderer.
Thunder and OpenField HF use UE captures, FactoryPark and Forest HF use Blender
authoring renders, and OmniCart/basic OpenField use offline MuJoCo geometry renders.
The latter use installed MuJoCo 3.5.0 with zero simulation steps; they are display
assets, not a change to the session's 3.10.0 physics version. The preview images
are package-family illustrations and do not certify every version/scenario.

Clicking the image opens a full-size view. Its background controls are disabled;
Back/Escape closes the image before navigating a step. This is static image
preview, not a live orbitable 3D viewer. Keyboard/gamepad focus belongs to the
visible Slate controls, while selector fallback confirmation advances the current
step. The runtime library remains read-only. Existing settings, movement and
physics authority are unchanged.

Verification of the step flow: `ubt-selection-steps-focus-20261016.log` built
successfully in 50.92 seconds. `build/unreal-tests/selection-steps-focus-20261016/report/index.json`
records **88 Success, 0 Failed, 0 NotRun, 0 SucceededWithWarnings**. Tests cover
actual step buttons, retained configuration, no early launch, full-size preview
and close, selector Escape, and lost child-focus recovery on robot/map pages.
The earlier run had 86 Success and two warning-bearing tests because fixture
packages lacked preview files; missing optional images now take the explicit
empty state without trying to read nonexistent files. That original report is
retained under `selection-steps-20261016`.

Actual UE screenshots of all three steps were captured and visually inspected in
`Saved/Automation/LingTuSimUI/FrontEnd/robot-map-20261016-165404/` under the UE
project: `asset-library.png` (robot), `map-selection.png`, and
`review-selection.png`, each 1920x1080. The existing screenshot driver now advances
only the first two steps, never confirms, and writes all four captures including
login to its success record. Runner exit was 0; no selection intent or physics
session was created. Independent follow-up review approved the scoped fixes.
Postflight found no UE/UBT/UAT/MuJoCo/Blender/dotnet/cl/link/MSBuild processes.

## Robot and map selection — 2026-10-16

The prelaunch workspace now uses a separate `SLingTuSimRobotMapPicker`: two
independent lists for robot and world packages, a selected-pair/version summary,
availability and blocker text, and a gated Enter Scene action. The old asset
review workspace remains reachable through a secondary action, rather than
being the deployment screen itself. The player menu has a Robot and Map library
entry; that runtime page is deliberately read-only.

`FGameSelectionModel` remembers the requested robot/world identities separately.
It resolves only a matching existing catalog option. Reselecting the same package
preserves the current exact option; moving to an ambiguous pair requires an
explicit Scene Configuration choice. It does not fabricate a Cartesian product
or compose new bundles from UI input. An unsupported pair keeps both requested
selections visible, has no selected option and cannot confirm. Exact versions,
session identity and bundle come from the selected option, not from the row's
deduplicated package identity. Committing the existing selection handoff freezes
both lists. Scene configurations select exact catalog entries; arbitrary package
version editing or new scenario authoring is not exposed by this component.

The local catalog contains six declared entries, including two compiled runnable
Thunder sessions (FactoryPark and OpenField). OmniCart FactoryPark is explicitly
preview-only: its Unreal session has no controller, and its controllable session
is currently headless-only. Forest and other review entries retain their original
non-runnable status. Catalog registration is not controller or motion qualification.

The selector continues to publish the existing immutable intent. The external
launcher revalidates the exact SessionBundle. A reproduced integration failure
showed that the launcher rejected the compiler's known `asset_review` field;
the reader now accepts that field as well as the legacy v1 field set, while
rejecting unrelated top-level fields and retaining catalog content checks.

This does **not** implement live robot replacement, map travel with physics left
running, or a complete stop/reselect/relaunch loop from the runtime menu. Those
must be integrated through the existing lifecycle owner. Do not present the
read-only runtime library as a completed switch operation.

Verification:

- Red: `build/unreal-tests/robot-map-picker-red-20261016/report/index.json`
  records the missing root-menu library entry before implementation.
- Controlled Editor build: `ubt-robot-map-picker-green-20261016.log`, Succeeded.
- `build/unreal-tests/robot-map-picker-green-20261016/report/index.json`:
  **86 Success, 0 Failed, 0 NotRun, 0 SucceededWithWarnings**. This includes
  actual Slate button dispatch, exact pair resolution, preview/unsupported
  launch gating, confirmation freeze and read-only runtime behavior.
- Independent review identified ambiguity when multiple catalog entries shared
  robot/world IDs. The fix preserves the exact selected configuration and adds
  explicit configuration buttons. The final controlled build
  `ubt-robot-map-picker-config-20261016.log` succeeded in 194.11 seconds;
  `build/unreal-tests/robot-map-picker-config-20261016/report/index.json` records
  **87 Success, 0 Failed, 0 NotRun, 0 SucceededWithWarnings**, including an
  actual Slate selection of the second runnable configuration with a different
  world version and session identity.
- Catalog and launcher Python regression: **50 passed**. A pytest warning about
  the existing `cache_dir` setting occurs with the cache plugin disabled.
- Follow-up scoped code review approved the exact-configuration fix with no
  blocking finding. Visual inspection then found a legacy full-page tooltip
  obscuring the picker. Removing it and clarifying the prelaunch status text
  built successfully in `ubt-robot-map-picker-polish-20261016.log` (43.69 seconds).

The runtime library requires a supplied compiled catalog. Existing manual
launches without `-LingTuGameSelectionCatalog` show its honest empty state; this
change does not silently scan source packages from a running physics session.
Full runtime re-entry must also pass that catalog through the lifecycle owner.

Final actual UE capture: `build/unreal-tests/robot-map-20261016-150153/result.json`
reports success and natural exit 0. Images are in
`sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/robot-map-20261016-150153/`.
The historical filename `asset-library.png` now shows the new robot/map workspace
with both FactoryPark configurations; it is 1920x1080 and was visually inspected.
No physics session or selection intent was started during this screenshot run.
Postflight found no UE/UBT/UAT/MuJoCo/dotnet/cl/link/MSBuild processes.

## Task journal first slice — 2026-10-16

The player menu now includes **任务日志**. The root remains a short navigation
list; pause/resume/cancel inspection intents live on the task page instead of
being mixed into the root menu. Opening the page does not start a task, pause
physics, resume navigation, or synthesize an event history.

The first page is a **current-task summary**, not a chronological event journal:

- Execution rail: native phase, confirmed point coverage, and native reason.
- Evidence rail: separately verified report status, acceptance, and evidence
  coverage, with readable Chinese labels.
- Missing task, stale task/report, or binding mismatch remains explicit; old
  progress and acceptance cannot be presented as current success.
- Task controls retain the existing native advertised-action, session and
  environment checks. Terminal tasks cannot be resumed by the menu.
- Escape returns one level; returning from the task page restores focus to its
  menu entry rather than silently entering Drive. Mouse/Enter use the same
  Slate action.
- The native `WAITING_NEXT_LOOP` phase is accepted and described as waiting,
  rather than rejecting the entire task or describing it as driving.
  A follow-up test also reproduced the missing phase in the public HTTP response
  model; its Literal list was aligned with the existing native/Gateway state.
  Lifecycle, guidance and inspection-route API regressions passed 149/149;
  Ruff passed for the two Python files. This is a response-contract fix, not a
  new native state or a second task state machine.

This slice is informed by the bounded local
[AAA task/journal study](../../research/game-ui/TASK_JOURNAL_STUDY_20261016.md)
and [MATRiX UI/runtime study](../../research/matrix/UI_RUNTIME_STUDY_20261016.md).
Commercial assets are reference-only; no texture, font, cooked widget or code is
copied into runtime content. REA could inspect the focused MATRiX ELF layout;
it could not parse the selected cooked Hogwarts `.uasset`. The latter's fallback
string evidence is not a reconstructed Blueprint tree.

An ordered event list should next adapt the existing Gateway task `timeline`,
`latest_event` and `delivery` fields; no new history service is required. The UE
list and evidence-detail bindings are not implemented yet. Slate must not
reconstruct them by remembering the sequence of its own polls.
Evidence for this slice:

- `build/unreal-tests/task-journal-red-20261016/report/index.json`: the first
  real Slate navigation test failed because the task-journal menu entry did not
  exist before the implementation.
- `build/unreal-tests/ubt-task-journal-green-20261016.log`: controlled Editor
  build Succeeded, 261.89 seconds.
- `build/unreal-tests/task-journal-green-20261016/report/index.json`: **83
  Success, 0 Failed, 0 NotRun, 0 SucceededWithWarnings**, runner exit 0.
  Native completion with 2/3 evidence is shown as review-required, stale reports
  hide previous acceptance, and opening/returning from the page does not change
  task truth or enter Drive.
- Independent scoped code review: APPROVE, no actionable finding. No new
  field/runtime control or third-party asset is introduced.
- The screenshot helper's existing path/dimension checks passed 3/3.

Actual native screenshots were then inspected at
`build/unreal-tests/task-journal-render-1080-20261016/`: `menu.png` and
`journal.png` are both 1920x1080, runner exit 0, no capture error. The menu shows
the six player-facing entries; the journal shows the truthful unbound-task
empty state with a reachable Back action. These are actual Slate captures over
the existing model-preview background, not AI images or a running task. A bound
report layout is covered by the interaction tests, not by this screenshot run.
Postflight found no UE/UBT/UAT/MuJoCo/dotnet/cl/link/MSBuild processes. None of
these component results is a new coupled motion qualification.

## Driving HUD v2 — four-corner redesign, 2026-10-15

The user's marked screenshot identifies four presentation problems: redundant
task branding, a debug-like connection card, a full-width keyboard ribbon and a
heavy speed/turn/recorder panel. This pass replaces their visual composition
without changing motion authority, sensor configuration or camera controls.

- Top left: a quiet outlined objective marker and one title. A bound task retains
  its real progress, blockers and stale warnings; no task is no longer described
  with an unbound-task diagnostic. An unbound presentation is explicitly named
  Model preview, not a working mission.
- Top right: an inline connection notice without a backplate or camera-echo
  diagnostic. Offline preview remains explicit; runtime failure, stale status,
  identity mismatch and safe-stop notices retain their authoritative meanings.
- Bottom right: freestanding speed digits and a separate m/s unit. Unknown data
  remains an em dash, not zero; stale/incoherent full status suppresses the value.
  Angular velocity and technical detail remain in the Tab view. Recording text
  is shown for active/saved/failed recording states rather than an idle diagnostic.
- Bottom left: two small outlined prompts instead of seven permanent commands.
  Hold F1 (or gamepad View) for the contextual guide; release to close. The guide
  uses the player's actual configured driving keys. Menu changes, focus loss,
  text editing and teardown clear it. This is UI-local state, never a control or
  lifecycle request. The guide does not pause physics or change mouse capture.
  Tab remains the reachable details action while the mouse belongs to camera look.

Reference method: reuse the existing local file-level studies in
`research/game-ui/README.md` and `HOGWARTS_ADDITIONAL.md` for component separation
and contextual key prompts. REA 6.4.0 is available at `D:/inovxio/tools/rea-cli/`;
this pass does not claim a new decompilation of an uninstalled title. Public
primary references are Naughty Dog's [HUD size/contrast and contextual indicator
options](https://www.naughtydog.com/blog/the_last_of_us_part_ii_accessibility_features_detailed)
and Sucker Punch's [world-first guidance design](https://blog.playstation.com/2020/05/29/ghost-of-tsushima-your-questions-answered/).
These motivate our own design; no commercial game texture, font or widget is
copied into runtime content.

The built-in image-generation tool produced
`assets/driving-hud-v2-concept.png` as a **visual concept**, not runtime evidence.
Prompt: preserve the existing robot and scene, replace only four HUD clusters,
use restrained white/gray, outlined objective and key prompts, an inline offline
notice and panel-free unknown speed, with no fabricated data or copied game logo.
Slate remains the source of the actual interface; the generated bitmap is not
used as a screenshot-shaped HUD asset.

Verification so far: controlled Editor build
`build/unreal-tests/ubt-driving-hud-v2-20261015.log` succeeded (341.15 seconds).
The affected UI Runtime/PlayerSettings/DriveKeyBindings Automation report at
`build/unreal-tests/driving-hud-v2-20261015/report/index.json` passed **76/76**,
zero warnings, failures or not-run cases, runner exit 0. Independent bounded
code review approved the change. The capture helper's three offline dimension
and path tests passed. Actual UE capture results are recorded after inspection.
The older 720p capture is not accepted: it was actually 1920x1009 and
showed the wrong tab. The revised harness verifies PNG dimensions, establishes
the requested mode after startup and records target commands honestly; semantic
page/layout approval still requires visual inspection.

Final native captures were inspected:

- `build/unreal-tests/driving-hud-v2-render-1080-20261015/drive.png` is actually
  **1920x1080**, shows the new four-corner composition, and the runner exited 0.
- `build/unreal-tests/driving-hud-v2-render-720-20261015/drive.png` and
  `controls.png` are both actually **1280x720**. The latter now shows Controls,
  not Graphics. Both target modes were visually checked; runner exit 0 and no
  capture error. The default driving HUD has no overlapping corner elements;
  small secondary type at 720p still motivates a future text-size/UI-scale option.
- F1/View hold/release and contextual guide visibility were covered by the
  Automation suite. No physical keyboard/gamepad walkthrough is claimed, and
  no real-RHI held-guide capture was produced in this batch.
- Both preview runs were unbound and did not start MuJoCo. They prove rendered
  presentation, not coupled movement, collisions or sensor qualification.
- Local capture checks live beside the ignored diagnostic harness at
  `build/unreal-tests/test_field_ui_capture.py`, rather than importing ignored
  build artifacts from the repository's normal tests/sim collection. Its three
  tests passed after relocation; disabling pytest's cache provider produced one
  cache_dir configuration warning, not a failed capture contract.
- Final process inspection found no UE, UBT, UAT, MuJoCo headless, dotnet,
  Blender, cl, link or MSBuild processes. Scoped diff whitespace checks passed.

## Configurable driving keys — 2026-10-15

Settings → Controls offers six single-key driving bindings: forward/backward,
left/right translation, and left/right turn. Defaults remain WASD/QE, without
Shift. Click an action, press a letter or top-row digit, then Save to retain the
configuration across launches. Changes apply immediately in memory. The driving
HUD displays the configured physical keys.

- Duplicate keys are rejected without stealing another action's binding.
- B/C/R/X and non-alphanumeric keys are unavailable; menu, camera and recording
  shortcuts remain fixed. This is not a modifier-chord or gamepad-rebind editor.
- Escape cancels only pending key capture. Repeats cannot bind. Capture runs
  before the regular action policy, preventing recording or mode shortcuts from
  firing while choosing a key.
- Changing category/page, application/focus loss or owner teardown cancels
  pending capture. Menu/focus loss and rebinding clear held driving state;
  repeat events do not restore motion without a fresh press.
- Controls defaults restore only sensitivity, inversion and driving bindings.
  Interface defaults do not change controls. Save failures remain visible.
- Bindings share the existing `LingTuSim.Player` INI section; old configurations
  use defaults. Incomplete/invalid binding sets fall back as a whole to avoid
  ambiguous mixed mappings. Input evidence names actual pressed physical keys;
  Control/MuJoCo authority and gamepad LB gating are unchanged.

Controlled build `build/unreal-tests/ubt-keybindings-20261015-r2.log` succeeded.
The affected Runtime UI, PlayerSettings and DriveKeyBindings Automation suite in
`build/unreal-tests/keybindings-20261015/report/index.json` passed **75/75**, with
zero warnings, failures or not-run cases and runner exit 0. It covers the real
Slate binding/save callbacks and input-processor route as well as isolated INI
round trips, corruption fallback, held-key clearing, physical-key evidence,
repeat/focus/menu behavior and existing mouse/gamepad regressions. Independent
code review approved the repaired Save-cancels-capture behavior. The unchanged
graphics-only tests were not rerun in this suite. These input/UI checks are not
coupled robot-motion or physical-device qualification.

## Categorized player settings — 2026-10-15

The next settings revision follows common game-menu organization rather than
adding technical runtime controls to the player menu:

- **Graphics:** display mode, supported resolution choices, frame cap, VSync,
  render percentage, a quality preset and separate view distance, texture,
  shadow, anti-aliasing and post-processing quality controls. Values are staged;
  Apply previews them, Keep confirms/saves, Revert or a fifteen-second deadline
  restores the preceding settings. Leaving the settings page also reverts an
  unconfirmed preview. Horizontal arrows/D-pad adjust the selected option.
- **Controls:** live mouse sensitivity/invert-Y, six configurable keyboard
  driving actions and the fixed mouse/gamepad mapping reference. Gamepad
  dead-zone configuration is still not connected.
- **Interface:** independently saved control-hint and client-frame-timing display.
  Frame timing describes the UE client, not physics RTF, sensor rate or GPU timing.
  Restore default affects the current category only.

Graphics settings are not sensor configuration. UE scalability is renderer-global
and can affect SceneCapture output, so graphics Apply is locked whenever Session
or Control is bound. It must not change MuJoCo or SensorPlan parameters. Saved
graphics preferences are read into the next settings draft, not auto-applied at
startup by this UI; the user must Apply the custom draft in an unbound view. This
UI avoids native `UGameUserSettings::ApplySettings/SaveSettings`, but UE's own
`UGameEngine::PreExit` still calls `SaveSettings` before world teardown. Therefore
this is NOT cross-launch graphics/sensor isolation: the engine may persist and
restore its native display/quality profile. Qualified simulation launches still
need an explicit fixed renderer profile and fresh performance/fidelity evidence.
The current guarantee is no graphics Apply while bound and no SensorPlan/MuJoCo
parameter change, not immunity of future sensor images to native user settings.

Remaining settings work, not represented as working switches:

1. Wider keyboard/chord support and gamepad remapping beyond the six driving keys.
2. Gamepad sensitivity, dead zones, inversion and per-device defaults.
3. Player-camera FOV and occlusion handling without modifying sensor cameras.
4. Text size, HUD scale/safe margins, language/localization and accessibility.
5. Audio routing/assets followed by master/effects/UI volume; the current
   `-NoSound` launch must not present fake audio controls.
6. HDR, upscaling and advanced renderer options only after support detection and
   the corresponding rendering path are actually integrated.

Build, runtime readback and rendered-page verification for this revision are
recorded after execution; this design section is not a completion claim.

Code verification for categorized settings:

- `build/unreal-tests/ubt-settings-categories-20261015-r2.log` succeeded. The
  initial build's only compile error was an attempt to modify the const core
  slider style; the implementation now makes a value copy before styling it.
- `build/unreal-tests/settings-categories-20261015/report/index.json` records
  **75 Success, zero warnings, zero failed and zero not-run**, runner exit 0.
  Coverage includes category navigation/default isolation, mouse and hint/frame
  preference persistence, graphics normalization/presets/native readback,
  preview-deadline policy, and the bound-session Apply gate. Existing driving,
  menu and mouse regression cases remain included.
- Bounded independent review approved the implemented scope, including its
  documented native UE persistence limitation. A native preview probe separately
  exercises actual frame-cap/shadow application and timeout restoration; its
  result is recorded only after that run completes.

Native capture exposed an additional UE sentinel case: resolution quality `0`
means project-default behavior, not a literal zero-percent image. The UI now
labels that choice as Project default. Preview normalization preserves the
sentinel; rollback restores both the stored quality value and the preceding
`r.ScreenPercentage` value. The native probe was extended to change render
percentage too and verify restoration of both values. Initial cap/shadow-only
probe logs remain retained and are not full render-percentage rollback evidence.

Final verification after the sentinel correction:

- `build/unreal-tests/ubt-settings-categories-20261015-r3.log` succeeded.
- `build/unreal-tests/settings-categories-graphics-20261015-r3/report/index.json`:
  **6 Success, zero warnings/failures/not-run**, runner exit 0. This focused
  rerun covers the five graphics cases and settings interaction; the earlier
  75-case input/UI report is separate evidence, not a new 75-case final rerun.
- The owned native probe in
  `build/unreal-tests/settings-categories-render-1080-20261015-r3/Unreal.log`
  recorded `START passed=true pending=true cap=45 shadow=1 scale=75
  screen_percentage=75`, then after timeout `REVERT passed=true pending=false
  cap=30 shadow=3 scale=0 screen_percentage=0`. The two zero scale values are
  project-default sentinels, not measured zero-resolution images. The probe
  verifies setting values and rollback, not achieved FPS or sensor performance.
- That run and `build/unreal-tests/settings-categories-render-720-20261015-r3/`
  both exited 0, each capturing actual `settings.png`, `controls.png` and
  `interface.png` without capture errors. Visual review found no clipped rows or
  overlapping controls at 1920×1080 and 1280×720. Secondary text remains small at
  720p; text-size/UI-scale controls are still needed.
- Previews were unbound: MuJoCo was not started. No physical gamepad/mouse
  walkthrough or exhaustive fullscreen/resolution combination test was claimed.
  Final process inspection found no UE, UBT, UAT, MuJoCo headless, dotnet, cl,
  link or MSBuild processes. Scoped `git diff --check` passed.

Remaining integration gaps also include a settings entry on the pre-session
front end (runtime currently owns these pages), and last-known-good display
recovery across abnormal exit or closing the OS window during preview. Normal
Back, timeout and owner teardown restore in-memory values, but native PreExit
can persist state before teardown; crash/force-close-safe persistence is not
guaranteed by this revision. Bound-session graphics remain locked until explicit
renderer-profile ownership supports permitted live adjustments.

## Player menu implementation — 2026-10-15

The driving menu now presents exactly five player entries: Continue game,
Restart, Settings, Return to main menu and Quit game. It uses the accepted sparse
left-aligned list, neutral selection/chevron and a dark-to-transparent backdrop;
the old right-hand session summary and technical action groups are removed.
Camera controls remain on the mouse/C key, diagnostics on Tab and recording on R.

Implemented behavior:

- Continue returns to Drive. Settings and confirmation pages remain in Menu;
  Escape/gamepad Back first returns to the menu root rather than resuming drive.
- Settings has a 0.25–3.0 mouse sensitivity slider, invert-Y and control-hint
  visibility. Changes apply to the local player view only. Save persists the
  preferences under `LingTuSim.Player` in UE's GameUserSettings ini; Restore
  default applies the original settings without changing robot or sensor data.
- Quit opens confirmation (including the X shortcut), then uses the existing
  runtime request and correlated STOPPED/safe-zero acknowledgement. It does not
  bypass the existing normal-exit policy. Unbound previews cannot terminate a
  simulation that is not attached. Pending state belongs to the world subsystem,
  not a widget message: exact current rejection or a ten-second confirmation
  timeout clears the pending event and exposes a retryable error. Closing and
  reopening the confirmation cannot send a duplicate while it is pending.
- Restart and Return to main menu have explicit unavailable explanations and
  disabled confirmation. They are NOT completed lifecycle features. Internal
  physics reset alone is insufficient: the control allocation/generation,
  external camera readiness and task lifecycle must be rebound together. Return
  to main menu also needs an owner-managed transition to the selector, not an
  alias for quitting.

This section supersedes the historical cool two-column menu layout below.
Build, Automation and render results for this implementation are recorded after
verification; implementation text alone is not live gameplay evidence.

The first Automation run is retained at
`build/unreal-tests/player-menu-20261015/report/index.json`: 66 passed and three
failed. Two failures required the intentionally removed camera-menu entry; those
tests now assert that camera control stays outside the player menu and Continue
preserves the view. The third exposed missing-file config persistence; Save now
creates the config branch before setting/flushing values and returns success to
the UI. Independent bounded review approved the exit-pending correction; its
rejected/missing/stale/mismatched reply paths have a dedicated test.

Final code verification:

- `build/unreal-tests/ubt-player-menu-20261015-r2.log`: `Result: Succeeded`.
- `build/unreal-tests/player-menu-20261015-r2/report/index.json`: **70 Success,
  zero warnings, zero failed and zero not-run**, runner exit code 0. This includes
  player menu interaction, the four settings tests (with an isolated on-disk ini
  round trip), exit retry policy and the existing mouse/input regression cases.
- No full scene restart or return-to-selector claim is implied by these tests.

Final presentation verification:

- Native capture first revealed that Slate's gradient orientation names describe
  the stops, not the fade direction. A paint-only correction switches to vertical
  stops for the intended left-to-right fade. No interaction code changed after
  the 70-case result. Final build
  `build/unreal-tests/ubt-player-menu-20261015-r3.log` succeeded.
- `build/unreal-tests/player-menu-render-1080-20261015-r3/` and
  `build/unreal-tests/player-menu-render-720-20261015-r3/` each contain actual UE
  `menu.png`, `settings.png`, `quit.png`, launch/log and result evidence. Both
  runners exited 0 with all three captures and no capture error. Visual review
  found intact text, controls and margins at 1920×1080 and 1280×720. Small
  secondary text and the low-contrast sensitivity track remain polish candidates.
- These are explicitly unbound presentation previews, not live MuJoCo driving,
  physical mouse/controller walkthroughs, or a new end-to-end exit qualification.
  All previews exited naturally. Final process inspection found no UnrealEditor,
  UBT, UAT, MuJoCo headless, dotnet, cl, link or MSBuild processes.

## Approved player-menu functions — 2026-10-15

The user accepted the generated left-aligned, restrained black/white menu
direction but rejected its technical action list. The player-facing menu contract
is now: **Continue game / Restart / Settings / Return to main menu / Quit game**.
This supersedes the earlier camera/info/preview/record menu as a product design,
not as a claim that all five actions are implemented.

Restart means resetting the current scene, robot and task to their initial state,
with confirmation and runtime-owned reset acknowledgement. It is not just moving
the UE visual Actor. Return to main menu means ending the current simulation and
returning to robot/scene/task selection; it is distinct from quitting the process.
Those lifecycle paths and the settings page still require implementation. Do not
silently rename an existing exit action to imply a working reset or main menu.

Camera mode switching belongs in driving input (C and middle mouse), not the
player menu. Detailed runtime information remains an optional Tab view. Asset
and scenario editing belong in the front end/editor, not the driving menu.

`references/menu-gameplay-study-20261015.png` is the generated visual study made
from an earlier actual FactoryPark frame. Its old six action labels are superseded
by this contract. The bitmap is design-reference-only, not a runtime background,
interactive widget, current screenshot or motion evidence.

## Menu colour refinement — 2026-10-15

The runtime menu now uses a cool charcoal/blue-grey background, neutral near-white
body text and muted secondary labels. The former brass decoration and brown
pressed-button colour are removed from this field interface. A small ice-blue
accent is reserved for the driving marker and keyboard selection; status-specific
warning/error text remains distinct from decorative emphasis.

All six menu actions share the same row treatment. Keyboard focus has a visible
two-pixel marker and a quiet filled row, hover has a weaker background and press
has a separate darker-blue treatment. The row observes its button through weak
references, rather than creating a button/content reference cycle. Existing action
delegates and availability rules are unchanged.

The camera action uses the same concise actual-view wording as Drive. The footer
now states plainly that the menu does not automatically pause simulation and
that recording/exit require runtime acknowledgement. Detailed state stays in
Tab/Menu; no additional driving panels, dependencies or image assets were added.

Verification:

- Controlled build `build/unreal-tests/ubt-cool-menu-20261015.log` succeeded.
- `build/unreal-tests/cool-menu-20261015/report/index.json`: **62 Success,
  0 warnings, 0 failed, 0 not-run**, runner exit code 0. The navigation test now
  also checks that the visible cool-colour marker moves from the old focused
  row to the new one. Camera dispatch and unavailable-action tests still pass.
- Actual UE Drive/Tactical/Menu captures are in
  `build/unreal-tests/cool-menu-render-1080-20261015/` and
  `build/unreal-tests/cool-menu-render-720-20261015/`. Both runners exited with
  code 0, captured all three screens and recorded no capture error. The menu
  PNG dimensions were read back as 1920×1080 and 1280×720 respectively. Visual
  inspection found no clipped/overlapping menu rows or columns at either size.
  Secondary text is small at 720p; a user text-size option remains future work.

Both runs are **unbound presentation previews**, not coupled MuJoCo motion or
physical keyboard/gamepad qualification. No robot/map assets were edited.
Owned previews exited normally, and the final process scan found no UE, UBT,
UAT, MuJoCo headless, dotnet, cl, link or MSBuild processes.

## Compact driving refinement — 2026-10-14

This pass keeps the existing Slate implementation, copied status snapshot and
action delegates. It does not change input ownership, motion limits, physics,
packages or map assets.

User-selected direction: simple game HUD, fewer words, thin separators and a
clear central view of the scene. Do not expand the driving overlay into an
industrial dashboard; detailed information belongs in Tab/Menu.

- Drive now shows a small task cluster, a concise simulation/camera status,
  horizontal observed speed, signed observed yaw rate and compact key hints.
  Binding IDs, truth sequences and detailed diagnostics remain in Tab/Menu,
  rather than occupying the normal driving view.
- Speed excludes vertical suspension motion. Turn rate converts the observed
  angular-Z velocity from radians/second to signed degrees/second. Missing or
  incoherent measurements remain unavailable, not measured zero.
- The healthy link label requires current coherent RUNNING status, applied
  truth, all four expected facets explicitly required/active/blocker-free and
  active blocker-free reported sensors; merely
  binding session/control is not enough. Failed, stale and blocked states keep
  explicit labels. This is display logic, not permission to move.
- The runtime menu groups driving and recording actions and uses a DownOnly
  ScaleToFit container. Existing disabled-action rules and focus/action
  delegates remain in place. The menu does not pause physics.

Controlled build: `build/unreal-tests/ubt-compact-driving-hud-20261014.log`,
`Result: Succeeded`. The actual `LingTuSim.UI.Runtime` Automation report at
`build/unreal-tests/compact-driving-hud-20261014/report/index.json` records
**62 Success, 0 warnings, 0 failed and 0 not-run**. The new tests cover horizontal
speed, signed turn and link-readiness wording; existing navigation/action tests
also pass.

Actual UE captures are in
`build/unreal-tests/compact-driving-render-20261014/`: `drive.png`,
`tactical.png` and `menu.png`, each at 1920×1080. The runner exited with code 0;
`result.json` records all three captures and no error. Visual inspection found
readable labels, intact safe margins and no overlapping menu columns at this
resolution. Smaller-screen scaling is implemented but not yet render-verified.

An independent review identified missing blocker/explicit-required checks in
the new healthy-link display. Those checks and regression cases were added
after the captures; no layout or unbound-preview text changed. The first 62/62
report above predates that final correction; final build/test results are
recorded separately below.

Final correction verification: controlled build
`build/unreal-tests/ubt-compact-driving-hud-readiness-20261014.log` succeeded;
`build/unreal-tests/compact-driving-hud-readiness-20261014/report/index.json`
records **62 Success, 0 warnings, 0 failed and 0 not-run**, runner exit code 0.
Independent bounded re-review returned **APPROVE** after the concrete blocker
checks and four regression assertions were added. No additional layout capture
was needed because the corrected predicate does not change unbound output.

These are **unbound UI presentation captures**, with MuJoCo not started. They
do not re-qualify movement, collision, physical keyboard/gamepad input or
recording acknowledgements. The preview UE process exited naturally. Other
repository work had MSBuild/cl processes during postflight; they were not owned
or terminated by this UI task. Do not claim machine-wide process cleanup.

Login, robot/map selection, settings and content-management pages remain outside
this pass. Lighting, mesh quality and physically coupled UI feedback need their
own verification; a quieter overlay does not fix those separate problems.

## Full interface reconstruction — 2026-10-14

The new implementation is `SLingTuSimFieldInterface.cpp`, sharing the existing
HUD's read-only status snapshot, UI mode controller, and authoritative action
delegates. It is an original Slate implementation informed by local UI structure
studies, not a commercial texture/font transplant.

Three runtime surfaces are implemented:

- **Drive:** quiet objective/status clusters, observed speed and simulation time,
  compact independent keycaps. No invented battery, minimap, or task progress.
- **Tactical / 运行信息:** full side panels separating requested input, admitted
  control, and observed motion; the world remains visible in the center.
- **Field menu / 现场菜单:** actual focused buttons for return, information,
  read-only scene preview, recording, and ending a session. Unbound recording
  and exit actions are disabled. This no longer opens the front-end asset/session
  workspace. A menu is not evidence that physics has paused or motion has stopped.

Direction keys and D-pad keys in the runtime menu pass to Slate navigation;
the separate front-end selector retains its existing session/asset actions.
Speed, simulation time, and truth identifiers require coherent identity; old
generation values must not be presented as current measurements.

### Actual UE capture evidence

`build/unreal-tests/field-interface-render-20261014-r2/` contains actual
1920×1080 `Shot showui` Drive, Tactical, and menu images, launch arguments,
Unreal log, and `result.json`. UE exited normally with code 0; all three captures
completed. The menu's focused return button, disabled unbound actions, readable
CJK labels, panel separation, and missing-value placeholders were inspected.

These images are **unbound presentation previews** in the existing
`ThunderV4_RuntimePreview` map. They do not prove current model quality,
robot locomotion, collision, recording, or coupled simulation readiness.
The development-only preview command requires an explicit render flag and
rejects any bound session/control transport; it is absent in Shipping builds.

The first capture attempt remains preserved in
`field-interface-render-20261014/`: its script used the unavailable
`PlayerController.get_pawn()` Python API, so no screenshots completed. Only the
owned preview received a normal close-window request, and UE exited with code 0;
the capture runner correctly returned failure. The corrected attempt uses
`GameplayStatics.get_player_pawn()`.

The first Automation report remains preserved in
`field-interface-20261014/report/index.json`: **52 success / 2 fail**, not a pass.
Both failing fixture checks read registered Slate attributes before prepass.
The fixture now evaluates prepass, and the runner checks report failures rather
than relying on Unreal's exit code. Independent review also found and fixed
identity-gate and runtime-navigation defects.

Final controlled build succeeded:
`build/unreal-tests/ubt-field-interface-20261014-r5.log`.
Final `LingTuSim.UI.Runtime` Automation report in
`field-interface-20261014-r2/report/index.json` records **56 success, 0 warnings,
0 failed, 0 not-run**, including six new interface contracts. The postprocessor
initially rejected Unreal's UTF-8 BOM even though the tests had completed;
this runner failure is preserved. Its reader now uses `utf-8-sig`, and the
offline `--verify-report` check returned code 0 against that same unchanged
report. No extra UE run was needed to validate the corrected report reader.

Postflight process inspection found no UnrealEditor/UnrealEditor-Cmd,
UnrealBuildTool, MuJoCo headless, dotnet, Blender, cl, link, or MSBuild processes.
The UI tests validate focus-path navigation pass-through and button dispatch;
they do not constitute physical keyboard/gamepad or coupled recording/exit
qualification. The screenshots were produced before the final additional
Tactical identity check; that check does not change their unbound presentation.

### Remaining interface work

The front-end login, robot selection, map selection, settings, sensor setup, and
asset management surfaces have not been redesigned in this batch. They should
reuse this visual language but remain separate from the runtime menu. Physical
movement and record/exit acknowledgements need live coupled validation; UI
screenshots and button dispatch tests cannot replace that evidence.

## Isolated robot library — 2026-10-16

The subsequent category/search refinement is recorded in
[`robot-library-categories.md`](robot-library-categories.md), including current
29-model category counts, filter/focus behavior, and newer real UE screenshots.
It supersedes the flat model-list layout below without changing session authority.

The robot/map picker now gives the robot body the main presentation area. The
robot step has a narrow, independently scrolling model list and a larger
transparent-image stage, without a plinth, ground, screenshot frame or hover
backplate. The map and final review remain separate steps. The body preview is
an offline render of the actual geometry, not an interactive 3D orbit view or
evidence of a running controller.

`sim/sessions/robot-library.v1.json` declares 27 imported body summaries:
22 local RobotLab families, the public ANYmal D simple description, Menagerie
H1, Panda, UR5e and Spot. Together with Thunder and OmniCart this is 29 choices.
The catalog compiler's optional `--robot-library` input places them in a
separate `robot_library` array. It never creates session entries or world
bindings for those bodies. Exact runnable SessionBundles remain authoritative.
The UI can select a source model, but cannot confirm a launch until a real
robot/world/controller configuration exists. A source-library duplicate does
not override metadata belonging to an existing session.

Current catalog generation, from the repository root:

```powershell
.venv/Scripts/python.exe -m sim.tools.game_selection_catalog `
  --repo-root D:/inovxio/brain/lingtu `
  --spec sim/sessions/game-selection.v1.yaml `
  --robot-library sim/sessions/robot-library.v1.json `
  --output-root build/robot-library-selection-render-20261016 `
  --asset-review-catalog build/game-selection/asset-review-v1/game-asset-review.catalog.json
```

Controlled UE build succeeded in
`build/unreal-tests/ubt-robot-library-isolated-20261016.log`. The UI Automation
report at `build/unreal-tests/robot-library-isolated-20261016/report/index.json`
records **89 successes, 0 warnings, 0 failures, 0 not-run**. The catalog/launcher
Python suite records **57 passed**. These prove the selection boundary, not
locomotion or field behavior. Source-acquisition scope is recorded in
[`robot-library-import.md`](robot-library-import.md) and
[`robot-library-external.md`](robot-library-external.md).

Real 1920x1080 selector capture completed with exit 0 and no selection intent:
`build/unreal-tests/robot-map-20261016-203812/result.json`. Robot, map and final
review images are in
`sim/runtime/visual/RobotSimUE/Saved/Automation/LingTuSimUI/FrontEnd/robot-map-20261016-203812/`.
No physics process ran for this capture. There are now 23 installed isolated
robot PNGs; the six remaining DAE models display the explicit missing-preview
state rather than another model's image.

## Prior compact overlay pass — 2026-10-13

The 2026-10-13 pass changes only the Drive overlay's upper-left task cluster
and upper-right runtime cluster. Menu, login, Tactical, input ownership and
physics authority are unchanged.

## Visual reference

Reference: **Tom Clancy's Ghost Recon Breakpoint**. Its objective hierarchy,
small mission markers and low-screen-coverage HUD are a useful direction for
robot field operation. Ubisoft documents optional HUD elements, per-element
size and readability backgrounds in its
[accessibility overview](https://news.ubisoft.com/en-us/article/Y5gZoUaHfDiPoR6o4mtEQ/ghost-recon-breakpoint-to-launch-with-extensive-accessibility-features).
A publicly shared gameplay screenshot was inspected through the
[Steam screenshot gallery](https://steamcommunity.com/app/2231380/screenshots/).

This is a visual-layout study, **not** binary UI-asset extraction. No Ubisoft
texture, icon atlas or font is shipped in RobotSimUE. The overlay uses existing
Unreal fonts and independent Slate text/geometry; no new dependency is required.

## Applied treatment

- Remove both large tinted rectangular backplates and English section headers.
- Anchor the clusters 40 pixels from the sides and approximately 36 pixels
  from the top; retain a 320-pixel task width and 340-pixel status width.
- Use warm white text with dark one-pixel shadows. Amber marks unverified
  or unavailable information; missing evidence is never styled as success.
- Keep the task title, task progress, session/control bindings, truth sequence,
  recording state and explicit unverified status. Hide only the no-task
  explanatory footer; bound report/warning text remains visible.
- Leave detailed diagnostics in the existing Tactical view. Do not intercept
  pointer input or modify movement keys as part of the style pass.

Validation: controlled UE 5.8.1 Editor build succeeded; existing
`LingTuSim.UI.Runtime` Automation passed 50/50 (0 warnings, failed or not-run).
The first coupled preview failed on the existing external-evidence sharing-lock
retry path, not on HUD construction. It is not runtime qualification.

A real 1920x1080 `Shot showui` image was then captured in the UE-only
`ThunderV4_RuntimePreview` map and visually inspected. This static preview has
no MuJoCo session and shows unavailable/unverified bindings as such. It is not
a current robot-asset or motion qualification. Its capture watcher expected
`hud-light.png`, while Unreal wrote `hud-light00000.png`: the runner returned
failure even though the screenshot was produced. Both that failure and the
independent capture inspection are preserved in
`build/unreal-tests/hud-light-camera-render-20261013/`.

## User-selected installed references

The user subsequently selected the installed **Hogwarts Legacy** and **Palworld**
as preferred references. Their real archive entries have now been inspected;
six small Hogwarts mission textures were decoded for local study. Evidence and
the distinction between indexed names, extracted bytes and decoded textures are
recorded in [`research/game-ui/README.md`](../../research/game-ui/README.md).

For subsequent refinement, prefer Hogwarts' task/title/marker composition and
Palworld's compact key-guide components. Keep the robot HUD's restrained field
style rather than copying parchment or fantasy crests. This research update
does not itself change the current Slate widget or invalidate its recorded build.
