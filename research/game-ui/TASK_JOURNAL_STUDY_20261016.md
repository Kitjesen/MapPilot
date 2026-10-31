# Task journal, result, and controller-focus study

Date: 2026-10-16  
Scope: bounded research for a future asset-free Slate task journal in RobotSimUE. No production code or content was changed. No game, UE, UBT, MuJoCo, or field runtime was started. No archive was decrypted, no key was supplied, and no copyrighted game asset is proposed for migration.

## Decision

Build the future experience as an original, technical Slate journal driven by LingTu's verified inspection projection. Borrow only interaction-level ideas from the installed-game evidence: a persistent category entry, a compact tracked-objective summary, a transient result notice that leads into durable history, and clearly visible focus states. Do not reproduce either game's art, fonts, textures, sounds, layout, or cooked widgets.

The strongest product rule is that four outcomes remain visibly separate:

1. the operator's command was admitted;
2. native execution advanced or terminated;
3. required evidence was verified;
4. the result was accepted, rejected, or sent for review.

An attractive completion banner must never collapse those four facts into one local `success` state.

## Sources and claim boundary

This pass read:

- `research/game-ui/README.md` and `research/game-ui/HOGWARTS_ADDITIONAL.md`;
- Hogwarts `pakchunk0-iostore.csv`, mission-cooked evidence, and the already extracted Field Guide index packages under `build/game-ui-research-20261013/hogwarts/`;
- Palworld archive/path evidence under `build/game-ui-research-20261013/palworld/`, including its quest display, tracking-canvas, key-guide, and input-icon path families;
- current LingTu product, native inspection, UE context, `FInspectionProjection`, runtime HUD, input processor, and controller-navigation tests.

The commercial-game evidence is file/path/string evidence. Asset names and printable strings are not Blueprint logic. In particular, they do not establish exact hierarchy, controller navigation, focus restoration, runtime state transitions, or animation timing.

## New bounded REA result

One already extracted cooked file was selected because it is narrowly relevant to menu navigation and journal entry affordance:

`UI_BP_FieldGuideButton_Quests.uasset` (111,221 bytes).

REA 6.4.0 was invoked twice against only this file:

```powershell
& 'D:\inovxio\tools\rea-cli\rea.ps1' inspect-artifact `
  'D:\inovxio\brain\lingtu\build\game-ui-research-20261013\hogwarts\additional\cooked\Phoenix\Content\UI\Menus\FieldGuide\Index\UI_BP_FieldGuideButton_Quests.uasset' `
  --json

& 'D:\inovxio\tools\rea-cli\rea.ps1' search `
  'D:\inovxio\brain\lingtu\build\game-ui-research-20261013\hogwarts\additional\cooked\Phoenix\Content\UI\Menus\FieldGuide\Index\UI_BP_FieldGuideButton_Quests.uasset' `
  'Quest' --kind strings --mode literal --json
```

Both returned `target_unavailable`, reason `unsupported binary format`, with exit code 1. REA therefore supplied no package parse and no string search. The exact JSON results are retained in `build/research/game-ui-journal-20261016/rea-single-file-result.txt`.

Because installing an analysis engine or opening a game executable would exceed this task, the same file received one transparent read-only fallback using the existing LLVM `strings.exe`. Selected names include `OnHovered`, `OnUnhovered`, `UI_Menu_Hover`, `UI_Menu_Select`, `CategoryLabelText`, `newIndicator`, `SetLockedState`, `GatherMenuReaderStrings`, `CursorScaleDown`, `CursorScaleUp`, and clicked/hovered delegate signatures. The exact command and output are retained in `build/research/game-ui-journal-20261016/single-file-visible-strings.txt`.

This supports only a package-composition hypothesis: the cooked quest-category button contains names for icon/label/new/locked/tutorial treatments, hover/select feedback, and animation resources. It does **not** prove focus ownership, D-pad behavior, controller mappings, widget hierarchy, or logic.

## Native facts available to a Slate projection

Current LingTu code already exposes a useful bounded model:

| UI question | Existing projection fact | Authority rule |
| --- | --- | --- |
| Which task is this? | `TaskId`, `RouteId`, `RouteRevision`, `MapId`, `MapContentEpoch`, `ProductSessionId` | Display only when immutable binding is verified. |
| What is happening now? | `State`, `Phase`, `Reason`, `CurrentPointNumber`, `CurrentPointId`, `Action` | Native task event or persisted native task event is the source. |
| How far has it progressed? | `CompletedPoints`, `PointCount`, `bProgressVerified` | Suppress numeric progress when execution, history continuity, or launcher binding is unverified. |
| Did execution finish? | `bTerminal`, `bExecutionConfirmed`, terminal `State`/`Phase` | Never infer from command ACK, overlap, animation, or HUD state. |
| Is the result usable? | `ReportStatus`, `Acceptance`, `RequiredEvidence`, `VerifiedEvidence`, `bReportVerified`, `bReportStale` | Execution success and evidence acceptance remain separate. |
| May the operator act? | `CanRequestControl()` plus `bCanPause`, `bCanResume`, `bCanCancel`, `bCommandPending` | Slate sends intent; it does not manufacture state. |

One important UE gap remains: the current `FInspectionProjection` is a present-state/read-model summary, not an ordered journal. The existing `InspectionTaskStatusResponse` already exposes `timeline`, `latest_event`, and `delivery`; the next implementation should first adapt that native-event history, preserving its continuity and sequence identity. A new Gateway/native history service is not a prerequisite. Slate must not synthesize old journal entries from changes it happened to observe while open. Evidence-detail browsing may need separate existing evidence API bindings.

## Original patterns for LingTu

### 1. Verified objective rail

Map the current native state to a compact, always-visible objective rail rather than a fantasy quest card.

- Header: route label plus immutable route revision; task ID is available in a detail disclosure rather than dominating the HUD.
- Primary line: current point number/ID and action.
- Progress: `CompletedPoints / PointCount` only while `bProgressVerified` is true.
- State chip: planning, navigating, settling, action pending, pausing, paused, cancelling, recovering, or terminal state; preserve these meanings instead of flattening them to “active”.
- Blocked presentation: stale, identity mismatch, incomplete history, and unavailable task each get a truthful non-progress message.

Acceptance:

- A command ACK alone cannot advance the rail.
- A missing launcher binding or event-history gap hides numeric progress.
- A skipped point must not use the same completed treatment when the future event history exposes skip outcomes.
- Long point IDs and localized actions remain readable at 1280x720 without covering the center navigation view.

### 2. Two-rail task journal

Use a focusable event list on the left and a detail/evidence pane on the right. Each row represents one authoritative native task event, not a frame-by-frame HUD observation.

- Row identity: native boot/event sequence plus task/route identity, retained internally for stable selection.
- Row content: timestamp, point, phase transition, outcome, and reason.
- Detail pane: immutable map/route binding, action request/evidence handle when present, and recovery explanation.
- Continuity break: insert a non-success “history unavailable” barrier and stop deriving cumulative progress beyond it.
- Restart: show `restart_resume_required` as an explicit journal fact; do not pretend motion resumed.

Acceptance:

- Reopening the journal yields the same order from the authoritative history source.
- Duplicate delivery does not create duplicate rows; a sequence gap is visible rather than silently bridged.
- Selection survives harmless list refresh by stable event identity.
- Slate never labels a row “completed” from an asset name, local animation, or current point change alone.

### 3. Split result gate

Present terminal execution and evidence acceptance as two adjacent but independent panels.

- Execution panel: terminal native state, point coverage, terminal reason, and history continuity.
- Evidence panel: verified/required count, report freshness, and `PENDING`, `ACCEPTABLE`, `REVIEW_REQUIRED`, `NOT_ACCEPTABLE`, or `UNKNOWN`.
- Primary result headline is conjunctive: for example, “Route finished · review required,” never merely “Mission complete.”
- Reinspection is a proposed operator choice only when an owning product/API supports it; the first Slate slice should remain read-only except for existing pause/resume/cancel intents.

Acceptance:

- Native `SUCCESS` with 2/3 verified evidence renders execution success and review-required separately.
- Stale or identity-mismatched reports cannot display an accepted result.
- `FAILED` and `CANCELLED` preserve their native meaning even if partial evidence exists.
- A result animation may acknowledge a verified transition but cannot become the source of result truth.

### 4. Receipt-to-journal handoff

Use a restrained transient notice for important transitions, then anchor the same fact in the journal. This adapts the separation seen in the Hogwarts notification/mission packages without copying their ornament or animation.

- Notice content: short state change, point label, and a “View journal” action.
- Notice source: a newly received, continuity-valid native event or a newly verified report transition.
- Notice lifetime: cosmetic only; dismissing it never dismisses the underlying event.
- Burst behavior: coalesce presentation noise, but never coalesce distinct terminal, skip, evidence-failure, or recovery facts in the journal.

Acceptance:

- Opening the notice focuses the exact matching journal row.
- If the journal source is unavailable, the notice says history is unavailable and does not invent a row.
- Replayed terminal events after restart do not repeatedly animate as new unless their native identity is genuinely new.

### 5. Controller-safe focus contract

Extend the current pure-Slate input seam with explicit journal focus zones, preserving the existing robot-control boundary.

- Pause/menu mode only: D-pad up/down traverses journal rows; left/right moves between journal and detail/actions; A activates the focused control; B returns one level and restores the prior focus.
- Drive mode: do not steal LB or movement axes; current code reserves LB as gamepad deadman outside the front end.
- On open: focus the last selected valid row, otherwise the newest valid row, otherwise a visible close/back control.
- On row removal or filter change: move focus deterministically to the nearest surviving row and announce the new selection.
- Visual focus: use an original high-contrast Slate outline and position marker that is distinct from hover and from selected/current-task state.
- Accessibility: expose concise row labels including state, point, and evidence status; a “new” badge is not the only indicator.

Acceptance:

- Every visible interactive element is reachable without a mouse, and focus never lands on collapsed/disabled content.
- A and Enter activate only the focused Slate control; B/Escape share back semantics.
- Switching mouse/controller input does not erase keyboard/gamepad focus indication until focus actually changes.
- Losing application/viewport focus cannot leave robot drive input admitted; existing release behavior remains intact.
- Opening and closing the journal three times restores focus and input mode without duplicate handlers.

## Recommended first slice

Implement none of this in the current research task. For the later production slice, the smallest honest order is:

1. Add a read-only Slate objective rail using only fields already present in `FInspectionProjection`.
2. Add the split result gate and tests for `SUCCESS + REVIEW_REQUIRED`, stale report, and identity mismatch.
3. Adapt the existing Gateway task `timeline` and `delivery` contract into a bounded UE list before adding historical rows; do not create a parallel history service.
4. Add focus-zone navigation and automation tests only after the list has stable event identities.

This order delivers useful operator clarity without inventing history or expanding UE into a second inspection authority.

## Out of scope

- Production edits, new UE assets, UMG/CommonUI adoption, UE/UBT builds, game launches, MuJoCo runs, or field claims.
- Archive extraction beyond the existing evidence, DRM/encryption bypass, key discovery, or executable analysis.
- Copying commercial fonts, textures, audio, widget packages, art direction, or exact layouts.
- Inferring Blueprint graphs, controller behavior, focus rules, or runtime semantics from filenames/strings.
- A UE-owned task state machine, UE-owned evidence acceptance, or motion control outside the declared native seams.
