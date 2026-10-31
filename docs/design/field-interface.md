# RobotSimUE field interface

Date: 2026-10-14. Status: implementation brief for the current Slate runtime UI.
This batch covers only **Drive HUD**, **Tactical view**, and **Field menu**. Robot
selection, map selection, login, asset review, and Build preview are not redesigned
here and are explicitly not complete.

## Direction: Field Codex

The interface should feel like a field instrument with the layered hierarchy of a
well-kept expedition codex: dark mineral glass, warm readable type, thin etched
rules, and one brass focus mark. It borrows component ideas from the installed-game
study—layered mission information, separate state plates, compact key legends, and
focusable menu entries—but no commercial artwork, font, texture, layout, or cooked
asset enters RobotSimUE.

The memorable device is a **mission spine**: a vertical line and diamond nodes that
compresses to one node in Drive, expands into the verified task/status chain in
Tactical, and becomes the focus rail in the Field menu. A node is lit only when its
underlying state exists and qualifies; it is never decorative evidence.

Purpose: keep Drive quiet, make Tactical the truthful diagnostic surface, and make
the menu a small set of real runtime actions. The world remains the visual center in
Drive and Tactical. The menu is full-screen but must say `MOTION RELEASED`; it must
not say the simulation is paused because opening the menu does not pause MuJoCo.

## Shared design tokens

Use these as semantic tokens rather than scattering literal colors and sizes.

| Token | Value | Use |
| --- | --- | --- |
| `surface.void` | `#06100F` at 96% | Field menu background |
| `surface.glass` | `#0A1716` at 78% | Readability plate over the world |
| `surface.raised` | `#102220` at 92% | Tactical panels and modal |
| `line.quiet` | `#52615B` at 55% | Dividers and inactive spine |
| `text.primary` | `#F2EFE6` | Titles and current values |
| `text.secondary` | `#B8B9B0` | Labels and explanation |
| `accent.brass` | `#CDB47A` | Focus rail and section identity, not readiness |
| `state.ready` | `#59BFA8` | Verified/current/available only |
| `state.caution` | `#E2A84F` | Stale, pending, unverified, or unavailable |
| `state.danger` | `#DD6557` | Failed, rejected, cancelled, blocked |
| `focus.ring` | 2 Slate units `accent.brass` plus a left diamond | Keyboard/gamepad focus; never color alone |
| `safe.gutter` | 40 at 1920x1080; minimum 24 | Critical-content inset from every edge |
| `space.1/2/3/4/5` | 4 / 8 / 12 / 20 / 32 | One spacing rhythm |
| `radius.panel` | 2 | Nearly square instrument plates, not generic rounded cards |

Typography uses a CJK-capable project-bundled sans when one exists. The repository
currently has no bundled UI font, so this batch should retain Slate `CoreStyle`
rather than introduce a package or silently depend on an OS font. A separately
licensed CJK face can replace the token later without changing layout.

| Type token | 1080p size | Treatment |
| --- | ---: | --- |
| `type.display` | 28 | Bold, menu title only |
| `type.section` | 16 | Bold, short section names |
| `type.body` | 12 | Regular, primary status copy |
| `type.data` | 11 | Tabular/monospace when an existing Slate face supports it |
| `type.label` | 9 | Bold, uppercase English wire terms or short Chinese labels |
| `type.hint` | 9 | Regular, key guide and evidence notes |

Do not force uppercase on Chinese. Keep wire values such as `ACTIVE`, `REJECTED`,
and `STALE` unchanged where they help compare UI with logs; place a readable Chinese
label before them.

## Layout and scaling contract

- Reference canvas is 1920x1080. Build with `SOverlay` edge alignment, fill slots,
  and vertical/horizontal containers; fixed values are width caps, not absolute
  screen coordinates.
- Apply `safe.gutter` to one root safe-area container. At 1280x720 use 24-unit
  gutters and reduce panel widths before reducing body type. Critical type stays at
  least 10 Slate units after application scale.
- On 21:9, keep corner HUD elements pinned to the safe edges and let the clear world
  window expand. Do not stretch panels across the additional width.
- Tactical uses three columns at 1600 Slate units or wider. From 1280–1599, side
  columns cap at 320 and the center flexes. Below 1280, stack the two detail columns
  along the bottom half and keep the task header visible; do not crop the right panel.
- Drive and Tactical remain `HitTestInvisible`. Only the top Field-menu screen owns
  pointer and focus input.
- Transition Drive ↔ Tactical with a 140 ms opacity change and a 12-unit outward
  panel reveal. Menu uses a 160 ms dim fade. If motion reduction is later exposed,
  use opacity only. No animation framework or continuous ornamental motion is needed.

## 1. Drive HUD

Drive is a transparent full-screen layer with three quiet clusters and no central
card. It must leave at least the central 60% of the world unobstructed.

### Layout

```text
┌ safe ───────────────────────────────────────────────────────────────┐
│ ◇ Task / route / verified progress       Session · Control · ●     │
│   report state                            Truth · Recording          │
│                                                                    │
│                        WORLD VIEW                                  │
│                                                                    │
│        movement guide     C Camera · R Record · Tab Tactical · Esc │
└────────────────────────────────────────────────────────────────────┘
```

- **Upper left, max 420 wide:** one spine node, task title, one progress/state line,
  and one report line. Use no backplate in normal state; add a narrow glass plate only
  when the scene does not provide enough contrast.
- **Upper right, max 400 wide:** right-aligned Session/Control line, then Truth and
  Recording. The status symbol includes both shape and text (`● CURRENT`,
  `◇ UNVERIFIED`, `! BLOCKED`).
- **Bottom center:** compact key-and-text legend. Show only bindings that work in the
  current mode. Movement keys are grouped separately from runtime actions.

### Drive state rules

- A fresh, launcher-verified inspection task may show route ID, native task state,
  `CompletedPoints / PointCount`, current point, and separately verified report state.
- If no inspection task is bound, show `未绑定巡检任务`; a confirmed game-session
  title may appear as secondary context labeled `场景`, never as the mission name.
- For identity blocker, stale projection, missing task, unverified progress, report
  stale, or unverified report, use the exact existing explanation from
  `GetInspectionProgressText()` / `GetInspectionReportText()` and a caution or danger
  node. Do not collapse these into a green generic “ready”.
- Truth sequence appears only when `bLatestAppliedTruthAvailable`. Recording appears
  as trusted only when full status is available, fresh, and identity coherent.
- `R` is labeled `开始记录`, `停止并提交`, or `记录不可用` from the two existing
  recording gates. A disabled action is muted and includes the reason; it is not
  presented as a working button.
- Never show invented mission distance, ETA, battery, signal strength, health,
  readiness percentage, waypoint bearing, or robot status bars.

## 2. Tactical view

Tactical is the expanded full-screen instrument view, not a fake map. The live world
remains visible through the center. Until a real map/render binding exists, the center
must not draw a minimap, route polyline, compass, range rings, or fabricated markers.

### Layout

```text
┌ LINGTU / TACTICAL ─ run · session · model · reset ─ CURRENT/STALE ┐
│ Session ─ Physics ─ Control ─ Visual ─ Sensors                    │
├──────────────────┬──────────────────────────┬─────────────────────┤
│ CONTROL INTENT   │       WORLD VIEW         │ OBSERVED TRUTH      │
│ owner / deadman  │  task spine + warnings  │ observed velocity   │
│ requested axes   │  no inferred map data   │ truth frame/base    │
│ ACK / reason     │                          │ sensor sample counts│
│ admitted twist   │                          │ recording / camera  │
├──────────────────┴──────────────────────────┴─────────────────────┤
│ C Camera · R Record · Tab Return to Drive · Esc Field Menu       │
└───────────────────────────────────────────────────────────────────┘
```

The top readiness chain is ordered to match authority flow. Each stage contains its
name and state; color is supplemental. The first stale, unavailable, failed, or
identity-incoherent stage breaks the line visually and produces a plain-language
ribbon immediately below the header.

The left panel deliberately preserves three separate facts:

1. **Requested:** control owner, deadman, sample age, focus, and requested axes.
2. **Admitted:** control ACK/status sequence/reason and admitted twist.
3. **Observed:** never duplicated into the left panel; it stays in the right panel.

This separation is more important than visual symmetry. An em dash means a field is
unavailable; `0.00` is shown only when zero was actually observed.

The right panel contains observed base velocity, truth sequence/sim time/base stable
ID, per-stream sample counts, recording state/elapsed sim time/action availability,
and UE camera mode versus runtime echo. Long sensor lists wrap or vertically scroll
inside the panel; they do not enlarge the whole HUD beyond the safe area.

The center mission spine uses only `FInspectionProjection`. Verified point progress
may be a segmented line when `bProgressVerified` and `PointCount > 0`; otherwise show
the named unavailable/unverified state instead of an empty or guessed meter. Report
acceptance remains a separate terminal node because execution confirmation is not
report verification.

### Tactical empty and error states

| Condition | Display |
| --- | --- |
| No session | Header `会话不可用`; readiness chain stops at Session; panels show em dashes |
| Full status unavailable | Caution ribbon `完整控制状态不可用`; no readiness stage is green |
| Status stale | `STALE` plus age when available; keep last values visibly dimmed and labeled stale |
| Identity incoherent | Danger ribbon using `Blocker`/`FullStatusBlocker`; suppress qualified styling |
| No truth frame | `真值帧不可用`; do not substitute requested or admitted motion |
| No sensors | `传感器状态不可用`; an empty array is `未报告传感器流`, not healthy |
| ACK rejected/failed | Danger state with exact reason and event/status sequence when present |
| Task unbound/stale/blocked | Use the inspection projection messages; do not infer from the scene |
| Recording rejected/failed | State plus runtime reason; action remains disabled until its gate reopens |

## 3. Field menu

Name the view `现场菜单` / `FIELD MENU`, not `PAUSED`. Dim the world to 28% visibility
under `surface.void`. The left 58% is an atmospheric identity area with the current
session/task summary and the text `运动输入已释放 · MuJoCo 仍在运行`. The right 42%
is one vertical action rail. Do not carry the current game-session and asset-review
selector into this runtime menu.

The only primary actions are:

| Visible action | Existing binding | Enablement and copy |
| --- | --- | --- |
| `返回驾驶` | `Escape` / gamepad FaceButton Right → `TogglePause()` | Always first and initially focused |
| `开始记录` | `R` → `ERuntimeUIAction::ToggleRecording` | Only when `CanRequestRecordStart()` |
| `停止并提交记录` | same action | Only when `CanRequestRecordStopCommit()` |
| `记录不可用` | no clickable action | Disabled; show current recording state/blocker |
| `结束会话` | `X` → `ERuntimeUIAction::Exit` | Opens confirmation; final request still passes `FRuntimeUIExitPolicy` |

Do not show Settings, Save, Load, Restart, Continue Mission, Online, Inventory, Map,
Robot, or any other decorative action unless a real binding is added later. Robot and
map selection are explicitly deferred beyond this batch.

`结束会话` opens a small modal over the menu. Its title is `结束当前会话？`; body
states that motion is already released and exit still waits for exact stopped/zero
confirmation. `取消` is initially focused; `结束会话` invokes the existing Exit
action. Rejection keeps the modal open and shows the exact policy blocker. There is no
fake success transition before the runtime confirms the request.

### Focus and mouse behavior

- On menu open, release drive input as today, push the menu over the previous mode,
  and focus `返回驾驶`.
- Up/Down moves through enabled actions in visual order and stops at the ends. Disabled
  recording state may receive accessibility focus for its explanation but cannot
  activate. Enter/Space/gamepad bottom button activates the focused action.
- Escape/gamepad right button closes the top confirmation first; otherwise it returns
  to the pre-menu mode. Restore focus to the viewport after closing the menu.
- The confirmation modal traps focus: `取消` ↔ `结束会话`. Closing it restores focus
  to the menu's `结束会话` entry.
- Mouse hover updates the visible focus/selection; mouse click invokes the same handler
  as keyboard/gamepad. Moving the mouse never clears keyboard focus, and no action is
  hover-only.
- Focus is a brass outline plus left diamond and a small luminance lift. Hover uses a
  thinner neutral outline, so focus and hover remain distinguishable without color.

## Existing binding map

This table is the implementation boundary. Presentational components may format these
facts but may not merge authorities or invent substitutes.

| Interface concept | Current source/binding | Presentation rule |
| --- | --- | --- |
| Session identity | `bSessionAvailable`, `SessionState`, `SessionId`, `ModelGeneration` | Short ID is identity context, not readiness |
| Control binding | `bControlBindingAvailable`, `ControlState`, `RunId`, `ResetGeneration` | Show unavailable verbatim when absent |
| Coherence/blocker | `bIdentityCoherent`, `Blocker`, `bFullStatusIdentityCoherent`, `FullStatusBlocker` | Overrides success styling |
| Freshness | `bFullStatusFresh`, `FullStatusAgeNs` | Stale values remain labeled stale, never current |
| Readiness chain | `FullStatus.Readiness.{Physics,Control,Visual,Sensors}.State` | Preserve each stage; no aggregate percentage |
| Requested input | `FullStatus.Runtime`, `FullStatus.Motion.RequestedAxes`, `RequestedInput.bViewportFocused` | Operator intent only |
| Control result | `FullStatus.Status`, `Reason`, `ServerStatusSequence`, `Motion.AdmittedTwist` | Admitted motion, not observed motion |
| Observed truth | `bLatestAppliedTruthAvailable`, truth sequence/time/base ID, `Motion.ObservedBaseVelocity` | MuJoCo truth only |
| Sensor activity | `FullStatus.Sensors[].StreamId/SampleCount` | Count is activity evidence, not sensor health |
| Recording | `FullStatus.Recording`, `CanRequestRecordStart()`, `CanRequestRecordStopCommit()` | State-driven dynamic action |
| Camera | `ActualCameraMode`, `FullStatus.UI.CameraMode`, `C`/`CycleCamera` | Show UE mode and runtime echo separately |
| Inspection task | `FInspectionProjection` identity, progress, history, report and stale/blocker fields | Progress and acceptance remain separate |
| View switching | `B`, `Tab`, `Escape` through `FRuntimeUIModeController` | This batch redesigns Drive/Tactical/Menu only; Build is preserved, not expanded |
| Exit | `X`/`ERuntimeUIAction::Exit` plus `FRuntimeUIExitPolicy` | UI confirmation does not bypass policy |

## Reference concepts translated, not copied

| Reference concept | RobotSimUE adaptation |
| --- | --- |
| Hogwarts layered mission plate | Drive task cluster: spine node, title, progress, separate report line |
| Separate notification/error layers | Tactical blocker ribbon and explicit Drive caution state |
| Segmented scalable meter | Verified inspection point progress only; never health, battery, or readiness |
| Legend item with key and text | Mode-aware bottom key guide using actual keyboard/gamepad bindings |
| Generic popup and confirmation | End-session confirmation backed by the existing Exit policy; no unbound hold meter |
| Field Guide focusable category entries | One vertical Field-menu action rail, not an eight-category hub |
| Palworld key-guide separation | Reusable key chip + action label; unavailable actions state why |
| Palworld radial-menu materials | Not used: no corresponding runtime radial action model exists |

## Concrete mismatch with the current HUD

The current Drive clusters are close to the desired information density, but the
mission fallback can label a selected session as if it were an inspection mission.
Tactical is currently only a centered `READ-ONLY WORLD OVERVIEW` placeholder while
the detailed request/admitted/observed values remain in pale corner panels. Pause
currently routes to the full session/asset selector even during runtime, and the
visible menu hint advertises Resume/Record/End without matching focusable buttons.
The current HUD also polls the snapshot in `Tick`; a later internal cleanup may use
event-driven invalidation, but that is not required to deliver this visual batch.

Implementation is complete for this batch when the three views follow the layouts
above, every displayed value traces to the binding table, menu focus works with
keyboard/gamepad/mouse, and 1920x1080 plus 1280x720 screenshots show no clipping.
Robot/map selection remains intentionally incomplete.
