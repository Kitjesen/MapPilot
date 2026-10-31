# LingTu Web — UI and interaction design

## Connection feedback and responsive workspace — 2026-10-22

The compact map launcher owns the disconnected connection notice. Before the
first telemetry receipt it says Waiting for robot connection; after a lost
connection it says Connection lost and reconnecting. Use neutral secondary text,
without a duplicate canvas warning or instruction-composer notice. Connected
motion holds, task errors and unconfirmed stops retain their existing feedback
and admission rules. Camera availability is separate from the control connection:
an empty preview uses one status row with accessible controls, and actual frames
restore the media view without duplicating empty-state copy.

The app follows the dynamic viewport height. Narrow screens reserve room above
Stop for the task bar; short viewports hide only the idle composer's heading and
example shortcuts. Landscape task controls leave space for Stop and right-side
map tools. The input, send action and active-task feedback remain available.

## Saved point-cloud editing — 2026-10-20

Scene's View settings opens Point-cloud cleanup for a loaded saved map. The
collapsed launcher fits the readable map name plus fixed icon gaps and padding;
it no longer reserves a fixed width. Long names are capped at 380 px or the
viewport minus 16 px and ellipsized. The full identity remains in its disclosure
and accessible name; drag bounds follow the measured capsule width. A compact
tool row offers selection, numeric bounds, original/draft
comparison, and exit. Selection controls float beside the volume. A shallow
bottom bar contains the draft state, Undo, More, and Save as new map. Active task
controls and the independent Stop remain available; the idle composer is hidden.

Selection is a map-frame XYZ box, not a screen-pixel mask. Entering selection
aligns the camera to top view while retaining the current location and zoom.
Dragging defines XY; its initial Z spans the original display cloud, including
points outside the previous display-height cutoff. Inspect front, side, or 3D
views and adjust Z before applying. Numeric XYZ inputs provide an alternative
to dragging. Both box faces are inclusive. Delete selected removes the inside;
Keep only selection is a secondary action under More.

The selection panel reports selected finite points and their proportion of the
current sampled draft. Previously removed points do not re-enter this count;
Undo/Redo updates its denominator. A full-source Z span shows a concise ground
reminder. This is geometric selection feedback, not ground classification or
navigation evidence. Delete is unavailable when no preview points are selected.

Edits form a local ordered history over an unchanged source. Undo/Redo and
original/draft comparison do not write map files. Comparison retains the camera;
the optional deleted-point overlay is visual context. The browser previews sampled
points; native mapd applies the same boxes to the complete source without moving
or resampling retained XYZ coordinates. A source map/epoch change invalidates the
old editor. Editing does not submit goals or relocalization requests.

Escape closes the open disclosure, then cancels selection, then requests exit.
Ctrl/Cmd+Z undoes; adding Shift redoes outside text inputs. Undoing the last edit
keeps Redo reachable. Leaving a dirty editor asks whether to discard; pending
save/build work blocks workspace changes. Dialogs use native modal focus behavior.
Save-as uses the approved A concept: a centred 380 px warm light frosted surface,
dark text, a visible name label, a full-width input, and right-aligned Cancel and
Save and rebuild actions. It has no duplicate close icon or explanatory footer.
A 28 px backdrop blur softens local map detail; the outside veil only lightly dims
the scene. Keep inputs and actions at least 44 px tall. Cancel and Escape return
to the draft; the save-and-rebuild contract and native modal behavior stay unchanged.
Names follow the public map-service limits before submitting: at most 100 ASCII
characters, no consecutive dots, and no reserved operations name. Suggested copy
names fit the same limit and differ from the source. A definite save rejection
reopens the name field with its value and error retained. An unconfirmed save
stays in the existing exact-request confirmation flow and does not enable renaming.
The selection panel scrolls within the viewport and keeps its apply action reachable.
It reserves the actual height of the entire bottom task stack, including an active
task before the first edit. On narrow screens it also stays clear of Stop. Secondary
tools move into More instead of reducing text size or touch targets.

Interaction references are W3C's [dragging alternative](https://www.w3.org/WAI/WCAG22/Understanding/dragging-movements.html),
[target-size minimum](https://www.w3.org/WAI/WCAG22/Understanding/target-size-minimum.html),
and [modal dialog pattern](https://www.w3.org/WAI/ARIA/apg/patterns/dialog-modal/).
The 44 px control target is LingTu's product goal; it is not a claim that WCAG AA
requires 44 px or that this bounded review establishes accessibility conformance.

Save as new map requires a different name and submits the source content epoch.
The UI reports source save and 3D navigation-data rebuild separately. A failed
rebuild retains the new source and offers rebuild-only retry plus View saved cloud.
The result keeps the target name and the service's point_count/removed_points
visible during and after rebuilding. Production counts refer to the saved source;
the local review marks its sample-only counts explicitly with point_count_scope.
Missing counts remain absent instead of being inferred from the preview. Busy
stages show their current phase and target, without inactive editing actions or
invented progress percentages. A lost save response
shows an unconfirmed outcome and offers confirmation with the identical request;
it does not label the save failed or create another target. Success offers View new
map without activating it. Point deletion neither fills ground nor establishes
free space; map activation remains a ProductControl operation, and route/motion
acceptance remains native navigation's responsibility.

Draft recovery stores only the source identity, target name and ordered history
in this browser's localStorage, never a point cloud. Opening the same source offers
Restore or Discard; it does not silently apply old edits. Undoing to zero applied
steps retains Redo. A changed source name, content epoch or frame cannot inherit
old selection geometry. Source identity also keys the live editor's lifetime.
Unavailable browser storage does not stop editing; it cannot guarantee recovery.

Before save dispatch, retain an unconfirmed request. Reopening that state keeps
editing and renaming locked and confirms the original source tuple, target and
applied operation order. Old-source operations are not projected onto a new
source preview. A definite rejection on a changed source returns to stale-draft
handling. Confirmed source save clears its draft even if rebuild later fails;
explicitly discarding ordinary edits also clears it. Leaving an unconfirmed save
keeps its record for later confirmation.

Next work follows the [map-editing queue](../docs/roadmap.md#地图编辑与人工障碍约束2026-10-21):
persistent 3D manual obstacles and native collision consumers, their compact glass
editing tool, then saved-map inspection and route-preview acceptance. A colored
glass annotation alone is not collision support. Keep human constraints separate
from measured clouds; removing a constraint must not mark unknown space free.

## Natural-language task bar — 2026-10-19

The approved task-flow concepts are implemented through the existing instruction
API. Dashboard owns one useInstructionTask controller above workspace changes;
Scene's bottom bar and the console share that owner. Enter commits only outside
IME composition. Examples fill a draft without dispatching it. Rejected drafts
remain editable. Motion admission still uses the shared live-state gate.

Submission acceptance, queried execution feedback and stop confirmation stay
separate. Agent run_id never enters native navigation task storage. Cancel ends
the whole owned instruction via its scoped Agent API. Active instructions cannot
be dismissed or replaced before a queryable terminal result and required stop
confirmation. Explicit not_current releases this client's ownership without a
claim of physical stop. Temporary Agent unavailability keeps querying.

The current instruction stays visible across workspaces. A matching owned native
child is represented by the parent instruction; an unrelated native task retains
its separate controls. Route editing and goal drafts preserve active-task controls.
Only an idle instruction composer is hidden during map-goal/route editing.

Agent/Follow currently provide query and cancel, not whole-intent pause/resume.
Their bar therefore exposes End task. Native navigation retains its existing
pause/resume controls. No UI-only pause or automatic resume is synthesized.

An active instruction must end before a competing manual map goal or route can
start. Read-only map target previews remain available while that owner runs.

Task details open above the shallow bar. They show actual feedback, IDs, domain
state and reasons; no parsed plan, target confidence, ETA or fixed progress stages
are invented. The task title preserves the submitted instruction.

Camera enlargement reuses the existing media element/subscription. Escape restores
the PiP without clearing the goal draft. Follow status captions require matching
Agent/follow/Product identities and fresh producer feedback. Current transports
lack a bbox aligned to the displayed camera frame and a coordinate epoch for the
person position, so this release draws no fabricated pixel lock or map target.
Completing those transport fields remains separate perception integration work.

Single-goal coordinates, height, radius, speed and heading live under Adjust.
At desktop widths the parameter surface sits beside the draft so its heading,
metrics and selected point stay visible; narrower layouts put it above the draft.
Coordinate blur commits finite edits; edits invalidate preview and outstanding
responses before another submit. Preview and submission carry the same heading;
the pale marker distinguishes center, arrival radius and heading arrow. Restore
resets tuning defaults, not a robot pose. These remain planning parameters; the
native controller authorizes and executes motion.

This is the current design source of truth. The approved direction was fixed on
2026-10-18 from the operator's supplied media-player reference and the reviewed
LingTu map/task concept. The map remains the product workspace. Controls reveal
themselves around the current task instead of turning the scene into a dashboard.

## Approved workspace layout — 2026-10-18

- The map or point cloud fills the workspace. Do not reserve permanent columns,
  full-width status bars, explanatory banners, or a large settings panel around it.
- The top-left launcher is a compact white capsule. It opens on click and contains
  navigation between workspaces and uncommon actions. It does not open on hover.
- The camera appears by default as a compact top-right picture-in-picture card.
  It has one visible close/reopen control. Opening it subscribes to media only;
  it does not start a camera process or recording.
- The contextual inspector is a floating card and starts closed. Settings, layers,
  target details, and diagnostics expand only when requested. Opening or closing it
  never resizes or resets the map.
- The right edge exposes only two quiet tools: layers and view settings. Each opens
  the relevant floating card; neither reserves a permanent inspector column.
- The current task is a bottom-centred mini-player. It shows one real task name,
  one authoritative state, and the single most relevant action. The more control
  reveals reason, freshness, identifiers, and secondary controls. Do not display a
  palette of possible states, a task dashboard, or a card inside the task card.
- Stop is an independent bottom-left control that remains reachable when menus,
  inspector, camera, or task details are open. It uses the existing software Stop
  command and never claims to be a hardware emergency stop.
- Use actual map, task, target, and status names. Do not invent a destination name,
  progress percentage, ETA, telemetry value, or decorative KPI.

The stable desktop composition is:

```text
┌──────────────────────────────────────────────────────────────┐
│ [launcher]                                      [camera PiP] │
│                                                              │
│                     map / point cloud                        │
│                                            [inspector, open] │
│                                                              │
│ [Stop]              [current task mini-player]               │
└──────────────────────────────────────────────────────────────┘
```

## Information hierarchy

The operator's normal sequence is: inspect the environment, choose a destination,
preview the route, start deliberately, monitor the task, then stop or recover when
needed. The interface follows that sequence.

- Target preview answers where the operator selected, whether a route exists, and
  whether Start is available. Editing the target invalidates the previous preview.
- Current task answers what is running, its native lifecycle state, and the next
  truthful action. Freshness, reasons, identifiers, and secondary controls stay in
  the disclosure unless they require immediate operator action.
- Robot status answers whether the robot is connected and localized, and presents
  measured motion only while the telemetry is fresh.
- Task/request IDs, raw reasons, coordinates, rates, and process details belong in
  disclosures. An actionable hold, disconnect, or failed stop confirmation remains
  visible without opening technical details.

Keep related content flat. Use whitespace, alignment, type size, and one primary
button to establish hierarchy. A normal state should not produce another badge,
legend, toast, or status card.

## Visual language

Use the shared tokens and self-hosted fonts in `src/App.css`.

- Light workspace: `#f4f3ef`; white surfaces; `#202321` primary text; `#626761`
  secondary text. The map canvas may retain its dark spatial background.
- Dark workspace: `#181818` canvas; `#212121` surfaces; `#ededed` text.
- Normal controls are monochrome. Muted amber marks a relevant hold or warning.
  Red is reserved for Stop, errors, and destructive actions. Point-cloud palettes
  remain user-configurable and do not define task state.
- Floating cards use restrained borders, 20–26 px radii, and broad quiet shadows.
  Glass is limited to overlays where preserving map context helps.
- Avoid ornamental gradients, glowing indicators, pulsing normal state, excessive
  transparency, and filled tiles around every property.

Typography uses LingTu Inter for Latin text and digits and LingTu Noto Sans SC for
Chinese. Use weights 400–550 for normal interface hierarchy and up to 700 only when
needed. Body text is 15 px, controls and primary data 14 px, and captions at least
13 px. Telemetry uses tabular numerals. Technical identifiers alone use the mono
stack. Wrap or collapse controls instead of shrinking readable text.

## Interaction contracts

- Selecting a point creates a draft. Explicit confirmation submits a navigation
  request. Dragging the map or dismissing a menu never selects a goal.
- Single-goal drafts stack above the current-task mini-player in the bottom
  centre. Selecting or editing a draft does not hide the current task's pause
  control. Both stay clear of the independent Stop on desktop and narrow screens.
- The closed draft has no internal scrollbar: show its target, effective route
  or straight-line distance, map-Z height, one status, and one current primary
  action. Speed, acceptance radius, coordinates, and raw failure details open
  through Adjust in an upward floating panel. Escape closes Adjust first.
- A failed preview offers Retry instead of a disabled Start beside Retry. A
  service failure and an infeasible route have distinct short messages. Native
  reason codes remain available in details. A ready draft says Switch to this
  target while a task is active, otherwise Start navigation; use the existing
  native goal-replacement contract. Submission feedback says submitted, never
  completed. The pale target ring keeps the actual acceptance radius; its
  default label does not repeat that parameter.
- Ordinary navigation clicks use the current body-height reference plane. Explicit
  3D surface picking uses the observed vertex. This UI reference is not observed
  ground and does not prove that a route exists.
- A changed start pose invalidates a preview while retaining the selected target.
  A map or Product identity change clears the draft. A recoverable transport
  disconnect keeps it and previews again after reconnection.
- Pause, resume, cancel, and Stop show request acceptance separately from native
  execution state. HTTP success never becomes physical completion.
- Confirmed native terminal state takes precedence over an older local request.
  Otherwise stale status takes precedence over pending confirmation copy.
- Successful and cancelled task mini-players may dismiss after eight seconds.
  Paused, failed, stale, and stop-pending states remain visible.
- Full-screen map/video keeps Stop reachable. Entering full screen closes the
  inspector and restores its previous state after exit.
- Use 160–180 ms opacity or short displacement feedback and respect reduced motion.
  Never animate fictional robot movement or interpolate status values.
- Primary motion controls target at least 44 × 44 CSS px, retain visible keyboard
  focus, and have touch/click alternatives to hover and dragging.

## Camera behavior

Camera PiP starts open when entering Scene. Show waiting, unavailable, or disconnected
until a fresh decoded frame arrives. Decode a JPEG before replacing the displayed
frame, and drop queued stale frames rather than displaying them later. A connected
socket alone is not live video. Closing the PiP releases its preview subscription;
reconnecting does not reopen a preview the operator closed.

Review documentation and screenshot captions must identify recorded frames and
fixture telemetry. Exercising a live-state label with fixture input is not
evidence of a live camera connection or robot motion.

## Map, cloud, and layer semantics

The registered scan is a current observation. Accumulated map points, saved-map
points, elevation, occupancy, traversability, and sampled collision diagnostics are
different products and remain separately labelled. A point-free area does not prove
that the full collision geometry is clear. A reference grid is not a live map.

The saved-map viewer uses the full canvas and small floating controls. The point-cloud
inspector starts closed and contains only these groups:

- Color: height Z or a single color. Color never implies a semantic class.
- Point rendering: point size and opacity with aligned values and units.
- Scene: background and coordinate grid, collapsed until opened.
- Display height: an upper map-Z cutoff with a readable slider and editable metre
  value. It is not robot height, navigation height, or a motion authorization.

Keep point counts and map version in one compact disclosure. Numeric edits apply on
Enter or blur, clamp to observed bounds, and restore the previous value on empty input
or Escape. Opening settings does not alter stored map data, picking coordinates,
planner inputs, or the current task.

Mapping's optional top-down occupancy view uses observed free, occupied return, and
unknown states within its rolling window. It does not claim support, traversability,
or map-completion percentage. Live radar remains an independent optional overlay and
does not replace the accumulated map. During mapping, a click inspects a grid cell and
never submits a navigation goal.

## Height and stair inspection

- Full, follow, and fixed display-height modes are visual tools. Follow mode retains
  faint context outside its focus band; fixed mode alone clips selection geometry.
- Changing a display-height mode never moves the goal plane or authorizes motion.
- Stair annotation selects observed 3D bounds and explicit lower and upper landing
  surfaces. Stored entrance Z is the observed landing surface, not a body target Z.
- Annotation clicks do not send navigation goals or relocalization commands. Surface
  misses leave the selection unchanged. Save, reload, edit, and delete use native map
  ownership; stale annotations require realignment.

## Data honesty and safety

Connection, control ownership, goal admission, motion permission, localization, task
state, and observed movement are separate facts. A connected stream does not prove
motion readiness. Missing or stale values remain unavailable; never show a placeholder
zero as measured stop.

The robot body follows measured pose. Without joint telemetry, legs use a fixed display
stance. Do not animate fictional walking. Preview is read-only and remains separate
from native motion gates. Native navigation retains final motion arbitration.

Read-only observation exposes viewing and display settings but no motion or map
mutation. Software Stop remains available while disconnected so the request can be
retried, but its UI distinguishes request submission, command acknowledgement, and
measured standstill.

## Responsive layout

- At narrow widths, keep the map first and preserve its useful area. The launcher,
  PiP, inspector, task mini-player, and Stop must avoid one another and device safe
  areas.
- Camera PiP reduces before it covers the task or target action. Inspector content
  scrolls internally and never creates an unreachable final control.
- The task mini-player may expand toward the centre/top, leaving the bottom-left Stop
  hit area clear. Long task names truncate on the collapsed row and wrap in details.
- Compact secondary view controls before reducing text below 13 px. Avoid permanent
  side columns or a second toolbar row merely to fit desktop actions.

## Verification boundary

Validate the affected UI contracts, build, keyboard interactions, narrow layout, and
actual browser rendering. Fixture controls must be closed to external robot APIs and
must identify recorded imagery and map sources. Browser fixtures do not prove field
navigation, real camera freshness, robot stop, accessibility conformance, or deployment.

The 2026-10-18 implementation review uses the retained 903 PCD and an explicitly
recorded NX frame. Browser visual verification is still required; the existence of
the fixture and passing source checks are not visual acceptance.
