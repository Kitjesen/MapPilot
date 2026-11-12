# LingTu Web — UI and interaction design

## Business workspaces — 2026-11-10 (current)

Primary navigation contains four workspaces: Control, Mapping, Navigation and
Inspection. The capsule names the current workspace; map identity and the actual
runtime Product remain separate context. Selecting a workspace only changes the
view. Starting mapping, using a map and dispatching a task retain explicit actions.

- Control: camera, an explicit keyboard/gamepad entry, and robot status. Manual
  input mounts only after the operator opens it and releases on leaving the page.
- Mapping: start, save and saved maps. The mapping page cannot pick navigation goals.
- Navigation: choose a map, localize and choose a goal. Real mapping mode still
  blocks goal dispatch. Point and heading preview keep their existing admission.
- Inspection: existing route, task progress and result workbench.

Map management is a child of Mapping or Navigation and returns to its origin.
Scanning overlays, follow-camera and viewpoint controls belong in the right-side
Display menu, not alongside business actions. Keep the independent Stop control and
current task feedback available across workspaces.

Localization diagnostics, data diagnostics and planning parameters open inside
Settings / Diagnostics. They reuse existing data sources and controls, offer a back
action, and do not become primary workspaces. Existing satellite/fusion details stay
available within localization diagnostics.

## Robot conversation — 2026-11-10

The conversation composer puts input above a single compact toolbar: attachments,
history, model name, thinking level and send. Model and thinking choices open
upward from their current values, with keyboard navigation and Escape dismissal.
The thinking glyph adapts T3 Code's MIT-licensed leaf component; its fixed source
and license live beside `components/agentConversation/EffortGlyph.tsx`.
Options come from the active Agent provider; changing the model
changes its supported thinking choices. Keep the selected model between messages.
Enter sends, Shift+Enter inserts a newline, and IME composition never sends a turn.
Pending retries retain their original message ID, model, thinking and attachments.

Attachment chips show name, size and removal; history keeps their metadata.
The current limits are four files per message and 10 MiB per file. Support text,
code, images and Kimi document extraction without adding a separate upload screen.
Assistant replies use the existing escaped Markdown renderer. Long history scrolls
inside the card while the composer remains usable on short screens. Connection
failures describe the actual unavailable service rather than showing raw backend
errors. Tool progress and final replies use conversation polling; token streaming
is not yet presented by this card. Chat availability does not establish robot
motion readiness.

During point-cloud editing the conversation composer is hidden, while its draft,
history and any active run remain owned by the existing hook. Task feedback and
the independent robot Stop remain available. Leaving editing restores the composer.

## Light surfaces and hover intent — 2026-11-09

The scene canvas is white with a light gray grid. Preserve the multicolor point-cloud
palette. Cards use white surfaces, subtle borders and shadows; selected controls use
a pale blue-gray fill with dark text. Camera imagery retains its original pixels.

The launcher and transient menus wait 220 ms before opening on mouse hover and close
180 ms after the pointer leaves the trigger and content. Crossing into the content
cancels closing. A click immediately after hover does not reverse the opening.
Keyboard and touch use native activation; Escape dismisses the innermost surface.
Map switching, cloud settings and the layer inspector open explicitly by click so
passing the pointer does not replace the working view. Explicit configuration panels
stay open until closed; transient hover menus disappear on exit.

## Operating controls — 2026-11-09 (current)

Scene tools open Record or History directly. There is no separate operation overview,
task summary grid, or second live camera. The map PiP owns live observation; task and
robot details stay in their existing panels. Record/History use compact header actions,
with heading focus after switching. Diagnostics mount only while expanded.

The Include video checkbox sits beside recording controls so the operator can see
what will be captured before starting. Previewing the camera does not enable video
recording. Preserve recorder readiness, incomplete-session recovery, selection,
history playback and download checks. Missing data remains unavailable, not zero.

Connection and localization labels share the timestamped snapshot interpretation.
Manual status refresh updates the same source as live events. Connection alone does
not imply localization or motion readiness. Fresh later events replace refreshed data.

The current task owns cancellation. Its visible cancel action keeps confirmation,
request progress and waiting-for-stop feedback until a terminal result arrives.
Instruction-owned tasks retain their own parent-task cancellation. Software Stop is
independent. Failed or blocked tasks expose one short reason in the collapsed task
bar; identifiers and raw diagnostics remain in details.

## Map gallery — 2026-11-10

The saved-map workspace uses the full viewport. The library attaches to the left
viewport edge and display settings attach to the right, with full-height panels.
Mouse hover or native click/keyboard activation slides the relevant panel out,
and leaving hides it. Each panel can be pinned independently, with the preference
retained in this browser. Panel opening and closing preserve the map framing. The initial
visual reference was Raul's [Shader Dial](https://www.inspora.design/posts/shader-dial).
Library rows show a thumbnail, name, file size, update time and status icons. Cleaning
state has an icon with a tooltip and accessible label, not a repeated sentence.
Search filters the library rows while retaining the selected map and camera view.
Navigation-artifact icons use green for ready, blue for processing and amber for
attention, retaining their shape and accessible label. Point counts appear below
the list. Patch counts, save history and management actions remain in details.

REA captured the reference page and its embedded video; the original interactive
shader source and timing are not available. Hover, keyboard focus and touch
selection load the selected map's complete PCD in the same workspace. Selection
uses the existing read-only artifact API and preserves explicit navigation
admission. All MapView entry points use this integrated viewer. Editing returns to
it. Each library row offers rename for that map, subject to management access and
the active-map restriction. A compact, persistent tool strip exposes Crop, Clean,
No-go and the pending Surface Fill entry. Crop opens the existing editor in box
selection mode. Clean opens its cleaning parameters after the full cloud loads.
No-go directly targets its section in Map Settings. Selection-specific delete/keep
actions and draft undo/save controls appear in the editor. Crop and Clean now run
on the same gallery canvas and the already-loaded complete cloud, retaining the
camera, zoom, point size, opacity, height cutoff, grid and source color range.
Entering selection never forces a top view. A projected rectangle creates a 3D
bounding box; the orange box and highlighted points show its actual scope. This
can include front and back points along the view direction and is not an occlusion
mask. Height and XY bounds remain adjustable. The selection card is light and
compact, draggable by its header, keyboard movable, and constrained above the
editor footer and within the viewport. Side library/settings docks hide while
editing to preserve the selected source. Original/draft, undo, recovery, save-as
and navigation rebuild retain their existing transaction behavior. Display controls remain
in the right panel. Rebuild and navigation actions stay in map details.

Manual surface filling means drawing ground support or wall blocking geometry.
The tool strip marks it as pending until drawing, persistence and native navigation
consumption are connected. Automatic support filling is a separate preview feature.
No-go editing currently uses numeric 3D box bounds.

Thumbnails come from the saved map's read-only preview API, including inactive
maps. Sample at most 24,000 points, draw a transparent isometric image for the row
and use those sampled coordinates for quick feedback while the full file loads.
Cache thumbnails by map name and content epoch for the page lifetime, serialize
native reads and load rows when visible. Complete PCDs use the existing versioned
artifact reader, with parsing and bounds calculation in a worker. Retain at most
two decoded full clouds for selection back and forth. A 160 ms selection delay
avoids full downloads for rows the pointer only crosses. Failed complete reads
keep their error visible and cannot advertise the sampled preview as complete.

The centre uses OrbitControls and the actual saved PCD or native OctoMap. The full
PCD layer keeps its typed coordinate buffer, transforms the coordinate basis once
on the object and computes height colors on the GPU. Shared coordinate and color
functions remain authoritative. Bounds from the worker also supply a conservative
bounding sphere, avoiding another full scan during first rendering.
Drag to rotate and scroll or pinch to zoom. Keyboard arrows pan, +/− zoom, and Home
restores the initial view; there is no central button strip or map-name caption.
A small, uncaptioned XYZ indicator sits above and to the right of Stop and follows
the camera in the map's coordinate frame, with red X,
green Y and blue Z up. Use thin axis lines and colored letters with no circular
backgrounds. Window resizing preserves orientation and relative
zoom. Camera changes share one animation frame with the map and XYZ indicator.
Orbit damping uses the existing scene's 0.1 factor and respects reduced motion.
The current full cloud keeps the same vertices, point size and opacity while
rotating, zooming, damping and idle. Interaction does not switch to a sampled
index or schedule a later density restoration. Cropping and cleaning still
display their current draft data. The loaded coordinates and stored map retain
their full data.
An idle saved-map view needs no continuous render loop.

The settings panel uses black text on white, compact single-row sliders with
2 px muted-blue tracks and 9 px knobs, and smaller selection buttons. Keep their
keyboard focus and touch hit areas. Both panels slide over 460 ms with deceleration
and a short content reveal. Rows use an animated selection mark and thumbnail
feedback. Reduced-motion preferences disable transitions. The panel switches
between the complete cloud and the actual saved OctoMap,
and directly controls point size, opacity, upper display height, reference grid
spacing and displayed voxel size. Voxel sizes aggregate occupied native leaves
for display only; they do not rebuild or change map resolution. A map without
OctoMap disables that choice. Switching maps clears the old height cutoff.
The full-height slider endpoint always shows all source heights.

Missing durable cleaning evidence says unknown; filenames and unrelated save history are
not evidence. The native metadata timestamp describes processing/update time,
not original acquisition time. Missing timestamps stay unrecorded.

Details and actions retain existing save history, retry, processing, optimization,
download, rename and deletion behavior. Unpublished saves remain discoverable by
name. An old successful job cannot mark a missing map ready. Page-owned polling
continues while details are closed. Readiness never authorizes motion.

The Scene workspace retains its compact map picker. Both surfaces use the same
status meanings: check for ready navigation artifacts, spinner for processing,
and an alert for incomplete data or unavailable status. Selection and readiness
remain separate. Popovers stay within the viewport on phones; their contents
scroll when needed. Closing a menu never hides the underlying map workspace.
Load failures remain visible even while the library menu is closed.

The top dock sizes to its actual content in both collapsed and expanded states.
An empty scene-action slot is hidden. Position clamping and saved layout width use
the measured width, without reserving an empty 920 px bar on other workspaces.

The idle instruction composer belongs to Scene. Saved maps, recording, tuning
and inspection management do not show it. Existing task state, stop/end actions,
pending submission feedback and receipts remain visible across workspaces;
the console retains its own chat input. Browsing maps never starts a new task.

## Configuration cards — 2026-11-09 (current)

The supplied Acme usage-card reference now guides cloud-display and map-parameter
cards: a quiet gray shell, white inner surface, white raised segmented selection,
prominent values with secondary units, and a gray action footer. Blue is a control
accent; the approved point-cloud height colors and saved values stay unchanged.
Use real controls only, without decorative usage charts or invented metrics.
Cloud settings may cover the task composer while open, but remain below Stop.
Cloud-display labels sit above prominent values and each 44 px slider. Keep the
palette selector in the header. Hover and focus highlight the associated parameter;
values and the rendering update immediately, without counting animations. Hide
redundant group legends visually while retaining their accessible names, and
keep the default panel compact enough to show its footer without scrolling.
Saved-cloud settings offer height-based color palettes only. Remove the redundant
height/monochrome switch; old monochrome preferences return to multicolor height
without resetting point size, opacity, background or grid.

Settings categories share a sliding selected surface (220 ms). Scene, legend,
preview notes and no-go disclosures interpolate their natural height with native
details semantics. Reduced-motion preferences remove these transitions. Browsers
without intrinsic-size interpolation keep immediate functional disclosure. Keep
rendered values immediate; animation must not delay parameter application.

## Stream usage — 2026-11-09

The compact revision uses a 960 px workbench: a maximum 440 px left column holds
bandwidth, filters and streams; the right column holds camera and the selected
stream inspector. Stack the columns below 760 px. Use one UI font family throughout
this workbench, 13 px body/secondary text, 16 px headings and 28 px primary values;
retain SVG axis scaling for legibility. The bandwidth card has a single surface,
one summary row, a short proportion bar and compact category labels. A fixed footer
shows coverage or the highlighted category without changing the card's geometry.
No-frame video uses a compact status row; it keeps the measurable video element and
retry controls. Sampling explanations and raw fields remain available in disclosures.

The live Data workspace places a bandwidth card above the stream list. Its total
and stacked segments summarize observed bytes per second by catalog category,
independently of the list filter. Report observed/catalog coverage; missing or stale
observations are unavailable, while measured zero remains zero. Do not add message
frequencies across categories or imply day/month history without stored aggregates.

Hover and keyboard focus link segments, category labels and matching stream rows.
Clicking a category toggles the existing filter; viewing a stream selects its
inspector. Neither action changes recording checkboxes. Keep unhighlighted text
readable, reserve a fixed readout area, and keep segment hit regions stable during
highlight animation. Preview data belongs only to the explicitly labelled test page.

The idle instruction composer uses the same neutral shell, white input and blue
send action. Omit the redundant heading when it has no close control, keep 44 px
input/send targets, and let errors or status text expand its natural height.
Examples only fill the input. Task feedback, receipts and details use theme colors
with semantic warning/error text; retain their existing lifecycle controls.

## Bento surfaces and soft-glass choices — 2026-11-08

This direction supersedes the earlier warm-charcoal confirmation treatment.
Task bars and task confirmations use neutral theme-aware surfaces, compact rounded
corners and contrasting circular icons. Confirmed task state retains its text,
small color cue and restrained motion; no completion percentage is inferred.
Point-size and opacity cards emphasize their existing numeric values and thin
colored native sliders. The labels remain associated with the range inputs.
The two parameter cards share one row; their native slider hit areas remain44px.
Background choices use compact44px rows with icon and label side by side. Appearance
settings reduce section padding and cap theme previews at84px without cropping;
only the appearance tab drops the fixed minimum panel height.

Settings categories sit beside the dialog title, with the close control fixed at
the far end. Long localized labels scroll within the category strip on narrow
screens. The scene dock groups spatial, top-down and fit-map controls at its right
edge as distinct 40 px icons with accessible names and hover titles. Mapping menus
retain their text labels. Preserve the 920 px dock width and existing drag bounds.

The LingTu map-information card starts closed independently of the scene dock.
The connection-status disclosure opens it on demand; its close control, Escape,
outside click and dock collapse dismiss it. Keep measured connection/localization
values and the existing map-library entry unchanged.

The right-side island contains camera, layers and display as direct 44 px actions
in one compact 52 px row. Camera has one show/hide entry and a short truthful
status. Before a frame is available, keep its measurable media container mounted
but transparent; do not show an empty preview or enlargement. Live video opens
below the stable island. Teleoperation stays a separate action. Menus remain
outside the clipped media area, and compact windows avoid the expanded dock.

Map settings use a two-column grid for the four surface-preview parameters, with
units beside short labels. Keep preview-only scope visible; details and no-go
coordinates expand on demand. Do not change numeric bounds or map mutation gates.
Modal focus traversal includes disclosure summaries and excludes closed content.

## Approved saved-cloud appearance and drag — 2026-11-09

Keep the user's approved saved-cloud preset: multicolor height palette, 1.1 CSS px
points, 29% opacity, theme background and coordinate grid. Height colors remain
blue → cyan → green → yellow → red, with the existing per-map percentile display
range. Stored user choices override defaults; reset uses this approved preset.
The preset is shared by saved-map and recording cloud views, not by world-unit
live-cloud rendering. Three-dimensional dragging rotates near-side geometry with
the pointer horizontally and vertically; preserve 2D panning and click thresholds.

Theme and point-cloud background choices use a localized, very pale peach/violet
surface: quiet dashed unselected tiles, a brighter selected tile and a top-right
round check. Preserve theme previews and native radio/checkbox semantics. The
glass appearance is drawn within the choice group; the scene behind it stays
sharp. Do not copy a popover pointer onto a centered settings dialog.

References: [Bento2](https://www.shadcnblocks.com/block/bento2) and
[Kyle Thacker's soft-glass picker](https://www.inspora.design/posts/soft-glass-workspace-picker).
REA public-page/video observations guide the visual implementation; private
component source and original animation timing were not obtained.

## Task card and confirmation — 2026-11-08

The current native navigation task uses a shallow borderless bar in the existing
SceneView task slot or global task slot. Confirmed execution has two slow crossing
cyan light fields; paused feedback is still amber, failure is a soft red pulse.
Pending requests and stale feedback do not animate as execution. Reduced motion
keeps state color without movement.

Resume and cancellation use the shared product ConfirmModal, not browser confirm.
Keep one title, one consequence sentence and two actions. Short confirmations use
a compact 344 px opaque themed surface, 18 px padding/corners and aligned copy.
Input forms retain 380 px. The backdrop only dims by 8%; neither it nor the dialog
blurs the workspace. There is no outer border or duplicate close icon. Native
dialog focus starts on the back action and returns to the opener on dismissal.
Confirmation belongs to the task that opened it; check current task and motion
admission again before dispatch. Accepted requests still await robot confirmation.

Next UI work, in order:

- [x] Integrate the animated task bar into the existing production task slots.
- [x] Replace task browser confirmations and unify the shared confirmation surface.
- [ ] Reveal one translated failure reason on the task bar; keep IDs and raw diagnostics in details.
- [ ] Make Browse / Select target discoverable without opening the more menu.
- [ ] Check and then adjust short-screen coexistence of task details, goal/heading controls and camera.

Full-page verification uses the real App with local task data and disconnected
transports. It establishes presentation and interaction, not field motion readiness.

### Inspora reference refinement — 2026-11-08

Reference: https://www.inspora.design/posts/agent-dashboard (Jessy Barret).
REA observed a motion video, not DOM task controls. Reuse its warm charcoal,
rounded row selection and small orange accents; transition durations here are
local choices, not recovered source values.

Task confirmations use a compact opaque warm charcoal surface and amber primary
action, while other shared dialogs retain their theme. Backgrounds stay sharp.
Appearance theme cards use a borderless selected surface and a small rounded
check. Point-cloud background radios use a two-column selection grid; coordinate
grid keeps its native checkbox with a small orange check and row feedback.
Selection means a real preference is selected, never that a robot task completed.
Preserve radio/checkbox keyboard behavior, existing callbacks, 44 px targets and
reduced-motion support. Do not copy the video's task-removal effect onto settings.

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
- The top-left launcher is a compact white capsule. Mouse hover or click expands
  it. Workspace, map-information and display dropdowns also open on mouse hover;
  touch and keyboard keep their native click/activation behavior. Hover only
  reveals choices: selecting a workspace or executing an action remains explicit.
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
  secondary text. The scene canvas uses a white spatial background and light gray grid.
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

- Color: multicolor height Z palettes. Color never implies a semantic class.
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


## Independent sensor cards — 2026-11-12 (current)

Sensor types open as independent cards beside chat. Each has its own close,
collapse, and expand actions; duplicate opens focus the existing card and retain
its selection and samples. Collapsing retains sampling, closing releases that
card's input, and Host IMU recording continues independently. The compact sensor
picker anchors above the chat control or below the workspace toolbar, supports
search and keyboard selection, and marks already-open cards.

Use the existing light/dark palette, restrained colored icons, 12–13 px body/title
text, 30 px primary numeric readings, and 10–11 px supporting information. Camera,
LiDAR, thermal and acoustic content each get a dedicated presentation. Device
controls and pixel/source detail remain accessible inside disclosure rows. Do not
show synthetic readings outside the explicitly labeled development preview.


## Sensor interaction and density — 2026-11-13 (current)

Use the custom SensorMark vector family throughout sensor entry points. Numeric
cards use a portal multichannel picker; different physical units stay in separate
plot groups. IMU defaults to stacked axes; each group can fold or overlay its axes.
Use timestamped inspection, explicit gaps, time-window zoom and review controls.

Normal cards resize from the right, bottom and corner. Pointer resize follows the
pointer without easing; settle/expand/fold transitions remain short and respect
reduced motion. Saved dimensions share the existing widget-layout store. Compact
views retain status and selected readings; reduce secondary content before hiding
plots. Keep recording stop actions and device errors available. Do not clear data
or restart subscriptions when changing the display layout.
