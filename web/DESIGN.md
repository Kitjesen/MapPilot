# LingTu Web — UI and interaction design

This is the current design source of truth. It follows the operator's requested
neutral light/dark palette, an icon-only brand, and a scene-first workspace.
The previous navy/fluorescent-green direction is retired.

## Purpose

Help an operator understand one robot's environment, position, motion, and next
action. Inspection routes, maps, recordings, and diagnostics remain available;
the default view should not require understanding process names or DDS topics.

## Information architecture

- Global header: logo, Scene, Maps, Inspection tasks, More, connection, Stop,
  and Settings. More contains the control console and advanced diagnostics.
- Scene: one persistent 3D canvas and one contextual inspector. The inspector
  has Status, Layers, and Actions. Detailed map/location/localization tools
  replace the inspector in the same column and provide a Back action.
- Status: measured pose and velocity, planning state, sensor availability.
  Raw coordinates, command components, and rates expand on demand.
  Keep the default view concise: one short status per section, with explanations
  inside details. Avoid repeating disconnected messages and introductory prose.
- Layers: explain what each visible layer represents. Common display toggles
  come first; terrain and specialist diagnostics are progressive disclosure.
- Actions: map tools, saved locations, localization and recording.
  Availability follows the existing Product and motion gates.
- Keyboard teleop is an explicit scene mode, initially off, not an inspector panel.
  Its compact strip precedes the canvas and exposes only a speed cap, input hints
  and connection state. Keep the inspector usable. Exit or opening a workspace
  drawer unmounts the controller and releases input. Never resume a held key on entry.
  Recording and camera preview stay in the inspector, never over the canvas.
  Mapping uses green local surface points and a separate blue current scan.
  Keep a compact source legend below the canvas and explanations in Layers.
- Settings: appearance and system information. It is not a duplicate navigator.

## Data honesty

Connection, control ownership, motion permission, and task state are separate.
A connected stream does not prove readiness for motion. Show missing or stale
values as unavailable; never show placeholder zero speed as measured stop.

The registered scan is a current sensor observation. Accumulated/saved map
points and sampled local safety diagnostics are different layers. A point-free
area does not prove that the full collision grid is clear. Explain this beside
the layer controls and inside sensor details, not only in documentation.

Use actual trajectory samples and scan-time transforms. The robot body follows
measured pose. Without joint telemetry, legs use a fixed display stance; do not
animate fictional walking. A background reference grid is not a live map.

Planning blockers and motion holds remain outside inspector tabs. Control
ownership and permission remain visible while inspecting layers or actions.
An accepted command is not a confirmed physical outcome.

## Visual language

Use shared tokens in src/App.css. Light: #f5f5f5 canvas, white surfaces,
#1a1c1f text. Dark: #181818 canvas, #212121 surfaces, #ededed text. Primary
controls and selected navigation are neutral. Muted amber indicates a relevant
hold or warning; red is reserved for stop, errors, or destructive actions.

Use one system UI font stack, tabular figures for telemetry, and monospace only
where coordinates or technical details benefit. Main title 20–22 px; UI labels
14–15 px; section headings 16 px; secondary data at least 13 px. Path legends
stay 14 px on mobile as well as desktop. Give numerical values stable widths.

Panels use 14–18 px corners, quiet borders, and restrained shadows. Glass is
limited to layered chrome and floating surfaces. Avoid glowing indicators,
ornamental gradients, moving backgrounds, and pulsing normal status.

## Interaction

- Selecting a point previews a goal; a deliberate confirmation submits it.
- Preview is read-only and is available independently of the motion-start gate.
  A changed start pose invalidates the preview, while the selected goal stays.
  Keep planning feedback in one fixed-width lower-left goal card; do not duplicate
  it in toasts. Camera preview shrinks while a goal is selected and never covers
  the goal actions. Compact secondary view buttons on narrow screens.
- Stop following the camera when selecting a goal. Open saved navigation maps
  in top view; local risk and live scans are opt-in overlays, separate from the
  static traversability legend. Empty paths mean no route found in this attempt.
- Show camera preview directly in Scene, with a visible close/toggle control.
  A socket connection alone is not live video; use fresh decoded frames and
  actual camera status. Decode new JPEGs before replacing the displayed frame.
- Successful or cancelled task cards dismiss after eight seconds. Failed or
  paused tasks remain visible. Normal goal replacement is not an obstacle alert.
- Resume keeps the selected goal and waits for displayed native state to confirm
  that the control hold has cleared; HTTP acknowledgement is not the UI state.
- Clicking outside a navigation menu only dismisses it, never selects a goal.
- Inspector tabs support arrows, Home/End, clear focus, and a selected state.
- Changing tools preserves the live canvas and its camera.
- Software Stop stays in the global header. Existing reset and motion gates,
  acknowledgement handling, and keyboard-release behavior remain intact.
- Read-only observation exposes viewing/settings, no motion or map mutations.
- Use 160–180 ms opacity/short displacement feedback. Respect reduced motion.
  Never interpolate status numbers or invent movement to make the UI feel live.

## Mapping observation view

The `map` Product opens the local surface point-cloud view. The optional top-down observation view uses a height-band projection. Native `maps.occupancy`
provides the rolling height-band projection: teal is observed free space,
coral is an occupied return, and hatched gray is unknown. These are observation
states, not ground-support or traversability claims. Show the window dimensions
and observed/unknown area within that window, never a map-completion percentage.
Outside the rolling window carries no statement about prior mapping.

The point-cloud view is a separate display of observed surfaces. Live radar is
an independent opt-in overlay; toggling it must not replace or hide the map.
During mapping, clicking inspects the actual grid cell and never submits a
navigation goal. Brief advice belongs in the fixed inspection card; fuller
reading guidance stays in one disclosure. Unavailable or stale grids disappear
instead of keeping old free-space coloring; point clouds can remain as a
clearly labelled reference. Saved navigation maps retain their separate static
support/clearance view and explicit path-preview/send interaction.

## Responsive layout

Desktop uses a flexible canvas and a 290–310 px inspector. Tools never add a
third column. Below 800 px the scene stays first and the inspector follows it;
all controls remain reachable. Below 700 px navigation wraps onto its own row,
while connection, Stop, and Settings stay at the top. Critical text wraps.

## Verification boundary

Validate build, affected UI contracts, and actual browser interaction. Local
preview is disconnected from the robot during design work. Browser rendering
checks are not field navigation acceptance or proof of Go2 deployment.

Scene content starts directly with the map and inspector. Do not repeat the active
workspace title in a separate heading. Put status and recovery actions in a fixed
36 px strip below the map toolbar; no separate alert band above the workspace.
Keep read-only/simulation context inside the toolbar rather than a header row.
