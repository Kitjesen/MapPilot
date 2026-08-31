# LingTu SimStudio Lightfield Design System

Status: **approved SimStudio management/evidence foundation; legacy preview migration in progress**
Scope: `tools/simstudio/ui`
Design intent: a lightweight management interface for packages, prepared sessions,
runs, recordings, evidence, and revisioned SceneDraft operations. RobotSimUE Runtime
is the playable world, and UE Create is the immersive 3D authoring surface. This Web
application must never present its preview as either UE product surface.

## 1. Product principles

### World context first — never a live-runtime claim

When a page contains a recording, evidence frame, map, or SceneDraft preview, that
context may own at least 80% of the usable viewport. It must be labelled as a Web
preview and may not claim live UE rendering, viewport input, robot control, or
immersive authoring. Permanent sidebars, inspectors, tables, and timelines should not
reduce a useful preview below that threshold; details appear as contextual drawers,
popovers, or mode-specific trays.

### Progressive disclosure

The default view answers only:

1. What world, robot, and session are active?
2. Is the runtime safe and ready?
3. What is the next objective?
4. Is evidence being captured?

Hashes, manifests, transport bindings, raw logs, and dependency graphs remain
available, but open on demand. They must never be removed or replaced with a
visual score.

### Game interaction, engineering language

The product may borrow navigation patterns from online games: minimap, party
strip, objective list, action dock, loadout, world map, and run result. It must
not use combat power, rarity, currency, loot, levels, or fictional health bars
as substitutes for simulation truth.

Use exact terms in status and evidence:

- `READY`, `RUNNING`, `PAUSED`, `FAILED`, `QUALIFIED`
- sample counts such as `Mid360 126`
- explicit blockers such as `DIGEST DRIFT` or `COOK NOT VERIFIED`
- controller and authority names from the runtime contract

### Quiet until important

Normal operation is visually quiet. Cyan or teal marks navigation and
selection. Green means verified success. Amber means partial, proxy, drift, or
attention required. Red is reserved for failure, unsafe state, destructive
actions, and E-stop.

## 2. Information architecture

The transitional top-level navigation has five destinations:

- **World Preview** — explicitly non-live Web context for prepared sessions and
  authoring ideas; it is not RobotSimUE Runtime and sends no robot input
- **Models** — robots, worlds, sensors, controllers, and scenarios
- **Create Management** — controlled import plus revisioned SceneDraft metadata and
  publication; immersive placement/editing belongs to UE Create
- **Runs** — active and historical sessions
- **Evidence** — recordings, replay, qualification, and release evidence

Existing pages remain functional during migration. Their target mapping is:

- Package Library → Models
- Import Workbench + Scene Layout → Create Management
- Session Composer → prepared-session check / Runs
- Run Monitor → Runs
- Replay Browser → Evidence

## 3. Visual modes

### Lightfield

Default for Models, Create, and run/evidence indexes. It uses warm white and
mist-gray surfaces, graphite text, hairline borders, and generous negative
space. The main object or world preview owns the visual weight.

### Preview overlay

Used over Web-owned map, recording, evidence, or SceneDraft previews. It is never the
RobotSimUE HUD and never overlays or controls a live UE viewport. Overlay pieces are
isolated translucent surfaces; a full-screen dark scrim or permanent opaque panel is
prohibited. Target overlay coverage is below 18% in the resting state.

### Evidence dark

Optional for dense replay, plots, point clouds, and raw engineering detail.
Apply `data-lt-theme="evidence-dark"` at the smallest practical subtree. Dark
mode is a content tool, not the global product identity.

## 4. Token architecture

Tokens are the single visual source of truth and are loaded in this order:

1. `src/design-system/primitives.css` — raw color, type, space, motion, shape
2. `src/design-system/semantic.css` — purpose and theme aliases
3. `src/design-system/components.css` — component-level contracts

New component CSS must use semantic or component tokens. Raw hex, RGB, radius,
shadow, spacing, and transition values are not permitted in new components.
Primitive tokens may only be referenced from the semantic layer. Component
tokens may reference semantic tokens and non-color primitives.

The legacy aliases in `styles.css` temporarily map existing pages to design
tokens. Do not add new legacy aliases. Delete an alias after its last consumer
has migrated.

## 5. Typography

- UI: `--lt-font-ui`; system Segoe UI Variable stack, no new font dependency
- Display: `--lt-font-display`; only for model/world titles
- Data: `--lt-font-data`; identifiers, hashes, time, counts, pose, and rates
- Body text: 14px default
- HUD labels and captions: 11–12px, never below 11px
- Page title: 28px; hero model title may use 40px
- Default weight is 400; 500 for controls; 600 only for short emphasis

Avoid uppercase paragraphs, large bold dashboard headings, and decorative
tracking. Technical IDs remain copyable and must not be truncated without a
reveal or copy affordance.

## 6. Layout and density

- Four-pixel spacing base
- Minimum pointer target: 44px square
- App bar: 56px
- Resting Web preview overlay coverage: less than 18%
- Context drawer: maximum 352px, shown only after selection
- Action dock: 56px, centered and floating
- Asset tray: 120px, visible only in transitional Create Management views
- Main actions per surface: one primary, at most two adjacent secondary actions

Tables belong in deep engineering views. Asset discovery uses preview-led
lists or tiles; manifests and dependency tables open in detail drawers.

## 7. Component contracts

### App bar

Shows product, current context, collaboration/run state, and one primary
action. It must not become a second navigation bar or log surface.

### Run/evidence summary

The resting Web summary may contain a compact objective, map/evidence preview,
current robot identity, event list, and management action dock. It is not the
RobotSimUE HUD, does not receive driving input, and keeps inspectors and replay
timelines closed by default.

### Action dock

Icons must have visible text labels or accessible names. Disabled actions
explain the exact prerequisite through adjacent help or a tooltip. `Reset`,
`Stop`, `Delete`, `Promote`, and `Publish` never share the primary-action style.

### Qualification status strip

The compact strip summarizes Physics, Visual, Sensors, Recording, Replay, and
Shipping. Every state uses icon + text, not color alone. Selecting a state
opens its exact evidence; the summary never fabricates a score.

### Context drawer

Appears after an explicit selection and preserves world context. Escape closes
it and returns focus to the invoking control. It must not contain unrelated
global settings.

### Asset tile

Shows preview, name, type, and one truthful readiness state. Version, digest,
license, dependencies, and validation history remain accessible in detail.

### Notices

Toasts report transient completion and non-blocking warnings. Runtime failure,
unsafe state, destructive confirmation, or integrity drift requiring a choice
must use a persistent inline alert or dialog.

## 8. Interaction and motion

- Hover/color changes: 120ms
- Drawers and trays: 180ms, maximum 8px travel
- Large scene mode transitions: 260ms maximum
- No looping ambient UI motion, pulsing borders, scanlines, or decorative glow
- `prefers-reduced-motion: reduce` collapses tokenized motion to 1ms and zero
  travel
- Loading preserves layout; it does not replace the page with a spinner unless
  no useful content exists

## 9. Accessibility

- Normal text contrast: at least 4.5:1
- Large text and non-text controls: at least 3:1
- Keyboard focus is always visible and uses the shared focus token
- All Web preview and management actions have keyboard-accessible equivalents outside
  the canvas
- Selection, readiness, warning, and failure never rely on color alone
- Drawers and dialogs manage focus and restore it on close
- Live run updates use restrained `aria-live` regions; high-rate telemetry is
  not announced continuously
- Charts and sensor views provide textual summaries
- E-stop remains reachable without opening a drawer or menu

## 10. Engineering-truth rules

1. A running animation is not evidence that a runtime binding is ready.
2. A green sensor state requires a valid source identity and positive sample
   count for the current generation.
3. Visual mesh and physics authority are labeled separately.
4. `qualified` is shown only from authoritative qualification evidence.
5. Digest drift, missing dependencies, proxy sensor fidelity, and Cook status
   are never silently normalized.
6. UI game metaphors do not appear in manifests, API fields, evidence files,
   or logs.
7. Destructive or externally consequential operations require explicit scope,
   target, and confirmation.
8. A Web image, canvas, map, or replay frame is labelled as a preview/evidence view;
   it never claims to be the live UE world, UE Runtime HUD, or UE Create viewport.

## 11. Responsive behavior

- Desktop ≥ 1200px: full preview/evidence canvas or three-zone model configurator
- Compact desktop/tablet 768–1199px: drawers overlay content; asset rail scrolls
- Mobile < 768px: monitoring, evidence review, and safe run controls only;
  geometry authoring is unavailable rather than misleadingly compressed
- Touch controls keep 44px targets and never depend on hover

## 12. Migration acceptance

A migrated surface is complete only when:

- it uses design-system tokens instead of raw visual values;
- existing API operations and error paths remain available;
- keyboard navigation and focus order are verified;
- loading, empty, blocked, warning, failed, and success states are represented;
- a 1366×768 viewport has no clipped primary action;
- reduced motion is respected;
- qualification and integrity status match the backend evidence exactly.

The first implementation slice is the application shell plus Models. World HUD
and Create follow after the shell proves navigation, focus, responsive layout,
and compatibility with the current catalog API.
