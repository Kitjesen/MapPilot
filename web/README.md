# LingTu Web Dashboard

Status: current Web dashboard contract.

LingTu Web Dashboard is the operator UI for the robot-side Gateway. Its default job is to supervise repeatable field-inspection missions, expose trustworthy evidence, and support safe takeover or recovery.

The dashboard's contract is the Gateway API. Field status reaches Gateway through HostBus/native ABI; ModulePorts are only an in-Host detail. Operators do not inspect ROS 2 topics directly.

## Inspection-First Product Contract

The primary workflow is task → route → point → evidence → review or reinspection. The Web surface should answer what the robot is doing, whether it is safe, who owns motion, and what the operator must do next. Process names, transports, topics, and tuning belong in advanced diagnostics.

The current verified inspection evidence is route, point, action, timestamp, RGB, pose, and detections. Thermal, gas, acoustic or partial-discharge sensing, trend analysis, automatic reports, and work-order integration are target capabilities; do not present them as delivered until Gateway/runtime contracts and field evidence exist.

Operators may work in strong sunlight, low light, gloves, dust, rain, cold, GPS-denied spaces, weak connectivity, and outside direct sight of the robot. Motion controls therefore require explicit intent, visible control ownership, actionable blockers, and feedback that distinguishes command acceptance from physical outcome.

## Run Locally

```bash
cd web
npm install
npm run dev
```

The Vite dev server listens on `http://localhost:3000`. It proxies `/api`, `/ws`, `/mcp`, and `/map` to `ROBOT_HOST`.

PowerShell example:

```powershell
$env:ROBOT_HOST = 'ROBOT_IP_OR_HOSTNAME:5050'
npm run dev
```

Bash example:

```bash
ROBOT_HOST=ROBOT_IP_OR_HOSTNAME:5050 npm run dev
```

On the robot, Gateway serves the production build from `web/dist` at `http://<robot>:5050/`.

## Typography and Interaction

The console serves Inter for Latin text and Noto Sans SC for Simplified Chinese
from `public/assets/fonts`, including their licenses. Font loading is same-origin and
does not require a public font CDN; local system fonts remain the fallback.
Shared `rem` typography tokens in `src/App.css` use 13 px captions, 14 px labels,
and 15 px body text at the default browser scale. Coordinates and telemetry use
tabular numbers; technical identifiers may use the monospace stack. Scene, map,
and inspection surfaces inherit the same light/dark palette.

The 2026-10-17 visual pass uses the supplied card-dashboard reference: warm
off-white workspace, white 20–26px rounded panels, soft shadows and neutral
capsule selections. Scene, saved-map controls, operator dock, task/status cards
and Settings share these tokens. Display settings remain collapsible, and Stop
stays independent of panel visibility. See [DESIGN.md](DESIGN.md) for the standard.

Single-point and continuous-point controls show actionable feedback next to the
button. A pending pause, resume, or cancel is a request in progress, not proof of
physical movement or a completed task transition. Keyboard focus remains visible,
and reduced-motion preferences disable decorative transitions and entry effects.

The scene starts with the map itself. Its active map and real/sim context appear
in a small canvas overlay, while connection belongs to the Status inspector.
The inspector follows its content height; Layers contains point counts and path
legends beside the display switches. Normal-state banners and the permanent
map footer are omitted. Actionable disconnect or motion-hold feedback remains
visible inside the canvas even while another inspector tab is open.

## Gateway Contract

The dashboard treats Gateway bootstrap as the contract source:

```text
GET /api/v1/app/bootstrap
GET /api/v1/app/capabilities
GET /api/v1/health
GET /ready
```

`src/services/api.ts` uses bootstrap `links` when available and falls back to the current default paths. Do not add new hardcoded REST paths without also exposing or checking the matching bootstrap link.

Runtime dataflow links report what each product stream can expose through Gateway and ModulePort samples. They are the field inspection surface for "is this data moving?", not a ROS 2 topic browser.

Important current links include:

| Link key | Current purpose |
| --- | --- |
| `events` | SSE telemetry stream |
| `cloud_ws`, `camera_ws` | point cloud and camera WebSocket streams |
| `runtime_dataflow`, `runtime_dataflow_topic`, `runtime_dataflow_subscribe` | Read-only motion-path/topic observability and Gateway SSE subscription plan |
| `visual_servo` | Hot target switch for find/follow/stop inside profiles that load `VisualServoModule` |
| `field_check` | Backend product verdict for field/simulation/non-motion readiness |
| `inspection_acceptance` | Read-only no-motion acceptance summary over saved locations and plan previews |
| `inspection_routes`, `inspection_route_detail` | persisted inspection route list, create/update, detail, and delete |
| `inspection_tasks`, `inspection_task_status`, `inspection_task_pause`, `inspection_task_resume`, `inspection_task_cancel` | one task-addressed inspection lifecycle: submit, read native facts, and request control for that exact task |
| `inspection_status` | native route-store and evidence-worker readiness; not task lifecycle truth |
| `session` | read-only Product and localization status |
| `navigation_status`, `navigation_plan`, `navigation_cancel` | planning and autonomy status |
| `goal`, `stop` | navigation goal and stop command |
| `maps`, `map_save`, `map_points` | map list, current Product map, save, live accumulated cloud JSON points |
| `slam_status`, `localization_relocalize`, `localization_map_tracking` | Native SLAM status, relocalization, and map tracking |
| `routecheck_latest` | latest no-motion route preflight evidence for readiness diagnostics |
| `real_runtime_evidence_latest` | latest real target-platform evidence for explicit field-mode status |

The Dataflow tab uses `runtime_dataflow` only as a read-only view of the declared motion path, Product topics, and currently visible Gateway evidence. It does not orchestrate processes or motion. `runtime_dataflow_topic` inspects one stream, and `runtime_dataflow_subscribe` discovers its filtered Gateway SSE URL. The Product Check strip uses the backend `field_check` verdict instead of recomputing it in the browser. There is no arbitrary ModulePort publish, no arbitrary runtime topic publish, and no motion bypass.

Scene's navigation overlay uses JSON points bound to the active map and live
frame. The independent Maps viewer uses `/api/v1/maps/{name}/pcd` for any saved
map, without activating it or placing the live robot in an unverified frame.

During mapping, Scene offers **遥控**, **保存地图**, and **已存地图**. The map
fills the workspace by default; **视图** holds display controls and **状态**
opens the status/layers/operations sidebar. Measured speed and unavailable or
cached local data remain visible with the sidebar closed. Ordinary mapping
clicks only manipulate the view: no probe marker, diagnostic popover or motion
target is created. Cell probing is available only under the existing
`debug_nav` diagnostics on a fresh local projection. Source context stays inside
the map; detailed data status and interpretation belong to Layers/diagnostics.
Saving waits for
the native save operation to succeed before opening that exact map in Maps.
The whole-map viewer's **已保存** disclosure explains **整图快照**: it covers the saved PCD's
extent, with a bounded point sample, and does not update with subsequent scans.
The map-name selector opens the library; selecting a row closes it. Map
management is under each row's **…** button. The viewer opens in **2D** top
view, with **3D** and fit controls; height/color information is collapsed.
**现场** opens Scene's rolling local map again. Viewing saved maps is also
available in observation mode; save and navigation controls are unavailable there.
There is currently no continuous, unsaved global-map stream. Do not present the
mapd local cloud or local projection as the complete mapping history.

In a navigation session, Scene separates target entry into **单点** and
**连续点** modes. **单点** keeps one target draft: a later click replaces the
draft. Selecting a target automatically previews its path; the main action stays
**开始导航**. Planning status or a short failure reason appears beside that
action, with an explicit retry when needed. Coordinates, speed, and arrival
tolerance are available in details. Preview does not command motion; clicking
**开始导航** submits one external navigation goal. **连续点** keeps an ordered draft in the browser,
bound to the active map ID and content epoch. Map clicks append numbered
`x/y/z` points; the operator can move a point up or down, delete it, or clear
the draft. A continuous route requires at least two points; use **单点** for one
destination. The numbered connector is an editing aid, not a planned path.
Scene does not preview every leg before start; the native executor plans each
leg when that point becomes current.
Changing the active map identity clears that draft.

Scene offers **保存路线** independently of **开始路线**. Give a draft a name and
save it without submitting motion. **任务设置** opens the existing Inspection
workbench after saving. If Start is clicked before saving, Scene saves the draft
and submits that saved revision once. The route remains in the persisted route list; an
unchanged draft may reuse that revision after its task terminates. The native
inspection executor owns point
advancement, arrival handling, and the task result; the browser only follows
the returned `task_id` and sends pause, resume, or cancel commands for that same
task. The single-target speed selector does not configure the route; continuous
execution uses the active runtime's path-following speed limit. A submission
response is an acknowledgement, not route completion. The route snapshot cannot
be edited while its task is running. This interaction contract is not evidence
of MuJoCo or supervised field-motion acceptance.

The console shows the current task, map, localization backend, and readiness.
It does not expose Product lifecycle or copy shell commands. The same console
exposes `visual_servo` find/follow/stop; `find` and `follow`
remain motion-capable commands guarded by Gateway safety policy, while `stop`
only releases visual-servo ownership.

The Inspection tab mounts `InspectionWorkbench`. It lists persisted routes and recently retained tasks, lets the operator explicitly select the task being inspected, loads route revisions, builds routes only from locations bound to the selected map revision, saves or deletes routes, submits a persisted revision as one `task_id`, and shows the bounded native event timeline for that exact task. SSE causes an immediate task refresh but does not itself become task authority. Route editing is no-motion; start and resume are motion-capable; pause and cancel are state-changing controls that reduce or stop motion.

**在地图中查看/编辑** brings a saved route on the active map into Scene. Save
workbench edits before switching. Point actions, dwell, yaw, enabled state and
route policies survive the round trip; saving an edited route advances its
revision. Both workspaces report native `completed_points` separately from the
point currently being executed. Maps has no independent goal-send flow: activate
a map there, then select and preview destinations in Scene.

### Point-cloud viewing height

The saved-map library is the canonical product preview: `/?tab=map&map=MAP_ID`.
The old `/map/viewer` entry redirects there. It reads the native PCD via Gateway,
including `map_id`, positive `content_epoch`, and `frame_id` in response headers;
there is no manual `view.json` in this product path. A selection change aborts
the previous request. Map content-version changes, Product-session changes,
successful same-name saves, and **刷新地图** reload the preview without HTTP cache.
Older map-list responses are ignored after a newer request or session change.
List failures or deleted maps clear the prior preview.

The header's **地图状态** disclosure distinguishes **仅可查看**, **导航数据齐全**,
and **已加载到现场**. It reports whether the saved PCD and planning map exist,
with native-service blockers available under **服务原因**. A PCD-only preview
does not offer navigation. Complete artifacts still require localization and
Scene route preview; an already-loaded map is not evidence of fresh localization
or permission to move. This disclosure reads existing map/session data and makes
no activation or motion request. Switching maps closes the prior disclosure.

The saved-map height slider clips only the display. Picking also excludes
hidden points. The legend reports visible sampled points, total sampled points,
the PCD's declared original point count, and map version/frame. Binary PCD
previews keep the existing 500,000-point budget; ASCII previews keep 300,000.
The live Scene active-map overlay's 30,000-point API is a separate display path.
These budgets do not reduce or modify the saved PCD. The local export viewer
under `tools/visualization/` remains an inspection tool; its service mode opens
this same product page.

Saved-map **点云显示** separates **颜色** (height Z or a uniform color), **点渲染**
(point size and opacity), and an initially collapsed **场景** (background and
coordinate grid). Height palettes are 青蓝 / 冷暖 / 随主题 / 自定义; a labeled ramp
shows the observed map-Z range, independently of the preview cutoff. Custom endpoint
colors appear at the corresponding ramp ends. Uniform color has its own saved
color value, without overwriting custom height colors. Color edits preview
immediately while choosing a color. Point size is 0.5–6 CSS px; opacity is 0–100%.
Background can follow the theme or use light/dark/custom.
**恢复默认显示** restores these defaults without
changing the map, camera or height clip. The browser remembers one display preference
object in `localStorage` (`lingtu.map.cloudStyle`). Changes redraw shader uniforms and
the background; they do not download or parse the PCD again. Point picking uses the
displayed point size and remains disabled at zero opacity. These controls affect the
saved-map preview; Scene's live layers retain their own existing controls.

The saved-map canvas fills the workspace. A small floating view-control group and
height slider replace full-width toolbar/footer rows. **显示设置** opens a compact
floating card. **显示高度** clips the preview at a map-Z upper limit; drag the slider
or enter a metre value and press Enter (or blur) to apply. Empty edits and Escape
restore the previous value, and numeric values clamp to the map's height bounds.
This affects preview only. The display settings are
initially closed, without resizing the canvas or resetting its view.
The card shows color meaning before its style; palette selection uses the browser's
native select. **场景** holds background and coordinate-grid controls. Custom colors
appear only when selected. Escape closes the settings card. Map counts/frame/version
remain under the separate, initially closed bottom-left **地图信息** legend.
The height label is 16px and its editable value 15px. The inspector uses a 16px
title, 14px main controls and 13px section/range text, with a sticky title/reset/close
header during internal scrolling. Reference sources and dimensions are in DESIGN.md.
Floating surfaces fit narrow viewports, scroll when needed and leave the global
Stop control reachable. On narrow screens the legend is hidden while settings are
open so the two cards cannot overlap. The legend retains the full-map color range independently
of the highest-Z display clip.

Layers offers **高度视图 → 所有高度 / 跟随 / 固定**. Navigation views initially
follow the valid robot-body map Z, highlighting 1.0 m above and below it.
Outside this band, point opacity decreases smoothly over 0.6 m to zero;
points beyond that transition are hidden. The window moves with the robot
on stairs and ramps. This is a viewing range, not selective ceiling recognition
or ground-connectivity classification; distant parts of a staircase can be hidden.
Mapping views initially show the full cloud. Each display margin is adjustable
from 0.3 to 3.0 m; both map-Z bounds are shown to two decimals. **固定** freezes
the current center and clips points outside the selected height slice; the
operator can enter another map-Z height. **跟随** returns to the robot. These
are display presets, not robot clearance or floor-identification parameters.

The existing render loop interpolates bounded height changes over 200 ms,
without rebuilding point buffers or causing per-frame React state updates.
Map/pose epoch changes, full-view transitions and reduced-motion preferences
snap to their new range. Missing live pose keeps the last valid range in the
same epoch; before the first valid pose, the full cloud is shown.

The same material effect applies to saved, accumulated and current-scan clouds.
Visible transition points remain available for explicit 3D surface picking and
fit view; fully hidden points are excluded in both **跟随** and **固定**.
Faded points do not write
depth, so a faint ceiling does not mask the points below it. Robot, path and
goal overlays remain visible. Returning to **所有高度** restores normal point opacity
and picking across the source cloud.
Source PCD, OctoMap, SLAM, global planning and local collision inputs are
unchanged. Blank, faint or hidden space is not a traversability statement.
Ordinary goal picking still uses the actual robot-body height even when viewing
another floor in **固定**; use explicit 3D surface picking for a different-height
target and check its Z and planning preview. A narrow **固定** slice can still
cut a staircase; widen the range or use **所有高度** to inspect its full connection.
The viewer preserves observed geometry, but does not fill missing scans,
identify connected ground, infer floor IDs or establish staircase motion capability.

The default inspection view keeps route selection, task selection, confirmed progress, and available task controls together. Route editing, detailed results, evidence, and runtime diagnostics are collapsed until requested; errors and required operator actions remain visible. The global emergency stop remains in the top bar, while the duplicate runtime footer is hidden on this page.

If manual takeover paused a task, the console shows a separate “Release manual control” action. That releases the safety-plane latch only; it never resumes the inspection task implicitly. The operator then submits the selected task's resume command and waits for a native event. A native command rejection is shown as a task conflict, not as a false claim that the endpoint is down.

In the **nav** scene, open **场景设置 → 人工接管** to use the existing keyboard
panel without switching Product or restarting navigation. Opening it only
prepares the connection. A new W/S, A/D or Q/E key requests manual motion;
the native endpoint pauses and retains the navigation destination, and its
local planner continues collision checking. The autonomy-resume marker does
not interrupt the operator's ongoing input. Independent safety/input holds
remain effective. Releasing the key, losing focus or exiting the panel holds
motion; the same task card's **继续导航** explicitly continues the retained goal.
Resume reconnects the retained 3D route from the next measured pose, including
after manual backward or lateral movement, while preserving route elevation,
arrival settings and speed. This interaction does not integrate the factory
hand controller or establish field takeover acceptance.

After an ordinary workspace remount or full-page reload, re-entering **连续点**
queries the active map's task projection to restore route controls. It selects a task
only when exactly one nonterminal task exists; multiple candidates remain
unselected. A disconnect preserves an already known `task_id`, disables its
controls, and resumes polling that same task after reconnect. This recovery is
still bounded by Gateway/native task projection. After a Gateway or endpoint
restart, Scene can recover only a nonterminal task that the projection reports;
it does not synthesize state or start a route automatically. There is not yet a
durable task journal or event replay guarantee. Voice/semantic tour start
submits a task, while pause, resume, and cancel require an explicitly selected
task in the console rather than guessing a global "current tour".

`inspection_acceptance` remains a read-only, no-motion diagnostic contract and the browser must not recompute its PASS/FAIL verdict. The operator workbench currently shows checksum-verified RGB, pose, and detection artifacts. It must label missing, stale, unsupported, or invalid evidence instead of implying that planned thermal, gas, acoustic, partial-discharge, reporting, or work-order capabilities are already available.

## Safety Classes

Read-only UI actions:

- Open dashboard, status cards, topbar, Scene view, Map preview, SLAM status.
- Open the Dataflow tab and inspect runtime stream summary/detail.
- Open the Inspection tab to review route definitions, current mission status, and verified evidence; run the separate read-only acceptance diagnostic.
- SSE `/api/v1/events`, point cloud `/ws/cloud`, camera `/ws/camera`.
- Gateway bootstrap, health, readiness, runtime dataflow, map list, map points, navigation status.

State-changing but no robot motion:

- Save, rename, or delete a map.
- Create, update, or delete an inspection route.
- Pause or cancel an inspection mission; these controls may reduce or stop existing motion but never initiate it.
- Manual or auto relocalization.
- Reset accumulated map cloud.
- Execute a product mode switch that only plans, stops current motion, or cold-restarts services without publishing a navigation goal.
- Send Visual Servo `stop`.

Robot motion capable:

- Send navigation goal from Scene view or slash command.
- Start exploration after safety/session gates pass.
- Start or resume a persisted inspection route revision.
- Teleop or any command that enters autonomous motion.
- Execute a product mode that starts an exploration/navigation behavior, or send Visual Servo `find`/`follow`.

Saved-map preview does not activate a map or relocalize the robot. Product
lifecycle changes remain outside this dashboard and use ProductControl.
Goal and inspection motion controls must remain disabled when Gateway reports
readiness blockers, localization loss, missing odometry, unsupported evidence
actions, or an incompatible control owner.

Command acceptance is not physical outcome confirmation. A command toast reports submitted, rejected, or failed; task state must then be confirmed by the native inspection event stream. In particular, `CANCELLED` is valid only after the native endpoint records its stop evidence.

State-changing communication is limited to Gateway's whitelisted commands, such as goal, stop, map, and SLAM operations. The dashboard must not provide arbitrary publish into ModulePorts or runtime topics.

## Sunrise No-Motion Smoke

Use `NO_PROXY` on sunrise because local Gateway requests can otherwise be routed through the configured HTTP proxy.

```bash
export LINGTU_ROBOT_HOST=ROBOT_IP_OR_HOSTNAME
export GATEWAY_URL="http://${LINGTU_ROBOT_HOST}:5050"
NO_PROXY=127.0.0.1,localhost,"$LINGTU_ROBOT_HOST" \
no_proxy=127.0.0.1,localhost,"$LINGTU_ROBOT_HOST" \
curl -sS "${GATEWAY_URL}/api/v1/app/bootstrap" >/tmp/lingtu_bootstrap.json
```

From this repo, the read-only contract smoke is:

```bash
cd web
GATEWAY_URL=http://ROBOT_IP_OR_HOSTNAME:5050 npm run smoke:gateway
```

If you are running it directly on sunrise, prefer:

```bash
cd ~/data/inovxio/lingtu/web
NO_PROXY=127.0.0.1,localhost \
no_proxy=127.0.0.1,localhost \
GATEWAY_URL=http://127.0.0.1:5050 npm run smoke:gateway
```

## Validate

```bash
cd web
npm run lint
npm run smoke:dataflow-ui
npx tsc -b --pretty false
npm run build
```

`npm run smoke:gateway` is intentionally not part of build because it requires a live Gateway.
`npm run smoke:dataflow-ui` is a dependency-free static contract check for the read-only Dataflow and inspection-acceptance surfaces.

## Source Map

| Path | Purpose |
| --- | --- |
| `src/App.tsx` | top-level tab layout and bootstrapping |
| `src/hooks/useSSE.ts` | bootstrap, capabilities, traffic, and telemetry stream |
| `src/services/api.ts` | centralized REST client and command receipt formatting |
| `src/components/SceneView.tsx` | 3D scene, single-target and continuous-route controls, saved map drawer, map/SLAM actions |
| `src/services/routeDraft.ts` | ordered continuous-route draft operations and one save-then-start submission |
| `src/components/MapView.tsx` | saved map CRUD and point cloud preview |
| `src/components/RobotStatusPanel.tsx` | current Product, environment, readiness, localization, and map status |
| `src/components/SlamStatusPanel.tsx` | read-only native SLAM metrics and relocalization action |
| `src/components/RuntimeDataflowView.tsx` | read-only Product motion/topic observability |
| `src/components/InspectionWorkbench.tsx` | persisted inspection route editing, native mission control, status, and verified evidence review |
| `scripts/gateway-smoke.mjs` | read-only Gateway contract smoke |

## Notes

- Keep command buttons bound to Gateway readiness, session gates, and control ownership.
- Keep no-motion actions explicit and confirmed when they alter robot state.
- Do not send a navigation goal until localization is tracking and `/ready` is 200.
- Treat ROS 2/GZ/simulation topics as adapter details; the dashboard entry point remains the Gateway API.


### Scene workspace controls

The map toolbar toggles the inspector and camera. Scene opens the camera preview
by default; closing it releases its WHEP peer, JPEG WebSocket and camera-status polling.
Reconnection preserves a manually closed preview. This does not start the camera
process or recording. Scene and `CameraFeed` share the WHEP lifecycle hooks and
use `/ws/camera` (binary JPEG frames) until a WebRTC frame is actually decoded.
`GET /api/v1/webrtc/status` reports the selected backend, ICE servers and physical
camera rotation. `POST /api/v1/webrtc/whep` negotiates the stream; the returned
session Location is deleted on closure. Missing initial frames and three-second
freezes cause reconnection and JPEG fallback. Quarter-turn mounts are fitted
without cropping. Closing the inspector gives its width back to the map. Layers
provides neutral/height point coloring, which changes rendering only.

Single-goal selection and execution use the same bottom-left task area. Target
settings hold speed, arrival radius and height; **清除目标** removes the draft,
while **取消任务** addresses an already submitted task. **停止** remains fixed
outside the collapsible launcher. Execution state comes from the existing task
endpoint, not from accepting the Start request. In continuous-point mode, click
a numbered list entry to center the view; this does not submit a motion goal.

### Reproduce the local video fixture

`tests/fixtures/video-check.html` mounts the real `CameraFeed` and shows browser
decode/connection counters. Its source is a continuous synthetic red 16×16 SHM
frame; its independent JPEG fallback is blue. This verifies software integration
and recovery, not camera capture, hardware encoding or throughput. The fixture
does not start Host, Product processes or a physical camera.

From the repository root in WSL/Linux, build with the pinned runtime prepared by
[`prepare_camera_video.sh`](../scripts/build/prepare_camera_video.sh), then run the
source in its own terminal:

```bash
source build/deps/video/prefix/env.sh
cmake -S src/drivers/real/camera/native/video -B build/camera_video_check -DBUILD_TESTING=ON
cmake --build build/camera_video_check --target video_fixture lingtu_camera_webrtc
printf '501\n' > build/video-check-control.txt
python3 web/tests/fixtures/video-check.py source \
  --fixture build/camera_video_check/video_fixture --control build/video-check-control.txt
```

In another WSL/Linux terminal at the repository root, start the native media
endpoint explicitly with the software encoder:

```bash
source build/deps/video/prefix/env.sh
build/camera_video_check/lingtu_camera_webrtc \
  --color-shm /lingtu_video_browser_check --encoder x264enc \
  --listen http://127.0.0.1:9090 --idle-timeout-ms 60000
```

At the repository root in Windows PowerShell, run the fixture Gateway, then Vite
in another terminal:

```powershell
.venv/Scripts/python.exe web/tests/fixtures/video-check.py gateway `
  --port 5094 --native-endpoint http://127.0.0.1:9090/whep/endpoint
```

```powershell
$env:ROBOT_HOST = '127.0.0.1:5094'
cd web
npm run dev -- --host 127.0.0.1 --port 4194
```

Open `http://127.0.0.1:4194/tests/fixtures/video-check.html`. On Windows/WSL,
HTTP localhost forwarding alone does not establish ICE reachability: use a WSL
network route/address reachable by the browser when inspecting candidate pairs.
Write `pause` to the control file to stop new frames, `502` to resume with a new
source generation, or `quit` to stop the source and remove its SHM. Use the page's
Stop/Start button to inspect peer cleanup and reconnection. Stop the native
endpoint to inspect the blue JPEG fallback, then restart it to inspect recovery.
Close these foreground processes with Ctrl+C when finished.

Replace the control file atomically when changing commands, especially across
Windows/WSL, so the source never reads a partially written generation or command:

```bash
printf 'pause\n' > build/video-check-control.next
mv build/video-check-control.next build/video-check-control.txt
```

For source-health checks, pass `--status-file /absolute/path/video-status.json`
to the native program and `--native-status-file D:/absolute/path/video-status.json`
to the fixture Gateway (use paths appropriate to each OS). Managed Product runs
provide this path through the session. Their status response distinguishes
`whep_available` from `source_ready`; the browser negotiates only when both are ready.
