# LingTu Web Dashboard

Status: current Web dashboard contract as of 2026-07-25.

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
| `real_runtime_evidence_latest` | latest real S100P runtime evidence for explicit field-mode status |

The Dataflow tab uses `runtime_dataflow` only as a read-only view of the declared motion path, Product topics, and currently visible Gateway evidence. It does not orchestrate processes or motion. `runtime_dataflow_topic` inspects one stream, and `runtime_dataflow_subscribe` discovers its filtered Gateway SSE URL. The Product Check strip uses the backend `field_check` verdict instead of recomputing it in the browser. There is no arbitrary ModulePort publish, no arbitrary runtime topic publish, and no motion bypass.

Saved-map previews use saved-map JSON points separately from raw saved-map PCD; `/api/v1/maps/{name}/pcd` is the raw PCD endpoint.

The console shows the current task, map, localization backend, and readiness.
It does not expose Product lifecycle or copy shell commands. The same console
exposes `visual_servo` find/follow/stop; `find` and `follow`
remain motion-capable commands guarded by Gateway safety policy, while `stop`
only releases visual-servo ownership.

The Inspection tab mounts `InspectionWorkbench`. It lists persisted routes and recently retained tasks, lets the operator explicitly select the task being inspected, loads route revisions, builds routes only from locations bound to the selected map revision, saves or deletes routes, submits a persisted revision as one `task_id`, and shows the bounded native event timeline for that exact task. SSE causes an immediate task refresh but does not itself become task authority. Route editing is no-motion; start and resume are motion-capable; pause and cancel are state-changing controls that reduce or stop motion.

If manual takeover paused a task, the console shows a separate “Release manual control” action. That releases the safety-plane latch only; it never resumes the inspection task implicitly. The operator then submits the selected task's resume command and waits for a native event. A native command rejection is shown as a task conflict, not as a false claim that the endpoint is down.

The dashboard can rehydrate an unresolved task only from Gateway's bounded, process-local projection. After a Gateway or endpoint restart it must show the continuity interruption and keep task controls disabled until native truth is available; there is not yet a durable task journal or event replay guarantee. Voice/semantic tour start submits a task, while pause, resume, and cancel require an explicitly selected task in the console rather than guessing a global "current tour".

`inspection_acceptance` remains a read-only, no-motion diagnostic contract and the browser must not recompute its PASS/FAIL verdict. The operator workbench currently shows checksum-verified RGB, pose, and detection artifacts. It must label missing, stale, unsupported, or invalid evidence instead of implying that planned thermal, gas, acoustic, partial-discharge, reporting, or work-order capabilities are already available.

## Safety Classes

Read-only UI actions:

- Open dashboard, status cards, topbar, Scene view, Map preview, SLAM status.
- Open the Dataflow tab and inspect runtime stream summary/detail.
- Open the Inspection tab to review route definitions, current mission status, and verified evidence; run the separate read-only acceptance diagnostic.
- Open the Runtime tab and run product switch Preflight.
- SSE `/api/v1/events`, point cloud `/ws/cloud`, camera `/ws/camera`.
- Gateway bootstrap, health, readiness, runtime dataflow, map list, map points, navigation status.

State-changing but no robot motion:

- Copy ProductControl switch or stop commands for mapping, navigation, or exploration.
- Save, activate, rename, or delete a map.
- Create, update, or delete an inspection route.
- Pause or cancel an inspection mission; these controls may reduce or stop existing motion but never initiate it.
- Switch SLAM mode.
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

The UI confirms map activation and saved-map load/relocalize. Goal and inspection motion controls must remain disabled when Gateway reports readiness blockers, localization loss, missing odometry, unsupported evidence actions, or an incompatible control owner.

Command acceptance is not physical outcome confirmation. A command toast reports submitted, rejected, or failed; task state must then be confirmed by the native inspection event stream. In particular, `CANCELLED` is valid only after the native endpoint records its stop evidence.

State-changing communication is limited to Gateway's whitelisted commands, such as goal, stop, map, and SLAM operations. Product lifecycle changes are copied as ProductControl commands for the operator; the dashboard must not provide arbitrary publish into ModulePorts or runtime topics.

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
| `src/components/SceneView.tsx` | 3D scene, goal placement, saved map drawer, map/SLAM actions |
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
