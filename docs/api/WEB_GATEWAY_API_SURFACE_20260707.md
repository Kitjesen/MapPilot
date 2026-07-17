# Web Gateway API Surface

Date: 2026-07-07

Robot tested: `192.168.66.13:5050`

This document records the Web-facing Gateway API surface exposed to the LingTu
dashboard. It is based on the live `openapi.json` route list plus the fallback
URLs declared in `web/src/services/api.ts`.

## Current Result

- Backend OpenAPI paths: 88
- Web-declared API URLs: 57
- Web URLs missing from OpenAPI: 0
- Safe live smoke checks: passed
- Camera snapshot and camera WebSocket: route exists, but the current robot
  runtime reports `camera_unavailable` / `no_color_frames`.
- Risky state-changing commands were not executed during the smoke pass:
  navigation goals, stop/mode/lease, session start/end, SLAM switch/restart,
  auto relocalize, active-map switch, map-cloud reset, and bag recording.

## App And State

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/app/bootstrap` | Initial Web/App payload | Pass |
| GET | `/api/v1/app/capabilities` | Endpoint and feature discovery | Pass |
| GET | `/api/v1/app/traffic` | Client polling/backpressure policy | Pass |
| GET | `/api/v1/state` | Full state snapshot | Pass |
| GET | `/api/v1/health` | Operator health snapshot | Pass |
| GET | `/api/v1/metrics` | Compact operator metrics snapshot | Pass |
| GET | `/api/v1/readiness` | Client readiness details | Pass |
| GET | `/health` | Liveness probe | Pass |
| GET | `/ready` | Readiness probe | Available |

## Runtime And Diagnostics

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/runtime/dataflow` | Runtime dataflow snapshot | Pass |
| GET | `/api/v1/runtime/dataflow/topic` | Topic detail | Pass |
| POST | `/api/v1/runtime/dataflow/subscribe` | Topic subscription declaration | Pass |
| POST | `/api/v1/runtime/switch-plan` | Dry-run runtime switch plan | Pass |
| POST | `/api/v1/runtime/switch` | Runtime switch, dry-run by default | Pass |
| GET | `/api/v1/diagnostics/routecheck/latest` | Latest routecheck report | Pass |
| GET | `/api/v1/diagnostics/real-runtime-evidence/latest` | Latest field evidence report | Pass |
| GET | `/api/v1/diagnostics/algorithm-benchmark/latest` | Latest benchmark report | Pass |
| GET | `/api/v1/diagnostics/plugins` | Plugin/backend diagnostics | Pass |
| GET | `/api/v1/diagnostics/runtime-contract` | Runtime contract summary | Pass |
| POST | `/api/v1/diagnostics/field-check` | Field/simulation readiness check | Pass |
| POST | `/api/v1/inspection/acceptance` | Inspection acceptance gate | Pass |
| GET | `/api/v1/diagnostic_pack` | Gzip diagnostic pack | Pass |

## Navigation And Control

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/navigation/status` | Navigation mission status | Pass |
| GET | `/api/v1/path` | Current path | Pass |
| POST | `/api/v1/navigation/goal_candidate` | No-motion goal preview | Pass |
| POST | `/api/v1/navigation/plan` | No-motion plan preview | Pass |
| POST | `/api/v1/goal` | Dispatch navigation goal | Skipped: motion/state-changing |
| POST | `/api/v1/navigate/click` | Dispatch clicked goal | Skipped: motion/state-changing |
| POST | `/api/v1/navigation/cancel` | Cancel navigation | Skipped: state-changing |
| POST | `/api/v1/stop` | Stop command | Skipped: state-changing |
| POST | `/api/v1/mode` | Runtime mode command | Skipped: state-changing |
| POST | `/api/v1/lease` | Command lease | Skipped: state-changing |

## SLAM And Localization

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/localization/status` | Localization status | Pass |
| GET | `/api/v1/slam/status` | SLAM service status | Pass |
| POST | `/api/v1/slam/relocalize` | Manual saved-map relocalize | Negative pass with missing map |
| POST | `/api/v1/slam/track_against_map` | Continuous saved-map tracking | Negative pass with missing map |
| POST | `/api/v1/slam/switch` | Switch SLAM profile | Skipped: state-changing |
| POST | `/api/v1/slam/restart` | Restart native SLAM/localization | Skipped: state-changing |
| POST | `/api/v1/slam/auto_relocalize` | 3D-BBS global relocalize | Skipped: state-changing |

Current robot service status during test:

- Product path: `native_dds`
- Running: `lingtu-livox-dds`, `lingtu-slam-dds`, `lingtu-nav-dds`, `lingtu.service`
- Stopped: `slam_pgo`, `hba`, `localizer`, `genz_icp`, `super_lio`, `super_lio_relocation`

## Maps

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/slam/maps` | List saved maps | Pass |
| GET | `/api/v1/map/points` | Live accumulated map cloud | Pass |
| GET | `/api/v1/maps/{name}/points` | Saved map points | Pass |
| GET | `/api/v1/maps/{name}/pcd` | Raw saved PCD | Pass |
| GET | `/api/v1/maps/{name}/voxels/edits` | Voxel edit overlay | Pass |
| POST | `/api/v1/map/save` | Save current SLAM map | Pass with temp map |
| POST | `/api/v1/maps` | Legacy map lifecycle | Pass for temp delete |
| POST | `/api/v1/maps/import_pcd` | Import PCD into map package | Pass with temp PCD |
| POST | `/api/v1/map/rename` | Rename saved map | Pass with temp map |
| POST | `/api/v1/maps/{name}/crop` | Crop saved map PCD | Pass with temp map |
| POST | `/api/v1/maps/{name}/build_octomap` | Build OctoMap artifact | Pass with temp map |
| POST | `/api/v1/maps/{name}/mark_zone` | Mark saved-map voxels | Pass with temp map |
| POST | `/api/v1/maps/{name}/voxels/edit` | Edit saved-map voxels | Covered by mark-zone flow |
| POST | `/api/v1/map/activate` | Switch active map | Skipped: state-changing |
| POST | `/api/v1/map_cloud/reset` | Clear live visualization cloud | Skipped: state-changing |
| POST | `/api/v1/map/restore_predufo` | Restore pre-DUFOMap backup | Skipped: destructive to selected map |

The save button path was tested with `codex_api_save_*`; the saved map exposed
non-empty points and was deleted afterward.

## Media And Realtime

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/events` | SSE event stream | Pass |
| WS | `/ws/cloud` | Binary point cloud stream | Pass |
| WS | `/ws/camera` | Camera frame stream | Route exists; no frames in current runtime |
| GET | `/api/v1/camera/snapshot` | Camera snapshot | Expected 503: `no_color_frames` |
| GET | `/api/v1/webrtc/stats` | WebRTC stats | Pass |
| GET | `/api/v1/webrtc/go2rtc/status` | go2rtc status | Pass |
| POST | `/api/v1/webrtc/offer` | WebRTC offer | Negative pass with invalid SDP |
| POST | `/api/v1/webrtc/whep` | WHEP proxy | Negative pass when sidecar unavailable |
| POST | `/api/v1/webrtc/bitrate` | Set WebRTC bitrate | Not exercised; requires active WebRTC |

## Memory, Locations, And Auth

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/locations` | List saved locations | Pass |
| POST | `/api/v1/locations` | Create location | Pass with temp location |
| PUT | `/api/v1/locations/{name}` | Update location | Pass with temp location |
| DELETE | `/api/v1/locations/{name}` | Delete location | Pass with temp location |
| GET | `/api/v1/memory/temporal` | Temporal observations | Pass |
| POST | `/api/v1/memory/temporal/semantic` | Semantic temporal query | Pass with embedding |
| GET | `/api/v1/auth/check` | Auth mode check | Pass |
| POST | `/api/v1/auth/login` | Login/API key check | Pass; auth disabled on tested robot |

## Recording And Exploration

| Method | Path | Purpose | Smoke |
| --- | --- | --- | --- |
| GET | `/api/v1/bag/status` | Bag recording status | Pass |
| POST | `/api/v1/bag/start` | Start recording | Skipped: state-changing |
| POST | `/api/v1/bag/stop` | Stop recording | Skipped: state-changing |
| GET | `/api/v1/explore/status` | Exploration status | Available |
| POST | `/api/v1/explore/start` | Start exploration | Skipped: motion/state-changing |
| POST | `/api/v1/explore/stop` | Stop exploration | Skipped: state-changing |
