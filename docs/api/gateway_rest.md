# Gateway REST API

> Auto-generated from route registrations in `src/gateway/routes/`.
> Generated: 2026-06-02 01:58:19

The GatewayModule serves these endpoints via FastAPI on port 5050.

---

## Summary

- **src\gateway\gateway_module.py**:
  - `POST /api/v1/driver/swap` — Swap the active driver backend at runtime
  - `POST /api/v1/maps` — Map lifecycle management
  - `POST /api/v1/runtime/backend` — 
- **src\gateway\routes\app.py**:
  - `GET /api/v1/app/bootstrap` — App/Web bootstrap snapshot
  - `GET /api/v1/app/capabilities` — App/Web API capability manifest
  - `GET /api/v1/app/traffic` — App/Web realtime traffic and client polling policy
  - `GET /api/v1/bootstrap` — 
- **src\gateway\routes\auth.py**:
  - `GET /api/v1/auth/check` — Check if auth is required
  - `POST /api/v1/auth/login` — Login with API key
- **src\gateway\routes\camera.py**:
  - `GET /api/v1/camera/snapshot` — Camera JPEG snapshot
- **src\gateway\routes\commands.py**:
  - `POST /api/v1/cmd_vel` — Direct velocity command
  - `POST /api/v1/goal` — Send navigation goal
  - `POST /api/v1/instruction` — Natural language navigation instruction
  - `POST /api/v1/lease` — Acquire/release/renew control lease
  - `POST /api/v1/mode` — Switch operating mode
  - `POST /api/v1/navigate/click` — Navigate to map-viewer click point
  - `POST /api/v1/navigation/cancel` — Gracefully cancel current navigation mission
  - `POST /api/v1/navigation/goal_candidate` — Construct and optionally preview a navigation goal without publishing it
  - `POST /api/v1/navigation/plan` — Preview navigation plan without publishing a goal
  - `POST /api/v1/stop` — Emergency stop
- **src\gateway\routes\diagnostics.py**:
  - `GET /api/v1/diagnostic_pack` — Export diagnostic tarball
  - `GET /api/v1/diagnostics/algorithm-benchmark/latest` — Read latest read-only algorithm benchmark summary
  - `POST /api/v1/diagnostics/field-check` — Run read-only product field readiness check
  - `GET /api/v1/diagnostics/plugins` — Read registered plugin categories and providers
  - `GET /api/v1/diagnostics/real-runtime-evidence/latest` — Read latest real S100P runtime evidence gate summary
  - `GET /api/v1/diagnostics/routecheck/latest` — Read latest non-motion routecheck summary
  - `GET /api/v1/diagnostics/runtime-contract` — Read canonical runtime interface contract
  - `POST /api/v1/inspection/acceptance` — Run read-only inspection acceptance without publishing motion commands
- **src\gateway\routes\maps.py**:
  - `POST /api/v1/map/activate` — Set active map (symlink)
  - `GET /api/v1/map/points` — Map point cloud as JSON (from ikd-tree snapshot)
  - `POST /api/v1/map/rename` — Rename a saved map
  - `POST /api/v1/map/restore_predufo` — Restore map.pcd from DUFOMap pre-filter backup
  - `POST /api/v1/map/save` — Save current SLAM map
  - `POST /api/v1/map_cloud/reset` — Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)
  - `GET /api/v1/maps/{name}/pcd` — Serve raw PCD file for inline preview
  - `GET /api/v1/maps/{name}/points` — Saved map point cloud as JSON
  - `GET /api/v1/slam/maps` — List maps from filesystem
  - `GET /map/viewer` — Interactive 3D map viewer
  - `GET /robot/meshes/{filename}` — Serve robot STL mesh files
- **src\gateway\routes\operations.py**:
  - `POST /api/v1/bag/start` — Start rosbag recording
  - `GET /api/v1/bag/status` — rosbag recording status
  - `POST /api/v1/bag/stop` — Stop rosbag recording
  - `POST /api/v1/explore/start` — Start autonomous frontier exploration
  - `GET /api/v1/explore/status` — Exploration status
  - `POST /api/v1/explore/stop` — Stop autonomous frontier exploration
  - `GET /api/v1/memory/temporal` — Query temporal entity observations
  - `POST /api/v1/memory/temporal/semantic` — Semantic similarity search over temporal observations
  - `POST /api/v1/slam/auto_relocalize` — Global relocalize via 3D-BBS (no guess required)
  - `POST /api/v1/slam/relocalize` — Relocalize against a saved map
  - `GET /api/v1/slam/status` — SLAM service status
  - `POST /api/v1/slam/switch` — Hot-switch SLAM profile
  - `POST /api/v1/webrtc/bitrate` — Live-tune WebRTC max bitrate without reconnect
  - `GET /api/v1/webrtc/go2rtc/status` — Probe the go2rtc sidecar (image transmission fast path)
  - `POST /api/v1/webrtc/offer` — WebRTC SDP offer/answer exchange for low-latency camera
  - `GET /api/v1/webrtc/stats` — WebRTC peer telemetry (bitrate, fps, encode time)
  - `POST /api/v1/webrtc/whep` — WHEP signalling proxy to go2rtc (image transmission path)
- **src\gateway\routes\session.py**:
  - `GET /api/v1/session` — Current session state + capabilities
  - `POST /api/v1/session/end` — Exit current mode and return to idle
  - `POST /api/v1/session/start` — Enter a low-level mapping, navigating, or exploring session
- **src\gateway\routes\status.py**:
  - `GET /api/v1/devices` — Hardware device registry status
  - `GET /api/v1/events` — SSE event stream
  - `GET /api/v1/health` — System health overview
  - `GET /api/v1/localization/status` — Localization status for app and web clients
  - `GET /api/v1/locations` — List tagged navigation locations
  - `POST /api/v1/locations` — Create or update a tagged navigation location
  - `PUT /api/v1/locations/{name}` — Update a tagged navigation location
  - `DELETE /api/v1/locations/{name}` — Delete a tagged navigation location
  - `GET /api/v1/navigation` — 
  - `GET /api/v1/navigation/status` — Navigation mission and control status
  - `GET /api/v1/path` — Latest planned path
  - `GET /api/v1/readiness` — Client readiness snapshot
  - `GET /api/v1/runtime/dataflow` — Runtime dataflow and Module port observability
  - `POST /api/v1/runtime/dataflow/subscribe` — Create a read-only runtime dataflow SSE subscription plan
  - `GET /api/v1/runtime/dataflow/topic` — Inspect one runtime dataflow topic
  - `POST /api/v1/runtime/switch-plan` — Dry-run runtime endpoint switch plan
  - `GET /api/v1/scene_graph` — Current scene graph
  - `GET /api/v1/state` — Full robot state snapshot
  - `GET /health` — Liveness probe
  - `GET /ready` — Readiness probe

---

## src\gateway\gateway_module.py

### `POST /api/v1/driver/swap`
**Summary:** Swap the active driver backend at runtime
**Response model:** `DriverSwapResponse`
**Handler:** `post_driver_swap`

### `POST /api/v1/maps`
**Summary:** Map lifecycle management
**Response model:** `MapLifecycleResponse`
**Handler:** `post_maps`

### `POST /api/v1/runtime/backend`
**Handler:** `post_runtime_backend`

## src\gateway\routes\app.py

### `GET /api/v1/app/bootstrap`
**Summary:** App/Web bootstrap snapshot
**Response model:** `AppBootstrapResponse`
**Handler:** `app_bootstrap`

### `GET /api/v1/app/capabilities`
**Summary:** App/Web API capability manifest
**Response model:** `AppCapabilitiesResponse`
**Handler:** `app_capabilities`

### `GET /api/v1/app/traffic`
**Summary:** App/Web realtime traffic and client polling policy
**Response model:** `AppTrafficResponse`
**Handler:** `app_traffic`

### `GET /api/v1/bootstrap`
**Response model:** `AppBootstrapResponse`
**Handler:** `app_bootstrap_legacy_alias`

## src\gateway\routes\auth.py

### `GET /api/v1/auth/check`
**Summary:** Check if auth is required
**Response model:** `AuthCheckResponse`
**Handler:** `auth_check`

### `POST /api/v1/auth/login`
**Summary:** Login with API key
**Response model:** `AuthLoginResponse`
**Handler:** `auth_login`

## src\gateway\routes\camera.py

### `GET /api/v1/camera/snapshot`
**Summary:** Camera JPEG snapshot
**Handler:** `camera_snapshot`

## src\gateway\routes\commands.py

### `POST /api/v1/cmd_vel`
**Summary:** Direct velocity command
**Response model:** `ControlCommandResponse`
**Handler:** `post_cmd_vel`

### `POST /api/v1/goal`
**Summary:** Send navigation goal
**Response model:** `ControlCommandResponse`
**Handler:** `post_goal`

### `POST /api/v1/instruction`
**Summary:** Natural language navigation instruction
**Response model:** `ControlCommandResponse`
**Handler:** `post_instruction`

### `POST /api/v1/lease`
**Summary:** Acquire/release/renew control lease
**Response model:** `LeaseResponse`
**Handler:** `post_lease`

### `POST /api/v1/mode`
**Summary:** Switch operating mode
**Response model:** `ControlCommandResponse`
**Handler:** `post_mode`

### `POST /api/v1/navigate/click`
**Summary:** Navigate to map-viewer click point
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigate_click`

### `POST /api/v1/navigation/cancel`
**Summary:** Gracefully cancel current navigation mission
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_cancel`

### `POST /api/v1/navigation/goal_candidate`
**Summary:** Construct and optionally preview a navigation goal without publishing it
**Response model:** `GoalCandidateResponse`
**Handler:** `post_navigation_goal_candidate`

### `POST /api/v1/navigation/plan`
**Summary:** Preview navigation plan without publishing a goal
**Response model:** `PlanPreviewResponse`
**Handler:** `post_navigation_plan`

### `POST /api/v1/stop`
**Summary:** Emergency stop
**Response model:** `ControlCommandResponse`
**Handler:** `post_stop`

## src\gateway\routes\diagnostics.py

### `GET /api/v1/diagnostic_pack`
**Summary:** Export diagnostic tarball
**Handler:** `diagnostic_pack`

### `GET /api/v1/diagnostics/algorithm-benchmark/latest`
**Summary:** Read latest read-only algorithm benchmark summary
**Response model:** `AlgorithmBenchmarkLatestResponse`
**Handler:** `algorithm_benchmark_latest`

### `POST /api/v1/diagnostics/field-check`
**Summary:** Run read-only product field readiness check
**Response model:** `ProductFieldCheckResponse`
**Handler:** `product_field_check`

### `GET /api/v1/diagnostics/plugins`
**Summary:** Read registered plugin categories and providers
**Handler:** `plugin_catalog`

### `GET /api/v1/diagnostics/real-runtime-evidence/latest`
**Summary:** Read latest real S100P runtime evidence gate summary
**Response model:** `RealRuntimeEvidenceLatestResponse`
**Handler:** `real_runtime_evidence_latest`

### `GET /api/v1/diagnostics/routecheck/latest`
**Summary:** Read latest non-motion routecheck summary
**Response model:** `RoutecheckLatestResponse`
**Handler:** `routecheck_latest`

### `GET /api/v1/diagnostics/runtime-contract`
**Summary:** Read canonical runtime interface contract
**Response model:** `RuntimeContractResponse`
**Handler:** `runtime_contract`

### `POST /api/v1/inspection/acceptance`
**Summary:** Run read-only inspection acceptance without publishing motion commands
**Response model:** `InspectionAcceptanceResponse`
**Handler:** `inspection_acceptance`

## src\gateway\routes\maps.py

### `POST /api/v1/map/activate`
**Summary:** Set active map (symlink)
**Response model:** `MapLifecycleResponse`
**Handler:** `activate_map`

### `GET /api/v1/map/points`
**Summary:** Map point cloud as JSON (from ikd-tree snapshot)
**Response model:** `MapPointsResponse`
**Handler:** `get_map_points`

### `POST /api/v1/map/rename`
**Summary:** Rename a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `rename_map`

### `POST /api/v1/map/restore_predufo`
**Summary:** Restore map.pcd from DUFOMap pre-filter backup
**Response model:** `MapLifecycleResponse`
**Handler:** `restore_predufo`

### `POST /api/v1/map/save`
**Summary:** Save current SLAM map
**Response model:** `MapLifecycleResponse`
**Handler:** `save_map_now`

### `POST /api/v1/map_cloud/reset`
**Summary:** Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)
**Response model:** `MapLifecycleResponse`
**Handler:** `reset_map_cloud`

### `GET /api/v1/maps/{name}/pcd`
**Summary:** Serve raw PCD file for inline preview
**Handler:** `get_map_pcd`

### `GET /api/v1/maps/{name}/points`
**Summary:** Saved map point cloud as JSON
**Response model:** `MapPointsResponse`
**Handler:** `get_saved_map_points`

### `GET /api/v1/slam/maps`
**Summary:** List maps from filesystem
**Response model:** `MapListResponse`
**Handler:** `slam_maps`

### `GET /map/viewer`
**Summary:** Interactive 3D map viewer
**Handler:** `map_viewer`

### `GET /robot/meshes/{filename}`
**Summary:** Serve robot STL mesh files
**Handler:** `serve_robot_mesh`

## src\gateway\routes\operations.py

### `POST /api/v1/bag/start`
**Summary:** Start rosbag recording
**Response model:** `BagOperationResponse`
**Handler:** `bag_start`

### `GET /api/v1/bag/status`
**Summary:** rosbag recording status
**Response model:** `BagStatusResponse`
**Handler:** `bag_status`

### `POST /api/v1/bag/stop`
**Summary:** Stop rosbag recording
**Response model:** `BagOperationResponse`
**Handler:** `bag_stop`

### `POST /api/v1/explore/start`
**Summary:** Start autonomous frontier exploration
**Response model:** `ExplorationCommandResponse`
**Handler:** `explore_start`

### `GET /api/v1/explore/status`
**Summary:** Exploration status
**Response model:** `ExplorationStatusResponse`
**Handler:** `explore_status`

### `POST /api/v1/explore/stop`
**Summary:** Stop autonomous frontier exploration
**Response model:** `ExplorationCommandResponse`
**Handler:** `explore_stop`

### `GET /api/v1/memory/temporal`
**Summary:** Query temporal entity observations
**Response model:** `TemporalMemoryResponse`
**Handler:** `get_temporal_memory`

### `POST /api/v1/memory/temporal/semantic`
**Summary:** Semantic similarity search over temporal observations
**Response model:** `TemporalMemoryResponse`
**Handler:** `post_temporal_semantic`

### `POST /api/v1/slam/auto_relocalize`
**Summary:** Global relocalize via 3D-BBS (no guess required)
**Response model:** `SlamOperationResponse`
**Handler:** `slam_auto_relocalize`

### `POST /api/v1/slam/relocalize`
**Summary:** Relocalize against a saved map
**Response model:** `SlamOperationResponse`
**Handler:** `slam_relocalize`

### `GET /api/v1/slam/status`
**Summary:** SLAM service status
**Response model:** `SlamStatusResponse`
**Handler:** `slam_status`

### `POST /api/v1/slam/switch`
**Summary:** Hot-switch SLAM profile
**Response model:** `SlamOperationResponse`
**Handler:** `slam_switch`

### `POST /api/v1/webrtc/bitrate`
**Summary:** Live-tune WebRTC max bitrate without reconnect
**Response model:** `WebRTCControlResponse`
**Handler:** `post_webrtc_bitrate`

### `GET /api/v1/webrtc/go2rtc/status`
**Summary:** Probe the go2rtc sidecar (image transmission fast path)
**Response model:** `Go2RTCStatusResponse`
**Handler:** `get_go2rtc_status`

### `POST /api/v1/webrtc/offer`
**Summary:** WebRTC SDP offer/answer exchange for low-latency camera
**Response model:** `WebRTCControlResponse`
**Handler:** `post_webrtc_offer`

### `GET /api/v1/webrtc/stats`
**Summary:** WebRTC peer telemetry (bitrate, fps, encode time)
**Response model:** `WebRTCStatsResponse`
**Handler:** `get_webrtc_stats`

### `POST /api/v1/webrtc/whep`
**Summary:** WHEP signalling proxy to go2rtc (image transmission path)
**Handler:** `post_webrtc_whep`

## src\gateway\routes\session.py

### `GET /api/v1/session`
**Summary:** Current session state + capabilities
**Response model:** `SessionResponse`
**Handler:** `session_get`

### `POST /api/v1/session/end`
**Summary:** Exit current mode and return to idle
**Response model:** `SessionTransitionResponse`
**Handler:** `session_end`

### `POST /api/v1/session/start`
**Summary:** Enter a low-level mapping, navigating, or exploring session
**Response model:** `SessionTransitionResponse`
**Handler:** `session_start`

## src\gateway\routes\status.py

### `GET /api/v1/devices`
**Summary:** Hardware device registry status
**Response model:** `DevicesResponse`
**Handler:** `get_devices`

### `GET /api/v1/events`
**Summary:** SSE event stream
**Handler:** `sse_events`

### `GET /api/v1/health`
**Summary:** System health overview
**Response model:** `HealthResponse`
**Handler:** `get_health`

### `GET /api/v1/localization/status`
**Summary:** Localization status for app and web clients
**Response model:** `LocalizationStatusResponse`
**Handler:** `get_localization_status`

### `GET /api/v1/locations`
**Summary:** List tagged navigation locations
**Response model:** `LocationsResponse`
**Handler:** `get_locations`

### `POST /api/v1/locations`
**Summary:** Create or update a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `post_location`

### `PUT /api/v1/locations/{name}`
**Summary:** Update a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `put_location`

### `DELETE /api/v1/locations/{name}`
**Summary:** Delete a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `delete_location`

### `GET /api/v1/navigation`
**Response model:** `NavigationStatusResponse`
**Handler:** `get_navigation_status_legacy_alias`

### `GET /api/v1/navigation/status`
**Summary:** Navigation mission and control status
**Response model:** `NavigationStatusResponse`
**Handler:** `get_navigation_status`

### `GET /api/v1/path`
**Summary:** Latest planned path
**Response model:** `PathResponse`
**Handler:** `get_path`

### `GET /api/v1/readiness`
**Summary:** Client readiness snapshot
**Response model:** `ReadinessResponse`
**Handler:** `api_readiness`

### `GET /api/v1/runtime/dataflow`
**Summary:** Runtime dataflow and Module port observability
**Response model:** `RuntimeDataflowResponse`
**Handler:** `get_runtime_dataflow`

### `POST /api/v1/runtime/dataflow/subscribe`
**Summary:** Create a read-only runtime dataflow SSE subscription plan
**Response model:** `RuntimeDataflowSubscribeResponse`
**Handler:** `post_runtime_dataflow_subscribe`

### `GET /api/v1/runtime/dataflow/topic`
**Summary:** Inspect one runtime dataflow topic
**Response model:** `RuntimeDataflowTopicDetailResponse`
**Handler:** `get_runtime_dataflow_topic`

### `POST /api/v1/runtime/switch-plan`
**Summary:** Dry-run runtime endpoint switch plan
**Response model:** `RuntimeSwitchPlanResponse`
**Handler:** `post_runtime_switch_plan`

### `GET /api/v1/scene_graph`
**Summary:** Current scene graph
**Response model:** `SceneGraphResponse`
**Handler:** `get_scene_graph`

### `GET /api/v1/state`
**Summary:** Full robot state snapshot
**Response model:** `StateResponse`
**Handler:** `get_state`

### `GET /health`
**Summary:** Liveness probe
**Response model:** `LivenessResponse`
**Handler:** `liveness_health`

### `GET /ready`
**Summary:** Readiness probe
**Response model:** `ReadinessResponse`
**Handler:** `readiness_ready`
