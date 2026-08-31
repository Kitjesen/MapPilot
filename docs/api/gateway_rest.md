# Gateway REST API

> Auto-generated from route registrations in `src/gateway/routes/`.
> Deterministic generated inventory; run `python tools/docs/extract_api_docs.py --check` to verify freshness.

The GatewayModule serves these endpoints via FastAPI on port 5050.

---

## Summary

- **src/gateway/routes/app.py**:
  - `GET /api/v1/app/bootstrap` — App/Web bootstrap snapshot
  - `GET /api/v1/app/capabilities` — App/Web API capability manifest
  - `GET /api/v1/app/traffic` — App/Web realtime traffic and client polling policy
- **src/gateway/routes/assets.py**:
  - `GET /robot/meshes/{filename}` — Serve robot STL mesh files
- **src/gateway/routes/auth.py**:
  - `GET /api/v1/auth/check` — Check if auth is required
  - `POST /api/v1/auth/login` — Login with API key
- **src/gateway/routes/camera.py**:
  - `GET /api/v1/camera/snapshot` — Camera JPEG snapshot
- **src/gateway/routes/commands.py**:
  - `POST /api/v1/estop/reset` — Explicitly release the native software emergency-stop latch
  - `POST /api/v1/goal` — Send navigation goal
  - `POST /api/v1/instruction` — Natural language navigation instruction
  - `POST /api/v1/lease` — Acquire/release/renew control lease
  - `POST /api/v1/mode` — Switch operating mode
  - `POST /api/v1/navigate/click` — Navigate to map-viewer click point
  - `POST /api/v1/navigation/cancel` — Compatibility cancel for the current navigation mission
  - `POST /api/v1/navigation/goal_candidate` — Construct and optionally preview a navigation goal without publishing it
  - `POST /api/v1/navigation/plan` — Preview navigation plan without publishing a goal
  - `POST /api/v1/navigation/resume` — Release a native motion hold without replaying previous motion
  - `POST /api/v1/navigation/tasks/{task_id}/cancel` — Request cancellation of one navigation task
  - `POST /api/v1/navigation/tasks/{task_id}/pause` — Request a stop-confirmed pause of one navigation task
  - `POST /api/v1/navigation/tasks/{task_id}/resume` — Request continuation of the same paused navigation task
  - `POST /api/v1/stop` — Emergency stop
  - `POST /api/v1/visual_servo` — Set visual servo target
- **src/gateway/routes/diagnostics.py**:
  - `GET /api/v1/diagnostic_pack` — Export diagnostic tarball
  - `POST /api/v1/diagnostics/field-check` — Run read-only product field readiness check
  - `GET /api/v1/diagnostics/plugins` — Read registered plugin categories and providers
  - `GET /api/v1/diagnostics/real-runtime-evidence/latest` — Read latest Thunder field runtime evidence gate summary
  - `GET /api/v1/diagnostics/routecheck/latest` — Read latest non-motion routecheck summary
  - `GET /api/v1/diagnostics/runtime-contract` — Read canonical runtime interface contract
  - `POST /api/v1/inspection/acceptance` — Run read-only inspection acceptance without publishing motion commands
- **src/gateway/routes/inspection.py**:
  - `GET /api/v1/inspection/evidence` — List recent verified inspection evidence
  - `GET /api/v1/inspection/evidence/{evidence_id}` — Read one verified inspection evidence manifest
  - `GET /api/v1/inspection/evidence/{evidence_id}/artifacts/{kind}` — Read one verified inspection evidence artifact
  - `GET /api/v1/inspection/routes` — List native inspection routes for a map
  - `POST /api/v1/inspection/routes` — Create or update a native inspection route
  - `DELETE /api/v1/inspection/routes/{route_id}` — Delete one native inspection route
  - `GET /api/v1/inspection/routes/{route_id}` — Read one native inspection route
  - `GET /api/v1/inspection/status` — Read native inspection store/status snapshot
  - `GET /api/v1/inspection/tasks` — List retained inspection task projections
  - `POST /api/v1/inspection/tasks` — Submit a task-addressed native inspection route
  - `GET /api/v1/inspection/tasks/{task_id}` — Read the fact-backed state of one inspection task
  - `POST /api/v1/inspection/tasks/{task_id}/cancel` — Request cancellation for one native inspection task
  - `POST /api/v1/inspection/tasks/{task_id}/pause` — Request pause for one native inspection task
  - `GET /api/v1/inspection/tasks/{task_id}/report` — Read the business result of one inspection task
  - `POST /api/v1/inspection/tasks/{task_id}/resume` — Request resume for one native inspection task
- **src/gateway/routes/maps.py**:
  - `GET /api/v1/map/points` — Map point cloud as JSON (from ikd-tree snapshot)
  - `POST /api/v1/map/rename` — Rename a saved map
  - `POST /api/v1/map/save` — Save current SLAM map
  - `POST /api/v1/map_cloud/reset` — Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)
  - `POST /api/v1/maps/import_pcd` — Import a PCD file into a LingTu map package
  - `GET /api/v1/maps/operations` — List durable map-save operations
  - `GET /api/v1/maps/operations/{operation_id}` — Get a durable map-save operation
  - `POST /api/v1/maps/operations/{operation_id}/cancel` — Cancel a durable map-save operation
  - `POST /api/v1/maps/operations/{operation_id}/retry` — Retry a failed durable map-save operation
  - `DELETE /api/v1/maps/{name}` — Delete a saved map
  - `POST /api/v1/maps/{name}/build_occupancy` — Build a 2D occupancy artifact from a saved map
  - `POST /api/v1/maps/{name}/build_octomap` — Build OctoPlanner3D octomap.ot from saved map.pcd
  - `POST /api/v1/maps/{name}/crop` — Crop a saved map point cloud and invalidate derived artifacts
  - `POST /api/v1/maps/{name}/mark_zone` — Mark occupied/free/preblocked/traversable zones in the saved OctoMap
  - `GET /api/v1/maps/{name}/pcd` — Serve raw PCD file for inline preview
  - `GET /api/v1/maps/{name}/points` — Saved map point cloud as JSON
  - `POST /api/v1/maps/{name}/validate_plan` — Preview a route on the active saved map without publishing a goal
  - `POST /api/v1/maps/{name}/voxels/edit` — Edit saved OctoMap voxels for OctoPlanner3D
  - `GET /api/v1/maps/{name}/voxels/edits` — Saved OctoMap voxel edit overlay
  - `GET /api/v1/slam/maps` — List maps through native mapd
  - `GET /map/viewer` — Interactive 3D map viewer
- **src/gateway/routes/operations.py**:
  - `POST /api/v1/explore/directed` — Set an explicit native TARE exploration direction intent
  - `POST /api/v1/explore/directed/clear` — Clear the explicit native TARE exploration direction intent
  - `GET /api/v1/explore/runs` — List recent durable native Explore executions
  - `GET /api/v1/explore/runs/{exploration_run_id}` — Query one durable native Explore execution
  - `POST /api/v1/explore/start` — Start autonomous frontier exploration
  - `GET /api/v1/explore/status` — Exploration status
  - `POST /api/v1/explore/stop` — Stop autonomous frontier exploration
  - `POST /api/v1/localization/map-tracking` — Start continuous tracking against the running Product's active map
  - `POST /api/v1/localization/relocalizations` — Relocalize against the running Product's active map
  - `GET /api/v1/memory/temporal` — Query temporal entity observations
  - `POST /api/v1/memory/temporal/semantic` — Semantic similarity search over temporal observations
  - `POST /api/v1/recordings/start` — Start native MCAP recording
  - `GET /api/v1/recordings/status` — Native MCAP recording status
  - `POST /api/v1/recordings/stop` — Stop native MCAP recording
  - `GET /api/v1/services/status` — Product service status
  - `GET /api/v1/slam/status` — SLAM service status
  - `GET /api/v1/webrtc/go2rtc/status` — Probe the go2rtc sidecar (image transmission fast path)
  - `POST /api/v1/webrtc/whep` — WHEP signalling proxy to go2rtc (image transmission path)
- **src/gateway/routes/places.py**:
  - `GET /api/v1/places` — List canonical semantic places
  - `POST /api/v1/places` — Create or update a canonical semantic place on a native map
  - `GET /api/v1/places/resolve` — Resolve a canonical semantic place by name or alias
- **src/gateway/routes/recordings.py**:
  - `GET /api/v1/recordings`
  - `DELETE /api/v1/recordings/{session_id}`
  - `GET /api/v1/recordings/{session_id}`
  - `GET /api/v1/recordings/{session_id}/files/{artifact_path:path}`
- **src/gateway/routes/safety.py**:
  - `GET /api/v1/safety/modes/estop` — Read emergency-stop state
  - `POST /api/v1/safety/modes/estop` — Activate emergency stop
- **src/gateway/routes/session.py**:
  - `GET /api/v1/session` — Current Product session state and capabilities
- **src/gateway/routes/status.py**:
  - `GET /api/v1/events` — SSE event stream
  - `GET /api/v1/health` — System health overview
  - `GET /api/v1/localization/status` — Localization status for app and web clients
  - `GET /api/v1/locations` — List tagged navigation locations
  - `POST /api/v1/locations` — Create or update a tagged navigation location
  - `DELETE /api/v1/locations/{name}` — Delete a tagged navigation location
  - `PUT /api/v1/locations/{name}` — Update a tagged navigation location
  - `GET /api/v1/maps/environment/layers` — Operator-facing environment map layer state
  - `GET /api/v1/metrics` — Operator-facing runtime metrics snapshot
  - `GET /api/v1/navigation/dds_snapshot` — Latest navigation data for the native DDS endpoint
  - `GET /api/v1/navigation/goals/{request_id}` — Request-correlated native navigation lifecycle status
  - `GET /api/v1/navigation/status` — Navigation mission and control status
  - `GET /api/v1/navigation/tasks/{task_id}` — Stable native navigation task lifecycle status
  - `GET /api/v1/path` — Latest planned path
  - `GET /api/v1/readiness` — Client readiness snapshot
  - `GET /api/v1/runtime/dataflow` — Read-only Product motion and Gateway observability
  - `POST /api/v1/runtime/dataflow/subscribe` — Create a read-only Gateway SSE subscription plan
  - `GET /api/v1/runtime/dataflow/topic` — Inspect one Gateway-observable Product topic
  - `GET /api/v1/scene_graph` — Current scene graph
  - `GET /api/v1/state` — Full robot state snapshot
  - `GET /health` — Liveness probe
  - `GET /ready` — Readiness probe

---

## src/gateway/routes/app.py

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

## src/gateway/routes/assets.py

### `GET /robot/meshes/{filename}`
**Summary:** Serve robot STL mesh files
**Handler:** `serve_robot_mesh`

## src/gateway/routes/auth.py

### `GET /api/v1/auth/check`
**Summary:** Check if auth is required
**Response model:** `AuthCheckResponse`
**Handler:** `auth_check`

### `POST /api/v1/auth/login`
**Summary:** Login with API key
**Response model:** `AuthLoginResponse`
**Handler:** `auth_login`

## src/gateway/routes/camera.py

### `GET /api/v1/camera/snapshot`
**Summary:** Camera JPEG snapshot
**Handler:** `camera_snapshot`

## src/gateway/routes/commands.py

### `POST /api/v1/estop/reset`
**Summary:** Explicitly release the native software emergency-stop latch
**Response model:** `ControlCommandResponse`
**Handler:** `post_estop_reset`

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
**Summary:** Compatibility cancel for the current navigation mission
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

### `POST /api/v1/navigation/resume`
**Summary:** Release a native motion hold without replaying previous motion
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_resume`

### `POST /api/v1/navigation/tasks/{task_id}/cancel`
**Summary:** Request cancellation of one navigation task
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_task_cancel`

### `POST /api/v1/navigation/tasks/{task_id}/pause`
**Summary:** Request a stop-confirmed pause of one navigation task
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_task_pause`

### `POST /api/v1/navigation/tasks/{task_id}/resume`
**Summary:** Request continuation of the same paused navigation task
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_task_resume`

### `POST /api/v1/stop`
**Summary:** Emergency stop
**Response model:** `ControlCommandResponse`
**Handler:** `post_stop`

### `POST /api/v1/visual_servo`
**Summary:** Set visual servo target
**Response model:** `ControlCommandResponse`
**Handler:** `post_visual_servo`

## src/gateway/routes/diagnostics.py

### `GET /api/v1/diagnostic_pack`
**Summary:** Export diagnostic tarball
**Handler:** `diagnostic_pack`

### `POST /api/v1/diagnostics/field-check`
**Summary:** Run read-only product field readiness check
**Response model:** `ProductFieldCheckResponse`
**Handler:** `product_field_check`

### `GET /api/v1/diagnostics/plugins`
**Summary:** Read registered plugin categories and providers
**Handler:** `plugin_catalog`

### `GET /api/v1/diagnostics/real-runtime-evidence/latest`
**Summary:** Read latest Thunder field runtime evidence gate summary
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

## src/gateway/routes/inspection.py

### `GET /api/v1/inspection/evidence`
**Summary:** List recent verified inspection evidence
**Handler:** `list_inspection_evidence`

### `GET /api/v1/inspection/evidence/{evidence_id}`
**Summary:** Read one verified inspection evidence manifest
**Handler:** `get_inspection_evidence`

### `GET /api/v1/inspection/evidence/{evidence_id}/artifacts/{kind}`
**Summary:** Read one verified inspection evidence artifact
**Handler:** `get_inspection_evidence_artifact`

### `GET /api/v1/inspection/routes`
**Summary:** List native inspection routes for a map
**Response model:** `InspectionRouteListResponse`
**Handler:** `list_inspection_routes`

### `POST /api/v1/inspection/routes`
**Summary:** Create or update a native inspection route
**Response model:** `InspectionRouteResponse`
**Handler:** `put_inspection_route`

### `DELETE /api/v1/inspection/routes/{route_id}`
**Summary:** Delete one native inspection route
**Response model:** `InspectionCommandResponse`
**Handler:** `delete_inspection_route`

### `GET /api/v1/inspection/routes/{route_id}`
**Summary:** Read one native inspection route
**Response model:** `InspectionRouteResponse`
**Handler:** `get_inspection_route`

### `GET /api/v1/inspection/status`
**Summary:** Read native inspection store/status snapshot
**Response model:** `InspectionStatusResponse`
**Handler:** `inspection_status`

### `GET /api/v1/inspection/tasks`
**Summary:** List retained inspection task projections
**Response model:** `InspectionTaskListResponse`
**Handler:** `list_inspection_tasks`

### `POST /api/v1/inspection/tasks`
**Summary:** Submit a task-addressed native inspection route
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `start_inspection_task`

### `GET /api/v1/inspection/tasks/{task_id}`
**Summary:** Read the fact-backed state of one inspection task
**Response model:** `InspectionTaskStatusResponse`
**Handler:** `get_inspection_task`

### `POST /api/v1/inspection/tasks/{task_id}/cancel`
**Summary:** Request cancellation for one native inspection task
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `cancel_inspection_task`

### `POST /api/v1/inspection/tasks/{task_id}/pause`
**Summary:** Request pause for one native inspection task
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `pause_inspection_task`

### `GET /api/v1/inspection/tasks/{task_id}/report`
**Summary:** Read the business result of one inspection task
**Response model:** `InspectionTaskReportResponse`
**Handler:** `get_inspection_task_report`

### `POST /api/v1/inspection/tasks/{task_id}/resume`
**Summary:** Request resume for one native inspection task
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `resume_inspection_task`

## src/gateway/routes/maps.py

### `GET /api/v1/map/points`
**Summary:** Map point cloud as JSON (from ikd-tree snapshot)
**Response model:** `MapPointsResponse`
**Handler:** `get_map_points`

### `POST /api/v1/map/rename`
**Summary:** Rename a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `rename_map`

### `POST /api/v1/map/save`
**Summary:** Save current SLAM map
**Response model:** `MapSaveOperationResponse`
**Handler:** `save_map_now`

### `POST /api/v1/map_cloud/reset`
**Summary:** Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)
**Response model:** `MapLifecycleResponse`
**Handler:** `reset_map_cloud`

### `POST /api/v1/maps/import_pcd`
**Summary:** Import a PCD file into a LingTu map package
**Response model:** `MapLifecycleResponse`
**Handler:** `import_pcd_map`

### `GET /api/v1/maps/operations`
**Summary:** List durable map-save operations
**Response model:** `MapSaveOperationResponse`
**Handler:** `list_map_operations`

### `GET /api/v1/maps/operations/{operation_id}`
**Summary:** Get a durable map-save operation
**Response model:** `MapSaveOperationResponse`
**Handler:** `get_map_operation`

### `POST /api/v1/maps/operations/{operation_id}/cancel`
**Summary:** Cancel a durable map-save operation
**Response model:** `MapSaveOperationResponse`
**Handler:** `cancel_map_operation`

### `POST /api/v1/maps/operations/{operation_id}/retry`
**Summary:** Retry a failed durable map-save operation
**Response model:** `MapSaveOperationResponse`
**Handler:** `retry_map_operation`

### `DELETE /api/v1/maps/{name}`
**Summary:** Delete a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `delete_saved_map`

### `POST /api/v1/maps/{name}/build_occupancy`
**Summary:** Build a 2D occupancy artifact from a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `build_saved_map_occupancy`

### `POST /api/v1/maps/{name}/build_octomap`
**Summary:** Build OctoPlanner3D octomap.ot from saved map.pcd
**Response model:** `MapLifecycleResponse`
**Handler:** `build_saved_map_octomap`

### `POST /api/v1/maps/{name}/crop`
**Summary:** Crop a saved map point cloud and invalidate derived artifacts
**Response model:** `MapLifecycleResponse`
**Handler:** `crop_saved_map`

### `POST /api/v1/maps/{name}/mark_zone`
**Summary:** Mark occupied/free/preblocked/traversable zones in the saved OctoMap
**Response model:** `MapLifecycleResponse`
**Handler:** `mark_saved_map_zone`

### `GET /api/v1/maps/{name}/pcd`
**Summary:** Serve raw PCD file for inline preview
**Handler:** `get_map_pcd`

### `GET /api/v1/maps/{name}/points`
**Summary:** Saved map point cloud as JSON
**Response model:** `MapPointsResponse`
**Handler:** `get_saved_map_points`

### `POST /api/v1/maps/{name}/validate_plan`
**Summary:** Preview a route on the active saved map without publishing a goal
**Handler:** `validate_saved_map_plan`

### `POST /api/v1/maps/{name}/voxels/edit`
**Summary:** Edit saved OctoMap voxels for OctoPlanner3D
**Response model:** `MapLifecycleResponse`
**Handler:** `edit_saved_map_voxels`

### `GET /api/v1/maps/{name}/voxels/edits`
**Summary:** Saved OctoMap voxel edit overlay
**Handler:** `get_saved_map_voxel_edits`

### `GET /api/v1/slam/maps`
**Summary:** List maps through native mapd
**Response model:** `MapListResponse`
**Handler:** `slam_maps`

### `GET /map/viewer`
**Summary:** Interactive 3D map viewer
**Handler:** `map_viewer`

## src/gateway/routes/operations.py

### `POST /api/v1/explore/directed`
**Summary:** Set an explicit native TARE exploration direction intent
**Response model:** `DirectedExplorationResponse`
**Handler:** `explore_directed`

### `POST /api/v1/explore/directed/clear`
**Summary:** Clear the explicit native TARE exploration direction intent
**Response model:** `DirectedExplorationResponse`
**Handler:** `clear_explore_directed`

### `GET /api/v1/explore/runs`
**Summary:** List recent durable native Explore executions
**Response model:** `ExplorationRunListResponse`
**Handler:** `explore_runs_list`

### `GET /api/v1/explore/runs/{exploration_run_id}`
**Summary:** Query one durable native Explore execution
**Response model:** `ExplorationRunResponse`
**Handler:** `explore_run_get`

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

### `POST /api/v1/localization/map-tracking`
**Summary:** Start continuous tracking against the running Product's active map
**Response model:** `LocalizationOperationResponse`
**Handler:** `localization_map_tracking`

### `POST /api/v1/localization/relocalizations`
**Summary:** Relocalize against the running Product's active map
**Response model:** `LocalizationOperationResponse`
**Handler:** `localization_relocalize`

### `GET /api/v1/memory/temporal`
**Summary:** Query temporal entity observations
**Response model:** `TemporalMemoryResponse`
**Handler:** `get_temporal_memory`

### `POST /api/v1/memory/temporal/semantic`
**Summary:** Semantic similarity search over temporal observations
**Response model:** `TemporalMemoryResponse`
**Handler:** `post_temporal_semantic`

### `POST /api/v1/recordings/start`
**Summary:** Start native MCAP recording
**Response model:** `RecordingOperationResponse`
**Handler:** `recording_start`

### `GET /api/v1/recordings/status`
**Summary:** Native MCAP recording status
**Response model:** `RecordingStatusResponse`
**Handler:** `recording_status`

### `POST /api/v1/recordings/stop`
**Summary:** Stop native MCAP recording
**Response model:** `RecordingOperationResponse`
**Handler:** `recording_stop`

### `GET /api/v1/services/status`
**Summary:** Product service status
**Response model:** `ServiceStatusResponse`
**Handler:** `service_status`

### `GET /api/v1/slam/status`
**Summary:** SLAM service status
**Response model:** `SlamStatusResponse`
**Handler:** `slam_status`

### `GET /api/v1/webrtc/go2rtc/status`
**Summary:** Probe the go2rtc sidecar (image transmission fast path)
**Response model:** `Go2RTCStatusResponse`
**Handler:** `get_go2rtc_status`

### `POST /api/v1/webrtc/whep`
**Summary:** WHEP signalling proxy to go2rtc (image transmission path)
**Handler:** `post_webrtc_whep`

## src/gateway/routes/places.py

### `GET /api/v1/places`
**Summary:** List canonical semantic places
**Response model:** `PlaceListResponse`
**Handler:** `get_places`

### `POST /api/v1/places`
**Summary:** Create or update a canonical semantic place on a native map
**Response model:** `PlaceUpsertResponse`
**Handler:** `post_place`

### `GET /api/v1/places/resolve`
**Summary:** Resolve a canonical semantic place by name or alias
**Response model:** `PlaceResolveResponse`
**Handler:** `resolve_place`

## src/gateway/routes/recordings.py

### `GET /api/v1/recordings`
**Handler:** `recording_list`

### `DELETE /api/v1/recordings/{session_id}`
**Handler:** `recording_remove`

### `GET /api/v1/recordings/{session_id}`
**Handler:** `recording_detail`

### `GET /api/v1/recordings/{session_id}/files/{artifact_path:path}`
**Handler:** `recording_artifact`

## src/gateway/routes/safety.py

### `GET /api/v1/safety/modes/estop`
**Summary:** Read emergency-stop state
**Response model:** `SafetyEstopResponse`
**Handler:** `get_safety_estop`

### `POST /api/v1/safety/modes/estop`
**Summary:** Activate emergency stop
**Response model:** `SafetyEstopResponse`
**Handler:** `post_safety_estop`

## src/gateway/routes/session.py

### `GET /api/v1/session`
**Summary:** Current Product session state and capabilities
**Response model:** `SessionResponse`
**Handler:** `session_get`

## src/gateway/routes/status.py

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

### `DELETE /api/v1/locations/{name}`
**Summary:** Delete a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `delete_location`

### `PUT /api/v1/locations/{name}`
**Summary:** Update a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `put_location`

### `GET /api/v1/maps/environment/layers`
**Summary:** Operator-facing environment map layer state
**Handler:** `get_environment_map_layers`

### `GET /api/v1/metrics`
**Summary:** Operator-facing runtime metrics snapshot
**Handler:** `get_metrics`

### `GET /api/v1/navigation/dds_snapshot`
**Summary:** Latest navigation data for the native DDS endpoint
**Response model:** `NavigationDdsSnapshotResponse`
**Handler:** `get_navigation_dds_snapshot`

### `GET /api/v1/navigation/goals/{request_id}`
**Summary:** Request-correlated native navigation lifecycle status
**Response model:** `NavigationGoalStatusQueryResponse`
**Handler:** `get_navigation_goal_status`

### `GET /api/v1/navigation/status`
**Summary:** Navigation mission and control status
**Response model:** `NavigationStatusResponse`
**Handler:** `get_navigation_status`

### `GET /api/v1/navigation/tasks/{task_id}`
**Summary:** Stable native navigation task lifecycle status
**Response model:** `NavigationTaskStatusQueryResponse`
**Handler:** `get_navigation_task_status`

### `GET /api/v1/path`
**Summary:** Latest planned path
**Response model:** `PathResponse`
**Handler:** `get_path`

### `GET /api/v1/readiness`
**Summary:** Client readiness snapshot
**Response model:** `ReadinessResponse`
**Handler:** `api_readiness`

### `GET /api/v1/runtime/dataflow`
**Summary:** Read-only Product motion and Gateway observability
**Response model:** `RuntimeDataflowResponse`
**Handler:** `get_runtime_dataflow`

### `POST /api/v1/runtime/dataflow/subscribe`
**Summary:** Create a read-only Gateway SSE subscription plan
**Response model:** `RuntimeDataflowSubscribeResponse`
**Handler:** `post_runtime_dataflow_subscribe`

### `GET /api/v1/runtime/dataflow/topic`
**Summary:** Inspect one Gateway-observable Product topic
**Response model:** `RuntimeDataflowTopicDetailResponse`
**Handler:** `get_runtime_dataflow_topic`

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
