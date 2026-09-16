# API

**Status:** Current generated interface reference
**Audience:** SDK, Web, MCP, Gateway, and integration developers
**Runs on:** One LingTu Host in `env=real` or `env=sim`

This file combines the small maintained integration contract with
inventories generated from source. Do not hand-edit the generated
method or route lists; run `python tools/docs/extract_api_docs.py`.

## Integration boundaries

| Surface | Endpoint | Owner |
| --- | --- | --- |
| REST and SSE | `http://<host>:5050` | Gateway routes and projections |
| MCP | `http://<host>:8090/mcp` | `@skill` methods discovered in the Host |
| Python SDK/CLI | `lingtu-sdk` and `lingtu.sdk` | Typed client facade |
| Camera Web media | go2rtc WHEP with Gateway JPEG fallback | Optional media sidecar plus Gateway |

Gateway submits typed intent and projects runtime facts. It does not
own Product lifecycle, maps, planning, final motion, or hardware.

Product lifecycle uses `lingtu` / `python -m lingtu.control`, not REST
service orchestration assembled by a client.

External map integrations operate on the canonical map identity
`map_id + content_epoch` and maintained artifact contracts. They must
not invent version directories or bypass ProductControl activation.

## Generation

```bash
python tools/docs/extract_api_docs.py
python tools/docs/extract_api_docs.py --check
```

## Navigation status

`GET /api/v1/navigation/status` and SSE `navigation_status` expose one root-level
v3 contract. It answers four independent questions:
Implementation lives in `src/gateway/navigation/`: `routes.py` registers the
HTTP surface, `status.py` evaluates admission and publishes SSE, `projection.py`
projects the public axes, and `tasks.py` handles exact task/request queries.

| Axis | Meaning |
| --- | --- |
| `task` | Current task phase: `IDLE`, planning, execution, recovery, explicit pause, or a terminal result |
| `goal_admission` | Whether a new or replacement goal can be accepted |
| `control` | Whether autonomy, the operator, or nobody owns control |
| `motion` | Permission, observed motion, and stop-confirmation evidence |

The task lifecycle is `IDLE -> PLANNING -> EXECUTING <-> RECOVERING`, with
explicit `PAUSED` and terminal `SUCCESS`, `FAILED`, or `CANCELLED` branches.
E-stop, takeover, and InputGate holds change control or motion; they do not fake
a task pause. `QUIET` is only a fresh odometry observation and is not equivalent
to `CONFIRMED`. Missing or stale evidence projects to `UNKNOWN`.

There is no `operator_state` wrapper or second rich navigation status. Full
blockers live at `/api/v1/readiness`, paths at `/api/v1/path`, native evidence at
`/api/v1/navigation/dds_snapshot`, and exact terminal evidence at
`/api/v1/navigation/tasks/{task_id}`.

## MCP tools

This inventory is generated from `@skill` decorators. The Host exposes
these methods through MCP JSON-RPC on port 8090 and to the Agent loop.

## src/decision/modules/semantic_planner.py
_SemanticPlannerModule - unified semantic planning in one Module._

### `send_instruction`
**Module:** `SemanticPlannerModule`
**Description:** Send a natural language navigation instruction to the semantic planner.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `text` | `str` |

### `get_planner_status`
**Module:** `SemanticPlannerModule`
**Description:** Return current semantic planner state and counters.
**Return type:** `str`
**Parameters:** None

### `get_scene_objects`
**Module:** `SemanticPlannerModule`
**Description:** Return the latest scene graph objects for facade/API clients.
**Return type:** `str`
**Parameters:** None

### `decompose_task`
**Module:** `SemanticPlannerModule`
**Description:** Decompose a complex instruction into ordered sub-goals.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `instruction` | `str` |

## src/decision/modules/visual_servo.py
_Visual servo Module._

### `find_object`
**Module:** `VisualServoModule`
**Description:** Trigger visual find mode for a target object.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `target` | `str` |

### `follow_person`
**Module:** `VisualServoModule`
**Description:** Trigger person following mode.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `description` | `str` |

### `stop_servo`
**Module:** `VisualServoModule`
**Description:** Stop all visual tracking.
**Return type:** `str`
**Parameters:** None

### `get_servo_status`
**Module:** `VisualServoModule`
**Description:** Return current servo state.
**Return type:** `dict`
**Parameters:** None

## src/decision/modules/vla.py
_VLA navigation module._

### `vla_navigate`
**Module:** `VLAModule`
**Description:** Send a natural-language navigation instruction to the VLA module.
**Return type:** `dict`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `instruction` | `str` |

### `vla_status`
**Module:** `VLAModule`
**Description:** Return current VLA health and metrics.
**Return type:** `dict`
**Parameters:** None

## src/explore/tare/module.py
_TAREExplorerModule - connects TARE exploration to LingTu navigation._

### `start_tare_exploration`
**Module:** `TAREExplorerModule`
**Description:** Send the start signal to the TARE planner. Use this to resume
**Return type:** `str`
**Parameters:** None

### `stop_tare_exploration`
**Module:** `TAREExplorerModule`
**Description:** Pause TARE exploration. The planner keeps running but stops
**Return type:** `str`
**Parameters:** None

### `get_tare_status`
**Module:** `TAREExplorerModule`
**Description:** Return TARE exploration state: waypoint count, last waypoint age,
**Return type:** `str`
**Parameters:** None

## src/explore/tare/supervisor.py
_ExplorationSupervisorModule — cross-process watchdog for TARE exploration._

### `get_exploration_supervisor`
**Module:** `ExplorationSupervisorModule`
**Description:** Return the latest exploration supervisor state as JSON.
**Return type:** `str`
**Parameters:** None

### `clear_exploration_fallback`
**Module:** `ExplorationSupervisorModule`
**Description:** Reset the fallback_requested flag (use after operator handled).
**Return type:** `str`
**Parameters:** None

## src/gateway/mcp_server.py
_LingTu MCP Server -Model Context Protocol for AI agent control._

### `get_health`
**Module:** `MCPServerModule`
**Description:** Return full system health: modules, connections, message counts.
**Return type:** `str`
**Parameters:** None

### `list_modules`
**Module:** `MCPServerModule`
**Description:** List all deployed modules with layer, port counts, and running state.
**Return type:** `str`
**Parameters:** None

### `get_config`
**Module:** `MCPServerModule`
**Description:** Return robot configuration: speed limits, geometry, safety thresholds.
**Return type:** `str`
**Parameters:** None

### `get_robot_position`
**Module:** `MCPServerModule`
**Description:** Return the robot's current position (x, y, z) and yaw in map frame.
**Return type:** `str`
**Parameters:** None

### `get_scene_graph`
**Module:** `MCPServerModule`
**Description:** Return the current scene graph -detected objects with positions and labels.
**Return type:** `str`
**Parameters:** None

### `detect_objects`
**Module:** `MCPServerModule`
**Description:** Search the current scene for objects matching *query* (case-insensitive).
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `query` | `str` |

### `query_memory`
**Module:** `MCPServerModule`
**Description:** Search episodic, spatial, and vector memory for past observations.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `query` | `str` |

### `list_tagged_locations`
**Module:** `MCPServerModule`
**Description:** List all named locations tagged by the robot or user.
**Return type:** `str`
**Parameters:** None

### `tag_location`
**Module:** `MCPServerModule`
**Description:** Save the robot's current position under *name* for future navigation.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `name` | `str` |

### `navigate_to_object`
**Module:** `MCPServerModule`
**Description:** Navigate to a described object or place using semantic understanding.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `instruction` | `str` |

### `send_instruction`
**Module:** `MCPServerModule`
**Description:** Send a natural language instruction to the semantic planner.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `text` | `str` |

### `emergency_stop`
**Module:** `MCPServerModule`
**Description:** Emergency stop -immediately halts all robot motion.
**Return type:** `str`
**Parameters:** None

### `set_mode`
**Module:** `MCPServerModule`
**Description:** Set robot operating mode: manual | autonomous | estop.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `mode` | `str` |

## src/memory/modules/episodic_module.py
_EpisodicMemoryModule — 时空情节记忆模块 (Module 模式封装)。_

### `get_recent_observations`
**Module:** `EpisodicMemoryModule`
**Description:** Return the most recent episodic observations (labels + position).
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `count` | `int` |

## src/memory/modules/mission_logger_module.py
_MissionLoggerModule — mission history recorder as a Module._

### `list_missions`
**Module:** `MissionLoggerModule`
**Description:** List the *count* most recent navigation missions (summary, no trajectory).
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `count` | `int` |

### `get_mission_stats`
**Module:** `MissionLoggerModule`
**Description:** Return aggregate statistics for all recorded navigation missions.
**Return type:** `str`
**Parameters:** None

## src/memory/modules/semantic_mapper_module.py
_SemanticMapperModule -drives RoomObjectKG + TopologySemGraph from live SceneGraph._

### `get_room_summary`
**Module:** `SemanticMapperModule`
**Description:** Return a text summary of all known rooms and their objects.
**Return type:** `str`
**Parameters:** None

### `query_room_for_object`
**Module:** `SemanticMapperModule`
**Description:** Return which room types are most likely to contain the given object label.
**Return type:** `dict`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `label` | `str` |

### `get_exploration_target`
**Module:** `SemanticMapperModule`
**Description:** Return the best exploration target for a given instruction.
**Return type:** `dict`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `instruction` | `str` |

### `get_semantic_status`
**Module:** `SemanticMapperModule`
**Description:** Return KG + TSG statistics.
**Return type:** `dict`
**Parameters:** None

## src/memory/modules/tagged_locations_module.py
_TaggedLocationsModule — 标签地点记忆模块 (Module 模式封装)。_

### `list_tags`
**Module:** `TaggedLocationsModule`
**Description:** List all saved location tags with positions.
**Return type:** `str`
**Parameters:** None

### `go_to_tag`
**Module:** `TaggedLocationsModule`
**Description:** Navigate to a tagged location by name (publishes saved_location if found).
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `name` | `str` |

## src/memory/modules/temporal_memory_module.py
_TemporalMemoryModule — Time-indexed scene memory for temporal queries._

### `query_temporal`
**Module:** `TemporalMemoryModule`
**Description:** Answer a natural language question about temporal scene memory.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `question` | `str` |

### `get_entity_history`
**Module:** `TemporalMemoryModule`
**Description:** Return a formatted summary of all sightings of a specific object label.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `label` | `str` |

## src/memory/modules/vector_memory_module.py
_VectorMemoryModule — CLIP embedding + ChromaDB vector search for fuzzy spatial queries._

### `query_location`
**Module:** `VectorMemoryModule`
**Description:** Fuzzy search for a location by natural language description.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `text` | `str` |

### `get_memory_stats`
**Module:** `VectorMemoryModule`
**Description:** Return vector memory statistics.
**Return type:** `str`
**Parameters:** None

## src/nav/skills/skills_module.py

### `navigate_to`
**Module:** `NavSkills`
**Description:** Submit a map-frame navigation goal.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `x` | `float` |
| `y` | `float` |
| `yaw` | `float` |
| `z` | `float | None` |

### `stop_navigation`
**Module:** `NavSkills`
**Description:** Stop the active navigation mission without invoking hardware E-stop.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `task_id` | `str` |

### `cancel_mission`
**Module:** `NavSkills`
**Description:** Cancel the active navigation mission.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `task_id` | `str` |
| `reason` | `str` |

### `get_navigation_status`
**Module:** `NavSkills`
**Description:** Return the canonical navigation mission status.
**Return type:** `str`
**Parameters:** None

### `get_navigation_result`
**Module:** `NavSkills`
**Description:** Return the native lifecycle result for a request submitted by this adapter.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `request_id` | `str` |

### `start_inspection`
**Module:** `NavSkills`
**Description:** Start a stored native inspection route.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `route_id` | `str` |
| `revision` | `int` |

### `navigate_to_deg`
**Module:** `NavSkills`
**Description:** Submit a map-frame navigation goal with heading in degrees.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `x` | `float` |
| `y` | `float` |
| `yaw_deg` | `float` |
| `z` | `float | None` |

### `is_navigating`
**Module:** `NavSkills`
**Description:** Return whether a navigation mission is currently active.
**Return type:** `str`
**Parameters:** None

### `get_navigation_progress`
**Module:** `NavSkills`
**Description:** Return a concise progress summary derived from mission status.
**Return type:** `str`
**Parameters:** None


## Gateway REST API

The Gateway serves these generated route registrations on port 5050.
FastAPI's live OpenAPI UI remains available at `/docs`.

### Route summary

- **src/gateway/maps/locations.py**:
  - `GET /api/v1/locations` — List tagged navigation locations
  - `POST /api/v1/locations` — Create or update a tagged navigation location
  - `DELETE /api/v1/locations/{name}` — Delete a tagged navigation location
  - `PUT /api/v1/locations/{name}` — Update a tagged navigation location
- **src/gateway/maps/places.py**:
  - `GET /api/v1/places` — List canonical semantic places
  - `POST /api/v1/places` — Create or update a canonical semantic place on a native map
  - `GET /api/v1/places/resolve` — Resolve a canonical semantic place by name or alias
- **src/gateway/maps/routes.py**:
  - `GET /api/v1/map/points` — Map point cloud as JSON (from ikd-tree snapshot)
  - `POST /api/v1/map/rename` — Rename a saved map
  - `POST /api/v1/map/save` — Save current SLAM map
  - `POST /api/v1/map_cloud/reset` — Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)
  - `GET /api/v1/maps/environment/layers` — Operator-facing environment map layer state
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
- **src/gateway/mcp_server.py**:
  - `GET /capabilities`
  - `GET /health`
  - `POST /mcp`
- **src/gateway/navigation/diagnostics.py**:
  - `GET /api/v1/navigation/dds_snapshot` — Latest navigation data for the native DDS endpoint
  - `GET /api/v1/path` — Latest planned path
- **src/gateway/navigation/routes.py**:
  - `POST /api/v1/goal` — Send navigation goal
  - `POST /api/v1/navigate/click` — Navigate to map-viewer click point
  - `POST /api/v1/navigation/cancel` — Compatibility cancel for the current navigation mission
  - `POST /api/v1/navigation/goal_candidate` — Construct and optionally preview a navigation goal without publishing it
  - `GET /api/v1/navigation/goals/{request_id}` — Request-correlated native navigation lifecycle status
  - `POST /api/v1/navigation/plan` — Preview navigation plan without publishing a goal
  - `POST /api/v1/navigation/resume` — Release a native motion hold without replaying previous motion
  - `GET /api/v1/navigation/status` — Public navigation status
  - `GET /api/v1/navigation/tasks/{task_id}` — Stable native navigation task lifecycle status
  - `POST /api/v1/navigation/tasks/{task_id}/cancel` — Request cancellation of one navigation task
  - `POST /api/v1/navigation/tasks/{task_id}/pause` — Request a stop-confirmed pause of one navigation task
  - `POST /api/v1/navigation/tasks/{task_id}/resume` — Request continuation of the same paused navigation task
- **src/gateway/routes/app.py**:
  - `GET /api/v1/app/bootstrap` — App/Web bootstrap snapshot
  - `GET /api/v1/app/capabilities` — App/Web API capability manifest
  - `GET /api/v1/app/traffic` — App/Web realtime traffic and client polling policy
- **src/gateway/routes/assets.py**:
  - `GET /robot/meshes/{filename}` — Serve robot STL mesh files
- **src/gateway/routes/camera.py**:
  - `GET /api/v1/camera/snapshot` — Camera JPEG snapshot
- **src/gateway/routes/commands.py**:
  - `POST /api/v1/estop/reset` — Explicitly release the native software emergency-stop latch
  - `POST /api/v1/instruction` — Natural language navigation instruction
  - `POST /api/v1/lease` — Acquire/release/renew control lease
  - `POST /api/v1/mode` — Switch operating mode
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
  - `GET /api/v1/runtime/dataflow` — Read-only Product motion and Gateway observability
  - `POST /api/v1/runtime/dataflow/subscribe` — Create a read-only Gateway SSE subscription plan
  - `GET /api/v1/runtime/dataflow/topic` — Inspect one Gateway-observable Product topic
- **src/gateway/routes/health.py**:
  - `GET /api/v1/health` — System health overview
  - `GET /api/v1/metrics` — Operator-facing runtime metrics snapshot
  - `GET /api/v1/readiness` — Client readiness snapshot
  - `GET /health` — Liveness probe
  - `GET /ready` — Readiness probe
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
- **src/gateway/routes/realtime.py**:
  - `GET /api/v1/events` — SSE event stream
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
  - `GET /api/v1/localization/status` — Localization status for app and web clients
  - `GET /api/v1/scene_graph` — Current scene graph
  - `GET /api/v1/state` — Full robot state snapshot

### src/gateway/maps/locations.py

#### `GET /api/v1/locations`
**Summary:** List tagged navigation locations
**Response model:** `LocationsResponse`
**Handler:** `get_locations`

#### `POST /api/v1/locations`
**Summary:** Create or update a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `post_location`

#### `DELETE /api/v1/locations/{name}`
**Summary:** Delete a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `delete_location`

#### `PUT /api/v1/locations/{name}`
**Summary:** Update a tagged navigation location
**Response model:** `LocationOperationResponse`
**Handler:** `put_location`

### src/gateway/maps/places.py

#### `GET /api/v1/places`
**Summary:** List canonical semantic places
**Response model:** `PlaceListResponse`
**Handler:** `get_places`

#### `POST /api/v1/places`
**Summary:** Create or update a canonical semantic place on a native map
**Response model:** `PlaceUpsertResponse`
**Handler:** `post_place`

#### `GET /api/v1/places/resolve`
**Summary:** Resolve a canonical semantic place by name or alias
**Response model:** `PlaceResolveResponse`
**Handler:** `resolve_place`

### src/gateway/maps/routes.py

#### `GET /api/v1/map/points`
**Summary:** Map point cloud as JSON (from ikd-tree snapshot)
**Response model:** `MapPointsResponse`
**Handler:** `get_map_points`

#### `POST /api/v1/map/rename`
**Summary:** Rename a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `rename_map`

#### `POST /api/v1/map/save`
**Summary:** Save current SLAM map
**Response model:** `MapSaveOperationResponse`
**Handler:** `save_map_now`

#### `POST /api/v1/map_cloud/reset`
**Summary:** Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)
**Response model:** `MapLifecycleResponse`
**Handler:** `reset_map_cloud`

#### `GET /api/v1/maps/environment/layers`
**Summary:** Operator-facing environment map layer state
**Handler:** `get_environment_map_layers`

#### `POST /api/v1/maps/import_pcd`
**Summary:** Import a PCD file into a LingTu map package
**Response model:** `MapLifecycleResponse`
**Handler:** `import_pcd_map`

#### `GET /api/v1/maps/operations`
**Summary:** List durable map-save operations
**Response model:** `MapSaveOperationResponse`
**Handler:** `list_map_operations`

#### `GET /api/v1/maps/operations/{operation_id}`
**Summary:** Get a durable map-save operation
**Response model:** `MapSaveOperationResponse`
**Handler:** `get_map_operation`

#### `POST /api/v1/maps/operations/{operation_id}/cancel`
**Summary:** Cancel a durable map-save operation
**Response model:** `MapSaveOperationResponse`
**Handler:** `cancel_map_operation`

#### `POST /api/v1/maps/operations/{operation_id}/retry`
**Summary:** Retry a failed durable map-save operation
**Response model:** `MapSaveOperationResponse`
**Handler:** `retry_map_operation`

#### `DELETE /api/v1/maps/{name}`
**Summary:** Delete a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `delete_saved_map`

#### `POST /api/v1/maps/{name}/build_occupancy`
**Summary:** Build a 2D occupancy artifact from a saved map
**Response model:** `MapLifecycleResponse`
**Handler:** `build_saved_map_occupancy`

#### `POST /api/v1/maps/{name}/build_octomap`
**Summary:** Build OctoPlanner3D octomap.ot from saved map.pcd
**Response model:** `MapLifecycleResponse`
**Handler:** `build_saved_map_octomap`

#### `POST /api/v1/maps/{name}/crop`
**Summary:** Crop a saved map point cloud and invalidate derived artifacts
**Response model:** `MapLifecycleResponse`
**Handler:** `crop_saved_map`

#### `POST /api/v1/maps/{name}/mark_zone`
**Summary:** Mark occupied/free/preblocked/traversable zones in the saved OctoMap
**Response model:** `MapLifecycleResponse`
**Handler:** `mark_saved_map_zone`

#### `GET /api/v1/maps/{name}/pcd`
**Summary:** Serve raw PCD file for inline preview
**Handler:** `get_map_pcd`

#### `GET /api/v1/maps/{name}/points`
**Summary:** Saved map point cloud as JSON
**Response model:** `MapPointsResponse`
**Handler:** `get_saved_map_points`

#### `POST /api/v1/maps/{name}/validate_plan`
**Summary:** Preview a route on the active saved map without publishing a goal
**Handler:** `validate_saved_map_plan`

#### `POST /api/v1/maps/{name}/voxels/edit`
**Summary:** Edit saved OctoMap voxels for OctoPlanner3D
**Response model:** `MapLifecycleResponse`
**Handler:** `edit_saved_map_voxels`

#### `GET /api/v1/maps/{name}/voxels/edits`
**Summary:** Saved OctoMap voxel edit overlay
**Handler:** `get_saved_map_voxel_edits`

#### `GET /api/v1/slam/maps`
**Summary:** List maps through native mapd
**Response model:** `MapListResponse`
**Handler:** `slam_maps`

#### `GET /map/viewer`
**Summary:** Interactive 3D map viewer
**Handler:** `map_viewer`

### src/gateway/mcp_server.py

#### `GET /capabilities`
**Handler:** `capabilities`

#### `GET /health`
**Handler:** `health`

#### `POST /mcp`
**Handler:** `mcp_endpoint`

### src/gateway/navigation/diagnostics.py

#### `GET /api/v1/navigation/dds_snapshot`
**Summary:** Latest navigation data for the native DDS endpoint
**Response model:** `NavigationDdsSnapshotResponse`
**Handler:** `get_navigation_dds_snapshot`

#### `GET /api/v1/path`
**Summary:** Latest planned path
**Response model:** `PathResponse`
**Handler:** `get_path`

### src/gateway/navigation/routes.py

#### `POST /api/v1/goal`
**Summary:** Send navigation goal
**Response model:** `ControlCommandResponse`
**Handler:** `post_goal`

#### `POST /api/v1/navigate/click`
**Summary:** Navigate to map-viewer click point
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigate_click`

#### `POST /api/v1/navigation/cancel`
**Summary:** Compatibility cancel for the current navigation mission
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_cancel`

#### `POST /api/v1/navigation/goal_candidate`
**Summary:** Construct and optionally preview a navigation goal without publishing it
**Response model:** `GoalCandidateResponse`
**Handler:** `post_navigation_goal_candidate`

#### `GET /api/v1/navigation/goals/{request_id}`
**Summary:** Request-correlated native navigation lifecycle status
**Response model:** `NavigationGoalStatusQueryResponse`
**Handler:** `get_navigation_goal_status`

#### `POST /api/v1/navigation/plan`
**Summary:** Preview navigation plan without publishing a goal
**Response model:** `PlanPreviewResponse`
**Handler:** `post_navigation_plan`

#### `POST /api/v1/navigation/resume`
**Summary:** Release a native motion hold without replaying previous motion
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_resume`

#### `GET /api/v1/navigation/status`
**Summary:** Public navigation status
**Response model:** `NavigationStatusResponse`
**Handler:** `get_navigation_status`

#### `GET /api/v1/navigation/tasks/{task_id}`
**Summary:** Stable native navigation task lifecycle status
**Response model:** `NavigationTaskStatusQueryResponse`
**Handler:** `get_navigation_task_status`

#### `POST /api/v1/navigation/tasks/{task_id}/cancel`
**Summary:** Request cancellation of one navigation task
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_task_cancel`

#### `POST /api/v1/navigation/tasks/{task_id}/pause`
**Summary:** Request a stop-confirmed pause of one navigation task
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_task_pause`

#### `POST /api/v1/navigation/tasks/{task_id}/resume`
**Summary:** Request continuation of the same paused navigation task
**Response model:** `ControlCommandResponse`
**Handler:** `post_navigation_task_resume`

### src/gateway/routes/app.py

#### `GET /api/v1/app/bootstrap`
**Summary:** App/Web bootstrap snapshot
**Response model:** `AppBootstrapResponse`
**Handler:** `app_bootstrap`

#### `GET /api/v1/app/capabilities`
**Summary:** App/Web API capability manifest
**Response model:** `AppCapabilitiesResponse`
**Handler:** `app_capabilities`

#### `GET /api/v1/app/traffic`
**Summary:** App/Web realtime traffic and client polling policy
**Response model:** `AppTrafficResponse`
**Handler:** `app_traffic`

### src/gateway/routes/assets.py

#### `GET /robot/meshes/{filename}`
**Summary:** Serve robot STL mesh files
**Handler:** `serve_robot_mesh`

### src/gateway/routes/camera.py

#### `GET /api/v1/camera/snapshot`
**Summary:** Camera JPEG snapshot
**Handler:** `camera_snapshot`

### src/gateway/routes/commands.py

#### `POST /api/v1/estop/reset`
**Summary:** Explicitly release the native software emergency-stop latch
**Response model:** `ControlCommandResponse`
**Handler:** `post_estop_reset`

#### `POST /api/v1/instruction`
**Summary:** Natural language navigation instruction
**Response model:** `ControlCommandResponse`
**Handler:** `post_instruction`

#### `POST /api/v1/lease`
**Summary:** Acquire/release/renew control lease
**Response model:** `LeaseResponse`
**Handler:** `post_lease`

#### `POST /api/v1/mode`
**Summary:** Switch operating mode
**Response model:** `ControlCommandResponse`
**Handler:** `post_mode`

#### `POST /api/v1/stop`
**Summary:** Emergency stop
**Response model:** `ControlCommandResponse`
**Handler:** `post_stop`

#### `POST /api/v1/visual_servo`
**Summary:** Set visual servo target
**Response model:** `ControlCommandResponse`
**Handler:** `post_visual_servo`

### src/gateway/routes/diagnostics.py

#### `GET /api/v1/diagnostic_pack`
**Summary:** Export diagnostic tarball
**Handler:** `diagnostic_pack`

#### `POST /api/v1/diagnostics/field-check`
**Summary:** Run read-only product field readiness check
**Response model:** `ProductFieldCheckResponse`
**Handler:** `product_field_check`

#### `GET /api/v1/diagnostics/plugins`
**Summary:** Read registered plugin categories and providers
**Handler:** `plugin_catalog`

#### `GET /api/v1/diagnostics/real-runtime-evidence/latest`
**Summary:** Read latest Thunder field runtime evidence gate summary
**Response model:** `RealRuntimeEvidenceLatestResponse`
**Handler:** `real_runtime_evidence_latest`

#### `GET /api/v1/diagnostics/routecheck/latest`
**Summary:** Read latest non-motion routecheck summary
**Response model:** `RoutecheckLatestResponse`
**Handler:** `routecheck_latest`

#### `GET /api/v1/diagnostics/runtime-contract`
**Summary:** Read canonical runtime interface contract
**Response model:** `RuntimeContractResponse`
**Handler:** `runtime_contract`

#### `POST /api/v1/inspection/acceptance`
**Summary:** Run read-only inspection acceptance without publishing motion commands
**Response model:** `InspectionAcceptanceResponse`
**Handler:** `inspection_acceptance`

#### `GET /api/v1/runtime/dataflow`
**Summary:** Read-only Product motion and Gateway observability
**Response model:** `RuntimeDataflowResponse`
**Handler:** `get_runtime_dataflow`

#### `POST /api/v1/runtime/dataflow/subscribe`
**Summary:** Create a read-only Gateway SSE subscription plan
**Response model:** `RuntimeDataflowSubscribeResponse`
**Handler:** `post_runtime_dataflow_subscribe`

#### `GET /api/v1/runtime/dataflow/topic`
**Summary:** Inspect one Gateway-observable Product topic
**Response model:** `RuntimeDataflowTopicDetailResponse`
**Handler:** `get_runtime_dataflow_topic`

### src/gateway/routes/health.py

#### `GET /api/v1/health`
**Summary:** System health overview
**Response model:** `HealthResponse`
**Handler:** `get_health`

#### `GET /api/v1/metrics`
**Summary:** Operator-facing runtime metrics snapshot
**Handler:** `get_metrics`

#### `GET /api/v1/readiness`
**Summary:** Client readiness snapshot
**Response model:** `ReadinessResponse`
**Handler:** `api_readiness`

#### `GET /health`
**Summary:** Liveness probe
**Response model:** `LivenessResponse`
**Handler:** `liveness_health`

#### `GET /ready`
**Summary:** Readiness probe
**Response model:** `ReadinessResponse`
**Handler:** `readiness_ready`

### src/gateway/routes/inspection.py

#### `GET /api/v1/inspection/evidence`
**Summary:** List recent verified inspection evidence
**Handler:** `list_inspection_evidence`

#### `GET /api/v1/inspection/evidence/{evidence_id}`
**Summary:** Read one verified inspection evidence manifest
**Handler:** `get_inspection_evidence`

#### `GET /api/v1/inspection/evidence/{evidence_id}/artifacts/{kind}`
**Summary:** Read one verified inspection evidence artifact
**Handler:** `get_inspection_evidence_artifact`

#### `GET /api/v1/inspection/routes`
**Summary:** List native inspection routes for a map
**Response model:** `InspectionRouteListResponse`
**Handler:** `list_inspection_routes`

#### `POST /api/v1/inspection/routes`
**Summary:** Create or update a native inspection route
**Response model:** `InspectionRouteResponse`
**Handler:** `put_inspection_route`

#### `DELETE /api/v1/inspection/routes/{route_id}`
**Summary:** Delete one native inspection route
**Response model:** `InspectionCommandResponse`
**Handler:** `delete_inspection_route`

#### `GET /api/v1/inspection/routes/{route_id}`
**Summary:** Read one native inspection route
**Response model:** `InspectionRouteResponse`
**Handler:** `get_inspection_route`

#### `GET /api/v1/inspection/status`
**Summary:** Read native inspection store/status snapshot
**Response model:** `InspectionStatusResponse`
**Handler:** `inspection_status`

#### `GET /api/v1/inspection/tasks`
**Summary:** List retained inspection task projections
**Response model:** `InspectionTaskListResponse`
**Handler:** `list_inspection_tasks`

#### `POST /api/v1/inspection/tasks`
**Summary:** Submit a task-addressed native inspection route
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `start_inspection_task`

#### `GET /api/v1/inspection/tasks/{task_id}`
**Summary:** Read the fact-backed state of one inspection task
**Response model:** `InspectionTaskStatusResponse`
**Handler:** `get_inspection_task`

#### `POST /api/v1/inspection/tasks/{task_id}/cancel`
**Summary:** Request cancellation for one native inspection task
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `cancel_inspection_task`

#### `POST /api/v1/inspection/tasks/{task_id}/pause`
**Summary:** Request pause for one native inspection task
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `pause_inspection_task`

#### `GET /api/v1/inspection/tasks/{task_id}/report`
**Summary:** Read the business result of one inspection task
**Response model:** `InspectionTaskReportResponse`
**Handler:** `get_inspection_task_report`

#### `POST /api/v1/inspection/tasks/{task_id}/resume`
**Summary:** Request resume for one native inspection task
**Response model:** `InspectionTaskCommandResponse`
**Handler:** `resume_inspection_task`

### src/gateway/routes/operations.py

#### `POST /api/v1/explore/directed`
**Summary:** Set an explicit native TARE exploration direction intent
**Response model:** `DirectedExplorationResponse`
**Handler:** `explore_directed`

#### `POST /api/v1/explore/directed/clear`
**Summary:** Clear the explicit native TARE exploration direction intent
**Response model:** `DirectedExplorationResponse`
**Handler:** `clear_explore_directed`

#### `GET /api/v1/explore/runs`
**Summary:** List recent durable native Explore executions
**Response model:** `ExplorationRunListResponse`
**Handler:** `explore_runs_list`

#### `GET /api/v1/explore/runs/{exploration_run_id}`
**Summary:** Query one durable native Explore execution
**Response model:** `ExplorationRunResponse`
**Handler:** `explore_run_get`

#### `POST /api/v1/explore/start`
**Summary:** Start autonomous frontier exploration
**Response model:** `ExplorationCommandResponse`
**Handler:** `explore_start`

#### `GET /api/v1/explore/status`
**Summary:** Exploration status
**Response model:** `ExplorationStatusResponse`
**Handler:** `explore_status`

#### `POST /api/v1/explore/stop`
**Summary:** Stop autonomous frontier exploration
**Response model:** `ExplorationCommandResponse`
**Handler:** `explore_stop`

#### `POST /api/v1/localization/map-tracking`
**Summary:** Start continuous tracking against the running Product's active map
**Response model:** `LocalizationOperationResponse`
**Handler:** `localization_map_tracking`

#### `POST /api/v1/localization/relocalizations`
**Summary:** Relocalize against the running Product's active map
**Response model:** `LocalizationOperationResponse`
**Handler:** `localization_relocalize`

#### `GET /api/v1/memory/temporal`
**Summary:** Query temporal entity observations
**Response model:** `TemporalMemoryResponse`
**Handler:** `get_temporal_memory`

#### `POST /api/v1/memory/temporal/semantic`
**Summary:** Semantic similarity search over temporal observations
**Response model:** `TemporalMemoryResponse`
**Handler:** `post_temporal_semantic`

#### `POST /api/v1/recordings/start`
**Summary:** Start native MCAP recording
**Response model:** `RecordingOperationResponse`
**Handler:** `recording_start`

#### `GET /api/v1/recordings/status`
**Summary:** Native MCAP recording status
**Response model:** `RecordingStatusResponse`
**Handler:** `recording_status`

#### `POST /api/v1/recordings/stop`
**Summary:** Stop native MCAP recording
**Response model:** `RecordingOperationResponse`
**Handler:** `recording_stop`

#### `GET /api/v1/services/status`
**Summary:** Product service status
**Response model:** `ServiceStatusResponse`
**Handler:** `service_status`

#### `GET /api/v1/slam/status`
**Summary:** SLAM service status
**Response model:** `SlamStatusResponse`
**Handler:** `slam_status`

#### `GET /api/v1/webrtc/go2rtc/status`
**Summary:** Probe the go2rtc sidecar (image transmission fast path)
**Response model:** `Go2RTCStatusResponse`
**Handler:** `get_go2rtc_status`

#### `POST /api/v1/webrtc/whep`
**Summary:** WHEP signalling proxy to go2rtc (image transmission path)
**Handler:** `post_webrtc_whep`

### src/gateway/routes/realtime.py

#### `GET /api/v1/events`
**Summary:** SSE event stream
**Handler:** `sse_events`

### src/gateway/routes/recordings.py

#### `GET /api/v1/recordings`
**Handler:** `recording_list`

#### `DELETE /api/v1/recordings/{session_id}`
**Handler:** `recording_remove`

#### `GET /api/v1/recordings/{session_id}`
**Handler:** `recording_detail`

#### `GET /api/v1/recordings/{session_id}/files/{artifact_path:path}`
**Handler:** `recording_artifact`

### src/gateway/routes/safety.py

#### `GET /api/v1/safety/modes/estop`
**Summary:** Read emergency-stop state
**Response model:** `SafetyEstopResponse`
**Handler:** `get_safety_estop`

#### `POST /api/v1/safety/modes/estop`
**Summary:** Activate emergency stop
**Response model:** `SafetyEstopResponse`
**Handler:** `post_safety_estop`

### src/gateway/routes/session.py

#### `GET /api/v1/session`
**Summary:** Current Product session state and capabilities
**Response model:** `SessionResponse`
**Handler:** `session_get`

### src/gateway/routes/status.py

#### `GET /api/v1/localization/status`
**Summary:** Localization status for app and web clients
**Response model:** `LocalizationStatusResponse`
**Handler:** `get_localization_status`

#### `GET /api/v1/scene_graph`
**Summary:** Current scene graph
**Response model:** `SceneGraphResponse`
**Handler:** `get_scene_graph`

#### `GET /api/v1/state`
**Summary:** Full robot state snapshot
**Response model:** `StateResponse`
**Handler:** `get_state`
