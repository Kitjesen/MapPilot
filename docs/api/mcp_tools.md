# MCP Tools (Auto-Discovered @skill Methods)

> Auto-generated from `@skill` decorators across all Module files.
> Generated: 2026-07-18 14:31:17

These tools are auto-discovered by `MCPServerModule` and exposed via JSON-RPC
at `http://<robot>:8090/mcp`. They are also available in the AgentLoop for
multi-turn LLM tool calling.

---

## src\decision\modules\semantic_planner.py
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

## src\decision\modules\visual_servo.py
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

### `tune_bbox_gains`
**Module:** `VisualServoModule`
**Description:** Report that live bbox gain tuning is not available from this module.
**Return type:** `dict`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `duration` | `float` |

## src\decision\modules\vla.py
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

## src\drivers\real\teleop_module.py
_TeleopModule — joystick remote control with live camera stream._

### `get_teleop_status`
**Module:** `TeleopModule`
**Description:** Return current teleop status.
**Return type:** `str`
**Parameters:** None

### `force_release`
**Module:** `TeleopModule`
**Description:** Force-release teleop control and resume autonomy.
**Return type:** `str`
**Parameters:** None

## src\explore\frontier.py
_WavefrontFrontierExplorer — autonomous frontier-based exploration planner._

### `begin_exploration`
**Module:** `WavefrontFrontierExplorer`
**Description:** Start autonomous frontier exploration. Returns status string.
**Return type:** `str`
**Parameters:** None

### `end_exploration`
**Module:** `WavefrontFrontierExplorer`
**Description:** Stop autonomous frontier exploration. Returns status string.
**Return type:** `str`
**Parameters:** None

### `get_frontiers`
**Module:** `WavefrontFrontierExplorer`
**Description:** Return the most recently computed frontier cluster list.
**Return type:** `str`
**Parameters:** None

### `clear_frontier_blocks`
**Module:** `WavefrontFrontierExplorer`
**Description:** Clear navigation-failed frontier goals from the local block list.
**Return type:** `str`
**Parameters:** None

## src\explore\tare\module.py
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

## src\explore\tare\supervisor.py
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

## src\explore\traversable_frontier.py
_Traversable frontier candidate generation for inspection/exploration preview._

### `get_traversable_frontiers`
**Module:** `TraversableFrontierModule`
**Description:** Return ranked frontier candidates enriched with traversability evidence.
**Return type:** `str`
**Parameters:** None

### `refresh_candidates`
**Module:** `TraversableFrontierModule`
**Description:** Compute and publish candidates without publishing any motion command.
**Return type:** `str`
**Parameters:** None

## src\gateway\mcp_server.py
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

### `switch_backend`
**Module:** `MCPServerModule`
**Description:** Switch a low-risk runtime backend; motion backends require navigation IDLE.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `category` | `str` |
| `backend` | `str` |
| `config_json` | `str` |

## src\gateway\visualization\rerun_bridge.py
_RerunBridgeModule — on-demand Rerun visualization as a Module._

### `start_rerun`
**Module:** `RerunBridgeModule`
**Description:** Start Rerun web viewer. Returns URL.
**Return type:** `str`
**Parameters:** None

### `stop_rerun`
**Module:** `RerunBridgeModule`
**Description:** Stop Rerun visualization and release resources.
**Return type:** `str`
**Parameters:** None

### `rerun_status`
**Module:** `RerunBridgeModule`
**Description:** Return Rerun status.
**Return type:** `str`
**Parameters:** None

## src\localization\ntrip_client_module.py
_NtripClientModule — pull RTCM differential corrections from a NTRIP caster_

### `get_ntrip_status`
**Module:** `NtripClientModule`
**Description:** Return NTRIP caster connection state: host, mount, bytes received,
**Return type:** `str`
**Parameters:** None

## src\localization\slam\module.py
_Native SLAM Module contract._

### `relocalize`
**Module:** `SlamModule`
**Description:** Relocalize against the loaded map, optionally with a pose guess.
**Return type:** `dict[(str, Any)]`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `x` | `float | None` |
| `y` | `float | None` |
| `z` | `float` |
| `yaw` | `float` |

## src\maps\modules\voxel_grid.py
_VoxelGridModule - native C++ 3D voxel layer from LiDAR point clouds._

### `get_voxel_stats`
**Module:** `VoxelGridModule`
**Description:** Return stats about the current native voxel map.
**Return type:** `str`
**Parameters:** None

### `checkpoint_voxels`
**Module:** `VoxelGridModule`
**Description:** Save the configured native voxel state file.
**Return type:** `str`
**Parameters:** None

### `clear_voxels`
**Module:** `VoxelGridModule`
**Description:** Reset the native voxel map.
**Return type:** `str`
**Parameters:** None

### `query_voxel`
**Module:** `VoxelGridModule`
**Description:** Check whether the voxel containing (x, y, z) is occupied.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `x` | `float` |
| `y` | `float` |
| `z` | `float` |

## src\maps\services\facade.py
_Maps Module compatibility helpers, public skills, and health._

### `list_maps`
**Module:** `MapsFacadeMixin`
**Description:** List saved maps and which one is active.
**Return type:** `str`
**Parameters:** None

### `list_map_types`
**Module:** `MapsFacadeMixin`
**Description:** List supported map classes, artifacts, and capabilities.
**Return type:** `str`
**Parameters:** None

### `save_map`
**Module:** `MapsFacadeMixin`
**Description:** Save current SLAM map as *name* and build all artifacts.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `name` | `str` |
| `slam_profile` | `str | None` |

### `use_map`
**Module:** `MapsFacadeMixin`
**Description:** Activate *name* as the current map.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `name` | `str` |

## src\memory\modules\episodic_module.py
_EpisodicMemoryModule — 时空情节记忆模块 (Module 模式封装)。_

### `get_recent_observations`
**Module:** `EpisodicMemoryModule`
**Description:** Return the most recent episodic observations (labels + position).
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `count` | `int` |

## src\memory\modules\mission_logger_module.py
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

## src\memory\modules\semantic_mapper_module.py
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

## src\memory\modules\tagged_locations_module.py
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

## src\memory\modules\temporal_memory_module.py
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

## src\memory\modules\vector_memory_module.py
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

## src\nav\services\safety\safety_ring.py
_SafetyRing -unified safety + evaluation + dialogue in one Module._

### `get_safety_status`
**Module:** `SafetyRing`
**Description:** Get current safety state: level, cross-track error, distance to goal, and module health.
**Return type:** `str`
**Parameters:** None

### `emergency_stop`
**Module:** `SafetyRing`
**Description:** Trigger an emergency stop (safety level STOP). Use for immediate halt.
**Return type:** `str`
**Parameters:** None

## src\nav\skills\skills_module.py

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
**Parameters:** None

### `cancel_mission`
**Module:** `NavSkills`
**Description:** Cancel the active navigation mission.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `reason` | `str` |

### `get_navigation_status`
**Module:** `NavSkills`
**Description:** Return the canonical navigation mission status.
**Return type:** `str`
**Parameters:** None

### `start_patrol`
**Module:** `NavSkills`
**Description:** Submit a structured map-frame patrol route.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `waypoints` | `list[dict[(str, Any)]]` |
| `loop` | `bool` |

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

### `navigate_relative`
**Module:** `NavSkills`
**Description:** Submit a goal relative to the current robot pose.
**Return type:** `str`
**Parameters:**
| Parameter | Type |
|-----------|------|
| `dx` | `float` |
| `dy` | `float` |
| `dyaw` | `float` |

### `get_robot_pose`
**Module:** `NavSkills`
**Description:** Return the latest robot pose in the planning frame.
**Return type:** `str`
**Parameters:** None
