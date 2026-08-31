# MCP Tools (Auto-Discovered @skill Methods)

> Auto-generated from `@skill` decorators across all Module files.
> Deterministic generated inventory; run `python tools/docs/extract_api_docs.py --check` to verify freshness.

These tools are auto-discovered by `MCPServerModule` and exposed via JSON-RPC
at `http://<robot>:8090/mcp`. They are also available in the AgentLoop for
multi-turn LLM tool calling.

---

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
