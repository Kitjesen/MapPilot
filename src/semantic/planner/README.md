# Semantic Planner — Module Index

> Source: `src/semantic/planner/semantic_planner/`  
> Architecture: **Module-First** — every runtime unit is a `core.Module`.  
> No standalone ROS2 nodes. C++ / ROS2 integration is handled by `NativeModule` wrappers in `src/slam/` and `src/base_autonomy/`.

---

## Runtime Modules (instantiated by Blueprint)

| File | Layer | Ports (In → Out) | Role |
|------|------:|-------------------|------|
| `semantic_planner_module.py` | L4 | `instruction, agent_instruction, scene_graph, odometry, detections, mission_status` → `goal_pose, task_plan, planner_status, cancel, servo_target` | Unified planner: decompose → resolve → explore → execute. Embeds GoalResolver, FrontierScorer, TaskDecomposer, ActionExecutor as internal strategies. LERa 3-step recovery on STUCK/FAILED. **@skill**: `send_instruction`, `get_planner_status`, `decompose_task`. |
| `visual_servo_module.py` | L4 | `servo_target, image, detections, depth, odometry` → `goal_pose, cmd_vel, servo_status` | Visual servo: BBoxNavigator (far ≥3 m → goal_pose, near <3 m → cmd_vel). PersonTracker with Kalman + Re-ID. **@skill**: `find_object`, `follow_person`, `stop_servo`, `get_servo_status`. |
| `llm_module.py` | L4 | `llm_request` → `llm_response` | Multi-backend LLM wrapper (kimi / openai / claude / qwen). Stateless; exposes `chat()` as an `@rpc`. |
| `goal_resolver_module.py` | L4 | `instruction, scene_graph, odometry` → `goal_pose, resolver_status` | Thin Module wrapper around `GoalResolver` (Fast-Slow dual process). |
| `task_decomposer_module.py` | L4 | `instruction` → `task_plan` | Thin Module wrapper around `TaskDecomposer`. |
| `frontier_module.py` | L3 | `occupancy_grid, scene_graph, odometry` → `frontier_goal, frontier_status` | Frontier-exploration sub-module used by `SemanticPlannerModule`. |
| `action_executor_module.py` | L4 | `subtask, mission_status` → `goal_pose, exec_status` | Executes decomposed subtasks; drives LERa retry/expand/requery/abort. |

---

## Goal Resolution Algorithms (strategy objects, not Modules)

These are plain Python classes instantiated **inside** `SemanticPlannerModule` — not separate runtime units.

| File | Role |
|------|------|
| `goal_resolver.py` | Fast-Slow coordinator: routes to Fast Path / Slow Path, applies AdaNav entropy trigger, integrates OmniNav hierarchical sub-goals. |
| `fast_path.py` | System 1 (~0.17 ms): keyword + scene-graph match, confidence fusion (label 35% + CLIP 35% + detector 15% + spatial 15%). Target >70 % hit rate, threshold 0.75. |
| `slow_path.py` | System 2 (~2 s): ESCA selective grounding (200→~15 objects, 92.5% token reduction), LLM reasoning, OmniNav room hint. |
| `adacot.py` | AdaCoT routing: 7-dim rules + Shannon entropy → FAST / SLOW / AUTO decision. |

---

## Execution Strategies (used inside SemanticPlannerModule)

| File | Role |
|------|------|
| `action_executor.py` | Primitive executor; LERa 3-step failure recovery (retry_different_path → expand_search → requery_goal → abort). |
| `bbox_navigator.py` | BBox → 3D → PD servo (far: goal_pose, near: cmd_vel). |
| `vlm_bbox_query.py` | VLM open-vocabulary bbox query; multi-round probing + confidence filter. |
| `person_tracker.py` | Kalman + CLIP Re-ID person following; dual-channel (far / near) same as BBoxNavigator. |
| `exploration_strategy.py` | Frontier selection + coverage priority scheduling. |

---

## Task Decomposition

| File | Role |
|------|------|
| `task_decomposer.py` | SayCan-style decomposer: natural-language instruction → ordered sub-goals. |
| `task_rules.py` | Rule library: place-skill mapping, task templates, pre/post-condition constraints. |

---

## Memory (strategy objects — Module wrappers live in `src/memory/modules/`)

| File | Role |
|------|------|
| `episodic_memory.py` | Local copy / alias of `memory.spatial.episodic.EpisodicMemory`. Used by SemanticPlannerModule internally. |
| `topological_memory.py` | Topology graph: TopoNode + FSR-VLN Jaccard edges + VLingMem region summaries. |
| `tagged_locations.py` | Local copy / alias of `memory.spatial.tagged_locations.TaggedLocationStore`. |
| `semantic_prior.py` | Semantic priors: place CLIP descriptions, `predict_room_type_from_labels()`, room-object co-occurrence. |

---

## Frontier & Scene Reasoning

| File | Role |
|------|------|
| `frontier_scorer.py` | Frontier scoring: MTU3D grounding potential + L3MVN NL description + TSP ordering. |
| `frontier_types.py` | Data types: `FrontierNode`, `FrontierCluster`, `ScoredFrontier`. |
| `sgnav_reasoner.py` | SGNav scene-graph reasoning: multi-view `ObservationRecord` accumulation + consistency voting. |

---

## LLM Client & Prompts

| File | Role |
|------|------|
| `llm_client.py` | Multi-backend async LLM client (kimi / openai / claude / qwen), auto-fallback, retry, streaming. |
| `prompt_templates.py` | Prompt templates: H-CoT 4-step reasoning, frontier descriptions, explored summaries. |
| `chinese_tokenizer.py` | jieba tokenizer wrapper: Chinese keyword extraction, stop-word filtering, Fast Path integration. |

---

## Planner State

| File | Role |
|------|------|
| `planner_state.py` | Enum: `PlannerState` (IDLE / PLANNING / EXECUTING / STUCK / ERROR). |

---

## Utilities

| File | Role |
|------|------|
| `implicit_fsm_policy.py` | LOVON-style implicit FSM navigation policy (no explicit state machine). |
| `voi_scheduler.py` | VOI (Value of Information) scheduler: decides when to trigger perception / LLM queries. |
| `room_object_kg.py` | Room-object knowledge graph: prior co-occurrence relations, `predict_room_type`, ESCA filter. |
| `vlm_scene_agent.py` | VLM-based scene agent: open-ended visual question answering over the scene graph. |
| `agent_loop.py` | Multi-turn agent loop (observe → think → act), 7 tool calls, max 10 steps / 120 s timeout. |

---

## Legacy (do not import in new code)

| File | Role |
|------|------|
| `src/legacy/semantic/skill_registry.py` | Old `@skill` decorator + LangChain `StructuredTool` registry. Superseded by `core.module.skill` + `MCPServerModule` auto-discovery. |

---

## MCP Skill Summary

Tools auto-discovered by `MCPServerModule.on_system_modules()`:

| Tool name | Source module |
|-----------|--------------|
| `send_instruction` | `SemanticPlannerModule` |
| `get_planner_status` | `SemanticPlannerModule` |
| `decompose_task` | `SemanticPlannerModule` |
| `find_object` | `VisualServoModule` |
| `follow_person` | `VisualServoModule` |
| `stop_servo` | `VisualServoModule` |
| `get_servo_status` | `VisualServoModule` |
| `navigate_to` | `NavigationModule` (L5) |
| `stop_navigation` | `NavigationModule` (L5) |
| `cancel_mission` | `NavigationModule` (L5) |
| `get_navigation_status` | `NavigationModule` (L5) |
| `start_patrol` | `NavigationModule` (L5) |
| `list_missions` | `MissionLoggerModule` |
| `get_mission_stats` | `MissionLoggerModule` |
| `list_maps` | `MapManagerModule` |
| `save_map` | `MapManagerModule` |
| `use_map` | `MapManagerModule` |
| `build_tomogram` | `MapManagerModule` |
| `list_tags` | `TaggedLocationsModule` |
| `go_to_tag` | `TaggedLocationsModule` |
| `get_recent_observations` | `EpisodicMemoryModule` |
| `query_location` | `VectorMemoryModule` |
| `add_observation` | `VectorMemoryModule` |
