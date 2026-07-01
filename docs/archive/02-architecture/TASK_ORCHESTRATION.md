# Task Orchestration

LingTu is one of several agents that can drive the Thunder quadruped. This
document describes the contract between the external orchestrator
(NOVA Dog Runtime) and LingTu's gRPC entry point, and the trigger paths
that bypass the orchestrator (development, debugging, fallback).

The orchestrator and gateway pieces live outside this repository (in
`products/nova-dog/runtime/`); they are summarised here only as far as
LingTu cares about them.

## Stack

```
┌─────────────────────────────────────────────────────────────────────┐
│                NOVA Dog Runtime  (products/nova-dog/runtime)        │
│                                                                     │
│  ┌─────────────┐   ┌─────────────────────┐   ┌────────────────────┐ │
│  │   Askme     │──▶│ mission-orchestrator│──▶│   nav-gateway      │ │
│  │ (voice)     │   │ (sole task owner)   │   │ (protocol bridge)  │ │
│  └─────────────┘   └─────────────────────┘   └─────────┬──────────┘ │
│                                                        │            │
│  ┌─────────────┐                                       │            │
│  │ Flutter App │── gRPC :50051 ────────────────────────┤            │
│  │ (UI)        │                                       │            │
│  └─────────────┘                                       │            │
└────────────────────────────────────────────────────────┼────────────┘
                                                         │
                                              LingTuGrpcBridge
                                              gRPC :50051 (LingTu)
                                                         │
┌────────────────────────────────────────────────────────┼────────────┐
│                           LingTu                       │            │
│                                                        ▼            │
│   ControlService → TaskManager                                       │
│       ↓                                                              │
│   GatewayModule.instruction Out[str]   (or .goal_pose Out[PoseStamped]) │
│       ↓                                                              │
│   SemanticPlannerModule._on_instruction → GoalResolver              │
│       ↓                                                              │
│   NavigationModule.goal_pose                                         │
│       ↓                                                              │
│   GlobalPlannerService → WaypointTracker                            │
│       ↓                                                              │
│   LocalPlannerModule → PathFollowerModule → CmdVelMux                │
│       ↓                                                              │
│   ThunderDriver → gRPC :13145 → brainstem → motors                  │
└─────────────────────────────────────────────────────────────────────┘
```

The new modular paths use Module ports instead of ROS2 topics: an
instruction arrives at `GatewayModule.instruction Out[str]`, fans out to
`SemanticPlannerModule.instruction In[str]` and `NavigationModule.instruction In[str]`.
Goals fan out from `GatewayModule`, `MCPServerModule`, and
`SemanticPlannerModule` into `NavigationModule.goal_pose` (see
`full_stack.py`).

## Trigger paths

| Method | Entry | Path | When to use |
|--------|-------|------|-------------|
| Voice | Askme `"go to the kitchen"` | Askme → orchestrator → nav-gateway → LingTu | Production NOVA Dog |
| Flutter App | UI tap | App → LingTu gRPC `:50051` | Remote monitoring |
| `grpcurl` | Direct call | `:50051` → ControlService → TaskManager | Development debugging |
| REST | Gateway `/api/v1/goto` and `/api/v1/instruction` | `GatewayModule` Out ports | Web dashboard / CLI |
| REPL | `go ...` / `agent ...` / `navigate x y` | `LingTuREPL` → `Gateway` ports | Manual operation |

The legacy `ros2 topic pub /nav/semantic/instruction` and
`ros2 launch navigation_explore.launch.py target:=…` paths no longer
exist — Module-First does not run a long-lived semantic ROS2 node.

## Voice path (Askme → LingTu)

```
"go to the kitchen"
   │
   ▼
Askme VoiceIntentPlanner
   regex match → target = "kitchen"
   emit VoicePlan(action_type="semantic_nav", mission_draft={target})
   │
   ▼
mission-orchestrator
   create Mission(type=SEMANTIC_NAV, params={target})
   PENDING → DISPATCHED
   │
   ▼
nav-gateway → LingTuGrpcBridge.start_semantic_nav(
       mission_id="m-xxx",
       semantic_target="kitchen",
       language="en",
       explore_if_unknown=True,
       timeout_sec=300,
   )
   │
   ▼
LingTu ControlService.StartTask
   task_type = TASK_TYPE_SEMANTIC_NAV (6)
   TaskManager publishes JSON
       { "instruction": "kitchen", "language": "en",
         "explore_if_unknown": true, "timeout_sec": 300,
         "arrival_radius": 1.0 }
   onto GatewayModule.instruction Out[str]
   │
   ▼
SemanticPlannerModule._on_instruction
   → GoalResolver.resolve()
       Fast Path: scene-graph match → goal_pose
       Slow Path: LLM reasoning over filtered scene graph
       Frontier: explore if no candidate found
   → goal_pose → NavigationModule
   → LocalPlanner → PathFollower → CmdVelMux → ThunderDriver
```

| Component | File |
|-----------|------|
| `VoiceIntentPlanner` | `products/nova-dog/runtime/services/askme-edge-service/.../planner.py` |
| `mission-orchestrator` | `products/nova-dog/runtime/services/mission-orchestrator/` |
| `LingTuGrpcBridge` | `products/nova-dog/runtime/services/nav-gateway/.../lingtu_grpc_bridge.py` |
| `ControlService` (C++) | `src/remote_monitoring/src/runtime/control_service.cpp` |
| `TaskManager` (C++) | `src/remote_monitoring/src/runtime/task_manager.cpp` |
| `SemanticPlannerModule` | `src/decision/semantic_planner/semantic_planner_module.py` |
| `GoalResolver` | `src/decision/semantic_planner/goal_resolver.py` |

## gRPC direct (development)

Skipping NOVA Dog Runtime and calling LingTu's `:50051` directly:

```bash
# Semantic navigation with optional exploration
grpcurl -plaintext -d '{
  "task_type": 6,
  "semantic_nav_params": {
    "instruction": "find the dining table",
    "language": "en",
    "explore_if_unknown": true,
    "timeout_sec": 300,
    "arrival_radius": 1.0
  }
}' localhost:50051 robot.v1.ControlService/StartTask

# Pure coverage exploration (no specific target)
grpcurl -plaintext -d '{
  "task_type": 6,
  "semantic_nav_params": {
    "instruction": "explore the whole building",
    "language": "en",
    "explore_if_unknown": true,
    "timeout_sec": 600
  }
}' localhost:50051 robot.v1.ControlService/StartTask

# Query task state
grpcurl -plaintext localhost:50051 robot.v1.ControlService/GetTaskStatus
```

Prerequisites: the LingTu daemon must be running with the gateway and
semantic stacks enabled. On the S100P that's `lingtu.py nav --no-repl`
(or any profile that keeps `enable_semantic=True`).

## Status reporting

```
SemanticPlannerModule
   → mission_status Out[dict] (NavigationModule)
   → agent_message Out[dict]   (Gateway → SSE chat)
NavigationModule
   → mission_status Out[dict]  (Gateway / MCP)
   → global_path Out[Path]     (Gateway)
   ↓
ControlService.GetTaskStatus aggregates →
   TaskStatus proto
   ↓
LingTuGrpcBridge._poll_loop polls every 3 s →
   notifies orchestrator
   ↓
Askme / Flutter App → user message / UI update
```

Mission states (from `NavigationModule._STATES`):

| State | Meaning |
|-------|---------|
| `IDLE` | No mission |
| `PLANNING` | A* / PCT search running |
| `EXECUTING` | Following waypoints |
| `SUCCESS` | Goal reached |
| `STUCK` | Stuck timeout, replan attempted |
| `FAILED` | Replan budget exhausted |
| `CANCELLED` | User cancelled |

## Design rules

1. **Single task owner.** `mission-orchestrator` is the only entity that
   manages mission lifecycle. Askme, Flutter, and CLI submit tasks; they
   do not own them.
2. **Single task per LingTu.** LingTu's TaskManager runs one task at a
   time; new tasks cancel the previous one. Queueing and parallelism are
   handled in the orchestrator.
3. **Protocol bridging.** `LingTuGrpcBridge` translates the orchestrator
   mission protocol into the `robot.v1` gRPC contract. It supports both a
   compiled-stub mode (`robot_proto.python.robot.v1`) and a raw-protobuf
   fallback when stubs are not available.
4. **Safety boundaries.**

   | Layer | Mechanism |
   |-------|-----------|
   | mission-orchestrator | Conflict detection, priority |
   | nav-gateway | Lease validation, mode checks |
   | LingTu ControlService | LeaseManager, SafetyGate |
   | SemanticPlannerModule | Timeout, LERa failure recovery |
   | NavigationModule | Stuck recovery, replan budget |
   | LocalPlanner / PathFollower | Obstacle avoidance, brake on terrain hit |
   | ThunderDriver | 200 ms `cmd_vel` watchdog |
   | brainstem arbiter | RC priority over gRPC |

## Internal semantic flow

```
GatewayModule.instruction Out[str]   (JSON or plain text)
   ↓
SemanticPlannerModule._on_instruction
   ↓
GoalResolver.resolve(instruction)
   ├─ Fast Path (~0.17 ms)
   │     scene-graph keyword match + spatial reasoning
   │     fusion: label 35% + CLIP 35% + detector 15% + spatial 15%
   │     threshold 0.75 → return PoseStamped directly
   │
   ├─ AdaNav entropy trigger
   │     score_entropy > 1.5 and confidence < 0.85 → escalate to Slow Path
   │
   ├─ Slow Path (~2 s)
   │     ESCA selective grounding (~200 → ~15 objects, ~92.5% token cut)
   │     LLM call (kimi / openai / claude / qwen, mock fallback)
   │     OmniNav hierarchical room hint
   │
   └─ explore_if_unknown=True
         FrontierScorer / TopologySemGraph search
         frontier weights: distance 0.25, novelty 0.35,
                            language 0.20, grounding 0.20
         emit best frontier as PoseStamped
         max_explore_steps: 20 in nav mode, 50 in explore mode
```

## Task types accepted by the bridge

| Task type | Enum | Bridge method | Description |
|-----------|------|---------------|-------------|
| Navigation | `TASK_TYPE_NAVIGATE` (1) | `start_navigation()` | Point-to-point goal |
| Mapping | `TASK_TYPE_MAPPING` (2) | `start_mapping()` | Manual mapping |
| Path follow | `TASK_TYPE_FOLLOW_PATH` (3) | `start_follow_path()` | Predefined polyline |
| Semantic nav | `TASK_TYPE_SEMANTIC_NAV` (6) | `start_semantic_nav()` | Semantic target + exploration |
| Person follow | `TASK_TYPE_FOLLOW_PERSON` (7) | `start_follow_person()` | VisualServo person tracker |

## Related

- [`ARCHITECTURE.md`](./ARCHITECTURE.md) — hardware boards and safety layers
- [`SYSTEM_OVERVIEW.md`](./SYSTEM_OVERVIEW.md) — Module layout and stacks
- [`TOPIC_CONTRACT.md`](./TOPIC_CONTRACT.md) — ROS2 topic contract used inside the C++ subsystems
- `config/semantic_planner.yaml` — semantic planner thresholds
- `config/semantic_exploration.yaml` — exploration parameters
