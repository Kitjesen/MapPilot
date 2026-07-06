# Decision

`src/decision/` owns semantic decision-making: instruction interpretation,
goal resolution, LLM/tool orchestration, visual servo decisions, task
decomposition, action recovery, and frontier scoring.

Semantic planning is pure Python Module orchestration. It does not own robot
drivers, velocity arbitration, global/local path planning, SLAM, or ROS 2
runtime bridges. The local autonomy uses in-process C++/Python kernels in the
navigation layer; decision code produces semantic goals, servo targets,
cancels, and task status.

## Functional Layout

| Path | Role |
| --- | --- |
| `modules/` | Runtime `Module` entrypoints used by blueprints and plugin registration. |
| `goal_resolution/` | Fast/slow target grounding, AdaCoT routing, SG-Nav reasoning, tokenizer helpers. |
| `llm/` | LLM clients, backend registry, and prompt builders. |
| `tasking/` | Agent loop, task decomposition, action execution, planner state, and service wrappers. |
| `exploration/` | Frontier types, scoring, semantic uncertainty, and exploration strategy helpers. |
| `vision/` | BBox navigation, person tracking, OSNet Re-ID, VLM bbox and scene queries. |
| `resource/` | Static decision resources. |
| `tests/` | Decision-layer tests. |

## Runtime Modules

| Module | File | Runtime role |
| --- | --- | --- |
| `SemanticPlannerModule` | `modules/semantic_planner_module.py` | Converts user/agent instructions into goals, plans, cancels, and servo targets. |
| `VisualServoModule` | `modules/visual_servo_module.py` | Converts visual targets into far `goal_pose` or near `cmd_vel`. |
| `LLMModule` | `modules/llm_module.py` | Multi-backend LLM wrapper exposed as a Module. |
| `GoalResolverModule` | `modules/goal_resolver_module.py` | Thin Module around fast/slow goal resolution. |
| `TaskDecomposerModule` | `modules/task_decomposer_module.py` | Turns natural-language tasks into ordered sub-goals. |
| `ActionExecutorModule` | `modules/action_executor_module.py` | Executes decomposed subtasks and recovery decisions. |
| `FrontierModule` | `modules/frontier_module.py` | Module wrapper around frontier-scoring exploration helpers. |

## Algorithm Use

Decision uses local Python strategies and model clients. It does not call the
native navigation kernels directly; navigation, local planning, path following,
and velocity muxing remain in `src/nav/` and `src/nav/kernel/`.

| Function | Decision entry | Strategy/algorithm used | Lower layer handoff |
| --- | --- | --- | --- |
| Natural-language goal resolution | `modules/semantic_planner_module.py`, `modules/goal_resolver_module.py` | `goal_resolution/goal_resolver.py` with `FastPathMixin`, `SlowPathMixin`, `AdaCoTRouter`, tokenizer, optional SG-Nav reasoning | Publishes `goal_pose`; `Navigation` and planner backends handle path planning. |
| Unknown-target exploration choice | `modules/semantic_planner_module.py`, `modules/frontier_module.py` | `exploration/frontier_scorer.py`, frontier BFS, information gain, uncertainty, KG/semantic prior, simple TSP ordering | Publishes an exploration `goal_pose`; map/frontier data comes from map/navigation layers. |
| Multi-step task execution | `modules/semantic_planner_module.py`, `modules/action_executor_module.py` | `tasking/task_decomposer.py`, `tasking/task_rules.py`, `tasking/action_executor.py`, LERa recovery rules | Converts high-level actions to `goal_pose`, `cmd_vel`, or `cancel`; actual motion arbitration stays outside decision. |
| Tool-calling agent loop | `modules/semantic_planner_module.py` | `tasking/agent_loop.py` plus discovered `@skill` tools and optional VLM scene agent | Calls Module skills; does not own hardware or planner execution. |
| Visual servo / follow | `modules/visual_servo_module.py` | `vision/bbox_navigator.py` bbox depth projection, PD control, Ziegler-Nichols gain tuning; `vision/person_tracker.py` IoU/CLIP/OSNet/VLM Re-ID | Far target publishes `goal_pose`; close target publishes short-range `cmd_vel` through the mux path. |
| LLM reasoning | `modules/llm_module.py`, `llm/llm_client.py` | Kimi/OpenAI/Claude/Qwen/mock clients and prompt builders | Supplies text/VLM decisions only; no navigation kernel access. |

## Imports

```python
from decision.goal_resolution.goal_resolver import GoalResolver
from decision.modules.visual_servo_module import VisualServoModule
```

## Boundaries

- Decision code may publish `goal_pose`, `cancel`, `servo_target`, and
  short-range `cmd_vel` through Module ports.
- Decision code does not directly drive hardware or own the `VelocityMux`.
- Decision code does not own global planning, local obstacle avoidance, path
  following, map maintenance, SLAM, or localization health.
- MCP skills are discovered from runtime Modules through
  `runtime.module.skill`; normal strategy helpers should stay plain Python.
