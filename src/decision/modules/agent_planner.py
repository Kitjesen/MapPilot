"""AgentPlannerModule - multi-turn agent loop for complex instructions.

Extracted from SemanticPlannerModule to separate single-shot instruction
resolution from multi-turn agent conversations.

The agent loop uses an LLM to decompose complex tasks into tool calls
(navigate_to, detect_object, query_memory, etc.), executing each step
and feeding results back until the task is complete or max steps reached.

Ports:
  In:  agent_instruction, scene_graph, odometry, mission_status
  Out: goal_pose, servo_target, agent_message, planner_status, cancel

Usage::

    bp.add(AgentPlannerModule, alias="AgentPlannerModule", llm_backend="kimi")
"""

from __future__ import annotations

import json as _json
import logging
import threading
import time
from typing import Any

from decision.backends import BackendManager
from runtime.module import Module
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import NavigationGoalStatus, NavigationState, Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SceneGraph
from runtime.registry import register
from runtime.runtime_interface import map_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

AGENT_PLANNER_MAP_FRAME_ID = map_frame_id()

# Skills blocked from the agent loop (safety-critical, must not be LLM-invoked).
_AGENT_SKILL_BLOCKLIST = {
    "navigate_to",  # keep motion through the planner-owned handler
    "navigate_to_object",
    "query_memory",  # keep memory filtering through the planner-owned handler
    "query_location",  # VectorMemory skill may expose query-only coordinates
    "send_instruction",  # avoid recursive self-triggering from inside the agent loop
    "emergency_stop",  # keep safety-state transitions outside the LLM control path
    "stop",
    "set_mode",
    "stop_navigation",
    "cancel_mission",
    "start_patrol",
}


@register("agent_planner", "default", description="Multi-turn agent loop module")
class AgentPlannerModule(Module, layer=4):
    """Autonomous multi-turn agent loop for complex task decomposition and execution.

    Handles multi-turn conversations via the AgentLoop, producing navigation
    goals, servo targets, and chat messages.

    The module subscribes to scene graph and odometry for context, and
    dispatches tool calls (navigate_to, detect_object, query_memory, etc.)
    through the AgentLoop's LLM-driven observe-think-act cycle.
    """

    _run_in_worker = True
    _worker_group = "semantic"
    SOFT_DEPENDS = ["VectorMemoryModule", "SemanticMapperModule", "LLMModule"]

    # -- Inputs --
    agent_instruction: In[str]  # multi-turn agent loop (observe->think->act cycle)
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]
    navigation_state: In[NavigationState]
    navigation_goal_status: In[NavigationGoalStatus]
    mission_status: In[dict]  # for nav_status context

    # -- Outputs --
    goal_pose: Out[PoseStamped]
    servo_target: Out[str]  # "find:<label>" -> VisualServoModule
    agent_message: Out[dict]  # chat-facing messages
    planner_status: Out[str]  # AGENT_RUNNING / AGENT_DONE / AGENT_FAILED
    cancel: Out[str]

    def __init__(
        self,
        llm_backend: str = "kimi",
        llm_model: str = "",
        max_steps: int = 10,
        timeout: float = 120.0,
        approach_distance: float = 0.5,
        **kw,
    ):
        super().__init__(**kw)
        self._llm_backend = llm_backend
        self._llm_model = llm_model
        self._max_steps = max_steps
        self._timeout = timeout
        self._approach_dist = approach_distance

        # LLM client (lazy init in setup)
        self._llm_client = None
        self._llm_init_attempted = False

        # Odometry / position
        self._robot_pos = [0.0, 0.0, 0.0]

        # Scene graph - keep both JSON string and the original object.
        self._latest_sg: str | None = None
        self._current_scene_graph: SceneGraph | None = None

        # Nav status (cached from mission_status for agent context)
        self._last_nav_state: str = ""
        self._navigation_goal_status_by_request: dict[str, dict[str, Any]] = {}

        # Latest camera frame for VLM tools in the agent loop
        self._latest_rgb: np.ndarray | None = None

        # Backend manager + agent tool discovery
        self._backends: BackendManager | None = None
        self._agent_tool_registry: dict[str, Any] = {}
        self._agent_tool_list: list[dict[str, Any]] = []

        # Stats
        self._agent_count: int = 0

    # ------------------------------------------------------------------
    # System integration
    # ------------------------------------------------------------------

    def on_system_modules(self, modules: dict) -> None:
        self._backends = BackendManager(modules)
        self._refresh_agent_tools(modules)

    def _refresh_agent_tools(self, modules: dict[str, Any]) -> None:
        """Mirror MCP skill discovery for the internal agent loop."""
        self._agent_tool_registry = {}
        tool_list: list[dict[str, Any]] = []

        for mod_name, mod in modules.items():
            if not hasattr(mod, "get_skill_infos"):
                continue
            try:
                infos = mod.get_skill_infos()
            except Exception as exc:
                logger.debug("agent planner: skill discovery failed for %s: %s", mod_name, exc)
                continue

            for info in infos:
                if info.func_name in _AGENT_SKILL_BLOCKLIST:
                    continue
                method = getattr(mod, info.func_name, None)
                if method is None:
                    continue
                self._agent_tool_registry[info.func_name] = method
                schema = _json.loads(info.args_schema)
                desc = schema.pop("description", "")
                tool_list.append(
                    {
                        "name": info.func_name,
                        "description": f"[{info.class_name}] {desc}".strip(),
                        "inputSchema": schema,
                    }
                )

        # Last discovered tool wins, matching MCPServerModule's behavior.
        seen: dict[str, dict[str, Any]] = {}
        for tool in tool_list:
            seen[tool["name"]] = tool
        self._agent_tool_list = list(seen.values())
        logger.info(
            "Agent planner tools: %d discovered (%d blocked)",
            len(self._agent_tool_list),
            len(_AGENT_SKILL_BLOCKLIST),
        )

    def setup(self) -> None:
        self._init_llm()
        self.agent_instruction.subscribe(self._on_agent_instruction)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)
        self.navigation_state.subscribe(self._on_navigation_state)
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)
        self.mission_status.subscribe(self._on_mission_status)

    def _init_llm(self) -> None:
        """Create LLM client for the agent loop."""
        self._llm_init_attempted = True
        try:
            from decision.llm.client import LLMConfig, create_llm_client

            llm_cfg = LLMConfig(backend=self._llm_backend, model=self._llm_model)
            self._llm_client = create_llm_client(llm_cfg)
            logger.info(
                "AgentPlanner LLM client initialized (backend=%s)",
                self._llm_backend,
            )
        except Exception as e:
            logger.warning("AgentPlanner LLM client not available: %s", e)
            self._llm_client = None

    # ------------------------------------------------------------------
    # Input handlers
    # ------------------------------------------------------------------

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        """Cache scene graph for agent context."""
        sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
        self._latest_sg = sg_json
        self._current_scene_graph = sg

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_pos = np.array([odom.x, odom.y, getattr(odom, "z", 0.0)])

    def _on_mission_status(self, status: dict) -> None:
        """Cache nav status for agent context (non-terminal states only)."""
        state = status.get("state", "")
        if state not in ("RECOVERING", "STUCK", "FAILED"):
            self._last_nav_state = state

    def _on_navigation_state(self, state: NavigationState) -> None:
        state_name = state.to_dict()["lifecycle_state_name"]
        if state_name not in ("RECOVERING", "FAILED"):
            self._last_nav_state = state_name

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        self._navigation_goal_status_by_request[status.request_id] = status.to_dict()
        if len(self._navigation_goal_status_by_request) > 256:
            oldest = next(iter(self._navigation_goal_status_by_request))
            self._navigation_goal_status_by_request.pop(oldest, None)

    # ------------------------------------------------------------------
    # Chat helper
    # ------------------------------------------------------------------

    def _chat(self, role: str, text: str, phase: str | None = None) -> None:
        """Publish a chat message for the Web ChatPanel.

        role: 'thinking' | 'assistant' | 'tool'
        phase: optional sub-state hint (e.g. 'agent_start', 'agent_done')
        """
        try:
            self.agent_message.publish(
                {
                    "role": role,
                    "text": text,
                    "ts": time.time(),
                    "phase": phase or "",
                }
            )
        except Exception:
            pass  # never let chat failures affect planning

    # ------------------------------------------------------------------
    # Multi-turn Agent Loop
    # ------------------------------------------------------------------

    def _on_agent_instruction(self, instruction: str) -> None:
        """Handle multi-turn agent instruction (observe->think->act cycle)."""
        if not instruction.strip():
            return
        self._agent_count += 1
        self.planner_status.publish("AGENT_RUNNING")
        self._chat("thinking", f"Agent loop started: {instruction[:60]}", phase="agent_start")
        threading.Thread(
            target=self._run_agent_loop_sync,
            args=(instruction,),
            name="agent_loop",
            daemon=True,
        ).start()

    def _run_agent_loop_sync(self, instruction: str) -> None:
        """Run agent loop in a background thread (wraps async)."""
        import asyncio

        loop = asyncio.new_event_loop()
        try:
            state = loop.run_until_complete(self._run_agent_loop(instruction))
            if state.completed:
                self.planner_status.publish("AGENT_DONE")
                self._chat("assistant", f"Agent done: {state.summary}", phase="agent_done")
                logger.info("Agent loop done: %s", state.summary)
            else:
                self.planner_status.publish("AGENT_FAILED")
        except Exception:
            logger.exception("Agent loop failed")
            self.planner_status.publish("AGENT_FAILED")
        finally:
            loop.close()

    async def _run_agent_loop(self, instruction: str):
        """Build AgentLoop with tool bindings and run."""
        from decision.tasks.agent import AgentLoop

        if self._llm_client is None:
            self.planner_status.publish("AGENT_FAILED")
            self._chat("assistant", "LLM not available for agent loop", phase="agent_error")
            return

        # Tool handlers bound to this module's capabilities
        handlers = {
            "navigate_to": self._tool_navigate_to,
            "navigate_to_object": self._tool_navigate_to_object,
            "detect_object": self._tool_detect_object,
            "query_memory": self._tool_query_memory,
            "tag_location": self._tool_tag_location,
            "say": self._tool_say,
        }

        agent = AgentLoop(
            llm_client=self._llm_client,
            tool_registry=self._agent_tool_registry,
            tool_list=self._agent_tool_list,
            tool_handlers=handlers,
            context_fn=self._agent_context,
            max_steps=self._max_steps,
            timeout=self._timeout,
        )
        return await agent.run(instruction)

    def _agent_context(self) -> dict:
        """Build context dict for the agent loop."""
        visible = ""
        if self._current_scene_graph:
            labels = [o.label for o in self._current_scene_graph.objects if o.label]
            visible = ", ".join(labels[:20])
        memory_context = "none"
        if self._backends is not None and self._backends.vector_memory is not None:
            try:
                stats = self._backends.vector_memory.get_memory_stats()
                if isinstance(stats, str):
                    stats = _json.loads(stats)
                memory_context = f"vector_memory_entries={stats.get('entries', 0)}"
            except Exception:
                memory_context = "vector_memory_available"
        scene_graph = None
        if self._latest_sg:
            try:
                scene_graph = _json.loads(self._latest_sg)
            except Exception:
                scene_graph = {"raw": self._latest_sg}
        return {
            "robot_x": float(self._robot_pos[0]),
            "robot_y": float(self._robot_pos[1]),
            "visible_objects": visible or "none",
            "nav_status": self._last_nav_state or "IDLE",
            "memory_context": memory_context,
            "camera_image": self._latest_rgb,
            "camera_available": self._latest_rgb is not None,
            "scene_graph": scene_graph,
        }

    # ------------------------------------------------------------------
    # Agent tool handlers
    # ------------------------------------------------------------------

    def _tool_navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
        """Navigate to map coordinates in the global frame."""
        q_w = np.cos(yaw / 2.0)
        q_z = np.sin(yaw / 2.0)
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(x=x, y=y, z=0.0),
                orientation=Quaternion(0.0, 0.0, float(q_z), float(q_w)),
            ),
            frame_id=AGENT_PLANNER_MAP_FRAME_ID,
        )
        self.goal_pose.publish(pose)
        return f"Navigating to ({x:.1f}, {y:.1f}, yaw={yaw:.2f})"

    def _tool_navigate_to_object(self, label: str) -> str:
        """Resolve an object label from the scene graph and navigate to it.

        If the object is visible, publishes a goal_pose at its position.
        If not visible, triggers visual servo to find it.
        """
        if not self._current_scene_graph:
            return "No scene graph available"
        matches = [o for o in self._current_scene_graph.objects if o.label and label.lower() in o.label.lower()]
        if not matches:
            # Not visible - trigger visual servo to find it.
            self.servo_target.publish(f"find:{label}")
            return f"Target '{label}' not visible; triggering visual servo"

        obj = matches[0]
        pos = getattr(obj, "position", None)
        if pos is None:
            return f"Found '{label}' but no position data"

        pose = PoseStamped(
            pose=Pose(
                position=Vector3(
                    x=float(pos.x),
                    y=float(pos.y),
                    z=0.0,
                ),
                orientation=Quaternion(0, 0, 0, 1),
            ),
            frame_id=AGENT_PLANNER_MAP_FRAME_ID,
            ts=time.time(),
        )
        self.goal_pose.publish(pose)
        return f"Navigating to {label} at ({pos.x:.1f}, {pos.y:.1f})"

    def _tool_detect_object(self, label: str) -> str:
        """Check whether an object label is visible in the current scene graph."""
        if not self._current_scene_graph:
            return "No scene graph"
        matches = [o for o in self._current_scene_graph.objects if o.label and label.lower() in o.label.lower()]
        if matches:
            positions = [f"({o.position.x:.1f},{o.position.y:.1f})" for o in matches if o.position]
            return f"Found {len(matches)}: {', '.join(positions)}"
        return f"'{label}' not visible"

    def _tool_query_memory(self, text: str) -> str:
        """Query spatial memory for a previously seen location."""
        if self._backends is None or self._backends.vector_memory is None:
            return "Vector memory not available"
        raw = self._backends.vector_memory.query_location(text)
        result = _json.loads(raw) if isinstance(raw, str) else raw
        if not result.get("found"):
            return f"No memory match for '{text}'"
        best = result["best"]
        coordinates_are_safe = (
            result.get("navigable") is True
            and result.get("semantic_encoder_ready") is True
            and result.get("degraded") is False
        )
        if not coordinates_are_safe:
            labels = best.get("labels", "")
            return (
                f"Found a query-only memory match for '{text}'"
                f" (score={best['score']:.2f}, labels={labels}); "
                "coordinates are withheld because semantic memory is degraded"
            )
        return (
            f"Found: ({best['x']:.1f}, {best['y']:.1f}, "
            f"{best.get('z', 0.0):.1f}) score={best['score']:.2f} "
            f"labels={best.get('labels', '')}"
        )

    def _tool_tag_location(self, name: str) -> str:
        """Tag the robot's current location with a name."""
        x, y = float(self._robot_pos[0]), float(self._robot_pos[1])
        if self._backends is not None and self._backends.tagged_locations is not None:
            try:
                self._backends.tagged_locations.tag_command._deliver(f"tag:{name}")
            except Exception:
                pass
        return f"Tagged '{name}' at ({x:.1f}, {y:.1f})"

    def _tool_say(self, text: str) -> str:
        """Emit a short message for the operator."""
        logger.info("Agent says: %s", text)
        self._chat("assistant", text, phase="agent_say")
        return f"Said: {text}"

    # ------------------------------------------------------------------
    # Health
    # ------------------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        info["agent_planner"] = {
            "llm_backend": self._llm_backend,
            "llm_ready": self._llm_client is not None,
            "llm_init_attempted": self._llm_init_attempted,
            "agent_runs": self._agent_count,
            "tools": len(self._agent_tool_list),
            "backends": self._backends.summary() if self._backends else {},
        }
        return info
