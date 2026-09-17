"""AgentPlannerModule - multi-turn agent loop for complex instructions.

Extracted from SemanticPlannerModule to separate single-shot instruction
resolution from multi-turn agent conversations.

The agent loop uses an LLM to decompose complex tasks into tool calls
(navigate_to, detect_object, query_memory, etc.), executing each step
and feeding results back until the task is complete or max steps reached.

Ports:
  In:  agent_instruction, scene_graph, robot_pose, mission_status
  Out: agent_message, planner_status

Usage::

    bp.add(AgentPlannerModule, alias="AgentPlannerModule", llm_backend="kimi")
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import json as _json
import logging
import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from functools import partial
from typing import Any

from decision.backends import BackendManager
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import NavigationGoalStatus, NavigationState
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.stream import In, Out
from runtime.tf.frames import map_frame_id

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
    "start_inspection",
    "navigate_to_deg", "find_object", "follow_person", "stop_servo",
    "run_agent_task", "cancel_agent_task",
    "vla_navigate", "go_to_tag",  # these adapters do not carry Agent task ownership
}


@dataclass
class AgentRun:
    instruction: str
    run_id: str = field(default_factory=lambda: f"agent-{uuid.uuid4().hex}")
    cancelled: bool = False
    future: concurrent.futures.Future | None = None
    planner: Any = None
    planner_revision: int | None = None
    motion_attempted: bool = False
    motion_submitted: bool = False
    motion_success: bool = False
    state: str = "running"


@register("agent_planner", "default", description="Multi-turn agent loop module")
class AgentPlannerModule(Module, layer=4):
    """Autonomous multi-turn agent loop for complex task decomposition and execution.

    Handles multi-turn conversations via the AgentLoop, producing navigation
    goals, servo targets, and chat messages.

    The module subscribes to scene graph and map-frame robot pose for context, and
    dispatches tool calls (navigate_to, detect_object, query_memory, etc.)
    through the AgentLoop's LLM-driven observe-think-act cycle.
    """

    SOFT_DEPENDS = ["VectorMemoryModule", "SemanticMapperModule", "LLMModule", "SemanticPlannerModule"]

    # -- Inputs --
    agent_instruction: In[str]  # multi-turn agent loop (observe->think->act cycle)
    scene_graph: In[SceneGraph]
    robot_pose: In[PoseStamped]
    observation_image: In[Image]
    navigation_state: In[NavigationState]
    navigation_goal_status: In[NavigationGoalStatus]
    mission_status: In[dict]  # for nav_status context

    # -- Outputs --
    agent_message: Out[dict]  # chat-facing messages
    planner_status: Out[str]  # AGENT_RUNNING / AGENT_DONE / AGENT_FAILED

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

        # Map-frame position from the same perception pipeline as scene objects.
        self._robot_pos = [0.0, 0.0, 0.0]
        self._current_robot_pose: PoseStamped | None = None
        self._navigation_context: tuple[str, str, int] | None = None
        self._navigation_sequence = 0

        # Scene graph - keep both JSON string and the original object.
        self._latest_sg: str | None = None
        self._current_scene_graph: SceneGraph | None = None

        # Nav status (cached from mission_status for agent context)
        self._last_nav_state: str = ""
        self._navigation_goal_status_by_request: dict[str, dict[str, Any]] = {}

        # Latest camera frame for VLM tools in the agent loop
        self._observation_image: Image | None = None

        # Backend manager + agent tool discovery
        self._backends: BackendManager | None = None
        self._agent_tool_registry: dict[str, Any] = {}
        self._agent_tool_list: list[dict[str, Any]] = []

        # Stats
        self._agent_count: int = 0
        self._run_lock = threading.RLock()
        self._active_run: AgentRun | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None

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
        self.robot_pose.subscribe(self._on_robot_pose)
        self.observation_image.subscribe(self._on_observation_image)
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

    @staticmethod
    def _valid_map_pose(pose: PoseStamped | None) -> bool:
        return (
            pose is not None and pose.frame_id == AGENT_PLANNER_MAP_FRAME_ID
            and all(math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.ts))
            and -0.2 <= time.time() - pose.ts <= 0.75
        )

    def _on_robot_pose(self, pose: PoseStamped) -> None:
        self._current_robot_pose = pose if self._valid_map_pose(pose) else None
        if self._current_robot_pose is not None:
            self._robot_pos = np.array([pose.x, pose.y, pose.z])

    def _on_observation_image(self, image: Image) -> None:
        self._observation_image = image

    def _fresh_scene(self) -> dict | None:
        scene = self._current_scene_graph
        if (scene is None or scene.frame_id != AGENT_PLANNER_MAP_FRAME_ID
                or not -0.2 <= time.time() - scene.ts <= 0.75):
            return None
        result = scene.to_dict()
        result["objects"] = [obj for obj in result["objects"] if -0.2 <= time.time() - obj["ts"] <= 0.75]
        ids = {obj["id"] for obj in result["objects"]}
        result["relations"] = [rel for rel in result["relations"]
                               if rel["subject_id"] in ids and rel["object_id"] in ids]
        for region in result["regions"]:
            region["object_ids"] = [oid for oid in region["object_ids"] if oid in ids]
        return result

    def _on_mission_status(self, status: dict) -> None:
        """Use legacy status only until native state is available."""
        if self._navigation_context is None:
            self._last_nav_state = status.get("state", "")

    def _on_navigation_state(self, state: NavigationState) -> None:
        context = (state.boot_id, state.map_id, int(state.map_content_epoch))
        if (self._navigation_context is not None and self._navigation_context[0] == state.boot_id
                and state.sequence <= self._navigation_sequence):
            return
        if self._navigation_context is not None and self._navigation_context != context:
            if self._cancel_active_run("navigation_context_changed"):
                self.planner_status.publish("AGENT_CANCELLED")
            self._current_robot_pose = None
            self._current_scene_graph = None
            self._latest_sg = None
            self._observation_image = None
        self._navigation_context = context
        self._navigation_sequence = int(state.sequence)
        self._last_nav_state = state.to_dict()["lifecycle_state_name"]

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
        if not instruction.strip() or self._closed:
            return
        with self._run_lock:
            run = self._new_run(instruction)
            if self._loop is None:
                self._loop = asyncio.new_event_loop()
                self._loop_thread = threading.Thread(target=self._loop.run_forever, name="agent_loop", daemon=True)
                self._loop_thread.start()
            self.planner_status.publish("AGENT_RUNNING")
            self._chat("thinking", f"Agent loop started: {instruction[:60]}", phase="agent_start")
            run.future = asyncio.run_coroutine_threadsafe(self._execute_run(run), self._loop)

    def _new_run(self, instruction: str) -> AgentRun:
        with self._run_lock:
            self._cancel_active_run("agent_instruction_replaced")
            self._active_run = AgentRun(instruction)
            planner = self._backends.get("SemanticPlannerModule") if self._backends else None
            if planner is not None:
                self._active_run.planner = planner
                self._active_run.planner_revision = planner.instruction_revision()
            self._agent_count += 1
            return self._active_run

    def _run_is_current(self, run: AgentRun) -> bool:
        return self._active_run is run and not run.cancelled and not self._closed

    @staticmethod
    def _motion_was_superseded(run: AgentRun) -> bool:
        if run.planner is None:
            return False
        if not run.motion_submitted:
            return run.planner.instruction_revision() != run.planner_revision
        return run.planner.owned_instruction_status(run.run_id).get("state") == "SUPERSEDED"

    def _cancel_active_run(self, reason: str) -> bool:
        with self._run_lock:
            run = self._active_run
            if run is None or run.cancelled:
                return False
            run.cancelled, run.state = True, "cancelled"
            if run.future is not None:
                run.future.cancel()
            if run.planner is not None:
                run.planner.cancel_owned_instruction(run.run_id, reason)
            return True

    async def _execute_run(self, run: AgentRun):
        try:
            state = await self._run_agent_loop(run.instruction, run=run)
            with self._run_lock:
                if not self._run_is_current(run):
                    return state
                if self._motion_was_superseded(run):
                    raise asyncio.CancelledError
                success = state is not None and state.completed and not state.failure_reason
                success = success and (not run.motion_attempted or run.motion_success)
                run.state = "done" if success else "failed"
                if not success and run.planner is not None:
                    run.planner.cancel_owned_instruction(run.run_id, "agent_failed")
                self.planner_status.publish("AGENT_DONE" if success else "AGENT_FAILED")
                self._chat("assistant", state.summary if state else "LLM unavailable",
                           phase="agent_done" if success else "agent_error")
                return state
        except asyncio.CancelledError:
            if run.planner is not None:
                run.planner.cancel_owned_instruction(run.run_id, "agent_cancelled")
            if self._run_is_current(run):
                run.cancelled, run.state = True, "cancelled"
                self.planner_status.publish("AGENT_CANCELLED")
        except Exception:
            logger.exception("Agent loop failed")
            if run.planner is not None:
                run.planner.cancel_owned_instruction(run.run_id, "agent_error")
            if self._run_is_current(run):
                run.state = "failed"
                self.planner_status.publish("AGENT_FAILED")

    def _run_agent_loop_sync(self, instruction: str) -> None:
        """Use the same managed loop for synchronous local callers."""
        self._on_agent_instruction(instruction)
        run = self._active_run
        if run is not None and run.future is not None:
            run.future.result(timeout=max(1.0, self._timeout + 5.0))

    async def _run_agent_loop(self, instruction: str, *, run: AgentRun | None = None):
        """Build AgentLoop with tool bindings and run."""
        from decision.tasks.agent import AgentLoop

        run = run or self._new_run(instruction)
        if not self._run_is_current(run) or self._llm_client is None:
            return

        # Tool handlers bound to this module's capabilities
        handlers = {
            "navigate_to": partial(self._tool_navigate_to, run=run),
            "navigate_to_object": partial(self._tool_navigate_to_object, run=run),
            "navigate_to_deg": partial(self._tool_navigate_to_deg, run=run),
            "find_object": partial(self._tool_find_object, run=run),
            "follow_person": partial(self._tool_follow_person, run=run),
            "stop_servo": partial(self._tool_stop_servo, run=run),
            "detect_object": self._tool_detect_object,
            "query_memory": self._tool_query_memory,
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
            cancelled=lambda: not self._run_is_current(run) or self._motion_was_superseded(run),
            completion_check=lambda: "The requested motion has not completed successfully."
            if run.motion_attempted and not run.motion_success else None,
        )
        return await agent.run(instruction)

    def stop(self) -> None:
        self._cancel_active_run("agent_stopped")
        loop = self._loop
        if loop is not None:
            async def shutdown():
                pending = [task for task in asyncio.all_tasks() if task is not asyncio.current_task()]
                for task in pending:
                    task.cancel()
                if pending:
                    await asyncio.gather(*pending, return_exceptions=True)
                close = getattr(self._llm_client, "close", None)
                if close is not None:
                    await close()
            try:
                asyncio.run_coroutine_threadsafe(shutdown(), loop).result(timeout=3.0)
            except Exception:
                logger.exception("Agent shutdown did not finish cleanly")
            loop.call_soon_threadsafe(loop.stop)
            if self._loop_thread is not None:
                self._loop_thread.join(timeout=3.0)
            if not loop.is_running():
                loop.close()
            self._loop = self._loop_thread = None
        super().stop()

    @skill
    def run_agent_task(self, instruction: str) -> str:
        """Replace the current Agent task and return its cancellation identity."""
        if not instruction.strip() or self._closed:
            return _json.dumps({"run_id": "", "state": "unavailable"})
        self._on_agent_instruction(instruction)
        return _json.dumps({"run_id": self._active_run.run_id if self._active_run else "",
                            "state": self._active_run.state if self._active_run else "unavailable"})

    @skill
    def cancel_agent_task(self, run_id: str) -> str:
        """Cancel only the specified Agent task; the result does not prove physical stopping."""
        with self._run_lock:
            if self._active_run is None or self._active_run.run_id != run_id:
                return _json.dumps({"cancel_requested": False, "reason": "not_current"})
            self._cancel_active_run("agent_user_cancel")
            self.planner_status.publish("AGENT_CANCELLED")
            return _json.dumps({"cancel_requested": True, "stop_confirmed": False})

    @skill
    def get_agent_status(self) -> str:
        """Report loop state separately from verified motion and its native task."""
        with self._run_lock:
            run = self._active_run
            if run is None:
                return _json.dumps({"state": "idle"})
            return _json.dumps({
                "run_id": run.run_id, "state": run.state, "motion_attempted": run.motion_attempted,
                "motion_success": run.motion_success,
                "navigation": run.planner.owned_instruction_status(run.run_id) if run.planner else None,
            })

    def _agent_context(self) -> dict:
        """Build context dict for the agent loop."""
        visible = ""
        scene_graph = self._fresh_scene()
        if scene_graph is not None:
            labels = [obj["label"] for obj in scene_graph["objects"] if obj["label"]]
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
        camera_image = None
        image, pose = self._observation_image, self._current_robot_pose
        if (image is not None and scene_graph is not None and self._valid_map_pose(pose)
                and image.ts == self._current_scene_graph.ts == pose.ts):
            try:
                camera_image = image.to_bgr().data.copy()
            except (ValueError, TypeError):
                pass
        return {
            "robot_x": float(self._robot_pos[0]),
            "robot_y": float(self._robot_pos[1]),
            "visible_objects": visible or "none",
            "nav_status": self._last_nav_state or "IDLE",
            "memory_context": memory_context,
            "camera_image": camera_image,
            "camera_available": camera_image is not None,
            "scene_graph": scene_graph,
        }

    # ------------------------------------------------------------------
    # Agent tool handlers
    # ------------------------------------------------------------------

    async def _tool_navigate_to(self, x: float, y: float, yaw: float = 0.0, z: float | None = None,
                                *, run: AgentRun | None = None) -> str:
        """Navigate to map coordinates in the global frame."""
        run = run or self._active_run
        if run is None or not self._run_is_current(run):
            raise asyncio.CancelledError
        run.motion_attempted, run.motion_success = True, False
        if z is None:
            if not self._valid_map_pose(self._current_robot_pose):
                return "Navigation unavailable: a fresh map pose is required when z is omitted"
            z = self._current_robot_pose.z
        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            return "Navigation unavailable: coordinates must be finite"
        q_w = np.cos(yaw / 2.0)
        q_z = np.sin(yaw / 2.0)
        pose = PoseStamped(
            pose=Pose(
                position=Vector3(x=x, y=y, z=z),
                orientation=Quaternion(0.0, 0.0, float(q_z), float(q_w)),
            ),
            frame_id=AGENT_PLANNER_MAP_FRAME_ID,
        )
        return await self._submit_motion(run, pose=pose)

    async def _tool_navigate_to_object(self, label: str, *, run: AgentRun | None = None) -> str:
        """Use the same observation preview and visual verification as other semantic requests."""
        run = run or self._active_run
        if run is None or not self._run_is_current(run):
            raise asyncio.CancelledError
        run.motion_attempted, run.motion_success = True, False
        return await self._submit_motion(run, instruction=f"find {label}")

    async def _submit_motion(self, run: AgentRun, *, pose: PoseStamped | None = None,
                             instruction: str = "") -> str:
        with self._run_lock:
            if not self._run_is_current(run) or self._motion_was_superseded(run):
                raise asyncio.CancelledError
            planner = self._backends.get("SemanticPlannerModule") if self._backends else None
            if planner is None:
                return "Semantic navigation unavailable: SemanticPlannerModule is not loaded"
            if run.planner is not None and run.planner is not planner:
                raise asyncio.CancelledError
            run.planner = planner
            result = (planner.submit_owned_pose(pose, run.instruction, run.run_id,
                                                expected_revision=run.planner_revision) if pose is not None
                      else planner.submit_owned_instruction(instruction, run.run_id,
                                                            expected_revision=run.planner_revision))
            run.motion_submitted = result.get("owned") is True
        while True:
            if not self._run_is_current(run):
                raise asyncio.CancelledError
            if result.get("state") == "SUPERSEDED":
                raise asyncio.CancelledError
            if result.get("terminal") is True:
                run.motion_success = result.get("success") is True
                return _json.dumps(result)
            await asyncio.sleep(0.05)
            result = planner.owned_instruction_status(run.run_id)

    async def _tool_navigate_to_deg(self, x: float, y: float, yaw_deg: float = 0.0,
                                   z: float | None = None, *, run: AgentRun) -> str:
        return await self._tool_navigate_to(x, y, math.radians(yaw_deg), z, run=run)

    async def _tool_find_object(self, target: str, *, run: AgentRun) -> str:
        return await self._tool_navigate_to_object(target, run=run)

    async def _tool_follow_person(self, description: str, *, run: AgentRun) -> str:
        run.motion_attempted, run.motion_success = True, False
        return await self._submit_motion(run, instruction=f"follow {description}")

    def _tool_stop_servo(self, *, run: AgentRun) -> str:
        with self._run_lock:
            if not self._run_is_current(run):
                raise asyncio.CancelledError
            requested = run.planner is not None and run.planner.cancel_owned_instruction(run.run_id, "agent_stop_servo")
            return _json.dumps({"cancel_requested": requested, "stop_confirmed": False})

    def _tool_detect_object(self, label: str) -> str:
        """Check whether an object label is visible in the current scene graph."""
        scene = self._fresh_scene()
        if scene is None:
            return "No fresh scene graph"
        matches = [obj for obj in scene["objects"] if obj["label"] and label.lower() in obj["label"].lower()]
        if matches:
            positions = [f"({obj['position']['x']:.1f},{obj['position']['y']:.1f})" for obj in matches]
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
