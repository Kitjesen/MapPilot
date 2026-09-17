"""SemanticPlannerModule - unified semantic planning in one Module.

Replaces 4 separate modules (GoalResolver, Frontier, TaskDecomposer, ActionExecutor).
Internal strategies handle different algorithms.

The multi-turn agent loop has been extracted to AgentPlannerModule.
This module focuses on single-shot instruction processing.

Pipeline:
  instruction -> decompose -> resolve goal -> explore frontiers -> execute action
  own native NavigationGoalStatus (FAILED) -> LERa recovery -> new goal

Ports:
  In:  instruction, scene_graph, robot_pose,
       detections, navigation_state, goal_status, navigation_goal_status,
       topo_summary, room_graph
  Out: nav_command, task_plan, planner_status, servo_target, agent_message

Strategies:
  decomposer: "rules" | "llm"
  resolver:   "fast_slow" (default, Fast Path + Slow Path)
  explorer:   "frontier" (default, frontier scoring)
  executor:   "lera" (default, LERa recovery)

Usage::

    bp.add(SemanticPlannerModule, decomposer="rules")
    bp.add(LLMModule, backend="kimi")  # separate, wired via Blueprint
"""

from __future__ import annotations

import json as _json
import logging
import math
import re
import threading
import time
import uuid
from typing import Any

from decision.backends import BackendManager
from decision.modules.llm import LLMRequest, LLMResponse
from decision.semantic_navigation.execution import GoalExecution
from decision.semantic_navigation.intent import HybridSemanticIntentParser, SemanticAction, SemanticIntent, TravelMode
from decision.semantic_navigation.intent import normalize_floor_id as normalize_semantic_floor_id
from decision.semantic_navigation.observation import ObjectTarget, ObservationRequest, observation_candidates
from decision.semantic_navigation.verification import (
    TargetVerification,
    VerificationSample,
    image_bbox,
    parse_verdict,
    verification_messages,
)
from memory.spatial.places import PlaceCatalog, PlaceCatalogError, PlaceRef
from runtime.endpoints.mapd import MapClient
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import NavigationGoalState, NavigationGoalStatus, NavigationState
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SceneGraph
from runtime.msgs.sensor import Image
from runtime.registry import register
from runtime.stream import In, Out
from runtime.tf.frames import map_frame_id

logger = logging.getLogger(__name__)

# Minimum seconds between consecutive LERa triggers.
_LERA_COOLDOWN = 15.0
SEMANTIC_PLANNER_MAP_FRAME_ID = map_frame_id()


class _MapdPlaceQueries:
    """Expose the read-only mapd calls consumed by ``PlaceCatalog``."""

    def __init__(self, transport: object) -> None:
        self._transport = transport

    def list_maps(self) -> dict[str, Any]:
        return self._service("list_maps")

    def get_record(self, map_id: str) -> dict[str, Any]:
        return self._service("get_record", map_id=map_id)

    def poi_list(self, map_id: str = "") -> dict[str, Any]:
        return self._service("list_poi", map_id=map_id)

    def _service(self, action: str, **arguments: Any) -> dict[str, Any]:
        service = getattr(self._transport, "service", None)
        if not callable(service):
            raise TypeError("map query transport does not implement service")
        response = service(action, **arguments)
        if not isinstance(response, dict):
            raise TypeError(f"mapd {action} response must be an object")
        return response


@register("semantic_planner", "default", description="Unified semantic planner module")
class SemanticPlannerModule(Module, layer=4):
    """Unified semantic planner: decompose ->resolve ->explore ->execute.

    Internally composes GoalResolver, FrontierScorer, TaskDecomposer,
    ActionExecutor. Each is a strategy, not a separate Module.

    LERa integration
    ----------------
    Subscribes to correlated native NavigationGoalStatus. On own-task FAILED, calls
    ActionExecutor.lera_recover() and dispatches one of four strategies:
      retry_different_path -resolve a fresh goal (Navigation replans)
      expand_search        -ask FrontierScorer for an alternative frontier
      requery_goal         -re-run Fast->Slow goal resolution from scratch
      abort                -cancel owned tasks through nav_command
    """

    SOFT_DEPENDS = ["VectorMemoryModule", "SemanticMapperModule", "LLMModule"]

    # -- Inputs --
    instruction: In[str]  # single-shot resolve (Fast->Frontier->VisualServo)
    scene_graph: In[SceneGraph]
    robot_pose: In[PoseStamped]
    observation_image: In[Image]
    detections: In[list]
    navigation_state: In[NavigationState]
    goal_status: In[dict]
    navigation_goal_status: In[NavigationGoalStatus]
    topo_summary: In[str]  # from SemanticMapperModule
    room_graph: In[dict]  # serialized TopologySemGraph snapshot
    llm_response: In[LLMResponse]  # symbolic semantic-intent slow path

    # -- Outputs --
    task_plan: Out[dict]
    planner_status: Out[str]
    servo_target: Out[str]  # "find:<label>" ->VisualServoModule
    agent_message: Out[dict]
    nav_command: Out[str]  # symbolic inspection/building commands -> nav.goals
    llm_request: Out[LLMRequest]  # bounded symbolic semantic-intent request

    def __init__(
        self,
        decomposer: str = "rules",
        fast_path_threshold: float = 0.75,
        frontier_score_threshold: float = 0.2,
        max_frontiers: int = 10,
        approach_distance: float = 0.5,
        verification_timeout_s: float = 20.0,
        lera_cooldown: float = _LERA_COOLDOWN,
        llm_backend: str = "kimi",
        llm_model: str = "",
        scene_graph_max_age_s: float = 0.75,
        scene_graph_future_tolerance_s: float = 0.20,
        goal_republish_position_epsilon_m: float = 0.05,
        goal_republish_yaw_epsilon_rad: float = math.radians(5.0),
        save_dir: str = "",
        map_query: object | None = None,
        **kw,
    ):
        super().__init__(**kw)
        self._decomposer_strategy = decomposer
        self._fast_threshold = fast_path_threshold
        self._save_dir = save_dir
        self._frontier_threshold = frontier_score_threshold
        self._max_frontiers = max_frontiers
        self._approach_dist = approach_distance
        self._verification_timeout_s = float(verification_timeout_s)
        if not math.isfinite(self._verification_timeout_s) or self._verification_timeout_s <= 0:
            raise ValueError("verification_timeout_s must be finite and positive")
        self._lera_cooldown = lera_cooldown
        self._llm_backend = llm_backend
        self._llm_model = llm_model
        self._scene_graph_max_age_s = max(0.0, float(scene_graph_max_age_s))
        self._scene_graph_future_tolerance_s = max(0.0, float(scene_graph_future_tolerance_s))
        self._goal_republish_position_epsilon_m = max(
            0.001,
            float(goal_republish_position_epsilon_m),
        )
        self._goal_republish_yaw_epsilon_rad = max(
            0.001,
            float(goal_republish_yaw_epsilon_rad),
        )

        # Backends (lazy init in setup)
        self._goal_resolver = None
        self._frontier_scorer = None
        self._task_decomposer = None
        self._action_executor = None
        self._backend_init_attempted = False
        self._backend_errors: dict[str, str] = {}

        # Map-frame pose published with perception, not raw odometry.
        self._robot_pos = [0.0, 0.0, 0.0]
        self._current_robot_pose: PoseStamped | None = None

        # Scene graph -keep both JSON string (for GoalResolver) and
        # the original object (for LERa label extraction).
        self._latest_sg: str | None = None
        self._current_scene_graph: SceneGraph | None = None

        # Active instruction + resolved goal
        self._current_instruction: str = ""
        self._instruction_owner = ""
        self._instruction_failure = ""
        self._instruction_lock = threading.RLock()
        self._current_goal_pose: PoseStamped | None = None
        self._last_goal_publish_signature: tuple[str, str, str, str] | None = None
        self._last_published_goal_pose: PoseStamped | None = None
        self._navigation_goals: dict[str, GoalExecution] = {}
        self._active_goal: GoalExecution | None = None
        self._visual_handoff = False
        self._pending_observation: ObservationRequest | None = None
        self._observation_retry_after = 0.0
        self._observation_image: Image | None = None
        self._verification: TargetVerification | None = None
        self._verification_lock = threading.RLock()
        self._verification_timer: threading.Timer | None = None
        self._object_candidates: dict[str, str] = {}
        self._object_verifications: dict[str, dict] = {}
        self._navigation_context: tuple[str, str, int] | None = None
        self._navigation_sequence = 0

        # LERa state -all guarded by _lera_lock
        self._lera_lock = threading.Lock()
        self._failure_count: int = 0
        self._last_nav_state: str = ""
        self._last_lera_time: float = 0.0
        self._lera_running: bool = False  # prevent concurrent LERa calls
        self._requery_count: int = 0  # cap requery_goal to avoid infinite loop

        # Sibling module references (set in on_system_modules)
        self._backends: BackendManager | None = None
        self._semantic_intent_parser = HybridSemanticIntentParser()
        self._map_query = map_query if map_query is not None else MapClient()
        self._place_catalog: PlaceCatalog | None = PlaceCatalog(_MapdPlaceQueries(self._map_query))
        self._nav_goal_service_available = False
        self._symbolic_llm_lock = threading.Lock()
        self._symbolic_llm_instruction_epoch = 0
        self._symbolic_llm_current_request_id = ""
        self._symbolic_llm_pending: dict[str, tuple[str, int]] = {}
        self._symbolic_llm_seq = 0
        self._last_vector_memory_query_only: bool = False
        self._latest_topo_summary: str = ""
        self._latest_room_graph: dict[str, Any] | None = None

        # Stats
        self._resolve_count: int = 0
        self._frontier_count: int = 0
        self._lera_count: int = 0
        self._lera_recoveries: int = 0

    def on_system_modules(self, modules: dict) -> None:
        self._backends = BackendManager(modules)
        self._nav_goal_service_available = self._backends.get("nav.goals") is not None

    def setup(self) -> None:
        self._init_backends()
        self.instruction.subscribe(self._on_instruction)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.robot_pose.subscribe(self._on_robot_pose)
        self.observation_image.subscribe(self._on_observation_image)
        self.detections.subscribe(self._on_detections)
        self.navigation_state.subscribe(self._on_navigation_state)
        self.goal_status.subscribe(self._on_goal_status)
        self.navigation_goal_status.subscribe(self._on_navigation_goal_status)
        self.topo_summary.subscribe(self._on_topo_summary)
        self.room_graph.subscribe(self._on_room_graph)
        self.llm_response.subscribe(self._on_llm_response)

    def stop(self) -> None:
        self._begin_symbolic_llm_instruction_epoch()
        self._current_instruction = ""
        self._cancel_owned_goals("semantic_planner_stopped")
        self._stop_visual_handoff()
        super().stop()

    def _init_backends(self) -> None:
        """Lazy-load algorithm backends. Each backend is independent -one failure doesn't block others."""
        self._backend_init_attempted = True
        self._backend_errors.clear()
        self._goal_resolver = None
        self._frontier_scorer = None
        self._task_decomposer = None
        self._action_executor = None

        try:
            from decision.goals.resolver import GoalResolver
            from decision.llm.client import LLMConfig

            llm_cfg = LLMConfig(backend=self._llm_backend, model=self._llm_model)
            self._goal_resolver = GoalResolver(
                primary_config=llm_cfg,
                fast_path_threshold=self._fast_threshold,
                save_dir=self._save_dir,
            )
            logger.info("GoalResolver initialized (threshold=%.2f)", self._fast_threshold)
        except Exception as e:
            self._record_backend_error("goal_resolver", e)
            logger.warning("GoalResolver not available: %s", e)

        try:
            from decision.frontiers.scorer import FrontierScorer

            self._frontier_scorer = FrontierScorer()
            logger.info("FrontierScorer initialized")
        except Exception as e:
            self._record_backend_error("frontier_scorer", e)
            logger.warning("FrontierScorer not available: %s", e)

        try:
            from decision.tasks.decomposition import TaskDecomposer

            self._task_decomposer = TaskDecomposer()
            logger.info("TaskDecomposer initialized (strategy=%s)", self._decomposer_strategy)
        except Exception as e:
            self._record_backend_error("task_decomposer", e)
            logger.warning("TaskDecomposer not available: %s", e)

        try:
            from decision.tasks.actions import ActionExecutor

            self._action_executor = ActionExecutor(
                approach_distance=self._approach_dist,
            )
            logger.info("ActionExecutor (LERa) initialized")
        except Exception as e:
            self._record_backend_error("action_executor", e)
            logger.warning("ActionExecutor not available: %s", e)

    def _record_backend_error(self, backend: str, exc: Exception) -> None:
        detail = str(exc).strip()
        if detail:
            self._backend_errors[backend] = f"{type(exc).__name__}: {detail}"
        else:
            self._backend_errors[backend] = type(exc).__name__

    # Input handlers

    # Chat-facing message helpers
    def _chat(self, role: str, text: str, phase: str | None = None) -> None:
        """Publish a chat message for the Web ChatPanel.

        role: 'thinking' | 'assistant' | 'tool'
        phase: optional sub-state hint (e.g. 'decompose', 'fast_path', 'vector')
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

    def _replace_instruction(self, owner_id: str = "") -> None:
        self._begin_symbolic_llm_instruction_epoch()
        self._instruction_owner = owner_id
        self._cancel_owned_goals("semantic_instruction_replaced")
        self._stop_visual_handoff()
        self._active_goal = None
        # Scene resolution owns only instructions that reach that branch.
        # Follow and symbolic routes must not retain an older scene goal.
        with self._lera_lock:
            self._current_instruction = ""
            self._current_goal_pose = None
            self._last_goal_publish_signature = None
            self._last_published_goal_pose = None
            self._failure_count = 0
            self._requery_count = 0
            self._last_nav_state = ""
            self._last_lera_time = 0.0

    def _on_instruction(self, text: str, *, owner_id: str = "") -> None:
        """New instruction ->decompose ->resolve (or hand off person-following)."""
        with self._instruction_lock:
            self._replace_instruction(owner_id)
            self._process_instruction(text)

    def _process_instruction(self, text: str) -> None:
        # Person-following intent ->VisualServo follow mode (bypass goal resolve).
        follow_target = self._detect_follow_intent(text)
        if follow_target is not None:
            self._visual_handoff = True
            self.servo_target.publish(f"follow:{follow_target}")
            self.planner_status.publish("FOLLOW")
            self._chat("assistant", f"Following target: {follow_target}", phase="follow")
            logger.info("Semantic planner: follow intent ->'%s'", follow_target)
            return

        self.planner_status.publish("PROCESSING")
        self._chat("thinking", "Parsing instruction", phase="parse")

        if self._try_semantic_navigation_intent(text):
            return

        self._continue_scene_resolution(text)

    def _continue_scene_resolution(self, text: str) -> None:
        """Run scene-graph, vector-memory, and frontier resolution for ``text``."""
        self._current_instruction = text
        plan = self._decompose(text)
        if plan:
            self.task_plan.publish(plan)
            subtasks = plan.get("subtasks") or []
            if len(subtasks) > 1:
                self._chat(
                    "assistant",
                    f"Created {len(subtasks)} subtasks: " + " -> ".join(subtasks[:4]),
                    phase="decompose",
                )

        if self._latest_sg:
            self._chat("thinking", "Resolving goal from scene graph", phase="resolve")
            self._try_resolve(text, self._latest_sg)
        else:
            self._chat("assistant", "No scene graph available", phase="no_sg")

    def _try_semantic_navigation_intent(self, text: str) -> bool:
        """Handle first-version symbolic navigation commands before scene-graph matching.

        Returns True when the command has been safely handled or intentionally
        refused. Returns False when scene-graph/vector/frontier resolution
        should continue.
        """

        try:
            intent = self._semantic_intent_parser.parse(text)
        except Exception as exc:
            logger.warning("Semantic intent parsing rejected instruction: %s", exc)
            self._fail_instruction("SEMANTIC_INTENT_REJECTED")
            self._chat("assistant", "I could not safely interpret that navigation command.", phase="semantic_intent")
            return True
        if intent is None:
            if self._maybe_request_symbolic_llm_intent(text):
                return True
            return False

        if intent.action is not SemanticAction.NAVIGATE:
            self._handle_tour_intent(intent)
            return True
        return self._handle_place_navigation_intent(intent)

    _SYMBOLIC_LLM_STRONG_FEATURE_RE = re.compile(
        r"(导览|参观|展厅|(?:第)?(?:负)?(?:\d{1,3}|[零〇一二两三四五六七八九十百]+)(?:楼|层)|电梯|升降梯|楼梯|步梯)"
    )
    _SYMBOLIC_LLM_MOVEMENT_RE = re.compile(r"(导航|带路|领路|带我|前往|过去|去往|去|到)")
    _SYMBOLIC_LLM_PLACE_KIND_RE = re.compile(r"(公司|会议室|办公室|展位|展区|房间|前台|大厅|展厅|工位|电梯厅|楼梯间)")
    _SYMBOLIC_LLM_REQUEST_MAX_CHARS = 160
    _SYMBOLIC_LLM_MAX_PENDING = 2

    def _maybe_request_symbolic_llm_intent(self, text: str) -> bool:
        raw_text = str(text or "").strip()
        if not raw_text or len(raw_text) > self._SYMBOLIC_LLM_REQUEST_MAX_CHARS:
            return False
        has_strong_feature = self._SYMBOLIC_LLM_STRONG_FEATURE_RE.search(raw_text) is not None
        has_place_movement = (
            self._SYMBOLIC_LLM_MOVEMENT_RE.search(raw_text) is not None
            and self._SYMBOLIC_LLM_PLACE_KIND_RE.search(raw_text) is not None
        )
        if not (has_strong_feature or has_place_movement):
            return False

        request_id = self._register_symbolic_llm_request(raw_text)
        self.llm_request.publish(
            LLMRequest(
                messages=self._symbolic_llm_messages(raw_text),
                request_id=request_id,
                temperature=0.0,
                caller="SemanticPlannerModule.symbolic_intent",
            )
        )
        self.planner_status.publish("SYMBOLIC_LLM_PENDING")
        self._chat("thinking", "Asking language model for a symbolic navigation intent.", phase="semantic_llm")
        return True

    def _register_symbolic_llm_request(self, raw_text: str) -> str:
        with self._symbolic_llm_lock:
            self._symbolic_llm_seq += 1
            request_id = f"semantic-intent-{time.time_ns()}-{self._symbolic_llm_seq}"
            self._symbolic_llm_current_request_id = request_id
            self._symbolic_llm_pending[request_id] = (
                raw_text[: self._SYMBOLIC_LLM_REQUEST_MAX_CHARS],
                self._symbolic_llm_instruction_epoch,
            )
            while len(self._symbolic_llm_pending) > self._SYMBOLIC_LLM_MAX_PENDING:
                oldest = next(iter(self._symbolic_llm_pending))
                if oldest == request_id:
                    break
                self._symbolic_llm_pending.pop(oldest, None)
            return request_id

    def _begin_symbolic_llm_instruction_epoch(self) -> None:
        self._instruction_owner = ""
        self._instruction_failure = ""
        with self._symbolic_llm_lock:
            self._symbolic_llm_instruction_epoch += 1
            self._symbolic_llm_current_request_id = ""
            self._symbolic_llm_pending.clear()
            self._pending_observation = None
            self._observation_retry_after = 0.0
        self._cancel_target_verification()
        self._observation_image = None
        self._object_candidates.clear()
        self._object_verifications.clear()

    def _symbolic_llm_epoch_is_current(self, epoch: int) -> bool:
        with self._symbolic_llm_lock:
            return epoch == self._symbolic_llm_instruction_epoch

    def _has_symbolic_llm_pending(self) -> bool:
        with self._symbolic_llm_lock:
            return bool(self._symbolic_llm_current_request_id)

    @staticmethod
    def _symbolic_llm_messages(raw_text: str) -> list[dict[str, str]]:
        schema = (
            "Return ONLY one JSON object with exactly these fields when relevant: "
            "action,target_query,floor_id,tour_id,travel_mode,confidence,"
            "needs_clarification,reason. "
            "Allowed action values: navigate,start_tour,pause_tour,resume_tour,cancel_tour. "
            "Allowed travel_mode values: any,stairs,elevator. "
            "Never include coordinates, pose, position, x, y, z, yaw, latitude, or longitude. "
            "If destination is missing, set needs_clarification true and do not invent a target. "
            "If it is not a navigation or tour-control command, return null."
        )
        return [
            {
                "role": "system",
                "content": (
                    "You extract a safe symbolic navigation intent for a quadruped robot. "
                    "You are forbidden to output coordinates or executable motion data."
                ),
            },
            {"role": "user", "content": f"{schema}\nUtterance: {raw_text}"},
        ]

    def _on_llm_response(self, resp: LLMResponse) -> None:
        request_id = str(getattr(resp, "request_id", "") or "")
        if request_id.startswith("semantic-verification-"):
            self._on_verification_response(resp)
            return
        with self._symbolic_llm_lock:
            pending = self._symbolic_llm_pending.get(request_id)
            raw_text, epoch = pending if pending is not None else ("", -1)
            is_current = bool(
                request_id
                and request_id == self._symbolic_llm_current_request_id
                and epoch == self._symbolic_llm_instruction_epoch
            )
            if not is_current:
                return
            self._symbolic_llm_pending.pop(request_id, None)
            self._symbolic_llm_current_request_id = ""

        # The LLM loop can publish this callback from a background thread.
        # Parse/map/publish outside the pending lock so a new instruction can
        # replace state without deadlocking on slow map calls or stream callbacks.
        if getattr(resp, "error", ""):
            self._fail_instruction("SYMBOLIC_LLM_FAILED", expected_epoch=epoch)
            self._chat("assistant", "I could not safely interpret that navigation command.", phase="semantic_llm")
            return
        try:
            payload = self._parse_symbolic_llm_json(getattr(resp, "text", ""))
            if payload is None:
                self._fail_instruction("SYMBOLIC_LLM_NO_INTENT", expected_epoch=epoch)
                self._chat(
                    "assistant",
                    "I could not map that sentence to a supported navigation command.",
                    phase="semantic_llm",
                )
                return
            intent = HybridSemanticIntentParser.from_symbolic_mapping(payload, raw_text=raw_text)
        except Exception as exc:
            logger.warning("Symbolic semantic LLM response rejected: %s", exc)
            self._fail_instruction("SYMBOLIC_LLM_REJECTED", expected_epoch=epoch)
            self._chat("assistant", "I could not safely interpret that navigation command.", phase="semantic_llm")
            return

        if not self._symbolic_llm_epoch_is_current(epoch):
            return
        if intent.needs_clarification:
            self._fail_instruction("PLACE_CLARIFICATION_REQUIRED", expected_epoch=epoch)
            self._chat("assistant", "Please name the place you want to go to.", phase="semantic_llm")
            return
        if intent.action is not SemanticAction.NAVIGATE:
            self._handle_tour_intent(intent, expected_symbolic_epoch=epoch)
            return
        if not self._handle_place_navigation_intent(intent, expected_symbolic_epoch=epoch):
            if not self._symbolic_llm_epoch_is_current(epoch):
                return
            self._chat(
                "thinking",
                "Symbolic intent was not grounded; trying scene-based resolution.",
                phase="semantic_llm",
            )
            self._continue_scene_resolution(raw_text)

    @staticmethod
    def _parse_symbolic_llm_json(text: str) -> dict[str, Any] | None:
        cleaned = str(text or "").strip()
        if not cleaned:
            raise ValueError("empty symbolic LLM response")
        fence = re.fullmatch(r"```(?:json)?\s*(.*?)\s*```", cleaned, flags=re.IGNORECASE | re.DOTALL)
        if fence:
            cleaned = fence.group(1).strip()
        payload = _json.loads(cleaned)
        if payload is None:
            return None
        if not isinstance(payload, dict):
            raise ValueError("symbolic LLM response must be a JSON object or null")
        return payload

    def _handle_tour_intent(self, intent: SemanticIntent, *, expected_symbolic_epoch: int | None = None) -> None:
        if not self._nav_goal_service_available:
            self._fail_instruction("TOUR_COMMAND_UNAVAILABLE", expected_epoch=expected_symbolic_epoch)
            self._chat(
                "assistant",
                "Tour control is recognized, but the navigation command service is unavailable.",
                phase="tour",
            )
            return

        if intent.action in {
            SemanticAction.PAUSE_TOUR,
            SemanticAction.RESUME_TOUR,
            SemanticAction.CANCEL_TOUR,
        }:
            # A semantic phrase such as "pause the tour" has no stable task
            # identity.  Sending it to a singleton/current-task compatibility
            # API could control a different operator's run, so keep the
            # selection explicit in the task console until session-scoped task
            # selection is designed and wired end-to-end.
            self._fail_instruction("TOUR_TASK_SELECTION_REQUIRED", expected_epoch=expected_symbolic_epoch)
            self._chat(
                "assistant",
                "Select the inspection task in the operations console before pausing, resuming, or cancelling it.",
                phase="tour",
            )
            return
        if intent.action is not SemanticAction.START_TOUR:
            self._fail_instruction("TOUR_COMMAND_REJECTED", expected_epoch=expected_symbolic_epoch)
            return
        payload = {
            "action": "inspection",
            "route_id": intent.tour_id,
        }
        if expected_symbolic_epoch is not None and not self._symbolic_llm_epoch_is_current(expected_symbolic_epoch):
            return
        self.nav_command.publish(_json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        self.planner_status.publish("TOUR_SUBMISSION_REQUESTED")
        self._chat(
            "assistant",
            "Tour task submission requested; waiting for native task confirmation.",
            phase="tour",
        )

    def _handle_place_navigation_intent(
        self,
        intent: SemanticIntent,
        *,
        expected_symbolic_epoch: int | None = None,
    ) -> bool:
        explicit_place_route = bool(intent.floor_id or intent.travel_mode is not TravelMode.ANY)
        if intent.needs_clarification or not intent.target_query:
            self._fail_instruction("PLACE_CLARIFICATION_REQUIRED", expected_epoch=expected_symbolic_epoch)
            self._chat("assistant", "Please name the place you want to go to.", phase="place")
            return True
        if self._place_catalog is None:
            if explicit_place_route:
                self._refuse_place_navigation("PLACE_CATALOG_UNAVAILABLE", "Place map is unavailable.", expected_epoch=expected_symbolic_epoch)
                return True
            return False

        active_map = self._active_map_id()
        if not active_map:
            if explicit_place_route:
                self._refuse_place_navigation("ACTIVE_MAP_REQUIRED", "No active map is selected.", expected_epoch=expected_symbolic_epoch)
                return True
            return False

        try:
            resolution = self._place_catalog.resolve(
                intent.target_query,
                map_id=active_map,
                floor_id=normalize_semantic_floor_id(intent.floor_id) if intent.floor_id else None,
            )
            if resolution.status == "not_found":
                global_resolution = self._place_catalog.resolve(
                    intent.target_query,
                    floor_id=(normalize_semantic_floor_id(intent.floor_id) if intent.floor_id else None),
                )
                if global_resolution.status != "not_found":
                    resolution = global_resolution
        except PlaceCatalogError as exc:
            logger.warning("Semantic place lookup failed: %s", exc)
            if explicit_place_route:
                self._refuse_place_navigation("PLACE_LOOKUP_FAILED", "Place lookup failed.", expected_epoch=expected_symbolic_epoch)
                return True
            return False
        except Exception as exc:
            logger.warning("Semantic place lookup unavailable: %s", exc)
            if explicit_place_route:
                self._refuse_place_navigation("PLACE_LOOKUP_UNAVAILABLE", "Place lookup is unavailable.", expected_epoch=expected_symbolic_epoch)
                return True
            return False

        if expected_symbolic_epoch is not None and not self._symbolic_llm_epoch_is_current(expected_symbolic_epoch):
            return True
        if resolution.status != "resolved" or resolution.place is None:
            if explicit_place_route or resolution.status in {"ambiguous", "stale_map"}:
                self._refuse_place_navigation(
                    f"PLACE_{resolution.status.upper()}",
                    self._place_refusal_text(resolution.status),
                    expected_epoch=expected_symbolic_epoch,
                )
                return True
            return False

        place = resolution.place
        if not self._nav_goal_service_available:
            self._refuse_place_navigation(
                "NAVIGATION_SERVICE_REQUIRED",
                "Named-place navigation is unavailable.",
                expected_epoch=expected_symbolic_epoch,
            )
            return True
        if place.map_id and place.map_id != active_map:
            self._refuse_place_navigation(
                "CROSS_MAP_NAVIGATION_UNSUPPORTED",
                "The place is on another map; cross-map navigation is not supported.",
                expected_epoch=expected_symbolic_epoch,
            )
            return True
        if not self._place_is_executable(place):
            self._refuse_place_navigation(
                "PLACE_NOT_EXECUTABLE",
                f"Place is not executable: {place.non_executable_reason or 'missing map pose'}.",
                expected_epoch=expected_symbolic_epoch,
            )
            return True
        self._publish_place_navigation(place, expected_symbolic_epoch=expected_symbolic_epoch)
        return True

    @staticmethod
    def _place_is_executable(place: PlaceRef) -> bool:
        return (
            place.executable
            and place.x is not None
            and place.y is not None
            and place.z is not None
            and bool(place.frame_id)
        )

    def _publish_place_navigation(
        self,
        place: PlaceRef,
        *,
        expected_symbolic_epoch: int | None = None,
    ) -> None:
        if expected_symbolic_epoch is not None and not self._symbolic_llm_epoch_is_current(expected_symbolic_epoch):
            return
        self.planner_status.publish("PLACE_GOAL_DISPATCHED")
        task = self._dispatch_navigation_goal(
            PoseStamped(
                Pose(Vector3(float(place.x), float(place.y), float(place.z)), Quaternion.from_yaw(float(place.yaw or 0.0))),
                frame_id=place.frame_id,
            ),
            instruction=place.name,
            purpose="place",
        )
        if task.state not in {"rejected", "unconfirmed"}:
            self._chat(
                "assistant",
                f"Navigation goal sent for {place.name}.",
                phase="place",
            )

    def _fail_instruction(self, status: str, *, expected_epoch: int | None = None) -> bool:
        with self._instruction_lock:
            if expected_epoch is not None and not self._symbolic_llm_epoch_is_current(expected_epoch):
                return False
            self._instruction_failure = status
            self._current_instruction = ""
            self.planner_status.publish(status)
            return True

    def _refuse_place_navigation(self, status: str, text: str, *, expected_epoch: int | None = None) -> None:
        if self._fail_instruction(status, expected_epoch=expected_epoch):
            self._chat("assistant", text, phase="place")

    @staticmethod
    def _place_refusal_text(status: str) -> str:
        if status == "ambiguous":
            return "More than one matching place was found; I will not guess."
        if status == "stale_map":
            return "The matching place is not bound to the current map version."
        return "I could not find that place in the active place map."

    def _active_map_id(self) -> str:
        service = getattr(self._map_query, "service", None)
        if not callable(service):
            return ""
        try:
            response = service("get_active_map")
        except Exception:
            logger.debug("active map query failed", exc_info=True)
            return ""
        if isinstance(response, dict) and response.get("success") is True:
            return str(response.get("active") or response.get("map_id") or "")
        return ""

    # Follow-verb patterns: explicit multi-character Chinese verbs plus "follow".
    _FOLLOW_PATTERN = re.compile(
        r"(\u8ddf\u968f|\u8ddf\u7740|\u8ddf\u4f4f|\u5c3e\u968f|follow)\s*(.*)",
        re.IGNORECASE,
    )

    def _detect_follow_intent(self, text: str) -> str | None:
        """Return the person description if `text` is a follow request, else None.

        Examples::

            "follow the person in red" -> "the person in red"
            "tell me where the person is" -> None

        """
        raw = (text or "").strip()
        if not raw:
            return None
        m = self._FOLLOW_PATTERN.search(raw)
        if not m:
            return None
        target = m.group(2).strip(" \t\r\n.!,;:")
        return target or "person"

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        """Scene graph update ->cache + re-resolve if active instruction."""
        if self._map_sample_is_stale(sg):
            self._latest_sg = None
            self._current_scene_graph = None
            if self._current_instruction:
                self.planner_status.publish("WAITING_FOR_FRESH_SCENE_GRAPH")
            return
        sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
        self._latest_sg = sg_json
        self._current_scene_graph = sg  # keep object for LERa label extraction
        self._try_verify_target()

        if self._current_instruction and self._goal_resolver and not self._has_symbolic_llm_pending():
            self._try_resolve(self._current_instruction, sg_json)

    def _on_robot_pose(self, pose: PoseStamped) -> None:
        if self._map_sample_is_stale(pose) or not all(math.isfinite(float(v)) for v in (pose.x, pose.y, pose.z)):
            self._current_robot_pose = None
            return
        self._current_robot_pose = pose
        self._robot_pos = np.array([pose.x, pose.y, pose.z])

    def _on_observation_image(self, image: Image) -> None:
        self._observation_image = image

    def _map_sample_is_stale(self, sg: SceneGraph | PoseStamped) -> bool:
        frame_id = str(getattr(sg, "frame_id", "") or "")
        if frame_id != SEMANTIC_PLANNER_MAP_FRAME_ID:
            return True
        return self._timestamp_is_stale(float(getattr(sg, "ts", 0.0) or 0.0))

    def _timestamp_is_stale(self, ts: float) -> bool:
        if ts <= 0.0 or not math.isfinite(ts):
            return True
        age = time.time() - ts
        if age < -self._scene_graph_future_tolerance_s:
            return True
        if self._scene_graph_max_age_s <= 0.0:
            return False
        return age > self._scene_graph_max_age_s

    @staticmethod
    def _wrap_angle_delta_rad(a: float, b: float) -> float:
        return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

    def _goal_pose_within_republish_hysteresis(
        self,
        previous: PoseStamped,
        current: PoseStamped,
    ) -> bool:
        dx = float(current.x) - float(previous.x)
        dy = float(current.y) - float(previous.y)
        dz = float(current.z) - float(previous.z)
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if not math.isfinite(distance):
            return False
        if distance > self._goal_republish_position_epsilon_m:
            return False
        yaw_delta = self._wrap_angle_delta_rad(float(current.yaw), float(previous.yaw))
        return yaw_delta <= self._goal_republish_yaw_epsilon_rad

    def _goal_updates_suspended(self) -> bool:
        task = self._active_goal
        return task is not None and (
            task.state in {"unconfirmed", "cancel_requested", "recovering", "paused"}
            or (not task.terminal and self._last_nav_state in {"RECOVERING", "PAUSED"})
        )

    def _publish_goal_pose_once(
        self, instruction: str, pose: PoseStamped, *, purpose: str = "object", target: ObjectTarget | None = None,
    ) -> bool:
        if self._goal_updates_suspended():
            return False
        signature = (instruction, str(pose.frame_id or ""), purpose, target.object_id if target else "")
        self._current_goal_pose = pose
        if (
            signature == self._last_goal_publish_signature
            and self._last_published_goal_pose is not None
            and self._goal_pose_within_republish_hysteresis(
                self._last_published_goal_pose,
                pose,
            )
        ):
            return False
        self._last_goal_publish_signature = signature
        self._last_published_goal_pose = pose
        self.planner_status.publish({"object": "RESOLVED", "memory": "VECTOR_MEMORY", "frontier": "EXPLORING"}[purpose])
        task = self._dispatch_navigation_goal(pose, instruction=instruction, purpose=purpose, target=target)
        return task.state in {"dispatching", "accepted", "planning", "path_active", "paused"}

    def _dispatch_navigation_goal(
        self, pose: PoseStamped, *, instruction: str, purpose: str, target: ObjectTarget | None = None,
        verification: TargetVerification | None = None,
    ) -> GoalExecution:
        self._stop_visual_handoff()
        task = GoalExecution(
            task_id=f"semantic-task-{uuid.uuid4().hex}",
            request_id=f"semantic-goal-{uuid.uuid4().hex}",
            instruction_epoch=self._symbolic_llm_instruction_epoch,
            instruction=instruction,
            purpose=purpose,
            pose=pose,
            target=target,
            verification=verification,
        )
        if verification is not None:
            verification.task_id = task.task_id
            verification.viewpoints.append((pose.x, pose.y, pose.z))
        elif self._verification is not None and self._verification.terminal:
            self._cancel_target_verification()
        self._navigation_goals[task.task_id] = task
        self._active_goal = task
        self._last_nav_state = ""
        self.nav_command.publish(_json.dumps({
            "action": "goto", "task_id": task.task_id, "request_id": task.request_id,
            "source": "semantic", "frame_id": pose.frame_id,
            "x": float(pose.x), "y": float(pose.y), "z": float(pose.z), "yaw": float(pose.yaw),
            # A new view must not be accepted inside the previous view's radius.
            **({"acceptance_radius_m": 0.15} if verification is not None else {}),
        }))
        return task

    def _cancel_owned_goals(self, reason: str) -> None:
        for task in tuple(self._navigation_goals.values()):
            if task.terminal or task.cancel_request_id:
                continue
            task.cancel_request_id = f"semantic-cancel-{uuid.uuid4().hex}"
            task.state = "cancel_requested"
            self.nav_command.publish(_json.dumps({
                "action": "cancel", "task_id": task.task_id,
                "request_id": task.cancel_request_id, "reason": reason,
            }))

    def _on_goal_status(self, status: dict) -> None:
        task = self._navigation_goals.get(str(status.get("task_id") or ""))
        if (task is None and status.get("action") in {"goal", "goal_pose"}
                and status.get("accepted") is True and not status.get("replay")
                and isinstance(status.get("target"), dict)):
            # Operator/API goals use the same GoalService, but do not enter the
            # semantic instruction port. Invalidate pending model work here.
            with self._instruction_lock:
                self._replace_instruction()
                self.planner_status.publish("SUPERSEDED")
            return
        if task is None or task.terminal:
            return
        request_id = str(status.get("request_id") or "")
        if request_id == task.cancel_request_id:
            # A cancel ACK is admission, not proof that motion has ended.
            if task is self._active_goal:
                self.planner_status.publish("CANCELLING" if status.get("accepted") is True else "CANCEL_UNCONFIRMED")
            return
        if request_id != task.request_id or task.cancel_request_id:
            return
        # Native lifecycle evidence can arrive before its synchronous ACK.
        if task.sequence:
            return
        task.reason = str(status.get("reason") or status.get("message") or "")
        if status.get("admission_unconfirmed") is True:
            task.state = "unconfirmed"
        elif status.get("accepted") is True:
            task.state = "accepted"
        else:
            task.state = "rejected"
            task.terminal = True
            self._navigation_goals.pop(task.task_id, None)
            if task is self._active_goal:
                self._current_instruction = ""
                self._cancel_owned_goals("semantic_replacement_rejected")
        if task is self._active_goal:
            self.planner_status.publish(f"NAVIGATION_{task.state.upper()}")
            if task.verification is not None and task.state in {"rejected", "unconfirmed"}:
                self._finish_target_verification(task.verification, "unavailable", f"observation_goal_{task.state}")
                if not task.terminal:
                    self._cancel_owned_goals("semantic_observation_admission_unconfirmed")

    def _on_navigation_goal_status(self, status: NavigationGoalStatus) -> None:
        # GoalService has validated this event against the admitted task ledger.
        task = self._navigation_goals.get(status.task_id)
        if task is None or task.terminal or status.frame_id != task.pose.frame_id:
            return
        if task.boot_id and status.boot_id != task.boot_id:
            return
        if int(status.sequence) <= task.sequence:
            return
        task.boot_id = status.boot_id
        task.sequence = int(status.sequence)
        task.reason = status.reason
        task.state = str(status.to_dict()["state_name"]).lower()
        task.terminal = status.terminal
        if status.terminal:
            self._navigation_goals.pop(task.task_id, None)
        if task is not self._active_goal or not self._symbolic_llm_epoch_is_current(task.instruction_epoch):
            return
        if task.cancel_request_id and not status.terminal:
            task.state = "cancel_requested"
            return
        if task.verification is not None and task.verification.terminal:
            self.planner_status.publish(task.verification.status)
            return
        if int(status.state) == int(NavigationGoalState.REACHED):
            self._current_instruction = ""
            self._current_goal_pose = None
            if task.purpose == "object" and task.target is not None and not task.cancel_request_id:
                self._start_target_verification(task, float(status.ts))
            else:
                self.planner_status.publish("COMPLETED" if task.purpose == "place" else "TARGET_VERIFICATION_REQUIRED")
        elif int(status.state) == int(NavigationGoalState.CANCELLED):
            self._begin_symbolic_llm_instruction_epoch()
            self._current_instruction = ""
            self._current_goal_pose = None
            self.planner_status.publish("CANCELLED")
        elif int(status.state) == int(NavigationGoalState.FAILED):
            self.planner_status.publish("NAVIGATION_FAILED")
            if task.verification is not None:
                self._finish_target_verification(task.verification, "uncertain", "observation_navigation_failed")
            elif task.cancel_request_id:
                self._current_instruction = ""
                self._current_goal_pose = None
            elif task.purpose != "place":
                self._request_recovery(task)
        else:
            self.planner_status.publish(f"NAVIGATION_{task.state.upper()}")

    def _cancel_target_verification(self) -> None:
        with self._verification_lock:
            if self._verification is not None and not self._verification.terminal:
                self._verification.state = "cancelled"
                self._verification.reason = "instruction_or_context_changed"
                self._verification.sample = None
                self._verification.request_id = ""
            self._verification = None
            if self._verification_timer is not None:
                self._verification_timer.cancel()
                self._verification_timer = None

    def _verification_is_current(self, state: TargetVerification) -> bool:
        return (
            self._verification is state and not state.terminal
            and state.instruction_epoch == self._symbolic_llm_instruction_epoch
            and self._active_goal is not None and self._active_goal.task_id == state.task_id
        )

    def _start_target_verification(self, task: GoalExecution, arrived_at: float) -> None:
        state = task.verification
        if state is not None and self._verification_is_current(state):
            # Reaching another view does not renew the original time/call budget.
            with self._verification_lock:
                state.state, state.arrived_at, state.view_attempts = "waiting", arrived_at, 0
                self.planner_status.publish("VERIFYING_TARGET")
            return
        self._cancel_target_verification()
        state = TargetVerification(
            task.task_id, task.instruction_epoch, task.instruction, task.target,
            arrived_at, time.monotonic() + self._verification_timeout_s,
        )
        with self._verification_lock:
            state.viewpoints.append((task.pose.x, task.pose.y, task.pose.z))
            task.verification = self._verification = state
            llm = self._backends.llm_module if self._backends else None
            client = getattr(llm, "client", None)
            if getattr(client, "supports_vision", False) is not True:
                self._finish_target_verification(state, "unavailable", "vision_model_unavailable")
                return
            self.planner_status.publish("VERIFYING_TARGET")
            self._chat("thinking", "Reached the observation position; checking the requested object.", phase="target_verification")
            self._verification_timer = threading.Timer(
                self._verification_timeout_s, self._expire_target_verification, args=(state,),
            )
            self._verification_timer.daemon = True
            self._verification_timer.start()

    def _expire_target_verification(self, state: TargetVerification) -> None:
        with self._verification_lock:
            if not self._verification_is_current(state):
                return
            self._pending_observation = None
            self._finish_target_verification(state, "timeout", "verification_deadline_exceeded")
            self._cancel_owned_goals("semantic_verification_timeout")

    def _expire_alternative_preview(self, state: TargetVerification) -> None:
        with self._verification_lock:
            if (
                self._verification is not state or not state.terminal
                or not self._symbolic_llm_epoch_is_current(state.instruction_epoch)
                or self._active_goal is None or self._active_goal.task_id != state.task_id
                or not self._active_goal.terminal
            ):
                return
            self._pending_observation = None
            for object_id, stage in self._object_candidates.items():
                if stage in {"waiting_path", "checking_path"}:
                    self._object_candidates[object_id] = "path_wait_expired"
            if self._verification_timer is not None:
                self._verification_timer.cancel()
                self._verification_timer = None
            self.planner_status.publish(state.status)

    def _finish_target_verification(self, state: TargetVerification, outcome: str, reason: str) -> None:
        with self._verification_lock:
            if not self._verification_is_current(state):
                return
            state.state, state.reason = outcome, reason
            self._current_goal_pose = None
            self._object_candidates[state.target.object_id] = outcome
            state.sample, state.request_id = None, ""
            self._object_verifications[state.target.object_id] = state.to_dict()
            if self._verification_timer is not None:
                self._verification_timer.cancel()
                self._verification_timer = None
            self.planner_status.publish(state.status)
            self._chat("assistant", {
                "confirmed": "The requested target was visually confirmed in two fresh observations.",
                "mismatch": "The observed candidate does not match the requested target.",
                "uncertain": "The available views are inconclusive; the target is not confirmed.",
                "timeout": "Target verification timed out; the target is not confirmed.",
                "unavailable": "Visual verification is unavailable; arrival alone does not confirm the target.",
            }[outcome], phase="target_verification")
            if outcome in {"mismatch", "uncertain"}:
                self._continue_object_search(state)

    def _continue_object_search(self, state: TargetVerification) -> None:
        """Try another freshly observed candidate, not the rejected object's memory."""
        if (
            self._verification is not state or not state.terminal
            or not self._symbolic_llm_epoch_is_current(state.instruction_epoch)
            or self._active_goal is None or not self._active_goal.terminal
            or self._active_goal.cancel_request_id
            or len(self._object_candidates) >= 3 or self._goal_resolver is None
        ):
            return
        scene, robot = self._current_scene_graph, self._current_robot_pose
        if scene is None or robot is None or self._map_sample_is_stale(scene) or self._map_sample_is_stale(robot):
            return
        grounding = self._grounding_scene()
        candidates = {str(obj["id"]) for obj in grounding["objects"]} - self._object_candidates.keys()
        if not candidates:
            return
        try:
            result = self._goal_resolver.fast_resolve(
                state.instruction, _json.dumps(grounding),
                robot_position=dict(zip(("x", "y", "z"), map(float, self._robot_pos))),
                excluded_object_ids=set(self._object_candidates),
            )
        except Exception:
            logger.exception("Alternative target resolution failed")
            return
        object_id = str(getattr(result, "candidate_id", ""))
        if (
            getattr(result, "confidence", 0.0) >= self._fast_threshold
            and getattr(result, "action", "navigate") != "explore"
            and object_id in candidates
            and self._verification is state
            and self._symbolic_llm_epoch_is_current(state.instruction_epoch)
        ):
            self._queue_observation_goal(state.instruction, object_id, after_verification=state)

    def _retry_target_view(self, state: TargetVerification) -> None:
        if not self._verification_is_current(state):
            return
        llm = self._backends.llm_module if self._backends else None
        if getattr(getattr(llm, "client", None), "supports_vision", False) is not True:
            self._finish_target_verification(state, "unavailable", "vision_model_unavailable")
            return
        if len(state.viewpoints) >= 3:
            self._finish_target_verification(state, "uncertain", "observation_view_budget_exhausted")
            return
        if state.state != "repositioning":
            state.state = "repositioning"
            self.planner_status.publish("REOBSERVING_TARGET")
            self._chat("thinking", "The current view is inconclusive; checking another observation position.",
                       phase="target_verification")
        self._queue_observation_goal(state.instruction, state.target.object_id, after_verification=state)

    def _verification_observation(self, state: TargetVerification):
        scene, robot, image = self._current_scene_graph, self._current_robot_pose, self._observation_image
        if scene is None or robot is None or image is None:
            return None
        if self._map_sample_is_stale(scene) or self._map_sample_is_stale(robot) or self._timestamp_is_stale(image.ts):
            return None
        obj = scene.get_object_by_id(state.target.object_id)
        if obj is None or obj.ts != scene.ts or image.ts != scene.ts or robot.ts != scene.ts:
            return None
        if obj.ts <= state.arrived_at or obj.label.casefold() != state.target.label.casefold():
            return None
        position = (obj.position.x, obj.position.y, obj.position.z)
        if not all(math.isfinite(value) for value in position):
            return None
        if math.dist(position, state.target.position) > max(0.15, self._goal_republish_position_epsilon_m):
            return None
        if math.hypot(robot.x - position[0], robot.y - position[1]) > self._approach_dist + 0.5:
            return None
        bbox = image_bbox(obj.bbox_2d, image.width, image.height)
        if bbox is None:
            return None
        return obj, robot, image, bbox

    def _try_verify_target(self) -> None:
        with self._verification_lock:
            state = self._verification
            if state is not None and state.terminal:
                waiting = next((oid for oid, stage in self._object_candidates.items() if stage == "waiting_path"), None)
                if (
                    waiting is not None and self._symbolic_llm_epoch_is_current(state.instruction_epoch)
                    and self._active_goal is not None and self._active_goal.task_id == state.task_id
                    and self._active_goal.terminal and not self._active_goal.cancel_request_id
                ):
                    if time.monotonic() >= state.deadline:
                        self._expire_alternative_preview(state)
                    else:
                        self._queue_observation_goal(state.instruction, waiting, after_verification=state)
                return
            if state is None or not self._verification_is_current(state) or state.request_id:
                return
            if time.monotonic() >= state.deadline:
                self._expire_target_verification(state)
                return
            if state.state == "repositioning":
                if self._active_goal.terminal:
                    self._retry_target_view(state)
                return
            observation = self._verification_observation(state)
            if observation is None:
                return
            obj, robot, image, bbox = observation
            if obj.ts <= state.last_sample_ts or (state.attempts and obj.ts - state.last_sample_ts < 0.5):
                return
            try:
                sample = VerificationSample(
                    obj.ts, (obj.position.x, obj.position.y, obj.position.z),
                    (robot.x, robot.y, robot.z), bbox, image.to_bgr().data.copy(),
                )
            except (ValueError, TypeError):
                self._finish_target_verification(state, "unavailable", "invalid_observation_image")
                return
            state.state, state.reason = "checking", ""
            state.attempts += 1
            state.view_attempts += 1
            state.sample = sample
            state.last_sample_ts = sample.timestamp
            state.request_id = f"semantic-verification-{uuid.uuid4().hex}"
            threading.Thread(
                target=self._publish_verification_request, args=(state, state.request_id, sample),
                name="semantic-verification", daemon=True,
            ).start()

    def _publish_verification_request(self, state: TargetVerification, request_id: str, sample: VerificationSample) -> None:
        try:
            messages = verification_messages(state.instruction, state.target, sample)
        except Exception:
            logger.exception("Target verification image preparation failed")
            self._finish_target_verification(state, "unavailable", "image_encoding_failed")
            return
        with self._verification_lock:
            if not self._verification_is_current(state) or state.request_id != request_id:
                return
            self.llm_request.publish(LLMRequest(
                messages=messages, request_id=request_id, temperature=0.0,
                caller="SemanticPlannerModule.target_verification",
            ))

    def _on_verification_response(self, response: LLMResponse) -> None:
        with self._verification_lock:
            state = self._verification
            if state is None or not self._verification_is_current(state) or response.request_id != state.request_id:
                return
            if time.monotonic() >= state.deadline:
                self._expire_target_verification(state)
                return
            if response.error:
                self._finish_target_verification(state, "unavailable", "vision_model_error")
                return
            try:
                verdict, reason = parse_verdict(response.text, state.target.object_id)
            except (ValueError, TypeError):
                self._finish_target_verification(state, "unavailable", "invalid_vision_response")
                return
            sample = state.sample
            state.request_id, state.sample = "", None
            current = self._verification_observation(state)
            if current is None or current[0].ts < sample.timestamp:
                verdict, reason = "uncertain", "observation_changed_during_verification"
            state.evidence.append({
                "task_id": state.task_id, "view_index": len(state.viewpoints) - 1,
                "timestamp": sample.timestamp, "robot_position": list(sample.robot_position),
                "object_position": list(sample.position), "bbox_pixels": list(sample.bbox),
                "verdict": verdict, "reason": reason, "model": response.model,
            })
            state.reason = reason
            if verdict == "mismatch":
                self._finish_target_verification(state, "mismatch", reason)
            elif verdict == "match":
                state.confirmations += 1
                if state.confirmations >= 2:
                    self._finish_target_verification(state, "confirmed", reason)
                else:
                    state.state = "waiting"
            else:
                state.state = "waiting"
            if not state.terminal and state.view_attempts >= 3:
                self._retry_target_view(state)
            if not state.terminal:
                self._try_verify_target()

    def _on_detections(self, dets: list) -> None:
        """Detection update -consumed by scene_graph path."""
        pass

    def _on_topo_summary(self, summary: str) -> None:
        self._latest_topo_summary = summary or ""

    def _on_room_graph(self, snapshot: dict) -> None:
        if not isinstance(snapshot, dict):
            return
        self._latest_room_graph = snapshot
        if self._goal_resolver is not None and hasattr(self._goal_resolver, "set_topology_graph_snapshot"):
            self._goal_resolver.set_topology_graph_snapshot(snapshot)

    def _on_navigation_state(self, status: NavigationState) -> None:
        """Cache native state; native recovery retains ownership until failure."""
        context = (status.boot_id, status.map_id, int(status.map_content_epoch))
        previous = self._navigation_context
        if previous is not None and previous[0] == context[0] and status.sequence <= self._navigation_sequence:
            return
        self._navigation_context = context
        self._navigation_sequence = int(status.sequence)
        if previous is not None and previous != context:
            self._begin_symbolic_llm_instruction_epoch()
            self._current_instruction = ""
            self._current_goal_pose = None
            self._latest_sg = None
            self._current_scene_graph = None
            self._current_robot_pose = None
            self._last_goal_publish_signature = None
            self._last_published_goal_pose = None
            self._cancel_owned_goals("semantic_navigation_context_changed")
            self._stop_visual_handoff()
            self._active_goal = None
            self.planner_status.publish("NAVIGATION_CONTEXT_CHANGED")
            return
        state = str(status.to_dict().get("lifecycle_state_name") or "")
        task = self._active_goal
        if task is None or status.active_task_id != task.task_id or task.cancel_request_id:
            return
        if task.boot_id and status.boot_id != task.boot_id:
            return

        with self._lera_lock:
            self._last_nav_state = state

    def _request_recovery(self, task: GoalExecution) -> None:
        """Recover only a failed goal owned by the current semantic instruction."""
        with self._lera_lock:
            if not self._current_instruction or task is not self._active_goal or task.state != "failed":
                return
            if self._lera_running:
                return
            task.state = "recovering"
            self._failure_count += 1
            self._lera_count += 1
            self._lera_running = True

            # Snapshot mutable state for the background thread (avoids races).
            labels: list[str] = (
                [obj.label for obj in self._current_scene_graph.objects if obj.label]
                if self._current_scene_graph
                else []
            )
            instruction = self._current_instruction
            failure_count = self._failure_count
            llm_client = getattr(self._goal_resolver, "_primary", None) if self._goal_resolver else None

        logger.info(
            "[LERa] Triggered: task=%s failure#%d instruction='%s'", task.task_id, failure_count, instruction[:40]
        )

        # Publish RECOVERING synchronously before thread launch so the UI
        # reflects the state change without waiting for the LLM call.
        self.planner_status.publish("RECOVERING")

        # Dispatch Explain+Replan to a daemon thread -never block odom chain.
        threading.Thread(
            target=self._run_lera,
            args=(instruction, labels, failure_count, llm_client, task),
            daemon=True,
            name="lera-recovery",
        ).start()

    def _run_lera(
        self,
        instruction: str,
        labels: list[str],
        failure_count: int,
        llm_client: Any | None,
        task: GoalExecution,
    ) -> None:
        """Explain/replan off the callback thread and discard superseded results."""
        try:
            # Delay a distinct retry instead of dropping its sole terminal event.
            delay = max(0.0, self._last_lera_time + self._lera_cooldown - time.monotonic())
            if delay:
                threading.Event().wait(delay)
            if not self._recovery_is_current(task):
                return
            self._last_lera_time = time.monotonic()
            if self._action_executor is not None:
                strategy = self._action_executor.lera_recover(
                    failed_action=instruction,
                    current_labels=labels,
                    original_goal=instruction,
                    failure_count=failure_count,
                    llm_client=llm_client,
                    event_loop=None,
                )
            else:
                # Rule-based fallback (mirrors ActionExecutor defaults).
                if failure_count >= 3:
                    strategy = "abort"
                elif failure_count >= 2:
                    strategy = "expand_search"
                else:
                    strategy = "retry_different_path"

            # An LLM response can arrive seconds after another instruction.
            if not self._recovery_is_current(task):
                return
            logger.info("[LERa] Strategy: %s (failure#%d)", strategy, failure_count)
            self._lera_recoveries += 1
            self._dispatch_recovery(strategy)
        except Exception:
            logger.exception("[LERa] Unexpected error in recovery thread")
            if self._recovery_is_current(task):
                self._current_instruction = ""
                self.planner_status.publish("FAILED")
        finally:
            with self._lera_lock:
                self._lera_running = False
            # A replacement can fail while the previous worker is returning.
            current = self._active_goal
            if current is not None and current is not task and current.terminal and current.state == "failed":
                self._request_recovery(current)

    def _recovery_is_current(self, task: GoalExecution) -> bool:
        return (
            task is self._active_goal
            and bool(self._current_instruction)
            and self._symbolic_llm_epoch_is_current(task.instruction_epoch)
        )

    # LERa recovery dispatch

    def _dispatch_recovery(self, strategy: str) -> None:
        """Map LERa strategy string to port actions."""
        if strategy == "retry_different_path":
            # Resolve again: a cached object pose may have expired during recovery.
            self._last_goal_publish_signature = None
            self._active_goal.state = "failed"
            self._try_resolve(self._current_instruction, self._latest_sg or "")

        elif strategy == "expand_search":
            # Ask FrontierScorer for an unexplored vantage point.
            self._active_goal.state = "failed"
            self._explore_frontier(self._current_instruction)

        elif strategy == "requery_goal":
            with self._lera_lock:
                self._requery_count += 1
                requery_count = self._requery_count
            if requery_count > 2:
                # LLM kept suggesting requery -force abort to avoid infinite loop.
                logger.warning("[LERa] requery_goal capped (%d), forcing abort", requery_count)
                self._cancel_owned_goals("lera_abort")
                self.planner_status.publish("ABORTED")
                with self._lera_lock:
                    self._failure_count = 0
                    self._requery_count = 0
                self._current_instruction = ""
                self._current_goal_pose = None
                self._last_goal_publish_signature = None
                self._last_published_goal_pose = None
            else:
                # Re-run full Fast->Slow resolution with the current scene graph.
                with self._lera_lock:
                    self._failure_count = 0
                self._last_goal_publish_signature = None
                self._active_goal.state = "failed"
                if self._latest_sg and self._current_instruction:
                    self._try_resolve(self._current_instruction, self._latest_sg)
                else:
                    self.planner_status.publish("FAILED")

        elif strategy == "abort":
            self._cancel_owned_goals("lera_abort")
            self.planner_status.publish("ABORTED")
            self._failure_count = 0
            self._current_instruction = ""
            self._current_goal_pose = None
            self._last_goal_publish_signature = None
            self._last_published_goal_pose = None

        else:
            logger.warning("[LERa] Unknown strategy '%s', defaulting to abort", strategy)
            self._dispatch_recovery("abort")

    # Decomposition

    def _decompose(self, instruction: str) -> dict | None:
        if self._task_decomposer is None:
            return {"subtasks": [instruction]}
        try:
            if self._decomposer_strategy == "rules":
                plan = self._task_decomposer.decompose_with_rules(instruction)
                if plan is None:
                    return {"subtasks": [instruction]}
                # decompose_with_rules returns a TaskPlan dataclass; normalize
                # to the dict shape that _on_instruction reads ("subtasks").
                return {
                    "instruction": plan.instruction,
                    "subtasks": [sg.target for sg in plan.subgoals],
                    "subgoals": [sg.to_dict() for sg in plan.subgoals],
                }
            return {"subtasks": [instruction]}
        except Exception:
            logger.exception("Task decomposition failed")
            return {"subtasks": [instruction]}

    # Goal Resolution

    def _try_resolve(self, instruction: str, sg_json: str) -> None:
        if self._goal_updates_suspended():
            return
        # A graph fresh on arrival can expire while perception is disconnected.
        # Recheck when consuming the cache, including recovery and LLM fallback.
        if self._current_scene_graph is None or self._map_sample_is_stale(self._current_scene_graph):
            self._latest_sg = None
            self._current_scene_graph = None
            self.planner_status.publish("WAITING_FOR_FRESH_SCENE_GRAPH")
            return
        self._last_vector_memory_query_only = False
        if self._current_robot_pose is None or self._map_sample_is_stale(self._current_robot_pose):
            self.planner_status.publish("WAITING_FOR_MAP_POSE")
            return
        active = self._active_goal
        if active is not None and not active.terminal and active.target is not None:
            # A transient recognition miss must not replace an admitted approach.
            # Native navigation still owns obstacle handling and task failure.
            self._queue_observation_goal(instruction, active.target.object_id)
            return
        if self._goal_resolver is None:
            # No GoalResolver -skip Fast Path, try remaining fallbacks
            if self._try_vector_memory(instruction):
                return
            if self._last_vector_memory_query_only:
                return
            self._explore_frontier(instruction)
            return

        # Level 2: Fast Path (scene graph matching)
        try:
            self._goal_resolver.maybe_reload_kg()
            grounding_scene = self._grounding_scene()
            result = self._goal_resolver.fast_resolve(
                instruction,
                _json.dumps(grounding_scene),
                robot_position=dict(zip(("x", "y", "z"), map(float, self._robot_pos))),
            )
            if result and hasattr(result, "confidence") and result.confidence >= self._fast_threshold:
                self._resolve_count += 1
                if getattr(result, "action", "navigate") == "explore":
                    self._explore_frontier(instruction)
                else:
                    self._queue_observation_goal(instruction, str(getattr(result, "candidate_id", "")))
                return
        except Exception:
            logger.exception("Fast path resolution failed")

        # Level 3: Vector Memory (CLIP embedding search)
        self._chat("thinking", "Fast path failed; checking vector memory", phase="vector")
        if self._try_vector_memory(instruction):
            return
        if self._last_vector_memory_query_only:
            return

        # Level 4: Frontier exploration ->Level 5: Visual servo
        self._chat("thinking", "Trying frontier exploration", phase="frontier")
        self._explore_frontier(instruction)

    def _grounding_scene(self) -> dict:
        # Retained tracks are memory, not current object-goal candidates.
        scene = self._current_scene_graph.to_dict()
        scene["objects"] = [obj for obj in scene["objects"]
                            if not self._timestamp_is_stale(float(obj["ts"]))]
        current_ids = {obj["id"] for obj in scene["objects"]}
        scene["relations"] = [rel for rel in scene["relations"]
                              if rel["subject_id"] in current_ids and rel["object_id"] in current_ids]
        for region in scene["regions"]:
            region["object_ids"] = [oid for oid in region["object_ids"] if oid in current_ids]
        return scene

    def _observed_target(self, object_id: str) -> ObjectTarget | None:
        scene = self._current_scene_graph
        if scene is None or self._map_sample_is_stale(scene):
            return None
        obj = scene.get_object_by_id(object_id)
        if obj is None:
            return None
        # Scene graphs can retain a track after its last actual observation.
        if self._timestamp_is_stale(float(obj.ts)):
            return None
        position = (float(obj.position.x), float(obj.position.y), float(obj.position.z))
        if not all(math.isfinite(v) for v in position):
            return None
        return ObjectTarget(object_id, obj.label, position)

    def _queue_observation_goal(
        self, instruction: str, object_id: str, *, after_verification: TargetVerification | None = None,
    ) -> None:
        if self._pending_observation is not None or time.monotonic() < self._observation_retry_after:
            return
        robot = self._current_robot_pose
        if robot is None or self._map_sample_is_stale(robot):
            self.planner_status.publish("WAITING_FOR_MAP_POSE")
            return
        target = self._observed_target(object_id)
        if target is None:
            self.planner_status.publish("TARGET_NOT_OBSERVED")
            return
        if object_id not in self._object_candidates and len(self._object_candidates) >= 3:
            self.planner_status.publish("TARGET_SEARCH_EXHAUSTED")
            return
        if after_verification is not None and (
            self._verification is not after_verification
            or not self._symbolic_llm_epoch_is_current(after_verification.instruction_epoch)
            or time.monotonic() >= after_verification.deadline
        ):
            return
        active = self._active_goal
        if active is not None and not active.terminal and active.target is not None:
            if active.target.object_id == object_id and math.dist(active.target.position, target.position) <= self._goal_republish_position_epsilon_m:
                return
        commands = self._backends.get("nav.commands") if self._backends else None
        preview = getattr(commands, "preview_plan", None)
        if not callable(preview):
            self.planner_status.publish("OBSERVATION_PLANNER_UNAVAILABLE")
            if after_verification is not None and not after_verification.terminal:
                self._finish_target_verification(after_verification, "unavailable", "observation_planner_unavailable")
            return
        request = ObservationRequest(self._symbolic_llm_instruction_epoch, instruction, target, robot,
                                     after_verification.task_id if after_verification is not None else "")
        self._object_candidates[object_id] = "checking_path"
        self._pending_observation = request
        if after_verification is not None and after_verification.terminal and self._verification_timer is None:
            self._verification_timer = threading.Timer(
                max(0.0, after_verification.deadline - time.monotonic()),
                self._expire_alternative_preview, args=(after_verification,),
            )
            self._verification_timer.daemon = True
            self._verification_timer.start()
        self.planner_status.publish("CHECKING_OBSERVATION_PATH")
        threading.Thread(
            target=self._select_observation_goal, args=(request, preview),
            daemon=True, name="semantic-observation",
        ).start()

    def _observation_request_is_current(self, request: ObservationRequest) -> bool:
        if request.verification_task_id:
            state = self._verification
            return (
                self._pending_observation is request and state is not None
                and self._symbolic_llm_epoch_is_current(request.instruction_epoch)
                and state.task_id == request.verification_task_id
                and time.monotonic() < state.deadline
                and ((state.state == "repositioning" and state.target.object_id == request.target.object_id)
                     or (state.state in {"mismatch", "uncertain"} and state.target.object_id != request.target.object_id))
            )
        return (
            self._pending_observation is request
            and self._symbolic_llm_epoch_is_current(request.instruction_epoch)
            and self._current_instruction == request.instruction
        )

    def _select_observation_goal(self, request: ObservationRequest, preview: Any) -> None:
        """Check bounded hypotheses through the registered native read-only RPC."""
        try:
            state = self._verification if request.verification_task_id else None
            changing_view = state is not None and state.target.object_id == request.target.object_id
            candidates = observation_candidates(request.target, request.robot_pose, self._approach_dist)
            for pose in candidates:
                if not self._observation_request_is_current(request):
                    return
                if changing_view and any(math.hypot(pose.x - old[0], pose.y - old[1]) < 0.30
                                         for old in [*state.viewpoints, (request.robot_pose.x, request.robot_pose.y)]):
                    continue
                result = preview(pose.x, pose.y, pose.z)
                if not self._observation_request_is_current(request):
                    return
                if result.get("reason") in {"navigation_busy", "planner_busy", "odometry_not_ready", "map_odom_tf_not_ready"}:
                    if state is not None and not changing_view:
                        self._object_candidates[request.target.object_id] = "waiting_path"
                    self.planner_status.publish("OBSERVATION_PLANNER_WAITING")
                    return
                if result.get("feasible") is not True or result.get("start_valid") is not True or result.get("frame_id") != "map" or not result.get("path"):
                    continue
                target = self._observed_target(request.target.object_id)
                if target is None or math.dist(target.position, request.target.position) > self._goal_republish_position_epsilon_m:
                    self.planner_status.publish("WAITING_FOR_FRESH_TARGET")
                    return
                robot = self._current_robot_pose
                if robot is None or self._map_sample_is_stale(robot):
                    self.planner_status.publish("WAITING_FOR_MAP_POSE")
                    return
                pose.ts = time.time()
                # Native failure feedback may synchronously start a fresh retry.
                self._pending_observation = None
                self._observation_retry_after = 0.0
                self._object_candidates[target.object_id] = "approaching"
                if changing_view:
                    self._current_goal_pose = pose
                    self._dispatch_navigation_goal(pose, instruction=request.instruction, purpose="object",
                                                   target=target, verification=state)
                    return
                dispatched = self._publish_goal_pose_once(request.instruction, pose, target=target)
                if dispatched:
                    self._chat("assistant", f"Approaching an observation position for {target.label}.", phase="observation")
                return
            self._pending_observation = None
            self._object_candidates[request.target.object_id] = "path_blocked"
            if changing_view:
                self._finish_target_verification(state, "uncertain", "no_reachable_untried_viewpoint")
            elif state is not None:
                self.planner_status.publish(state.status)
                self._continue_object_search(state)
            else:
                self._observation_retry_after = time.monotonic() + 1.0
                self.planner_status.publish("OBSERVATION_PATH_BLOCKED")
        except Exception:
            logger.exception("Native observation-goal preview failed")
            if self._observation_request_is_current(request):
                self.planner_status.publish("OBSERVATION_PLANNER_UNAVAILABLE")
        finally:
            if self._pending_observation is request:
                self._pending_observation = None
                # Do not run eight previews again on every perception frame.
                self._observation_retry_after = time.monotonic() + 1.0

    # Vector Memory Search

    def _vector_memory_allows_navigation(self, result: dict[str, Any]) -> bool:
        if result.get("navigable") is not True:
            return False
        if result.get("degraded") is not False:
            return False
        if result.get("semantic_encoder_ready") is not True:
            return False
        best = result.get("best") or {}
        context = self._navigation_context
        if (
            context is None or not context[1] or context[2] <= 0
            or best.get("frame_id") != SEMANTIC_PLANNER_MAP_FRAME_ID
            or best.get("map_id") != context[1]
            or best.get("map_content_epoch") != context[2]
            or best.get("navigable") is not True
        ):
            return False

        stats_fn = getattr(self._backends.vector_memory, "get_memory_stats", None)
        if not callable(stats_fn):
            return False

        try:
            raw = stats_fn()
            # get_memory_stats is a @skill method returning JSON string
            stats = _json.loads(raw) if isinstance(raw, str) else raw
        except Exception as exc:
            logger.debug("Vector memory stats unavailable: %s", exc)
            return False

        if stats.get("degraded") is not False:
            return False
        if stats.get("semantic_encoder_ready") is not True:
            return False
        return True

    def _try_vector_memory(self, instruction: str) -> bool:
        """Query VectorMemoryModule for fuzzy location match. Returns True if navigating."""
        self._last_vector_memory_query_only = False
        if self._backends is None or self._backends.vector_memory is None:
            return False
        try:
            raw = self._backends.vector_memory.query_location(instruction)
            # query_location is a @skill method returning JSON string
            result = _json.loads(raw) if isinstance(raw, str) else raw
            if not result.get("found"):
                return False
            if not self._vector_memory_allows_navigation(result):
                logger.info(
                    "Vector memory hit lacks a usable encoder or current map binding: %s",
                    result.get("encoder_type", "unknown"),
                )
                self._last_vector_memory_query_only = True
                self.planner_status.publish("VECTOR_MEMORY_QUERY_ONLY")
                self._chat(
                    "thinking",
                    "Vector memory hit is query-only; its encoder or map binding is not ready for navigation.",
                    phase="vector",
                )
                return False
            best = result["best"]
            if best.get("score", 0) < 0.3:
                return False
            pose = PoseStamped(
                pose=Pose(
                    position=Vector3(
                        x=float(best["x"]),
                        y=float(best["y"]),
                        z=float(best.get("z", 0.0)),
                    ),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id=SEMANTIC_PLANNER_MAP_FRAME_ID,
                ts=time.time(),
            )
            if self._publish_goal_pose_once(instruction, pose, purpose="memory"):
                self._chat(
                    "assistant",
                    f"Vector memory hit: ({best['x']:.2f}, {best['y']:.2f}) score={best.get('score', 0):.2f}",
                    phase="vector",
                )
            logger.info(
                "Vector memory hit: '%s' ->(%.1f, %.1f) score=%.2f", instruction, best["x"], best["y"], best["score"]
            )
            return True
        except Exception as e:
            logger.debug("Vector memory query failed: %s", e)
            return False

    # Frontier Exploration

    def _explore_frontier(self, instruction: str) -> None:
        if self._frontier_scorer is None:
            self._fallback_visual_servo(instruction)
            return
        try:
            best = self._frontier_scorer.get_best_frontier()
            if best is not None:
                pos = best.center_world
                pose = PoseStamped(
                    pose=Pose(
                        position=Vector3(float(pos[0]), float(pos[1]), 0.0),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id=SEMANTIC_PLANNER_MAP_FRAME_ID,
                    ts=time.time(),
                )
                if self._publish_goal_pose_once(instruction, pose, purpose="frontier"):
                    self._frontier_count += 1
                    self._chat(
                        "assistant",
                        f"Exploring frontier near ({pos[0]:.2f}, {pos[1]:.2f})",
                        phase="frontier",
                    )
            else:
                self._chat("assistant", "No frontier found; falling back to visual servo", phase="visual_servo")
                self._fallback_visual_servo(instruction)
        except Exception:
            logger.exception("Frontier exploration failed")
            self._fallback_visual_servo(instruction)

    # Multi-turn Agent Loop (moved to AgentPlannerModule)

    # MCP @skill

    def instruction_revision(self) -> int:
        return self._symbolic_llm_instruction_epoch

    def submit_owned_instruction(self, text: str, owner_id: str, *, expected_revision: int | None = None) -> dict:
        """Submit an Agent subgoal without exposing cancellation of other owners."""
        if not owner_id or not self._nav_goal_service_available:
            return {"owned": False, "terminal": True, "success": False, "state": "NAVIGATION_UNAVAILABLE"}
        with self._instruction_lock:
            if (expected_revision is not None and self._instruction_owner != owner_id
                    and self.instruction_revision() != expected_revision):
                return {"owned": False, "terminal": True, "success": False, "state": "SUPERSEDED"}
            self._on_instruction(text, owner_id=owner_id)
            return self.owned_instruction_status(owner_id)

    def submit_owned_pose(self, pose: PoseStamped, instruction: str, owner_id: str,
                          *, expected_revision: int | None = None) -> dict:
        if not owner_id or not self._nav_goal_service_available:
            return {"owned": False, "terminal": True, "success": False, "state": "NAVIGATION_UNAVAILABLE"}
        if pose.frame_id != SEMANTIC_PLANNER_MAP_FRAME_ID or not all(
            math.isfinite(value) for value in (pose.x, pose.y, pose.z, pose.yaw)
        ):
            return {"owned": False, "terminal": True, "success": False, "state": "INVALID_MAP_GOAL"}
        with self._instruction_lock:
            if (expected_revision is not None and self._instruction_owner != owner_id
                    and self.instruction_revision() != expected_revision):
                return {"owned": False, "terminal": True, "success": False, "state": "SUPERSEDED"}
            self._replace_instruction(owner_id)
            self._dispatch_navigation_goal(pose, instruction=instruction, purpose="place")
            return self.owned_instruction_status(owner_id)

    def cancel_owned_instruction(self, owner_id: str, reason: str = "agent_cancelled") -> bool:
        """Request scoped cancellation; True is not confirmation that motion stopped."""
        with self._instruction_lock:
            if not owner_id or self._instruction_owner != owner_id:
                return False
            self._begin_symbolic_llm_instruction_epoch()
            self._current_instruction = ""
            self._current_goal_pose = None
            self._cancel_owned_goals(reason)
            self._stop_visual_handoff()
            self.planner_status.publish("CANCELLING" if self._navigation_goals else "CANCELLED")
            return True

    def owned_instruction_status(self, owner_id: str) -> dict:
        """Separate command ownership, native arrival and visual task success."""
        with self._instruction_lock:
            if not owner_id or self._instruction_owner != owner_id:
                return {"owned": False, "terminal": True, "success": False, "state": "SUPERSEDED"}
            if self._instruction_failure:
                return {"owned": True, "terminal": True, "success": False, "state": self._instruction_failure}
            task, verification = self._active_goal, self._verification
            result = {"owned": True, "terminal": False, "success": False,
                      "state": "FOLLOW" if self._visual_handoff else "RESOLVING"}
            if task is not None:
                result.update(task_id=task.task_id, request_id=task.request_id)
            if self._pending_observation is not None or "waiting_path" in self._object_candidates.values():
                result["state"] = "CHECKING_OBSERVATION_PATH"
            elif verification is not None:
                result.update(state=verification.status, terminal=verification.terminal,
                              success=verification.state == "confirmed", confirmations=verification.confirmations)
            elif task is not None:
                result.update(state=task.state, task_id=task.task_id, request_id=task.request_id)
                if task.state in {"rejected", "unconfirmed"} or (task.terminal and task.purpose == "place"):
                    result.update(terminal=True, success=task.state == "reached")
            return result

    @skill
    def send_instruction(self, text: str) -> str:
        """Send a natural language navigation instruction to the semantic planner.

        Args:
            text: Instruction in natural language, e.g. "go to the kitchen"
        """
        self._on_instruction(text)
        return _json.dumps({"status": "sent", "instruction": text})

    @skill
    def get_planner_status(self) -> str:
        """Return current semantic planner state and counters."""
        state = self._last_nav_state or (f"NAVIGATION_{self._active_goal.state.upper()}" if self._active_goal else "IDLE")
        if self._verification is not None:
            state = self._verification.status
        if "waiting_path" in self._object_candidates.values():
            state = "OBSERVATION_PLANNER_WAITING"
        if self._pending_observation is not None:
            state = "CHECKING_OBSERVATION_PATH"
        return _json.dumps(
            {
                "state": state,
                "object_candidates": dict(self._object_candidates),
                "object_verifications": dict(self._object_verifications),
                "current_instruction": self._current_instruction[:80] if self._current_instruction else "",
                "navigation_goal": self._active_goal.to_dict() if self._active_goal is not None else None,
                "resolve_count": self._resolve_count,
                "frontier_explores": self._frontier_count,
                "lera_triggers": self._lera_count,
                "failure_count": self._failure_count,
                "position": [
                    round(float(self._robot_pos[0]), 3),
                    round(float(self._robot_pos[1]), 3),
                    round(float(self._robot_pos[2]), 3),
                ],
            }
        )

    @skill
    def get_scene_objects(self) -> str:
        """Return the latest scene graph objects for facade/API clients."""
        sg = self._current_scene_graph
        if sg is None or not getattr(sg, "objects", None):
            return _json.dumps([])
        objects: list[dict[str, Any]] = []
        for obj in sg.objects:
            pos = getattr(obj, "position", None)
            objects.append(
                {
                    "label": getattr(obj, "label", "") or "",
                    "confidence": float(getattr(obj, "confidence", 0.0)),
                    "position": [
                        float(getattr(pos, "x", 0.0)),
                        float(getattr(pos, "y", 0.0)),
                        float(getattr(pos, "z", 0.0)),
                    ]
                    if pos is not None
                    else [0.0, 0.0, 0.0],
                }
            )
        return _json.dumps(objects)

    @skill
    def decompose_task(self, instruction: str) -> str:
        """Decompose a complex instruction into ordered sub-goals.

        Args:
            instruction: High-level task description, e.g. "fetch coffee from the kitchen"
        """
        if self._task_decomposer is None:
            return _json.dumps({"error": "task decomposer not loaded"})
        try:
            subtasks = self._task_decomposer.decompose(instruction)
            return _json.dumps(
                {
                    "instruction": instruction,
                    "subtasks": subtasks if isinstance(subtasks, list) else list(subtasks),
                }
            )
        except Exception as exc:
            return _json.dumps({"error": str(exc)})

    # Visual Servo Fallback

    def _fallback_visual_servo(self, instruction: str) -> None:
        """Last resort: trigger VisualServoModule to find the target visually."""
        self._cancel_owned_goals("semantic_visual_handoff")
        self._current_instruction = ""
        self._current_goal_pose = None
        self._active_goal = None
        self._visual_handoff = True
        self.servo_target.publish(f"find:{instruction}")
        self.planner_status.publish("VISUAL_SERVO")
        logger.info("Semantic planner: fallback to visual servo for '%s'", instruction)

    def _stop_visual_handoff(self) -> None:
        if self._visual_handoff:
            self._visual_handoff = False
            self.servo_target.publish("stop")

    # Health

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        backends = {
            "goal_resolver": self._goal_resolver is not None,
            "frontier_scorer": self._frontier_scorer is not None,
            "task_decomposer": self._task_decomposer is not None,
            "action_executor": self._action_executor is not None,
        }
        missing = [name for name, ready in backends.items() if not ready]
        degraded = self._backend_init_attempted and bool(missing)
        degraded_reason = "; ".join(f"{name}: {self._backend_errors.get(name, 'unavailable')}" for name in missing)
        info["degraded"] = degraded
        info["degraded_reason"] = degraded_reason
        info["semantic_planner"] = {
            "decomposer": self._decomposer_strategy,
            "resolver": self._goal_resolver is not None,
            "frontier": self._frontier_scorer is not None,
            "executor": self._action_executor is not None,
            "backends": backends,
            "backend_init_attempted": self._backend_init_attempted,
            "backend_errors": dict(self._backend_errors),
            "degraded": degraded,
            "degraded_reason": degraded_reason,
            "resolves": self._resolve_count,
            "frontier_explores": self._frontier_count,
            "lera_triggers": self._lera_count,
            "failure_count": self._failure_count,
            "last_nav_state": self._last_nav_state,
            "current_instruction": self._current_instruction[:40] if self._current_instruction else "",
        }

        # Merge decision-layer metrics from GoalResolver.
        resolver = self._goal_resolver
        if resolver is not None and hasattr(resolver, "get_metrics"):
            info["semantic_planner"].update(resolver.get_metrics())
        info["semantic_planner"]["lera_recoveries"] = self._lera_recoveries

        return info
