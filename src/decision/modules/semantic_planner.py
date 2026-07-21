"""SemanticPlannerModule - unified semantic planning in one Module.

Replaces 4 separate modules (GoalResolver, Frontier, TaskDecomposer, ActionExecutor).
Internal strategies handle different algorithms.

The multi-turn agent loop has been extracted to AgentPlannerModule.
This module focuses on single-shot instruction processing.

Pipeline:
  instruction -> decompose -> resolve goal -> explore frontiers -> execute action
  nav.mission.mission_status (RECOVERING/FAILED) -> LERa recovery -> new goal

Ports:
  In:  instruction, scene_graph, odometry,
       detections, mission_status, topo_summary, room_graph
  Out: goal_pose, task_plan, planner_status, cancel, servo_target, agent_message

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
from typing import Any

from decision.backends import BackendManager
from decision.modules.llm import LLMRequest, LLMResponse
from decision.semantic_navigation.intent import HybridSemanticIntentParser, SemanticAction, SemanticIntent, TravelMode
from decision.semantic_navigation.intent import normalize_floor_id as normalize_semantic_floor_id
from maps.places import PlaceCatalog, PlaceCatalogError, PlaceRef
from runtime.module import Module, skill
from runtime.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from runtime.msgs.nav import Odometry
from runtime.msgs.numpy_compat import np
from runtime.msgs.semantic import SceneGraph
from runtime.registry import register
from runtime.runtime_interface import map_frame_id
from runtime.stream import In, Out

logger = logging.getLogger(__name__)

# Minimum seconds between consecutive LERa triggers.
_LERA_COOLDOWN = 15.0
SEMANTIC_PLANNER_MAP_FRAME_ID = map_frame_id()


@register("semantic_planner", "default", description="Unified semantic planner module")
class SemanticPlannerModule(Module, layer=4):
    """Unified semantic planner: decompose ->resolve ->explore ->execute.

    Internally composes GoalResolver, FrontierScorer, TaskDecomposer,
    ActionExecutor. Each is a strategy, not a separate Module.

    LERa integration
    ----------------
    Subscribes to nav.mission.mission_status. On RECOVERING or FAILED, calls
    ActionExecutor.lera_recover() and dispatches one of four strategies:
      retry_different_path -republish current goal (Navigation replans)
      expand_search        -ask FrontierScorer for an alternative frontier
      requery_goal         -re-run Fast->Slow goal resolution from scratch
      abort                -publish "lera_abort" to cancel port
    """

    _run_in_worker = True
    _worker_group = "semantic"
    SOFT_DEPENDS = ["VectorMemoryModule", "SemanticMapperModule", "LLMModule"]

    # -- Inputs --
    instruction: In[str]  # single-shot resolve (Fast->Frontier->VisualServo)
    scene_graph: In[SceneGraph]
    odometry: In[Odometry]
    detections: In[list]
    mission_status: In[dict]  # from Navigation -drives LERa recovery
    topo_summary: In[str]  # from SemanticMapperModule
    room_graph: In[dict]  # serialized TopologySemGraph snapshot
    llm_response: In[LLMResponse]  # symbolic semantic-intent slow path

    # -- Outputs --
    goal_pose: Out[PoseStamped]
    task_plan: Out[dict]
    planner_status: Out[str]
    cancel: Out[str]  # "lera_abort" ->Navigation.cancel
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
        lera_cooldown: float = _LERA_COOLDOWN,
        llm_backend: str = "kimi",
        llm_model: str = "",
        save_dir: str = "",
        **kw,
    ):
        super().__init__(**kw)
        self._decomposer_strategy = decomposer
        self._fast_threshold = fast_path_threshold
        self._save_dir = save_dir
        self._frontier_threshold = frontier_score_threshold
        self._max_frontiers = max_frontiers
        self._approach_dist = approach_distance
        self._lera_cooldown = lera_cooldown
        self._llm_backend = llm_backend
        self._llm_model = llm_model

        # Backends (lazy init in setup)
        self._goal_resolver = None
        self._frontier_scorer = None
        self._task_decomposer = None
        self._action_executor = None
        self._backend_init_attempted = False
        self._backend_errors: dict[str, str] = {}

        # Odometry / position
        self._robot_pos = [0.0, 0.0, 0.0]

        # Scene graph -keep both JSON string (for GoalResolver) and
        # the original object (for LERa label extraction).
        self._latest_sg: str | None = None
        self._current_scene_graph: SceneGraph | None = None

        # Active instruction + resolved goal
        self._current_instruction: str = ""
        self._current_goal_pose: PoseStamped | None = None
        self._last_goal_publish_signature: tuple[str, str, float, float, float] | None = None

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
        self._place_catalog: PlaceCatalog | None = None
        self._maps_service: Any | None = None
        self._nav_goal_service_available = False
        self._nav_building_service_available = False
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
        self._nav_building_service_available = self._backends.get("nav.building") is not None
        self._maps_service = self._backends.get("maps.service") or self._backends.get("MapsModule")
        self._place_catalog = None
        if self._maps_service is None:
            return
        try:
            # Prefer the dict-returning API service; the MapsModule skill
            # compatibility methods intentionally return JSON strings.
            catalog_source = getattr(self._maps_service, "api", None) or self._maps_service
            self._place_catalog = PlaceCatalog(catalog_source)
        except Exception as exc:
            logger.warning("Semantic place catalog unavailable: %s", exc)
            self._place_catalog = None

    def setup(self) -> None:
        self._init_backends()
        self.instruction.subscribe(self._on_instruction)
        self.scene_graph.subscribe(self._on_scene_graph)
        self.odometry.subscribe(self._on_odom)
        self.detections.subscribe(self._on_detections)
        self.mission_status.subscribe(self._on_mission_status)
        self.topo_summary.subscribe(self._on_topo_summary)
        self.room_graph.subscribe(self._on_room_graph)
        self.llm_response.subscribe(self._on_llm_response)

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

    def _on_instruction(self, text: str) -> None:
        """New instruction ->decompose ->resolve (or hand off person-following)."""
        self._begin_symbolic_llm_instruction_epoch()
        # Person-following intent ->VisualServo follow mode (bypass goal resolve).
        follow_target = self._detect_follow_intent(text)
        if follow_target is not None:
            self.servo_target.publish(f"follow:{follow_target}")
            self.planner_status.publish("FOLLOW")
            self._chat("assistant", f"Following target: {follow_target}", phase="follow")
            logger.info("Semantic planner: follow intent ->'%s'", follow_target)
            return

        self._current_instruction = text
        self._last_goal_publish_signature = None
        with self._lera_lock:
            self._failure_count = 0
            self._requery_count = 0
            self._last_nav_state = ""
        self.planner_status.publish("PROCESSING")
        self._chat("thinking", "Parsing instruction", phase="parse")

        if self._try_semantic_navigation_intent(text):
            return

        self._continue_legacy_instruction(text)

    def _continue_legacy_instruction(self, text: str) -> None:
        """Run the pre-existing scene-graph/vector/frontier path for ``text``."""

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
        refused.  Returns False when legacy scene-graph/vector/frontier behavior
        should continue.
        """

        try:
            intent = self._semantic_intent_parser.parse(text)
        except Exception as exc:
            logger.warning("Semantic intent parsing rejected instruction: %s", exc)
            self.planner_status.publish("SEMANTIC_INTENT_REJECTED")
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
        with self._symbolic_llm_lock:
            self._symbolic_llm_instruction_epoch += 1
            self._symbolic_llm_current_request_id = ""
            self._symbolic_llm_pending.clear()

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
            self.planner_status.publish("SYMBOLIC_LLM_FAILED")
            self._chat("assistant", "I could not safely interpret that navigation command.", phase="semantic_llm")
            return
        try:
            payload = self._parse_symbolic_llm_json(getattr(resp, "text", ""))
            if payload is None:
                self.planner_status.publish("SYMBOLIC_LLM_NO_INTENT")
                self._chat(
                    "assistant",
                    "I could not map that sentence to a supported navigation command.",
                    phase="semantic_llm",
                )
                return
            intent = HybridSemanticIntentParser.from_symbolic_mapping(payload, raw_text=raw_text)
        except Exception as exc:
            logger.warning("Symbolic semantic LLM response rejected: %s", exc)
            self.planner_status.publish("SYMBOLIC_LLM_REJECTED")
            self._chat("assistant", "I could not safely interpret that navigation command.", phase="semantic_llm")
            return

        if not self._symbolic_llm_epoch_is_current(epoch):
            return
        if intent.needs_clarification:
            self.planner_status.publish("PLACE_CLARIFICATION_REQUIRED")
            self._chat("assistant", "Please name the place you want to go to.", phase="semantic_llm")
            return
        if intent.action is not SemanticAction.NAVIGATE:
            self._handle_tour_intent(intent, expected_symbolic_epoch=epoch)
            return
        if not self._handle_place_navigation_intent(intent, expected_symbolic_epoch=epoch):
            if not self._symbolic_llm_epoch_is_current(epoch):
                return
            self._chat("thinking", "Symbolic intent was not grounded; trying legacy navigation.", phase="semantic_llm")
            self._continue_legacy_instruction(raw_text)

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
            self.planner_status.publish("TOUR_COMMAND_UNAVAILABLE")
            self._chat(
                "assistant",
                "Tour control is recognized, but the navigation command service is unavailable.",
                phase="tour",
            )
            return

        action_payload: dict[SemanticAction, dict[str, str]] = {
            SemanticAction.START_TOUR: {
                "action": "inspection",
                "route_id": intent.tour_id,
            },
            SemanticAction.PAUSE_TOUR: {
                "action": "inspection_pause",
                "reason": "semantic_pause",
            },
            SemanticAction.RESUME_TOUR: {
                "action": "inspection_resume",
                "reason": "semantic_resume",
            },
            SemanticAction.CANCEL_TOUR: {
                "action": "inspection_cancel",
                "reason": "semantic_cancel",
            },
        }
        payload = action_payload.get(intent.action)
        if payload is None:
            self.planner_status.publish("TOUR_COMMAND_REJECTED")
            return
        if expected_symbolic_epoch is not None and not self._symbolic_llm_epoch_is_current(expected_symbolic_epoch):
            return
        self.nav_command.publish(_json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        self.planner_status.publish("TOUR_COMMAND_DISPATCHED")
        self._chat(
            "assistant",
            "Tour command sent to the native inspection service.",
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
            self.planner_status.publish("PLACE_CLARIFICATION_REQUIRED")
            self._chat("assistant", "Please name the place you want to go to.", phase="place")
            return True
        if self._place_catalog is None:
            if explicit_place_route:
                self._refuse_place_navigation("PLACE_CATALOG_UNAVAILABLE", "Place map is unavailable.")
                return True
            return False

        active_map = self._active_map_id()
        if not active_map:
            if explicit_place_route:
                self._refuse_place_navigation("ACTIVE_MAP_REQUIRED", "No active map is selected.")
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
                self._refuse_place_navigation("PLACE_LOOKUP_FAILED", "Place lookup failed.")
                return True
            return False
        except Exception as exc:
            logger.warning("Semantic place lookup unavailable: %s", exc)
            if explicit_place_route:
                self._refuse_place_navigation("PLACE_LOOKUP_UNAVAILABLE", "Place lookup is unavailable.")
                return True
            return False

        if resolution.status != "resolved" or resolution.place is None:
            if explicit_place_route or resolution.status in {"ambiguous", "stale_map"}:
                self._refuse_place_navigation(
                    f"PLACE_{resolution.status.upper()}",
                    self._place_refusal_text(resolution.status),
                )
                return True
            return False

        place = resolution.place
        if not (self._nav_building_service_available and self._nav_goal_service_available):
            self._refuse_place_navigation(
                "CONNECTOR_RUNTIME_REQUIRED",
                "Named-place navigation requires the building mission runtime.",
            )
            return True
        if not self._building_place_is_executable(place):
            self._refuse_place_navigation(
                "PLACE_NOT_EXECUTABLE",
                f"Place is not executable: {place.non_executable_reason or 'incomplete building binding'}.",
            )
            return True
        self._publish_building_navigation(intent, place, expected_symbolic_epoch=expected_symbolic_epoch)
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

    @classmethod
    def _building_place_is_executable(cls, place: PlaceRef) -> bool:
        return (
            cls._place_is_executable(place)
            and place.yaw is not None
            and place.map_version is not None
            and bool(place.place_id)
            and bool(place.name)
            and bool(place.building_id)
            and bool(place.floor_id)
            and bool(place.map_id)
            and bool(place.version_id)
            and bool(place.map_pcd_sha256)
        )

    def _publish_building_navigation(
        self,
        intent: SemanticIntent,
        place: PlaceRef,
        *,
        expected_symbolic_epoch: int | None = None,
    ) -> None:
        if expected_symbolic_epoch is not None and not self._symbolic_llm_epoch_is_current(expected_symbolic_epoch):
            return
        payload = {
            "action": "building_navigate",
            "schema_version": "lingtu.building_goal.v1",
            "request_id": f"semantic-{time.time_ns()}",
            "source": "semantic",
            "travel_mode": intent.travel_mode.value,
            "connector_id": place.connector_id,
            "target": {
                "place_id": place.place_id,
                "name": place.name,
                "building_id": place.building_id,
                "floor_id": place.floor_id,
                "map_id": place.map_id,
                "frame_id": place.frame_id,
                "x": float(place.x),
                "y": float(place.y),
                "z": float(place.z),
                "yaw": float(place.yaw),
                "map_version": place.map_version,
                "version_id": place.version_id,
                "map_pcd_sha256": place.map_pcd_sha256,
            },
        }
        self.nav_command.publish(_json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
        self.planner_status.publish("BUILDING_MISSION_DISPATCHED")
        self._chat(
            "assistant",
            f"Building navigation mission sent for {place.name} on {place.floor_id}.",
            phase="place",
        )

    def _refuse_place_navigation(self, status: str, text: str) -> None:
        self.planner_status.publish(status)
        self._chat("assistant", text, phase="place")

    @staticmethod
    def _place_refusal_text(status: str) -> str:
        if status == "ambiguous":
            return "More than one matching place was found; I will not guess."
        if status == "stale_map":
            return "The matching place is not bound to the current map version."
        return "I could not find that place in the active place map."

    def _active_map_id(self) -> str:
        service = self._maps_service
        if service is None:
            return ""
        api = getattr(service, "api", None)
        owner = api if api is not None else service
        method = getattr(owner, "get_active_map", None)
        if callable(method):
            try:
                response = method()
            except Exception:
                logger.debug("active map query failed", exc_info=True)
            else:
                if isinstance(response, dict) and response.get("success") is True:
                    return str(response.get("active") or response.get("map_id") or "")
        for attr in ("_active_map", "active_map"):
            value = getattr(service, attr, "")
            if value:
                return str(value)
        storage = getattr(service, "storage", None)
        value = getattr(storage, "active_map", "") if storage is not None else ""
        if value:
            return str(value)
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
        target = m.group(2).strip(" \t\r\n.!,.;:")
        return target or "person"

    def _on_scene_graph(self, sg: SceneGraph) -> None:
        """Scene graph update ->cache + re-resolve if active instruction."""
        sg_json = sg.to_json() if hasattr(sg, "to_json") else str(sg)
        self._latest_sg = sg_json
        self._current_scene_graph = sg  # keep object for LERa label extraction

        if self._current_instruction and self._goal_resolver and not self._has_symbolic_llm_pending():
            self._try_resolve(self._current_instruction, sg_json)

    def _on_odom(self, odom: Odometry) -> None:
        self._robot_pos = np.array([odom.x, odom.y, getattr(odom, "z", 0.0)])

    @staticmethod
    def _rounded_goal_coord(value: float) -> float:
        value = float(value)
        return round(value, 3) if math.isfinite(value) else value

    def _publish_goal_pose_once(self, instruction: str, pose: PoseStamped) -> bool:
        signature = (
            instruction,
            str(pose.frame_id or ""),
            self._rounded_goal_coord(pose.x),
            self._rounded_goal_coord(pose.y),
            self._rounded_goal_coord(pose.z),
        )
        self._current_goal_pose = pose
        if signature == self._last_goal_publish_signature:
            return False
        self._last_goal_publish_signature = signature
        self.goal_pose.publish(pose)
        return True

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

    def _on_mission_status(self, status: dict) -> None:
        """Navigation failure ->trigger LERa recovery.

        This method runs on the caller's callback thread (synchronous publish
        chain). It must return immediately -the actual LERa call (which may
        block up to 15 s on a network LLM) is dispatched to a daemon thread.
        """
        state = status.get("state", "")

        # Fast path: non-terminal states only update cached nav state.
        if state not in ("RECOVERING", "STUCK", "FAILED"):
            with self._lera_lock:
                self._last_nav_state = state
            return

        # All checks and mutations under lock to avoid races with _run_lera.
        with self._lera_lock:
            # Ignore repeated publishes of the same state.
            if state == self._last_nav_state:
                return
            self._last_nav_state = state

            # Cooldown -prevent storm if Navigation bounces quickly.
            now = time.time()
            if now - self._last_lera_time < self._lera_cooldown:
                logger.debug("[LERa] cooldown active (%.1fs left)", self._lera_cooldown - (now - self._last_lera_time))
                return

            # Prevent concurrent LERa calls -only one in-flight at a time.
            if self._lera_running:
                logger.debug("[LERa] already running, skipping duplicate trigger")
                return

            self._last_lera_time = now
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
            llm_client = getattr(self._goal_resolver, "_llm", None) if self._goal_resolver else None

        logger.info(
            "[LERa] Triggered: nav_state=%s failure#%d instruction='%s'", state, failure_count, instruction[:40]
        )

        # Publish RECOVERING synchronously before thread launch so the UI
        # reflects the state change without waiting for the LLM call.
        self.planner_status.publish("RECOVERING")

        # Dispatch Explain+Replan to a daemon thread -never block odom chain.
        threading.Thread(
            target=self._run_lera,
            args=(instruction, labels, failure_count, llm_client),
            daemon=True,
            name="lera-recovery",
        ).start()

    def _run_lera(
        self,
        instruction: str,
        labels: list[str],
        failure_count: int,
        llm_client: Any | None,
    ) -> None:
        """Background thread: Explain+Replan, then dispatch recovery action.

        Out.publish() is thread-safe (Lock-protected), so dispatching from
        here is safe.
        """
        try:
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

            logger.info("[LERa] Strategy: %s (failure#%d)", strategy, failure_count)
            self._lera_recoveries += 1
            self._dispatch_recovery(strategy)
        except Exception:
            logger.exception("[LERa] Unexpected error in recovery thread")
            self.planner_status.publish("FAILED")
        finally:
            with self._lera_lock:
                self._lera_running = False

    # LERa recovery dispatch

    def _dispatch_recovery(self, strategy: str) -> None:
        """Map LERa strategy string to port actions."""
        if strategy == "retry_different_path":
            # Re-send the same semantic goal -Navigation will cancel
            # the current path and replan from scratch.
            if self._current_goal_pose is not None:
                self.goal_pose.publish(self._current_goal_pose)
                self.planner_status.publish("RETRYING")
            else:
                # No goal cached yet -fall through to frontier exploration.
                self._explore_frontier(self._current_instruction)

        elif strategy == "expand_search":
            # Ask FrontierScorer for an unexplored vantage point.
            self._explore_frontier(self._current_instruction)

        elif strategy == "requery_goal":
            with self._lera_lock:
                self._requery_count += 1
                requery_count = self._requery_count
            if requery_count > 2:
                # LLM kept suggesting requery -force abort to avoid infinite loop.
                logger.warning("[LERa] requery_goal capped (%d), forcing abort", requery_count)
                self.cancel.publish("lera_abort")
                self.planner_status.publish("ABORTED")
                with self._lera_lock:
                    self._failure_count = 0
                    self._requery_count = 0
                self._current_instruction = ""
                self._current_goal_pose = None
                self._last_goal_publish_signature = None
            else:
                # Re-run full Fast->Slow resolution with the current scene graph.
                with self._lera_lock:
                    self._failure_count = 0
                if self._latest_sg and self._current_instruction:
                    self._try_resolve(self._current_instruction, self._latest_sg)
                else:
                    self.planner_status.publish("FAILED")

        elif strategy == "abort":
            self.cancel.publish("lera_abort")
            self.planner_status.publish("ABORTED")
            self._failure_count = 0
            self._current_instruction = ""
            self._current_goal_pose = None
            self._last_goal_publish_signature = None

        else:
            logger.warning("[LERa] Unknown strategy '%s', defaulting to abort", strategy)
            self.cancel.publish("lera_abort")
            self.planner_status.publish("ABORTED")

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
        self._last_vector_memory_query_only = False
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
            result = self._goal_resolver.fast_resolve(instruction, sg_json)
            if result and hasattr(result, "confidence") and result.confidence >= self._fast_threshold:
                self._resolve_count += 1
                pos = self._goal_result_position(result)
                if pos is not None:
                    pose = PoseStamped(
                        pose=Pose(
                            position=Vector3(
                                float(pos[0]),
                                float(pos[1]),
                                float(pos[2]) if len(pos) > 2 else 0.0,
                            ),
                            orientation=Quaternion(0, 0, 0, 1),
                        ),
                        frame_id=(
                            getattr(result, "frame_id", SEMANTIC_PLANNER_MAP_FRAME_ID) or SEMANTIC_PLANNER_MAP_FRAME_ID
                        ),
                        ts=time.time(),
                    )
                    if self._publish_goal_pose_once(instruction, pose):
                        self.planner_status.publish("RESOLVED")
                        self._chat(
                            "assistant",
                            f"Resolved target at ({pos[0]:.2f}, {pos[1]:.2f})",
                            phase="fast_path",
                        )
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

    # Vector Memory Search

    @staticmethod
    def _goal_result_position(result: Any) -> list[float] | None:
        pos = getattr(result, "position", None)
        if isinstance(pos, dict):
            return [
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
                float(pos.get("z", 0.0)),
            ]
        if isinstance(pos, (list, tuple)) and len(pos) >= 2:
            return [
                float(pos[0]),
                float(pos[1]),
                float(pos[2]) if len(pos) > 2 else 0.0,
            ]

        tx = getattr(result, "target_x", None)
        ty = getattr(result, "target_y", None)
        if tx is None or ty is None:
            return None
        tz = getattr(result, "target_z", 0.0)
        return [float(tx), float(ty), float(tz or 0.0)]

    def _vector_memory_allows_navigation(self, result: dict[str, Any]) -> bool:
        if result.get("navigable") is not True:
            return False
        if result.get("degraded") is not False:
            return False
        if result.get("semantic_encoder_ready") is not True:
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
                    "Vector memory hit ignored for navigation because encoder is degraded: %s",
                    result.get("encoder_type", "unknown"),
                )
                self._last_vector_memory_query_only = True
                self.planner_status.publish("VECTOR_MEMORY_QUERY_ONLY")
                self._chat(
                    "thinking",
                    "Vector memory query is degraded; not using it for navigation.",
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
            if self._publish_goal_pose_once(instruction, pose):
                self.planner_status.publish("VECTOR_MEMORY")
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
            if best and hasattr(best, "position"):
                pos = best.position
                pose = PoseStamped(
                    pose=Pose(
                        position=Vector3(float(pos[0]), float(pos[1]), 0.0),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id=SEMANTIC_PLANNER_MAP_FRAME_ID,
                    ts=time.time(),
                )
                if self._publish_goal_pose_once(instruction, pose):
                    self._frontier_count += 1
                    self.planner_status.publish("EXPLORING")
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

    @skill
    def send_instruction(self, text: str) -> str:
        """Send a natural language navigation instruction to the semantic planner.

        Args:
            text: Instruction in natural language, e.g. "go to the kitchen"
        """
        self.instruction.publish(text)
        return _json.dumps({"status": "sent", "instruction": text})

    @skill
    def get_planner_status(self) -> str:
        """Return current semantic planner state and counters."""
        return _json.dumps(
            {
                "state": self._last_nav_state or "IDLE",
                "current_instruction": self._current_instruction[:80] if self._current_instruction else "",
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
        self.servo_target.publish(f"find:{instruction}")
        self.planner_status.publish("VISUAL_SERVO")
        logger.info("Semantic planner: fallback to visual servo for '%s'", instruction)

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
