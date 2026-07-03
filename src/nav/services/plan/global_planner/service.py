"""Map-backed global planner service.

Not a Module. Used internally by Navigation to separate planning
policy orchestration from the mission FSM.
"""
from __future__ import annotations

import logging
import pickle
from typing import Any

from nav.services.plan.contracts import (
    GlobalPlanRequest,
    GlobalPlanResult,
    PlanningMap,
    coerce_planning_map,
)
from nav.services.plan.global_planner.artifact_gate import GlobalPlannerArtifactGateMixin
from nav.services.plan.global_planner.backend_runtime import (
    backend_plan_diagnostics,
    backend_unavailable_reason,
    configure_backend,
    create_planner_backend,
    load_static_occupancy_into_backend,
    normalize_octoplanner3d_constraints,
    plan_backend,
    push_backend_map_update,
)
from nav.services.plan.global_planner.grid_safety import GlobalPlannerGridSafetyMixin
from nav.services.plan.global_planner.postprocess import downsample_path
from nav.services.safety.plan_safety import evaluate_backend_path_safety
from runtime.msgs.numpy_compat import np
from runtime.profiles.planner_backends import normalize_planner_name

logger = logging.getLogger(__name__)


class GlobalPlanner(
    GlobalPlannerArtifactGateMixin,
    GlobalPlannerGridSafetyMixin,
):
    """Orchestrates map gate, safety policy, backend calls, and path output.

        Not a Module -no ports, no lifecycle.
        Swap the backend by passing a different planner_name.
    """

    def __init__(
        self,
        planner_name: str = "octoplanner3d",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        downsample_dist: float = 2.0,
        plan_safety_policy: str = "observe",
        fallback_planner_name: str = "",
        expected_saved_map_frame_id: str | None = None,
        map_artifact_gate_required: bool | None = None,
        octoplanner3d_constraints: dict[str, Any] | None = None,
        octoplanner3d_timeout_s: float | None = None,
    ) -> None:
        self._planner_name = normalize_planner_name(planner_name) or "octoplanner3d"
        self._tomogram = tomogram
        self._obstacle_thr = obstacle_thr
        self._downsample_dist = downsample_dist
        self._plan_safety_policy = plan_safety_policy
        self._fallback_planner_name = normalize_planner_name(fallback_planner_name)
        self._expected_saved_map_frame_id = expected_saved_map_frame_id
        self._map_artifact_gate_required = map_artifact_gate_required
        self._octoplanner3d_timeout_s = octoplanner3d_timeout_s
        self._octoplanner3d_constraints = normalize_octoplanner3d_constraints(
            octoplanner3d_constraints
        )
        self._backend = None
        self._fallback_backend = None
        self._last_plan_report: dict[str, Any] = {}
        self._map_artifact_gate: dict[str, Any] = self._default_map_artifact_gate()
        self._last_map_update: PlanningMap | None = None
        self._warned_no_grid: bool = False

    def setup(self) -> None:
        """Create the planner backend. Must be called before plan()."""
        self._map_artifact_gate = self._validate_map_artifact_gate()
        self._backend = self._create_backend()

    def _create_backend(self, name: str | None = None):
        name = normalize_planner_name(name or self._planner_name)
        map_path = self._backend_constructor_map_path(name)
        backend = create_planner_backend(
            name,
            map_path,
            self._obstacle_thr,
            octoplanner3d_timeout_s=self._octoplanner3d_timeout_s,
        )
        configure_backend(backend, name, self._octoplanner3d_constraints)
        load_static_occupancy_into_backend(backend, self._resolve_occupancy_path(map_path))
        return backend

    def _backend_constructor_map_path(self, name: str) -> str:
        return self._resolve_map_path(normalize_planner_name(name))

    def _resolve_occupancy_path(self, map_path: str = "") -> str:
        return self._saved_map_artifacts().static_occupancy_path(map_path)

    @property
    def planner_name(self) -> str:
        """Return the canonical configured backend name."""
        return self._planner_name

    @property
    def plan_safety_policy(self) -> str:
        """Return the active safety policy for planner output."""
        return self._plan_safety_policy

    @property
    def is_ready(self) -> bool:
        """Return whether a backend has been created."""
        return self._backend is not None

    @property
    def has_map(self) -> bool:
        """Return whether the active backend has a populated 2-D grid."""
        if self._backend is None:
            return False
        grid = getattr(self._backend, "_grid", None)
        return grid is not None and hasattr(grid, "shape") and getattr(grid, "size", 0) > 0

    @property
    def map_artifact_gate(self) -> dict[str, Any]:
        """Return saved-map artifact gate diagnostics."""
        return dict(self._map_artifact_gate)

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        safe_goal_tolerance: float = 4.0,
    ) -> tuple[list[np.ndarray], float]:
        """Plan a path from start to goal -multi-stage with safety fallbacks.

        Algorithm flow:
          1. Input validation     -backend readiness + map artifact gate
          2. Goal safety check    -BFS nearest free cell within safe_goal_tolerance
          3. Primary planning     -call the configured backend
          4. Empty path recovery  -if the primary returned no path, try safe replan
                                    to a nearby reachable cell
          5. Path safety eval     -if non-empty, evaluate against live obstacles;
                                    on failure either replan to a nearby goal
                                    (_try_primary_safe_replan) or take the safe
                                    prefix of the original path (_try_primary_safe_prefix)
          6. Fallback planner     -legacy explicit fallback path, disabled in
                                    product profiles and CLI defaults
          7. Downsample + return  -thin waypoints to minimum spacing, log stats

        If the goal lands on an obstacle, BFS-searches the nearest free cell
        within safe_goal_tolerance meters (reference: dimos _find_safe_goal).

        Returns:
            (path, plan_ms) where path is list of np.ndarray([x, y, z]).
        Raises:
            RuntimeError if backend not ready or planner returns empty path.
        """
        if self._backend is None:
            raise RuntimeError("GlobalPlanner: backend not set up")
        if self._map_artifact_gate_blocks():
            reason = self._map_artifact_gate_failure_reason()
            self._last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "selected_path_safety": None,
                "rejected_plans": [
                    {
                        "planner": self._planner_name,
                        "reason": reason,
                        "artifact_gate": self.map_artifact_gate,
                    }
                ],
                "fallback_reason": reason,
                "policy": self._plan_safety_policy,
                "reached_goal": False,
            }
            raise RuntimeError(f"GlobalPlanner: {reason}")

        # --- Phase 2: Goal safety adjustment (BFS nearest free cell) ---
        requested_goal = np.asarray(goal[:3], dtype=float).copy()

        # Goal safety: if backend exposes a costmap, verify goal is in free space
        goal_grid_checked = self._backend_has_grid(self._backend)
        safe_goal = self._find_safe_goal(
            goal,
            tolerance=safe_goal_tolerance,
            start=start,
        )
        if safe_goal is not None:
            if not np.array_equal(safe_goal[:2], goal[:2]):
                logger.info(
                    "Goal adjusted: (%.1f,%.1f) ->(%.1f,%.1f) (nearest free cell)",
                    goal[0], goal[1], safe_goal[0], safe_goal[1],
                )
            goal = safe_goal
        elif goal_grid_checked:
            reason = (
                "goal has no reachable free cell within "
                f"{float(safe_goal_tolerance):.1f}m"
            )
            self._last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "selected_path_safety": None,
                "rejected_plans": [
                    {
                        "planner": self._planner_name,
                        "reason": reason,
                        "goal": requested_goal.tolist(),
                    }
                ],
                "fallback_reason": reason,
                "policy": self._plan_safety_policy,
                "reached_goal": False,
            }
            raise RuntimeError(f"GlobalPlanner: {reason}")

        # --- Phase 3: Primary planning ---
        raw_path, plan_ms = self._plan_with_backend(self._backend, start, goal)

        selected_backend = self._backend
        selected_planner = self._planner_name
        selected_path = raw_path
        selected_plan_ms = plan_ms
        selected_reached_goal = bool(getattr(selected_backend, "_last_plan_reached_goal", True))
        selected_safety = None
        rejected_plans: list[dict[str, Any]] = []
        primary_replan: dict[str, Any] | None = None
        fallback_reason = ""
        # --- Phase 4: Empty path recovery (replan to nearby safe goal) ---
        if not raw_path:
            backend_plan_error = str(
                getattr(self._backend, "_last_plan_error", "") or ""
            ).strip()
            empty_path_reason = backend_plan_error or "primary planner returned empty path"
            planner_diagnostics = backend_plan_diagnostics(self._backend)
            if self._plan_safety_policy == "reject":
                self._last_plan_report = {
                    "primary_planner": self._planner_name,
                    "selected_planner": self._planner_name,
                    "selected_path_safety": None,
                    "rejected_plans": [
                        {
                            "planner": self._planner_name,
                            "reason": empty_path_reason,
                            "planner_diagnostics": planner_diagnostics,
                        }
                    ],
                    "fallback_reason": empty_path_reason,
                    "policy": self._plan_safety_policy,
                    "reached_goal": False,
                    "planner_diagnostics": planner_diagnostics,
                }
                raise RuntimeError(f"GlobalPlanner: {empty_path_reason}")
            repaired = self._try_primary_safe_replan(
                selected_backend,
                start,
                requested_goal,
                {
                    "ok": False,
                    "reason": "empty_path",
                    "blocked_sample_count": 0,
                },
                tolerance=safe_goal_tolerance,
                reason="initial_primary_empty_path",
                alternate_goals=[goal],
                include_line_back=True,
            )
            if repaired is None:
                primary_rejection = {
                    "planner": self._planner_name,
                    "reason": empty_path_reason,
                    "planner_diagnostics": planner_diagnostics,
                }
                rejected_plans.append(primary_rejection)
                fallback_selection = self._try_fallback_after_primary_failure(
                    start,
                    goal,
                    reason=empty_path_reason,
                    primary_diagnostics=planner_diagnostics,
                    selected_plan_ms=selected_plan_ms,
                    selected_safety=selected_safety,
                    rejected_plans=rejected_plans,
                )
                if fallback_selection is not None:
                    selected_backend = fallback_selection["backend"]
                    selected_planner = fallback_selection["planner"]
                    selected_path = fallback_selection["path"]
                    selected_plan_ms = fallback_selection["plan_ms"]
                    selected_safety = fallback_selection["safety"]
                    selected_reached_goal = fallback_selection["reached_goal"]
                    fallback_reason = empty_path_reason
                    logger.warning(
                        "GlobalPlanner: %s; using fallback planner '%s'",
                        empty_path_reason,
                        selected_planner,
                    )
                    repaired = None
                else:
                    self._last_plan_report = {
                        "primary_planner": self._planner_name,
                        "selected_planner": self._planner_name,
                        "selected_path_safety": None,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": empty_path_reason,
                        "policy": self._plan_safety_policy,
                        "reached_goal": False,
                        "planner_diagnostics": planner_diagnostics,
                    }
                    raise RuntimeError(f"GlobalPlanner: {empty_path_reason}")
            if repaired is not None:
                (
                    repaired_goal,
                    repaired_path,
                    repaired_ms,
                    repaired_safety,
                    repaired_reached_goal,
                    repair_report,
                ) = repaired
                selected_path = repaired_path
                selected_plan_ms += repaired_ms
                selected_safety = repaired_safety
                selected_reached_goal = repaired_reached_goal
                goal = repaired_goal
                primary_replan = repair_report
                logger.warning(
                    "GlobalPlanner: %s returned empty path; replanned with "
                    "%s to reachable nearby goal (%.2f, %.2f, %.2f)",
                    self._planner_name,
                    self._planner_name,
                    repaired_goal[0],
                    repaired_goal[1],
                    repaired_goal[2],
                )
        # --- Phase 5: Path safety evaluation + repair ---
        else:
            selected_safety = self._evaluate_path_safety(selected_backend, selected_path)
        if selected_safety is not None and not selected_safety.get("ok", True):
            fallback_reason = (
                f"{self._planner_name} path_safety failed "
                f"({selected_safety.get('blocked_sample_count', 0)} blocked samples)"
            )
            rejected_plans.append(
                {
                    "planner": self._planner_name,
                    "path_safety": selected_safety,
                    "reason": fallback_reason,
                }
            )
            if self._plan_safety_policy != "reject":
                repaired = self._try_primary_safe_replan(
                    selected_backend,
                    start,
                    goal,
                    selected_safety,
                    tolerance=safe_goal_tolerance,
                    alternate_goals=[requested_goal],
                    include_line_back=True,
                )
                if repaired is not None:
                    (
                        repaired_goal,
                        repaired_path,
                        repaired_ms,
                        repaired_safety,
                        repaired_reached_goal,
                        repair_report,
                    ) = repaired
                    selected_path = repaired_path
                    selected_plan_ms += repaired_ms
                    selected_safety = repaired_safety
                    selected_reached_goal = repaired_reached_goal
                    goal = repaired_goal
                    primary_replan = repair_report
                    fallback_reason = ""
                    rejected_plans.clear()
                    logger.warning(
                        "GlobalPlanner: %s path crossed live obstacles; "
                        "replanned with %s to nearby safe goal (%.2f, %.2f, %.2f)",
                        self._planner_name,
                        self._planner_name,
                        repaired_goal[0],
                        repaired_goal[1],
                        repaired_goal[2],
                    )
                else:
                    prefix_repair = self._try_primary_safe_prefix(
                        selected_backend,
                        selected_path,
                        selected_safety,
                        reason="initial_primary_path_safety_failed",
                    )
                    if prefix_repair is not None:
                        (
                            repaired_goal,
                            repaired_path,
                            repaired_safety,
                            repair_report,
                        ) = prefix_repair
                        selected_path = repaired_path
                        selected_safety = repaired_safety
                        selected_reached_goal = False
                        goal = repaired_goal
                        primary_replan = repair_report
                        fallback_reason = ""
                        rejected_plans.clear()
                        logger.warning(
                            "GlobalPlanner: %s path crossed live obstacles; "
                            "using safe PCT prefix to (%.2f, %.2f, %.2f)",
                            self._planner_name,
                            repaired_goal[0],
                            repaired_goal[1],
                            repaired_goal[2],
                        )

        # --- Phase 6: Fallback planner decision ---
        if fallback_reason and selected_backend is self._backend:
            if self._plan_safety_policy == "reject":
                self._last_plan_report = {
                    "primary_planner": self._planner_name,
                    "selected_planner": self._planner_name,
                    "selected_path_safety": selected_safety,
                    "rejected_plans": rejected_plans,
                    "fallback_reason": fallback_reason,
                    "policy": self._plan_safety_policy,
                }
                raise RuntimeError(f"GlobalPlanner: {fallback_reason}")
            if self._plan_safety_policy == "fallback_astar":
                if not self._can_use_fallback_planner():
                    self._last_plan_report = {
                        "primary_planner": self._planner_name,
                        "selected_planner": self._planner_name,
                        "selected_path_safety": selected_safety,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": fallback_reason,
                        "policy": self._plan_safety_policy,
                    }
                    raise RuntimeError(
                        "GlobalPlanner: plan safety failed and fallback planner is not allowed"
                    )
                fb_backend = self._get_fallback_backend()
                fb_path, fb_ms = self._plan_with_backend(fb_backend, start, goal)
                fb_safety = self._evaluate_path_safety(fb_backend, fb_path)
                if fb_path and (fb_safety is None or fb_safety.get("ok", False)):
                    selected_backend = fb_backend
                    selected_planner = self._fallback_planner_name
                    selected_path = fb_path
                    selected_plan_ms += fb_ms
                    selected_safety = fb_safety
                    selected_reached_goal = bool(getattr(fb_backend, "_last_plan_reached_goal", True))
                    logger.warning(
                        "GlobalPlanner: %s; using fallback planner '%s'",
                        fallback_reason,
                        self._fallback_planner_name,
                    )
                else:
                    rejected_plans.append(
                        {
                            "planner": self._fallback_planner_name,
                            "path_safety": fb_safety,
                            "reason": "fallback planner unsafe or empty",
                        }
                    )
                    self._last_plan_report = {
                        "primary_planner": self._planner_name,
                        "selected_planner": self._planner_name,
                        "selected_path_safety": selected_safety,
                        "rejected_plans": rejected_plans,
                        "fallback_reason": fallback_reason,
                        "policy": self._plan_safety_policy,
                    }
                    raise RuntimeError(
                        "GlobalPlanner: plan safety failed and fallback planner did not produce a safe path"
                    )
            elif self._plan_safety_policy == "observe":
                logger.warning("GlobalPlanner: %s", fallback_reason)

        # --- Phase 7: Downsample and return ---
        downsample_goal = goal
        if not selected_reached_goal and selected_path:
            endpoint = np.array(selected_path[-1][:3], dtype=float)
            downsample_goal = endpoint
            logger.warning(
                "GlobalPlanner: planner '%s' returned a safe partial path to "
                "(%.2f, %.2f, %.2f); original goal remains unreachable in current map",
                selected_planner,
                endpoint[0],
                endpoint[1],
                endpoint[2],
            )
        self._last_plan_report = {
            "primary_planner": self._planner_name,
            "selected_planner": selected_planner,
            "selected_path_safety": selected_safety,
            "fallback_reason": fallback_reason,
            "rejected_plans": rejected_plans,
            "policy": self._plan_safety_policy,
            "reached_goal": selected_reached_goal,
        }
        planner_diagnostics = backend_plan_diagnostics(selected_backend)
        if planner_diagnostics:
            self._last_plan_report["planner_diagnostics"] = planner_diagnostics
        if primary_replan is not None:
            self._last_plan_report["primary_replan"] = primary_replan
        path = downsample_path(selected_path, downsample_goal, self._downsample_dist)
        accepted_goal = (
            np.asarray(path[-1][:3], dtype=float).copy()
            if path
            else np.asarray(downsample_goal[:3], dtype=float).copy()
        )
        adjusted_goal = (
            accepted_goal.tolist()
            if not np.allclose(accepted_goal, requested_goal, atol=0.05)
            else None
        )
        requested_goal_payload = requested_goal.tolist()
        accepted_goal_payload = accepted_goal.tolist()
        logger.info(
            "Planned %d waypoints in %.1fms (planner=%s)",
            len(path), selected_plan_ms, selected_planner,
        )
        self._last_plan_report.update({
            "requested_goal": requested_goal_payload,
            "accepted_goal": accepted_goal_payload,
            "adjusted_goal": adjusted_goal,
        })
        return path, selected_plan_ms

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        try:
            path, plan_ms = self.plan(
                request.start,
                request.goal,
                safe_goal_tolerance=request.safe_goal_tolerance,
            )
        except Exception as exc:
            report = self.last_plan_report
            return GlobalPlanResult(
                error=str(exc) or type(exc).__name__,
                frame_id=request.frame_id,
                request_id=request.request_id,
                map_version=request.map_version,
                diagnostics=dict(report.get("planner_diagnostics", {}) or {}),
                report=report,
            )
        report = self.last_plan_report
        return GlobalPlanResult(
            path=path,
            plan_ms=float(plan_ms),
            reached_goal=bool(report.get("reached_goal", True)),
            frame_id=request.frame_id,
            request_id=request.request_id,
            map_version=request.map_version,
            adjusted_goal=report.get("adjusted_goal"),
            diagnostics=dict(report.get("planner_diagnostics", {}) or {}),
            report=report,
        )

    def plan_map_only(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        *,
        planner_constraints: dict[str, Any] | None = None,
    ) -> tuple[list[np.ndarray], float]:
        """Run the configured map-backed backend without live safety overlays.

        This is for saved-map validation and no-motion route preview. It proves
        the active map artifact can drive OctoPlanner3D, but does not imply that
        the current live costmap/local planner would allow motion.
        """
        if self._backend is None:
            raise RuntimeError("GlobalPlanner: backend not set up")
        if self._map_artifact_gate_blocks():
            reason = self._map_artifact_gate_failure_reason()
            self._last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "selected_path_safety": None,
                "rejected_plans": [
                    {
                        "planner": self._planner_name,
                        "reason": reason,
                        "artifact_gate": self.map_artifact_gate,
                    }
                ],
                "fallback_reason": reason,
                "policy": "map_only",
                "reached_goal": False,
            }
            raise RuntimeError(f"GlobalPlanner: {reason}")

        previous_constraints = dict(self._octoplanner3d_constraints)
        override_constraints = normalize_octoplanner3d_constraints(planner_constraints)
        if override_constraints:
            self._octoplanner3d_constraints = {
                **self._octoplanner3d_constraints,
                **override_constraints,
            }
            configure_backend(self._backend, self._planner_name, self._octoplanner3d_constraints)

        try:
            requested_goal = np.asarray(goal[:3], dtype=float).copy()
            raw_path, plan_ms = self._plan_with_backend(self._backend, start, goal)
        finally:
            if override_constraints:
                self._octoplanner3d_constraints = previous_constraints
                configure_backend(self._backend, self._planner_name, previous_constraints)
        reached_goal = bool(getattr(self._backend, "_last_plan_reached_goal", True))
        planner_diagnostics = backend_plan_diagnostics(self._backend)
        if not raw_path:
            reason = str(getattr(self._backend, "_last_plan_error", "") or "empty path")
            self._last_plan_report = {
                "primary_planner": self._planner_name,
                "selected_planner": self._planner_name,
                "selected_path_safety": None,
                "rejected_plans": [
                    {
                        "planner": self._planner_name,
                        "reason": reason,
                        "planner_diagnostics": planner_diagnostics,
                    }
                ],
                "fallback_reason": reason,
                "policy": "map_only",
                "reached_goal": False,
                "planner_diagnostics": planner_diagnostics,
            }
            raise RuntimeError(f"GlobalPlanner: {reason}")

        downsample_goal = goal
        if not reached_goal and raw_path:
            downsample_goal = np.asarray(raw_path[-1][:3], dtype=float)
        path = downsample_path(raw_path, downsample_goal, self._downsample_dist)
        accepted_goal = (
            np.asarray(path[-1][:3], dtype=float).copy()
            if path
            else np.asarray(downsample_goal[:3], dtype=float).copy()
        )
        adjusted_goal = (
            accepted_goal.tolist()
            if not np.allclose(accepted_goal, requested_goal, atol=0.05)
            else None
        )
        self._last_plan_report = {
            "primary_planner": self._planner_name,
            "selected_planner": self._planner_name,
            "selected_path_safety": None,
            "fallback_reason": "",
            "rejected_plans": [],
            "policy": "map_only",
            "reached_goal": reached_goal,
            "planner_diagnostics": planner_diagnostics,
            "requested_goal": requested_goal.tolist(),
            "accepted_goal": accepted_goal.tolist(),
            "adjusted_goal": adjusted_goal,
            "map_only_preview": True,
        }
        return path, plan_ms

    def update_map(
        self,
        grid: PlanningMap | np.ndarray,
        resolution: float = 0.2,
        origin: np.ndarray | None = None,
    ) -> None:
        """Push live costmap to the backend (if supported)."""
        planning_map = coerce_planning_map(
            grid,
            resolution=resolution,
            origin=origin,
        )
        self._last_map_update = planning_map
        push_backend_map_update(self._backend, planning_map)
        push_backend_map_update(self._fallback_backend, planning_map)

    # ------------------------------------------------------------------ #
    # Internals                                                            #
    # ------------------------------------------------------------------ #

    def _plan_with_backend(
        self,
        backend: Any,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> tuple[list, float]:
        execution = plan_backend(backend, GlobalPlanRequest(start=start, goal=goal))
        return execution.path, execution.plan_ms

    def _evaluate_path_safety(self, backend: Any, path: list) -> dict[str, Any] | None:
        if self._plan_safety_policy == "off":
            return None
        start_ignore_radius_m = float(
            self._octoplanner3d_constraints.get("robot_radius") or 0.35
        )
        return evaluate_backend_path_safety(
            path,
            backend,
            obstacle_thr=self._obstacle_thr,
            start_ignore_radius_m=start_ignore_radius_m,
        )

    def _can_use_fallback_planner(self) -> bool:
        fallback = str(self._fallback_planner_name or "").strip().lower()
        primary = str(self._planner_name or "").strip().lower()
        if primary == "octoplanner3d":
            return False
        return (
            self._plan_safety_policy == "fallback_astar"
            and bool(fallback)
            and fallback != primary
        )

    def _try_fallback_after_primary_failure(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        *,
        reason: str,
        primary_diagnostics: dict[str, Any],
        selected_plan_ms: float,
        selected_safety: dict[str, Any] | None,
        rejected_plans: list[dict[str, Any]],
    ) -> dict[str, Any] | None:
        if not self._can_use_fallback_planner():
            return None

        fb_backend = self._get_fallback_backend()
        fb_path, fb_ms = self._plan_with_backend(fb_backend, start, goal)
        fb_safety = self._evaluate_path_safety(fb_backend, fb_path)
        fb_diagnostics = backend_plan_diagnostics(fb_backend)

        if fb_path and (fb_safety is None or fb_safety.get("ok", False)):
            return {
                "backend": fb_backend,
                "planner": self._fallback_planner_name,
                "path": fb_path,
                "plan_ms": selected_plan_ms + fb_ms,
                "safety": fb_safety,
                "reached_goal": bool(getattr(fb_backend, "_last_plan_reached_goal", True)),
            }

        rejected_plans.append(
            {
                "planner": self._fallback_planner_name,
                "path_safety": fb_safety,
                "reason": "fallback planner unsafe or empty",
                "planner_diagnostics": fb_diagnostics,
                "primary_reason": reason,
                "primary_planner_diagnostics": primary_diagnostics,
                "primary_path_safety": selected_safety,
            }
        )
        return None

    def _get_fallback_backend(self):
        if self._fallback_backend is None:
            self._fallback_backend = self._create_backend(self._fallback_planner_name)
            if self._last_map_update is not None:
                push_backend_map_update(self._fallback_backend, self._last_map_update)
        return self._fallback_backend

    @property
    def last_plan_report(self) -> dict[str, Any]:
        """Return a copy of the latest plan report."""
        return dict(self._last_plan_report)

    def backend_status(self) -> dict[str, Any]:
        """Return planner backend status for health endpoints."""
        report = self.last_plan_report
        primary_unavailable_reason = backend_unavailable_reason(self._backend)
        can_fallback = self._can_use_fallback_planner()
        selected = str(
            report.get("selected_planner")
            or (
                self._fallback_planner_name
                if primary_unavailable_reason and can_fallback
                else self.planner_name
            )
        )
        fallback_reason = str(
            report.get("fallback_reason") or primary_unavailable_reason or ""
        )
        return {
            "configured_backend": self.planner_name,
            "backend": selected,
            "fallback_backend": self._fallback_planner_name,
            "degraded": bool(fallback_reason) or selected != self.planner_name,
            "degraded_reason": fallback_reason,
            "octoplanner3d_constraints": dict(self._octoplanner3d_constraints),
        }

    def evaluate_current_path_safety(self, path: list) -> dict[str, Any] | None:
        """Evaluate a precomputed path against the current live safety overlay.

        This is intentionally not a planner call. It answers the product
        question "is this already-planned route executable right now?" without
        rerunning OctoPlanner3D or invoking legacy fallback repair.
        """
        if self._backend is None:
            return None
        return self._evaluate_path_safety(self._backend, path)

    def reload_map(self, map_path: str = "") -> dict[str, Any]:
        """Reload the saved-map artifact through the planner service boundary."""
        self._tomogram = str(map_path or "")
        self._map_artifact_gate = self._validate_map_artifact_gate()
        if self._backend is None:
            return {
                "ok": False,
                "backend": self._planner_name,
                "reason": "planner_backend_not_ready",
            }
        if self._map_artifact_gate_blocks():
            return {
                "ok": False,
                "backend": self._planner_name,
                "reason": self._map_artifact_gate_failure_reason(),
                "artifact_gate": self.map_artifact_gate,
            }
        self._backend = self._create_backend(self._planner_name)
        if self._last_map_update is not None:
            push_backend_map_update(self._backend, self._last_map_update)
        return {
            "ok": True,
            "backend": self._planner_name,
            "mode": "map",
            "map_path": self._resolve_map_path(self._planner_name),
            "artifact_gate": self.map_artifact_gate,
        }

    def reload_tomogram(self, tomogram: str) -> dict[str, Any]:
        """Compatibility alias for old callers; use reload_map()."""
        if not self._is_pct_planner():
            return self.reload_map(tomogram)
        self._tomogram = str(tomogram or "")
        self._map_artifact_gate = self._validate_map_artifact_gate()
        if self._backend is None:
            return {
                "ok": False,
                "backend": self._planner_name,
                "reason": "planner_backend_not_ready",
            }
        load_tomogram = getattr(self._backend, "_load_tomogram", None)
        if callable(load_tomogram):
            load_tomogram(self._tomogram)
            return {"ok": True, "backend": self._planner_name, "mode": "tomogram"}
        update_map = getattr(self._backend, "update_map", None)
        if callable(update_map):
            with open(self._tomogram, "rb") as f:
                data = pickle.load(f)  # noqa: S301  # trusted local tomogram
            grid = data.get("grid") if isinstance(data, dict) else None
            if grid is None:
                return {
                    "ok": False,
                    "backend": self._planner_name,
                    "reason": "tomogram_has_no_grid",
                }
            push_backend_map_update(
                self._backend,
                PlanningMap(grid=grid, resolution=data.get("resolution", 0.2)),
            )
            return {"ok": True, "backend": self._planner_name, "mode": "costmap"}
        return {
            "ok": False,
            "backend": self._planner_name,
            "reason": "planner_backend_reload_unsupported",
        }

    def _is_pct_planner(self, name: str | None = None) -> bool:
        return (name or self._planner_name).lower() == "pct"

    def _is_octoplanner3d(self, name: str | None = None) -> bool:
        return normalize_planner_name(name or self._planner_name) == "octoplanner3d"


