import json

from runtime.msgs.numpy_compat import np

from nav.services.plan.contracts import (
    GLOBAL_PLAN_SCHEMA_VERSION,
    GlobalPlannerDiagnostics,
    GlobalPlanningMap,
    GlobalPlanRequest,
    GlobalPlanResult,
    PlanRequest,
    PlanResult,
    PlannerDiagnostics,
    PlanningMap,
    assert_global_plan_result_wire,
    global_plan_result_schema,
    validate_global_plan_result_wire,
)
from nav.services.plan.global_planner.backend_runtime import (
    backend_plan_diagnostics,
    plan_backend,
    push_backend_map_update,
)
from nav.services.plan.global_planner.service import GlobalPlanner
from nav.services.plan.preview import PlanPreviewService
from nav.services.safety.plan_safety import PlanSafetyGrid, evaluate_plan_safety


class _Backend:
    def __init__(self):
        self.map_update = None

    def plan_request(self, request: PlanRequest) -> PlanResult:
        return PlanResult(
            path=[request.start, request.goal],
            reached_goal=True,
            diagnostics={"planner": "fake", "stage": "ok"},
        )

    def update_map(self, grid, resolution=0.2, origin=None):
        self.map_update = (grid, resolution, origin)


class _PreviewPlanner:
    is_ready = True
    has_map = True
    planner_name = "fake"
    plan_safety_policy = "off"

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        return GlobalPlanResult(
            path=[request.start, request.goal],
            plan_ms=4.0,
            reached_goal=True,
            frame_id=request.frame_id,
        )


class _SnappingPreviewPlanner:
    is_ready = True
    has_map = True
    planner_name = "octoplanner3d"
    plan_safety_policy = "map_only"

    def __init__(self):
        self._last_plan_report = {}

    @property
    def last_plan_report(self):
        return dict(self._last_plan_report)

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        path = [
            np.asarray([0.1, 0.0, 0.0], dtype=float),
            np.asarray([0.8, 0.0, 0.0], dtype=float),
        ]
        self._last_plan_report = {
            "selected_planner": "octoplanner3d",
            "policy": "map_only",
            "reached_goal": False,
            "requested_start": request.start.tolist(),
            "accepted_start": path[0].tolist(),
            "adjusted_start": path[0].tolist(),
            "requested_goal": request.goal.tolist(),
            "accepted_goal": path[-1].tolist(),
            "adjusted_goal": path[-1].tolist(),
            "planner_diagnostics": {"stage": "cxx_plan_success"},
        }
        return GlobalPlanResult(
            path=path,
            plan_ms=5.0,
            reached_goal=False,
            adjusted_goal=path[-1],
            frame_id=request.frame_id,
            report=self._last_plan_report,
        )


class _FailingSnappingPreviewPlanner:
    is_ready = True
    has_map = True
    planner_name = "octoplanner3d"
    plan_safety_policy = "map_only"

    def __init__(self):
        self._last_plan_report = {}

    @property
    def last_plan_report(self):
        return dict(self._last_plan_report)

    def plan_request(self, request: GlobalPlanRequest) -> GlobalPlanResult:
        diagnostics = {
            "stage": "cxx_plan_failed",
            "start_xyz": request.start.tolist(),
            "goal_xyz": request.goal.tolist(),
            "stderr": (
                "GlobalPlanner::startPlan() Start snapped to free cell: "
                "[0.30, -0.50, 0.50]\n"
                "GlobalPlanner::startPlan() Goal snapped to free cell: "
                "[0.50, 0.30, 0.30]\n"
                "GlobalPlanner::tryPlan() A* planning failed."
            ),
        }
        self._last_plan_report = {
            "selected_planner": "octoplanner3d",
            "policy": "map_only",
            "reached_goal": False,
            "fallback_reason": "empty path",
            "planner_diagnostics": diagnostics,
            "rejected_plans": [
                {
                    "planner": "octoplanner3d",
                    "reason": "empty path",
                    "planner_diagnostics": diagnostics,
                }
            ],
        }
        return GlobalPlanResult(
            error="empty path",
            reached_goal=False,
            frame_id=request.frame_id,
            diagnostics=diagnostics,
            report=self._last_plan_report,
        )


def test_global_planner_contracts_keep_old_names_and_shape_map():
    assert PlanRequest is GlobalPlanRequest
    assert PlanResult is GlobalPlanResult
    assert PlanningMap is GlobalPlanningMap
    assert PlannerDiagnostics is GlobalPlannerDiagnostics

    planning_map = PlanningMap(
        grid=[[0, 1], [2, 3]],
        resolution=0.5,
        origin=[1.0, 2.0],
        map_version="m1",
    )

    assert planning_map.grid.dtype == np.float32
    assert planning_map.grid.shape == (2, 2)
    assert planning_map.origin.tolist() == [1.0, 2.0]
    assert PlannerDiagnostics(planner="p", stage="s").to_dict() == {
        "planner": "p",
        "stage": "s",
    }


def test_global_planner_contracts_round_trip_json_ready_payloads():
    request = GlobalPlanRequest(
        start=[0, 1],
        goal=[2, 3, 0.4],
        request_id="r1",
        map_version="m1",
    )
    result = GlobalPlanResult(
        path=[[0, 1, 0], [2, 3, 0.4]],
        plan_ms=12.5,
        reached_goal=True,
        request_id="r1",
        map_version="m1",
        diagnostics={"stage": "ok"},
    )
    planning_map = GlobalPlanningMap(grid=[[0.0]], resolution=0.25, origin=[1, 2])

    request_payload = request.to_wire()
    result_payload = result.to_wire()
    map_payload = planning_map.to_wire()

    assert request_payload["schema_version"] == GLOBAL_PLAN_SCHEMA_VERSION
    assert GlobalPlanRequest.from_wire(request_payload).goal.tolist() == [2.0, 3.0, 0.4]
    assert GlobalPlanResult.from_wire(result_payload).points()[-1].tolist() == [2.0, 3.0, 0.4]
    assert GlobalPlanningMap.from_wire(map_payload).origin.tolist() == [1.0, 2.0]
    assert result_payload["path"] == [[0.0, 1.0, 0.0], [2.0, 3.0, 0.4]]
    assert validate_global_plan_result_wire(result_payload) == []
    assert_global_plan_result_wire(result_payload)


def test_global_plan_result_wire_schema_is_json_ready_and_strict():
    schema = global_plan_result_schema()

    json.dumps(schema, allow_nan=False)
    assert schema["$id"] == "lingtu.global_plan.result"
    assert schema["properties"]["schema_version"]["const"] == GLOBAL_PLAN_SCHEMA_VERSION
    assert "path" in schema["required"]

    invalid = GlobalPlanResult(path=[[0, 0, 0]], frame_id="map").to_wire()
    invalid["path"] = [[0.0, 0.0]]
    issues = validate_global_plan_result_wire(invalid)

    assert any("path" in issue for issue in issues)


def test_backend_runtime_uses_plan_request_result_and_planning_map():
    backend = _Backend()
    request = PlanRequest(
        start=np.asarray([0.0, 0.0, 0.0]),
        goal=np.asarray([1.0, 0.0, 0.0]),
    )

    execution = plan_backend(backend, request)
    push_backend_map_update(backend, PlanningMap(grid=[[0.0]], resolution=0.25))

    assert len(execution.path) == 2
    assert execution.result.reached_goal is True
    assert backend_plan_diagnostics(backend)["stage"] == "ok"
    assert backend.map_update[1] == 0.25


def test_plan_preview_exposes_global_plan_wire_payload():
    preview = PlanPreviewService(timeout_s=1.0)
    try:
        result = preview.preview(
            planner=_PreviewPlanner(),
            start=[0.0, 0.0, 0.0],
            goal=[1.0, 0.0, 0.0],
            frame_id="map",
            frame_blocker=None,
        )
    finally:
        preview.shutdown()

    assert result["feasible"] is True
    assert result["global_plan"]["schema_version"] == GLOBAL_PLAN_SCHEMA_VERSION
    assert result["global_plan"]["path"] == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]


def test_plan_preview_exposes_start_and_goal_snap_diagnostics():
    preview = PlanPreviewService(timeout_s=1.0)
    try:
        result = preview.preview(
            planner=_SnappingPreviewPlanner(),
            start=[0.0, 0.0, 0.0],
            goal=[1.0, 0.0, 0.0],
            frame_id="map",
            frame_blocker=None,
        )
    finally:
        preview.shutdown()

    snap = result["snap_diagnostics"]
    assert result["feasible"] is True
    assert result["reached_goal"] is False
    assert snap["requested_start"] == [0.0, 0.0, 0.0]
    assert snap["snapped_start"] == [0.1, 0.0, 0.0]
    assert snap["requested_goal"] == [1.0, 0.0, 0.0]
    assert snap["snapped_goal"] == [0.8, 0.0, 0.0]
    assert snap["goal_snap_accepted"] is False


def test_plan_preview_parses_snap_diagnostics_from_failed_octoplanner_logs():
    preview = PlanPreviewService(timeout_s=1.0)
    try:
        result = preview.preview(
            planner=_FailingSnappingPreviewPlanner(),
            start=[0.0, 0.0, 0.0],
            goal=[1.0, 0.0, 0.0],
            frame_id="map",
            frame_blocker=None,
        )
    finally:
        preview.shutdown()

    snap = result["snap_diagnostics"]
    assert result["feasible"] is False
    assert result["reasons"] == ["planning_failed"]
    assert snap["snapped_start"] == [0.3, -0.5, 0.5]
    assert snap["snapped_goal"] == [0.5, 0.3, 0.3]
    assert snap["start_snap_distance_m"] > 0.7
    assert snap["goal_snap_accepted"] is False


def test_plan_safety_ignores_start_footprint_but_blocks_later_obstacle():
    grid = np.zeros((1, 12), dtype=float)
    grid[0, 2] = 100.0
    safety = PlanSafetyGrid(grid=grid, resolution=0.1, origin=(0.0, 0.0))
    path = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]

    result = evaluate_plan_safety(
        path,
        safety,
        obstacle_thr=50.0,
        max_step_m=0.1,
        start_ignore_radius_m=0.25,
    )

    assert result["ok"] is True
    assert result["ignored_start_sample_count"] == 1

    grid[0, 7] = 100.0
    blocked = evaluate_plan_safety(
        path,
        safety,
        obstacle_thr=50.0,
        max_step_m=0.1,
        start_ignore_radius_m=0.25,
    )

    assert blocked["ok"] is False
    assert blocked["ignored_start_sample_count"] == 1
    assert blocked["blocked_sample_count"] == 1
    assert blocked["blocked_samples"][0]["x"] == 0.7


def test_reload_map_replays_live_safety_overlay_to_recreated_backend():
    old_backend = _Backend()
    new_backend = _Backend()
    svc = GlobalPlanner(planner_name="octoplanner3d", plan_safety_policy="off")
    svc._backend = old_backend
    svc._create_backend = lambda _name=None: new_backend
    svc._validate_map_artifact_gate = lambda: {
        "required": True,
        "ok": True,
        "reason": "saved_map_artifact_ok",
        "blockers": [],
    }

    svc.update_map(PlanningMap(grid=[[1.0]], resolution=0.4, origin=[2.0, 3.0]))
    result = svc.reload_map()

    assert result["ok"] is True
    assert old_backend.map_update[1] == 0.4
    assert new_backend.map_update[1] == 0.4
    assert new_backend.map_update[2].tolist() == [2.0, 3.0]
