from __future__ import annotations


def test_navigation_plan_service_entrypoints_live_under_services_plan() -> None:
    from nav.services.plan.contracts import PlannerService
    from nav.services.plan.factory import create_planner_service
    from nav.services.plan.global_planner.service import GlobalPlanner
    from nav.local.local_planner import LocalPlanner

    svc = create_planner_service(planner_name="direct")

    assert isinstance(svc, PlannerService)
    assert svc.__class__.__name__ == "MaplessDirectPlannerService"
    assert GlobalPlanner.__name__ == "GlobalPlanner"
    assert LocalPlanner.__name__ == "LocalPlanner"


def test_planning_implementation_entrypoints_are_in_services_plan() -> None:
    from nav.services.plan.mapless.direct_path import DirectPathBackend
    from nav.services.plan.global_planner.service import GlobalPlanner
    from nav.services.plan.global_planner.path_feasibility import evaluate_ground_path
    from nav.services.plan.global_planner.algorithm.pct.planner import PCTPlanner
    from nav.local.cmu_py import score_cmu_py_paths
    from nav.local.models import CmuPyLocalPlannerRequest
    from nav.local.native import create_nanobind_backend
    from nav.local.local_planner_runtime import setup_local_planner_backend
    from nav.local.local_planner import LocalPlanner
    from nav.services.plan.factory import create_planner_service

    assert GlobalPlanner.__module__ == "nav.services.plan.global_planner.service"
    assert LocalPlanner.__module__ == "nav.local.local_planner"
    assert CmuPyLocalPlannerRequest.__module__ == "nav.local.models"
    assert score_cmu_py_paths.__module__ == "nav.local.cmu_py"
    assert create_nanobind_backend.__module__ == "nav.local.native"
    assert setup_local_planner_backend.__module__ == "nav.local.local_planner_runtime"
    assert DirectPathBackend.__module__ == "nav.services.plan.mapless.direct_path"
    assert evaluate_ground_path.__module__ == "nav.services.plan.global_planner.path_feasibility"
    assert PCTPlanner.__module__ == "nav.services.plan.global_planner.algorithm.pct.planner"
    assert create_planner_service.__module__ == "nav.services.plan.factory"


def test_local_plan_contract_matches_local_planner_ports() -> None:
    from nav.services.plan.contracts import LOCAL_PLAN_PORT_CONTRACT
    from nav.local.local_planner import LocalPlanner

    mod = LocalPlanner(backend="simple")

    assert set(LOCAL_PLAN_PORT_CONTRACT) == set(mod._ports_in) | set(mod._ports_out)
    assert {
        name
        for name, direction in LOCAL_PLAN_PORT_CONTRACT.items()
        if direction[0] == "In"
    } == set(mod._ports_in)
    assert {
        name
        for name, direction in LOCAL_PLAN_PORT_CONTRACT.items()
        if direction[0] == "Out"
    } == set(mod._ports_out)


def test_local_plan_result_wire_contract() -> None:
    from nav.services.plan.contracts import (
        LocalPlanResult,
        assert_local_plan_result_wire,
        validate_local_plan_result_wire,
    )

    payload = LocalPlanResult(
        local_path=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        control_hint={"reason": "test"},
        backend="simple",
    ).to_wire()

    assert payload["schema_version"] == "lingtu.local_plan.v1"
    assert payload["path_found"] is True
    assert validate_local_plan_result_wire(payload) == []
    assert_local_plan_result_wire(payload)

    bad = dict(payload)
    bad["local_path"] = [[0.0, 0.0]]
    assert "local_path" in validate_local_plan_result_wire(bad)[0]
