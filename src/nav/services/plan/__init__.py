"""Planning service entrypoints used by nav.navigation."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "GLOBAL_PLANNING_MAP_CONTRACT",
    "GLOBAL_PLAN_REQUEST_CONTRACT",
    "GLOBAL_PLAN_RESULT_CONTRACT",
    "GLOBAL_PLAN_RESULT_SCHEMA",
    "GLOBAL_PLAN_SCHEMA_VERSION",
    "LOCAL_PLANNER_BACKENDS",
    "LOCAL_PLAN_PORT_CONTRACT",
    "LOCAL_PLAN_REQUEST_CONTRACT",
    "LOCAL_PLAN_RESULT_CONTRACT",
    "LOCAL_PLAN_SCHEMA_VERSION",
    "GlobalPlanRequest",
    "GlobalPlanResult",
    "GlobalPlanner",
    "GlobalPlannerDiagnostics",
    "GlobalPlanningMap",
    "LocalPlanRequest",
    "LocalPlanResult",
    "MaplessDirectPlannerService",
    "PlanPreviewService",
    "PlanRequest",
    "PlanResult",
    "PlannerDiagnostics",
    "PlannerService",
    "PlanningMap",
    "assert_global_plan_result_wire",
    "assert_local_plan_result_wire",
    "create_planner_service",
    "global_plan_result_schema",
    "require_local_planner_backend",
    "validate_global_plan_result_wire",
    "validate_local_plan_result_wire",
]

_EXPORTS = {
    "GlobalPlanner": "nav.services.plan.global_planner.service",
    "GlobalPlannerDiagnostics": "nav.services.plan.contracts",
    "GlobalPlanningMap": "nav.services.plan.contracts",
    "GlobalPlanRequest": "nav.services.plan.contracts",
    "GlobalPlanResult": "nav.services.plan.contracts",
    "GLOBAL_PLAN_RESULT_CONTRACT": "nav.services.plan.contracts",
    "GLOBAL_PLAN_RESULT_SCHEMA": "nav.services.plan.contracts",
    "GLOBAL_PLAN_SCHEMA_VERSION": "nav.services.plan.contracts",
    "GLOBAL_PLAN_REQUEST_CONTRACT": "nav.services.plan.contracts",
    "GLOBAL_PLANNING_MAP_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_PORT_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_REQUEST_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_RESULT_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_SCHEMA_VERSION": "nav.services.plan.contracts",
    "LOCAL_PLANNER_BACKENDS": "nav.services.plan.contracts",
    "LocalPlanRequest": "nav.services.plan.contracts",
    "LocalPlanResult": "nav.services.plan.contracts",
    "MaplessDirectPlannerService": "nav.services.plan.mapless.direct",
    "PlanRequest": "nav.services.plan.contracts",
    "PlanResult": "nav.services.plan.contracts",
    "PlannerDiagnostics": "nav.services.plan.contracts",
    "PlanPreviewService": "nav.services.plan.preview",
    "PlanningMap": "nav.services.plan.contracts",
    "PlannerService": "nav.services.plan.contracts",
    "assert_global_plan_result_wire": "nav.services.plan.contracts",
    "assert_local_plan_result_wire": "nav.services.plan.contracts",
    "create_planner_service": "nav.services.plan.factory",
    "global_plan_result_schema": "nav.services.plan.contracts",
    "require_local_planner_backend": "nav.services.plan.contracts",
    "validate_global_plan_result_wire": "nav.services.plan.contracts",
    "validate_local_plan_result_wire": "nav.services.plan.contracts",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
