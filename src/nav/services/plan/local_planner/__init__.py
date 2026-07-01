"""Local planning service entrypoints."""

from __future__ import annotations

from importlib import import_module
from typing import Any

__all__ = [
    "LOCAL_PLAN_PORT_CONTRACT",
    "LOCAL_PLAN_REQUEST_CONTRACT",
    "LOCAL_PLAN_RESULT_CONTRACT",
    "LOCAL_PLAN_SCHEMA_VERSION",
    "LOCAL_PLANNER_BACKENDS",
    "LocalPlanRequest",
    "LocalPlanResult",
    "LocalPlanner",
    "PathFollower",
    "Terrain",
    "assert_local_plan_result_wire",
    "require_local_planner_backend",
    "validate_local_plan_result_wire",
]

_EXPORTS = {
    "LOCAL_PLAN_PORT_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_REQUEST_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_RESULT_CONTRACT": "nav.services.plan.contracts",
    "LOCAL_PLAN_SCHEMA_VERSION": "nav.services.plan.contracts",
    "LOCAL_PLANNER_BACKENDS": "nav.services.plan.contracts",
    "LocalPlanRequest": "nav.services.plan.contracts",
    "LocalPlanResult": "nav.services.plan.contracts",
    "LocalPlanner": "nav.services.plan.local_planner.service",
    "PathFollower": "nav.local.path_follower",
    "Terrain": "nav.local.terrain",
    "assert_local_plan_result_wire": "nav.services.plan.contracts",
    "require_local_planner_backend": "nav.services.plan.contracts",
    "validate_local_plan_result_wire": "nav.services.plan.contracts",
}


def __getattr__(name: str) -> Any:
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(name)
    module = import_module(module_name)
    return getattr(module, name)
