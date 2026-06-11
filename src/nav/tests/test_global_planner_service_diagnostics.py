"""Focused tests for GlobalPlannerService report diagnostics."""

import numpy as np
import pytest

from nav.global_planner_service import GlobalPlannerService


class _EmptyPctBackend:
    _last_plan_error = "pct native plan raised exception"
    _last_plan_diagnostics = {
        "planner": "pct",
        "stage": "native_plan_exception",
        "start_xy": [0.0, 0.0],
        "goal_xy": [1.0, 0.0],
        "error_type": "RuntimeError",
        "error_message": "native planner transient state",
    }

    def plan(self, _start, _goal):
        return []


def test_global_planner_service_reports_backend_diagnostics_on_pct_failure():
    svc = GlobalPlannerService(planner_name="pct", plan_safety_policy="reject")
    svc._backend = _EmptyPctBackend()
    svc._map_artifact_gate = {
        "required": True,
        "ok": True,
        "reason": "saved_map_artifact_ok",
        "blockers": [],
    }

    with pytest.raises(RuntimeError, match="pct native plan raised exception"):
        svc.plan(
            np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]),
        )

    report = svc.last_plan_report
    diagnostics = report["planner_diagnostics"]
    assert diagnostics["stage"] == "native_plan_exception"
    assert diagnostics["error_type"] == "RuntimeError"
    assert report["rejected_plans"][0]["planner_diagnostics"] == diagnostics
