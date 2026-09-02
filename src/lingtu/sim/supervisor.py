"""Client and report decoding for the simulation supervisor daemon."""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from lingtu.run_plan import RunPlan
from lingtu.sim.rpc import (
    REQUEST_SCHEMA,
    SupervisorRequest,
    round_trip,
)
from lingtu.switch_contracts import PROCESS_REPORT_SCHEMA, ProcessReport

_MIN_RPC_TIMEOUT_S = 10.0
_RPC_TRANSPORT_MARGIN_S = 5.0
_DEFAULT_TARGET_TIMEOUT_S = 15.0


class SimulationSupervisorError(RuntimeError):
    """Raised when a simulation supervisor operation cannot be completed."""


class SimulationSupervisorClient:
    """Execute one already published simulation RunPlan through private RPC."""

    def __init__(self, session_root: str | os.PathLike[str]) -> None:
        try:
            root = Path(session_root).resolve(strict=True)
        except OSError as exc:
            raise SimulationSupervisorError("session_root is unavailable") from exc
        if not root.is_dir():
            raise SimulationSupervisorError(
                "session_root must be a directory"
            )
        self._session_root = root

    def apply(
        self,
        plan_path: str | os.PathLike[str],
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        """Apply the exact published RunPlan through its session supervisor."""
        return self._execute(
            "apply",
            plan_path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    def quiesce(
        self,
        plan_path: str | os.PathLike[str],
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        """Quiesce the exact published RunPlan through its session supervisor."""

        return self._execute(
            "quiesce",
            plan_path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    def stop(
        self,
        plan_path: str | os.PathLike[str],
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        """Stop the exact published RunPlan through its session supervisor."""

        return self._execute(
            "stop",
            plan_path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    def status(
        self,
        plan_path: str | os.PathLike[str],
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        """Read the latest monitored Product process state."""

        return self._execute(
            "status",
            plan_path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    def _execute(
        self,
        action: str,
        plan_path: str | os.PathLike[str],
        *,
        product_session_id: str,
        timeout_s: float | None,
    ) -> ProcessReport:
        exact_path = self._exact_plan_path(plan_path)
        plan = RunPlan.load(exact_path)
        if plan.env != "sim" or plan.process_control != "subprocess":
            raise SimulationSupervisorError(
                "simulation supervisor requires a subprocess RunPlan"
            )
        if Path(exact_path).name != f"plan-{product_session_id}.json":
            raise SimulationSupervisorError(
                "published RunPlan path does not match the Product session"
            )
        request = SupervisorRequest(
            schema_version=REQUEST_SCHEMA,
            action=action,
            run_plan_path=exact_path,
            product_session_id=product_session_id,
        )
        operation_timeout_s = (
            _default_operation_timeout_s(plan, action)
            if timeout_s is None
            else timeout_s
        )
        response = round_trip(self._session_root, request, operation_timeout_s)
        if not response.success:
            error = response.error
            if error is None:  # SupervisorResponse validates this invariant.
                raise SimulationSupervisorError("supervisor returned an invalid response")
            raise SimulationSupervisorError(error["message"])
        return _process_report_from_payload(
            response.result,
            plan=plan,
            action="apply" if action == "status" else action,
        )

    def _exact_plan_path(self, plan_path: str | os.PathLike[str]) -> str:
        try:
            path = Path(plan_path).resolve(strict=True)
        except OSError as exc:
            raise SimulationSupervisorError("published RunPlan is unavailable") from exc
        if path.parent != self._session_root or path.suffix != ".json":
            raise SimulationSupervisorError(
                "plan_path must be the exact published session RunPlan path"
            )
        return str(path)


def _default_operation_timeout_s(plan: RunPlan, action: str) -> float:
    """Cover sequential process deadlines plus RPC transport."""

    timeouts = {
        process.target: float(process.timeout_s)
        for process in plan.available_processes
    }
    if action == "apply":
        conflict_budget = sum(
            timeouts.get(target, _DEFAULT_TARGET_TIMEOUT_S)
            for target in plan.stop_before_start
        )
        selected_budget = sum(
            float(process.timeout_s) for process in plan.processes
        )
        # A failed start may consume start + readiness, then one rollback stop.
        process_budget = 3.0 * selected_budget
    elif action == "quiesce":
        conflict_budget = sum(
            timeouts.get(target, _DEFAULT_TARGET_TIMEOUT_S)
            for target in plan.stop_before_start
        )
        process_budget = 0.0
    elif action == "stop":
        conflict_budget = 0.0
        process_budget = sum(
            float(process.timeout_s) for process in plan.managed_processes
        )
    elif action == "status":
        conflict_budget = 0.0
        process_budget = 0.0
    else:
        raise SimulationSupervisorError(
            f"unsupported simulation supervisor action: {action}"
        )
    return max(
        _MIN_RPC_TIMEOUT_S,
        _RPC_TRANSPORT_MARGIN_S + conflict_budget + process_budget,
    )


def _process_report_from_payload(
    payload: Mapping[str, Any] | None,
    *,
    plan: RunPlan,
    action: str,
) -> ProcessReport:
    if type(payload) is not dict:
        raise SimulationSupervisorError("supervisor returned an invalid process report")
    if (
        payload.get("schema_version") != PROCESS_REPORT_SCHEMA
        or payload.get("product") != plan.product
        or payload.get("env") != plan.env
        or payload.get("action") != action
    ):
        raise SimulationSupervisorError("supervisor returned an invalid process report")
    try:
        return ProcessReport(
            product=plan.product,
            env=plan.env,
            action=action,
            dry_run=payload["dry_run"],
            ok=payload["ok"],
            status=payload["status"],
            planned=list(payload["planned"]),
            stopped=list(payload["stopped"]),
            started=list(payload["started"]),
            preserved=list(payload["preserved"]),
            ready=dict(payload["ready"]),
            stop_evidence=dict(payload["stop_evidence"]),
            rolled_back=list(payload["rolled_back"]),
            rollback_errors=list(payload["rollback_errors"]),
            error=payload.get("error"),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise SimulationSupervisorError(
            "supervisor returned an invalid process report"
        ) from exc


__all__ = [
    "SimulationSupervisorClient",
    "SimulationSupervisorError",
]
