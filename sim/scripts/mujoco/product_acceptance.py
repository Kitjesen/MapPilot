"""Dispatch one RunPlan to its canonical MuJoCo runner."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import uuid
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, NoReturn
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.mujoco.evidence import (
    SimFeederStatusError,
    SimMotionEvidenceError,
    load_feeder_status,
    load_motion_evidence,
)

from lingtu.control import ProductControl
from lingtu.product_lock import CURRENT_RUN_FILE_NAME
from lingtu.run_plan import RunPlan
from lingtu.sim.acceptance import load_manifest
from lingtu.sim.daemon import ensure_sim_supervisor
from lingtu.sim.identity import SimChildLedger, SimChildLedgerError, SimChildSnapshot
from lingtu.sim.readiness import (
    readiness_expectation_for_process,
)
from lingtu.sim.switch import _load_committed_plan
from lingtu.switch_contracts import ProcessReport, SwitchFailed

_BOOT_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z")
_LIFECYCLE_PRODUCTS = frozenset({"teleop", "teleop_avoid", "map"})
_SENSOR_MIN_WINDOW_S = 3.0
_SIM_TRUTH_SLAM_CONFIG = "src/localization/fastlio2/config/sim_mid360.yaml"


@dataclass(frozen=True)
class AcceptanceTarget:
    """Verified runner and manifest from one immutable RunPlan."""

    product: str
    runner: Path
    manifest: Path
    roles: tuple[str, ...] = ()
    run_plan: Path | None = None


Scenario = Callable[[RunPlan, Path, str], Mapping[str, Any]]
Check = Callable[
    [Path, RunPlan, Sequence[Any], Mapping[str, Any], str],
    Mapping[str, Any],
]


def resolve_target(
    plan: RunPlan,
    *,
    runner: Path,
    manifest: Path,
    run_plan_path: Path | None = None,
) -> AcceptanceTarget:
    """Validate an explicitly selected acceptance runner and manifest."""

    if plan.env != "sim":
        raise ValueError(f"MuJoCo acceptance requires env='sim', received {plan.env!r}")
    resolved_runner = runner.expanduser().resolve()
    if not resolved_runner.is_file():
        raise ValueError(f"acceptance runner is missing: {resolved_runner}")
    resolved_manifest = manifest.expanduser().resolve()
    target = AcceptanceTarget(
        product=plan.product,
        runner=resolved_runner,
        manifest=resolved_manifest,
        roles=tuple(sorted({role for process in plan.processes for role in process.provides})),
        run_plan=(run_plan_path.expanduser().resolve() if run_plan_path else None),
    )
    validate_product_contract(target, plan=plan)
    return target


def validate_product_contract(
    target: AcceptanceTarget,
    *,
    plan: RunPlan | None = None,
) -> None:
    """Require manifest intent to agree with its target and optional RunPlan."""

    payload = load_manifest(target.manifest, root=ROOT)
    contract = payload.get("product_contract")
    if not isinstance(contract, Mapping):
        raise ValueError(
            f"MuJoCo acceptance manifest has no product_contract: {target.manifest}"
        )
    declared_product = str(contract.get("product") or "").strip()
    expected_product = plan.product if plan is not None else target.product
    if declared_product != expected_product:
        raise ValueError(
            "MuJoCo acceptance Product mismatch: "
            f"expected={expected_product!r}, manifest={declared_product!r}"
        )
    if plan is not None:
        expected = {
            "native_control_mode": str(plan.lifecycle.get("native_control_mode") or ""),
            "slam_mode": str(plan.lifecycle.get("slam_mode") or ""),
            "requires_map": bool(plan.lifecycle.get("requires_map")),
        }
        actual = {
            "native_control_mode": str(contract.get("native_control_mode") or ""),
            "slam_mode": str(contract.get("slam_mode") or ""),
            "requires_map": contract.get("requires_map"),
        }
        if actual != expected:
            raise ValueError(
                f"MuJoCo acceptance contract for {target.product} does not match "
                f"RunPlan: expected={expected}, actual={actual}"
            )
        route = str(contract.get("route") or "").strip() or None
        if route != plan.product_variant:
            raise ValueError(
                "MuJoCo acceptance Product variant mismatch: "
                f"plan={plan.product_variant!r}, manifest={route!r}"
            )
        navigation_runtime = payload.get("navigation_runtime")
        if isinstance(navigation_runtime, Mapping):
            declared_local_planner = str(
                navigation_runtime.get("local_planner") or ""
            ).strip()
            if declared_local_planner:
                selected_local_planner = str(
                    plan.native_nav.get("local_planner") or ""
                ).strip()
                if declared_local_planner != selected_local_planner:
                    raise ValueError(
                        "MuJoCo acceptance local-planner mismatch: "
                        f"plan={selected_local_planner!r}, "
                        f"manifest={declared_local_planner!r}"
                    )
    _acceptance_scope(payload, product=target.product)


def acceptance_scope(target: AcceptanceTarget) -> dict[str, Any]:
    """Return the claim boundary from one already verified target."""

    payload = load_manifest(target.manifest, root=ROOT)
    return _acceptance_scope(payload, product=target.product)


def classify_evidence(
    scope: Mapping[str, Any] | None,
    *,
    run_plan_verified: bool,
    acceptance_evaluated: bool,
    ok: bool,
    lifecycle_verified: bool = False,
    stop_verified: bool = False,
    cleanup_verified: bool = False,
    rollback_verified: bool = False,
) -> dict[str, Any]:
    """Classify evidence without promoting an incomplete Product lifecycle."""

    coverage = str((scope or {}).get("coverage") or "").strip()
    acceptance_was_evaluated = acceptance_evaluated is True
    run_plan_was_verified = run_plan_verified is True
    product_lifecycle_verified = (
        lifecycle_verified is True
        and stop_verified is True
        and cleanup_verified is True
        and rollback_verified is True
    )
    if not acceptance_was_evaluated:
        evidence_scope = "preflight"
    elif not run_plan_was_verified:
        evidence_scope = "diagnostic_component"
    elif coverage == "component":
        evidence_scope = "component_e2e"
    elif coverage == "product" and product_lifecycle_verified:
        evidence_scope = "product_e2e"
    elif coverage == "product":
        evidence_scope = "product_lifecycle_incomplete"
    else:
        evidence_scope = "invalid_scope"
    return {
        "acceptance_evaluated": acceptance_was_evaluated,
        "evidence_scope": evidence_scope,
        "product_acceptance_passed": bool(
            ok is True
            and acceptance_was_evaluated
            and run_plan_was_verified
            and coverage == "product"
            and product_lifecycle_verified
        ),
    }


def run(
    control: ProductControl,
    target: AcceptanceTarget,
    state_root: Path,
    scenario: Scenario,
    *,
    rollback_control: ProductControl | None = None,
    rollback_root: Path | None = None,
    check: Check | None = None,
    expected_plan: RunPlan | None = None,
) -> dict[str, Any]:
    """Run one Product scenario under ProductControl."""

    scope = acceptance_scope(target)
    verify_rollback = scope.get("coverage") == "product"
    root = _fresh_root(state_root)
    if verify_rollback:
        if rollback_control is None or rollback_root is None:
            raise ValueError("product coverage requires rollback control and state root")
        rollback_root = _fresh_root(rollback_root)
    product = target.product
    lifecycle: dict[str, Any] = {}
    scenario_report: Mapping[str, Any] = {}
    blockers: list[str] = []
    children: tuple[Any, ...] = ()
    switched = False
    lifecycle_verified = False
    session_id: str | None = None
    stop_report: Mapping[str, Any] = {}
    plan: RunPlan | None = None
    runtime_status: dict[str, Any] = {}
    sensor_runtime_evidence: dict[str, Any] = {
        "ok": True,
        "measurement_scope": "feeder_scheduler",
        "dds_delivery_verified": False,
        "published_count_scope": "endpoint_write",
        "window_s": None,
        "min_window_s": _SENSOR_MIN_WINDOW_S,
        "max_schedule_lateness_ms": None,
        "streams": {},
    }
    selected_local_planner = (
        str(expected_plan.native_nav.get("local_planner") or "").strip() or None
        if expected_plan is not None and product == "nav"
        else None
    )
    try:
        switch = control.switch(
            product,
            local_planner=selected_local_planner,
            state_dir=root,
        )
        if not isinstance(switch, Mapping):
            raise TypeError("ProductControl.switch must return a mapping")
        lifecycle["switch"] = dict(switch)
        switched = switch.get("ok") is True and switch.get("status") == "active"
        if not switched:
            raise RuntimeError(f"{product} switch did not become active")
        session_id = str(switch.get("product_session_id") or "").strip() or None
        committed, snapshot, current = _active_context(root, switch, product)
        plan = committed.plan
        if expected_plan is not None and committed.plan != expected_plan:
            raise RuntimeError(f"{product} RunPlan changed after dispatch")
        children = snapshot.children
        _check_ready(
            committed.plan,
            switch.get("readiness"),
            committed.product_session_id,
        )
        lifecycle_verified = True
        lifecycle["current"] = current
        lifecycle["ledger"] = snapshot.as_dict()
        result = scenario(
            committed.plan,
            committed.path,
            committed.product_session_id,
        )
        if not isinstance(result, Mapping):
            raise TypeError("scenario must return a mapping")
        scenario_report = result
        _check_current(root, committed)
        if scenario_report.get("ok") is not True:
            blockers.append("scenario_failed")
    except Exception as exc:
        blockers.append(f"{type(exc).__name__}:{exc}")
    finally:
        if switched:
            try:
                stopped = control.stop(
                    expected_product=product,
                    expected_product_session_id=session_id,
                    state_dir=root,
                )
                if not isinstance(stopped, Mapping):
                    raise TypeError("ProductControl.stop must return a mapping")
                stop_report = dict(stopped)
                lifecycle["stop"] = stop_report
            except Exception as exc:
                blockers.append(f"stop:{type(exc).__name__}:{exc}")

    stop_verified = False
    cleanup_verified = False
    evidence_verified = check is None and scenario_report.get("ok") is True
    details: Mapping[str, Any] = {}
    if stop_report:
        try:
            if plan is None:
                raise RuntimeError("active RunPlan was not captured before stop")
            _check_stop(plan, stop_report)
            stop_verified = True
        except Exception as exc:
            blockers.append(f"stop_evidence:{type(exc).__name__}:{exc}")
        if plan is not None:
            try:
                status, sensor_runtime_evidence = _load_sensor_runtime(
                    root,
                    plan,
                    children,
                    product_session_id=str(session_id or ""),
                )
                if status:
                    runtime_status["mujoco_feeder"] = status
            except Exception as exc:
                blockers.append(f"sensor_runtime:{type(exc).__name__}:{exc}")
        if check is not None and plan is not None:
            try:
                result = check(root, plan, children, scenario_report, str(session_id or ""))
                if not isinstance(result, Mapping):
                    raise TypeError("check must return a mapping")
                details = result
                evidence_verified = True
            except Exception as exc:
                blockers.append(f"evidence:{type(exc).__name__}:{exc}")
        try:
            _check_clean(root, children)
            cleanup_verified = True
        except Exception as exc:
            blockers.append(f"cleanup:{type(exc).__name__}:{exc}")

    if verify_rollback:
        try:
            rollback = check_rollback(
                rollback_control,
                rollback_root,
                product,
                local_planner=selected_local_planner,
            )
        except Exception as exc:
            rollback = {"ok": False, "error": f"{type(exc).__name__}:{exc}"}
        if rollback.get("ok") is not True:
            blockers.append("previous_product_rollback_not_verified")
    else:
        rollback = {
            "ok": None,
            "skipped": True,
            "reason": "component_coverage",
        }

    rollback_verified = rollback.get("ok") is True
    technical_ok = not blockers
    evidence = classify_evidence(
        scope,
        run_plan_verified=switched,
        acceptance_evaluated=bool(scenario_report),
        ok=technical_ok,
        lifecycle_verified=lifecycle_verified,
        stop_verified=stop_verified,
        cleanup_verified=cleanup_verified,
        rollback_verified=rollback_verified,
    )
    return {
        "ok": technical_ok,
        **evidence,
        "lifecycle_verified": lifecycle_verified,
        "stop_verified": stop_verified,
        "cleanup_verified": cleanup_verified,
        "evidence_verified": evidence_verified,
        "evidence": dict(details),
        "rollback_verified": rollback_verified if verify_rollback else None,
        "lifecycle": lifecycle,
        "scenario": dict(scenario_report),
        "runtime_status": runtime_status,
        "sensor_runtime_evidence": sensor_runtime_evidence,
        "rollback": rollback,
        "blockers": blockers,
    }


def check_rollback(
    control: ProductControl,
    state_root: Path,
    product: str,
    *,
    local_planner: str | None = None,
) -> dict[str, Any]:
    """Prove target failure restores one exact previous Product session."""

    root = _fresh_root(state_root)
    result: dict[str, Any] = {"ok": False}
    active = False
    session_id: str | None = None
    try:
        previous_switch = control.switch(
            product,
            local_planner=local_planner,
            state_dir=root,
        )
        if not isinstance(previous_switch, Mapping):
            raise TypeError("ProductControl.switch must return a mapping")
        active = previous_switch.get("ok") is True and previous_switch.get("status") == "active"
        session_id = str(previous_switch.get("product_session_id") or "").strip() or None
        previous, previous_ledger, _previous_current = _active_context(
            root,
            previous_switch,
            product,
        )
        try:
            replacement = control.switch(
                product,
                local_planner=local_planner,
                state_dir=root,
            )
        except SwitchFailed as exc:
            restored = _load_committed_plan(root, {})
            restored_ledger = SimChildLedger(root).load()
            current_exact = restored is not None and (
                restored.path,
                restored.product_session_id,
            ) == (
                previous.path,
                previous.product_session_id,
            )
            restored_identity = restored is not None and (
                restored.path,
                restored.plan,
                restored.product_session_id,
            ) == (
                previous.path,
                previous.plan,
                previous.product_session_id,
            )
            restored_targets = (
                restored_ledger is not None
                and restored_ledger.product_session_id
                == previous_ledger.product_session_id
                and {child.target for child in restored_ledger.children}
                == {child.target for child in previous_ledger.children}
                and all(
                    child.process_identity.matches()
                    for child in restored_ledger.children
                )
            )
            journal_absent = not (root / "switch.json").exists()
            result = {
                "ok": (
                    exc.report.status == "failed_rolled_back"
                    and restored_identity
                    and current_exact
                    and restored_targets
                    and journal_absent
                    and "target:quiesced" in exc.report.cleanup
                    and "previous:restored" in exc.report.cleanup
                ),
                "report": exc.report.as_dict(),
                "previous_identity_restored": restored_identity,
                "current_identity_restored": current_exact,
                "ledger_children_restored": restored_targets,
                "journal_absent": journal_absent,
            }
        else:
            if not isinstance(replacement, Mapping):
                raise TypeError("ProductControl.switch must return a mapping")
            session_id = str(replacement.get("product_session_id") or "").strip() or None
            result = {
                "ok": False,
                "reason": "fault_injection_switch_unexpectedly_succeeded",
            }
    except Exception as exc:
        result = {"ok": False, "error": f"{type(exc).__name__}:{exc}"}

    cleanup = False
    if active:
        try:
            cleanup_report = control.stop(
                expected_product=product,
                expected_product_session_id=session_id,
                state_dir=root,
            )
            cleanup = (
                isinstance(cleanup_report, Mapping)
                and cleanup_report.get("ok") is True
            )
        except Exception as exc:
            result["cleanup_error"] = f"{type(exc).__name__}:{exc}"
    result["probe_cleanup_verified"] = cleanup
    result["ok"] = result.get("ok") is True and cleanup
    return result


class _FailSecondApplyRunner:
    """One-shot fault seam that delegates every operation except target apply."""

    def __init__(self, delegate: Any) -> None:
        self._delegate = delegate
        self._apply_count = 0

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport:
        self._apply_count += 1
        if self._apply_count == 2:
            plan = RunPlan.load(run_plan_path)
            return ProcessReport(
                product=plan.product,
                env=plan.env,
                action="apply",
                ok=False,
                status="failed",
                error="exact acceptance injected target apply failure",
            )
        return self._delegate.apply(
            run_plan_path,
            product_session_id=product_session_id,
            timeout_s=timeout_s,
        )

    def quiesce(self, run_plan_path: Path, **kwargs: Any) -> ProcessReport:
        return self._delegate.quiesce(run_plan_path, **kwargs)

    def stop(self, run_plan_path: Path, **kwargs: Any) -> ProcessReport:
        return self._delegate.stop(run_plan_path, **kwargs)


def _fresh_root(path: Path) -> Path:
    root = path.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    if not root.is_dir():
        raise ValueError("acceptance state_root is not a directory")
    if (root / CURRENT_RUN_FILE_NAME).exists() or SimChildLedger(root).load() is not None:
        raise ValueError("acceptance requires an empty state_root")
    return root


def _active_context(
    state_root: Path,
    switch: Mapping[str, Any],
    product: str,
) -> tuple[Any, SimChildSnapshot, dict[str, Any]]:
    committed = _load_committed_plan(state_root, {})
    if committed is None:
        raise RuntimeError("current RunPlan is absent")
    switch_session_id = str(switch.get("product_session_id") or "").strip()
    if not switch_session_id:
        raise RuntimeError("switch report has no committed Product session")
    current = {
        "product": committed.plan.product,
        "product_variant": committed.plan.product_variant,
        "env": committed.plan.env,
        "run_plan_path": str(committed.path),
        "product_session_id": committed.product_session_id,
        "map_name": None,
        "map_identity": None,
    }
    expected = (
        product,
        switch.get("product_variant"),
        switch_session_id,
        None,
        None,
    )
    actual = (
        current.get("product"),
        current.get("product_variant"),
        current.get("product_session_id"),
        current.get("map_name"),
        current.get("map_identity"),
    )
    if actual != expected:
        raise RuntimeError("current identity does not match switch evidence")
    try:
        snapshot = SimChildLedger(state_root).load()
    except SimChildLedgerError as exc:
        raise RuntimeError("child ledger is not trusted") from exc
    if snapshot is None:
        raise RuntimeError("child ledger is absent")
    if snapshot.product_session_id != committed.product_session_id:
        raise RuntimeError("child ledger identity is invalid")
    expected_targets = {process.target for process in committed.plan.processes}
    if {child.target for child in snapshot.children} != expected_targets:
        raise RuntimeError("child ledger process set is incomplete")
    for child in snapshot.children:
        if not child.process_identity.matches():
            raise RuntimeError(f"child is not live: {child.target}")
    return committed, snapshot, current


def _check_current(state_root: Path, expected: Any) -> None:
    current = _load_committed_plan(state_root, {})
    if current is None or (
        current.path,
        current.plan,
        current.plan.product,
        current.plan.product_variant,
        current.product_session_id,
    ) != (
        expected.path,
        expected.plan,
        expected.plan.product,
        expected.plan.product_variant,
        expected.product_session_id,
    ):
        raise RuntimeError("current Product session changed during scenario")


def _check_ready(
    plan: RunPlan,
    ready: Mapping[str, Mapping[str, Any]] | None,
    product_session_id: str,
) -> None:
    if not isinstance(ready, Mapping):
        raise RuntimeError("switch has no typed readiness")
    processes = {process.name: process.target for process in plan.processes}
    if set(ready) != set(processes):
        raise RuntimeError("readiness process set is incomplete")
    for name, target in processes.items():
        process = next(item for item in plan.processes if item.name == name)
        item = ready.get(name)
        readiness = process.command.readiness if process.command is not None else None
        if not isinstance(item, Mapping) or readiness is None or item.get("target") != target:
            raise RuntimeError(f"readiness identity is invalid: {name}")
        if readiness.kind == "process":
            if (
                frozenset(item) != {"kind", "target", "active"}
                or item.get("kind") != "process"
                or item.get("active") is not True
            ):
                raise RuntimeError(f"process readiness is invalid: {name}")
            continue
        expectation = readiness_expectation_for_process(name, readiness.target)
        if (
            readiness.kind != "file"
            or expectation is None
            or frozenset(item) != {"kind", "target", "adapter", "payload"}
            or item.get("kind") != "file"
            or item.get("adapter") != expectation.adapter
            or type(item.get("payload")) is not dict
        ):
            raise RuntimeError(f"file readiness is invalid: {name}")
        payload = item["payload"]
        if (
            payload.get("adapter") != expectation.adapter
            or payload.get("product_session_id") != product_session_id
            or payload.get("product") != plan.product
            or payload.get("process") != name
            or payload.get("ready") is not True
        ):
            raise RuntimeError(f"typed readiness is invalid: {name}")


def _check_stop(plan: RunPlan, report: Mapping[str, Any]) -> None:
    expected_targets = {process.target for process in plan.processes}
    if (
        report.get("ok") is not True
        or report.get("status") != "stopped"
        or set(report.get("stopped") or ()) != expected_targets
    ):
        raise RuntimeError("stop report is incomplete")
    evidence = report.get("stop_evidence")
    evidence = evidence if isinstance(evidence, Mapping) else {}
    if set(evidence) != {process.name for process in plan.processes}:
        raise RuntimeError("stop evidence process set is incomplete")
    for process in plan.processes:
        shutdown = process.command.shutdown if process.command is not None else None
        item = evidence.get(process.name)
        if shutdown is None or shutdown.kind != "file":
            if not isinstance(item, Mapping) or (
                item.get("process") != process.name
                or item.get("target") != process.target
                or item.get("inactive") is not True
                or item.get("forced") is not False
            ):
                raise RuntimeError(f"process stop ACK is missing: {process.name}")
            continue
        if not isinstance(item, Mapping) or (
            item.get("terminal_ack") is not True
            or item.get("outcome") != "zero_applied"
        ):
            raise RuntimeError(f"terminal zero ACK is missing: {process.name}")


def _load_sensor_runtime(
    state_root: Path,
    plan: RunPlan,
    children: Sequence[Any],
    *,
    product_session_id: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    feeder = next(
        (process for process in plan.processes if process.name == "mujoco_feeder"),
        None,
    )
    if feeder is None:
        return {}, {
            "ok": True,
            "measurement_scope": "feeder_scheduler",
            "dds_delivery_verified": False,
            "published_count_scope": "endpoint_write",
            "window_s": None,
            "min_window_s": _SENSOR_MIN_WINDOW_S,
            "max_schedule_lateness_ms": None,
            "streams": {},
        }
    child = next((item for item in children if item.target == feeder.target), None)
    if child is None:
        raise RuntimeError("mujoco_feeder launch identity is missing")
    try:
        status = load_feeder_status(
            session_root=state_root,
            product=plan.product,
            product_session_id=product_session_id,
            process=feeder.name,
            launch_id=child.launch_id,
        )
    except SimFeederStatusError as exc:
        raise RuntimeError("mujoco_feeder terminal status is invalid") from exc
    updated_wall_ns = status.get("updated_wall_ns")
    if (
        isinstance(updated_wall_ns, bool)
        or not isinstance(updated_wall_ns, int)
        or updated_wall_ns < child.started_wall_ns
    ):
        raise RuntimeError("mujoco_feeder terminal status predates its launch")
    return dict(status), _sensor_runtime_evidence(plan, status)


def _sensor_runtime_evidence(
    plan: RunPlan,
    status: Mapping[str, Any],
) -> dict[str, Any]:
    if status.get("state") != "stopped":
        raise RuntimeError("mujoco_feeder terminal state is not stopped")
    expected = _expected_sensor_rates(plan)
    streams = status.get("streams")
    if not isinstance(streams, Mapping) or set(streams) != set(expected):
        raise RuntimeError("mujoco_feeder stream set does not match the RunPlan")
    window_s = _nonnegative_number(status.get("window_s"), "window_s")
    if expected and window_s < _SENSOR_MIN_WINDOW_S:
        raise RuntimeError(
            "mujoco_feeder insufficient observation: "
            f"window_s={window_s:g} is below {_SENSOR_MIN_WINDOW_S:g}s"
        )
    checked: dict[str, Any] = {}
    lateness: list[float] = []
    truth_localized = _uses_mujoco_truth_localization(plan)
    for name, expected_hz in expected.items():
        stream = streams[name]
        if not isinstance(stream, Mapping):
            raise RuntimeError(f"mujoco_feeder {name} status is invalid")
        reported_hz = _finite_number(stream.get("expected_hz"), f"{name} expected_hz")
        if reported_hz != expected_hz:
            raise RuntimeError(f"mujoco_feeder {name} expected_hz does not match the RunPlan")
        scheduled = _positive_count(stream.get("scheduled_count"), f"{name} scheduled_count")
        published = _positive_count(stream.get("published_count"), f"{name} published_count")
        dropped = _nonnegative_count(stream.get("dropped_count"), f"{name} dropped_count")
        if published + dropped != scheduled:
            raise RuntimeError(f"mujoco_feeder {name} counts are inconsistent")
        actual_hz = _finite_number(stream.get("actual_hz"), f"{name} actual_hz")
        lateness.append(
            _nonnegative_number(
                stream.get("max_schedule_lateness_ms"),
                f"{name} max_schedule_lateness_ms",
            )
        )
        drop_rate = dropped / scheduled
        rate_error = abs(actual_hz - expected_hz) / expected_hz
        within_limits = drop_rate <= 0.01 and rate_error <= 0.05
        required_for_gate = not (truth_localized and name == "imu")
        if required_for_gate and drop_rate > 0.01:
            raise RuntimeError(f"mujoco_feeder {name} drop rate exceeds 1%")
        if required_for_gate and rate_error > 0.05:
            raise RuntimeError(f"mujoco_feeder {name} actual_hz differs by more than 5%")
        checked[name] = {
            "expected_hz": expected_hz,
            "actual_hz": actual_hz,
            "drop_rate": drop_rate,
            "rate_error": rate_error,
            "required_for_gate": required_for_gate,
            "within_limits": within_limits,
            "gate_passed": within_limits or not required_for_gate,
        }
    return {
        "ok": True,
        "localization_authority": "mujoco_truth" if truth_localized else "slam_estimator",
        "measurement_scope": "feeder_scheduler",
        "dds_delivery_verified": False,
        "published_count_scope": "endpoint_write",
        "window_s": window_s,
        "min_window_s": _SENSOR_MIN_WINDOW_S,
        "max_schedule_lateness_ms": max(lateness) if lateness else None,
        "streams": checked,
    }


def _uses_mujoco_truth_localization(plan: RunPlan) -> bool:
    if plan.env != "sim":
        return False
    environment = plan.native_process_environment
    config = str(environment.get("LINGTU_SLAM_CONFIG") or "").strip().replace("\\", "/")
    return config == _SIM_TRUTH_SLAM_CONFIG


def _expected_sensor_rates(plan: RunPlan) -> dict[str, float]:
    sensor_plan = plan.simulation.get("sensor_plan")
    streams = sensor_plan.get("streams") if isinstance(sensor_plan, Mapping) else None
    if not isinstance(streams, Mapping):
        raise RuntimeError("RunPlan sensor streams are missing")
    expected: dict[str, float] = {}
    if plan.has_process("imu"):
        expected["imu"] = _planned_rate(streams, "imu")
    if plan.has_process("lidar"):
        expected["lidar"] = _planned_rate(streams, "mid360")
    if plan.has_process("camera"):
        rgb_hz = _planned_rate(streams, "rgb")
        depth_hz = _planned_rate(streams, "depth")
        if rgb_hz != depth_hz:
            raise RuntimeError("RunPlan RGB and depth rates differ")
        expected["camera_rgbd"] = rgb_hz
    return expected


def _planned_rate(streams: Mapping[str, Any], name: str) -> float:
    declarations = streams.get(name)
    if not isinstance(declarations, list) or not declarations:
        raise RuntimeError(f"RunPlan {name} stream is missing")
    if any(not isinstance(item, Mapping) for item in declarations):
        raise RuntimeError(f"RunPlan {name} stream is invalid")
    rates = {
        _finite_number(item.get("rate_hz"), f"RunPlan {name} rate_hz")
        for item in declarations
    }
    if len(rates) != 1:
        raise RuntimeError(f"RunPlan {name} stream rates are inconsistent")
    return next(iter(rates))


def _finite_number(value: Any, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise RuntimeError(f"{field} is invalid")
    return float(value)


def _nonnegative_number(value: Any, field: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise RuntimeError(f"{field} is invalid")
    return float(value)


def _positive_count(value: Any, field: str) -> int:
    parsed = _nonnegative_count(value, field)
    if parsed == 0:
        raise RuntimeError(f"{field} must be positive")
    return parsed


def _nonnegative_count(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise RuntimeError(f"{field} is invalid")
    return value


def _check_motion(
    state_root: Path,
    plan: RunPlan,
    children: Sequence[Any],
    scenario: Mapping[str, Any],
    product_session_id: str,
) -> Mapping[str, Any]:
    threshold = scenario.get("min_path_length_m")
    if (
        isinstance(threshold, bool)
        or not isinstance(threshold, (int, float))
        or not math.isfinite(float(threshold))
        or threshold <= 0
    ):
        raise RuntimeError("exact teleop physical motion threshold is missing")
    feeder = next((child for child in children if child.target == "mujoco_feeder"), None)
    if feeder is None:
        raise RuntimeError("exact teleop feeder launch identity is missing")
    try:
        evidence = load_motion_evidence(
            session_root=state_root,
            product_session_id=product_session_id,
            product=plan.product,
            process="mujoco_feeder",
            launch_id=feeder.launch_id,
        )
    except SimMotionEvidenceError as exc:
        raise RuntimeError("exact teleop physical motion evidence is invalid") from exc
    if (
        evidence.get("commanded_motion_observed") is not True
        or int(evidence.get("nonzero_command_count") or 0) <= 0
        or int(evidence.get("nonzero_physics_steps") or 0) <= 0
        or int(evidence.get("last_output_sequence") or 0) <= 0
        or float(evidence.get("path_length_xy_m") or 0.0) < float(threshold)
    ):
        raise RuntimeError("exact teleop physical APPLIED ACK or path evidence is incomplete")
    return dict(evidence)


def _check_teleop_avoid_motion(
    state_root: Path,
    plan: RunPlan,
    children: Sequence[Any],
    scenario: Mapping[str, Any],
    product_session_id: str,
) -> Mapping[str, Any]:
    feeder = next((child for child in children if child.target == "mujoco_feeder"), None)
    if feeder is None:
        raise RuntimeError("exact teleop_avoid feeder launch identity is missing")
    try:
        evidence = load_motion_evidence(
            session_root=state_root,
            product_session_id=product_session_id,
            product=plan.product,
            process="mujoco_feeder",
            launch_id=feeder.launch_id,
        )
    except SimMotionEvidenceError as exc:
        raise RuntimeError("exact teleop_avoid physical motion evidence is invalid") from exc

    thresholds = scenario.get("physical_acceptance")
    command = scenario.get("command")
    if not isinstance(thresholds, Mapping) or not isinstance(command, Mapping):
        raise RuntimeError("exact teleop_avoid physical acceptance contract is missing")

    def threshold(name: str) -> float:
        value = thresholds.get(name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) < 0.0
        ):
            raise RuntimeError(f"exact teleop_avoid {name} is invalid")
        return float(value)

    def command_component(name: str) -> float:
        value = command.get(name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise RuntimeError(f"teleop_avoid command {name} is invalid")
        return float(value)

    def evidence_number(name: str) -> float:
        value = evidence.get(name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
        ):
            raise RuntimeError(f"teleop_avoid {name} is invalid")
        return float(value)

    robot_geometry = scenario.get("robot_geometry")
    if not isinstance(robot_geometry, Mapping):
        raise RuntimeError("exact teleop_avoid robot geometry is missing")

    def robot_dimension(name: str) -> float:
        value = robot_geometry.get(name)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise RuntimeError(f"exact teleop_avoid {name} is invalid")
        return float(value)

    vx = command_component("vx")
    vy = command_component("vy")
    if math.hypot(vx, vy) <= 1e-6:
        raise RuntimeError("exact teleop_avoid translation command is missing")
    start = evidence.get("start_position_m")
    end = evidence.get("end_position_m")
    trajectory = evidence.get("trajectory")
    if (
        not isinstance(start, list)
        or len(start) != 3
        or not isinstance(end, list)
        or len(end) != 3
        or not isinstance(trajectory, list)
        or len(trajectory) < 2
    ):
        raise RuntimeError("exact teleop_avoid trajectory evidence is incomplete")
    start_yaw = evidence_number("start_yaw_rad")
    end_yaw = evidence_number("end_yaw_rad")
    direction = start_yaw + math.atan2(vy, vx)
    c = math.cos(direction)
    s = math.sin(direction)
    origin_x = float(start[0])
    origin_y = float(start[1])
    forward: list[float] = []
    lateral: list[float] = []
    for sample in trajectory:
        if not isinstance(sample, list) or len(sample) != 5:
            raise RuntimeError("exact teleop_avoid trajectory sample is invalid")
        dx = float(sample[1]) - origin_x
        dy = float(sample[2]) - origin_y
        forward.append(dx * c + dy * s)
        lateral.append(-dx * s + dy * c)

    peak_index = max(range(len(lateral)), key=lambda index: abs(lateral[index]))
    forward_progress = forward[-1]
    max_lateral = abs(lateral[peak_index])
    restored_offset = abs(lateral[-1])
    restoration_progress = forward_progress - forward[peak_index]
    heading_error = abs(math.atan2(math.sin(end_yaw - start_yaw), math.cos(end_yaw - start_yaw)))
    obstacle = _teleop_avoid_obstacle(plan, thresholds)
    vehicle_length = robot_dimension("vehicle_length_m")
    vehicle_width = robot_dimension("vehicle_width_m")
    minimum_clearance = _trajectory_box_clearance(
        trajectory,
        half_length=vehicle_length * 0.5,
        half_width=vehicle_width * 0.5,
        obstacle=obstacle,
    )
    final_projection_extent = (
        abs(math.cos(end_yaw - direction)) * vehicle_length * 0.5
        + abs(math.sin(end_yaw - direction)) * vehicle_width * 0.5
    )
    obstacle_forward_edge = max(x * c + y * s for x, y in obstacle["corners"])
    robot_rear_edge = float(end[0]) * c + float(end[1]) * s - final_projection_extent
    pass_margin = robot_rear_edge - obstacle_forward_edge
    metrics = {
        "forward_progress_m": forward_progress,
        "max_lateral_detour_m": max_lateral,
        "restored_corridor_offset_m": restored_offset,
        "post_detour_progress_m": restoration_progress,
        "restored_heading_error_rad": heading_error,
        "minimum_obstacle_clearance_m": minimum_clearance,
        "obstacle_pass_margin_m": pass_margin,
        "path_length_xy_m": float(evidence.get("path_length_xy_m") or 0.0),
    }
    failures: list[str] = []
    if evidence.get("commanded_motion_observed") is not True:
        failures.append("commanded_motion")
    if int(evidence.get("nonzero_command_count") or 0) <= 0 or int(
        evidence.get("nonzero_physics_steps") or 0
    ) <= 0:
        failures.append("applied_motion")
    if forward_progress + 1e-9 < threshold("minimum_forward_progress_m"):
        failures.append("forward_progress")
    if max_lateral + 1e-9 < threshold("minimum_lateral_detour_m"):
        failures.append("lateral_detour")
    if restored_offset > threshold("maximum_restored_corridor_offset_m") + 1e-9:
        failures.append("corridor_restoration")
    if heading_error > threshold("maximum_restored_heading_error_rad") + 1e-9:
        failures.append("heading_restoration")
    if restoration_progress + 1e-9 < threshold("minimum_restoration_m"):
        failures.append("post_detour_progress")
    if minimum_clearance + 1e-9 < threshold("minimum_clearance_m"):
        failures.append("obstacle_clearance")
    if pass_margin + 1e-9 < threshold("minimum_pass_margin_m"):
        failures.append("obstacle_pass_margin")
    if failures:
        raise RuntimeError("teleop_avoid physical evidence failed: " + ",".join(failures))
    return {
        "motion": dict(evidence),
        "metrics": metrics,
        "obstacle": {
            "geom": obstacle["name"],
            "world": str(obstacle["world"]),
            "center_xy_m": list(obstacle["center"]),
            "half_size_xy_m": list(obstacle["half_size"]),
        },
    }


def _teleop_avoid_obstacle(
    plan: RunPlan,
    thresholds: Mapping[str, Any],
) -> dict[str, Any]:
    obstacle_name = str(thresholds.get("obstacle_geom") or "").strip()
    if not obstacle_name:
        raise RuntimeError("exact teleop_avoid obstacle geom is missing")
    simulation = getattr(plan, "simulation", None)
    physics = simulation.get("physics_plan") if isinstance(simulation, Mapping) else None
    world = physics.get("world") if isinstance(physics, Mapping) else None
    mjcf = world.get("mjcf") if isinstance(world, Mapping) else None
    if not isinstance(mjcf, str) or not mjcf.strip():
        raise RuntimeError("exact teleop_avoid world MJCF is missing")
    world_path = Path(mjcf)
    if not world_path.is_absolute():
        world_path = ROOT / world_path
    world_path = world_path.resolve()
    if not world_path.is_file():
        raise RuntimeError(f"exact teleop_avoid world MJCF is missing: {world_path}")
    try:
        root = ElementTree.parse(world_path).getroot()  # noqa: S314 - repository-owned MJCF
    except (ElementTree.ParseError, OSError) as exc:
        raise RuntimeError("exact teleop_avoid world MJCF is invalid") from exc
    worldbody = root.find("worldbody")
    matches = (
        []
        if worldbody is None
        else [geom for geom in worldbody.findall("geom") if geom.get("name") == obstacle_name]
    )
    if len(matches) != 1:
        raise RuntimeError(
            f"exact teleop_avoid obstacle geom must appear once in worldbody: {obstacle_name}"
        )
    geom = matches[0]
    if geom.get("type") != "box":
        raise RuntimeError("exact teleop_avoid obstacle geom must be a box")
    if any(name in geom.attrib for name in ("quat", "axisangle", "euler", "xyaxes", "zaxis")):
        raise RuntimeError("exact teleop_avoid obstacle box must be axis-aligned")

    def vector(attribute: str) -> tuple[float, float, float]:
        raw = str(geom.get(attribute) or "").split()
        if len(raw) != 3:
            raise RuntimeError(f"exact teleop_avoid obstacle {attribute} is invalid")
        try:
            values = tuple(float(value) for value in raw)
        except ValueError as exc:
            raise RuntimeError(f"exact teleop_avoid obstacle {attribute} is invalid") from exc
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"exact teleop_avoid obstacle {attribute} is invalid")
        return values[0], values[1], values[2]

    pos = vector("pos")
    size = vector("size")
    if size[0] <= 0.0 or size[1] <= 0.0:
        raise RuntimeError("exact teleop_avoid obstacle size is invalid")
    corners = (
        (pos[0] - size[0], pos[1] - size[1]),
        (pos[0] + size[0], pos[1] - size[1]),
        (pos[0] + size[0], pos[1] + size[1]),
        (pos[0] - size[0], pos[1] + size[1]),
    )
    return {
        "name": obstacle_name,
        "world": world_path,
        "center": (pos[0], pos[1]),
        "half_size": (size[0], size[1]),
        "corners": corners,
    }


def _trajectory_box_clearance(
    trajectory: Sequence[Any],
    *,
    half_length: float,
    half_width: float,
    obstacle: Mapping[str, Any],
) -> float:
    poses: list[tuple[float, float, float]] = []
    for sample in trajectory:
        pose = (float(sample[1]), float(sample[2]), float(sample[4]))
        if not poses:
            poses.append(pose)
            continue
        previous = poses[-1]
        distance = math.hypot(pose[0] - previous[0], pose[1] - previous[1])
        yaw_delta = math.atan2(
            math.sin(pose[2] - previous[2]),
            math.cos(pose[2] - previous[2]),
        )
        subdivisions = max(1, math.ceil(max(distance / 0.02, abs(yaw_delta) / 0.04)))
        for index in range(1, subdivisions + 1):
            fraction = index / subdivisions
            poses.append(
                (
                    previous[0] + (pose[0] - previous[0]) * fraction,
                    previous[1] + (pose[1] - previous[1]) * fraction,
                    previous[2] + yaw_delta * fraction,
                )
            )
    obstacle_corners = tuple(obstacle["corners"])
    return min(
        _polygon_distance(
            _oriented_box(x, y, yaw, half_length, half_width),
            obstacle_corners,
        )
        for x, y, yaw in poses
    )


def _oriented_box(
    center_x: float,
    center_y: float,
    yaw: float,
    half_length: float,
    half_width: float,
) -> tuple[tuple[float, float], ...]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return tuple(
        (
            center_x + forward * c - lateral * s,
            center_y + forward * s + lateral * c,
        )
        for forward, lateral in (
            (-half_length, -half_width),
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
        )
    )


def _polygon_distance(
    first: Sequence[tuple[float, float]],
    second: Sequence[tuple[float, float]],
) -> float:
    for polygon in (first, second):
        for index, start in enumerate(polygon):
            end = polygon[(index + 1) % len(polygon)]
            axis_x = -(end[1] - start[1])
            axis_y = end[0] - start[0]
            first_projection = [x * axis_x + y * axis_y for x, y in first]
            second_projection = [x * axis_x + y * axis_y for x, y in second]
            if max(first_projection) < min(second_projection) or max(second_projection) < min(
                first_projection
            ):
                break
        else:
            continue
        break
    else:
        return 0.0

    def point_segment_distance(
        point: tuple[float, float],
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> float:
        edge_x = end[0] - start[0]
        edge_y = end[1] - start[1]
        length_squared = edge_x * edge_x + edge_y * edge_y
        projection = (
            0.0
            if length_squared == 0.0
            else max(
                0.0,
                min(
                    1.0,
                    ((point[0] - start[0]) * edge_x + (point[1] - start[1]) * edge_y)
                    / length_squared,
                ),
            )
        )
        closest_x = start[0] + projection * edge_x
        closest_y = start[1] + projection * edge_y
        return math.hypot(point[0] - closest_x, point[1] - closest_y)

    return min(
        point_segment_distance(point, edge_start, edge_end)
        for vertices, edges in ((first, second), (second, first))
        for point in vertices
        for edge_index, edge_start in enumerate(edges)
        for edge_end in (edges[(edge_index + 1) % len(edges)],)
    )


def _check_clean(state_root: Path, children: Sequence[Any]) -> None:
    if (state_root / CURRENT_RUN_FILE_NAME).exists():
        raise RuntimeError("current record survived stop")
    if SimChildLedger(state_root).load() is not None:
        raise RuntimeError("child ledger survived stop")
    live = [child.target for child in children if child.process_identity.matches()]
    if live:
        raise RuntimeError("child identities survived stop: " + ", ".join(live))


def command_for(target: AcceptanceTarget) -> list[str]:
    """Build the exact runner command from a verified RunPlan target."""

    if target.run_plan is None:
        raise ValueError("acceptance command requires the published RunPlan path")
    return [
        sys.executable,
        str(target.runner),
        "--run-plan",
        str(target.run_plan),
        "--manifest",
        str(target.manifest),
        "--strict",
    ]


def _runner_environment() -> dict[str, str]:
    """Return one boot identity shared by the complete acceptance process tree."""

    environment = dict(os.environ)
    boot_id = environment.get("LINGTU_HOST_BOOT_ID") or str(uuid.uuid4())
    if _BOOT_ID_RE.fullmatch(boot_id) is None:
        raise ValueError("LINGTU_HOST_BOOT_ID contains unsupported characters")
    environment["LINGTU_HOST_BOOT_ID"] = boot_id
    return environment


class _StrictParser(argparse.ArgumentParser):
    def error(self, message: str) -> NoReturn:
        raise ValueError(message)


def _parser() -> argparse.ArgumentParser:
    parser = _StrictParser(description=__doc__)
    parser.add_argument("--run-plan")
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--state-root", type=Path)
    parser.add_argument("--rollback-state-root", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--json", action="store_true", dest="json_output")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Execute an explicit acceptance target against one published RunPlan."""

    values = list(sys.argv[1:] if argv is None else argv)
    try:
        if "--run-plan" not in values:
            raise ValueError("--run-plan is required; Product names are not executable input")
        args = _parser().parse_args(values)
        run_plan_path = str(args.run_plan or "").strip()
        if not run_plan_path:
            raise ValueError("--run-plan requires an exact published plan path")
        resolved_plan_path = Path(run_plan_path).expanduser().resolve()
        plan = RunPlan.load(resolved_plan_path)
        target = resolve_target(
            plan,
            runner=args.runner,
            manifest=args.manifest,
            run_plan_path=resolved_plan_path,
        )
        scope = acceptance_scope(target)
        verify_rollback = scope.get("coverage") == "product"
        command = command_for(target)
        if args.dry_run:
            if target.product in _LIFECYCLE_PRODUCTS:
                payload = {
                    "backend": "mujoco",
                    "product": target.product,
                    "run_plan": str(Path(run_plan_path).expanduser().resolve()),
                    "acceptance_scope": scope,
                    "executed": False,
                    "exact_transaction": {
                        "state_root": (
                            str(args.state_root.expanduser().resolve())
                            if args.state_root is not None
                            else "required_at_execution"
                        ),
                        "steps": [
                            "ProductControl.switch",
                            f"{target.product} scenario",
                            "ProductControl.stop_current compare-and-stop",
                        ] + (["previous Product fault-injection rollback"] if verify_rollback else []),
                        "rollback_state_root": (
                            str(args.rollback_state_root.expanduser().resolve())
                            if args.rollback_state_root is not None
                            else "required_at_execution" if verify_rollback else None
                        ),
                    },
                }
                print(json.dumps(payload, ensure_ascii=False, indent=2))
                return 0
            payload = {
                "backend": "mujoco",
                "product": target.product,
                "run_plan": str(Path(run_plan_path).expanduser().resolve()),
                "roles": list(target.roles),
                "runner": str(target.runner),
                "manifest": str(target.manifest),
                "acceptance_scope": scope,
                "command": command,
            }
            print(json.dumps(payload, ensure_ascii=False, indent=2))
            return 0
        if target.product in _LIFECYCLE_PRODUCTS:
            if args.state_root is None:
                raise ValueError(f"{target.product} acceptance requires --state-root")
            if verify_rollback and args.rollback_state_root is None:
                raise ValueError(
                    f"{target.product} acceptance requires --rollback-state-root"
                )
            control = ProductControl(
                robot=plan.robot,
                env="sim",
                env_config={"backend": "mujoco"},
                process_env=_runner_environment(),
            )
            if target.product == "teleop":
                scenario = _teleop_case(target)
            elif target.product == "teleop_avoid":
                scenario = _avoid_case(target)
            else:
                scenario = _runner_case(target)
            rollback_root = (
                args.rollback_state_root.expanduser().resolve()
                if args.rollback_state_root is not None
                else None
            )
            rollback_control = None
            if verify_rollback and rollback_root is not None:
                rollback_root.mkdir(parents=True, exist_ok=True)
                delegate = ensure_sim_supervisor(
                    rollback_root,
                    ROOT,
                    timeout_s=30.0,
                )
                rollback_control = ProductControl(
                    robot=plan.robot,
                    env="sim",
                    env_config={"backend": "mujoco"},
                    process_env=_runner_environment(),
                    simulation_runner=_FailSecondApplyRunner(delegate),
                )
            report = run(
                control,
                target,
                args.state_root,
                scenario,
                check=(
                    _check_motion
                    if target.product == "teleop"
                    else _check_teleop_avoid_motion
                    if target.product == "teleop_avoid"
                    else None
                ),
                expected_plan=plan,
                rollback_control=rollback_control,
                rollback_root=rollback_root,
            )
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 0 if report["ok"] is True else 1
        completed = subprocess.run(  # noqa: S603
            command,
            check=False,
            env=_runner_environment(),
        )
        return int(completed.returncode)
    except (OSError, TypeError, ValueError) as exc:
        if "--json" in values:
            print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        else:
            print(f"ERROR: {exc}", file=sys.stderr)
        return 2


def _selected_nav_dds_domain_id(plan: RunPlan) -> str:
    selected = tuple(process for process in plan.processes if process.name == "nav_runtime")
    if len(selected) != 1 or selected[0].command is None:
        raise ValueError("exact teleop RunPlan must select exactly one nav_runtime command")
    command = selected[0].command
    argv = tuple(command.argv)
    positions = tuple(index for index, value in enumerate(argv) if value == "--domain-id")
    if len(positions) != 1 or positions[0] + 1 >= len(argv):
        raise ValueError("exact teleop nav_runtime must declare --domain-id exactly once")
    domain_id = argv[positions[0] + 1]
    try:
        parsed = int(domain_id)
    except (TypeError, ValueError) as exc:
        raise ValueError("exact teleop nav_runtime DDS domain ID is invalid") from exc
    if str(parsed) != domain_id or not 0 <= parsed <= 232:
        raise ValueError("exact teleop nav_runtime DDS domain ID is invalid")
    environment_domain = dict(command.env).get("LINGTU_DDS_DOMAIN_ID")
    if environment_domain is not None and environment_domain != domain_id:
        raise ValueError("exact teleop nav_runtime DDS domain ID conflicts with command env")
    return str(domain_id)


def _teleop_case(target: AcceptanceTarget) -> Scenario:
    """Bind the formal teleop dispatcher to its attach-only runner path."""

    def scenario(plan: RunPlan, run_plan_path: Path, product_session_id: str) -> Mapping[str, Any]:
        from sim.scripts.mujoco import teleop_native_acceptance as teleop

        artifact_dir = run_plan_path.parent / "teleop_acceptance"
        domain_id = _selected_nav_dds_domain_id(plan)
        args = teleop.build_parser().parse_args(
            [
                "--run-plan",
                str(run_plan_path),
                "--manifest",
                str(target.manifest),
                "--artifact-dir",
                str(artifact_dir),
                "--domain-id",
                domain_id,
                "--attach-only",
            ]
        )
        prepared = teleop.prepare_runtime(args)
        if prepared.get("ok") is not True:
            return {
                "ok": False,
                "mode": "attach_only",
                "blockers": list(prepared.get("blockers") or ()),
            }
        return dict(
            teleop.run_attached(
                plan=plan,
                run_plan_path=run_plan_path,
                product_session_id=product_session_id,
                prepared=prepared,
                args=args,
            )
        )

    return scenario


def _avoid_case(target: AcceptanceTarget) -> Scenario:
    """Bind assisted teleop evidence to the already active Product."""

    def scenario(
        plan: RunPlan,
        run_plan_path: Path,
        product_session_id: str,
    ) -> Mapping[str, Any]:
        from sim.scripts.mujoco import teleop_avoid_native_acceptance as avoid

        args = avoid.build_parser().parse_args(
            [
                "--run-plan",
                str(run_plan_path),
                "--manifest",
                str(target.manifest),
                "--artifact-dir",
                str(run_plan_path.parent / "teleop_avoid_acceptance"),
                "--domain-base",
                _selected_nav_dds_domain_id(plan),
            ]
        )
        prepared = avoid.prepare_runtime(args)
        if prepared.get("ok") is not True:
            return {
                "ok": False,
                "mode": "attach_only",
                "blockers": list(prepared.get("blockers") or ()),
            }
        return dict(
            avoid.run_attached(
                plan=plan,
                run_plan_path=run_plan_path,
                product_session_id=product_session_id,
                prepared=prepared,
                args=args,
            )
        )

    return scenario


def _runner_case(target: AcceptanceTarget) -> Scenario:
    """Run an existing attach-only Product runner."""

    def scenario(_plan: RunPlan, path: Path, _session_id: str) -> Mapping[str, Any]:
        completed = subprocess.run(  # noqa: S603
            command_for(replace(target, run_plan=path)),
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=_runner_environment(),
        )
        return {
            "ok": completed.returncode == 0,
            "returncode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }

    return scenario


def _acceptance_scope(payload: Mapping[str, Any], *, product: str) -> dict[str, Any]:
    scope = payload.get("acceptance_scope")
    if not isinstance(scope, Mapping):
        raise ValueError(f"MuJoCo acceptance manifest has no acceptance_scope: {product}")
    coverage = str(scope.get("coverage") or "").strip()
    if coverage not in {"product", "component"}:
        raise ValueError(
            f"MuJoCo acceptance coverage for {product} must be product or component"
        )
    claims = scope.get("claims")
    excluded = scope.get("excluded_claims")
    if (
        not isinstance(claims, list)
        or not claims
        or any(not str(value).strip() for value in claims)
    ):
        raise ValueError(f"MuJoCo acceptance claims for {product} are invalid")
    if not isinstance(excluded, list):
        raise ValueError(f"MuJoCo acceptance excluded_claims for {product} are invalid")
    if coverage == "component" and not excluded:
        raise ValueError(
            f"component MuJoCo acceptance for {product} must declare excluded_claims"
        )
    return dict(scope)


if __name__ == "__main__":
    raise SystemExit(main())
