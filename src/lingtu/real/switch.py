"""Fail-closed Product switch transaction for the real/systemd runtime."""

from __future__ import annotations

import json
import math
import os
import time
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from lingtu.product_lock import resolve_current_run_path, resolve_product_state_dir
from lingtu.products import (
    ProductLifecycle,
    ProductName,
    product_name,
)
from lingtu.real.backend import (
    FieldBackend,
    MapActivationToken,
    MapRollbackFailed,
    SessionStage,
    SwitchBackend,
)
from lingtu.real.systemd import SystemdRunner
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.switch_contracts import (
    MapIdentity,
    ProcessFailed,
    ProcessReport,
    SwitchFailed,
    SwitchReport,
    SwitchRequest,
    is_product_session_id,
    map_identity_as_record,
    map_identity_from_record,
    new_product_session_id,
    required_bool,
    required_text,
)

_ALREADY_ACTIVE_READINESS_TIMEOUT_S = 1.5


@dataclass(frozen=True)
class _CommittedSystemdPlan:
    """Validated previous field RunPlan."""

    path: Path
    plan: RunPlan
    product_session_id: str


class SwitchControl(Protocol):
    """ProductControl surface used by the transaction."""

    env: str

    def _resolve(
        self,
        product: str | None = None,
        *,
        product_variant: str | None = None,
        local_planner: str | None = None,
    ) -> RunPlan: ...

    def _apply_plan_for_switch(
        self,
        path: str | Path,
        *,
        previous_plan: RunPlan | None = None,
        dry_run: bool = False,
    ) -> ProcessReport: ...

    def _systemd_runner(self) -> SystemdRunner: ...

    def _quiesce_plan_for_switch(
        self,
        plan: RunPlan,
        *,
        dry_run: bool = False,
    ) -> ProcessReport: ...


def execute_switch(
    control: SwitchControl,
    request: SwitchRequest,
    *,
    backend: SwitchBackend | None = None,
    environment: Mapping[str, str] | None = None,
    state_dir: str | Path | None = None,
    dry_run: bool = False,
    resolved_plan: RunPlan | None = None,
) -> SwitchReport:
    """Compile once, apply one selective process transition, and fail closed."""

    process_environment = environment if environment is not None else os.environ
    backend = backend or FieldBackend(environment=process_environment)
    systemd_runner = control._systemd_runner()
    target_product = product_name(required_text(request.target_product, "target Product"))
    runtime_env = control.env
    committed = _current_run_record(state_dir, process_environment)
    committed_env = _text(committed.get("env"))
    if committed_env is not None and committed_env != runtime_env:
        raise RuntimeError(f"current Product belongs to Env {committed_env!r}, not {runtime_env!r}")
    raw_current_product = _text(committed.get("product"))
    current_product = product_name(raw_current_product) if raw_current_product is not None else None
    report = SwitchReport(
        current_product=current_product,
        target_product=target_product,
        env=runtime_env,
        product_variant=request.product_variant,
        local_planner=request.local_planner,
        dry_run=dry_run,
    )
    plan: RunPlan | None = None
    map_activation: MapActivationToken | None = None
    map_identity: MapIdentity | None = None
    session_stage: SessionStage | None = None
    plan_created = False
    plan_committed = False
    map_activation_committed = False
    mutated = False
    previous: _CommittedSystemdPlan | None = None
    transition_report: ProcessReport | None = None
    try:
        plan = resolved_plan or control._resolve(
            target_product,
            product_variant=request.product_variant,
            local_planner=request.local_planner,
        )
        if plan.product != target_product:
            raise RuntimeError("resolved RunPlan Product does not match switch request")
        if plan.product_variant != request.product_variant:
            raise RuntimeError("resolved RunPlan variant does not match switch request")
        if plan.env != runtime_env:
            raise RuntimeError("resolved RunPlan Env does not match ProductControl")
        report.local_planner = plan.native_nav.get("local_planner")
        target_lifecycle = _lifecycle(plan)
        if plan.process_control != "systemd":
            raise RuntimeError(f"Product {target_product} is controlled by {plan.process_control}, not systemd")
        previous = _committed_systemd_plan(
            committed,
            state_dir=state_dir,
            environment=process_environment,
        )
        if not plan.has_process("nav"):
            raise RuntimeError(f"Product {target_product} has no native navigation process")
        native_environment = plan.native_process_environment
        required_lifecycle = target_lifecycle.switch_policy
        if required_lifecycle != "cold_restart":
            raise RuntimeError(f"unsupported Product switch lifecycle: {required_lifecycle or 'missing'}")
        initial_pose = _initial_pose(request.initial_pose)
        map_name = _requested_map(target_lifecycle, request.map_name)
        if map_name and target_lifecycle.slam_mode != "localization":
            raise RuntimeError(f"Product {target_product} does not accept a saved map")
        if initial_pose is not None and target_lifecycle.slam_mode != "localization":
            raise RuntimeError(f"Product {target_product} does not accept an initial localization pose")
        if initial_pose is not None and not request.relocalize:
            raise RuntimeError("initial_pose requires relocalize=True")
        report.phases.append("preflight")
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        existing_binding = _committed_explore_binding(
            request,
            plan,
            target_lifecycle,
            map_name=map_name,
            current_product=current_product,
            state_dir=state_dir,
            environment=process_environment,
        )
        if existing_binding is not None:
            existing_plan_path, existing_map_identity, product_session_id = existing_binding
            probe_phase_count = len(report.phases)
            try:
                backend.wait_native_nav(
                    native_environment,
                    timeout_s=_ALREADY_ACTIVE_READINESS_TIMEOUT_S,
                )
                report.phases.append("existing_native_nav_ready")
                if target_lifecycle.slam_mode != "none" and plan.has_process("slam"):
                    backend.wait_slam(
                        target_lifecycle.slam_mode,
                        require_map=target_lifecycle.slam_mode == "localization",
                        require_localization=target_lifecycle.slam_mode == "localization",
                        timeout_s=_ALREADY_ACTIVE_READINESS_TIMEOUT_S,
                    )
                    report.phases.append("existing_slam_ready")
                backend.wait_exploration(
                    _explore_route(target_lifecycle.slam_mode),
                    map_identity=existing_map_identity,
                    product_session_id=product_session_id,
                    timeout_s=_ALREADY_ACTIVE_READINESS_TIMEOUT_S,
                    allow_active=True,
                )
                report.phases.append("existing_exploration_ready")
            except Exception:
                # Missing or stale evidence means "perform the normal cold
                # switch", never "assume healthy".
                del report.phases[probe_phase_count:]
            else:
                report.run_plan_path = str(existing_plan_path)
                report.product_session_id = product_session_id
                report.phases.append("already_active")
                report.ok = True
                report.status = "already_active"
                return report

        # SaveMap and recording sessions are bound to an existing Product
        # session. A cold start has no Gateway or recorder session to query.
        if current_product is not None:
            backend.assert_map_save_idle()
            report.phases.append("map_save_idle")

            backend.assert_recording_idle()
            report.phases.append("recording_idle")


        product_session_id = new_product_session_id()
        run_plan_path = _run_plan_path(state_dir, process_environment, product_session_id)
        if run_plan_path.exists():
            existing_plan = RunPlan.load(run_plan_path)
            if existing_plan != plan:
                raise RuntimeError("existing RunPlan does not match the resolved plan")
        else:
            plan.write(run_plan_path)
            plan_created = True
        report.run_plan_path = str(run_plan_path)
        report.phases.append("plan_published")

        backend.stop_motion(current_product)
        report.phases.append("motion_stopped")
        mutated = True
        report.phases.append("previous_run_retained")
        if map_name:
            map_activation = backend.stage_map(map_name)
            map_identity = map_activation.target
            report.phases.append("map_prepared")
        staged_session = backend.stage_session(
            run_plan_path,
            plan,
            native_environment,
            slam_mode=target_lifecycle.slam_mode,
            map_identity=map_identity,
            product_session_id=product_session_id,
            parameter_overrides=request.parameter_overrides,
        )
        if staged_session is None:
            raise RuntimeError("Product session staging returned no rollback token")
        session_stage = staged_session
        report.phases.append("session_staged")
        backend.clear_runtime_status()
        report.phases.append("stale_status_cleared")

        if RunPlan.load(run_plan_path) != plan:
            raise RuntimeError("published RunPlan changed after staging")
        try:
            transition_report = control._apply_plan_for_switch(
                run_plan_path,
                previous_plan=previous.plan if previous is not None else None,
            )
        except ProcessFailed as exc:
            transition_report = exc.report
            raise
        report.phases.append("processes_active")
        backend.wait_native_nav(native_environment, timeout_s=10.0)
        report.phases.append("native_nav_ready")
        if target_lifecycle.slam_mode != "none" and plan.has_process("slam"):
            backend.wait_slam(
                target_lifecycle.slam_mode,
                require_map=target_lifecycle.slam_mode == "localization",
                require_localization=False,
                timeout_s=35.0,
            )
            report.phases.append(
                "slam_frontend_ready"
                if target_lifecycle.slam_mode == "localization"
                else "slam_ready"
            )
        if target_lifecycle.slam_mode == "localization":
            backend.prepare_localization(
                target_lifecycle,
                map_name=map_name,
                relocalize=bool(request.relocalize),
                initial_pose=initial_pose,
            )
            report.phases.append("localization_prepared")
        if target_lifecycle.slam_mode == "localization" and plan.has_process("slam"):
            backend.wait_slam(
                target_lifecycle.slam_mode,
                require_map=True,
                require_localization=True,
                timeout_s=35.0,
            )
            report.phases.append("slam_ready")
        if target_lifecycle.session_mode == "navigating" or target_lifecycle.native_control_mode in {
            "teleop",
            "teleop_avoid",
        }:
            backend.wait_navigation(
                map_name=map_name,
                control_mode=target_lifecycle.native_control_mode,
                timeout_s=45.0,
            )
            report.phases.append("goal_acceptance_ready")
        if target_lifecycle.native_control_mode in {"teleop", "teleop_avoid"}:
            backend.wait_motion_output(
                target_lifecycle.native_control_mode,
                timeout_s=10.0,
            )
            report.phases.append("motion_output_ready")
        if target_lifecycle.session_mode == "exploring":
            backend.wait_exploration(
                _explore_route(target_lifecycle.slam_mode),
                map_identity=map_identity,
                product_session_id=product_session_id,
                timeout_s=45.0,
            )
            report.phases.append("exploration_ready")
        if "inspection_evidence_capture_and_result_ack" in plan.required_capabilities:
            backend.wait_inspection(timeout_s=45.0)
            report.phases.append("inspection_ready")
        if map_activation is not None:
            backend.commit_map(map_activation)
        _commit_current_run(
            run_plan_path,
            plan,
            process_environment,
            state_dir,
            product_session_id=product_session_id,
            map_name=map_name or None,
            map_identity=map_identity,
        )
        plan_committed = True
        map_activation_committed = True
        if map_activation is not None:
            report.phases.append("map_committed")
        report.phases.append("committed")
        report.product_session_id = product_session_id
        report.ok = True
        report.status = "active"
        return report
    except Exception as exc:
        report.error = str(exc) or exc.__class__.__name__
        if isinstance(exc, MapRollbackFailed):
            report.status = "rollback_failed"
        elif "motion_stopped" in report.phases:
            report.status = "failed_stopped"
        elif "plan_published" in report.phases:
            report.status = "stop_unconfirmed"
        else:
            report.status = "failed"
        if mutated and plan is not None:
            rollback_errors = [
                f"transition:{error}"
                for error in (transition_report.rollback_errors if transition_report is not None else ())
            ]
            try:
                backend.stop_motion(target_product)
                report.cleanup.append("motion:stopped")
            except Exception as cleanup_error:
                if report.status != "rollback_failed":
                    report.status = "stop_unconfirmed"
                report.cleanup.append(f"motion_session_failed:{cleanup_error}")
                rollback_errors.append(f"motion_session:{cleanup_error}")
            if map_activation is not None and not map_activation_committed:
                map_process_error: Exception | None = None
                if transition_report is not None:
                    try:
                        systemd_runner.ensure_transition_process_active(
                            plan,
                            transition_report,
                            "maps",
                        )
                    except Exception as cleanup_error:
                        # A start may report failure after activation.  Let the
                        # native RESTORE acknowledgement decide whether mapd is
                        # usable before escalating this as a rollback failure.
                        map_process_error = cleanup_error
                try:
                    backend.restore_map(map_activation)
                    report.cleanup.append("map:restored")
                except Exception as cleanup_error:
                    report.status = "rollback_failed"
                    if map_process_error is not None:
                        report.cleanup.append(f"map_process_failed:{map_process_error}")
                        rollback_errors.append(f"map_process:{map_process_error}")
                    report.cleanup.append(f"map_failed:{cleanup_error}")
                    rollback_errors.append(f"map:{cleanup_error}")
            if previous is not None and transition_report is not None:
                try:
                    stopped_target = systemd_runner.stop_transition_target(
                        plan,
                        transition_report,
                    )
                    report.cleanup.append(f"target_processes:{stopped_target.status}")
                except Exception as cleanup_error:
                    report.cleanup.append(f"target_processes_failed:{cleanup_error}")
                    rollback_errors.append(f"target_processes:{cleanup_error}")
            if previous is not None and session_stage is not None:
                try:
                    backend.rollback_session(session_stage)
                    report.cleanup.append("session:restored")
                except Exception as cleanup_error:
                    report.cleanup.append(f"session_failed:{cleanup_error}")
                    rollback_errors.append(f"session:{cleanup_error}")
            if previous is not None:
                if transition_report is not None and not rollback_errors:
                    try:
                        restored = systemd_runner.restore_transition_previous(
                            previous.plan,
                            transition_report,
                        )
                        report.cleanup.append(f"previous_processes:{restored.status}")
                    except Exception as cleanup_error:
                        report.cleanup.append(f"previous_processes_failed:{cleanup_error}")
                        rollback_errors.append(f"previous_processes:{cleanup_error}")
                if not rollback_errors:
                    try:
                        _restore_previous_product_session(
                            backend,
                            previous,
                            committed,
                        )
                        report.cleanup.append("previous_session:active")
                    except Exception as cleanup_error:
                        report.cleanup.append(f"previous_session_failed:{cleanup_error}")
                        rollback_errors.append(f"previous_session:{cleanup_error}")
                if rollback_errors:
                    report.status = "rollback_failed"
                    for rollback_plan in (plan, previous.plan):
                        try:
                            quiesce = control._quiesce_plan_for_switch(rollback_plan)
                            report.cleanup.append(f"fail_closed_processes:{quiesce.status}")
                        except Exception as cleanup_error:
                            report.cleanup.append(f"fail_closed_processes_failed:{cleanup_error}")
                    try:
                        backend.remove_session(previous.plan)
                        report.cleanup.append("fail_closed_session:removed")
                    except Exception as cleanup_error:
                        report.cleanup.append(f"fail_closed_session_failed:{cleanup_error}")
                    resolve_current_run_path(
                        state_dir,
                        environment=process_environment,
                    ).unlink(missing_ok=True)
                else:
                    report.status = "failed_rolled_back"
            else:
                try:
                    quiesce = control._quiesce_plan_for_switch(plan)
                    report.cleanup.append(f"processes:{quiesce.status}")
                except Exception as cleanup_error:
                    if report.status != "rollback_failed":
                        report.status = "stop_unconfirmed"
                    report.cleanup.append(f"processes_failed:{cleanup_error}")
                    rollback_errors.append(f"processes:{cleanup_error}")
                if session_stage is not None:
                    try:
                        backend.rollback_session(session_stage)
                        report.cleanup.append("session:removed")
                    except Exception as cleanup_error:
                        report.cleanup.append(f"session_failed:{cleanup_error}")
                resolve_current_run_path(
                    state_dir,
                    environment=process_environment,
                ).unlink(missing_ok=True)
        if plan_created and not plan_committed and report.run_plan_path:
            try:
                Path(report.run_plan_path).unlink(missing_ok=True)
                report.cleanup.append("plan:removed")
            except OSError as cleanup_error:
                report.cleanup.append(f"plan_failed:{cleanup_error}")
        raise SwitchFailed(report) from exc


def _lifecycle(plan: RunPlan) -> ProductLifecycle:
    payload = plan.lifecycle
    lifecycle_product = product_name(required_text(payload.get("product"), "RunPlan lifecycle Product"))
    if lifecycle_product != plan.product:
        raise RuntimeError("RunPlan lifecycle Product does not match plan")
    lifecycle_variant = _text(payload.get("product_variant"))
    if lifecycle_variant != plan.product_variant:
        raise RuntimeError("RunPlan lifecycle variant does not match plan")
    return ProductLifecycle(
        product=lifecycle_product,
        label=required_text(payload.get("label"), "RunPlan lifecycle label"),
        session_mode=required_text(
            payload.get("session_mode"),
            "RunPlan lifecycle session_mode",
        ),
        native_control_mode=required_text(
            payload.get("native_control_mode"),
            "RunPlan lifecycle native_control_mode",
        ),
        slam_mode=required_text(
            payload.get("slam_mode"),
            "RunPlan lifecycle slam_mode",
        ),
        requires_map=required_bool(
            payload.get("requires_map"),
            "RunPlan lifecycle requires_map",
        ),
        switch_policy=required_text(
            payload.get("switch_policy"),
            "RunPlan lifecycle switch_policy",
        ),
        product_variant=lifecycle_variant,
        default_for_session_mode=required_bool(
            payload.get("default_for_session_mode"),
            "RunPlan lifecycle default_for_session_mode",
        ),
        online_hot_switch_supported=required_bool(
            payload.get("online_hot_switch_supported"),
            "RunPlan lifecycle online_hot_switch_supported",
        ),
    )


def _requested_map(
    lifecycle: ProductLifecycle,
    requested_map: str | None,
) -> str:
    map_name = _text(requested_map) or ""
    if lifecycle.requires_map and not map_name:
        raise RuntimeError(f"Product {lifecycle.product} requires a map")
    return map_name




def _run_plan_path(
    state_dir: str | Path | None,
    environment: Mapping[str, str],
    product_session_id: str,
) -> Path:
    root = resolve_product_state_dir(state_dir, environment=environment)
    root.mkdir(parents=True, exist_ok=True)
    return root / f"plan-{product_session_id}.json"


def _current_run_record(
    state_dir: str | Path | None,
    environment: Mapping[str, str],
) -> Mapping[str, Any]:
    path = resolve_current_run_path(state_dir, environment=environment)
    if not path.is_file():
        return {}
    payload = _read_json(path)
    if payload.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError(f"current run record has unsupported schema: {path}")
    return payload


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, Mapping) else {}


def _text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _committed_systemd_plan(
    committed: Mapping[str, Any],
    *,
    state_dir: str | Path | None,
    environment: Mapping[str, str],
) -> _CommittedSystemdPlan | None:
    """Load the exact previous plan when a complete field record exists."""

    required = {
        "product",
        "product_variant",
        "env",
        "run_plan_path",
        "product_session_id",
    }
    if not required.issubset(committed):
        return None
    product_session_id = _text(committed.get("product_session_id"))
    if not is_product_session_id(product_session_id):
        return None
    root = resolve_product_state_dir(state_dir, environment=environment).resolve()
    raw_path = _text(committed.get("run_plan_path"))
    if raw_path is None:
        return None
    path = Path(raw_path)
    expected = root / f"plan-{product_session_id}.json"
    if not path.is_absolute() or path != expected or path.is_symlink() or not path.is_file():
        return None
    try:
        plan = RunPlan.load(path)
    except (OSError, RuntimeError, ValueError):
        return None
    recorded_variant = committed.get("product_variant")
    if recorded_variant is not None:
        recorded_variant = _text(recorded_variant)
        if recorded_variant is None:
            return None
    recorded_product = _text(committed.get("product"))
    recorded_env = _text(committed.get("env"))
    if (
        recorded_product is None
        or plan.product != recorded_product
        or plan.product_variant != recorded_variant
        or recorded_env is None
        or plan.env != recorded_env
        or plan.process_control != "systemd"
    ):
        return None
    return _CommittedSystemdPlan(
        path=path,
        plan=plan,
        product_session_id=product_session_id,
    )


def _restore_previous_product_session(
    backend: SwitchBackend,
    previous: _CommittedSystemdPlan,
    committed: Mapping[str, Any],
) -> None:
    """Re-activate the previous Product after its changed processes return."""

    plan = previous.plan
    lifecycle = _lifecycle(plan)
    map_name = _text(committed.get("map_name")) or ""
    raw_map_identity = committed.get("map_identity")
    map_identity = (
        map_identity_from_record(raw_map_identity, field_name="committed map identity")
        if raw_map_identity is not None
        else None
    )
    if lifecycle.requires_map and (not map_name or map_identity is None):
        raise RuntimeError("previous Product map binding is incomplete")
    if not lifecycle.requires_map and map_identity is not None:
        raise RuntimeError("previous Product has an unexpected map binding")

    backend.wait_native_nav(plan.native_process_environment, timeout_s=10.0)
    if lifecycle.slam_mode != "none" and plan.has_process("slam"):
        backend.wait_slam(
            lifecycle.slam_mode,
            require_map=lifecycle.slam_mode == "localization",
            require_localization=False,
            timeout_s=35.0,
        )
    if lifecycle.slam_mode == "localization":
        backend.prepare_localization(
            lifecycle,
            map_name=map_name,
            relocalize=False,
            initial_pose=None,
        )
    if lifecycle.slam_mode == "localization" and plan.has_process("slam"):
        backend.wait_slam(
            lifecycle.slam_mode,
            require_map=True,
            require_localization=True,
            timeout_s=35.0,
        )
    if lifecycle.session_mode == "navigating" or lifecycle.native_control_mode in {"teleop", "teleop_avoid"}:
        backend.wait_navigation(
            map_name=map_name,
            control_mode=lifecycle.native_control_mode,
            timeout_s=45.0,
        )
    if lifecycle.native_control_mode in {"teleop", "teleop_avoid"}:
        backend.wait_motion_output(
            lifecycle.native_control_mode,
            timeout_s=10.0,
        )
    if lifecycle.session_mode == "exploring":
        backend.wait_exploration(
            _explore_route(lifecycle.slam_mode),
            map_identity=map_identity,
            product_session_id=previous.product_session_id,
            timeout_s=45.0,
        )
    if "inspection_evidence_capture_and_result_ack" in plan.required_capabilities:
        backend.wait_inspection(timeout_s=45.0)


def _committed_explore_binding(
    request: SwitchRequest,
    plan: RunPlan,
    lifecycle: ProductLifecycle,
    *,
    map_name: str,
    current_product: ProductName | None,
    state_dir: str | Path | None,
    environment: Mapping[str, str],
) -> tuple[Path, MapIdentity | None, str] | None:
    """Return a proven committed Explore binding, otherwise require restart."""

    if (
        plan.product != "explore"
        or current_product != "explore"
        or request.initial_pose is not None
        or bool(request.parameter_overrides)
    ):
        return None
    try:
        committed = _current_run_record(state_dir, environment)
        required_fields = {
            "schema_version",
            "product",
            "product_variant",
            "env",
            "run_plan_path",
            "product_session_id",
            "map_name",
            "map_identity",
        }
        if not required_fields.issubset(committed):
            return None
        if (
            committed.get("schema_version") != CURRENT_RUN_SCHEMA
            or _text(committed.get("product")) != plan.product
            or _text(committed.get("product_variant")) != plan.product_variant
            or _text(committed.get("env")) != plan.env
        ):
            return None

        run_plan_path = Path(
            required_text(
                committed.get("run_plan_path"),
                "committed Explore RunPlan path",
            )
        )
        if not run_plan_path.is_file():
            return None
        committed_plan = RunPlan.load(run_plan_path)
        if (
            committed_plan != plan
            or committed_plan.product != plan.product
            or committed_plan.product_variant != plan.product_variant
            or committed_plan.env != plan.env
        ):
            return None

        product_session_id = required_text(
            committed.get("product_session_id"),
            "committed Explore Product session ID",
        )
        raw_map_identity = committed.get("map_identity")
        if lifecycle.requires_map:
            if _text(committed.get("map_name")) != map_name:
                return None
            map_identity = map_identity_from_record(
                raw_map_identity,
                field_name="committed Explore map identity",
            )
        else:
            if committed.get("map_name") is not None or raw_map_identity is not None or map_name:
                return None
            map_identity = None
        return run_plan_path, map_identity, product_session_id
    except (OSError, RuntimeError, TypeError, ValueError):
        return None




def _commit_current_run(
    run_plan_path: Path,
    plan: RunPlan,
    environment: Mapping[str, str],
    state_dir: str | Path | None,
    *,
    product_session_id: str,
    map_name: str | None,
    map_identity: MapIdentity | None,
) -> None:
    root = resolve_product_state_dir(state_dir, environment=environment)
    root.mkdir(parents=True, exist_ok=True)
    current = resolve_current_run_path(root, environment=environment)
    temp = current.with_name(f".{current.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    payload = {
        "schema_version": CURRENT_RUN_SCHEMA,
        "product": plan.product,
        "product_variant": plan.product_variant,
        "env": plan.env,
        "run_plan_path": str(run_plan_path),
        "product_session_id": product_session_id,
        "map_name": map_name,
        "map_identity": map_identity_as_record(map_identity),
        "committed_at": time.time(),
    }
    try:
        temp.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.chmod(temp, 0o600)
        os.replace(temp, current)
    finally:
        temp.unlink(missing_ok=True)


def _initial_pose(
    value: tuple[float, float, float] | None,
) -> tuple[float, float, float] | None:
    if value is None:
        return None
    if len(value) != 3:
        raise RuntimeError("initial pose must contain X, Y, and YAW")
    pose = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in pose):
        raise RuntimeError("initial pose must contain finite values")
    return pose


def _explore_route(slam_mode: str) -> str:
    if slam_mode == "mapping":
        return "live"
    if slam_mode == "localization":
        return "map"
    raise RuntimeError(f"exploration requires mapping or localization SLAM, got: {slam_mode}")
