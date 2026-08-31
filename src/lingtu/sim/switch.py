"""Execute one simulation Product switch from an exact RunPlan."""

from __future__ import annotations

import json
import os
import subprocess
import time
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from lingtu.product_lock import CURRENT_RUN_FILE_ENV, CURRENT_RUN_FILE_NAME
from lingtu.products import product_name
from lingtu.run_plan import CURRENT_RUN_SCHEMA, RunPlan
from lingtu.sim.identity import SimChildLedger, SimChildLedgerError
from lingtu.sim.journal import (
    _advance_switch_journal,
    _load_switch_journal,
    _new_switch_journal,
    _publish_switch_journal,
    _remove_switch_journal,
    _SimSwitchJournal,
    _SwitchPlanRef,
)
from lingtu.sim.readiness import (
    SimReadinessError,
    SimReadinessPending,
    load_typed_readiness,
    readiness_expectation_for_process,
)
from lingtu.switch_contracts import (
    MAP_ACTIVATION_TOKEN_SCHEMA,
    MapIdentity,
    ProcessReport,
    SwitchFailed,
    SwitchReport,
    SwitchRequest,
    is_product_session_id,
    map_identity_as_record,
    map_identity_environment,
    map_identity_from_native,
    map_identity_from_record,
    new_product_session_id,
    occupancy_artifact,
    octomap_artifact,
    optional_map_identity_from_native,
    pointcloud_artifact,
)

_REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
class SimSwitchRunner(Protocol):
    """Path-only process runner used by the simulation supervisor seam."""

    def apply(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport: ...

    def quiesce(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport: ...

    def stop(
        self,
        run_plan_path: Path,
        *,
        product_session_id: str,
        timeout_s: float | None = None,
    ) -> ProcessReport: ...


@dataclass(frozen=True)
class _CommittedPlan:
    """One fully validated committed simulation RunPlan identity."""

    path: Path
    plan: RunPlan
    product_session_id: str


class _CurrentCommitError(RuntimeError):
    def __init__(self, message: str, *, current_replaced: bool) -> None:
        super().__init__(message)
        self.current_replaced = current_replaced


def _plan_switch(
    request: SwitchRequest,
    *,
    resolved_plan: RunPlan,
) -> SwitchReport:
    """Return a side-effect-free preview for one exact simulation plan."""

    _validate_sim_request(request)
    _validate_target(resolved_plan, request)
    phases = ["preflight"]
    if request.map_name is not None:
        phases.append("map_runtime_pending")
    return SwitchReport(
        current_product=None,
        target_product=product_name(resolved_plan.product),
        env=resolved_plan.env,
        product_variant=resolved_plan.product_variant,
        local_planner=resolved_plan.native_nav.get("local_planner"),
        dry_run=True,
        ok=True,
        status="planned",
        phases=phases,
    )


def _execute_locked_switch(
    request: SwitchRequest,
    *,
    runner: SimSwitchRunner,
    environment: Mapping[str, str],
    state_root: Path,
    resolved_plan: RunPlan,
) -> SwitchReport:
    """Execute one exact sim plan while ProductControl owns ``state_root``."""

    target_product = product_name(str(request.target_product).strip())
    report = SwitchReport(
        current_product=None,
        target_product=target_product,
        env="sim",
        product_variant=request.product_variant,
        local_planner=resolved_plan.native_nav.get("local_planner"),
        dry_run=False,
    )
    plan = resolved_plan
    plan_path: Path | None = None
    plan_created = False
    committed = False
    target_attempted = False
    previous_quiesce_attempted = False
    previous: _CommittedPlan | None = None
    map_activation: dict[str, Any] | None = None
    prepare_map_environment: dict[str, str] | None = None
    map_identity: MapIdentity | None = None
    map_staged = False
    journal: _SimSwitchJournal | None = None
    product_session_id = new_product_session_id()
    try:
        _validate_sim_request(request)
        _validate_target(plan, request)
        report.phases.append("preflight")
        root = state_root
        if not root.is_absolute() or root != root.resolve() or not root.is_dir():
            raise RuntimeError("locked simulation state root is invalid")
        _reconcile_incomplete_switch(root, runner, environment)
        previous = _load_committed_plan(root, environment)
        report.current_product = (
            product_name(previous.plan.product) if previous is not None else None
        )
        if request.map_name is not None:
            if previous is not None:
                _require_saved_map_previous(previous, root)
            prepare_map_environment = _map_control_environment(
                environment,
                previous.plan if previous is not None else plan,
            )
            map_activation = _mapctl(
                prepare_map_environment,
                "prepare",
                request.map_name,
                timeout_s=10.0,
            )
            map_identity = _prepared_map_identity(map_activation, request.map_name)
            plan = plan.with_native_process_environment(
                _saved_map_environment(
                    plan,
                    map_identity,
                    initial_pose=_sim_initial_pose(request.initial_pose),
                )
            )
            _validate_target(plan, request)
            report.phases.append("map_prepared")
        plan_path = root / f"plan-{product_session_id}.json"
        plan.write(plan_path)
        plan_created = True
        report.run_plan_path = str(plan_path)
        report.phases.append("plan_published")
        journal = _new_switch_journal(
            target_path=plan_path,
            target_product_session_id=product_session_id,
            previous_path=None if previous is None else previous.path,
            previous_product_session_id=(
                None if previous is None else previous.product_session_id
            ),
            map_activation=map_activation,
        )
        _publish_switch_journal(root, journal)

        if map_activation is not None and previous is not None:
            if prepare_map_environment is None:  # pragma: no cover - invariant.
                raise RuntimeError("sim saved-map map control environment is unavailable")
            _require_mapctl_transition(
                _mapctl(
                    prepare_map_environment,
                    "stage",
                    request.map_name,
                    timeout_s=20.0,
                ),
                prepared=map_activation,
                operation="stage",
            )
            map_staged = True
            report.phases.append("map_staged")

        if previous is not None:
            journal = _advance_switch_journal(
                root,
                journal,
                "previous_stopping",
            )
            previous_quiesce_attempted = True
            _require_success(
                runner.quiesce(
                    previous.path,
                    product_session_id=previous.product_session_id,
                ),
                action="quiesce previous Product",
            )
            report.phases.append("previous_quiesced")

        journal = _advance_switch_journal(root, journal, "target_starting")
        target_attempted = True
        launch = runner.apply(plan_path, product_session_id=product_session_id)
        _require_success(launch, action="apply target Product")
        report.readiness = dict(launch.ready)
        report.phases.append("processes_active")
        if map_activation is not None:
            if not map_staged:
                _require_mapctl_transition(
                    _mapctl(
                        _map_control_environment(environment, plan),
                        "stage",
                        request.map_name,
                        timeout_s=20.0,
                    ),
                    prepared=map_activation,
                    operation="stage",
                )
                map_staged = True
                report.phases.append("map_staged")
            _require_mapctl_transition(
                _mapctl(
                    _map_control_environment(environment, plan),
                    "verify",
                    str(map_activation["activation_token"]),
                    timeout_s=10.0,
                ),
                prepared=map_activation,
                operation="verify",
            )
            report.phases.append("map_verified")
            # The sim readiness contract already proves that slamd completed
            # its launch-time seeded or global localization before commit.
            report.phases.append("localization_initialized")
        try:
            _commit_current(
                plan_path,
                plan,
                environment,
                root,
                product_session_id=product_session_id,
                map_name=request.map_name,
                map_identity=map_identity,
            )
            committed = True
        except _CurrentCommitError as exc:
            committed = exc.current_replaced
            raise
        report.phases.append("committed")
        journal = _advance_switch_journal(
            root,
            journal,
            "current_committed",
        )
        _remove_switch_journal(root, journal)
        journal = None
        if previous is not None and previous.path != plan_path:
            try:
                previous.path.unlink()
            except FileNotFoundError:
                pass
            except OSError:
                # The committed Product no longer references this plan; a stale
                # diagnostic file must not turn a successful switch into rollback.
                pass
        report.product_session_id = product_session_id
        report.ok = True
        report.status = "active"
        return report
    except Exception as exc:
        report.error = str(exc) or exc.__class__.__name__
        if committed:
            report.status = "rollback_failed"
            raise SwitchFailed(report) from exc
        rollback_ok = True
        if map_activation is not None and map_staged:
            try:
                restore_environment = (
                    _map_control_environment(environment, plan)
                    if target_attempted
                    else prepare_map_environment
                )
                if restore_environment is None:
                    raise RuntimeError(
                        "sim saved-map restore has no exact mapd DDS environment"
                    )
                _require_mapctl_transition(
                    _mapctl(
                        restore_environment,
                        "restore",
                        str(map_activation["activation_token"]),
                        timeout_s=20.0,
                    ),
                    prepared=map_activation,
                    operation="restore",
                )
                report.cleanup.append("map:restored")
            except Exception as cleanup_error:
                rollback_ok = False
                report.cleanup.append(f"map_failed:{cleanup_error}")
        target_stopped = not target_attempted
        if target_attempted and plan_path is not None:
            try:
                _require_success(
                    runner.quiesce(
                        plan_path,
                        product_session_id=product_session_id,
                    ),
                    action="quiesce failed target Product",
                )
                report.cleanup.append("target:quiesced")
                target_stopped = True
            except Exception as cleanup_error:
                rollback_ok = False
                report.cleanup.append(f"target_failed:{cleanup_error}")
        if target_stopped and previous is not None and previous_quiesce_attempted:
            try:
                _require_success(
                    runner.apply(
                        previous.path,
                        product_session_id=previous.product_session_id,
                    ),
                    action="restore previous Product",
                )
                report.cleanup.append("previous:restored")
            except Exception as cleanup_error:
                rollback_ok = False
                report.cleanup.append(f"previous_failed:{cleanup_error}")
        if journal is not None and rollback_ok:
            try:
                _remove_switch_journal(
                    state_root,
                    journal,
                )
                journal = None
            except Exception as cleanup_error:
                rollback_ok = False
                report.cleanup.append(f"journal_failed:{cleanup_error}")
        if (
            plan_path is not None
            and plan_created
            and not committed
            and journal is None
            and (previous is None or plan_path != previous.path)
        ):
            try:
                plan_path.unlink(missing_ok=True)
                report.cleanup.append("plan:removed")
            except OSError as cleanup_error:
                rollback_ok = False
                report.cleanup.append(f"plan_failed:{cleanup_error}")
        if previous_quiesce_attempted:
            report.status = "failed_rolled_back" if rollback_ok else "rollback_failed"
        elif target_attempted:
            report.status = "failed_stopped" if rollback_ok else "rollback_failed"
        else:
            report.status = "failed"
        raise SwitchFailed(report) from exc


def _reconcile_incomplete_switch(
    state_root: Path,
    runner: SimSwitchRunner,
    environment: Mapping[str, str],
) -> None:
    """Conservatively settle an interrupted switch before a new mutation."""

    journal = _load_switch_journal(state_root)
    if journal is None:
        return
    target_plan = _load_journal_plan(journal.target)
    try:
        current = _load_committed_plan(state_root, environment)
    except Exception:
        if journal.state in {"target_starting", "current_committed"}:
            _require_success(
                runner.quiesce(
                    journal.target.path,
                    product_session_id=journal.target.product_session_id,
                ),
                action="quiesce interrupted target Product",
            )
        raise
    if _committed_matches(current, journal.target):
        if journal.map_activation is not None:
            _require_mapctl_transition(
                _mapctl(
                    _map_control_environment(environment, target_plan),
                    "verify",
                    str(journal.map_activation["activation_token"]),
                    timeout_s=10.0,
                ),
                prepared=journal.map_activation,
                operation="verify",
            )
        _remove_switch_journal(state_root, journal)
        if journal.previous is not None:
            try:
                journal.previous.path.unlink(missing_ok=True)
            except OSError:
                pass
        return

    if journal.state == "prepared":
        if journal.previous is None:
            if current is not None:
                raise RuntimeError(
                    "simulation switch journal cannot prove the committed RunPlan identity"
                )
        elif not _committed_matches(current, journal.previous):
            raise RuntimeError(
                "simulation switch journal cannot prove the previous RunPlan identity"
            )
        _remove_switch_journal(state_root, journal)
        journal.target.path.unlink(missing_ok=True)
        return

    owned_session = _owned_child_session(state_root)
    if journal.state in {"target_starting", "current_committed"}:
        if owned_session is None or owned_session == journal.target.product_session_id:
            _require_success(
                runner.quiesce(
                    journal.target.path,
                    product_session_id=journal.target.product_session_id,
                ),
                action="quiesce interrupted target Product",
            )
        elif journal.previous is None or owned_session != journal.previous.product_session_id:
            raise RuntimeError(
                "simulation switch child ownership matches neither target nor previous Product"
            )
    if current is None and owned_session is None:
        _remove_switch_journal(state_root, journal)
        for ref in (journal.target, journal.previous):
            if ref is not None:
                ref.path.unlink(missing_ok=True)
        return
    if journal.previous is None:
        if current is not None:
            raise RuntimeError(
                "simulation switch journal cannot prove the committed RunPlan identity"
            )
    else:
        if not _committed_matches(current, journal.previous):
            raise RuntimeError(
                "simulation switch journal cannot prove the previous RunPlan identity"
            )
        if journal.map_activation is not None:
            _require_mapctl_transition(
                _mapctl(
                    _map_control_environment(
                        environment,
                        current.plan if current is not None else target_plan,
                    ),
                    "restore",
                    str(journal.map_activation["activation_token"]),
                    timeout_s=20.0,
                ),
                prepared=journal.map_activation,
                operation="restore",
            )
        _require_success(
            runner.quiesce(
                journal.previous.path,
                product_session_id=journal.previous.product_session_id,
            ),
            action="quiesce restored previous Product",
        )
        _require_success(
            runner.apply(
                journal.previous.path,
                product_session_id=journal.previous.product_session_id,
            ),
            action="restore interrupted previous Product",
        )
    _remove_switch_journal(state_root, journal)
    try:
        journal.target.path.unlink(missing_ok=True)
    except OSError:
        pass


def _owned_child_session(state_root: Path) -> str | None:
    try:
        snapshot = SimChildLedger(state_root).load()
    except SimChildLedgerError as exc:
        raise RuntimeError("simulation switch child ledger is not trusted") from exc
    if snapshot is None or not snapshot.children:
        return None
    return snapshot.product_session_id


def _load_journal_plan(ref: _SwitchPlanRef) -> RunPlan:
    plan = RunPlan.load(ref.path)
    if plan.env != "sim" or plan.process_control != "subprocess":
        raise RuntimeError("simulation switch journal RunPlan is not a sim subprocess plan")
    return plan


def _committed_matches(
    current: _CommittedPlan | None,
    ref: _SwitchPlanRef,
) -> bool:
    return (
        current is not None
        and current.path == ref.path
        and current.product_session_id == ref.product_session_id
    )


def _require_saved_map_previous(
    previous: _CommittedPlan,
    state_root: Path,
) -> None:
    ready_maps = [
        process
        for process in previous.plan.processes
        if "maps" in process.provides
        and process.command is not None
        and process.command.readiness.kind == "file"
        and process.command.readiness.target == "mapd.status.json"
    ]
    if len(ready_maps) != 1:
        raise RuntimeError(
            "sim saved-map switch requires one committed map runtime with typed readiness"
        )
    process = ready_maps[0]
    command = process.command
    if command is None or command.readiness.target is None:  # pragma: no cover
        raise RuntimeError("sim saved-map map runtime readiness is unavailable")
    expectation = readiness_expectation_for_process(
        process.name,
        command.readiness.target,
    )
    if expectation is None:
        raise RuntimeError("sim saved-map map runtime readiness is not typed")
    readiness_path = state_root / command.readiness.target
    deadline = time.monotonic() + 5.0
    while True:
        try:
            load_typed_readiness(
                readiness_path,
                expectation=expectation,
                product_session_id=previous.product_session_id,
                product=previous.plan.product,
                process=process.name,
                started_wall_ns=_exact_map_runtime_start(
                    previous,
                    state_root,
                    process_target=process.target,
                ),
                lidar_required=previous.plan.has_process("lidar"),
                imu_required=previous.plan.has_process("imu"),
                camera_required=previous.plan.has_process("camera"),
            )
        except SimReadinessPending as exc:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "sim saved-map switch requires fresh exact map runtime readiness"
                ) from exc
            time.sleep(0.02)
            continue
        except (OSError, SimReadinessError) as exc:
            raise RuntimeError(
                "sim saved-map switch requires fresh exact map runtime readiness"
            ) from exc
        break


def _exact_map_runtime_start(
    previous: _CommittedPlan,
    state_root: Path,
    *,
    process_target: str,
) -> int:
    try:
        snapshot = SimChildLedger(state_root).load()
    except SimChildLedgerError as exc:
        raise RuntimeError("sim saved-map child ledger is not trusted") from exc
    if snapshot is None or snapshot.product_session_id != previous.product_session_id:
        raise RuntimeError("sim saved-map child ledger session identity is invalid")
    records = tuple(
        record for record in snapshot.children if record.target == process_target
    )
    if len(records) != 1:
        raise RuntimeError("sim saved-map child ledger map runtime is missing")
    record = records[0]
    if not record.process_identity.matches():
        raise RuntimeError("sim saved-map map runtime child is not live")
    return int(record.started_wall_ns)


def _map_control_environment(
    environment: Mapping[str, str],
    plan: RunPlan,
) -> dict[str, str]:
    map_processes = tuple(
        process
        for process in plan.processes
        if "maps" in process.provides and process.command is not None
    )
    if len(map_processes) != 1:
        raise RuntimeError(
            "sim saved-map control requires one exact map runtime in the RunPlan"
        )
    command = map_processes[0].command
    if command is None:  # pragma: no cover - filtered above.
        raise RuntimeError("sim saved-map map runtime command is unavailable")
    command_environment = dict(command.env)
    domain_id = str(command_environment.get("LINGTU_DDS_DOMAIN_ID") or "").strip()
    if not domain_id:
        raise RuntimeError("sim saved-map map runtime DDS domain is unavailable")
    resolved = dict(environment)
    resolved.update(command_environment)
    resolved["LINGTU_DDS_DOMAIN_ID"] = domain_id
    return resolved


def _prepared_map_identity(
    prepared: Mapping[str, Any],
    map_name: str | None,
) -> MapIdentity:
    identity = map_identity_from_native(prepared.get("target"), field_name="prepared target map")
    if map_name is None or identity.map_id != map_name:
        raise RuntimeError("native map prepare did not resolve the exact requested map")
    previous = optional_map_identity_from_native(
        prepared.get("previous"),
        field_name="prepared previous map",
    )
    active = optional_map_identity_from_native(
        prepared.get("active"),
        field_name="prepared active map",
    )
    if active != previous:
        raise RuntimeError("native map prepare mutated or misreported the active map")
    return identity


def _saved_map_environment(
    plan: RunPlan,
    identity: MapIdentity,
    *,
    initial_pose: tuple[float, float, float] | None = None,
) -> dict[str, str]:
    environment = plan.native_process_environment
    for key in (
        "LINGTU_SLAM_TRACK_INITIAL_X",
        "LINGTU_SLAM_TRACK_INITIAL_Y",
        "LINGTU_SLAM_TRACK_INITIAL_Z",
        "LINGTU_SLAM_TRACK_INITIAL_YAW",
    ):
        environment.pop(key, None)
    environment.update(map_identity_environment(identity))
    slam_map_path = pointcloud_artifact(identity).uri
    planner = environment.get("NAV_GLOBAL_PLANNER")
    if planner == "octoplanner3d":
        environment["OCTOPLANNER_MAP_PATH"] = octomap_artifact(identity).uri
        environment["FAR_OCCUPANCY_PATH"] = ""
    elif planner == "far":
        environment["OCTOPLANNER_MAP_PATH"] = ""
        environment["FAR_OCCUPANCY_PATH"] = occupancy_artifact(identity).uri
    else:
        raise RuntimeError(f"unsupported native global planner: {planner}")
    environment["LINGTU_SLAM_MODE"] = "localization"
    environment["LINGTU_SLAM_MAP"] = slam_map_path
    environment["EXPLORE_OCCUPANCY_PATH"] = (
        occupancy_artifact(identity).uri if plan.product == "explore" else ""
    )
    if initial_pose is not None:
        x, y, yaw = initial_pose
        environment.update(
            {
                "LINGTU_SLAM_TRACK_INITIAL_X": f"{x:.17g}",
                "LINGTU_SLAM_TRACK_INITIAL_Y": f"{y:.17g}",
                "LINGTU_SLAM_TRACK_INITIAL_Z": "0",
                "LINGTU_SLAM_TRACK_INITIAL_YAW": f"{yaw:.17g}",
            }
        )
    if plan.product == "explore":
        environment["LINGTU_EXPLORE_ROUTE"] = "map"
    return dict(sorted(environment.items()))


def _mapctl(
    environment: Mapping[str, str],
    operation: str,
    operand: str,
    *,
    timeout_s: float,
) -> dict[str, Any]:
    default_binary = (
        _REPOSITORY_ROOT
        / "build"
        / "maps-windows"
        / "Release"
        / "lingtu-mapctl.exe"
        if os.name == "nt"
        else Path("/opt/lingtu/current/build/maps/lingtu-mapctl")
    )
    binary = str(environment.get("LINGTU_MAPCTL_BIN") or default_binary)
    home = Path(environment.get("HOME") or Path.home())
    map_root = Path(
        environment.get("NAV_MAP_DIR")
        or home / "data" / "lingtu" / "maps"
    ).expanduser().resolve()
    command = [
        binary,
        operation,
        operand,
        "--map-root",
        str(map_root),
        "--caller",
        "product-control",
        "--timeout-ms",
        str(max(1, round(timeout_s * 1000))),
    ]
    domain_id = str(environment.get("LINGTU_DDS_DOMAIN_ID") or "").strip()
    if not domain_id:
        raise RuntimeError("native sim map control requires the exact mapd DDS domain")
    command.extend(("--domain-id", domain_id))
    completed = subprocess.run(  # noqa: S603 - executable is an explicit native runtime path.
        command,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout_s + 1.0,
        env={**os.environ, **environment},
    )
    try:
        payload = json.loads(str(completed.stdout or ""))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError("native sim map control returned invalid JSON") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("native sim map control response must be an object")
    if (
        completed.returncode != 0
        or payload.get("accepted") is not True
    ):
        message = str(payload.get("message") or completed.stderr or "rejected")
        raise RuntimeError(f"native sim map {operation} rejected: {message}")
    _validate_mapctl_receipt(
        payload,
        operation=operation,
    )
    return payload


def _validate_mapctl_receipt(
    payload: Mapping[str, Any],
    *,
    operation: str,
) -> None:
    if type(payload) is not dict:
        raise RuntimeError("native sim map control response is invalid")
    if (
        payload.get("schema_version") != MAP_ACTIVATION_TOKEN_SCHEMA
        or payload.get("operation") != operation
        or payload.get("accepted") is not True
    ):
        raise RuntimeError("native sim map control response identity is invalid")
    if type(payload.get("changed")) is not bool:
        raise RuntimeError("native sim map control changed is invalid")
    producer_boot_id = payload.get("producer_boot_id")
    if (
        not isinstance(producer_boot_id, str)
        or producer_boot_id != producer_boot_id.strip()
        or (operation != "prepare" and not producer_boot_id)
    ):
        raise RuntimeError("native sim map control producer_boot_id is invalid")
    if operation == "prepare" and (
        payload.get("changed") is not False or producer_boot_id != ""
    ):
        raise RuntimeError(
            "native sim map prepare must be read-only and have no producer boot identity"
        )
    activation_token = payload.get("activation_token")
    if operation in {"prepare", "stage"}:
        if (
            not isinstance(activation_token, str)
            or not activation_token
            or activation_token != activation_token.strip()
        ):
            raise RuntimeError(f"native sim map {operation} activation_token is invalid")
    elif activation_token != "":
        raise RuntimeError(f"native sim map {operation} activation_token is invalid")
    if operation == "verify" and payload.get("changed") is not False:
        raise RuntimeError("native sim map verify must not report a mutation")
    target = map_identity_from_native(payload.get("target"), field_name=f"{operation} target map")
    previous = optional_map_identity_from_native(
        payload.get("previous"),
        field_name=f"{operation} previous map",
    )
    active = optional_map_identity_from_native(
        payload.get("active"),
        field_name=f"{operation} active map",
    )
    expected_active = previous if operation in {"prepare", "restore"} else target
    if active != expected_active:
        raise RuntimeError(f"native sim map {operation} active identity is invalid")


def _require_mapctl_transition(
    response: Mapping[str, Any],
    *,
    prepared: Mapping[str, Any],
    operation: str,
) -> None:
    if (
        operation == "stage"
        and response.get("activation_token") != prepared.get("activation_token")
    ):
        raise RuntimeError("native map stage activation token drifted")
    if operation == "verify" and response.get("changed") is not False:
        raise RuntimeError("native sim map verify must not report a mutation")
    if response.get("target") != prepared.get("target"):
        raise RuntimeError(f"native map {operation} target identity drifted")
    if response.get("previous") != prepared.get("previous"):
        raise RuntimeError(f"native map {operation} previous identity drifted")
    expected_active = (
        prepared.get("previous") if operation == "restore" else prepared.get("target")
    )
    if response.get("active") != expected_active:
        raise RuntimeError(f"native map {operation} active identity is invalid")


def _validate_target(
    plan: RunPlan,
    request: SwitchRequest,
) -> None:
    if plan.product != request.target_product:
        raise RuntimeError("resolved RunPlan Product does not match switch request")
    if plan.product_variant != request.product_variant:
        raise RuntimeError("resolved RunPlan variant does not match switch request")
    if plan.env != "sim":
        raise RuntimeError("resolved RunPlan does not belong to Env sim")
    if plan.process_control != "subprocess":
        raise RuntimeError(
            f"Product {plan.product} is controlled by {plan.process_control}, not subprocess"
        )
    if plan.lifecycle.get("requires_map") is True and request.map_name is None:
        raise RuntimeError(f"Product {plan.product} requires a map")


def _validate_sim_request(request: SwitchRequest) -> None:
    if request.map_name is not None:
        map_name = request.map_name
        target_product = product_name(str(request.target_product).strip())
        allowed = target_product in {"nav", "tracking", "inspection"} or (
            target_product == "explore" and request.product_variant == "map"
        )
        if (
            not allowed
            or not isinstance(map_name, str)
            or not map_name
            or map_name != map_name.strip()
        ):
            raise RuntimeError("sim saved-map switch does not accept this map_name route")
    if request.initial_pose is not None:
        if request.map_name is None:
            raise RuntimeError("sim initial_pose requires a saved-map switch")
        if not request.relocalize:
            raise RuntimeError("sim initial_pose requires relocalize=True")
        _sim_initial_pose(request.initial_pose)
def _sim_initial_pose(
    value: tuple[float, float, float] | None,
) -> tuple[float, float, float] | None:
    if value is None:
        return None
    if not isinstance(value, tuple) or len(value) != 3:
        raise RuntimeError("sim initial_pose must contain x, y, yaw")
    try:
        pose = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError("sim initial_pose must contain finite numbers") from exc
    if any(not (-float("inf") < item < float("inf")) for item in pose):
        raise RuntimeError("sim initial_pose must contain finite numbers")
    x, y, yaw = pose
    return x, y, yaw


def _load_committed_plan(
    state_dir: Path,
    environment: Mapping[str, str],
) -> _CommittedPlan | None:
    current_path = _sim_current_path(state_dir, environment)
    try:
        raw = current_path.read_bytes()
    except FileNotFoundError:
        return None
    except OSError as exc:
        raise RuntimeError(f"current run record is unavailable: {current_path}") from exc
    try:
        if len(raw) > 64 * 1024:
            raise ValueError("current run record exceeds the byte limit")
        payload = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        raise RuntimeError(f"current run record is invalid: {current_path}: {exc}") from exc
    if not isinstance(payload, Mapping):
        raise RuntimeError(f"current run record must be a JSON object: {current_path}")
    if payload.get("schema_version") != CURRENT_RUN_SCHEMA:
        raise RuntimeError("current run record has unsupported schema")
    product_session_id = _exact_text(
        payload.get("product_session_id"),
        "product_session_id",
    )
    if not is_product_session_id(product_session_id):
        raise RuntimeError("current run record product_session_id is invalid")
    raw_plan_path = _exact_text(payload.get("run_plan_path"), "run_plan_path")
    expected_path = state_dir / f"plan-{product_session_id}.json"
    if raw_plan_path != str(expected_path):
        raise RuntimeError("current RunPlan path must match its Product session")
    plan_path = expected_path
    plan = RunPlan.load(plan_path)
    if plan.env != "sim" or plan.process_control != "subprocess":
        raise RuntimeError("current sim RunPlan is not controlled by subprocess")
    if _exact_text(payload.get("product"), "product") != plan.product:
        raise RuntimeError("current Product does not match RunPlan")
    recorded_variant = payload.get("product_variant")
    if recorded_variant is not None:
        if not isinstance(recorded_variant, str) or not recorded_variant.strip():
            raise RuntimeError("current Product variant is invalid")
        recorded_variant = recorded_variant.strip()
    if recorded_variant != plan.product_variant:
        raise RuntimeError("current Product variant does not match RunPlan")
    if _exact_text(payload.get("env"), "env") != plan.env:
        raise RuntimeError("current Env does not match RunPlan")
    recorded_map_name = payload.get("map_name")
    recorded_map_identity = payload.get("map_identity")
    if recorded_map_name is None:
        if recorded_map_identity is not None:
            raise RuntimeError("current run record map identity requires map_name")
    else:
        recorded_map_name = _exact_text(recorded_map_name, "map_name")
        identity = map_identity_from_record(
            recorded_map_identity,
            field_name="current run record map identity",
        )
        if identity.map_id != recorded_map_name:
            raise RuntimeError("current run record map identity does not match map_name")
    return _CommittedPlan(
        path=plan_path,
        plan=plan,
        product_session_id=product_session_id,
    )


def _exact_text(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise RuntimeError(f"current run record requires exact {field}")
    return value


def _require_success(report: ProcessReport, *, action: str) -> None:
    if not isinstance(report, ProcessReport):
        raise RuntimeError(f"{action} returned an invalid process report")
    if not report.ok:
        raise RuntimeError(report.error or f"{action} failed")

def _commit_current(
    plan_path: Path,
    plan: RunPlan,
    environment: Mapping[str, str],
    state_dir: Path,
    *,
    product_session_id: str,
    map_name: str | None = None,
    map_identity: MapIdentity | None = None,
) -> None:
    current_path = _sim_current_path(state_dir, environment)
    temp_path = current_path.with_name(
        f".{current_path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp"
    )
    payload: dict[str, Any] = {
        "schema_version": CURRENT_RUN_SCHEMA,
        "product": plan.product,
        "product_variant": plan.product_variant,
        "env": plan.env,
        "run_plan_path": str(plan_path),
        "product_session_id": product_session_id,
        "map_name": map_name,
        "map_identity": map_identity_as_record(map_identity),
        "committed_at": time.time(),
    }
    raw = (
        json.dumps(
            payload,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        + "\n"
    ).encode("utf-8")
    current_replaced = False
    try:
        with temp_path.open("xb") as stream:
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, current_path)
        current_replaced = True
        _fsync_parent(current_path)
    except OSError as exc:
        raise _CurrentCommitError(
            "current run record cannot be committed durably",
            current_replaced=current_replaced,
        ) from exc
    finally:
        temp_path.unlink(missing_ok=True)


def _fsync_parent(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(
        path.parent,
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _sim_current_path(
    state_dir: Path,
    environment: Mapping[str, str],
) -> Path:
    configured = environment.get(CURRENT_RUN_FILE_ENV)
    if configured is not None:
        if not isinstance(configured, str) or configured.strip():
            raise RuntimeError(
                f"Env sim does not accept {CURRENT_RUN_FILE_ENV}"
            )
    return state_dir / str(CURRENT_RUN_FILE_NAME)


__all__ = ["SimSwitchRunner"]
