"""Direct child-process ownership for simulation."""

from __future__ import annotations

import math
import os
import secrets
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Mapping
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import IO, Any, BinaryIO, cast

from lingtu.run_plan import RunPlan
from lingtu.sim.identity import (
    ProcessIdentity,
    ProcessIdentityError,
    SimChildLedger,
    SimChildLedgerError,
    SimChildRecord,
    SimChildSnapshot,
)
from lingtu.sim.readiness import (
    SimReadinessError,
    SimReadinessPending,
    load_typed_readiness,
    readiness_expectation_for_process,
)
from lingtu.sim.stop import (
    MOTION_STOP_SCHEMA,
    PROCESS_LAUNCH_ID_ENV,
    SimStopEvidenceError,
    load_motion_stop_evidence,
)
from lingtu.switch_contracts import ProcessError, ProcessFailed, ProcessReport, is_product_session_id
from runtime.graph import ProcessSpec

_WINDOWS_BOOTSTRAP_ENV = frozenset(
    {
        "COMSPEC",
        "SYSTEMDRIVE",
        "SYSTEMROOT",
        "TEMP",
        "TMP",
        "NAV_MAP_DIR",
        "USERPROFILE",
        "WINDIR",
    }
)

_PROCESS_LOG_LIMIT_BYTES = 128 * 1024
_PROCESS_LOG_READ_BYTES = 16 * 1024


def _repository_path(
    repository_root: Path,
    value: str,
    *,
    label: str,
    require_file: bool,
) -> Path:
    candidate = repository_root / PurePosixPath(value)
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(repository_root)
    except (OSError, ValueError) as exc:
        raise ProcessError(f"direct process {label} is not repository-safe") from exc
    if require_file and not resolved.is_file():
        raise ProcessError(f"direct process {label} must be a file")
    if not require_file and not resolved.is_dir():
        raise ProcessError(f"direct process {label} must be a directory")
    return resolved


@dataclass
class _BoundedStreamCapture:
    stream: IO[bytes]
    output: BinaryIO
    path: Path
    thread: threading.Thread | None = None

    def start(self, *, thread_name: str) -> None:
        thread = threading.Thread(
            target=self._capture,
            name=thread_name,
            daemon=False,
        )
        self.thread = thread
        thread.start()

    def close(self) -> None:
        thread = self.thread
        if thread is None:
            self._close_handles()
            return
        thread.join(timeout=2.0)
        if thread.is_alive():
            try:
                self.stream.close()
            except OSError:
                pass
            thread.join(timeout=2.0)
        if thread.is_alive():
            raise ProcessError(f"direct process log capture did not stop: {self.path}")

    def _capture(self) -> None:
        tail = bytearray()
        try:
            while True:
                chunk = self.stream.read(_PROCESS_LOG_READ_BYTES)
                if not chunk:
                    break
                tail.extend(chunk)
                if len(tail) > _PROCESS_LOG_LIMIT_BYTES:
                    del tail[: len(tail) - _PROCESS_LOG_LIMIT_BYTES]
                self.output.seek(0)
                self.output.write(tail)
                self.output.truncate()
                self.output.flush()
        except (OSError, ValueError):
            # The owning process transaction still reports the stable log path.
            # Child output is diagnostic only and never controls readiness.
            pass
        finally:
            self._close_handles()

    def _close_handles(self) -> None:
        for handle in (self.stream, self.output):
            try:
                handle.close()
            except OSError:
                pass


@dataclass
class _ProcessLogCapture:
    stdout: _BoundedStreamCapture
    stderr: _BoundedStreamCapture

    def start(self, *, process_name: str) -> None:
        self.stdout.start(thread_name=f"lingtu-log-{process_name}-stdout")
        self.stderr.start(thread_name=f"lingtu-log-{process_name}-stderr")

    def close(self) -> None:
        errors: list[ProcessError] = []
        for capture in (self.stdout, self.stderr):
            try:
                capture.close()
            except ProcessError as exc:
                errors.append(exc)
        if errors:
            raise errors[0]


@dataclass(frozen=True)
class _BoundProcess:
    process: ProcessSpec
    artifact: Path
    cwd: Path
    argv: tuple[str, ...]
    environment: Mapping[str, str]


@dataclass
class _OwnedProcess:
    child: subprocess.Popen[bytes] | None
    identity: ProcessIdentity
    started_wall_ns: int
    process_group: int
    launch_id: str
    job_handle: int | None = None
    stdout_log: Path | None = None
    stderr_log: Path | None = None
    log_capture: _ProcessLogCapture | None = None


class SimProcessManager:
    """Own direct child processes declared by one immutable simulation RunPlan.

    ``bind`` consumes an already loaded plan and its exact published path.  The
    manager never loads or resolves RuntimeGraph configuration.
    """

    def __init__(self, repository_root: str | os.PathLike[str]) -> None:
        supplied_root = Path(repository_root)
        try:
            root = supplied_root.resolve(strict=True)
        except OSError as exc:
            raise ProcessError("simulation repository root is unavailable") from exc
        if not root.is_dir():
            raise ProcessError("simulation repository root must be a directory")
        self._repository_root = root
        self._bound: dict[str, _BoundProcess] = {}
        self._bound_plan: RunPlan | None = None
        self._product_session_id: str | None = None
        self._children: dict[str, _OwnedProcess] = {}
        self._session_root: Path | None = None
        self._ledger: SimChildLedger | None = None

    def bind(
        self,
        plan: RunPlan,
        *,
        run_plan_path: str | os.PathLike[str],
        product_session_id: str,
    ) -> None:
        """Bind exact plan identity and direct commands before any mutation."""

        if not isinstance(plan, RunPlan):
            raise ProcessError("simulation process manager requires a typed RunPlan")
        if not is_product_session_id(product_session_id):
            raise ProcessError("simulation product_session_id is invalid")
        if plan.env != "sim" or plan.process_control != "subprocess":
            raise ProcessError("simulation process manager requires env='sim' and controller='subprocess'")
        for target in tuple(self._children):
            if self.active(target):
                raise ProcessError("cannot replace a RunPlan while owned processes are active")
        if self._children:
            raise ProcessError("cannot replace a RunPlan while process ownership is foreign")

        plan_path = Path(run_plan_path)
        identity = {
            "LINGTU_ENV": plan.env,
            "LINGTU_HOST_BOOT_ID": product_session_id,
            "LINGTU_RUN_PLAN": str(plan_path),
            "LINGTU_PRODUCT": plan.product,
            "LINGTU_PRODUCT_SESSION_ID": product_session_id,
            "LINGTU_SESSION_ROOT": str(plan_path.parent),
        }
        bound: dict[str, _BoundProcess] = {}
        for process in plan.available_processes:
            if process.manager != "direct" or process.command is None:
                raise ProcessError(f"simulation process is not a typed direct process: {process.name}")
            command = process.command
            command_environment = dict(command.env)
            artifact = _repository_path(
                self._repository_root,
                command.artifact.path,
                label="artifact",
                require_file=True,
            )
            cwd = _repository_path(
                self._repository_root,
                command.cwd,
                label="cwd",
                require_file=False,
            )
            argv = list(command.argv)
            if argv[0] == command.artifact.path:
                argv[0] = str(artifact)
            else:
                argv[0] = sys.executable
                argv[1] = str(artifact)
            environment = self._bootstrap_environment()
            environment.update(plan.native_process_environment)
            environment.update(command_environment)
            environment.update(identity)
            if process.target in bound:
                raise ProcessError(f"duplicate simulation process target: {process.target}")
            bound[process.target] = _BoundProcess(
                process=process,
                artifact=artifact,
                cwd=cwd,
                argv=tuple(argv),
                environment=environment,
            )
        if not bound:
            raise ProcessError("simulation RunPlan has no available direct processes")
        ledger = SimChildLedger(plan_path.parent)
        try:
            snapshot = ledger.load()
            adopted = self._adopt_snapshot(
                snapshot,
                ledger=ledger,
                plan_path=plan_path,
                product_session_id=product_session_id,
                bound=bound,
            )
        except SimChildLedgerError as exc:
            raise ProcessError("simulation child ledger is invalid") from exc
        self._bound = bound
        self._bound_plan = plan
        self._product_session_id = product_session_id
        self._children = adopted
        self._session_root = plan_path.parent
        self._ledger = ledger

    def assert_bound(self, plan: RunPlan) -> None:
        """Require *plan* to be the complete RunPlan bound by ``bind``."""

        if not isinstance(plan, RunPlan) or self._bound_plan != plan:
            raise ProcessError("RunPlan does not match the bound RunPlan")

    def apply(self, plan: RunPlan) -> ProcessReport:
        """Start the bound simulation processes in declared order."""

        self.assert_bound(plan)
        processes = self._ordered_processes(plan)
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            planned=[process.target for process in processes],
        )

        started: list[ProcessSpec] = []
        try:
            by_target = {process.target: process for process in plan.available_processes}
            self._stop_processes(
                tuple(
                    by_target[target]
                    for target in plan.stop_before_start
                    if target in by_target
                ),
                report,
                completed=report.stopped,
                continue_on_error=False,
            )

            for stage in self._process_stages(processes):
                stage_started = time.monotonic()
                stage_deadline = stage_started + max(
                    float(process.timeout_s) for process in stage
                )
                for process in stage:
                    if self.active(process.target):
                        if process.lifecycle != "persistent":
                            raise ProcessError(
                                "mode process remained active after conflict stop: "
                                f"{process.target}"
                            )
                        report.preserved.append(process.target)
                        continue
                    self.start(
                        process.target,
                        self._remaining(stage_deadline),
                    )
                    started.append(process)
                    report.started.append(process.target)
                for process in stage:
                    process_deadline = stage_started + float(process.timeout_s)
                    report.ready[process.name] = dict(
                        self.wait(
                            process,
                            self._remaining(min(stage_deadline, process_deadline)),
                        )
                    )
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            failures = self._stop_processes(
                self._stop_order(plan, tuple(started)),
                report,
                completed=report.rolled_back,
                continue_on_error=True,
            )
            report.rollback_errors.extend(
                f"{target}: {error}" for target, error in failures
            )
            raise ProcessFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def quiesce(self, plan: RunPlan) -> ProcessReport:
        """Stop every owned process named by the plan's stop contract."""

        self.assert_bound(plan)
        by_target = {process.target: process for process in plan.available_processes}
        return self._stop_report(
            plan,
            action="quiesce",
            processes=tuple(
                by_target[target]
                for target in plan.stop_before_start
                if target in by_target
            ),
            planned=list(plan.stop_before_start),
        )

    def stop_plan(self, plan: RunPlan) -> ProcessReport:
        """Stop every owned process managed by the bound plan."""

        self.assert_bound(plan)
        return self._stop_report(
            plan,
            action="stop",
            processes=self._stop_order(plan, tuple(plan.managed_processes)),
        )

    def _stop_report(
        self,
        plan: RunPlan,
        *,
        action: str,
        processes: tuple[ProcessSpec, ...],
        planned: list[str] | None = None,
    ) -> ProcessReport:
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action=action,
            planned=(
                planned
                if planned is not None
                else [process.target for process in processes]
            ),
        )
        failures = self._stop_processes(
            processes,
            report,
            completed=report.stopped,
            continue_on_error=True,
        )
        if failures:
            report.error = f"failed to {action} Product processes: " + "; ".join(
                f"{target}: {error}" for target, error in failures
            )
            report.status = "failed"
            raise ProcessFailed(report) from failures[0][1]
        report.ok = True
        report.status = "stopped"
        return report

    def _stop_processes(
        self,
        processes: tuple[ProcessSpec, ...],
        report: ProcessReport,
        *,
        completed: list[str],
        continue_on_error: bool,
    ) -> list[tuple[str, Exception]]:
        failures: list[tuple[str, Exception]] = []
        for process in processes:
            if not self.owns(process.target):
                continue
            try:
                evidence = self.stop_process(process, float(process.timeout_s))
                if evidence:
                    report.stop_evidence[process.name] = dict(evidence)
                completed.append(process.target)
            except Exception as exc:
                if not continue_on_error:
                    raise
                failures.append((process.target, exc))
        return failures

    @staticmethod
    def _ordered_processes(plan: RunPlan) -> tuple[ProcessSpec, ...]:
        processes = tuple(
            sorted(
                plan.processes,
                key=lambda item: (item.order, not item.provides, item.name),
            )
        )
        if not processes:
            raise ProcessError(
                f"Product {plan.product} has no deployment processes in Env {plan.env}"
            )
        unsupported = sorted(
            {process.manager for process in processes if process.manager != "direct"}
        )
        if unsupported:
            raise ProcessError(
                f"unsupported process managers: {', '.join(unsupported)}"
            )
        return processes

    @staticmethod
    def _process_stages(
        processes: tuple[ProcessSpec, ...],
    ) -> tuple[tuple[ProcessSpec, ...], ...]:
        stages: list[list[ProcessSpec]] = []
        for process in processes:
            if not stages or stages[-1][0].order != process.order:
                stages.append([])
            stages[-1].append(process)
        return tuple(tuple(stage) for stage in stages)

    @staticmethod
    def _stop_order(
        plan: RunPlan,
        processes: tuple[ProcessSpec, ...],
    ) -> tuple[ProcessSpec, ...]:
        by_target = {process.target: process for process in processes}
        ordered = [
            by_target[target] for target in plan.stop_before_start if target in by_target
        ]
        declared = {process.target for process in ordered}
        ordered.extend(
            process
            for process in reversed(processes)
            if process.target not in declared
        )
        return tuple(ordered)

    @staticmethod
    def _remaining(deadline: float) -> float:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ProcessError("process stage deadline expired")
        return remaining

    def active(self, target: str) -> bool:
        """Return whether the exact child owned for *target* is still alive."""

        owned = self._children.get(target)
        if owned is None:
            return False
        bound = self._bound.get(target)
        requires_stop_evidence = bound is not None and self._process_requires_file_stop_evidence(
            bound.process
        )
        if self._child_exit_code(owned) is not None:
            if not requires_stop_evidence:
                if bound is not None:
                    self._retire_readiness(bound.process)
                self._release_owned(target, owned)
            return False
        if not self._owned_alive(owned):
            if not requires_stop_evidence and owned.child is None:
                if bound is not None:
                    self._retire_readiness(bound.process)
                self._release_owned(target, owned)
            return False
        return True

    def owns(self, target: str) -> bool:
        """Return whether an exact child identity is retained for *target*.

        Ownership intentionally outlives process liveness when file-backed stop
        evidence is still required. It is released only after that evidence has
        been validated or after an evidence-free process has been reaped.
        """

        return target in self._children

    @staticmethod
    def _process_requires_file_stop_evidence(process: ProcessSpec) -> bool:
        if process.command is None:
            return False
        shutdown = process.command.shutdown
        return shutdown is not None and shutdown.kind == "file"

    def start(self, target: str, timeout_s: float) -> None:
        """Start one bound target without shell interpretation."""

        self._timeout(timeout_s)
        bound = self._bound.get(target)
        if bound is None:
            raise ProcessError(f"required direct process is not available: {target}")
        if self.active(target):
            raise ProcessError(f"direct process is already active: {target}")
        if target in self._children:
            raise ProcessError(f"direct process ownership identity is foreign: {target}")
        command = bound.process.command
        for dependency in command.dependencies:
            _repository_path(
                self._repository_root,
                dependency.path,
                label="dependency",
                require_file=True,
            )
        self._retire_readiness(bound.process)
        started_wall_ns = time.time_ns()
        launch_id = secrets.token_hex(16)
        child_environment = dict(bound.environment)
        child_environment[PROCESS_LAUNCH_ID_ENV] = launch_id
        child: subprocess.Popen[bytes] | None = None
        job_handle: int | None = None
        log_capture: _ProcessLogCapture | None = None
        stdout_log, stderr_log = self._log_paths(bound.process.name)
        try:
            child = subprocess.Popen(  # noqa: S603
                bound.argv,
                cwd=bound.cwd,
                env=child_environment,
                shell=False,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=os.name != "nt",
                creationflags=(subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0),
            )
            if os.name == "nt":
                job_handle = self._create_windows_job(child)
            log_capture = self._start_log_capture(
                child,
                process_name=bound.process.name,
                stdout_log=stdout_log,
                stderr_log=stderr_log,
            )
            identity = ProcessIdentity.current(child.pid)
        except Exception as exc:
            if job_handle is not None:
                with suppress(Exception):
                    self._terminate_windows_job(job_handle)
                with suppress(Exception):
                    self._close_windows_handle(job_handle)
            if child is not None and child.poll() is None:
                try:
                    if os.name == "nt":
                        child.kill()
                    else:
                        kill_process_group = cast(
                            Callable[[int, int], None],
                            vars(os)["killpg"],
                        )
                        kill_process_group(child.pid, signal.SIGKILL)
                except Exception:
                    with suppress(Exception):
                        child.kill()
                with suppress(Exception):
                    child.wait(timeout=1)
            if log_capture is not None:
                with suppress(Exception):
                    log_capture.close()
            if child is not None:
                for pipe in (child.stdout, child.stderr):
                    if pipe is None:
                        continue
                    try:
                        pipe.close()
                    except OSError:
                        pass
            if isinstance(exc, ProcessIdentityError):
                raise ProcessError("cannot read direct process start identity") from exc
            if isinstance(exc, ProcessError):
                raise
            raise ProcessError(f"failed to start direct process: {target}") from exc
        owned = _OwnedProcess(
            child=child,
            identity=identity,
            started_wall_ns=started_wall_ns,
            process_group=child.pid,
            launch_id=launch_id,
            job_handle=job_handle,
            stdout_log=stdout_log,
            stderr_log=stderr_log,
            log_capture=log_capture,
        )
        self._children[target] = owned
        try:
            # This is intentionally the first operation after OS identity/job
            # capture. A hard supervisor crash inside this unavoidable spawn-to-
            # ledger window cannot be recovered; every ordinary publication
            # failure below synchronously reclaims the just-spawned child.
            self._persist_children()
        except ProcessError:
            self._cleanup_unpublished_child(target, owned)
            raise

    def stop_process(
        self,
        process: ProcessSpec,
        timeout_s: float,
    ) -> Mapping[str, Any]:
        """Stop one exact typed process and return validated stop evidence."""

        if process.manager != "direct":
            raise ProcessError(f"unsupported process manager: {process.manager}")
        shutdown = process.command.shutdown if process.command is not None else None
        if shutdown is not None and shutdown.kind == "file":
            if shutdown.schema != MOTION_STOP_SCHEMA:
                raise ProcessError("unsupported direct process shutdown evidence schema")
            owned = self._children.get(process.target)
            if owned is None:
                raise ProcessError("motion stop evidence requires an owned process")
            if owned.identity.matches():
                if not self._process_group_matches(owned):
                    raise ProcessError(
                        "direct process ownership identity is foreign: "
                        f"{process.target}"
                    )
                self._stop_owned_target(
                    process.target,
                    timeout_s,
                    process=process,
                    allow_already_inactive=True,
                    release_owned=False,
                )
            if self._session_root is None:  # pragma: no cover - bind invariant
                raise ProcessError("simulation process manager is not bound")
            try:
                plan = self._bound_plan
                if plan is None:  # pragma: no cover - bind invariant
                    raise ProcessError("simulation process manager is not bound")
                evidence = load_motion_stop_evidence(
                    session_root=self._session_root,
                    target=shutdown.target,
                    product_session_id=self._require_product_session_id(),
                    product=plan.product,
                    process=process.name,
                    launch_id=owned.launch_id,
                )
            except SimStopEvidenceError:
                self._retire_readiness(process)
                self._release_owned(process.target, owned)
                return {
                    "schema": "lingtu.sim.process_stop.v1",
                    "process": process.name,
                    "target": process.target,
                    "outcome": "inactive_without_motion_evidence",
                    "process_identity": owned.identity.as_dict(),
                    "graceful": False,
                    "forced": False,
                    "inactive": True,
                    "motion_stop_confirmed": False,
                }
            self._retire_readiness(process)
            self._release_owned(process.target, owned)
            return cast(Mapping[str, Any], evidence)
        return self._stop_owned_target(
            process.target,
            timeout_s,
            process=process,
            allow_already_inactive=True,
        )

    def _stop_owned_target(
        self,
        target: str,
        timeout_s: float,
        *,
        process: ProcessSpec | None = None,
        allow_already_inactive: bool,
        release_owned: bool = True,
    ) -> Mapping[str, Any]:
        """Stop an owned target, cleaning forced exits but reporting failure."""

        timeout = self._timeout(timeout_s)
        owned = self._children.get(target)
        if owned is None:
            if allow_already_inactive and process is not None:
                return {
                    "schema": "lingtu.sim.process_stop.v1",
                    "process": process.name,
                    "target": process.target,
                    "outcome": "already_inactive",
                    "graceful": False,
                    "forced": False,
                    "inactive": True,
                }
            raise ProcessError(f"direct process is not owned: {target}")
        inactive_owned = owned.child is not None and owned.child.poll() is not None
        if not inactive_owned and owned.child is None:
            inactive_owned = not self._owned_alive(owned)
        if inactive_owned:
            if release_owned:
                bound = self._bound.get(target)
                if bound is not None:
                    self._retire_readiness(bound.process)
                self._release_owned(target, owned)
            if allow_already_inactive and process is not None:
                evidence = {
                    "schema": "lingtu.sim.process_stop.v1",
                    "process": process.name,
                    "target": process.target,
                    "outcome": "already_inactive",
                    "graceful": False,
                    "forced": False,
                    "inactive": True,
                }
                evidence["process_identity"] = owned.identity.as_dict()
                return evidence
            raise ProcessError(f"direct process ownership is stale: {target}")
        if not self._owned_alive(owned):
            raise ProcessError(f"direct process ownership identity is foreign: {target}")

        deadline = time.monotonic() + timeout
        forced_budget = min(0.1, timeout / 4.0)
        graceful_deadline = deadline - forced_budget
        self._graceful_stop_group(owned)
        self._wait_for_exit(owned, graceful_deadline)
        forced = False
        if self._owned_alive(owned):
            forced = True
            self._force_stop_group(owned)
            self._wait_for_exit(owned, deadline)
        if self._owned_alive(owned):
            raise ProcessError(f"direct process group did not stop before deadline: {target}")
        if release_owned:
            bound = self._bound.get(target)
            if bound is not None:
                self._retire_readiness(bound.process)
            self._release_owned(target, owned)
        evidence = {
            "schema": "lingtu.sim.process_stop.v1",
            "process": process.name if process is not None else target,
            "target": target,
            "outcome": "forced_exit" if forced else "graceful_exit",
            "process_identity": owned.identity.as_dict(),
            "graceful": not forced,
            "forced": forced,
            "inactive": True,
        }
        if forced:
            raise ProcessError(f"direct process required force stop: {target}")
        return evidence

    def wait(self, process: ProcessSpec, timeout_s: float) -> Mapping[str, Any]:
        """Wait for exact process or fresh session-file readiness."""

        timeout = self._timeout(timeout_s)
        bound = self._bound.get(process.target)
        if bound is None or bound.process != process:
            raise ProcessError(f"readiness process is not bound: {process.target}")
        deadline = time.monotonic() + timeout
        readiness = process.command.readiness
        pending_reason: str | None = None
        while True:
            owned = self._children.get(process.target)
            if owned is None:
                raise ProcessError(f"direct process exited before readiness: {process.target}")
            if not self.active(process.target):
                raise ProcessError(
                    f"direct process exited before readiness: {process.target}"
                    f"{self._diagnostic_suffix(owned)}"
                )
            if readiness.kind == "process":
                return {
                    "kind": "process",
                    "target": process.target,
                    "active": True,
                }
            ready_path = self._readiness_path(readiness.target)
            if ready_path.exists():
                expectation = readiness_expectation_for_process(
                    process.name,
                    readiness.target,
                )
                if expectation is None:
                    raise ProcessError(
                        f"direct process readiness evidence is invalid: {process.name}"
                    )
                if self._bound_plan is None:  # pragma: no cover - bind invariant
                    raise ProcessError("simulation process manager is not bound")
                try:
                    expected_control_mode = None
                    if expectation.adapter == "nav_status":
                        expected_control_mode = self._bound_plan.lifecycle.get(
                            "native_control_mode"
                        )
                    expected_slam_mode = None
                    if expectation.adapter == "slam_status":
                        expected_slam_mode = (
                            self._bound_plan.native_process_environment.get(
                                "LINGTU_SLAM_MODE"
                            )
                        )
                        if expected_slam_mode not in {"mapping", "localization"}:
                            raise SimReadinessError(
                                "bound RunPlan slam mode is invalid"
                            )
                    expected_explore_route = None
                    if expectation.adapter == "explore_status":
                        expected_explore_route = (
                            self._bound_plan.native_process_environment.get(
                                "LINGTU_EXPLORE_ROUTE"
                            )
                        )
                        if expected_explore_route not in {"live", "map"}:
                            raise SimReadinessError(
                                "bound RunPlan explore route is invalid"
                            )
                    typed = load_typed_readiness(
                        ready_path,
                        expectation=expectation,
                        product_session_id=self._require_product_session_id(),
                        product=self._bound_plan.product,
                        process=process.name,
                        started_wall_ns=owned.started_wall_ns,
                        lidar_required=self._bound_plan.has_process("lidar"),
                        imu_required=self._bound_plan.has_process("imu"),
                        camera_required=self._bound_plan.has_process("camera"),
                        expected_control_mode=expected_control_mode,
                        expected_slam_mode=expected_slam_mode,
                        expected_explore_route=expected_explore_route,
                    )
                except SimReadinessPending as exc:
                    pending_reason = str(exc)
                except SimReadinessError as exc:
                    raise ProcessError(
                        "direct process readiness evidence is invalid: "
                        f"{process.name}: {exc}"
                    ) from exc
                else:
                    return {
                        "kind": "file",
                        "target": process.target,
                        "adapter": expectation.adapter,
                        "payload": dict(typed),
                    }
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                suffix = f": {pending_reason}" if pending_reason else ""
                raise ProcessError(
                    f"direct process readiness timed out: {process.name}{suffix}"
                    f"{self._diagnostic_suffix(owned)}"
                )
            time.sleep(min(0.02, remaining))

    def _adopt_snapshot(
        self,
        snapshot: SimChildSnapshot | None,
        *,
        ledger: SimChildLedger,
        plan_path: Path,
        product_session_id: str,
        bound: Mapping[str, _BoundProcess],
    ) -> dict[str, _OwnedProcess]:
        if snapshot is None:
            return {}
        if snapshot.product_session_id != product_session_id:
            if snapshot.children:
                raise ProcessError(
                    "simulation child ledger belongs to a different unreleased Product session"
                )
            ledger.clear()
            return {}

        live_records = tuple(
            record
            for record in snapshot.children
            if record.process_identity.matches()
        )

        adopted: dict[str, _OwnedProcess] = {}
        retained_records: list[SimChildRecord] = []
        for record in snapshot.children:
            declared = bound.get(record.target)
            live = record in live_records
            if declared is None:
                if live:
                    raise ProcessError(
                        "simulation child ledger contains an undeclared live target"
                    )
                continue
            retain_dead_evidence = self._process_requires_file_stop_evidence(
                declared.process
            )
            if not live and not retain_dead_evidence:
                continue
            if live and os.name == "nt":
                raise ProcessError(
                    "cannot adopt live Windows simulation child without job ownership"
                )
            owned = _OwnedProcess(
                child=None,
                identity=record.process_identity,
                started_wall_ns=record.started_wall_ns,
                process_group=record.process_group,
                launch_id=record.launch_id,
                stdout_log=plan_path.parent / "logs" / f"{declared.process.name}.stdout.log",
                stderr_log=plan_path.parent / "logs" / f"{declared.process.name}.stderr.log",
            )
            if live and not self._process_group_matches(owned):
                raise ProcessError("simulation child ledger contains a foreign process group")
            adopted[record.target] = owned
            retained_records.append(record)
        if len(retained_records) != len(snapshot.children):
            ledger.replace(
                SimChildSnapshot.create(
                    product_session_id=product_session_id,
                    children=retained_records,
                )
            )
        return adopted

    @staticmethod
    def _bootstrap_environment() -> dict[str, str]:
        """Return the explicit ambient environment required to spawn children.

        Direct simulation children must be reproducible from the RunPlan.  This
        allowlist intentionally excludes arbitrary shell state such as
        PYTHONPATH, LD_PRELOAD, SSL_CERT_FILE, credentials, and ad-hoc local
        variables.  POSIX PATH is fixed to ``os.defpath`` instead of copied from
        the caller.  Windows keeps a small explicit process-creation allowlist
        for system bootstrap; Python direct commands are rewritten to this
        supervisor's exact ``sys.executable`` and native commands use the exact
        artifact path, so ambient PATH/PATHEXT are not needed. LingTu identity
        and product settings still come only from the RunPlan below.
        """

        if os.name != "nt":
            result = {"PATH": os.defpath}
            for key in ("HOME", "NAV_MAP_DIR"):
                value = os.environ.get(key)
                if value:
                    result[key] = value
            return result

        allowed = _WINDOWS_BOOTSTRAP_ENV
        result: dict[str, str] = {}
        allowed_upper = {key.upper() for key in allowed}
        for key, value in os.environ.items():
            upper = key.upper()
            if upper.startswith("LINGTU_"):
                continue
            if upper in allowed_upper:
                result[key] = value
        return result

    def _persist_children(self) -> None:
        ledger = self._ledger
        product_session_id = self._product_session_id
        if ledger is None or product_session_id is None:
            raise ProcessError("simulation process manager is not bound")
        records = tuple(
            SimChildRecord(
                target=target,
                process_identity=owned.identity,
                process_group=owned.process_group,
                started_wall_ns=owned.started_wall_ns,
                launch_id=owned.launch_id,
            )
            for target, owned in self._children.items()
        )
        try:
            if not records:
                ledger.clear()
            else:
                ledger.replace(
                    SimChildSnapshot.create(
                        product_session_id=product_session_id,
                        children=records,
                    )
                )
        except SimChildLedgerError as exc:
            raise ProcessError("simulation child ledger cannot be updated") from exc

    def _cleanup_unpublished_child(
        self,
        target: str,
        owned: _OwnedProcess,
    ) -> None:
        try:
            if owned.identity.matches() and self._process_group_matches(owned):
                self._force_stop_group(owned)
                self._wait_for_exit(owned, time.monotonic() + 1.0)
        finally:
            self._children.pop(target, None)
            if owned.job_handle is not None:
                self._close_windows_handle(owned.job_handle)
            self._close_log_capture(owned)

    def _log_paths(self, process_name: str) -> tuple[Path, Path]:
        session_root = self._session_root
        if session_root is None:
            raise ProcessError("simulation process manager is not bound")
        logs_root = session_root / "logs"
        try:
            logs_root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise ProcessError("direct process log directory is unavailable") from exc
        return (
            logs_root / f"{process_name}.stdout.log",
            logs_root / f"{process_name}.stderr.log",
        )

    def _start_log_capture(
        self,
        child: subprocess.Popen[bytes],
        *,
        process_name: str,
        stdout_log: Path,
        stderr_log: Path,
    ) -> _ProcessLogCapture:
        if child.stdout is None or child.stderr is None:  # pragma: no cover - Popen invariant
            raise ProcessError("direct process log pipes are unavailable")
        stdout_output = self._open_log_file(stdout_log)
        try:
            stderr_output = self._open_log_file(stderr_log)
        except Exception:
            stdout_output.close()
            raise
        capture = _ProcessLogCapture(
            stdout=_BoundedStreamCapture(child.stdout, stdout_output, stdout_log),
            stderr=_BoundedStreamCapture(child.stderr, stderr_output, stderr_log),
        )
        try:
            capture.start(process_name=process_name)
        except Exception:
            with suppress(Exception):
                capture.close()
            raise
        return capture

    def _open_log_file(self, path: Path) -> BinaryIO:
        try:
            return path.open("w+b", buffering=0)
        except OSError as exc:
            raise ProcessError("direct process log file cannot be opened") from exc

    @staticmethod
    def _diagnostic_suffix(owned: _OwnedProcess) -> str:
        details: list[str] = []
        exit_code = SimProcessManager._child_exit_code(owned)
        if exit_code is not None:
            details.append(f"exit_code={exit_code}")
        if owned.stdout_log is not None:
            details.append(f"stdout_log={owned.stdout_log}")
        if owned.stderr_log is not None:
            details.append(f"stderr_log={owned.stderr_log}")
        return f" ({'; '.join(details)})" if details else ""

    @staticmethod
    def _child_exit_code(owned: _OwnedProcess) -> int | None:
        child = owned.child
        if child is None:
            return None
        exit_code = child.poll()
        if exit_code is not None or owned.identity.matches():
            return exit_code
        try:
            return child.wait(timeout=0.1)
        except subprocess.TimeoutExpired:
            return None

    @staticmethod
    def _close_log_capture(owned: _OwnedProcess) -> None:
        capture = owned.log_capture
        if capture is None:
            return
        capture.close()
        owned.log_capture = None

    def _retire_readiness(
        self,
        process: ProcessSpec,
    ) -> None:
        """Remove readiness before a new launch or after a process stops."""

        readiness = process.command.readiness
        if readiness.kind != "file":
            return
        ready_path = self._readiness_path(readiness.target)
        try:
            ready_path.unlink()
        except FileNotFoundError:
            return
        except OSError as exc:
            raise ProcessError(
                f"direct process readiness file cannot be retired: {process.name}"
            ) from exc

    def _readiness_path(self, target: str | None) -> Path:
        if target is None:  # pragma: no cover - ProcessReadiness invariant
            raise ProcessError("direct process readiness target is missing")
        if self._session_root is None:
            raise ProcessError("simulation process manager is not bound")
        return self._session_root / PurePosixPath(target)

    @staticmethod
    def _timeout(value: float) -> float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or value <= 0
        ):
            raise ProcessError("direct process timeout must be positive and finite")
        return float(value)

    @staticmethod
    def _process_group_matches(owned: _OwnedProcess) -> bool:
        if owned.process_group != owned.identity.pid:
            return False
        if os.name == "nt":
            return bool(owned.identity.platform == "windows")
        if owned.identity.platform != "posix-procfs":
            return False
        try:
            get_process_group = cast(
                Callable[[int], int],
                vars(os)["getpgid"],
            )
            return bool(get_process_group(owned.identity.pid) == owned.process_group)
        except (OSError, ProcessLookupError):
            return False

    @classmethod
    def _owned_alive(cls, owned: _OwnedProcess) -> bool:
        if owned.child is not None and owned.child.poll() is not None:
            return False
        if not owned.identity.matches():
            return False
        return cls._process_group_matches(owned)

    @classmethod
    def _wait_for_exit(cls, owned: _OwnedProcess, deadline: float) -> None:
        if owned.child is not None:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            try:
                owned.child.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                return
            return
        while cls._owned_alive(owned):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(0.02, remaining))

    @classmethod
    def _graceful_stop_group(cls, owned: _OwnedProcess) -> None:
        if not cls._owned_alive(owned):
            return
        try:
            if os.name == "nt":
                if owned.child is not None:  # live Windows children are never adopted
                    owned.child.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                kill_process_group = cast(
                    Callable[[int, int], None],
                    vars(os)["killpg"],
                )
                kill_process_group(owned.process_group, signal.SIGTERM)
        except (OSError, ProcessLookupError):
            try:
                if owned.child is None:
                    return
                owned.child.terminate()
            except OSError:
                pass

    def _force_stop_group(self, owned: _OwnedProcess) -> None:
        if not self._owned_alive(owned):
            return
        if os.name == "nt":
            if owned.job_handle is not None:
                self._terminate_windows_job(owned.job_handle)
            elif owned.child is not None:
                owned.child.kill()
            return
        try:
            kill_process_group = cast(
                Callable[[int, int], None],
                vars(os)["killpg"],
            )
            kill_process_group(
                owned.process_group,
                cast(int, vars(signal)["SIGKILL"]),
            )
        except (OSError, ProcessLookupError):
            if owned.child is not None and owned.child.poll() is None:
                owned.child.kill()

    def _release_owned(self, target: str, owned: _OwnedProcess) -> None:
        existing = self._children.get(target)
        if existing is not owned:
            return
        self._children.pop(target, None)
        try:
            self._persist_children()
        except ProcessError:
            self._children[target] = owned
            raise
        if owned.job_handle is not None:
            self._close_windows_handle(owned.job_handle)
        self._close_log_capture(owned)

    @staticmethod
    def _create_windows_job(child: subprocess.Popen[bytes]) -> int | None:
        if os.name != "nt":
            return None
        import ctypes
        from ctypes import wintypes

        class IoCounters(ctypes.Structure):
            _fields_ = [
                ("ReadOperationCount", ctypes.c_ulonglong),
                ("WriteOperationCount", ctypes.c_ulonglong),
                ("OtherOperationCount", ctypes.c_ulonglong),
                ("ReadTransferCount", ctypes.c_ulonglong),
                ("WriteTransferCount", ctypes.c_ulonglong),
                ("OtherTransferCount", ctypes.c_ulonglong),
            ]

        class BasicLimitInformation(ctypes.Structure):
            _fields_ = [
                ("PerProcessUserTimeLimit", ctypes.c_longlong),
                ("PerJobUserTimeLimit", ctypes.c_longlong),
                ("LimitFlags", wintypes.DWORD),
                ("MinimumWorkingSetSize", ctypes.c_size_t),
                ("MaximumWorkingSetSize", ctypes.c_size_t),
                ("ActiveProcessLimit", wintypes.DWORD),
                ("Affinity", ctypes.c_size_t),
                ("PriorityClass", wintypes.DWORD),
                ("SchedulingClass", wintypes.DWORD),
            ]

        class ExtendedLimitInformation(ctypes.Structure):
            _fields_ = [
                ("BasicLimitInformation", BasicLimitInformation),
                ("IoInfo", IoCounters),
                ("ProcessMemoryLimit", ctypes.c_size_t),
                ("JobMemoryLimit", ctypes.c_size_t),
                ("PeakProcessMemoryUsed", ctypes.c_size_t),
                ("PeakJobMemoryUsed", ctypes.c_size_t),
            ]

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
        kernel32.CreateJobObjectW.restype = wintypes.HANDLE
        handle = kernel32.CreateJobObjectW(None, None)
        if not handle:
            raise ProcessError("cannot create direct process job")
        information = ExtendedLimitInformation()
        information.BasicLimitInformation.LimitFlags = 0x2000
        kernel32.SetInformationJobObject.argtypes = [
            wintypes.HANDLE,
            ctypes.c_int,
            ctypes.c_void_p,
            wintypes.DWORD,
        ]
        kernel32.SetInformationJobObject.restype = wintypes.BOOL
        kernel32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
        kernel32.AssignProcessToJobObject.restype = wintypes.BOOL
        process_handle = int(vars(child)["_handle"])
        if not kernel32.SetInformationJobObject(
            handle,
            9,
            ctypes.byref(information),
            ctypes.sizeof(information),
        ) or not kernel32.AssignProcessToJobObject(handle, process_handle):
            kernel32.CloseHandle(handle)
            raise ProcessError("cannot assign direct process to owned job")
        return int(handle)

    @staticmethod
    def _terminate_windows_job(handle: int) -> None:
        if os.name != "nt":
            return
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
        kernel32.TerminateJobObject.restype = wintypes.BOOL
        kernel32.TerminateJobObject(handle, 1)

    @staticmethod
    def _close_windows_handle(handle: int) -> None:
        if os.name != "nt":
            return
        import ctypes
        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
        kernel32.CloseHandle.restype = wintypes.BOOL
        kernel32.CloseHandle(handle)

    def _require_product_session_id(self) -> str:
        product_session_id = self._product_session_id
        if product_session_id is None:  # pragma: no cover - bind invariant
            raise ProcessError("simulation process manager is not bound")
        return product_session_id

__all__ = ["SimProcessManager"]
