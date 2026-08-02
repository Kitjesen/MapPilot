"""Apply the deployment processes declared by a RunPlan.

Blueprint owns one in-process Module graph. SystemdRunner is the only product-layer
component allowed to apply Product processes to an external process manager.
"""

from __future__ import annotations

import os
import subprocess
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

from lingtu.run_plan import RunPlan
from runtime.graph import ProcessSpec
from runtime.service_catalogs.thunder import thunder_service_spec
from runtime.service_manager import ServiceManager

PROCESS_REPORT_SCHEMA = "lingtu.process_report.v1"


class ProcessError(RuntimeError):
    """Base error for invalid or failed product launch operations."""


class ProcessFailed(ProcessError):
    """Raised after a launch transaction fails and rollback has run."""

    def __init__(self, report: ProcessReport):
        super().__init__(report.error or "product launch failed")
        self.report = report


class ProcessControl(Protocol):
    """Strict process-manager operations required by SystemdRunner."""

    def available(self, target: str) -> bool:
        """Return whether a process target is installed."""

    def active(self, target: str) -> bool:
        """Return whether a process target is active."""

    def start(self, target: str, timeout_s: float) -> None:
        """Start a target or raise on failure."""

    def stop(self, target: str, timeout_s: float) -> None:
        """Stop a target or raise on failure."""


class Readiness(Protocol):
    """Product-process readiness boundary."""

    def wait(self, process: ProcessSpec, timeout_s: float) -> Mapping[str, Any]:
        """Wait for one process contract or raise on timeout."""


@dataclass
class ProcessReport:
    """Machine-readable evidence for one SystemdRunner transaction."""

    product: str
    env: str
    action: str
    dry_run: bool = False
    ok: bool = False
    status: str = "pending"
    planned: list[str] = field(default_factory=list)
    stopped: list[str] = field(default_factory=list)
    started: list[str] = field(default_factory=list)
    preserved: list[str] = field(default_factory=list)
    ready: dict[str, Mapping[str, Any]] = field(default_factory=dict)
    rolled_back: list[str] = field(default_factory=list)
    rollback_errors: list[str] = field(default_factory=list)
    error: str | None = None

    def as_dict(self) -> dict[str, Any]:
        """Return the stable JSON contract used by CLI and Gateway."""

        return {
            "schema_version": PROCESS_REPORT_SCHEMA,
            "product": self.product,
            "env": self.env,
            "action": self.action,
            "dry_run": self.dry_run,
            "ok": self.ok,
            "status": self.status,
            "planned": list(self.planned),
            "stopped": list(self.stopped),
            "started": list(self.started),
            "preserved": list(self.preserved),
            "ready": dict(self.ready),
            "rolled_back": list(self.rolled_back),
            "rollback_errors": list(self.rollback_errors),
            "error": self.error,
        }


class Systemd:
    """Strict systemd adapter used by the field SystemdRunner."""

    def __init__(
        self,
        *,
        use_sudo: bool | None = None,
        runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if use_sudo is None:
            getuid = getattr(os, "geteuid", None)
            use_sudo = os.name != "nt" and (getuid is None or getuid() != 0)
        self._use_sudo = use_sudo
        self._runner = runner
        self._sleep = sleep
        self._monotonic = monotonic

    def available(self, target: str) -> bool:
        """Return whether systemd has a loaded unit for *target*."""

        result = self._run(
            ["systemctl", "show", "-p", "LoadState", "--value", target],
            timeout_s=5.0,
        )
        state = (result.stdout or "").strip()
        return result.returncode == 0 and state not in {"", "not-found", "masked"}

    def active(self, target: str) -> bool:
        """Return whether *target* is active."""

        result = self._run(
            ["systemctl", "is-active", "--quiet", target],
            timeout_s=5.0,
        )
        return result.returncode == 0

    def start(self, target: str, timeout_s: float) -> None:
        """Start *target* and require it to become active."""

        if not self.available(target):
            raise ProcessError(f"required systemd unit is not installed: {target}")
        self._require_success(self._control("start", target, timeout_s), "start", target)
        self._wait_state(target, active=True, timeout_s=timeout_s)

    def stop(self, target: str, timeout_s: float) -> None:
        """Stop *target* and require it to become inactive."""

        if not self.available(target):
            return
        self._require_success(self._control("stop", target, timeout_s), "stop", target)
        self._wait_state(target, active=False, timeout_s=timeout_s)

    def _control(
        self,
        action: str,
        target: str,
        timeout_s: float,
    ) -> subprocess.CompletedProcess[str]:
        prefix = ["sudo", "-n"] if self._use_sudo else []
        return self._run(
            [*prefix, "systemctl", action, target],
            timeout_s=max(1.0, timeout_s),
        )

    def _run(
        self,
        command: list[str],
        *,
        timeout_s: float,
    ) -> subprocess.CompletedProcess[str]:
        try:
            return self._runner(
                command,
                check=False,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=timeout_s,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise ProcessError(f"process-manager command failed: {' '.join(command)}: {exc}") from exc

    def _wait_state(self, target: str, *, active: bool, timeout_s: float) -> None:
        deadline = self._monotonic() + timeout_s
        while self._monotonic() < deadline:
            if self.active(target) is active:
                return
            self._sleep(0.2)
        expected = "active" if active else "inactive"
        raise ProcessError(f"systemd unit did not become {expected}: {target}")

    @staticmethod
    def _require_success(
        result: subprocess.CompletedProcess[str],
        action: str,
        target: str,
    ) -> None:
        if result.returncode == 0:
            return
        detail = (result.stderr or result.stdout or "").strip()
        suffix = f": {detail}" if detail else ""
        raise ProcessError(f"systemd {action} failed for {target}{suffix}")


class ServiceReadiness:
    """Wait for catalog-defined systemd, binary, status, DDS, and HTTP checks."""

    def __init__(
        self,
        manager: ServiceManager | None = None,
        *,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._manager = manager or ServiceManager()
        self._sleep = sleep
        self._monotonic = monotonic

    def wait(self, process: ProcessSpec, timeout_s: float) -> Mapping[str, Any]:
        """Require every readiness check declared for *process*."""

        service = process.name
        spec = thunder_service_spec(service)
        if spec is not None and process.target not in (spec.start_units or spec.units):
            raise ProcessError(f"Product target disagrees with readiness catalog for {service}: {process.target}")
        deadline = self._monotonic() + timeout_s
        last: Mapping[str, Any] = {}
        while self._monotonic() < deadline:
            last = self._manager.status_details(
                service,
                dds_check=True,
                http_check=True,
            )[service]
            if bool(last.get("ready")):
                return last
            self._sleep(0.25)
        blockers = ", ".join(str(item) for item in last.get("blockers", []))
        detail = blockers or "readiness contract did not become true"
        raise ProcessError(f"{process.name} is not ready: {detail}")


class SystemdRunner:
    """Apply Product processes as a fail-closed external transaction."""

    def __init__(
        self,
        control: ProcessControl | None = None,
        readiness: Readiness | None = None,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        self._control = control or Systemd()
        self._readiness = readiness or ServiceReadiness()
        self._environment = environment if environment is not None else os.environ

    def apply(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        """Stop conflicting mode processes and apply one RunPlan."""

        plan.assert_compatible(environment=self._environment)
        processes = tuple(sorted(self._require_processes(plan), key=lambda item: (item.order, item.name)))
        self._require_supported_managers(processes)
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="apply",
            dry_run=dry_run,
            planned=[process.target for process in processes],
        )
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        self._reject_self_management(processes)
        started: list[ProcessSpec] = []
        try:
            timeouts = {process.target: process.timeout_s for process in plan.available_processes}
            for target in plan.stop_targets:
                if not self._control.active(target):
                    continue
                self._control.stop(target, float(timeouts.get(target, 15)))
                report.stopped.append(target)

            for process in processes:
                if self._control.active(process.target):
                    if process.lifecycle != "persistent":
                        raise ProcessError(f"mode process remained active after conflict stop: {process.target}")
                    report.preserved.append(process.target)
                else:
                    self._control.start(process.target, float(process.timeout_s))
                    started.append(process)
                    report.started.append(process.target)
                report.ready[process.name] = self._readiness.wait(
                    process,
                    float(process.timeout_s),
                )
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            self._rollback(started, report)
            raise ProcessFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def stop(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        """Stop the plan's mode processes and preserve persistent ones."""

        plan.assert_compatible(environment=self._environment)
        self._require_processes(plan)
        processes = tuple(reversed(plan.managed_processes))
        self._require_supported_managers(processes)
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="stop",
            dry_run=dry_run,
            planned=[process.target for process in processes],
        )
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        self._reject_self_management(processes)
        try:
            for process in processes:
                if not self._control.active(process.target):
                    continue
                self._control.stop(process.target, float(process.timeout_s))
                report.stopped.append(process.target)
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            raise ProcessFailed(report) from exc
        report.ok = True
        report.status = "stopped"
        return report

    def quiesce(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        """Stop every mode target that can conflict with this plan.

        This is the process-level fail-closed primitive used after a product
        switch mutates deployment state. ProductControl owns the decision to
        invoke it; SystemdRunner owns the systemd operations.
        """

        plan.assert_compatible(environment=self._environment)
        self._require_processes(plan)
        targets = tuple(plan.stop_targets)
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="quiesce",
            dry_run=dry_run,
            planned=list(targets),
        )
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        timeouts = {process.target: process.timeout_s for process in plan.available_processes}
        failures: list[tuple[str, Exception]] = []
        for target in targets:
            try:
                if not self._control.active(target):
                    continue
                self._control.stop(target, float(timeouts.get(target, 15)))
                report.stopped.append(target)
            except Exception as exc:
                failures.append((target, exc))
        if failures:
            details = "; ".join(
                f"{target}: {str(error) or error.__class__.__name__}"
                for target, error in failures
            )
            report.error = f"failed to quiesce Product processes: {details}"
            report.status = "failed"
            raise ProcessFailed(report) from failures[0][1]
        report.ok = True
        report.status = "stopped"
        return report

    def restart(
        self,
        plan: RunPlan,
        process_name: str,
        *,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Restart one process declared by the RunPlan.

        The logical process name, target unit, timeout, and readiness checks all
        come from the plan. A failed restart is stopped fail-closed; if the
        unit was active before the operation, SystemdRunner makes one recovery start
        and records whether the previous running state was restored.
        """

        plan.assert_compatible(environment=self._environment)
        processes = self._require_processes(plan)
        process = next(
            (item for item in processes if item.name == process_name),
            None,
        )
        if process is None:
            known = ", ".join(item.name for item in processes)
            raise ProcessError(
                f"process {process_name!r} is not in "
                f"{plan.product}/{plan.env}; "
                f"known processes: {known}"
            )
        self._require_supported_managers((process,))
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="restart",
            dry_run=dry_run,
            planned=[process.target],
        )
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        self._reject_self_management((process,))
        was_active = self._control.active(process.target)
        try:
            if was_active:
                self._control.stop(process.target, float(process.timeout_s))
                report.stopped.append(process.target)
            self._control.start(process.target, float(process.timeout_s))
            report.started.append(process.target)
            report.ready[process.name] = self._readiness.wait(
                process,
                float(process.timeout_s),
            )
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            self._stop_failed_restart(process, report)
            if was_active:
                self._restore_restarted_process(process, report)
            raise ProcessFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def _stop_failed_restart(
        self,
        process: ProcessSpec,
        report: ProcessReport,
    ) -> None:
        try:
            if self._control.active(process.target):
                self._control.stop(process.target, float(process.timeout_s))
        except Exception as exc:
            report.rollback_errors.append(f"stop failed {process.target}: {exc}")

    def _restore_restarted_process(
        self,
        process: ProcessSpec,
        report: ProcessReport,
    ) -> None:
        try:
            self._control.start(process.target, float(process.timeout_s))
            report.ready[process.name] = self._readiness.wait(
                process,
                float(process.timeout_s),
            )
            report.rolled_back.append(process.target)
        except Exception as exc:
            report.rollback_errors.append(f"restore failed {process.target}: {exc}")
            try:
                if self._control.active(process.target):
                    self._control.stop(process.target, float(process.timeout_s))
            except Exception as cleanup_exc:
                report.rollback_errors.append(f"restore cleanup failed {process.target}: {cleanup_exc}")

    def _rollback(self, started: list[ProcessSpec], report: ProcessReport) -> None:
        for process in reversed(started):
            try:
                if self._control.active(process.target):
                    self._control.stop(process.target, float(process.timeout_s))
                report.rolled_back.append(process.target)
            except Exception as exc:
                report.rollback_errors.append(f"{process.target}: {exc}")

    def _reject_self_management(self, processes: tuple[ProcessSpec, ...]) -> None:
        current_unit = str(self._environment.get("LINGTU_SYSTEMD_UNIT") or "").strip()
        if current_unit and any(process.target == current_unit for process in processes):
            raise ProcessError(f"SystemdRunner must run outside {current_unit}; use an independent transient unit")

    @staticmethod
    def _require_processes(plan: RunPlan) -> tuple[ProcessSpec, ...]:
        if plan.process_control != "systemd":
            raise ProcessError(
                f"Product {plan.product} is controlled by {plan.process_control}, not SystemdRunner"
            )
        if not plan.processes:
            raise ProcessError(
                f"Product {plan.product} has no deployment processes in Env {plan.env}"
            )
        return plan.processes

    @staticmethod
    def _require_supported_managers(processes: tuple[ProcessSpec, ...]) -> None:
        unsupported = sorted({process.manager for process in processes if process.manager != "systemd"})
        if unsupported:
            raise ProcessError(f"unsupported process managers: {', '.join(unsupported)}")
