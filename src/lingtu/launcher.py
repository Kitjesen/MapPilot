"""Apply the deployment processes declared by a compiled Product.

Blueprint owns one in-process Module graph. Launcher is the only product-layer
component allowed to apply Product processes to an external process manager.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

from lingtu.product import Product, ProductManifest
from runtime.graph import ProcessSpec
from runtime.service_catalogs.thunder import thunder_service_spec
from runtime.service_manager import ServiceManager

LAUNCH_REPORT_SCHEMA = "lingtu.launch_report.v1"


class LaunchError(RuntimeError):
    """Base error for invalid or failed product launch operations."""


class LaunchFailed(LaunchError):
    """Raised after a launch transaction fails and rollback has run."""

    def __init__(self, report: LaunchReport):
        super().__init__(report.error or "product launch failed")
        self.report = report


class ProcessControl(Protocol):
    """Strict process-manager operations required by Launcher."""

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
class LaunchReport:
    """Machine-readable evidence for one Launcher transaction."""

    product: str
    endpoint: str
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
            "schema_version": LAUNCH_REPORT_SCHEMA,
            "product": self.product,
            "endpoint": self.endpoint,
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
    """Strict systemd adapter used by the field Launcher."""

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
            raise LaunchError(f"required systemd unit is not installed: {target}")
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
            raise LaunchError(f"process-manager command failed: {' '.join(command)}: {exc}") from exc

    def _wait_state(self, target: str, *, active: bool, timeout_s: float) -> None:
        deadline = self._monotonic() + timeout_s
        while self._monotonic() < deadline:
            if self.active(target) is active:
                return
            self._sleep(0.2)
        expected = "active" if active else "inactive"
        raise LaunchError(f"systemd unit did not become {expected}: {target}")

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
        raise LaunchError(f"systemd {action} failed for {target}{suffix}")


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
            raise LaunchError(f"Product target disagrees with readiness catalog for {service}: {process.target}")
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
        raise LaunchError(f"{process.name} is not ready: {detail}")


LaunchProduct = Product | ProductManifest


class Launcher:
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

    def apply(self, product: LaunchProduct, *, dry_run: bool = False) -> LaunchReport:
        """Stop conflicting mode processes and start one compiled product."""

        processes = tuple(sorted(self._require_processes(product), key=lambda item: (item.order, item.name)))
        self._require_supported_managers(processes)
        report = LaunchReport(
            product=product.profile,
            endpoint=self._endpoint(product),
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
            timeouts = {process.target: process.timeout_s for process in product.available_processes}
            for target in product.stop_targets:
                if not self._control.active(target):
                    continue
                self._control.stop(target, float(timeouts.get(target, 15)))
                report.stopped.append(target)

            for process in processes:
                if self._control.active(process.target):
                    if process.lifecycle != "persistent":
                        raise LaunchError(f"mode process remained active after conflict stop: {process.target}")
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
            raise LaunchFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def stop(self, product: LaunchProduct, *, dry_run: bool = False) -> LaunchReport:
        """Stop the selected product's mode processes and preserve persistent ones."""

        self._require_processes(product)
        processes = tuple(reversed(product.managed_processes))
        self._require_supported_managers(processes)
        report = LaunchReport(
            product=product.profile,
            endpoint=self._endpoint(product),
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
            raise LaunchFailed(report) from exc
        report.ok = True
        report.status = "stopped"
        return report

    def quiesce(self, product: LaunchProduct, *, dry_run: bool = False) -> LaunchReport:
        """Stop every mode target that can conflict with this Product.

        This is the process-level fail-closed primitive used after a product
        switch mutates deployment state. ProductControl owns the decision to
        invoke it; Launcher owns the systemd operations.
        """

        self._require_processes(product)
        targets = tuple(product.stop_targets)
        report = LaunchReport(
            product=product.profile,
            endpoint=self._endpoint(product),
            action="quiesce",
            dry_run=dry_run,
            planned=list(targets),
        )
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        timeouts = {process.target: process.timeout_s for process in product.available_processes}
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
            raise LaunchFailed(report) from failures[0][1]
        report.ok = True
        report.status = "stopped"
        return report

    def restart(
        self,
        product: LaunchProduct,
        process_name: str,
        *,
        dry_run: bool = False,
    ) -> LaunchReport:
        """Restart one process declared by the compiled Product.

        The logical process name, target unit, timeout, and readiness checks all
        come from the Product. A failed restart is stopped fail-closed; if the
        unit was active before the operation, Launcher makes one recovery start
        and records whether the previous running state was restored.
        """

        processes = self._require_processes(product)
        process = next(
            (item for item in processes if item.name == process_name),
            None,
        )
        if process is None:
            known = ", ".join(item.name for item in processes)
            raise LaunchError(
                f"process {process_name!r} is not in "
                f"{product.profile}/{self._endpoint(product)}; "
                f"known processes: {known}"
            )
        self._require_supported_managers((process,))
        report = LaunchReport(
            product=product.profile,
            endpoint=self._endpoint(product),
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
            raise LaunchFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def _stop_failed_restart(
        self,
        process: ProcessSpec,
        report: LaunchReport,
    ) -> None:
        try:
            if self._control.active(process.target):
                self._control.stop(process.target, float(process.timeout_s))
        except Exception as exc:
            report.rollback_errors.append(f"stop failed {process.target}: {exc}")

    def _restore_restarted_process(
        self,
        process: ProcessSpec,
        report: LaunchReport,
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

    def _rollback(self, started: list[ProcessSpec], report: LaunchReport) -> None:
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
            raise LaunchError(f"Launcher must run outside {current_unit}; use an independent transient unit")

    @staticmethod
    def _require_processes(product: LaunchProduct) -> tuple[ProcessSpec, ...]:
        if product.process_control != "launcher":
            raise LaunchError(f"product {product.profile} is controlled by {product.process_control}, not Launcher")
        if not product.processes:
            raise LaunchError(f"product {product.profile} has no deployment processes on endpoint {product.endpoint}")
        return product.processes

    @staticmethod
    def _endpoint(product: LaunchProduct) -> str:
        if not product.endpoint:
            raise LaunchError(f"product {product.profile} has no deployment endpoint")
        return product.endpoint

    @staticmethod
    def _require_supported_managers(processes: tuple[ProcessSpec, ...]) -> None:
        unsupported = sorted({process.manager for process in processes if process.manager != "systemd"})
        if unsupported:
            raise LaunchError(f"unsupported process managers: {', '.join(unsupported)}")


def main(argv: list[str] | None = None) -> int:
    """Execute one already compiled Product manifest."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("plan", "apply", "stop", "restart"))
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--process")
    parser.add_argument("--no-sudo", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    try:
        product = ProductManifest.load(args.manifest)
        if args.action == "apply" and product.process_control == "launcher":
            raise LaunchError(
                "field Product cannot be applied through the Launcher CLI; "
                "use `python -m lingtu.control switch` so map/runtime staging and "
                "active Product commit remain one transaction"
            )
        launcher = Launcher(control=Systemd(use_sudo=not args.no_sudo))
        if args.action == "plan":
            report = launcher.apply(product, dry_run=True)
        elif args.action == "stop":
            report = launcher.stop(product)
        elif args.action == "restart":
            if not args.process:
                raise LaunchError("restart requires --process <logical-name>")
            report = launcher.restart(product, args.process)
        else:
            report = launcher.apply(product)
    except LaunchFailed as exc:
        report = exc.report
        print(json.dumps(report.as_dict(), ensure_ascii=False, indent=2))
        return 1
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2))
        return 2

    payload = report.as_dict()
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(f"{report.status}: {report.product} on {report.endpoint}")
        for target in report.started:
            print(f"  started {target}")
        for target in report.preserved:
            print(f"  preserved {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
