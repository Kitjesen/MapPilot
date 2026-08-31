"""Systemd process ownership for real Products."""

from __future__ import annotations

import json
import os
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from lingtu.run_plan import RunPlan
from lingtu.switch_contracts import (
    PROCESS_REPORT_SCHEMA,
    ProcessError,
    ProcessFailed,
    ProcessReport,
)
from runtime.graph import ProcessSpec
from runtime.service_catalogs.thunder import ThunderServiceSpec, thunder_service_spec


class ProcessManager(Protocol):
    """Systemd operations required by the real Product runner."""

    def available(self, target: str) -> bool:
        """Return whether *target* is installed."""

    def active(self, target: str) -> bool:
        """Return whether *target* is active."""

    def start(self, target: str, timeout_s: float) -> None:
        """Start *target*."""

    def stop(self, target: str, timeout_s: float) -> None:
        """Stop *target*."""


class Readiness(Protocol):
    """Readiness check for one resolved real process."""

    def wait(self, process: ProcessSpec, timeout_s: float) -> Mapping[str, Any]:
        """Wait until one process satisfies its real readiness contract."""


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

    def identity(self, target: str) -> dict[str, int | str]:
        """Return the exact live systemd invocation identity for *target*."""

        result = self._run(
            [
                "systemctl",
                "show",
                "--property=MainPID",
                "--property=InvocationID",
                target,
            ],
            timeout_s=5.0,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout or "").strip()
            suffix = f": {detail}" if detail else ""
            raise ProcessError(f"systemd identity failed for {target}{suffix}")

        properties: dict[str, str] = {}
        for line in (result.stdout or "").splitlines():
            key, separator, value = line.partition("=")
            if separator and key in {"MainPID", "InvocationID"}:
                properties[key] = value.strip()

        try:
            pid = int(properties.get("MainPID", ""))
        except ValueError as exc:
            raise ProcessError(f"systemd identity failed for {target}: invalid MainPID") from exc
        invocation_id = properties.get("InvocationID", "")
        if pid <= 0 or not invocation_id:
            raise ProcessError(f"systemd identity failed for {target}: incomplete process identity")
        return {"pid": pid, "invocation_id": invocation_id}

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


class _ServiceInspector:
    """Observe the readiness contract for one resolved systemd process."""

    def __init__(
        self,
        *,
        systemd: Systemd | None = None,
        runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
        now: Callable[[], float] = time.time,
    ) -> None:
        self._systemd = systemd or Systemd(use_sudo=False)
        self._runner = runner
        self._now = now

    def status_details(
        self,
        service: str,
        *,
        dds_check: bool,
        http_check: bool,
    ) -> dict[str, Mapping[str, Any]]:
        spec = thunder_service_spec(service)
        if spec is None:
            return {service: {"ready": False, "blockers": ["readiness_catalog_missing"]}}
        target = (spec.start_units or spec.units)[0]
        blockers: list[str] = []
        observed: dict[str, Any] = {}

        if "systemd" in spec.checks:
            installed = self._systemd.available(target)
            active = installed and self._systemd.active(target)
            observed["systemd"] = active
            if not installed:
                blockers.append(f"systemd_unit_missing:{target}")
            elif not active:
                blockers.append("systemd_inactive")
        if "native_binary" in spec.checks:
            observed["native_binary"] = self._binaries(spec, blockers)
        if "status_file" in spec.checks:
            observed["status_file"] = self._status_files(spec, blockers)
        if "dds" in spec.checks:
            observed["dds"] = self._dds(spec, blockers) if dds_check else {"checked": False}
        if "http" in spec.checks:
            observed["http"] = self._http(service, blockers) if http_check else {"checked": False}

        return {
            service: {
                "ready": not blockers,
                "blockers": blockers,
                "observed": observed,
                "contract": {
                    "checks": list(spec.checks),
                    "topics": list(spec.topics),
                    "dds_topics": list(spec.dds_topics),
                    "shm_topics": list(spec.shm_topics),
                    "shm_channels": list(spec.shm_channels),
                    "files": list(spec.files),
                    "binaries": [
                        {"name": name, "env": env, "path": path}
                        for name, env, path in spec.binaries
                    ],
                },
            }
        }

    @staticmethod
    def _binaries(spec: ThunderServiceSpec, blockers: list[str]) -> dict[str, Any]:
        rows = []
        for name, env, default_path in spec.binaries:
            raw_path = os.environ.get(env, default_path)
            path = Path(raw_path)
            executable = path.exists() and os.access(path, os.X_OK)
            rows.append(
                {
                    "name": name,
                    "env": env,
                    "path": raw_path,
                    "exists": path.exists(),
                    "executable": executable,
                }
            )
            if not executable:
                blockers.append(f"native_binary_missing_or_not_executable:{name}:{raw_path}")
        return {"ok": all(row["executable"] for row in rows), "binaries": rows}

    def _status_files(self, spec: ThunderServiceSpec, blockers: list[str]) -> dict[str, Any]:
        rows = []
        for file_name in spec.files:
            path = Path(file_name)
            row: dict[str, Any] = {"path": file_name, "exists": path.exists()}
            if not path.exists():
                blockers.append(f"status_file_missing:{file_name}")
                rows.append(row)
                continue
            try:
                stat = path.stat()
                row["age_s"] = max(0.0, self._now() - stat.st_mtime)
                payload = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                row["read_error"] = str(exc)
                blockers.append(f"status_file_unreadable:{file_name}")
                rows.append(row)
                continue
            if isinstance(payload, dict):
                status = str(payload.get("status") or "").strip().lower()
                row["status"] = status
                if isinstance(payload.get("ready"), bool):
                    row["ready"] = payload["ready"]
                    if payload["ready"] is False:
                        blockers.append(f"status_file_not_ready:{file_name}")
                if status in {"error", "failed", "fatal"}:
                    blockers.append(f"status_file_error:{file_name}")
            if spec.status_max_age_s is not None and row["age_s"] > spec.status_max_age_s:
                blockers.append(f"status_file_stale:{file_name}")
            rows.append(row)
        return {"ok": not any(item.startswith("status_file_") for item in blockers), "files": rows}

    def _dds(self, spec: ThunderServiceSpec, blockers: list[str]) -> dict[str, Any]:
        topics = list(spec.dds_topics or spec.topics)
        root = Path(__file__).resolve().parents[3]
        command = [
            os.environ.get(
                "LINGTU_DDS_PROBE_BIN",
                str(root / "build" / "dds_probe" / "lingtu_dds_probe"),
            ),
            "--json",
            "--seconds",
            os.environ.get("LINGTU_SERVICE_DDS_CHECK_TIMEOUT", "2.0"),
            "--domain",
            os.environ.get("LINGTU_DDS_DOMAIN_ID", "0"),
            *topics,
        ]
        samples = {topic: 0 for topic in topics}
        try:
            result = self._runner(
                command,
                check=False,
                capture_output=True,
                text=True,
                cwd=str(root),
            )
            rows = json.loads(result.stdout or "[]") if result.returncode in (0, 1) else []
            for row in rows:
                samples[str(row.get("topic") or "")] = int(row.get("samples", 0) or 0)
            if result.returncode not in (0, 1):
                blockers.append((result.stderr or "").strip() or f"dds_probe_failed:{result.returncode}")
            else:
                blockers.extend(f"dds_topic_silent:{topic}" for topic in topics if samples[topic] <= 0)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            blockers.append(f"dds_check_error:{type(exc).__name__}")
        return {"ok": all(samples.values()), "checked": True, "topics": topics, "samples": samples}

    @staticmethod
    def _http(service: str, blockers: list[str]) -> dict[str, Any]:
        url = os.environ.get("LINGTU_SERVICE_HTTP_URL", "http://127.0.0.1:5050/ready")
        scheme = urllib.parse.urlparse(url).scheme.lower()
        if scheme not in {"http", "https"}:
            blockers.append(f"http_url_invalid_scheme:{scheme or 'missing'}")
            return {"ok": False, "checked": True, "service": service, "url": url}
        payload: Any = None
        status_code: int | None = None
        response_body: bytes | str = b""
        try:
            with urllib.request.urlopen(  # noqa: S310 - scheme checked above
                url,
                timeout=max(0.1, float(os.environ.get("LINGTU_SERVICE_HTTP_CHECK_TIMEOUT", "1.0"))),
            ) as response:
                status_code = int(getattr(response, "status", 0) or 0)
                response_body = response.read()
        except urllib.error.HTTPError as exc:
            status_code = int(exc.code)
            response_body = exc.read()
        except (OSError, ValueError) as exc:
            blockers.append(f"http_check_error:{type(exc).__name__}")
        if status_code is not None:
            try:
                payload = json.loads(response_body)
            except (json.JSONDecodeError, UnicodeDecodeError):
                blockers.append("http_invalid_json")
        if isinstance(payload, dict):
            for field in ("data_ready", "non_motion_safe"):
                if field not in payload:
                    blockers.append(f"http_payload_missing:{field}")
                elif type(payload[field]) is not bool:
                    blockers.append(f"http_payload_invalid_type:{field}")
                elif payload[field] is False:
                    blockers.append(
                        "http_data_not_ready" if field == "data_ready" else "http_non_motion_safe_false"
                    )
            for field in ("failed_modules", "critical_failed_modules"):
                if field not in payload:
                    blockers.append(f"http_payload_missing:{field}")
                    continue
                value = payload.get(field)
                if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
                    blockers.append(f"http_payload_invalid_type:{field}")
                elif value:
                    blockers.append(f"http_{field}:{','.join(value)}")
        safe_degraded = (
            status_code == 503
            and isinstance(payload, dict)
            and payload.get("data_ready") is True
            and payload.get("non_motion_safe") is True
            and payload.get("failed_modules") == []
            and payload.get("critical_failed_modules") == []
        )
        if status_code is not None and not 200 <= status_code < 300 and not safe_degraded:
            blockers.append(f"http_status:{status_code}")
        return {
            "ok": not blockers,
            "checked": True,
            "service": service,
            "url": url,
            "status_code": status_code,
            "payload": payload,
        }


class ServiceReadiness:
    """Wait for catalog-defined systemd, binary, status, DDS, and HTTP checks."""

    def __init__(
        self,
        manager: Any | None = None,
        *,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._manager = manager or _ServiceInspector()
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
    """Apply one resolved real Product through systemd."""

    def __init__(
        self,
        manager: ProcessManager | None = None,
        readiness: Readiness | None = None,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        self._environment = environment if environment is not None else os.environ
        self._manager = manager or Systemd()
        self._readiness = readiness or ServiceReadiness()
        self._controller = "systemd"
        self._process_manager = "systemd"
        self._runner_name = "SystemdRunner"
        self._monotonic = time.monotonic

    def apply(
        self,
        plan: RunPlan,
        *,
        dry_run: bool = False,
        defer_rollback: bool = False,
    ) -> ProcessReport:
        """Stop conflicting processes and apply one resolved RunPlan."""

        self._validate_plan(plan)
        processes = tuple(
            sorted(
                self._require_processes(plan),
                key=lambda item: (item.order, item.name),
            )
        )
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

        self._before_process_mutation(processes)
        started: list[ProcessSpec] = []
        try:
            process_by_target = {
                process.target: process for process in plan.available_processes
            }
            timeouts = {
                process.target: process.timeout_s
                for process in plan.available_processes
            }
            for target in plan.stop_before_start:
                conflicting_process = process_by_target.get(target)
                if conflicting_process is None:
                    if not self._manager.active(target):
                        continue
                    self._stop_target(target, float(timeouts.get(target, 15)))
                else:
                    if conflicting_process.lifecycle == "persistent":
                        continue
                    if not self._should_stop_process(conflicting_process):
                        continue
                    evidence = self._stop_process(
                        conflicting_process,
                        float(conflicting_process.timeout_s),
                    )
                    if evidence:
                        report.stop_evidence[conflicting_process.name] = evidence
                report.stopped.append(target)

            for stage in self._stages(processes):
                stage_start = self._monotonic()
                stage_deadline = self._stage_deadline(stage_start, stage)
                for process in stage:
                    process_deadline = self._process_deadline(stage_start, process)
                    if self._manager.active(process.target):
                        if process.lifecycle != "persistent":
                            raise ProcessError(f"mode process remained active after conflict stop: {process.target}")
                        report.preserved.append(process.target)
                    else:
                        started.append(process)
                        report.started.append(process.target)
                        self._manager.start(
                            process.target,
                            self._remaining_timeout(
                                min(stage_deadline, process_deadline),
                            ),
                        )
                for process in sorted(
                    stage,
                    key=lambda item: (
                        self._process_deadline(stage_start, item),
                        item.name,
                    ),
                ):
                    process_deadline = self._process_deadline(stage_start, process)
                    report.ready[process.name] = self._readiness.wait(
                        process,
                        self._remaining_timeout(
                            min(stage_deadline, process_deadline),
                        ),
                    )
                    identity = self._runtime_identity(
                        process.target,
                        required=False,
                    )
                    if identity is not None:
                        report.identities[process.name] = identity
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            if not defer_rollback:
                self._rollback(plan, started, report)
            raise ProcessFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def apply_deferred(
        self,
        plan: RunPlan,
        *,
        dry_run: bool = False,
    ) -> ProcessReport:
        """Apply while leaving target cleanup to an outer transaction."""

        return self.apply(
            plan,
            dry_run=dry_run,
            defer_rollback=True,
        )

    def transition(
        self,
        previous: RunPlan,
        plan: RunPlan,
        *,
        dry_run: bool = False,
        defer_rollback: bool = False,
    ) -> ProcessReport:
        """Cold-restart mode processes between two exact RunPlans.

        Persistent processes remain active by lifecycle declaration. By
        default, processes started for the target are reclaimed on failure.
        ProductControl sets ``defer_rollback`` because resource compensation
        (notably map activation RESTORE through mapd) must run while the target
        process is still alive.  It then owns stop_transition_target().
        """

        self._validate_transition(previous, plan)
        previous_processes = self._require_processes(previous)
        processes = tuple(
            sorted(
                self._require_processes(plan),
                key=lambda item: (item.order, item.name),
            )
        )
        self._require_supported_managers(
            tuple({process.target: process for process in (*previous_processes, *processes)}.values())
        )
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="transition",
            dry_run=dry_run,
            planned=[process.target for process in processes],
        )
        if dry_run:
            report.ok = True
            report.status = "planned"
            return report

        stopped_targets: set[str] = set()
        started: list[ProcessSpec] = []
        mutation_processes = tuple(
            {process.target: process for process in (*previous_processes, *processes)}.values()
        )
        self._before_process_mutation(mutation_processes)
        try:
            for process in self._processes_in_stop_order(
                previous,
                previous.managed_processes,
            ):
                if not self._should_stop_process(process):
                    continue
                evidence = self._stop_process(process, float(process.timeout_s))
                if evidence:
                    report.stop_evidence[process.name] = evidence
                report.stopped.append(process.target)
                stopped_targets.add(process.target)

            process_by_target = {
                process.target: process
                for process in (*previous.available_processes, *plan.available_processes)
            }
            timeouts = {
                process.target: process.timeout_s
                for process in (*previous.available_processes, *plan.available_processes)
            }
            for target in plan.stop_before_start:
                if target in stopped_targets:
                    continue
                conflicting_process = process_by_target.get(target)
                if conflicting_process is None:
                    if not self._manager.active(target):
                        continue
                    self._stop_target(target, float(timeouts.get(target, 15)))
                else:
                    if conflicting_process.lifecycle == "persistent":
                        continue
                    if not self._should_stop_process(conflicting_process):
                        continue
                    evidence = self._stop_process(
                        conflicting_process,
                        float(conflicting_process.timeout_s),
                    )
                    if evidence:
                        report.stop_evidence[conflicting_process.name] = evidence
                report.stopped.append(target)
                stopped_targets.add(target)

            for stage in self._stages(processes):
                stage_start = self._monotonic()
                stage_deadline = self._stage_deadline(stage_start, stage)
                for process in stage:
                    process_deadline = self._process_deadline(stage_start, process)
                    if self._manager.active(process.target):
                        if process.lifecycle != "persistent":
                            raise ProcessError(
                                "mode process remained active after transition stop: "
                                f"{process.target}"
                            )
                        report.preserved.append(process.target)
                        continue
                    started.append(process)
                    report.started.append(process.target)
                    self._manager.start(
                        process.target,
                        self._remaining_timeout(
                            min(stage_deadline, process_deadline),
                        ),
                    )
                for process in sorted(
                    stage,
                    key=lambda item: (
                        self._process_deadline(stage_start, item),
                        item.name,
                    ),
                ):
                    process_deadline = self._process_deadline(stage_start, process)
                    report.ready[process.name] = self._readiness.wait(
                        process,
                        self._remaining_timeout(
                            min(stage_deadline, process_deadline),
                        ),
                    )
                    identity = self._runtime_identity(process.target, required=False)
                    if identity is not None:
                        report.identities[process.name] = identity
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            if not defer_rollback:
                self._rollback(plan, started, report)
            raise ProcessFailed(report) from exc

        report.ok = True
        report.status = "active"
        return report

    def stop_transition_target(
        self,
        plan: RunPlan,
        transition: ProcessReport,
    ) -> ProcessReport:
        """Stop target processes activated or rejected by one transition."""

        self._validate_plan(plan)
        changed_targets = (
            set(transition.started) | set(transition.stopped)
        ) - set(transition.preserved)
        processes = self._processes_in_stop_order(
            plan,
            tuple(
                process
                for process in self._require_processes(plan)
                if process.target in changed_targets
            ),
        )
        self._require_supported_managers(processes)
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="transition-stop-target",
            planned=[process.target for process in processes],
        )
        self._before_process_mutation(processes)
        failures: list[tuple[str, Exception]] = []
        for process in processes:
            try:
                if not self._should_stop_process(process):
                    continue
                evidence = self._stop_process(process, float(process.timeout_s))
                if evidence:
                    report.stop_evidence[process.name] = evidence
                report.stopped.append(process.target)
            except Exception as exc:
                failures.append((process.target, exc))
        if failures:
            details = "; ".join(
                f"{target}: {str(error) or error.__class__.__name__}"
                for target, error in failures
            )
            report.error = f"failed to stop transitioned target processes: {details}"
            report.status = "failed"
            raise ProcessFailed(report) from failures[0][1]
        report.ok = True
        report.status = "stopped"
        return report

    def ensure_transition_process_active(
        self,
        plan: RunPlan,
        transition: ProcessReport,
        process_name: str,
    ) -> ProcessReport:
        """Activate one changed process needed by an outer compensation."""

        self._validate_plan(plan)
        process = next(
            (
                candidate
                for candidate in self._require_processes(plan)
                if candidate.name == process_name
            ),
            None,
        )
        if process is None:
            raise ProcessError(
                f"transition compensation process is absent: {process_name}"
            )
        report = ProcessReport(
            product=plan.product,
            env=plan.env,
            action="transition-ensure-process",
            planned=[process.target],
        )
        if self._manager.active(process.target):
            report.ok = True
            report.status = "active"
            return report
        if process.target not in transition.stopped:
            transition.stopped.append(process.target)
        if process.target not in transition.started:
            transition.started.append(process.target)
        self._before_process_mutation((process,))
        try:
            self._manager.start(process.target, float(process.timeout_s))
        except Exception as exc:
            if not self._manager.active(process.target):
                report.error = str(exc) or exc.__class__.__name__
                report.status = "failed"
                raise ProcessFailed(report) from exc
        report.started.append(process.target)
        report.ok = True
        report.status = "active"
        return report

    def restore_transition_previous(
        self,
        previous: RunPlan,
        transition: ProcessReport,
    ) -> ProcessReport:
        """Restart only previous processes stopped by one transition."""

        self._validate_plan(previous)
        processes = tuple(
            sorted(
                self._require_processes(previous),
                key=lambda item: (item.order, item.name),
            )
        )
        self._require_supported_managers(processes)
        stopped_targets = set(transition.stopped)
        to_restore = tuple(
            process
            for process in processes
            if process.target in stopped_targets
        )
        report = ProcessReport(
            product=previous.product,
            env=previous.env,
            action="transition-restore-previous",
            planned=[process.target for process in to_restore],
        )
        self._before_process_mutation(to_restore)
        started: list[ProcessSpec] = []
        try:
            for stage in self._stages(processes):
                stage_start = self._monotonic()
                stage_deadline = self._stage_deadline(stage_start, stage)
                for process in stage:
                    process_deadline = self._process_deadline(stage_start, process)
                    if process in to_restore:
                        if self._manager.active(process.target):
                            raise ProcessError(
                                "target process remained active before previous restore: "
                                f"{process.target}"
                            )
                        started.append(process)
                        report.started.append(process.target)
                        self._manager.start(
                            process.target,
                            self._remaining_timeout(
                                min(stage_deadline, process_deadline),
                            ),
                        )
                    if not self._manager.active(process.target):
                        raise ProcessError(
                            f"previous process is inactive after restore: {process.target}"
                        )
                for process in sorted(
                    stage,
                    key=lambda item: (
                        self._process_deadline(stage_start, item),
                        item.name,
                    ),
                ):
                    process_deadline = self._process_deadline(stage_start, process)
                    report.ready[process.name] = self._readiness.wait(
                        process,
                        self._remaining_timeout(
                            min(stage_deadline, process_deadline),
                        ),
                    )
                    identity = self._runtime_identity(process.target, required=False)
                    if identity is not None:
                        report.identities[process.name] = identity
        except Exception as exc:
            report.error = str(exc) or exc.__class__.__name__
            report.status = "failed"
            self._rollback(previous, started, report)
            raise ProcessFailed(report) from exc
        report.ok = True
        report.status = "active"
        return report

    def stop(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        """Stop the plan's mode processes and preserve persistent ones."""

        self._validate_plan(plan)
        self._require_processes(plan)
        processes = self._processes_in_stop_order(
            plan,
            plan.managed_processes,
        )
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

        self._before_process_mutation(processes)
        failures: list[tuple[str, Exception]] = []
        for process in processes:
            try:
                if not self._should_stop_process(process):
                    continue
                evidence = self._stop_process(process, float(process.timeout_s))
                if evidence:
                    report.stop_evidence[process.name] = evidence
                report.stopped.append(process.target)
            except Exception as exc:
                failures.append((process.target, exc))
        if failures:
            details = "; ".join(
                f"{target}: {str(error) or error.__class__.__name__}"
                for target, error in failures
            )
            report.error = f"failed to stop Product processes: {details}"
            report.status = "failed"
            raise ProcessFailed(report) from failures[0][1]
        report.ok = True
        report.status = "stopped"
        return report

    def quiesce(self, plan: RunPlan, *, dry_run: bool = False) -> ProcessReport:
        """Stop every mode target that can conflict with this plan."""

        self._validate_plan(plan)
        self._require_processes(plan)
        targets = tuple(plan.stop_before_start)
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

        process_by_target = {
            process.target: process for process in plan.available_processes
        }
        timeouts = {
            process.target: process.timeout_s for process in plan.available_processes
        }
        failures: list[tuple[str, Exception]] = []
        for target in targets:
            try:
                process = process_by_target.get(target)
                if process is None:
                    if not self._manager.active(target):
                        continue
                    self._stop_target(target, float(timeouts.get(target, 15)))
                else:
                    if not self._should_stop_process(process):
                        continue
                    evidence = self._stop_process(
                        process,
                        float(process.timeout_s),
                    )
                    if evidence:
                        report.stop_evidence[process.name] = evidence
                report.stopped.append(target)
            except Exception as exc:
                failures.append((target, exc))
        if failures:
            details = "; ".join(f"{target}: {str(error) or error.__class__.__name__}" for target, error in failures)
            report.error = f"failed to quiesce Product processes: {details}"
            report.status = "failed"
            raise ProcessFailed(report) from failures[0][1]
        report.ok = True
        report.status = "stopped"
        return report

    def _validate_transition(self, previous: RunPlan, plan: RunPlan) -> None:
        self._validate_plan(previous)
        self._validate_plan(plan)
        if previous.env != plan.env:
            raise ProcessError("process transition RunPlans must use the same Env")
        if previous.process_control != plan.process_control:
            raise ProcessError(
                "process transition RunPlans must use the same controller"
            )

    def _runtime_identity(
        self,
        target: str,
        *,
        required: bool,
    ) -> Mapping[str, Any] | None:
        identity = getattr(self._manager, "identity", None)
        if identity is None:
            if required:
                raise ProcessError(
                    f"process manager cannot prove runtime identity: {target}"
                )
            return None
        value = identity(target)
        if type(value) is not dict or not value:
            raise ProcessError(f"process runtime identity is invalid: {target}")
        return dict(value)

    def _rollback(
        self,
        plan: RunPlan,
        started: list[ProcessSpec],
        report: ProcessReport,
    ) -> None:
        for process in self._processes_in_stop_order(plan, tuple(started)):
            try:
                if self._should_stop_process(process):
                    evidence = self._stop_process(
                        process,
                        float(process.timeout_s),
                    )
                    if evidence:
                        report.stop_evidence[process.name] = evidence
                report.rolled_back.append(process.target)
            except Exception as exc:
                report.rollback_errors.append(f"{process.target}: {exc}")

    @staticmethod
    def _processes_in_stop_order(
        plan: RunPlan,
        processes: tuple[ProcessSpec, ...],
    ) -> tuple[ProcessSpec, ...]:
        """Order owned processes by the RunPlan stop contract.

        Older synthetic plans may omit some targets. Those processes retain
        the historical reverse-start fallback after every explicitly ordered
        target.
        """

        by_target = {process.target: process for process in processes}
        ordered = [
            by_target[target]
            for target in plan.stop_before_start
            if target in by_target
        ]
        declared = {process.target for process in ordered}
        ordered.extend(
            process
            for process in reversed(processes)
            if process.target not in declared
        )
        return tuple(ordered)

    def _stages(
        self,
        processes: tuple[ProcessSpec, ...],
    ) -> tuple[tuple[ProcessSpec, ...], ...]:
        stages: list[tuple[ProcessSpec, ...]] = []
        current_order: int | None = None
        current: list[ProcessSpec] = []
        for process in processes:
            if current_order is None or process.order == current_order:
                current_order = process.order
                current.append(process)
                continue
            stages.append(tuple(current))
            current_order = process.order
            current = [process]
        if current:
            stages.append(tuple(current))
        return tuple(stages)

    def _stage_deadline(
        self,
        stage_start: float,
        stage: tuple[ProcessSpec, ...],
    ) -> float:
        budget = max(float(process.timeout_s) for process in stage)
        return stage_start + budget

    @staticmethod
    def _process_deadline(
        stage_start: float,
        process: ProcessSpec,
    ) -> float:
        return stage_start + float(process.timeout_s)

    def _remaining_timeout(self, deadline: float) -> float:
        remaining = deadline - self._monotonic()
        if remaining <= 0:
            raise ProcessError("process stage deadline expired")
        return remaining

    def _stop_target(self, target: str, timeout_s: float) -> None:
        """Stop a conflict tombstone that is not tied to a selected process."""

        self._manager.stop(target, timeout_s)

    def _stop_process(
        self,
        process: ProcessSpec,
        timeout_s: float,
    ) -> Mapping[str, Any]:
        """Stop one typed process through the process-aware safe-stop seam."""

        stop_process = getattr(self._manager, "stop_process", None)
        if stop_process is None:
            self._manager.stop(process.target, timeout_s)
            return {}
        evidence = stop_process(process, timeout_s)
        if type(evidence) is not dict:
            raise ProcessError(
                f"process stop evidence is invalid: {process.name}"
            )
        return dict(evidence)

    @staticmethod
    def _requires_stop_evidence(process: ProcessSpec) -> bool:
        command = process.command
        return (
            command is not None
            and command.shutdown is not None
            and command.shutdown.kind == "file"
        )

    def _should_stop_process(self, process: ProcessSpec) -> bool:
        """Keep evidence-bearing stops independent of transient liveness probes."""

        return self._requires_stop_evidence(process) or self._manager.active(
            process.target
        )

    def _require_processes(self, plan: RunPlan) -> tuple[ProcessSpec, ...]:
        if plan.process_control != self._controller:
            raise ProcessError(
                f"Product {plan.product} is controlled by {plan.process_control}, not {self._runner_name}"
            )
        if not plan.processes:
            raise ProcessError(f"Product {plan.product} has no deployment processes in Env {plan.env}")
        return plan.processes

    def _require_supported_managers(
        self,
        processes: tuple[ProcessSpec, ...],
    ) -> None:
        unsupported = sorted({process.manager for process in processes if process.manager != self._process_manager})
        if unsupported:
            raise ProcessError(f"unsupported process managers: {', '.join(unsupported)}")

    def _validate_plan(self, plan: RunPlan) -> None:
        if plan.process_control != "systemd" or plan.env != "real":
            raise ProcessError("SystemdRunner requires a real/systemd RunPlan")

    def _before_process_mutation(
        self,
        processes: tuple[ProcessSpec, ...],
    ) -> None:
        current_unit = str(self._environment.get("LINGTU_SYSTEMD_UNIT") or "").strip()
        if current_unit and any(process.target == current_unit for process in processes):
            raise ProcessError(
                f"SystemdRunner must run outside {current_unit}; "
                "use an independent transient unit"
            )


__all__ = [
    "PROCESS_REPORT_SCHEMA",
    "ProcessError",
    "ProcessFailed",
    "ProcessManager",
    "ProcessReport",
    "Readiness",
    "ServiceReadiness",
    "Systemd",
    "SystemdRunner",
]
