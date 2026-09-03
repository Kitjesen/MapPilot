from __future__ import annotations

import io
import json
import subprocess
import urllib.error
from typing import Any

import pytest

from lingtu.assembly.compiler import compile_run_plan
from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.real.systemd import (
    ServiceReadiness,
    Systemd,
    SystemdRunner,
    _ServiceInspector,
)
from lingtu.switch_contracts import ProcessError, ProcessFailed
from runtime.graph import ProcessSpec


class FakeControl:
    def __init__(
        self,
        active: set[str] | None = None,
        identities: dict[str, dict[str, int | str]] | None = None,
    ) -> None:
        self.active_targets = set(active or ())
        self.started: list[str] = []
        self.stopped: list[str] = []
        self._next_pid = 1000
        self.identities = dict(identities or {})
        for target in sorted(self.active_targets):
            self.identities.setdefault(target, self._new_identity())

    def _new_identity(self) -> dict[str, int | str]:
        self._next_pid += 1
        return {
            "pid": self._next_pid,
            "invocation_id": f"fake-invocation-{self._next_pid}",
        }

    def available(self, target: str) -> bool:
        return True

    def active(self, target: str) -> bool:
        return target in self.active_targets

    def identity(self, target: str) -> dict[str, int | str]:
        if target not in self.active_targets:
            raise ProcessError(f"target is not active: {target}")
        return dict(self.identities[target])

    def start(self, target: str, timeout_s: float) -> None:
        self.started.append(target)
        self.active_targets.add(target)
        self.identities[target] = self._new_identity()

    def stop(self, target: str, timeout_s: float) -> None:
        self.stopped.append(target)
        self.active_targets.discard(target)
        self.identities.pop(target, None)


class FakeReadiness:
    def __init__(self, fail: str | None = None) -> None:
        self.fail = fail
        self.calls: list[str] = []

    def wait(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
        self.calls.append(process.name)
        if process.name == self.fail:
            raise ProcessError(f"{process.name} readiness failed")
        return {"ready": True, "target": process.target}


def test_systemd_identity_returns_positive_pid_and_invocation_id() -> None:
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(
            command,
            0,
            stdout="MainPID=1234\nInvocationID=0123456789abcdef\n",
            stderr="",
        )

    identity = Systemd(use_sudo=False, runner=run).identity("lt-maps.service")

    assert identity == {"pid": 1234, "invocation_id": "0123456789abcdef"}
    assert calls == [
        [
            "systemctl",
            "show",
            "--property=MainPID",
            "--property=InvocationID",
            "lt-maps.service",
        ]
    ]


@pytest.mark.parametrize(
    "returncode,stdout",
    [
        (1, ""),
        (0, "MainPID=0\nInvocationID=0123456789abcdef\n"),
        (0, "MainPID=not-a-pid\nInvocationID=0123456789abcdef\n"),
        (0, "MainPID=1234\nInvocationID=\n"),
        (0, "MainPID=1234\n"),
    ],
)
def test_systemd_identity_rejects_failed_or_invalid_show_output(
    returncode: int,
    stdout: str,
) -> None:
    def run(command, **_kwargs):
        return subprocess.CompletedProcess(
            command,
            returncode,
            stdout=stdout,
            stderr="unit lookup failed" if returncode else "",
        )

    with pytest.raises(ProcessError, match=r"systemd identity failed for lt-maps\.service"):
        Systemd(use_sudo=False, runner=run).identity("lt-maps.service")


def _field_product():
    resolved = resolve_product_host_runtime("nav", "real", robot="unitree/go2")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        robot="unitree/go2",
    )


def test_systemd_runner_dry_run_has_no_process_side_effects() -> None:
    control = FakeControl()
    readiness = FakeReadiness()

    report = SystemdRunner(control, readiness).apply(_field_product(), dry_run=True)

    assert report.ok is True
    assert report.status == "planned"
    assert report.planned == [
        "lt-lidar.service",
        "lt-slam.service",
        "lt-maps.service",
        "lt-nav.service",
        "lt-driver.service",
        "lt-host.service",
    ]
    assert control.started == []
    assert control.stopped == []
    assert readiness.calls == []


def test_systemd_runner_applies_plan_in_order_and_restarts_active_mode_driver() -> None:
    control = FakeControl(
        {
            "lt-driver.service",
            "lingtu-teleop-dds.service",
        }
    )
    readiness = FakeReadiness()

    report = SystemdRunner(control, readiness).apply(_field_product())

    assert report.ok is True
    assert report.status == "active"
    assert report.stop_evidence == {}
    assert control.stopped == ["lt-driver.service", "lingtu-teleop-dds.service"]
    assert control.started == [
        "lt-lidar.service",
        "lt-slam.service",
        "lt-maps.service",
        "lt-nav.service",
        "lt-driver.service",
        "lt-host.service",
    ]
    assert report.preserved == []
    assert readiness.calls == [
        "lidar",
        "slam",
        "maps",
        "nav",
        "driver",
        "host",
    ]


def test_systemd_runner_rolls_back_only_processes_started_by_failed_transaction() -> None:
    control = FakeControl({"lt-driver.service"})
    readiness = FakeReadiness(fail="nav")

    with pytest.raises(ProcessFailed) as failure:
        SystemdRunner(control, readiness).apply(_field_product())

    report = failure.value.report
    assert report.status == "failed"
    assert report.error == "nav readiness failed"
    assert report.rolled_back == [
        "lt-nav.service",
        "lt-maps.service",
        "lt-slam.service",
        "lt-lidar.service",
    ]
    assert "lt-driver.service" not in control.active_targets
    assert "lt-driver.service" in control.stopped


def test_systemd_runner_rejects_stopping_its_own_systemd_unit() -> None:
    control = FakeControl()

    with pytest.raises(ProcessError, match=r"must run outside lt-host\.service"):
        SystemdRunner(
            control,
            FakeReadiness(),
            environment={"LINGTU_SYSTEMD_UNIT": "lt-host.service"},
        ).apply(_field_product())

    assert control.started == []
    assert control.stopped == []


def test_systemd_runner_stop_stops_all_mode_processes() -> None:
    product = _field_product()
    control = FakeControl({process.target for process in product.processes})

    report = SystemdRunner(control, FakeReadiness()).stop(product)

    assert report.ok is True
    assert report.stop_evidence == {}
    assert report.stopped == [
        "lt-host.service",
        "lt-nav.service",
        "lt-driver.service",
        "lt-maps.service",
        "lt-slam.service",
        "lt-lidar.service",
    ]
    assert "lt-driver.service" not in control.active_targets


def test_systemd_runner_quiesce_stops_all_conflicting_mode_targets() -> None:
    product = _field_product()
    control = FakeControl(set(product.stop_before_start))

    report = SystemdRunner(control, FakeReadiness()).quiesce(product)

    assert report.ok is True
    assert report.action == "quiesce"
    assert control.stopped == list(product.stop_before_start)
    assert control.active_targets == set()


def test_systemd_runner_quiesce_attempts_every_target_and_aggregates_stop_failures() -> None:
    product = _field_product()
    failed_targets = {product.stop_before_start[0], product.stop_before_start[-1]}

    class FailingStopControl(FakeControl):
        def stop(self, target: str, timeout_s: float) -> None:
            self.stopped.append(target)
            if target in failed_targets:
                raise RuntimeError(f"stop failed for {target}")
            self.active_targets.discard(target)

    control = FailingStopControl(set(product.stop_before_start))

    with pytest.raises(ProcessFailed) as failure:
        SystemdRunner(control, FakeReadiness()).quiesce(product)

    report = failure.value.report
    assert control.stopped == list(product.stop_before_start)
    assert report.ok is False
    assert report.status == "failed"
    for failed_target in failed_targets:
        assert f"{failed_target}: stop failed for {failed_target}" in (report.error or "")
    assert control.active_targets == failed_targets


def test_service_readiness_rejects_product_catalog_drift() -> None:
    process = ProcessSpec(
        name="nav",
        manager="systemd",
        target="wrong-nav.service",
        order=1,
        timeout_s=1,
        lifecycle="mode",
    )

    with pytest.raises(ProcessError, match="disagrees with readiness catalog"):
        ServiceReadiness().wait(process, 1.0)


def test_service_readiness_allows_inactive_navigation_when_data_is_safe() -> None:
    calls = []

    class Manager:
        def status_details(self, service, *, dds_check, http_check):
            calls.append((service, dds_check, http_check))
            return {
                service: {
                    "ready": True,
                    "blockers": [],
                    "observed": {
                        "http": {
                            "payload": {
                                "ready": False,
                                "motion_ready": False,
                                "data_ready": True,
                                "non_motion_safe": True,
                                "failed_modules": [],
                                "critical_failed_modules": [],
                            }
                        }
                    },
                }
            }

    process = ProcessSpec(
        name="gateway",
        manager="systemd",
        target="lt-host.service",
        order=1,
        timeout_s=1,
        lifecycle="mode",
    )
    readiness = ServiceReadiness(
        manager=Manager(),
        sleep=lambda _seconds: None,
        monotonic=lambda: 0.0,
    )

    detail = readiness.wait(process, 1.0)

    assert calls == [("gateway", True, True)]
    assert detail["ready"] is True
    assert detail["observed"]["http"]["payload"]["motion_ready"] is False


def test_service_http_accepts_safe_degraded_readiness(monkeypatch: pytest.MonkeyPatch) -> None:
    payload = {
        "ready": False,
        "motion_ready": False,
        "data_ready": True,
        "non_motion_safe": True,
        "failed_modules": [],
        "critical_failed_modules": [],
    }

    def urlopen(*_args: Any, **_kwargs: Any) -> None:
        raise urllib.error.HTTPError(
            "http://127.0.0.1:5050/ready",
            503,
            "Service Unavailable",
            {},
            io.BytesIO(json.dumps(payload).encode("utf-8")),
        )

    monkeypatch.setattr("lingtu.real.systemd.urllib.request.urlopen", urlopen)
    blockers: list[str] = []

    observed = _ServiceInspector._http("gateway", blockers)

    assert blockers == []
    assert observed["ok"] is True
    assert observed["status_code"] == 503
    assert observed["payload"] == payload


def test_service_http_rejects_degraded_readiness_without_safe_stop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    payload = {
        "ready": False,
        "motion_ready": False,
        "data_ready": True,
        "non_motion_safe": False,
        "failed_modules": [],
        "critical_failed_modules": [],
    }

    def urlopen(*_args: Any, **_kwargs: Any) -> None:
        raise urllib.error.HTTPError(
            "http://127.0.0.1:5050/ready",
            503,
            "Service Unavailable",
            {},
            io.BytesIO(json.dumps(payload).encode("utf-8")),
        )

    monkeypatch.setattr("lingtu.real.systemd.urllib.request.urlopen", urlopen)
    blockers: list[str] = []

    observed = _ServiceInspector._http("gateway", blockers)

    assert observed["ok"] is False
    assert "http_non_motion_safe_false" in blockers
    assert "http_status:503" in blockers
