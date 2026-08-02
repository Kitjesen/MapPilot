# ruff: noqa: S101

from __future__ import annotations

from typing import Any

import pytest

from lingtu.assembly.products import resolve_product_host_runtime
from lingtu.assembly.profile_builder import compile_run_plan
from lingtu.systemd import (
    SystemdRunner,
    ProcessError,
    ProcessFailed,
    ServiceReadiness,
)
from runtime.graph import ProcessSpec


class FakeControl:
    def __init__(self, active: set[str] | None = None) -> None:
        self.active_targets = set(active or ())
        self.started: list[str] = []
        self.stopped: list[str] = []

    def available(self, target: str) -> bool:
        return True

    def active(self, target: str) -> bool:
        return target in self.active_targets

    def start(self, target: str, timeout_s: float) -> None:
        self.started.append(target)
        self.active_targets.add(target)

    def stop(self, target: str, timeout_s: float) -> None:
        self.stopped.append(target)
        self.active_targets.discard(target)


class FakeReadiness:
    def __init__(self, fail: str | None = None) -> None:
        self.fail = fail
        self.calls: list[str] = []

    def wait(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
        self.calls.append(process.name)
        if process.name == self.fail:
            raise ProcessError(f"{process.name} readiness failed")
        return {"ready": True, "target": process.target}


def _field_product():
    resolved = resolve_product_host_runtime("nav", "real")
    return compile_run_plan(
        resolved.product,
        resolved.env,
        resolved.config,
    )


def test_systemd_runner_dry_run_has_no_process_side_effects() -> None:
    control = FakeControl()
    readiness = FakeReadiness()

    report = SystemdRunner(control, readiness).apply(_field_product(), dry_run=True)

    assert report.ok is True
    assert report.status == "planned"
    assert report.planned == [
        "lingtu-livox-dds.service",
        "lingtu-slam-dds.service",
        "mapd.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu-driver.service",
        "lingtu.service",
    ]
    assert control.started == []
    assert control.stopped == []
    assert readiness.calls == []


def test_systemd_runner_applies_plan_in_order_and_preserves_running_driver() -> None:
    control = FakeControl(
        {
            "lingtu-driver.service",
            "lingtu-teleop-dds.service",
        }
    )
    readiness = FakeReadiness()

    report = SystemdRunner(control, readiness).apply(_field_product())

    assert report.ok is True
    assert report.status == "active"
    assert control.stopped == ["lingtu-teleop-dds.service"]
    assert control.started == [
        "lingtu-livox-dds.service",
        "lingtu-slam-dds.service",
        "mapd.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu.service",
    ]
    assert report.preserved == ["lingtu-driver.service"]
    assert readiness.calls == [
        "lidar",
        "slam",
        "maps",
        "traversability",
        "nav",
        "driver",
        "host",
    ]


def test_systemd_runner_rolls_back_only_processes_started_by_failed_transaction() -> None:
    control = FakeControl({"lingtu-driver.service"})
    readiness = FakeReadiness(fail="nav")

    with pytest.raises(ProcessFailed) as failure:
        SystemdRunner(control, readiness).apply(_field_product())

    report = failure.value.report
    assert report.status == "failed"
    assert report.error == "nav readiness failed"
    assert report.rolled_back == [
        "lingtu-nav-dds.service",
        "lingtu-traversability-dds.service",
        "mapd.service",
        "lingtu-slam-dds.service",
        "lingtu-livox-dds.service",
    ]
    assert "lingtu-driver.service" in control.active_targets
    assert "lingtu-driver.service" not in control.stopped


def test_systemd_runner_rejects_stopping_its_own_systemd_unit() -> None:
    control = FakeControl()

    with pytest.raises(ProcessError, match=r"must run outside lingtu\.service"):
        SystemdRunner(
            control,
            FakeReadiness(),
            environment={"LINGTU_SYSTEMD_UNIT": "lingtu.service"},
        ).apply(_field_product())

    assert control.started == []
    assert control.stopped == []


def test_systemd_runner_stop_preserves_persistent_processes() -> None:
    product = _field_product()
    control = FakeControl({process.target for process in product.processes})

    report = SystemdRunner(control, FakeReadiness()).stop(product)

    assert report.ok is True
    assert report.stopped == [
        "lingtu.service",
        "lingtu-nav-dds.service",
        "lingtu-traversability-dds.service",
        "mapd.service",
        "lingtu-slam-dds.service",
        "lingtu-livox-dds.service",
    ]
    assert "lingtu-driver.service" in control.active_targets


def test_systemd_runner_restarts_one_product_process_and_checks_readiness() -> None:
    control = FakeControl({"lingtu-slam-dds.service"})
    readiness = FakeReadiness()

    report = SystemdRunner(control, readiness).restart(_field_product(), "slam")

    assert report.ok is True
    assert report.action == "restart"
    assert report.stopped == ["lingtu-slam-dds.service"]
    assert report.started == ["lingtu-slam-dds.service"]
    assert readiness.calls == ["slam"]


def test_systemd_runner_failed_restart_restores_previously_active_process() -> None:
    control = FakeControl({"lingtu-slam-dds.service"})
    readiness = FakeReadiness(fail="slam")

    with pytest.raises(ProcessFailed) as failure:
        SystemdRunner(control, readiness).restart(_field_product(), "slam")

    report = failure.value.report
    assert report.status == "failed"
    assert control.stopped == [
        "lingtu-slam-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-slam-dds.service",
    ]
    assert control.started == [
        "lingtu-slam-dds.service",
        "lingtu-slam-dds.service",
    ]
    assert report.rolled_back == []
    assert report.rollback_errors == ["restore failed lingtu-slam-dds.service: slam readiness failed"]
    assert "lingtu-slam-dds.service" not in control.active_targets


def test_systemd_runner_rejects_restart_of_process_outside_product() -> None:
    with pytest.raises(ProcessError, match="is not in"):
        SystemdRunner(FakeControl(), FakeReadiness()).restart(
            _field_product(),
            "legacy_localizer",
        )


def test_systemd_runner_quiesce_stops_all_conflicting_mode_targets() -> None:
    product = _field_product()
    control = FakeControl(set(product.stop_targets))

    report = SystemdRunner(control, FakeReadiness()).quiesce(product)

    assert report.ok is True
    assert report.action == "quiesce"
    assert control.stopped == list(product.stop_targets)
    assert control.active_targets == set()

def test_systemd_runner_quiesce_attempts_every_target_and_aggregates_stop_failures() -> None:
    product = _field_product()
    failed_targets = {product.stop_targets[0], product.stop_targets[-1]}

    class FailingStopControl(FakeControl):
        def stop(self, target: str, timeout_s: float) -> None:
            self.stopped.append(target)
            if target in failed_targets:
                raise RuntimeError(f"stop failed for {target}")
            self.active_targets.discard(target)

    control = FailingStopControl(set(product.stop_targets))

    with pytest.raises(ProcessFailed) as failure:
        SystemdRunner(control, FakeReadiness()).quiesce(product)

    report = failure.value.report
    assert control.stopped == list(product.stop_targets)
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
        target="lingtu.service",
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
