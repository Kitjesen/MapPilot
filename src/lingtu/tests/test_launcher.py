from __future__ import annotations

from typing import Any

import pytest

from lingtu.assembly.profile_builder import compile_product
from lingtu.launcher import (
    LaunchError,
    LaunchFailed,
    Launcher,
    ServiceReadiness,
)
from runtime.graph import RuntimeProcess
from runtime.profiles.resolver import resolve_runtime_config


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

    def wait(self, process: RuntimeProcess, timeout_s: float) -> dict[str, Any]:
        self.calls.append(process.name)
        if process.name == self.fail:
            raise LaunchError(f"{process.name} readiness failed")
        return {"ready": True, "target": process.target}


def _field_product():
    resolved = resolve_runtime_config("nav")
    return compile_product(
        resolved.profile,
        resolved.config,
        endpoint=resolved.runtime_endpoint,
    )


def test_launcher_dry_run_has_no_process_side_effects() -> None:
    control = FakeControl()
    readiness = FakeReadiness()

    report = Launcher(control, readiness).apply(_field_product(), dry_run=True)

    assert report.ok is True
    assert report.status == "planned"
    assert report.planned == [
        "lingtu-livox-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu-driver.service",
        "lingtu.service",
    ]
    assert control.started == []
    assert control.stopped == []
    assert readiness.calls == []


def test_launcher_applies_plan_in_order_and_preserves_running_driver() -> None:
    control = FakeControl(
        {
            "lingtu-driver.service",
            "lingtu-teleop-dds.service",
        }
    )
    readiness = FakeReadiness()

    report = Launcher(control, readiness).apply(_field_product())

    assert report.ok is True
    assert report.status == "active"
    assert control.stopped == ["lingtu-teleop-dds.service"]
    assert control.started == [
        "lingtu-livox-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-traversability-dds.service",
        "lingtu-nav-dds.service",
        "lingtu.service",
    ]
    assert report.preserved == ["lingtu-driver.service"]
    assert readiness.calls == [
        "lidar",
        "slam",
        "traversability",
        "nav",
        "driver",
        "runtime",
    ]


def test_launcher_rolls_back_only_processes_started_by_failed_transaction() -> None:
    control = FakeControl({"lingtu-driver.service"})
    readiness = FakeReadiness(fail="nav")

    with pytest.raises(LaunchFailed) as failure:
        Launcher(control, readiness).apply(_field_product())

    report = failure.value.report
    assert report.status == "failed"
    assert report.error == "nav readiness failed"
    assert report.rolled_back == [
        "lingtu-nav-dds.service",
        "lingtu-traversability-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-livox-dds.service",
    ]
    assert "lingtu-driver.service" in control.active_targets
    assert "lingtu-driver.service" not in control.stopped


def test_launcher_rejects_stopping_its_own_systemd_unit() -> None:
    control = FakeControl()

    with pytest.raises(LaunchError, match=r"must run outside lingtu\.service"):
        Launcher(
            control,
            FakeReadiness(),
            environment={"LINGTU_SYSTEMD_UNIT": "lingtu.service"},
        ).apply(_field_product())

    assert control.started == []
    assert control.stopped == []


def test_launcher_stop_preserves_persistent_processes() -> None:
    product = _field_product()
    assert product.plan is not None
    control = FakeControl({process.target for process in product.plan.processes})

    report = Launcher(control, FakeReadiness()).stop(product)

    assert report.ok is True
    assert report.stopped == [
        "lingtu.service",
        "lingtu-nav-dds.service",
        "lingtu-traversability-dds.service",
        "lingtu-slam-dds.service",
        "lingtu-livox-dds.service",
    ]
    assert "lingtu-driver.service" in control.active_targets


def test_launcher_restarts_one_runtime_plan_process_and_checks_readiness() -> None:
    control = FakeControl({"lingtu-slam-dds.service"})
    readiness = FakeReadiness()

    report = Launcher(control, readiness).restart(_field_product(), "slam")

    assert report.ok is True
    assert report.action == "restart"
    assert report.stopped == ["lingtu-slam-dds.service"]
    assert report.started == ["lingtu-slam-dds.service"]
    assert readiness.calls == ["slam"]


def test_launcher_failed_restart_restores_previously_active_process() -> None:
    control = FakeControl({"lingtu-slam-dds.service"})
    readiness = FakeReadiness(fail="slam")

    with pytest.raises(LaunchFailed) as failure:
        Launcher(control, readiness).restart(_field_product(), "slam")

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
    assert report.rollback_errors == [
        "restore failed lingtu-slam-dds.service: slam readiness failed"
    ]
    assert "lingtu-slam-dds.service" not in control.active_targets


def test_launcher_rejects_restart_of_process_outside_runtime_plan() -> None:
    with pytest.raises(LaunchError, match="is not in"):
        Launcher(FakeControl(), FakeReadiness()).restart(
            _field_product(),
            "legacy_localizer",
        )


def test_service_readiness_rejects_runtime_plan_catalog_drift() -> None:
    process = RuntimeProcess(
        name="nav",
        manager="systemd",
        target="wrong-nav.service",
        order=1,
        timeout_s=1,
        lifecycle="mode",
    )

    with pytest.raises(LaunchError, match="disagrees with readiness catalog"):
        ServiceReadiness().wait(process, 1.0)
