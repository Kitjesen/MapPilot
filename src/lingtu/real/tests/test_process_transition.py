from __future__ import annotations

from typing import Any

import pytest

from lingtu.real.systemd import SystemdRunner
from lingtu.run_plan import RunPlan
from lingtu.switch_contracts import ProcessError, ProcessFailed
from runtime.graph import ProcessSpec


class RecordingManager:
    def __init__(self, processes: tuple[ProcessSpec, ...]) -> None:
        self.active_targets = {process.target for process in processes}
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

    def stop_process(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
        self.stop(process.target, timeout_s)
        return {}


class RecordingReadiness:
    def __init__(self, fail: str | None = None) -> None:
        self.calls: list[str] = []
        self.fail = fail

    def wait(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
        self.calls.append(process.name)
        if process.name == self.fail:
            raise ProcessError(f"{process.name} readiness failed")
        return {"ready": True}


def _plan(product: str, processes: tuple[ProcessSpec, ...]) -> RunPlan:
    contract = {
        "teleop_avoid": "lingtu.product.teleop_avoid.v1",
        "nav": "lingtu.product.nav.v1",
    }[product]
    return RunPlan.create(
        product=product,
        env="real",
        robot="unitree/go2",
        process_control="systemd",
        modules=(),
        processes=processes,
        available_processes=processes,
        stop_before_start=tuple(process.target for process in reversed(processes)),
        contracts=(contract,),
        critical_modules=(),
        route_contract=None,
        host_config={},
        lifecycle={"product": product, "product_variant": None},
        native_nav={
            "control_mode": "teleop_avoid" if product == "teleop_avoid" else "autonomy",
            "publish_cmd_vel": True,
            "check_obstacle": True,
            "global_planner": "none" if product == "teleop_avoid" else "astar",
            "local_planner": "cmu",
            "use_traversability_cost": True,
            "allow_teleop_takeover": product == "nav",
            "teleop_local_planner": True,
        },
    )


def test_transition_cold_restarts_all_mode_processes() -> None:
    sensor = ProcessSpec("sensor", "systemd", "sensor.service", 10, 10, "mode")
    nav = ProcessSpec("nav", "systemd", "nav.service", 20, 10, "mode")
    previous = _plan("teleop_avoid", (sensor, nav))
    target = _plan("nav", (sensor, nav))
    manager = RecordingManager(previous.processes)
    readiness = RecordingReadiness()

    report = SystemdRunner(manager, readiness).transition(previous, target)

    assert report.ok is True
    assert report.preserved == []
    assert manager.stopped == [nav.target, sensor.target]
    assert manager.started == [sensor.target, nav.target]
    assert readiness.calls == ["sensor", "nav"]


def test_transition_keeps_persistent_processes() -> None:
    daemon = ProcessSpec("daemon", "systemd", "daemon.service", 5, 10, "persistent")
    nav = ProcessSpec("nav", "systemd", "nav.service", 20, 10, "mode")
    previous = _plan("teleop_avoid", (daemon, nav))
    target = _plan("nav", (daemon, nav))
    manager = RecordingManager(previous.processes)

    report = SystemdRunner(manager, RecordingReadiness()).transition(previous, target)

    assert report.preserved == [daemon.target]
    assert daemon.target not in manager.stopped
    assert daemon.target not in manager.started
    assert manager.stopped == [nav.target]
    assert manager.started == [nav.target]


def test_failed_transition_reclaims_target_before_previous_restore() -> None:
    sensor = ProcessSpec("sensor", "systemd", "sensor.service", 10, 10, "mode")
    nav = ProcessSpec("nav", "systemd", "nav.service", 20, 10, "mode")
    previous = _plan("teleop_avoid", (sensor, nav))
    target = _plan("nav", (sensor, nav))
    manager = RecordingManager(previous.processes)
    readiness = RecordingReadiness(fail="nav")
    runner = SystemdRunner(manager, readiness)

    with pytest.raises(ProcessFailed) as failure:
        runner.transition(previous, target)

    transition = failure.value.report
    assert transition.stopped == [nav.target, sensor.target]
    assert transition.started == [sensor.target, nav.target]
    assert transition.rolled_back == [nav.target, sensor.target]

    readiness.fail = None
    restored = runner.restore_transition_previous(previous, transition)

    assert restored.ok is True
    assert restored.started == [sensor.target, nav.target]
    assert manager.active_targets == {sensor.target, nav.target}
