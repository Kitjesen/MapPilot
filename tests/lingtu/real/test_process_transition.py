from __future__ import annotations

from typing import Any

import pytest

from lingtu.assembly.graph import ProcessSpec
from lingtu.real.systemd import ServiceReadiness, SystemdRunner
from lingtu.run_plan import RunPlan
from lingtu.switch_contracts import ProcessError, ProcessFailed, ProcessReport


class RecordingManager:
    def __init__(
        self,
        processes: tuple[ProcessSpec, ...],
        *,
        fail_start: dict[str, str] | None = None,
        fail_stop: dict[str, str] | None = None,
    ) -> None:
        self.active_targets = {process.target for process in processes}
        self.started: list[str] = []
        self.stopped: list[str] = []
        self.fail_start = dict(fail_start or {})
        self.fail_stop = dict(fail_stop or {})

    def available(self, target: str) -> bool:
        return True

    def active(self, target: str) -> bool:
        return target in self.active_targets

    def start(self, target: str, timeout_s: float) -> None:
        self.started.append(target)
        if target in self.fail_start:
            raise ProcessError(self.fail_start[target])
        self.active_targets.add(target)

    def stop(self, target: str, timeout_s: float) -> None:
        self.stopped.append(target)
        if target in self.fail_stop:
            raise ProcessError(self.fail_stop[target])
        self.active_targets.discard(target)

    def stop_process(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
        self.stop(process.target, timeout_s)
        return {}


class RecordingReadiness:
    def __init__(self, fail: str | None = None) -> None:
        self.calls: list[str] = []
        self.inspect_calls: list[str] = []
        self.fail = fail

    def wait(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
        self.calls.append(process.name)
        if process.name == self.fail:
            raise ProcessError(f"{process.name} readiness failed")
        return {"ready": True}

    def inspect(self, process: ProcessSpec) -> dict[str, Any]:
        self.inspect_calls.append(process.name)
        if process.name == self.fail:
            raise ProcessError(f"{process.name} readiness failed")
        return {"ready": True}


def _plan(product: str, processes: tuple[ProcessSpec, ...]) -> RunPlan:
    return RunPlan.create(
        product=product,
        env="real",
        robot="unitree/go2",
        process_control="systemd",
        modules=(),
        processes=processes,
        available_processes=processes,
        stop_before_start=tuple(process.target for process in reversed(processes)),
        required_topics=(),
        required_capabilities=(),
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


@pytest.mark.parametrize(
    ("failure", "reason"),
    [
        ("missing", "unit_missing"),
        ("start", "start_failed"),
        ("readiness", "readiness_failed"),
    ],
)
def test_apply_keeps_product_active_when_optional_video_fails(
    failure: str,
    reason: str,
) -> None:
    camera = ProcessSpec("camera", "systemd", "camera.service", 10, 10, "mode")
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        20,
        5,
        "mode",
        required=False,
    )
    host = ProcessSpec("host", "systemd", "host.service", 30, 10, "mode")
    plan = _plan("nav", (camera, video, host))
    start_error = {
        "missing": "required systemd unit is not installed: video.service",
        "start": "systemd start failed for video.service",
    }.get(failure)
    manager = RecordingManager(
        (),
        fail_start={video.target: start_error} if start_error else None,
    )
    readiness = RecordingReadiness(fail="video" if failure == "readiness" else None)

    report = SystemdRunner(manager, readiness).apply(plan)

    assert report.ok is True
    assert report.status == "active"
    assert report.degraded["video"]["reason"] == reason
    assert report.degraded["video"]["evidence"]["target"] == video.target
    assert host.target in manager.active_targets
    assert readiness.calls == (
        ["camera", "video", "host"] if failure == "readiness" else ["camera", "host"]
    )
    assert readiness.inspect_calls == []


@pytest.mark.parametrize("becomes_ready", [True, False])
def test_optional_video_uses_declared_startup_readiness_budget(becomes_ready: bool) -> None:
    video = ProcessSpec(
        "video", "systemd", "lt-video.service", 20, 5, "mode", required=False,
    )
    host = ProcessSpec("host", "systemd", "lt-host.service", 30, 10, "mode")
    now = [0.0]
    probes: list[str] = []

    class VideoStartup:
        def status_details(self, service, *, dds_check, http_check, target):
            probes.append(service)
            ready = service != "video" or (becomes_ready and now[0] >= 0.75)
            return {
                service: {"ready": ready, "blockers": [] if ready else ["waiting_for_source"]}
            }

    def sleep(seconds: float) -> None:
        now[0] += seconds

    readiness = ServiceReadiness(
        manager=VideoStartup(), sleep=sleep, monotonic=lambda: now[0],
    )
    manager = RecordingManager(())
    report = SystemdRunner(manager, readiness).apply(_plan("nav", (video, host)))

    assert report.ok is True
    assert host.target in manager.active_targets
    assert probes.count("video") >= 4
    if becomes_ready:
        assert 0.75 <= now[0] < video.timeout_s
        assert report.ready["video"]["ready"] is True
        assert "video" not in report.degraded
    else:
        assert now[0] >= 4.75
        assert "video" not in report.ready
        assert report.degraded["video"]["reason"] == "readiness_failed"


def test_required_failure_rolls_back_started_optional_video() -> None:
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        10,
        5,
        "mode",
        required=False,
    )
    nav = ProcessSpec("nav", "systemd", "nav.service", 20, 10, "mode")
    plan = _plan("nav", (video, nav))
    manager = RecordingManager(())

    with pytest.raises(ProcessFailed) as failure:
        SystemdRunner(manager, RecordingReadiness(fail="nav")).apply(plan)

    assert failure.value.report.rolled_back == [nav.target, video.target]
    assert video.target in manager.stopped
    assert manager.active_targets == set()


def test_transition_from_camera_product_stops_optional_video() -> None:
    host = ProcessSpec("host", "systemd", "host.service", 20, 10, "mode")
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        10,
        5,
        "mode",
        required=False,
    )
    previous = _plan("nav", (video, host))
    target = _plan("nav", (host,))
    manager = RecordingManager(previous.processes)

    report = SystemdRunner(manager, RecordingReadiness()).transition(previous, target)

    assert report.ok is True
    assert video.target in manager.stopped
    assert video.target not in manager.active_targets


def test_transition_keeps_target_active_when_optional_video_is_unready() -> None:
    host = ProcessSpec("host", "systemd", "host.service", 30, 10, "mode")
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        20,
        5,
        "mode",
        required=False,
    )
    previous = _plan("nav", (host,))
    target = _plan("nav", (video, host))
    manager = RecordingManager(previous.processes)

    report = SystemdRunner(
        manager,
        RecordingReadiness(fail="video"),
    ).transition(previous, target)

    assert report.ok is True
    assert report.degraded["video"]["reason"] == "readiness_failed"
    assert host.target in manager.active_targets


def test_optional_false_readiness_snapshot_is_degraded_without_ready_callback() -> None:
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        20,
        5,
        "mode",
        required=False,
    )
    host = ProcessSpec("host", "systemd", "host.service", 30, 10, "mode")
    plan = _plan("nav", (video, host))
    ready_callbacks: list[str] = []

    class FalseSnapshotReadiness(RecordingReadiness):
        def wait(self, process: ProcessSpec, timeout_s: float) -> dict[str, Any]:
            if process.name != "video":
                return super().wait(process, timeout_s)
            self.calls.append(process.name)
            return {
                "ready": False,
                "blockers": ["video_source_not_fresh"],
                "observed": {"frame_age_ms": 6000},
            }

    report = SystemdRunner(
        RecordingManager(()),
        FalseSnapshotReadiness(),
    ).apply(
        plan,
        on_process_ready=lambda process: ready_callbacks.append(process.name),
    )

    assert report.ok is True
    assert "video" not in report.ready
    assert report.degraded["video"]["reason"] == "readiness_failed"
    assert "video_source_not_fresh" in report.degraded["video"]["evidence"]["error"]
    assert ready_callbacks == ["host"]


def test_restore_keeps_previous_product_active_when_optional_video_cannot_restart() -> None:
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        10,
        5,
        "mode",
        required=False,
    )
    host = ProcessSpec("host", "systemd", "host.service", 20, 10, "mode")
    previous = _plan("nav", (video, host))
    manager = RecordingManager(
        (),
        fail_start={video.target: "systemd start failed for video.service"},
    )
    transition = ProcessReport(
        product="nav",
        env="real",
        action="transition",
        stopped=[video.target, host.target],
    )

    report = SystemdRunner(manager, RecordingReadiness()).restore_transition_previous(
        previous,
        transition,
    )

    assert report.ok is True
    assert report.degraded["video"]["reason"] == "start_failed"
    assert host.target in manager.active_targets


def test_optional_video_stop_failure_remains_fatal() -> None:
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        10,
        5,
        "mode",
        required=False,
    )
    plan = _plan("nav", (video,))
    manager = RecordingManager(
        plan.processes,
        fail_stop={video.target: "systemd stop failed for video.service"},
    )

    with pytest.raises(ProcessFailed, match="failed to stop Product processes"):
        SystemdRunner(manager, RecordingReadiness()).stop(plan)


def test_optional_video_callback_failure_is_not_degraded() -> None:
    video = ProcessSpec(
        "video",
        "systemd",
        "video.service",
        10,
        5,
        "mode",
        required=False,
    )
    plan = _plan("nav", (video,))
    manager = RecordingManager(())

    with pytest.raises(ProcessFailed, match="callback failed") as failure:
        SystemdRunner(manager, RecordingReadiness()).apply(
            plan,
            on_process_started=lambda _process: (_ for _ in ()).throw(
                RuntimeError("callback failed")
            ),
        )

    assert failure.value.report.degraded == {}
    assert manager.active_targets == set()
