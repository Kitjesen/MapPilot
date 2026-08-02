"""Fail-closed startup barrier tests for critical Blueprint modules."""

from __future__ import annotations

import pytest

from runtime.blueprint import Blueprint, SystemStartupError
from runtime.module import Module


class _ReadyModule(Module):
    pass


def test_build_rejects_missing_critical_alias() -> None:
    blueprint = Blueprint().add(_ReadyModule, alias="present").require_modules("missing")

    with pytest.raises(ValueError, match="missing"):
        blueprint.build()


def test_namespace_and_merge_preserve_critical_aliases() -> None:
    first = Blueprint().add(_ReadyModule, alias="core").require_modules("core").namespace("robot_a")
    second = Blueprint().add(_ReadyModule, alias="core").require_modules("core").namespace("robot_b")

    merged = Blueprint().merge(first).merge(second)

    assert merged.required_module_names == ("robot_a/core", "robot_b/core")
    system = merged.build()
    assert system.critical_modules == ("robot_a/core", "robot_b/core")


def test_critical_modules_reject_worker_mode_before_worker_build(monkeypatch: pytest.MonkeyPatch) -> None:
    from runtime.blueprint import SystemStartupError

    blueprint = Blueprint().add(_ReadyModule, alias="core").require_modules("core")
    worker_build_called = False

    def _unexpected_worker_build(_n_workers: int):
        nonlocal worker_build_called
        worker_build_called = True
        pytest.fail("worker mode must not start for critical modules")

    monkeypatch.setattr(blueprint, "_build_worker_mode", _unexpected_worker_build)

    with pytest.raises(SystemStartupError, match="worker"):
        blueprint.build(n_workers=1)
    assert worker_build_called is False

class _TrackingTransport:
    def __init__(self) -> None:
        self.closed = False

    def publish(self, topic: str, msg: object) -> None:
        del topic, msg

    def subscribe(self, topic: str, callback) -> None:
        del topic, callback

    def close(self) -> None:
        self.closed = True


class _LifecycleModule(Module):
    def __init__(
        self,
        name: str,
        events: list[str],
        *,
        preflight_failure: str | None = None,
        setup_failure: bool = False,
        start_failure: bool = False,
        readiness_result: str | None = None,
        readiness_error: Exception | None = None,
    ) -> None:
        super().__init__()
        self.name = name
        self.events = events
        self.preflight_failure = preflight_failure
        self.setup_failure = setup_failure
        self.start_failure = start_failure
        self.readiness_result = readiness_result
        self.readiness_error = readiness_error

    def preflight(self) -> str | None:
        self.events.append(f"preflight:{self.name}")
        return self.preflight_failure

    def setup(self) -> None:
        self.events.append(f"setup:{self.name}")
        if self.setup_failure:
            raise RuntimeError(f"{self.name} setup failed")

    def start(self) -> None:
        self.events.append(f"start:{self.name}")
        super().start()
        if self.start_failure:
            raise RuntimeError(f"{self.name} start failed")

    def startup_readiness(self) -> str | None:
        if self.readiness_error is not None:
            raise self.readiness_error
        if self.readiness_result is not None:
            return self.readiness_result
        return super().startup_readiness()

    def stop(self) -> None:
        if not self._closed:
            self.events.append(f"stop:{self.name}")
        super().stop()


class _HandleAwareModule(Module):
    def __init__(self, events: list[str]) -> None:
        super().__init__()
        self.events = events
        self.system_handle = None

    def set_system_handle(self, handle) -> None:
        self.system_handle = handle
        self.events.append("injected")

    def start(self) -> None:
        assert self.system_handle is not None
        self.events.append("started")
        super().start()


def test_module_default_startup_readiness_tracks_running_state() -> None:
    module = _ReadyModule()

    assert module.startup_readiness() == "not_running"
    module.start()
    assert module.startup_readiness() is None


def test_critical_preflight_failure_fails_closed() -> None:
    events: list[str] = []
    transport = _TrackingTransport()
    critical = _LifecycleModule("critical", events, preflight_failure="sensor unavailable")
    later = _LifecycleModule("later", events)
    system = (
        Blueprint()
        .add(critical, alias="critical")
        .add(later, alias="later")
        .require_modules("critical")
        .build(transport=transport)
    )

    assert system.startup_state == "built"
    with pytest.raises(SystemStartupError, match=r"critical.*preflight"):
        system.start()

    assert system.started is False
    assert system.startup_state == "failed"
    assert system.failed_modules == {"critical": "preflight: sensor unavailable"}
    assert system.critical_failures == {"critical": "preflight: sensor unavailable"}
    assert "start:later" not in events
    assert transport.closed is True


def test_critical_setup_failure_rolls_back_touched_modules_in_reverse_order() -> None:
    events: list[str] = []
    first = _LifecycleModule("first", events)
    critical = _LifecycleModule("critical", events, setup_failure=True)
    later = _LifecycleModule("later", events)
    system = (
        Blueprint()
        .add(first, alias="first")
        .add(critical, alias="critical")
        .add(later, alias="later")
        .require_modules("critical")
        .build()
    )

    with pytest.raises(SystemStartupError, match=r"critical.*setup"):
        system.start()

    assert [event for event in events if event.startswith("stop:")] == [
        "stop:critical",
        "stop:first",
    ]
    assert "setup:later" not in events
    assert "start:later" not in events


def test_critical_start_failure_stops_all_setup_modules_in_reverse_topology() -> None:
    events: list[str] = []
    first = _LifecycleModule("first", events)
    critical = _LifecycleModule("critical", events, start_failure=True)
    later = _LifecycleModule("later", events)
    system = (
        Blueprint()
        .add(first, alias="first")
        .add(critical, alias="critical")
        .add(later, alias="later")
        .require_modules("critical")
        .build()
    )

    with pytest.raises(SystemStartupError, match=r"critical.*start"):
        system.start()

    assert "start:later" not in events
    assert [event for event in events if event.startswith("stop:")] == [
        "stop:later",
        "stop:critical",
        "stop:first",
    ]


def test_critical_readiness_timeout_rolls_back_and_closes_transport() -> None:
    events: list[str] = []
    transport = _TrackingTransport()
    critical = _LifecycleModule("critical", events, readiness_result="warming_up")
    system = (
        Blueprint()
        .add(critical, alias="critical")
        .require_modules("critical")
        .build(transport=transport)
    )

    with pytest.raises(SystemStartupError, match="readiness"):
        system.start(startup_timeout_s=0.01, readiness_poll_interval_s=0.001)

    assert system.failed_modules["critical"].startswith("startup_readiness: timeout")
    assert events[-1] == "stop:critical"
    assert transport.closed is True


def test_critical_readiness_exception_fails_closed() -> None:
    events: list[str] = []
    critical = _LifecycleModule(
        "critical",
        events,
        readiness_error=RuntimeError("probe crashed"),
    )
    system = (
        Blueprint()
        .add(critical, alias="critical")
        .require_modules("critical")
        .build()
    )

    with pytest.raises(SystemStartupError, match="readiness"):
        system.start()

    assert system.critical_failures == {
        "critical": "startup_readiness: probe crashed",
    }


def test_optional_failure_is_cleaned_and_system_becomes_ready() -> None:
    events: list[str] = []
    optional = _LifecycleModule("optional", events, start_failure=True)
    critical = _LifecycleModule("critical", events)
    system = (
        Blueprint()
        .add(optional, alias="optional")
        .add(critical, alias="critical")
        .require_modules("critical")
        .build()
    )

    system.start()

    assert system.started is True
    assert system.startup_state == "ready"
    assert system.failed_modules == {"optional": "start: optional start failed"}
    assert system.critical_failures == {}
    assert "stop:optional" in events
    assert critical.running is True
    system.stop()


def test_system_handle_is_injected_before_module_start() -> None:
    events: list[str] = []
    module = _HandleAwareModule(events)
    system = Blueprint().add(module, alias="aware").require_modules("aware").build()

    assert module.system_handle is system
    system.start()

    assert events == ["injected", "started"]
    system.stop()


def test_stopped_and_failed_handles_cannot_restart() -> None:
    ready_system = Blueprint().add(_ReadyModule, alias="core").require_modules("core").build()
    ready_system.start()
    ready_system.stop()

    assert ready_system.startup_state == "stopped"
    with pytest.raises(SystemStartupError, match="stopped"):
        ready_system.start()

    failing = _LifecycleModule("critical", [], preflight_failure="no device")
    failed_system = Blueprint().add(failing, alias="critical").require_modules("critical").build()
    with pytest.raises(SystemStartupError):
        failed_system.start()

    with pytest.raises(SystemStartupError, match="failed"):
        failed_system.start()
