"""Real Host process tests."""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

import pytest

import lingtu.real.host as host_process


class _FakeGateway:
    def __init__(self, events: list[str], *, server_result: bool = True) -> None:
        self._events = events
        self._server_result = server_result
        self._defer_server = False

    def _run_server(self, stop_event: threading.Event) -> bool:
        assert self._defer_server is True
        assert isinstance(stop_event, threading.Event)
        self._events.append("server")
        return self._server_result


class _FakeSystem:
    def __init__(self, events: list[str], *, server_result: bool = True) -> None:
        self._events = events
        self.gateway = _FakeGateway(events, server_result=server_result)

    def get_module(self, name: str) -> Any:
        if name == "GatewayModule":
            return self.gateway
        raise KeyError(name)

    def start(self) -> None:
        assert self.gateway._defer_server is True
        self._events.append("start")

    def stop(self) -> None:
        self._events.append("stop")


class _FakePlan:
    def __init__(
        self,
        events: list[str],
        *,
        env: str = "real",
        process_control: str = "systemd",
        has_host: bool = True,
        server_result: bool = True,
    ) -> None:
        self.env = env
        self.process_control = process_control
        self._has_host = has_host
        self._events = events
        self._server_result = server_result

    def has_process(self, name: str) -> bool:
        return name == "host" and self._has_host

    def build(self) -> _FakeSystem:
        self._events.append("build")
        return _FakeSystem(self._events, server_result=self._server_result)


def _patch_plan(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    plan: _FakePlan,
) -> Path:
    path = tmp_path / "run-plan.json"
    path.write_text("{}", encoding="utf-8")
    monkeypatch.setenv("LINGTU_RUN_PLAN", str(path))

    def load(observed: str | Path) -> _FakePlan:
        assert Path(observed) == path
        return plan

    monkeypatch.setattr(host_process.RunPlan, "load", load)
    return path


def test_load_plan_requires_real_systemd_host(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    for override, match in (
        ({"env": "sim"}, "env=real"),
        ({"process_control": "subprocess"}, "process_control=systemd"),
        ({"has_host": False}, "host process role"),
    ):
        plan = _FakePlan([], **override)
        path = _patch_plan(monkeypatch, tmp_path, plan)
        with pytest.raises(RuntimeError, match=match):
            host_process._load_plan({"LINGTU_RUN_PLAN": str(path)})

    with pytest.raises(RuntimeError, match="LINGTU_RUN_PLAN"):
        host_process._load_plan({})


def test_run_builds_starts_runs_gateway_and_stops(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    _patch_plan(monkeypatch, tmp_path, _FakePlan(events))

    assert host_process.run(threading.Event()) == 0
    assert events == ["build", "start", "server", "stop"]


def test_run_stops_system_when_gateway_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    _patch_plan(monkeypatch, tmp_path, _FakePlan(events, server_result=False))

    assert host_process.run(threading.Event()) == 1
    assert events[-1] == "stop"


def test_run_stops_system_when_start_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events: list[str] = []
    class FailingSystem(_FakeSystem):
        def start(self) -> None:
            events.append("start")
            raise RuntimeError("start failed")

    class FailingPlan(_FakePlan):
        def build(self) -> _FakeSystem:
            events.append("build")
            return FailingSystem(events)

    plan = FailingPlan(events)
    _patch_plan(monkeypatch, tmp_path, plan)

    with pytest.raises(RuntimeError, match="start failed"):
        host_process.run(threading.Event())
    assert events == ["build", "start", "stop"]


def test_main_rejects_product_or_profile_arguments() -> None:
    with pytest.raises(SystemExit, match="does not accept CLI arguments"):
        host_process.main(["nav"])
