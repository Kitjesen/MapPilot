# ruff: noqa: S101
"""Simulation Host process tests."""

from __future__ import annotations

import ast
import json
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pytest

import lingtu.sim.host as host_process
from lingtu.sim.readiness import HOST_READY_FILE, HOST_READY_SCHEMA

_PRODUCT_SESSION_ID = "a" * 32
_OTHER_PRODUCT_SESSION_ID = "b" * 32


@dataclass(frozen=True)
class _FakeProcess:
    name: str = "host_runtime"


class _FakeSystem:
    def __init__(self, events: list[str], stop_event: threading.Event) -> None:
        self._events = events
        self._stop_event = stop_event

    def start(self) -> None:
        self._events.append("start")
        assert not (self._session_root() / HOST_READY_FILE).exists()
        self._stop_event.set()

    def stop(self) -> None:
        self._events.append("stop")

    def _session_root(self) -> Path:
        return Path(os.environ["LINGTU_RUN_PLAN"]).parent


class _FakePlan:
    def __init__(
        self,
        events: list[str],
        stop_event: threading.Event,
        *,
        product: str = "nav",
        env: str = "sim",
        process_control: str = "subprocess",
        has_host: bool = True,
    ) -> None:
        self.product = product
        self.env = env
        self.process_control = process_control
        self._has_host = has_host
        self._events = events
        self._stop_event = stop_event

    def has_process(self, name: str) -> bool:
        return name == "host" and self._has_host

    def process(self, name: str) -> _FakeProcess:
        if not self.has_process(name):
            raise KeyError(name)
        return _FakeProcess()

    def build(self) -> _FakeSystem:
        self._events.append("build")
        return _FakeSystem(self._events, self._stop_event)


def _write_identity_env(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    product_session_id: str = _PRODUCT_SESSION_ID,
    product: str = "nav",
) -> Path:
    session_root = tmp_path.resolve()
    plan_path = session_root / f"plan-{product_session_id}.json"
    plan_path.write_text("{}", encoding="utf-8")
    values = {
        "LINGTU_RUN_PLAN": str(plan_path),
        "LINGTU_PRODUCT_SESSION_ID": product_session_id,
    }
    for key, value in values.items():
        monkeypatch.setenv(key, value)
    return plan_path


def _patch_load(
    monkeypatch: pytest.MonkeyPatch,
    plan: _FakePlan,
    expected_path: Path,
) -> None:
    assert expected_path.is_file()

    def load(path: Path) -> _FakePlan:
        assert Path(path) == expected_path
        return plan

    monkeypatch.setattr(host_process.RunPlan, "load", load)


def test_host_process_loads_identity_builds_starts_then_publishes_ready(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _write_identity_env(tmp_path, monkeypatch)
    events: list[str] = []
    stop_event = threading.Event()
    plan = _FakePlan(events, stop_event)
    _patch_load(monkeypatch, plan, plan_path)

    assert host_process.run(stop_event) == 0

    assert events == ["build", "start", "stop"]
    assert not (tmp_path / HOST_READY_FILE).exists()


def test_ready_file_is_published_after_start_and_removed_on_stop(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _write_identity_env(tmp_path, monkeypatch)
    events: list[str] = []
    stop_event = threading.Event()
    ready_seen: dict[str, Any] = {}

    class ObservingSystem(_FakeSystem):
        def start(self) -> None:
            events.append("start")

        def stop(self) -> None:
            ready_path = tmp_path / HOST_READY_FILE
            ready_seen.update(json.loads(ready_path.read_text(encoding="utf-8")))
            stop_event.set()
            events.append("stop")

    class ObservingPlan(_FakePlan):
        def build(self) -> ObservingSystem:
            events.append("build")
            return ObservingSystem(events, stop_event)

    plan = ObservingPlan(events, stop_event)
    _patch_load(monkeypatch, plan, plan_path)
    stop_event.set()

    assert host_process.run(stop_event) == 0

    assert events == ["build", "start", "stop"]
    assert ready_seen == {
        "schema": HOST_READY_SCHEMA,
        "product_session_id": _PRODUCT_SESSION_ID,
        "product": "nav",
        "env": "sim",
        "process": "host_runtime",
        "protocol": "host-process-v1",
        "ready": True,
        "role": "host",
    }
    assert not (tmp_path / HOST_READY_FILE).exists()


def test_signal_handler_only_sets_stop_event(monkeypatch: pytest.MonkeyPatch) -> None:
    installed: list[Any] = []
    event = threading.Event()
    monkeypatch.setattr(
        host_process.signal,
        "signal",
        lambda signum, handler: installed.append((signum, handler)),
    )

    host_process._install_signal_handlers(event)

    assert len(installed) == 2
    installed[0][1](installed[0][0], object())
    assert event.is_set()


@pytest.mark.parametrize(
    ("overrides", "match"),
    [
        ({"env": "real"}, "env"),
        ({"process_control": "systemd"}, "process_control"),
        ({"has_host": False}, "host process role"),
    ],
)
def test_rejects_tampered_or_missing_host_run_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    overrides: dict[str, Any],
    match: str,
) -> None:
    plan_path = _write_identity_env(tmp_path, monkeypatch)
    events: list[str] = []
    stop_event = threading.Event()
    plan = _FakePlan(events, stop_event, **overrides)
    _patch_load(monkeypatch, plan, plan_path)

    with pytest.raises(RuntimeError, match=match):
        host_process.run(stop_event)

    assert events == []
    assert not (tmp_path / HOST_READY_FILE).exists()


def test_fails_closed_without_ready_when_build_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan_path = _write_identity_env(tmp_path, monkeypatch)
    events: list[str] = []
    stop_event = threading.Event()

    class FailingBuildPlan(_FakePlan):
        def build(self) -> _FakeSystem:
            events.append("build")
            raise RuntimeError("boom")

    _patch_load(monkeypatch, FailingBuildPlan(events, stop_event), plan_path)

    with pytest.raises(RuntimeError, match="boom"):
        host_process.run(stop_event)

    assert events == ["build"]
    assert not (tmp_path / HOST_READY_FILE).exists()


def test_main_rejects_cli_arguments() -> None:
    with pytest.raises(SystemExit, match="does not accept CLI arguments"):
        host_process.main(["nav"])


def test_source_does_not_import_or_call_runtime_graph() -> None:
    source = Path(host_process.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported_modules = {
        alias.name
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imported_modules.update(
        node.module or ""
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    )

    assert not any(name.startswith("runtime.graph") for name in imported_modules)
    assert "load_runtime_graph" not in source
    assert "compile_run_plan" not in source
