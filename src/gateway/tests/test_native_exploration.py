"""Native exploration command and telemetry boundary tests."""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace

from fastapi import FastAPI

from gateway.routes.operations import register_operation_routes
from gateway.services import exploration
from gateway.services.native_exploration import read_fresh_status, read_status, status_is_fresh


class _Commands:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []

    def start_exploration(self, *, reason: str) -> bool:
        self.calls.append(("start", reason))
        return True

    def stop_exploration(self, *, reason: str) -> bool:
        self.calls.append(("stop", reason))
        return True


def _route_endpoint(app: FastAPI, path: str):
    for route in app.routes:
        if getattr(route, "path", None) == path:
            return route.endpoint
    raise AssertionError(f"route not found: {path}")


def test_native_status_requires_schema_and_fresh_timestamp(tmp_path, monkeypatch) -> None:
    status_file = tmp_path / "explore_status.json"
    monkeypatch.setenv("LINGTU_EXPLORE_STATUS_FILE", str(status_file))
    monkeypatch.setenv("LINGTU_EXPLORE_STATUS_MAX_AGE_S", "6")

    status_file.write_text(
        json.dumps({"schema_version": "wrong", "stamp_s": 100.0}),
        encoding="utf-8",
    )
    assert read_status() is None

    payload = {
        "schema_version": "lingtu.explore.status.v2",
        "stamp_s": 100.0,
        "active": False,
    }
    status_file.write_text(json.dumps(payload), encoding="utf-8")
    assert read_status() == payload
    assert read_fresh_status(now_s=105.9) == payload
    assert read_fresh_status(now_s=106.1) is None
    assert status_is_fresh({**payload, "stamp_s": 101.0}, now_s=100.0) is False


def test_native_commands_require_status_for_start_but_not_for_stop(monkeypatch) -> None:
    commands = _Commands()
    gateway = SimpleNamespace(_frontier_explorer=None, _tare_explorer=None)
    monkeypatch.setattr(exploration, "_native_commands", lambda _gw: commands)
    monkeypatch.setattr(exploration, "_native_status", lambda: {"active": False})

    assert exploration.explorer_available(gateway) is True
    assert exploration.explorer_stop_available(gateway) is True
    assert exploration.begin_exploration(gateway) == {
        "accepted": True,
        "backend": "tare",
        "runtime": "native_dds",
    }

    monkeypatch.setattr(exploration, "_native_status", lambda: None)
    assert exploration.explorer_available(gateway) is False
    assert exploration.explorer_stop_available(gateway) is True
    assert exploration.end_exploration(gateway) == {
        "accepted": True,
        "backend": "tare",
        "runtime": "native_dds",
    }
    assert commands.calls == [("start", "gateway_start"), ("stop", "gateway_stop")]


def test_stop_route_remains_available_when_native_status_is_stale() -> None:
    events: list[dict] = []
    gateway = SimpleNamespace(
        _go2rtc_upstream="",
        _explorer_available=lambda: False,
        _explorer_stop_available=lambda: True,
        _end_exploration=lambda: {"accepted": True},
        _exploring=True,
        push_event=events.append,
    )
    app = FastAPI()
    register_operation_routes(app, gateway)

    response = asyncio.run(_route_endpoint(app, "/api/v1/explore/stop")())

    assert response == {"status": {"accepted": True}}
    assert gateway._exploring is False
    assert events == [{"type": "exploring", "active": False}]


def test_stop_route_reports_command_timeout_without_faking_success() -> None:
    events: list[dict] = []
    gateway = SimpleNamespace(
        _go2rtc_upstream="",
        _explorer_available=lambda: False,
        _explorer_stop_available=lambda: True,
        _end_exploration=lambda: (_ for _ in ()).throw(RuntimeError("timed out waiting for exploration command ACK")),
        _exploring=True,
        push_event=events.append,
    )
    app = FastAPI()
    register_operation_routes(app, gateway)

    response = asyncio.run(_route_endpoint(app, "/api/v1/explore/stop")())
    payload = json.loads(response.body)

    assert response.status_code == 503
    assert payload["error"] == "exploration_command_unavailable"
    assert payload["detail"] == {"source": "exploration_command_boundary"}
    assert gateway._exploring is True
    assert events == []
