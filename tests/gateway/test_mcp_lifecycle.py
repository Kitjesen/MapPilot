"""Lifecycle contracts for the MCP HTTP server."""

from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from gateway.mcp_server import MCPServerModule


class _FakeServer:
    instances = []

    def __init__(self, config):
        self.config = config
        self.should_exit = False
        self.force_exit = False
        self.started = True
        self.run_saw_should_exit = None
        type(self).instances.append(self)

    def run(self):
        self.run_saw_should_exit = self.should_exit
        self.should_exit = True


def test_run_server_uses_managed_uvicorn_without_binding_socket(monkeypatch):
    _FakeServer.instances = []
    monkeypatch.setattr("uvicorn.Server", _FakeServer)
    module = MCPServerModule(host="127.0.0.1", port=0)

    result = module._run_server(threading.Event())

    assert result is True
    assert module._server is None
    server = _FakeServer.instances[0]
    assert server.config.app is not None
    assert server.config.host == "127.0.0.1"
    assert server.config.port == 0


def test_run_server_honors_stop_requested_before_assignment(monkeypatch):
    _FakeServer.instances = []
    monkeypatch.setattr("uvicorn.Server", _FakeServer)
    stop_event = threading.Event()
    stop_event.set()
    module = MCPServerModule(host="127.0.0.1", port=0)

    assert module._run_server(stop_event) is True

    assert _FakeServer.instances[0].run_saw_should_exit is True


def test_start_is_idempotent_and_stop_joins_server_thread(monkeypatch):
    module = MCPServerModule(host="127.0.0.1", port=0)
    entered = threading.Event()

    def fake_run(stop_event):
        entered.set()
        stop_event.wait(timeout=2.0)
        return True

    monkeypatch.setattr(module, "_run_server", fake_run)
    module.start()
    assert entered.wait(timeout=1.0)
    thread = module._server_thread

    module.start()

    assert module._server_thread is thread
    module.stop()
    assert not thread.is_alive()
    assert module._server_thread is None


def test_stop_signals_server_and_joins_thread():
    module = MCPServerModule(host="127.0.0.1", port=0)
    server = SimpleNamespace(should_exit=False)
    joined = []

    class FakeThread:
        def __init__(self):
            self.alive = True

        def is_alive(self):
            return self.alive

        def join(self, timeout):
            joined.append(timeout)
            self.alive = False

    thread = FakeThread()
    module._server = server
    module._server_thread = thread

    module.stop()

    assert module._stop_event.is_set()
    assert server.should_exit is True
    assert joined == [2.0]
    assert module._server_thread is None


def test_stop_retains_stuck_thread_reference():
    module = MCPServerModule(host="127.0.0.1", port=0)
    server = SimpleNamespace(should_exit=False)

    class StuckThread:
        def is_alive(self):
            return True

        def join(self, timeout):
            return None

    thread = StuckThread()
    module._server = server
    module._server_thread = thread

    module.stop()

    assert server.should_exit is True
    assert module._server_thread is thread


def test_start_is_rejected_while_stop_is_joining():
    module = MCPServerModule(host="127.0.0.1", port=0)
    join_entered = threading.Event()
    release_join = threading.Event()

    class JoiningThread:
        def __init__(self):
            self.alive = True

        def is_alive(self):
            return self.alive

        def join(self, timeout):
            join_entered.set()
            release_join.wait(timeout=timeout)
            self.alive = False

    original_thread = JoiningThread()
    module._server = SimpleNamespace(should_exit=False)
    module._server_thread = original_thread
    stopping_thread = threading.Thread(target=module.stop)
    stopping_thread.start()
    assert join_entered.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="stopping"):
        module.start()

    release_join.set()
    stopping_thread.join(timeout=1.0)
    assert not stopping_thread.is_alive()
    assert module._closed is True
    assert module._server_thread is None


def test_run_server_records_system_exit(monkeypatch):
    class ExitingServer(_FakeServer):
        def run(self):
            raise SystemExit("bind failed")

    monkeypatch.setattr("uvicorn.Server", ExitingServer)
    module = MCPServerModule(host="127.0.0.1", port=0)

    assert module._run_server(threading.Event()) is False

    health = module.health()["mcp"]
    assert health["thread_alive"] is False
    assert health["last_error"] == "SystemExit: bind failed"
    assert module._server is None


def test_health_exposes_server_lifecycle():
    module = MCPServerModule(host="127.0.0.1", port=8090)
    module._server_thread = SimpleNamespace(is_alive=lambda: True)
    module._server = SimpleNamespace(started=True)
    module._server_error = "bind failed"
    module._stop_event.set()

    health = module.health()["mcp"]

    assert health["thread_alive"] is True
    assert health["server_started"] is True
    assert health["stop_requested"] is True
    assert health["last_error"] == "bind failed"
