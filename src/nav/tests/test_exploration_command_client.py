from __future__ import annotations

from pathlib import Path

import pytest

from nav.adapters.native.abi import NativeCommandClientError
from nav.adapters.native.exploration_commands import NativeExplorationCommandClient


class _Function:
    def __init__(self, implementation):
        self.implementation = implementation
        self.argtypes = None
        self.restype = None

    def __call__(self, *args):
        return self.implementation(*args)


class _Library:
    def __init__(self, *, capabilities: int = 0x07) -> None:
        self.calls: list[tuple[str, tuple[object, ...]]] = []
        self.lingtu_nav_client_abi_version = _Function(lambda: 1)
        self.lingtu_nav_client_capabilities = _Function(lambda: capabilities)
        self.lingtu_nav_client_create = _Function(lambda _domain: 1)
        self.lingtu_nav_client_destroy = _Function(lambda _handle: None)
        self.lingtu_nav_client_last_error = _Function(lambda _handle: b"native failure")
        for name in (
            "lingtu_nav_client_start_exploration",
            "lingtu_nav_client_pause_exploration",
            "lingtu_nav_client_resume_exploration",
            "lingtu_nav_client_stop_exploration",
        ):
            setattr(self, name, _Function(self._record(name)))

    def _record(self, name: str):
        def invoke(*args):
            self.calls.append((name, args))
            return 0

        return invoke


def _client(tmp_path: Path, library: _Library) -> NativeExplorationCommandClient:
    return NativeExplorationCommandClient(
        tmp_path / "liblingtu_nav_client.so",
        domain_id=7,
        timeout_ms=2500,
        library=library,
    )


def test_exploration_client_uses_typed_native_abi(tmp_path: Path) -> None:
    library = _Library()
    client = _client(tmp_path, library)

    client.start(session_id="session-a", reason="web_start", request_id="start-1")
    client.pause("web_pause", request_id="pause-1")
    client.resume("web_resume", request_id="resume-1")
    client.stop("web_stop", request_id="stop-1")

    assert [name for name, _args in library.calls] == [
        "lingtu_nav_client_start_exploration",
        "lingtu_nav_client_pause_exploration",
        "lingtu_nav_client_resume_exploration",
        "lingtu_nav_client_stop_exploration",
    ]
    start_args = library.calls[0][1]
    assert start_args[1:4] == (b"start-1", b"session-a", b"web_start")
    assert start_args[-1] == 2500
    assert library.calls[-1][1][1:3] == (b"stop-1", b"web_stop")
    client.close()


def test_exploration_client_requires_capability(tmp_path: Path) -> None:
    with pytest.raises(NativeCommandClientError, match="exploration commands capability"):
        _client(tmp_path, _Library(capabilities=0x03))
