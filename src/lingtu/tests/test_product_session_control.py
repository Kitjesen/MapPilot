from __future__ import annotations

from types import SimpleNamespace

import pytest

from lingtu.product_switch import FieldBackend


def _inactive_service(_command, **_kwargs):
    return SimpleNamespace(returncode=3, stdout="", stderr="")


def test_field_backend_reads_control_credential_from_transient_session_file(
    monkeypatch,
    tmp_path,
):
    (tmp_path / "session.env").write_text(
        'LINGTU_PRODUCT_SESSION_ID="session-token-1234"\n',
        encoding="utf-8",
    )
    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=_inactive_service,
    )
    calls = []

    def http(method, path, **kwargs):
        calls.append((method, path, kwargs))
        return {"ok": True, "success": True}

    monkeypatch.setattr(backend, "_http", http)

    backend.stop_motion_and_session("map")

    assert calls == [
        (
            "POST",
            "/api/v1/session/end",
            {
                "timeout_s": 10.0,
                "headers": {
                    "X-LingTu-Product-Session": "session-token-1234",
                },
            },
        )
    ]


def test_field_backend_refuses_unauthenticated_active_session_shutdown(
    monkeypatch,
    tmp_path,
):
    backend = FieldBackend(
        environment={"LINGTU_SESSION_ROOT": str(tmp_path)},
        runner=_inactive_service,
    )
    monkeypatch.setattr(
        backend,
        "_http",
        lambda *_args, **_kwargs: pytest.fail("Gateway must not be called"),
    )

    with pytest.raises(RuntimeError, match="credential is unavailable"):
        backend.stop_motion_and_session("map")
