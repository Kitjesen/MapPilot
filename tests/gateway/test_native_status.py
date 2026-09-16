"""Native snapshot reads do not promote raw telemetry into control evidence."""

from __future__ import annotations

import json

import pytest

from gateway.services import native_status
from gateway.services.native_status import read_navigation_status, read_traversability_status


@pytest.mark.parametrize(
    ("reader", "env_name", "session_name", "field_path"),
    [
        (
            read_navigation_status,
            "LINGTU_NAV_STATUS_FILE",
            "nav.status.json",
            "/dev/shm/lingtu/nav_endpoint_status.json",
        ),
        (
            read_traversability_status,
            "LINGTU_TRAVERSABILITY_STATUS_FILE",
            "traversability.status.json",
            "/dev/shm/lingtu/traversability_status.json",
        ),
    ],
)
@pytest.mark.parametrize("env", [None, "real", "sim"])
def test_snapshot_default_path_respects_env(reader, env_name, session_name, field_path, env, monkeypatch, tmp_path):
    if env is None:
        monkeypatch.delenv("LINGTU_ENV", raising=False)
    else:
        monkeypatch.setenv("LINGTU_ENV", env)
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path))
    monkeypatch.delenv(env_name, raising=False)
    monkeypatch.setattr(native_status, "read_json_snapshot", lambda path: {"path": path})

    expected = str(tmp_path / session_name) if env == "sim" else field_path
    assert reader() == {"path": expected}


@pytest.mark.parametrize(
    ("reader", "env_name"),
    [
        (read_navigation_status, "LINGTU_NAV_STATUS_FILE"),
        (read_traversability_status, "LINGTU_TRAVERSABILITY_STATUS_FILE"),
    ],
)
@pytest.mark.parametrize("content", [None, "{", "[]", "null"])
def test_snapshot_unavailable_is_not_an_empty_success(reader, env_name, content, monkeypatch, tmp_path):
    path = tmp_path / "status.json"
    monkeypatch.setenv(env_name, str(path))
    if content is not None:
        path.write_text(content, encoding="utf-8")

    assert reader() is None


@pytest.mark.parametrize(
    ("reader", "env_name"),
    [
        (read_navigation_status, "LINGTU_NAV_STATUS_FILE"),
        (read_traversability_status, "LINGTU_TRAVERSABILITY_STATUS_FILE"),
    ],
)
@pytest.mark.parametrize("env", ["real", "sim"])
def test_snapshot_retains_original_timestamp_and_evidence(reader, env_name, env, monkeypatch, tmp_path):
    path = tmp_path / "status.json"
    payload = {"stamp_s": 1.0, "ready": True, "output_ack": {"accepted": False}}
    path.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setenv("LINGTU_ENV", env)
    monkeypatch.setenv("LINGTU_SESSION_ROOT", str(tmp_path / "session"))
    monkeypatch.setenv(env_name, str(path))

    assert reader() == payload
