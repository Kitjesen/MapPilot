# ruff: noqa: S101
"""Simulation local RPC tests."""

from __future__ import annotations

import json
import multiprocessing
import os
import threading
import time
from contextlib import suppress
from dataclasses import FrozenInstanceError
from multiprocessing import connection as mp_connection
from pathlib import Path

import pytest

import lingtu.sim.rpc as supervisor_module
from lingtu.sim.rpc import (
    MAX_MESSAGE_BYTES,
    REQUEST_SCHEMA,
    LocalRpcServer,
    SupervisorDiscovery,
    SupervisorProtocolError,
    SupervisorRequest,
    SupervisorResponse,
    load_connection,
    round_trip,
)


def _request(
    tmp_path: Path,
    **overrides: object,
) -> SupervisorRequest:
    product_session_id = overrides.get("product_session_id", "1" * 32)
    values: dict[str, object] = {
        "schema_version": REQUEST_SCHEMA,
        "action": "apply",
        "run_plan_path": str(
            (tmp_path / f"plan-{product_session_id}.json").resolve()
        ),
        "product_session_id": product_session_id,
    }
    values.update(overrides)
    return SupervisorRequest(**values)  # type: ignore[arg-type]


def _json_bytes(value: object) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _fix_authkey(
    monkeypatch: pytest.MonkeyPatch,
    authkey: bytes,
) -> None:
    token_bytes = supervisor_module.secrets.token_bytes
    monkeypatch.setattr(
        supervisor_module.secrets,
        "token_bytes",
        lambda size: authkey if size == 32 else token_bytes(size),
    )


def _client_address(discovery: SupervisorDiscovery) -> str | tuple[str, int]:
    if discovery.family != "AF_INET":
        return discovery.address
    host, port = discovery.address.rsplit(":", 1)
    return host, int(port)


def test_protocol_value_objects_are_frozen(tmp_path: Path) -> None:
    request = _request(tmp_path)

    with pytest.raises(FrozenInstanceError):
        request.action = "stop"  # type: ignore[misc]


def test_server_publishes_discovery_after_binding(
    tmp_path: Path,
) -> None:
    endpoint_path = tmp_path / "supervisor" / "endpoint.json"
    auth_path = tmp_path / "supervisor" / "auth.key"

    with LocalRpcServer(tmp_path):
        discovery, authkey = load_connection(tmp_path)

        assert isinstance(discovery, SupervisorDiscovery)
        assert endpoint_path.is_file()
        assert authkey == auth_path.read_bytes()
        assert "auth" not in endpoint_path.read_text(encoding="utf-8").lower()
        if os.name == "nt":
            assert discovery.family == "AF_INET"
            host, port = _client_address(discovery)
            assert host == "127.0.0.1"
            assert port > 0
        else:
            assert discovery.family == "AF_UNIX"
            assert discovery.supervisor_nonce in discovery.address
            assert discovery.address == str(
                (
                    tmp_path
                    / "supervisor"
                    / f"control-{discovery.supervisor_nonce}.sock"
                ).resolve()
            )

    assert not endpoint_path.exists()
    assert not auth_path.exists()
    if os.name != "nt":
        assert not Path(discovery.address).exists()


def test_second_server_never_overwrites_live_discovery(tmp_path: Path) -> None:
    with LocalRpcServer(tmp_path):
        endpoint_path = tmp_path / "supervisor" / "endpoint.json"
        auth_path = tmp_path / "supervisor" / "auth.key"
        endpoint_before = endpoint_path.read_bytes()
        auth_before = auth_path.read_bytes()

        with pytest.raises(SupervisorProtocolError, match="already exists"):
            with LocalRpcServer(tmp_path):
                pass

        assert endpoint_path.read_bytes() == endpoint_before
        assert auth_path.read_bytes() == auth_before


@pytest.mark.parametrize(
    "raw",
    (
        b"[]",
        b"\xff",
        b"{" + (b" " * MAX_MESSAGE_BYTES),
    ),
    ids=("array", "utf8", "oversize"),
)
def test_request_codec_rejects_invalid_json_and_oversize(
    raw: bytes,
) -> None:
    with pytest.raises(SupervisorProtocolError):
        SupervisorRequest.from_bytes(raw)


def test_request_codec_accepts_extra_fields_and_rejects_missing_required(
    tmp_path: Path,
) -> None:
    payload = _request(tmp_path).as_dict()
    payload["extra"] = True
    assert SupervisorRequest.from_bytes(_json_bytes(payload)) == _request(tmp_path)

    payload.pop("action")
    with pytest.raises(SupervisorProtocolError, match="missing action"):
        SupervisorRequest.from_bytes(_json_bytes(payload))


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("action", "restart"),
        ("product_session_id", "unsafe/session"),
    ),
)
def test_request_contract_rejects_invalid_identity_fields(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    with pytest.raises(SupervisorProtocolError):
        _request(tmp_path, **{field: value})


def test_response_codec_accepts_extra_fields_and_rejects_missing_required() -> None:
    response = SupervisorResponse.ok({"status": "accepted"})
    payload = response.as_dict()
    payload["extra"] = True
    assert SupervisorResponse.from_bytes(_json_bytes(payload)) == response

    payload.pop("success")
    with pytest.raises(SupervisorProtocolError, match="missing success"):
        SupervisorResponse.from_bytes(_json_bytes(payload))


def test_response_success_and_failure_shapes_are_strict(tmp_path: Path) -> None:
    del tmp_path
    success = SupervisorResponse.ok({"status": "ready"})
    failure = SupervisorResponse.failed(
        code="apply_failed",
        message="supervisor operation failed",
    )

    assert success.result == {"status": "ready"}
    assert success.error is None
    assert failure.result is None
    assert failure.error == {
        "code": "apply_failed",
        "message": "supervisor operation failed",
    }

    invalid = success.as_dict()
    invalid["success"] = True
    invalid["error"] = {"code": "bad", "message": "bad"}
    with pytest.raises(SupervisorProtocolError):
        SupervisorResponse.from_bytes(_json_bytes(invalid))


def test_authenticated_round_trip_uses_bytes_only_and_reads_no_plan(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def forbidden_pickle(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("pickle transport is forbidden")

    monkeypatch.setattr(mp_connection._ConnectionBase, "send", forbidden_pickle)
    monkeypatch.setattr(mp_connection._ConnectionBase, "recv", forbidden_pickle)
    calls: list[SupervisorRequest] = []
    errors: list[BaseException] = []

    with LocalRpcServer(tmp_path) as server:
        request = _request(tmp_path)

        def handler(received: SupervisorRequest) -> SupervisorResponse:
            calls.append(received)
            assert not Path(received.run_plan_path).exists()
            return SupervisorResponse.ok({"accepted": True})

        def serve() -> None:
            try:
                server.serve_once(handler)
            except BaseException as exc:  # pragma: no cover - asserted below
                errors.append(exc)

        thread = threading.Thread(target=serve, name="sim-supervisor-test")
        thread.start()
        response = round_trip(tmp_path, request, timeout_s=5)
        thread.join(timeout=5)

        assert response.result == {"accepted": True}
        assert calls == [request]
        assert errors == []
        assert not thread.is_alive()


def test_server_waits_for_delayed_client(tmp_path: Path) -> None:
    errors: list[BaseException] = []
    serving = threading.Event()

    with LocalRpcServer(tmp_path) as server:
        request = _request(tmp_path)

        def serve() -> None:
            serving.set()
            try:
                server.serve_once(
                    lambda _request: SupervisorResponse.ok({"accepted": True})
                )
            except BaseException as exc:  # pragma: no cover - asserted below
                errors.append(exc)

        thread = threading.Thread(target=serve, name="sim-supervisor-delay-test")
        thread.start()
        assert serving.wait(timeout=1)
        time.sleep(0.1)
        assert thread.is_alive(), errors

        response = round_trip(tmp_path, request, timeout_s=5)
        thread.join(timeout=5)

        assert response.result == {"accepted": True}
        assert errors == []
        assert not thread.is_alive()


@pytest.mark.parametrize("location", ("wrong_name", "nested", "outside"))
def test_round_trip_rejects_run_plan_path_not_exactly_bound_to_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    location: str,
) -> None:
    with LocalRpcServer(tmp_path):
        product_session_id = "1" * 32
        paths = {
            "wrong_name": tmp_path / f"plan-{'2' * 32}.json",
            "nested": tmp_path / "nested" / f"plan-{product_session_id}.json",
            "outside": tmp_path.parent / f"plan-{product_session_id}.json",
        }
        request = _request(
            tmp_path,
            run_plan_path=str(paths[location].resolve()),
        )

        def forbidden_client(*_args: object, **_kwargs: object) -> None:
            raise AssertionError("path binding must fail before Client")

        monkeypatch.setattr(supervisor_module.mp_connection, "Client", forbidden_client)
        with pytest.raises(SupervisorProtocolError, match="run_plan_path"):
            round_trip(tmp_path, request)


def test_server_rejects_wrong_run_plan_binding_before_handler(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    authkey = b"k" * 32
    _fix_authkey(monkeypatch, authkey)
    calls = 0
    errors: list[BaseException] = []
    with LocalRpcServer(tmp_path) as server:
        discovery, published_authkey = load_connection(tmp_path)
        assert published_authkey == authkey
        request = _request(
            tmp_path,
            run_plan_path=str((tmp_path / "wrong-plan.json").resolve()),
        )

        def handler(received: SupervisorRequest) -> SupervisorResponse:
            nonlocal calls
            calls += 1
            return SupervisorResponse.ok({})

        def serve() -> None:
            try:
                server.serve_once(handler)
            except BaseException as exc:
                errors.append(exc)

        thread = threading.Thread(target=serve)
        thread.start()
        client = mp_connection.Client(
            _client_address(discovery),
            family=discovery.family,
            authkey=authkey,
        )
        try:
            client.send_bytes(request.to_bytes())
        finally:
            client.close()
        thread.join(timeout=5)

        assert calls == 0
        assert len(errors) == 1
        assert isinstance(errors[0], SupervisorProtocolError)
        assert "run_plan_path" in str(errors[0])
        assert not thread.is_alive()


def test_round_trip_total_deadline_covers_connect_and_leaves_no_helper(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    helper_prefix = "lingtu-sim-supervisor-client-"
    baseline = {
        child.pid
        for child in multiprocessing.active_children()
        if child.name.startswith(helper_prefix)
    }

    def slow_client(*_args: object, **_kwargs: object) -> None:
        time.sleep(0.5)
        raise OSError("simulated blocked connect")

    monkeypatch.setattr(supervisor_module.mp_connection, "Client", slow_client)
    with LocalRpcServer(tmp_path):
        request = _request(tmp_path)
        started = time.monotonic()
        with pytest.raises(SupervisorProtocolError, match="timed out"):
            round_trip(tmp_path, request, timeout_s=0.15)
        elapsed = time.monotonic() - started

    remaining = {
        child.pid
        for child in multiprocessing.active_children()
        if child.name.startswith(helper_prefix)
    }
    assert elapsed < 1.5
    assert remaining == baseline


def test_handler_exception_returns_message_without_traceback(
    tmp_path: Path,
) -> None:
    with LocalRpcServer(tmp_path) as server:
        request = _request(tmp_path)

        def handler(_request: SupervisorRequest) -> SupervisorResponse:
            raise RuntimeError("secret exception detail")

        thread = threading.Thread(target=server.serve_once, args=(handler,))
        thread.start()
        response = round_trip(tmp_path, request, timeout_s=15)
        thread.join(timeout=15)

        assert response.success is False
        assert response.error == {
            "code": "handler_failed",
            "message": "secret exception detail",
        }
        assert "Traceback" not in response.to_bytes().decode("utf-8")
        assert not thread.is_alive()


def test_server_receive_limit_rejects_oversize_before_handler(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = 0
    errors: list[BaseException] = []
    authkey = b"k" * 32
    _fix_authkey(monkeypatch, authkey)
    with LocalRpcServer(tmp_path) as server:
        discovery, published_authkey = load_connection(tmp_path)
        assert published_authkey == authkey

        def handler(request: SupervisorRequest) -> SupervisorResponse:
            nonlocal calls
            calls += 1
            return SupervisorResponse.ok({})

        def serve() -> None:
            try:
                server.serve_once(handler)
            except BaseException as exc:
                errors.append(exc)

        thread = threading.Thread(target=serve)
        thread.start()
        client = mp_connection.Client(
            _client_address(discovery),
            family=discovery.family,
            authkey=authkey,
        )
        try:
            with suppress(BrokenPipeError):
                client.send_bytes(b"x" * (MAX_MESSAGE_BYTES + 1))
        finally:
            client.close()
        thread.join(timeout=5)

        assert calls == 0
        assert len(errors) == 1
        assert isinstance(errors[0], SupervisorProtocolError)
        assert not thread.is_alive()


def test_supervisor_module_does_not_import_runtime_graph_or_run_plan() -> None:
    source = Path(supervisor_module.__file__).read_text(encoding="utf-8")

    assert "runtime.graph" not in source
    assert "lingtu.run_plan" not in source
