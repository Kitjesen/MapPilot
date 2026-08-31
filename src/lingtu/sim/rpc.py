"""Authenticated RPC protocol for the local simulation supervisor.

This module owns only discovery and request/response transport. It deliberately
does not load a RunPlan or manage child-process lifecycles.
"""

from __future__ import annotations

import json
import math
import multiprocessing
import os
import re
import secrets
import time
from collections.abc import Callable
from dataclasses import dataclass
from multiprocessing import AuthenticationError
from multiprocessing import connection as mp_connection
from multiprocessing.process import BaseProcess
from pathlib import Path
from typing import Any

from lingtu.switch_contracts import is_product_session_id

MAX_MESSAGE_BYTES = 64 * 1024
DISCOVERY_SCHEMA = "lingtu.sim_supervisor.discovery.v1"
REQUEST_SCHEMA = "lingtu.sim_supervisor.request.v1"
RESPONSE_SCHEMA = "lingtu.sim_supervisor.response.v1"

_DISCOVERY_MAX_BYTES = 4096
_AUTH_KEY_BYTES = 32
_WORKER_RESULT_MAX_BYTES = MAX_MESSAGE_BYTES + 128
_WORKER_PROCESS_PREFIX = "lingtu-sim-supervisor-client-"
_NONCE_PATTERN = re.compile(r"[0-9a-f]{32}\Z")
_ERROR_CODE_PATTERN = re.compile(r"[a-z][a-z0-9_]{0,63}\Z")
_ACTIONS = frozenset({"apply", "quiesce", "stop"})


class SupervisorProtocolError(RuntimeError):
    """Raised when local supervisor discovery or RPC data is invalid."""


def _load_json_object(raw: bytes, *, max_bytes: int, context: str) -> dict[str, Any]:
    if not isinstance(raw, bytes):
        raise SupervisorProtocolError(f"{context} must be encoded as bytes")
    if len(raw) > max_bytes:
        raise SupervisorProtocolError(f"{context} exceeds the byte limit")
    try:
        text = raw.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise SupervisorProtocolError(f"{context} is not valid UTF-8") from exc
    try:
        value = json.loads(text)
    except (json.JSONDecodeError, RecursionError, ValueError) as exc:
        raise SupervisorProtocolError(f"{context} is not valid JSON") from exc
    if type(value) is not dict:
        raise SupervisorProtocolError(f"{context} must be a JSON object")
    return value


def _dump_json_object(value: dict[str, Any], *, context: str) -> bytes:
    try:
        raw = json.dumps(
            value,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError, RecursionError) as exc:
        raise SupervisorProtocolError(f"{context} is not serializable") from exc
    if len(raw) > MAX_MESSAGE_BYTES:
        raise SupervisorProtocolError(f"{context} exceeds the byte limit")
    return raw


def _required(payload: dict[str, Any], field: str, *, context: str) -> Any:
    try:
        return payload[field]
    except KeyError:
        raise SupervisorProtocolError(f"{context} is missing {field}") from None


def _require_text(value: Any, *, field: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise SupervisorProtocolError(f"{field} must be a non-empty trimmed string")
    return value


def _require_nonce(value: Any, *, field: str = "supervisor_nonce") -> str:
    text = _require_text(value, field=field)
    if _NONCE_PATTERN.fullmatch(text) is None:
        raise SupervisorProtocolError(f"{field} must be 32 lowercase hex characters")
    return text


def _require_run_plan_binding(
    session_root: Path,
    request: SupervisorRequest,
) -> None:
    expected = str(
        session_root / f"plan-{request.product_session_id}.json"
    )
    if request.run_plan_path != expected:
        raise SupervisorProtocolError(
            "run_plan_path does not match the Product session"
        )


@dataclass(frozen=True)
class SupervisorDiscovery:
    schema_version: str
    supervisor_nonce: str
    family: str
    address: str

    def __post_init__(self) -> None:
        if self.schema_version != DISCOVERY_SCHEMA:
            raise SupervisorProtocolError("unsupported supervisor discovery schema")
        _require_nonce(self.supervisor_nonce)
        if self.family not in {"AF_UNIX", "AF_PIPE", "AF_INET"}:
            raise SupervisorProtocolError("unsupported supervisor endpoint family")
        _require_text(self.address, field="address")

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "supervisor_nonce": self.supervisor_nonce,
            "family": self.family,
            "address": self.address,
        }

    def to_bytes(self) -> bytes:
        return _dump_json_object(self.as_dict(), context="supervisor discovery")

    @classmethod
    def from_bytes(cls, raw: bytes) -> SupervisorDiscovery:
        payload = _load_json_object(
            raw,
            max_bytes=_DISCOVERY_MAX_BYTES,
            context="supervisor discovery",
        )
        return cls(
            schema_version=_required(payload, "schema_version", context="discovery"),
            supervisor_nonce=_required(payload, "supervisor_nonce", context="discovery"),
            family=_required(payload, "family", context="discovery"),
            address=_required(payload, "address", context="discovery"),
        )


@dataclass(frozen=True)
class SupervisorRequest:
    schema_version: str
    action: str
    run_plan_path: str
    product_session_id: str

    def __post_init__(self) -> None:
        if self.schema_version != REQUEST_SCHEMA:
            raise SupervisorProtocolError("unsupported supervisor request schema")
        if self.action not in _ACTIONS:
            raise SupervisorProtocolError("unsupported supervisor action")
        _require_text(self.run_plan_path, field="run_plan_path")
        if not is_product_session_id(self.product_session_id):
            raise SupervisorProtocolError("product_session_id is invalid")

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "action": self.action,
            "run_plan_path": self.run_plan_path,
            "product_session_id": self.product_session_id,
        }

    def to_bytes(self) -> bytes:
        return _dump_json_object(self.as_dict(), context="supervisor request")

    @classmethod
    def from_bytes(cls, raw: bytes) -> SupervisorRequest:
        payload = _load_json_object(
            raw,
            max_bytes=MAX_MESSAGE_BYTES,
            context="supervisor request",
        )
        return cls(
            schema_version=_required(payload, "schema_version", context="request"),
            action=_required(payload, "action", context="request"),
            run_plan_path=_required(payload, "run_plan_path", context="request"),
            product_session_id=_required(
                payload, "product_session_id", context="request"
            ),
        )


@dataclass(frozen=True)
class SupervisorResponse:
    schema_version: str
    success: bool
    result: dict[str, Any] | None
    error: dict[str, str] | None

    def __post_init__(self) -> None:
        if self.schema_version != RESPONSE_SCHEMA:
            raise SupervisorProtocolError("unsupported supervisor response schema")
        if type(self.success) is not bool:
            raise SupervisorProtocolError("response success must be a boolean")
        if self.success:
            if type(self.result) is not dict or self.error is not None:
                raise SupervisorProtocolError(
                    "successful response requires an object result and null error"
                )
        else:
            if self.result is not None or type(self.error) is not dict:
                raise SupervisorProtocolError(
                    "failed response requires null result and an error object"
                )
            code = _require_text(self.error.get("code"), field="error.code")
            if _ERROR_CODE_PATTERN.fullmatch(code) is None:
                raise SupervisorProtocolError("response error code is invalid")
            _require_text(self.error.get("message"), field="error.message")

    @classmethod
    def ok(cls, result: dict[str, Any]) -> SupervisorResponse:
        return cls(
            schema_version=RESPONSE_SCHEMA,
            success=True,
            result=result,
            error=None,
        )

    @classmethod
    def failed(cls, *, code: str, message: str) -> SupervisorResponse:
        return cls(
            schema_version=RESPONSE_SCHEMA,
            success=False,
            result=None,
            error={"code": code, "message": message},
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "success": self.success,
            "result": self.result,
            "error": self.error,
        }

    def to_bytes(self) -> bytes:
        return _dump_json_object(self.as_dict(), context="supervisor response")

    @classmethod
    def from_bytes(cls, raw: bytes) -> SupervisorResponse:
        payload = _load_json_object(
            raw,
            max_bytes=MAX_MESSAGE_BYTES,
            context="supervisor response",
        )
        return cls(
            schema_version=_required(payload, "schema_version", context="response"),
            success=_required(payload, "success", context="response"),
            result=_required(payload, "result", context="response"),
            error=_required(payload, "error", context="response"),
        )


def _expected_endpoint(session_root: Path, nonce: str) -> tuple[str, str]:
    if os.name == "nt":
        return "AF_INET", "127.0.0.1:0"
    return "AF_UNIX", str(
        (session_root / "supervisor" / f"control-{nonce}.sock").absolute()
    )


def _inet_address(address: str, *, allow_zero: bool = False) -> tuple[str, int]:
    host, separator, port_text = address.rpartition(":")
    if separator != ":" or host != "127.0.0.1" or not port_text.isdecimal():
        raise SupervisorProtocolError("supervisor endpoint identity is invalid")
    port = int(port_text)
    minimum = 0 if allow_zero else 1
    if port < minimum or port > 65535:
        raise SupervisorProtocolError("supervisor endpoint identity is invalid")
    return host, port


def _connection_address(family: str, address: str) -> str | tuple[str, int]:
    if family == "AF_INET":
        return _inet_address(address)
    return address


def _normalized_session_root(session_root: str | os.PathLike[str]) -> Path:
    root = Path(session_root)
    text = os.fspath(root)
    if not root.is_absolute() or os.path.normpath(text) != text:
        raise SupervisorProtocolError(
            "session_root must be a normalized absolute directory"
        )
    if not root.is_dir():
        raise SupervisorProtocolError("session_root must be an existing directory")
    return root


def _require_private_directory(path: Path, *, create: bool) -> None:
    try:
        if create:
            path.mkdir(mode=0o700, exist_ok=True)
    except OSError as exc:
        raise SupervisorProtocolError("cannot prepare supervisor directory") from exc
    if not path.is_dir():
        raise SupervisorProtocolError("supervisor path is not a directory")


def _require_private_file(path: Path, *, label: str) -> bytes:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise SupervisorProtocolError(f"{label} is unavailable") from exc
    if len(raw) > _DISCOVERY_MAX_BYTES:
        raise SupervisorProtocolError(f"{label} exceeds the byte limit")
    return raw


def _read_auth_key(path: Path) -> bytes:
    authkey = _require_private_file(path, label="supervisor auth key")
    if len(authkey) != _AUTH_KEY_BYTES:
        raise SupervisorProtocolError("supervisor auth key is invalid")
    return authkey


def _publish_private_file(path: Path, data: bytes) -> None:
    temporary = path.with_name(f".{path.name}.{secrets.token_hex(16)}.tmp")
    try:
        temporary.write_bytes(data)
        if os.name != "nt":
            temporary.chmod(0o600)
        os.replace(temporary, path)
    except OSError as exc:
        raise SupervisorProtocolError(
            f"cannot publish supervisor discovery: {path.name}"
        ) from exc
    finally:
        temporary.unlink(missing_ok=True)


def load_connection(
    session_root: str | os.PathLike[str],
) -> tuple[SupervisorDiscovery, bytes]:
    """Load and validate the endpoint and auth key for one session."""

    root = _normalized_session_root(session_root)
    supervisor_dir = root / "supervisor"
    _require_private_directory(supervisor_dir, create=False)
    endpoint_path = supervisor_dir / "endpoint.json"
    auth_path = supervisor_dir / "auth.key"
    discovery = SupervisorDiscovery.from_bytes(
        _require_private_file(endpoint_path, label="supervisor endpoint")
    )
    authkey = _read_auth_key(auth_path)

    expected_family, expected_address = _expected_endpoint(
        root, discovery.supervisor_nonce
    )
    if discovery.family != expected_family:
        raise SupervisorProtocolError("supervisor endpoint identity is invalid")
    if discovery.family == "AF_INET":
        _inet_address(discovery.address)
    elif discovery.address != expected_address:
        raise SupervisorProtocolError("supervisor endpoint identity is invalid")
    return discovery, authkey


def remove_discovery(
    session_root: str | os.PathLike[str],
    discovery: SupervisorDiscovery | None = None,
) -> None:
    """Remove published endpoint files and any Unix-domain socket."""

    root = Path(session_root)
    supervisor_dir = root / "supervisor"
    if discovery is None:
        try:
            discovery = SupervisorDiscovery.from_bytes(
                _require_private_file(
                    supervisor_dir / "endpoint.json",
                    label="supervisor endpoint",
                )
            )
        except (OSError, RuntimeError):
            pass
    (supervisor_dir / "endpoint.json").unlink(missing_ok=True)
    (supervisor_dir / "auth.key").unlink(missing_ok=True)
    if discovery is not None and discovery.family == "AF_UNIX":
        Path(discovery.address).unlink(missing_ok=True)
    for socket_path in supervisor_dir.glob("*.sock"):
        socket_path.unlink(missing_ok=True)


class LocalRpcServer:
    """One-listener authenticated local RPC server for a simulation session."""

    def __init__(self, session_root: str | os.PathLike[str]) -> None:
        self._session_root = _normalized_session_root(session_root)
        self._supervisor_dir = self._session_root / "supervisor"
        self._listener: mp_connection.Listener | None = None
        self._discovery: SupervisorDiscovery | None = None

    def __enter__(self) -> LocalRpcServer:
        if self._listener is not None:
            raise SupervisorProtocolError("supervisor server is already active")
        _require_private_directory(self._supervisor_dir, create=True)
        endpoint_path = self._supervisor_dir / "endpoint.json"
        auth_path = self._supervisor_dir / "auth.key"
        if endpoint_path.exists():
            raise SupervisorProtocolError("supervisor discovery already exists")
        if auth_path.exists():
            raise SupervisorProtocolError("supervisor discovery already exists")

        nonce = secrets.token_hex(16)
        authkey = secrets.token_bytes(_AUTH_KEY_BYTES)
        family, address = _expected_endpoint(self._session_root, nonce)
        listener: mp_connection.Listener | None = None
        listener_bound = False
        try:
            listener = mp_connection.Listener(
                address=(
                    _inet_address(address, allow_zero=True)
                    if family == "AF_INET"
                    else address
                ),
                family=family,
                authkey=authkey,
            )
            listener_bound = True
            if family == "AF_INET":
                host, port = listener.address
                address = f"{host}:{port}"
            discovery = SupervisorDiscovery(
                schema_version=DISCOVERY_SCHEMA,
                supervisor_nonce=nonce,
                family=family,
                address=address,
            )
            _publish_private_file(auth_path, authkey)
            _publish_private_file(endpoint_path, discovery.to_bytes())
        except BaseException:
            if listener is not None:
                listener.close()
            endpoint_path.unlink(missing_ok=True)
            auth_path.unlink(missing_ok=True)
            if family == "AF_UNIX" and listener_bound:
                Path(address).unlink(missing_ok=True)
            try:
                self._supervisor_dir.rmdir()
            except OSError:
                pass
            raise

        self._listener = listener
        self._discovery = discovery
        return self

    def serve_once(
        self,
        handler: Callable[[SupervisorRequest], SupervisorResponse],
    ) -> SupervisorResponse:
        listener = self._listener
        if listener is None:
            raise SupervisorProtocolError("supervisor server is not active")
        connection = listener.accept()
        try:
            try:
                raw_request = connection.recv_bytes(MAX_MESSAGE_BYTES)
            except (EOFError, OSError) as exc:
                raise SupervisorProtocolError(
                    "supervisor request exceeds the receive limit or is incomplete"
                ) from exc
            request = SupervisorRequest.from_bytes(raw_request)
            _require_run_plan_binding(self._session_root, request)
            try:
                response = handler(request)
                if not isinstance(response, SupervisorResponse):
                    raise TypeError("handler returned an unsupported value")
            except Exception as exc:
                message = str(exc).strip() or type(exc).__name__
                response = SupervisorResponse.failed(
                    code="handler_failed",
                    message=message[:512],
                )
            connection.send_bytes(response.to_bytes())
            return response
        finally:
            connection.close()

    def close(self) -> None:
        listener = self._listener
        discovery = self._discovery
        self._listener = None
        self._discovery = None
        if listener is not None:
            listener.close()
        if discovery is None:
            return

        remove_discovery(self._session_root, discovery)
        try:
            self._supervisor_dir.rmdir()
        except OSError:
            pass

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc: BaseException | None,
        _traceback: object | None,
    ) -> None:
        self.close()


def _send_worker_error(control: mp_connection.Connection, code: str) -> None:
    try:
        control.send_bytes(b"E" + code.encode("ascii"))
    except (EOFError, OSError):
        pass


def _client_worker(
    control: mp_connection.Connection,
    address: str,
    family: str,
    authkey: bytes,
    deadline: float,
    request_raw: bytes,
) -> None:
    connection: mp_connection.Connection | None = None
    try:
        if time.monotonic() >= deadline:
            _send_worker_error(control, "deadline_expired")
            return
        try:
            connection = mp_connection.Client(
                _connection_address(family, address),
                family=family,
                authkey=authkey,
            )
            connection.send_bytes(request_raw)
        except (AuthenticationError, EOFError, OSError):
            _send_worker_error(control, "connect_failed")
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not connection.poll(remaining):
            _send_worker_error(control, "deadline_expired")
            return
        try:
            response_raw = connection.recv_bytes(MAX_MESSAGE_BYTES)
        except (EOFError, OSError):
            _send_worker_error(control, "response_failed")
            return
        control.send_bytes(b"R" + response_raw)
    except Exception:
        _send_worker_error(control, "worker_failed")
    finally:
        if connection is not None:
            connection.close()
        control.close()


def _terminate_worker(process: BaseProcess) -> None:
    if process.pid is None:
        return
    if process.is_alive():
        process.terminate()
    process.join(timeout=0.5)
    if process.is_alive():
        process.kill()
        process.join(timeout=0.5)
    if process.is_alive():
        raise SupervisorProtocolError("supervisor client worker did not terminate")
    process.close()


def _round_trip_inet(
    discovery: SupervisorDiscovery,
    authkey: bytes,
    request_raw: bytes,
    deadline: float,
) -> SupervisorResponse:
    connection: mp_connection.Connection | None = None
    try:
        if time.monotonic() >= deadline:
            raise SupervisorProtocolError("supervisor response timed out")
        try:
            connection = mp_connection.Client(
                _connection_address(discovery.family, discovery.address),
                family=discovery.family,
                authkey=authkey,
            )
            if time.monotonic() >= deadline:
                raise SupervisorProtocolError("supervisor response timed out")
            connection.send_bytes(request_raw)
        except SupervisorProtocolError:
            raise
        except (AuthenticationError, EOFError, OSError) as exc:
            message = (
                "supervisor response timed out"
                if time.monotonic() >= deadline
                else "supervisor connection failed"
            )
            raise SupervisorProtocolError(message) from exc

        remaining = deadline - time.monotonic()
        if remaining <= 0 or not connection.poll(remaining):
            raise SupervisorProtocolError("supervisor response timed out")
        try:
            response_raw = connection.recv_bytes(MAX_MESSAGE_BYTES)
        except (EOFError, OSError) as exc:
            raise SupervisorProtocolError("supervisor response failed") from exc
        return SupervisorResponse.from_bytes(response_raw)
    finally:
        if connection is not None:
            connection.close()


def round_trip(
    session_root: str | os.PathLike[str],
    request: SupervisorRequest,
    timeout_s: float = 10,
) -> SupervisorResponse:
    """Send one authenticated request and return its response."""

    if (
        isinstance(timeout_s, bool)
        or not isinstance(timeout_s, (int, float))
        or not math.isfinite(float(timeout_s))
        or timeout_s <= 0
    ):
        raise SupervisorProtocolError("timeout_s must be a positive finite number")
    if not isinstance(request, SupervisorRequest):
        raise SupervisorProtocolError("request must be a SupervisorRequest")
    root = _normalized_session_root(session_root)
    _require_run_plan_binding(root, request)
    discovery, authkey = load_connection(root)
    deadline = time.monotonic() + float(timeout_s)
    request_raw = request.to_bytes()
    if discovery.family == "AF_INET":
        return _round_trip_inet(discovery, authkey, request_raw, deadline)

    context = multiprocessing.get_context("spawn")
    parent_control, child_control = context.Pipe(duplex=False)
    process = context.Process(
        target=_client_worker,
        args=(
            child_control,
            discovery.address,
            discovery.family,
            authkey,
            deadline,
            request_raw,
        ),
        name=f"{_WORKER_PROCESS_PREFIX}{request.action}",
    )
    started = False
    try:
        process.start()
        started = True
        child_control.close()
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not parent_control.poll(remaining):
            raise SupervisorProtocolError("supervisor response timed out")
        try:
            worker_result = parent_control.recv_bytes(_WORKER_RESULT_MAX_BYTES)
        except (EOFError, OSError) as exc:
            raise SupervisorProtocolError("supervisor client worker failed") from exc
        if not worker_result:
            raise SupervisorProtocolError("supervisor client worker failed")
        if worker_result[:1] == b"E":
            code = worker_result[1:]
            if code == b"deadline_expired":
                raise SupervisorProtocolError("supervisor response timed out")
            raise SupervisorProtocolError("supervisor connection failed")
        if worker_result[:1] != b"R":
            raise SupervisorProtocolError("supervisor client worker failed")
        return SupervisorResponse.from_bytes(worker_result[1:])
    finally:
        parent_control.close()
        if not started:
            child_control.close()
        _terminate_worker(process)
