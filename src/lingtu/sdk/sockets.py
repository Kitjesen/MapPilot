"""Optional WebSocket feeds and connection-scoped operator input."""

from __future__ import annotations

import asyncio
import importlib
import json
import threading
import time
from typing import TYPE_CHECKING, Any, TypeVar
from urllib.parse import urlencode, urlsplit, urlunsplit

from ._common import _new_request_id

if TYPE_CHECKING:
    from .client import LingTuClient

_SyncSession = TypeVar("_SyncSession", bound="_Socket")
_AsyncSession = TypeVar("_AsyncSession", bound="_AsyncSocket")


def _connector(asynchronous: bool):
    module = "websockets.asyncio.client" if asynchronous else "websockets.sync.client"
    try:
        return importlib.import_module(module).connect
    except ImportError as exc:
        raise ImportError("WebSocket support requires the optional extra. Install 'lingtu-sdk[websocket]'.") from exc


def _connection(client: LingTuClient, path: str, timeout: float | None, viewer_token: str | None):
    address = urlsplit(client.base_url)
    url = urlunsplit(
        (
            "wss" if address.scheme == "https" else "ws",
            address.netloc,
            path.split("?", 1)[0],
            path.partition("?")[2],
            "",
        )
    )
    headers = {}
    if client._api_key:
        headers["X-API-Key"] = client._api_key
    if viewer_token is not None:
        headers["Authorization"] = f"Bearer {viewer_token}"
    resolved_timeout = client._timeout if timeout is None else timeout
    if resolved_timeout <= 0:
        raise ValueError("WebSocket timeout must be positive")
    options = {
        "additional_headers": headers,
        "open_timeout": resolved_timeout,
        "close_timeout": resolved_timeout,
        "max_size": None,
    }
    return url, resolved_timeout, options


def _control_response(raw: str | bytes, request_id: str, expected: str) -> dict[str, Any] | None:
    response = json.loads(raw)
    if not isinstance(response, dict):
        raise RuntimeError("Teleop response must be a JSON object")
    kind = response.get("type")
    if kind == "control_rejected" and not response.get("request_id"):
        return response
    if response.get("request_id") == request_id and kind in {expected, "control_rejected"}:
        return response
    return None


def _velocity(
    acknowledgement: dict[str, Any],
    request_id: str,
    vx_mps: float,
    vy_mps: float,
    yaw_rps: float,
    manual_mode: bool,
) -> dict[str, Any]:
    window = acknowledgement.get("input_window")
    if not isinstance(window, str) or not window:
        raise RuntimeError("Teleop input acknowledgement did not include an input window")
    return {
        "type": "velocity",
        "request_id": request_id,
        "input_window": window,
        "deadman": True,
        "vx_mps": vx_mps,
        "vy_mps": vy_mps,
        "yaw_rps": yaw_rps,
        "manual_mode": manual_mode,
    }


class _Socket:
    def __init__(
        self,
        client: LingTuClient,
        path: str,
        *,
        timeout: float | None = None,
        viewer_token: str | None = None,
    ) -> None:
        self._url, self._timeout, self._options = _connection(client, path, timeout, viewer_token)
        self._socket: Any = None

    def open(self: _SyncSession) -> _SyncSession:
        """Open this session explicitly."""
        if self._socket is not None:
            raise RuntimeError("WebSocket session is already open")
        self._socket = _connector(False)(self._url, **self._options)
        return self

    def __enter__(self: _SyncSession) -> _SyncSession:
        return self.open()

    def __exit__(self, *_args: object) -> None:
        self.close()

    def close(self) -> None:
        """Close the WebSocket and release its connection-scoped resources."""
        socket, self._socket = self._socket, None
        if socket is not None:
            socket.close()

    def _opened(self):
        if self._socket is None:
            raise RuntimeError("Open the WebSocket session with 'with' or open() first")
        return self._socket


class Feed(_Socket):
    """Binary cloud, scan or camera frames. Use ``with`` and close on early exit."""

    def __iter__(self) -> Feed:
        return self

    def __next__(self) -> bytes:
        socket = self._opened()
        from websockets.exceptions import ConnectionClosedOK

        try:
            frame = socket.recv(timeout=self._timeout)
            if not isinstance(frame, bytes):
                raise RuntimeError("Binary feed received a text message")
            return frame
        except ConnectionClosedOK:
            self.close()
            raise StopIteration from None
        except BaseException:
            self.close()
            raise


class Teleop(_Socket):
    """One operator connection. Receipts retain Gateway queue and native evidence."""

    def __init__(self, client: LingTuClient, *, client_id: str = "sdk", timeout: float | None = None) -> None:
        super().__init__(client, "/ws/teleop?" + urlencode({"client_id": client_id}), timeout=timeout)
        self._lock = threading.Lock()

    def _exchange(self, payload: dict[str, Any], expected: str) -> dict[str, Any]:
        socket = self._opened()
        from websockets.exceptions import ConnectionClosed

        try:
            try:
                socket.send(json.dumps(payload, allow_nan=False))
            except ConnectionClosed:
                # Gateway may queue admission rejection and close before the first input.
                pass
            deadline = time.monotonic() + self._timeout
            while True:
                response = _control_response(
                    socket.recv(timeout=max(0, deadline - time.monotonic())),
                    payload["request_id"],
                    expected,
                )
                if response is not None:
                    return response
        except BaseException:
            self.close()
            raise

    def move(
        self,
        vx_mps: float,
        vy_mps: float,
        yaw_rps: float,
        *,
        manual_mode: bool = False,
        request_id: str | None = None,
    ) -> dict[str, Any]:
        """Obtain a fresh input window and submit one velocity. Return the original ingress receipt."""
        with self._lock:
            request_id = request_id or _new_request_id()
            acknowledgement = self._exchange({"type": "input_request", "request_id": request_id}, "input_ack")
            if acknowledgement.get("type") == "control_rejected":
                return acknowledgement
            try:
                velocity = _velocity(acknowledgement, request_id, vx_mps, vy_mps, yaw_rps, manual_mode)
            except BaseException:
                self.close()
                raise
            return self._exchange(velocity, "ingress_ack")

    def hold(self, *, request_id: str | None = None) -> dict[str, Any]:
        """Release the deadman and return the original hold acknowledgement or rejection."""
        with self._lock:
            return self._exchange(
                {"type": "velocity", "deadman": False, "request_id": request_id or _new_request_id()}, "control_ack"
            )


class _AsyncSocket:
    def __init__(
        self,
        client: LingTuClient,
        path: str,
        *,
        timeout: float | None = None,
        viewer_token: str | None = None,
    ) -> None:
        self._url, self._timeout, self._options = _connection(client, path, timeout, viewer_token)
        self._socket: Any = None

    async def open(self: _AsyncSession) -> _AsyncSession:
        """Open this session explicitly without blocking the event loop."""
        if self._socket is not None:
            raise RuntimeError("WebSocket session is already open")
        self._socket = await _connector(True)(self._url, **self._options)
        return self

    async def __aenter__(self: _AsyncSession) -> _AsyncSession:
        return await self.open()

    async def __aexit__(self, *_args: object) -> None:
        await self.close()

    async def close(self) -> None:
        """Close the WebSocket and release its connection-scoped resources."""
        socket, self._socket = self._socket, None
        if socket is not None:
            await socket.close()

    def _opened(self):
        if self._socket is None:
            raise RuntimeError("Open the WebSocket session with 'async with' or await open() first")
        return self._socket


class AsyncFeed(_AsyncSocket):
    """Binary cloud, scan or camera frames using native asynchronous WebSockets."""

    def __aiter__(self) -> AsyncFeed:
        return self

    async def __anext__(self) -> bytes:
        socket = self._opened()
        from websockets.exceptions import ConnectionClosedOK

        try:
            frame = await asyncio.wait_for(socket.recv(), timeout=self._timeout)
            if not isinstance(frame, bytes):
                raise RuntimeError("Binary feed received a text message")
            return frame
        except ConnectionClosedOK:
            await self.close()
            raise StopAsyncIteration from None
        except BaseException:
            await self.close()
            raise


class AsyncTeleop(_AsyncSocket):
    """One native async operator connection with serialized input transactions."""

    def __init__(self, client: LingTuClient, *, client_id: str = "sdk", timeout: float | None = None) -> None:
        super().__init__(client, "/ws/teleop?" + urlencode({"client_id": client_id}), timeout=timeout)
        self._lock = asyncio.Lock()

    async def _exchange(self, payload: dict[str, Any], expected: str) -> dict[str, Any]:
        socket = self._opened()
        from websockets.exceptions import ConnectionClosed

        try:
            try:
                await asyncio.wait_for(socket.send(json.dumps(payload, allow_nan=False)), timeout=self._timeout)
            except ConnectionClosed:
                # Gateway may queue admission rejection and close before the first input.
                pass
            deadline = time.monotonic() + self._timeout
            while True:
                raw = await asyncio.wait_for(socket.recv(), timeout=max(0, deadline - time.monotonic()))
                response = _control_response(raw, payload["request_id"], expected)
                if response is not None:
                    return response
        except BaseException:
            await self.close()
            raise

    async def move(
        self,
        vx_mps: float,
        vy_mps: float,
        yaw_rps: float,
        *,
        manual_mode: bool = False,
        request_id: str | None = None,
    ) -> dict[str, Any]:
        """Obtain a fresh input window and submit one velocity. Return the original ingress receipt."""
        async with self._lock:
            request_id = request_id or _new_request_id()
            acknowledgement = await self._exchange({"type": "input_request", "request_id": request_id}, "input_ack")
            if acknowledgement.get("type") == "control_rejected":
                return acknowledgement
            try:
                velocity = _velocity(acknowledgement, request_id, vx_mps, vy_mps, yaw_rps, manual_mode)
            except BaseException:
                await self.close()
                raise
            return await self._exchange(velocity, "ingress_ack")

    async def hold(self, *, request_id: str | None = None) -> dict[str, Any]:
        """Release the deadman and return the original hold acknowledgement or rejection."""
        async with self._lock:
            return await self._exchange(
                {"type": "velocity", "deadman": False, "request_id": request_id or _new_request_id()}, "control_ack"
            )
