"""Brainstem command sink for the Thunder field LCM endpoint."""

from __future__ import annotations

import asyncio
import logging
import math
import os
import threading
import time
from collections import Counter
from collections.abc import Mapping
from concurrent.futures import TimeoutError as FutureTimeoutError
from typing import Any

from runtime.msgs.geometry import Twist
from runtime.runtime_interface import TOPICS

from ..endpoint_service import LCMEndpointEvent, LCMEndpointService

logger = logging.getLogger(__name__)


class ThunderBrainstemSource:
    """Consume LingTu endpoint command velocity and call Brainstem ``Walk``.

    The source lives outside the LingTu Module graph. It is the hardware command
    sink for ``command_output_mode=endpoint_only`` field deployments.
    """

    name = "thunder_brainstem"

    def __init__(
        self,
        *,
        dog_host: str = "127.0.0.1",
        dog_port: int = 13145,
        max_linear_speed: float = 1.0,
        max_angular_speed: float = 1.0,
        cmd_vel_timeout_ms: float = 200.0,
        control_rate: float = 50.0,
        reconnect_interval: float = 3.0,
        connect_timeout_sec: float = 3.0,
        auto_enable: bool = False,
        auto_standup: bool = False,
        safe_sitdown: bool = False,
        safe_disable: bool = False,
        require_sdk: bool = True,
    ) -> None:
        self._dog_host = str(dog_host or "127.0.0.1")
        self._dog_port = int(dog_port)
        self._max_linear = max(0.0, float(max_linear_speed))
        self._max_angular = max(0.0, float(max_angular_speed))
        self._cmd_timeout = max(0.02, float(cmd_vel_timeout_ms) / 1000.0)
        self._control_period = 1.0 / max(1.0, float(control_rate))
        self._reconnect_interval = max(0.1, float(reconnect_interval))
        self._connect_timeout_sec = max(0.1, float(connect_timeout_sec))
        self._auto_enable = bool(auto_enable)
        self._auto_standup = bool(auto_standup)
        self._safe_sitdown = bool(safe_sitdown)
        self._safe_disable = bool(safe_disable)
        self._require_sdk = bool(require_sdk)

        self._service: LCMEndpointService | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._dog_msg: Any | None = None
        self._grpc_aio: Any | None = None
        self._stub: Any | None = None

        self._started = False
        self._connected = False
        self._enabled = False
        self._standing = False
        self._sdk_available = False
        self._watchdog_zero_sent = True
        self._last_cmd_ts = 0.0
        self._last_walk_ts = 0.0
        self._last_receive_ts = 0.0
        self._last_error = ""
        self._last_walk: tuple[float, float, float] | None = None
        self._received: Counter[str] = Counter()
        self._sent: Counter[str] = Counter()
        self._errors: Counter[str] = Counter()

    def start(self, service: LCMEndpointService) -> None:
        """Attach to the endpoint service and start the Brainstem connection."""

        if self._started:
            return
        self._service = service
        self._resolve_brainstem_sdk()
        self._stop.clear()
        self._started = True
        self._thread = threading.Thread(
            target=self._run_loop,
            name="lingtu-thunder-brainstem-source",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the command sink and send a final zero velocity when possible."""

        self._stop.set()
        loop = self._loop
        if loop is not None and self._connected:
            future = asyncio.run_coroutine_threadsafe(self._safe_stop(), loop)
            try:
                future.result(timeout=2.0)
            except (RuntimeError, TimeoutError, FutureTimeoutError, OSError, ValueError):
                logger.debug("Brainstem safe stop did not complete", exc_info=True)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._loop = None
        self._service = None
        self._stub = None
        self._connected = False
        self._started = False

    def on_lingtu_message(self, event: LCMEndpointEvent) -> None:
        """Consume one LingTu-to-endpoint event from the LCM endpoint service."""

        self._received[event.topic] += 1
        self._last_receive_ts = event.ts
        if event.topic != TOPICS.cmd_vel:
            return

        walk = self._twist_to_walk(event.message)
        self._last_cmd_ts = time.time()
        self._watchdog_zero_sent = False
        self._last_walk = walk
        self._submit_walk(walk, reason="cmd_vel")

    def health(self) -> Mapping[str, Any]:
        """Return command-sink status without exposing transport internals."""

        return {
            "name": self.name,
            "hardware": True,
            "role": "brainstem_command_sink",
            "started": self._started,
            "connected": self._connected,
            "enabled": self._enabled,
            "standing": self._standing,
            "host": f"{self._dog_host}:{self._dog_port}",
            "sdk_required": self._require_sdk,
            "sdk_available": self._sdk_available,
            "auto_enable": self._auto_enable,
            "auto_standup": self._auto_standup,
            "cmd_vel_timeout_sec": self._cmd_timeout,
            "received": dict(self._received),
            "sent": dict(self._sent),
            "errors": dict(self._errors),
            "last_receive_ts": self._last_receive_ts,
            "last_cmd_ts": self._last_cmd_ts,
            "last_walk_ts": self._last_walk_ts,
            "last_walk": list(self._last_walk) if self._last_walk is not None else None,
            "last_error": self._last_error,
        }

    def _resolve_brainstem_sdk(self) -> None:
        try:
            import brainstem_api as dog_msg
            import grpc.aio as grpc_aio
        except ImportError as exc:
            self._sdk_available = False
            self._last_error = str(exc)
            self._errors["sdk_import"] += 1
            if self._require_sdk:
                raise RuntimeError(
                    "thunder_brainstem source requires brainstem_api and grpc.aio; "
                    "install LingTu with the thunder extra or use --source smoke/jsonl"
                ) from exc
            return

        self._dog_msg = dog_msg
        self._grpc_aio = grpc_aio
        self._sdk_available = True

    def _run_loop(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._main())
        finally:
            try:
                loop.close()
            finally:
                self._loop = None

    async def _main(self) -> None:
        while not self._stop.is_set():
            try:
                await self._connect_and_run()
            except Exception as exc:
                self._connected = False
                self._stub = None
                self._last_error = str(exc)
                self._errors[type(exc).__name__] += 1
                logger.warning("Brainstem endpoint source reconnecting after error: %s", exc)
                await self._sleep_with_stop(self._reconnect_interval)

    async def _connect_and_run(self) -> None:
        if self._dog_msg is None or self._grpc_aio is None:
            await self._sleep_with_stop(self._reconnect_interval)
            return

        addr = f"{self._dog_host}:{self._dog_port}"
        async with self._grpc_aio.insecure_channel(addr) as channel:
            await asyncio.wait_for(channel.channel_ready(), timeout=self._connect_timeout_sec)
            self._stub = self._dog_msg.RobotControlStub(channel)
            self._connected = True
            self._last_error = ""
            logger.info("Brainstem endpoint source connected to %s", addr)
            if self._auto_enable:
                await self._stub.Enable(self._dog_msg.Empty())
                self._enabled = True
            if self._auto_standup:
                await self._stub.StandUp(self._dog_msg.Empty())
                self._standing = True
            await self._watchdog_loop()

    async def _watchdog_loop(self) -> None:
        while not self._stop.is_set():
            await asyncio.sleep(self._control_period)
            last_cmd = self._last_cmd_ts
            if not last_cmd or self._watchdog_zero_sent:
                continue
            if time.time() - last_cmd >= self._cmd_timeout:
                await self._send_walk((0.0, 0.0, 0.0), reason="watchdog")
                self._watchdog_zero_sent = True

    async def _safe_stop(self) -> None:
        await self._send_walk((0.0, 0.0, 0.0), reason="stop")
        if self._stub is None or self._dog_msg is None:
            return
        if self._safe_sitdown:
            await self._stub.SitDown(self._dog_msg.Empty())
            self._standing = False
        if self._safe_disable:
            await self._stub.Disable(self._dog_msg.Empty())
            self._enabled = False

    async def _send_walk(self, walk: tuple[float, float, float], *, reason: str) -> None:
        if self._stub is None or self._dog_msg is None or not self._connected:
            self._errors["not_connected"] += 1
            return
        try:
            await self._stub.Walk(self._dog_msg.Vector3(x=walk[0], y=walk[1], z=walk[2]))
        except Exception as exc:
            self._last_error = str(exc)
            raise
        self._sent[reason] += 1
        self._last_walk = walk
        self._last_walk_ts = time.time()

    async def _sleep_with_stop(self, delay: float) -> None:
        deadline = time.time() + max(0.0, delay)
        while not self._stop.is_set() and time.time() < deadline:
            await asyncio.sleep(min(0.1, max(0.0, deadline - time.time())))

    def _submit_walk(self, walk: tuple[float, float, float], *, reason: str) -> None:
        loop = self._loop
        if loop is None or not self._started:
            self._errors["not_started"] += 1
            return
        future = asyncio.run_coroutine_threadsafe(self._send_walk(walk, reason=reason), loop)
        future.add_done_callback(self._record_async_error)

    def _record_async_error(self, future: asyncio.Future[Any]) -> None:
        try:
            future.result()
        except Exception as exc:
            self._last_error = str(exc)
            self._errors[type(exc).__name__] += 1

    def _twist_to_walk(self, message: Any) -> tuple[float, float, float]:
        vx, vy, wz = _twist_components(message)
        return (
            _clamp_normalized(vx, self._max_linear),
            _clamp_normalized(vy, self._max_linear),
            _clamp_normalized(wz, self._max_angular),
        )


def create(**overrides: Any) -> ThunderBrainstemSource:
    """Factory used by endpoint runner ``--source thunder_brainstem``."""

    values = {
        "dog_host": _env_str("LINGTU_BRAINSTEM_HOST", _env_str("LINGTU_DOG_HOST", "127.0.0.1")),
        "dog_port": _env_int("LINGTU_BRAINSTEM_PORT", _env_int("LINGTU_DOG_PORT", 13145)),
        "max_linear_speed": _env_float(
            "LINGTU_BRAINSTEM_MAX_LINEAR",
            _env_float("LINGTU_MAX_LINEAR_SPEED", 1.0),
        ),
        "max_angular_speed": _env_float(
            "LINGTU_BRAINSTEM_MAX_ANGULAR",
            _env_float("LINGTU_MAX_ANGULAR_SPEED", 1.0),
        ),
        "cmd_vel_timeout_ms": _env_float(
            "LINGTU_BRAINSTEM_CMD_TIMEOUT_MS",
            _env_float("LINGTU_CMD_VEL_TIMEOUT_MS", 200.0),
        ),
        "control_rate": _env_float("LINGTU_BRAINSTEM_CONTROL_RATE", 50.0),
        "reconnect_interval": _env_float("LINGTU_BRAINSTEM_RECONNECT_SEC", 3.0),
        "connect_timeout_sec": _env_float("LINGTU_BRAINSTEM_CONNECT_TIMEOUT_SEC", 3.0),
        "auto_enable": _env_bool("LINGTU_BRAINSTEM_AUTO_ENABLE", False),
        "auto_standup": _env_bool("LINGTU_BRAINSTEM_AUTO_STANDUP", False),
        "safe_sitdown": _env_bool("LINGTU_BRAINSTEM_SAFE_SITDOWN", False),
        "safe_disable": _env_bool("LINGTU_BRAINSTEM_SAFE_DISABLE", False),
        "require_sdk": _env_bool("LINGTU_BRAINSTEM_REQUIRE_SDK", True),
    }
    values.update(overrides)
    return ThunderBrainstemSource(**values)


def _twist_components(message: Any) -> tuple[float, float, float]:
    if isinstance(message, Twist):
        return (
            _finite(message.linear.x),
            _finite(message.linear.y),
            _finite(message.angular.z),
        )
    if isinstance(message, Mapping):
        linear = message.get("linear", {})
        angular = message.get("angular", {})
        if not isinstance(linear, Mapping):
            linear = {}
        if not isinstance(angular, Mapping):
            angular = {}
        return (
            _finite(linear.get("x", message.get("linear_x", 0.0))),
            _finite(linear.get("y", message.get("linear_y", 0.0))),
            _finite(angular.get("z", message.get("angular_z", 0.0))),
        )
    linear = getattr(message, "linear", None)
    angular = getattr(message, "angular", None)
    return (
        _finite(getattr(linear, "x", 0.0)),
        _finite(getattr(linear, "y", 0.0)),
        _finite(getattr(angular, "z", 0.0)),
    )


def _clamp_normalized(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return max(-1.0, min(1.0, value / limit))


def _finite(value: Any) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return 0.0
    return number if math.isfinite(number) else 0.0


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return default if value in (None, "") else value


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value in (None, ""):
        return int(default)
    try:
        return int(value)
    except ValueError:
        return int(default)


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value in (None, ""):
        return float(default)
    try:
        return float(value)
    except ValueError:
        return float(default)


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value in (None, ""):
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}
