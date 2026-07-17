"""Teleop gateway helpers.

This module owns the gateway-side transport details for operator velocity
requests. GatewayModule keeps the public method names for route/test
compatibility, but the socket/DDS mechanics live here.
"""

from __future__ import annotations

import logging
import os
import socket
import threading
import time
from typing import Any

from runtime.adapters.native.navigation import (
    NavigationClientError,
    get_native_navigation_client,
)
from runtime.msgs.geometry import Twist, Vector3
from runtime.runtime_interface import TOPICS

logger = logging.getLogger(__name__)


class LatestNativeTeleopPublisher:
    """Non-blocking latest-value publisher for high-rate joystick commands.

    Ordinary joystick samples are replaceable: while one typed DDS command is
    awaiting its application ACK, only the newest pending sample is retained.
    Safety-critical release/stop paths use :meth:`quiesce_and_send_zero`, which
    prevents an older sample from being published after the zero command.
    """

    def __init__(self, client: Any) -> None:
        self._client = client
        self._condition = threading.Condition()
        self._pending: tuple[float, float, float, str | None] | None = None
        self._inflight = False
        self._accepting = True
        self._closed = False
        self._last_error: str | None = None
        self._thread = threading.Thread(
            target=self._run,
            name="lingtu-native-teleop",
            daemon=True,
        )
        self._thread.start()

    @property
    def last_error(self) -> str | None:
        with self._condition:
            return self._last_error

    def submit(
        self,
        vx: float,
        vy: float,
        wz: float,
        *,
        request_id: str | None = None,
    ) -> bool:
        with self._condition:
            if self._closed or not self._accepting:
                return False
            self._pending = (float(vx), float(vy), float(wz), request_id)
            self._condition.notify()
            return True

    def quiesce_and_send_zero(
        self,
        *,
        request_id: str | None = None,
        timeout_s: float = 2.0,
    ) -> None:
        """Drop replaceable samples, wait for in-flight ACK, then send zero."""

        deadline = time.monotonic() + max(0.1, float(timeout_s))
        with self._condition:
            if self._closed:
                raise NavigationClientError("native teleop publisher is closed")
            self._accepting = False
            self._pending = None
            while self._inflight:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise NavigationClientError("native teleop publisher did not quiesce before release")
                self._condition.wait(remaining)
        try:
            self._client.send_teleop(0.0, 0.0, 0.0, request_id=request_id)
            error = None
        except Exception as exc:
            error = str(exc)
            raise
        finally:
            with self._condition:
                self._last_error = error
                self._accepting = not self._closed and error is None
                self._condition.notify_all()

    def quiesce(self, *, timeout_s: float = 2.0) -> None:
        """Prevent queued/in-flight joystick samples from crossing a stop."""

        deadline = time.monotonic() + max(0.1, float(timeout_s))
        with self._condition:
            if self._closed:
                return
            self._accepting = False
            self._pending = None
            while self._inflight:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise NavigationClientError("native teleop publisher did not quiesce before stop")
                self._condition.wait(remaining)

    def resume(self) -> None:
        with self._condition:
            if not self._closed:
                self._last_error = None
                self._accepting = True
                self._condition.notify_all()

    def close(self, *, timeout_s: float = 2.0) -> None:
        with self._condition:
            self._closed = True
            self._accepting = False
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout_s)))

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                command = self._pending
                self._pending = None
                self._inflight = True
            assert command is not None
            try:
                self._client.send_teleop(
                    command[0],
                    command[1],
                    command[2],
                    request_id=command[3],
                )
                error = None
            except Exception as exc:  # endpoint freshness still fails closed
                error = str(exc)
                logger.error("GatewayModule: native teleop publish failed: %s", exc)
            finally:
                with self._condition:
                    self._last_error = error
                    if error is not None:
                        self._accepting = False
                        self._pending = None
                    self._inflight = False
                    self._condition.notify_all()


def _parse_legacy_bool(raw: str) -> bool:
    value = raw.strip().lower()
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"invalid LINGTU_TELEOP_CMD_DDS value: {raw!r}")


def resolve_native_command_boundary(
    *,
    command_output_mode: str,
    legacy_dds_env: str | None,
) -> bool:
    """Resolve the product command boundary from one authoritative policy.

    ``LINGTU_TELEOP_CMD_DDS`` remains a compatibility input only when no
    product command-output mode is declared.  An explicit contradiction is a
    startup error because silently accepting it can create two velocity
    writers in the same runtime.
    """

    mode = (command_output_mode or "").strip().lower()
    legacy = _parse_legacy_bool(legacy_dds_env) if legacy_dds_env is not None else None
    if mode == "endpoint_only":
        if legacy is False:
            raise ValueError("LINGTU_TELEOP_CMD_DDS=0 conflicts with LINGTU_COMMAND_OUTPUT_MODE=endpoint_only")
        return True
    if mode == "local_driver":
        if legacy is True:
            raise ValueError("LINGTU_TELEOP_CMD_DDS=1 conflicts with LINGTU_COMMAND_OUTPUT_MODE=local_driver")
        return False
    return bool(legacy)


def parse_bridge_addr(raw: str) -> tuple[str, int] | None:
    raw = (raw or "").strip()
    if not raw:
        return None
    if ":" not in raw:
        raise ValueError("LINGTU_TELEOP_BRIDGE_ADDR must be HOST:PORT")
    host, port = raw.rsplit(":", 1)
    return host.strip(), int(port)


def init_teleop_state(
    gw: Any,
    *,
    max_speed: float,
    max_yaw: float,
    release_timeout: float,
    bridge_addr_raw: str,
    dds_enabled: bool,
) -> None:
    gw._teleop_module = None
    gw._teleop_clients = 0
    gw._teleop_clients_lock = threading.Lock()
    gw._latest_jpeg = None
    gw._latest_jpeg_seq = 0
    gw._jpeg_lock = threading.Lock()
    gw._teleop_max_speed = max_speed
    gw._teleop_max_yaw = max_yaw
    gw._teleop_release_timeout = release_timeout
    gw._teleop_bridge_addr = parse_bridge_addr(bridge_addr_raw)
    gw._teleop_bridge_sock = (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if gw._teleop_bridge_addr is not None else None
    )
    gw._teleop_dds_enabled = dds_enabled
    gw._teleop_native_client = None
    gw._teleop_native_publisher = None
    if dds_enabled:
        try:
            gw._teleop_native_client = get_native_navigation_client(required=True)
            gw._teleop_native_publisher = LatestNativeTeleopPublisher(gw._teleop_native_client)
        except NavigationClientError as exc:
            logger.error(
                "GatewayModule: native client for %s unavailable: %s; "
                "field teleop is disabled until the typed command boundary recovers",
                TOPICS.teleop_cmd_vel,
                exc,
            )


def configure_teleop(
    gw: Any,
    *,
    max_speed: float,
    max_yaw: float,
    release_timeout: float,
) -> None:
    gw._teleop_max_speed = max_speed
    gw._teleop_max_yaw = max_yaw
    gw._teleop_release_timeout = release_timeout


def write_bridge(gw: Any, twist: Twist) -> bool:
    sock = gw._teleop_bridge_sock
    addr = gw._teleop_bridge_addr
    if sock is None or addr is None:
        return False
    try:
        payload = (f"{float(twist.linear.x):.9g} {float(twist.linear.y):.9g} {float(twist.angular.z):.9g}\n").encode(
            "ascii"
        )
        sock.sendto(payload, addr)
        return True
    except Exception as exc:
        logger.debug("GatewayModule: teleop bridge write failed: %s", exc)
        return False


def publish_remote_velocity_request(
    gw: Any,
    twist: Twist,
    *,
    publish_local_compat: bool = True,
    request_id: str | None = None,
) -> bool:
    """Publish an operator velocity request.

    Field endpoints send a typed ``NavigationCommandRequest(kind=teleop)`` to
    the C++ command arbiter. It checks freshness/localization/live obstacles/
    traversability and is the only writer of final ``/nav/cmd_vel``. The
    in-process publish is kept only for explicit dev/sim/compat profiles.
    """

    if bool(getattr(gw, "_teleop_dds_enabled", False)):
        client = getattr(gw, "_teleop_native_client", None)
        if client is None:
            raise NavigationClientError("native teleop command boundary is unavailable")
        client.send_teleop(
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
            request_id=request_id,
        )
        return True

    wrote_dds = write_bridge(gw, twist)
    if publish_local_compat:
        gw.cmd_vel.publish(twist)
    return wrote_dds


def twist_from_joy(gw: Any, lx: float, ly: float, az: float) -> Twist:
    vx = max(-1.0, min(1.0, float(lx))) * getattr(gw, "_teleop_max_speed", 0.5)
    vy = max(-1.0, min(1.0, float(ly))) * getattr(gw, "_teleop_max_speed", 0.5)
    wz = max(-1.0, min(1.0, float(az))) * getattr(gw, "_teleop_max_yaw", 1.0)
    return Twist(
        linear=Vector3(x=vx, y=vy, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=wz),
    )


def on_joy(gw: Any, lx: float, ly: float, az: float) -> bool:
    request = twist_from_joy(gw, lx, ly, az)
    if bool(getattr(gw, "_teleop_dds_enabled", False)):
        publisher = getattr(gw, "_teleop_native_publisher", None)
        if publisher is None:
            logger.error("GatewayModule: dropping field teleop request: native teleop publisher is unavailable")
            return False
        if not publisher.submit(
            request.linear.x,
            request.linear.y,
            request.angular.z,
        ):
            logger.debug("GatewayModule: teleop request rejected by stop/release barrier")
            return False
        return True
    tm = gw._teleop_module
    if tm is not None and hasattr(tm, "joy_input"):
        tm.joy_input._deliver({"lx": lx, "ly": ly, "az": az})
    else:
        gw.cmd_vel.publish(request)
    return True


def release(gw: Any) -> bool:
    if bool(getattr(gw, "_teleop_dds_enabled", False)):
        publisher = getattr(gw, "_teleop_native_publisher", None)
        if publisher is None:
            logger.error("GatewayModule: field teleop release failed: publisher unavailable")
            return False
        try:
            publisher.quiesce_and_send_zero(timeout_s=max(2.0, float(gw._teleop_release_timeout) + 1.0))
        except NavigationClientError as exc:
            logger.error("GatewayModule: field teleop release failed: %s", exc)
            return False
        return True
    tm = gw._teleop_module
    if tm is not None:
        tm.force_release()
    else:
        gw.cmd_vel.publish(Twist())
    return True


def quiesce_native_teleop(gw: Any, *, timeout_s: float = 2.0) -> bool:
    """Drain replaceable joystick work before a synchronous stop/estop."""

    if not bool(getattr(gw, "_teleop_dds_enabled", False)):
        return False
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is None:
        raise NavigationClientError("native teleop publisher is unavailable")
    publisher.quiesce(timeout_s=timeout_s)
    return True


def resume_native_teleop(gw: Any) -> None:
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is not None:
        publisher.resume()


def shutdown_teleop(gw: Any) -> None:
    publisher = getattr(gw, "_teleop_native_publisher", None)
    if publisher is not None:
        publisher.close()
    gw._teleop_native_publisher = None
