"""Thunder Brainstem endpoint source.

This source keeps the real robot motion driver outside the LingTu module graph.
It consumes muxed LingTu ``/nav/cmd_vel`` endpoint events and forwards them to
the existing Thunder Brainstem driver. Optional dead-reckoning odometry can be
published for bring-up, but field navigation should normally get localization
from the dedicated SLAM/localization endpoint source.
"""

from __future__ import annotations

import logging
import os
import time
from collections import Counter
from collections.abc import Callable, Mapping
from typing import Any

from core.msgs.geometry import Twist
from core.msgs.nav import Odometry
from core.runtime_interface import TOPICS

from ..endpoint_service import LCMEndpointEvent, LCMEndpointService

logger = logging.getLogger(__name__)


class ThunderBrainstemEndpointSource:
    """Endpoint-side Brainstem command sink for Thunder field deployments."""

    name = "thunder_brainstem"

    def __init__(
        self,
        *,
        driver_factory: Callable[..., Any] | None = None,
        driver_config: Mapping[str, Any] | None = None,
        publish_odometry: bool = False,
        zero_on_stop: bool = True,
    ) -> None:
        self._driver_factory = driver_factory or _default_driver_factory
        self._driver_config = dict(driver_config or {})
        self._publish_odometry = bool(publish_odometry)
        self._zero_on_stop = bool(zero_on_stop)
        self._driver: Any | None = None
        self._service: LCMEndpointService | None = None
        self._unsubscribers: list[Callable[[], None]] = []
        self._started = False
        self._received: Counter[str] = Counter()
        self._published: Counter[str] = Counter()
        self._last_receive_ts = 0.0
        self._last_publish_ts = 0.0
        self._last_robot_state: Mapping[str, Any] = {}
        self._alive: bool | None = None

    def start(self, service: LCMEndpointService) -> None:
        """Start the Brainstem driver and attach endpoint publishers."""

        if self._started:
            return
        self._service = service
        self._driver = self._driver_factory(**self._driver_config)
        self._subscribe_driver_outputs(self._driver)
        setup = getattr(self._driver, "setup", None)
        if callable(setup):
            setup()
        start = getattr(self._driver, "start", None)
        if callable(start):
            start()
        self._started = True

    def stop(self) -> None:
        """Stop the driver and release endpoint subscriptions."""

        driver = self._driver
        if driver is not None and self._zero_on_stop:
            self._deliver_to_driver("cmd_vel", Twist())
        for unsubscribe in reversed(self._unsubscribers):
            try:
                unsubscribe()
            except Exception:
                logger.debug("Thunder endpoint source unsubscribe failed", exc_info=True)
        self._unsubscribers.clear()
        if driver is not None:
            stop = getattr(driver, "stop", None)
            if callable(stop):
                stop()
        self._driver = None
        self._service = None
        self._started = False

    def on_lingtu_message(self, event: LCMEndpointEvent) -> None:
        """Forward LingTu endpoint commands to the Brainstem driver."""

        self._received[event.topic] += 1
        self._last_receive_ts = event.ts
        if event.topic == TOPICS.cmd_vel:
            if not isinstance(event.message, Twist):
                raise TypeError(
                    f"{self.name} expected Twist for {TOPICS.cmd_vel}, "
                    f"got {type(event.message).__name__}"
                )
            self._deliver_to_driver("cmd_vel", event.message)

    def health(self) -> Mapping[str, Any]:
        """Return command-sink and driver status."""

        driver_health: Mapping[str, Any] = {}
        if self._driver is not None:
            health = getattr(self._driver, "health", None)
            if callable(health):
                try:
                    driver_health = dict(health())
                except Exception:
                    logger.debug("Thunder driver health failed", exc_info=True)
        return {
            "name": self.name,
            "hardware": True,
            "role": "motion_command_sink",
            "started": self._started,
            "driver_config": _public_driver_config(self._driver_config),
            "publish_odometry": self._publish_odometry,
            "zero_on_stop": self._zero_on_stop,
            "received": dict(self._received),
            "published": dict(self._published),
            "last_receive_ts": self._last_receive_ts,
            "last_publish_ts": self._last_publish_ts,
            "alive": self._alive,
            "robot_state": dict(self._last_robot_state),
            "driver": dict(driver_health),
        }

    def _subscribe_driver_outputs(self, driver: Any) -> None:
        self._subscribe_output(driver, "alive", self._on_alive)
        self._subscribe_output(driver, "robot_state", self._on_robot_state)
        if self._publish_odometry:
            self._subscribe_output(driver, "odometry", self._on_odometry)

    def _subscribe_output(self, driver: Any, port_name: str, callback: Callable[[Any], None]) -> None:
        port = getattr(driver, port_name, None)
        subscribe = getattr(port, "subscribe", None)
        if callable(subscribe):
            unsubscribe = subscribe(callback)
            if callable(unsubscribe):
                self._unsubscribers.append(unsubscribe)

    def _on_alive(self, value: Any) -> None:
        self._alive = bool(value)

    def _on_robot_state(self, state: Any) -> None:
        if isinstance(state, Mapping):
            self._last_robot_state = dict(state)
        else:
            self._last_robot_state = {"value": state}

    def _on_odometry(self, odometry: Odometry) -> None:
        service = self._service
        if service is None:
            return
        count = service.publish_localization_snapshot(
            odometry=odometry,
            localization_health={
                "state": "BRAINSTEM_DEAD_RECKONING",
                "quality": 0.0,
                "source": self.name,
            },
            localization_quality=0.0,
        )
        self._published[TOPICS.odometry] += 1
        self._published[TOPICS.localization_health] += 1
        self._published[TOPICS.localization_quality] += 1
        self._last_publish_ts = time.time()
        if count != 3:
            logger.debug("Thunder Brainstem odometry publish count was %s", count)

    def _deliver_to_driver(self, port_name: str, message: Any) -> None:
        driver = self._driver
        if driver is None:
            logger.warning("Thunder endpoint source dropped %s before driver start", port_name)
            return
        port = getattr(driver, port_name, None)
        deliver = getattr(port, "_deliver", None)
        if callable(deliver):
            deliver(message)
            return
        callback = getattr(driver, f"_on_{port_name}", None)
        if callable(callback):
            callback(message)
            return
        raise TypeError(f"Thunder driver input port '{port_name}' is not deliverable")


def create(
    *,
    driver_factory: Callable[..., Any] | None = None,
    **overrides: Any,
) -> ThunderBrainstemEndpointSource:
    """Factory used by endpoint runner ``--source thunder_brainstem``."""

    config = _driver_config_from_env()
    publish_odometry = _env_bool("LINGTU_THUNDER_PUBLISH_ODOMETRY", False)
    zero_on_stop = _env_bool("LINGTU_THUNDER_ZERO_ON_STOP", True)
    if "publish_odometry" in overrides:
        publish_odometry = bool(overrides.pop("publish_odometry"))
    if "zero_on_stop" in overrides:
        zero_on_stop = bool(overrides.pop("zero_on_stop"))
    config.update(overrides)
    return ThunderBrainstemEndpointSource(
        driver_factory=driver_factory,
        driver_config=config,
        publish_odometry=publish_odometry,
        zero_on_stop=zero_on_stop,
    )


def _default_driver_factory(**config: Any) -> Any:
    from drivers.real.thunder.han_dog_module import ThunderDriver

    return ThunderDriver(**config)


def _driver_config_from_env() -> dict[str, Any]:
    return {
        "dog_host": _env_str(
            "LINGTU_THUNDER_DOG_HOST",
            _env_str("LINGTU_DOG_HOST", _env_str("DOG_HOST", "127.0.0.1")),
        ),
        "dog_port": _env_int(
            "LINGTU_THUNDER_DOG_PORT",
            _env_int("LINGTU_DOG_PORT", _env_int("DOG_PORT", 13145)),
        ),
        "max_linear_speed": _env_float("LINGTU_THUNDER_MAX_LINEAR_SPEED", 1.0),
        "max_angular_speed": _env_float("LINGTU_THUNDER_MAX_ANGULAR_SPEED", 1.0),
        "cmd_vel_timeout_ms": _env_float("LINGTU_THUNDER_CMD_VEL_TIMEOUT_MS", 200.0),
        "control_rate": _env_float("LINGTU_THUNDER_CONTROL_RATE", 50.0),
        "auto_enable": _env_bool("LINGTU_THUNDER_AUTO_ENABLE", False),
        "auto_standup": _env_bool("LINGTU_THUNDER_AUTO_STANDUP", False),
    }


def _public_driver_config(config: Mapping[str, Any]) -> dict[str, Any]:
    public = dict(config)
    for key in tuple(public):
        if "token" in key.lower() or "password" in key.lower() or "secret" in key.lower():
            public[key] = "<redacted>"
    return public


def _env_str(name: str, default: str) -> str:
    value = os.getenv(name)
    return str(value).strip() if value not in (None, "") else default


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value in (None, ""):
        return int(default)
    return int(str(value).strip())


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value in (None, ""):
        return float(default)
    return float(str(value).strip())


def _env_bool(name: str, default: bool) -> bool:
    value = os.getenv(name)
    if value in (None, ""):
        return bool(default)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}
