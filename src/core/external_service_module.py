"""ExternalServiceManagerModule — lifecycle boundary for robot systemd services.

Stack factories should describe Module graphs, not start/stop system services while
building a Blueprint. This module moves the side effect to SystemHandle.start():
setup() applies the requested stop/ensure/wait plan before downstream bridge and
perception modules start reading from those services.
"""

from __future__ import annotations

import logging
from typing import Iterable

from core.module import Module

logger = logging.getLogger(__name__)


class ExternalServiceManagerModule(Module, layer=0):
    """Apply a logical robot service plan during runtime startup."""

    def __init__(
        self,
        stop_services: Iterable[str] = (),
        ensure_services: Iterable[str] = (),
        wait_ready_services: Iterable[str] = (),
        wait_ready_timeout: float = 10.0,
        stop_on_shutdown: bool = False,
        **kw,
    ):
        super().__init__(**kw)
        self._stop_services = tuple(service for service in stop_services if service)
        self._ensure_services = tuple(service for service in ensure_services if service)
        self._wait_ready_services = tuple(service for service in wait_ready_services if service)
        self._wait_ready_timeout = float(wait_ready_timeout)
        self._stop_on_shutdown = bool(stop_on_shutdown)
        self._service_manager = None

    def setup(self) -> None:
        try:
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            self._service_manager = svc
            if self._stop_services:
                svc.stop(*self._stop_services)
            if self._ensure_services:
                svc.ensure(*self._ensure_services)
            if self._wait_ready_services:
                svc.wait_ready(*self._wait_ready_services, timeout=self._wait_ready_timeout)
            logger.info(
                "External services prepared: stop=%s ensure=%s wait=%s",
                self._stop_services,
                self._ensure_services,
                self._wait_ready_services,
            )
        except Exception as exc:
            logger.warning(
                "ExternalServiceManagerModule: service manager unavailable; "
                "external services were not prepared: %s",
                exc,
            )

    def stop(self) -> None:
        if self._stop_on_shutdown and self._service_manager is not None:
            try:
                self._service_manager.stop_all_started()
            except Exception as exc:
                logger.warning("ExternalServiceManagerModule shutdown cleanup failed: %s", exc)
        super().stop()
