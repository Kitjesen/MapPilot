"""GnssBridge — connects DeviceManager.wtrtk980_main → existing GnssModule.

Bridge pattern: DeviceManager produces GnssFix from raw NMEA. We subscribe
and forward into GnssModule.inject_fix() so all existing fusion logic
(LLA→ENU, quality filter, status loop) continues to work unchanged.

This lets the device-registry refactor coexist with the legacy GnssModule
without disrupting Phase 1+2 work.
"""

from __future__ import annotations

import logging
from typing import Any

from runtime.module import Module
from runtime.registry import register
from runtime.stream import Out

logger = logging.getLogger(__name__)


@register("gnss_bridge", "device_manager", description="DeviceManager → GnssModule bridge")
class GnssBridgeModule(Module, layer=1):
    """Forwards GnssFix from DeviceManager into GnssModule pipeline.

    Decoupling: DeviceManager handles hardware/protocol; this module
    handles fusion-pipeline integration. Either side can change without
    breaking the other.
    """

    alive: Out[bool]

    def __init__(
        self,
        device_id: str = "wtrtk980_main",
        gnss_module_name: str = "GnssModule",
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._device_id = device_id
        self._gnss_module_name = gnss_module_name
        self._gnss_module = None
        self._device_manager = None

    def on_system_modules(self, modules: dict) -> None:
        """Called by Blueprint after all modules are constructed."""
        self._gnss_module = modules.get(self._gnss_module_name)
        self._device_manager = modules.get("DeviceManager")
        if self._device_manager is None:
            logger.warning("GnssBridge: DeviceManager not in system")
            return
        if self._gnss_module is None:
            logger.warning(
                "GnssBridge: %s not found — GNSS bridge inactive",
                self._gnss_module_name,
            )
            return
        self._device_manager.subscribe(self._device_id, self._on_fix)
        logger.info(
            "GnssBridge: subscribed to device '%s' → %s",
            self._device_id, self._gnss_module_name,
        )

    def start(self) -> None:
        super().start()
        self.alive.publish(True)

    def _on_fix(self, fix: Any) -> None:
        """Forward GnssFix from DeviceManager into GnssModule."""
        if self._gnss_module is None:
            return
        try:
            self._gnss_module.inject_fix(fix)
        except Exception as e:
            logger.debug("GnssBridge dispatch error: %s", e)
