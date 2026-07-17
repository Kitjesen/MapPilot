"""GNSS bridge from hw device data into GnssModule."""

from __future__ import annotations

import logging
from typing import Any

from runtime.contracts import HW_COMPAT_ALIAS, HW_ROLE
from runtime.module import Module
from runtime.registry import register
from runtime.stream import Out

logger = logging.getLogger(__name__)


@register("gnss_bridge", "hw", description="hw -> GnssModule bridge")
@register("gnss_bridge", "device_manager", description="compat hw -> GnssModule bridge")
class GnssBridgeModule(Module, layer=1):
    """Forward GNSS fixes from hw into the existing GnssModule pipeline."""

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
        self._hw = None

    def on_system_modules(self, modules: dict) -> None:
        """Called by Blueprint after all modules are constructed."""
        self._gnss_module = modules.get(self._gnss_module_name)
        self._hw = modules.get(HW_ROLE) or modules.get(HW_COMPAT_ALIAS)
        if self._hw is None:
            logger.warning("GnssBridge: hw not in system")
            return
        if self._gnss_module is None:
            logger.warning(
                "GnssBridge: %s not found; GNSS bridge inactive",
                self._gnss_module_name,
            )
            return
        self._hw.subscribe(self._device_id, self._on_fix)
        logger.info(
            "GnssBridge: subscribed to device %r -> %s",
            self._device_id,
            self._gnss_module_name,
        )

    def start(self) -> None:
        super().start()
        self.alive.publish(True)

    def _on_fix(self, fix: Any) -> None:
        """Forward GNSS fix into GnssModule."""
        if self._gnss_module is None:
            return
        try:
            self._gnss_module.inject_fix(fix)
        except Exception as e:
            logger.debug("GnssBridge dispatch error: %s", e)
