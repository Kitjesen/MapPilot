"""Driver swap request handling."""

from __future__ import annotations

import logging
import time
from typing import Any

logger = logging.getLogger(__name__)


def handle_driver_swap(gw: Any, payload: dict[str, Any]) -> dict[str, Any]:
    """Swap the active driver backend through the registered SwapManager."""
    swap = gw._swap_manager
    if swap is None:
        return {
            "success": False,
            "message": "SwapManager not available -driver swap requires a running SwapManager",
            "swap_time_ms": 0.0,
        }

    driver_name = str(payload.get("driver", ""))
    if not driver_name:
        return {
            "success": False,
            "message": "Missing required field: driver",
            "swap_time_ms": 0.0,
        }

    config = payload.get("config") or {}
    if not isinstance(config, dict):
        return {
            "success": False,
            "message": "config must be a dict",
            "swap_time_ms": 0.0,
        }

    try:
        t0 = time.monotonic()
        result = swap.swap(driver_name, **config)
        elapsed = (time.monotonic() - t0) * 1000.0
        if isinstance(result, dict) and not result.get("success", True):
            return {
                "success": False,
                "message": result.get("message", "swap failed"),
                "swap_time_ms": elapsed,
                "detail": result,
            }
        return {
            "success": True,
            "message": f"Swapped to {driver_name}",
            "swap_time_ms": elapsed,
        }
    except Exception as exc:
        logger.exception("Driver swap to %s failed", driver_name)
        return {
            "success": False,
            "message": str(exc),
            "swap_time_ms": 0.0,
        }
