"""Gateway background thread lifecycle helpers."""

from __future__ import annotations

import logging
import threading
from typing import Any

logger = logging.getLogger(__name__)


def start_background_threads(gw: Any) -> None:
    stop_event = gw._stop_event
    if not gw._defer_server and (gw._server_thread is None or not gw._server_thread.is_alive()):
        gw._server_thread = threading.Thread(
            target=gw._run_server,
            args=(stop_event,),
            daemon=True,
            name="gateway",
        )
        gw._server_thread.start()
        gw._start_client_http_prewarm(stop_event, timeout_s=15.0)

    if gw._saved_map_loader_thread is None or not gw._saved_map_loader_thread.is_alive():
        gw._saved_map_loader_thread = threading.Thread(
            target=gw._saved_map_loader_loop,
            args=(stop_event,),
            daemon=True,
            name="saved_map_loader",
        )
        gw._saved_map_loader_thread.start()

    if gw._drift_watchdog_enabled and (gw._drift_watchdog_thread is None or not gw._drift_watchdog_thread.is_alive()):
        gw._drift_watchdog_thread = threading.Thread(
            target=gw._drift_watchdog_loop,
            args=(stop_event,),
            daemon=True,
            name="drift_watchdog",
        )
        gw._drift_watchdog_thread.start()


def stop_background_threads(gw: Any) -> None:
    gw._stop_event.set()
    server = gw._server
    if server is not None:
        try:
            server.should_exit = True
        except Exception:
            logger.debug(
                "GatewayModule: failed to signal uvicorn shutdown",
                exc_info=True,
            )

    current = threading.current_thread()
    for attr_name in (
        "_server_thread",
        "_client_http_prewarm_thread",
        "_saved_map_loader_thread",
        "_drift_watchdog_thread",
        "_brainstem_health_refresh_thread",
    ):
        thread = getattr(gw, attr_name, None)
        if thread is None:
            continue
        if thread is current:
            setattr(gw, attr_name, None)
            continue
        if thread.is_alive():
            thread.join(timeout=2.0)
            if thread.is_alive():
                logger.warning(
                    "GatewayModule: %s did not stop within timeout",
                    thread.name,
                )
                continue
        setattr(gw, attr_name, None)
