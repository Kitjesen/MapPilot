"""Uvicorn runner for the Gateway FastAPI app."""

from __future__ import annotations

import logging
import sys
import threading
from typing import Any

logger = logging.getLogger(__name__)


def run_server(gw: Any, stop_event: threading.Event | None = None) -> bool:
    gw._server_error = None
    if gw._app is None:
        gw._server_error = "RuntimeError: FastAPI app is not configured"
        logger.error("uvicorn server cannot start: FastAPI app is not configured")
        return False
    stop_event = stop_event or gw._stop_event
    old_interval = sys.getswitchinterval()
    sys.setswitchinterval(0.001)
    try:
        try:
            import uvicorn
        except ImportError:
            gw._server_error = "ImportError: uvicorn is not installed"
            logger.error("uvicorn not installed -run: pip install 'uvicorn[standard]'")
            return False
        server = None
        try:
            config = uvicorn.Config(
                gw._app,
                host=gw._host,
                port=gw._port,
                log_level="warning",
                loop="auto",
                ws="auto",
                lifespan="off",
                timeout_keep_alive=30,
                ws_max_size=2 * 1024 * 1024,
            )
            server = uvicorn.Server(config)
            gw._server = server
            if stop_event.is_set():
                server.should_exit = True
            logger.info("uvicorn server.run() starting on %s:%d", gw._host, gw._port)
            server.run()
            if bool(getattr(server, "should_exit", False)) or bool(getattr(server, "force_exit", False)):
                logger.info("uvicorn server stopped cleanly")
                return True
            gw._server_error = (
                "RuntimeError: uvicorn server.run() returned without a shutdown signal"
            )
            logger.error("uvicorn server.run() returned without a shutdown signal")
            return False
        except (Exception, SystemExit) as exc:
            gw._server_error = f"{type(exc).__name__}: {exc}"
            logger.exception("uvicorn crashed")
            return False
        finally:
            if server is not None and gw._server is server:
                gw._server = None
    finally:
        sys.setswitchinterval(old_interval)
