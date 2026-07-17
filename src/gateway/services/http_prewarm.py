"""HTTP route prewarm helpers for GatewayModule."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)


def start_client_http_prewarm(
    gw: Any,
    stop_event: threading.Event | None = None,
    *,
    timeout_s: float = 15.0,
) -> bool:
    stop_event = stop_event or gw._stop_event
    if stop_event.is_set():
        return False
    if gw._client_http_prewarm_thread is not None and gw._client_http_prewarm_thread.is_alive():
        return False
    gw._client_http_prewarm_thread = threading.Thread(
        target=gw._prewarm_client_http_routes,
        args=(stop_event,),
        kwargs={"timeout_s": timeout_s},
        daemon=True,
        name="gateway_client_prewarm",
    )
    gw._client_http_prewarm_thread.start()
    return True


def prewarm_client_http_routes(
    gw: Any,
    stop_event: threading.Event | None = None,
    *,
    timeout_s: float = 4.0,
) -> bool:
    """Consume first-use FastAPI/Pydantic cost before App/Web clients arrive."""
    stop_event = stop_event or gw._stop_event
    if stop_event.is_set():
        return False
    try:
        from urllib.error import URLError
        from urllib.request import Request, urlopen
    except Exception as exc:
        logger.debug("GatewayModule: urllib import in prewarm failed: %s", exc)
        return False

    headers: dict[str, str] = {}
    try:
        from gateway.auth import _get_configured_key

        api_key = _get_configured_key()
        if api_key:
            headers["X-API-Key"] = api_key
    except Exception as exc:
        logger.debug("GatewayModule: failed to load API key for prewarm: %s", exc)

    url = f"http://127.0.0.1:{gw._port}/api/v1/app/capabilities"
    deadline = time.time() + max(0.0, float(timeout_s))
    while time.time() < deadline and not stop_event.is_set():
        try:
            request_timeout = min(2.5, max(0.1, deadline - time.time()))
            with urlopen(
                Request(url, headers=headers),
                timeout=request_timeout,
            ) as response:
                response.read()
            logger.info("GatewayModule: prewarmed App/Web capabilities route")
            return True
        except (OSError, URLError):
            stop_event.wait(0.1)
        except Exception:
            logger.debug(
                "GatewayModule: App/Web HTTP prewarm failed",
                exc_info=True,
            )
            return False
    return False
