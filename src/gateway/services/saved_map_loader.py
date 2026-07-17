"""Saved-map point loader loop for GatewayModule."""

from __future__ import annotations

import logging
import threading
from typing import Any

from runtime.msgs.numpy_compat import np

logger = logging.getLogger(__name__)


def load_saved_maps_loop(
    gw: Any,
    stop_event: threading.Event | None = None,
) -> None:
    """Push active saved map points once per active-map change."""
    max_send = 80_000
    last_target = None
    stop_event = stop_event or gw._stop_event
    while not stop_event.is_set():
        try:
            target = gw._active_map_from_maps_service()
            if target != last_target:
                last_target = target
                pts = (
                    gw._saved_map_points_from_maps_service(
                        target,
                        max_points=max_send,
                    )
                    if target
                    else None
                )
                if pts is not None and len(pts) > 0:
                    if len(pts) > max_send:
                        idx = np.random.choice(len(pts), max_send, replace=False)
                        pts = pts[idx]
                    flat = pts[:, :3].astype(np.float32).flatten().tolist()
                    count = len(pts)
                    gw._last_saved_map_event = {
                        "type": "saved_map",
                        "points": flat,
                        "count": count,
                        "source": "maps_service",
                    }
                    gw.push_event(gw._last_saved_map_event)
                    logger.info(
                        "saved_map loader: loaded %s via %s (%d pts) -pushed once",
                        target,
                        "maps_service",
                        count,
                    )
        except Exception as exc:
            logger.debug("saved_map loader error: %s", exc)
        stop_event.wait(5.0)
