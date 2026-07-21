"""SLAM drift recovery helpers for GatewayModule."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)

_RESTART_SERVICE_NAMES = (
    "slam",
    "slam_pgo",
    "localizer",
    "genz_icp",
    "hba",
    "super_lio",
    "super_lio_relocation",
)


def odom_diverged(
    gw: Any,
    odom: dict[str, Any],
) -> tuple[bool, float, float, float, float, bool]:
    def abs_finite(key: str) -> tuple[float, bool]:
        try:
            value = float(odom.get(key, 0.0))
        except (TypeError, ValueError):
            return float("inf"), False
        if not math.isfinite(value):
            return float("inf"), False
        return abs(value), True

    x, x_ok = abs_finite("x")
    y, y_ok = abs_finite("y")
    z, z_ok = abs_finite("z")
    vx, vx_ok = abs_finite("vx")
    vy, vy_ok = abs_finite("vy")
    vz, vz_ok = abs_finite("vz")
    v = max(vx, vy, vz)
    invalid = not all((x_ok, y_ok, z_ok, vx_ok, vy_ok, vz_ok))
    xy_bad = x > gw._drift_watchdog_xy_limit or y > gw._drift_watchdog_xy_limit or z > gw._drift_watchdog_xy_limit
    v_bad = v > gw._drift_watchdog_v_limit
    return invalid or xy_bad or v_bad, x, y, z, v, invalid


def current_odom_divergence(
    gw: Any,
) -> tuple[bool, float, float, float, float, bool]:
    with gw._state_lock:
        invalid_odom = dict(gw._last_invalid_odometry) if gw._last_invalid_odometry else None
        odom = dict(gw._odom) if gw._odom else {}
    if invalid_odom:
        return True, float("inf"), 0.0, 0.0, float("inf"), True
    if not odom:
        return False, 0.0, 0.0, 0.0, 0.0, False
    return odom_diverged(gw, odom)


def watchdog_loop(
    gw: Any,
    stop_event: threading.Event | None = None,
) -> None:
    """Periodic sanity check on odom; restart SLAM services if IEKF diverged."""
    xy_lim = gw._drift_watchdog_xy_limit
    v_lim = gw._drift_watchdog_v_limit
    interval = gw._drift_watchdog_interval
    logger.info(
        "drift_watchdog: enabled, interval=%.0fs, |xy|<%.0fm, |v|<%.1fm/s",
        interval,
        xy_lim,
        v_lim,
    )
    stop_event = stop_event or gw._stop_event
    while not stop_event.wait(interval):
        try:
            diverged, x, y, z, v, invalid = gw._drift_current_odom_divergence()
            if not diverged:
                continue
            now = time.time()
            since = now - gw._drift_last_restart_ts
            if since < gw._drift_watchdog_cooldown:
                logger.warning(
                    "drift_watchdog: still diverged (xy=%.0f,%.0f v=%.1f) but "
                    "cooldown (%.0fs) not elapsed -skipping restart",
                    x,
                    y,
                    v,
                    gw._drift_watchdog_cooldown - since,
                )
                continue
            logger.error(
                "drift_watchdog: IEKF DIVERGED xy=(%.0f,%.0f) z=%.0f v=%.1f -"
                "restarting SLAM services + session companions",
                x,
                y,
                z,
                v,
            )
            if stop_event.is_set():
                return
            gw._drift_restart_do_restart(
                xy=max(x, y, z),
                y_abs=y,
                v=v,
                stop_event=stop_event,
            )
            gw._drift_last_restart_ts = time.time()
            gw._drift_restart_count += 1
        except Exception as exc:
            logger.exception("drift_watchdog tick failed: %s", exc)


def restart_after_drift(
    gw: Any,
    *,
    xy: float,
    y_abs: float,
    v: float,
    stop_event: threading.Event | None = None,
) -> None:
    """Restart SLAM services after odometry divergence."""
    stop_event = stop_event or gw._stop_event
    if stop_event.is_set():
        return
    try:
        from lingtu.control import ProductControl

        control = ProductControl()
    except Exception as exc:
        logger.error("drift_watchdog: product control unavailable: %s", exc)
        return

    mode = gw._session_mode
    running_before = (
        control.legacy_states(tuple(_RESTART_SERVICE_NAMES))
        if gw._manage_session_services
        else {}
    )

    dump_path = None
    try:
        dump_path = gw._blackbox.dump(
            reason="drift_watchdog",
            metadata={
                "xy": float(xy),
                "y_abs": float(y_abs),
                "v": float(v),
                "session_mode": mode,
                "restart_count": gw._drift_restart_count + 1,
            },
        )
    except Exception as exc:
        logger.warning("drift_watchdog: blackbox dump failed (continuing): %s", exc)

    if gw._manage_session_services:
        try:
            control.legacy_stop(tuple(_RESTART_SERVICE_NAMES))
        except Exception as exc:
            logger.warning("drift_watchdog: compatibility stop failed (continuing): %s", exc)

    gw.clear_localization_runtime_cache(reason="drift_watchdog")

    event: dict[str, Any] = {
        "type": "slam_drift",
        "level": "error",
        "xy": max(xy, y_abs),
        "v": v,
        "action": "slam_restart",
        "count": gw._drift_restart_count + 1,
    }
    if dump_path is not None:
        event["dump_path"] = str(dump_path)
    gw.push_event(event)

    if not gw._manage_session_services:
        try:
            report = control.restart("slam")
            logger.info(
                "drift_watchdog: RuntimePlan restart complete (ready=%s)",
                bool(report.ok),
            )
        except Exception as exc:
            logger.error("drift_watchdog: RuntimePlan restart failed: %s", exc)
        return

    if stop_event.wait(gw._drift_restart_delay_s):
        return
    try:
        restart_services = _restart_services_for_mode(
            mode,
            running_before=running_before,
        )
        if restart_services and not stop_event.is_set():
            control.legacy_ensure(tuple(restart_services), timeout_s=10.0)
        logger.info(
            "drift_watchdog: restart complete (mode=%s, restored=%s)",
            mode,
            ",".join(restart_services) if restart_services else "none",
        )
    except Exception as exc:
        logger.error("drift_watchdog: ensure failed: %s", exc)


def _restart_services_for_mode(
    mode: str,
    *,
    running_before: dict[str, bool],
) -> list[str]:
    if running_before.get("super_lio_relocation"):
        return ["lidar", "super_lio_relocation"]
    if running_before.get("super_lio"):
        return ["lidar", "super_lio"]
    if running_before.get("genz_icp"):
        return ["lidar", "genz_icp"]
    if mode in ("mapping", "exploring", "navigating"):
        return ["slam"]
    services = [name for name in _RESTART_SERVICE_NAMES if running_before.get(name) and name != "localizer"]
    if ("localizer" in services or "slam_pgo" in services) and "slam" not in services:
        services.insert(0, "slam")
    return services
