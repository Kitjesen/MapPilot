"""SLAM drift detection and operator handoff for GatewayModule."""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

logger = logging.getLogger(__name__)


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
    xy_bad = (
        x > gw._drift_watchdog_xy_limit
        or y > gw._drift_watchdog_xy_limit
        or z > gw._drift_watchdog_xy_limit
    )
    v_bad = v > gw._drift_watchdog_v_limit
    return invalid or xy_bad or v_bad, x, y, z, v, invalid


def current_odom_divergence(
    gw: Any,
) -> tuple[bool, float, float, float, float, bool]:
    with gw._state_lock:
        invalid_odom = (
            dict(gw._last_invalid_odometry)
            if gw._last_invalid_odometry
            else None
        )
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
    """Report divergence without taking ProductControl ownership."""

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
            diverged, x, y, z, v, _invalid = gw._drift_current_odom_divergence()
            if not diverged:
                continue
            now = time.time()
            since = now - gw._drift_last_report_ts
            if since < gw._drift_watchdog_cooldown:
                logger.warning(
                    "drift_watchdog: divergence persists (xy=%.0f,%.0f v=%.1f); "
                    "incident report cooldown %.0fs",
                    x,
                    y,
                    v,
                    gw._drift_watchdog_cooldown - since,
                )
                continue
            logger.error(
                "drift_watchdog: IEKF diverged xy=(%.0f,%.0f) z=%.0f v=%.1f; "
                "ProductControl restart required",
                x,
                y,
                z,
                v,
            )
            if stop_event.is_set():
                return
            reported = gw._drift_report_divergence(
                xy=max(x, y, z),
                y_abs=y,
                v=v,
                stop_event=stop_event,
            )
            if reported:
                gw._drift_last_report_ts = time.time()
                gw._drift_incident_count += 1
        except Exception as exc:
            logger.exception("drift_watchdog tick failed: %s", exc)


def report_drift(
    gw: Any,
    *,
    xy: float,
    y_abs: float,
    v: float,
    stop_event: threading.Event | None = None,
) -> bool:
    """Quarantine stale localization state and publish an operator incident."""

    stop_event = stop_event or gw._stop_event
    if stop_event.is_set():
        return False

    mode = gw._session_mode
    incident = gw._drift_incident_count + 1
    dump_path = None
    try:
        dump_path = gw._blackbox.dump(
            reason="drift_watchdog",
            metadata={
                "xy": float(xy),
                "y_abs": float(y_abs),
                "v": float(v),
                "session_mode": mode,
                "incident_count": incident,
            },
        )
    except Exception as exc:
        logger.warning("drift_watchdog: blackbox dump failed (continuing): %s", exc)

    gw.clear_localization_runtime_cache(reason="drift_watchdog")

    plan = getattr(gw, "_compiled_run_plan", None)
    event: dict[str, Any] = {
        "type": "slam_drift",
        "level": "error",
        "xy": max(xy, y_abs),
        "v": v,
        "action": (
            "operator_restart_required"
            if plan is not None
            else "restart_unavailable"
        ),
        "reason": (
            "operator_product_control_required"
            if plan is not None
            else "run_plan_missing"
        ),
        "count": incident,
    }
    if plan is not None:
        env = str(getattr(plan, "env", "") or "").strip()
        command = "python -m lingtu.control restart --process slam"
        if env in {"real", "sim"}:
            command += f" --env {env}"
        event["operator_command"] = command
    if dump_path is not None:
        event["dump_path"] = str(dump_path)
    gw.push_event(event)
    logger.error(
        "drift_watchdog: localization quarantined; action=%s reason=%s",
        event["action"],
        event["reason"],
    )
    return True
