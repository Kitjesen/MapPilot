"""Health, metrics, liveness, and readiness routes."""

from __future__ import annotations

import asyncio
import math
import os
import threading
import time
from collections.abc import Mapping
from typing import Annotated, Any

from fastapi import Query
from fastapi.responses import JSONResponse

from gateway.schemas import HealthResponse, LivenessResponse, ReadinessResponse
from gateway.services.media_status import build_camera_status
from gateway.services.native_status import read_json_snapshot, read_navigation_status
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.traffic import snapshot as traffic_snapshot
from runtime.contracts import CAMERA_ROLE, LIDAR_ROLE


def _probe_brainstem() -> dict[str, Any]:
    """Project the authoritative native driver snapshot into health telemetry."""

    path = os.environ.get("LINGTU_DRIVER_STATUS_FILE", "").strip() or (
        "/dev/shm/lingtu/driver_status.json"
    )
    payload = read_json_snapshot(path)
    if payload is None:
        return {
            "status": "unavailable",
            "source": "lingtu-driver-status",
            "reason": "driver_status_missing",
            "driver_status_file": path,
        }

    adapter = payload.get("adapter")
    adapter = dict(adapter) if isinstance(adapter, Mapping) else {}
    control = payload.get("control")
    control = dict(control) if isinstance(control, Mapping) else {}
    output_ack = payload.get("output_ack")
    output_ack = dict(output_ack) if isinstance(output_ack, Mapping) else {}
    try:
        stamp_s = float(payload.get("stamp_s"))
        max_age_s = float(os.environ.get("LINGTU_DRIVER_STATUS_MAX_AGE_S", "1.5") or "1.5")
        age_s = time.time() - stamp_s
    except (TypeError, ValueError):
        age_s = math.inf
        max_age_s = 1.5
    stale = not math.isfinite(age_s) or age_s < -0.05 or age_s > max_age_s
    connected = payload.get("connected") is True
    status = "stale" if stale else ("connected" if connected else "unreachable")

    info = dict(control)
    info.update(
        {
            "status": status,
            "source": "lingtu-driver-status",
            "driver_status_file": path,
            "host": str(adapter.get("target") or ""),
            "protocol": adapter.get("protocol"),
            "owner": adapter.get("control_owner"),
            "owner_id": adapter.get("control_owner_id"),
            "connected": connected,
            "ready": payload.get("ready") is True,
            "output_ack": output_ack,
            "last_reason": payload.get("last_reason"),
            "last_error": payload.get("last_error"),
            "status_age_s": round(age_s, 3) if math.isfinite(age_s) else None,
            "stale": stale,
        }
    )
    return info


_BRAINSTEM_TRANSIENT_FIELDS = {
    "cached",
    "cache_age_s",
    "stale",
    "refreshing",
}


def _cacheable_brainstem_info(info: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in dict(info).items() if key not in _BRAINSTEM_TRANSIENT_FIELDS}


def _probe_brainstem_safely() -> dict[str, Any]:
    try:
        return _probe_brainstem()
    except Exception as e:
        return {
            "status": "unreachable",
            "source": "lingtu-driver-status",
            "reason": "driver_status_probe_failed",
            "error": str(e)[:120],
        }


def _store_brainstem_health(gw, info: dict[str, Any]) -> None:
    lock = getattr(gw, "_brainstem_health_lock", None)
    if lock is None:
        return
    with lock:
        gw._brainstem_health_cache = _cacheable_brainstem_info(info)
        gw._brainstem_health_cache_ts = time.monotonic()


def _start_brainstem_refresh(gw) -> bool:
    lock = getattr(gw, "_brainstem_health_lock", None)
    if lock is None:
        return False
    with lock:
        if getattr(gw, "_brainstem_health_refreshing", False):
            return False
        gw._brainstem_health_refreshing = True

    def _refresh() -> None:
        try:
            _store_brainstem_health(gw, _probe_brainstem_safely())
        finally:
            with lock:
                gw._brainstem_health_refreshing = False

    try:
        thread = threading.Thread(
            target=_refresh,
            daemon=True,
            name="brainstem_health_refresh",
        )
        with lock:
            gw._brainstem_health_refresh_thread = thread
        thread.start()
        return True
    except Exception:
        with lock:
            gw._brainstem_health_refreshing = False
            gw._brainstem_health_refresh_thread = None
        return False


async def _brainstem_health(gw, *, force_live: bool = False) -> dict[str, Any]:
    now = time.monotonic()
    ttl = float(getattr(gw, "_brainstem_health_cache_ttl_s", 0.0) or 0.0)
    lock = getattr(gw, "_brainstem_health_lock", None)
    if ttl > 0.0 and lock is not None:
        cached = None
        age = 0.0
        refreshing = False
        with lock:
            cached = getattr(gw, "_brainstem_health_cache", None)
            cache_ts = float(getattr(gw, "_brainstem_health_cache_ts", 0.0) or 0.0)
            age = now - cache_ts
            refreshing = bool(getattr(gw, "_brainstem_health_refreshing", False))
        if cached is not None and age <= ttl and not force_live:
            info = dict(cached)
            info["cached"] = True
            info["cache_age_s"] = round(max(0.0, age), 3)
            return info
        if not force_live:
            scheduled = _start_brainstem_refresh(gw)
            if cached is not None:
                info = dict(cached)
                info["cached"] = True
                info["cache_age_s"] = round(max(0.0, age), 3)
                info["stale"] = True
                info["refreshing"] = bool(scheduled or refreshing)
                return info
            return {
                "status": "unknown",
                "source": "lingtu-driver-status",
                "reason": "driver_status_probe_pending",
                "driver_status_file": (
                    os.environ.get("LINGTU_DRIVER_STATUS_FILE", "").strip()
                    or "/dev/shm/lingtu/driver_status.json"
                ),
                "cached": False,
                "stale": True,
                "refreshing": bool(scheduled or refreshing),
            }

    try:
        loop = asyncio.get_running_loop()
        info = await loop.run_in_executor(None, _probe_brainstem_safely)
    except Exception as e:
        info = {
            "status": "unreachable",
            "source": "lingtu-driver-status",
            "reason": "driver_status_probe_failed",
            "error": str(e)[:120],
        }

    info = dict(info)
    info["cached"] = False
    if ttl > 0.0 and lock is not None:
        _store_brainstem_health(gw, info)
    return info


def _health_module_needs_detail(name: str) -> bool:
    lowered = name.lower()
    if lowered in {LIDAR_ROLE, CAMERA_ROLE}:
        return True
    return any(
        token in lowered
        for token in (
            "slambridge",
            "slamadapter",
            "slammodule",
            "navigation",
        )
    )


def _module_odometry_status(health: Mapping[str, Any]) -> dict[str, Any] | None:
    """Return live SLAM telemetry when a module publishes odometry."""
    ports_out = health.get("ports_out")
    if not isinstance(ports_out, Mapping):
        return None
    odometry = ports_out.get("odometry")
    if not isinstance(odometry, Mapping):
        return None
    rate_hz = round(_positive_float(odometry.get("rate_hz")), 1)
    return {
        "status": "active" if rate_hz > 0.0 else "inactive",
        "hz": rate_hz,
        "messages": odometry.get("msg_count", 0),
    }


def _positive_float(value: Any) -> float:
    try:
        num = float(value)
    except (TypeError, ValueError):
        return 0.0
    if num != num or num in (float("inf"), float("-inf")):
        return 0.0
    return num if num > 0.0 else 0.0


def _slam_rates(
    gw: Any,
    localization_status: Mapping[str, Any],
    observed_odom_hz: Any = 0.0,
) -> tuple[float, float, float]:
    processed_scan_hz = _positive_float(localization_status.get("processed_scan_hz"))
    odom_hz = _positive_float(observed_odom_hz)
    if odom_hz <= 0.0:
        odom_hz = _positive_float(gw._get_slam_hz_cached())
    return processed_scan_hz, odom_hz, processed_scan_hz or odom_hz


def register_health_routes(app, gw) -> None:
    @app.get(
        "/api/v1/health",
        summary="System health overview",
        response_model=HealthResponse,
    )
    async def get_health(
        details: Annotated[
            bool,
            Query(
                description="Probe every module health detail; default app polling path only probes displayed sensors.",
            ),
        ] = False,
    ):
        traffic = traffic_snapshot(gw)
        commands = gw._command_stats_snapshot()
        cloud_debug = gw._cloud_viewer.debug_snapshot()
        n_sse = traffic["sse"]["clients"]
        map_pts = gw._cloud_viewer.cache_point_count()

        sensors: dict[str, Any] = {}
        modules_ok = 0
        modules_fail = 0
        module_summary: dict[str, str] = {}

        def _probe_modules() -> tuple[dict[str, str], dict[str, Any], int, int]:
            """Probe module health in a thread to avoid blocking the event loop."""
            _summary: dict[str, str] = {}
            _sensors: dict[str, Any] = {}
            _ok = 0
            _fail = 0
            modules = getattr(gw, "_all_modules", None) or {}
            for name, mod in modules.items():
                probe_module = details or _health_module_needs_detail(str(name))
                if not probe_module:
                    _summary[name] = "ok"
                    _ok += 1
                    continue
                try:
                    h = mod.health() if hasattr(mod, "health") else {}
                    _summary[name] = "ok"
                    _ok += 1

                    name_l = str(name).lower()
                    if name_l == LIDAR_ROLE:
                        lidar_h = h.get("lidar", {})
                        _sensors["lidar"] = {
                            "status": lidar_h.get("state", "unknown"),
                            "ip": lidar_h.get("ip", "?"),
                            "cloud_hz": round(
                                h.get("ports_out", {}).get("scan", {}).get("rate_hz", 0),
                                1,
                            ),
                        }
                    elif name_l == CAMERA_ROLE:
                        _sensors["camera"] = build_camera_status(gw)
                    elif "slam" in name_l:
                        slam_status = _module_odometry_status(h)
                        if slam_status is not None:
                            _sensors["slam"] = slam_status
                except Exception:
                    _summary[name] = "error"
                    _fail += 1
            return _summary, _sensors, _ok, _fail

        # Run module probing in a thread to avoid blocking SSE/WS heartbeats
        modules = getattr(gw, "_all_modules", None) or {}
        if modules:
            module_summary, sensors, modules_ok, modules_fail = (
                await asyncio.get_running_loop().run_in_executor(None, _probe_modules)
            )

        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        processed_scan_hz, odom_hz, slam_hz = _slam_rates(
            gw,
            localization_status,
            sensors.get("slam", {}).get("hz"),
        )
        has_odom = gw._odom is not None
        if localization_status or has_odom or odom_hz > 0.0:
            slam_sensor = sensors.setdefault("slam", {})
            state = str(
                localization_status.get("state")
                or slam_sensor.get("status")
                or ("active" if odom_hz > 0.0 else "inactive")
            ).lower()
            slam_sensor.update(
                {
                    "status": state,
                    "hz": round(slam_hz, 1),
                    "processed_scan_hz": round(processed_scan_hz, 1),
                    "odom_hz": round(odom_hz, 1),
                    "source": (
                        "processed_scan_hz"
                        if processed_scan_hz > 0.0
                        else ("localization_status" if localization_status else "gateway_odom_window")
                    ),
                }
            )
            if not localization_status:
                slam_sensor.setdefault("reason", "localization_status_missing")
            for key in (
                "reason",
                "status_snapshot_stale",
                "status_snapshot_age_s",
                "lidar_input_hz",
                "imu_input_hz",
                "slam_tick_hz",
            ):
                if key in localization_status:
                    slam_sensor[key] = localization_status[key]

        brainstem_info = await _brainstem_health(gw, force_live=details)

        return {
            "status": "ok" if modules_fail == 0 else "degraded",
            "modules_ok": modules_ok,
            "modules_fail": modules_fail,
            "gateway": {
                "port": gw._port,
                "mode": gw._mode,
                "sse_clients": n_sse,
                "traffic": traffic,
                "cloud": cloud_debug,
                "commands": commands,
                "diagnostic_details": details,
            },
            "teleop": {
                "active": gw._teleop_active,
                "clients": gw._teleop_client_count(),
            },
            "sensors": sensors,
            "slam_hz": round(slam_hz, 1),
            "map_points": map_pts,
            "has_odom": has_odom,
            "modules": module_summary,
            "brainstem": brainstem_info,
        }

    @app.get(
        "/api/v1/metrics",
        summary="Operator-facing runtime metrics snapshot",
    )
    async def get_metrics():
        traffic = traffic_snapshot(gw)
        commands = gw._command_stats_snapshot()
        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        camera = build_camera_status(gw)
        map_points = gw._cloud_viewer.cache_point_count()
        nav_endpoint = read_navigation_status() or {}
        control_loop_health = nav_endpoint.get("control_loop_health")
        if not isinstance(control_loop_health, Mapping):
            control_loop_health = {}

        processed_scan_hz, odom_hz, slam_hz = _slam_rates(gw, localization_status)
        lidar_input_hz = _positive_float(localization_status.get("lidar_input_hz"))
        imu_input_hz = _positive_float(localization_status.get("imu_input_hz"))
        slam_tick_hz = _positive_float(localization_status.get("slam_tick_hz"))
        return {
            "schema_version": 1,
            "ok": True,
            "ts": time.time(),
            "gateway": {
                "port": gw._port,
                "mode": gw._mode,
                "sse_clients": traffic.get("sse", {}).get("clients", 0),
                "teleop_clients": gw._teleop_client_count(),
            },
            "slam": {
                "hz": round(slam_hz, 3),
                "processed_scan_hz": round(processed_scan_hz, 3),
                "lidar_input_hz": round(lidar_input_hz, 3),
                "imu_input_hz": round(imu_input_hz, 3),
                "slam_tick_hz": round(slam_tick_hz, 3),
                "odom_hz": round(odom_hz, 3),
                "state": str(localization_status.get("state") or "").lower(),
                "backend": localization_status.get("backend"),
                "mode": localization_status.get("mode"),
                "map_tf": localization_status.get("map_tf"),
            },
            "navigation": {
                "tick_hz": _positive_float(nav_endpoint.get("tick_hz")),
                "control_loop_health": dict(control_loop_health),
            },
            "map": {
                "points": map_points,
                "active": getattr(gw, "_active_map", None),
            },
            "camera": {
                "available": camera.get("available", False),
                "status": camera.get("status", "unknown"),
                "reason": camera.get("reason"),
                "fps": camera.get("fps", 0.0),
                "frames": camera.get("frames", 0),
                "backend": camera.get("backend"),
            },
            "traffic": traffic,
            "commands": commands,
            "websocket": (
                gw._ws_registry.snapshot()
                if hasattr(gw, "_ws_registry") and gw._ws_registry is not None
                else {}
            ),
        }

    @app.get(
        "/health",
        summary="Liveness probe",
        response_model=LivenessResponse,
    )
    async def liveness_health():
        traffic = traffic_snapshot(gw)
        cloud = gw._cloud_viewer.debug_snapshot()
        localization_status = getattr(gw, "_localization_status", None)
        localization_status = localization_status if isinstance(localization_status, Mapping) else {}
        processed_scan_hz, odom_hz, slam_hz = _slam_rates(gw, localization_status)
        return {
            "status": "ok",
            "ts": time.time(),
            "details_url": "/api/v1/health?details=true",
            "gateway": {
                "mode": gw._mode,
                "sse_clients": traffic.get("sse", {}).get("clients", 0),
                "cloud_clients": traffic.get("cloud", {}).get("clients", 0),
                "cloud": cloud,
            },
            "sensors": {
                "slam": {
                    "status": str(localization_status.get("state") or "").lower() or "unknown",
                    "hz": round(slam_hz, 1),
                    "processed_scan_hz": round(processed_scan_hz, 1),
                    "odom_hz": round(odom_hz, 1),
                    "lidar_input_hz": localization_status.get("lidar_input_hz"),
                }
            },
        }

    @app.get(
        "/ready",
        summary="Readiness probe",
        response_model=ReadinessResponse,
        responses={503: {"model": ReadinessResponse}},
    )
    async def readiness_ready(
        details: Annotated[
            bool,
            Query(
                description="Include per-module health details; default probe payload is summary-only.",
            ),
        ] = False,
    ):
        payload, status_code = build_readiness_snapshot(gw, include_details=details)
        return JSONResponse(payload, status_code=status_code)

    @app.get(
        "/api/v1/readiness",
        summary="Client readiness snapshot",
        response_model=ReadinessResponse,
    )
    async def api_readiness(
        details: Annotated[
            bool,
            Query(
                description="Include per-module health details for operator screens.",
            ),
        ] = False,
    ):
        payload, _status_code = build_readiness_snapshot(gw, include_details=details)
        return payload
