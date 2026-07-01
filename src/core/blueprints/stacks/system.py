"""System service stack helpers for full-stack composition."""

from __future__ import annotations

import logging
from typing import Any

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def run_startup_preflight(
    *,
    enable_semantic: bool,
    slam_profile: str,
) -> None:
    from core.utils.calibration_check import run_calibration_check

    needs_camera = enable_semantic or slam_profile not in ("", "none")
    needs_slam = slam_profile not in ("", "none")
    calib = run_calibration_check(
        require_camera=needs_camera,
        require_slam=needs_slam,
    )
    if not calib.ok:
        raise RuntimeError(
            f"Calibration self-check failed ({len(calib.errors)} error(s)): "
            + "; ".join(calib.errors)
        )


def device_manager() -> Blueprint:
    device_bp = Blueprint()
    try:
        import os
        from pathlib import Path

        devices_yaml = Path(__file__).resolve().parents[4] / "config" / "devices.yaml"
        if devices_yaml.exists():
            from core.devices import DeviceManager

            device_bp.add(
                DeviceManager,
                config_path=str(devices_yaml),
                enable_hotplug=os.environ.get("LINGTU_HOTPLUG", "0") == "1",
            )
    except Exception as exc:
        logger.debug("DeviceManager not loaded: %s", exc)
    return device_bp


def gnss(*, enabled: bool | None = None) -> Blueprint:
    gnss_bp = Blueprint()
    if enabled is False:
        return gnss_bp
    try:
        from core.config import get_config

        gnss_cfg = get_config().raw.get("gnss", {})
        if gnss_cfg.get("enabled", False):
            from slam.gnss_bridge import GnssBridgeModule
            from slam.gnss_module import GnssModule

            gnss_bp.add(
                GnssModule,
                device_model=gnss_cfg.get("model", "WTRTK-980"),
                origin_lat=(gnss_cfg.get("origin") or {}).get("lat"),
                origin_lon=(gnss_cfg.get("origin") or {}).get("lon"),
                origin_alt=(gnss_cfg.get("origin") or {}).get("alt"),
                auto_init_origin=(gnss_cfg.get("origin") or {}).get("auto_init", True),
                min_sat_used=(gnss_cfg.get("quality") or {}).get("min_sat_used", 8),
                max_hdop=(gnss_cfg.get("quality") or {}).get("max_hdop", 2.5),
            )
            gnss_bp.add(
                GnssBridgeModule,
                device_id=gnss_cfg.get("device_id", "wtrtk980_main"),
            )
            rtcm_cfg = gnss_cfg.get("rtcm") or {}
            if rtcm_cfg.get("enabled", False):
                from slam.ntrip_client_module import NtripClientModule

                gnss_bp.add(
                    NtripClientModule,
                    enabled=True,
                    host=rtcm_cfg.get("ntrip_host", ""),
                    port=int(rtcm_cfg.get("ntrip_port", 2101)),
                    mount=rtcm_cfg.get("ntrip_mount", ""),
                    user=rtcm_cfg.get("ntrip_user", ""),
                    password=rtcm_cfg.get("ntrip_pass", ""),
                )
    except Exception as exc:
        logger.debug("GNSS not loaded: %s", exc)
    return gnss_bp


def external_services(
    *,
    enabled: bool,
    driver_module: str,
    slam_profile: str,
    enable_semantic: bool,
    config: dict[str, Any],
) -> Blueprint:
    """Return a runtime service plan without touching systemd at Blueprint build time."""

    service_bp = Blueprint()
    if not enabled or driver_module != "ThunderDriver":
        return service_bp

    from core.runtime_policy import normalize_slam_profile, slam_switch_plan

    profile = normalize_slam_profile(slam_profile)
    if profile in {"", "none", "bridge"}:
        stop_services: list[str] = []
        ensure_services: list[str] = []
        wait_ready_services: list[str] = []
    else:
        plan = slam_switch_plan(profile)
        stop_services = list(plan.stop)
        ensure_services = list(plan.ensure)
        wait_ready_services = list(plan.wait_ready)

    needs_external_camera = enable_semantic and not bool(config.get("use_driver_camera", False))
    if needs_external_camera:
        ensure_services.append("camera")

    if not (stop_services or ensure_services or wait_ready_services):
        return service_bp

    try:
        from core.external_service_module import ExternalServiceManagerModule

        service_bp.add(
            ExternalServiceManagerModule,
            alias="ExternalServiceManagerModule",
            stop_services=tuple(stop_services),
            ensure_services=tuple(dict.fromkeys(ensure_services)),
            wait_ready_services=tuple(dict.fromkeys(wait_ready_services)),
            wait_ready_timeout=float(config.get("external_service_wait_ready_timeout", 10.0)),
            stop_on_shutdown=bool(config.get("external_service_stop_on_shutdown", False)),
        )
    except Exception as exc:
        logger.debug("ExternalServiceManagerModule not loaded: %s", exc)
    return service_bp
