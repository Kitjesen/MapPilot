"""System service stack helpers for full-stack composition."""

from __future__ import annotations

import logging
from typing import Any

from runtime.blueprint import Blueprint
from runtime.contracts import (
    GNSS_BACKEND_DDS,
    GNSS_BACKEND_HW,
    GNSS_BACKEND_REPLAY,
    GNSS_BACKEND_WTRTK980,
    GNSS_BACKENDS,
    GNSS_CONFIG_BACKEND,
    GNSS_ROLE,
    HW_CONFIG_BRIDGE,
    HW_ROLE,
)

logger = logging.getLogger(__name__)


def run_startup_preflight(
    *,
    enable_semantic: bool,
    slam_profile: str,
) -> None:
    from runtime.utils.calibration_check import run_calibration_check

    needs_camera = enable_semantic or slam_profile not in ("", "none")
    needs_slam = slam_profile not in ("", "none")
    calib = run_calibration_check(
        require_camera=needs_camera,
        require_slam=needs_slam,
    )
    if not calib.ok:
        raise RuntimeError(f"Calibration self-check failed ({len(calib.errors)} error(s)): " + "; ".join(calib.errors))


def hw() -> Blueprint:
    device_bp = Blueprint()
    try:
        import os
        from pathlib import Path

        devices_yaml = Path(__file__).resolve().parents[4] / "config" / "devices.yaml"
        if devices_yaml.exists():
            from runtime.devices import Hw

            device_bp.add(
                Hw,
                alias=HW_ROLE,
                config_path=str(devices_yaml),
                enable_hotplug=os.environ.get("LINGTU_HOTPLUG", "0") == "1",
            )
    except Exception as exc:
        logger.debug("hw not loaded: %s", exc)
    return device_bp


def gnss(*, enabled: bool | None = None, backend: str | None = None) -> Blueprint:
    gnss_bp = Blueprint()
    if enabled is False:
        return gnss_bp
    try:
        from runtime.config import get_config

        gnss_cfg = get_config().raw.get("gnss", {})
        if enabled is True or gnss_cfg.get("enabled", False):
            from localization.gnss_module import GnssModule

            serial_port = gnss_cfg.get("device") or gnss_cfg.get("serial_port")
            requested_backend = backend or gnss_cfg.get(GNSS_CONFIG_BACKEND) or gnss_cfg.get("backend")
            requested_backend = _normalize_gnss_backend(requested_backend)
            use_hw_bridge = _optional_bool(gnss_cfg.get(HW_CONFIG_BRIDGE))
            if use_hw_bridge is None:
                if requested_backend == GNSS_BACKEND_HW:
                    use_hw_bridge = True
                elif requested_backend in {
                    GNSS_BACKEND_WTRTK980,
                    GNSS_BACKEND_DDS,
                    GNSS_BACKEND_REPLAY,
                }:
                    use_hw_bridge = False
                else:
                    use_hw_bridge = not bool(serial_port)
            source_backend = requested_backend or (GNSS_BACKEND_HW if use_hw_bridge else GNSS_BACKEND_WTRTK980)
            direct_serial = not use_hw_bridge and source_backend == GNSS_BACKEND_WTRTK980 and bool(serial_port)
            rtcm_cfg = gnss_cfg.get("rtcm") or {}

            if direct_serial:
                logger.info(
                    "gnss/wtrtk980 is owned by lingtu-gnss-dds.service; skipping Python GnssModule serial ownership"
                )
                if rtcm_cfg.get("enabled", False):
                    logger.warning(
                        "gnss.rtcm is configured but Python NTRIP is not started for the native C++ GNSS service path"
                    )
                return gnss_bp

            gnss_bp.add(
                GnssModule,
                alias=GNSS_ROLE,
                device_model=gnss_cfg.get("model", "WTRTK-980"),
                fix_topic=gnss_cfg.get("topic_fix", "/gps/fix"),
                serial_port=serial_port if direct_serial else None,
                serial_baud=int(gnss_cfg.get("baud", 115200)),
                source_backend=source_backend,
                origin_lat=(gnss_cfg.get("origin") or {}).get("lat"),
                origin_lon=(gnss_cfg.get("origin") or {}).get("lon"),
                origin_alt=(gnss_cfg.get("origin") or {}).get("alt"),
                auto_init_origin=(gnss_cfg.get("origin") or {}).get("auto_init", True),
                min_sat_used=(gnss_cfg.get("quality") or {}).get("min_sat_used", 8),
                max_hdop=(gnss_cfg.get("quality") or {}).get("max_hdop", 2.5),
            )
            if use_hw_bridge:
                from localization.gnss_bridge import GnssBridgeModule

                gnss_bp.add(
                    GnssBridgeModule,
                    device_id=gnss_cfg.get("device_id", "wtrtk980_main"),
                    gnss_module_name=GNSS_ROLE,
                )
            if rtcm_cfg.get("enabled", False):
                from localization.ntrip_client_module import NtripClientModule

                gnss_bp.add(
                    NtripClientModule,
                    enabled=True,
                    host=rtcm_cfg.get("ntrip_host", ""),
                    port=int(rtcm_cfg.get("ntrip_port", 2101)),
                    mount=rtcm_cfg.get("ntrip_mount", ""),
                    user=rtcm_cfg.get("ntrip_user", ""),
                    password=rtcm_cfg.get("ntrip_pass", ""),
                )
    except ValueError:
        raise
    except Exception as exc:
        logger.debug("GNSS not loaded: %s", exc)
    return gnss_bp


def _optional_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    return bool(value)


def _normalize_gnss_backend(value: Any) -> str | None:
    if value is None:
        return None
    backend = str(value).strip().lower()
    if not backend:
        return None
    if backend not in GNSS_BACKENDS:
        raise ValueError(f"Unsupported gnss backend: {value!r}; expected one of {GNSS_BACKENDS}")
    return backend
