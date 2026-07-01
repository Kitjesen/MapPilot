"""Lite-safe process lifecycle helpers for the CLI."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path

from . import term as T
from runtime.profiles.binding_policy import (
    LEGACY_NAV_IN_ENABLE_KEYS,
    LEGACY_NAV_OUT_ENABLE_KEYS,
    LEGACY_MAP_OUT_ENABLE_KEYS,
    MAP_OUT_ENABLE_KEYS,
    NAV_IN_ENABLE_KEYS,
    NAV_OUT_ENABLE_KEYS,
    nav_kernel_backend_required,
)

FULL_PREFLIGHT_SLAM_PROFILES = frozenset(
    {
        "fastlio2",
        "pointlio",
        "localizer",
        "super_lio",
        "super_lio_relocation",
    }
)

LITE_PROFILE_NAMES = frozenset({"lite", "thunder-lite", "thunder-basic"})
LITE_RUNTIME_ENDPOINTS = frozenset({"thunder_lite"})

_LITE_FALSE_FLAGS = (
    "enable_native",
    "enable_gateway",
    "enable_semantic",
    "enable_teleop",
    "enable_map_modules",
    "enable_rerun",
    "manage_external_services",
    "run_startup_checks",
    "enable_gnss",
    *NAV_IN_ENABLE_KEYS,
    *NAV_OUT_ENABLE_KEYS,
    *LEGACY_NAV_IN_ENABLE_KEYS,
    *LEGACY_NAV_OUT_ENABLE_KEYS,
    *MAP_OUT_ENABLE_KEYS,
    *LEGACY_MAP_OUT_ENABLE_KEYS,
    "enable_ros2_camera_bridge",
    "enable_ros2_rerun_bridge",
)


def _as_bool(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _normalized(value: object, default: str = "") -> str:
    if value is None:
        return default
    return str(value).strip().lower()


def _normalized_endpoint(value: object) -> str:
    return _normalized(value).replace("-", "_")


def needs_full_preflight(cfg: Mapping[str, object]) -> bool:
    """Return true when a config needs the full runtime preflight package."""

    slam_profile = _normalized(cfg.get("slam_profile"), "none")
    return (
        slam_profile in FULL_PREFLIGHT_SLAM_PROFILES
        or _as_bool(cfg.get("enable_native", False))
        or _as_bool(cfg.get("enable_gateway", False))
        or nav_kernel_backend_required(
            cfg,
            enable_native=_as_bool(cfg.get("enable_native", True)),
        )
    )


def is_lite_runtime_config(profile_name: str, cfg: Mapping[str, object]) -> bool:
    profile = _normalized(profile_name)
    runtime_mode = _normalized(cfg.get("runtime_mode"))
    endpoint = _normalized_endpoint(
        cfg.get("_runtime_endpoint", cfg.get("runtime_endpoint"))
    )
    return (
        profile in LITE_PROFILE_NAMES
        or runtime_mode == "lite"
        or endpoint in LITE_RUNTIME_ENDPOINTS
    )


def lite_runtime_lifecycle_blockers(
    profile_name: str,
    cfg: Mapping[str, object],
) -> tuple[str, ...]:
    if not is_lite_runtime_config(profile_name, cfg):
        return ()

    blockers: list[str] = []
    endpoint = _normalized_endpoint(
        cfg.get("_runtime_endpoint", cfg.get("runtime_endpoint"))
    )
    if endpoint and endpoint not in LITE_RUNTIME_ENDPOINTS:
        blockers.append("runtime_endpoint must be thunder_lite")

    module_transport = _normalized(cfg.get("module_transport"), "local")
    if module_transport not in {"", "local"}:
        blockers.append("module_transport must be local")

    endpoint_transport = _normalized(cfg.get("endpoint_transport"), "local")
    if endpoint_transport not in {"", "local"}:
        blockers.append("endpoint_transport must be local")

    slam_profile = _normalized(cfg.get("slam_profile"), "none")
    if slam_profile not in {"", "none"}:
        blockers.append("slam_profile must be none")

    for flag in _LITE_FALSE_FLAGS:
        if _as_bool(cfg.get(flag, False)):
            blockers.append(f"{flag} must be false")

    exploration_backend = _normalized(cfg.get("exploration_backend"), "none")
    if exploration_backend not in {"", "none"}:
        blockers.append("exploration_backend must be none")

    if cfg.get("scene_xml"):
        blockers.append("scene_xml is not supported by Lite runtime")

    return tuple(blockers)


def kill_residual_ports(cfg: dict) -> None:
    import platform

    if platform.system() != "Linux":
        return
    ports = [cfg.get("gateway_port", 5050), 8090]
    for port in ports:
        try:
            result = subprocess.run(
                ["fuser", "-k", "%d/tcp" % port],
                capture_output=True,
                timeout=3,
            )
            if result.returncode == 0:
                logging.getLogger(__name__).info("Killed residual process on port %d", port)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass


def health_check(system) -> bool:
    ok = True
    for name, mod in system.modules.items():
        if mod is None:
            logging.getLogger("lingtu").error("Health check: module %s is None", name)
            ok = False
            continue
        if not hasattr(mod, "ports_in") or not hasattr(mod, "ports_out"):
            logging.getLogger("lingtu").error("Health check: %s missing ports", name)
            ok = False
    return ok


def daemonize(log_file: str | Path) -> bool:
    """Unix: fork and detach. Windows: warn and stay foreground."""

    if os.name == "nt":
        print(f"  {T.yellow('Daemon mode not supported on Windows. Running in foreground.')}")
        return False

    pid = os.fork()
    if pid > 0:
        print(f"  Daemon started (PID {pid})")
        print(f"  Logs: {log_file}")
        print("  Stop: lingtu stop")
        sys.exit(0)

    os.setsid()

    pid = os.fork()
    if pid > 0:
        sys.exit(0)

    sys.stdout.flush()
    sys.stderr.flush()
    devnull = open(os.devnull)
    os.dup2(devnull.fileno(), sys.stdin.fileno())
    log_f = open(log_file, "a")
    os.dup2(log_f.fileno(), sys.stdout.fileno())
    os.dup2(log_f.fileno(), sys.stderr.fileno())

    return True
