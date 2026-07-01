"""Unified configuration loader 鈥?dict-based API for robot config, config discovery, and validation.

This module complements the typed ``runtime.config`` module.  Where ``runtime.config``
provides dataclass-validated access to common sections (speed, geometry, driver,
safety, lidar, camera, gnss), this module provides:

* ``load_robot_config(path=None) -> dict`` 鈥?raw dict from ``robot_config.yaml``
  (useful for code that consumes arbitrary sections not modelled as dataclasses)
* ``get_config_paths() -> dict`` 鈥?discovers all known config files in the project
* ``validate_config(config: dict) -> list[str]`` 鈥?runs the same checks as
  ``runtime.config.validate_config`` but on a plain dict

Usage::

    from runtime.config_loader import load_robot_config, get_config_paths

    cfg = load_robot_config()
    lidar_extrinsics = cfg["lidar"]                 # raw dict access

    paths = get_config_paths()
    print(paths["robot_config"])                    # absolute path
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml

try:
    from runtime.config import load_config  # noqa: F401
    from runtime.config import validate_config as _validate_typed  # noqa: F401
    _HAS_TYPED_CONFIG = True
except ImportError:
    _HAS_TYPED_CONFIG = False

# 鈹€鈹€ Repo root discovery 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent


# 鈹€鈹€ Config path helpers 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€


def _resolve(rel: str) -> str:
    """Resolve a repo-relative path to an absolute path."""
    return str(_REPO_ROOT / rel)


# 鈹€鈹€ Public API 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€


def load_robot_config(path: str | None = None) -> dict[str, Any]:
    """Load ``robot_config.yaml`` as a plain dict.

    Resolution order for *path*:
    1. Explicit *path* argument.
    2. ``LINGTU_CONFIG_PATH`` environment variable.
    3. ``config/robot_config.yaml`` relative to repo root.

    Returns an empty dict if the file is missing or unreadable.
    """
    if path is None:
        path = os.environ.get("LINGTU_CONFIG_PATH") or _resolve("config/robot_config.yaml")

    try:
        with open(path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    except (FileNotFoundError, OSError):
        cfg = {}

    if not isinstance(cfg, dict):
        cfg = {}

    return cfg


def get_config_paths() -> dict[str, str]:
    """Discover all known LingTu config files.

    Returns a dict keyed by short name 鈫?absolute file path.  Every key maps
    to a file that exists on disk at the time of the call.  Files under
    ``src/`` subdirectories (SLAM-specific, per-algorithm configs) are
    intentionally excluded 鈥?this method covers the top-level ``config/``
    directory and a few well-known calibration files.
    """
    base = _REPO_ROOT / "config"
    slam_cfg = _REPO_ROOT / "src" / "slam" / "fastlio2" / "config" / "lio.yaml"
    pointlio_cfg = _REPO_ROOT / "config" / "pointlio.yaml"

    entries: dict[str, Path] = {
        # Top-level config/
        "robot_config":         base / "robot_config.yaml",
        "endpoints":            base / "endpoints.yaml",
        "topic_contract":       base / "topic_contract.yaml",
        "qos_profiles":         base / "qos_profiles.yaml",
        "devices":              base / "devices.yaml",
        "dufomap":              base / "dufomap.toml",
        "go2rtc":               base / "go2rtc.yaml",
        "pointlio":             pointlio_cfg,
        # Perception and decision configs
        "perception":           base / "perception.yaml",
        "decision":             base / "decision.yaml",
        "semantic_scoring":     base / "semantic_scoring.yaml",
        "semantic_exploration": base / "semantic_exploration.yaml",
        # Navigation configs
        "far_planner":          base / "far_planner.yaml",
        # SLAM
        "fastlio2":             slam_cfg,
    }

    result: dict[str, str] = {}
    for name, p in entries.items():
        if p.exists():
            result[name] = str(p)
    return result


def validate_config(config: dict[str, Any]) -> list[str]:
    """Validate a raw ``robot_config.yaml`` dict and return a list of issues.

    Checks mirror the typed ``runtime.config.validate_config()`` 鈥?safety-critical
    ranges for speed, geometry, safety, driver, camera, and GNSS.

    An empty list means the config is well-formed.
    """
    issues: list[str] = []

    # Speed
    speed = config.get("speed", {})
    if speed.get("max_linear", 1.0) <= 0:
        issues.append(f"speed.max_linear must be > 0, got {speed.get('max_linear')}")
    if speed.get("max_angular", 1.0) <= 0:
        issues.append(f"speed.max_angular must be > 0, got {speed.get('max_angular')}")

    # Geometry
    geom = config.get("geometry", {})
    for dim in ("vehicle_width", "vehicle_length", "vehicle_height"):
        val = geom.get(dim, 0.5)
        if val <= 0:
            issues.append(f"geometry.{dim} must be > 0, got {val}")

    # Safety
    safety = config.get("safety", {})
    stop_dist = safety.get("stop_distance", 0.8)
    slow_dist = safety.get("slow_distance", 2.0)
    if stop_dist <= 0:
        issues.append(f"safety.stop_distance must be > 0, got {stop_dist}")
    if slow_dist <= stop_dist:
        issues.append(f"safety.slow_distance ({slow_dist}) must be > stop_distance ({stop_dist})")
    if safety.get("deadman_timeout_ms", 300.0) <= 0:
        issues.append(f"safety.deadman_timeout_ms must be > 0, got {safety.get('deadman_timeout_ms')}")
    if safety.get("tilt_limit_deg", 30.0) <= 0:
        issues.append(f"safety.tilt_limit_deg must be > 0, got {safety.get('tilt_limit_deg')}")

    # Driver
    driver = config.get("driver", {})
    if driver.get("control_rate", 50.0) <= 0:
        issues.append(f"driver.control_rate must be > 0, got {driver.get('control_rate')}")

    # Camera
    camera = config.get("camera", {})
    fx = camera.get("fx", 615.0)
    fy = camera.get("fy", 615.0)
    if fx <= 0 or fy <= 0:
        issues.append(f"camera.fx/fy must be > 0, got fx={fx}, fy={fy}")
    if camera.get("width", 640) <= 0 or camera.get("height", 480) <= 0:
        issues.append(
            f"camera.width/height must be > 0, got {camera.get('width')}x{camera.get('height')}"
        )

    # GNSS (only when enabled)
    gnss = config.get("gnss", {})
    if gnss.get("enabled", False):
        quality = gnss.get("quality", {})
        if quality.get("min_sat_used", 8) < 0:
            issues.append(f"gnss.quality.min_sat_used must be >= 0, got {quality.get('min_sat_used')}")
        if quality.get("max_hdop", 2.5) <= 0:
            issues.append(f"gnss.quality.max_hdop must be > 0, got {quality.get('max_hdop')}")
        if quality.get("max_age_s", 2.0) <= 0:
            issues.append(f"gnss.quality.max_age_s must be > 0, got {quality.get('max_age_s')}")

        fusion = gnss.get("fusion", {})
        alpha_h = fusion.get("alpha_healthy", 0.05)
        alpha_d = fusion.get("alpha_degraded", 0.5)
        if not (0.0 <= alpha_h <= 1.0):
            issues.append(f"gnss.fusion.alpha_healthy must be in [0, 1], got {alpha_h}")
        if not (0.0 <= alpha_d <= 1.0):
            issues.append(f"gnss.fusion.alpha_degraded must be in [0, 1], got {alpha_d}")
        if fusion.get("residual_warn_m", 5.0) <= 0:
            issues.append(f"gnss.fusion.residual_warn_m must be > 0, got {fusion.get('residual_warn_m')}")
        rw_ratio = fusion.get("residual_warn_ratio", 0.7)
        if not (0.0 < rw_ratio <= 1.0):
            issues.append(f"gnss.fusion.residual_warn_ratio must be in (0, 1], got {rw_ratio}")

    return issues
