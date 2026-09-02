"""Shared discovery and response parsing for the native SLAM control tool."""

from __future__ import annotations

import json
import os
import platform
import shutil
from pathlib import Path
from typing import Any


def slam_control_binary() -> str:
    """Return the configured native SLAM control executable."""
    explicit = os.environ.get("LINGTU_SLAM_CONTROL", "").strip()
    if explicit:
        return explicit

    repo_root = Path(__file__).resolve().parents[2]
    windows = platform.system() == "Windows"
    if windows:
        candidates = (
            repo_root / "build" / "slam-core-windows-x64" / "stage" / "bin" / "slamctl.exe",
            repo_root / "build" / "slam-core-windows-x64" / "Release" / "slamctl.exe",
        )
    else:
        candidates = (
            repo_root / "build" / "slam_core" / "slamctl",
            Path("/opt/lingtu/current/bin/slamctl"),
        )
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    resolved = shutil.which("slamctl.exe" if windows else "slamctl")
    if resolved:
        return resolved
    raise FileNotFoundError(
        "native SLAM control binary not found; build src/localization/slam/cpp "
        "with LINGTU_SLAM_BUILD_DDS_RUNTIME=ON or set LINGTU_SLAM_CONTROL"
    )


def last_json_object(text: str) -> dict[str, Any]:
    """Return the last JSON object emitted by a line-oriented control tool."""
    for line in reversed([item.strip() for item in text.splitlines()]):
        if not line:
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    return {}
