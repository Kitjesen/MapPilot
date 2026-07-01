"""ROS 2 map-save compatibility adapter.

Product modules should call a map-save adapter instead of constructing ROS 2
commands directly. This file keeps the transitional ``/pgo/save_maps`` service
call behind the ROS compatibility boundary.
"""

from __future__ import annotations

import json
import shlex
import subprocess
from pathlib import Path
from typing import Any

from core.map_save import MapSaveError, MapSaveTimeout, MapSaveUnavailable
from core.registry import register

_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
)


def save_pgo_map(
    file_path: str | Path,
    *,
    save_patches: bool = True,
    timeout_sec: float = 30.0,
) -> dict[str, Any]:
    """Save a SLAM map through the legacy ROS 2 PGO service."""

    payload = {"file_path": str(file_path), "save_patches": bool(save_patches)}
    try:
        result = subprocess.run(
            [
                "ros2",
                "service",
                "call",
                "/pgo/save_maps",
                "interface/srv/SaveMaps",
                json.dumps(payload),
            ],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_sec,
        )
    except FileNotFoundError as exc:
        raise MapSaveUnavailable(f"ROS2 not available: {exc}") from exc
    except subprocess.TimeoutExpired as exc:
        raise MapSaveTimeout(f"ROS2 map save timed out: {exc}") from exc

    if result.returncode != 0:
        raise MapSaveError(f"SaveMaps failed: {result.stderr}")

    return {
        "success": True,
        "source": "ros2_pgo_save_maps",
        "service": "/pgo/save_maps",
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


def save_nav_map(
    pcd_path: str | Path,
    *,
    timeout_sec: float = 30.0,
) -> dict[str, Any]:
    """Save the active map through the legacy ``/nav/save_map`` service."""

    safe_pcd = shlex.quote(str(pcd_path))
    command = (
        _ROS_SETUP
        + "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
        + "ros2 service call /nav/save_map interface/srv/SaveMaps "
        + f"\"{{file_path: {safe_pcd}}}\""
    )
    try:
        result = subprocess.run(
            ["bash", "-c", command],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_sec,
        )
    except FileNotFoundError as exc:
        raise MapSaveUnavailable(f"ROS2 not available: {exc}") from exc
    except subprocess.TimeoutExpired as exc:
        raise MapSaveTimeout(f"ROS2 map save timed out: {exc}") from exc

    if "success=True" not in (result.stdout or ""):
        detail = result.stderr[-200:] if result.stderr else (result.stdout or "")[-200:]
        raise MapSaveError(f"SaveMap failed: {detail}")

    return {
        "success": True,
        "source": "ros2_nav_save_map",
        "service": "/nav/save_map",
        "returncode": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }


@register("map_save_adapter", "ros2", description="ROS 2 map-save service adapter")
class Ros2MapSaveAdapter:
    """ROS 2 implementation of the core map-save adapter contract."""

    def save_nav_map(
        self,
        pcd_path: str | Path,
        *,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        return save_nav_map(pcd_path, timeout_sec=timeout_sec)

    def save_pgo_map(
        self,
        file_path: str | Path,
        *,
        save_patches: bool = True,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        return save_pgo_map(
            file_path,
            save_patches=save_patches,
            timeout_sec=timeout_sec,
        )
